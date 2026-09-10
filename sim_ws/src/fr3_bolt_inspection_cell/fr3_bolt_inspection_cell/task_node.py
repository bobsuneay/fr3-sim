"""Explicitly started, single-run dual-arm inspection workflow."""
import json
from datetime import datetime
from pathlib import Path
import threading
import time
import numpy as np
import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from rclpy.time import Time
from scipy.spatial.transform import Rotation
from sensor_msgs.msg import CameraInfo, Image, JointState, PointCloud2
from sensor_msgs_py import point_cloud2
from gazebo_msgs.msg import ModelStates
from std_msgs.msg import String
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformListener
import yaml
from .core import validate, estimate_bolt, grasp_in_object, centered_views, interpolate_object
from .ros_io import IO, PlanningFailure, matrix
from .handover import transfer


def stamp_seconds(stamp):
    return stamp.sec+stamp.nanosec*1e-9


class Inspection(Node):
    def __init__(self):
        super().__init__('bolt_inspection_task')
        self.declare_parameter('config_file', '')
        self.declare_parameter('arms_file', '')
        self.declare_parameter('mode', 'mock')
        self.declare_parameter('enable_execution', False)
        self.cfg = validate(yaml.safe_load(Path(self.get_parameter('config_file').value).read_text()))
        self.arms = yaml.safe_load(Path(self.get_parameter('arms_file').value).read_text())
        self.stop_event = threading.Event()
        self.data_lock = threading.Lock()
        self.run_lock = threading.Lock()
        self.joints, self.images, self.infos = {}, {}, {}
        self.cloud = None
        self.object_state = None
        self.worker = None
        self.phase = 'IDLE'
        self.report = {'backend': 'gazebo_assisted', 'events': [], 'views': []}
        self.output = None
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.status_pub = self.create_publisher(String, '/inspection/status',
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.create_subscription(JointState, '/joint_states', self.on_joints, qos_profile_sensor_data)
        self.create_subscription(PointCloud2, self.cfg['cloud_topic'], self.on_cloud, qos_profile_sensor_data)
        self.create_subscription(ModelStates, '/inspection/sim/model_states', self.on_models, qos_profile_sensor_data)
        for name in self.cfg['cameras']:
            for stream, topic in (('rgb', 'image_raw'), ('depth', 'depth/image_raw')):
                self.create_subscription(Image, f'/{name}/{topic}',
                    lambda msg, key=(name, stream): self.on_image(key, msg), qos_profile_sensor_data)
            self.create_subscription(CameraInfo, f'/{name}/camera_info',
                lambda msg, key=name: self.on_info(key, msg), qos_profile_sensor_data)
        self.io = IO(self)
        self.create_service(Trigger, '/inspection/start', self.start)
        self.create_service(Trigger, '/inspection/stop', self.stop)
        self.create_service(Trigger, '/inspection/get_status', self.status)
        self.publish('IDLE', 'Ready for explicit start; Gazebo assisted grasp only')

    def on_joints(self, msg):
        if len(msg.name) != len(msg.position) or not np.all(np.isfinite(msg.position)):
            return
        with self.data_lock:
            self.joints.update({n: (q, time.monotonic()) for n, q in zip(msg.name, msg.position)})

    def on_cloud(self, msg):
        with self.data_lock:
            self.cloud = (msg, time.monotonic())

    def on_models(self, msg):
        try:
            index = msg.name.index(self.cfg['simulation_entity'])
            value = matrix(msg.pose[index])
        except (ValueError, IndexError):
            return
        with self.data_lock:
            self.object_state = (value, time.monotonic())

    def on_image(self, key, msg):
        with self.data_lock:
            self.images[key] = (msg, time.monotonic())

    def on_info(self, key, msg):
        with self.data_lock:
            self.infos[key] = msg

    def publish(self, phase, detail):
        self.phase = phase
        event = {'phase': phase, 'detail': detail, 'time': datetime.now().isoformat()}
        self.report['events'].append(event)
        self.status_pub.publish(String(data=json.dumps(event, ensure_ascii=False)))
        self.get_logger().info(phase+': '+detail)
        self.save_report()

    def save_report(self):
        if self.output:
            temp = self.output/'report.tmp'
            temp.write_text(json.dumps(self.report, indent=2, ensure_ascii=False), encoding='utf-8')
            temp.replace(self.output/'report.json')

    def start(self, req, res):
        with self.run_lock:
            if self.phase != 'IDLE':
                res.message = 'Run already started/latched; inspect status, then restart simulation for a new run'
                return res
            if self.get_parameter('mode').value != 'gazebo' or not self.get_parameter('enable_execution').value:
                res.message = 'Require mode:=gazebo enable_execution:=true; mock is model preview only'
                return res
            self.stop_event.clear()
            self.publish('STARTING', 'Checking feedback, cameras and point cloud')
            self.worker = threading.Thread(target=self.run, daemon=True)
            self.worker.start()
            res.success, res.message = True, 'Started; monitor /inspection/status'
            return res

    def stop(self, req, res):
        self.stop_event.set()
        self.io.cancel()
        res.success, res.message = True, 'Cancellation requested; no automatic gripper opening'
        return res

    def status(self, req, res):
        res.success, res.message = True, json.dumps(self.report['events'][-1], ensure_ascii=False)
        return res

    def settle(self):
        # Wall-clock responsiveness plus advancing ROS time; paused sim cannot pass.
        start = self.get_clock().now().nanoseconds
        end = time.monotonic()+15
        while (self.get_clock().now().nanoseconds-start)*1e-9 < self.cfg['settle_seconds']:
            self.io.check()
            if time.monotonic() > end:
                raise TimeoutError('Simulation clock stopped while settling')
            self.stop_event.wait(.05)

    def perceive(self):
        deadline = time.monotonic()+self.cfg['cloud_timeout']
        last, error = None, 'No cloud received'
        estimates = []
        while time.monotonic() < deadline:
            self.io.check()
            with self.data_lock:
                cloud = self.cloud
            if cloud and cloud[0] is not last and time.monotonic()-cloud[1] < self.cfg['max_data_age']:
                msg = cloud[0]
                last = msg
                age = self.get_clock().now().nanoseconds*1e-9-stamp_seconds(msg.header.stamp)
                if not 0 <= age <= self.cfg['max_data_age']:
                    continue
                try:
                    # Humble read_points returns a structured NumPy array.
                    data = point_cloud2.read_points(msg, field_names=('x', 'y', 'z'), skip_nans=True)
                    if isinstance(data, np.ndarray) and data.dtype.names:
                        p = np.column_stack([data[k].reshape(-1) for k in ('x', 'y', 'z')])
                    else:
                        p = np.asarray(list(data), dtype=float).reshape(-1, 3)
                    t = self.io.tf(msg.header.frame_id, Time.from_msg(msg.header.stamp))
                    found = estimate_bolt(p@t[:3, :3].T+t[:3, 3], self.cfg)
                    if estimates:
                        drift = np.linalg.norm(found.pose[:3, 3]-estimates[-1].pose[:3, 3])
                        alignment = found.pose[:3, 0]@estimates[-1].pose[:3, 0]
                        if drift > .002 or alignment < .98:
                            estimates.clear()
                    estimates.append(found)
                    if len(estimates) >= 3:
                        np.savez_compressed(self.output/'detection.npz', points=found.points,
                                            object_pose=found.pose, dimensions=found.dimensions)
                        return found
                except Exception as exc:
                    error = str(exc)
                    estimates.clear()
            self.stop_event.wait(.1)
        raise RuntimeError('Point-cloud detection failed: '+error)

    def capture(self, label, expected_object=None):
        # All three cameras must provide a fresh RGB/depth pair after settling.
        self.settle()
        after = self.get_clock().now().nanoseconds*1e-9
        deadline = time.monotonic()+8
        selected = {}
        while time.monotonic() < deadline:
            self.io.check()
            with self.data_lock:
                images, infos = dict(self.images), dict(self.infos)
            selected = {}
            for camera in self.cfg['cameras']:
                rgb, depth = images.get((camera, 'rgb')), images.get((camera, 'depth'))
                if rgb is None or depth is None or camera not in infos:
                    continue
                a, b = stamp_seconds(rgb[0].header.stamp), stamp_seconds(depth[0].header.stamp)
                if min(a, b) <= after or abs(a-b) > .05:
                    continue
                if max(time.monotonic()-rgb[1], time.monotonic()-depth[1]) > self.cfg['max_data_age']:
                    continue
                selected[camera] = (rgb[0], depth[0], infos[camera])
            if len(selected) == len(self.cfg['cameras']):
                break
            self.stop_event.wait(.05)
        if len(selected) != len(self.cfg['cameras']):
            raise RuntimeError('Fresh synchronized camera frames unavailable: '+str(set(self.cfg['cameras'])-set(selected)))
        folder = self.output/label
        folder.mkdir()
        meta = {}
        for camera, (rgb, depth, info) in selected.items():
            world_camera = self.io.tf(rgb.header.frame_id, Time.from_msg(rgb.header.stamp))
            np.savez_compressed(folder/(camera+'.npz'),
                rgb=np.frombuffer(bytes(rgb.data), dtype=np.uint8),
                depth=np.frombuffer(bytes(depth.data), dtype=np.uint8),
                intrinsic=np.array(info.k).reshape(3, 3), world_optical=world_camera)
            # Portable viewable RGB preview, preserving row stride.
            if rgb.encoding in ('rgb8', 'bgr8'):
                pixels = np.frombuffer(bytes(rgb.data), dtype=np.uint8).reshape(rgb.height, rgb.step)
                pixels = pixels[:, :rgb.width*3].reshape(rgb.height, rgb.width, 3)
                if rgb.encoding == 'bgr8':
                    pixels = pixels[:, :, ::-1]
                (folder/(camera+'.ppm')).write_bytes(
                    f'P6\n{rgb.width} {rgb.height}\n255\n'.encode()+pixels.tobytes())
            meta[camera] = {kind: {'stamp': stamp_seconds(m.header.stamp), 'encoding': m.encoding,
                'height': m.height, 'width': m.width, 'step': m.step, 'is_bigendian': m.is_bigendian}
                for kind, m in (('rgb', rgb), ('depth', depth))}
        if expected_object is not None:
            # Verify projected centre lies inside fixed inspection camera FOV.
            camera = 'waist_camera'
            rgb, _, info = selected[camera]
            local = np.linalg.inv(self.io.tf(rgb.header.frame_id, Time.from_msg(rgb.header.stamp))) @ np.r_[expected_object[:3, 3], 1]
            if local[2] <= .1:
                raise RuntimeError('Part behind inspection camera / too close')
            u = info.k[0]*local[0]/local[2]+info.k[2]
            v = info.k[4]*local[1]/local[2]+info.k[5]
            if not (20 < u < rgb.width-20 and 20 < v < rgb.height-20):
                raise RuntimeError('Part centre outside inspection camera field of view')
        (folder/'metadata.json').write_text(json.dumps(meta, indent=2), encoding='utf-8')

    def verify(self, expected):
        actual = self.io.truth()
        predicted = expected@self.object_truth
        drift = np.linalg.norm(actual[:3, 3]-predicted[:3, 3])
        angle = Rotation.from_matrix(predicted[:3, :3].T@actual[:3, :3]).magnitude()
        if drift > self.cfg['center_tolerance'] or angle > self.cfg['angular_tolerance']:
            raise RuntimeError(f'Part slipped / tracking error: {drift*1000:.1f} mm, {angle:.3f} rad')
        return float(drift)

    def scan(self, side, neutral, object_tcp):
        self.publish('INSPECT_'+side.upper(), 'Rotating around the estimated object centre')
        completed = 0
        for index, (obj, _) in enumerate(centered_views(neutral[:3, 3], neutral[:3, :3],
                                                       object_tcp, self.cfg['views_deg'])):
            entry = {'arm': side, 'index': index, 'angles_deg': self.cfg['views_deg'][index]}
            waypoints = interpolate_object(neutral, obj, object_tcp, self.cfg['cartesian_step'])
            try:
                if index != 0:
                    self.io.cartesian(side, waypoints, self.cfg['scan_speed'], object_tcp, neutral[:3, 3])
            except PlanningFailure as exc:
                entry.update(status='unreachable', reason=str(exc))
                self.report['views'].append(entry)
                self.save_report()
                continue
            # Execution/capture/verification failure aborts, never counted as a skipped plan.
            self.settle()
            entry['drift_m'] = self.verify(obj)
            self.capture(f'{side}_view_{index:02d}', obj)
            entry.update(status='captured', object_pose=obj.tolist())
            self.report['views'].append(entry)
            self.save_report()
            completed += 1
            if index != 0:
                self.io.cartesian(side, interpolate_object(obj, neutral, object_tcp,
                    self.cfg['cartesian_step']), self.cfg['scan_speed'], object_tcp, neutral[:3, 3])
                self.settle()
                self.verify(neutral)
        if completed < self.cfg['minimum_views']:
            raise RuntimeError(f'Only {completed} views reachable for {side}; requires {self.cfg["minimum_views"]}')

    def run(self):
        try:
            self.output = Path(self.cfg['output_directory']).expanduser()/datetime.now().strftime('%Y%m%d_%H%M%S_%f')
            self.output.mkdir(parents=True)
            (self.output/'config.yaml').write_text(yaml.safe_dump(self.cfg), encoding='utf-8')
            self.io.state()
            if self.io.call(self.io.owner, Trigger.Request()).message:
                raise RuntimeError('A gripper already owns the object; restart simulation')
            self.capture('camera_check')
            self.publish('DETECT', 'Waiting for three consistent point-cloud estimates')
            estimate = self.perceive()
            obj = estimate.pose
            first = self.cfg['first_arm']
            second = 'left' if first == 'right' else 'right'
            donor_grasp = grasp_in_object(-self.cfg['grasp_offset'])
            receiver_grasp = grasp_in_object(self.cfg['grasp_offset'], below=True)
            self.report['estimated_pose'] = obj.tolist()
            self.object_truth = np.linalg.inv(obj)@self.io.truth()  # verification reference only
            if np.linalg.norm(self.object_truth[:3, 3]) > .004:
                raise RuntimeError('Estimated centre differs by >4 mm from simulation verification')
            self.io.object_scene(obj)
            self.io.allow_touch(['table_top', first+'_left_finger', first+'_right_finger'])
            self.publish('APPROACH', first+' approaching above the detected shaft')
            self.io.gripper(first, self.cfg['open_width'])
            self.io.gripper(second, self.cfg['open_width'])
            target = obj@donor_grasp
            above = target.copy()
            above[2, 3] += self.cfg['approach_height']
            self.get_logger().info(
                f'APPROACH poses: above=({above[0, 3]:.4f}, {above[1, 3]:.4f}, '
                f'{above[2, 3]:.4f}), grasp=({target[0, 3]:.4f}, {target[1, 3]:.4f}, '
                f'{target[2, 3]:.4f}), descent={self.cfg["approach_height"]:.3f} m')
            # A reachable above-pick pose can be on an IK branch that cannot
            # descend. Check its full continuation before moving to that pose.
            descent = self.io.global_move(first, above, continuation=target)
            self.publish('DESCEND', 'Straight downward approach at configured slow speed')
            self.io.execute_prepared_cartesian(descent, self.cfg['descent_speed'])
            self.publish('GRASP', 'Close jaws; validate shaft enclosure before assisted attachment')
            self.io.gripper(first, self.cfg['close_width'])
            self.io.assisted_grasp(first)
            self.io.object_scene(obj, first)
            lifted = obj.copy()
            lifted[2, 3] += self.cfg['lift_height']
            self.io.cartesian(first, [lifted@donor_grasp], self.cfg['descent_speed'])
            self.settle()
            self.verify(lifted)
            self.io.allow_touch(['table_top'], False)
            neutral = obj.copy()
            neutral[:3, 3] = self.cfg['inspection_center']
            self.publish('LIFT_TO_CAMERA', 'Moving the part into the fixed inspection camera view')
            self.io.global_move(first, neutral@donor_grasp)
            self.settle()
            self.verify(neutral)
            self.scan(first, neutral, donor_grasp)
            handover = neutral.copy()
            handover[:3, 3] = self.cfg['handover_center']
            self.io.cartesian(first, [handover@donor_grasp], self.cfg['transfer_speed'])
            self.publish('HANDOVER_APPROACH', second+' approaching the free shaft region from below')
            self.io.allow_touch([second+'_left_finger', second+'_right_finger'])
            target = handover@receiver_grasp
            pre = target.copy()
            pre[:3, 3] -= target[:3, 2]*self.cfg['approach_height']
            self.io.global_move(second, pre)
            self.io.cartesian(second, [target], self.cfg['descent_speed'])
            self.publish('HANDOVER_CONFIRM', 'Validate receiver before opening donor')
            transfer(self.io, first, second, handover, self.cfg['close_width'],
                     self.cfg['open_width'], self.settle, self.verify)
            retreat = handover@donor_grasp
            retreat[:3, 3] -= retreat[:3, 2]*self.cfg['approach_height']
            self.io.cartesian(first, [retreat], self.cfg['transfer_speed'])
            self.io.allow_touch([first+'_left_finger', first+'_right_finger'], False)
            self.io.global_move(first, joints=self.arms[first]['initial'])
            self.io.cartesian(second, [neutral@receiver_grasp], self.cfg['transfer_speed'])
            self.scan(second, neutral, receiver_grasp)
            skipped = sum(v['status'] == 'unreachable' for v in self.report['views'])
            self.publish('DONE_HOLDING_'+second.upper(),
                f'Both arms inspected; receiver retains part. Unreachable views: {skipped}. Results: {self.output}')
        except Exception as exc:
            self.io.cancel()
            self.publish('STOPPED' if self.stop_event.is_set() else 'FAILED', str(exc))


def main():
    rclpy.init()
    node = Inspection()
    executor = MultiThreadedExecutor(num_threads=3)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.stop_event.set()
        node.io.cancel()
        if node.worker:
            node.worker.join(timeout=4)
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
