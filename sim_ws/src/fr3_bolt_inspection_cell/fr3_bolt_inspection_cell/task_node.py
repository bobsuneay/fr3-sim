"""Explicitly started inspection workflow with failed-pick recovery."""
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
from std_msgs.msg import String, Float64
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformListener
import yaml
from .core import (validate, estimate_bolt, grasp_in_object, centered_views,
                   interpolate_object, random_disk_xy, transform)
from .core import fingertip_table_pick_tcp, fingertip_world_min_z, perpendicular_receiver_grasp
from .geometry import fingertip_points_tcp
from .ros_io import IO, PlanningFailure, matrix
from .handover import transfer
from .retry import prepare_pick_retry
from .feedback import feedback_joint_name


def stamp_seconds(stamp):
    return stamp.sec+stamp.nanosec*1e-9


class Inspection(Node):
    def __init__(self):
        super().__init__('bolt_inspection_task')
        self.declare_parameter('config_file', '')
        self.declare_parameter('arms_file', '')
        self.declare_parameter('fingertip_geometry_file', '')
        self.declare_parameter('mode', 'mock')
        self.declare_parameter('enable_execution', False)
        self.cfg = validate(yaml.safe_load(Path(self.get_parameter('config_file').value).read_text()))
        self.arms = yaml.safe_load(Path(self.get_parameter('arms_file').value).read_text())
        self.fingertips = yaml.safe_load(
            Path(self.get_parameter('fingertip_geometry_file').value).read_text())
        self.stop_event = threading.Event()
        self.data_lock = threading.Lock()
        self.run_lock = threading.Lock()
        self.joints, self.images, self.infos = {}, {}, {}
        self.cloud = None
        self.object_state = None
        self.worker = None
        self.phase = 'IDLE'
        self.pick_secured = False
        self.pick_target = None
        self.handover_context = None
        self.handover_requested = threading.Event()
        self.scan_speed_scale = 1.0
        self.create_subscription(Float64, '/inspection/scan_speed_scale', self.set_scan_speed, 10)
        self.report = {'backend': 'gazebo_assisted', 'events': [], 'views': []}
        self.output = None
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.status_pub = self.create_publisher(String, '/inspection/status',
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.create_subscription(JointState, '/joint_states', self.on_joints, qos_profile_sensor_data)
        self.create_subscription(JointState, '/inspection/sim/gripper_states', self.on_joints,
                                 qos_profile_sensor_data)
        self.create_subscription(PointCloud2, self.cfg['cloud_topic'], self.on_cloud, qos_profile_sensor_data)
        self.create_subscription(ModelStates, '/inspection/sim/model_states', self.on_models, qos_profile_sensor_data)
        for name in self.cfg['cameras']:
            streams = [('rgb', 'image_raw')]
            if self.cfg['cameras'][name].get('depth', True):
                streams.append(('depth', 'depth/image_raw'))
            for stream, topic in streams:
                self.create_subscription(Image, f'/{name}/{topic}',
                    lambda msg, key=(name, stream): self.on_image(key, msg), qos_profile_sensor_data)
            self.create_subscription(CameraInfo, f'/{name}/camera_info',
                lambda msg, key=name: self.on_info(key, msg), qos_profile_sensor_data)
        self.io = IO(self)
        self.create_service(Trigger, '/inspection/start', self.start)
        self.create_service(Trigger, '/inspection/retry_pick', self.retry_pick)
        self.create_service(Trigger, '/inspection/skip_to_handover', self.skip_to_handover)
        self.create_service(Trigger, '/inspection/randomize_object', self.randomize_object)
        self.create_service(Trigger, '/inspection/stop', self.stop)
        self.create_service(Trigger, '/inspection/get_status', self.status)
        self.get_logger().info(
            f"Point-cloud pose source: {self.cfg['point_cloud_camera']} ({self.cfg['cloud_topic']}); "
            "waist_camera=RGB-only, wrist cameras=RGB-D")
        self.publish('IDLE', 'Ready for explicit start; Gazebo assisted grasp only')

    def on_joints(self, msg):
        if len(msg.name) != len(msg.position) or not np.all(np.isfinite(msg.position)):
            return
        with self.data_lock:
            self.joints.update({feedback_joint_name(n): (q, time.monotonic())
                                for n, q in zip(msg.name, msg.position)})

    def set_scan_speed(self, msg):
        value = float(msg.data)
        if not np.isfinite(value) or not .25 <= value <= 2.0:
            self.get_logger().warning('Rejected scan speed scale: require 0.25 to 2.0')
            return
        self.scan_speed_scale = value
        self.get_logger().info(
            f'Inspection speed scale={value:.2f}x; applies to next rotation segment, current motion unchanged')

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
        return self.start_run(res, retry=False)

    def retry_pick(self, req, res):
        return self.start_run(res, retry=True)

    def can_handover(self):
        busy = self.worker is not None and self.worker.is_alive()
        return (self.get_parameter('mode').value == 'gazebo'
                and self.get_parameter('enable_execution').value
                and self.handover_context is not None
                and not self.handover_requested.is_set()
                and ((busy and self.phase == 'INSPECT_RIGHT')
                     or (not busy and self.phase in ('FAILED', 'STOPPED'))))

    def skip_to_handover(self, req, res):
        with self.run_lock:
            if not self.can_handover():
                res.message = 'Available during right-arm inspection or after it stops with a retained grasp'
                return res
            self.handover_requested.set()
            if self.worker is not None and self.worker.is_alive():
                res.success, res.message = True, 'Queued: finish current motion, then hand over to left arm'
            else:
                self.stop_event.clear()
                self.worker = threading.Thread(target=self.resume_handover, daemon=True)
                self.worker.start()
                res.success, res.message = True, 'Checking retained right grasp before left-arm handover'
            return res

    def resume_handover(self):
        try:
            self.io.wait_stationary()
            self.finish_handover(*self.handover_context)
        except Exception as exc:
            self.io.cancel()
            self.publish('STOPPED' if self.stop_event.is_set() else 'FAILED', str(exc))
        finally:
            self.handover_requested.clear()

    def start_run(self, res, retry):
        with self.run_lock:
            if self.worker is not None and self.worker.is_alive():
                res.message = 'Task still running/cancelling; wait for it to finish before retrying'
                return res
            if ((retry and (self.phase not in ('FAILED', 'STOPPED') or self.pick_secured))
                    or (not retry and self.phase != 'IDLE')):
                res.message = 'Retry is available after an initial pick fails/stops, before the object is held'
                return res
            if self.get_parameter('mode').value != 'gazebo' or not self.get_parameter('enable_execution').value:
                res.message = 'Require mode:=gazebo enable_execution:=true; mock is model preview only'
                return res
            self.stop_event.clear()
            self.handover_context = None
            self.handover_requested.clear()
            previous = str(self.output) if self.output else None
            self.output = None
            self.report = {'backend': 'gazebo_assisted', 'events': [], 'views': [],
                           'retry_of': previous if retry else None}
            self.publish('RETRY_STARTING' if retry else 'STARTING',
                         'Preparing failed-pick recovery' if retry else 'Checking feedback, cameras and point cloud')
            self.worker = threading.Thread(target=self.run, kwargs={'retry': retry}, daemon=True)
            self.worker.start()
            res.success, res.message = True, 'Started; monitor /inspection/status'
            return res

    def stop(self, req, res):
        self.stop_event.set()
        self.io.cancel()
        res.success, res.message = True, 'Cancellation requested; no automatic gripper opening'
        return res

    def randomize_object(self, req, res):
        with self.run_lock:
            busy = self.worker is not None and self.worker.is_alive()
            if busy or self.pick_secured:
                res.message = 'Random position requires an idle task with no confirmed grasp'
                return res
            if self.get_parameter('mode').value != 'gazebo':
                res.message = 'Random position is available in Gazebo mode only'
                return res
            self.stop_event.clear()
            previous_phase = self.phase
            self.publish('RANDOMIZING', 'Checking ownership and waiting for stationary arms')
            # Service clients share the node's default mutually exclusive callback
            # group. Waiting here for a response would prevent it from completing.
            self.worker = threading.Thread(target=self.run_randomize,
                kwargs={'previous_phase': previous_phase}, daemon=True)
            self.worker.start()
            res.success, res.message = True, 'Randomization started; monitor /inspection/status for the result'
            return res

    def run_randomize(self, previous_phase):
        try:
            self.io.check()
            if self.io.grasp_owner():
                raise RuntimeError('A gripper holds the object; position unchanged')
            self.io.wait_stationary()
            self.io.check()
            if self.io.grasp_owner():
                raise RuntimeError('Object acquired while waiting; position unchanged')
            xy = random_disk_xy(self.cfg['random_position_center'],
                                self.cfg['random_position_radius'], np.random.default_rng())
            # The Gazebo model's bolt axis is local Z; perception uses local X.
            world_model = transform([xy[0], xy[1],
                self.cfg['table_z']+self.cfg['head_radius']+.0005], [0, np.pi/2, 0])
            self.publish('RANDOMIZING',
                f'Moving object to x={xy[0]:.4f}, y={xy[1]:.4f} and verifying feedback')
            try:
                actual = self.io.relocate_object(world_model)
            finally:
                # A timed-out request may already have moved the model. Never
                # retain a detection/target from before an attempted relocation.
                with self.data_lock:
                    self.cloud = None
                    self.object_state = None
                self.pick_target = None
            self.io.check()
            self.publish(previous_phase,
                f'Object randomized and verified at x={actual[0, 3]:.4f}, '
                f'y={actual[1, 3]:.4f}; next pick will redetect')
        except Exception as exc:
            self.publish('STOPPED' if self.stop_event.is_set() else 'FAILED',
                         f'Randomization failed: {exc}')

    def status(self, req, res):
        with self.run_lock:
            busy = self.worker is not None and self.worker.is_alive()
            event = dict(self.report['events'][-1])
            enabled = (self.get_parameter('mode').value == 'gazebo'
                       and self.get_parameter('enable_execution').value)
            event.update(busy=busy, can_start=bool(enabled and not busy and self.phase == 'IDLE'),
                         scan_speed_scale=self.scan_speed_scale,
                         can_handover=bool(self.can_handover()),
                         can_retry=bool(enabled and not busy and not self.pick_secured
                                        and self.phase in ('FAILED', 'STOPPED')),
                         can_randomize=bool(self.get_parameter('mode').value == 'gazebo'
                                            and not busy and not self.pick_secured))
        res.success, res.message = True, json.dumps(event, ensure_ascii=False)
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
        # Industrial waist camera is RGB-only; D435i cameras require RGB/depth.
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
                rgb = images.get((camera, 'rgb'))
                needs_depth = self.cfg['cameras'][camera].get('depth', True)
                depth = images.get((camera, 'depth')) if needs_depth else None
                if rgb is None or (needs_depth and depth is None) or camera not in infos:
                    continue
                a = stamp_seconds(rgb[0].header.stamp)
                b = stamp_seconds(depth[0].header.stamp) if depth is not None else a
                if min(a, b) <= after or (depth is not None and abs(a-b) > .05):
                    continue
                ages = [time.monotonic()-rgb[1]] + ([] if depth is None else [time.monotonic()-depth[1]])
                if max(ages) > self.cfg['max_data_age']:
                    continue
                selected[camera] = (rgb[0], depth[0] if depth is not None else None, infos[camera])
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
            payload = dict(rgb=np.frombuffer(bytes(rgb.data), dtype=np.uint8),
                           intrinsic=np.array(info.k).reshape(3, 3), world_optical=world_camera)
            if depth is not None:
                payload['depth'] = np.frombuffer(bytes(depth.data), dtype=np.uint8)
            np.savez_compressed(folder/(camera+'.npz'), **payload)
            # Portable viewable RGB preview, preserving row stride.
            if rgb.encoding in ('rgb8', 'bgr8'):
                pixels = np.frombuffer(bytes(rgb.data), dtype=np.uint8).reshape(rgb.height, rgb.step)
                pixels = pixels[:, :rgb.width*3].reshape(rgb.height, rgb.width, 3)
                if rgb.encoding == 'bgr8':
                    pixels = pixels[:, :, ::-1]
                (folder/(camera+'.ppm')).write_bytes(
                    f'P6\n{rgb.width} {rgb.height}\n255\n'.encode()+pixels.tobytes())
            meta[camera] = {'rgb': {'stamp': stamp_seconds(rgb.header.stamp), 'encoding': rgb.encoding,
                'height': rgb.height, 'width': rgb.width, 'step': rgb.step, 'is_bigendian': rgb.is_bigendian},
                'depth': None if depth is None else {
                    'stamp': stamp_seconds(depth.header.stamp), 'encoding': depth.encoding,
                    'height': depth.height, 'width': depth.width, 'step': depth.step,
                    'is_bigendian': depth.is_bigendian}}
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
            if side == 'right' and self.handover_requested.is_set():
                self.get_logger().info('Operator requested left handover; skipping remaining right views')
                return
            entry = {'arm': side, 'index': index, 'angles_deg': self.cfg['views_deg'][index]}
            waypoints = interpolate_object(neutral, obj, object_tcp, self.cfg['cartesian_step'])
            try:
                if index != 0:
                    self.io.cartesian(side, waypoints, self.cfg['scan_speed'], object_tcp, neutral[:3, 3])
            except PlanningFailure as exc:
                self.get_logger().warning(
                    f'Inspection view skipped: arm={side}, index={index}, '
                    f'angles_deg={self.cfg["views_deg"][index]}; {exc}')
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
            if side == 'right' and self.handover_requested.is_set():
                return
            if index != 0:
                self.io.cartesian(side, interpolate_object(obj, neutral, object_tcp,
                    self.cfg['cartesian_step']), self.cfg['scan_speed'], object_tcp, neutral[:3, 3])
                self.settle()
                self.verify(neutral)
        if side == 'right' and self.handover_requested.is_set():
            return
        if completed < self.cfg['minimum_views']:
            raise RuntimeError(f'Only {completed} views reachable for {side}; requires {self.cfg["minimum_views"]}')

    def reset_failed_pick(self, first):
        """Retry an empty grasp only; never release an acquired part here."""
        owner = self.io.grasp_owner()
        if self.pick_secured or owner:
            self.pick_secured = True
            raise RuntimeError('Object acquired or ownership uncertain; retaining closed jaws, no automatic release')
        actual = self.io.truth()
        # Safe even when attachment was never applied: remove the named
        # attached object and replace its current world collision geometry.
        # truth() is the physical model frame (bolt axis Z); MoveIt uses axis X.
        self.io.object_scene(actual@transform(rpy=(0, -np.pi/2, 0)), previous=first)
        self.pick_secured = False
        prepare_pick_retry(self.io, first, self.arms, self.cfg, self.pick_target, self.settle)

    def pick_once(self, first, second, attempt):
        """Detect, approach, close and prove the object follows a short test lift."""
        self.publish('DETECT',
            f'Pick attempt {attempt}/{self.cfg["max_grasp_attempts"]}: waiting for three point-cloud estimates')
        estimate = self.perceive()
        obj = estimate.pose
        open_positions = {f'{first}_{finger}_finger_joint': self.cfg['open_width']/2
                          for finger in ('left', 'right')}
        tip_points = fingertip_points_tcp(self.fingertips[first], open_positions)
        target = fingertip_table_pick_tcp(
            obj, -self.cfg['grasp_offset'], self.cfg['table_z'],
            self.cfg['fingertip_table_clearance'], tip_points)
        self.report['estimated_pose'] = obj.tolist()
        self.object_truth = np.linalg.inv(obj)@self.io.truth()
        if np.linalg.norm(self.object_truth[:3, 3]) > .004:
            raise RuntimeError('Estimated centre differs by >4 mm from simulation verification')
        self.io.object_scene(obj)
        self.io.allow_touch(['table_top', first+'_left_finger', first+'_right_finger'])
        self.publish('APPROACH',
            f'Pick attempt {attempt}: {first} approaching above the detected shaft')
        self.io.gripper(first, self.cfg['open_width'])
        self.io.gripper(second, self.cfg['open_width'])
        above = target.copy()
        above[2, 3] += self.cfg['approach_height']
        donor_grasp = np.linalg.inv(obj)@target
        receiver_grasp = perpendicular_receiver_grasp(self.cfg['grasp_offset'], donor_grasp)
        self.get_logger().info('Handover geometry: receiver rotated 90 degrees around part axis; perpendicular jaw closing axes')
        self.pick_target = target.copy()
        self.get_logger().info(
            f'APPROACH poses: above=({above[0, 3]:.4f}, {above[1, 3]:.4f}, '
            f'{above[2, 3]:.4f}), grasp=({target[0, 3]:.4f}, {target[1, 3]:.4f}, '
            f'{target[2, 3]:.4f}), tip_min_z={fingertip_world_min_z(target, tip_points):.4f}, '
            f'descent={self.cfg["approach_height"]:.3f} m, '
            f'fingertip_clearance={self.cfg["fingertip_table_clearance"]*1000:.1f} mm')
        descent = self.io.global_move(first, above, continuation=target)
        self.publish('DESCEND', f'Pick attempt {attempt}: straight downward approach')
        self.io.execute_prepared_cartesian(descent, self.cfg['descent_speed'])
        self.publish('CHECK_GRASP_REACH', 'Check measured TCP; supplement a short descent if needed')
        self.io.ensure_grasp_reached(first, target, self.cfg['descent_speed'], self.settle)
        self.check_fingertip_clearance(first)
        self.publish('GRASP', f'Pick attempt {attempt}: close jaws and request assisted contact validation')
        self.io.gripper(first, self.cfg['close_width'])
        self.publish('CHECK_CONTACT', 'Jaws closed; validate physical bolt contact before lifting')
        self.io.assisted_grasp(first)
        # Safety latch: ownership exists even though the test lift has not yet
        # proved a usable grasp. Automatic cleanup clears it after release.
        self.pick_secured = True
        self.publish('GRASP_ACQUIRED', 'Contact accepted; retain closed jaws and prepare test lift')
        self.io.object_scene(obj, first)
        test_pose = obj.copy()
        test_pose[2, 3] += self.cfg['grasp_test_lift']
        self.publish('VERIFY_GRASP',
            f'Test lift {self.cfg["grasp_test_lift"]*1000:.0f} mm and verify object follows')
        self.io.cartesian(first, [test_pose@donor_grasp], self.cfg['descent_speed'])
        self.settle()
        drift = self.verify(test_pose)
        if self.io.grasp_owner() != first:
            raise RuntimeError('Grasp ownership was lost during test lift')
        self.publish('GRASP_CONFIRMED',
            f'Object followed test lift; tracking error={drift*1000:.2f} mm')
        return obj, donor_grasp, receiver_grasp, test_pose

    def check_fingertip_clearance(self, side):
        """Use fresh physical finger feedback and FK before allowing closure."""
        actual = self.io.tcp_pose(side)
        state = self.io.state().joint_state
        points = fingertip_points_tcp(self.fingertips[side], dict(zip(state.name, state.position)))
        actual_clearance = fingertip_world_min_z(actual, points)-self.cfg['table_z']
        desired = self.cfg['fingertip_table_clearance']
        self.get_logger().info(
            f'Fingertip clearance: side={side}, measured={actual_clearance*1000:.2f} mm, '
            f'target={desired*1000:.2f} mm')
        if abs(actual_clearance-desired) > .001:
            raise RuntimeError(
                f'Fingertip clearance out of 1 mm tolerance: {actual_clearance*1000:.2f} mm; '
                'jaws remain open')
        self.report['fingertip_clearance'] = {'side': side, 'target': desired, 'measured': actual_clearance}

    def acquire_initial_pick(self, first, second):
        failures = self.report.setdefault('pick_attempts', [])
        maximum = self.cfg['max_grasp_attempts']
        for attempt in range(1, maximum+1):
            try:
                result = self.pick_once(first, second, attempt)
                failures.append({'attempt': attempt, 'status': 'confirmed'})
                self.save_report()
                return result
            except Exception as exc:
                if self.stop_event.is_set():
                    raise
                # An error after acquisition can be a scene/planning/execution
                # failure. It is not proof of an empty grasp. Also query owner
                # for a service timeout after the simulator attached the bolt.
                try:
                    owner = self.io.grasp_owner()
                except Exception as ownership_error:
                    self.pick_secured = True
                    raise RuntimeError(f'Pick interrupted ({exc}); ownership unknown '
                        f'({ownership_error}); retaining jaws closed') from exc
                if self.pick_secured or owner:
                    self.pick_secured = True
                    failures.append({'attempt': attempt, 'status': 'holding_interrupted',
                                     'reason': str(exc)})
                    self.publish('HOLDING_INTERRUPTED',
                        f'Grasp acquired; lift/verification interrupted: {exc}; jaws remain closed')
                    raise RuntimeError(f'Grasp retained; lift/verification failed: {exc}; '
                                       'no automatic release or regrasp') from exc
                failures.append({'attempt': attempt, 'status': 'failed', 'reason': str(exc)})
                self.publish('GRASP_RETRY',
                    f'Pick attempt {attempt}/{maximum} failed: {exc}; cleaning up for retry')
                try:
                    self.reset_failed_pick(first)
                except Exception as cleanup:
                    raise RuntimeError(f'Pick failed ({exc}); retry cleanup failed: {cleanup}') from cleanup
                if attempt == maximum:
                    raise RuntimeError(f'Grasp not confirmed after {maximum} attempts: {exc}') from exc
        raise AssertionError('unreachable')

    def run(self, retry=False):
        try:
            self.output = Path(self.cfg['output_directory']).expanduser()/datetime.now().strftime('%Y%m%d_%H%M%S_%f')
            self.output.mkdir(parents=True)
            (self.output/'config.yaml').write_text(yaml.safe_dump(self.cfg), encoding='utf-8')
            self.io.state()
            if self.io.grasp_owner():
                raise RuntimeError('A gripper already owns the object; retaining grasp, no retry motion')
            if retry:
                self.publish('RETRY_CLEARANCE',
                    f'Open empty jaws and lift locally {self.cfg["retry_lift_height"]*1000:.0f} mm before new detection')
                prepare_pick_retry(self.io, self.cfg['first_arm'], self.arms, self.cfg,
                                   self.pick_target, self.settle)
            self.capture('camera_check')
            first = self.cfg['first_arm']
            second = 'left' if first == 'right' else 'right'
            obj, donor_grasp, receiver_grasp, test_pose = self.acquire_initial_pick(first, second)
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
            if first == 'right':
                self.handover_context = (first, second, neutral.copy(), donor_grasp.copy(), receiver_grasp.copy())
            self.scan(first, neutral, donor_grasp)
            self.finish_handover(first, second, neutral, donor_grasp, receiver_grasp)
        except Exception as exc:
            self.io.cancel()
            self.publish('STOPPED' if self.stop_event.is_set() else 'FAILED', str(exc))
        finally:
            self.handover_requested.clear()

    def finish_handover(self, first, second, neutral, donor_grasp, receiver_grasp):
        self.io.check()
        if self.handover_requested.is_set():
            self.report['right_inspection_skipped_by_operator'] = True
        self.publish('HANDOVER_PREPARE', 'Checking retained grasp and moving to handover pose')
        if self.io.grasp_owner() != first:
            raise RuntimeError('Donor ownership not confirmed; no handover motion sent')
        # Recover the actual held pose, including a stopped scan mid-rotation.
        # Keep the original grasp transform and refresh the scene before moving.
        actual = self.io.truth()@np.linalg.inv(self.object_truth)
        self.io.object_scene(actual, first)
        handover = neutral.copy()
        handover[:3, 3] = self.cfg['handover_center']
        self.io.global_move(first, handover@donor_grasp)
        self.settle()
        self.verify(handover)
        # Once receiver approach starts, this shortcut must not repeat a transfer.
        self.handover_context = None
        self.publish('HANDOVER_APPROACH', second+' approaching from the side with perpendicular jaws')
        self.io.allow_touch([second+'_left_finger', second+'_right_finger'])
        target = handover@receiver_grasp
        pre = target.copy()
        pre[:3, 3] -= target[:3, 2]*self.cfg['approach_height']
        self.io.global_move(second, pre)
        self.io.cartesian(second, [target], self.cfg['descent_speed'])
        actual_tcp = self.io.tcp_pose(second)
        position_error = float(np.linalg.norm(actual_tcp[:3, 3]-target[:3, 3]))
        angle_error = float(Rotation.from_matrix(target[:3, :3].T@actual_tcp[:3, :3]).magnitude())
        self.get_logger().info(
            f'Receiver reach check: position_error={position_error*1000:.3f} mm, '
            f'angle_error={angle_error:.5f} rad, shaft_offset={receiver_grasp[0, 3]*1000:.2f} mm')
        if position_error > .001 or angle_error > .02:
            raise RuntimeError('Receiver TCP has not reached shaft grasp; donor retains object')
        self.publish('HANDOVER_CONFIRM', 'Validate receiver before opening donor')
        transfer(self.io, first, second, handover, self.cfg['close_width'],
                 self.cfg['open_width'], self.settle, self.verify)
        retreat = handover@donor_grasp
        retreat[:3, 3] -= retreat[:3, 2]*self.cfg['approach_height']
        self.io.cartesian(first, [retreat], self.cfg['transfer_speed'])
        self.io.allow_touch([first+'_left_finger', first+'_right_finger'], False)
        self.io.global_move(first, joints=self.arms[first]['initial'])
        self.publish('HANDOVER_FOLLOW_TEST', 'Receiver moves part 10 mm; verify pose tracking before inspection')
        probe = handover.copy()
        probe[2, 3] += .010
        self.io.cartesian(second, [probe@receiver_grasp], self.cfg['descent_speed'])
        self.settle()
        self.verify(probe)
        if self.io.grasp_owner() != second:
            raise RuntimeError('Receiver ownership lost during follow test')
        self.publish('HANDOVER_CONFIRMED', 'Receiver passed motion follow test')
        self.io.cartesian(second, [neutral@receiver_grasp], self.cfg['transfer_speed'])
        self.scan(second, neutral, receiver_grasp)
        skipped = sum(v['status'] == 'unreachable' for v in self.report['views'])
        self.publish('DONE_HOLDING_'+second.upper(),
            f'Handover and receiver inspection complete; receiver retains part. '
            f'Right inspection manually skipped={self.report.get("right_inspection_skipped_by_operator", False)}. '
            f'Unreachable views: {skipped}. Results: {self.output}')


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
