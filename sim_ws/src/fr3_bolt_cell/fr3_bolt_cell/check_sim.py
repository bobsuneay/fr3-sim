"""Read-only live acceptance check. Fails instead of reporting a non-running scene as OK."""
import argparse
import math
from pathlib import Path
import struct
import time

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from ament_index_python.packages import get_package_share_directory
from controller_manager_msgs.srv import ListControllers
from gazebo_msgs.msg import ModelStates
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import CameraInfo, Image, JointState, PointCloud2
from tf2_ros import Buffer, TransformListener

from fr3_bolt_cell.world import load_scene, bolt_poses


def has_finite_cloud_point(cloud):
    fields = {f.name: f for f in cloud.fields}
    if not all(name in fields and fields[name].datatype == 7 for name in ('x', 'y', 'z')):
        return False
    count = cloud.width*cloud.height
    if not count or not cloud.data:
        return False
    endian = '>' if cloud.is_bigendian else '<'
    # Sample on a 2D grid, accounting for possible row padding.
    for index in range(0, count, max(1, count//300)):
        row, col = divmod(index, cloud.width)
        base = row*cloud.row_step + col*cloud.point_step
        values = [struct.unpack_from(endian+'f', cloud.data, base+fields[k].offset)[0]
                  for k in ('x', 'y', 'z')]
        if all(math.isfinite(v) for v in values) and values[2] > 0:
            return True
    return False


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--timeout', type=float, default=60.0)
    parser.add_argument('--scene', type=Path)
    args, ros_args = parser.parse_known_args()
    if args.timeout <= 0:
        parser.error('--timeout must be positive')
    scene_path = args.scene or Path(get_package_share_directory('fr3_bolt_cell'))/'config/scene.yaml'
    scene = load_scene(scene_path)
    c = scene['camera']
    expected_bolts = {name for name, _ in bolt_poses(scene)}
    rclpy.init(args=ros_args)
    node = Node('fr3_cell_acceptance', parameter_overrides=[Parameter('use_sim_time', value=True)])
    latest, received_at, ticks = {}, {}, []
    subscriptions = []
    def receive(key):
        def callback(message):
            latest[key] = message
            received_at[key] = time.monotonic()
        return callback
    def clock_callback(message):
        value = message.clock.sec+message.clock.nanosec*1e-9
        if not ticks:
            ticks.append(value)
        elif len(ticks) == 1:
            ticks.append(value)
        else:
            ticks[1] = value
        received_at['clock'] = time.monotonic()
    for key, kind, topic in (
        ('joints', JointState, '/joint_states'),
        ('rgb', Image, '/head_camera/image_raw'),
        ('depth', Image, '/head_camera/depth/image_raw'),
        ('info', CameraInfo, '/head_camera/camera_info'),
        ('cloud', PointCloud2, '/head_camera/points'),
        ('models', ModelStates, '/gazebo/model_states'),
    ):
        subscriptions.append(node.create_subscription(kind, topic, receive(key), qos_profile_sensor_data))
    subscriptions.append(node.create_subscription(Clock, '/clock', clock_callback, qos_profile_sensor_data))
    buffer = Buffer()
    listener = TransformListener(buffer, node)
    client = node.create_client(ListControllers, '/controller_manager/list_controllers')
    future = None
    checks = {}
    deadline = time.monotonic()+args.timeout
    try:
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
            now = time.monotonic()
            fresh = lambda key: now-received_at.get(key, -math.inf) < 5.0
            checks['advancing /clock'] = len(ticks) == 2 and ticks[1] > ticks[0] and fresh('clock')
            joints = latest.get('joints')
            checks['eight finite joints'] = (
                fresh('joints') and joints is not None
                and set(['j1','j2','j3','j4','j5','j6','left_finger_joint','right_finger_joint']).issubset(joints.name)
                and len(joints.name) == len(joints.position)
                and all(math.isfinite(v) for v in joints.position))
            for key in ('rgb', 'depth', 'info', 'cloud'):
                msg = latest.get(key)
                checks[key+' image/frame'] = (
                    fresh(key) and msg is not None and msg.width == c['width']
                    and msg.height == c['height']
                    and msg.header.frame_id == 'head_camera_optical_frame')
            depth = latest.get('depth')
            checks['depth encoding 32FC1'] = depth is not None and depth.encoding == '32FC1'
            cloud = latest.get('cloud')
            checks['nonempty finite point cloud'] = cloud is not None and has_finite_cloud_point(cloud)
            checks['world -> optical TF'] = buffer.can_transform('world', 'head_camera_optical_frame', Time())
            checks['world -> TCP TF'] = buffer.can_transform('world', 'gripper_tcp', Time())
            models = latest.get('models')
            checks['all bolt models present'] = (
                fresh('models') and models is not None
                and expected_bolts.issubset(models.name)
                and 'fr3_cell' in models.name and 'work_table' in models.name)
            checks['bolt positions above tabletop'] = False
            if checks['all bolt models present']:
                positions = [models.pose[models.name.index(name)].position for name in expected_bolts]
                checks['bolt positions above tabletop'] = all(
                    all(math.isfinite(v) for v in (p.x, p.y, p.z))
                    and scene['table']['top_z'] <= p.z <= scene['table']['top_z']+0.04
                    for p in positions)
            if future is None and client.service_is_ready():
                future = client.call_async(ListControllers.Request())
            checks['three active controllers'] = False
            if future is not None and future.done():
                response = future.result()
                active = {c.name for c in response.controller if c.state == 'active'}
                checks['three active controllers'] = {
                    'joint_state_broadcaster','fairino3_controller','gripper_controller'}.issubset(active)
                if not checks['three active controllers']:
                    future = None
            if all(checks.values()):
                break
        for name, passed in checks.items():
            print(('PASS ' if passed else 'FAIL ')+name)
        if not checks or not all(checks.values()):
            raise SystemExit(1)
        print('Runtime interfaces passed. This does not certify grasp stability or visual defect resolution.')
    finally:
        node.destroy_node()
        rclpy.shutdown()
