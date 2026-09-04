"""Command the simulated parallel gripper opening (metres); not a grasp-force controller."""
import argparse
import math
import time

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from control_msgs.action import FollowJointTrajectory
from rosgraph_msgs.msg import Clock
from trajectory_msgs.msg import JointTrajectoryPoint


def width_to_positions(width):
    if not math.isfinite(width) or not 0.0 <= width <= 0.060:
        raise ValueError('width must be 0.000 .. 0.060 metres')
    return [width/2, width/2]


def wait_result(node, future, timeout):
    rclpy.spin_until_future_complete(node, future, timeout_sec=timeout)
    if not future.done():
        raise TimeoutError('Timed out waiting for gripper action')
    return future.result()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--width', type=float, required=True,
                        help='Distance between the inner finger faces, in metres')
    parser.add_argument('--seconds', type=float, default=3.0)
    args, ros_args = parser.parse_known_args()
    positions = width_to_positions(args.width)
    if not math.isfinite(args.seconds) or args.seconds < 2.0:
        parser.error('--seconds must be at least 2.0')
    rclpy.init(args=ros_args)
    node = Node('fr3_sim_gripper_command',
                parameter_overrides=[Parameter('use_sim_time', value=True)])
    handle = None
    try:
        ticks = []
        def on_clock(message):
            stamp = message.clock.sec + message.clock.nanosec*1e-9
            if not ticks or stamp != ticks[-1]:
                ticks.append(stamp)
        subscription = node.create_subscription(Clock, '/clock', on_clock, qos_profile_sensor_data)
        deadline = time.monotonic()+10.0
        while len(ticks) < 2 and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
        if len(ticks) < 2 or ticks[-1] <= ticks[0]:
            raise RuntimeError('No advancing /clock; start and unpause this Gazebo cell first')
        client = ActionClient(node, FollowJointTrajectory,
                              '/gripper_controller/follow_joint_trajectory')
        if not client.wait_for_server(timeout_sec=15.0):
            raise RuntimeError('gripper_controller action server unavailable')
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = ['left_finger_joint', 'right_finger_joint']
        point = JointTrajectoryPoint()
        point.positions = positions
        point.velocities = [0.0, 0.0]
        point.time_from_start.sec = int(args.seconds)
        point.time_from_start.nanosec = int((args.seconds-int(args.seconds))*1e9)
        goal.trajectory.points = [point]
        handle = wait_result(node, client.send_goal_async(goal), 10.0)
        if not handle.accepted:
            raise RuntimeError('Gripper trajectory was rejected')
        result = wait_result(node, handle.get_result_async(), args.seconds*3+15).result
        if result.error_code != FollowJointTrajectory.Result.SUCCESSFUL:
            raise RuntimeError(f'Controller error {result.error_code}: {result.error_string}')
        node.get_logger().info(f'Completed opening command: {args.width*1000:.1f} mm')
    except Exception as exc:
        if handle is not None and handle.accepted:
            rclpy.spin_until_future_complete(node, handle.cancel_goal_async(), timeout_sec=2.0)
        node.get_logger().error(str(exc))
        raise SystemExit(1)
    finally:
        node.destroy_node()
        rclpy.shutdown()
