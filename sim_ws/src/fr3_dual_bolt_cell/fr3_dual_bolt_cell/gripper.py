"""Command a dual finger trajectory, matching the original fr3_bolt_cell."""
import argparse
import math
from pathlib import Path

from fr3_dual_bolt_cell.model import read_yaml, validate_arms


def target_for_width(width, cfg):
    g = validate_arms(cfg)['gripper']
    if not math.isfinite(width) or not 0.0 <= width <= g['open_gap']:
        raise ValueError(f'width must be in [0, {g["open_gap"]}] metres')
    return width / 2.0


def main():
    from ament_index_python.packages import get_package_share_directory
    import rclpy
    from rclpy.action import ActionClient
    from rclpy.node import Node
    from control_msgs.action import FollowJointTrajectory
    from trajectory_msgs.msg import JointTrajectoryPoint

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--arm', choices=['left', 'right'], required=True)
    parser.add_argument('--width', type=float, required=True, help='Inner gap in metres')
    parser.add_argument('--seconds', type=float, default=3.0)
    parser.add_argument('--arms', type=Path, default=Path(get_package_share_directory(
        'fr3_dual_bolt_cell'))/'config/arms.yaml')
    opts = parser.parse_args()
    cfg = read_yaml(opts.arms)
    g = validate_arms(cfg)['gripper']
    if not math.isfinite(opts.seconds) or opts.seconds < 2.0:
        parser.error('--seconds must be at least 2.0')
    if not math.isfinite(opts.width) or not 0.0 <= opts.width <= g['open_gap']:
        parser.error(f'width must be in [0, {g["open_gap"]}] metres')
    q = opts.width / 2.0
    rclpy.init()
    node = Node('dual_cell_gripper_client')
    client = ActionClient(node, FollowJointTrajectory,
                          '/'+opts.arm+'_gripper_controller/follow_joint_trajectory')
    try:
        if not client.wait_for_server(timeout_sec=10):
            raise RuntimeError('Gripper action unavailable; check active controller')
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = [opts.arm+'_left_finger_joint', opts.arm+'_right_finger_joint']
        point = JointTrajectoryPoint()
        point.positions = [q, q]
        point.velocities = [0.0, 0.0]
        point.time_from_start.sec = int(opts.seconds)
        point.time_from_start.nanosec = int((opts.seconds-int(opts.seconds))*1e9)
        goal.trajectory.points = [point]
        future = client.send_goal_async(goal)
        rclpy.spin_until_future_complete(node, future, timeout_sec=10)
        if not future.done() or not future.result().accepted:
            raise RuntimeError('Gripper goal was not accepted')
        handle = future.result()
        result = handle.get_result_async()
        rclpy.spin_until_future_complete(node, result, timeout_sec=15)
        if not result.done():
            cancel = handle.cancel_goal_async()
            rclpy.spin_until_future_complete(node, cancel, timeout_sec=3)
            raise RuntimeError('Gripper result timed out; cancellation requested')
        if result.result().result.error_code != FollowJointTrajectory.Result.SUCCESSFUL:
            raise RuntimeError(result.result().result.error_string)
        node.get_logger().info(f'Completed {opts.arm} gripper command: {opts.width*1000:.1f} mm')
    finally:
        client.destroy()
        node.destroy_node()
        rclpy.shutdown()
