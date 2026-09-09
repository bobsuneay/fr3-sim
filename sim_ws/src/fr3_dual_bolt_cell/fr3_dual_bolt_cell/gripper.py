"""Explicit, bounded GripperCommand client; no movement at bringup."""
import argparse
import math
from pathlib import Path

from fr3_dual_bolt_cell.model import read_yaml, validate_arms


def target_for_width(width, cfg):
    g = validate_arms(cfg)['gripper']
    minimum = g['open_gap']-2*g['finger_travel']
    if not math.isfinite(width) or not minimum <= width <= g['open_gap']:
        raise ValueError(f'width must be in [{minimum}, {g["open_gap"]}] metres')
    return (g['open_gap']-width)/2


def main():
    from ament_index_python.packages import get_package_share_directory
    import rclpy
    from rclpy.action import ActionClient
    from rclpy.node import Node
    from control_msgs.action import GripperCommand
    from action_msgs.msg import GoalStatus

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--arm', choices=['left', 'right'], required=True)
    parser.add_argument('--width', type=float, required=True, help='Inner gap in metres')
    parser.add_argument('--arms', type=Path, default=Path(get_package_share_directory(
        'fr3_dual_bolt_cell'))/'config/arms.yaml')
    opts = parser.parse_args()
    q = target_for_width(opts.width, read_yaml(opts.arms))
    rclpy.init()
    node = Node('dual_cell_gripper_client')
    client = ActionClient(node, GripperCommand, '/'+opts.arm+'_gripper_controller/gripper_cmd')
    try:
        if not client.wait_for_server(timeout_sec=10):
            raise RuntimeError('Gripper action unavailable; check active controller')
        goal = GripperCommand.Goal()
        goal.command.position = q
        goal.command.max_effort = 20.0
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
        if result.result().status != GoalStatus.STATUS_SUCCEEDED:
            raise RuntimeError('Gripper action did not succeed')
        node.get_logger().info(str(result.result().result))
    finally:
        client.destroy()
        node.destroy_node()
        rclpy.shutdown()
