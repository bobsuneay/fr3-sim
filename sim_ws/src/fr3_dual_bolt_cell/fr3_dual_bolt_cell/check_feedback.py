"""Read-only ROS subscriber: require fresh finite feedback from both arms."""
import argparse
import math
import time


def main():
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import JointState
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--timeout', type=float, default=20)
    opts = parser.parse_args()
    if not math.isfinite(opts.timeout) or opts.timeout <= 0:
        parser.error('timeout must be positive and finite')
    expected = {f'{side}_j{i}' for side in ('left', 'right') for i in range(1, 7)}
    expected |= {'left_left_finger_joint', 'left_right_finger_joint',
                 'right_left_finger_joint', 'right_right_finger_joint',
                 # The real HKV hardware exposes one command joint per gripper.
                 'left_gripper_joint', 'right_gripper_joint'}
    seen = {}
    counts = {}
    rclpy.init()
    node = Node('dual_cell_feedback_check')

    def receive(msg):
        now = time.monotonic()
        for name, value in zip(msg.name, msg.position):
            if name in expected and math.isfinite(value):
                seen[name] = now
                counts[name] = counts.get(name, 0)+1

    subscription = node.create_subscription(JointState, '/joint_states', receive, qos_profile_sensor_data)
    deadline = time.monotonic()+opts.timeout
    try:
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=.1)
            required = {j for j in expected if not j.endswith('_gripper_joint')}
            if all(time.monotonic()-seen.get(j, -1e20) < 1 and counts.get(j, 0) >= 5 for j in required):
                node.get_logger().info('PASS: all 14 commanded joints have repeated fresh finite feedback')
                return
        raise RuntimeError('Missing/stale feedback: '+str(sorted(
            j for j in expected if not j.endswith('_gripper_joint') and
            (time.monotonic()-seen.get(j, -1e20) >= 1 or counts.get(j, 0) < 5)))
    finally:
        node.destroy_subscription(subscription)
        node.destroy_node()
        rclpy.shutdown()
