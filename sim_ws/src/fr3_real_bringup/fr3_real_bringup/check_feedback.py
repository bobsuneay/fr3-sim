"""Observe /joint_states only; does not connect to an SDK or command the robot."""
import argparse
import math
import time
from .configuration import JOINTS


def extract_positions(names, positions):
    if len(names) != len(positions) or len(names) != len(set(names)):
        raise ValueError('Duplicate names or malformed position vector')
    values = dict(zip(names, positions))
    result = [values[name] for name in JOINTS]
    if not all(math.isfinite(value) for value in result):
        raise ValueError('Non-finite joint feedback')
    return result


def main():
    import rclpy
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import JointState
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--seconds', type=float, default=5.0)
    args = parser.parse_args()
    if not math.isfinite(args.seconds) or args.seconds < 1:
        parser.error('--seconds must be finite and >= 1')
    rclpy.init()
    node = rclpy.create_node('fr3_feedback_check')
    samples, invalid = [], []

    def receive(msg):
        try:
            values = extract_positions(msg.name, msg.position)
            stamp = msg.header.stamp.sec + msg.header.stamp.nanosec/1e9
            samples.append((time.monotonic(), stamp, values))
        except (KeyError, ValueError) as error:
            invalid.append(str(error))

    sub = node.create_subscription(JointState, '/joint_states', receive, qos_profile_sensor_data)
    start = time.monotonic()
    try:
        while rclpy.ok() and time.monotonic()-start < args.seconds:
            rclpy.spin_once(node, timeout_sec=.1)
        if len(samples) < 2 or time.monotonic()-samples[-1][0] > 1:
            raise RuntimeError('No fresh six-axis /joint_states; do NOT execute')
        age = node.get_clock().now().nanoseconds/1e9 - samples[-1][1]
        if not -0.1 <= age <= 1.0 or samples[-1][1] <= samples[0][1]:
            raise RuntimeError('JointState timestamps stale/invalid; check clocks and use_sim_time')
        rate = (len(samples)-1)/(samples[-1][0]-samples[0][0])
        print(f'Received {len(samples)} six-axis messages at approximately {rate:.1f} Hz')
        for name, value in zip(JOINTS, samples[-1][2]):
            print(f'{name}: {value:+.6f} rad = {math.degrees(value):+.3f} deg')
        if invalid:
            raise RuntimeError(f'Malformed/incomplete messages also observed: {invalid[0]}')
        print('Compare all six values with WebApp. This cannot detect cached SDK feedback, '
              'verify joint calibration, or distinguish mock from real hardware.')
    finally:
        node.destroy_subscription(sub)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
