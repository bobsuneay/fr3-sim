import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

class SimController(Node):
    def __init__(self):
        super().__init__('sim_controller')
        self.joints = [f'{side}_joint{i}' for side in ('left', 'right') for i in range(1, 7)]
        self.q = [0.0] * 12
        self.target = [0.0] * 12
        self.create_subscription(JointState, '/sim/joint_target', self.target_cb, 10)
        self.pub = self.create_publisher(JointState, '/joint_states', 10)
        self.create_timer(0.02, self.step)
    def target_cb(self, msg):
        for name, value in zip(msg.name, msg.position):
            if name in self.joints:
                self.target[self.joints.index(name)] = float(value)
    def step(self):
        for i in range(12):
            d = self.target[i] - self.q[i]
            self.q[i] += max(-0.025, min(0.025, d))
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name, msg.position = self.joints, self.q
        self.pub.publish(msg)

def main():
    rclpy.init(); rclpy.spin(SimController()); rclpy.shutdown()
