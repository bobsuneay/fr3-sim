import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState

class GraspDemo(Node):
    def __init__(self):
        super().__init__('grasp_demo')
        self.target = None; self.stage = 'WAIT_VISION'; self.sent = False
        self.joints = [f'left_joint{i}' for i in range(1, 7)]
        self.pub = self.create_publisher(JointState, '/sim/joint_target', 10)
        self.create_subscription(PoseStamped, '/vision/object_pose', self.pose_cb, 10)
        self.timer = self.create_timer(1.0, self.tick)
        self.get_logger().info('仿真抓取状态机已启动，等待视觉目标')
    def pose_cb(self, msg):
        if self.target is None: self.target = msg.pose.position
    def send(self, q, label):
        msg = JointState(); msg.name = self.joints; msg.position = q; self.pub.publish(msg)
        self.get_logger().info(label)
    def tick(self):
        if self.target is None: return
        if self.stage == 'WAIT_VISION':
            self.send([0.0, -0.55, 1.05, 0.0, 0.50, 0.0], 'MOVE_PRE_GRASP'); self.stage = 'PRE_GRASP'
        elif self.stage == 'PRE_GRASP':
            self.send([0.0, -0.72, 1.35, 0.0, 0.65, 0.0], 'MOVE_APPROACH'); self.stage = 'APPROACH'
        elif self.stage == 'APPROACH':
            self.get_logger().info('CLOSE_GRIPPER (fake)'); self.stage = 'CLOSE'
        elif self.stage == 'CLOSE':
            self.send([0.0, -0.45, 1.05, 0.0, 0.50, 0.0], 'LIFT'); self.stage = 'LIFT'
        elif self.stage == 'LIFT':
            self.send([0.75, -0.45, 1.05, 0.0, 0.50, 0.0], 'MOVE_PLACE'); self.stage = 'PLACE'
        elif self.stage == 'PLACE':
            self.get_logger().info('OPEN_GRIPPER (fake)'); self.stage = 'DONE'
        elif self.stage == 'DONE':
            self.get_logger().info('一次仿真抓取完成；重新启动节点可再次运行'); self.destroy_timer(self.timer); self.stage = 'FINISHED'

def main():
    rclpy.init(); rclpy.spin(GraspDemo()); rclpy.shutdown()
