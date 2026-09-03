import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker

class SimVision(Node):
    def __init__(self):
        super().__init__('sim_vision')
        self.pub = self.create_publisher(PoseStamped, '/vision/object_pose', 10)
        self.marker_pub = self.create_publisher(Marker, '/vision/object_marker', 10)
        self.create_timer(0.2, self.publish_detection)
    def publish_detection(self):
        now = self.get_clock().now().to_msg()
        p = PoseStamped(); p.header.stamp = now; p.header.frame_id = 'world'
        p.pose.position.x, p.pose.position.y, p.pose.position.z = 0.42, 0.0, 0.91
        p.pose.orientation.w = 1.0; self.pub.publish(p)
        m = Marker(); m.header = p.header; m.ns = 'sim_vision'; m.id = 0
        m.type, m.action = Marker.SPHERE, Marker.ADD; m.pose = p.pose
        m.scale.x = m.scale.y = m.scale.z = 0.11; m.color.g = 1.0; m.color.a = 0.8
        self.marker_pub.publish(m)

def main():
    rclpy.init(); rclpy.spin(SimVision()); rclpy.shutdown()
