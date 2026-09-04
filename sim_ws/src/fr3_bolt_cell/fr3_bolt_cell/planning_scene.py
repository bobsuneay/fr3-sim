"""Insert the same static tabletop and legs into MoveIt's planning scene."""
from pathlib import Path

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
from moveit_msgs.msg import CollisionObject
from moveit_msgs.srv import ApplyPlanningScene
from shape_msgs.msg import SolidPrimitive

from fr3_bolt_cell.world import load_scene, table_boxes


def main(args=None):
    rclpy.init(args=args)
    node = Node('fr3_static_planning_scene')
    node.declare_parameter('scene_file', '')
    try:
        scene = load_scene(Path(node.get_parameter('scene_file').value))
        client = node.create_client(ApplyPlanningScene, '/apply_planning_scene')
        if not client.wait_for_service(timeout_sec=60.0):
            raise RuntimeError('MoveIt /apply_planning_scene unavailable after 60 s')
        request = ApplyPlanningScene.Request()
        request.scene.is_diff = True
        request.scene.robot_state.is_diff = True
        for name, size, position in table_boxes(scene):
            obj = CollisionObject()
            obj.header.frame_id = 'world'
            obj.id = name
            obj.operation = CollisionObject.ADD
            primitive = SolidPrimitive()
            primitive.type = SolidPrimitive.BOX
            primitive.dimensions = [float(x) for x in size]
            pose = Pose()
            pose.position.x, pose.position.y, pose.position.z = [float(x) for x in position]
            pose.orientation.w = 1.0
            obj.primitives = [primitive]
            obj.primitive_poses = [pose]
            request.scene.world.collision_objects.append(obj)
        future = client.call_async(request)
        rclpy.spin_until_future_complete(node, future, timeout_sec=30.0)
        if not future.done() or not future.result().success:
            raise RuntimeError('MoveIt did not acknowledge the static scene')
        node.get_logger().info(
            'Tabletop + 4 legs inserted. Bolts are dynamic Gazebo bodies, '
            'not stale fixed MoveIt obstacles; add perception before automated picking.')
    except Exception as exc:
        node.get_logger().error(str(exc))
        raise SystemExit(1)
    finally:
        node.destroy_node()
        rclpy.shutdown()
