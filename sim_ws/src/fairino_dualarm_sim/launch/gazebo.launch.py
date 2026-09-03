from launch import LaunchDescription
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    pkg = FindPackageShare('fairino_dualarm_sim')
    urdf = PathJoinSubstitution([pkg, 'urdf', 'dual_arm.urdf.xacro'])
    description = {'robot_description': Command(['xacro ', urdf])}
    return LaunchDescription([
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             parameters=[description], output='screen'),
        Node(package='gazebo_ros', executable='gazebo',
             arguments=['--verbose', '-s', 'libgazebo_ros_factory.so'], output='screen'),
        Node(package='gazebo_ros', executable='spawn_entity.py',
             arguments=['-topic', 'robot_description', '-entity', 'fr3_bolt_cell'], output='screen'),
    ])
