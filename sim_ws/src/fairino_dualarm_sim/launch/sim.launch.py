from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    pkg = FindPackageShare('fairino_dualarm_sim')
    urdf = PathJoinSubstitution([pkg, 'urdf', 'dual_arm.urdf.xacro'])
    params = PathJoinSubstitution([pkg, 'config', 'params.yaml'])
    rviz = PathJoinSubstitution([pkg, 'rviz', 'dual_arm.rviz'])
    return LaunchDescription([
        DeclareLaunchArgument('run_demo', default_value='true'),
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             parameters=[{'robot_description': Command(['xacro ', urdf])}]),
        Node(package='fairino_dualarm_sim', executable='sim_controller',
             parameters=[params]),
        Node(package='fairino_dualarm_sim', executable='sim_vision',
             parameters=[params]),
        Node(package='fairino_dualarm_sim', executable='grasp_demo',
             parameters=[params], condition=None),
        Node(package='rviz2', executable='rviz2', name='rviz2',
             arguments=['-d', rviz], output='screen'),
    ])
