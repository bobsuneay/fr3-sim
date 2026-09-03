from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction
from launch.substitutions import Command, PathJoinSubstitution
from launch.substitutions import FindExecutable
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    pkg = FindPackageShare('fairino_dualarm_sim')
    urdf = PathJoinSubstitution([pkg, 'urdf', 'dual_arm.urdf.xacro'])
    # xacro output is XML text, not YAML. Explicitly typing it as a string
    # avoids ROS 2 launch trying to parse the URDF as a YAML parameter.
    description = {
        'robot_description': ParameterValue(
            Command(['xacro ', urdf]), value_type=str)
    }
    return LaunchDescription([
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             parameters=[description], output='screen'),
        # Gazebo Classic is a system executable, not an executable in the
        # gazebo_ros libexec directory. gazebo_ros supplies the ROS plugins.
        ExecuteProcess(
            cmd=[FindExecutable(name='gazebo'), '--verbose',
                 '-s', 'libgazebo_ros_factory.so'],
            output='screen'),
        # Wait for Gazebo's factory service and robot_state_publisher's
        # robot_description topic before inserting the model.
        TimerAction(period=5.0, actions=[
            Node(package='gazebo_ros', executable='spawn_entity.py',
                 arguments=['-topic', 'robot_description',
                            '-entity', 'fr3_bolt_cell', '-timeout', '30'],
                 output='screen')
        ]),
    ])
