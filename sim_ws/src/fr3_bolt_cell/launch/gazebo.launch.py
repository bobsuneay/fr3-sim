"""Humble + Gazebo Classic only. No physical robot driver is launched."""
from pathlib import Path
import tempfile

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, EmitEvent, IncludeLaunchDescription,
                            LogInfo, OpaqueFunction, RegisterEventHandler)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit, OnShutdown
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
import xacro
import yaml

from fr3_bolt_cell.world import load_scene, world_xml


def start_cell(context):
    share = Path(get_package_share_directory('fr3_bolt_cell'))
    scene_file = Path(LaunchConfiguration('scene').perform(context)).expanduser().resolve()
    scene = load_scene(scene_file)
    moveit_enabled = LaunchConfiguration('moveit').perform(context).lower() in ('true', '1')
    run_dir = tempfile.TemporaryDirectory(prefix='fr3_bolt_cell_')
    world_file = Path(run_dir.name)/'cell.world'
    world_file.write_text(world_xml(scene), encoding='utf-8')
    urdf = xacro.process_file(str(share/'urdf/cell.urdf.xacro'), mappings={
        'arm_file': (share/'urdf/fr3_arm.urdf').as_posix(),
        'scene_file': scene_file.as_posix(),
        'initial_file': (share/'config/initial_positions.yaml').as_posix(),
        'controllers_file': (share/'config/controllers.yaml').as_posix(),
    }).toxml()
    robot = {'robot_description': ParameterValue(urdf, value_type=str)}

    def read_yaml(name):
        with open(share/'config'/name, encoding='utf-8') as stream:
            return yaml.safe_load(stream)

    moveit = {
        **robot,
        'robot_description_semantic': (share/'config/cell.srdf').read_text(encoding='utf-8'),
        'robot_description_kinematics': read_yaml('kinematics.yaml'),
        'robot_description_planning': read_yaml('joint_limits.yaml'),
        'planning_pipelines': ['ompl'], 'default_planning_pipeline': 'ompl',
        'ompl': read_yaml('ompl.yaml'),
        **read_yaml('moveit_controllers.yaml'),
        'use_sim_time': True,
        'allow_trajectory_execution': True,
        'publish_robot_description_semantic': True,
        'publish_planning_scene': True,
        'publish_geometry_updates': True,
        'publish_state_updates': True,
        'publish_transforms_updates': True,
        'trajectory_execution.allowed_execution_duration_scaling': 2.0,
        'trajectory_execution.allowed_goal_duration_margin': 1.0,
        'trajectory_execution.allowed_start_tolerance': 0.01,
    }
    rsp = Node(package='robot_state_publisher', executable='robot_state_publisher',
               name='robot_state_publisher', parameters=[robot, {'use_sim_time': True}],
               output='screen')
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(Path(get_package_share_directory('gazebo_ros'))/
                                           'launch/gazebo.launch.py')),
        launch_arguments={'world': str(world_file), 'gui': LaunchConfiguration('gui'),
                          'verbose': 'true', 'pause': 'false'}.items())
    spawn = Node(package='gazebo_ros', executable='spawn_entity.py',
                 arguments=['-entity', 'fr3_cell', '-topic', 'robot_description',
                            '-timeout', '120'], output='screen')

    def spawner(name):
        return Node(package='controller_manager', executable='spawner',
                    arguments=[name, '--controller-manager', '/controller_manager',
                               '--controller-manager-timeout', '120'], output='screen')

    jsb = spawner('joint_state_broadcaster')
    arm = spawner('fairino3_controller')
    grip = spawner('gripper_controller')
    move_group = Node(package='moveit_ros_move_group', executable='move_group',
                      parameters=[moveit], output='screen',
                      condition=IfCondition(LaunchConfiguration('moveit')))
    scene_node = Node(package='fr3_bolt_cell', executable='publish_scene',
                      parameters=[{'use_sim_time': True, 'scene_file': str(scene_file)}],
                      condition=IfCondition(LaunchConfiguration('moveit')), output='screen')
    rviz = Node(package='rviz2', executable='rviz2',
                arguments=['-d', str(share/'rviz/cell.rviz')],
                parameters=[moveit], condition=IfCondition(LaunchConfiguration('rviz')),
                output='screen')

    def after_success(next_actions, stage):
        def callback(event, _context):
            if event.returncode != 0:
                return [LogInfo(msg=f'FR3 cell failed at {stage}; stopping launch.'),
                        EmitEvent(event=Shutdown(reason=f'{stage} failed'))]
            return next_actions
        return callback

    # Register handlers BEFORE processes. No arbitrary startup sleeps or duplicate managers.
    handlers = [
        RegisterEventHandler(OnProcessExit(target_action=spawn,
            on_exit=after_success([jsb], 'spawn_entity'))),
        RegisterEventHandler(OnProcessExit(target_action=jsb,
            on_exit=after_success([arm], 'joint_state_broadcaster'))),
        RegisterEventHandler(OnProcessExit(target_action=arm,
            on_exit=after_success([grip], 'fairino3_controller'))),
        RegisterEventHandler(OnProcessExit(target_action=grip,
            on_exit=after_success([move_group, scene_node] if moveit_enabled else [rviz],
                                  'gripper_controller'))),
        RegisterEventHandler(OnProcessExit(target_action=scene_node,
            on_exit=after_success([rviz], 'MoveIt static planning scene'))),
        RegisterEventHandler(OnShutdown(on_shutdown=lambda event, context: run_dir.cleanup())),
    ]
    return handlers + [
        LogInfo(msg='Simulation only: side-mounted FR3, fixed head RGB-D and 25 mm bolts.'),
        LogInfo(msg='Generated world: '+str(world_file)), rsp, gazebo, spawn]


def generate_launch_description():
    share = Path(get_package_share_directory('fr3_bolt_cell'))
    return LaunchDescription([
        DeclareLaunchArgument('gui', default_value='true', description='Gazebo client window'),
        DeclareLaunchArgument('rviz', default_value='true', description='RViz / MoveIt window'),
        DeclareLaunchArgument('moveit', default_value='true', description='Start MoveIt move_group'),
        DeclareLaunchArgument('scene', default_value=str(share/'config/scene.yaml')),
        OpaqueFunction(function=start_cell),
    ])
