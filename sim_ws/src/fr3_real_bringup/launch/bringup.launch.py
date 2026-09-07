"""No Gazebo, no arm home command, no gripper activation."""
from pathlib import Path
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory, get_packages_with_prefixes
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, LogInfo, OpaqueFunction, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
import xacro

from fr3_real_bringup.configuration import (
    exports_plugin, load_yaml, validate_cell, validate_real,
)


def start(context):
    share = Path(get_package_share_directory('fr3_real_bringup'))
    arg = lambda name: LaunchConfiguration(name).perform(context)

    def flag(name):
        value = arg(name).lower()
        if value not in ('true', 'false'):
            raise ValueError(f'{name} must be true or false')
        return value == 'true'

    mode = arg('mode')
    if mode not in ('mock', 'real'):
        raise ValueError('mode must be mock or real')
    execute = flag('enable_execution')
    cell = Path(arg('cell')).expanduser().resolve()
    validate_cell(load_yaml(cell))
    update_rate = 100
    if mode == 'real':
        real = validate_real(load_yaml(Path(arg('real_config')).expanduser()),
                             flag('confirm_real'))
        selected = real['driver_package']
        # Multiple version packages may export the identical plugin class/library.
        exporters = [
            name for name in get_packages_with_prefixes()
            if name.startswith('fairino_hardware')
            and exports_plugin(get_package_share_directory(name))
        ]
        if exporters != [selected]:
            raise RuntimeError(f'Expected only {selected} to export the hardware plugin; '
                               f'found {exporters}. Source one matching driver underlay.')
        update_rate = real['update_rate']

    document = xacro.process_file(str(share/'urdf/robot.urdf.xacro'), mappings={
        'mode': mode, 'cell_file': cell.as_posix(),
        'arm_file': (share/'urdf/fr3_arm.urdf').as_posix(),
        'initial_file': (share/'config/initial_positions.yaml').as_posix(),
        'serial_port': arg('serial_port'),
        'baud_rate': arg('baud_rate'),
        'timeout': arg('timeout'),
        'slave_address': arg('slave_address'),
        'position_mode_speed_register': arg('position_mode_speed_register'),
        'target_force_percent': arg('target_force_percent'),
        'gripper_closed_position': arg('gripper_closed_position'),
    })
    urdf = document.toxml()
    for mesh in ET.fromstring(urdf).iter('mesh'):
        uri = mesh.get('filename')
        path = share / uri.removeprefix('package://fr3_real_bringup/')
        if not path.is_file():
            raise FileNotFoundError(f'Missing installed mesh: {path}; rebuild and source workspace')
    robot = {'robot_description': ParameterValue(urdf, value_type=str),
             'use_sim_time': False}
    read = lambda name: load_yaml(share/'config'/name)
    moveit = {
        **robot,
        'robot_description_semantic': (share/'config/robot.srdf').read_text(encoding='utf-8'),
        'robot_description_kinematics': read('kinematics.yaml'),
        'robot_description_planning': read('joint_limits.yaml'),
        'planning_pipelines': ['ompl'], 'default_planning_pipeline': 'ompl',
        'ompl': read('ompl.yaml'), **read('moveit_controllers.yaml'),
        'allow_trajectory_execution': execute,
        'publish_robot_description_semantic': True,
        'publish_planning_scene': True, 'publish_geometry_updates': True,
        'publish_state_updates': True, 'publish_transforms_updates': True,
        'trajectory_execution.allowed_execution_duration_scaling': 1.5,
        'trajectory_execution.allowed_goal_duration_margin': 1.0,
        'trajectory_execution.allowed_start_tolerance': 0.01,
    }
    rsp = Node(package='robot_state_publisher', executable='robot_state_publisher',
               parameters=[robot], output='screen')
    manager = Node(package='controller_manager', executable='ros2_control_node',
                   parameters=[robot, str(share/'config/controllers.yaml'),
                               {'update_rate': update_rate}], output='screen')

    def spawner(name, inactive=False):
        args = [name, '-c', '/controller_manager', '--controller-manager-timeout', '60']
        if inactive:
            args.append('--inactive')
        return Node(package='controller_manager', executable='spawner', prefix='/usr/bin/python3',
                    arguments=args, output='screen')

    jsb = spawner('joint_state_broadcaster')
    arm = spawner('fairino3_controller', inactive=not execute)
    gripper = spawner('tg9801_gripper_controller', inactive=not execute)
    move_group = Node(package='moveit_ros_move_group', executable='move_group',
                      parameters=[moveit], output='screen')
    rviz = Node(package='rviz2', executable='rviz2',
                arguments=['-d', str(share/'rviz/real.rviz')], parameters=[moveit],
                condition=IfCondition(LaunchConfiguration('rviz')), output='screen')

    def success(next_actions, name):
        def callback(event, _context):
            if event.returncode != 0:
                return [EmitEvent(event=Shutdown(reason=f'{name} failed'))]
            return next_actions
        return callback

    handlers = [
        RegisterEventHandler(OnProcessExit(target_action=jsb,
            on_exit=success([arm], 'joint_state_broadcaster'))),
        RegisterEventHandler(OnProcessExit(target_action=arm,
            on_exit=success([gripper], 'fairino3_controller'))),
        RegisterEventHandler(OnProcessExit(target_action=gripper,
            on_exit=success([move_group, rviz], 'tg9801_gripper_controller'))),
    ]
    for process in (manager, rsp, move_group):
        handlers.append(RegisterEventHandler(OnProcessExit(target_action=process,
            on_exit=[EmitEvent(event=Shutdown(reason='Required node exited'))])))
    warning = ('MOCK ONLY: no robot connection.' if mode == 'mock' else
               'REAL HARDWARE: driver may continuously send ServoJ even with '
               'enable_execution=false. This is NOT read-only mode. '
               'ROS shutdown is NOT a physical emergency stop.')
    return handlers + [LogInfo(msg=warning), rsp, manager, jsb]


def generate_launch_description():
    share = Path(get_package_share_directory('fr3_real_bringup'))
    return LaunchDescription([
        DeclareLaunchArgument('mode', default_value='mock'),
        DeclareLaunchArgument('enable_execution', default_value='false'),
        DeclareLaunchArgument('confirm_real', default_value='false'),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('cell', default_value=str(share/'config/cell.yaml')),
        DeclareLaunchArgument('real_config', default_value=str(share/'config/real.example.yaml')),
        DeclareLaunchArgument('serial_port', default_value='/dev/ttyACM0'),
        DeclareLaunchArgument('baud_rate', default_value='1000000'),
        DeclareLaunchArgument('timeout', default_value='1000'),
        DeclareLaunchArgument('slave_address', default_value='1'),
        DeclareLaunchArgument('position_mode_speed_register', default_value='1000'),
        DeclareLaunchArgument('target_force_percent', default_value='50'),
        DeclareLaunchArgument('gripper_closed_position', default_value='0.1'),
        OpaqueFunction(function=start),
    ])
