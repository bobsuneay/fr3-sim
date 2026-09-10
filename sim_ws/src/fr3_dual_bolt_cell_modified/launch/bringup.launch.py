"""A cold-start backend switch: stop the previous launch before changing mode."""
from pathlib import Path
import tempfile
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory, get_packages_with_prefixes
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, EmitEvent, IncludeLaunchDescription,
                            LogInfo, OpaqueFunction, RegisterEventHandler, SetEnvironmentVariable)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit, OnShutdown
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
import yaml

from fr3_dual_bolt_cell.model import (SIDES, build_model, controllers, manager_model,
    moveit_config, read_yaml, validate_hardware)
from fr3_dual_bolt_cell.world import load_scene, world_xml


def start(context):
    share = Path(get_package_share_directory('fr3_dual_bolt_cell'))
    arg = lambda name: LaunchConfiguration(name).perform(context)
    mode = arg('mode')
    if mode not in ('gazebo', 'mock', 'real'):
        raise ValueError('mode must be gazebo, mock or real')
    for name in ('enable_execution', 'rviz', 'gui'):
        if arg(name) not in ('true', 'false'):
            raise ValueError(f'{name} must be true or false')
    execute = arg('enable_execution') == 'true'
    sim = mode == 'gazebo'
    scene_path = Path(arg('scene')).expanduser().resolve()
    scene = load_scene(scene_path)
    arms = read_yaml(Path(arg('arms')).expanduser())
    hardware = None
    if mode == 'real':
        hardware = validate_hardware(read_yaml(Path(arg('hardware')).expanduser()))
        selected = hardware['driver_package']
        exporters = []
        for package in get_packages_with_prefixes():
            if package.startswith('fairino_hardware'):
                folder = Path(get_package_share_directory(package))
                if any('fairino_hardware/FairinoHardwareInterface' in f.read_text(encoding='utf-8')
                       for f in folder.rglob('*.xml')):
                    exporters.append(package)
        if exporters != [selected]:
            raise RuntimeError(f'Source exactly one matching FAIRINO driver; found {exporters}')
        marker = Path(get_package_share_directory(selected))/'dual_cell_adapter_v1.txt'
        if not marker.is_file():
            raise RuntimeError('Run tools/prepare_driver.py and rebuild the selected driver first')
        get_package_share_directory('ros2_hkv_gripper')
        get_package_share_directory('position_controllers')
        for side in SIDES:
            if not Path(hardware[side]['serial_port']).exists():
                raise ValueError(f'{side} serial device does not exist')
        if Path(hardware['left']['serial_port']).resolve() == Path(hardware['right']['serial_port']).resolve():
            raise ValueError('Both serial names resolve to the same device')

    temp = tempfile.TemporaryDirectory(prefix='fr3_dual_bolt_cell_')
    run = Path(temp.name)
    combined = run/'gazebo_controllers.yaml'
    combined.write_text(yaml.safe_dump(controllers('gazebo')), encoding='utf-8')
    root = build_model(share, scene_path, arms, mode, combined, hardware)
    for mesh in root.iter('mesh'):
        uri = mesh.get('filename')
        if not uri.startswith('package://fr3_dual_bolt_cell/'):
            raise ValueError(f'Unexpected mesh URI: {uri}')
        if not (share/uri.removeprefix('package://fr3_dual_bolt_cell/')).is_file():
            raise FileNotFoundError(uri)
    xml = ET.tostring(root, encoding='unicode')
    (run/'robot.urdf').write_text(xml, encoding='utf-8')
    description = {'robot_description': ParameterValue(xml, value_type=str), 'use_sim_time': sim}
    moveit = {**description, **moveit_config(root, arms, share),
              'allow_trajectory_execution': execute,
              'publish_robot_description_semantic': True, 'publish_planning_scene': True,
              'publish_geometry_updates': True, 'publish_state_updates': True,
              'publish_transforms_updates': True,
              'trajectory_execution.allowed_execution_duration_scaling': 2.0,
              'trajectory_execution.allowed_goal_duration_margin': 2.0,
              'trajectory_execution.allowed_start_tolerance': .01}
    # Avoid launch parsing the SRDF as YAML.
    moveit['robot_description_semantic'] = ParameterValue(moveit['robot_description_semantic'], value_type=str)
    rsp = Node(package='robot_state_publisher', executable='robot_state_publisher',
               name='robot_state_publisher', parameters=[description], output='screen')
    group = Node(package='moveit_ros_move_group', executable='move_group',
                 parameters=[moveit], output='screen')
    scene_node = Node(package='fr3_dual_bolt_cell', executable='publish_scene', prefix='/usr/bin/python3',
                      parameters=[{'scene_file': str(scene_path), 'use_sim_time': sim}], output='screen')
    rviz = Node(package='rviz2', executable='rviz2', parameters=[moveit],
                arguments=['-d', str(share/'rviz/cell.rviz')], output='screen',
                condition=IfCondition(LaunchConfiguration('rviz')))

    def success(actions, stage):
        def callback(event, _context):
            if event.returncode != 0:
                return [EmitEvent(event=Shutdown(reason=f'{stage} failed'))]
            return actions
        return callback

    handlers = [RegisterEventHandler(OnShutdown(on_shutdown=lambda event, context: temp.cleanup()))]
    processes = [rsp, group]
    startup = [rsp]
    spawners = []
    for side in SIDES:
        manager_name = 'controller_manager' if sim else side+'_controller_manager'
        if not sim:
            file = run/(side+'_controllers.yaml')
            file.write_text(yaml.safe_dump(controllers(mode, side)), encoding='utf-8')
            manager = Node(package='controller_manager', executable='ros2_control_node',
                # Scoped rename avoids renaming the HKV plugin's internal node.
                arguments=['--ros-args', '-r', f'controller_manager:__node:={manager_name}'],
                parameters=[{'robot_description': ParameterValue(manager_model(root, side), value_type=str),
                             'use_sim_time': False}, str(file)],
                remappings=[('/gripper_registers', '/'+side+'/gripper_registers')], output='screen')
            startup.append(manager)
            processes.append(manager)
        for suffix in ('joint_state_broadcaster', 'arm_controller', 'gripper_controller'):
            args = [side+'_'+suffix, '-c', '/'+manager_name, '--controller-manager-timeout', '120']
            if suffix != 'joint_state_broadcaster' and not execute:
                args.append('--inactive')
            spawners.append(Node(package='controller_manager', executable='spawner',
                                 prefix='/usr/bin/python3', arguments=args, output='screen'))
    for current, following in zip(spawners, spawners[1:]):
        handlers.append(RegisterEventHandler(OnProcessExit(target_action=current,
            on_exit=success([following], 'controller spawner'))))
    handlers.append(RegisterEventHandler(OnProcessExit(target_action=spawners[-1],
        on_exit=success([group, scene_node], 'last controller spawner'))))
    handlers.append(RegisterEventHandler(OnProcessExit(target_action=scene_node,
        on_exit=success([rviz], 'static planning scene'))))
    for process in processes:
        handlers.append(RegisterEventHandler(OnProcessExit(target_action=process,
            on_exit=[EmitEvent(event=Shutdown(reason='Required node exited'))])))
    if sim:
        world = run/'cell.world'
        world.write_text(world_xml(scene), encoding='utf-8')
        gazebo = IncludeLaunchDescription(PythonLaunchDescriptionSource(
            str(Path(get_package_share_directory('gazebo_ros'))/'launch/gazebo.launch.py')),
            launch_arguments={'world': str(world), 'gui': arg('gui'), 'pause': 'false'}.items())
        spawn = Node(package='gazebo_ros', executable='spawn_entity.py', prefix='/usr/bin/python3',
                     arguments=['-entity', 'fr3_dual_cell', '-topic', 'robot_description',
                                '-timeout', '120'], output='screen')
        handlers.append(RegisterEventHandler(OnProcessExit(target_action=spawn,
            on_exit=success([spawners[0]], 'Gazebo entity spawn'))))
        startup += [SetEnvironmentVariable('GAZEBO_MODEL_DATABASE_URI', ''), gazebo, spawn]
    else:
        startup.append(spawners[0])
    warning = ('REAL: drivers may send holding ServoJ / gripper commands even with '
               'enable_execution=false. This is not read-only. ROS shutdown is not an E-stop.'
               if mode == 'real' else f'{mode.upper()}: no physical hardware connection.')
    return handlers + [LogInfo(msg=warning), LogInfo(msg='Generated model/config: '+str(run))] + startup


def generate_launch_description():
    share = Path(get_package_share_directory('fr3_dual_bolt_cell'))
    return LaunchDescription([
        DeclareLaunchArgument('mode', default_value='gazebo', choices=['gazebo', 'mock', 'real']),
        DeclareLaunchArgument('enable_execution', default_value='false'),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('scene', default_value=str(share/'config/scene.yaml')),
        DeclareLaunchArgument('arms', default_value=str(share/'config/arms.yaml')),
        DeclareLaunchArgument('hardware', default_value=str(share/'config/hardware.example.yaml')),
        OpaqueFunction(function=start),
    ])
