"""A cold-start backend switch: stop the previous launch before changing mode."""
from pathlib import Path
import tempfile
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory
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
    moveit_config, read_yaml)
from fr3_dual_bolt_cell.world import load_scene, world_xml
from fr3_bolt_inspection_cell.model import augment, inspection_world
from fr3_bolt_inspection_cell.model import linked_controllers as controllers
from fr3_bolt_inspection_cell.core import validate


def start(context):
    share = Path(get_package_share_directory('fr3_dual_bolt_cell'))
    arg = lambda name: LaunchConfiguration(name).perform(context)
    mode = arg('mode')
    if mode not in ('gazebo', 'mock'):
        raise ValueError('Inspection supports gazebo/mock only; real grasp/feedback needs commissioning')
    for name in ('enable_execution', 'rviz', 'gui', 'panel'):
        if arg(name) not in ('true', 'false'):
            raise ValueError(f'{name} must be true or false')
    execute = arg('enable_execution') == 'true'
    sim = mode == 'gazebo'
    scene_path = Path(arg('scene')).expanduser().resolve()
    scene = load_scene(scene_path)
    arms = read_yaml(Path(arg('arms')).expanduser())
    cfg_path = Path(arg('inspection')).expanduser().resolve()
    cfg = validate(read_yaml(cfg_path))
    if scene['bolts']['rows'] != 1 or scene['bolts']['cols'] != 1:
        raise ValueError('First inspection demo requires one isolated bolt')
    if abs(scene['table']['top_z']-cfg['table_z']) > 1e-6:
        raise ValueError('Perception table_z must match scene table')
    for key in ('length', 'shaft_radius', 'head_radius', 'head_length'):
        expected = cfg['bolt_length' if key == 'length' else key]
        if abs(scene['bolts'][key]-expected) > 1e-9:
            raise ValueError('Scene bolt dimensions differ from inspection calibration')
    hardware = None
    temp = tempfile.TemporaryDirectory(prefix='fr3_bolt_inspection_cell_')
    run = Path(temp.name)
    combined = run/'gazebo_controllers.yaml'
    combined.write_text(yaml.safe_dump(controllers('gazebo')), encoding='utf-8')
    root = augment(build_model(share, scene_path, arms, mode, combined, hardware), cfg, sim)
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
    semantic = ET.fromstring(moveit['robot_description_semantic'])
    for side in SIDES:
        master = side+'_left_finger_joint'
        for parent in semantic.iter():
            for joint in list(parent.findall('joint')):
                if joint.get('name') == side+'_right_finger_joint':
                    parent.remove(joint)
                elif joint.get('name') == side+'_gripper_joint':
                    joint.set('name', master)
        mapping = moveit['moveit_simple_controller_manager'][side+'_gripper_controller']
        mapping.update(type='FollowJointTrajectory', action_ns='follow_joint_trajectory', joints=[master])
    moveit['robot_description_semantic'] = ET.tostring(semantic, encoding='unicode')
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

    task = Node(package='fr3_bolt_inspection_cell', executable='inspection_task',
                parameters=[{'config_file': str(cfg_path), 'arms_file': arg('arms'),
                             'mode': mode, 'enable_execution': execute, 'use_sim_time': sim}],
                output='screen')
    panel = Node(package='fr3_bolt_inspection_cell', executable='inspection_panel',
                 parameters=[{'use_sim_time': sim}], output='screen',
                 condition=IfCondition(LaunchConfiguration('panel')))

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
        on_exit=success([rviz, task, panel], 'static planning scene'))))
    for process in processes:
        handlers.append(RegisterEventHandler(OnProcessExit(target_action=process,
            on_exit=[EmitEvent(event=Shutdown(reason='Required node exited'))])))
    if sim:
        world = run/'cell.world'
        world.write_text(inspection_world(world_xml(scene), cfg), encoding='utf-8')
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
    warning = 'Inspection: assisted Gazebo grasp; start explicitly via /inspection/start.'
    return handlers + [LogInfo(msg=warning), LogInfo(msg='Generated model/config: '+str(run))] + startup


def generate_launch_description():
    share = Path(get_package_share_directory('fr3_bolt_inspection_cell'))
    return LaunchDescription([
        DeclareLaunchArgument('mode', default_value='gazebo', choices=['gazebo', 'mock']),
        DeclareLaunchArgument('enable_execution', default_value='false'),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('panel', default_value='true'),
        DeclareLaunchArgument('scene', default_value=str(share/'config/scene.yaml')),
        DeclareLaunchArgument('arms', default_value=str(share/'config/arms.yaml')),
        DeclareLaunchArgument('inspection', default_value=str(share/'config/inspection.yaml')),
        OpaqueFunction(function=start),
    ])
