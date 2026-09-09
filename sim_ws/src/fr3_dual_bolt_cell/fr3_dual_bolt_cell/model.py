"""Pure model/config assembly, shared by launch and offline tests.

Arm vendor joints are kept in j1..j6 order; only their names get a prefix.
One real ros2_control process per arm isolates the vendor SDK and HKV threads.
"""
from copy import deepcopy
from pathlib import Path
import ipaddress
import math
import xml.etree.ElementTree as ET

import xacro
import yaml

SIDES = ('left', 'right')
PACKAGE = 'fr3_dual_bolt_cell'


def read_yaml(path):
    return yaml.safe_load(Path(path).read_text(encoding='utf-8'))


def vector(value, count, label):
    if (not isinstance(value, list) or len(value) != count or
            any(type(v) not in (float, int) or not math.isfinite(v) for v in value)):
        raise ValueError(f'{label} must contain {count} finite numbers')


def validate_arms(cfg):
    for side in SIDES:
        for key, length in (('xyz', 3), ('rpy', 3), ('initial', 6)):
            vector(cfg[side][key], length, f'{side}.{key}')
    g = cfg['gripper']
    for key in ('flange_xyz', 'flange_rpy', 'tcp_xyz', 'tcp_rpy'):
        vector(g[key], 3, key)
    if not (0 < g['open_gap']/2 <= g['finger_travel'] <= 0.05):
        raise ValueError('Require 0 < open_gap/2 <= finger_travel <= 0.05 m')
    if cfg['left']['xyz'] == cfg['right']['xyz']:
        raise ValueError('Both bases cannot occupy the same position')
    return cfg


def validate_hardware(cfg):
    if cfg.get('commissioned') is not True:
        raise ValueError('Complete hardware.example.yaml commissioning before mode:=real')
    if cfg.get('driver_package') != 'fairino_hardware_v3_9_7' or cfg.get('firmware') != '3.9.7':
        raise ValueError('This adapter is verified against supplied 3.9.7 driver source only')
    for side in SIDES:
        address = ipaddress.IPv4Address(cfg[side]['robot_ip'])
        if address.is_unspecified or address.is_multicast or address.is_loopback:
            raise ValueError(f'Invalid {side} robot IP')
        port = cfg[side]['serial_port']
        if not port.startswith('/dev/') or 'REPLACE' in port:
            raise ValueError(f'Configure actual {side} serial_port')
    if cfg['left']['robot_ip'] == cfg['right']['robot_ip']:
        raise ValueError('Left and right IPs must differ on this host')
    if cfg['left']['serial_port'] == cfg['right']['serial_port']:
        raise ValueError('Each gripper needs its own serial adapter')
    g = cfg['gripper']
    for key, low, high in (('baud_rate', 1, 4000000), ('timeout', 1, 10000),
                           ('slave_address', 1, 247), ('position_mode_speed_register', 200, 1500),
                           ('target_force_percent', 1, 100)):
        if type(g[key]) is not int or not low <= g[key] <= high:
            raise ValueError(f'Invalid gripper.{key}')
    if not 0 <= g['position_closed_register'] < g['position_open_register'] <= 100:
        raise ValueError('Supplied HKV feedback is 0..100; verify register calibration')
    return cfg


def element(parent, tag, **attributes):
    return ET.SubElement(parent, tag, {k: str(v) for k, v in attributes.items()})


def numbers(values):
    return ' '.join(map(str, values))


def fixed(root, name, parent, child, xyz=(0, 0, 0), rpy=(0, 0, 0)):
    joint = element(root, 'joint', name=name, type='fixed')
    element(joint, 'parent', link=parent)
    element(joint, 'child', link=child)
    element(joint, 'origin', xyz=numbers(xyz), rpy=numbers(rpy))


def inertial(link, mass, size, xyz=(0, 0, 0)):
    block = element(link, 'inertial')
    element(block, 'origin', xyz=numbers(xyz))
    element(block, 'mass', value=mass)
    a, b, c = size
    element(block, 'inertia', ixx=mass*(b*b+c*c)/12, iyy=mass*(a*a+c*c)/12,
            izz=mass*(a*a+b*b)/12, ixy=0, ixz=0, iyz=0)


def box(link, size, xyz=(0, 0, 0), visual=False):
    block = element(link, 'visual' if visual else 'collision')
    element(block, 'origin', xyz=numbers(xyz))
    element(element(block, 'geometry'), 'box', size=numbers(size))
    if visual:
        material = element(block, 'material', name=link.get('name')+'_metal')
        element(material, 'color', rgba='0.35 0.38 0.40 1')


def mesh(link, name, xyz=(0, 0, 0), yaw=0):
    block = element(link, 'visual')
    element(block, 'origin', xyz=numbers(xyz), rpy=f'0 0 {yaw}')
    element(element(block, 'geometry'), 'mesh',
            filename=f'package://{PACKAGE}/meshes/hkv_tg9801/{name}.stl', scale='.001 .001 .001')
    material = element(block, 'material', name=link.get('name')+'_'+name+'_material')
    color = '0.1882 0.1882 0.1882 1' if name in ('base_body', 'finger') else '0.68 0.70 0.72 1'
    element(material, 'color', rgba=color)


def add_gripper(root, side, cfg):
    p = side + '_'
    g = cfg['gripper']
    element(root, 'link', name=p+'tool0')
    fixed(root, p+'wrist_to_tool', p+'wrist3_link', p+'tool0', g['flange_xyz'], g['flange_rpy'])
    palm = element(root, 'link', name=p+'gripper_palm')
    inertial(palm, 0.65, (.06, .16, .09), (0, 0, .04))
    mesh(palm, 'flange')
    mesh(palm, 'base_body', (0, 0, .0615))
    mesh(palm, 'rail_155', (0, 0, .067))
    # Match the original HKV collision primitives. Display material is handled
    # separately: missing URDF materials fall back to RVIZ/ShadedRed.
    box(palm, (.16, .0705, .0615), (0, .00325, .03075))
    box(palm, (.155, .007, .0048), (0, 0, .0694))
    fixed(root, p+'tool_to_gripper', p+'tool0', p+'gripper_palm')
    # Keep the same convention as fr3_bolt_cell: q=0 is closed and q=0.05 m
    # is open. Both prismatic joints are commanded independently by the
    # JointTrajectoryController; this is more reliable in Gazebo than relying
    # on a mimic joint in ros2_control.
    for index, sign in enumerate((-1, 1)):
        finger = 'left' if index == 0 else 'right'
        link_name = p+finger+'_finger'
        joint_name = p+finger+'_finger_joint'
        link = element(root, 'link', name=link_name)
        inertial(link, .08, (.008, .018, .07), (0, 0, .04))
        mesh(link, 'slider')
        mesh(link, 'finger', (0, 0, .008), 0 if index == 0 else math.pi)
        # Collision inner faces match the configured gap; visual CAD is illustrative.
        box(link, (.024, .017, .0065), (0, 0, .00485))
        box(link, (.0285, .040, .0717), (0, 0, .04385), visual=False)
        joint = element(root, 'joint', name=joint_name, type='prismatic')
        element(joint, 'parent', link=p+'gripper_palm')
        element(joint, 'child', link=link_name)
        element(joint, 'origin', xyz=f'{sign*0.01545} 0 .067')
        element(joint, 'axis', xyz=f'{sign} 0 0')
        element(joint, 'limit', lower=0, upper=0.05, effort=100, velocity=0.10)
        element(joint, 'dynamics', damping=2.0, friction=.10)
        surface = element(root, 'gazebo', reference=link_name)
        element(surface, 'selfCollide').text = 'true'
        for tag, value in (('mu1', 1), ('mu2', 1), ('kp', 100000), ('kd', 10)):
            element(surface, tag).text = str(value)
    element(root, 'link', name=p+'gripper_tcp')
    fixed(root, p+'palm_to_tcp', p+'gripper_palm', p+'gripper_tcp', g['tcp_xyz'], g['tcp_rpy'])


def control(root, name, plugin, joints, initial=None, parameters=None, mimic=None):
    system = element(root, 'ros2_control', name=name, type='system')
    hardware = element(system, 'hardware')
    element(hardware, 'plugin').text = plugin
    for key, value in (parameters or {}).items():
        element(hardware, 'param', name=key).text = str(value)
    for name in joints:
        joint = element(system, 'joint', name=name)
        element(joint, 'command_interface', name='position')
        state = element(joint, 'state_interface', name='position')
        if 'finger_joint' in name or 'gripper_joint' in name:
            element(joint, 'state_interface', name='velocity')
        if initial is not None:
            element(state, 'param', name='initial_value').text = str(initial.get(name, 0))
        if mimic and name in mimic:
            element(joint, 'param', name='mimic').text = mimic[name]
            element(joint, 'param', name='multiplier').text = '1'
    return system


def build_model(share, scene_file, arms, mode, controller_file='', hardware=None):
    if mode not in ('gazebo', 'mock', 'real'):
        raise ValueError('mode must be gazebo, mock or real')
    validate_arms(arms)
    if mode == 'real':
        validate_hardware(hardware)
    share = Path(share)
    common = xacro.process_file(str(share/'urdf/common.urdf.xacro'),
                               mappings={'scene_file': Path(scene_file).resolve().as_posix()})
    root = ET.fromstring(common.toxml())
    root.set('name', 'fr3_dual_bolt_cell')
    vendor = ET.parse(share/'urdf/fr3_arm.urdf').getroot()
    for side in SIDES:
        prefix = side+'_'
        plate = element(root, 'link', name=prefix+'mount_plate')
        inertial(plate, .5, (.15, .15, .02))
        box(plate, (.15, .15, .02), (0, 0, -.01))
        box(plate, (.15, .15, .02), (0, 0, -.01), visual=True)
        fixed(root, prefix+'plate_mount', 'support_link', prefix+'mount_plate',
              arms[side]['xyz'], arms[side]['rpy'])
        for original in vendor:
            node = deepcopy(original)
            for part in node.iter():
                for key in ('name', 'link', 'joint', 'reference'):
                    if key in part.attrib:
                        part.set(key, prefix+part.get(key))
            root.append(node)
        fixed(root, prefix+'base_mount', 'support_link', prefix+'base_link',
              arms[side]['xyz'], arms[side]['rpy'])
        add_gripper(root, side, arms)
        if mode == 'gazebo':
            for link in vendor.findall('link'):
                surface = element(root, 'gazebo', reference=prefix+link.get('name'))
                element(surface, 'selfCollide').text = 'true'
                element(surface, 'material').text = 'Gazebo/White'
    if mode != 'gazebo':
        for node in list(root.findall('gazebo')):
            root.remove(node)
    initial = {f'{side}_j{i+1}': v for side in SIDES for i, v in enumerate(arms[side]['initial'])}
    for joint in root.findall('joint'):
        if joint.get('name') in initial:
            limit = joint.find('limit')
            if not float(limit.get('lower')) <= initial[joint.get('name')] <= float(limit.get('upper')):
                raise ValueError(f'Initial position outside URDF limit: {joint.get("name")}')
    for side in SIDES:
        arm_joints = [f'{side}_j{i}' for i in range(1, 7)]
        grip = side+'_gripper_joint'
        follower = side+'_finger_mimic_joint'
        finger_joints = [f'{side}_left_finger_joint', f'{side}_right_finger_joint']
        if mode == 'gazebo':
            control(root, side+'_system', 'gazebo_ros2_control/GazeboSystem',
                    arm_joints+finger_joints, initial)
        else:
            control(root, side+'_arm_system', 'mock_components/GenericSystem' if mode == 'mock'
                    else 'fairino_hardware/FairinoHardwareInterface', arm_joints,
                    initial if mode == 'mock' else None,
                    None if mode == 'mock' else {'robot_ip': hardware[side]['robot_ip']})
            params = {} if mode == 'mock' else {
                **hardware['gripper'], 'serial_port': hardware[side]['serial_port'],
                'gripper_closed_position': arms['gripper']['finger_travel']}
            real_gripper_joints = [grip] if mode == 'real' else finger_joints
            control(root, side+'_gripper_system', 'mock_components/GenericSystem' if mode == 'mock'
                    else 'ros2_hkv_gripper/GripperHardwareInterface', real_gripper_joints,
                    {} if mode == 'mock' else None, params)
    if mode == 'gazebo':
        plugin = element(element(root, 'gazebo'), 'plugin', name='gazebo_ros2_control',
                         filename='libgazebo_ros2_control.so')
        element(plugin, 'robot_param').text = 'robot_description'
        element(plugin, 'robot_param_node').text = 'robot_state_publisher'
        element(plugin, 'parameters').text = str(controller_file)
    return root


def manager_model(root, side):
    result = deepcopy(root)
    for node in list(result.findall('ros2_control')):
        if not node.get('name').startswith(side+'_'):
            result.remove(node)
    return ET.tostring(result, encoding='unicode')


def controllers(mode, side=None):
    manager = 'controller_manager' if mode == 'gazebo' else side+'_controller_manager'
    params = {'update_rate': 100 if mode == 'gazebo' else 125, 'use_sim_time': mode == 'gazebo'}
    result = {manager: {'ros__parameters': params}}
    for arm in SIDES if side is None else (side,):
        names = (arm+'_joint_state_broadcaster', arm+'_arm_controller', arm+'_gripper_controller')
        for name, kind in zip(names, ('joint_state_broadcaster/JointStateBroadcaster',
                                      'joint_trajectory_controller/JointTrajectoryController',
                                      'joint_trajectory_controller/JointTrajectoryController')):
            params[name] = {'type': kind}
        result[names[0]] = {'ros__parameters': {
            'joints': [f'{arm}_j{i}' for i in range(1, 7)] +
                      [arm+'_left_finger_joint', arm+'_right_finger_joint'],
            'interfaces': ['position'], 'use_local_topics': False}}
        result[names[1]] = {'ros__parameters': {
            'joints': [f'{arm}_j{i}' for i in range(1, 7)],
            'command_interfaces': ['position'], 'state_interfaces': ['position'],
            'allow_partial_joints_goal': False, 'state_publish_rate': 50.0,
            'constraints': {'goal_time': 2.0, 'stopped_velocity_tolerance': 0.05}}}
        result[names[2]] = {'ros__parameters': {
            'joints': [arm+'_left_finger_joint', arm+'_right_finger_joint'],
            'command_interfaces': ['position'], 'state_interfaces': ['position', 'velocity'],
            'state_publish_rate': 30.0, 'action_monitor_rate': 20.0,
            'allow_partial_joints_goal': False}}
    return result


def semantic(root, arms):
    srdf = ET.Element('robot', name=root.get('name'))
    for side in SIDES:
        group = element(srdf, 'group', name=side+'_arm')
        element(group, 'chain', base_link=side+'_base_link', tip_link=side+'_gripper_tcp')
        group = element(srdf, 'group', name=side+'_gripper')
        element(group, 'joint', name=side+'_left_finger_joint')
        element(group, 'joint', name=side+'_right_finger_joint')
        element(srdf, 'end_effector', name=side+'_hkv', parent_link=side+'_gripper_palm',
                group=side+'_gripper', parent_group=side+'_arm')
        ready = element(srdf, 'group_state', name='ready', group=side+'_arm')
        for i, v in enumerate(arms[side]['initial'], 1):
            element(ready, 'joint', name=f'{side}_j{i}', value=v)
        for name, q in (('closed', .00025), ('open', arms['gripper']['open_gap']/2)):
            state = element(srdf, 'group_state', name=name, group=side+'_gripper')
            element(state, 'joint', name=side+'_left_finger_joint', value=q)
            element(state, 'joint', name=side+'_right_finger_joint', value=q)
        element(srdf, 'disable_collisions', link1=side+'_wrist3_link',
                link2=side+'_gripper_palm', reason='Mounting')
        element(srdf, 'disable_collisions', link1=side+'_mount_plate',
                link2=side+'_base_link', reason='Mounting')
    both = element(srdf, 'group', name='both_arms')
    for side in SIDES:
        element(both, 'group', name=side+'_arm')
    for joint in root.findall('joint'):
        element(srdf, 'disable_collisions', link1=joint.find('parent').get('link'),
                link2=joint.find('child').get('link'), reason='Adjacent')
    element(srdf, 'disable_collisions', link1='head_bracket', link2='head_camera_link', reason='Mounting')
    return ET.tostring(srdf, encoding='unicode')


def moveit_config(root, arms, share):
    ompl = read_yaml(Path(share)/'config/ompl.yaml')
    ompl.pop('fairino3_v6_group', None)
    ompl.pop('gripper', None)
    mapping = {'controller_names': []}
    kinematics = {}
    for group in ('left_arm', 'right_arm', 'both_arms', 'left_gripper', 'right_gripper'):
        ompl[group] = {'planner_configs': ['RRTConnectkConfigDefault'],
                       'longest_valid_segment_fraction': .005}
    for side in SIDES:
        kinematics[side+'_arm'] = {'kinematics_solver': 'kdl_kinematics_plugin/KDLKinematicsPlugin',
                                  'kinematics_solver_timeout': .1,
                                  'kinematics_solver_search_resolution': .005}
        for suffix, kind, action, joints in (
                ('arm_controller', 'FollowJointTrajectory', 'follow_joint_trajectory',
                 [f'{side}_j{i}' for i in range(1, 7)]),
                ('gripper_controller', 'FollowJointTrajectory', 'follow_joint_trajectory',
                 [side+'_left_finger_joint', side+'_right_finger_joint'])):
            name = side+'_'+suffix
            mapping['controller_names'].append(name)
            mapping[name] = {'type': kind, 'action_ns': action, 'default': True, 'joints': joints}
    limits = {}
    for joint in root.findall('joint'):
        if joint.get('type') in ('revolute', 'prismatic') and joint.find('mimic') is None:
            velocity = min(float(joint.find('limit').get('velocity')), .3)
            limits[joint.get('name')] = {'has_velocity_limits': True, 'max_velocity': velocity,
                                       'has_acceleration_limits': True, 'max_acceleration': .3}
    return {'robot_description_semantic': semantic(root, arms),
            'robot_description_kinematics': kinematics,
            'robot_description_planning': {'joint_limits': limits,
                'default_velocity_scaling_factor': .1, 'default_acceleration_scaling_factor': .1},
            'planning_pipelines': ['ompl'], 'default_planning_pipeline': 'ompl', 'ompl': ompl,
            'moveit_controller_manager': 'moveit_simple_controller_manager/MoveItSimpleControllerManager',
            'moveit_simple_controller_manager': mapping}
