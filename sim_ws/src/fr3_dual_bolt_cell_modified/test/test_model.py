from copy import deepcopy
import importlib.util
import math
from pathlib import Path
import sys
import xml.etree.ElementTree as ET

import pytest

SHARE = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(SHARE))
from fr3_dual_bolt_cell.model import (build_model, controllers, manager_model, moveit_config,
                                    read_yaml, semantic, validate_arms, validate_hardware)
from fr3_dual_bolt_cell.gripper import target_for_width
from fr3_dual_bolt_cell.world import load_scene, world_xml


@pytest.fixture
def arms():
    return read_yaml(SHARE/'config/arms.yaml')


@pytest.fixture
def hardware():
    cfg = read_yaml(SHARE/'config/hardware.example.yaml')
    cfg['commissioned'] = True
    cfg['left']['robot_ip'], cfg['right']['robot_ip'] = '192.168.58.2', '192.168.58.3'
    cfg['left']['serial_port'], cfg['right']['serial_port'] = '/dev/ttyACM0', '/dev/ttyACM1'
    return cfg


def model(arms, mode='mock', hardware=None):
    return build_model(SHARE, SHARE/'config/scene.yaml', arms, mode, '/tmp/controllers.yaml', hardware)


@pytest.mark.parametrize('mode', ['gazebo', 'mock', 'real'])
def test_complete_tree_and_assets(arms, hardware, mode):
    root = model(arms, mode, hardware)
    links = [link.get('name') for link in root.findall('link')]
    joints = root.findall('joint')
    assert len(links) == len(set(links)) == 32
    assert len(joints) == len(links)-1
    assert len({j.get('name') for j in joints}) == len(joints)
    parents = {j.find('child').get('link'): j.find('parent').get('link') for j in joints}
    assert set(links)-parents.keys() == {'world'}
    for link in links:
        visited = set()
        while link in parents:
            assert link not in visited
            visited.add(link)
            link = parents[link]
        assert link == 'world'
    for mesh in root.iter('mesh'):
        assert (SHARE/mesh.get('filename').removeprefix('package://fr3_dual_bolt_cell_modified/')).is_file()


@pytest.mark.parametrize('mode', ['gazebo', 'mock', 'real'])
def test_backend_isolation_and_interfaces(arms, hardware, mode):
    root = model(arms, mode, hardware)
    plugins = [n.find('hardware/plugin').text for n in root.findall('ros2_control')]
    if mode == 'gazebo':
        assert set(plugins) == {'gazebo_ros2_control/GazeboSystem'}
        assert len(root.findall("gazebo/plugin[@filename='libgazebo_ros2_control.so']")) == 1
    elif mode == 'mock':
        assert set(plugins) == {'mock_components/GenericSystem'}
        assert not root.findall('gazebo')
    else:
        assert plugins.count('fairino_hardware/FairinoHardwareInterface') == 2
        assert not root.findall('gazebo')
        for side in ('left', 'right'):
            component = root.find(f"ros2_control[@name='{side}_arm_system']")
            assert [j.get('name') for j in component.findall('joint')] == [f'{side}_j{i}' for i in range(1, 7)]
            assert component.find("hardware/param[@name='robot_ip']").text == hardware[side]['robot_ip']
            assert all(len(j.findall('state_interface')) == 1 for j in component.findall('joint'))
            assert not component.findall('.//param[@name="initial_value"]')
        for side in ('left', 'right'):
            command = root.find(f".//ros2_control/joint[@name='{side}_gripper_left_finger_joint']")
            assert {s.get('name') for s in command.findall('state_interface')} == {'position', 'velocity'}


def test_ros2_control_boundary_is_standalone_xacro():
    fragment = (SHARE/'urdf/my_robot.ros2_control.xacro').read_text(encoding='utf-8')
    assert 'mock_components/GenericSystem' in fragment
    assert '<plugin>$(arg arm_plugin)</plugin>' in fragment
    assert '<plugin>$(arg gripper_plugin)</plugin>' in fragment
    assert 'fairino_hardware/FairinoHardwareInterface' not in fragment


def test_real_manager_separation(arms, hardware):
    root = model(arms, 'real', hardware)
    for side, other in (('left', 'right'), ('right', 'left')):
        xml = ET.fromstring(manager_model(root, side))
        assert len(xml.findall('ros2_control')) == 2
        assert all(n.get('name').startswith(side+'_') for n in xml.findall('ros2_control'))
        cfg = controllers('real', side)
        assert side+'_controller_manager' in cfg
        assert not any(k.startswith(other+'_') for k in cfg)


def test_mimic_joint_is_published_only_by_simulation_state_broadcaster():
    gazebo = controllers('gazebo')
    for side in ('left', 'right'):
        gazebo_joints = gazebo[f'{side}_joint_state_broadcaster']['ros__parameters']['joints']
        real_joints = controllers('real', side)[f'{side}_joint_state_broadcaster']['ros__parameters']['joints']
        assert f'{side}_gripper_right_finger_joint' in gazebo_joints
        assert f'{side}_gripper_right_finger_joint' not in real_joints


def test_moveit_has_both_arms_and_preserves_interarm_collisions(arms):
    root = model(arms)
    config = moveit_config(root, arms, SHARE)
    srdf = ET.fromstring(semantic(root, arms))
    groups = {g.get('name') for g in srdf.findall('group')}
    assert groups == {'left_arm', 'right_arm', 'both_arms', 'left_gripper', 'right_gripper'}
    for exclusion in srdf.findall('disable_collisions'):
        pair = (exclusion.get('link1'), exclusion.get('link2'))
        assert not (pair[0].startswith('left_') and pair[1].startswith('right_'))
        assert not (pair[0].startswith('right_') and pair[1].startswith('left_'))
    mapping = config['moveit_simple_controller_manager']
    for name in mapping['controller_names']:
        assert name in controllers('gazebo')
        assert name in controllers('real', name.split('_')[0])
    assert len(config['robot_description_planning']['joint_limits']) == 16


def test_gripper_width_mapping_and_physical_gap(arms):
    g = arms['gripper']
    assert target_for_width(g['open_gap'], arms) == pytest.approx(g['open_gap'])
    assert target_for_width(0, arms) == 0
    assert target_for_width(.01, arms) == pytest.approx(.01)
    root = model(arms)
    for side in ('left', 'right'):
        left = root.find(f"joint[@name='{side}_gripper_left_finger_joint']")
        right = root.find(f"joint[@name='{side}_gripper_right_finger_joint']")
        assert left.find('origin').get('xyz').split()[0] == '-0.01545'
        assert right.find('origin').get('xyz').split()[0] == '0.01545'
        assert left.find('axis').get('xyz') == '-1 0 0'
        assert right.find('axis').get('xyz') == '-1 0 0'
        assert right.find('mimic').get('joint') == f'{side}_gripper_left_finger_joint'
        assert right.find('mimic').get('multiplier') == '-1'


@pytest.mark.parametrize('width', [-.001, .061, math.nan, math.inf])
def test_bad_width_is_rejected(arms, width):
    with pytest.raises(ValueError):
        target_for_width(width, arms)


@pytest.mark.parametrize('case', ['uncommissioned', 'same_ip', 'bad_ip', 'same_serial', 'firmware', 'speed', 'register'])
def test_bad_real_config_rejected(hardware, case):
    if case == 'uncommissioned': hardware['commissioned'] = False
    elif case == 'same_ip': hardware['right']['robot_ip'] = hardware['left']['robot_ip']
    elif case == 'bad_ip': hardware['left']['robot_ip'] = 'garbage'
    elif case == 'same_serial': hardware['right']['serial_port'] = hardware['left']['serial_port']
    elif case == 'firmware': hardware['firmware'] = '3.9.6'
    elif case == 'speed': hardware['gripper']['position_mode_speed_register'] = 0
    elif case == 'register': hardware['gripper']['position_open_register'] = 1000
    with pytest.raises(ValueError): validate_hardware(hardware)


@pytest.mark.parametrize('case', ['nonfinite', 'same_base', 'overtravel', 'limit'])
def test_bad_geometry_rejected(arms, case):
    if case == 'nonfinite': arms['left']['xyz'][0] = math.nan
    elif case == 'same_base': arms['left']['xyz'] = arms['right']['xyz'][:]
    elif case == 'overtravel': arms['gripper']['finger_travel'] = .1
    elif case == 'limit': arms['left']['initial'][0] = 100
    with pytest.raises(ValueError): model(arms)


def test_single_table_and_twenty_dynamic_bolts():
    root = ET.fromstring(world_xml(load_scene(SHARE/'config/scene.yaml')))
    bolts = [m for m in root.findall('world/model') if m.get('name').startswith('bolt_')]
    assert len(bolts) == 20
    assert all(m.find('static') is None or m.find('static').text == 'false' for m in bolts)


def test_unknown_mode_rejected(arms):
    with pytest.raises(ValueError): model(arms, 'gazbeo')


def test_all_installed_python_sources_parse():
    import ast
    for folder in ('fr3_dual_bolt_cell', 'launch', 'tools'):
        for source in (SHARE/folder).rglob('*.py'):
            ast.parse(source.read_text(encoding='utf-8'), filename=str(source))
