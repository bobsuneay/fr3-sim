from pathlib import Path
import xml.etree.ElementTree as ET

import xacro

ROOT = Path(__file__).resolve().parents[1]


def expanded(mode='mock'):
    return ET.fromstring(xacro.process_file(
        str(ROOT / 'urdf/robot.urdf.xacro'),
        mappings={
            'mode': mode,
            'arm_file': (ROOT / 'urdf/fr3_arm.urdf').as_posix(),
            'cell_file': (ROOT / 'config/cell.yaml').as_posix(),
            'initial_file': (ROOT / 'config/initial_positions.yaml').as_posix(),
        }).toxml())


def test_real_cell_is_not_the_gazebo_inspection_cell():
    text = (ROOT / 'urdf/robot.urdf.xacro').read_text(encoding='utf-8').lower()
    assert 'gazebo' not in text
    assert 'camera' not in text
    assert 'bolt' not in text


def test_mock_has_only_six_arm_control_joints():
    root = expanded()
    control = root.find('ros2_control')
    names = [joint.get('name') for joint in control.findall('joint')]
    assert names == [f'j{i}' for i in range(1, 7)]
    assert control.find('hardware/plugin').text == 'mock_components/GenericSystem'
    assert root.find("./joint[@name='table_to_base']/origin").get('rpy') == '0 0 0.0'
    assert root.find("./joint[@name='gripper_to_tcp']") is not None


def test_real_selects_only_fairino_plugin():
    root = expanded('real')
    assert root.find('ros2_control/hardware/plugin').text == \
        'fairino_hardware/FairinoHardwareInterface'
