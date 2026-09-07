"""ROS-independent validation; confirmations are commissioning gates, not safety interlocks."""
import ipaddress
import math
from pathlib import Path
import xml.etree.ElementTree as ET

import yaml

PLUGIN = 'fairino_hardware/FairinoHardwareInterface'
JOINTS = [f'j{i}' for i in range(1, 7)]


def load_yaml(path):
    with open(path, encoding='utf-8') as stream:
        result = yaml.safe_load(stream)
    if not isinstance(result, dict):
        raise ValueError(f'Expected a YAML mapping: {path}')
    return result


def number(value):
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
        raise ValueError(f'Expected finite numeric value, got {value!r}')
    return value


def validate_cell(cfg):
    t, b, g = cfg['table'], cfg['base'], cfg['tool']
    for field in ('length', 'width', 'height', 'thickness', 'leg_width'):
        if number(t[field]) <= 0:
            raise ValueError(f'table.{field} must be positive')
    if t['thickness'] >= t['height'] or t['leg_width'] >= min(t['length'], t['width']):
        raise ValueError('Invalid tabletop/leg dimensions')
    for field in ('x', 'y', 'yaw'):
        number(b[field])
    if abs(b['x']) + .08 >= t['length']/2 or abs(b['y']) + .08 >= t['width']/2:
        raise ValueError('FR3 base must be fully supported by the tabletop')
    if not 0 <= number(g['finger_travel']) <= .05:
        raise ValueError('tool.finger_travel must be in [0, 0.05] m')
    for field in ('wrist_to_flange_xyz', 'wrist_to_flange_rpy', 'flange_to_gripper_xyz',
                  'flange_to_gripper_rpy', 'gripper_to_tcp_xyz', 'gripper_to_tcp_rpy',
                  'collision_xyz', 'collision_size'):
        if not isinstance(g[field], list) or len(g[field]) != 3:
            raise ValueError(f'tool.{field} must have 3 numbers')
        for value in g[field]:
            number(value)
    if min(g['collision_size']) <= 0:
        raise ValueError('Collision envelope must have positive dimensions')
    # Require the configured envelope to retain the modeled full finger sweep.
    for center, size, low, high in zip(g['collision_xyz'], g['collision_size'],
                                     [-.09, -.04, -.005], [.09, .04, .148]):
        if center-size/2 > low+1e-9 or center+size/2 < high-1e-9:
            raise ValueError('Collision box must cover the full HKV modeled envelope')
    return cfg


def validate_real(cfg, confirmed):
    if not confirmed:
        raise ValueError('Real hardware may send ServoJ as soon as it starts. '
                         'Complete docs/COMMISSIONING.md, then pass confirm_real:=true.')
    for key in ('firmware_version', 'driver_package'):
        if not isinstance(cfg.get(key), str) or not cfg[key].strip():
            raise ValueError(f'Fill {key} in your real YAML first')
    for key in ('controller_ip', 'driver_configured_ip'):
        ipaddress.IPv4Address(cfg.get(key, ''))
    if cfg['controller_ip'] != cfg['driver_configured_ip']:
        raise ValueError('Robot IP differs from the address configured in the actual driver')
    for key in ('driver_reviewed', 'geometry_tcp_payload_verified',
                'workcell_estop_and_low_speed_verified'):
        if cfg.get('checks', {}).get(key) is not True:
            raise ValueError(f'Commissioning check not confirmed: {key}')
    rate = cfg.get('update_rate')
    if isinstance(rate, bool) or not isinstance(rate, int) or not 1 <= rate <= 1000:
        raise ValueError('update_rate must be an integer in [1, 1000], matched to your driver')
    return cfg


def exports_plugin(share):
    for path in Path(share).rglob('*.xml'):
        try:
            root = ET.parse(path).getroot()
        except ET.ParseError:
            continue
        if any(c.get('name') == PLUGIN for c in root.iter('class')):
            return True
    return False
