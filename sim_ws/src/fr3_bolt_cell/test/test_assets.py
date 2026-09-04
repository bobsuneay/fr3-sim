"""Offline tests; no ROS imports. Run from the package directory with pytest."""
import copy
import ast
import hashlib
import math
import shutil
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np
import pytest
import xacro
import yaml

from fr3_bolt_cell.world import load_scene, bolt_poses, bolt_inertia, table_boxes, world_xml

ROOT = Path(__file__).resolve().parents[1]


def transform(xyz, rpy):
    a, b, c = rpy
    ca, cb, cc = np.cos(rpy)
    sa, sb, sc = np.sin(rpy)
    result = np.eye(4)
    result[:3, 3] = xyz
    result[:3, :3] = [
        [cc*cb, cc*sb*sa-sc*ca, cc*sb*ca+sc*sa],
        [sc*cb, sc*sb*sa+cc*ca, sc*sb*ca-cc*sa],
        [-sb, cb*sa, cb*ca]]
    return result


def expanded():
    mappings = {k: (ROOT/v).as_posix() for k, v in {
        'arm_file': 'urdf/fr3_arm.urdf', 'scene_file': 'config/scene.yaml',
        'initial_file': 'config/initial_positions.yaml',
        'controllers_file': 'config/controllers.yaml'}.items()}
    return ET.fromstring(xacro.process_file(str(ROOT/'urdf/cell.urdf.xacro'),
                                           mappings=mappings).toxml())


def forward_kinematics(root):
    q = yaml.safe_load((ROOT/'config/initial_positions.yaml').read_text())['initial_positions']
    poses, pending = {'world': np.eye(4)}, list(root.findall('joint'))
    while pending:
        before = len(pending)
        for joint in pending[:]:
            parent = joint.find('parent').get('link')
            if parent not in poses:
                continue
            origin = joint.find('origin')
            values = lambda key: [float(v) for v in (
                origin.get(key, '0 0 0') if origin is not None else '0 0 0').split()]
            value = transform(values('xyz'), values('rpy'))
            position = q.get(joint.get('name'), 0.0)
            if joint.get('type') == 'revolute':
                # All six joints in the supplied FR3 have local axis +Z.
                assert joint.find('axis').get('xyz') == '0 0 1'
                value = value @ transform([0, 0, 0], [0, 0, position])
            elif joint.get('type') == 'prismatic':
                axis = np.array([float(v) for v in joint.find('axis').get('xyz').split()])
                value = value @ transform(axis*position, [0, 0, 0])
            poses[joint.find('child').get('link')] = poses[parent] @ value
            pending.remove(joint)
        assert len(pending) < before, 'URDF tree contains a cycle or disconnected link'
    return poses


def mesh_vertices(path):
    data = path.read_bytes()
    dtype = np.dtype([('normal', '<f4', 3), ('vertices', '<f4', (3, 3)), ('attr', '<u2')])
    return np.frombuffer(data, dtype=dtype, offset=84)['vertices'].reshape(-1, 3)


def link_vertices(link):
    geom = link.find('./collision/geometry')
    if geom is None:
        return None
    mesh = geom.find('mesh')
    if mesh is not None:
        path = ROOT / mesh.get('filename').split('package://fr3_bolt_cell/')[1]
        assert path.exists()
        return mesh_vertices(path)
    box = geom.find('box')
    size = np.array([float(v) for v in box.get('size').split()])
    origin = link.find('./collision/origin')
    offset = np.array([float(v) for v in origin.get('xyz', '0 0 0').split()])
    return np.array([[x, y, z] for x in (-.5, .5) for y in (-.5, .5)
                     for z in (-.5, .5)])*size + offset


def test_xacro_and_tree():
    root = expanded()
    assert not any('xacro' in e.tag for e in root.iter())
    links = [e.get('name') for e in root.findall('link')]
    assert len(set(links)) == len(links)
    assert set(forward_kinematics(root)) == set(links)
    assert len(root.findall("./gazebo/plugin[@filename='libgazebo_ros2_control.so']")) == 1
    assert len(root.findall('ros2_control')) == 1
    assert root.find("./joint[@name='side_mount_to_fr3']/parent").get('link') == 'support_link'
    assert root.find("./joint[@name='support_to_head_camera']/parent").get('link') == 'support_link'


def test_inertias_positive():
    for link in expanded().findall('link'):
        inertial = link.find('inertial')
        if link.find('collision') is None:
            continue
        assert inertial is not None
        assert float(inertial.find('mass').get('value')) > 0
        values = {k: float(v) for k, v in inertial.find('inertia').attrib.items()}
        matrix = [[values['ixx'], values['ixy'], values['ixz']],
                  [values['ixy'], values['iyy'], values['iyz']],
                  [values['ixz'], values['iyz'], values['izz']]]
        eig = np.linalg.eigvalsh(matrix)
        assert np.all(eig > 0)
        assert eig[-1] <= eig[0] + eig[1] + 1e-9


def test_initial_pose_above_table_and_ground():
    root = expanded()
    poses = forward_kinematics(root)
    scene = load_scene(ROOT/'config/scene.yaml')
    for link in root.findall('link'):
        vertices = link_vertices(link)
        if vertices is None:
            continue
        pose = poses[link.get('name')]
        points = vertices @ pose[:3, :3].T + pose[:3, 3]
        lo, hi = points.min(axis=0), points.max(axis=0)
        assert lo[2] >= -1e-6, link.get('name')+' below ground'
        for name, size, center in table_boxes(scene):
            # Conservative AABB broad-phase; passing means no overlap with table.
            low, high = np.array(center)-np.array(size)/2, np.array(center)+np.array(size)/2
            assert not (np.all(hi > low) and np.all(lo < high)), (
                link.get('name'), name, lo, hi)
    print('Initial TCP world XYZ:', poses['gripper_tcp'][:3, 3])


def test_world_bolts_and_table():
    scene = load_scene(ROOT/'config/scene.yaml')
    root = ET.fromstring(world_xml(scene))
    bolts = [m for m in root.findall('./world/model') if m.get('name').startswith('bolt_')]
    assert len(bolts) == 20
    assert len(root.findall('.//include')) == 0
    mass, com, transverse, axial = bolt_inertia(scene['bolts'])
    assert 0.001 < mass < 0.02
    assert transverse > axial > 0
    for model in bolts:
        assert model.findtext('static') == 'false'
        assert len(model.findall('joint')) == 0
        assert len(model.findall('.//collision')) == 2
        assert float(model.findtext('.//inertial/mass')) == pytest.approx(mass)
        lengths = [float(x.text) for x in model.findall('.//collision/geometry/cylinder/length')]
        assert sum(lengths) == pytest.approx(0.025)


def test_camera_sees_array_geometrically():
    # Pinhole frustum only: this does NOT claim freedom from robot occlusion.
    scene = load_scene(ROOT/'config/scene.yaml')
    c = scene['camera']
    optical = transform(c['xyz'], c['rpy']) @ transform([0, 0, 0], [-math.pi/2, 0, -math.pi/2])
    inverse = np.linalg.inv(optical)
    fx = c['width']/(2*math.tan(c['horizontal_fov']/2))
    pixels = []
    for _, xyz in bolt_poses(scene):
        x, y, z, _ = inverse @ np.array(xyz+[1])
        assert c['near'] < z < c['far']
        u, v = fx*x/z+c['width']/2, fx*y/z+c['height']/2
        assert 15 < u < c['width']-15
        assert 15 < v < c['height']-15
        pixels.append([u, v])
    print('Bolt-center image bounds:', np.min(pixels, axis=0), np.max(pixels, axis=0))


def test_controller_joint_consistency():
    root = expanded()
    limits = {j.get('name'): j.find('limit') for j in root.findall('joint') if j.find('limit') is not None}
    controlled = {j.get('name') for j in root.findall('./ros2_control/joint')}
    initial = yaml.safe_load((ROOT/'config/initial_positions.yaml').read_text())['initial_positions']
    controllers = yaml.safe_load((ROOT/'config/controllers.yaml').read_text())
    commanded = set()
    for name in ('fairino3_controller', 'gripper_controller'):
        commanded.update(controllers[name]['ros__parameters']['joints'])
    assert controlled == commanded == set(initial)
    for name, value in initial.items():
        assert float(limits[name].get('lower')) <= value <= float(limits[name].get('upper'))


def test_moveit_names_and_states():
    robot = expanded()
    srdf = ET.parse(ROOT/'config/cell.srdf').getroot()
    assert srdf.get('name') == robot.get('name')
    links = {e.get('name') for e in robot.findall('link')}
    limits = {e.get('name'): e.find('limit') for e in robot.findall('joint')}
    for pair in srdf.findall('disable_collisions'):
        assert pair.get('link1') in links and pair.get('link2') in links
    for chain in srdf.findall('.//chain'):
        assert chain.get('base_link') in links and chain.get('tip_link') in links
    for joint in srdf.findall('./group_state/joint'):
        limit = limits[joint.get('name')]
        assert float(limit.get('lower')) <= float(joint.get('value')) <= float(limit.get('upper'))
    cm = yaml.safe_load((ROOT/'config/controllers.yaml').read_text())
    mm = yaml.safe_load((ROOT/'config/moveit_controllers.yaml').read_text())['moveit_simple_controller_manager']
    for name in mm['controller_names']:
        assert mm[name]['joints'] == cm[name]['ros__parameters']['joints']
        assert mm[name]['type'] == 'FollowJointTrajectory'
    initial = yaml.safe_load((ROOT/'config/initial_positions.yaml').read_text())['initial_positions']
    for joint in srdf.findall("./group_state[@name='ready']/joint"):
        assert initial[joint.get('name')] == pytest.approx(float(joint.get('value')))


def test_asset_hashes():
    hashes = {
        'base_link': 'A4C94C0FCC939C6EA20BBC6E5548DF78194FC6F7BA9C405FB8885B64F4587929',
        'forearm_link': '384388E6F6BDEE3749A1B732BEF1143AB35D1DC1B5BAB551829034E7438D3EE8',
        'shoulder_link': '7DD0C503F694CE38F9E5950B925B081F8DBFECD828147788EF3A5586699573B1',
        'upperarm_link': 'CEA67D53323AEA9881B81F1EBA55BAAF12799C26C07E902DA2CBEC084CF6CB5B',
        'wrist1_link': '5BB16EE905EAD03D7C350C2BAEC64CB99744B7D7DC430A1D3B103201F189916D',
        'wrist2_link': '4257EB654ACF51BFE52F2477C512C40CCCFD855FD3ED551C3826268E050BFB6D',
        'wrist3_link': 'B4C06DD49A457BFFD4A6B73DD3DFA1803A61CDA8462A3B287CC7F9C9978F13B1',
    }
    for name, expected in hashes.items():
        assert hashlib.sha256((ROOT/'meshes/fairino3_v6'/f'{name}.STL').read_bytes()).hexdigest().upper() == expected


@pytest.mark.parametrize('layout', ['isolated', 'merged'])
def test_gazebo_model_uris_resolve_in_install_tree(tmp_path, layout):
    # Reproduce the missing model:// lookup with both colcon install layouts.
    # RViz package:// lookup passing alone does not exercise Gazebo's search root.
    prefix = tmp_path/'install'
    if layout == 'isolated':
        prefix = prefix/'fr3_bolt_cell'
    share = prefix/'share/fr3_bolt_cell'
    share.mkdir(parents=True)
    shutil.copy2(ROOT/'package.xml', share/'package.xml')
    shutil.copytree(ROOT/'meshes', share/'meshes')
    manifest = ET.parse(share/'package.xml')
    roots = [Path(export.get('gazebo_model_path').replace('${prefix}', str(share)))
             for export in manifest.findall('./export/gazebo_ros')
             if export.get('gazebo_model_path')]
    urdf = ET.parse(ROOT/'urdf/fr3_arm.urdf')
    for mesh in urdf.findall('.//mesh'):
        uri = mesh.get('filename').replace('package://', 'model://', 1)
        relative = uri.removeprefix('model://')
        assert any((root/relative).is_file() for root in roots), uri


def test_python_yaml_and_package_syntax():
    for path in ROOT.rglob('*.py'):
        ast.parse(path.read_text(encoding='utf-8'), filename=str(path))
    for path in (ROOT/'config').glob('*.yaml'):
        assert isinstance(yaml.safe_load(path.read_text()), dict)
    assert ET.parse(ROOT/'package.xml').getroot().findtext('name') == 'fr3_bolt_cell'
    rviz = yaml.safe_load((ROOT/'rviz/cell.rviz').read_text())
    assert rviz['Visualization Manager']['Global Options']['Fixed Frame'] == 'world'


@pytest.mark.parametrize('change', [
    {'length': 0.03}, {'length': -0.001}, {'rows': 0},
    {'cols': 201}, {'first_xy': [4, 0]}, {'spacing_xy': [0.001, 0.001]},
    {'density': float('nan')}, {'length': float('inf')}, {'first_xy': [0.2]},
])
def test_reject_invalid_scene(change, tmp_path):
    scene = copy.deepcopy(load_scene(ROOT/'config/scene.yaml'))
    scene['bolts'].update(change)
    path = tmp_path/'invalid.yaml'
    path.write_text(yaml.safe_dump(scene))
    with pytest.raises(ValueError):
        load_scene(path)
