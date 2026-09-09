"""Conservative initial-pose AABB checks; not a full trajectory collision test."""
import itertools
from pathlib import Path
import sys

import numpy as np

SHARE = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(SHARE))
from fr3_dual_bolt_cell.model import build_model, read_yaml
from fr3_dual_bolt_cell.world import load_scene, table_boxes


def transform(xyz, rpy):
    ca, cb, cc = np.cos(rpy)
    sa, sb, sc = np.sin(rpy)
    result = np.eye(4)
    result[:3, 3] = xyz
    result[:3, :3] = [[cc*cb, cc*sb*sa-sc*ca, cc*sb*ca+sc*sa],
                      [sc*cb, sc*sb*sa+cc*ca, sc*sb*ca-cc*sa], [-sb, cb*sa, cb*ca]]
    return result


def origin(node):
    return transform(*[[float(v) for v in (node.get(key, '0 0 0') if node is not None
                                         else '0 0 0').split()] for key in ('xyz', 'rpy')])


def poses(root, arms):
    positions = {f'{s}_j{i}': v for s in ('left', 'right') for i, v in enumerate(arms[s]['initial'], 1)}
    result = {'world': np.eye(4)}
    pending = root.findall('joint')
    while pending:
        old = len(pending)
        for joint in pending[:]:
            parent = joint.find('parent').get('link')
            if parent not in result:
                continue
            t = origin(joint.find('origin'))
            if joint.get('type') == 'revolute':
                assert joint.find('axis').get('xyz') == '0 0 1'
                t = t @ transform([0, 0, 0], [0, 0, positions[joint.get('name')]])
            result[joint.find('child').get('link')] = result[parent] @ t
            pending.remove(joint)
        assert len(pending) < old
    return result


def bounds(root, arms):
    frames = poses(root, arms)
    output = {}
    for link in root.findall('link'):
        all_points = []
        for collision in link.findall('collision'):
            geom = collision.find('geometry')
            mesh = geom.find('mesh')
            if mesh is not None:
                data = (SHARE/mesh.get('filename').split('package://fr3_dual_bolt_cell/')[1]).read_bytes()
                dtype = np.dtype([('normal', '<f4', 3), ('vertices', '<f4', (3, 3)), ('attr', '<u2')])
                vertices = np.frombuffer(data, dtype=dtype, offset=84)['vertices'].reshape(-1, 3)
                vertices = vertices * np.array([float(v) for v in mesh.get('scale', '1 1 1').split()])
            else:
                size = [float(v) for v in geom.find('box').get('size').split()]
                vertices = np.array(list(itertools.product((-.5, .5), repeat=3))) * size
            t = frames[link.get('name')] @ origin(collision.find('origin'))
            all_points.append(vertices @ t[:3, :3].T + t[:3, 3])
        if all_points:
            vertices = np.concatenate(all_points)
            output[link.get('name')] = (vertices.min(axis=0), vertices.max(axis=0))
    return output


def overlap(a, b):
    return np.all(a[1] > b[0]+1e-6) and np.all(a[0] < b[1]-1e-6)


def test_initial_pose_clear_of_table_ground_and_opposite_arm():
    arms = read_yaml(SHARE/'config/arms.yaml')
    root = build_model(SHARE, SHARE/'config/scene.yaml', arms, 'mock')
    boxes = bounds(root, arms)
    for name, limits in boxes.items():
        assert limits[0][2] >= -1e-6, (name, 'below ground')
        for table, size, center in table_boxes(load_scene(SHARE/'config/scene.yaml')):
            table_bounds = (np.array(center)-np.array(size)/2, np.array(center)+np.array(size)/2)
            assert not overlap(limits, table_bounds), (name, table)
    for left, lb in boxes.items():
        if left.startswith('left_'):
            for right, rb in boxes.items():
                if right.startswith('right_'):
                    assert not overlap(lb, rb), (left, right)


def test_collision_link_inertias_positive():
    arms = read_yaml(SHARE/'config/arms.yaml')
    root = build_model(SHARE, SHARE/'config/scene.yaml', arms, 'gazebo')
    for link in root.findall('link'):
        if link.find('collision') is None:
            continue
        inertia = link.find('inertial')
        assert inertia is not None
        assert float(inertia.find('mass').get('value')) > 0
        v = {k: float(x) for k, x in inertia.find('inertia').attrib.items()}
        eig = np.linalg.eigvalsh([[v['ixx'], v['ixy'], v['ixz']],
                                 [v['ixy'], v['iyy'], v['iyz']], [v['ixz'], v['iyz'], v['izz']]])
        assert min(eig) > 0
        assert eig[-1] <= eig[0]+eig[1]+1e-9
