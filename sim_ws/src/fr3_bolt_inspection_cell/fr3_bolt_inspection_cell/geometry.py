"""Derive fingertip geometry from the same URDF/STL used by the simulator.

The serializable result is generated at launch. Runtime height checks only
need joint feedback and the TCP pose; they do not load meshes or query Gazebo.
"""
from pathlib import Path
import struct

import numpy as np
from scipy.spatial import ConvexHull

from .core import transform


def stl_vertices(path):
    """Read binary or ASCII STL vertices, retaining the mesh's native units."""
    data = Path(path).read_bytes()
    count = struct.unpack_from('<I', data, 80)[0] if len(data) >= 84 else 0
    if count and len(data) == 84 + 50*count:
        dtype = np.dtype([('normal', '<f4', 3), ('vertices', '<f4', (3, 3)),
                          ('attribute', '<u2')])
        vertices = np.frombuffer(data, dtype=dtype, offset=84)['vertices'].reshape(-1, 3)
    else:
        try:
            vertices = np.asarray([[float(v) for v in line.split()[1:]]
                                   for line in data.decode('ascii').splitlines()
                                   if line.strip().startswith('vertex ')], dtype=float)
        except (UnicodeDecodeError, ValueError) as error:
            raise ValueError(f'Invalid STL geometry: {path}') from error
    if (vertices.ndim != 2 or vertices.shape[1] != 3 or len(vertices) < 3 or
            not np.all(np.isfinite(vertices))):
        raise ValueError(f'Empty or non-finite STL geometry: {path}')
    return np.unique(vertices, axis=0).astype(float)


def _origin(element):
    return transform(*[[float(v) for v in (element.get(key, '0 0 0')
                       if element is not None else '0 0 0').split()]
                       for key in ('xyz', 'rpy')])


def _fixed_transform(root, parent, child):
    """Resolve a fixed descendant frame, rejecting an unexpected moving TCP."""
    result = np.eye(4)
    visited = set()
    while child != parent:
        if child in visited:
            raise ValueError('Cycle in gripper frame tree')
        visited.add(child)
        joint = next((j for j in root.findall('joint')
                      if j.find('child') is not None and
                      j.find('child').get('link') == child), None)
        if joint is None or joint.get('type') != 'fixed':
            raise ValueError(f'{child} must be fixed relative to {parent}')
        result = _origin(joint.find('origin')) @ result
        child = joint.find('parent').get('link')
    return result


def fingertip_geometry(root, side, mesh_resolver):
    """Return both fingertips at q=0 in TCP coordinates and their motion axes.

    ``mesh_resolver(uri)`` must resolve the actual installed mesh URI to a
    readable path. Convex-hull vertices preserve the exact lowest mesh point
    for every orientation, while making the launch-generated data compact.
    This hull is only a height support function, never a collision replacement.
    """
    palm, tcp = side+'_gripper_palm', side+'_gripper_tcp'
    tcp_palm = np.linalg.inv(_fixed_transform(root, palm, tcp))
    output = []
    for finger in ('left', 'right'):
        name = side+'_'+finger+'_finger'
        link = root.find(f"link[@name='{name}']")
        joint = next((j for j in root.findall('joint')
                      if j.find('child') is not None and
                      j.find('child').get('link') == name), None)
        if (link is None or joint is None or joint.get('type') != 'prismatic' or
                joint.find('parent').get('link') != palm):
            raise ValueError(f'{name} needs a prismatic joint relative to its palm')
        collisions = [c for c in link.findall('collision')
                      if c.find('geometry/mesh') is not None and
                      c.find('geometry/mesh').get('filename', '').endswith('/finger.stl')]
        if len(collisions) != 1:
            raise ValueError(f'{name} must contain one finger.stl collision mesh')
        collision = collisions[0]
        mesh = collision.find('geometry/mesh')
        vertices = stl_vertices(mesh_resolver(mesh.get('filename')))
        scale = np.asarray([float(v) for v in mesh.get('scale', '1 1 1').split()])
        if scale.shape != (3,) or not np.all(np.isfinite(scale)) or np.any(scale <= 0):
            raise ValueError(f'{name} needs a finite positive mesh scale')
        vertices *= scale
        # Support extrema lie on this hull; no bounding-box corners are invented.
        vertices = vertices[ConvexHull(vertices).vertices]
        tcp_joint = tcp_palm @ _origin(joint.find('origin'))
        tcp_mesh = tcp_joint @ _origin(collision.find('origin'))
        vertices = vertices @ tcp_mesh[:3, :3].T + tcp_mesh[:3, 3]
        axis = np.asarray([float(v) for v in joint.find('axis').get('xyz').split()])
        if axis.shape != (3,) or not np.all(np.isfinite(axis)) or np.linalg.norm(axis) == 0:
            raise ValueError(f'{name} has an invalid joint axis')
        axis = tcp_joint[:3, :3] @ (axis/np.linalg.norm(axis))
        output.append({'joint': joint.get('name'), 'vertices_tcp': vertices.tolist(),
                       'axis_tcp': axis.tolist()})
    return output


def fingertip_points_tcp(geometry, positions):
    """Apply measured (or commanded) finger positions to launch-generated data."""
    if len(geometry) != 2:
        raise ValueError('Require geometry for both fingers')
    output = []
    for finger in geometry:
        vertices = np.asarray(finger['vertices_tcp'], dtype=float)
        axis = np.asarray(finger['axis_tcp'], dtype=float)
        position = float(positions[finger['joint']])
        if (vertices.ndim != 2 or vertices.shape[1] != 3 or not len(vertices) or
                axis.shape != (3,) or not np.all(np.isfinite(vertices)) or
                not np.all(np.isfinite(axis)) or not np.isfinite(position)):
            raise ValueError('Invalid fingertip geometry or joint feedback')
        output.append(vertices+axis*position)
    return np.concatenate(output)
