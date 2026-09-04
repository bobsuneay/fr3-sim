"""Generate self-contained SDF: no Fuel models, fake welds, or floating URDF joints."""
import argparse
from copy import deepcopy
import math
from pathlib import Path
import xml.etree.ElementTree as ET

import yaml


def load_scene(path):
    with open(path, encoding='utf-8') as stream:
        scene = yaml.safe_load(stream)
    def check_finite(value):
        if isinstance(value, dict):
            for item in value.values():
                check_finite(item)
        elif isinstance(value, list):
            for item in value:
                check_finite(item)
        elif isinstance(value, (int, float)) and not math.isfinite(value):
            raise ValueError('Scene must not contain NaN or infinity')
    check_finite(scene)
    b, t, c, p = (scene[k] for k in ('bolts', 'table', 'camera', 'physics'))
    for owner, key, length in (
        (scene['support'], 'size', 3), (scene['support'], 'foot_size', 3),
        (scene['mount'], 'xyz', 3), (scene['mount'], 'rpy', 3),
        (c, 'xyz', 3), (c, 'rpy', 3), (t, 'center_xy', 2),
        (t, 'size_xy', 2), (b, 'first_xy', 2), (b, 'spacing_xy', 2),
    ):
        if not isinstance(owner[key], list) or len(owner[key]) != length:
            raise ValueError(f'{key} must contain {length} numbers')
    if min(scene['support']['size']+scene['support']['foot_size']) <= 0:
        raise ValueError('Support dimensions must be positive')
    for key in ('rows', 'cols'):
        if not isinstance(b[key], int) or isinstance(b[key], bool) or b[key] <= 0:
            raise ValueError('bolts.rows and cols must be positive integers')
    if b['rows'] * b['cols'] > 200:
        raise ValueError('Limit the scene to 200 dynamic bolts')
    if not 0 < b['head_length'] < b['length'] < 0.03:
        raise ValueError('Require 0 < head_length < total bolt length < 0.03 m')
    if not 0 < b['shaft_radius'] < b['head_radius'] < 0.015:
        raise ValueError('Require shaft_radius < head_radius < 0.015 m')
    if b['density'] <= 0 or b['spawn_clearance'] < 0:
        raise ValueError('Invalid bolt density or spawn clearance')
    if b['spacing_xy'][0] <= b['length'] or b['spacing_xy'][1] <= 2*b['head_radius']:
        raise ValueError('Bolt spacing would overlap the parts')
    if not 0 < t['thickness'] < t['top_z'] or min(t['size_xy']) <= 0:
        raise ValueError('Invalid table dimensions')
    if not 0 < c['near'] < c['far'] or not 0 < c['horizontal_fov'] < math.pi:
        raise ValueError('Invalid camera clipping or FOV')
    if min(c['width'], c['height'], c['rate']) <= 0:
        raise ValueError('Invalid camera resolution/rate')
    if not all(isinstance(c[k], int) and not isinstance(c[k], bool) for k in ('width', 'height')):
        raise ValueError('Camera width/height must be integers')
    if not 0 < t['leg_width'] < min(t['size_xy'])/4:
        raise ValueError('Invalid table leg width')
    if p['step_size'] <= 0 or p['update_rate'] <= 0 or p['solver_iterations'] <= 0:
        raise ValueError('Invalid physics parameters')
    for _, position in bolt_poses(scene):
        for axis, margin in ((0, b['length']/2), (1, b['head_radius'])):
            if abs(position[axis]-t['center_xy'][axis])+margin >= t['size_xy'][axis]/2:
                raise ValueError('Bolt array extends beyond the tabletop')
    return scene


def table_boxes(scene):
    """Single geometry source shared by Gazebo and MoveIt, in world coordinates."""
    t = scene['table']
    x, y = t['center_xy']
    sx, sy = t['size_xy']
    top, thick, leg = t['top_z'], t['thickness'], t['leg_width']
    result = [('table_top', [sx, sy, thick], [x, y, top-thick/2])]
    for i, sign_x in enumerate((-1, 1)):
        for j, sign_y in enumerate((-1, 1)):
            result.append((
                f'table_leg_{i}_{j}', [leg, leg, top-thick],
                [x+sign_x*(sx/2-leg), y+sign_y*(sy/2-leg), (top-thick)/2]))
    return result


def bolt_poses(scene):
    b = scene['bolts']
    z = scene['table']['top_z'] + b['head_radius'] + b['spawn_clearance']
    return [
        (f'bolt_{r:02d}_{c:02d}', [
            b['first_xy'][0]+r*b['spacing_xy'][0],
            b['first_xy'][1]+c*b['spacing_xy'][1], z])
        for r in range(b['rows']) for c in range(b['cols'])
    ]


def bolt_inertia(b):
    """Exact inertia for the two solid, coaxial cylindrical approximations."""
    length, head = b['length'], b['head_length']
    shaft = length-head
    r, rh, density = b['shaft_radius'], b['head_radius'], b['density']
    ms, mh = density*math.pi*r*r*shaft, density*math.pi*rh*rh*head
    zs, zh = -head/2, shaft/2
    mass = ms+mh
    com = (ms*zs+mh*zh)/mass
    transverse = (
        ms*(3*r*r+shaft*shaft)/12 + ms*(zs-com)**2
        + mh*(3*rh*rh+head*head)/12 + mh*(zh-com)**2)
    axial = (ms*r*r+mh*rh*rh)/2
    return mass, com, transverse, axial


def element(parent, tag, value=None, **attributes):
    child = ET.SubElement(parent, tag, attributes)
    if value is not None:
        child.text = (' '.join(f'{v:.10g}' for v in value)
                      if isinstance(value, (list, tuple)) else str(value))
    return child


def geometry(parent, kind, dimensions):
    shape = element(element(parent, 'geometry'), kind)
    if kind == 'box':
        element(shape, 'size', dimensions)
    elif kind == 'plane':
        element(shape, 'normal', [0, 0, 1])
        element(shape, 'size', [10, 10])
    else:
        element(shape, 'radius', dimensions[0])
        element(shape, 'length', dimensions[1])


def appearance(parent, rgba):
    material = element(parent, 'material')
    element(material, 'ambient', rgba)
    element(material, 'diffuse', rgba)
    element(material, 'specular', [0.15, 0.15, 0.15, 1])


def contact(collision, friction=0.8):
    surface = element(collision, 'surface')
    ode = element(element(surface, 'friction'), 'ode')
    element(ode, 'mu', friction)
    element(ode, 'mu2', friction)
    ode = element(element(surface, 'contact'), 'ode')
    element(ode, 'kp', 100000)
    element(ode, 'kd', 10)
    element(ode, 'max_vel', 0.05)
    element(ode, 'min_depth', 0.00005)


def world_xml(scene):
    sdf = ET.Element('sdf', version='1.6')
    world = element(sdf, 'world', name='fr3_bolt_cell')
    element(world, 'gravity', [0, 0, -9.81])
    state_plugin = element(world, 'plugin', name='gazebo_state', filename='libgazebo_ros_state.so')
    element(element(state_plugin, 'ros'), 'namespace', '/gazebo')
    element(state_plugin, 'update_rate', 30.0)
    physics = element(world, 'physics', name='small_part_physics', type='ode')
    element(physics, 'max_step_size', scene['physics']['step_size'])
    element(physics, 'real_time_update_rate', scene['physics']['update_rate'])
    element(physics, 'real_time_factor', 1)
    ode = element(physics, 'ode')
    solver = element(ode, 'solver')
    element(solver, 'type', 'quick')
    element(solver, 'iters', scene['physics']['solver_iterations'])
    constraints = element(ode, 'constraints')
    element(constraints, 'cfm', 0)
    element(constraints, 'erp', 0.2)
    element(constraints, 'contact_max_correcting_vel', 0.05)
    element(constraints, 'contact_surface_layer', 0.00005)
    visual_scene = element(world, 'scene')
    element(visual_scene, 'ambient', [0.65, 0.65, 0.65, 1])
    element(visual_scene, 'background', [0.88, 0.91, 0.95, 1])
    element(visual_scene, 'shadows', 'true')
    light = element(world, 'light', name='cell_light', type='directional')
    element(light, 'pose', [0, 0, 3, 0, 0, 0])
    element(light, 'diffuse', [0.9, 0.9, 0.9, 1])
    element(light, 'specular', [0.2, 0.2, 0.2, 1])
    element(light, 'direction', [-0.5, 0.2, -1])
    element(light, 'cast_shadows', 'true')
    gui_camera = element(element(world, 'gui', fullscreen='0'), 'camera', name='overview')
    element(gui_camera, 'pose', [1.5, -1.9, 1.8, 0, 0.43, 2.12])
    element(gui_camera, 'view_controller', 'orbit')

    ground = element(world, 'model', name='ground')
    element(ground, 'static', 'true')
    ground_link = element(ground, 'link', name='ground_link')
    ground_collision = element(ground_link, 'collision', name='collision')
    geometry(ground_collision, 'plane', [])
    contact(ground_collision)
    ground_visual = element(ground_link, 'visual', name='visual')
    geometry(ground_visual, 'plane', [])
    appearance(ground_visual, [0.55, 0.58, 0.60, 1])
    table = element(world, 'model', name='work_table')
    element(table, 'static', 'true')
    table_link = element(table, 'link', name='table_link')
    for name, size, position in table_boxes(scene):
        collision = element(table_link, 'collision', name=name+'_collision')
        element(collision, 'pose', position+[0, 0, 0])
        geometry(collision, 'box', size)
        contact(collision)
        visual = element(table_link, 'visual', name=name+'_visual')
        element(visual, 'pose', position+[0, 0, 0])
        geometry(visual, 'box', size)
        appearance(visual, [0.56, 0.38, 0.23, 1] if name == 'table_top' else [0.3, 0.32, 0.35, 1])

    b = scene['bolts']
    mass, com, transverse, axial = bolt_inertia(b)
    # Clone a two-cylinder template, keeping each bolt as an independent dynamic model.
    template = ET.Element('model', name='bolt')
    element(template, 'static', 'false')
    element(template, 'allow_auto_disable', 'true')
    link = element(template, 'link', name='body')
    inertial = element(link, 'inertial')
    element(inertial, 'pose', [0, 0, com, 0, 0, 0])
    element(inertial, 'mass', mass)
    inertia = element(inertial, 'inertia')
    for key, value in dict(ixx=transverse, iyy=transverse, izz=axial,
                           ixy=0, ixz=0, iyz=0).items():
        element(inertia, key, value)
    for name, radius, length, z in (
        ('shaft', b['shaft_radius'], b['length']-b['head_length'], -b['head_length']/2),
        ('head', b['head_radius'], b['head_length'], (b['length']-b['head_length'])/2),
    ):
        collision = element(link, 'collision', name=name+'_collision')
        element(collision, 'pose', [0, 0, z, 0, 0, 0])
        element(collision, 'max_contacts', 8)
        geometry(collision, 'cylinder', [radius, length])
        contact(collision)
        visual = element(link, 'visual', name=name+'_visual')
        element(visual, 'pose', [0, 0, z, 0, 0, 0])
        geometry(visual, 'cylinder', [radius, length])
        appearance(visual, [0.65, 0.69, 0.73, 1])
    for name, position in bolt_poses(scene):
        model = deepcopy(template)
        model.set('name', name)
        element(model, 'pose', position+[0, math.pi/2, 0])
        world.append(model)
    ET.indent(sdf, space='  ')
    return ET.tostring(sdf, encoding='unicode', xml_declaration=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--scene', type=Path)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    if args.scene is None:
        from ament_index_python.packages import get_package_share_directory
        args.scene = Path(get_package_share_directory('fr3_bolt_cell'))/'config/scene.yaml'
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(world_xml(load_scene(args.scene)), encoding='utf-8')
    print(args.output)


if __name__ == '__main__':
    main()
