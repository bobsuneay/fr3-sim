"""Extend the existing dual cell without changing its assets or launch behaviour."""
from copy import deepcopy
import math
import xml.etree.ElementTree as ET
from fr3_dual_bolt_cell.model import element, fixed, box, inertial
from .core import validate


def simplified_hkv_collisions(link, boxes, keep_mesh=None):
    """Replace unstable CAD collisions with boxes plus an optional tip mesh.

    The palm and finger sliders use the same simple collision proxies as the
    dual bolt cell.  Only ``finger.stl`` is retained on each moving finger so
    the two actual gripping tips still have their HKV shape.
    """
    for collision in list(link.findall('collision')):
        link.remove(collision)
    for size, xyz in boxes:
        box(link, size, xyz)
    if keep_mesh is None:
        return
    for visual in link.findall('visual'):
        mesh = visual.find('geometry/mesh')
        if mesh is None or not mesh.get('filename', '').endswith('/'+keep_mesh):
            continue
        collision = deepcopy(visual)
        collision.tag = 'collision'
        for material in list(collision.findall('material')):
            collision.remove(material)
        link.append(collision)
        return
    raise ValueError(f'Expected {keep_mesh} visual on {link.get("name")}')


def stabilize_gripper_contacts(root, side):
    """Prevent CAD self-contacts from exciting the fixed mount or fingers.

    The HKV flange mesh overlaps the last FR3 wrist link by design: it is a
    mounting surface, not a physical clearance gap.  Gazebo otherwise applies
    contact impulses between those links while also enforcing the fixed
    wrist/tool/palm joints.  The resulting constraint/contact feedback looks
    like the flange is sliding.  ``selfCollide`` only disables collisions
    between links in this robot; contacts against the bolt, table and other
    models remain enabled.
    """
    for link_name in (f'{side}_tool0', f'{side}_gripper_palm',
                      f'{side}_gripper_tcp', f'{side}_left_finger',
                      f'{side}_right_finger'):
        gazebo = root.find(f"gazebo[@reference='{link_name}']")
        if gazebo is None:
            gazebo = element(root, 'gazebo', reference=link_name)
        self_collide = gazebo.find('selfCollide')
        if self_collide is None:
            self_collide = element(gazebo, 'selfCollide')
        self_collide.text = 'false'

    for which in ('left', 'right'):
        reference = f'{side}_{which}_finger'
        gazebo = root.find(f"gazebo[@reference='{reference}']")
        if gazebo is None:
            gazebo = element(root, 'gazebo', reference=reference)
        for tag, value in (('mu1', '0.35'), ('mu2', '0.35'),
                           ('kp', '30000'), ('kd', '80')):
            node = gazebo.find(tag)
            if node is None:
                node = element(gazebo, tag)
            node.text = value
        joint = root.find(f"joint[@name='{reference}_joint']")
        if joint is None:
            # Also support the scalar/mimic HKV layout while keeping the same
            # contact stabilization for its physical finger child link.
            joint = next((candidate for candidate in root.findall('joint')
                          if candidate.find('child') is not None and
                          candidate.find('child').get('link') == reference), None)
        if joint is not None:
            dynamics = joint.find('dynamics')
            if dynamics is None:
                dynamics = element(joint, 'dynamics')
            dynamics.set('damping', '15.0')
            dynamics.set('friction', '0.40')


def augment(root, cfg, sim):
    validate(cfg)
    root.set('name', 'fr3_bolt_inspection_cell')
    for side in ('left', 'right'):
        # The baseline keeps tool0 as a massless coordinate frame. Gazebo can
        # reliably preserve this fixed joint only when both links have inertia.
        tool = root.find(f"link[@name='{side}_tool0']")
        if tool.find('inertial') is None:
            inertial(tool, .01, (.02, .02, .01))
        # Keep the stable dual-cell proxies for the mounting body and rail.
        # Only the two moving fingertip meshes remain detailed for grasping.
        simplified_hkv_collisions(
            root.find(f"link[@name='{side}_gripper_palm']"),
            [((.16, .0705, .0615), (0, .00325, .03075)),
             ((.155, .007, .0048), (0, 0, .0694))])
        for which in ('left', 'right'):
            simplified_hkv_collisions(
                root.find(f"link[@name='{side}_{which}_finger']"),
                [((.024, .017, .0065), (0, 0, .00485))],
                keep_mesh='finger.stl')
        stabilize_gripper_contacts(root, side)
        # The simulator grasp plugin attaches to this physical palm frame.
        if sim:
            # Gazebo's URDF importer otherwise reduces the wrist/tool/palm fixed
            # chain. On Gazebo 11 that can also discard the downstream finger
            # joints before gazebo_ros2_control discovers them.
            for joint in (side+'_wrist_to_tool', side+'_tool_to_gripper'):
                g = element(root, 'gazebo', reference=joint)
                element(g, 'preserveFixedJoint').text = 'true'
    for name, c in cfg['cameras'].items():
        if name == 'waist_camera':
            bracket_xyz, bracket_size = (.068, 0, 1.22), (.035, .015, .015)
        else:
            # Both palm frames already mirror in the assembled robot. Derive
            # the bracket side from the calibrated local camera position so a
            # name-based second mirror cannot put the right camera underneath.
            sign = 1 if c['xyz'][1] >= 0 else -1
            bracket_xyz, bracket_size = (0, sign*.055, .020), (.015, .050, .015)
        mount = element(root, 'link', name=name+'_bracket')
        inertial(mount, .025, bracket_size)
        box(mount, bracket_size, visual=True)
        box(mount, bracket_size)
        fixed(root, name+'_bracket_joint', c['parent'], name+'_bracket', bracket_xyz)
        link = element(root, 'link', name=name+'_link')
        size = (.025, .090, .025)  # camera +X forward; D435i housing 90 x 25 x 25 mm
        inertial(link, .075, size)
        box(link, size, visual=True)
        box(link, size)
        fixed(root, name+'_mount', name+'_bracket', name+'_link',
              [a-b for a, b in zip(c['xyz'], bracket_xyz)], c['rpy'])
        element(root, 'link', name=name+'_optical_frame')
        fixed(root, name+'_optical_joint', name+'_link', name+'_optical_frame',
              rpy=(-math.pi/2, 0, -math.pi/2))
        if not sim:
            continue
        g = element(root, 'gazebo', reference=name+'_mount')
        element(g, 'preserveFixedJoint').text = 'true'
        g = element(root, 'gazebo', reference=name+'_link')
        element(g, 'material').text = 'Gazebo/Black'
        sensor = element(g, 'sensor', name=name+'_rgbd', type='depth')
        element(sensor, 'always_on').text = 'true'
        element(sensor, 'update_rate').text = str(c['rate'])
        camera = element(sensor, 'camera', name=name)
        element(camera, 'horizontal_fov').text = str(c['horizontal_fov'])
        image = element(camera, 'image')
        for tag in ('width', 'height'):
            element(image, tag).text = str(c[tag])
        element(image, 'format').text = 'R8G8B8'
        clip = element(camera, 'clip')
        for tag in ('near', 'far'):
            element(clip, tag).text = str(c[tag])
        plugin = element(sensor, 'plugin', name=name+'_ros', filename='libgazebo_ros_camera.so')
        ros = element(plugin, 'ros')
        element(ros, 'namespace').text = '/'
        element(plugin, 'camera_name').text = name
        element(plugin, 'frame_name').text = name+'_optical_frame'
        element(plugin, 'min_depth').text = str(c['near'])
        element(plugin, 'max_depth').text = str(c['far'])
    if sim:
        # gazebo_ros2_control Humble declares plugin-wide parameters (including
        # hold_joints) for every <ros2_control> block. A dual block therefore
        # emits a duplicate-parameter error. Both arms use the same GazeboSystem,
        # so expose all 16 joints through one hardware block.
        systems = root.findall('ros2_control')
        if len(systems) != 2:
            raise ValueError('Expected one Gazebo ros2_control system per arm')
        combined = systems[0]
        combined.set('name', 'inspection_gazebo_system')
        for joint in systems[1].findall('joint'):
            combined.append(joint)
        root.remove(systems[1])
    return root


def inspection_world(base_xml, cfg):
    root = ET.fromstring(base_xml)
    world = root.find('world')
    p = element(world, 'plugin', name='inspection_grasp', filename='libfr3_inspection_grasp.so')
    ros = element(p, 'ros')
    element(ros, 'namespace').text = '/inspection/sim'
    element(p, 'robot_model').text = 'fr3_dual_cell'
    element(p, 'object_model').text = cfg['simulation_entity']
    # State is used exclusively to validate grasp/centre drift, never to estimate a pick.
    p = element(world, 'plugin', name='inspection_state', filename='libgazebo_ros_state.so')
    element(element(p, 'ros'), 'namespace').text = '/inspection/sim'
    element(p, 'update_rate').text = '30'
    return ET.tostring(root, encoding='unicode')
