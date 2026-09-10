"""Extend the existing dual cell without changing its assets or launch behaviour."""
from copy import deepcopy
import math
import xml.etree.ElementTree as ET
from fr3_dual_bolt_cell.model import element, fixed, box, inertial
from .core import validate


def mesh_collisions_from_visual(link):
    """Use the supplied HKV CAD mesh for collision as well as display."""
    for collision in list(link.findall('collision')):
        link.remove(collision)
    for visual in link.findall('visual'):
        if visual.find('geometry/mesh') is None:
            continue
        collision = deepcopy(visual)
        collision.tag = 'collision'
        for material in list(collision.findall('material')):
            collision.remove(material)
        link.append(collision)


def stabilize_gripper_contacts(root, side):
    """Prevent tiny CAD self-contacts from exciting Gazebo finger joints."""
    for which in ('left', 'right'):
        reference = f'{side}_{which}_finger'
        gazebo = root.find(f"gazebo[@reference='{reference}']")
        if gazebo is None:
            gazebo = element(root, 'gazebo', reference=reference)
        self_collide = gazebo.find('selfCollide')
        if self_collide is None:
            self_collide = element(gazebo, 'selfCollide')
        self_collide.text = 'false'
        for tag, value in (('mu1', '0.35'), ('mu2', '0.35'),
                           ('kp', '30000'), ('kd', '80')):
            node = gazebo.find(tag)
            if node is None:
                node = element(gazebo, tag)
            node.text = value


def augment(root, cfg, sim):
    validate(cfg)
    root.set('name', 'fr3_bolt_inspection_cell')
    for side in ('left', 'right'):
        # The baseline keeps tool0 as a massless coordinate frame. Gazebo can
        # reliably preserve this fixed joint only when both links have inertia.
        tool = root.find(f"link[@name='{side}_tool0']")
        if tool.find('inertial') is None:
            inertial(tool, .01, (.02, .02, .01))
        # Keep the original HKV palm and fingers. Their CAD meshes are used
        # directly for collision, avoiding oversized rectangular proxies.
        mesh_collisions_from_visual(root.find(f"link[@name='{side}_gripper_palm']"))
        for which in ('left', 'right'):
            mesh_collisions_from_visual(root.find(f"link[@name='{side}_{which}_finger']"))
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
            sign = 1 if name.startswith('left') else -1
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
