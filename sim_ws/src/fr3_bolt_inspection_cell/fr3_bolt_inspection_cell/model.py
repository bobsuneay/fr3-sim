"""Extend the existing dual cell without changing its assets or launch behaviour."""
import math
import xml.etree.ElementTree as ET
from fr3_dual_bolt_cell.model import element, fixed, box, inertial
from .core import validate


def augment(root, cfg, sim):
    validate(cfg)
    root.set('name', 'fr3_bolt_inspection_cell')
    for side in ('left', 'right'):
        # Keep the HKV palm/rail; substitute explicit narrow inspection jaws.
        for which, sign in (('left', -1), ('right', 1)):
            name = f'{side}_{which}_finger'
            link = root.find(f"link[@name='{name}']")
            for node in list(link):
                link.remove(node)
            inertial(link, .03, (.008, .004, .078), (0, 0, .039))
            box(link, (.008, .004, .078), (0, 0, .039), visual=True)
            box(link, (.008, .004, .078), (0, 0, .039))
            joint = root.find(f"joint[@name='{name}_joint']")
            joint.find('origin').set('xyz', f'{sign*.004} 0 .072')
            state = root.find(f"ros2_control/joint[@name='{name}_joint']/state_interface[@name='position']")
            initial = state.find("param[@name='initial_value']")
            if initial is None:
                initial = element(state, 'param', name='initial_value')
            initial.text = str(cfg['open_width']/2)
        tcp = root.find(f"joint[@name='{side}_palm_to_tcp']/origin")
        tcp.set('xyz', '0 0 .149')
        # The simulator grasp plugin attaches to this physical palm frame.
        if sim:
            g = element(root, 'gazebo', reference=side+'_tool_to_gripper')
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
