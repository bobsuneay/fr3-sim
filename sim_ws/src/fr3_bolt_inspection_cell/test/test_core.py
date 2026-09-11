"""Offline regressions. They do not replace ROS/Gazebo integration acceptance."""
from pathlib import Path
import ast
import itertools
import sys
import xml.etree.ElementTree as ET
import numpy as np
import pytest
from scipy.spatial.transform import Rotation
import yaml

SHARE = Path(__file__).resolve().parents[1]
BASE = SHARE.parent/'fr3_dual_bolt_cell'
sys.path[:0] = [str(SHARE), str(BASE)]
from fr3_bolt_inspection_cell.core import (SimClockDeadline, centered_views, estimate_bolt,
    grasp_in_object, interpolate_object, random_disk_xy, segment_times, table_pick_tcp,
    transform, validate)
from fr3_bolt_inspection_cell.model import augment, inspection_world
from fr3_dual_bolt_cell.model import build_model, semantic
from fr3_dual_bolt_cell.world import load_scene, world_xml


def config():
    return yaml.safe_load((SHARE/'config/inspection.yaml').read_text())


def robot(mode='mock'):
    arms = yaml.safe_load((SHARE/'config/arms.yaml').read_text())
    return augment(build_model(BASE, SHARE/'config/scene.yaml', arms, mode), config(), mode == 'gazebo'), arms


def cloud(yaw=.4, noisy=True):
    rng = np.random.default_rng(712)
    # Visible upper half of a horizontal bolt; denser head and shuffled points.
    points = []
    for lo, hi, radius in ((-.0175, .0115, .004), (.0115, .0175, .006)):
        x = np.linspace(lo, hi, 65)
        theta = np.linspace(.08, np.pi-.08, 33)
        points.extend([a, radius*np.cos(b), radius*np.sin(b)] for a in x for b in theta)
    p = np.array(points)
    if noisy:
        p += rng.normal(0, .00008, p.shape)
    r = Rotation.from_euler('z', yaw).as_matrix()
    return p@r.T+[.50, -.2, .7245]


@pytest.mark.parametrize('angle', [0, .6, 2.6, -2.9])
def test_partial_cloud_finds_head_and_pose(angle):
    result = estimate_bolt(cloud(angle), config())
    assert np.linalg.norm(result.pose[:3, 3]-[.50, -.2, .7245]) < .0008
    assert result.pose[:3, 0] @ np.array([np.cos(angle), np.sin(angle), 0]) > .998
    assert result.head_resolved


def test_reject_bad_clouds():
    cfg = config()
    for p in (np.full((100, 3), np.nan), np.zeros((100, 3)),
              np.r_[cloud()+[0, -.014, 0], cloud()+[0, .014, 0]],
              np.repeat(cloud()[:2], 20000, axis=0)):
        with pytest.raises(ValueError):
            estimate_bolt(p, cfg)


def test_object_center_remains_fixed_through_interpolated_views():
    c = config()
    for below in (False, True):
        grasp = grasp_in_object(.006 if below else -.006, below)
        assert np.linalg.det(grasp[:3, :3]) == pytest.approx(1)
        neutral = transform(c['inspection_center'], (.2, -.1, .7))
        for obj, tcp in centered_views(c['inspection_center'], neutral[:3, :3], grasp, c['views_deg']):
            assert np.allclose(tcp@np.linalg.inv(grasp), obj)
            for sample in interpolate_object(neutral, obj, grasp):
                assert np.allclose((sample@np.linalg.inv(grasp))[:3, 3], c['inspection_center'])


def test_table_pick_lowers_tcp_without_mutating_estimate():
    obj = transform([.5, -.2, .725], [0, 0, .6])
    original = obj.copy()
    nominal = obj@grasp_in_object(-.01)
    target = table_pick_tcp(obj, -.01, .002)
    assert np.allclose(target[:2, 3], nominal[:2, 3])
    assert target[2, 3] == pytest.approx(nominal[2, 3]-.002)
    assert np.allclose(target[:3, :3], nominal[:3, :3])
    assert np.allclose(obj, original)


def test_default_fingertip_clearance_is_five_mm():
    assert config()['fingertip_table_clearance'] == pytest.approx(.005)


def test_random_object_positions_are_uniform_disk_bounded_and_reproducible():
    c = config()
    rng = np.random.default_rng(91)
    a = np.array([random_disk_xy(c['random_position_center'],
        c['random_position_radius'], rng) for _ in range(2000)])
    b = np.array([random_disk_xy(c['random_position_center'],
        c['random_position_radius'], np.random.default_rng(91)) for _ in range(1)])
    first = random_disk_xy(c['random_position_center'], c['random_position_radius'],
                           np.random.default_rng(91))
    radii = np.linalg.norm(a-np.asarray(c['random_position_center']), axis=1)
    assert np.max(radii) <= .05+1e-12
    assert .032 < np.mean(radii) < .035  # uniform area has mean radius 2R/3
    assert np.allclose(b[0], first)


def test_sim_clock_deadline_accepts_slow_simulation_and_detects_stalls():
    deadline = SimClockDeadline(100.0, 10.0, timeout=8.0, stall_timeout=30.0)
    assert deadline.check(102.0, 35.0) == pytest.approx(2.0)
    assert deadline.check(104.0, 64.0) == pytest.approx(4.0)
    with pytest.raises(TimeoutError, match='clock stopped'):
        deadline.check(104.0, 94.1)

    deadline = SimClockDeadline(100.0, 10.0, timeout=8.0)
    with pytest.raises(TimeoutError, match='ROS-time deadline'):
        deadline.check(108.1, 11.0)


def test_simplified_hkv_body_keeps_only_fingertip_mesh_collision():
    root, _ = robot('mock')
    for side in ('left', 'right'):
        palm = root.find(f"link[@name='{side}_gripper_palm']")
        assert len(palm.findall('collision')) == 2
        assert all(c.find('geometry/box') is not None for c in palm.findall('collision'))
        assert not palm.findall('collision/geometry/mesh')
        for part in ('left_finger', 'right_finger'):
            link = root.find(f"link[@name='{side}_{part}']")
            collisions = link.findall('collision')
            assert len(collisions) == 2
            assert sum(c.find('geometry/box') is not None for c in collisions) == 1
            meshes = [c.find('geometry/mesh').get('filename') for c in collisions
                      if c.find('geometry/mesh') is not None]
            assert meshes == [f'package://fr3_dual_bolt_cell/meshes/hkv_tg9801/finger.stl']
            assert not any('slider.stl' in name for name in meshes)
            contact = root.find(f"gazebo[@reference='{side}_{part}']")
            assert contact.findtext('selfCollide') == 'false'
            assert contact.findtext('kd') == '80'
            joint = next(j for j in root.findall('joint')
                         if j.find('child') is not None and
                         j.find('child').get('link') == f'{side}_{part}')
            joint = joint.find('dynamics')
            assert float(joint.get('damping')) == pytest.approx(15.0)
            assert float(joint.get('friction')) == pytest.approx(.40)


def test_gazebo_mount_links_do_not_self_collide_with_fixed_wrist_chain():
    root, _ = robot('gazebo')
    for side in ('left', 'right'):
        for part in ('tool0', 'gripper_palm', 'gripper_tcp',
                     'left_finger', 'right_finger'):
            gazebo = root.find(f"gazebo[@reference='{side}_{part}']")
            assert gazebo is not None
            assert gazebo.findtext('selfCollide') == 'false'
        for joint in ('wrist_to_tool', 'tool_to_gripper'):
            gazebo = root.find(
                f"gazebo[@reference='{side}_{joint}']/preserveFixedJoint")
            assert gazebo is not None and gazebo.text == 'true'


def test_grasp_config_remains_valid():
    # The grasp gate still validates the original controller opening range.
    c = config()
    assert c['close_width'] < .005 < c['open_width']


def test_rest_to_rest_timing_bounds():
    q = np.array([[0, 0], [.03, .01], [.035, .04]])
    xyz = np.array([[0, 0, 0], [.01, 0, 0], [.02, 0, 0]])
    times = segment_times(q, xyz, .008, .12, .15)
    dt = np.diff(times)
    assert np.all(1.875*np.linalg.norm(np.diff(xyz, axis=0), axis=1)/dt <= .008+1e-10)
    dq = np.max(np.abs(np.diff(q, axis=0)), axis=1)
    assert np.all(1.875*dq/dt <= .12+1e-10)
    assert np.all(5.8*dq/dt**2 <= .15+1e-10)


@pytest.mark.parametrize('key,value', [('first_arm', 'bad'), ('grasp_offset', .02),
    ('descent_speed', 0), ('center_tolerance', float('nan')), ('open_width', .001),
    ('minimum_views', 1), ('max_points', 1000000), ('fingertip_table_clearance', .021),
    ('random_position_radius', 0), ('grasp_test_lift', .05),
    ('max_grasp_attempts', 0), ('max_grasp_attempts', 2.5)])
def test_invalid_config_rejected(key, value):
    c = config()
    c[key] = value
    with pytest.raises(ValueError):
        validate(c)


def test_camera_frames_materials_and_backends():
    c = config()
    assert c['point_cloud_camera'] == 'head_camera'
    assert c['cameras']['waist_camera']['depth'] is False
    assert c['cameras']['left_d435i']['depth'] is True
    assert c['cameras']['right_d435i']['depth'] is True
    for mode in ('gazebo', 'mock'):
        root, _ = robot(mode)
        names = {l.get('name') for l in root.findall('link')}
        assert all(c+'_optical_frame' in names for c in config()['cameras'])
        for visual in root.findall('.//visual'):
            assert visual.find('material/color') is not None
        plugins = [p.get('filename') for p in root.findall('.//plugin')]
        assert plugins.count('libgazebo_ros_camera.so') == (4 if mode == 'gazebo' else 0)
        if mode == 'gazebo':
            sensors = {s.get('name'): s.get('type') for s in root.findall('.//sensor')}
            assert sensors['waist_camera_rgb'] == 'camera'
            assert sensors['left_d435i_rgbd'] == 'depth'
            assert sensors['right_d435i_rgbd'] == 'depth'
        for joint in root.findall('ros2_control/joint'):
            assert root.find(f"joint[@name='{joint.get('name')}']") is not None
        systems = root.findall('ros2_control')
        assert len(systems) == (1 if mode == 'gazebo' else 4)
        if mode == 'gazebo':
            controlled = {j.get('name') for j in systems[0].findall('joint')}
            assert len(controlled) == 16
            for side in ('left', 'right'):
                assert root.find(
                    f"gazebo[@reference='{side}_wrist_to_tool']/preserveFixedJoint") is not None
                assert root.find(
                    f"gazebo[@reference='{side}_tool_to_gripper']/preserveFixedJoint") is not None
                tool = root.find(f"link[@name='{side}_tool0']/inertial")
                assert tool is not None and float(tool.find('mass').get('value')) > 0


def test_camera_aims_at_part_and_has_realistic_standoff():
    c = config()
    waist = c['cameras']['waist_camera']
    local = np.linalg.inv(transform(waist['xyz'], waist['rpy'])) @ np.r_[c['inspection_center'], 1]
    assert local[0] > .15
    assert abs(np.arctan2(local[1], local[0])) < waist['horizontal_fov']/2
    assert abs(np.arctan2(local[2], local[0])) < .10
    for side in ('left', 'right'):
        cam = c['cameras'][side+'_d435i']
        local = np.linalg.inv(transform(cam['xyz'], cam['rpy'])) @ [0, 0, .149, 1]
        assert local[0] > .15
        assert abs(np.arctan2(local[1], local[0])) < .1
        assert abs(np.arctan2(local[2], local[0])) < .1


def test_wrist_cameras_are_body_centreline_mirrors_at_initial_pose():
    root, arms = robot()
    sys.path.insert(0, str(BASE/'test'))
    import test_geometry as g
    frames = g.poses(root, arms)
    left, right = frames['left_d435i_link'], frames['right_d435i_link']
    reflection = np.diag([1, -1, 1])
    assert np.allclose(right[:3, 3], reflection@left[:3, 3], atol=1e-8)
    # Reflect world Y and camera-local Y to keep a right-handed rotation.
    assert np.allclose(right[:3, :3], reflection@left[:3, :3]@reflection, atol=1e-8)
    assert left[2, 3] > frames['left_gripper_palm'][2, 3]+.05
    assert right[2, 3] > frames['right_gripper_palm'][2, 3]+.05


def test_initial_collision_geometry():
    # Reuse baseline's SAT implementation against the extended model.
    sys.path.insert(0, str(BASE/'test'))
    import test_geometry as g
    root, arms = robot()
    srdf = ET.fromstring(semantic(root, arms))
    allowed = {frozenset((e.get('link1'), e.get('link2'))) for e in srdf.findall('disable_collisions')}
    frames = g.poses(root, arms)
    for joint in root.findall("joint[@type='prismatic']"):
        q = float(root.find(f"ros2_control/joint[@name='{joint.get('name')}']/state_interface[@name='position']/param").text)
        child = joint.find('child').get('link')
        axis = np.array([float(x) for x in joint.find('axis').get('xyz').split()])
        frames[child][:3, 3] += frames[child][:3, :3]@axis*q
    boxes = g.oriented_boxes(g.local_collision_boxes(root), frames)
    conflicts = []
    for a, b in itertools.combinations(boxes, 2):
        if frozenset((a, b)) not in allowed and any(g.obb_overlap(x, y) for x in boxes[a] for y in boxes[b]):
            conflicts.append((a, b))
    assert not conflicts
    from fr3_dual_bolt_cell.world import table_boxes
    for link, entries in boxes.items():
        for table, size, center in table_boxes(load_scene(SHARE/'config/scene.yaml')):
            assert not any(g.obb_overlap(b, (np.array(center), np.eye(3), np.array(size)/2)) for b in entries), (link, table)


def test_world_has_one_bolt_and_explicit_assistance():
    world = ET.fromstring(inspection_world(world_xml(load_scene(SHARE/'config/scene.yaml')), config()))
    bolts = [m for m in world.findall('world/model') if m.get('name').startswith('bolt_')]
    assert len(bolts) == 1
    assert world.find("world/plugin[@name='inspection_grasp']/object_model").text == bolts[0].get('name')


def test_default_pick_and_handover_have_valid_fk_witnesses():
    from copy import deepcopy
    from fr3_dual_bolt_cell.world import table_boxes
    sys.path.insert(0, str(BASE/'test'))
    import test_geometry as g
    root, initial = robot()
    q = yaml.safe_load((SHARE/'config/pose_witnesses.yaml').read_text())
    local = g.local_collision_boxes(root)
    srdf = ET.fromstring(semantic(root, initial))
    allowed = {frozenset((e.get('link1'), e.get('link2'))) for e in srdf.findall('disable_collisions')}
    c = config()
    center = c['handover_center']
    scenarios = [
        ({'right': q['pick_right']}, {'right': transform([.5, -.2, .7245])@grasp_in_object(-.006)}),
        ({'right': q['above_right']}, {'right': transform([.5, -.2, .7745])@grasp_in_object(-.006)}),
        ({'right': q['handover_right'], 'left': q['handover_left']},
         {'right': transform(center)@grasp_in_object(-.006), 'left': transform(center)@grasp_in_object(.006, True)}),
        ({'right': q['handover_right'], 'left': q['handover_pre_left']},
         {'left': transform(np.array(center)+[0, 0, -.05])@grasp_in_object(.006, True)})]
    for values, targets in scenarios:
        arms = deepcopy(initial)
        for side, positions in values.items():
            arms[side]['initial'] = positions
            for i, value in enumerate(positions, 1):
                limit = root.find(f"joint[@name='{side}_j{i}']/limit")
                assert float(limit.get('lower')) < value < float(limit.get('upper'))
        frames = g.poses(root, arms)
        for side, target in targets.items():
            assert np.allclose(frames[side+'_gripper_tcp'], target, atol=2e-5)
        boxes = g.oriented_boxes(local, frames)
        for a, b in itertools.combinations(boxes, 2):
            if frozenset((a, b)) not in allowed:
                assert not any(g.obb_overlap(x, y) for x in boxes[a] for y in boxes[b]), (a, b)
        for link, entries in boxes.items():
            for table, size, pos in table_boxes(load_scene(SHARE/'config/scene.yaml')):
                assert not any(g.obb_overlap(b, (np.array(pos), np.eye(3), np.array(size)/2)) for b in entries), (link, table)


def test_all_python_parses():
    for path in SHARE.rglob('*.py'):
        ast.parse(path.read_text(encoding='utf-8'), filename=str(path))


@pytest.mark.parametrize('fail_stage', ['close', 'grasp', 'scene', 'settle', 'verify', None])
def test_handover_never_opens_donor_before_confirmation(fail_stage):
    from fr3_bolt_inspection_cell.handover import transfer
    events = []
    def event(stage):
        events.append(stage)
        if fail_stage == stage:
            raise RuntimeError('Injected '+stage+' failure')
    class FakeIO:
        def gripper(self, side, width):
            event('close' if side == 'left' else 'open_donor')
        def assisted_grasp(self, side):
            event('grasp')
        def object_scene(self, *args, **kwargs):
            event('scene')
    def run():
        transfer(FakeIO(), 'right', 'left', np.eye(4), .004, .035,
                 lambda: event('settle'), lambda obj: event('verify'))
    if fail_stage:
        with pytest.raises(RuntimeError):
            run()
        assert 'open_donor' not in events
    else:
        run()
        assert events == ['close', 'grasp', 'scene', 'settle', 'verify', 'open_donor']
