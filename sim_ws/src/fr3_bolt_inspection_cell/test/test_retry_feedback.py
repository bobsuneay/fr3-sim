"""Recovery order, repeat admission, and measured telemetry regressions."""
import importlib.util
import json
from pathlib import Path
import sys
import threading
from types import SimpleNamespace as NS
from unittest.mock import MagicMock
import numpy as np
import pytest

SHARE = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(SHARE))
from fr3_bolt_inspection_cell.feedback import JointFeedback, arm_rows, gripper_text
from fr3_bolt_inspection_cell.retry import prepare_pick_retry


def test_partial_joint_messages_preserve_both_arms_and_expire_individually():
    store = JointFeedback()
    store.receive(NS(name=['left_j1'], position=[np.pi/2]), 1)
    store.receive(NS(name=['right_j1'], position=[-np.pi/2]), 2)
    row = arm_rows(store.snapshot(2))[0]
    assert row == ['J1', '1.5708', '90.00', '实时', '-1.5708', '-90.00', '实时']
    row = arm_rows(store.snapshot(3.5))[0]
    assert row[1:4] == ['--', '--', '无数据/过期']
    assert row[5] == '-90.00'
    store.receive(NS(name=['right_j1'], position=[float('nan')]), 3.6)
    assert store.snapshot(3.6) == {}


def test_gripper_gap_requires_both_fingers_and_does_not_imply_holding():
    joints = {'left_left_finger_joint': .0175, 'left_right_finger_joint': .0125}
    assert '30.00 mm' in gripper_text('left', joints, '')
    assert '未夹持' in gripper_text('left', joints, '')
    assert '夹持中' in gripper_text('left', joints, 'left')
    assert '未知/反馈过期' in gripper_text('left', joints, None)
    del joints['left_right_finger_joint']
    assert '开口宽度：--' in gripper_text('left', joints, '')


@pytest.fixture
def recovery():
    io = MagicMock()
    io.grasp_owner.return_value = ''
    actual = np.eye(4)
    actual[2, 3] = .726
    io.tcp_pose.return_value = actual
    arms = {s: {'initial': [0]*6} for s in ('left', 'right')}
    cfg = dict(open_width=.035, approach_height=.05, descent_speed=.008, retry_lift_height=.02)
    return io, arms, cfg, actual


@pytest.mark.parametrize('owner', ['left', 'right'])
def test_retry_never_opens_a_held_object(recovery, owner):
    io, arms, cfg, target = recovery
    io.grasp_owner.return_value = owner
    with pytest.raises(RuntimeError, match='already holds'):
        prepare_pick_retry(io, 'right', arms, cfg, target, MagicMock())
    io.gripper.assert_not_called()
    io.global_move.assert_not_called()


def test_retry_rechecks_ownership_after_motion_stops(recovery):
    io, arms, cfg, target = recovery
    io.grasp_owner.side_effect = ['', 'right']
    with pytest.raises(RuntimeError, match='acquired while waiting'):
        prepare_pick_retry(io, 'right', arms, cfg, target, MagicMock())
    io.wait_stationary.assert_called_once()
    io.gripper.assert_not_called()


@pytest.mark.parametrize('side', ['left', 'right'])
def test_retry_lifts_locally_without_homing_or_moving_other_arm(recovery, side):
    io, arms, cfg, target = recovery
    original = target.copy()
    prepare_pick_retry(io, side, arms, cfg, target, MagicMock())
    method_names = [call[0] for call in io.mock_calls]
    assert method_names.index('wait_stationary') < method_names.index('gripper')
    assert method_names.index('gripper') < method_names.index('cartesian')
    above = io.cartesian.call_args.args[1][0]
    assert above[2, 3] == pytest.approx(.746)
    assert np.allclose(above[:2, 3], original[:2, 3])
    assert np.allclose(above[:3, :3], original[:3, :3])
    assert np.allclose(target, original)
    io.global_move.assert_not_called()
    io.gripper.assert_called_once_with(side, cfg['open_width'])


def test_failed_retreat_does_not_try_homing(recovery):
    io, arms, cfg, target = recovery
    io.cartesian.side_effect = RuntimeError('Blocked retreat')
    with pytest.raises(RuntimeError, match='Blocked retreat'):
        prepare_pick_retry(io, 'right', arms, cfg, target, MagicMock())
    io.global_move.assert_not_called()


@pytest.mark.parametrize('old_z', [None, .60, 1.1])
def test_local_retry_uses_measured_pose_not_old_pick_height(recovery, old_z):
    io, arms, cfg, actual = recovery
    target = None if old_z is None else np.eye(4)
    if target is not None:
        target[2, 3] = old_z
    prepare_pick_retry(io, 'right', arms, cfg, target, MagicMock())
    assert io.cartesian.call_args.args[1][0][2, 3] == pytest.approx(actual[2, 3]+.02)
    io.global_move.assert_not_called()


@pytest.fixture
def task(monkeypatch):
    for name in ('rclpy', 'rclpy.executors', 'rclpy.qos', 'rclpy.time',
                 'sensor_msgs.msg', 'sensor_msgs_py', 'gazebo_msgs.msg',
                 'std_msgs.msg', 'std_srvs.srv', 'tf2_ros'):
        monkeypatch.setitem(sys.modules, name, MagicMock())
    monkeypatch.setitem(sys.modules, 'rclpy.node', NS(Node=object))
    monkeypatch.setitem(sys.modules, 'fr3_bolt_inspection_cell.ros_io',
                        NS(IO=MagicMock(), PlanningFailure=RuntimeError, matrix=MagicMock()))
    spec = importlib.util.spec_from_file_location('fr3_bolt_inspection_cell._test_task',
                                                 SHARE/'fr3_bolt_inspection_cell/task_node.py')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    node = module.Inspection.__new__(module.Inspection)
    node.run_lock, node.data_lock, node.stop_event = threading.Lock(), threading.Lock(), threading.Event()
    node.phase, node.pick_secured, node.pick_target = 'FAILED', False, np.eye(4)
    node.handover_context = None
    node.handover_requested = threading.Event()
    node.scan_speed_scale = 1.0
    node.worker, node.output = None, Path('previous_run')
    node.report = {'events': [{'phase': 'FAILED', 'detail': 'old'}], 'views': ['old']}
    node.io, node.status_pub = MagicMock(), MagicMock()
    node.io.grasp_owner.return_value = ''
    node.cloud, node.object_state = object(), object()
    node.get_logger = MagicMock()
    node.fingertips = {side: [dict(joint=f'{side}_{finger}_finger_joint',
        vertices_tcp=[[0, 0, -.0023]], axis_tcp=[1, 0, 0])
        for finger in ('left', 'right')] for side in ('left', 'right')}
    node.check_fingertip_clearance = MagicMock()
    node.get_parameter = lambda name: NS(value='gazebo' if name == 'mode' else True)
    class Thread:
        def __init__(self, target, kwargs=None, daemon=True):
            self.target, self.kwargs, self.alive = target, kwargs, False
        def start(self):
            self.alive = True
        def is_alive(self):
            return self.alive
    monkeypatch.setattr(module.threading, 'Thread', Thread)
    return module, node


def response():
    return NS(success=False, message='')


@pytest.mark.parametrize('result', ['second', 'none', 'execution_error'])
def test_perpendicular_candidates_preflight_before_any_motion(task, result):
    module, node = task
    from fr3_bolt_inspection_cell.handover import approach_receiver
    from fr3_bolt_inspection_cell.core import grasp_in_object, perpendicular_receiver_grasp
    donor = grasp_in_object(-.01)
    receiver = perpendicular_receiver_grasp(.01, donor)
    candidates = []
    def plan(side, pre, target, plan_only):
        assert plan_only
        node.io.execute.assert_not_called()
        assert abs(target[:3, 0]@donor[:3, 0]) < 1e-12
        assert abs(target[:3, 2]@donor[:3, 2]) < 1e-12
        assert np.linalg.norm(pre[:3, 3]-target[:3, 3]) == pytest.approx(.05)
        candidates.append(target.copy())
        if result == 'none' or len(candidates) == 1:
            raise module.PlanningFailure('unreachable side')
        return 'connection', 'prepared'
    node.io.pick_approach.side_effect = plan
    if result == 'execution_error':
        node.io.execute.side_effect = RuntimeError('controller stopped')
    def run():
        return approach_receiver(node.io, 'left', np.eye(4), receiver, .05, .008, MagicMock())
    if result == 'none':
        with pytest.raises(module.PlanningFailure, match='All 4'):
            run()
        assert len(candidates) == 4
        node.io.execute.assert_not_called()
    elif result == 'execution_error':
        with pytest.raises(RuntimeError, match='controller stopped'):
            run()
        assert len(candidates) == 2
        node.io.execute_prepared_cartesian.assert_not_called()
    else:
        grasp, target = run()
        assert np.allclose(target, candidates[1])
        assert np.allclose(grasp, target)
        assert np.dot(candidates[0][:3, 2], candidates[1][:3, 2]) == pytest.approx(-1)
        node.io.execute_prepared_cartesian.assert_called_once_with('prepared', .008)


@pytest.mark.parametrize('scale,expected', [(.25, .25), (1.5, 1.5), (2., 2.),
    (0, 1.), (-1., 1.), (2.01, 1.), (float('nan'), 1.), (float('inf'), 1.)])
def test_scan_slider_bounds_and_status_readback(task, scale, expected):
    _, node = task
    node.set_scan_speed(NS(data=scale))
    assert node.scan_speed_scale == expected
    assert json.loads(node.status(None, response()).message)['scan_speed_scale'] == expected
    node.io.cancel.assert_not_called()


def test_handover_during_scan_queues_without_parallel_motion(task):
    _, node = task
    node.phase = 'INSPECT_RIGHT'
    node.handover_context = ('right', 'left', np.eye(4), np.eye(4), np.eye(4))
    worker = NS(is_alive=lambda: True)
    node.worker = worker
    assert node.skip_to_handover(None, response()).success
    assert node.handover_requested.is_set()
    assert node.worker is worker
    assert not node.skip_to_handover(None, response()).success
    node.io.cancel.assert_not_called()
    node.io.global_move.assert_not_called()


@pytest.mark.parametrize('phase', ['GRASP', 'HANDOVER_CONFIRM', 'INSPECT_LEFT', 'DONE_HOLDING_LEFT'])
def test_handover_rejects_other_stages(task, phase):
    _, node = task
    node.phase = phase
    node.handover_context = ('right', 'left', np.eye(4), np.eye(4), np.eye(4))
    assert not node.skip_to_handover(None, response()).success


def test_resume_handover_checks_owner_before_moving(task):
    _, node = task
    node.output = None
    node.handover_context = ('right', 'left', np.eye(4), np.eye(4), np.eye(4))
    assert node.skip_to_handover(None, response()).success
    node.worker.target()
    node.io.wait_stationary.assert_called_once()
    node.io.global_move.assert_not_called()
    node.io.gripper.assert_not_called()
    assert node.phase == 'FAILED'
    assert 'ownership' in node.report['events'][-1]['detail']
    assert not node.handover_requested.is_set()


def test_skip_scan_bypasses_minimum_views_without_any_motion(task):
    _, node = task
    node.output = None
    node.cfg = dict(views_deg=[[0, 0, 0]], minimum_views=3)
    node.handover_requested.set()
    node.scan('right', np.eye(4), np.eye(4))
    node.io.cartesian.assert_not_called()


@pytest.mark.parametrize('follow_ok', [False, True])
def test_handover_requires_receiver_motion_follow_before_scan(task, monkeypatch, follow_ok):
    module, node = task
    node.publish = MagicMock()
    node.report = {'views': []}
    node.cfg = dict(handover_center=[.25, 0, 1.1], approach_height=.05,
                    descent_speed=.008, transfer_speed=.02, close_width=.004, open_width=.035)
    node.arms = {'right': {'initial': [0]*6}}
    node.object_truth = np.eye(4)
    node.io.truth.return_value = np.eye(4)
    node.io.grasp_owner.side_effect = ['right', 'left']
    target = np.eye(4)
    target[:3, 3] = node.cfg['handover_center']
    node.io.tcp_pose.return_value = target.copy()
    node.settle, node.scan = MagicMock(), MagicMock()
    monkeypatch.setattr(module, 'transfer', MagicMock())
    monkeypatch.setattr(module, 'approach_receiver', MagicMock(return_value=(np.eye(4), target)))
    node.verify = MagicMock(side_effect=[0, 0 if follow_ok else RuntimeError('part did not follow')])
    if follow_ok:
        node.finish_handover('right', 'left', target, np.eye(4), np.eye(4))
        node.scan.assert_called_once()
    else:
        with pytest.raises(RuntimeError, match='did not follow'):
            node.finish_handover('right', 'left', target, np.eye(4), np.eye(4))
        node.scan.assert_not_called()
    probe = node.verify.call_args.args[0]
    assert probe[2, 3] == pytest.approx(1.11)
    calls = node.io.mock_calls
    home = next(i for i, call in enumerate(calls) if call[0] == 'global_move' and 'joints' in call.kwargs)
    assert calls[home+1][0] == 'cartesian'
    assert calls[home+1].args[0] == 'left'


def test_humble_mimic_feedback_preserves_measured_values_and_age(task):
    _, node = task
    node.joints = {}
    msg = NS(name=['left_left_finger_joint', 'left_right_finger_joint_mimic'],
             position=[.0175, .016])
    node.on_joints(msg)
    assert node.joints['left_right_finger_joint'][0] == .016
    store = JointFeedback()
    store.receive(msg, 1.)
    assert store.snapshot(2.)['left_right_finger_joint'] == .016
    assert store.snapshot(4.) == {}


def test_retry_can_run_repeatedly_but_never_concurrently(task):
    _, node = task
    result = node.retry_pick(None, response())
    assert result.success and node.worker.kwargs == {'retry': True}
    assert node.report['retry_of'] == 'previous_run'
    assert node.report['views'] == [] and len(node.report['events']) == 1
    assert node.output is None
    assert not node.retry_pick(None, response()).success
    node.worker.alive, node.phase = False, 'FAILED'
    assert node.retry_pick(None, response()).success


@pytest.mark.parametrize('phase,secured', [('IDLE', False), ('GRASP', False),
                                         ('FAILED', True), ('DONE_HOLDING_LEFT', True)])
def test_retry_rejects_wrong_phase_or_secured_pick(task, phase, secured):
    _, node = task
    node.phase, node.pick_secured = phase, secured
    assert not node.retry_pick(None, response()).success
    assert node.worker is None


def test_status_enables_retry_only_after_worker_exits(task):
    _, node = task
    node.worker = NS(is_alive=lambda: True)
    assert not json.loads(node.status(None, response()).message)['can_retry']
    node.worker = NS(is_alive=lambda: False)
    assert json.loads(node.status(None, response()).message)['can_retry']
    node.pick_secured = True
    assert not json.loads(node.status(None, response()).message)['can_retry']
    assert not json.loads(node.status(None, response()).message)['can_randomize']


def test_randomize_object_moves_within_disk_and_clears_old_detection(task):
    _, node = task
    node.phase, node.output = 'FAILED', None
    node.stop_event.set()
    node.cfg = dict(random_position_center=[.5, -.2], random_position_radius=.05,
                    table_z=.72, head_radius=.006)
    node.io.grasp_owner.return_value = ''
    node.io.relocate_object.side_effect = lambda value: value.copy()
    result = node.randomize_object(None, response())
    assert result.success
    assert node.phase == 'RANDOMIZING'
    # Returning from the service must not wait for another ROS service callback.
    node.io.grasp_owner.assert_not_called()
    node.io.relocate_object.assert_not_called()
    state = json.loads(node.status(None, response()).message)
    assert state['busy'] and not state['can_start'] and not state['can_retry']
    assert not state['can_randomize']
    assert not node.start(None, response()).success
    assert not node.randomize_object(None, response()).success
    node.worker.target(**node.worker.kwargs)
    node.worker.alive = False
    world_object = node.io.relocate_object.call_args.args[0]
    assert np.linalg.norm(world_object[:2, 3]-[.5, -.2]) <= .05
    assert world_object[2, 3] == pytest.approx(.7265)
    assert node.cloud is node.object_state is node.pick_target is None
    assert not node.stop_event.is_set()
    assert node.phase == 'FAILED'  # Preserve the original pick-retry opportunity.
    state = json.loads(node.status(None, response()).message)
    assert not state['busy'] and state['can_retry'] and state['can_randomize']
    assert 'randomized and verified' in state['detail']


@pytest.mark.parametrize('busy,secured', [(True, False), (False, True)])
def test_randomize_rejects_busy_or_secured_object(task, busy, secured):
    _, node = task
    node.worker = NS(is_alive=lambda: busy)
    node.pick_secured = secured
    result = node.randomize_object(None, response())
    assert not result.success
    node.io.relocate_object.assert_not_called()


@pytest.mark.parametrize('owners', [['right'], ['', 'right']])
def test_randomize_worker_refuses_owned_object_before_and_after_wait(task, owners):
    _, node = task
    node.output = None
    node.io.grasp_owner.side_effect = owners
    assert node.randomize_object(None, response()).success
    node.worker.target(**node.worker.kwargs)
    node.worker.alive = False
    node.io.relocate_object.assert_not_called()
    assert node.phase == 'FAILED'
    assert 'position unchanged' in node.report['events'][-1]['detail']


def test_randomize_verification_failure_is_reported_and_invalidates_old_pose(task):
    _, node = task
    node.phase, node.output = 'IDLE', None
    node.cfg = dict(random_position_center=[.5, -.2], random_position_radius=.05,
                    table_z=.72, head_radius=.006)
    node.io.grasp_owner.return_value = ''
    node.io.relocate_object.side_effect = TimeoutError('No position confirmation')
    assert node.randomize_object(None, response()).success
    node.worker.target(**node.worker.kwargs)
    node.worker.alive = False
    assert node.cloud is node.object_state is node.pick_target is None
    state = json.loads(node.status(None, response()).message)
    assert state['phase'] == 'FAILED' and not state['busy']
    assert 'Randomization failed: No position confirmation' in state['detail']


def test_randomize_success_restores_start_and_reports_verified_position(task):
    _, node = task
    node.phase, node.output = 'IDLE', None
    node.cfg = dict(random_position_center=[.5, -.2], random_position_radius=.05,
                    table_z=.72, head_radius=.006)
    node.io.grasp_owner.return_value = ''
    actual = np.eye(4)
    actual[:2, 3] = [.501, -.201]
    node.io.relocate_object.return_value = actual
    assert node.randomize_object(None, response()).success
    node.worker.target(**node.worker.kwargs)
    node.worker.alive = False
    state = json.loads(node.status(None, response()).message)
    assert state['phase'] == 'IDLE' and state['can_start']
    assert 'x=0.5010, y=-0.2010' in state['detail']


def test_randomize_stop_before_worker_runs_does_not_move_object(task):
    _, node = task
    node.output = None
    assert node.randomize_object(None, response()).success
    assert node.stop(None, response()).success
    node.io.check.side_effect = RuntimeError('Stop requested')
    node.worker.target(**node.worker.kwargs)
    node.worker.alive = False
    node.io.relocate_object.assert_not_called()
    assert node.phase == 'STOPPED'


def test_retry_runs_new_camera_and_detection_after_recovery(task, monkeypatch, tmp_path):
    module, node = task
    node.cfg = dict(output_directory=str(tmp_path), first_arm='right', retry_lift_height=.02)
    node.arms, node.settle = {}, MagicMock()
    node.io.grasp_owner.return_value = ''
    calls = []
    monkeypatch.setattr(module, 'prepare_pick_retry', lambda *args: calls.append('recovery'))
    node.capture = lambda label: calls.append(label)
    def acquire(*_):
        calls.append('new_detection')
        raise RuntimeError('No new cloud')
    node.acquire_initial_pick = acquire
    node.run(retry=True)
    assert calls == ['recovery', 'camera_check', 'new_detection']
    assert node.phase == 'FAILED'
    assert (node.output/'report.json').is_file()


def test_automatic_pick_retries_until_test_lift_is_confirmed(task):
    _, node = task
    node.output = None
    node.cfg = {'max_grasp_attempts': 5}
    confirmed = (np.eye(4), np.eye(4), np.eye(4), np.eye(4))
    node.pick_once = MagicMock(side_effect=[RuntimeError('miss one'),
                                            RuntimeError('miss two'), confirmed])
    node.reset_failed_pick = MagicMock()
    assert node.acquire_initial_pick('right', 'left') is confirmed
    assert node.pick_once.call_count == 3
    assert node.reset_failed_pick.call_count == 2
    assert node.report['pick_attempts'] == [
        {'attempt': 1, 'status': 'failed', 'reason': 'miss one'},
        {'attempt': 2, 'status': 'failed', 'reason': 'miss two'},
        {'attempt': 3, 'status': 'confirmed'}]


def test_automatic_pick_limit_cleans_last_failure_then_stops(task):
    _, node = task
    node.output = None
    node.cfg = {'max_grasp_attempts': 2}
    node.pick_once = MagicMock(side_effect=RuntimeError('no object follow'))
    node.reset_failed_pick = MagicMock()
    with pytest.raises(RuntimeError, match='not confirmed after 2'):
        node.acquire_initial_pick('right', 'left')
    assert node.pick_once.call_count == node.reset_failed_pick.call_count == 2


def test_stop_during_pick_preserves_grasp_and_does_not_auto_release(task):
    _, node = task
    node.output = None
    node.cfg = {'max_grasp_attempts': 5}
    node.stop_event.set()
    node.pick_once = MagicMock(side_effect=RuntimeError('Stopped'))
    node.reset_failed_pick = MagicMock()
    with pytest.raises(RuntimeError, match='Stopped'):
        node.acquire_initial_pick('right', 'left')
    node.reset_failed_pick.assert_not_called()


def test_pick_is_confirmed_only_after_object_follows_test_lift(task):
    _, node = task
    node.output = None
    node.phase = 'IDLE'
    node.report = {'events': [], 'views': []}
    node.cfg = dict(max_grasp_attempts=5, grasp_offset=.01, grasp_depth_offset=.002,
                    open_width=.035, close_width=.004, approach_height=.05,
                    descent_speed=.008, grasp_test_lift=.015,
                    table_z=.720, fingertip_table_clearance=.005,
                    bolt_length=.045, head_length=.008, head_radius=.009)
    obj = np.eye(4)
    obj[:3, 3] = [.5, -.2, .724]
    node.perceive = MagicMock(return_value=NS(pose=obj))
    node.io.truth.return_value = obj.copy()
    node.io.grasp_owner.return_value = 'right'
    node.io.global_move.return_value = object()
    node.settle = MagicMock()
    node.verify = MagicMock(return_value=.001)
    result = node.pick_once('right', 'left', 1)
    assert node.pick_secured
    test_pose = result[3]
    assert test_pose[2, 3] == pytest.approx(.739)
    node.verify.assert_called_once_with(test_pose)
    node.io.assisted_grasp.assert_called_once_with('right')
    assert node.io.object_scene.call_args_list[-1].args == (obj, 'right')
    lift_target = node.io.cartesian.call_args.args[1][0]
    assert np.allclose(lift_target, test_pose@result[1])


def test_failed_test_lift_keeps_safety_latch_until_cleanup(task):
    _, node = task
    node.output = None
    node.phase = 'IDLE'
    node.report = {'events': [], 'views': []}
    node.cfg = dict(max_grasp_attempts=5, grasp_offset=.01, grasp_depth_offset=.002,
                    open_width=.035, close_width=.004, approach_height=.05,
                    descent_speed=.008, grasp_test_lift=.015,
                    table_z=.720, fingertip_table_clearance=.005,
                    bolt_length=.045, head_length=.008, head_radius=.009)
    obj = np.eye(4)
    obj[:3, 3] = [.5, -.2, .724]
    node.perceive = MagicMock(return_value=NS(pose=obj))
    node.io.truth.return_value = obj.copy()
    node.io.global_move.return_value = object()
    node.settle = MagicMock()
    node.verify = MagicMock(side_effect=RuntimeError('object stayed on table'))
    with pytest.raises(RuntimeError, match='stayed on table'):
        node.pick_once('right', 'left', 1)
    assert node.pick_secured


def test_failed_pick_cleanup_preserves_acquired_object(task):
    _, node = task
    node.output = None
    node.cfg = dict(open_width=.035, approach_height=.05, descent_speed=.008)
    node.arms = {s: {'initial': [0]*6} for s in ('left', 'right')}
    node.io.grasp_owner.side_effect = ['right', '', '']
    actual = np.eye(4)
    node.io.truth.return_value = actual
    node.pick_secured = True
    node.pick_target = None
    node.settle = MagicMock()
    with pytest.raises(RuntimeError, match='no automatic release'):
        node.reset_failed_pick('right')
    node.io.assisted_grasp.assert_not_called()
    node.io.gripper.assert_not_called()
    node.io.object_scene.assert_not_called()
    assert node.pick_secured


@pytest.mark.parametrize('owner,latch', [('right', True), ('right', False), ('', True)])
def test_post_grasp_failure_never_enters_open_and_retry(task, owner, latch):
    _, node = task
    node.output = None
    node.cfg = {'max_grasp_attempts': 5}
    node.io.grasp_owner.return_value = owner
    node.pick_secured = latch
    node.pick_once = MagicMock(side_effect=RuntimeError('lift planning failed'))
    node.reset_failed_pick = MagicMock()
    with pytest.raises(RuntimeError, match='Grasp retained'):
        node.acquire_initial_pick('right', 'left')
    node.reset_failed_pick.assert_not_called()
    assert node.pick_once.call_count == 1
    assert node.pick_secured
    assert node.report['pick_attempts'][-1]['status'] == 'holding_interrupted'


def test_unknown_ownership_never_opens_jaws(task):
    _, node = task
    node.output = None
    node.cfg = {'max_grasp_attempts': 5}
    node.io.grasp_owner.side_effect = RuntimeError('service timeout')
    node.pick_once = MagicMock(side_effect=RuntimeError('attach response timeout'))
    node.reset_failed_pick = MagicMock()
    with pytest.raises(RuntimeError, match='ownership unknown'):
        node.acquire_initial_pick('right', 'left')
    node.reset_failed_pick.assert_not_called()
    assert node.pick_secured
