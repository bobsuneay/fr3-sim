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
    cfg = dict(open_width=.035, approach_height=.05, descent_speed=.008)
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
def test_retry_retreats_before_homing_and_preserves_target(recovery, side):
    io, arms, cfg, target = recovery
    original = target.copy()
    prepare_pick_retry(io, side, arms, cfg, target, MagicMock())
    method_names = [call[0] for call in io.mock_calls]
    assert method_names.index('wait_stationary') < method_names.index('gripper')
    assert method_names.index('cartesian') < method_names.index('global_move')
    above = io.cartesian.call_args.args[1][0]
    assert above[2, 3] == pytest.approx(.776)
    assert np.allclose(target, original)
    assert io.global_move.call_args_list[0].args == (side,)
    assert io.global_move.call_count == 2


def test_failed_retreat_does_not_try_homing(recovery):
    io, arms, cfg, target = recovery
    io.cartesian.side_effect = RuntimeError('Blocked retreat')
    with pytest.raises(RuntimeError, match='Blocked retreat'):
        prepare_pick_retry(io, 'right', arms, cfg, target, MagicMock())
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
    node.worker, node.output = None, Path('previous_run')
    node.report = {'events': [{'phase': 'FAILED', 'detail': 'old'}], 'views': ['old']}
    node.io, node.status_pub = MagicMock(), MagicMock()
    node.cloud, node.object_state = object(), object()
    node.get_logger = MagicMock()
    node.get_parameter = lambda name: NS(value='gazebo' if name == 'mode' else True)
    class Thread:
        def __init__(self, target, kwargs, daemon):
            self.target, self.kwargs, self.alive = target, kwargs, False
        def start(self):
            self.alive = True
        def is_alive(self):
            return self.alive
    monkeypatch.setattr(module.threading, 'Thread', Thread)
    return module, node


def response():
    return NS(success=False, message='')


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
    result = node.randomize_object(None, response())
    assert result.success
    world_object = node.io.relocate_object.call_args.args[0]
    assert np.linalg.norm(world_object[:2, 3]-[.5, -.2]) <= .05
    assert world_object[2, 3] == pytest.approx(.7265)
    assert node.cloud is node.object_state is node.pick_target is None
    assert not node.stop_event.is_set()


@pytest.mark.parametrize('busy,secured,owner', [(True, False, ''),
                                                (False, True, ''),
                                                (False, False, 'right')])
def test_randomize_rejects_busy_secured_or_owned_object(task, busy, secured, owner):
    _, node = task
    node.worker = NS(is_alive=lambda: busy)
    node.pick_secured = secured
    node.io.grasp_owner.return_value = owner
    result = node.randomize_object(None, response())
    assert not result.success
    node.io.relocate_object.assert_not_called()


def test_retry_runs_new_camera_and_detection_after_recovery(task, monkeypatch, tmp_path):
    module, node = task
    node.cfg = dict(output_directory=str(tmp_path), first_arm='right')
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
                    descent_speed=.008, grasp_test_lift=.015)
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
                    descent_speed=.008, grasp_test_lift=.015)
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


def test_failed_pick_cleanup_releases_attachment_before_opening(task):
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
    node.reset_failed_pick('right')
    node.io.assisted_grasp.assert_called_once_with('right', False)
    node.io.object_scene.assert_called_once_with(actual, previous='right')
    assert not node.pick_secured
