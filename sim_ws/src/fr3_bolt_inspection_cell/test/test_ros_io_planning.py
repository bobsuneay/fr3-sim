"""Exercise IO planning/execution decisions with stand-in ROS message types."""
import importlib.util
from pathlib import Path
import sys
from types import SimpleNamespace as NS
from unittest.mock import MagicMock
import numpy as np
import pytest

SHARE = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(SHARE))


@pytest.fixture
def adapter(monkeypatch):
    # ROS is unavailable on Windows. Only transport/message factories are
    # replaced; all IO planning, retry, retiming and execution decisions run.
    for name in ('rclpy', 'rclpy.action', 'rclpy.duration', 'rclpy.time',
                 'action_msgs.msg', 'builtin_interfaces.msg', 'control_msgs.action',
                 'geometry_msgs.msg', 'moveit_msgs.action', 'moveit_msgs.msg',
                 'moveit_msgs.srv', 'shape_msgs.msg', 'std_srvs.srv',
                 'gazebo_msgs.srv', 'trajectory_msgs.msg'):
        monkeypatch.setitem(sys.modules, name, MagicMock())
    spec = importlib.util.spec_from_file_location(
        'fr3_bolt_inspection_cell._test_ros_io', SHARE/'fr3_bolt_inspection_cell/ros_io.py')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    io = module.IO.__new__(module.IO)
    io.n = MagicMock()
    io.c = dict(joint_speed=.12, joint_acceleration=.15, cartesian_step=.003,
                joint_step_limit=.2, center_tolerance=.003)
    io.move, io.cart = object(), object()
    io.state = lambda: NS(joint_state=NS(name=['right_j1'], position=[0.0]))
    io.execute = MagicMock()
    return module, io


def trajectory(value=.1):
    return NS(joint_trajectory=NS(joint_names=['right_j1'], points=[
        NS(positions=[0.0]), NS(positions=[value])]))


def test_partial_cartesian_is_replaced_only_after_full_fallback(adapter):
    _, io = adapter
    partial, complete = trajectory(.02), trajectory(.1)
    io.call = lambda *args: NS(solution=partial, error_code=NS(val=1), fraction=4/9)
    io.seeded_cartesian = MagicMock(return_value=(complete, np.array([[0], [.1]]), [np.eye(4)]*2))
    io.cartesian('right', [np.eye(4)], .008)
    io.execute.assert_called_once_with(complete)
    assert io.watch_center is None
    assert complete.joint_trajectory.points[-1].velocities == [0.0]


def test_fallback_failure_never_executes_partial_path(adapter):
    module, io = adapter
    io.call = lambda *args: NS(solution=trajectory(), error_code=NS(val=1), fraction=4/9)
    io.seeded_cartesian = MagicMock(side_effect=module.PlanningFailure('table contact'))
    with pytest.raises(module.PlanningFailure, match='table contact'):
        io.cartesian('right', [np.eye(4)], .008)
    io.execute.assert_not_called()


def test_approach_checks_endpoint_and_retries_before_moving(adapter):
    module, io = adapter
    first, second = trajectory(.1), trajectory(.2)
    io.action = MagicMock(side_effect=[NS(error_code=NS(val=1), planned_trajectory=t)
                                      for t in (first, second)])
    states = []
    def preflight(side, state, target):
        io.execute.assert_not_called()
        states.append(state.joint_state.position[:])
        if len(states) == 1:
            raise module.PlanningFailure('IK branch blocked')
    io.seeded_cartesian = preflight
    io.global_move('right', target=np.eye(4), continuation=np.eye(4))
    assert states == [[.1], [.2]]
    io.execute.assert_called_once_with(second)


def test_all_approach_candidates_fail_without_motion(adapter):
    module, io = adapter
    io.action = MagicMock(return_value=NS(error_code=NS(val=1), planned_trajectory=trajectory()))
    io.seeded_cartesian = MagicMock(side_effect=module.PlanningFailure('collision'))
    with pytest.raises(module.PlanningFailure, match='exhausted 3'):
        io.global_move('right', target=np.eye(4), continuation=np.eye(4))
    assert io.action.call_count == 3
    io.execute.assert_not_called()


def test_diagnostic_ik_reports_collision_without_returning_a_path(adapter):
    module, io = adapter
    io.fk, io.ik, io.validity = object(), object(), object()
    names = [f'right_j{i}' for i in range(1, 7)]
    start = NS(joint_state=NS(name=names, position=[0.0]*6))
    attempts = []
    def call(client, request):
        if client is io.fk:
            return NS(error_code=NS(val=1), pose_stamped=[NS(pose=NS(
                position=NS(x=0.0, y=0.0, z=0.0), orientation=NS(x=0.0, y=0.0, z=0.0, w=1.0)))])
        if client is io.ik:
            attempts.append(request.ik_request.avoid_collisions)
            return NS(error_code=NS(val=-31 if attempts[-1] else 1),
                      solution=NS(joint_state=NS(name=names, position=[.01]*6)))
        if client is io.validity:
            return NS(valid=not any(request.robot_state.joint_state.position), contacts=[
                NS(contact_body_1='right_wrist2_link', contact_body_2='table_top')])
        raise AssertionError('Unexpected service')
    io.call = call
    target = np.eye(4)
    target[2, 3] = -.001
    with pytest.raises(module.PlanningFailure, match='right_wrist2_link <-> table_top'):
        io.seeded_cartesian('right', start, target)
    assert attempts == [True, False]
    io.execute.assert_not_called()
