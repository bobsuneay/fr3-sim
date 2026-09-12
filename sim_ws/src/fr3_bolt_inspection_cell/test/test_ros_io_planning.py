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
                 'gazebo_msgs.msg', 'gazebo_msgs.srv', 'trajectory_msgs.msg'):
        monkeypatch.setitem(sys.modules, name, MagicMock())
    spec = importlib.util.spec_from_file_location(
        'fr3_bolt_inspection_cell._test_ros_io', SHARE/'fr3_bolt_inspection_cell/ros_io.py')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    module.JointTrajectoryPoint = NS
    module.Duration = NS
    io = module.IO.__new__(module.IO)
    io.n = MagicMock()
    io.c = dict(joint_speed=.12, joint_acceleration=.15, cartesian_step=.003,
                joint_step_limit=.2, center_tolerance=.003, angular_tolerance=.12,
                grasp_reach_tolerance=.003, grasp_recovery_max=.025,
                shaft_radius=.004, close_width=.004)
    io.move, io.cart = object(), object()
    io.state = lambda: NS(joint_state=NS(name=['right_j1'], position=[0.0]))
    io.execute = MagicMock()
    return module, io


def trajectory(value=.1):
    return NS(joint_trajectory=NS(joint_names=['right_j1'], points=[
        NS(positions=[0.0]), NS(positions=[value])]))


@pytest.mark.parametrize('side', ['left', 'right'])
def test_gripper_commands_only_master_and_checks_follower(adapter, side):
    module, io = adapter
    module.FollowJointTrajectory.Goal = lambda: NS(trajectory=NS())
    io.fingers = {side: object()}
    io.action = MagicMock(return_value=NS(error_code=0))
    positions = [.0175, .0175]
    io.state = lambda: NS(joint_state=NS(
        name=[side+'_left_finger_joint', side+'_right_finger_joint'], position=positions))
    io.gripper(side, .035)
    goal = io.action.call_args.args[1]
    assert goal.trajectory.joint_names == [side+'_left_finger_joint']
    assert goal.trajectory.points[0].positions == [.0175]
    positions[1] = .0163
    with pytest.raises(RuntimeError, match='Linked gripper'):
        io.gripper(side, .035)


@pytest.mark.parametrize('positions,accepted', [([.00505, .00505], True),
    ([.0034, .0034], True),
    ([.0175, .0175], False), ([.00505, .007], False)])
def test_close_accepts_shaft_contact_but_not_still_open(adapter, positions, accepted):
    module, io = adapter
    module.FollowJointTrajectory.Goal = lambda: NS(trajectory=NS())
    io.fingers = {'right': object()}
    io.action = MagicMock(return_value=NS(error_code=0))
    io.state = lambda: NS(joint_state=NS(name=['right_left_finger_joint',
        'right_right_finger_joint'], position=positions))
    if accepted:
        io.gripper('right', .004)
    else:
        with pytest.raises(RuntimeError):
            io.gripper('right', .004)


def test_partial_cartesian_is_replaced_only_after_full_fallback(adapter):
    _, io = adapter
    partial, complete = trajectory(.02), trajectory(.1)
    io.call = lambda *args: NS(solution=partial, error_code=NS(val=1), fraction=4/9)
    io.seeded_cartesian = MagicMock(return_value=(complete, np.array([[0], [.1]]), [np.eye(4)]*2))
    io.cartesian('right', [np.eye(4)], .008)
    io.execute.assert_called_once_with(complete)
    assert io.watch_center is None
    assert complete.joint_trajectory.points[-1].velocities == [0.0]


def test_scan_speed_changes_only_centered_rotation(adapter):
    _, io = adapter
    io.c.update(scan_joint_speed=.24, scan_joint_acceleration=.60)
    q = np.array([[0.0], [.1]])
    normal, scan = trajectory(), trajectory()
    io.run_cartesian_trajectory(normal, q, [np.eye(4)]*2, .03)
    io.run_cartesian_trajectory(scan, q, [np.eye(4)]*2, .03, np.eye(4), np.zeros(3))
    def seconds(t):
        value = t.joint_trajectory.points[-1].time_from_start
        start = t.joint_trajectory.points[0].time_from_start
        return value.sec + value.nanosec*1e-9 - start.sec - start.nanosec*1e-9
    assert seconds(scan) == pytest.approx(seconds(normal)/2, abs=1e-8)


def test_fallback_failure_never_executes_partial_path(adapter):
    module, io = adapter
    io.call = lambda *args: NS(solution=trajectory(), error_code=NS(val=1), fraction=4/9)
    io.seeded_cartesian = MagicMock(side_effect=module.PlanningFailure('table contact'))
    with pytest.raises(module.PlanningFailure, match='table contact'):
        io.cartesian('right', [np.eye(4)], .008)
    io.execute.assert_not_called()


@pytest.mark.parametrize('side', ['left', 'right'])
def test_single_point_at_measured_target_needs_no_execution(adapter, side):
    _, io = adapter
    single = trajectory()
    single.joint_trajectory.points = single.joint_trajectory.points[:1]
    io.call = lambda *args: NS(solution=single, error_code=NS(val=1), fraction=1.0)
    io.tcp_pose = MagicMock(return_value=np.eye(4))
    io.cartesian(side, [np.eye(4)], .008, np.eye(4), np.zeros(3))
    io.tcp_pose.assert_called_once_with(side)
    io.execute.assert_not_called()


@pytest.mark.parametrize('failure', ['translation', 'rotation', 'loop', 'center', 'empty'])
def test_stationary_path_cannot_hide_missing_motion_or_drift(adapter, failure):
    module, io = adapter
    single = trajectory()
    single.joint_trajectory.points = single.joint_trajectory.points[:0 if failure == 'empty' else 1]
    io.call = lambda *args: NS(solution=single, error_code=NS(val=1), fraction=1.0)
    io.tcp_pose = MagicMock(return_value=np.eye(4))
    target = np.eye(4)
    center = np.zeros(3)
    if failure in ('translation', 'loop'):
        target[0, 3] = .01
    if failure == 'rotation':
        target[:3, :3] = module.Rotation.from_euler('x', .1).as_matrix()
    if failure == 'center':
        center[0] = .01
    waypoints = [target, np.eye(4)] if failure == 'loop' else [target]
    with pytest.raises(module.PlanningFailure):
        io.cartesian('right', waypoints, .008, np.eye(4), center)
    io.execute.assert_not_called()


def test_grasp_first_retries_and_reverses_verified_path_before_connection(adapter):
    module, io = adapter
    names = [f'right_j{i}' for i in range(1, 7)]
    io.state = lambda: NS(joint_state=NS(name=names, position=[.5]*6))
    io.ik = object()
    io.call = MagicMock(side_effect=[NS(error_code=NS(val=1),
        solution=NS(joint_state=NS(name=names, position=[v]*6))) for v in (.1, .3)])
    io.validate_robot_state = MagicMock()
    connection = NS(joint_trajectory=NS(joint_names=names,
                                        points=[NS(positions=[.5]*6), NS(positions=[.4]*6)]))
    io.action = MagicMock(return_value=NS(error_code=NS(val=1), planned_trajectory=connection))
    states = []
    def preflight(side, state, target):
        io.execute.assert_not_called()
        states.append(state.joint_state.position[:])
        if len(states) == 1:
            raise module.PlanningFailure('IK branch blocked')
        assert target[2, 3] == pytest.approx(.776)
        return connection, np.array([[.3]*6, [.4]*6]), [np.eye(4)]*2
    io.seeded_cartesian = preflight
    above = np.eye(4)
    above[2, 3] = .776
    selected = io.global_move('right', target=above, continuation=np.eye(4))
    assert states == [[.1]*6, [.3]*6]
    io.execute.assert_called_once_with(connection)
    assert selected.start.joint_state.position == [.4]*6
    assert np.allclose(selected.positions, [[.4]*6, [.3]*6])
    assert [p.positions for p in selected.trajectory.joint_trajectory.points] == [[.4]*6, [.3]*6]


def test_all_approach_candidates_fail_without_motion(adapter):
    module, io = adapter
    names = [f'right_j{i}' for i in range(1, 7)]
    io.state = lambda: NS(joint_state=NS(name=names, position=[.5]*6))
    io.ik = object()
    io.call = MagicMock(return_value=NS(error_code=NS(val=-31)))
    io.action = MagicMock()
    with pytest.raises(module.PlanningFailure, match='exhausted 8'):
        io.global_move('right', target=np.eye(4), continuation=np.eye(4))
    assert io.call.call_count == 8
    io.action.assert_not_called()
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
            assert request.ik_request.timeout.sec == 1
            assert request.ik_request.timeout.nanosec == 0
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
    assert attempts == [True, True, False]
    io.execute.assert_not_called()


def prepared_plan(module):
    start = NS(joint_state=NS(name=['right_j1', 'left_j1', 'right_left_finger_joint'],
                             position=[.2, 0.0, .0175]))
    selected = trajectory(.3)
    selected.joint_trajectory.points[0].positions = [.2]
    return module.PreparedCartesian('right', start, selected, np.array([[.2], [.3]]), [np.eye(4)]*2)


def test_prepared_descent_executes_selected_solution_without_replanning(adapter):
    from copy import deepcopy
    module, io = adapter
    selected = prepared_plan(module)
    io.state = lambda: deepcopy(selected.start)
    io.validate_robot_state = MagicMock()
    io.call = MagicMock(side_effect=AssertionError('Must not call IK or Cartesian planner again'))
    io.execute_prepared_cartesian(selected, .008)
    io.execute.assert_called_once_with(selected.trajectory)
    io.call.assert_not_called()
    states = [call.args[0].joint_state.position for call in io.validate_robot_state.call_args_list]
    assert states == [[.2, 0, .0175], [.2, 0, .0175], [.3, 0, .0175]]
    assert selected.trajectory.joint_trajectory.points[-1].velocities == [0.0]


@pytest.mark.parametrize('joint_index,drift', [(0, .02), (1, .02), (2, .002)])
def test_prepared_descent_rejects_arm_or_finger_drift(adapter, joint_index, drift):
    from copy import deepcopy
    module, io = adapter
    selected = prepared_plan(module)
    actual = deepcopy(selected.start)
    actual.joint_state.position[joint_index] += drift
    io.state = lambda: actual
    with pytest.raises(module.PlanningFailure, match='start changed'):
        io.execute_prepared_cartesian(selected, .008)
    io.execute.assert_not_called()


def test_prepared_descent_rechecks_scene_after_approach(adapter):
    from copy import deepcopy
    module, io = adapter
    selected = prepared_plan(module)
    io.state = lambda: deepcopy(selected.start)
    io.validate_robot_state = MagicMock(side_effect=[None, None,
        module.CartesianPlanningError('right_wrist2_link <-> new_obstacle')])
    with pytest.raises(module.PlanningFailure, match='scene changed.*new_obstacle'):
        io.execute_prepared_cartesian(selected, .008)
    io.execute.assert_not_called()


def test_grasp_first_execution_failure_never_tries_another_candidate(adapter):
    module, io = adapter
    names = [f'right_j{i}' for i in range(1, 7)]
    io.state = lambda: NS(joint_state=NS(name=names, position=[.5]*6))
    io.ik = object()
    requests = []
    def call(client, request):
        requests.append(request)
        assert request.ik_request.timeout.sec == 1
        assert request.ik_request.timeout.nanosec == 0
        assert request.ik_request.avoid_collisions is True
        return NS(error_code=NS(val=1), solution=NS(joint_state=NS(name=names, position=[.3]*6)))
    io.call = call
    io.validate_robot_state = MagicMock()
    connection = NS(joint_trajectory=NS(joint_names=names, points=[NS(positions=[.4]*6)]))
    io.action = MagicMock(return_value=NS(error_code=NS(val=1), planned_trajectory=connection))
    io.seeded_cartesian = MagicMock(return_value=(connection, np.array([[.3]*6, [.4]*6]), [np.eye(4)]*2))
    io.execute.side_effect = RuntimeError('Controller execution aborted')
    with pytest.raises(RuntimeError, match='Controller execution aborted'):
        io.global_move('right', target=np.eye(4), continuation=np.eye(4))
    assert len(requests) == 1
    io.execute.assert_called_once()


def test_valid_but_distant_diagnostic_ik_is_reported_and_not_executed(adapter):
    module, io = adapter
    io.fk, io.ik = object(), object()
    io.validate_robot_state = MagicMock()
    names = [f'right_j{i}' for i in range(1, 7)]
    start = NS(joint_state=NS(name=names, position=[0.0]*6))
    def call(client, request):
        if client is io.fk:
            return NS(error_code=NS(val=1), pose_stamped=[NS(pose=NS(
                position=NS(x=0.0, y=0.0, z=0.0), orientation=NS(x=0.0, y=0.0, z=0.0, w=1.0)))])
        return NS(error_code=NS(val=-31 if request.ik_request.avoid_collisions else 1),
                  solution=NS(joint_state=NS(name=names, position=[.5]*6)))
    io.call = call
    target = np.eye(4)
    target[2, 3] = -.001
    with pytest.raises(module.PlanningFailure, match='diagnostic state valid.*delta=0.50000'):
        io.seeded_cartesian('right', start, target)
    io.execute.assert_not_called()


@pytest.mark.parametrize('retry_joint,accepted', [(.001, True), (.5, False)])
def test_collision_aware_retry_retains_raw_jump_guard(adapter, retry_joint, accepted):
    module, io = adapter
    module.Constraints = lambda: NS(joint_constraints=[])
    io.fk, io.ik = object(), object()
    io.validate_robot_state = MagicMock()
    names = [f'right_j{i}' for i in range(1, 7)]
    start = NS(joint_state=NS(name=names, position=[0.0]*6))
    attempts = []
    def call(client, request):
        if client is io.fk:
            return NS(error_code=NS(val=1), pose_stamped=[NS(pose=NS(
                position=NS(x=0.0, y=0.0, z=request.robot_state.joint_state.position[0]),
                orientation=NS(x=0.0, y=0.0, z=0.0, w=1.0)))])
        assert client is io.ik
        attempts.append((request.ik_request.avoid_collisions,
                         len(request.ik_request.constraints.joint_constraints)))
        return NS(error_code=NS(val=-31 if len(attempts) == 1 else 1),
                  solution=NS(joint_state=NS(name=names, position=[retry_joint]+[0.0]*5)))
    io.call = call
    target = np.eye(4)
    target[2, 3] = .001
    if accepted:
        _, q, frames = io.seeded_cartesian('right', start, target)
        assert q[-1, 0] == pytest.approx(.001)
        assert np.allclose(frames[-1], target)
    else:
        with pytest.raises(module.PlanningFailure, match='IK branch jump'):
            io.seeded_cartesian('right', start, target)
    assert attempts == [(True, 6), (True, 0)]
    io.execute.assert_not_called()


def test_grasp_reach_within_tolerance_does_not_move(adapter):
    _, io = adapter
    actual = np.eye(4)
    actual[2, 3] = .002
    io.tcp_pose = MagicMock(return_value=actual)
    io.cartesian = MagicMock()
    io.ensure_grasp_reached('right', np.eye(4), .008, MagicMock())
    io.cartesian.assert_not_called()


def test_grasp_short_descent_is_corrected_once_and_remeasured(adapter):
    _, io = adapter
    actual = np.eye(4)
    actual[2, 3] = .015
    target = np.eye(4)
    io.tcp_pose = MagicMock(side_effect=[actual, target])
    io.cartesian = MagicMock()
    settle = MagicMock()
    io.ensure_grasp_reached('right', target, .008, settle)
    io.cartesian.assert_called_once_with('right', [target], .008)
    assert io.tcp_pose.call_count == settle.call_count == 2


@pytest.mark.parametrize('xyz,roll', [((0, 0, .03), 0), ((0, 0, -.01), 0),
    ((.004, 0, .01), 0), ((0, 0, .01), .2), ((float('nan'), 0, 0), 0)])
def test_grasp_bad_feedback_refuses_correction(adapter, xyz, roll):
    module, io = adapter
    actual = np.eye(4)
    actual[:3, 3] = xyz
    actual[:3, :3] = module.Rotation.from_euler('x', roll).as_matrix()
    io.tcp_pose = MagicMock(return_value=actual)
    io.cartesian = MagicMock()
    with pytest.raises(RuntimeError, match='jaws remain open'):
        io.ensure_grasp_reached('right', np.eye(4), .008, MagicMock())
    io.cartesian.assert_not_called()


@pytest.mark.parametrize('aborted', [False, True])
def test_grasp_correction_failure_never_loops_or_reports_success(adapter, aborted):
    _, io = adapter
    actual = np.eye(4)
    actual[2, 3] = .01
    io.tcp_pose = MagicMock(return_value=actual)
    io.cartesian = MagicMock(side_effect=RuntimeError('Controller aborted') if aborted else None)
    with pytest.raises(RuntimeError, match='Controller aborted' if aborted else 'jaws remain open'):
        io.ensure_grasp_reached('right', np.eye(4), .008, MagicMock())
    io.cartesian.assert_called_once()
    assert io.tcp_pose.call_count == (1 if aborted else 2)


def test_tcp_pose_uses_measured_joints_and_checks_fk_result(adapter):
    _, io = adapter
    io.fk = object()
    measured = io.state()
    io.state = MagicMock(return_value=measured)
    def call(client, request):
        assert client is io.fk
        assert request.robot_state is measured
        assert request.fk_link_names == ['right_gripper_tcp']
        return NS(error_code=NS(val=1), pose_stamped=[NS(pose=NS(
            position=NS(x=.4, y=-.2, z=.726), orientation=NS(x=0, y=0, z=0, w=1)))])
    io.call = call
    assert np.allclose(io.tcp_pose('right')[:3, 3], [.4, -.2, .726])
    io.call = MagicMock(return_value=NS(error_code=NS(val=-1)))
    with pytest.raises(RuntimeError, match='feedback FK failed'):
        io.tcp_pose('right')


@pytest.mark.parametrize('success,owner', [(False, ''), (True, 'unknown')])
def test_retry_requires_valid_ownership_service_response(adapter, success, owner):
    _, io = adapter
    io.owner = object()
    io.call = MagicMock(return_value=NS(success=success, message=owner))
    with pytest.raises(RuntimeError, match='ownership unavailable'):
        io.grasp_owner()


@pytest.mark.parametrize('keeps_moving', [False, True])
def test_retry_waits_for_stable_feedback_in_simulation_time(adapter, keeps_moving):
    _, io = adapter
    seconds = [0.0]
    io.check = MagicMock()
    io.n.get_clock.return_value.now.side_effect = lambda: NS(nanoseconds=round(seconds[0]*1e9))
    io.n.stop_event.wait.side_effect = lambda _: seconds.__setitem__(0, seconds[0]+.1)
    io.state = lambda: NS(joint_state=NS(name=['right_j1'], position=[
        (seconds[0] if keeps_moving else min(.3, seconds[0]))*.1]))
    if keeps_moving:
        with pytest.raises(TimeoutError):
            io.wait_stationary()
    else:
        io.wait_stationary()
        assert .8-1e-9 <= seconds[0] < 1.1
    io.execute.assert_not_called()


def test_relocate_object_stops_motion_and_replaces_planning_scene(adapter):
    module, io = adapter
    io.set_entity = MagicMock()
    io.c['simulation_entity'] = 'bolt_00_00'
    module.EntityState = NS
    module.SetEntityState.Request = NS
    target = np.eye(4)
    target[:3, :3] = module.Rotation.from_euler('y', np.pi/2).as_matrix()
    target[:3, 3] = [.52, -.18, .7265]
    io.call = MagicMock(return_value=NS(success=True))
    io.truth = MagicMock(return_value=target.copy())
    io.object_scene = MagicMock()
    io.relocate_object(target)
    request = io.call.call_args.args[1]
    assert request.state.name == 'bolt_00_00'
    assert request.state.reference_frame == 'world'
    assert [request.state.pose.position.x, request.state.pose.position.y,
            request.state.pose.position.z] == pytest.approx([.52, -.18, .7265])
    scene = io.object_scene.call_args.args[0]
    assert np.allclose(scene[:3, 3], target[:3, 3])
    assert np.allclose(scene[:3, :3], np.eye(3))


def test_relocate_object_failure_does_not_change_planning_scene(adapter):
    module, io = adapter
    io.set_entity = MagicMock()
    io.c['simulation_entity'] = 'bolt_00_00'
    module.EntityState = NS
    module.SetEntityState.Request = NS
    io.call = MagicMock(return_value=NS(success=False))
    io.object_scene = MagicMock()
    with pytest.raises(RuntimeError, match='refused object relocation'):
        io.relocate_object(np.eye(4))
    io.object_scene.assert_not_called()


def test_relocation_success_response_requires_actual_motion(adapter):
    module, io = adapter
    io.set_entity = MagicMock()
    io.c['simulation_entity'] = 'bolt_00_00'
    module.EntityState = NS
    module.SetEntityState.Request = NS
    io.call = MagicMock(return_value=NS(success=True))
    io.truth = MagicMock(return_value=np.eye(4))
    io.object_scene = MagicMock()
    target = np.eye(4)
    target[0, 3] = .05
    with pytest.raises(RuntimeError, match='not confirmed'):
        io.relocate_object(target)
    io.object_scene.assert_not_called()


def test_entity_service_fallback_and_missing_services(adapter):
    _, io = adapter
    primary = MagicMock(srv_name='/inspection/sim/set_entity_state')
    fallback = MagicMock(srv_name='/gazebo/set_entity_state')
    primary.wait_for_service.return_value = False
    fallback.wait_for_service.return_value = True
    assert io._available_service((primary, fallback)) is fallback
    fallback.wait_for_service.return_value = False
    with pytest.raises(RuntimeError, match='/inspection/sim/set_entity_state'):
        io._available_service((primary, fallback))


@pytest.fixture
def object_scene_adapter(adapter, monkeypatch):
    """Model Humble's attachment-before-world processing and automatic removal.

    This regression transport is not an actual MoveIt integration test.
    Message factories use real lists to expose duplicate operations hidden by mocks.
    """
    from copy import deepcopy
    module, io = adapter

    class Collision(NS):
        ADD, REMOVE = 0, 1
        def __init__(self, id='', operation=0):
            super().__init__(id=id, operation=operation, header=NS(frame_id=''),
                             primitives=[], primitive_poses=[])

    def attached(link_name='', object=None):
        return NS(link_name=link_name, object=object or Collision(), touch_links=[])

    def scene():
        return NS(is_diff=False, robot_state=NS(is_diff=False, attached_collision_objects=[]),
                  world=NS(collision_objects=[]))

    monkeypatch.setattr(module, 'CollisionObject', Collision)
    monkeypatch.setattr(module, 'AttachedCollisionObject', attached)
    monkeypatch.setattr(module, 'PlanningScene', scene)
    monkeypatch.setattr(module, 'Pose', lambda: NS(position=NS(), orientation=NS()))
    monkeypatch.setattr(module, 'SolidPrimitive', type('Primitive', (NS,), {'CYLINDER': 3}))
    monkeypatch.setattr(module, 'PlanningSceneComponents',
                        NS(WORLD_OBJECT_NAMES=8, ROBOT_STATE_ATTACHED_OBJECTS=4))
    monkeypatch.setattr(module, 'ApplyPlanningScene', NS(Request=NS))
    monkeypatch.setattr(module, 'GetPlanningScene', NS(Request=lambda: NS(components=NS(components=0))))
    io.c.update(bolt_length=.045, head_length=.008, shaft_radius=.006, head_radius=.009)
    io.apply, io.scene = object(), object()
    backend = NS(world={}, attached={}, updates=[], read_requests=[], corrupt=False)

    def call(client, request):
        if client is io.scene:
            backend.read_requests.append(request)
            result = scene()
            result.world.collision_objects = list(backend.world.values())
            if not backend.corrupt:
                result.robot_state.attached_collision_objects = list(backend.attached.values())
            return NS(scene=deepcopy(result))
        assert client is io.apply
        diff = request.scene
        assert diff.is_diff and diff.robot_state.is_diff
        backend.updates.append(deepcopy(diff))
        for item in diff.robot_state.attached_collision_objects:
            if item.object.operation == Collision.ADD:
                backend.world.pop(item.object.id, None)
                backend.attached[item.object.id] = deepcopy(item)
            else:
                old = backend.attached.pop(item.object.id, None)
                if old:
                    backend.world[item.object.id] = old.object
        success = True
        for obj in diff.world.collision_objects:
            if obj.operation == Collision.REMOVE:
                success = (backend.world.pop(obj.id, None) is not None) and success
            else:
                backend.world[obj.id] = deepcopy(obj)
        return NS(success=success)

    io.call = call
    return module, io, backend


def test_attach_repeat_handover_and_detach_keep_one_object(object_scene_adapter):
    module, io, backend = object_scene_adapter
    target = np.eye(4)
    target[:3, 3] = [.5, -.2, .729]
    io.object_scene(target)
    assert list(backend.world) == ['inspection_bolt'] and not backend.attached
    io.object_scene(target, 'right')
    assert not backend.world
    attached = backend.attached['inspection_bolt']
    assert attached.link_name == 'right_gripper_tcp'
    assert len(attached.object.primitives) == 2
    assert attached.object.primitives[0].dimensions == pytest.approx([.037, .006])
    assert attached.touch_links == ['right_left_finger', 'right_right_finger']
    assert module.matrix(attached.object.primitive_poses[0])[:3, 3] == pytest.approx([.496, -.2, .729])
    io.object_scene(target, 'right')  # Retry after a previous partial scene update.
    io.object_scene(target, 'left', previous='right')
    assert not backend.world
    assert backend.attached['inspection_bolt'].link_name == 'left_gripper_tcp'
    io.object_scene(target, previous='left')
    assert not backend.attached and list(backend.world) == ['inspection_bolt']
    assert len(backend.read_requests) == 5


def test_old_duplicate_remove_reproduces_logged_failure(object_scene_adapter):
    module, io, backend = object_scene_adapter
    io.object_scene(np.eye(4))
    io.object_scene(np.eye(4), 'right')
    diff = backend.updates[-1]
    diff.world.collision_objects.append(module.CollisionObject(
        id='inspection_bolt', operation=module.CollisionObject.REMOVE))
    with pytest.raises(RuntimeError, match='Planning scene update failed'):
        io.scene_diff(diff)
    assert backend.attached['inspection_bolt'].link_name == 'right_gripper_tcp'


def test_scene_success_without_attachment_is_not_accepted(object_scene_adapter):
    _, io, backend = object_scene_adapter
    backend.corrupt = True
    with pytest.raises(RuntimeError, match='placement mismatch'):
        io.object_scene(np.eye(4), 'right')
    io.execute.assert_not_called()
