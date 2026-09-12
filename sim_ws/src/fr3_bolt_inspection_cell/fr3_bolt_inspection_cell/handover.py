"""Small testable transaction: receiver verification precedes donor opening."""
import numpy as np
from .core import transform


def approach_receiver(io, side, object_pose, receiver_grasp, distance, speed, log):
    """Preflight perpendicular approaches from both sides and two wrist rolls."""
    from .ros_io import PlanningFailure
    failures = []
    selected = None
    for side_turn, wrist_turn in ((0, 0), (np.pi, 0), (0, np.pi), (np.pi, np.pi)):
        grasp = receiver_grasp.copy()
        grasp[:3, :3] = (transform(rpy=(side_turn, 0, 0))[:3, :3]@
                         receiver_grasp[:3, :3]@transform(rpy=(0, 0, wrist_turn))[:3, :3])
        target = object_pose@grasp
        pre = target.copy()
        pre[:3, 3] -= target[:3, 2]*distance
        log(f'Perpendicular handover preflight: side_turn={np.degrees(side_turn):.0f}, '
            f'wrist_turn={np.degrees(wrist_turn):.0f}, pre={np.round(pre[:3, 3], 4).tolist()}')
        try:
            connection, prepared = io.pick_approach(side, pre, target, plan_only=True)
        except PlanningFailure as exc:
            failures.append(str(exc))
            log(f'Perpendicular handover candidate rejected: {exc}')
            continue
        selected = connection, prepared, grasp, target
        break
    if selected is None:
        raise PlanningFailure('All 4 perpendicular handover candidates failed; no receiver motion: '+' | '.join(failures))
    connection, prepared, grasp, target = selected
    # Execution errors must never cause automatic exploration of another pose.
    io.execute(connection)
    io.execute_prepared_cartesian(prepared, speed)
    return grasp, target


def transfer(io, first, second, object_pose, close_width, open_width, settle, verify):
    io.gripper(second, close_width)
    io.assisted_grasp(second)
    io.object_scene(object_pose, second, previous=first)
    settle()
    verify(object_pose)
    if io.grasp_owner() != second:
        raise RuntimeError('Receiver ownership not confirmed; donor stays closed')
    io.gripper(first, open_width)
