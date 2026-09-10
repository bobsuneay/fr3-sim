"""Failure-injection tests for full-segment Cartesian preflight without ROS."""
from pathlib import Path
import sys
import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from fr3_bolt_inspection_cell.cartesian import CartesianPlanningError, seeded_path
from fr3_bolt_inspection_cell.core import transform


def plan(solve=None, inspect=None, target=None):
    # Synthetic mechanism: joint displacement maps to TCP height at 0.1 m/rad.
    return seeded_path([0.0], transform([0, 0, .776]),
                       transform([0, 0, .726]) if target is None else target,
                       solve or (lambda pose, seed: [(pose[2, 3]-.776)*10]),
                       inspect or (lambda q: transform([0, 0, .776+q[0]*.1])),
                       step=.001, joint_limit=.2)


def test_complete_descent_uses_previous_solution_as_seed():
    seeds, inspected = [], []
    def solve(pose, seed):
        seeds.append(seed.copy())
        return [(pose[2, 3]-.776)*10]
    def inspect(q):
        inspected.append(q.copy())
        return transform([0, 0, .776+q[0]*.1])
    q, frames = plan(solve, inspect)
    assert q[-1, 0] == pytest.approx(-.5)
    assert frames[-1][2, 3] == pytest.approx(.726)
    assert np.all(np.diff(q[:, 0]) < 0)
    assert np.allclose(seeds, q[:-1])
    assert len(inspected) == len(q)


def test_collision_stops_preflight_with_pair_and_position():
    def inspect(q):
        if q[0] < -.23:
            raise CartesianPlanningError('right_wrist2_link <-> table_top')
        return transform([0, 0, .776+q[0]*.1])
    with pytest.raises(CartesianPlanningError, match='right_wrist2_link <-> table_top') as exc:
        plan(inspect=inspect)
    assert 'Fallback sample' in str(exc.value) and 'nothing executed' in str(exc.value)


def test_raw_ik_branch_jump_is_rejected_before_resampling():
    checked = []
    def inspect(q):
        checked.append(q)
        return transform([0, 0, .776])
    with pytest.raises(CartesianPlanningError, match='IK branch jump'):
        plan(solve=lambda pose, seed: [6.28], inspect=inspect)
    assert len(checked) == 1  # only the start; never subdivide a branch jump


def test_collision_between_valid_ik_endpoints_is_checked():
    def inspect(q):
        if .04 < q[0] < .06:
            raise CartesianPlanningError('intermediate collision')
        return transform([0, 0, .776-q[0]*.01])
    with pytest.raises(CartesianPlanningError, match='intermediate collision'):
        plan(solve=lambda pose, seed: [.1], inspect=inspect,
             target=transform([0, 0, .7755]))


def test_fk_departure_and_nonfinite_ik_rejected():
    with pytest.raises(CartesianPlanningError, match='FK left Cartesian segment'):
        plan(inspect=lambda q: transform([.01, 0, .776+q[0]*.1]))
    with pytest.raises(CartesianPlanningError, match='non-finite'):
        plan(solve=lambda pose, seed: [float('nan')])


def test_missing_ik_does_not_return_partial_path():
    def solve(pose, seed):
        if pose[2, 3] < .75:
            raise CartesianPlanningError('IK failed code=-31')
        return [(pose[2, 3]-.776)*10]
    with pytest.raises(CartesianPlanningError, match='IK failed code=-31'):
        plan(solve=solve)
