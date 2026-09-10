"""Numerical FR3 FK/IK regression at the logged failed pick position.

This verifies kinematic continuity only. Scene collisions and the installed
KDL solver still require ROS integration testing.
"""
from pathlib import Path
import sys
import xml.etree.ElementTree as ET
import numpy as np
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation
import yaml

SHARE = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(SHARE))
from fr3_bolt_inspection_cell.cartesian import seeded_path
from fr3_bolt_inspection_cell.core import transform, grasp_in_object


def test_logged_descent_has_continuous_grasp_first_joint_branch():
    cfg = yaml.safe_load((SHARE/'config/arms.yaml').read_text())
    robot = ET.parse(SHARE.parent/'fr3_dual_bolt_cell/urdf/fr3_arm.urdf').getroot()
    joints = [robot.find(f"joint[@name='j{i}']") for i in range(1, 7)]
    offsets = [transform(*[[float(v) for v in j.find('origin').get(key, '0 0 0').split()]
                           for key in ('xyz', 'rpy')]) for j in joints]
    bounds = np.array([[float(j.find('limit').get(k)) for j in joints] for k in ('lower', 'upper')])
    arm, gripper = cfg['right'], cfg['gripper']
    def fk(q):
        t = transform(arm['xyz'], arm['rpy'])
        for offset, angle in zip(offsets, q):
            t = t@offset@transform(rpy=[0, 0, angle])
        return t@transform(gripper['flange_xyz'], gripper['flange_rpy'])@transform(
            gripper['tcp_xyz'], gripper['tcp_rpy'])
    def solve(target, seed):
        def error(q):
            actual = fk(q)
            return np.r_[actual[:3, 3]-target[:3, 3],
                         .25*Rotation.from_matrix(target[:3, :3].T@actual[:3, :3]).as_rotvec()]
        result = least_squares(error, seed, bounds=bounds, max_nfev=150,
                               ftol=1e-10, gtol=1e-10, xtol=1e-10)
        assert np.linalg.norm(error(result.x)) < 1e-5
        return result.x
    grasp = transform([.4893, -.2001, .726])@grasp_in_object(0)
    above = grasp.copy()
    above[2, 3] += .05
    # A numerical seed, used only in this regression, never sent to a robot.
    grasp_q = solve(grasp, [-.0467, -2.75, -.5793, -2.9539, -.0467, -1.5709])
    q, frames = seeded_path(grasp_q, fk(grasp_q), above, solve, fk, step=.001, joint_limit=.2)
    assert np.allclose(frames[-1], above, atol=2e-5)
    assert q[:, 4].min() < 0 < q[:, 4].max()  # passes through the difficult wrist region
    assert np.max(np.abs(np.diff(q, axis=0))) < .05
    assert all(np.allclose(fk(row), frame, atol=1e-8) for row, frame in zip(q[::-1], frames[::-1]))
    assert np.allclose(frames[0], grasp, atol=2e-5)
