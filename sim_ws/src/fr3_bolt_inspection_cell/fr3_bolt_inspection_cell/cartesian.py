"""Seeded Cartesian fallback, independent of ROS and with no execution side effects."""
from dataclasses import dataclass
import math
import numpy as np
from scipy.spatial.transform import Rotation
from .core import interpolate_object


class CartesianPlanningError(RuntimeError):
    pass


@dataclass
class PreparedCartesian:
    """The selected stage solution, retained across the preceding arm motion."""
    side: str
    start: object  # ROS RobotState, kept opaque in this ROS-independent module
    trajectory: object
    positions: np.ndarray
    frames: list


def seeded_path(start_q, start_pose, target, solve, inspect, *, step, joint_limit):
    """Plan the complete segment before returning anything to the executor.

    solve(pose, seed) must use collision-aware IK. inspect(q) must check the
    full robot state against the current scene, then return TCP FK. Raw IK
    jumps are checked before interpolation so resampling cannot hide them.
    The caller owns cancellation, service deadlines and collision diagnostics.
    """
    targets = interpolate_object(start_pose, target, np.eye(4), step, .02)
    if len(targets) > 500:
        raise CartesianPlanningError('Fallback exceeds 500 Cartesian samples')
    q = np.asarray(start_q, dtype=float)
    frames, positions = [np.asarray(start_pose)], [q.copy()]
    inspect(q)
    previous_pose = start_pose
    for index, desired in enumerate(targets, 1):
        try:
            next_q = np.asarray(solve(desired, q.copy()), dtype=float)
            if next_q.shape != q.shape or not np.all(np.isfinite(next_q)):
                raise CartesianPlanningError('IK returned missing/non-finite joint positions')
            jump = float(np.max(np.abs(next_q-q)))
            if jump > joint_limit:
                raise CartesianPlanningError(
                    f'IK branch jump {jump:.4f} rad exceeds {joint_limit:.4f} rad')
            # Validate along the joint-space interpolation, not just IK endpoints.
            count = max(1, math.ceil(jump/.02))
            for k in range(1, count+1):
                f = k/count
                sample = q+(next_q-q)*f
                actual = inspect(sample)
                # Translation along this <=1 mm segment must stay within 1 mm.
                xyz = (1-f)*previous_pose[:3, 3]+f*desired[:3, 3]
                relative = Rotation.from_matrix(previous_pose[:3, :3].T@desired[:3, :3])
                rotation = previous_pose[:3, :3]@Rotation.from_rotvec(relative.as_rotvec()*f).as_matrix()
                error = np.linalg.norm(actual[:3, 3]-xyz)
                angle = Rotation.from_matrix(rotation.T@actual[:3, :3]).magnitude()
                if error > .001 or angle > .02:
                    raise CartesianPlanningError(
                        f'FK left Cartesian segment: position={error:.5f} m, angle={angle:.4f} rad')
                positions.append(sample.copy())
                frames.append(actual)
            q, previous_pose = next_q, desired
        except CartesianPlanningError as exc:
            xyz = ', '.join(f'{v:.4f}' for v in desired[:3, 3])
            raise CartesianPlanningError(
                f'Fallback sample {index}/{len(targets)} at ({xyz}): {exc}; nothing executed') from exc
    return np.asarray(positions), frames
