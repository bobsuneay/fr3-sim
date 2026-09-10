"""ROS-independent perception and SE(3) geometry; no Gazebo truth input here."""
from dataclasses import dataclass
import math
import numpy as np
from scipy.spatial import cKDTree
from scipy.spatial.transform import Rotation, Slerp


def transform(xyz=(0, 0, 0), rpy=(0, 0, 0)):
    out = np.eye(4)
    out[:3, :3] = Rotation.from_euler('xyz', rpy).as_matrix()
    out[:3, 3] = xyz
    return out


def validate(cfg):
    def finite(value):
        if isinstance(value, dict):
            for v in value.values():
                finite(v)
        elif isinstance(value, list):
            for v in value:
                finite(v)
        elif isinstance(value, (int, float)) and not math.isfinite(value):
            raise ValueError('Configuration contains non-finite numbers')
    finite(cfg)
    if cfg['first_arm'] not in ('left', 'right'):
        raise ValueError('first_arm must be left or right')
    for key in ('roi_min', 'roi_max', 'inspection_center', 'handover_center'):
        if len(cfg[key]) != 3:
            raise ValueError(key+' must have three entries')
    if not np.all(np.array(cfg['roi_max']) > cfg['roi_min']):
        raise ValueError('Invalid ROI')
    for key in ('cloud_timeout', 'max_data_age', 'cluster_radius', 'approach_height',
                'lift_height', 'cartesian_step', 'descent_speed', 'transfer_speed',
                'scan_speed', 'joint_speed', 'joint_acceleration', 'joint_step_limit',
                'center_tolerance', 'angular_tolerance', 'settle_seconds'):
        if cfg[key] <= 0:
            raise ValueError(key+' must be positive')
    if not (0 < cfg['close_width'] < 2*cfg['shaft_radius'] < cfg['open_width'] <= .10):
        raise ValueError('Invalid jaw widths')
    if not (0 < cfg['head_length'] < cfg['bolt_length'] <= .10):
        raise ValueError('Require 0 < head_length < bolt_length <= .10 m')
    if not (0 < cfg['shaft_radius'] < cfg['head_radius'] <= .03):
        raise ValueError('Require shaft_radius < head_radius <= .03 m')
    shaft_length = cfg['bolt_length']-cfg['head_length']
    if not 0 < cfg['grasp_offset'] < shaft_length/2:
        raise ValueError('grasp_offset must lie inside the shaft, on either side of centre')
    if not (3 <= cfg['minimum_views'] <= len(cfg['views_deg'])):
        raise ValueError('Require at least three views')
    if any(len(v) != 3 for v in cfg['views_deg']):
        raise ValueError('Each view must be roll/pitch/yaw degrees')
    if not 10 <= cfg['min_points'] <= cfg['max_points'] <= 100000:
        raise ValueError('Invalid cloud limits')
    if set(cfg['cameras']) != {'waist_camera', 'left_d435i', 'right_d435i'}:
        raise ValueError('Require the fixed waist camera and both wrist D435i cameras')
    for name, camera in cfg['cameras'].items():
        if any(len(camera[k]) != 3 for k in ('xyz', 'rpy')):
            raise ValueError(name+' needs three xyz/rpy values')
        if not (0 < camera['near'] < camera['far'] and 0 < camera['horizontal_fov'] < math.pi):
            raise ValueError(name+' invalid depth range/FOV')
        if camera['rate'] <= 0 or any(type(camera[k]) is not int or not 16 <= camera[k] <= 4096
                                      for k in ('width', 'height')):
            raise ValueError(name+' invalid rate/resolution')
    return cfg


@dataclass
class Estimate:
    pose: np.ndarray                 # object X along shaft, Z nominally upward
    dimensions: np.ndarray
    points: np.ndarray
    head_resolved: bool


def estimate_bolt(points, cfg):
    p = np.asarray(points, dtype=float).reshape(-1, 3)
    keep = np.all(np.isfinite(p), axis=1)
    keep &= np.all((p >= cfg['roi_min']) & (p <= cfg['roi_max']), axis=1)
    p = p[keep]
    if len(p) < cfg['min_points']:
        raise ValueError('Not enough bolt points above the calibrated tabletop')
    if len(p) > cfg['max_points']:
        raise ValueError('ROI too large or tabletop not removed; refine ROI/table_z')
    tree = cKDTree(p)
    visited = np.zeros(len(p), dtype=bool)
    clusters = []
    for seed in range(len(p)):
        if visited[seed]:
            continue
        stack, indices = [seed], []
        visited[seed] = True
        while stack:
            i = stack.pop()
            indices.append(i)
            for j in tree.query_ball_point(p[i], cfg['cluster_radius']):
                if not visited[j]:
                    visited[j] = True
                    stack.append(j)
        if len(indices) >= cfg['min_points']:
            clusters.append(p[indices])
    candidates = []
    for cloud in clusters:
        xy = cloud[:, :2]
        eig, vec = np.linalg.eigh(np.cov(xy.T))
        if eig[-1] < 2.5*max(eig[0], 1e-12):
            continue
        axis = np.r_[vec[:, -1], 0.0]
        if axis[0] < 0:
            axis = -axis
        lateral = np.cross([0, 0, 1], axis)
        along, across = cloud@axis, cloud@lateral
        lo, hi = np.quantile(along, [.01, .99])
        width = np.quantile(across, .99)-np.quantile(across, .01)
        if not (.028 <= hi-lo <= .042 and .003 <= width <= .016):
            continue
        ends = [across[along < lo+.005], across[along > hi-.005]]
        spans = [np.ptp(e) if len(e) >= 4 else 0 for e in ends]
        resolved = min(spans) > 0 and max(spans)/min(spans) > 1.15
        if resolved and spans[0] > spans[1]:
            axis, lateral = -axis, -lateral
            along, across = cloud@axis, cloud@lateral
            lo, hi = np.quantile(along, [.01, .99])
        # A table-constrained approximation, not an observable full 6D screw pose.
        center = axis*(lo+hi)/2 + lateral*np.mean(np.quantile(across, [.01, .99]))
        center[2] = np.quantile(cloud[:, 2], .98)-cfg['head_radius']
        center[2] = np.clip(center[2], cfg['table_z']+cfg['shaft_radius'],
                            cfg['table_z']+cfg['head_radius'])
        pose = np.eye(4)
        pose[:3, :3] = np.column_stack((axis, lateral, [0, 0, 1]))
        pose[:3, 3] = center
        candidates.append(Estimate(pose, np.array([hi-lo, width, .009]), cloud, resolved))
    if len(candidates) != 1:
        raise ValueError(f'Expected one isolated bolt, found {len(candidates)}; narrow ROI')
    if not candidates[0].head_resolved:
        raise ValueError('Bolt head/tail ambiguous; need a clearer point cloud before handover')
    return candidates[0]


def grasp_in_object(offset, below=False):
    """T_object_tcp: the jaws close along object Y at an axial offset."""
    result = np.eye(4)
    result[:3, :3] = [[0, 1, 0], [-1 if below else 1, 0, 0], [0, 0, 1 if below else -1]]
    result[:3, 3] = [offset, 0, 0]
    return result


def centered_views(center, neutral_rotation, object_tcp, views):
    for angles in views:
        obj = transform(center)
        obj[:3, :3] = neutral_rotation @ Rotation.from_euler('xyz', angles, degrees=True).as_matrix()
        yield obj, obj @ object_tcp


def interpolate_object(a, b, object_tcp, step=.002, angle_step=.04):
    """Interpolate OBJECT pose, then map to TCP: preserves centre during rotation."""
    rotations = Rotation.from_matrix([a[:3, :3], b[:3, :3]])
    angle = (rotations[0].inv()*rotations[1]).magnitude()
    n = max(1, math.ceil(np.linalg.norm(b[:3, 3]-a[:3, 3])/step), math.ceil(angle/angle_step))
    slerp = Slerp([0, 1], rotations)
    output = []
    for f in np.linspace(0, 1, n+1)[1:]:
        obj = np.eye(4)
        obj[:3, 3] = (1-f)*a[:3, 3]+f*b[:3, 3]
        obj[:3, :3] = slerp(f).as_matrix()
        output.append(obj@object_tcp)
    return output


def segment_times(positions, tcp_positions, speed, joint_speed, acceleration):
    """Rest at every waypoint. Conservative quintic timing; no zero-time segments."""
    q, xyz = np.asarray(positions), np.asarray(tcp_positions)
    if len(q) < 2 or len(q) != len(xyz) or not np.all(np.isfinite(q)):
        raise ValueError('Invalid Cartesian trajectory')
    dq = np.max(np.abs(np.diff(q, axis=0)), axis=1)
    dx = np.linalg.norm(np.diff(xyz, axis=0), axis=1)
    dt = np.maximum.reduce([1.875*dx/speed, 1.875*dq/joint_speed,
                            np.sqrt(5.8*dq/acceleration), np.full(len(dq), .05)])
    return np.r_[.1, .1+np.cumsum(dt)]


class SimClockDeadline:
    """Track a simulated-time deadline while detecting a stalled /clock."""

    def __init__(self, ros_time, wall_time, timeout, stall_timeout=30.0):
        self.start_ros = self.last_ros = float(ros_time)
        self.last_progress_wall = float(wall_time)
        self.timeout = float(timeout)
        self.stall_timeout = float(stall_timeout)

    def check(self, ros_time, wall_time):
        ros_time, wall_time = float(ros_time), float(wall_time)
        if ros_time > self.last_ros + 1e-6:
            self.last_ros = ros_time
            self.last_progress_wall = wall_time
        elapsed = ros_time-self.start_ros
        if elapsed > self.timeout:
            raise TimeoutError('Controller exceeded its ROS-time deadline')
        if wall_time-self.last_progress_wall > self.stall_timeout:
            raise TimeoutError('ROS/Gazebo clock stopped during controller action')
        return elapsed
