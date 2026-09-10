"""MoveIt Humble service/action adapter with simulation-clock-aware execution."""
import threading
import time
import numpy as np
import rclpy
from action_msgs.msg import GoalStatus
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
from geometry_msgs.msg import Pose
from moveit_msgs.action import ExecuteTrajectory, MoveGroup
from moveit_msgs.msg import (AllowedCollisionEntry, AttachedCollisionObject, CollisionObject,
    Constraints, JointConstraint, OrientationConstraint, PlanningScene, PlanningSceneComponents,
    PositionConstraint, RobotState)
from moveit_msgs.srv import ApplyPlanningScene, GetCartesianPath, GetPlanningScene, GetPositionFK
from rclpy.action import ActionClient
from rclpy.duration import Duration as RclDuration
from rclpy.time import Time
from scipy.spatial.transform import Rotation
from shape_msgs.msg import SolidPrimitive
from std_srvs.srv import SetBool, Trigger
from gazebo_msgs.srv import GetEntityState
from trajectory_msgs.msg import JointTrajectoryPoint
from .core import SimClockDeadline, segment_times


class PlanningFailure(RuntimeError):
    """No trajectory was executed; skipping an inspection view is permissible."""


def matrix(pose):
    result = np.eye(4)
    result[:3, 3] = [pose.position.x, pose.position.y, pose.position.z]
    q = pose.orientation
    result[:3, :3] = Rotation.from_quat([q.x, q.y, q.z, q.w]).as_matrix()
    return result


def pose(t):
    p = Pose()
    p.position.x, p.position.y, p.position.z = map(float, t[:3, 3])
    p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = map(
        float, Rotation.from_matrix(t[:3, :3]).as_quat())
    return p


def duration(seconds):
    ns = round(seconds*1e9)
    return Duration(sec=ns//1000000000, nanosec=ns % 1000000000)


class IO:
    def __init__(self, node):
        self.n, self.c = node, node.cfg
        self.move = ActionClient(node, MoveGroup, '/move_action')
        self.execute_client = ActionClient(node, ExecuteTrajectory, '/execute_trajectory')
        self.fingers = {s: ActionClient(node, FollowJointTrajectory,
            f'/{s}_gripper_controller/follow_joint_trajectory') for s in ('left', 'right')}
        self.cart = node.create_client(GetCartesianPath, '/compute_cartesian_path')
        self.fk = node.create_client(GetPositionFK, '/compute_fk')
        self.apply = node.create_client(ApplyPlanningScene, '/apply_planning_scene')
        self.scene = node.create_client(GetPlanningScene, '/get_planning_scene')
        self.entity = node.create_client(GetEntityState, '/inspection/sim/get_entity_state')
        self.owner = node.create_client(Trigger, '/inspection/sim/owner')
        self.grasps = {s: node.create_client(SetBool, '/inspection/sim/'+s+'_grasp')
                       for s in ('left', 'right')}
        self.active = None
        self.watch_center = None
        self.goal_lock = threading.Lock()

    def check(self):
        if self.n.stop_event.is_set() or not rclpy.ok():
            raise RuntimeError('Stopped; gripper holding state preserved')
        if self.watch_center is not None:
            with self.n.data_lock:
                actual = self.n.object_state
            if actual is None or time.monotonic()-actual[1] > self.c['max_data_age']:
                raise RuntimeError('Simulation object feedback stale during rotation')
            tolerance = self.c['center_tolerance'] + np.linalg.norm(self.n.object_truth[:3, 3])
            if np.linalg.norm(actual[0][:3, 3]-self.watch_center) > tolerance:
                raise RuntimeError('Observed object centre drift exceeded rotation tolerance')

    def wait(self, future, timeout=15):
        end = time.monotonic()+timeout
        while not future.done():
            self.check()
            if time.monotonic() > end:
                raise TimeoutError('ROS request/action timed out')
            self.n.stop_event.wait(.02)
        self.check()
        if future.exception():
            raise future.exception()
        return future.result()

    def wait_sim_time(self, future, timeout, stalled_wall_timeout=30.0):
        """Wait in ROS time so a slow Gazebo run is not mistaken for a failure."""
        now = self.n.get_clock().now().nanoseconds*1e-9
        deadline = SimClockDeadline(now, time.monotonic(), timeout, stalled_wall_timeout)
        last_report = time.monotonic()
        while not future.done():
            self.check()
            wall_now = time.monotonic()
            ros_now = self.n.get_clock().now().nanoseconds*1e-9
            elapsed = deadline.check(ros_now, wall_now)
            if wall_now-last_report >= 10.0:
                self.n.get_logger().info(
                    f'Controller still running: {elapsed:.1f}/{timeout:.1f} s simulation time')
                last_report = wall_now
            self.n.stop_event.wait(.02)
        self.check()
        if future.exception():
            raise future.exception()
        return future.result()

    def call(self, client, request, timeout=15):
        self.check()
        if not client.wait_for_service(timeout_sec=3):
            raise RuntimeError('Service unavailable: '+client.srv_name)
        return self.wait(client.call_async(request), timeout)

    def cancel(self):
        with self.goal_lock:
            if self.active is not None:
                self.active.cancel_goal_async()

    def action(self, client, goal, timeout, planning_only=False, controller_time=False):
        self.check()
        if not client.wait_for_server(timeout_sec=3):
            raise RuntimeError('Action server unavailable')
        sent = client.send_goal_async(goal)
        # Late acceptance after timeout/stop must also be cancelled.
        abandoned = threading.Event()
        def late(future):
            if abandoned.is_set() or self.n.stop_event.is_set():
                handle = future.result()
                if handle and handle.accepted:
                    handle.cancel_goal_async()
        sent.add_done_callback(late)
        try:
            handle = self.wait(sent, 10)
        except Exception:
            abandoned.set()
            if sent.done():
                late(sent)
            raise
        if not handle.accepted:
            raise RuntimeError('Action goal rejected')
        with self.goal_lock:
            self.active = handle
        try:
            result_future = handle.get_result_async()
            use_sim_time = bool(self.n.get_parameter('use_sim_time').value)
            if controller_time and use_sim_time:
                result = self.wait_sim_time(result_future, timeout)
            else:
                result = self.wait(result_future, timeout)
            if result.status != GoalStatus.STATUS_SUCCEEDED and not planning_only:
                raise RuntimeError(f'Action ended with status {result.status}')
            return result.result
        except Exception:
            handle.cancel_goal_async()
            raise
        finally:
            with self.goal_lock:
                self.active = None

    def state(self):
        result = RobotState()
        now = time.monotonic()
        names = [f'{s}_j{i}' for s in ('left', 'right') for i in range(1, 7)]
        names += [f'{s}_{f}_finger_joint' for s in ('left', 'right') for f in ('left', 'right')]
        with self.n.data_lock:
            values = [self.n.joints.get(name) for name in names]
        if any(v is None or now-v[1] > self.c['max_data_age'] for v in values):
            raise RuntimeError('Missing/stale joint feedback; need all 16 joints')
        result.joint_state.name = names
        result.joint_state.position = [v[0] for v in values]
        result.is_diff = True
        return result

    def tf(self, link, stamp=None):
        tr = self.n.tf_buffer.lookup_transform('world', link, stamp or Time(),
                                               timeout=RclDuration(seconds=1)).transform
        p = Pose()
        p.position.x, p.position.y, p.position.z = tr.translation.x, tr.translation.y, tr.translation.z
        p.orientation = tr.rotation
        return matrix(p)

    def fk_poses(self, side, trajectory, start):
        result = []
        names = trajectory.joint_trajectory.joint_names
        for point in trajectory.joint_trajectory.points:
            req = GetPositionFK.Request()
            req.header.frame_id = 'world'
            req.fk_link_names = [side+'_gripper_tcp']
            req.robot_state = RobotState()
            req.robot_state.joint_state.name = list(start.joint_state.name)
            req.robot_state.joint_state.position = list(start.joint_state.position)
            for name, q in zip(names, point.positions):
                req.robot_state.joint_state.position[req.robot_state.joint_state.name.index(name)] = q
            res = self.call(self.fk, req)
            if res.error_code.val != 1 or len(res.pose_stamped) != 1:
                raise RuntimeError('Forward kinematics failed')
            result.append(matrix(res.pose_stamped[0].pose))
        return result

    def execute(self, trajectory):
        points = trajectory.joint_trajectory.points
        if not points:
            raise PlanningFailure('Empty trajectory')
        self.state()
        goal = ExecuteTrajectory.Goal()
        goal.trajectory = trajectory
        t = points[-1].time_from_start
        result = self.action(self.execute_client, goal, t.sec+t.nanosec*1e-9+30,
                             controller_time=True)
        if result.error_code.val != 1:
            raise RuntimeError(f'Trajectory execution failed: {result.error_code.val}')

    def global_move(self, side, target=None, joints=None):
        goal = MoveGroup.Goal()
        req = goal.request
        req.group_name = side+'_arm'
        req.start_state = self.state()
        req.num_planning_attempts = 5
        req.allowed_planning_time = 10.0
        # The MoveIt model limits these joints to 0.3 rad/s and 0.3 rad/s^2.
        # Match inspection.yaml's 0.12 rad/s and 0.15 rad/s^2 limits.
        req.max_velocity_scaling_factor = min(1.0, self.c['joint_speed']/.3)
        req.max_acceleration_scaling_factor = min(1.0, self.c['joint_acceleration']/.3)
        constraint = Constraints()
        if joints is not None:
            for i, value in enumerate(joints, 1):
                constraint.joint_constraints.append(JointConstraint(joint_name=f'{side}_j{i}',
                    position=float(value), tolerance_above=.005, tolerance_below=.005, weight=1.0))
        else:
            pos = PositionConstraint()
            pos.header.frame_id = 'world'
            pos.link_name = side+'_gripper_tcp'
            pos.weight = 1.0
            pos.constraint_region.primitives = [SolidPrimitive(type=SolidPrimitive.SPHERE, dimensions=[.001])]
            pos.constraint_region.primitive_poses = [pose(target)]
            ori = OrientationConstraint()
            ori.header.frame_id = 'world'
            ori.link_name = pos.link_name
            ori.orientation = pose(target).orientation
            ori.absolute_x_axis_tolerance = .02
            ori.absolute_y_axis_tolerance = .02
            ori.absolute_z_axis_tolerance = .02
            ori.weight = 1.0
            constraint.position_constraints = [pos]
            constraint.orientation_constraints = [ori]
        req.goal_constraints = [constraint]
        goal.planning_options.plan_only = True
        goal.planning_options.planning_scene_diff.is_diff = True
        goal.planning_options.planning_scene_diff.robot_state.is_diff = True
        result = self.action(self.move, goal, 40, planning_only=True)
        if result.error_code.val != 1:
            raise PlanningFailure(f'Pose planning failed: {result.error_code.val}')
        self.execute(result.planned_trajectory)

    def cartesian(self, side, waypoints, speed, object_tcp=None, center=None):
        req = GetCartesianPath.Request()
        req.header.frame_id = 'world'
        req.start_state = self.state()
        req.group_name = side+'_arm'
        req.link_name = side+'_gripper_tcp'
        req.waypoints = [pose(t) for t in waypoints]
        req.max_step = self.c['cartesian_step']
        req.jump_threshold = 2.0
        req.revolute_jump_threshold = self.c['joint_step_limit']
        req.avoid_collisions = True
        # MoveIt's Humble GetCartesianPath.srv has no velocity or acceleration
        # scaling fields.  The returned path is retimed below using the
        # configured Cartesian speed and joint limits before execution.
        response = self.call(self.cart, req, 40)
        if response.error_code.val != 1 or response.fraction < .99999:
            raise PlanningFailure(f'Cartesian path incomplete ({response.fraction:.1%}); nothing executed')
        trajectory = response.solution
        q = np.array([p.positions for p in trajectory.joint_trajectory.points])
        if len(q) < 2 or np.max(np.abs(np.diff(q, axis=0))) > self.c['joint_step_limit']:
            raise PlanningFailure('Empty path or joint discontinuity')
        frames = self.fk_poses(side, trajectory, req.start_state)
        if object_tcp is not None and center is not None:
            object_poses = [t@np.linalg.inv(object_tcp) for t in frames]
            if max(np.linalg.norm(t[:3, 3]-center) for t in object_poses) > self.c['center_tolerance']:
                raise PlanningFailure('Planned rotation exceeds object centre drift tolerance')
        times = segment_times(q, [t[:3, 3] for t in frames], speed,
                              self.c['joint_speed'], self.c['joint_acceleration'])
        for point, seconds in zip(trajectory.joint_trajectory.points, times):
            point.velocities = [0.0]*len(point.positions)
            point.accelerations = [0.0]*len(point.positions)
            point.time_from_start = duration(seconds)
        self.watch_center = np.asarray(center) if center is not None else None
        try:
            self.execute(trajectory)
        finally:
            self.watch_center = None

    def gripper(self, side, width):
        if not 0 <= width <= .10:
            raise ValueError('Invalid gripper width')
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = [side+'_left_finger_joint', side+'_right_finger_joint']
        goal.trajectory.points = [JointTrajectoryPoint(positions=[width/2]*2,
            velocities=[0.0, 0.0], time_from_start=duration(3.0))]
        result = self.action(self.fingers[side], goal, 15, controller_time=True)
        if result.error_code != 0:
            raise RuntimeError('Gripper trajectory failed: '+result.error_string)
        state = self.state().joint_state
        measured = [state.position[state.name.index(j)] for j in goal.trajectory.joint_names]
        if max(abs(v-width/2) for v in measured) > .0015:
            raise RuntimeError('Gripper position feedback did not reach target')

    def assisted_grasp(self, side, close=True):
        res = self.call(self.grasps[side], SetBool.Request(data=close), 5)
        if not res.success:
            raise RuntimeError('Assisted grasp refused: '+res.message)
        status = self.call(self.owner, Trigger.Request())
        if close and status.message != side:
            raise RuntimeError('Simulator grasp ownership not confirmed')

    def truth(self):
        # Deliberately isolated from estimate_bolt and from pick pose generation.
        res = self.call(self.entity, GetEntityState.Request(
            name=self.c['simulation_entity'], reference_frame='world'))
        if not res.success:
            raise RuntimeError('Simulation verification state unavailable')
        return matrix(res.state.pose)

    def scene_diff(self, diff):
        diff.is_diff = True
        diff.robot_state.is_diff = True
        if not self.call(self.apply, ApplyPlanningScene.Request(scene=diff)).success:
            raise RuntimeError('Planning scene update failed')

    def allow_touch(self, links, allowed=True):
        req = GetPlanningScene.Request()
        req.components.components = PlanningSceneComponents.ALLOWED_COLLISION_MATRIX
        acm = self.call(self.scene, req).scene.allowed_collision_matrix
        for name in ['inspection_bolt', *links]:
            if name not in acm.entry_names:
                acm.entry_names.append(name)
                for entry in acm.entry_values:
                    entry.enabled.append(False)
                acm.entry_values.append(AllowedCollisionEntry(enabled=[False]*len(acm.entry_names)))
        i = acm.entry_names.index('inspection_bolt')
        for name in links:
            j = acm.entry_names.index(name)
            acm.entry_values[i].enabled[j] = allowed
            acm.entry_values[j].enabled[i] = allowed
        diff = PlanningScene()
        diff.allowed_collision_matrix = acm
        self.scene_diff(diff)

    def object_scene(self, world_object, side=None, previous=None):
        obj = CollisionObject(id='inspection_bolt', operation=CollisionObject.ADD)
        obj.header.frame_id = 'world'
        # Object frame X is the bolt axis; cylinder local Z is the bolt axis.
        lengths = [self.c['bolt_length']-self.c['head_length'], self.c['head_length']]
        radii = [self.c['shaft_radius'], self.c['head_radius']]
        centers = [-self.c['head_length']/2, lengths[0]/2]
        for length, radius, x in zip(lengths, radii, centers):
            t = np.eye(4)
            t[:3, :3] = Rotation.from_euler('y', np.pi/2).as_matrix()
            t[0, 3] = x
            obj.primitives.append(SolidPrimitive(type=SolidPrimitive.CYLINDER,
                                                 dimensions=[length, radius]))
            obj.primitive_poses.append(pose(world_object@t))
        diff = PlanningScene()
        if previous:
            old = AttachedCollisionObject(link_name=previous+'_gripper_tcp')
            old.object.id = obj.id
            old.object.operation = CollisionObject.REMOVE
            diff.robot_state.attached_collision_objects.append(old)
        if side:
            attachment = AttachedCollisionObject(link_name=side+'_gripper_tcp', object=obj)
            attachment.touch_links = [side+'_'+f+'_finger' for f in ('left', 'right')]
            diff.robot_state.attached_collision_objects.append(attachment)
            removed = CollisionObject(id=obj.id, operation=CollisionObject.REMOVE)
            diff.world.collision_objects.append(removed)
        else:
            diff.world.collision_objects.append(obj)
        self.scene_diff(diff)
