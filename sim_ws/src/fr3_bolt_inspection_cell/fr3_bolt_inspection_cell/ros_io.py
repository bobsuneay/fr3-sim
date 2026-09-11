"""MoveIt Humble service/action adapter with simulation-clock-aware execution."""
from copy import deepcopy
import itertools
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
    PositionConstraint, RobotState, RobotTrajectory)
from moveit_msgs.srv import (ApplyPlanningScene, GetCartesianPath, GetPlanningScene,
                             GetPositionFK, GetPositionIK, GetStateValidity)
from rclpy.action import ActionClient
from rclpy.duration import Duration as RclDuration
from rclpy.time import Time
from scipy.spatial.transform import Rotation
from shape_msgs.msg import SolidPrimitive
from std_srvs.srv import SetBool, Trigger
from gazebo_msgs.msg import EntityState
from gazebo_msgs.srv import GetEntityState, SetEntityState
from trajectory_msgs.msg import JointTrajectoryPoint
from .core import SimClockDeadline, segment_times
from .cartesian import CartesianPlanningError, PreparedCartesian, seeded_path


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
        self.ik = node.create_client(GetPositionIK, '/compute_ik')
        self.validity = node.create_client(GetStateValidity, '/check_state_validity')
        self.apply = node.create_client(ApplyPlanningScene, '/apply_planning_scene')
        self.scene = node.create_client(GetPlanningScene, '/get_planning_scene')
        self.entity = node.create_client(GetEntityState, '/inspection/sim/get_entity_state')
        self.set_entity = node.create_client(SetEntityState, '/inspection/sim/set_entity_state')
        # gazebo_ros_api_plugin exposes the same services globally on some
        # Humble installations; keep a fallback so the randomize button works
        # with either namespacing layout.
        self.entity_fallback = node.create_client(GetEntityState, '/gazebo/get_entity_state')
        self.set_entity_fallback = node.create_client(SetEntityState, '/gazebo/set_entity_state')
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

    def tcp_pose(self, side):
        """Compute the measured TCP from fresh joint feedback, not a goal/TF cache."""
        req = GetPositionFK.Request()
        req.header.frame_id = 'world'
        req.fk_link_names = [side+'_gripper_tcp']
        req.robot_state = self.state()
        response = self.call(self.fk, req)
        if response.error_code.val != 1 or len(response.pose_stamped) != 1:
            raise RuntimeError('Grasp feedback FK failed')
        result = matrix(response.pose_stamped[0].pose)
        if not np.all(np.isfinite(result)):
            raise RuntimeError('Non-finite grasp TCP feedback')
        return result

    def ensure_grasp_reached(self, side, target, speed, settle):
        """Check actual reach before closing; allow one bounded, fully planned correction."""
        tolerance = self.c['grasp_reach_tolerance']
        for attempt in range(2):
            settle()
            actual = self.tcp_pose(side)
            if not np.all(np.isfinite(actual)):
                raise RuntimeError('Non-finite grasp TCP feedback; jaws remain open')
            delta = actual[:3, 3]-target[:3, 3]
            position_error = float(np.linalg.norm(delta))
            angle = Rotation.from_matrix(target[:3, :3].T@actual[:3, :3]).magnitude()
            self.n.get_logger().info(
                f'Grasp reach check {attempt+1}/2: side={side}, '
                f'target={np.round(target[:3, 3], 5).tolist()}, '
                f'actual={np.round(actual[:3, 3], 5).tolist()}, '
                f'error={position_error*1000:.2f} mm, height_gap={delta[2]*1000:.2f} mm, '
                f'angle={angle:.4f} rad')
            if position_error <= tolerance and angle <= self.c['angular_tolerance']:
                return
            if (attempt == 1 or np.linalg.norm(delta[:2]) > tolerance
                    or not 0 < delta[2] <= self.c['grasp_recovery_max']
                    or angle > self.c['angular_tolerance']):
                raise RuntimeError(
                    f'Grasp target not reached: error={position_error*1000:.2f} mm, '
                    f'height_gap={delta[2]*1000:.2f} mm, angle={angle:.4f} rad; jaws remain open')
            self.n.get_logger().warning(
                'Supplemental grasp descent: one retry from measured state; '
                'exact target pose, collision checks ON, complete path required')
            # cartesian() never sends an incomplete path. Planning/execution
            # failures propagate immediately; do not close or retry an aborted action.
            self.cartesian(side, [target], speed)

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

    def global_move(self, side, target=None, joints=None, continuation=None, plan_only=False):
        if continuation is not None:
            return self.pick_approach(side, target, continuation)
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
                    position=float(value), tolerance_above=.0001, tolerance_below=.0001, weight=1.0))
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
        if plan_only:
            return result.planned_trajectory
        self.execute(result.planned_trajectory)

    def pick_approach(self, side, above, grasp):
        """Generate grasp IK first, propagate upward, then connect to that branch.

        A free pose goal above the part can land on a slightly tilted wrist
        solution that cannot follow the precise downward segment near j5=0.
        Planning backward from the grasp keeps the selected grasp orientation.
        """
        start = self.state()
        names = [f'{side}_j{i}' for i in range(1, 7)]
        indices = [start.joint_state.name.index(name) for name in names]
        initial = np.array([start.joint_state.position[i] for i in indices])
        # FR3 j1/j3/j5 have symmetric limits. Sign changes explore base,
        # elbow and wrist seeds without altering the actual robot state.
        seen = []
        failure = 'No grasp IK candidate'
        for attempt, signs in enumerate(itertools.product((1, -1), repeat=3), 1):
            seed = initial.copy()
            seed[[0, 2, 4]] *= signs
            request = GetPositionIK.Request()
            request.ik_request.group_name = side+'_arm'
            request.ik_request.ik_link_name = side+'_gripper_tcp'
            request.ik_request.robot_state = deepcopy(start)
            for index, value in zip(indices, seed):
                request.ik_request.robot_state.joint_state.position[index] = float(value)
            request.ik_request.pose_stamped.header.frame_id = 'world'
            request.ik_request.pose_stamped.pose = pose(grasp)
            request.ik_request.avoid_collisions = True
            request.ik_request.timeout = duration(1.0)
            response = self.call(self.ik, request)
            if response.error_code.val != 1:
                failure = f'Grasp IK code={response.error_code.val}'
                self.n.get_logger().warning(f'Grasp candidate {attempt}/8: {failure}')
                continue
            values = dict(zip(response.solution.joint_state.name, response.solution.joint_state.position))
            grasp_q = np.array([values[name] for name in names])
            if any(np.max(np.abs(grasp_q-previous)) < .01 for previous in seen):
                self.n.get_logger().info(f'Grasp candidate {attempt}/8: repeated joint solution, skipping')
                continue
            seen.append(grasp_q)
            at_grasp = deepcopy(start)
            for index, value in zip(indices, grasp_q):
                at_grasp.joint_state.position[index] = float(value)
            self.n.get_logger().info(
                f'Grasp-first preflight {attempt}/8: q={np.round(grasp_q, 5).tolist()}; '
                'planning upward from grasp, no motion sent')
            try:
                self.validate_robot_state(at_grasp)
                upward, q_up, frames_up = self.seeded_cartesian(side, at_grasp, above)
                # Reverse geometry only. Timing is assigned later for descent.
                q = q_up[::-1].copy()
                frames = list(reversed(frames_up))
                downward = deepcopy(upward)
                downward.joint_trajectory.points = [
                    JointTrajectoryPoint(positions=row.tolist()) for row in q]
                at_above = deepcopy(start)
                for index, value in zip(indices, q[0]):
                    at_above.joint_state.position[index] = float(value)
                connection = self.global_move(side, joints=q[0], plan_only=True)
                points = connection.joint_trajectory.points
                if not points:
                    raise PlanningFailure('Empty connection to pre-grasp')
                end = dict(zip(connection.joint_trajectory.joint_names, points[-1].positions))
                if max(abs(end[name]-value) for name, value in zip(names, q[0])) > .001:
                    raise PlanningFailure('Connection did not reach selected pre-grasp joint branch')
            except (PlanningFailure, CartesianPlanningError) as exc:
                failure = str(exc)
                self.n.get_logger().warning(f'Grasp candidate {attempt}/8 rejected: {failure}')
                continue
            prepared = PreparedCartesian(side, at_above, downward, q, frames)
            self.n.get_logger().info('Grasp-first plan selected: executing connection to verified pre-grasp')
            # An execution failure must propagate, never trigger another candidate.
            self.execute(connection)
            return prepared
        raise PlanningFailure(f'Grasp-first planning exhausted 8 seeds: {failure}; no arm motion executed')

    def validate_robot_state(self, robot_state):
        req = GetStateValidity.Request()
        req.robot_state = robot_state
        # Empty group checks the whole robot, including the other arm.
        response = self.call(self.validity, req)
        if not response.valid:
            pairs = sorted({f'{c.contact_body_1} <-> {c.contact_body_2}'
                            for c in response.contacts})
            raise CartesianPlanningError('Invalid state: '+(
                '; '.join(pairs[:8]) or 'joint bounds/constraints; no contacts returned'))

    def execute_prepared_cartesian(self, plan, speed):
        """Execute the selected descent, as a stage of the planned pick sequence.

        Revalidate feedback and scene; do not discard the successful solution
        and invoke a different IK/Cartesian planner after reaching the object.
        """
        if not isinstance(plan, PreparedCartesian):
            raise PlanningFailure('Missing prepared descent; nothing executed')
        current = self.state()
        measured = dict(zip(current.joint_state.name, current.joint_state.position))
        for name, expected in zip(plan.start.joint_state.name, plan.start.joint_state.position):
            tolerance = .0015 if 'finger_joint' in name else .01
            value = measured.get(name, float('nan'))
            if not np.isfinite(value) or abs(value-expected) > tolerance:
                raise PlanningFailure(
                    f'Prepared descent start changed: {name}, expected={expected:.5f}, '
                    f'actual={value:.5f}, tolerance={tolerance:.5f}; nothing executed')
        names = plan.trajectory.joint_trajectory.joint_names
        self.n.get_logger().info(
            f'Revalidating prepared descent: side={plan.side}, points={len(plan.positions)}; '
            'using selected trajectory, no new IK request')
        try:
            self.validate_robot_state(current)
            for point in plan.positions:
                sample = deepcopy(current)
                for name, value in zip(names, point):
                    sample.joint_state.position[sample.joint_state.name.index(name)] = float(value)
                self.validate_robot_state(sample)
        except CartesianPlanningError as exc:
            raise PlanningFailure(f'Prepared descent scene changed: {exc}; nothing executed') from exc
        self.run_cartesian_trajectory(plan.trajectory, plan.positions, plan.frames, speed)

    def seeded_cartesian(self, side, start, target):
        """Use timed, seeded IK when the Cartesian service truncates a short move."""
        names = [f'{side}_j{i}' for i in range(1, 7)]
        indices = [start.joint_state.name.index(name) for name in names]
        def state(q):
            result = deepcopy(start)
            for i, value in zip(indices, q):
                result.joint_state.position[i] = float(value)
            return result

        def fk(robot_state):
            req = GetPositionFK.Request()
            req.header.frame_id = 'world'
            req.fk_link_names = [side+'_gripper_tcp']
            req.robot_state = robot_state
            response = self.call(self.fk, req)
            if response.error_code.val != 1 or len(response.pose_stamped) != 1:
                raise CartesianPlanningError('Fallback FK failed')
            return matrix(response.pose_stamped[0].pose)

        last_report = time.monotonic()
        def inspect(q):
            nonlocal last_report
            robot_state = state(q)
            self.validate_robot_state(robot_state)
            if time.monotonic()-last_report >= 5:
                self.n.get_logger().info('Seeded Cartesian preflight: checking IK and collisions; no motion sent')
                last_report = time.monotonic()
            return fk(robot_state)

        def solve(target_pose, seed):
            req = GetPositionIK.Request()
            req.ik_request.group_name = side+'_arm'
            req.ik_request.ik_link_name = side+'_gripper_tcp'
            req.ik_request.robot_state = state(seed)
            req.ik_request.pose_stamped.header.frame_id = 'world'
            req.ik_request.pose_stamped.pose = pose(target_pose)
            req.ik_request.avoid_collisions = True
            # Some Humble kinematics service releases read timeout.sec only.
            # Use a whole second so the retry budget is not truncated to zero.
            req.ik_request.timeout = duration(1.0)
            req.ik_request.constraints.joint_constraints = [
                JointConstraint(joint_name=name, position=float(value),
                                tolerance_above=self.c['joint_step_limit'],
                                tolerance_below=self.c['joint_step_limit'], weight=1.0)
                for name, value in zip(names, seed)]
            response = self.call(self.ik, req)
            if response.error_code.val != 1:
                self.n.get_logger().warning(
                    f'Seed-local IK code={response.error_code.val}; retrying without '
                    'seed joint constraints, exact TCP and collision checks retained')
                # Remove only the solver's seed window. seeded_path still rejects
                # raw joint jumps and checks all interpolated states before motion.
                req.ik_request.constraints = Constraints()
                response = self.call(self.ik, req)
                if response.error_code.val == 1:
                    values = dict(zip(response.solution.joint_state.name,
                                      response.solution.joint_state.position))
                    jump = max(abs(values[name]-value) for name, value in zip(names, seed))
                    self.n.get_logger().info(
                        f'Collision-aware IK retry solved: raw max_delta={jump:.5f} rad; '
                        f'continuity limit={self.c["joint_step_limit"]:.5f} rad, '
                        'pending full path validation')
            if response.error_code.val != 1:
                # Diagnostic only: a collision-disabled solution is never returned
                # to the planner/executor. It lets us name actual collision pairs.
                error = response.error_code.val
                req.ik_request.avoid_collisions = False
                req.ik_request.constraints = Constraints()
                diagnostic = self.call(self.ik, req)
                detail = 'no diagnostic joint solution'
                if diagnostic.error_code.val == 1:
                    values = dict(zip(diagnostic.solution.joint_state.name,
                                      diagnostic.solution.joint_state.position))
                    self.validate_robot_state(state([values[name] for name in names]))
                    delta = np.array([values[name] for name in names])-seed
                    worst = int(np.argmax(np.abs(delta)))
                    detail = (f'diagnostic state valid; {names[worst]} delta={abs(delta[worst]):.5f} rad, '
                              f'local limit={self.c["joint_step_limit"]:.5f} rad')
                    self.n.get_logger().warning(
                        f'IK diagnostic: state_valid=True, seed={np.round(seed, 5).tolist()}, '
                        f'solution={[round(values[name], 5) for name in names]}, '
                        f'max_delta={abs(delta[worst]):.5f} rad at {names[worst]}, '
                        f'limit={self.c["joint_step_limit"]:.5f} rad')
                raise CartesianPlanningError(
                    f'Seed-local IK failed (code={error}); diagnostic IK code='
                    f'{diagnostic.error_code.val}; {detail}; continuous solution not confirmed')
            values = dict(zip(response.solution.joint_state.name, response.solution.joint_state.position))
            return [values[name] for name in names]

        self.n.get_logger().info(
            'Preflighting seeded IK at <=1 mm, timeout=1 s, '
            'collision checks ON, raw joint jump guard ON')
        try:
            q, frames = seeded_path(
                [start.joint_state.position[i] for i in indices], fk(start), target,
                solve, inspect, step=min(.001, self.c['cartesian_step']),
                joint_limit=self.c['joint_step_limit'])
        except CartesianPlanningError as exc:
            raise PlanningFailure(str(exc)) from exc
        trajectory = RobotTrajectory()
        trajectory.joint_trajectory.joint_names = names
        trajectory.joint_trajectory.points = [JointTrajectoryPoint(positions=row.tolist()) for row in q]
        self.n.get_logger().info(f'Seeded Cartesian preflight complete: fraction=100%, points={len(q)}')
        return trajectory, q, frames

    def cartesian(self, side, waypoints, speed, object_tcp=None, center=None):
        req = GetCartesianPath.Request()
        req.header.frame_id = 'world'
        req.start_state = self.state()
        req.group_name = side+'_arm'
        req.link_name = side+'_gripper_tcp'
        req.waypoints = [pose(t) for t in waypoints]
        req.max_step = self.c['cartesian_step']
        req.jump_threshold = 3.0
        req.revolute_jump_threshold = self.c['joint_step_limit']
        req.avoid_collisions = True
        # MoveIt's Humble GetCartesianPath.srv has no velocity or acceleration
        # scaling fields.  The returned path is retimed below using the
        # configured Cartesian speed and joint limits before execution.
        target = waypoints[-1]
        self.n.get_logger().info(
            f'Cartesian request: side={side}, waypoints={len(waypoints)}, '
            f'step={req.max_step:.4f} m, relative_jump_factor={req.jump_threshold:.2f}, '
            f'revolute_jump={req.revolute_jump_threshold:.2f} rad, '
            f'avoid_collisions={req.avoid_collisions}, '
            f'target=({target[0, 3]:.4f}, {target[1, 3]:.4f}, {target[2, 3]:.4f})')
        response = self.call(self.cart, req, 40)
        trajectory = response.solution
        points = trajectory.joint_trajectory.points
        q = np.array([p.positions for p in points]) if points else np.empty((0, 0))
        max_joint_step = float(np.max(np.abs(np.diff(q, axis=0)))) if len(q) > 1 else 0.0
        self.n.get_logger().info(
            f'Cartesian result: error_code={response.error_code.val}, '
            f'fraction={response.fraction:.1%}, points={len(points)}, '
            f'max_joint_step={max_joint_step:.4f} rad')
        frames = None
        if response.error_code.val == 1 and response.fraction < .99999 and len(waypoints) == 1:
            self.n.get_logger().warning('Cartesian service truncated path; trying seeded IK fallback')
            trajectory, q, frames = self.seeded_cartesian(side, req.start_state, target)
            points = trajectory.joint_trajectory.points
            max_joint_step = float(np.max(np.abs(np.diff(q, axis=0)))) if len(q) > 1 else 0.0
        elif response.error_code.val != 1 or response.fraction < .99999:
            raise PlanningFailure(
                f'Cartesian path incomplete ({response.fraction:.1%}); '
                f'error_code={response.error_code.val}, points={len(points)}, '
                f'max_joint_step={max_joint_step:.4f} rad; nothing executed')
        if len(q) < 2 or max_joint_step > self.c['joint_step_limit']:
            raise PlanningFailure(
                f'Empty path or joint discontinuity; points={len(points)}, '
                f'max_joint_step={max_joint_step:.4f} rad, '
                f'limit={self.c["joint_step_limit"]:.4f} rad')
        if frames is None:
            frames = self.fk_poses(side, trajectory, req.start_state)
        self.run_cartesian_trajectory(trajectory, q, frames, speed, object_tcp, center)

    def run_cartesian_trajectory(self, trajectory, q, frames, speed, object_tcp=None, center=None):
        """Apply the same slow timing and centre guard to fresh or prepared plans."""
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
        self.n.get_logger().info(
            f'Gripper settled: side={side}, requested_gap={width:.4f} m, '
            f'joint_targets=({width/2:.4f}, {width/2:.4f}), '
            f'measured=({measured[0]:.4f}, {measured[1]:.4f})')
        if max(abs(v-width/2) for v in measured) > .0015:
            raise RuntimeError('Gripper position feedback did not reach target')

    def grasp_owner(self):
        response = self.call(self.owner, Trigger.Request())
        if not response.success or response.message not in ('', 'left', 'right'):
            raise RuntimeError('Simulator grasp ownership unavailable; no retry motion')
        return response.message

    def wait_stationary(self):
        """After cancellation, require 0.5 ROS seconds of stable joint feedback."""
        def now():
            return self.n.get_clock().now().nanoseconds*1e-9
        deadline = SimClockDeadline(now(), time.monotonic(), 10.0, 30.0)
        previous = self.state().joint_state
        reference = np.array(previous.position)
        limits = np.array([.0003 if 'finger_joint' in n else .002 for n in previous.name])
        stable_since = now()
        while True:
            self.check()
            current_time = now()
            deadline.check(current_time, time.monotonic())
            current = self.state().joint_state
            values = dict(zip(current.name, current.position))
            measured = np.array([values[n] for n in previous.name])
            if not np.all(np.isfinite(measured)):
                raise RuntimeError('Invalid joint feedback while waiting for stop')
            if np.any(np.abs(measured-reference) > limits):
                reference, stable_since = measured, current_time
            if current_time-stable_since >= .5:
                return
            self.n.stop_event.wait(.05)

    def assisted_grasp(self, side, close=True):
        res = self.call(self.grasps[side], SetBool.Request(data=close), 5)
        if not res.success:
            raise RuntimeError('Assisted grasp refused: '+res.message)
        status = self.call(self.owner, Trigger.Request())
        if close and status.message != side:
            raise RuntimeError('Simulator grasp ownership not confirmed')

    def truth(self):
        # Deliberately isolated from estimate_bolt and from pick pose generation.
        request = GetEntityState.Request(name=self.c['simulation_entity'], reference_frame='world')
        client = self._available_service((self.entity, getattr(self, 'entity_fallback', None)))
        res = self.call(client, request)
        if not res.success:
            raise RuntimeError('Simulation verification state unavailable')
        return matrix(res.state.pose)

    def relocate_object(self, world_object):
        request = SetEntityState.Request()
        request.state = EntityState()
        request.state.name = self.c['simulation_entity']
        request.state.reference_frame = 'world'
        request.state.pose = pose(world_object)
        # A default Twist explicitly removes velocity left by a failed pick.
        client = self._available_service((self.set_entity, getattr(self, 'set_entity_fallback', None)))
        response = self.call(client, request)
        if not response.success:
            raise RuntimeError('Gazebo refused object relocation')
        self.object_scene(world_object)

    def _available_service(self, clients):
        """Return the first advertised service, including Gazebo global fallback."""
        for client in clients:
            if client is None:
                continue
            if not hasattr(client, 'wait_for_service'):
                # Lightweight test doubles and custom adapters are already
                # selected by their caller; do not probe them as ROS clients.
                return client
            if client.wait_for_service(timeout_sec=.25):
                return client
        names = ', '.join(getattr(c, 'srv_name', '<unknown>') for c in clients)
        raise RuntimeError('Service unavailable: '+names)

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
