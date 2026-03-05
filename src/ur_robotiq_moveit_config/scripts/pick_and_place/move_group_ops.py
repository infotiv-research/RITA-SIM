"""
MoveGroup request builder and execution helper.

Responsibilities:
- Build pose and joint-space MotionPlanRequest goals.
- Send MoveGroup goals and evaluate result codes.
- Apply orientation fallback attempts and home-joint motion.
"""

import math
import time
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import Pose, PoseStamped
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (
    Constraints,
    JointConstraint,
    MoveItErrorCodes,
    MotionPlanRequest,
    OrientationConstraint,
    PlanningOptions,
    PositionConstraint,
)
from shape_msgs.msg import SolidPrimitive

from .constants import (
    END_EFFECTOR_LINK,
    HOME_JOINT_VALUES,
    JOINT_NAMES,
    PLANNING_GROUP,
)


class MoveGroupOpsMixin:
    @staticmethod
    def _wait_future_result(future, timeout_sec=30.0, poll_period_sec=0.01):
        """Wait for an async future without spinning a second executor."""
        deadline = time.monotonic() + float(timeout_sec)
        while time.monotonic() < deadline:
            if future.done():
                try:
                    return future.result()
                except Exception:
                    return None
            time.sleep(float(poll_period_sec))
        return None

    def _configure_request_pipeline(self, request, pipeline_override=None):
        """Set a planner pipeline hint in MotionPlanRequest when supported."""
        requested_pipeline = (
            pipeline_override
            if pipeline_override is not None
            else getattr(self, "planning_pipeline", "cumotion")
        )
        requested_pipeline = str(requested_pipeline).strip().lower()
        pipeline_map = {
            "cumotion": "isaac_ros_cumotion",
            "ompl": "ompl",
        }
        pipeline_id = pipeline_map.get(requested_pipeline)
        if pipeline_id is None:
            self.get_logger().warn(
                f"Unknown planning_pipeline '{requested_pipeline}', leaving MoveIt pipeline unset."
            )
            return

        if hasattr(request, "pipeline_id"):
            request.pipeline_id = pipeline_id
        elif requested_pipeline != "cumotion":
            self.get_logger().warn(
                "This MoveIt interface does not expose MotionPlanRequest.pipeline_id; "
                "cannot force OMPL from this node."
            )

    def _resolve_move_group_replan_attempts(self):
        """Return sanitized move_group replan attempt count."""
        configured_attempts = getattr(self, "move_group_replan_attempts", 3)
        try:
            attempts = int(configured_attempts)
        except (TypeError, ValueError):
            self.get_logger().warn(
                f"Invalid move_group_replan_attempts='{configured_attempts}', using 3."
            )
            attempts = 3
        if attempts < 1:
            self.get_logger().warn(
                f"move_group_replan_attempts={attempts} is invalid, clamping to 1."
            )
            attempts = 1
        return attempts

    def _moveit_error_name(self, error_code):
        """Return symbolic MoveIt error name for an integer error code."""
        if not hasattr(self, "_moveit_error_name_map"):
            mapping = {}
            for name in dir(MoveItErrorCodes):
                if not name.isupper():
                    continue
                value = getattr(MoveItErrorCodes, name)
                if isinstance(value, int):
                    mapping[int(value)] = name
            self._moveit_error_name_map = mapping
        return self._moveit_error_name_map.get(int(error_code), "UNKNOWN_ERROR_CODE")

    @staticmethod
    def _goal_status_name(status_code):
        """Return symbolic action status name for a GoalStatus code."""
        mapping = {
            GoalStatus.STATUS_UNKNOWN: "STATUS_UNKNOWN",
            GoalStatus.STATUS_ACCEPTED: "STATUS_ACCEPTED",
            GoalStatus.STATUS_EXECUTING: "STATUS_EXECUTING",
            GoalStatus.STATUS_CANCELING: "STATUS_CANCELING",
            GoalStatus.STATUS_SUCCEEDED: "STATUS_SUCCEEDED",
            GoalStatus.STATUS_CANCELED: "STATUS_CANCELED",
            GoalStatus.STATUS_ABORTED: "STATUS_ABORTED",
        }
        return mapping.get(int(status_code), "STATUS_INVALID")

    @staticmethod
    def _duration_to_seconds(duration_msg):
        """Convert builtin_interfaces/Duration to float seconds."""
        if duration_msg is None:
            return None
        sec = getattr(duration_msg, "sec", None)
        nanosec = getattr(duration_msg, "nanosec", None)
        if sec is None or nanosec is None:
            return None
        return float(sec) + float(nanosec) * 1e-9

    @staticmethod
    def _trajectory_point_count(robot_trajectory):
        """Best-effort joint-trajectory point count from RobotTrajectory."""
        if robot_trajectory is None:
            return 0
        joint_traj = getattr(robot_trajectory, "joint_trajectory", None)
        if joint_traj is None:
            return 0
        points = getattr(joint_traj, "points", None)
        if points is None:
            return 0
        return len(points)

    def _extract_pose_goal_debug_summary(self, goal):
        """Extract target pose/tolerances from a pose goal for diagnostics."""
        summary = {
            "request_frame": None,
            "target_pose_request": None,
            "position_tolerance_m": None,
            "orientation_tolerance_xyz": None,
        }
        if goal is None or getattr(goal, "request", None) is None:
            return summary

        request = goal.request
        workspace_header = getattr(request, "workspace_parameters", None)
        if workspace_header is not None:
            header = getattr(workspace_header, "header", None)
            if header is not None:
                summary["request_frame"] = getattr(header, "frame_id", None)

        goal_constraints = getattr(request, "goal_constraints", None) or []
        if not goal_constraints:
            return summary
        constraints = goal_constraints[0]

        pos_constraints = getattr(constraints, "position_constraints", None) or []
        if pos_constraints:
            pos_constraint = pos_constraints[0]
            region = getattr(pos_constraint, "constraint_region", None)
            if region is not None:
                region_poses = getattr(region, "primitive_poses", None) or []
                if region_poses:
                    p = region_poses[0]
                    summary["target_pose_request"] = (
                        float(p.position.x),
                        float(p.position.y),
                        float(p.position.z),
                        None,
                        None,
                        None,
                        None,
                    )
                primitives = getattr(region, "primitives", None) or []
                if primitives:
                    dims = getattr(primitives[0], "dimensions", None) or []
                    if dims:
                        summary["position_tolerance_m"] = float(dims[0])
            if not summary["request_frame"]:
                header = getattr(pos_constraint, "header", None)
                if header is not None:
                    summary["request_frame"] = getattr(header, "frame_id", None)

        orient_constraints = (
            getattr(constraints, "orientation_constraints", None) or []
        )
        if orient_constraints:
            oc = orient_constraints[0]
            q = oc.orientation
            pose = summary["target_pose_request"]
            if pose is None:
                summary["target_pose_request"] = (
                    None,
                    None,
                    None,
                    float(q.x),
                    float(q.y),
                    float(q.z),
                    float(q.w),
                )
            else:
                summary["target_pose_request"] = (
                    pose[0],
                    pose[1],
                    pose[2],
                    float(q.x),
                    float(q.y),
                    float(q.z),
                    float(q.w),
                )
            summary["orientation_tolerance_xyz"] = (
                float(oc.absolute_x_axis_tolerance),
                float(oc.absolute_y_axis_tolerance),
                float(oc.absolute_z_axis_tolerance),
            )
            if not summary["request_frame"]:
                header = getattr(oc, "header", None)
                if header is not None:
                    summary["request_frame"] = getattr(header, "frame_id", None)
        return summary

    def _pose_only(self, pose_tuple):
        if pose_tuple is None:
            return None
        if len(pose_tuple) < 3:
            return None
        if pose_tuple[0] is None or pose_tuple[1] is None or pose_tuple[2] is None:
            return None
        return (float(pose_tuple[0]), float(pose_tuple[1]), float(pose_tuple[2]))

    def _quat_only(self, pose_tuple):
        if pose_tuple is None or len(pose_tuple) < 7:
            return None
        if (
            pose_tuple[3] is None
            or pose_tuple[4] is None
            or pose_tuple[5] is None
            or pose_tuple[6] is None
        ):
            return None
        return (
            float(pose_tuple[3]),
            float(pose_tuple[4]),
            float(pose_tuple[5]),
            float(pose_tuple[6]),
        )

    def _distance(self, p0, p1):
        if p0 is None or p1 is None:
            return None
        dx = float(p0[0]) - float(p1[0])
        dy = float(p0[1]) - float(p1[1])
        dz = float(p0[2]) - float(p1[2])
        return math.sqrt(dx * dx + dy * dy + dz * dz)

    def _quat_angular_distance_rad(self, q0, q1):
        if q0 is None or q1 is None:
            return None
        q0x, q0y, q0z, q0w = self._normalize_quaternion(*q0)
        q1x, q1y, q1z, q1w = self._normalize_quaternion(*q1)
        dot = abs(q0x * q1x + q0y * q1y + q0z * q1z + q0w * q1w)
        dot = max(0.0, min(1.0, dot))
        return 2.0 * math.acos(dot)

    def _sample_frame_jitter(self, source_frame, target_frame, samples=4):
        """Sample transform repeatedly and report max translation/orientation spread."""
        captured = []
        for _ in range(max(1, int(samples))):
            pose = self._transform_pose_between_frames(
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                1.0,
                source_frame,
                target_frame,
            )
            if pose is not None:
                captured.append(pose)
            time.sleep(0.02)

        if len(captured) < 2:
            return None

        base_pose = captured[0]
        base_xyz = self._pose_only(base_pose)
        base_q = self._quat_only(base_pose)
        max_pos = 0.0
        max_ang = 0.0
        for pose in captured[1:]:
            pos_delta = self._distance(base_xyz, self._pose_only(pose))
            if pos_delta is not None:
                max_pos = max(max_pos, pos_delta)
            ang_delta = self._quat_angular_distance_rad(base_q, self._quat_only(pose))
            if ang_delta is not None:
                max_ang = max(max_ang, ang_delta)
        return (max_pos, max_ang)

    def _log_failure_hints(self, attempt_id, error_name, debug_meta):
        """Emit likely-cause hints for failed pose plans."""
        request_frame = str(
            (debug_meta or {}).get("request_frame")
            or getattr(self, "goal_request_frame", self.world_frame)
        )
        target_pose_request = (debug_meta or {}).get("target_pose_request")
        target_pos = self._pose_only(target_pose_request)
        target_q = self._quat_only(target_pose_request)
        pos_tol = (debug_meta or {}).get("position_tolerance_m")
        orient_tol_xyz = (debug_meta or {}).get("orientation_tolerance_xyz")

        ee_pose_request = self._transform_pose_between_frames(
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            1.0,
            END_EFFECTOR_LINK,
            request_frame,
        )
        ee_pos = self._pose_only(ee_pose_request)
        ee_q = self._quat_only(ee_pose_request)

        pos_delta = self._distance(ee_pos, target_pos)
        ang_delta = self._quat_angular_distance_rad(ee_q, target_q)
        if pos_delta is not None:
            self.get_logger().warn(
                f"[{attempt_id}] EE->goal position delta in {request_frame}: {pos_delta:.4f} m "
                f"(position tolerance={pos_tol if pos_tol is not None else 'n/a'})."
            )
        if ang_delta is not None:
            self.get_logger().warn(
                f"[{attempt_id}] EE->goal orientation delta in {request_frame}: {ang_delta:.4f} rad "
                f"(orientation tolerance xyz={orient_tol_xyz if orient_tol_xyz is not None else 'n/a'})."
            )

        jitter = self._sample_frame_jitter(self.world_frame, request_frame, samples=4)
        if jitter is not None:
            jitter_pos, jitter_ang = jitter
            self.get_logger().warn(
                f"[{attempt_id}] Frame jitter sample world->{request_frame}: "
                f"max_pos={jitter_pos:.6f} m, max_ang={jitter_ang:.6f} rad."
            )

        target_obj = (debug_meta or {}).get("target_object_id")
        if target_obj:
            world_obj_present = None
            if hasattr(self, "_get_world_collision_object"):
                world_obj_present = self._get_world_collision_object(target_obj) is not None
            self.get_logger().warn(
                f"[{attempt_id}] Collision state: "
                f"target_object={target_obj}, "
                f"attached_id={getattr(self, '_attached_object_id', None)}, "
                f"target_suppressed={getattr(self, '_target_object_suppressed', None)}, "
                f"cumotion_attached={getattr(self, '_cumotion_object_attached', None)}, "
                f"world_object_present={world_obj_present}."
            )

        hints = []
        if error_name in (
            "NO_IK_SOLUTION",
            "GOAL_CONSTRAINTS_VIOLATED",
            "INVALID_GOAL_CONSTRAINTS",
            "PLANNING_FAILED",
        ):
            hints.append(
                "Goal may be near reach/orientation feasibility boundary for current start state."
            )
        if ang_delta is not None and orient_tol_xyz is not None:
            max_tol = max(float(orient_tol_xyz[0]), float(orient_tol_xyz[1]), float(orient_tol_xyz[2]))
            if ang_delta > max_tol:
                hints.append(
                    "Current end-effector orientation gap exceeds configured orientation tolerance."
                )
        if jitter is not None and (jitter[0] > 0.003 or jitter[1] > 0.03):
            hints.append(
                "Detected non-trivial frame jitter; check TF stability and frame/tool consistency."
            )
        if target_obj and getattr(self, "_attached_object_id", None) == str(target_obj):
            if hasattr(self, "_get_world_collision_object"):
                obj = self._get_world_collision_object(target_obj)
                if obj is not None:
                    hints.append(
                        "Target object appears both attached and in world collisions; duplicate collision geometry can block IK."
                    )
        if (
            target_obj
            and getattr(self, "_target_object_suppressed", None) is False
            and str((debug_meta or {}).get("stage", "")) == "dropoff"
        ):
            hints.append(
                "Dropoff planned while target world object was not suppressed; environment collision may over-constrain IK."
            )

        if hints:
            for hint in hints:
                self.get_logger().warn(f"[{attempt_id}] Possible cause: {hint}")
        else:
            self.get_logger().warn(
                f"[{attempt_id}] No single dominant cause detected from local diagnostics."
            )

    def _send_move_group_goal(
        self,
        goal,
        context_label=None,
        debug_meta=None,
        run_failure_diagnostics=False,
    ):
        """Send a MoveGroup goal and wait for result."""
        context = str(context_label) if context_label else "unlabeled_goal"
        self.get_logger().info(f"Sending MoveGroup goal [{context}]...")
        future = self.move_group_client.send_goal_async(goal)
        goal_handle = self._wait_future_result(future, timeout_sec=30.0)
        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().error(f"MoveGroup goal [{context}] was rejected!")
            return False

        self.get_logger().info(
            f"MoveGroup goal accepted [{context}], waiting for result..."
        )
        result_future = goal_handle.get_result_async()
        result = self._wait_future_result(result_future, timeout_sec=120.0)
        if result is None:
            self.get_logger().error(f"MoveGroup returned no result for [{context}]!")
            return False

        error_code = int(result.result.error_code.val)
        error_name = self._moveit_error_name(error_code)
        status_code = int(getattr(result, "status", GoalStatus.STATUS_UNKNOWN))
        status_name = self._goal_status_name(status_code)
        planning_time = self._duration_to_seconds(
            getattr(result.result, "planning_time", None)
        )
        planned_points = self._trajectory_point_count(
            getattr(result.result, "planned_trajectory", None)
        )
        executed_points = self._trajectory_point_count(
            getattr(result.result, "executed_trajectory", None)
        )

        if error_code == 1:  # SUCCESS
            if planning_time is not None:
                self.get_logger().info(
                    f"MoveGroup motion succeeded [{context}] "
                    f"(status={status_name}, planning_time={planning_time:.3f}s, "
                    f"planned_points={planned_points}, executed_points={executed_points})."
                )
            else:
                self.get_logger().info(
                    f"MoveGroup motion succeeded [{context}] "
                    f"(status={status_name}, planned_points={planned_points}, "
                    f"executed_points={executed_points})."
                )
            return True

        self.get_logger().error(
            f"MoveGroup failed [{context}] with error_code={error_code} ({error_name}), "
            f"status={status_name}, planned_points={planned_points}, "
            f"executed_points={executed_points}."
        )
        if run_failure_diagnostics:
            meta = dict(debug_meta or {})
            meta.setdefault("stage", "unknown")
            meta.setdefault("attempt_id", context)
            pose_summary = self._extract_pose_goal_debug_summary(goal)
            for key, value in pose_summary.items():
                if key not in meta or meta[key] is None:
                    meta[key] = value
            self._log_failure_hints(
                str(meta.get("attempt_id", context)),
                error_name=error_name,
                debug_meta=meta,
            )
        return False

    def _build_pose_goal(
        self,
        x,
        y,
        z,
        qx,
        qy,
        qz,
        qw,
        planning_time=None,
        num_attempts=None,
        constrain_orientation=True,
        orientation_tolerance=None,
        orientation_tolerance_xyz=None,
        position_tolerance_m=None,
    ):
        """Build a MoveGroup.Goal for a pose target with a specified EE orientation."""
        request_frame = self.world_frame
        transformed_pose = self._transform_pose_between_frames(
            x,
            y,
            z,
            qx,
            qy,
            qz,
            qw,
            self.world_frame,
            self.goal_request_frame,
        )
        if transformed_pose is not None:
            x, y, z, qx, qy, qz, qw = transformed_pose
            request_frame = self.goal_request_frame
        else:
            self.get_logger().warn(
                "Using world-frame goal because transform to goal_request_frame failed."
            )
        self.get_logger().info(
            f"Goal request frame={request_frame}, pose=({x:.3f}, {y:.3f}, {z:.3f})"
        )

        target_pose = PoseStamped()
        target_pose.header.frame_id = request_frame
        target_pose.header.stamp = self.get_clock().now().to_msg()
        target_pose.pose.position.x = x
        target_pose.pose.position.y = y
        target_pose.pose.position.z = z
        target_pose.pose.orientation.x = qx
        target_pose.pose.orientation.y = qy
        target_pose.pose.orientation.z = qz
        target_pose.pose.orientation.w = qw

        request = MotionPlanRequest()
        request.group_name = PLANNING_GROUP
        self._configure_request_pipeline(request)
        request.num_planning_attempts = num_attempts or self.num_planning_attempts
        request.allowed_planning_time = planning_time or self.planning_time
        request.max_velocity_scaling_factor = self.max_velocity_scaling
        request.max_acceleration_scaling_factor = self.max_acceleration_scaling

        request.workspace_parameters.header.frame_id = request_frame
        request.workspace_parameters.min_corner.x = -3.0
        request.workspace_parameters.min_corner.y = -3.0
        request.workspace_parameters.min_corner.z = -0.5
        request.workspace_parameters.max_corner.x = 3.0
        request.workspace_parameters.max_corner.y = 3.0
        request.workspace_parameters.max_corner.z = 3.0

        constraints = Constraints()

        pos_constraint = PositionConstraint()
        pos_constraint.header = target_pose.header
        pos_constraint.link_name = END_EFFECTOR_LINK
        pos_constraint.target_point_offset.x = 0.0
        pos_constraint.target_point_offset.y = 0.0
        pos_constraint.target_point_offset.z = 0.0

        bounding_box = SolidPrimitive()
        bounding_box.type = SolidPrimitive.SPHERE
        pos_tol = (
            float(position_tolerance_m)
            if position_tolerance_m is not None
            else float(self.position_tolerance_m)
        )
        bounding_box.dimensions = [pos_tol]
        pos_constraint.constraint_region.primitives.append(bounding_box)

        region_pose = Pose()
        region_pose.position.x = x
        region_pose.position.y = y
        region_pose.position.z = z
        region_pose.orientation.w = 1.0
        pos_constraint.constraint_region.primitive_poses.append(region_pose)
        pos_constraint.weight = 1.0
        constraints.position_constraints.append(pos_constraint)

        if constrain_orientation:
            if orientation_tolerance_xyz is not None:
                tol_x, tol_y, tol_z = (
                    float(orientation_tolerance_xyz[0]),
                    float(orientation_tolerance_xyz[1]),
                    float(orientation_tolerance_xyz[2]),
                )
            else:
                tol = (
                    float(orientation_tolerance)
                    if orientation_tolerance is not None
                    else float(self.orientation_tolerance_rad)
                )
                tol_x = tol
                tol_y = tol
                tol_z = tol
            orient_constraint = OrientationConstraint()
            orient_constraint.header = target_pose.header
            orient_constraint.link_name = END_EFFECTOR_LINK
            orient_constraint.orientation = target_pose.pose.orientation
            orient_constraint.absolute_x_axis_tolerance = tol_x
            orient_constraint.absolute_y_axis_tolerance = tol_y
            orient_constraint.absolute_z_axis_tolerance = tol_z
            orient_constraint.weight = 1.0
            constraints.orientation_constraints.append(orient_constraint)

        request.goal_constraints.append(constraints)

        goal = MoveGroup.Goal()
        goal.request = request
        goal.planning_options = PlanningOptions()
        goal.planning_options.plan_only = False
        goal.planning_options.look_around = False
        replan_attempts = self._resolve_move_group_replan_attempts()
        goal.planning_options.replan = replan_attempts > 1
        goal.planning_options.replan_attempts = replan_attempts

        return goal

    def _build_joint_goal(
        self,
        joint_targets,
        planning_time=None,
        num_attempts=None,
        pipeline_override=None,
    ):
        """Build a MoveGroup.Goal for a joint-space target."""
        request = MotionPlanRequest()
        request.group_name = PLANNING_GROUP
        self._configure_request_pipeline(request, pipeline_override=pipeline_override)
        request.num_planning_attempts = num_attempts or self.num_planning_attempts
        request.allowed_planning_time = planning_time or self.planning_time
        request.max_velocity_scaling_factor = self.max_velocity_scaling
        request.max_acceleration_scaling_factor = self.max_acceleration_scaling

        request.workspace_parameters.header.frame_id = self.world_frame
        request.workspace_parameters.min_corner.x = -3.0
        request.workspace_parameters.min_corner.y = -3.0
        request.workspace_parameters.min_corner.z = -0.5
        request.workspace_parameters.max_corner.x = 3.0
        request.workspace_parameters.max_corner.y = 3.0
        request.workspace_parameters.max_corner.z = 3.0

        constraints = Constraints()
        tol = float(self.home_joint_tolerance_rad)
        for joint_name in JOINT_NAMES:
            joint_constraint = JointConstraint()
            joint_constraint.joint_name = joint_name
            joint_constraint.position = float(joint_targets[joint_name])
            joint_constraint.tolerance_above = tol
            joint_constraint.tolerance_below = tol
            joint_constraint.weight = 1.0
            constraints.joint_constraints.append(joint_constraint)

        request.goal_constraints.append(constraints)

        goal = MoveGroup.Goal()
        goal.request = request
        goal.planning_options = PlanningOptions()
        goal.planning_options.plan_only = False
        goal.planning_options.look_around = False
        replan_attempts = self._resolve_move_group_replan_attempts()
        goal.planning_options.replan = replan_attempts > 1
        goal.planning_options.replan_attempts = replan_attempts
        return goal

    @staticmethod
    def _wrapped_angle_error_rad(target_rad, measured_rad):
        """Return shortest angular distance in radians."""
        delta = float(measured_rad) - float(target_rad)
        wrapped = (delta + math.pi) % (2.0 * math.pi) - math.pi
        return abs(wrapped)

    def _wait_for_recent_joint_positions(self, timeout_sec=1.0, max_age_sec=0.5):
        """Wait briefly for a fresh joint-state snapshot and return name->position."""
        deadline = time.monotonic() + float(timeout_sec)
        while time.monotonic() < deadline:
            latest = getattr(self, "_last_joint_positions", None)
            latest_time = float(getattr(self, "_last_joint_positions_msg_time", 0.0))
            age = time.monotonic() - latest_time
            if latest and age <= float(max_age_sec):
                return dict(latest)
            time.sleep(0.05)
        latest = getattr(self, "_last_joint_positions", None)
        if latest:
            return dict(latest)
        return None

    def _verify_home_joint_result_warn_only(self):
        """Compare achieved joints to HOME target and only warn on mismatch."""
        measured = self._wait_for_recent_joint_positions(timeout_sec=1.0, max_age_sec=0.5)
        if measured is None:
            self.get_logger().warn(
                "HOME verification skipped: no recent /joint_states sample available."
            )
            return

        target = self._home_joint_values or {
            joint_name: float(HOME_JOINT_VALUES[joint_name]) for joint_name in JOINT_NAMES
        }
        tolerance = float(
            getattr(
                self,
                "home_verify_joint_tolerance_rad_m",
                self.home_joint_tolerance_rad,
            )
        )
        missing_joints = []
        per_joint_errors = []
        max_error = -1.0
        max_error_joint = None

        for joint_name in JOINT_NAMES:
            if joint_name not in measured:
                missing_joints.append(joint_name)
                continue
            target_value = float(target[joint_name])
            measured_value = float(measured[joint_name])
            if joint_name == "gantry_joint":
                error = abs(measured_value - target_value)
            else:
                error = self._wrapped_angle_error_rad(target_value, measured_value)
            per_joint_errors.append((joint_name, target_value, measured_value, error))
            if error > max_error:
                max_error = error
                max_error_joint = joint_name

        if missing_joints:
            self.get_logger().warn(
                "HOME verification: missing joints in latest /joint_states: "
                f"{', '.join(missing_joints)}"
            )

        if not per_joint_errors:
            self.get_logger().warn(
                "HOME verification skipped: no manipulator joints were present in /joint_states."
            )
            return

        failed = [item for item in per_joint_errors if item[3] > tolerance]
        details = ", ".join(
            f"{joint}=|err|{error:.4f}"
            for joint, _, _, error in per_joint_errors
        )

        if failed:
            failed_joints = ", ".join(
                f"{joint}({error:.4f})" for joint, _, _, error in failed
            )
            self.get_logger().warn(
                "HOME verification mismatch (warn-only): "
                f"tolerance={tolerance:.4f} rad/m, "
                f"max_error={max_error:.4f} on {max_error_joint}. "
                f"Outside tolerance: {failed_joints}. "
                f"Per-joint errors: {details}"
            )
        else:
            self.get_logger().info(
                "HOME verification passed: "
                f"all joints within tolerance={tolerance:.4f} rad/m. "
                f"max_error={max_error:.4f} on {max_error_joint}. "
                f"Per-joint errors: {details}"
            )

    def _move_to_home(self):
        """Move robot to hardcoded home joint target."""
        if self._home_joint_values is None:
            missing = [
                joint_name
                for joint_name in JOINT_NAMES
                if joint_name not in HOME_JOINT_VALUES
            ]
            if missing:
                self.get_logger().error(
                    "HOME_JOINT_VALUES is missing manipulator joints: "
                    f"{', '.join(missing)}"
                )
                return False
            self._home_joint_values = {
                joint_name: float(HOME_JOINT_VALUES[joint_name])
                for joint_name in JOINT_NAMES
            }

        self.get_logger().info(
            "Planning HOME with OMPL pipeline for exact joint-target behavior."
        )
        goal = self._build_joint_goal(
            self._home_joint_values,
            pipeline_override="ompl",
        )
        success = self._send_move_group_goal(goal, context_label="home_joint_goal_ompl")
        if success:
            self._verify_home_joint_result_warn_only()
        return success

    def _move_to_pose(
        self, x, y, z, qx, qy, qz, qw, planning_time=None, num_attempts=None
    ):
        """Plan and execute a move to pose with cuMotion-friendly fallback behavior."""
        orientation_attempts = [("target orientation", (qx, qy, qz, qw))]
        current_tool_orientation = self._lookup_end_effector_orientation()
        if current_tool_orientation is not None:
            orientation_attempts.append(
                ("current EE orientation", current_tool_orientation)
            )

        tried_orientations = []
        tolerance_attempts = [
            ("strict", float(self.orientation_tolerance_rad)),
            ("relaxed", 1.57),
        ]
        for label, (aqx, aqy, aqz, aqw) in orientation_attempts:
            if any(
                self._quaternions_equivalent((aqx, aqy, aqz, aqw), seen_q)
                for seen_q in tried_orientations
            ):
                continue
            tried_orientations.append((aqx, aqy, aqz, aqw))
            for tol_label, tol in tolerance_attempts:
                self.get_logger().info(
                    f"Pose planning attempt mode: {label} ({tol_label} tolerance)"
                )
                goal = self._build_pose_goal(
                    x,
                    y,
                    z,
                    aqx,
                    aqy,
                    aqz,
                    aqw,
                    planning_time,
                    num_attempts,
                    constrain_orientation=True,
                    orientation_tolerance=tol,
                )
                if self._send_move_group_goal(goal):
                    return True

        # Upstream cuMotion rejects position-only goal constraints and can crash.
        # Keep this disabled by default; it can still be useful when using OMPL.
        if self.enable_position_only_fallback:
            self.get_logger().warn(
                "Trying position-only fallback. This is intended for OMPL; "
                "upstream cuMotion may reject it."
            )
            goal = self._build_pose_goal(
                x,
                y,
                z,
                qx,
                qy,
                qz,
                qw,
                planning_time,
                num_attempts,
                constrain_orientation=False,
                orientation_tolerance=None,
            )
            if self._send_move_group_goal(goal):
                return True
        return False
