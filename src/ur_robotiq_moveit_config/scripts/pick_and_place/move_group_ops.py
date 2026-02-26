"""
MoveGroup request builder and execution helper.

Responsibilities:
- Build pose and joint-space MotionPlanRequest goals.
- Send MoveGroup goals and evaluate result codes.
- Apply orientation fallback attempts and home-joint motion.
"""

import time
from geometry_msgs.msg import Pose, PoseStamped
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (
    Constraints,
    JointConstraint,
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

    def _configure_request_pipeline(self, request):
        """Set a planner pipeline hint in MotionPlanRequest when supported."""
        requested_pipeline = getattr(self, "planning_pipeline", "cumotion")
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

    def _send_move_group_goal(self, goal):
        """Send a MoveGroup goal and wait for result. Returns (success, error_code)."""
        self.get_logger().info("Sending MoveGroup goal...")
        future = self.move_group_client.send_goal_async(goal)
        goal_handle = self._wait_future_result(future, timeout_sec=30.0)
        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().error("MoveGroup goal was rejected!")
            return False

        self.get_logger().info("MoveGroup goal accepted, waiting for result...")
        result_future = goal_handle.get_result_async()
        result = self._wait_future_result(result_future, timeout_sec=120.0)
        if result is None:
            self.get_logger().error("MoveGroup returned no result!")
            return False

        error_code = result.result.error_code.val
        if error_code == 1:  # SUCCESS
            self.get_logger().info("MoveGroup motion succeeded.")
            return True

        self.get_logger().error(f"MoveGroup failed with error code: {error_code}")
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

    def _build_joint_goal(self, joint_targets, planning_time=None, num_attempts=None):
        """Build a MoveGroup.Goal for a joint-space target."""
        request = MotionPlanRequest()
        request.group_name = PLANNING_GROUP
        self._configure_request_pipeline(request)
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

        goal = self._build_joint_goal(self._home_joint_values)
        return self._send_move_group_goal(goal)

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
