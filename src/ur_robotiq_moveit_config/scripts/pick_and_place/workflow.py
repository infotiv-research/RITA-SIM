"""
Main node workflow and orchestration for target-object pick-and-place.

Responsibilities:
- Declare/read runtime parameters and initialize ROS interfaces.
- Execute the full pick->grasp->attach->release->home sequence.
- Coordinate TF/math, planning, scene, and gripper helpers.
"""

from collections import deque
import json
import math
import threading
import time

import rclpy
from control_msgs.action import FollowJointTrajectory
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import PlanningScene
from moveit_msgs.srv import GetPlanningScene
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformListener

from rclpy.duration import Duration
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from .constants import (
    ARM_TRAJECTORY_ACTION,
    END_EFFECTOR_LINK,
    GRIPPER_TRAJECTORY_ACTION,
    HOME_JOINT_VALUES,
    JOINT_NAMES,
)
from .backend_curobo import CuroboMotionBackend
from .backend_hybrid import HybridMotionBackend
from .backend_moveit import MoveItMotionBackend
from .cumotion_attachment_ops import CumotionAttachmentOpsMixin
from .curobo_object_tracking import CuroboObjectTracker
from .gripper_control import GripperControlMixin
from .math_tf_utils import MathTfMixin
from .move_group_ops import MoveGroupOpsMixin
from .planning_scene_ops import PlanningSceneOpsMixin

try:
    from curobo_msgs.srv import AddObject, RemoveObject, SetLinkCollision
except ImportError:
    AddObject = None
    RemoveObject = None
    SetLinkCollision = None


class PickAndPlaceTargetObject(
    Node,
    GripperControlMixin,
    MathTfMixin,
    PlanningSceneOpsMixin,
    MoveGroupOpsMixin,
    CumotionAttachmentOpsMixin,
):
    def __init__(self):
        super().__init__("pick_and_place")

        # Parameters
        self.declare_parameter("target_object_id", "rubiks_cube")
        self.declare_parameter("target_object_frame", "rubiks_cube")
        self.declare_parameter("assets_root", "")
        self.declare_parameter("world_frame", "world")
        self.declare_parameter("pre_grasp_offset_z", 0.15)
        self.declare_parameter("planning_time", 10.0)
        self.declare_parameter("max_velocity_scaling", 0.3)
        self.declare_parameter("max_acceleration_scaling", 0.3)
        self.declare_parameter("num_planning_attempts", 10)
        self.declare_parameter("motion_backend", "moveit")
        self.declare_parameter("planning_pipeline", "cumotion")
        self.declare_parameter("curobo_planner_type", "classic")
        self.declare_parameter("hybrid_precision_descent_enabled", True)
        self.declare_parameter("hybrid_precision_descent_planner_type", "classic")
        self.declare_parameter("hybrid_precision_descent_grasp_enabled", True)
        self.declare_parameter("hybrid_precision_descent_dropoff_enabled", True)
        self.declare_parameter("hybrid_precision_descent_planning_time_s", 0.0)
        self.declare_parameter("hybrid_precision_descent_num_attempts", 0)
        self.declare_parameter("curobo_carried_object_sphere_count", 3)
        self.declare_parameter("grasp_offset_z", 0.00)
        self.declare_parameter("post_grasp_lift_z", 0.08)
        self.declare_parameter("grasp_retry_count", 4)
        self.declare_parameter("grasp_retry_step_z", 0.005)
        self.declare_parameter("position_tolerance_m", 0.01)
        self.declare_parameter("orientation_tolerance_rad", 0.1)
        self.declare_parameter("move_group_replan_attempts", 1)
        self.declare_parameter("enable_position_only_fallback", False)
        self.declare_parameter("goal_request_frame", "world")
        self.declare_parameter("gripper_close_position", 0.6)
        self.declare_parameter("post_grasp_settle_s", 0.5)
        self.declare_parameter("home_joint_tolerance_rad", 0.01)
        self.declare_parameter("home_verify_joint_tolerance_rad_m", 0.03)
        self.declare_parameter("gripper_result_timeout_s", 30.0)
        self.declare_parameter("close_early_success_wait_s", 2.0)
        self.declare_parameter("close_success_min_position", 0.2)
        self.declare_parameter("close_success_min_delta", 0.05)
        self.declare_parameter("release_pose_x", -1.301)
        self.declare_parameter("release_pose_y", 1.342)
        self.declare_parameter("release_pose_z", 0.7)
        self.declare_parameter("release_pose_qx", 0.0)
        self.declare_parameter("release_pose_qy", 0.0)
        self.declare_parameter("release_pose_qz", 0.0)
        self.declare_parameter("release_pose_qw", 1.0)
        self.declare_parameter("predrop_offset_z", 0.45)
        self.declare_parameter("post_release_retreat_z", 0.25)
        self.declare_parameter("dropoff_planning_time_s", 8.0)
        self.declare_parameter("dropoff_num_planning_attempts", 8)
        self.declare_parameter("dropoff_position_tolerance_m", 0.015)
        self.declare_parameter("dropoff_orientation_tolerance_rad", 0.35)
        self.declare_parameter("dropoff_orientation_z_tolerance_rad", 2.5)
        self.declare_parameter("dropoff_relaxed_orientation_tolerance_rad", 0.8)
        self.declare_parameter("dropoff_relaxed_orientation_z_tolerance_rad", 3.14)
        self.declare_parameter("dropoff_use_current_orientation_fallback", True)
        self.declare_parameter("dropoff_max_pose_retries", 3)
        self.declare_parameter("dropoff_debug_diagnostics", True)
        self.declare_parameter("verify_attached_in_scene", True)
        self.declare_parameter("enable_cumotion_object_attachment", True)
        self.declare_parameter("cumotion_attach_action_name", "planner_attach_object")
        self.declare_parameter("cumotion_attach_object_link_name", "attached_object")
        self.declare_parameter("cumotion_object_collision_inflation_m", 0.008)
        self.declare_parameter("cumotion_attachment_target_sphere_diameter_m", 0.03)
        self.declare_parameter("cumotion_attachment_min_spheres", 8)
        self.declare_parameter("cumotion_attachment_max_spheres", 32)
        self.declare_parameter("cumotion_attach_timeout_s", 2.0)
        self.declare_parameter("cumotion_detach_timeout_s", 2.0)
        self.declare_parameter(
            "gripper_touch_links",
            [
                END_EFFECTOR_LINK,
                "tool0",
                "robotiq_140_base_link",
                "left_inner_finger",
                "right_inner_finger",
                "left_inner_finger_pad",
                "right_inner_finger_pad",
                "left_outer_finger",
                "right_outer_finger",
                "left_inner_knuckle",
                "right_inner_knuckle",
                "left_outer_knuckle",
                "right_outer_knuckle",
            ],
        )

        self.target_object_id = str(self.get_parameter("target_object_id").value)
        self.target_object_frame = str(self.get_parameter("target_object_frame").value)
        self._curobo_attachment_assets_root = str(self.get_parameter("assets_root").value)
        self.world_frame = self.get_parameter("world_frame").value
        self.pre_grasp_offset_z = self.get_parameter("pre_grasp_offset_z").value
        self.planning_time = self.get_parameter("planning_time").value
        self.max_velocity_scaling = self.get_parameter("max_velocity_scaling").value
        self.max_acceleration_scaling = self.get_parameter("max_acceleration_scaling").value
        self.num_planning_attempts = self.get_parameter("num_planning_attempts").value
        requested_motion_backend = str(self.get_parameter("motion_backend").value).strip().lower()
        if requested_motion_backend in {"curobo", "curobo_ros"}:
            self.motion_backend = "curobo_ros"
        elif requested_motion_backend in {"moveit", "move_it"}:
            self.motion_backend = "moveit"
        elif requested_motion_backend in {"hybrid", "hybrid_planner"}:
            self.motion_backend = "hybrid"
        else:
            self.motion_backend = "moveit"
            self.get_logger().warn(
                f"Unknown motion_backend '{requested_motion_backend}', falling back to MoveIt."
            )
        self.planning_pipeline = str(self.get_parameter("planning_pipeline").value).strip().lower()
        self.curobo_planner_type = str(self.get_parameter("curobo_planner_type").value)
        self.hybrid_precision_descent_enabled = bool(
            self.get_parameter("hybrid_precision_descent_enabled").value
        )
        self.hybrid_precision_descent_planner_type = str(
            self.get_parameter("hybrid_precision_descent_planner_type").value
        )
        self.hybrid_precision_descent_grasp_enabled = bool(
            self.get_parameter("hybrid_precision_descent_grasp_enabled").value
        )
        self.hybrid_precision_descent_dropoff_enabled = bool(
            self.get_parameter("hybrid_precision_descent_dropoff_enabled").value
        )
        self.hybrid_precision_descent_planning_time_s = float(
            self.get_parameter("hybrid_precision_descent_planning_time_s").value
        )
        self.hybrid_precision_descent_num_attempts = int(
            self.get_parameter("hybrid_precision_descent_num_attempts").value
        )
        self.curobo_carried_object_sphere_count = int(
            self.get_parameter("curobo_carried_object_sphere_count").value
        )
        self.grasp_offset_z = self.get_parameter("grasp_offset_z").value
        self.post_grasp_lift_z = self.get_parameter("post_grasp_lift_z").value
        self.grasp_retry_count = self.get_parameter("grasp_retry_count").value
        self.grasp_retry_step_z = self.get_parameter("grasp_retry_step_z").value
        self.position_tolerance_m = self.get_parameter("position_tolerance_m").value
        self.orientation_tolerance_rad = self.get_parameter("orientation_tolerance_rad").value
        self.move_group_replan_attempts = int(
            self.get_parameter("move_group_replan_attempts").value
        )
        self.enable_position_only_fallback = self.get_parameter(
            "enable_position_only_fallback"
        ).value
        self.goal_request_frame = self.get_parameter("goal_request_frame").value
        self.gripper_close_position = self.get_parameter("gripper_close_position").value
        self.post_grasp_settle_s = self.get_parameter("post_grasp_settle_s").value
        self.home_joint_tolerance_rad = self.get_parameter(
            "home_joint_tolerance_rad"
        ).value
        self.home_verify_joint_tolerance_rad_m = float(
            self.get_parameter("home_verify_joint_tolerance_rad_m").value
        )
        self.gripper_result_timeout_s = self.get_parameter("gripper_result_timeout_s").value
        self.close_early_success_wait_s = self.get_parameter(
            "close_early_success_wait_s"
        ).value
        self.close_success_min_position = self.get_parameter(
            "close_success_min_position"
        ).value
        self.close_success_min_delta = self.get_parameter("close_success_min_delta").value
        self.release_pose_x = self.get_parameter("release_pose_x").value
        self.release_pose_y = self.get_parameter("release_pose_y").value
        self.release_pose_z = self.get_parameter("release_pose_z").value
        self.release_pose_qx = self.get_parameter("release_pose_qx").value
        self.release_pose_qy = self.get_parameter("release_pose_qy").value
        self.release_pose_qz = self.get_parameter("release_pose_qz").value
        self.release_pose_qw = self.get_parameter("release_pose_qw").value
        self.predrop_offset_z = float(self.get_parameter("predrop_offset_z").value)
        self.post_release_retreat_z = float(
            self.get_parameter("post_release_retreat_z").value
        )
        self.dropoff_planning_time_s = float(
            self.get_parameter("dropoff_planning_time_s").value
        )
        self.dropoff_num_planning_attempts = int(
            self.get_parameter("dropoff_num_planning_attempts").value
        )
        self.dropoff_position_tolerance_m = float(
            self.get_parameter("dropoff_position_tolerance_m").value
        )
        self.dropoff_orientation_tolerance_rad = float(
            self.get_parameter("dropoff_orientation_tolerance_rad").value
        )
        self.dropoff_orientation_z_tolerance_rad = float(
            self.get_parameter("dropoff_orientation_z_tolerance_rad").value
        )
        self.dropoff_relaxed_orientation_tolerance_rad = float(
            self.get_parameter("dropoff_relaxed_orientation_tolerance_rad").value
        )
        self.dropoff_relaxed_orientation_z_tolerance_rad = float(
            self.get_parameter("dropoff_relaxed_orientation_z_tolerance_rad").value
        )
        self.dropoff_use_current_orientation_fallback = bool(
            self.get_parameter("dropoff_use_current_orientation_fallback").value
        )
        self.dropoff_max_pose_retries = int(
            self.get_parameter("dropoff_max_pose_retries").value
        )
        self.dropoff_debug_diagnostics = bool(
            self.get_parameter("dropoff_debug_diagnostics").value
        )
        self.verify_attached_in_scene = self.get_parameter(
            "verify_attached_in_scene"
        ).value
        self.enable_cumotion_object_attachment = bool(
            self.get_parameter("enable_cumotion_object_attachment").value
        )
        self.cumotion_attach_action_name = str(
            self.get_parameter("cumotion_attach_action_name").value
        )
        self.cumotion_attach_object_link_name = str(
            self.get_parameter("cumotion_attach_object_link_name").value
        )
        self.cumotion_object_collision_inflation_m = float(
            self.get_parameter("cumotion_object_collision_inflation_m").value
        )
        self.cumotion_attachment_target_sphere_diameter_m = float(
            self.get_parameter("cumotion_attachment_target_sphere_diameter_m").value
        )
        self.cumotion_attachment_min_spheres = int(
            self.get_parameter("cumotion_attachment_min_spheres").value
        )
        self.cumotion_attachment_max_spheres = int(
            self.get_parameter("cumotion_attachment_max_spheres").value
        )
        self.cumotion_attach_timeout_s = float(
            self.get_parameter("cumotion_attach_timeout_s").value
        )
        self.cumotion_detach_timeout_s = float(
            self.get_parameter("cumotion_detach_timeout_s").value
        )
        self._use_cumotion_object_attachment = (
            self.enable_cumotion_object_attachment
            and str(self.motion_backend).lower() == "moveit"
            and str(self.planning_pipeline).lower() == "cumotion"
        )
        self.cumotion_attachment_min_spheres = max(
            1, int(self.cumotion_attachment_min_spheres)
        )
        self.cumotion_attachment_max_spheres = max(
            self.cumotion_attachment_min_spheres,
            int(self.cumotion_attachment_max_spheres),
        )
        self.gripper_touch_links = list(self.get_parameter("gripper_touch_links").value)
        if self.motion_backend not in ("moveit",):
            self.get_logger().info(
                f"motion_backend={self.motion_backend} selected. "
                "planning_pipeline parameter is ignored for this run."
            )

        # TF
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # MoveGroup action client (MoveIt 2 serves at /move_action)
        self.cb_group = ReentrantCallbackGroup()
        self.move_group_client = ActionClient(
            self, MoveGroup, "/move_action", callback_group=self.cb_group
        )
        self.arm_traj_client = ActionClient(
            self,
            FollowJointTrajectory,
            ARM_TRAJECTORY_ACTION,
            callback_group=self.cb_group,
        )
        self.gripper_traj_client = ActionClient(
            self,
            FollowJointTrajectory,
            GRIPPER_TRAJECTORY_ACTION,
            callback_group=self.cb_group,
        )
        self._init_cumotion_attachment_client()
        self.curobo_add_object_client = (
            self.create_client(AddObject, "/unified_planner/add_object")
            if AddObject is not None
            else None
        )
        self.curobo_remove_object_client = (
            self.create_client(RemoveObject, "/unified_planner/remove_object")
            if RemoveObject is not None
            else None
        )
        self.curobo_set_link_collision_client = (
            self.create_client(SetLinkCollision, "/unified_planner/set_link_collision")
            if SetLinkCollision is not None
            else None
        )
        self.curobo_get_obstacles_client = (
            self.create_client(Trigger, "/unified_planner/get_obstacles")
            if str(self.motion_backend).lower() == "curobo_ros"
            else None
        )

        # Planning scene + environment-collision interfaces.
        self.planning_scene_pub = self.create_publisher(
            PlanningScene, "/planning_scene", 10
        )
        self.get_planning_scene_client = self.create_client(
            GetPlanningScene, "/get_planning_scene"
        )
        self.gripper_command_pub = self.create_publisher(Bool, "/gripper_command", 10)
        self.environment_collision_control_pub = self.create_publisher(
            String, "/environment_collision/control", 10
        )
        self.metrics_event_pub = self.create_publisher(
            String, "/pick_and_place/metrics_events", 10
        )
        self._attached_object_id = None
        self._target_object_suppressed = False
        self._home_joint_values = None
        self._last_finger_joint_position = None
        self._last_finger_joint_msg_time = 0.0
        self._finger_joint_history = deque(maxlen=400)
        self._last_joint_positions = {}
        self._last_joint_positions_msg_time = 0.0

        self.joint_state_sub = self.create_subscription(
            JointState, "/joint_states", self._on_joint_states, 20
        )

        self._executed = False
        self._shutdown_requested = False
        self.sequence_success = False
        self.sequence_failed = False
        if str(self.motion_backend).lower() == "curobo_ros":
            self._motion_backend_adapter = CuroboMotionBackend(self)
            self._curobo_object_tracker = CuroboObjectTracker(self)
        elif str(self.motion_backend).lower() == "hybrid":
            self._motion_backend_adapter = HybridMotionBackend(self)
            self._curobo_object_tracker = CuroboObjectTracker(self)
        else:
            self._motion_backend_adapter = MoveItMotionBackend(self)
            self._curobo_object_tracker = None

        # Run execution in a background thread after a wall-clock delay
        self._thread = threading.Thread(target=self._delayed_execute, daemon=True)
        self._thread.start()

        self.get_logger().info(
            "Pick-and-place node initialized. Waiting 3s for TF... "
            "Surface gripper commands will be published to /gripper_command. "
            f"Motion backend: {self.motion_backend}. "
            f"Requested planning pipeline: {self.planning_pipeline}. "
            f"Requested curobo planner type: {self.curobo_planner_type}. "
            f"Target object id/frame: {self.target_object_id}/{self.target_object_frame}. "
            f"cuMotion object attachment={'enabled' if self._use_cumotion_object_attachment else 'disabled'}. "
            f"attachment sphere target_d={float(self.cumotion_attachment_target_sphere_diameter_m):.3f} m "
            f"(min={self.cumotion_attachment_min_spheres}, max={self.cumotion_attachment_max_spheres}). "
            f"post_grasp_lift_z={float(self.post_grasp_lift_z):.3f} m"
        )
        self.get_logger().info(
            "Dropoff tuning: "
            f"planning_time={float(self.dropoff_planning_time_s):.2f}s, "
            f"num_planning_attempts={int(self.dropoff_num_planning_attempts)}, "
            f"predrop_offset_z={float(self.predrop_offset_z):.3f}m, "
            f"post_release_retreat_z={float(self.post_release_retreat_z):.3f}m, "
            f"position_tolerance={float(self.dropoff_position_tolerance_m):.3f}m, "
            f"orientation_tolerance_xy={float(self.dropoff_orientation_tolerance_rad):.3f}rad, "
            f"orientation_tolerance_z={float(self.dropoff_orientation_z_tolerance_rad):.3f}rad, "
            f"relaxed_orientation_tolerance_xy={float(self.dropoff_relaxed_orientation_tolerance_rad):.3f}rad, "
            f"relaxed_orientation_tolerance_z={float(self.dropoff_relaxed_orientation_z_tolerance_rad):.3f}rad, "
            f"use_current_orientation_fallback={bool(self.dropoff_use_current_orientation_fallback)}, "
            f"max_pose_retries={int(self.dropoff_max_pose_retries)}, "
            f"debug_diagnostics={bool(self.dropoff_debug_diagnostics)}, "
            f"move_group_replan_attempts={int(self.move_group_replan_attempts)}"
        )
        self.get_logger().info(
            "Hybrid precision descents: "
            f"enabled={bool(self.hybrid_precision_descent_enabled)}, "
            f"planner_type={self.hybrid_precision_descent_planner_type}, "
            f"grasp_enabled={bool(self.hybrid_precision_descent_grasp_enabled)}, "
            f"dropoff_enabled={bool(self.hybrid_precision_descent_dropoff_enabled)}, "
            f"override_planning_time="
            f"{float(self.hybrid_precision_descent_planning_time_s):.2f}s, "
            f"override_num_attempts={int(self.hybrid_precision_descent_num_attempts)}"
        )

    def _hybrid_precision_descent_requested(self, stage):
        if str(self.motion_backend).lower() != "hybrid":
            return False
        if not bool(self.hybrid_precision_descent_enabled):
            return False

        normalized_stage = str(stage).strip().lower()
        if normalized_stage == "grasp":
            return bool(self.hybrid_precision_descent_grasp_enabled)
        if normalized_stage in {"dropoff", "release", "retreat"}:
            return bool(self.hybrid_precision_descent_dropoff_enabled)

        self.get_logger().warn(
            f"Unknown hybrid precision descent stage '{stage}', leaving hybrid path unchanged."
        )
        return False

    def _resolve_hybrid_precision_descent_request(
        self,
        stage,
        default_planning_time,
        default_num_attempts,
    ):
        planning_time = (
            None if default_planning_time is None else float(default_planning_time)
        )
        num_attempts = (
            None if default_num_attempts is None else int(default_num_attempts)
        )
        if not self._hybrid_precision_descent_requested(stage):
            return "default", None, planning_time, num_attempts

        if float(self.hybrid_precision_descent_planning_time_s) > 0.0:
            planning_time = float(self.hybrid_precision_descent_planning_time_s)
        if int(self.hybrid_precision_descent_num_attempts) > 0:
            num_attempts = int(self.hybrid_precision_descent_num_attempts)

        planner_type = (
            str(self.hybrid_precision_descent_planner_type).strip().lower() or "classic"
        )
        return "precision_descent", planner_type, planning_time, num_attempts

    def _initial_home_via_controller(self):
        """Move robot to home configuration via direct joint trajectory.

        Bypasses motion planning entirely by sending a FollowJointTrajectory
        goal straight to the arm controller.  This is used when the planner
        cannot plan from the initial robot pose (e.g. hybrid backend's cuRobo
        considers the URDF-default all-zeros configuration to be in collision).
        """
        self.get_logger().info(
            "Moving to home via direct joint trajectory (no collision avoidance)..."
        )
        if not self.arm_traj_client.wait_for_server(timeout_sec=30.0):
            self.get_logger().error(
                f"Arm trajectory action server not available after 30s: "
                f"{ARM_TRAJECTORY_ACTION}"
            )
            return False

        trajectory = JointTrajectory()
        trajectory.joint_names = list(JOINT_NAMES)

        point = JointTrajectoryPoint()
        point.positions = [HOME_JOINT_VALUES[j] for j in JOINT_NAMES]
        point.time_from_start = Duration(seconds=6.0).to_msg()
        trajectory.points = [point]

        goal = FollowJointTrajectory.Goal()
        goal.trajectory = trajectory

        future = self.arm_traj_client.send_goal_async(goal)
        goal_handle = self._wait_future_result(future, timeout_sec=30.0)
        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().error("Initial home trajectory goal was rejected!")
            return False

        result_future = goal_handle.get_result_async()
        result = self._wait_future_result(result_future, timeout_sec=15.0)
        if result is None:
            self.get_logger().error(
                "Initial home trajectory did not complete within timeout!"
            )
            return False

        self.get_logger().info(
            "Robot reached home configuration via direct trajectory."
        )
        return True

    def _delayed_execute(self):
        """Wait for wall-clock delay, then run the execution sequence."""
        time.sleep(3.0)
        self._execute()

    def _publish_metrics_event(self, event: str, phase: str, **fields) -> None:
        msg = String()
        msg.data = json.dumps({"event": event, "phase": phase, **fields})
        self.metrics_event_pub.publish(msg)

    def _start_metrics_phase(self, phase: str) -> None:
        self._publish_metrics_event("phase_start", phase)

    def _finish_metrics_phase(
        self, phase: str, success: bool, failure_reason: str = ""
    ) -> None:
        self._publish_metrics_event(
            "phase_end",
            phase,
            success=bool(success),
            failure_reason=failure_reason,
        )

    def _fail_metrics_phase(self, phase: str, reason: str) -> None:
        self._finish_metrics_phase(phase, False, reason)
        self._abort_sequence(reason)

    def _abort_sequence(self, reason: str) -> None:
        self.sequence_failed = True
        self.get_logger().info(
            f"Sequence failed: {reason}. Waiting for external timeout."
        )

    def _execute(self):
        """Main execution sequence."""
        if self._executed:
            return
        self._executed = True
        shutdown_reason = None
        try:
            if not self.gripper_traj_client.wait_for_server(timeout_sec=30.0):
                self.get_logger().error(
                    f"Gripper trajectory action server not available after 30s: "
                    f"{GRIPPER_TRAJECTORY_ACTION}"
                )
                self._abort_sequence("gripper_trajectory_action_server_unavailable")
                return
            if not self._motion_backend_adapter.wait_until_ready():
                self._abort_sequence("motion_backend_not_ready")
                return

            if str(self.motion_backend).lower() == "hybrid":
                if not self._initial_home_via_controller():
                    self.get_logger().error(
                        "Failed to move robot to home via direct trajectory. "
                        "Aborting — the hybrid planner cannot plan from the "
                        "default all-zeros configuration."
                    )
                    self._abort_sequence("hybrid_initial_home_failed")
                    return

            self.get_logger().info(
                f"{self.motion_backend} motion backend connected. Starting sequence..."
            )

            # 1. Look up target object pose
            object_pose = self._lookup_target_object_pose()
            if object_pose is None:
                self.get_logger().error(
                    f"Failed to look up target object frame '{self.target_object_frame}'. Aborting."
                )
                self._abort_sequence("target_object_pose_lookup_failed")
                return

            ox, oy, oz, oqx, oqy, oqz, oqw = object_pose
            self.get_logger().info(
                f"Target object '{self.target_object_id}' pose in world: "
                f"x={ox:.3f}, y={oy:.3f}, z={oz:.3f}, "
                f"q=({oqx:.3f}, {oqy:.3f}, {oqz:.3f}, {oqw:.3f})"
            )

            # 2. Open gripper via ros2_control trajectory action
            self.get_logger().info("Opening gripper...")
            success = self._move_gripper(0.0)
            if not success:
                self.get_logger().error("Failed to open gripper. Aborting.")
                self._abort_sequence("failed_to_open_gripper")
                return
            self.get_logger().info("Gripper opened.")

            # 3. Move to pre-grasp pose (above target object)
            pre_grasp_z = oz + self.pre_grasp_offset_z
            self.get_logger().info(
                f"Moving to pre-grasp pose: x={ox:.3f}, y={oy:.3f}, z={pre_grasp_z:.3f}"
            )
            self._start_metrics_phase("pre_grasp")
            success = self._motion_backend_adapter.move_to_pose(
                ox, oy, pre_grasp_z, oqx, oqy, oqz, oqw
            )
            if not success:
                self.get_logger().warn(
                    "Pre-grasp plan failed. Retrying once with extended planning budget."
                )
                success = self._motion_backend_adapter.move_to_pose(
                    ox,
                    oy,
                    pre_grasp_z,
                    oqx,
                    oqy,
                    oqz,
                    oqw,
                    planning_time=20.0,
                    num_attempts=20,
                )
                if not success:
                    self._fail_metrics_phase(
                        "pre_grasp", "failed_to_move_to_pre_grasp_pose"
                    )
                    self.get_logger().error("Failed to move to pre-grasp pose. Aborting.")
                    return
            self._finish_metrics_phase("pre_grasp", True)
            self.get_logger().info("Reached pre-grasp pose.")

            # 4. Move to grasp pose (gripper around object) while object remains in scene
            # Try grasp poses with increasing z offset. This helps recover from
            # cuMotion IK failures when the nominal low grasp target is too constrained.
            success = False
            grasp_retry_count = max(1, int(self.grasp_retry_count))
            grasp_retry_step = float(self.grasp_retry_step_z)
            used_grasp_offset_z = float(self.grasp_offset_z)
            grasp_planning_time = 20.0
            grasp_num_attempts = 20
            grasp_execution_mode = "default"
            grasp_planner_override = None
            (
                grasp_execution_mode,
                grasp_planner_override,
                grasp_planning_time,
                grasp_num_attempts,
            ) = self._resolve_hybrid_precision_descent_request(
                "grasp",
                grasp_planning_time,
                grasp_num_attempts,
            )
            if grasp_execution_mode == "precision_descent":
                self.get_logger().info(
                    "Hybrid precision grasp descent enabled: using direct cuRobo "
                    f"planner='{grasp_planner_override}', "
                    f"planning_time={float(grasp_planning_time):.2f}s, "
                    f"num_attempts={int(grasp_num_attempts)}."
                )
            elif (
                str(self.motion_backend).lower() == "curobo_ros"
                and str(self.curobo_planner_type).strip().lower() == "mpc"
            ):
                grasp_planner_override = "classic"
                self.get_logger().info(
                    "Using curobo classic planner for final grasp descent while "
                    "keeping MPC for the surrounding pick-and-place phases."
                )
            self._start_metrics_phase("grasp")
            for grasp_try in range(grasp_retry_count):
                offset_z = float(self.grasp_offset_z) + grasp_try * grasp_retry_step
                used_grasp_offset_z = offset_z
                grasp_z = oz + offset_z
                self.get_logger().info(
                    f"Moving to grasp pose attempt {grasp_try + 1}/{grasp_retry_count}: "
                    f"x={ox:.3f}, y={oy:.3f}, z={grasp_z:.3f}, offset_z={offset_z:.3f} "
                    f"(TCP at z={grasp_z:.3f}, object center at z={oz:.3f})"
                )
                success = self._motion_backend_adapter.move_to_pose(
                    ox,
                    oy,
                    grasp_z,
                    oqx,
                    oqy,
                    oqz,
                    oqw,
                    planning_time=grasp_planning_time,
                    num_attempts=grasp_num_attempts,
                    planner_type=grasp_planner_override,
                    execution_mode=grasp_execution_mode,
                )
                if success:
                    break

            if not success:
                self._fail_metrics_phase("grasp", "failed_to_move_to_grasp_pose")
                self.get_logger().error("Failed to move to grasp pose. Aborting.")
                return
            self._finish_metrics_phase("grasp", True)
            self.get_logger().info("Reached grasp pose. Gripper is around the object.")

            object_snapshot = None
            if str(self.motion_backend).lower() in ("moveit", "hybrid"):
            # Snapshot exact world geometry for MoveIt and hybrid backends.
                self.get_logger().info(
                    f"Snapshotting world collision geometry for '{self.target_object_id}'..."
                )
                object_snapshot = self._snapshot_world_object_geometry(
                    self.target_object_id, timeout_sec=1.0
                )
                if object_snapshot is None:
                    self.get_logger().warn(
                        f"Could not snapshot world geometry for '{self.target_object_id}'. "
                        "Attempting one recovery pass (unsuppress + resnapshot)..."
                    )
                    recovered = self._set_environment_object_suppressed(
                        self.target_object_id, suppress=False, wait_timeout_sec=2.0
                    )
                    if not recovered:
                        self.get_logger().warn(
                            f"Recovery unsuppress could not confirm '{self.target_object_id}' "
                            "in world collision objects."
                        )
                    self._target_object_suppressed = False
                    object_snapshot = self._snapshot_world_object_geometry(
                        self.target_object_id, timeout_sec=2.0
                    )
                    if object_snapshot is None:
                        self.get_logger().error(
                            f"Could not snapshot world geometry for '{self.target_object_id}' "
                            "after recovery. Aborting because MoveIt/cumotion attachment requires "
                            "runtime object collision geometry."
                        )
                        self._abort_sequence("target_object_geometry_snapshot_failed")
                        return

            # 6. Suppress world collision right before the close command.
            self.get_logger().info(
                f"Suppressing world collision object '{self.target_object_id}' before closing..."
            )
            success = self._set_environment_object_suppressed(
                self.target_object_id, suppress=True, wait_timeout_sec=2.0
            )
            if not success:
                self.get_logger().error(
                    f"Failed to suppress '{self.target_object_id}' in environment collisions. Aborting."
                )
                self._abort_sequence("failed_to_suppress_target_object_collision")
                return
            self._target_object_suppressed = True

            # 7. Close gripper and attach object collision geometry to the gripper
            self.get_logger().info(
                f"Closing gripper to {float(self.gripper_close_position):.3f} rad..."
            )
            success = self._move_gripper(
                float(self.gripper_close_position), allow_stall_success=True
            )
            if not success:
                self.get_logger().error("Failed to close gripper. Aborting.")
                self._restore_suppressed_object_collision()
                self._abort_sequence("failed_to_close_gripper")
                return
            self.get_logger().info("Gripper closed.")

            self._sleep(float(self.post_grasp_settle_s))
            # Isaac attachment must happen before lift so the simulated object follows
            # the gripper during the lift motion.
            self.get_logger().info(
                "Attaching in Isaac (surface gripper command) before post-grasp lift..."
            )
            self._publish_surface_gripper_command(True)

            # Lift slightly after close.
            lift_z = oz + used_grasp_offset_z + float(self.post_grasp_lift_z)
            self.get_logger().info(
                "Performing post-grasp lift: "
                f"x={ox:.3f}, y={oy:.3f}, z={lift_z:.3f}"
            )
            self._start_metrics_phase("post_grasp_lift")
            success = self._motion_backend_adapter.move_to_pose(
                ox,
                oy,
                lift_z,
                oqx,
                oqy,
                oqz,
                oqw,
                planning_time=10.0,
                num_attempts=10,
            )
            if not success:
                self._fail_metrics_phase("post_grasp_lift", "failed_post_grasp_lift")
                self.get_logger().error("Failed post-grasp lift. Aborting.")
                self._restore_suppressed_object_collision()
                return
            self._finish_metrics_phase("post_grasp_lift", True)
            self.get_logger().info("Post-grasp lift completed.")

            object_pose_after_lift = self._lookup_target_object_pose()
            if object_pose_after_lift is None:
                object_pose_after_lift = (ox, oy, lift_z, oqx, oqy, oqz, oqw)
                self.get_logger().warn(
                    "Could not refresh object world pose after post-grasp lift. "
                    "Using commanded lifted pose for attachment projection fallback."
                )
            if object_snapshot is not None:
                ax, ay, az, aqx, aqy, aqz, aqw = object_pose_after_lift
                object_snapshot.pose.position.x = float(ax)
                object_snapshot.pose.position.y = float(ay)
                object_snapshot.pose.position.z = float(az)
                object_snapshot.pose.orientation.x = float(aqx)
                object_snapshot.pose.orientation.y = float(aqy)
                object_snapshot.pose.orientation.z = float(aqz)
                object_snapshot.pose.orientation.w = float(aqw)
                object_snapshot.header.frame_id = self.world_frame

            if str(self.motion_backend).lower() == "moveit":
                if self._use_cumotion_object_attachment:
                    success = self._attach_object_to_cumotion(
                        target_object_world_pose=object_pose_after_lift,
                        grasp_offset_z=used_grasp_offset_z,
                        world_object_snapshot=object_snapshot,
                    )
                    if not success:
                        self.get_logger().error(
                            "Failed to attach target object to cuMotion robot collision spheres. Aborting."
                        )
                        self._restore_suppressed_object_collision()
                        self._abort_sequence("failed_to_attach_object_to_cumotion")
                        return

                self.get_logger().info(
                    f"Attaching '{self.target_object_id}' collision object to gripper in MoveIt after lift..."
                )
                success = self._attach_object_collision(
                    self.target_object_id,
                    object_pose_after_lift,
                    world_object_snapshot=object_snapshot,
                    grasp_offset_z=used_grasp_offset_z,
                    publish_surface_gripper_command=False,
                )
                if not success:
                    self.get_logger().error(
                        f"Failed to attach '{self.target_object_id}' collision object. Aborting."
                    )
                    self._detach_object_from_cumotion(best_effort=True)
                    self._restore_suppressed_object_collision()
                    self._abort_sequence("failed_to_attach_object_collision")
                    return
                self.get_logger().info(
                    f"Collision object '{self.target_object_id}' attached to gripper."
                )
                if self.verify_attached_in_scene:
                    attached_seen = self._wait_for_attached_object_in_planning_scene(
                        self.target_object_id, timeout_sec=2.0
                    )
                    if attached_seen:
                        self.get_logger().info(
                            f"Verified '{self.target_object_id}' is attached in MoveIt's planning scene."
                        )
                    else:
                        self.get_logger().warn(
                            f"Could not verify '{self.target_object_id}' in MoveIt's attached objects "
                            "before release planning."
                        )
            elif str(self.motion_backend).lower() == "hybrid":
                # Dual attachment: MoveIt planning scene (for global planner)
                # + CuroboObjectTracker (for cuRobo MotionGen/MPC world model).
                self.get_logger().info(
                    f"Attaching '{self.target_object_id}' collision object to gripper "
                    "in MoveIt planning scene for hybrid global planner..."
                )
                success = self._attach_object_collision(
                    self.target_object_id,
                    object_pose_after_lift,
                    world_object_snapshot=object_snapshot,
                    grasp_offset_z=used_grasp_offset_z,
                    publish_surface_gripper_command=False,
                )
                if not success:
                    self.get_logger().error(
                        f"Failed to attach '{self.target_object_id}' collision object "
                        "in MoveIt planning scene. Aborting."
                    )
                    self._restore_suppressed_object_collision()
                    self._abort_sequence("failed_to_attach_object_collision")
                    return
                if self.verify_attached_in_scene:
                    attached_seen = self._wait_for_attached_object_in_planning_scene(
                        self.target_object_id, timeout_sec=2.0
                    )
                    if attached_seen:
                        self.get_logger().info(
                            f"Verified '{self.target_object_id}' is attached in MoveIt's planning scene."
                        )
                    else:
                        self.get_logger().warn(
                            f"Could not verify '{self.target_object_id}' in MoveIt's attached objects."
                        )

                object_pose_in_ee = self._lookup_target_object_pose_in_end_effector(
                    target_object_world_pose=object_pose_after_lift,
                )
                success = self._curobo_object_tracker.attach(
                    object_pose_world=object_pose_after_lift,
                    object_pose_in_ee=object_pose_in_ee,
                )
                if not success:
                    self.get_logger().error(
                        "Failed to attach carried-object spheres in cuRobo world model. Aborting."
                    )
                    self._detach_object_collision(
                        self.target_object_id, publish_surface_gripper_command=False
                    )
                    self._restore_suppressed_object_collision()
                    self._abort_sequence("failed_to_attach_curobo_carried_object")
                    return
            else:
                object_pose_in_ee = self._lookup_target_object_pose_in_end_effector(
                    target_object_world_pose=object_pose_after_lift,
                )
                success = self._curobo_object_tracker.attach(
                    object_pose_world=object_pose_after_lift,
                    object_pose_in_ee=object_pose_in_ee,
                )
                if not success:
                    self.get_logger().error(
                        "Failed to attach carried-object spheres for curobo backend. Aborting."
                    )
                    self._restore_suppressed_object_collision()
                    self._abort_sequence("failed_to_attach_curobo_carried_object")
                    return

            release_orientation = self._resolve_release_orientation()
            if release_orientation is None:
                self.get_logger().error("Configured release orientation is invalid.")
                self._detach_dynamic_collision_representations(best_effort=True)
                self._restore_suppressed_object_collision()
                self._abort_sequence("invalid_release_orientation")
                return
            release_qx, release_qy, release_qz, release_qw = release_orientation

            predrop_z = float(self.release_pose_z) + max(0.0, float(self.predrop_offset_z))
            self.get_logger().info(
                "Moving to pre-drop pose: "
                f"x={float(self.release_pose_x):.3f}, "
                f"y={float(self.release_pose_y):.3f}, "
                f"z={predrop_z:.3f}"
            )
            self._start_metrics_phase("pre_drop")
            success = self._motion_backend_adapter.move_to_pose(
                float(self.release_pose_x),
                float(self.release_pose_y),
                predrop_z,
                release_qx,
                release_qy,
                release_qz,
                release_qw,
                planning_time=float(self.dropoff_planning_time_s),
                num_attempts=int(self.dropoff_num_planning_attempts),
            )
            if not success:
                self._fail_metrics_phase("pre_drop", "failed_to_move_to_pre_drop_pose")
                self.get_logger().error("Failed to move to pre-drop pose. Aborting.")
                self._detach_dynamic_collision_representations(best_effort=True)
                self._restore_suppressed_object_collision()
                return
            self._finish_metrics_phase("pre_drop", True)
            self.get_logger().info("Reached pre-drop pose.")

            # 8. Move directly to release pose while carrying attached collision geometry.
            self._start_metrics_phase("release")
            success = self._motion_backend_adapter.move_to_release_pose_with_retries()
            if not success:
                self._fail_metrics_phase("release", "failed_to_move_to_release_pose")
                self.get_logger().error("Failed to move to release pose. Aborting.")
                self._detach_dynamic_collision_representations(best_effort=True)
                self._restore_suppressed_object_collision()
                return
            self._finish_metrics_phase("release", True)
            self.get_logger().info("Reached release pose.")

            # 9. Open gripper to release object
            self.get_logger().info(f"Opening gripper to release '{self.target_object_id}'...")
            success = self._move_gripper(0.0)
            if not success:
                self.get_logger().error(
                    f"Failed to open gripper at release pose for '{self.target_object_id}'. Aborting."
                )
                self._detach_dynamic_collision_representations(best_effort=True)
                self._restore_suppressed_object_collision()
                self._abort_sequence("failed_to_open_gripper_at_release_pose")
                return
            self.get_logger().info("Gripper opened at release pose.")

            if str(self.motion_backend).lower() == "moveit":
                # 10. Detach object in MoveIt planning scene
                self.get_logger().info(
                    f"Releasing '{self.target_object_id}' attachment in MoveIt planning scene..."
                )
                success = self._detach_object_collision(self.target_object_id)
                if not success:
                    self.get_logger().error(
                        f"Failed to release '{self.target_object_id}' attachment. Aborting."
                    )
                    self._detach_object_from_cumotion(best_effort=True)
                    self._restore_suppressed_object_collision()
                    self._abort_sequence("failed_to_detach_moveit_object")
                    return

                success = self._detach_object_from_cumotion(retry_once=True)
                if not success:
                    self.get_logger().error(
                        "Failed to detach target object from cuMotion collision spheres. Aborting."
                    )
                    self._restore_suppressed_object_collision()
                    self._abort_sequence("failed_to_detach_object_from_cumotion")
                    return
            elif str(self.motion_backend).lower() == "hybrid":
                # Dual detach: MoveIt planning scene + cuRobo world model.
                self.get_logger().info(
                    f"Releasing '{self.target_object_id}' attachment in MoveIt planning scene..."
                )
                success = self._detach_object_collision(self.target_object_id)
                if not success:
                    self.get_logger().error(
                        f"Failed to release '{self.target_object_id}' attachment "
                        "in MoveIt planning scene. Aborting."
                    )
                    self._curobo_object_tracker.detach(best_effort=True)
                    self._restore_suppressed_object_collision()
                    self._abort_sequence("failed_to_detach_moveit_object")
                    return

                success = self._curobo_object_tracker.detach()
                if not success:
                    self.get_logger().error(
                        "Failed to detach carried-object spheres from cuRobo world model. Aborting."
                    )
                    self._restore_suppressed_object_collision()
                    self._abort_sequence("failed_to_detach_curobo_carried_object")
                    return
            else:
                self._publish_surface_gripper_command(False)
                success = self._curobo_object_tracker.detach()
                if not success:
                    self.get_logger().error(
                        "Failed to detach carried-object spheres from curobo backend. Aborting."
                    )
                    self._restore_suppressed_object_collision()
                    self._abort_sequence("failed_to_detach_curobo_carried_object")
                    return

            retreat_z = float(self.release_pose_z) + max(
                0.0, float(self.post_release_retreat_z)
            )
            self.get_logger().info(
                "Retreating upward after release: "
                f"x={float(self.release_pose_x):.3f}, "
                f"y={float(self.release_pose_y):.3f}, "
                f"z={retreat_z:.3f}"
            )
            (
                retreat_execution_mode,
                retreat_planner_override,
                retreat_planning_time,
                retreat_num_attempts,
            ) = self._resolve_hybrid_precision_descent_request(
                "retreat",
                float(self.dropoff_planning_time_s),
                int(self.dropoff_num_planning_attempts),
            )
            if retreat_execution_mode == "precision_descent":
                self.get_logger().info(
                    "Hybrid precision retreat enabled: using direct cuRobo "
                    f"planner='{retreat_planner_override}', "
                    f"planning_time={float(retreat_planning_time):.2f}s, "
                    f"num_attempts={int(retreat_num_attempts)}."
                )
            self._start_metrics_phase("post_release_retreat")
            success = self._motion_backend_adapter.move_to_pose(
                float(self.release_pose_x),
                float(self.release_pose_y),
                retreat_z,
                release_qx,
                release_qy,
                release_qz,
                release_qw,
                planning_time=float(retreat_planning_time),
                num_attempts=int(retreat_num_attempts),
                planner_type=retreat_planner_override,
                execution_mode=retreat_execution_mode,
            )
            if not success:
                self._fail_metrics_phase(
                    "post_release_retreat",
                    "failed_to_retreat_upward_after_release",
                )
                self.get_logger().error("Failed to retreat upward after release. Aborting.")
                self._restore_suppressed_object_collision()
                return
            self._finish_metrics_phase("post_release_retreat", True)
            self.get_logger().info("Post-release retreat completed.")

            # 11. Unsuppress object so environment publisher resumes world ownership
            self.get_logger().info(
                f"Unsuppressing world collision object '{self.target_object_id}' after release..."
            )
            success = self._set_environment_object_suppressed(
                self.target_object_id, suppress=False, wait_timeout_sec=2.0
            )
            if not success:
                self.get_logger().error(
                    f"Failed to unsuppress world collision object '{self.target_object_id}'. Aborting."
                )
                self._detach_dynamic_collision_representations(best_effort=True)
                self._abort_sequence("failed_to_unsuppress_world_collision_object")
                return
            self._target_object_suppressed = False

            # 12. Return to home
            self.get_logger().info("Returning to home joint state...")
            self._start_metrics_phase("return_home")
            success = self._motion_backend_adapter.move_to_home()
            if not success:
                self._fail_metrics_phase(
                    "return_home", "failed_to_return_to_home_joint_state"
                )
                self.get_logger().error("Failed to return to home joint state. Aborting.")
                self._detach_dynamic_collision_representations(best_effort=True)
                return
            self._finish_metrics_phase("return_home", True)

            self.get_logger().info(
                "Pick-and-place sequence complete. Object released, environment collision restored, and robot returned home."
            )
            self.sequence_success = True
            shutdown_reason = "Sequence complete"
        finally:
            restored = self._motion_backend_adapter.restore_previous_planner()
            if not restored:
                self.get_logger().warn(
                    "Pick-and-place could not restore the previous planner state."
                )

        if shutdown_reason is not None:
            self._request_shutdown(shutdown_reason)

    def _move_to_release_pose_with_retries(self):
        """Move to release pose using a deterministic dropoff retry ladder."""
        release_orientation = self._resolve_release_orientation()
        if release_orientation is None:
            self.get_logger().error("Configured release orientation is invalid.")
            return False

        planning_time = max(0.1, float(self.dropoff_planning_time_s))
        num_attempts = max(1, int(self.dropoff_num_planning_attempts))
        (
            dropoff_execution_mode,
            dropoff_planner_override,
            planning_time,
            num_attempts,
        ) = self._resolve_hybrid_precision_descent_request(
            "dropoff",
            planning_time,
            num_attempts,
        )
        planning_time = max(0.1, float(planning_time))
        num_attempts = max(1, int(num_attempts))
        if dropoff_execution_mode == "precision_descent":
            self.get_logger().info(
                "Hybrid precision dropoff descent enabled: using direct cuRobo "
                f"planner='{dropoff_planner_override}', "
                f"planning_time={planning_time:.2f}s, "
                f"num_attempts={num_attempts}."
            )
        position_tolerance = max(1e-4, float(self.dropoff_position_tolerance_m))
        strict_orientation_tol_xy = max(
            1e-4, float(self.dropoff_orientation_tolerance_rad)
        )
        strict_orientation_tol_z = max(
            1e-4, float(self.dropoff_orientation_z_tolerance_rad)
        )
        relaxed_orientation_tol_xy = max(
            strict_orientation_tol_xy,
            float(self.dropoff_relaxed_orientation_tolerance_rad),
        )
        relaxed_orientation_tol_z = max(
            strict_orientation_tol_z,
            float(self.dropoff_relaxed_orientation_z_tolerance_rad),
        )
        strict_orientation_tol_xyz = (
            strict_orientation_tol_xy,
            strict_orientation_tol_xy,
            strict_orientation_tol_z,
        )
        relaxed_orientation_tol_xyz = (
            relaxed_orientation_tol_xy,
            relaxed_orientation_tol_xy,
            relaxed_orientation_tol_z,
        )
        max_pose_retries = max(1, int(self.dropoff_max_pose_retries))

        if dropoff_execution_mode == "precision_descent":
            attempt_specs = [
                (
                    "configured",
                    "direct",
                    release_orientation,
                    strict_orientation_tol_xyz,
                ),
            ]
        else:
            attempt_specs = [
                ("configured", "strict", release_orientation, strict_orientation_tol_xyz),
                ("configured", "relaxed", release_orientation, relaxed_orientation_tol_xyz),
            ]

        if self.dropoff_use_current_orientation_fallback:
            current_orientation = self._lookup_end_effector_orientation()
            if current_orientation is None:
                self.get_logger().warn(
                    "Could not get current EE orientation for dropoff fallback attempt."
                )
            elif self._quaternions_equivalent(current_orientation, release_orientation):
                self.get_logger().warn(
                    "Current EE orientation matches configured release orientation; "
                    "skipping duplicate dropoff fallback attempt."
                )
            else:
                attempt_specs.append(
                    (
                        "current_ee",
                        "direct" if dropoff_execution_mode == "precision_descent" else "relaxed",
                        current_orientation,
                        relaxed_orientation_tol_xyz,
                    )
                )

        if dropoff_execution_mode != "precision_descent" and max_pose_retries > len(attempt_specs):
            attempt_specs.extend([attempt_specs[-1]] * (max_pose_retries - len(attempt_specs)))
        else:
            attempt_specs = attempt_specs[:max_pose_retries]

        target_x = float(self.release_pose_x)
        target_y = float(self.release_pose_y)
        target_z = float(self.release_pose_z)
        total_attempts = len(attempt_specs)
        for idx, (source, mode, orientation, orient_tol_xyz) in enumerate(
            attempt_specs, start=1
        ):
            oqx, oqy, oqz, oqw = orientation
            tol_x, tol_y, tol_z = orient_tol_xyz
            attempt_id = f"dropoff_attempt_{idx}_{source}_{mode}"
            self.get_logger().info(
                f"Dropoff planning attempt {idx}/{total_attempts}: "
                f"source={source}, mode={mode}, "
                f"pose=({target_x:.3f}, {target_y:.3f}, {target_z:.3f}), "
                f"q=({oqx:.3f}, {oqy:.3f}, {oqz:.3f}, {oqw:.3f}), "
                f"position_tol={position_tolerance:.3f}m, "
                f"orientation_tol_xyz=({tol_x:.3f}, {tol_y:.3f}, {tol_z:.3f})rad, "
                f"planning_time={planning_time:.2f}s, "
                f"num_planning_attempts={num_attempts}"
            )
            if dropoff_execution_mode == "precision_descent":
                if self._motion_backend_adapter.move_to_pose(
                    target_x,
                    target_y,
                    target_z,
                    oqx,
                    oqy,
                    oqz,
                    oqw,
                    planning_time=planning_time,
                    num_attempts=num_attempts,
                    planner_type=dropoff_planner_override,
                    execution_mode=dropoff_execution_mode,
                ):
                    self.get_logger().info(
                        f"Dropoff precision descent succeeded on attempt {idx}/{total_attempts}: "
                        f"source={source}, mode={mode}, attempt_id={attempt_id}."
                    )
                    return True
            else:
                goal = self._build_pose_goal(
                    target_x,
                    target_y,
                    target_z,
                    oqx,
                    oqy,
                    oqz,
                    oqw,
                    planning_time=planning_time,
                    num_attempts=num_attempts,
                    constrain_orientation=True,
                    orientation_tolerance=None,
                    orientation_tolerance_xyz=orient_tol_xyz,
                    position_tolerance_m=position_tolerance,
                )
                debug_meta = {
                    "stage": "dropoff",
                    "attempt_id": attempt_id,
                    "attempt_index": int(idx),
                    "total_attempts": int(total_attempts),
                    "attempt_source": source,
                    "attempt_mode": mode,
                    "target_object_id": str(self.target_object_id),
                    "target_pose_world": (
                        float(target_x),
                        float(target_y),
                        float(target_z),
                        float(oqx),
                        float(oqy),
                        float(oqz),
                        float(oqw),
                    ),
                    "orientation_tolerance_xyz": (
                        float(tol_x),
                        float(tol_y),
                        float(tol_z),
                    ),
                    "position_tolerance_m": float(position_tolerance),
                }
                if not self._send_move_group_goal(
                    goal,
                    context_label=attempt_id,
                    debug_meta=debug_meta,
                    run_failure_diagnostics=bool(self.dropoff_debug_diagnostics),
                ):
                    self.get_logger().warn(
                        f"Dropoff planning attempt {idx}/{total_attempts} failed "
                        f"(attempt_id={attempt_id})."
                    )
                    continue
                if source == "current_ee" and mode == "relaxed":
                    achieved_orientation = self._lookup_end_effector_orientation()
                    cfg_qx, cfg_qy, cfg_qz, cfg_qw = release_orientation
                    used_qx, used_qy, used_qz, used_qw = orientation
                    self.get_logger().info(
                        "Dropoff accepted with current_ee+relaxed orientation. "
                        f"configured_q=({cfg_qx:.4f}, {cfg_qy:.4f}, {cfg_qz:.4f}, {cfg_qw:.4f}), "
                        f"used_target_q=({used_qx:.4f}, {used_qy:.4f}, {used_qz:.4f}, {used_qw:.4f}), "
                        f"orientation_tol_xyz=({tol_x:.3f}, {tol_y:.3f}, {tol_z:.3f})rad."
                    )
                    config_vs_used = self._quat_angular_distance_rad(
                        release_orientation, orientation
                    )
                    if config_vs_used is not None:
                        self.get_logger().info(
                            "Dropoff orientation compare: configured->used_target "
                            f"delta={config_vs_used:.4f} rad "
                            f"({math.degrees(config_vs_used):.2f} deg)."
                        )
                    if achieved_orientation is not None:
                        ach_qx, ach_qy, ach_qz, ach_qw = achieved_orientation
                        self.get_logger().info(
                            "Dropoff achieved EE orientation (world frame): "
                            f"achieved_q=({ach_qx:.4f}, {ach_qy:.4f}, {ach_qz:.4f}, {ach_qw:.4f})."
                        )
                        config_vs_achieved = self._quat_angular_distance_rad(
                            release_orientation, achieved_orientation
                        )
                        used_vs_achieved = self._quat_angular_distance_rad(
                            orientation, achieved_orientation
                        )
                        if config_vs_achieved is not None:
                            self.get_logger().info(
                                "Dropoff orientation compare: configured->achieved "
                                f"delta={config_vs_achieved:.4f} rad "
                                f"({math.degrees(config_vs_achieved):.2f} deg)."
                            )
                        if used_vs_achieved is not None:
                            self.get_logger().info(
                                "Dropoff orientation compare: used_target->achieved "
                                f"delta={used_vs_achieved:.4f} rad "
                                f"({math.degrees(used_vs_achieved):.2f} deg)."
                            )
                    else:
                        self.get_logger().warn(
                            "Dropoff accepted with current_ee+relaxed but could not "
                            "read achieved EE orientation for comparison."
                        )
                self.get_logger().info(
                    f"Dropoff planning succeeded on attempt {idx}/{total_attempts}: "
                    f"source={source}, mode={mode}, attempt_id={attempt_id}."
                )
                return True
            self.get_logger().warn(
                f"Dropoff planning attempt {idx}/{total_attempts} failed "
                f"(attempt_id={attempt_id})."
            )
        return False

    def _resolve_release_orientation(self):
        """Return a normalized release orientation quaternion from node parameters."""
        release_q = (
            float(self.release_pose_qx),
            float(self.release_pose_qy),
            float(self.release_pose_qz),
            float(self.release_pose_qw),
        )
        norm = math.sqrt(sum(component * component for component in release_q))
        if norm <= 1e-9:
            self.get_logger().error(
                "Configured release orientation quaternion has near-zero norm."
            )
            return None
        if abs(norm - 1.0) > 1e-3:
            self.get_logger().warn(
                f"Normalizing non-unit release quaternion (norm={norm:.6f})."
            )
        return tuple(component / norm for component in release_q)

    def _detach_dynamic_collision_representations(self, best_effort=False):
        """Cleanup for carried-object collision state across backends."""
        success = True
        if self._curobo_object_tracker is not None:
            success = self._curobo_object_tracker.detach(best_effort=best_effort) and success
        success = self._detach_object_from_cumotion(best_effort=best_effort) and success
        if self._attached_object_id is not None:
            success = self._detach_object_collision(
                publish_surface_gripper_command=False,
            ) and success
        return success or best_effort

    def _restore_suppressed_object_collision(self):
        """Cleanup to re-enable world collisions after an abort path."""
        if not self._target_object_suppressed:
            return True
        restored = self._set_environment_object_suppressed(
            self.target_object_id, suppress=False, wait_timeout_sec=2.0
        )
        if restored:
            self._target_object_suppressed = False
        return restored

    def _sleep(self, seconds):
        """Block for a wall-clock duration; main thread spin handles callbacks."""
        if seconds <= 0.0:
            return
        time.sleep(float(seconds))

    def _request_shutdown(self, reason):
        """Request process shutdown once so launch exits without manual Ctrl+C."""
        if self._shutdown_requested:
            return
        self._shutdown_requested = True
        self.get_logger().info(
            f"Shutting down pick-and-place node: {reason}. "
            "No manual Ctrl+C required."
        )
        if rclpy.ok():
            rclpy.shutdown()
