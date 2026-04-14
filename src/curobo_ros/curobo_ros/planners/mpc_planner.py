#!/usr/bin/env python3
"""
Model Predictive Control (MPC) planner.

This planner uses cuRobo's MPC solver for real-time closed-loop control,
recalculating the trajectory at each time step based on the current state.
"""

import time
from typing import Optional

import torch
from curobo.types.robot import JointState
from curobo.types.math import Pose
from curobo.rollout.rollout_base import Goal
import traceback
from .trajectory_planner import TrajectoryPlanner, PlannerResult, ExecutionMode
from curobo_ros.core.controller_trajectory import extract_ordered_waypoints


from curobo_msgs.action import SendTrajectory
from curobo_msgs.srv import TrajectoryGeneration

class MPCPlanner(TrajectoryPlanner):
    """
    Model Predictive Control planner.

    Unlike open-loop planners, MPC continuously recalculates the optimal
    trajectory based on the current robot state, enabling reactive control.

    Execution flow:
    1. plan() - Setup MPC goal buffer
    2. execute() - Iterative loop: sense → replan → act → repeat
    """

    def __init__(self, node, config_wrapper):
        """
        Initialize MPC planner.

        Args:
            node: ROS2 node
            config_wrapper: ConfigWrapperMPC with MpcSolver configured
        """
        super().__init__(node, config_wrapper)

        # Store reference to MPC solver
        self.mpc = None

        # MPC state
        self.goal_buffer = None
        self.start_state = None
        self.goal_pose = None
        self.goal_joint_positions = None
        self.goal_state = None
        self.is_goal_active = False

        # Goal update from topic (for real-time tracking)
        self.latest_goal_from_topic = None

        # Performance tracking
        self.mpc_time = []
        self.initial_pose_error = None
        self.best_pose_error = None
        self.best_position_error = None
        self.best_rotation_error = None
        self.received_goal_update_count = 0
        self.applied_goal_update_count = 0
        self.ignored_goal_update_count = 0
        self.last_goal_topic_raw = None
        self.trajectory_constraints = []
        self._classic_handoff_requested = False
        self._classic_handoff_metadata = None

        # MPC parameters
        self.convergence_threshold = 0.01  # meters
        self.position_convergence_threshold = 0.02  # meters
        self.rotation_convergence_threshold = 0.35  # radians
        self.joint_convergence_threshold = 0.05  # radians / meters in c-space
        self.max_iterations = 1000

    def _get_execution_mode(self) -> ExecutionMode:
        """This planner uses closed-loop execution."""
        return ExecutionMode.CLOSED_LOOP

    def get_planner_name(self) -> str:
        """Return planner name."""
        return "Model Predictive Control (MPC)"

    def get_config_parameters(self) -> list:
        """List of ROS parameters used by this planner."""
        return [
            'convergence_threshold',
            'position_convergence_threshold',
            'rotation_convergence_threshold',
            'joint_convergence_threshold',
            'command_speed_scale',
            'max_mpc_iterations',
        ]

    def set_mpc_solver(self, mpc_solver):
        """
        Set the MPC solver instance.

        Args:
            mpc_solver: Initialized MpcSolver instance
        """
        self.mpc = mpc_solver

    @staticmethod
    def _pose_msg_has_values(pose_msg) -> bool:
        """Return True when a ROS Pose contains a non-default value."""
        if pose_msg is None:
            return False
        values = (
            pose_msg.position.x,
            pose_msg.position.y,
            pose_msg.position.z,
            pose_msg.orientation.x,
            pose_msg.orientation.y,
            pose_msg.orientation.z,
            pose_msg.orientation.w,
        )
        return any(abs(float(value)) > 1e-9 for value in values)

    def _extract_goal_pose(self, goal_request, required: bool) -> Optional[Pose]:
        """Extract a pose goal when one is present in the request."""
        pose_msg = getattr(goal_request, 'target_pose', None)
        if pose_msg is None:
            if required:
                raise ValueError("MPCPlanner requires a target_pose when no joint target is provided.")
            return None

        if not required and not self._pose_msg_has_values(pose_msg):
            return None

        return Pose.from_list([
            pose_msg.position.x,
            pose_msg.position.y,
            pose_msg.position.z,
            pose_msg.orientation.x,
            pose_msg.orientation.y,
            pose_msg.orientation.z,
            pose_msg.orientation.w,
        ], q_xyzw=True)

    def _extract_goal_joint_positions(self, goal_request) -> Optional[list]:
        """Extract and validate a joint-space goal from the request."""
        goal_joint_positions = list(
            getattr(goal_request, 'target_joint_positions', []) or []
        )
        if not goal_joint_positions:
            return None

        robot_dof = self.mpc.kinematics.get_dof()
        if len(goal_joint_positions) != robot_dof:
            raise ValueError(
                f"MPC joint goal expects {robot_dof} joints, received {len(goal_joint_positions)}."
            )

        normalized_positions = [float(value) for value in goal_joint_positions]
        if any(not (-1e6 < value < 1e6) or value != value for value in normalized_positions):
            raise ValueError(
                f"Invalid joint positions detected (NaN/Inf): {normalized_positions}"
            )
        return normalized_positions

    @staticmethod
    def _zero_like_or_shape(reference_tensor, fallback_tensor):
        """Create a zero tensor matching the reference when possible."""
        if reference_tensor is not None:
            return torch.zeros_like(reference_tensor)
        return torch.zeros_like(fallback_tensor)

    def _build_goal_state(self, start_state: JointState, goal_joint_positions: list) -> JointState:
        """Build a cuRobo JointState goal in the solver's joint order."""
        goal_position = torch.tensor(
            [goal_joint_positions],
            device=start_state.position.device,
            dtype=start_state.position.dtype,
        )
        goal_velocity = self._zero_like_or_shape(
            getattr(start_state, 'velocity', None),
            goal_position,
        )
        goal_acceleration = self._zero_like_or_shape(
            getattr(start_state, 'acceleration', None),
            goal_position,
        )
        goal_jerk = self._zero_like_or_shape(
            getattr(start_state, 'jerk', None),
            goal_position,
        )
        return JointState(
            position=goal_position,
            velocity=goal_velocity,
            acceleration=goal_acceleration,
            jerk=goal_jerk,
            joint_names=list(
                getattr(start_state, 'joint_names', []) or self.mpc.kinematics.joint_names
            ),
        )

    def uses_joint_goal(self) -> bool:
        """Whether the current MPC target is a joint-space goal."""
        return bool(self.goal_joint_positions)

    def _apply_plan_config(self, config: dict):
        """Load planner thresholds from the provided config dictionary."""
        self.convergence_threshold = config.get('convergence_threshold', 0.01)
        self.position_convergence_threshold = config.get(
            'position_convergence_threshold',
            self.convergence_threshold,
        )
        self.rotation_convergence_threshold = config.get(
            'rotation_convergence_threshold',
            0.35,
        )
        self.joint_convergence_threshold = config.get(
            'joint_convergence_threshold',
            0.05,
        )
        self.max_iterations = config.get('max_iterations', 1000)

    def _reset_goal_tracking(self):
        """Reset per-goal tracking statistics."""
        self.initial_pose_error = None
        self.best_pose_error = None
        self.best_position_error = None
        self.best_rotation_error = None
        self.received_goal_update_count = 0
        self.applied_goal_update_count = 0
        self.ignored_goal_update_count = 0
        self._classic_handoff_requested = False
        self._classic_handoff_metadata = None

    def _sync_robot_context_start(self, robot_context, start_state: JointState):
        """Visualize the current MPC start state in RobotContext."""
        if robot_context is None:
            return

        start_position = (
            start_state.position[0].cpu().tolist()
            if start_state.position.is_cuda
            else start_state.position[0].tolist()
        )
        joint_names = (
            robot_context.robot_strategy.get_joint_name()
            if robot_context.robot_strategy
            else self.mpc.kinematics.joint_names
        )
        robot_context.set_command(
            joint_names,
            [[0.0] * len(start_position)],
            [[0.0] * len(start_position)],
            [start_position],
        )
        self.node.get_logger().info(
            f"MPC: Initialized robot position to start_state: {[f'{x:.3f}' for x in start_position]}"
        )

    def _set_active_goal(self, start_state, goal_pose, goal_joint_positions):
        """Create and install the active MPC goal buffer."""
        if self.goal_buffer is not None:
            self.mpc.reset()
        self.start_state = start_state
        self.goal_pose = goal_pose
        self.goal_joint_positions = goal_joint_positions
        self.goal_state = (
            self._build_goal_state(start_state, goal_joint_positions)
            if goal_joint_positions is not None
            else None
        )
        self.last_goal_topic_raw = (
            goal_pose.tolist(q_xyzw=True)
            if goal_pose is not None
            else None
        )

        goal = Goal(
            current_state=start_state,
            goal_state=self.goal_state,
            goal_pose=goal_pose if self.goal_state is None else Pose(),
        )
        self.goal_buffer = self.mpc.setup_solve_single(goal, 1)
        self.mpc.update_goal(self.goal_buffer)
        self.is_goal_active = True

    def _goal_setup_metadata(self):
        """Return the common PlannerResult metadata for an initialized MPC goal."""
        goal_mode = "joint_target" if self.uses_joint_goal() else "pose_target"
        return {
            'goal_buffer': self.goal_buffer,
            'convergence_threshold': self.convergence_threshold,
            'position_convergence_threshold': self.position_convergence_threshold,
            'rotation_convergence_threshold': self.rotation_convergence_threshold,
            'joint_convergence_threshold': self.joint_convergence_threshold,
            'max_iterations': self.max_iterations,
            'goal_mode': goal_mode,
        }

    def _log_goal_setup(self):
        """Emit goal-setup logging after the MPC goal buffer is ready."""
        goal_mode = "joint_target" if self.uses_joint_goal() else "pose_target"
        if self.uses_joint_goal():
            self.node.get_logger().info(
                f"MPC goal setup ({goal_mode}): "
                f"joint_convergence={self.joint_convergence_threshold}, "
                f"speed_scale={self._get_command_speed_scale():.3f}, "
                f"max_iter={self.max_iterations}"
            )
        else:
            self.node.get_logger().info(
                f"MPC goal setup ({goal_mode}): "
                f"pos_convergence={self.position_convergence_threshold}m, "
                f"rot_convergence={self.rotation_convergence_threshold}rad, "
                f"speed_scale={self._get_command_speed_scale():.3f}, "
                f"max_iter={self.max_iterations}"
            )

    def goal_request_differs(self, goal_request) -> bool:
        """Return True when the provided request differs from the active MPC goal."""
        if self.goal_buffer is None or not self.is_goal_active:
            return True

        goal_joint_positions = self._extract_goal_joint_positions(goal_request)
        goal_pose = self._extract_goal_pose(
            goal_request,
            required=(goal_joint_positions is None),
        )

        if self.uses_joint_goal() != bool(goal_joint_positions):
            return True

        if goal_joint_positions is not None:
            if self.goal_joint_positions is None or len(goal_joint_positions) != len(self.goal_joint_positions):
                return True
            joint_epsilon = float(self.node.get_parameter('mpc_goal_update_joint_epsilon').value)
            return any(
                abs(float(new_value) - float(old_value)) > joint_epsilon
                for new_value, old_value in zip(goal_joint_positions, self.goal_joint_positions)
            )

        if self.goal_pose is None or goal_pose is None:
            return self.goal_pose is not goal_pose

        new_pose_values = goal_pose.tolist(q_xyzw=True)
        old_pose_values = self.goal_pose.tolist(q_xyzw=True)
        position_delta = max(
            abs(float(new_value) - float(old_value))
            for new_value, old_value in zip(new_pose_values[:3], old_pose_values[:3])
        )
        orientation_delta = max(
            abs(float(new_value) - float(old_value))
            for new_value, old_value in zip(new_pose_values[3:], old_pose_values[3:])
        )
        return (
            position_delta > float(self.node.get_parameter('mpc_goal_update_position_epsilon').value)
            or orientation_delta > float(self.node.get_parameter('mpc_goal_update_orientation_epsilon').value)
        )

    def plan(self, start_state: JointState, goal_request, config: dict, robot_context=None) -> PlannerResult:
        """
        Setup MPC goal buffer for closed-loop execution.

        Unlike open-loop planners, this doesn't generate a full trajectory.
        Instead, it prepares the MPC solver for iterative execution.

        Args:
            start_state: Initial joint configuration
            goal_request: TrajectoryGeneration request (uses target_pose)
            config: Dictionary with keys:
                - convergence_threshold: Error threshold for goal (default 0.01)
                - max_iterations: Maximum MPC iterations (default 1000)
            robot_context: Optional RobotContext (not used for MPC, visualizes during execute)

        Returns:
            PlannerResult with goal buffer setup status
        """
        if self.mpc is None:
            return PlannerResult(
                success=False,
                message="MPC solver not initialized. Call set_mpc_solver() first.",
            )

        goal_joint_positions = self._extract_goal_joint_positions(goal_request)
        goal_pose = self._extract_goal_pose(
            goal_request,
            required=(goal_joint_positions is None),
        )

        # Store for execution
        self._reset_goal_tracking()
        self.trajectory_constraints = list(
            getattr(goal_request, 'trajectory_constraints', []) or []
        )

        # Extract config
        self._apply_plan_config(config)

        try:
            self._set_active_goal(start_state, goal_pose, goal_joint_positions)
            self._sync_robot_context_start(robot_context, start_state)
            self._log_goal_setup()

            return PlannerResult(
                success=True,
                message="MPC goal buffer initialized",
                trajectory=None,  # No pre-computed trajectory for MPC
                metadata=self._goal_setup_metadata(),
            )

        except Exception as e:
            self.node.get_logger().error(f"MPC setup error: {e}")
            
            self.node.get_logger().error(traceback.format_exc())

            return PlannerResult(
                success=False,
                message=f"MPC setup error: {str(e)}",
            )

    def update_goal_pose(
        self,
        new_goal_pose: Pose,
        current_state: Optional[JointState] = None,
    ) -> bool:
        """
        Update MPC goal during execution (for real-time tracking).

        Args:
            new_goal_pose: New target end-effector pose
            current_state: Latest measured robot joint state to seed the new goal buffer

        Returns:
            True if goal update succeeded
        """
        try:
            goal_start_state = current_state if current_state is not None else self.start_state
            if goal_start_state is None:
                raise ValueError("Cannot update MPC goal without a valid start state.")
            self._set_active_goal(goal_start_state, new_goal_pose, None)
            # Restart progress tracking against the new target so debug output is
            # meaningful after a live target change.
            self.initial_pose_error = None
            self.best_pose_error = None
            self.best_position_error = None
            self.best_rotation_error = None
            self.applied_goal_update_count += 1

            return True

        except Exception as e:
            self.node.get_logger().error(f"Failed to update MPC goal: {e}")
            return False

    def refresh_hybrid_goal(self, start_state: JointState, goal_request, config: dict) -> PlannerResult:
        """
        Reuse the active MPC goal buffer when the hybrid local target is unchanged.

        MoveIt Hybrid Planning may call the local step service at a fixed control rate while
        keeping the same sampled local waypoint for multiple cycles. Rebuilding the MPC goal
        buffer on every tick prevents the solver from behaving like a persistent controller.
        """
        self._apply_plan_config(config)
        self.trajectory_constraints = list(
            getattr(goal_request, 'trajectory_constraints', []) or []
        )

        if self.goal_request_differs(goal_request):
            return self.plan(start_state, goal_request, config, robot_context=None)

        self.start_state = start_state
        return PlannerResult(
            success=True,
            message="MPC goal buffer reused",
            trajectory=None,
            metadata=self._goal_setup_metadata(),
        )

    def build_classic_handoff_request(self):
        """Build a single-pose MotionGen request from the latest MPC goal pose."""
        if (
            self.goal_pose is None
            and self.latest_goal_from_topic is None
            and not self.uses_joint_goal()
        ):
            return None

        request = TrajectoryGeneration.Request()
        if self.uses_joint_goal():
            request.target_joint_positions = [
                float(value) for value in self.goal_joint_positions
            ]
        else:
            goal_values = (
                list(self.latest_goal_from_topic)
                if self.latest_goal_from_topic is not None
                else self.goal_pose.tolist(q_xyzw=True)
            )

            request.target_pose.position.x = float(goal_values[0])
            request.target_pose.position.y = float(goal_values[1])
            request.target_pose.position.z = float(goal_values[2])
            request.target_pose.orientation.x = float(goal_values[3])
            request.target_pose.orientation.y = float(goal_values[4])
            request.target_pose.orientation.z = float(goal_values[5])
            request.target_pose.orientation.w = float(goal_values[6])

        if self.trajectory_constraints:
            request.trajectory_constraints = list(self.trajectory_constraints)

        return request

    def _request_classic_handoff(self, position_error, rotation_error, step_index):
        """Record a near-goal fallback request for a final classic plan."""
        self._classic_handoff_requested = True
        self._classic_handoff_metadata = {
            'step_index': int(step_index),
            'position_error': float(position_error),
            'rotation_error': float(rotation_error),
        }
        self.node.get_logger().info(
            "MPC requested classic finisher handoff at "
            f"step={step_index}, position_error={float(position_error):.4f}m, "
            f"rotation_error={float(rotation_error):.4f}rad"
        )

    def consume_classic_handoff_request(self):
        """Return and clear any pending MPC -> classic handoff request."""
        metadata = self._classic_handoff_metadata if self._classic_handoff_requested else None
        self._classic_handoff_requested = False
        self._classic_handoff_metadata = None
        return metadata

    @staticmethod
    def _metric_scalar(value):
        """Best-effort conversion of cuRobo metric tensors to a Python float."""
        if value is None:
            return None
        if isinstance(value, torch.Tensor):
            if value.numel() == 0:
                return None
            return float(value.detach().abs().max().item())
        try:
            return float(value)
        except (TypeError, ValueError):
            return None

    @staticmethod
    def _max_abs_delta(row_a, row_b):
        """Maximum absolute difference between two equal-length numeric vectors."""
        if row_a is None or row_b is None or len(row_a) != len(row_b):
            return None
        return max(abs(float(value_a) - float(value_b)) for value_a, value_b in zip(row_a, row_b))

    @staticmethod
    def _format_vector(values, precision=3):
        """Compact string formatting for short numeric vectors."""
        if values is None:
            return "[]"
        return "[" + ", ".join(f"{float(value):.{precision}f}" for value in values) + "]"

    def _get_command_speed_scale(self):
        """Return the configured MPC command scaling clamped to [0, 1]."""
        raw_value = float(self.node.get_parameter('mpc_command_speed_scale').value)
        if raw_value < 0.0:
            self.node.get_logger().warn(
                f"mpc_command_speed_scale={raw_value:.3f} is invalid; clamping to 0.0"
            )
            return 0.0
        if raw_value > 1.0:
            self.node.get_logger().warn(
                f"mpc_command_speed_scale={raw_value:.3f} is invalid; clamping to 1.0"
            )
            return 1.0
        return raw_value

    def _scale_stream_command(
        self,
        current_positions,
        current_joint_names,
        target_positions,
        target_velocities,
        target_accelerations,
        target_joint_names,
    ):
        """
        Reduce the per-cycle MPC position delta before publishing to the controller.
        """
        speed_scale = self._get_command_speed_scale()
        if speed_scale >= 1.0:
            return target_positions, target_velocities, target_accelerations

        current_by_name = {
            joint_name: float(position)
            for joint_name, position in zip(current_joint_names, current_positions)
        }
        if any(joint_name not in current_by_name for joint_name in target_joint_names):
            return target_positions, target_velocities, target_accelerations

        scaled_positions = []
        for row in target_positions:
            scaled_positions.append([
                current_by_name[joint_name] + speed_scale * (
                    float(row[index]) - current_by_name[joint_name]
                )
                for index, joint_name in enumerate(target_joint_names)
            ])

        scaled_velocities = [
            [speed_scale * float(value) for value in row]
            for row in target_velocities
        ]
        scaled_accelerations = [
            [speed_scale * float(value) for value in row]
            for row in target_accelerations
        ]
        return scaled_positions, scaled_velocities, scaled_accelerations

    def execute(self, robot_context, goal_handle=None) -> bool:
        """
        Execute MPC closed-loop control.

        Iteratively:
        1. Get current robot state
        2. Solve MPC optimization
        3. Send next action to robot
        4. Repeat until goal reached or timeout

        Args:
            robot_context: RobotContext for sending commands
            goal_handle: Optional ROS action goal handle for feedback

        Returns:
            True if goal reached successfully
        """
        if self.goal_buffer is None:
            self.node.get_logger().error("MPC not initialized. Call plan() first.")
            return False

        try:
            # Initialize MPC loop
            current_state = self.start_state.clone()
            converged = False
            tstep = 0
            self.mpc_time = []
            self._classic_handoff_requested = False
            self._classic_handoff_metadata = None
            execution_interface = getattr(
                self.node,
                "get_mpc_execution_interface",
                lambda: "stream_position",
            )()
            stream_mode = execution_interface == 'stream_position'
            controller_dt = max(
                float(self.node.get_parameter('mpc_step_dt').value),
                1e-3,
            )
            step_max_attempts = max(
                int(self.node.get_parameter('mpc_step_max_attempts').value),
                1,
            )
            stream_publish_count = 0
            infeasible_streak = 0
            divergence_warned = False
            joint_goal_active = self.uses_joint_goal()
            classic_handoff_enabled = bool(
                self.node.get_parameter('mpc_classic_handoff_enabled').value
            ) and not joint_goal_active
            classic_handoff_distance = max(
                float(self.node.get_parameter('mpc_classic_handoff_distance').value),
                0.0,
            )

            self.node.get_logger().info("Starting MPC execution loop")
            if not stream_mode:
                self.node.get_logger().error(
                    f"Unsupported MPC execution interface: {execution_interface}"
                )
                return False
            if not self.node.activate_mpc_streaming_controller():
                self.node.get_logger().error("Failed to activate MPC streaming controller.")
                return False
            self.node.emit_mpc_debug(
                " ".join([
                    "config",
                    f"execution_interface={execution_interface}",
                    f"controller_dt={controller_dt:.3f}",
                    f"speed_scale={self._get_command_speed_scale():.3f}",
                    f"goal_mode={'joint' if joint_goal_active else 'pose'}",
                    f"step_max_attempts={step_max_attempts}",
                    f"position_threshold={self.position_convergence_threshold:.4f}",
                    f"rotation_threshold={self.rotation_convergence_threshold:.4f}",
                    f"joint_threshold={self.joint_convergence_threshold:.4f}",
                    f"classic_handoff_enabled={classic_handoff_enabled}",
                    f"classic_handoff_distance={classic_handoff_distance:.4f}",
                    f"max_iterations={self.max_iterations}",
                ])
            )
            self.node.sync_mpc_world()

            while not converged and self.is_goal_active:
                loop_started_at = time.monotonic()
                self.node.sync_mpc_world()
                # Check for cancellation
                if goal_handle is not None and not goal_handle.is_active:
                    self.node.get_logger().warn("MPC execution cancelled")
                    self.node.publish_mpc_stream_position(
                        self.node.get_controller_joint_names(),
                        actual_joint_state['position'] if 'actual_joint_state' in locals() else self.start_state.position[0].detach().cpu().tolist(),
                    )
                    return False

                # Read actual robot state to close the feedback loop.
                # Velocities matter for MPC smoothness; forcing zero at every step
                # makes the optimizer behave as if the robot keeps stopping.
                actual_joint_state = self.node.get_current_joint_state()
                if not actual_joint_state:
                    self.node.get_logger().error(
                        "MPC execution requires a recent /joint_states sample."
                    )
                    return False
                current_state = JointState(
                    position=torch.tensor(
                        [actual_joint_state['position']],
                        device=self.node.tensor_args.device,
                        dtype=self.node.tensor_args.dtype,
                    ),
                    velocity=torch.tensor(
                        [actual_joint_state['velocity']],
                        device=self.node.tensor_args.device,
                        dtype=self.node.tensor_args.dtype,
                    ),
                    acceleration=torch.tensor(
                        [actual_joint_state.get('acceleration', [0.0] * len(actual_joint_state['position']))],
                        device=self.node.tensor_args.device,
                        dtype=self.node.tensor_args.dtype,
                    ),
                    jerk=torch.tensor(
                        [actual_joint_state.get('jerk', [0.0] * len(actual_joint_state['position']))],
                        device=self.node.tensor_args.device,
                        dtype=self.node.tensor_args.dtype,
                    ),
                    joint_names=list(actual_joint_state.get('joint_names', []))
                    or (
                        list(self.node._robot_joint_names)
                        if getattr(self.node, "_robot_joint_names", None)
                        else None
                    ),
                )
                self.start_state = current_state

                # Apply the latest RViz target against the current measured state.
                # Re-seeding the goal buffer from the original planning start causes
                # the solver to keep "jumping back" whenever tracking updates arrive.
                if self.latest_goal_from_topic is not None:
                    raw = self.latest_goal_from_topic
                    self.latest_goal_from_topic = None  # Consume before CUDA work
                    new_goal = Pose.from_list(raw, q_xyzw=True)  # Safe: we are on the MPC thread
                    self.update_goal_pose(new_goal, current_state=current_state)
                joint_goal_active = self.uses_joint_goal()

                st_time = time.time()

                # MPC optimization step with actual robot state
                result = self.mpc.step(
                    current_state,
                    shift_steps=1,
                    max_attempts=step_max_attempts,
                )
                pose_error = self._metric_scalar(getattr(result.metrics, 'pose_error', None))
                position_error = self._metric_scalar(getattr(result.metrics, 'position_error', None))
                rotation_error = self._metric_scalar(getattr(result.metrics, 'rotation_error', None))
                cspace_error = self._metric_scalar(getattr(result.metrics, 'cspace_error', None))
                tracking_error = (
                    cspace_error
                    if joint_goal_active and cspace_error is not None
                    else pose_error
                )
                total_cost = self._metric_scalar(getattr(result.metrics, 'cost', None))
                constraint_cost = self._metric_scalar(getattr(result.metrics, 'constraint', None))
                feasible = bool(torch.all(result.metrics.feasible).item()) if result.metrics.feasible is not None else None
                current_kinematics = self.mpc.compute_kinematics(current_state)
                current_ee_pose = current_kinematics.ee_pose.tolist(q_xyzw=True)
                goal_ee_pose = (
                    self.goal_pose.tolist(q_xyzw=True)
                    if self.goal_pose is not None
                    else None
                )
                infeasible_streak = 0 if feasible else infeasible_streak + 1
                if self.initial_pose_error is None:
                    self.initial_pose_error = max(
                        tracking_error if tracking_error is not None else self.convergence_threshold,
                        self.convergence_threshold,
                        1e-6,
                    )
                if tracking_error is not None:
                    self.best_pose_error = (
                        tracking_error
                        if self.best_pose_error is None
                        else min(self.best_pose_error, tracking_error)
                    )
                if position_error is not None:
                    self.best_position_error = (
                        position_error
                        if self.best_position_error is None
                        else min(self.best_position_error, position_error)
                    )
                if rotation_error is not None:
                    self.best_rotation_error = (
                        rotation_error
                        if self.best_rotation_error is None
                        else min(self.best_rotation_error, rotation_error)
                    )

                if (
                    not divergence_warned
                    and self.best_pose_error is not None
                    and tracking_error is not None
                    and tstep > 20
                    and tracking_error > max(self.best_pose_error * 1.75, self.best_pose_error + 0.5)
                ):
                    self.node.get_logger().warn(
                        "MPC appears to be diverging away from its best state: "
                        f"step={tstep}, tracking_error={tracking_error:.4f}, "
                        f"best_pose_error={self.best_pose_error:.4f}, "
                        f"position_error={(position_error if position_error is not None else float('nan')):.4f}, "
                        f"rotation_error={(rotation_error if rotation_error is not None else float('nan')):.4f}, "
                        f"stream_publishes={stream_publish_count}"
                    )
                    divergence_warned = True

                # Synchronize CUDA
                torch.cuda.synchronize()

                # Track timing (skip first few iterations for warmup)
                if tstep > 5:
                    self.mpc_time.append(time.time() - st_time)

                extractor = getattr(self.node, "_extract_first_valid_ordered_command", None)
                if callable(extractor):
                    immediate_positions, immediate_velocities, immediate_accelerations = extractor(
                        getattr(result, 'js_action', None),
                        getattr(result, 'action', None),
                        max_points=1,
                    )
                else:
                    immediate_positions, immediate_velocities, immediate_accelerations = extract_ordered_waypoints(
                        result.js_action if result.js_action is not None else result.action,
                        self.node.get_controller_joint_names(),
                        max_points=1,
                    )
                if not immediate_positions:
                    self.node.get_logger().error("Failed to extract a finite MPC stream command.")
                    return False
                joint_names = self.node.get_controller_joint_names()
                immediate_positions, immediate_velocities, immediate_accelerations = self._scale_stream_command(
                    actual_joint_state['position'],
                    actual_joint_state.get('joint_names', []),
                    immediate_positions,
                    immediate_velocities,
                    immediate_accelerations,
                    joint_names,
                )
                robot_context.set_command(
                    joint_names,
                    immediate_velocities,
                    immediate_accelerations,
                    immediate_positions,
                )
                if not self.node.publish_mpc_stream_position(
                    joint_names,
                    immediate_positions[0],
                ):
                    return False
                stream_publish_count += 1

                # Publish feedback (use generic SendTrajectory feedback)
                if goal_handle is not None and tstep % 10 == 0:
                    feedback_msg = SendTrajectory.Feedback()
                    progress_error = (
                        tracking_error
                        if tracking_error is not None
                        else self.initial_pose_error
                    )
                    progress = 1.0 - min(
                        progress_error / max(self.initial_pose_error, 1e-6),
                        1.0,
                    )
                    feedback_msg.step_progression = progress
                    goal_handle.publish_feedback(feedback_msg)

                if self.node.should_log_mpc_debug(tstep):
                    first_waypoint_delta = self._max_abs_delta(
                        actual_joint_state['position'],
                        immediate_positions[0] if immediate_positions else None,
                    )
                    velocity_peak = max(
                        abs(float(value)) for value in actual_joint_state.get('velocity', [])
                    ) if actual_joint_state.get('velocity') else 0.0
                    collision_debug = self.node.get_state_collision_debug(
                        actual_joint_state['position']
                    )
                    self.node.emit_mpc_debug(
                        " ".join([
                            f"step={tstep}",
                            f"goal_mode={'joint' if joint_goal_active else 'pose'}",
                            f"pose_error={(pose_error if pose_error is not None else float('nan')):.5f}",
                            f"best_pose_error={(self.best_pose_error if self.best_pose_error is not None else float('nan')):.5f}",
                            f"best_position_error={(self.best_position_error if self.best_position_error is not None else float('nan')):.5f}",
                            f"best_rotation_error={(self.best_rotation_error if self.best_rotation_error is not None else float('nan')):.5f}",
                            f"position_error={(position_error if position_error is not None else float('nan')):.5f}",
                            f"rotation_error={(rotation_error if rotation_error is not None else float('nan')):.5f}",
                            f"cspace_error={(cspace_error if cspace_error is not None else float('nan')):.5f}",
                            f"solve_ms={float(result.solve_time) * 1000.0:.2f}",
                            f"feasible={feasible}",
                            f"cost={(total_cost if total_cost is not None else float('nan')):.5f}",
                            f"constraint={(constraint_cost if constraint_cost is not None else float('nan')):.5f}",
                            f"first_cmd_delta={(first_waypoint_delta if first_waypoint_delta is not None else float('nan')):.5f}",
                            f"joint_vel_peak={velocity_peak:.5f}",
                            f"stream_publishes={stream_publish_count}",
                            f"infeasible_streak={infeasible_streak}",
                            f"goal_updates_rx={self.received_goal_update_count}",
                            f"goal_updates_applied={self.applied_goal_update_count}",
                            f"goal_updates_ignored={self.ignored_goal_update_count}",
                            f"min_sphere_dist={(collision_debug['min_sphere_dist'] if collision_debug is not None else float('nan')):.5f}",
                            f"max_sphere_dist={(collision_debug['max_sphere_dist'] if collision_debug is not None else float('nan')):.5f}",
                            f"world_collision_free={collision_debug['world_collision_free'] if collision_debug is not None else 'unknown'}",
                        ]),
                        step_index=tstep,
                    )
                    self.node.emit_mpc_debug(
                        " ".join([
                            f"ee_current_pos={self._format_vector(current_ee_pose[:3])}",
                            f"ee_goal_pos={self._format_vector(goal_ee_pose[:3] if goal_ee_pose is not None else None)}",
                            f"ee_current_quat={self._format_vector(current_ee_pose[3:])}",
                            f"ee_goal_quat={self._format_vector(goal_ee_pose[3:] if goal_ee_pose is not None else None)}",
                        ]),
                        step_index=tstep,
                    )

                if infeasible_streak >= max(
                    int(self.node.get_parameter('mpc_infeasible_abort_steps').value),
                    1,
                ):
                    self.node.get_logger().warn(
                        "Aborting MPC streaming after repeated infeasible solves: "
                        f"streak={infeasible_streak}, "
                        f"position_error={(position_error if position_error is not None else float('nan')):.4f}m, "
                        f"rotation_error={(rotation_error if rotation_error is not None else float('nan')):.4f}rad"
                    )
                    return False

                # Check convergence
                if (
                    joint_goal_active
                    and cspace_error is not None
                    and cspace_error < self.joint_convergence_threshold
                ):
                    converged = True
                    self.node.get_logger().info(
                        "MPC converged at step "
                        f"{tstep} with cspace_error={cspace_error:.4f}"
                    )
                elif (
                    position_error is not None
                    and rotation_error is not None
                    and position_error < self.position_convergence_threshold
                    and rotation_error < self.rotation_convergence_threshold
                ):
                    converged = True
                    self.node.get_logger().info(
                        "MPC converged at step "
                        f"{tstep} with position_error={position_error:.4f}m "
                        f"and rotation_error={rotation_error:.4f}rad"
                    )
                elif (
                    classic_handoff_enabled
                    and position_error is not None
                    and rotation_error is not None
                    and position_error <= classic_handoff_distance
                    and rotation_error <= self.rotation_convergence_threshold
                ):
                    self._request_classic_handoff(
                        position_error=position_error,
                        rotation_error=rotation_error,
                        step_index=tstep,
                    )
                    break

                remaining_time = controller_dt - (time.monotonic() - loop_started_at)
                if remaining_time > 0.0:
                    time.sleep(remaining_time)

                tstep += 1

                if tstep >= self.max_iterations:
                    self.node.get_logger().warn(
                        f"MPC max iterations ({self.max_iterations}) reached without convergence "
                        f"(last position_error: "
                        f"{(position_error if position_error is not None else float('nan')):.4f}m, "
                        f"last rotation_error: "
                        f"{(rotation_error if rotation_error is not None else float('nan')):.4f}rad, "
                        f"last cspace_error: "
                        f"{(cspace_error if cspace_error is not None else float('nan')):.4f})"
                    )
                    break

            # Log statistics
            if self.mpc_time:
                avg_time = sum(self.mpc_time) / len(self.mpc_time)
                self.node.get_logger().info(
                    f"MPC completed: {tstep} steps, avg time={avg_time*1000:.1f}ms/step"
                )
            self.node.emit_mpc_debug(
                " ".join([
                    f"summary steps={tstep}",
                    f"converged={converged}",
                    f"goal_mode={'joint' if joint_goal_active else 'pose'}",
                    f"best_pose_error={(self.best_pose_error if self.best_pose_error is not None else float('nan')):.5f}",
                    f"best_position_error={(self.best_position_error if self.best_position_error is not None else float('nan')):.5f}",
                    f"best_rotation_error={(self.best_rotation_error if self.best_rotation_error is not None else float('nan')):.5f}",
                    f"final_pose_error={(pose_error if 'pose_error' in locals() and pose_error is not None else float('nan')):.5f}",
                    f"final_position_error={(position_error if 'position_error' in locals() and position_error is not None else float('nan')):.5f}",
                    f"final_rotation_error={(rotation_error if 'rotation_error' in locals() and rotation_error is not None else float('nan')):.5f}",
                    f"final_cspace_error={(cspace_error if 'cspace_error' in locals() and cspace_error is not None else float('nan')):.5f}",
                    f"classic_handoff_requested={self._classic_handoff_requested}",
                    f"stream_publishes={stream_publish_count}",
                    f"goal_updates_rx={self.received_goal_update_count}",
                    f"goal_updates_applied={self.applied_goal_update_count}",
                    f"goal_updates_ignored={self.ignored_goal_update_count}",
                ])
            )

            if not converged:
                self.node.publish_mpc_stream_position(
                    self.node.get_controller_joint_names(),
                    actual_joint_state['position'] if 'actual_joint_state' in locals() else self.start_state.position[0].detach().cpu().tolist(),
                )
                if self._classic_handoff_requested:
                    self.node.get_logger().info(
                        "MPC stopped streaming and is waiting for the classic finishing plan."
                    )
            return converged

        except Exception as e:
            self.node.get_logger().error(f"MPC execution error: {e}")
            self.node.get_logger().error(traceback.format_exc())
            try:
                self.node.publish_mpc_stream_position(
                    self.node.get_controller_joint_names(),
                    current_state.position[0].detach().cpu().tolist(),
                )
            except Exception:
                pass
            robot_context.stop_robot()
            return False

    def cancel(self):
        """Cancel the current MPC execution."""
        self.is_goal_active = False
        self.node.get_logger().info("MPC execution cancelled")
