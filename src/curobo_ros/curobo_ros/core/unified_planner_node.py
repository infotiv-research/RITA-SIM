#!/usr/bin/env python3
"""
Unified trajectory planner node using Strategy Pattern.

This node supports multiple planning strategies (Classic, MPC, etc.)
and allows dynamic switching between them.
"""

import math
import torch
import rclpy
import time
import threading
from rclpy.node import Node
from rclpy.action import ActionClient, ActionServer
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup

from control_msgs.action import FollowJointTrajectory
from controller_manager_msgs.srv import ListControllers, SwitchController
from sensor_msgs.msg import JointState as JointStateMsg
from std_msgs.msg import Float64MultiArray, String
from curobo_msgs.srv import (
    TrajectoryGeneration,
    SetPlanner,
    GetPlanners,
    MpcStep,
    MpcReset,
)
from curobo_msgs.action import SendTrajectory, MpcMove

from curobo.types.base import TensorDeviceType
from curobo.types.math import Pose
from curobo.types.robot import JointState
from curobo.wrap.reacher.mpc import MpcSolver, MpcSolverConfig
from curobo.cuda_robot_model.cuda_robot_model import CudaRobotModel
from curobo.wrap.wrap_base import WrapBase
from curobo.geom.types import Cuboid
from curobo.geom.sdf.world import CollisionQueryBuffer
from curobo.util.logger import log_warn
from curobo.util_file import load_yaml
import curobo.util.trajectory as curobo_trajectory_utils
import curobo.wrap.reacher.motion_gen as curobo_motion_gen_module
import curobo.wrap.reacher.trajopt as curobo_trajopt_module

from curobo_ros.robot.robot_context import RobotContext
from curobo_ros.core.controller_trajectory import (
    build_joint_trajectory,
    extract_ordered_waypoints,
    wait_future_result,
)
from curobo_ros.core.config_wrapper_motion import ConfigWrapperMotion, ConfigWrapperMPC
from curobo_ros.core.mpc_solver_config_utils import (
    build_mpc_load_from_robot_config_kwargs,
)
from curobo_ros.planners import (
    PlannerFactory,
    PlannerManager,
    ClassicPlanner,
    JointSpacePlanner,
    MPCPlanner,
    SinglePlanner,
)


def _coerce_python_float(value):
    """Convert scalar torch tensors from cuRobo internals into plain Python floats."""
    if isinstance(value, torch.Tensor):
        if value.numel() == 0:
            raise ValueError("Cannot coerce an empty tensor to float.")
        return float(value.detach().cpu().reshape(-1)[0].item())
    return float(value)


def _apply_curobo_compat_patches():
    """Install runtime shims for known cuRobo/Jazzy compatibility issues."""
    current = curobo_trajectory_utils.get_batch_interpolated_trajectory
    if not getattr(current, "_isaac_sim_rita_compat_patch", False):
        original_get_batch_interpolated_trajectory = current

        def patched_get_batch_interpolated_trajectory(*args, **kwargs):
            args = list(args)
            if len(args) >= 10:
                args[9] = _coerce_python_float(args[9])
            if len(args) >= 11:
                args[10] = _coerce_python_float(args[10])
            if "interpolation_dt" in kwargs:
                kwargs["interpolation_dt"] = _coerce_python_float(kwargs["interpolation_dt"])
            if "min_dt" in kwargs:
                kwargs["min_dt"] = _coerce_python_float(kwargs["min_dt"])
            if "max_dt" in kwargs:
                kwargs["max_dt"] = _coerce_python_float(kwargs["max_dt"])
            return original_get_batch_interpolated_trajectory(*args, **kwargs)

        patched_get_batch_interpolated_trajectory._isaac_sim_rita_compat_patch = True
        curobo_trajectory_utils.get_batch_interpolated_trajectory = patched_get_batch_interpolated_trajectory
        curobo_motion_gen_module.get_batch_interpolated_trajectory = patched_get_batch_interpolated_trajectory
        curobo_trajopt_module.get_batch_interpolated_trajectory = patched_get_batch_interpolated_trajectory

    if getattr(MpcSolver.step, "_isaac_sim_rita_compat_patch", False):
        return

    def patched_mpc_step(self, current_state, shift_steps=1, seed_traj=None, max_attempts=1):
        """Retry the installed cuRobo step and mark unrecoverable results infeasible."""
        converged = False
        result = None

        # Flush any pending async CUDA errors from previous operations
        try:
            torch.cuda.synchronize()
        except RuntimeError as _sync_err:
            log_warn(f"CUDA sync error BEFORE mpc step: {_sync_err}")
            raise

        for _ in range(max(int(max_attempts), 1)):
            result = self._step_once(current_state.clone(), shift_steps, seed_traj)
            if (
                torch.count_nonzero(torch.isnan(result.action.position)) == 0
                and torch.count_nonzero(~result.metrics.feasible) == 0
            ):
                converged = True
                break
            self.reset()

        if not converged and result is not None:
            log_warn(
                "Patched cuRobo MPC compatibility layer: solver produced a non-finite or "
                "infeasible action after retries; marking this step infeasible."
            )
            try:
                if result.metrics.feasible is not None:
                    result.metrics.feasible[:] = False
            except Exception:
                pass

        return result

    patched_mpc_step._isaac_sim_rita_compat_patch = True
    MpcSolver.step = patched_mpc_step

    # --- cuRobo CUDA forward kinematics contiguity fix ---
    # The MPC optimizer (MPPI) creates non-contiguous tensor views during
    # rollout (e.g. trajectory[:, t, :]).  When these views reach the CUDA
    # kernel `kin_fused_forward`, it asserts `joint_vec.is_contiguous()`.
    # Fix: ensure `q` is contiguous at the Python/CUDA boundary.
    if not getattr(CudaRobotModel._cuda_forward, "_isaac_sim_rita_compat_patch", False):
        _original_cuda_forward = CudaRobotModel._cuda_forward

        def _patched_cuda_forward(self, q):
            return _original_cuda_forward(self, q.contiguous())

        _patched_cuda_forward._isaac_sim_rita_compat_patch = True
        CudaRobotModel._cuda_forward = _patched_cuda_forward

    # --- MPPI optimizer non-contiguous state fix + CUDA sync barriers ---
    # The MPPI optimizer produces JointState objects whose position/velocity/
    # acceleration/jerk tensors are non-contiguous views (from trajectory
    # slicing).  Downstream Warp/CUDA kernels use .view(-1) which requires
    # contiguous memory.  Additionally, we decompose get_metrics into
    # FK → collision → convergence → cost with torch.cuda.synchronize()
    # barriers between each to prevent async CUDA race conditions between
    # Warp kernels and PyTorch operations.
    if not getattr(WrapBase.get_metrics, "_isaac_sim_rita_compat_patch", False):
        _original_wrap_get_metrics = WrapBase.get_metrics

        def _patched_wrap_get_metrics(self, state, use_cuda_graph=False):
            try:
                torch.cuda.synchronize()
            except RuntimeError as _sync_err:
                log_warn(f"CUDA sync error BEFORE get_metrics: {_sync_err}")
                raise
            for attr in ("position", "velocity", "acceleration", "jerk"):
                val = getattr(state, attr, None)
                if isinstance(val, torch.Tensor) and not val.is_contiguous():
                    setattr(state, attr, val.contiguous())
            # --- Granular CUDA error isolation ---
            rollout = self.safety_rollout
            from curobo.types.state import JointState as _CuroboJS
            if isinstance(state, _CuroboJS):
                state = rollout._get_augmented_state(state)
            try:
                torch.cuda.synchronize()
            except RuntimeError as _e:
                log_warn(f"CUDA error after _get_augmented_state (FK): {_e}")
                raise
            out_metrics = rollout.constraint_fn(state)
            try:
                torch.cuda.synchronize()
            except RuntimeError as _e:
                log_warn(f"CUDA error after constraint_fn (collision): {_e}")
                raise
            out_metrics.state = state
            out_metrics = rollout.convergence_fn(state, out_metrics)
            try:
                torch.cuda.synchronize()
            except RuntimeError as _e:
                log_warn(f"CUDA error after convergence_fn: {_e}")
                raise
            out_metrics.cost = rollout.cost_fn(state)
            return out_metrics

        _patched_wrap_get_metrics._isaac_sim_rita_compat_patch = True
        WrapBase.get_metrics = _patched_wrap_get_metrics


class UnifiedPlannerNode(Node):
    """
    Unified trajectory planning node with multiple strategies.

    Supports:
    - Classic motion generation (open-loop)
    - Model Predictive Control (closed-loop)
    - Dynamic planner switching via ROS service

    Services:
    - /generate_trajectory: Plan a trajectory
    - /set_planner: Switch active planner

    Actions:
    - /execute_trajectory: Execute planned trajectory
    """

    def __init__(self):
        super().__init__('unified_planner')

        _apply_curobo_compat_patches()

        # Initialize tensor arguments
        self.tensor_args = TensorDeviceType()

        # Robot context for command execution
        self.robot_context = RobotContext(self, 0.03)

        # Track the real robot joint state for planning start-state extraction.
        self.declare_parameter('joint_states_topic', '/joint_states')
        self._latest_joint_state_msg = None
        self._robot_joint_names = []
        self._joint_state_sub = self.create_subscription(
            JointStateMsg,
            self.get_parameter('joint_states_topic').get_parameter_value().string_value,
            self._joint_state_callback,
            10,
        )

        # Declare planner selection parameter
        self.declare_parameter('planner_type', 'classic')
        self.declare_parameter('max_attempts', 1)
        self.declare_parameter('timeout', 5.0)
        self.declare_parameter('time_dilation_factor', 0.5)
        self.declare_parameter('voxel_size', 0.05)
        self.declare_parameter('collision_activation_distance', 0.025)
        self.declare_parameter('convergence_threshold', 0.01)
        self.declare_parameter('max_mpc_iterations', 1000)
        self.declare_parameter('mpc_position_convergence_threshold', 0.02)
        self.declare_parameter('mpc_rotation_convergence_threshold', 0.35)
        self.declare_parameter('mpc_joint_convergence_threshold', 0.05)
        self.declare_parameter('mpc_step_dt', 0.03)
        self.declare_parameter('mpc_horizon_steps', 30)
        self.declare_parameter('mpc_use_cuda_graph', False)
        self.declare_parameter('mpc_use_cuda_graph_metrics', False)
        self.declare_parameter('mpc_force_shift_steps_one', False)
        self.declare_parameter('mpc_command_speed_scale', 1.0)
        self.declare_parameter('mpc_step_max_attempts', 2)
        self.declare_parameter('mpc_execution_interface', 'trajectory_action')
        self.declare_parameter('mpc_stream_controller_name', 'forward_position_controller')
        self.declare_parameter('mpc_stream_controller_topic', '/forward_position_controller/commands')
        self.declare_parameter('mpc_deactivate_controller_name', 'joint_trajectory_controller')
        self.declare_parameter(
            'controller_action_name',
            '/joint_trajectory_controller/follow_joint_trajectory',
        )
        self.declare_parameter('controller_manager_switch_service', '/controller_manager/switch_controller')
        self.declare_parameter('reject_colliding_request_joint_states', False)
        self.declare_parameter('mpc_infeasible_abort_steps', 10)
        self.declare_parameter('hybrid_mpc_report_path_invalidated', True)
        self.declare_parameter('hybrid_mpc_stall_steps', 25)
        self.declare_parameter('hybrid_mpc_stall_threshold', 0.01)
        self.declare_parameter('mpc_classic_handoff_enabled', True)
        self.declare_parameter('mpc_classic_handoff_distance', 0.015)
        self.declare_parameter('mpc_debug_logging', False)
        self.declare_parameter('mpc_debug_log_every_n_steps', 10)
        self.declare_parameter('mpc_debug_publish_topic', True)
        # Ignore tiny RViz marker jitter so standalone MPC does not keep
        # rebuilding its goal buffer for what is effectively the same target.
        self.declare_parameter('mpc_goal_update_position_epsilon', 0.005)
        self.declare_parameter('mpc_goal_update_orientation_epsilon', 0.01)
        self.declare_parameter('mpc_goal_update_joint_epsilon', 0.001)
        self.declare_parameter('mpc_dynamic_world_updates_enabled', True)
        self.declare_parameter('startup_ready', False)

        self._world_update_lock = threading.Lock()
        self._hybrid_mpc_lock = threading.Lock()
        self._world_state_version = 0
        self._pending_world_version = 0
        self._motion_gen_world_version = 0
        self._mpc_world_version = 0
        self._mpc_execution_active = False
        self._mpc_robot_geometry_dirty = False
        self._hybrid_mpc_session_active = False
        self._hybrid_mpc_infeasible_streak = 0
        self._hybrid_mpc_last_step_time = None
        self._hybrid_mpc_stall_count = 0
        self._hybrid_mpc_best_error = None
        self.node_is_available = False
        self._startup_ready_logged = False
        self._startup_ready_deadline = None
        self._startup_ready_debounce_sec = 0.75

        # Initialize ONLY the base config wrapper
        # Other wrappers will be created on-demand (lazy loading)
        self.config_wrapper_motion = ConfigWrapperMotion(self, self.robot_context)
        self.config_wrapper_mpc = None  # Created on-demand when MPC is first used

        # Resolve joint order after robot_config_file has been declared by ConfigWrapperMotion.
        self._robot_joint_names = self._load_robot_joint_names()
        self.declare_parameter('controller_joint_names', list(self._robot_joint_names))
        configured_controller_names = list(
            self.get_parameter('controller_joint_names').get_parameter_value().string_array_value
        )
        self.controller_joint_names = configured_controller_names or list(self._robot_joint_names)
        self.controller_action_name = (
            self.get_parameter('controller_action_name').get_parameter_value().string_value
        )
        self.controller_action_client = ActionClient(
            self,
            FollowJointTrajectory,
            self.controller_action_name,
        )
        self._controller_goal_lock = threading.Lock()
        self._active_controller_goal_handle = None
        self.controller_switch_client = self.create_client(
            SwitchController,
            self.get_parameter('controller_manager_switch_service').get_parameter_value().string_value,
        )
        self.controller_list_client = self.create_client(
            ListControllers,
            '/controller_manager/list_controllers',
        )
        self.mpc_stream_controller_name = (
            self.get_parameter('mpc_stream_controller_name').get_parameter_value().string_value
        )
        self.mpc_stream_controller_topic = (
            self.get_parameter('mpc_stream_controller_topic').get_parameter_value().string_value
        )
        self.mpc_deactivate_controller_name = (
            self.get_parameter('mpc_deactivate_controller_name').get_parameter_value().string_value
        )
        self.mpc_stream_publisher = self.create_publisher(
            Float64MultiArray,
            self.mpc_stream_controller_topic,
            10,
        )
        self._startup_ready_timer = self.create_timer(0.1, self._maybe_log_startup_ready)
        self.mpc_debug_publisher = self.create_publisher(
            String,
            f'{self.get_name()}/mpc_debug',
            10,
        )

        # Shared world_cfg for all planners - references ObstacleManager's world_cfg
        # This is a reference, not a copy, so all planners see the same obstacles
        self.shared_world_cfg = self.config_wrapper_motion.obstacle_manager.get_world_cfg()

        # IMPORTANT: shared_world_cfg is a reference to obstacle_manager.world_cfg.
        # When obstacles are added/removed via ROS services, all planners automatically
        # see the changes after update_world_config() is called.

        # Initialize solvers (created on-demand)
        self.motion_gen = None
        self.mpc = None

        # Planner manager (handles caching)
        self.planner_manager = PlannerManager(self, self.config_wrapper_motion)

        # Get initial planner type
        initial_planner = self.get_parameter('planner_type').get_parameter_value().string_value

        # Warmup only the initial planner
        self._warmup_initial_planner(initial_planner)

        # Set initial planner (will be retrieved from cache)
        self.planner_manager.set_current_planner(initial_planner)

        # Create services
        self.generate_trajectory_srv = self.create_service(
            TrajectoryGeneration,
            f'{self.get_name()}/generate_trajectory',
            self.generate_trajectory_callback,
            callback_group=MutuallyExclusiveCallbackGroup()
        )

        self.set_planner_srv = self.create_service(
            SetPlanner,
            f'{self.get_name()}/set_planner',
            self.set_planner_callback,
            callback_group=MutuallyExclusiveCallbackGroup()
        )

        # Service to get available planners (structured, for RViz plugin)
        self.get_planners_srv = self.create_service(
            GetPlanners,
            f'{self.get_name()}/get_planners',
            self.get_planners_callback,
            callback_group=MutuallyExclusiveCallbackGroup()
        )

        self.mpc_step_srv = self.create_service(
            MpcStep,
            f'{self.get_name()}/mpc_step',
            self.mpc_step_callback,
            callback_group=MutuallyExclusiveCallbackGroup(),
        )

        self.mpc_reset_srv = self.create_service(
            MpcReset,
            f'{self.get_name()}/mpc_reset',
            self.mpc_reset_callback,
            callback_group=MutuallyExclusiveCallbackGroup(),
        )

        # Create action server (unified for all planner types)
        self._action_server = ActionServer(
            self,
            SendTrajectory,
            f'{self.get_name()}/execute_trajectory',
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
            callback_group=MutuallyExclusiveCallbackGroup()
        )

        # Create subscription for MPC goal updates (for real-time tracking)
        from geometry_msgs.msg import Pose as PoseMsg
        self.mpc_goal_sub = self.create_subscription(
            PoseMsg,
            f'{self.get_name()}/mpc_goal',
            self.mpc_goal_callback,
            10
        )

        self.get_logger().info(
            f"Unified planner ready with initial planner: "
            f"{self.planner_manager.get_current_planner().get_planner_name()}"
        )

    def _warmup_initial_planner(self, planner_type: str):
        """
        Warmup only the initial planner (lazy loading).

        Args:
            planner_type: Type of planner to warmup ('classic', 'mpc', etc.)
        """
        self.get_logger().info(f"Warming up {planner_type} planner...")
        self.node_is_available = False

        if planner_type == 'classic':
            self._warmup_classic()
        elif planner_type == 'mpc':
            self._warmup_mpc()
        else:
            # For future planners (batch, constrained), default to classic config
            self._warmup_classic()
            self.get_logger().warn(
                f"Planner '{planner_type}' not fully implemented yet, "
                "using classic warmup"
            )

        self.node_is_available = True
        self.get_logger().info(f"✅ {planner_type} planner ready")

    def _warmup_classic(self):
        """Warmup MotionGen for Classic/Batch/Constrained planners."""
        if self.motion_gen is None:
            self.get_logger().info("  → Initializing MotionGen...")
            self.config_wrapper_motion.set_motion_gen_config(self, None, None)
            self.motion_gen = self.motion_gen  # Set by config wrapper

            # Share MotionGen instance with all SinglePlanner children
            # This allows switching between SinglePlanner-based planners without re-warmup
            SinglePlanner.set_motion_gen(self.motion_gen)
            self._record_solver_world_sync('motion_gen', self._get_world_state_version())
            self.get_logger().info("  → MotionGen ready and shared with SinglePlanner hierarchy")
        else:
            self.get_logger().info("  → MotionGen already initialized (using cache)")

    def _warmup_mpc(self):
        """Warmup MPC solver on-demand."""
        if self.mpc is None:
            self.get_logger().info("  → Initializing MPC solver...")

            # Get robot config from existing wrapper
            robot_cfg = self.config_wrapper_motion.robot_cfg

            # Add ground plane to shared world_cfg if not already there
            ground_exists = any(obj.name == "ground" for obj in self.shared_world_cfg.objects)
            if not ground_exists:
                ground_plane = Cuboid(
                    name="ground",
                    pose=[0, 0, -0.1, 1, 0, 0, 0],
                    dims=[3.0, 3.0, 0.01],
                    color=[0.5, 0.5, 0.5, 1.0]
                )
                self.shared_world_cfg.add_obstacle(ground_plane)
                self._record_world_dirty()
                self.get_logger().info("  → Added ground plane to shared world_cfg")

            # MPC must use its own collision checker — MotionGen's checker
            # pre-allocates buffers sized for (num_seeds × trajopt_tsteps) which
            # differ from MPC's (num_particles × horizon).  Sharing causes
            # out-of-bounds CUDA writes on the 2nd MPC step.
            shared_checker = None
            self.get_logger().info("  → Using separate world_coll_checker for MPC (independent VRAM)")

            with build_mpc_load_from_robot_config_kwargs(
                self,
                world_coll_checker=shared_checker,
            ) as mpc_config_kwargs:
                mpc_config = MpcSolverConfig.load_from_robot_config(
                    robot_cfg,
                    self.shared_world_cfg,
                    **mpc_config_kwargs,
                )

            self.mpc = MpcSolver(mpc_config)
            self._record_solver_world_sync('mpc', self._get_world_state_version())
            self.get_logger().info(
                "  → MPC solver ready "
                f"(collision_activation_distance={float(self.get_parameter('collision_activation_distance').value):.3f}m, "
                f"horizon={int(self.get_parameter('mpc_horizon_steps').value)} steps)"
            )
        else:
            self.get_logger().info("  → MPC solver already initialized (using cache)")

    def _solvers_share_world_checker(self):
        """Return whether MotionGen and MPC are backed by the same collision checker."""
        return (
            self.motion_gen is not None
            and self.mpc is not None
            and self.motion_gen.world_coll_checker is self.mpc.world_coll_checker
        )

    def _get_shared_robot_kinematics_target(self):
        """Return the base robot-kinematics config used for future solver warmups."""
        return getattr(getattr(self.config_wrapper_motion, 'robot_cfg', None), 'kinematics', None)

    def _iter_solver_robot_models(self):
        """Yield unique active CudaRobotModel instances that need config refresh."""
        targets = []

        robot_model_manager = getattr(self.config_wrapper_motion, 'robot_model_manager', None)
        if robot_model_manager is not None and getattr(robot_model_manager, 'kin_model', None) is not None:
            targets.append(('robot_model_manager', robot_model_manager.kin_model))

        if self.motion_gen is not None:
            motion_gen_targets = []
            get_all_kinematics_instances = getattr(self.motion_gen, 'get_all_kinematics_instances', None)
            if callable(get_all_kinematics_instances):
                motion_gen_targets.extend(list(get_all_kinematics_instances() or []))
            else:
                motion_gen_kinematics = getattr(self.motion_gen, 'kinematics', None)
                if motion_gen_kinematics is not None:
                    motion_gen_targets.append(motion_gen_kinematics)

            for index, target in enumerate(motion_gen_targets):
                if target is not None:
                    targets.append((f'motion_gen[{index}]', target))

        if self.mpc is not None:
            rollout_targets = []
            rollout_fn = getattr(self.mpc, 'rollout_fn', None)
            if rollout_fn is not None:
                rollout_targets.append(('mpc_aux', rollout_fn))

            solver = getattr(self.mpc, 'solver', None)
            if solver is not None:
                safety_rollout = getattr(solver, 'safety_rollout', None)
                if safety_rollout is not None:
                    rollout_targets.append(('mpc_safety', safety_rollout))

                for index, optimizer in enumerate(list(getattr(solver, 'optimizers', []) or [])):
                    optimizer_rollout = getattr(optimizer, 'rollout_fn', None)
                    if optimizer_rollout is not None:
                        rollout_targets.append((f'mpc_optimizer[{index}]', optimizer_rollout))

            for label, rollout in rollout_targets:
                dynamics_model = getattr(rollout, 'dynamics_model', None)
                robot_model = getattr(dynamics_model, 'robot_model', None)
                if robot_model is not None:
                    targets.append((label, robot_model))

        unique_targets = []
        seen = set()
        for label, target in targets:
            object_id = id(target)
            if object_id in seen:
                continue
            seen.add(object_id)
            unique_targets.append((label, target))
        return unique_targets

    def _refresh_solver_robot_models_from_shared_config(self):
        """Push the shared robot kinematics config into every active robot model."""
        shared_target = self._get_shared_robot_kinematics_target()
        shared_config = getattr(shared_target, 'kinematics_config', None)
        if shared_config is None:
            return False, [], 'Shared robot kinematics config is unavailable.'

        refreshed_labels = []
        failures = []
        for label, robot_model in self._iter_solver_robot_models():
            update_kinematics_config = getattr(robot_model, 'update_kinematics_config', None)
            if not callable(update_kinematics_config):
                failures.append(f'{label}: missing update_kinematics_config')
                continue
            try:
                update_kinematics_config(shared_config)
                refreshed_labels.append(label)
            except Exception as exc:
                failures.append(f'{label}: {exc}')

        if failures:
            return False, refreshed_labels, '; '.join(failures)
        return True, refreshed_labels, 'ok'

    def _mark_mpc_robot_geometry_dirty(self, reason: str):
        """
        Force the cached MPC solver to be rebuilt on next use after robot-geometry changes.

        MPC reuses warm-start buffers and rollout state across phases. When attached-object
        spheres or link-collision masks change, rebuilding is safer than relying on all
        internal rollout state to pick up the new robot geometry in-place.
        """
        if self.mpc is None:
            self._mpc_robot_geometry_dirty = False
            return
        self._mpc_robot_geometry_dirty = True
        self.get_logger().info(
            f'Marked cached MPC solver for rebuild on next use ({reason}).'
        )

    def propagate_link_collision_state(self, link_names, enabled):
        """Mirror link-collision toggles into robot_cfg and refresh active solver models."""
        manager = self.config_wrapper_motion.robot_model_manager
        link_names = list(link_names or [])
        shared_target = self._get_shared_robot_kinematics_target()

        applied, unknown = manager.set_link_collision_on_target(shared_target, link_names, enabled)
        if unknown:
            return False, f'shared_robot_cfg: unknown links {unknown}'

        refresh_ok, refreshed_labels, refresh_message = self._refresh_solver_robot_models_from_shared_config()
        if not refresh_ok:
            return False, refresh_message

        if applied or refreshed_labels:
            state = 'enabled' if enabled else 'disabled'
            self.get_logger().info(
                f'Propagated link collision {state} to solver kinematics: '
                f'{["shared_robot_cfg", *refreshed_labels]}'
            )
            self._mark_mpc_robot_geometry_dirty(
                f'link collision {state} for {applied or link_names}'
            )
        return True, 'ok'

    def propagate_attached_object_state(self, link_name, sphere_data, attach=True):
        """Mirror attach/detach updates into robot_cfg and refresh active solver models."""
        manager = self.config_wrapper_motion.robot_model_manager
        shared_target = self._get_shared_robot_kinematics_target()

        if attach:
            success, message, _ = manager.attach_object_to_target(shared_target, link_name, sphere_data)
        else:
            success, message = manager.detach_object_from_target(shared_target, link_name)
        if not success:
            return False, f'shared_robot_cfg: {message}'

        refresh_ok, refreshed_labels, refresh_message = self._refresh_solver_robot_models_from_shared_config()
        if not refresh_ok:
            return False, refresh_message

        if refreshed_labels:
            action = 'attach' if attach else 'detach'
            self.get_logger().info(
                f'Propagated attached-object {action} to solver kinematics: '
                f'{["shared_robot_cfg", *refreshed_labels]}'
            )
            self._mark_mpc_robot_geometry_dirty(
                f'attached-object {action} on {link_name}'
            )
        return True, 'ok'

    def _get_world_state_version(self):
        with self._world_update_lock:
            return self._world_state_version

    def _record_world_dirty(self):
        with self._world_update_lock:
            self._world_state_version += 1
            self._pending_world_version = self._world_state_version
            return self._world_state_version

    def notify_startup_warmup_started(self):
        if not self._startup_ready_logged:
            self._startup_ready_deadline = None
            self.set_parameters([
                rclpy.parameter.Parameter('startup_ready', rclpy.Parameter.Type.BOOL, False)
            ])

    def notify_startup_world_update_applied(self):
        if not self._startup_ready_logged:
            self._startup_ready_deadline = time.monotonic() + self._startup_ready_debounce_sec

    def _maybe_log_startup_ready(self):
        if self._startup_ready_logged or self._startup_ready_deadline is None:
            return

        if time.monotonic() < self._startup_ready_deadline:
            return

        self._startup_ready_logged = True
        self._startup_ready_deadline = None
        self.set_parameters([
            rclpy.parameter.Parameter('startup_ready', rclpy.Parameter.Type.BOOL, True)
        ])
        self.get_logger().info(
            "✅ cuRobo fully ready: warmup complete and all collisions are loaded."
        )

    def _record_solver_world_sync(self, solver_name: str, target_version: int):
        with self._world_update_lock:
            if solver_name == 'motion_gen':
                self._motion_gen_world_version = max(self._motion_gen_world_version, target_version)
                if self._solvers_share_world_checker():
                    self._mpc_world_version = max(self._mpc_world_version, target_version)
            elif solver_name == 'mpc':
                self._mpc_world_version = max(self._mpc_world_version, target_version)
                if self._solvers_share_world_checker():
                    self._motion_gen_world_version = max(self._motion_gen_world_version, target_version)

            if self._pending_world_version <= self._mpc_world_version:
                self._pending_world_version = 0

    def is_mpc_execution_active(self):
        with self._world_update_lock:
            return self._mpc_execution_active

    def set_mpc_execution_active(self, is_active: bool):
        with self._world_update_lock:
            self._mpc_execution_active = bool(is_active)

    def _sync_motion_gen_world_to_version(self, world_cfg, target_version: int):
        if self.motion_gen is None:
            return False

        with self._world_update_lock:
            if self._motion_gen_world_version >= target_version:
                return False

        self.motion_gen.world_coll_checker.clear_cache()
        self.motion_gen.update_world(world_cfg)
        self._record_solver_world_sync('motion_gen', target_version)
        return True

    def _sync_mpc_world_to_version(self, world_cfg, target_version: int):
        if self.mpc is None:
            return False

        with self._world_update_lock:
            if self._mpc_world_version >= target_version:
                return False

        self.mpc.world_coll_checker.clear_cache()
        self.mpc.update_world(world_cfg)
        self._record_solver_world_sync('mpc', target_version)
        return True

    def request_world_update(self, world_cfg, log_summary=True):
        """
        Mark the authoritative world dirty and apply it immediately when safe.

        During active MPC execution, world updates are only staged here. The MPC
        thread consumes the newest version between solve steps to avoid racing
        against CUDA graph capture.
        """
        target_version = self._record_world_dirty()
        dynamic_updates_enabled = bool(self.get_parameter('mpc_dynamic_world_updates_enabled').value)
        should_queue = dynamic_updates_enabled and self.is_mpc_execution_active()

        if should_queue:
            return target_version, False

        self.update_all_solvers_world(world_cfg, target_version=target_version)
        return target_version, True

    def sync_motion_gen_world(self):
        """Synchronize MotionGen to the latest authoritative world state."""
        if self.motion_gen is None:
            return False

        with self._world_update_lock:
            target_version = self._world_state_version
            current_version = self._motion_gen_world_version

        if current_version >= target_version:
            return False

        world_cfg = self.config_wrapper_motion.obstacle_manager.get_world_cfg()
        return self._sync_motion_gen_world_to_version(world_cfg, target_version)

    def sync_mpc_world(self):
        """Synchronize MPC to the latest staged world update."""
        if self.mpc is None:
            return False

        with self._world_update_lock:
            target_version = max(self._pending_world_version, self._world_state_version)
            current_version = self._mpc_world_version

        if current_version >= target_version:
            return False

        world_cfg = self.config_wrapper_motion.obstacle_manager.get_world_cfg()
        return self._sync_mpc_world_to_version(world_cfg, target_version)

    def update_all_solvers_world(self, world_cfg, target_version=None):
        """
        Propagate a world configuration update to all initialized solvers.

        Solvers that share the same world_coll_checker instance (VRAM sharing) only
        need one update — the identity check avoids the redundant call.
        """
        if target_version is None:
            target_version = self._get_world_state_version()

        self._sync_motion_gen_world_to_version(world_cfg, target_version)
        self._sync_mpc_world_to_version(world_cfg, target_version)

    def _load_robot_joint_names(self):
        """Load planning joint order from the robot_config_file YAML."""
        robot_config_file = self.get_parameter('robot_config_file').get_parameter_value().string_value
        try:
            config_file = load_yaml(robot_config_file)
            return list(config_file['robot_cfg']['kinematics']['cspace']['joint_names'])
        except Exception as e:
            self.get_logger().warn(
                f'Could not load joint order from {robot_config_file}: {e}'
            )
            return []

    def _joint_state_callback(self, msg: JointStateMsg):
        self._latest_joint_state_msg = msg

    def _get_current_joint_pose(self, require_live_state=False):
        """Return the latest /joint_states ordered for the loaded cuRobo robot config."""
        live_state = self._get_live_joint_state_dict()
        if live_state is not None:
            return list(live_state['position'])

        if require_live_state:
            return None

        return self.robot_context.get_joint_pose()

    def _get_live_joint_state_dict(self):
        """Return ordered live joint state data from `/joint_states` when available."""
        if self._latest_joint_state_msg is None or not self._robot_joint_names:
            return None

        msg = self._latest_joint_state_msg
        positions_by_name = {
            name: float(position)
            for name, position in zip(msg.name, msg.position)
        }
        velocity_values = list(msg.velocity) if len(msg.velocity) == len(msg.name) else []
        velocities_by_name = {
            name: float(velocity)
            for name, velocity in zip(msg.name, velocity_values)
        }

        missing_names = [
            joint_name for joint_name in self._robot_joint_names
            if joint_name not in positions_by_name
        ]
        if missing_names:
            self.get_logger().warn(
                f'Latest /joint_states is missing required joints: {missing_names}.'
            )
            return None

        ordered_positions = [
            positions_by_name[joint_name]
            for joint_name in self._robot_joint_names
        ]
        ordered_velocities = [
            velocities_by_name.get(joint_name, 0.0)
            for joint_name in self._robot_joint_names
        ]

        return {
            'joint_names': list(self._robot_joint_names),
            'position': ordered_positions,
            'velocity': ordered_velocities,
            'acceleration': [0.0] * len(ordered_positions),
            'jerk': [0.0] * len(ordered_positions),
        }

    def get_current_joint_pose(self):
        """Public state accessor for planners that need real robot feedback."""
        return self._get_current_joint_pose()

    def get_live_joint_pose(self):
        """Return ordered /joint_states only, or None if no live robot state is available."""
        return self._get_current_joint_pose(require_live_state=True)

    def get_current_joint_state(self):
        """Return the latest ordered joint state, falling back to robot_context if needed."""
        live_state = self._get_live_joint_state_dict()
        if live_state is not None:
            return live_state

        fallback_positions = list(self.robot_context.get_joint_pose())
        fallback_joint_names = list(self._robot_joint_names) or list(self.robot_context.get_joint_name())

        return {
            'joint_names': fallback_joint_names,
            'position': fallback_positions,
            'velocity': [0.0] * len(fallback_positions),
            'acceleration': [0.0] * len(fallback_positions),
            'jerk': [0.0] * len(fallback_positions),
        }

    def _joint_state_dict_to_curobo_state(self, joint_state_dict):
        """Convert an ordered joint-state dict into a cuRobo JointState."""
        joint_names = list(joint_state_dict.get('joint_names', [])) or list(self._robot_joint_names) or None
        positions = list(joint_state_dict.get('position', []))
        velocities = list(joint_state_dict.get('velocity', [0.0] * len(positions)))
        accelerations = list(joint_state_dict.get('acceleration', [0.0] * len(positions)))
        jerks = list(joint_state_dict.get('jerk', [0.0] * len(positions)))

        return JointState(
            position=torch.tensor(
                [positions],
                device=self.tensor_args.device,
                dtype=self.tensor_args.dtype,
            ),
            velocity=torch.tensor(
                [velocities],
                device=self.tensor_args.device,
                dtype=self.tensor_args.dtype,
            ),
            acceleration=torch.tensor(
                [accelerations],
                device=self.tensor_args.device,
                dtype=self.tensor_args.dtype,
            ),
            jerk=torch.tensor(
                [jerks],
                device=self.tensor_args.device,
                dtype=self.tensor_args.dtype,
            ),
            joint_names=joint_names,
        )

    def _joint_state_msg_to_ordered_dict(self, joint_state_msg: JointStateMsg):
        """Convert a ROS JointState into the planner's joint order when possible."""
        if joint_state_msg is None or not joint_state_msg.position:
            return None

        joint_names = list(joint_state_msg.name or [])
        if not joint_names or len(joint_names) != len(joint_state_msg.position):
            return None

        positions_by_name = {
            name: float(position)
            for name, position in zip(joint_names, joint_state_msg.position)
        }
        velocity_values = (
            list(joint_state_msg.velocity)
            if len(joint_state_msg.velocity) == len(joint_names)
            else []
        )
        velocities_by_name = {
            name: float(velocity)
            for name, velocity in zip(joint_names, velocity_values)
        }

        target_joint_names = list(self._robot_joint_names) or joint_names
        missing = [joint_name for joint_name in target_joint_names if joint_name not in positions_by_name]
        if missing:
            return {
                'joint_names': joint_names,
                'position': [float(value) for value in joint_state_msg.position],
                'velocity': [float(value) for value in velocity_values] if velocity_values else [0.0] * len(joint_names),
                'acceleration': [0.0] * len(joint_names),
                'jerk': [0.0] * len(joint_names),
            }

        ordered_positions = [
            positions_by_name[joint_name]
            for joint_name in target_joint_names
        ]
        ordered_velocities = [
            velocities_by_name.get(joint_name, 0.0)
            for joint_name in target_joint_names
        ]
        return {
            'joint_names': target_joint_names,
            'position': ordered_positions,
            'velocity': ordered_velocities,
            'acceleration': [0.0] * len(ordered_positions),
            'jerk': [0.0] * len(ordered_positions),
        }

    @staticmethod
    def _ordered_joint_state_msg(joint_names, positions, velocities=None):
        """Build a JointState message from ordered vectors."""
        joint_state_msg = JointStateMsg()
        joint_state_msg.name = list(joint_names or [])
        joint_state_msg.position = [float(value) for value in positions]
        if velocities is not None:
            joint_state_msg.velocity = [float(value) for value in velocities]
        return joint_state_msg

    @staticmethod
    def _ordered_rows_are_finite(rows):
        """Return True when every numeric value in the nested row structure is finite."""
        if rows is None:
            return False
        for row in rows:
            for value in row:
                if not math.isfinite(float(value)):
                    return False
        return True

    def _extract_first_valid_ordered_command(
        self,
        *joint_state_candidates,
        max_points=1,
        current_positions=None,
        target_positions=None,
    ):
        """Extract the best finite controller-space command from the provided cuRobo candidates."""
        ordered_joint_names = self.get_controller_joint_names()
        candidate_labels = ("js_action", "action")
        best_candidate = None
        best_target_error = None

        for index, joint_state in enumerate(joint_state_candidates):
            if joint_state is None:
                continue

            candidate_label = (
                candidate_labels[index]
                if index < len(candidate_labels)
                else f"candidate_{index}"
            )
            try:
                positions, velocities, accelerations = extract_ordered_waypoints(
                    joint_state,
                    ordered_joint_names,
                    max_points=max_points,
                )
            except Exception as exc:
                self.get_logger().warn(
                    f"Failed to extract MPC command from {candidate_label}: {exc}"
                )
                continue

            if not positions:
                continue

            if not self._ordered_rows_are_finite(positions):
                self.get_logger().warn(
                    f"MPC {candidate_label} positions contain non-finite values; ignoring candidate."
                )
                continue
            if velocities and not self._ordered_rows_are_finite(velocities):
                self.get_logger().warn(
                    f"MPC {candidate_label} velocities contain non-finite values; zeroing velocities."
                )
                velocities = [[0.0] * len(ordered_joint_names) for _ in positions]
            if accelerations and not self._ordered_rows_are_finite(accelerations):
                self.get_logger().warn(
                    f"MPC {candidate_label} accelerations contain non-finite values; zeroing accelerations."
                )
                accelerations = [[0.0] * len(ordered_joint_names) for _ in positions]

            if current_positions is None or not target_positions:
                return positions, velocities, accelerations

            target_error = self._max_joint_error(positions[0], target_positions)
            if target_error is None:
                continue

            if best_target_error is None or target_error < best_target_error:
                best_candidate = (positions, velocities, accelerations)
                best_target_error = target_error

        if best_candidate is not None:
            return best_candidate

        return None, None, None

    def _reorder_positions(self, joint_names, positions, target_joint_names, context):
        """Reorder a joint vector into the requested target joint-name order."""
        target_positions = [float(value) for value in (positions or [])]
        if not target_positions:
            return []

        source_joint_names = list(joint_names or [])
        ordered_target_joint_names = list(target_joint_names or source_joint_names)

        if not source_joint_names or len(source_joint_names) != len(target_positions):
            return target_positions

        positions_by_name = {
            name: float(position)
            for name, position in zip(source_joint_names, target_positions)
        }
        missing = [
            joint_name for joint_name in ordered_target_joint_names
            if joint_name not in positions_by_name
        ]
        if missing:
            self.get_logger().warn(
                f'{context} is missing joints required by the requested order: {missing}. '
                'Using the provided order as-is.'
            )
            return target_positions

        return [
            positions_by_name[joint_name]
            for joint_name in ordered_target_joint_names
        ]

    def _reorder_joint_target_positions(self, joint_names, positions):
        """Reorder a joint target vector into the planner's configured joint order."""
        return self._reorder_positions(
            joint_names,
            positions,
            list(self._robot_joint_names) or list(joint_names or []),
            'Hybrid MPC joint target',
        )

    @staticmethod
    def _max_joint_error(current_positions, target_positions):
        """Return the maximum absolute joint delta between two same-length vectors."""
        if not current_positions or not target_positions:
            return None
        if len(current_positions) != len(target_positions):
            return None
        return max(
            abs(float(current) - float(target))
            for current, target in zip(current_positions, target_positions)
        )


    def _build_mpc_step_request(
        self,
        target_pose_msg,
        trajectory_constraints,
        target_joint_positions=None,
        target_joint_names=None,
    ):
        """Create a TrajectoryGeneration request from the hybrid local target."""
        request = TrajectoryGeneration.Request()
        request.target_pose = target_pose_msg
        if target_joint_positions:
            request.target_joint_positions = self._reorder_joint_target_positions(
                target_joint_names,
                target_joint_positions,
            )
        if trajectory_constraints:
            request.trajectory_constraints = list(trajectory_constraints)
        return request

    def _clear_hybrid_mpc_state(self):
        """Reset cached state for the hybrid-local MPC stepping interface."""
        planner = getattr(self.planner_manager, '_planners', {}).get('mpc')
        if isinstance(planner, MPCPlanner):
            planner.cancel()
            planner.goal_buffer = None
            planner.start_state = None
            planner.goal_pose = None
            planner.goal_joint_positions = None
            planner.goal_state = None
            planner.is_goal_active = False
            planner.latest_goal_from_topic = None
            planner.last_goal_topic_raw = None
            planner.trajectory_constraints = []
            planner.initial_pose_error = None
            planner.best_pose_error = None
            planner.best_position_error = None
            planner.best_rotation_error = None
            planner._classic_handoff_requested = False
            planner._classic_handoff_metadata = None

        self._hybrid_mpc_session_active = False
        self._hybrid_mpc_infeasible_streak = 0
        self._hybrid_mpc_last_step_time = None
        self._hybrid_mpc_stall_count = 0
        self._hybrid_mpc_best_error = None
        self.set_mpc_execution_active(False)

    def _reset_hybrid_mpc_session(self, restore_trajectory_controller=True):
        """Stop the hybrid-local MPC stepping session and optionally restore controllers."""
        self._clear_hybrid_mpc_state()

        if restore_trajectory_controller:
            if not self.activate_trajectory_execution_controller():
                return False, (
                    f'Failed to activate {self.mpc_deactivate_controller_name} after hybrid MPC reset.'
                )

        return True, 'Hybrid MPC state reset'

    def get_live_joint_state(self):
        """Return ordered live joint position/velocity data or None."""
        return self._get_live_joint_state_dict()

    def get_controller_joint_names(self):
        """Controller joint order used for Isaac/ros2_control execution."""
        return list(self.controller_joint_names or self._robot_joint_names)

    def get_mpc_execution_interface(self):
        """Return the configured MPC execution transport."""
        return str(self.get_parameter('mpc_execution_interface').value).strip().lower()

    def use_mpc_position_streaming(self):
        """Whether MPC should stream one joint-position command per tick."""
        return self.get_mpc_execution_interface() == 'stream_position'

    def should_log_mpc_debug(self, step_index=None):
        """Return True when verbose MPC debug output is enabled for the current step."""
        if not bool(self.get_parameter('mpc_debug_logging').value):
            return False
        if step_index is None:
            return True
        log_every_n_steps = max(int(self.get_parameter('mpc_debug_log_every_n_steps').value), 1)
        return int(step_index) % log_every_n_steps == 0

    def emit_mpc_debug(self, message, step_index=None):
        """Emit MPC debug information to logs and an optional ROS topic."""
        if self.should_log_mpc_debug(step_index):
            self.get_logger().info(f"[MPC_DEBUG] {message}")
        if bool(self.get_parameter('mpc_debug_publish_topic').value):
            self.mpc_debug_publisher.publish(String(data=str(message)))

    def get_state_collision_debug(self, joint_positions):
        """
        Return a compact world-collision summary for a joint configuration.

        The summary is based on robot collision-sphere distances against the
        active world collision checker. Negative values indicate penetration.
        """
        checker = None
        robot_model = None
        if self.mpc is not None:
            checker = self.mpc.world_coll_checker
            robot_model = getattr(self.mpc, 'kinematics', None)
        if self.motion_gen is not None:
            checker = self.motion_gen.world_coll_checker if checker is None else checker
            robot_model = getattr(self.motion_gen, 'kinematics', None) if robot_model is None else robot_model

        if robot_model is None:
            robot_model = self.config_wrapper_motion.kin_model

        if checker is None or joint_positions is None:
            return None

        try:
            joint_tensor = torch.tensor(
                joint_positions,
                dtype=self.config_wrapper_motion._ops_dtype,
                device=self.config_wrapper_motion._device,
            )
            kinematics_state = robot_model.get_state(joint_tensor)
            robot_spheres = kinematics_state.link_spheres_tensor.view(1, 1, -1, 4)
            query_buffer = CollisionQueryBuffer.initialize_from_shape(
                robot_spheres.shape,
                self.tensor_args,
                checker.collision_types,
            )
            activation_distance = self.tensor_args.to_device([
                float(self.get_parameter('collision_activation_distance').value)
            ])
            weight = self.tensor_args.to_device([1.0])
            env_query_idx = torch.zeros(
                (robot_spheres.shape[0],),
                device=self.tensor_args.device,
                dtype=torch.int32,
            )
            sphere_dist = checker.get_sphere_distance(
                robot_spheres,
                query_buffer,
                weight,
                activation_distance,
                env_query_idx,
                compute_esdf=True,
            )
            sphere_dist = torch.flatten(sphere_dist, start_dim=0)
            if sphere_dist.numel() == 0:
                return None

            min_dist = float(torch.min(sphere_dist).item())
            max_dist = float(torch.max(sphere_dist).item())
            return {
                'min_sphere_dist': min_dist,
                'max_sphere_dist': max_dist,
                'world_collision_free': min_dist >= 0.0,
            }
        except Exception as exc:
            self.get_logger().debug(f'Collision debug unavailable: {exc}')
            return None

    @staticmethod
    def _format_collision_debug(label, collision_debug):
        """Format world-collision debug info for logs and operator feedback."""
        if collision_debug is None:
            return f"{label}(collision_debug=unavailable)"

        return (
            f"{label}(collision_free={collision_debug['world_collision_free']}, "
            f"min_sphere_dist={collision_debug['min_sphere_dist']:.5f}, "
            f"max_sphere_dist={collision_debug['max_sphere_dist']:.5f})"
        )

    def _validate_request_joint_states(self, start_joint_pose, request):
        """Reject plans whose provided joint-state endpoints already collide with the world."""
        if not bool(self.get_parameter('reject_colliding_request_joint_states').value):
            return None

        failing_states = []
        debug_parts = []

        start_debug = self.get_state_collision_debug(start_joint_pose)
        debug_parts.append(self._format_collision_debug('start', start_debug))
        if start_debug is not None and not start_debug['world_collision_free']:
            failing_states.append(
                f"start(min_sphere_dist={start_debug['min_sphere_dist']:.5f})"
            )

        goal_joint_positions = list(getattr(request, 'target_joint_positions', []) or [])
        if goal_joint_positions:
            goal_debug = self.get_state_collision_debug(goal_joint_positions)
            debug_parts.append(self._format_collision_debug('goal', goal_debug))
            if goal_debug is not None and not goal_debug['world_collision_free']:
                failing_states.append(
                    f"goal(min_sphere_dist={goal_debug['min_sphere_dist']:.5f})"
                )

        if not failing_states:
            return None

        return (
            "Rejected planning request because request joint state collides with the world: "
            f"{', '.join(failing_states)}. "
            f"Collision debug: {', '.join(debug_parts)}"
        )

    def activate_mpc_streaming_controller(self, wait_timeout_sec=2.0):
        """Switch ros2_control to the configured MPC streaming controller."""
        return self._switch_controllers(
            activate_controllers=[self.mpc_stream_controller_name],
            deactivate_controllers=[self.mpc_deactivate_controller_name],
            wait_timeout_sec=wait_timeout_sec,
        )

    def activate_trajectory_execution_controller(self, wait_timeout_sec=2.0):
        """Switch ros2_control back to the trajectory controller for classic execution."""
        return self._switch_controllers(
            activate_controllers=[self.mpc_deactivate_controller_name],
            deactivate_controllers=[self.mpc_stream_controller_name],
            wait_timeout_sec=wait_timeout_sec,
        )

    def _set_active_controller_goal_handle(self, controller_goal_handle):
        """Track the currently executing FollowJointTrajectory goal, if any."""
        with self._controller_goal_lock:
            self._active_controller_goal_handle = controller_goal_handle

    def cancel_active_controller_goal(self, wait_timeout_sec=2.0):
        """Cancel the active FollowJointTrajectory goal, if one is still running."""
        with self._controller_goal_lock:
            controller_goal_handle = self._active_controller_goal_handle

        if controller_goal_handle is None:
            return True

        try:
            cancel_future = controller_goal_handle.cancel_goal_async()
        except Exception as exc:
            self.get_logger().warn(f'Failed to cancel active controller goal: {exc}')
            return False

        response = wait_future_result(cancel_future, timeout_sec=float(wait_timeout_sec))
        if response is None:
            self.get_logger().warn('Controller goal cancel request returned no response.')
            return False
        return True

    def execute_planned_trajectory_via_controller(
        self,
        planned_trajectory,
        dt,
        goal_handle=None,
        planner_name='Trajectory',
    ):
        """Execute a planned cuRobo trajectory through FollowJointTrajectory."""
        if planned_trajectory is None:
            self.get_logger().error(f'{planner_name}: No trajectory available for controller execution.')
            return False

        if not self.controller_action_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().warn(
                f'{planner_name}: Controller action server unavailable at '
                f'{self.controller_action_name}; falling back to robot_context execution.'
            )
            return None

        ordered_joint_names = self.get_controller_joint_names()
        try:
            positions, velocities, accelerations = extract_ordered_waypoints(
                planned_trajectory,
                ordered_joint_names,
            )
            joint_trajectory = build_joint_trajectory(
                ordered_joint_names,
                positions,
                velocities,
                accelerations,
                dt,
            )
        except Exception as exc:
            self.get_logger().error(f'{planner_name}: Failed to build controller trajectory: {exc}')
            return False

        goal = FollowJointTrajectory.Goal()
        goal.trajectory = joint_trajectory
        send_future = self.controller_action_client.send_goal_async(goal)
        controller_goal_handle = wait_future_result(send_future, timeout_sec=5.0)
        if controller_goal_handle is None or not controller_goal_handle.accepted:
            self.get_logger().error(f'{planner_name}: Controller trajectory goal was rejected.')
            return False

        self._set_active_controller_goal_handle(controller_goal_handle)
        self.get_logger().info(f'{planner_name}: Trajectory execution started')
        result_future = controller_goal_handle.get_result_async()

        try:
            while not result_future.done():
                if goal_handle is not None and not goal_handle.is_active:
                    self.get_logger().warn(
                        f'{planner_name}: Execution goal cancelled while controller action was active.'
                    )
                    self.cancel_active_controller_goal()
                    return False
                time.sleep(0.05)

            result = wait_future_result(result_future, timeout_sec=0.1)
        finally:
            self._set_active_controller_goal_handle(None)

        if result is None:
            self.get_logger().error(f'{planner_name}: Controller action returned no result.')
            return False

        raw_error_code = result.result.error_code
        error_code = raw_error_code.val if hasattr(raw_error_code, 'val') else int(raw_error_code)
        if error_code != FollowJointTrajectory.Result.SUCCESSFUL:
            self.get_logger().error(
                f'{planner_name}: Controller trajectory execution failed with error code '
                f'{error_code}: {result.result.error_string}'
            )
            return False

        final_joint_state = self.get_live_joint_state() or self.get_current_joint_state()
        if final_joint_state is not None:
            self.get_logger().info(
                f'{planner_name}: Trajectory execution completed. '
                f'Final position: {final_joint_state["position"]}'
            )
        else:
            self.get_logger().info(f'{planner_name}: Trajectory execution completed.')
        return True

    def _get_controller_states(self, wait_timeout_sec=2.0):
        """Return a mapping of controller name -> lifecycle state."""
        if not self.controller_list_client.wait_for_service(timeout_sec=float(wait_timeout_sec)):
            self.get_logger().warn('Controller list service unavailable.')
            return None

        future = self.controller_list_client.call_async(ListControllers.Request())
        response = wait_future_result(future, timeout_sec=5.0)
        if response is None:
            self.get_logger().warn('Controller list request returned no response.')
            return None

        return {
            controller.name: controller.state
            for controller in response.controller
        }

    def _switch_controllers(
        self,
        activate_controllers,
        deactivate_controllers,
        wait_timeout_sec=2.0,
    ):
        """Switch ros2_control controllers, filtering out no-op requests when possible."""
        activate_controllers = list(activate_controllers or [])
        deactivate_controllers = list(deactivate_controllers or [])

        controller_states = self._get_controller_states(wait_timeout_sec=wait_timeout_sec)
        if controller_states is not None:
            activate_controllers = [
                name for name in activate_controllers
                if controller_states.get(name) != 'active'
            ]
            deactivate_controllers = [
                name for name in deactivate_controllers
                if controller_states.get(name) == 'active'
            ]

        if not activate_controllers and not deactivate_controllers:
            return True

        if not self.controller_switch_client.wait_for_service(timeout_sec=float(wait_timeout_sec)):
            self.get_logger().error('Controller switch service unavailable.')
            return False

        request = SwitchController.Request(
            deactivate_controllers=deactivate_controllers,
            activate_controllers=activate_controllers,
            strictness=SwitchController.Request.BEST_EFFORT,
        )
        future = self.controller_switch_client.call_async(request)
        response = wait_future_result(future, timeout_sec=5.0)
        if response is None:
            self.get_logger().error('Controller switch request returned no response.')
            return False
        if not response.ok:
            self.get_logger().error(
                'Failed to switch controllers. '
                f'activate={activate_controllers}, deactivate={deactivate_controllers}'
            )
            return False
        return True

    def publish_mpc_stream_position(self, joint_names, positions):
        """Publish one ordered joint-position command to the streaming controller."""
        ordered_joint_names = self.get_controller_joint_names()
        if list(joint_names) != ordered_joint_names:
            self.get_logger().error(
                'Streaming command joint order mismatch: '
                f'{list(joint_names)} != {ordered_joint_names}'
            )
            return False

        if len(positions) != len(ordered_joint_names):
            self.get_logger().error(
                f'Streaming command length mismatch: {len(positions)} != {len(ordered_joint_names)}'
            )
            return False

        if any(not math.isfinite(float(value)) for value in positions):
            self.get_logger().error('Streaming command contains non-finite values.')
            return False

        message = Float64MultiArray()
        message.data = [float(value) for value in positions]
        self.mpc_stream_publisher.publish(message)
        return True

    def _execute_mpc_classic_handoff(self, mpc_planner, handoff_metadata, goal_handle):
        """Finish a near-converged MPC move with one final classic MotionGen plan."""
        request = mpc_planner.build_classic_handoff_request()
        if request is None:
            self.get_logger().error(
                "MPC requested a classic finisher handoff but did not expose a valid goal request."
            )
            return False

        current_joint_state = self.get_live_joint_state() or self.get_current_joint_state()
        if current_joint_state is None:
            self.get_logger().error(
                "Classic finisher handoff requires a current joint state, but none is available."
            )
            return False

        if goal_handle is not None and not goal_handle.is_active:
            self.get_logger().warn("Skipping classic finisher handoff because the goal is no longer active.")
            return False

        classic_planner = self.planner_manager.get_planner('classic')
        self._setup_planner(classic_planner)

        start_state = self._joint_state_dict_to_curobo_state(current_joint_state)
        config = self._get_planner_config(classic_planner)

        self.get_logger().info(
            "Handing off near-goal MPC motion to Classic Motion Generation "
            f"(step={handoff_metadata['step_index']}, "
            f"position_error={handoff_metadata['position_error']:.4f}m, "
            f"rotation_error={handoff_metadata['rotation_error']:.4f}rad)"
        )

        result = classic_planner.plan(start_state, request, config, self.robot_context)
        if not result.success:
            self.get_logger().error(
                f"Classic finisher planning failed after MPC handoff: {result.message}"
            )
            return False

        if not self.activate_trajectory_execution_controller():
            self.get_logger().error(
                f"Failed to activate {self.mpc_deactivate_controller_name} for classic finisher execution."
            )
            return False

        return classic_planner.execute(self.robot_context, goal_handle)

    def mpc_step_callback(self, request: MpcStep.Request, response: MpcStep.Response):
        """
        Run exactly one MPC iteration for MoveIt Hybrid Planning.

        The hybrid-local adapter owns the outer control loop. This service only:
        1. Refreshes the current MPC goal from the sampled local target
        2. Runs one solver step from the supplied robot state
        3. Returns a single joint command plus status flags
        """
        with self._hybrid_mpc_lock:
            try:
                callback_started_at = time.monotonic()
                planner = self.planner_manager.get_planner('mpc')
                self._setup_planner(planner)
                config = self._get_planner_config(planner)

                current_joint_state = (
                    self.get_live_joint_state()
                    or self._joint_state_msg_to_ordered_dict(request.current_state)
                    or self.get_current_joint_state()
                )
                if current_joint_state is None:
                    response.success = False
                    response.message = 'No current joint state is available for MPC stepping.'
                    return response

                if not self._hybrid_mpc_session_active:
                    if not request.initialize_if_needed:
                        response.success = False
                        response.message = 'Hybrid MPC session is not active and initialization was not allowed.'
                        return response

                    if not self.activate_mpc_streaming_controller():
                        response.success = False
                        response.message = 'Failed to activate the MPC streaming controller.'
                        return response

                    self._hybrid_mpc_session_active = True
                    self._hybrid_mpc_infeasible_streak = 0
                    self._hybrid_mpc_stall_count = 0
                    self._hybrid_mpc_best_error = None
                    self.set_mpc_execution_active(True)

                start_state = self._joint_state_dict_to_curobo_state(current_joint_state)
                plan_request = self._build_mpc_step_request(
                    request.target_pose,
                    request.trajectory_constraints,
                    request.target_joint_positions,
                    request.current_state.name,
                )
                controller_current_positions = self._reorder_positions(
                    current_joint_state.get('joint_names', []),
                    current_joint_state.get('position', []),
                    self.get_controller_joint_names(),
                    'Hybrid MPC current joint state',
                )
                controller_target_positions = self._reorder_positions(
                    request.current_state.name,
                    request.target_joint_positions,
                    self.get_controller_joint_names(),
                    'Hybrid MPC joint target',
                )

                goal_changed = (
                    hasattr(planner, 'goal_request_differs')
                    and planner.goal_request_differs(plan_request)
                )

                plan_started_at = time.monotonic()
                planning_result = planner.refresh_hybrid_goal(start_state, plan_request, config)
                plan_finished_at = time.monotonic()
                if not planning_result.success:
                    self._clear_hybrid_mpc_state()
                    response.success = False
                    response.message = f'MPC goal setup failed: {planning_result.message}'
                    return response

                if goal_changed:
                    # The hybrid local planner advances through many sampled waypoints
                    # within one MPC session. Carrying progress metrics across those
                    # target changes makes "stall" detection compare different goals.
                    self._hybrid_mpc_infeasible_streak = 0
                    self._hybrid_mpc_stall_count = 0
                    self._hybrid_mpc_best_error = None

                sync_started_at = time.monotonic()
                self.sync_mpc_world()
                sync_finished_at = time.monotonic()
                mpc_step_dt = float(self.get_parameter('mpc_step_dt').value)
                mpc_horizon = max(int(self.get_parameter('mpc_horizon_steps').value), 2)
                now = time.monotonic()
                if bool(self.get_parameter('mpc_force_shift_steps_one').value):
                    shift_steps = 1
                elif self._hybrid_mpc_last_step_time is not None and mpc_step_dt > 0:
                    elapsed = now - self._hybrid_mpc_last_step_time
                    shift_steps = max(1, min(round(elapsed / mpc_step_dt), mpc_horizon - 1))
                else:
                    shift_steps = 1
                self._hybrid_mpc_last_step_time = now
                solve_started_at = time.monotonic()
                step_result = self.mpc.step(
                    start_state,
                    shift_steps=shift_steps,
                    max_attempts=max(int(self.get_parameter('mpc_step_max_attempts').value), 1),
                )
                solve_finished_at = time.monotonic()

                position_error = MPCPlanner._metric_scalar(
                    getattr(step_result.metrics, 'position_error', None)
                )
                rotation_error = MPCPlanner._metric_scalar(
                    getattr(step_result.metrics, 'rotation_error', None)
                )
                cspace_error = MPCPlanner._metric_scalar(
                    getattr(step_result.metrics, 'cspace_error', None)
                )
                metric_feasible = (
                    bool(torch.all(step_result.metrics.feasible).item())
                    if step_result.metrics.feasible is not None
                    else True
                )
                joint_goal_active = bool(getattr(planner, 'goal_joint_positions', None))

                immediate_positions, immediate_velocities, _ = self._extract_first_valid_ordered_command(
                    getattr(step_result, 'js_action', None),
                    getattr(step_result, 'action', None),
                    max_points=1,
                    current_positions=controller_current_positions,
                    target_positions=controller_target_positions,
                )
                command_valid = bool(immediate_positions)
                step_feasible = metric_feasible and command_valid

                if step_feasible:
                    self._hybrid_mpc_infeasible_streak = 0
                else:
                    self._hybrid_mpc_infeasible_streak += 1

                # --- Progress stall detection ---
                # Track whether the error toward the goal is actually improving.
                # If it stagnates for N steps the path is treated as invalidated
                # even when individual MPC steps remain feasible.
                current_error = cspace_error if joint_goal_active else position_error
                stall_threshold = float(self.get_parameter('hybrid_mpc_stall_threshold').value)
                if current_error is not None:
                    if (
                        self._hybrid_mpc_best_error is None
                        or current_error < self._hybrid_mpc_best_error - stall_threshold
                    ):
                        self._hybrid_mpc_best_error = current_error
                        self._hybrid_mpc_stall_count = 0
                    else:
                        self._hybrid_mpc_stall_count += 1

                if not immediate_positions:
                    if not controller_current_positions:
                        response.success = False
                        response.message = 'No current joint state is available to hold position safely.'
                        return response

                    self.get_logger().warn(
                        'Hybrid MPC step produced no finite solver command. '
                        'Holding current position until the solver recovers or the path is invalidated.'
                    )
                    immediate_positions = [controller_current_positions]
                    immediate_velocities = [[0.0] * len(controller_current_positions)]

                response.joint_command = self._ordered_joint_state_msg(
                    self.get_controller_joint_names(),
                    immediate_positions[0],
                    immediate_velocities[0] if immediate_velocities else None,
                )
                response.position_error = (
                    float(position_error)
                    if position_error is not None
                    else math.nan
                )
                response.rotation_error = (
                    float(rotation_error)
                    if rotation_error is not None
                    else math.nan
                )
                if joint_goal_active:
                    response.goal_reached = (
                        cspace_error is not None
                        and cspace_error < float(planner.joint_convergence_threshold)
                    )
                else:
                    response.goal_reached = (
                        position_error is not None
                        and rotation_error is not None
                        and position_error < float(planner.position_convergence_threshold)
                        and rotation_error < float(planner.rotation_convergence_threshold)
                    )
                infeasible_invalidated = (
                    self._hybrid_mpc_infeasible_streak
                    >= max(int(self.get_parameter('mpc_infeasible_abort_steps').value), 1)
                )
                stall_steps = max(int(self.get_parameter('hybrid_mpc_stall_steps').value), 1)
                stall_invalidated = self._hybrid_mpc_stall_count >= stall_steps
                path_invalidated = infeasible_invalidated or stall_invalidated
                response.path_invalidated = bool(
                    self.get_parameter('hybrid_mpc_report_path_invalidated').value
                ) and path_invalidated
                response.success = True

                if response.path_invalidated:
                    reason = (
                        f"progress stall ({self._hybrid_mpc_stall_count} steps, "
                        f"best_error={self._hybrid_mpc_best_error:.4f})"
                        if stall_invalidated
                        else f"infeasible streak ({self._hybrid_mpc_infeasible_streak} steps)"
                    )
                    self.get_logger().warn(
                        f"Hybrid MPC path invalidated: {reason}"
                    )
                    self.publish_mpc_stream_position(
                        self.get_controller_joint_names(),
                        controller_current_positions,
                    )
                    self._clear_hybrid_mpc_state()
                    response.message = 'Collision ahead'
                elif response.goal_reached:
                    response.message = (
                        'Local MPC joint goal reached'
                        if joint_goal_active
                        else 'Local MPC goal reached'
                    )
                elif not command_valid:
                    response.message = 'MPC step produced no valid command; holding current position'
                elif not metric_feasible:
                    response.message = 'MPC reported an infeasible step'
                else:
                    response.message = 'MPC step produced a valid streaming command'

                total_duration_ms = (time.monotonic() - callback_started_at) * 1000.0
                if total_duration_ms > 250.0:
                    self.get_logger().info(
                        "Hybrid MPC step timings: "
                        f"total={total_duration_ms:.1f}ms, "
                        f"goal_setup={(plan_finished_at - plan_started_at) * 1000.0:.1f}ms, "
                        f"world_sync={(sync_finished_at - sync_started_at) * 1000.0:.1f}ms, "
                        f"solve={(solve_finished_at - solve_started_at) * 1000.0:.1f}ms, "
                        f"shift_steps={shift_steps}, "
                        f"goal_reached={response.goal_reached}, "
                        f"path_invalidated={response.path_invalidated}, "
                        f"feasible={step_feasible}"
                    )

                return response

            except Exception as exc:
                self.get_logger().error(f'Hybrid MPC step failed: {exc}')
                import traceback
                self.get_logger().error(traceback.format_exc())
                response.success = False
                response.message = f'Hybrid MPC step failed: {exc}'
                return response

    def mpc_reset_callback(self, request: MpcReset.Request, response: MpcReset.Response):
        """Reset the hybrid-local MPC stepping session."""
        with self._hybrid_mpc_lock:
            success, message = self._reset_hybrid_mpc_session(
                restore_trajectory_controller=bool(request.restore_trajectory_controller)
            )
            response.success = bool(success)
            response.message = str(message)
            return response

    def generate_trajectory_callback(self, request: TrajectoryGeneration, response):
        """
        Generate a trajectory using the current planner.

        This service plans but doesn't execute. Use the action to execute.
        """
        try:
            # Get current planner
            planner = self.planner_manager.get_current_planner()

            if planner is None:
                response.success = False
                response.message = "No planner selected"
                return response

            # Get robot state - check if start pose is provided in request
            if hasattr(request, 'start_pose') and request.start_pose.position:
                # Use start_pose from request (works for both classic and multipoint)
                start_joint_pose = list(request.start_pose.position)
                self.get_logger().info(
                    f"📍 Using start position from request: {[f'{x:.3f}' for x in start_joint_pose]}"
                )
            else:
                # Fall back to current robot position from /joint_states if available
                start_joint_state = self.get_current_joint_state()
                start_joint_pose = start_joint_state['position']
                self.get_logger().info(
                    f"📍 Using robot current position for planning: {[f'{x:.3f}' for x in start_joint_pose]}"
                )

            joint_names = (
                list(start_joint_state.get('joint_names', []))
                if 'start_joint_state' in locals()
                else list(self._robot_joint_names)
            ) or None
            if 'start_joint_state' not in locals():
                start_joint_state = {
                    'position': start_joint_pose,
                    'velocity': [0.0] * len(start_joint_pose),
                    'acceleration': [0.0] * len(start_joint_pose),
                    'jerk': [0.0] * len(start_joint_pose),
                    'joint_names': joint_names or [],
                }

            start_state = self._joint_state_dict_to_curobo_state(start_joint_state)

            # Build config from parameters
            config = self._get_planner_config(planner)

            # Initialize planner if needed
            self._setup_planner(planner)

            validation_error = self._validate_request_joint_states(start_joint_pose, request)
            if validation_error is not None:
                self.get_logger().error(validation_error)
                response.success = False
                response.message = validation_error
                response.trajectory = []
                response.dt = 0.0
                return response

            # Plan
            # Each planner extracts its goal from the request (target_pose or target_poses)
            self.get_logger().info(
                f"Planning with {planner.get_planner_name()}"
            )

            result = planner.plan(start_state, request, config, self.robot_context)

            # Build response
            response.success = result.success
            response.message = result.message

            # Populate trajectory and dt fields
            if result.success and result.trajectory is not None:
                # Get trajectory from result
                traj = result.trajectory

                # Get interpolation dt
                if hasattr(planner, 'motion_gen') and planner.motion_gen is not None:
                    response.dt = float(planner.motion_gen.interpolation_dt)
                else:
                    response.dt = 0.03  # Default fallback

                # Convert cuRobo JointState to ROS2 sensor_msgs/JointState[]
                trajectory_msgs = []
                n_waypoints = len(traj.position)

                for i in range(n_waypoints):
                    waypoint = JointStateMsg()

                    # Set joint names if available
                    if hasattr(traj, 'joint_names') and traj.joint_names is not None:
                        waypoint.name = list(traj.joint_names)

                    # Convert tensors to lists
                    waypoint.position = traj.position[i].cpu().tolist()
                    waypoint.velocity = traj.velocity[i].cpu().tolist()

                    trajectory_msgs.append(waypoint)

                response.trajectory = trajectory_msgs

                self.get_logger().info(
                    f"Planning succeeded: {result.message} "
                    f"(trajectory: {n_waypoints} waypoints, dt: {response.dt}s)"
                )
            else:
                # Empty trajectory for failed planning
                response.trajectory = []
                response.dt = 0.0

                if result.success:
                    self.get_logger().info(
                        f"Planning succeeded: {result.message}"
                    )
                else:
                    self.get_logger().error(
                        f"Planning failed: {result.message}"
                    )

            return response

        except Exception as e:
            self.get_logger().error(f"Trajectory generation error: {e}")
            import traceback
            self.get_logger().error(traceback.format_exc())

            response.success = False
            response.message = f"Error: {str(e)}"
            response.trajectory = []
            response.dt = 0.0
            return response

    def execute_callback(self, goal_handle):
        """
        Execute the planned trajectory using the current planner.
        """
        try:
            planner = self.planner_manager.get_current_planner()

            if planner is None:
                result_msg = SendTrajectory.Result()
                result_msg.success = False
                result_msg.message = "No planner selected"
                goal_handle.abort()
                return result_msg

            self.get_logger().info(
                f"Executing with {planner.get_planner_name()}"
            )

            if not isinstance(planner, MPCPlanner):
                if not self.activate_trajectory_execution_controller():
                    result_msg = SendTrajectory.Result()
                    result_msg.success = False
                    result_msg.message = (
                        f"Failed to activate {self.mpc_deactivate_controller_name} "
                        "for trajectory execution"
                    )
                    goal_handle.abort()
                    return result_msg

            # Execute using the planner's strategy
            is_mpc = isinstance(planner, MPCPlanner)
            if is_mpc:
                self.set_mpc_execution_active(True)

            try:
                success = planner.execute(self.robot_context, goal_handle)
            finally:
                if is_mpc:
                    self.set_mpc_execution_active(False)

            handoff_metadata = None
            if is_mpc:
                handoff_metadata = planner.consume_classic_handoff_request()
                if not success and handoff_metadata is not None:
                    success = self._execute_mpc_classic_handoff(
                        planner,
                        handoff_metadata,
                        goal_handle,
                    )

            # Build result
            result_msg = SendTrajectory.Result()
            result_msg.success = success
            if success and handoff_metadata is not None:
                result_msg.message = "Execution completed via MPC -> Classic finisher handoff"
            elif not success and handoff_metadata is not None:
                result_msg.message = "Execution failed after MPC -> Classic finisher handoff"
            else:
                result_msg.message = "Execution completed" if success else "Execution failed"

            if success:
                goal_handle.succeed()
            else:
                goal_handle.abort()

            return result_msg

        except Exception as e:
            self.get_logger().error(f"Execution error: {e}")
            import traceback
            self.get_logger().error(traceback.format_exc())

            result_msg = SendTrajectory.Result()
            result_msg.success = False
            result_msg.message = f"Error: {str(e)}"
            goal_handle.abort()
            return result_msg

    def set_planner_callback(self, request: SetPlanner.Request, response: SetPlanner.Response):
        """
        Switch the active planner using enum-based service.

        Usage:
            ros2 service call /unified_planner/set_planner curobo_msgs/srv/SetPlanner "{planner_type: 1}"
        """
        try:
            previous = self.planner_manager.get_current_planner()
            previous_name = previous.get_planner_name() if previous else "None"

            key, error = PlannerFactory.switch_planner(request.planner_type, self.planner_manager)
            if error:
                response.success = False
                response.message = error
                response.previous_planner = previous_name
                response.current_planner = previous_name
                self.get_logger().error(error)
                return response

            planner = self.planner_manager.get_current_planner()
            self._setup_planner(planner)

            response.success = True
            response.message = f"Successfully switched to {planner.get_planner_name()}"
            response.previous_planner = previous_name
            response.current_planner = planner.get_planner_name()
            self.get_logger().info(f"✅ Planner switch: {previous_name} → {planner.get_planner_name()}")

        except Exception as e:
            response.success = False
            response.message = f"Failed to switch planner: {str(e)}"
            response.previous_planner = previous_name if 'previous_name' in locals() else "Unknown"
            response.current_planner = response.previous_planner
            self.get_logger().error(response.message)
            import traceback
            self.get_logger().error(traceback.format_exc())

        return response


    def mpc_goal_callback(self, msg):
        """
        Callback for MPC goal updates (real-time tracking).

        Receives goal pose from RViz plugin and updates MPC planner during execution.
        NOTE: do NOT create CUDA tensors here — this runs on the ROS2 executor thread
        and may race with CUDA graph capture in the MPC execution thread.
        Store raw Python data; the MPC thread will convert it to a Pose safely.
        """
        # Get current planner
        planner = self.planner_manager.get_current_planner()

        # Only update if MPC planner is active
        from curobo_ros.planners.mpc_planner import MPCPlanner
        if isinstance(planner, MPCPlanner):
            raw_goal = [
                msg.position.x, msg.position.y, msg.position.z,
                msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w
            ]
            previous_goal = getattr(planner, 'latest_goal_from_topic', None)
            if previous_goal is None:
                previous_goal = getattr(planner, 'last_goal_topic_raw', None)

            if previous_goal is not None:
                position_delta = max(
                    abs(float(new_value) - float(old_value))
                    for new_value, old_value in zip(raw_goal[:3], previous_goal[:3])
                )
                orientation_delta = max(
                    abs(float(new_value) - float(old_value))
                    for new_value, old_value in zip(raw_goal[3:], previous_goal[3:])
                )
                if (
                    position_delta < float(self.get_parameter('mpc_goal_update_position_epsilon').value)
                    and orientation_delta < float(self.get_parameter('mpc_goal_update_orientation_epsilon').value)
                ):
                    planner.ignored_goal_update_count = getattr(planner, 'ignored_goal_update_count', 0) + 1
                    if self.should_log_mpc_debug():
                        self.emit_mpc_debug(
                            f"ignored_goal_update pos_delta={position_delta:.6f} "
                            f"quat_delta={orientation_delta:.6f}"
                        )
                    return

            # Store raw list — converted to cuRobo Pose inside the MPC execution thread
            planner.latest_goal_from_topic = raw_goal
            planner.last_goal_topic_raw = list(raw_goal)
            planner.received_goal_update_count = getattr(planner, 'received_goal_update_count', 0) + 1
            self.get_logger().debug(
                f"MPC goal updated from topic: [{msg.position.x:.3f}, {msg.position.y:.3f}, {msg.position.z:.3f}]"
            )
        else:
            self.get_logger().warn(
                f"Received MPC goal but current planner is {planner.get_planner_name()}"
            )

    def get_planners_callback(self, request: GetPlanners.Request, response: GetPlanners.Response):
        """
        Return the list of available planners with their enum IDs.

        Usage:
            ros2 service call /unified_planner/get_planners curobo_msgs/srv/GetPlanners
        """
        current_type = self.planner_manager.get_current_planner_type()
        catalog = PlannerFactory.get_catalog()

        response.planner_names = [name for _, _, name in catalog]
        response.planner_ids   = [int(eid) for _, eid, _ in catalog]

        response.current_planner_name = 'Unknown'
        response.current_planner_id   = 255
        for key, eid, name in catalog:
            if key == current_type:
                response.current_planner_name = name
                response.current_planner_id   = int(eid)
                break

        response.success = True
        self.get_logger().info(
            f"GetPlanners: {len(catalog)} planners, current={response.current_planner_name}"
        )
        return response

    def _setup_planner(self, planner):
        """
        Initialize planner-specific components on-demand.

        This performs lazy warmup: if the planner's solver isn't initialized yet,
        it will be warmed up now. Subsequent switches to the same planner will
        be instant (retrieved from cache).
        """
        if isinstance(planner, SinglePlanner):
            # All MotionGen-based planners share the same warmed-up solver and
            # must see the latest world before planning, especially after a
            # hybrid MPC invalidation queues obstacle updates mid-execution.
            if self.motion_gen is None:
                self.get_logger().info(
                    f"On-demand warmup: {planner.get_planner_name()}"
                )
                self.node_is_available = False
                self._warmup_classic()
                self.node_is_available = True

            planner.set_motion_gen(self.motion_gen)
            self.sync_motion_gen_world()

        elif isinstance(planner, MPCPlanner):
            # Warmup MPC if not already done
            if self.mpc is None or self._mpc_robot_geometry_dirty:
                self.node_is_available = False
                if self._mpc_robot_geometry_dirty and self.mpc is not None:
                    self.get_logger().info(
                        "On-demand MPC rebuild: robot geometry changed since the last MPC phase."
                    )
                    self.mpc = None
                else:
                    self.get_logger().info("On-demand warmup: MPC planner")
                self._warmup_mpc()
                self._mpc_robot_geometry_dirty = False
                self.node_is_available = True

            planner.set_mpc_solver(self.mpc)
            self.sync_mpc_world()

            # MPC now uses shared world_cfg, no need to update config wrapper

    def _get_planner_config(self, planner) -> dict:
        """Build configuration dictionary for planner."""
        if isinstance(planner, (ClassicPlanner, JointSpacePlanner)):
            return {
                'max_attempts': self.get_parameter('max_attempts').value,
                'timeout': self.get_parameter('timeout').value,
                'time_dilation_factor': self.get_parameter('time_dilation_factor').value,
            }

        elif isinstance(planner, MPCPlanner):
            return {
                'convergence_threshold': self.get_parameter('convergence_threshold').value,
                'position_convergence_threshold': self.get_parameter(
                    'mpc_position_convergence_threshold'
                ).value,
                'rotation_convergence_threshold': self.get_parameter(
                    'mpc_rotation_convergence_threshold'
                ).value,
                'joint_convergence_threshold': self.get_parameter(
                    'mpc_joint_convergence_threshold'
                ).value,
                'command_speed_scale': self.get_parameter(
                    'mpc_command_speed_scale'
                ).value,
                'max_iterations': self.get_parameter('max_mpc_iterations').value,
            }

        return {}

    def goal_callback(self, goal):
        """Accept all goals."""
        self.get_logger().info("Received execution goal")
        return rclpy.action.GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        """Handle goal cancellation."""
        self.robot_context.stop_robot()

        # Cancel MPC if active
        planner = self.planner_manager.get_current_planner()
        if hasattr(planner, 'cancel'):
            planner.cancel()
        self.cancel_active_controller_goal()
        with self._hybrid_mpc_lock:
            self._clear_hybrid_mpc_state()

        self.get_logger().info("Goal cancelled")
        return rclpy.action.CancelResponse.ACCEPT


def main(args=None):
    rclpy.init(args=args)
    node = UnifiedPlannerNode()

    executor = MultiThreadedExecutor()
    executor.add_node(node)

    try:
        node.get_logger().info('Unified planner running, shut down with CTRL-C')
        executor.spin()
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard interrupt, shutting down.\n')
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
