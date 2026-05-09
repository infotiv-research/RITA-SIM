"""Hybrid planner backend adapter for pick-and-place arm motion.

Routes MoveGroup goals through /move_action, where the
hybrid_move_action_bridge intercepts them and forwards to
/run_hybrid_planning (cuRobo MotionGen global + MPC local).

Object attachment uses CuroboObjectTracker (unified_planner/attach_object)
rather than the cuMotion UpdateLinkSpheres action, which is not served
by the unified planner node.
"""

from .backend_curobo import CuroboMotionBackend
from .backend_interface import MotionBackendInterface


class HybridMotionBackend(MotionBackendInterface):
    name = "hybrid"

    def __init__(self, node):
        super().__init__(node)
        self._precision_backend = CuroboMotionBackend(node)

    def wait_until_ready(self):
        self.node.get_logger().info(
            "Waiting for MoveGroup action server (/move_action) "
            "— hybrid bridge should be intercepting..."
        )
        if not self.node.move_group_client.wait_for_server(timeout_sec=30.0):
            self.node.get_logger().error(
                "MoveGroup action server not available after 30s! "
                "Is the hybrid planner (curobo_hybrid_planning.launch.py) running?"
            )
            return False

        self.node.get_logger().info(
            "Waiting for direct cuRobo services used by hybrid precision descents..."
        )
        if not self._precision_backend.wait_until_ready():
            self.node.get_logger().error(
                "Direct cuRobo MotionGen path is unavailable; hybrid precision descents "
                "cannot execute."
            )
            return False

        tracker = getattr(self.node, "_curobo_object_tracker", None)
        if tracker is not None:
            attach_client = getattr(tracker, "_attach_client", None)
            if attach_client is not None:
                self.node.get_logger().info(
                    "Waiting for cuRobo attach_object service "
                    "(/unified_planner/attach_object)..."
                )
                if not attach_client.wait_for_service(timeout_sec=30.0):
                    self.node.get_logger().error(
                        "cuRobo attach_object service not available after 30s! "
                        "Is the unified planner node running?"
                    )
                    return False

        return True

    @staticmethod
    def _is_precision_descent_mode(execution_mode):
        return str(execution_mode).strip().lower() == "precision_descent"

    def move_to_pose(
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
        planner_type=None,
        execution_mode="default",
    ):
        if self._is_precision_descent_mode(execution_mode):
            requested_planner = (
                str(planner_type).strip().lower()
                if planner_type is not None
                else str(
                    getattr(
                        self.node,
                        "hybrid_precision_descent_planner_type",
                        "classic",
                    )
                )
                .strip()
                .lower()
            ) or "classic"
            self.node.get_logger().info(
                "Hybrid precision descent: bypassing /move_action and using direct "
                f"cuRobo MotionGen planner='{requested_planner}'."
            )
            return self._precision_backend.move_to_pose(
                x,
                y,
                z,
                qx,
                qy,
                qz,
                qw,
                planning_time=planning_time,
                num_attempts=num_attempts,
                planner_type=requested_planner,
            )

        if str(execution_mode).strip().lower() not in {"", "default"}:
            self.node.get_logger().warn(
                f"Unknown hybrid execution_mode '{execution_mode}', using default hybrid path."
            )

        return self.node._move_to_pose(
            x,
            y,
            z,
            qx,
            qy,
            qz,
            qw,
            planning_time=planning_time,
            num_attempts=num_attempts,
        )

    def move_to_home(self):
        self.node.get_logger().info(
            "Hybrid return-home: using direct cuRobo joint-space home planner "
            "for periodic joint normalization."
        )
        return self._precision_backend.move_to_home()

    def move_to_release_pose_with_retries(self):
        return self.node._move_to_release_pose_with_retries()

    def restore_previous_planner(self):
        return self._precision_backend.restore_previous_planner()
