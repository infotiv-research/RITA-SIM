"""MoveIt backend adapter that preserves the current pick-and-place behavior."""

from .backend_interface import MotionBackendInterface


class MoveItMotionBackend(MotionBackendInterface):
    name = "moveit"

    def wait_until_ready(self):
        self.node.get_logger().info("Waiting for MoveGroup action server (/move_action)...")
        if not self.node.move_group_client.wait_for_server(timeout_sec=30.0):
            self.node.get_logger().error(
                "MoveGroup action server not available after 30s! "
                "Is ur_robotiq_isaac_moveit.launch.py running?"
            )
            return False

        if getattr(self.node, "_use_cumotion_object_attachment", False):
            self.node.get_logger().info(
                f"Waiting for cuMotion object-attachment action server "
                f"'{self.node.cumotion_attach_action_name}'..."
            )
            if not self.node._wait_for_cumotion_attachment_server(
                timeout_sec=float(self.node.cumotion_attach_timeout_s)
            ):
                self.node.get_logger().error(
                    f"cuMotion attachment action server '{self.node.cumotion_attach_action_name}' "
                    "is required but not available. Aborting."
                )
                return False
        return True

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
    ):
        del planner_type
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
        return self.node._move_to_home()

    def move_to_release_pose_with_retries(self):
        return self.node._move_to_release_pose_with_retries()
