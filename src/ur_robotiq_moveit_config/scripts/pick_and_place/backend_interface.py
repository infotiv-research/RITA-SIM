"""Backend interface for arm-motion execution in pick-and-place."""


class MotionBackendInterface:
    name = "abstract"

    def __init__(self, node):
        self.node = node

    def wait_until_ready(self):
        raise NotImplementedError

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
        raise NotImplementedError

    def move_to_home(self):
        raise NotImplementedError

    def move_to_release_pose_with_retries(self):
        raise NotImplementedError

    def restore_previous_planner(self):
        return True
