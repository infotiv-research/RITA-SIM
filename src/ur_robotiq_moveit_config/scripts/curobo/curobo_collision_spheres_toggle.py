#!/usr/bin/env python3

import sys

import rclpy
from rclpy.node import Node
from std_srvs.srv import SetBool


class CuroboCollisionSpheresToggle(Node):
    def __init__(self):
        super().__init__("curobo_collision_spheres_toggle")
        self.declare_parameter(
            "set_collision_spheres_service",
            "/unified_planner/set_collision_spheres_enabled",
        )
        self.declare_parameter("enabled", True)
        self.declare_parameter("wait_timeout_sec", 20.0)

        self._client = self.create_client(
            SetBool,
            str(self.get_parameter("set_collision_spheres_service").value),
        )

    def _wait_future_result(self, future, timeout_sec):
        rclpy.spin_until_future_complete(self, future, timeout_sec=float(timeout_sec))
        if not future.done():
            return None
        try:
            return future.result()
        except Exception:
            return None

    def run(self):
        wait_timeout_sec = float(self.get_parameter("wait_timeout_sec").value)
        enabled = bool(self.get_parameter("enabled").value)

        if not self._client.wait_for_service(timeout_sec=wait_timeout_sec):
            self.get_logger().warn(
                "set_collision_spheres_enabled service unavailable; "
                "skipping robot sphere visualization toggle."
            )
            return 0

        request = SetBool.Request()
        request.data = enabled
        response = self._wait_future_result(
            self._client.call_async(request),
            timeout_sec=wait_timeout_sec,
        )
        if response is None:
            self.get_logger().warn(
                "set_collision_spheres_enabled call timed out; "
                "robot sphere visualization state is unknown."
            )
            return 0

        if not response.success:
            self.get_logger().warn(
                "set_collision_spheres_enabled failed: "
                f"{response.message}"
            )
            return 0

        state = "enabled" if enabled else "disabled"
        self.get_logger().info(
            f"Robot collision sphere visualization {state}: {response.message}"
        )
        return 0


def main(args=None):
    rclpy.init(args=args)
    node = CuroboCollisionSpheresToggle()
    try:
        return node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
