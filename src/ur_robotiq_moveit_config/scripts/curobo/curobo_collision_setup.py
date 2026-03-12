#!/usr/bin/env python3

import sys

import rclpy
from curobo_msgs.srv import SetCollisionCache
from rclpy.node import Node
from std_srvs.srv import Trigger


class CuroboCollisionSetup(Node):
    def __init__(self):
        super().__init__("curobo_collision_setup")
        self.declare_parameter("set_collision_cache_service", "/unified_planner/set_collision_cache")
        self.declare_parameter(
            "update_motion_gen_service", "/unified_planner/update_motion_gen_config"
        )
        self.declare_parameter("obb_cache", 100)
        self.declare_parameter("mesh_cache", 10)
        self.declare_parameter("blox_cache", 10)
        self.declare_parameter("wait_timeout_sec", 20.0)

        self._set_collision_cache_client = self.create_client(
            SetCollisionCache,
            str(self.get_parameter("set_collision_cache_service").value),
        )
        self._update_motion_gen_client = self.create_client(
            Trigger,
            str(self.get_parameter("update_motion_gen_service").value),
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
        obb_cache = int(self.get_parameter("obb_cache").value)
        mesh_cache = int(self.get_parameter("mesh_cache").value)
        blox_cache = int(self.get_parameter("blox_cache").value)

        if not self._set_collision_cache_client.wait_for_service(timeout_sec=wait_timeout_sec):
            self.get_logger().warn("SetCollisionCache service unavailable; skipping cache setup.")
            return 0

        if not self._update_motion_gen_client.wait_for_service(timeout_sec=wait_timeout_sec):
            self.get_logger().warn("update_motion_gen_config service unavailable; skipping cache setup.")
            return 0

        cache_request = SetCollisionCache.Request()
        cache_request.obb = obb_cache
        cache_request.mesh = mesh_cache
        cache_request.blox = blox_cache
        cache_response = self._wait_future_result(
            self._set_collision_cache_client.call_async(cache_request),
            timeout_sec=wait_timeout_sec,
        )
        if cache_response is None:
            self.get_logger().warn("SetCollisionCache call timed out; skipping cache setup.")
            return 0
        if not cache_response.success:
            self.get_logger().warn(
                f"SetCollisionCache failed: {cache_response.message}"
            )
            return 0

        self.get_logger().info(
            "Configured curobo collision cache: "
            f"obb={cache_response.obb_cache}, mesh={cache_response.mesh_cache}, "
            f"blox={cache_response.blox_cache}"
        )

        trigger_response = self._wait_future_result(
            self._update_motion_gen_client.call_async(Trigger.Request()),
            timeout_sec=wait_timeout_sec,
        )
        if trigger_response is None:
            self.get_logger().warn("update_motion_gen_config call timed out after cache update.")
            return 0
        if not trigger_response.success:
            self.get_logger().warn(
                f"update_motion_gen_config failed after cache update: {trigger_response.message}"
            )
            return 0

        self.get_logger().info("Applied curobo collision cache update to MotionGen.")
        return 0


def main(args=None):
    rclpy.init(args=args)
    node = CuroboCollisionSetup()
    try:
        return node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
