#!/usr/bin/env python3
"""Remove a test obstacle from cuRobo and delete its RViz marker.

Edit the variable below, then run:
    ros2 run ur_robotiq_moveit_config remove_test_obstacle.py
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from std_srvs.srv import Trigger
from visualization_msgs.msg import Marker, MarkerArray
from curobo_msgs.srv import RemoveObject

# ── Edit this to configure removal ─────────────────────────────────
OBSTACLE_NAME = "test_wall"   # Set to None to remove ALL test obstacles
# ────────────────────────────────────────────────────────────────────

MARKER_TOPIC = "/test_obstacle_markers"
WORLD_FRAME = "world"


def remove_from_curobo_and_delete_marker(name):
    """Remove the obstacle from cuRobo's collision world and delete the RViz marker."""
    rclpy.init()
    node = Node("remove_test_obstacle")

    if name is not None:
        client = node.create_client(RemoveObject, "/unified_planner/remove_object")
        if not client.wait_for_service(timeout_sec=5.0):
            print("[cuRobo] Service /unified_planner/remove_object not available")
            node.destroy_node()
            rclpy.shutdown()
            return False

        request = RemoveObject.Request()
        request.name = name
        future = client.call_async(request)
        rclpy.spin_until_future_complete(node, future, timeout_sec=10.0)

        if future.result() is not None:
            resp = future.result()
            if resp.success:
                print(f"[cuRobo] Removed obstacle '{name}'")
            else:
                print(f"[cuRobo] Failed: {resp.message}")
        else:
            print("[cuRobo] Service call timed out")
    else:
        client = node.create_client(Trigger, "/unified_planner/remove_all_objects")
        if not client.wait_for_service(timeout_sec=5.0):
            print("[cuRobo] Service /unified_planner/remove_all_objects not available")
            node.destroy_node()
            rclpy.shutdown()
            return False

        request = Trigger.Request()
        future = client.call_async(request)
        rclpy.spin_until_future_complete(node, future, timeout_sec=10.0)

        if future.result() is not None:
            print("[cuRobo] Removed all obstacles")
        else:
            print("[cuRobo] Service call timed out")

    # -- Delete the RViz marker --
    qos = QoSProfile(
        depth=10,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
        reliability=ReliabilityPolicy.RELIABLE,
    )
    marker_pub = node.create_publisher(MarkerArray, MARKER_TOPIC, qos)

    if name is not None:
        marker = Marker()
        marker.header.frame_id = WORLD_FRAME
        marker.ns = "test_obstacles"
        marker.id = hash(name) % 2147483647
        marker.action = Marker.DELETE
        marker_array = MarkerArray(markers=[marker])
    else:
        marker = Marker()
        marker.header.frame_id = WORLD_FRAME
        marker.ns = "test_obstacles"
        marker.action = Marker.DELETEALL
        marker_array = MarkerArray(markers=[marker])

    for _ in range(10):
        marker.header.stamp = node.get_clock().now().to_msg()
        marker_pub.publish(marker_array)
        rclpy.spin_once(node, timeout_sec=0.1)

    print(f"[RViz] Deleted marker(s) on {MARKER_TOPIC}")

    node.destroy_node()
    rclpy.shutdown()
    return True


def main():
    if OBSTACLE_NAME is not None:
        print(f"Removing obstacle '{OBSTACLE_NAME}'")
    else:
        print("Removing ALL test obstacles")
    remove_from_curobo_and_delete_marker(OBSTACLE_NAME)


if __name__ == "__main__":
    main()
