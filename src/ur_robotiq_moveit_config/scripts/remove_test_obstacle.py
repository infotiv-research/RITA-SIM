#!/usr/bin/env python3
"""Remove a test obstacle from cuRobo and delete its RViz marker.

Edit the variable below, then run:
    ros2 run ur_robotiq_moveit_config remove_test_obstacle.py
"""

import zlib

import rclpy
from moveit_msgs.msg import CollisionObject, PlanningScene
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


def _stable_marker_id(name):
    """Return a deterministic RViz marker id for a given obstacle name."""
    return zlib.crc32(name.encode("utf-8")) & 0x7FFFFFFF


def remove_from_curobo_and_delete_marker(name):
    """Remove the obstacle from planner worlds and delete the RViz marker."""
    rclpy.init()
    node = Node("remove_test_obstacle")

    if name is not None:
        client = node.create_client(RemoveObject, "/unified_planner/remove_object")
        if not client.wait_for_service(timeout_sec=5.0):
            print("[cuRobo] Service /unified_planner/remove_object not available; continuing")
        else:
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
            print("[cuRobo] Service /unified_planner/remove_all_objects not available; continuing")
        else:
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
    planning_scene_pub = node.create_publisher(PlanningScene, "/planning_scene", 10)

    if name is not None:
        marker = Marker()
        marker.header.frame_id = WORLD_FRAME
        marker.ns = "test_obstacles"
        marker.id = _stable_marker_id(name)
        marker.action = Marker.DELETE
        marker_array = MarkerArray(markers=[marker])

        collision_object = CollisionObject()
        collision_object.header.frame_id = WORLD_FRAME
        collision_object.id = str(name)
        collision_object.operation = CollisionObject.REMOVE
        planning_scene = PlanningScene()
        planning_scene.is_diff = True
        planning_scene.world.collision_objects.append(collision_object)
    else:
        marker = Marker()
        marker.header.frame_id = WORLD_FRAME
        marker.ns = "test_obstacles"
        marker.action = Marker.DELETEALL
        marker_array = MarkerArray(markers=[marker])
        planning_scene = None

    for _ in range(10):
        marker.header.stamp = node.get_clock().now().to_msg()
        marker_pub.publish(marker_array)
        if planning_scene is not None:
            planning_scene.world.collision_objects[0].header.stamp = node.get_clock().now().to_msg()
            planning_scene_pub.publish(planning_scene)
        rclpy.spin_once(node, timeout_sec=0.1)

    print(f"[RViz] Deleted marker(s) on {MARKER_TOPIC}")
    if name is not None:
        print(f"[MoveIt] Removed obstacle '{name}' from /planning_scene")

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
