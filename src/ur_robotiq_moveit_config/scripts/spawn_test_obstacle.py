#!/usr/bin/env python3
"""Spawn a test obstacle in cuRobo and publish an RViz marker.

Edit the variables below, then run:
    ros2 run ur_robotiq_moveit_config spawn_test_obstacle.py
"""

import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from geometry_msgs.msg import Pose, Point, Vector3
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray
from curobo_msgs.srv import AddObject

# ── Edit these to configure your test obstacle ─────────────────────
OBSTACLE_NAME = "test_wall"
POSITION      = [-1.2, 0.8, 0.9]       # x, y, z in meters
SIZE          = [0.5, 0.02, 0.4]       # x, y, z dimensions in meters
COLOR         = [1.0, 0.0, 0.0, 0.8]  # r, g, b, a
ROTATION_DEG  = [0.0, 0.0, 90.0]      # roll, pitch, yaw in degrees
# ────────────────────────────────────────────────────────────────────

# RViz marker settings — matches curobo_world_bridge topic so existing
# RViz config picks it up automatically.
MARKER_TOPIC = "/test_obstacle_markers"
WORLD_FRAME = "world"


def _quaternion_from_euler_deg(rotation_deg):
    """Convert roll, pitch, yaw in degrees to a quaternion."""
    roll = math.radians(rotation_deg[0])
    pitch = math.radians(rotation_deg[1])
    yaw = math.radians(rotation_deg[2])

    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)

    return (
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    )


def spawn_in_curobo_and_publish_marker(name, position, size, color, rotation_deg):
    """Add the obstacle to cuRobo's collision world and publish an RViz marker."""
    rclpy.init()
    node = Node("spawn_test_obstacle")
    quat_x, quat_y, quat_z, quat_w = _quaternion_from_euler_deg(rotation_deg)

    qos = QoSProfile(
        depth=1,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
        reliability=ReliabilityPolicy.RELIABLE,
    )
    marker_pub = node.create_publisher(MarkerArray, MARKER_TOPIC, qos)

    marker = Marker()
    marker.header.frame_id = WORLD_FRAME
    marker.ns = "test_obstacles"
    marker.id = hash(name) % 2147483647
    marker.type = Marker.CUBE
    marker.action = Marker.ADD
    marker.pose.position = Point(x=position[0], y=position[1], z=position[2])
    marker.pose.orientation.x = quat_x
    marker.pose.orientation.y = quat_y
    marker.pose.orientation.z = quat_z
    marker.pose.orientation.w = quat_w
    marker.scale = Vector3(x=size[0], y=size[1], z=size[2])
    marker.color = ColorRGBA(r=color[0], g=color[1], b=color[2], a=color[3])
    marker.text = name
    marker.frame_locked = False

    marker_array = MarkerArray(markers=[marker])

    def _publish_marker():
        marker.header.stamp = node.get_clock().now().to_msg()
        marker_pub.publish(marker_array)

    def _delete_marker():
        delete_marker = Marker()
        delete_marker.header.frame_id = WORLD_FRAME
        delete_marker.header.stamp = node.get_clock().now().to_msg()
        delete_marker.ns = marker.ns
        delete_marker.id = marker.id
        delete_marker.action = Marker.DELETE
        marker_pub.publish(MarkerArray(markers=[delete_marker]))

    # -- cuRobo service call --
    client = node.create_client(AddObject, "/unified_planner/add_object")

    if not client.wait_for_service(timeout_sec=5.0):
        print("[cuRobo] Service /unified_planner/add_object not available")
        node.destroy_node()
        rclpy.shutdown()
        return False

    # Publish immediately so RViz does not wait for the service round-trip
    # or the first timer tick before showing the obstacle.
    _publish_marker()
    print(f"[RViz] Published marker immediately on {MARKER_TOPIC}")

    request = AddObject.Request()
    request.type = AddObject.Request.CUBOID
    request.name = name
    request.pose = Pose()
    request.pose.position = Point(x=position[0], y=position[1], z=position[2])
    request.pose.orientation.x = quat_x
    request.pose.orientation.y = quat_y
    request.pose.orientation.z = quat_z
    request.pose.orientation.w = quat_w
    request.dimensions = Vector3(x=size[0], y=size[1], z=size[2])
    request.color = ColorRGBA(r=color[0], g=color[1], b=color[2], a=color[3])

    future = client.call_async(request)
    rclpy.spin_until_future_complete(node, future, timeout_sec=10.0)

    if future.result() is not None:
        resp = future.result()
        if resp.success:
            print(f"[cuRobo] Added obstacle '{name}' to collision world")
        else:
            print(f"[cuRobo] Failed: {resp.message}")
            _delete_marker()
            node.destroy_node()
            rclpy.shutdown()
            return False
    else:
        print("[cuRobo] Service call timed out")
        _delete_marker()
        node.destroy_node()
        rclpy.shutdown()
        return False

    # Keep the durable publisher alive so late-joining RViz subscribers
    # still receive the last marker sample.
    print(f"[RViz] Holding marker publisher on {MARKER_TOPIC} (Ctrl+C to stop)")
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()
    return True


def main():
    print(
        f"Spawning obstacle '{OBSTACLE_NAME}' at {POSITION} "
        f"with size {SIZE} and rotation {ROTATION_DEG} deg"
    )
    spawn_in_curobo_and_publish_marker(OBSTACLE_NAME, POSITION, SIZE, COLOR, ROTATION_DEG)


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\nStopped marker publishing. Obstacle remains in cuRobo collision world.")
