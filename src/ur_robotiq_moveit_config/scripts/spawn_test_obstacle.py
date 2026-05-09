#!/usr/bin/env python3
"""Spawn a predefined test obstacle in cuRobo and publish an RViz marker.

Edit ``OBSTACLE_PRESETS`` below, then run:
    ros2 run ur_robotiq_moveit_config spawn_test_obstacle.py -- --list
    ros2 run ur_robotiq_moveit_config spawn_test_obstacle.py -- --preset wall
"""

import argparse
import math
import zlib

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from geometry_msgs.msg import Pose, Point, Vector3
from moveit_msgs.msg import CollisionObject, PlanningScene
from shape_msgs.msg import SolidPrimitive
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray
from curobo_msgs.srv import AddObject

# ── Edit these to configure your obstacle presets ──────────────────
OBSTACLE_PRESETS = {
    "wall": {
        "description": "Thin wall near the robot workspace",
        "name": "test_wall",
        "position": [-1.2, 0.8, 0.9],       # x, y, z in meters
        "size": [0.5, 0.02, 0.4],           # x, y, z dimensions in meters
        "color": [1.0, 0.0, 0.0, 0.8],      # r, g, b, a
        "rotation_deg": [0.0, 0.0, 90.0],  # roll, pitch, yaw in degrees
    },
    "wall_p_p": {
        "description": "Thin wall for p&p tests",
        "name": "test_wall",
        "position": [-1.1, 1.1, 0.9],       # x, y, z in meters
        "size": [0.5, 0.02, 0.4],           # x, y, z dimensions in meters
        "color": [1.0, 0.0, 0.0, 0.8],      # r, g, b, a
        "rotation_deg": [0.0, 0.0, 45.0],  # roll, pitch, yaw in degrees
    },

}
DEFAULT_PRESET = "wall"
# ────────────────────────────────────────────────────────────────────

# RViz marker settings — matches curobo_world_bridge topic so existing
# RViz config picks it up automatically.
MARKER_TOPIC = "/test_obstacle_markers"
WORLD_FRAME = "world"
MARKER_REPUBLISH_PERIOD_SEC = 0.25
PLANNING_SCENE_REPUBLISH_PERIOD_SEC = 0.5


def _stable_marker_id(name):
    """Return a deterministic RViz marker id for a given obstacle name."""
    return zlib.crc32(name.encode("utf-8")) & 0x7FFFFFFF


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


def _build_arg_parser():
    parser = argparse.ArgumentParser(
        description="Spawn one predefined test obstacle in cuRobo and RViz."
    )
    parser.add_argument(
        "--preset",
        "-p",
        default=DEFAULT_PRESET,
        choices=sorted(OBSTACLE_PRESETS.keys()),
        help="Obstacle preset to spawn",
    )
    parser.add_argument(
        "--list",
        action="store_true",
        help="List available obstacle presets and exit",
    )
    return parser


def _print_available_presets():
    print("Available obstacle presets:")
    for preset_name in sorted(OBSTACLE_PRESETS.keys()):
        preset = OBSTACLE_PRESETS[preset_name]
        description = preset.get("description", "No description")
        print(
            f"  {preset_name}: {description} | "
            f"name={preset['name']} position={preset['position']} "
            f"size={preset['size']} rotation_deg={preset['rotation_deg']}"
        )


def spawn_in_curobo_and_publish_marker(
    name, position, size, color, rotation_deg, ros_args=None
):
    """Add the obstacle to available planner worlds and publish an RViz marker."""
    rclpy.init(args=ros_args)
    node = Node("spawn_test_obstacle")
    quat_x, quat_y, quat_z, quat_w = _quaternion_from_euler_deg(rotation_deg)

    qos = QoSProfile(
        depth=1,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
        reliability=ReliabilityPolicy.RELIABLE,
    )
    marker_pub = node.create_publisher(MarkerArray, MARKER_TOPIC, qos)
    planning_scene_pub = node.create_publisher(PlanningScene, "/planning_scene", 10)

    marker = Marker()
    marker.header.frame_id = WORLD_FRAME
    marker.ns = "test_obstacles"
    marker.id = _stable_marker_id(name)
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

    # Republish periodically so RViz still receives the marker if it
    # subscribes late, reconnects, or uses volatile QoS.
    node.create_timer(MARKER_REPUBLISH_PERIOD_SEC, _publish_marker)

    collision_object = CollisionObject()
    collision_object.header.frame_id = WORLD_FRAME
    collision_object.id = str(name)
    collision_object.operation = CollisionObject.ADD
    primitive = SolidPrimitive()
    primitive.type = SolidPrimitive.BOX
    primitive.dimensions = [float(size[0]), float(size[1]), float(size[2])]
    collision_object.primitives.append(primitive)
    primitive_pose = Pose()
    primitive_pose.position = Point(x=position[0], y=position[1], z=position[2])
    primitive_pose.orientation.x = quat_x
    primitive_pose.orientation.y = quat_y
    primitive_pose.orientation.z = quat_z
    primitive_pose.orientation.w = quat_w
    collision_object.primitive_poses.append(primitive_pose)

    planning_scene = PlanningScene()
    planning_scene.is_diff = True
    planning_scene.world.collision_objects.append(collision_object)

    def _publish_planning_scene():
        collision_object.header.stamp = node.get_clock().now().to_msg()
        planning_scene_pub.publish(planning_scene)

    node.create_timer(PLANNING_SCENE_REPUBLISH_PERIOD_SEC, _publish_planning_scene)

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

    # Publish immediately so RViz does not wait for the service round-trip
    # or the first timer tick before showing the obstacle.
    _publish_marker()
    _publish_planning_scene()
    print(f"[RViz] Published marker immediately on {MARKER_TOPIC}")
    print("[MoveIt] Published obstacle on /planning_scene")

    if client.wait_for_service(timeout_sec=5.0):
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
        else:
            print("[cuRobo] Service call timed out")
    else:
        print(
            "[cuRobo] Service /unified_planner/add_object not available; "
            "continuing with MoveIt/RViz only"
        )

    # Keep publishers alive and republish so late subscribers recover.
    print(
        f"[RViz/MoveIt] Republishing obstacle every "
        f"{MARKER_REPUBLISH_PERIOD_SEC:.1f}/{PLANNING_SCENE_REPUBLISH_PERIOD_SEC:.1f}s "
        "(Ctrl+C to stop)"
    )
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()
    return True


def main():
    parser = _build_arg_parser()
    args, ros_args = parser.parse_known_args()

    if args.list:
        _print_available_presets()
        return

    obstacle = OBSTACLE_PRESETS[args.preset]
    print(
        f"Spawning preset '{args.preset}' as obstacle '{obstacle['name']}' "
        f"at {obstacle['position']} with size {obstacle['size']} "
        f"and rotation {obstacle['rotation_deg']} deg"
    )
    spawn_in_curobo_and_publish_marker(
        obstacle["name"],
        obstacle["position"],
        obstacle["size"],
        obstacle["color"],
        obstacle["rotation_deg"],
        ros_args=ros_args,
    )


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\nStopped marker publishing. Obstacle remains in cuRobo collision world.")
