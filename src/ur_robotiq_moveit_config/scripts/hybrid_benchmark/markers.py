"""Publish and delete RViz markers for benchmark obstacles."""

from __future__ import annotations

from visualization_msgs.msg import Marker, MarkerArray

from .constants import WORLD_FRAME, quaternion_from_euler_deg, stable_marker_id


class ObstacleMarkerManager:
    # Keep the ROS node and marker publisher used by marker helpers.
    def __init__(self, node, publisher):
        self._node = node
        self._publisher = publisher

    # Publish a cube marker that mirrors the obstacle inserted into the planner.
    def show_obstacle_marker(self, obstacle: dict):
        marker = Marker()
        marker.header.frame_id = WORLD_FRAME
        marker.header.stamp = self._node.get_clock().now().to_msg()
        marker.ns = "test_obstacles"
        marker.id = stable_marker_id(str(obstacle["name"]))
        marker.type = Marker.CUBE
        marker.action = Marker.ADD
        marker.pose.position.x, marker.pose.position.y, marker.pose.position.z = (
            float(v) for v in obstacle["position"]
        )
        (
            marker.pose.orientation.x,
            marker.pose.orientation.y,
            marker.pose.orientation.z,
            marker.pose.orientation.w,
        ) = quaternion_from_euler_deg(obstacle["rotation_deg"])
        marker.scale.x, marker.scale.y, marker.scale.z = (float(v) for v in obstacle["size"])
        marker.color.r, marker.color.g, marker.color.b, marker.color.a = (
            float(v) for v in obstacle["color"]
        )
        marker.text = str(obstacle["name"])
        self._publisher.publish(MarkerArray(markers=[marker]))

    # Publish a delete marker for the named obstacle.
    def hide_obstacle_marker(self, name: str):
        delete_marker = Marker()
        delete_marker.header.frame_id = WORLD_FRAME
        delete_marker.header.stamp = self._node.get_clock().now().to_msg()
        delete_marker.ns = "test_obstacles"
        delete_marker.id = stable_marker_id(str(name))
        delete_marker.action = Marker.DELETE
        self._publisher.publish(MarkerArray(markers=[delete_marker]))
