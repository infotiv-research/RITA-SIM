#!/usr/bin/env python3

import sys
from typing import Dict, List, Optional

import rclpy
import torch
import yaml
from curobo.cuda_robot_model.cuda_robot_model import CudaRobotModel
from curobo.types.base import TensorDeviceType
from curobo.types.robot import RobotConfig
from rclpy.node import Node
from sensor_msgs.msg import JointState
from visualization_msgs.msg import Marker, MarkerArray


class CuroboLiveCollisionSpheres(Node):
    def __init__(self):
        super().__init__("curobo_live_collision_spheres")

        self.declare_parameter("robot_config_file", "")
        self.declare_parameter("joint_state_topic", "/joint_states")
        self.declare_parameter("base_link", "gantry_base_link")
        self.declare_parameter("marker_topic", "/curobo_live_collision_spheres")
        self.declare_parameter("marker_namespace", "curobo_live_collision_spheres")
        self.declare_parameter("marker_alpha", 0.5)
        self.declare_parameter("publish_rate_hz", 10.0)
        self.declare_parameter("joint_names", [""])

        robot_config_file = str(self.get_parameter("robot_config_file").value).strip()
        if not robot_config_file:
            raise RuntimeError("robot_config_file parameter is required")

        self.base_link = str(self.get_parameter("base_link").value)
        self.marker_namespace = str(self.get_parameter("marker_namespace").value)
        self.marker_alpha = float(self.get_parameter("marker_alpha").value)
        publish_rate_hz = max(0.1, float(self.get_parameter("publish_rate_hz").value))
        joint_names_param = [str(name) for name in self.get_parameter("joint_names").value]

        tensor_device = torch.device("cuda:0" if torch.cuda.is_available() else "cpu")
        self.tensor_args = TensorDeviceType(device=tensor_device, dtype=torch.float32)

        with open(robot_config_file, "r", encoding="utf-8") as handle:
            robot_config_dict = yaml.safe_load(handle)

        self.robot_cfg = RobotConfig.from_dict(robot_config_dict, tensor_args=self.tensor_args)
        self.kin_model = CudaRobotModel(self.robot_cfg.kinematics)
        self.joint_names = (
            [name for name in joint_names_param if name]
            if any(name for name in joint_names_param)
            else [
                str(name)
                for name in robot_config_dict["robot_cfg"]["kinematics"]["cspace"]["joint_names"]
            ]
        )

        self._joint_positions_by_name: Dict[str, float] = {}
        self._has_published = False
        self._warned_missing_joints = False

        self.marker_pub = self.create_publisher(
            MarkerArray,
            str(self.get_parameter("marker_topic").value),
            10,
        )
        self.joint_state_sub = self.create_subscription(
            JointState,
            str(self.get_parameter("joint_state_topic").value),
            self._on_joint_state,
            20,
        )
        self.create_timer(1.0 / publish_rate_hz, self._publish_markers)

        self.get_logger().info(
            "Live curobo collision spheres ready: "
            f"{len(self.joint_names)} joints, base_link={self.base_link}, device={self.tensor_args.device}"
        )

    def _on_joint_state(self, msg: JointState):
        for name, position in zip(msg.name, msg.position):
            self._joint_positions_by_name[str(name)] = float(position)

    def _ordered_joint_positions(self) -> Optional[List[float]]:
        missing = [name for name in self.joint_names if name not in self._joint_positions_by_name]
        if missing:
            if not self._warned_missing_joints:
                self.get_logger().warn(
                    "Waiting for live joint states for curobo spheres. "
                    f"Missing joints: {missing}"
                )
                self._warned_missing_joints = True
            return None

        if self._warned_missing_joints:
            self.get_logger().info("Live joint states received for all curobo sphere joints.")
            self._warned_missing_joints = False

        return [self._joint_positions_by_name[name] for name in self.joint_names]

    def _publish_delete_all(self):
        marker_array = MarkerArray()
        marker_array.markers.append(Marker(action=Marker.DELETEALL))
        self.marker_pub.publish(marker_array)

    def _publish_markers(self):
        ordered_positions = self._ordered_joint_positions()
        if ordered_positions is None:
            if self._has_published:
                self._publish_delete_all()
                self._has_published = False
            return

        joint_tensor = torch.tensor(
            [ordered_positions],
            dtype=self.tensor_args.dtype,
            device=self.tensor_args.device,
        )
        kinematics_state = self.kin_model.get_state(joint_tensor)
        robot_spheres = kinematics_state.link_spheres_tensor.view(-1, 4).detach().cpu().numpy()

        now = self.get_clock().now().to_msg()
        marker_array = MarkerArray()
        clear = Marker()
        clear.action = Marker.DELETEALL
        marker_array.markers.append(clear)

        marker_id = 0
        for sphere in robot_spheres:
            radius = float(sphere[3])
            if radius <= 0.0:
                continue

            marker = Marker()
            marker.header.frame_id = self.base_link
            marker.header.stamp = now
            marker.ns = self.marker_namespace
            marker.id = marker_id
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.pose.orientation.w = 1.0
            marker.pose.position.x = float(sphere[0])
            marker.pose.position.y = float(sphere[1])
            marker.pose.position.z = float(sphere[2])
            marker.scale.x = radius * 2.0
            marker.scale.y = radius * 2.0
            marker.scale.z = radius * 2.0
            marker.color.a = self.marker_alpha
            marker.color.r = 1.0
            marker.color.g = 0.0
            marker.color.b = 0.0
            marker_array.markers.append(marker)
            marker_id += 1

        self.marker_pub.publish(marker_array)
        self._has_published = True


def main(args=None):
    rclpy.init(args=args)
    node = CuroboLiveCollisionSpheres()
    try:
        rclpy.spin(node)
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
