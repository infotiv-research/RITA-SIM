#!/usr/bin/env python3

import numpy as np
from scipy.spatial.transform import Rotation
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point, Pose, Quaternion
from visualization_msgs.msg import Marker, MarkerArray
from tf2_ros import Buffer, TransformListener
from curobo_msgs.srv import AddObject, RemoveObject, UpdateObjectPoses

LIMBS = [
    ("human_upper_arm_r", "RightShoulder_Skele", "RightArm_Skele", 0.06),
    ("human_forearm_r", "RightArm_Skele", "RightForeArm_Skele", 0.05),
    ("human_hand_r", "RightForeArm_Skele", "RightHand_Skele", 0.05),
    ("human_upper_arm_l", "LeftShoulder_Skele", "LeftArm_Skele", 0.06),
    ("human_forearm_l", "LeftArm_Skele", "LeftForeArm_Skele", 0.05),
    ("human_hand_l", "LeftForeArm_Skele", "LeftHand_Skele", 0.05),
    ("human_thigh_r", "RightThigh_Skele", "RightShin_Skele", 0.09),
    ("human_shin_r", "RightShin_Skele", "RightFoot_Skele", 0.075),
    ("human_thigh_l", "LeftThigh_Skele", "LeftShin_Skele", 0.09),
    ("human_shin_l", "LeftShin_Skele", "LeftFoot_Skele", 0.075),
]

class CuroboHumanCollisionPublisher(Node):
    def __init__(self):
        super().__init__("curobo_human_collision_publisher")
        self.world_frame = self.declare_parameter("world_frame", "world").value
        rate = self.declare_parameter("publish_rate_hz", 20.0).value
        self.marker_topic = self.declare_parameter("marker_topic", "/curobo_human_collision_markers").value
        
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        self.marker_pub = self.create_publisher(MarkerArray, self.marker_topic, 10)
        self.add_client = self.create_client(AddObject, "/unified_planner/add_object")
        self.remove_client = self.create_client(RemoveObject, "/unified_planner/remove_object")
        self.move_client = self.create_client(UpdateObjectPoses, "/unified_planner/update_object_poses")
        
        self.active_shapes = {}
        self.create_timer(1.0 / rate, self.update_scene)

    def get_pos(self, frame):
        try:
            t = self.tf_buffer.lookup_transform(self.world_frame, frame, rclpy.time.Time()).transform.translation
            return np.array([t.x, t.y, t.z])
        except Exception:
            return None

    def create_pose(self, pos, quat=(0, 0, 0, 1)):
        return Pose(
            position=Point(x=float(pos[0]), y=float(pos[1]), z=float(pos[2])),
            orientation=Quaternion(x=float(quat[0]), y=float(quat[1]), z=float(quat[2]), w=float(quat[3]))
        )

    def publish_markers(self, objects):
        marker_array = MarkerArray()
        
        clear = Marker()
        clear.action = Marker.DELETEALL
        marker_array.markers.append(clear)

        now = self.get_clock().now().to_msg()
        for marker_id, (name, obj) in enumerate(objects.items()):
            m = Marker()
            m.header.frame_id = self.world_frame
            m.header.stamp = now
            m.ns = "curobo_human_collision"
            m.id = marker_id
            m.action = Marker.ADD
            m.pose = obj["pose"]
            
            m.color.a = 0.8  
            m.color.r = 1.0  
            m.color.g = 0.5  
            m.color.b = 0.0

            if obj["type"] == AddObject.Request.CUBOID:
                m.type = Marker.CUBE
                m.scale.x, m.scale.y, m.scale.z = [float(v) for v in obj["dims"]]
            elif obj["type"] == AddObject.Request.CYLINDER:
                m.type = Marker.CYLINDER
                m.scale.x = float(obj["dims"][0]) * 2.0  
                m.scale.y = float(obj["dims"][0]) * 2.0  
                m.scale.z = float(obj["dims"][1])        
            else: 
                m.type = Marker.SPHERE
                m.scale.x = m.scale.y = m.scale.z = float(obj["dims"][0]) * 2.0

            marker_array.markers.append(m)

        self.marker_pub.publish(marker_array)

    def update_scene(self):
        new_objects = {}
        
        # 1. Limbs (Cylinders)
        for name, start_f, end_f, radius in LIMBS:
            p1, p2 = self.get_pos(start_f), self.get_pos(end_f)
            if p1 is None or p2 is None: continue
            
            vec = p2 - p1
            dist = np.linalg.norm(vec)
            if dist < 0.01: continue

            z_axis = vec / dist
            rot = Rotation.align_vectors([z_axis], [[0, 0, 1]])[0]
            
            new_objects[name] = {
                "type": AddObject.Request.CYLINDER, 
                "dims": [radius, dist, dist], 
                "pose": self.create_pose((p1 + p2) / 2, rot.as_quat())
            }

        # 2. Torso (Cuboid)
        l_h, r_h = self.get_pos("LeftThigh_Skele"), self.get_pos("RightThigh_Skele")
        l_s, r_s = self.get_pos("LeftShoulder_Skele"), self.get_pos("RightShoulder_Skele")
        
        if all(p is not None for p in [l_h, r_h, l_s, r_s]):
            hip_mid, sh_mid = (l_h + r_h) / 2, (l_s + r_s) / 2
            spine = sh_mid - hip_mid
            shoulders = r_s - l_s
            
            z_idx = spine / np.linalg.norm(spine)
            x_idx = np.cross(shoulders / np.linalg.norm(shoulders), z_idx)
            y_idx = np.cross(z_idx, x_idx)
            
            rot_m = np.stack([x_idx, y_idx, z_idx], axis=1)
            q_torso = Rotation.from_matrix(rot_m).as_quat()
            
            new_objects["human_torso"] = {
                "type": AddObject.Request.CUBOID, 
                "dims": [0.2, np.linalg.norm(shoulders), np.linalg.norm(spine)], 
                "pose": self.create_pose((hip_mid + sh_mid) / 2, q_torso)
            }

        # 3. Head (Sphere)
        hp = self.get_pos("Head_Skele")
        if hp is not None:
            new_objects["human_head"] = {
                "type": AddObject.Request.SPHERE, 
                "dims": [0.12, 0.12, 0.12], 
                "pose": self.create_pose(hp)
            }

        # --- Publisher & Service Logic ---
        self.publish_markers(new_objects)

        if not (self.add_client.service_is_ready() and self.remove_client.service_is_ready() and self.move_client.service_is_ready()):
            return

        for old_id in list(self.active_shapes.keys()):
            if old_id not in new_objects:
                self.remove_client.call_async(RemoveObject.Request(name=old_id))
                del self.active_shapes[old_id]

        move_req = UpdateObjectPoses.Request()
        for name, obj in new_objects.items():
            shape_sig = (obj["type"], tuple(round(float(d), 3) for d in obj["dims"]))

            if name not in self.active_shapes or self.active_shapes[name] != shape_sig:
                if name in self.active_shapes:
                    self.remove_client.call_async(RemoveObject.Request(name=name))
                
                req = AddObject.Request(name=name, type=obj["type"], pose=obj["pose"])
                req.dimensions.x, req.dimensions.y, req.dimensions.z = float(obj["dims"][0]), float(obj["dims"][1]), float(obj["dims"][2])
                self.add_client.call_async(req)
                self.active_shapes[name] = shape_sig
            else:
                move_req.names.append(name)
                move_req.poses.append(obj["pose"])

        if move_req.names:
            self.move_client.call_async(move_req)

def main():
    rclpy.init()
    node = CuroboHumanCollisionPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == "__main__":
    main()