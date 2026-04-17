#!/usr/bin/env python3

import numpy as np
from scipy.spatial.transform import Rotation
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point, Pose, Quaternion
from moveit_msgs.msg import CollisionObject, PlanningScene
from shape_msgs.msg import SolidPrimitive
from tf2_ros import Buffer, TransformListener

LIMBS = [
    ("upper_arm_r", "RightShoulder_Skele", "RightArm_Skele", 0.06),
    ("forearm_r", "RightArm_Skele", "RightForeArm_Skele", 0.05),
    ("hand_r", "RightForeArm_Skele", "RightHand_Skele", 0.05),
    ("upper_arm_l", "LeftShoulder_Skele", "LeftArm_Skele", 0.06),
    ("forearm_l", "LeftArm_Skele", "LeftForeArm_Skele", 0.05),
    ("hand_l", "LeftForeArm_Skele", "LeftHand_Skele", 0.05),
    ("thigh_r", "RightThigh_Skele", "RightShin_Skele", 0.09),
    ("shin_r", "RightShin_Skele", "RightFoot_Skele", 0.075),
    ("thigh_l", "LeftThigh_Skele", "LeftShin_Skele", 0.09),
    ("shin_l", "LeftShin_Skele", "LeftFoot_Skele", 0.075),
]

class HumanCollisionPublisher(Node):
    def __init__(self):
        super().__init__("human_collision_publisher")
        self.world_frame = self.declare_parameter("world_frame", "world").value
        rate = self.declare_parameter("publish_rate_hz", 20.0).value
        
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.scene_pub = self.create_publisher(PlanningScene, "/planning_scene", 10)
        
        self.active_ids = set()
        self.create_timer(1.0 / rate, self.update_scene)

    def get_pos(self, frame):
        try:
            t = self.tf_buffer.lookup_transform(self.world_frame, frame, rclpy.time.Time()).transform.translation
            return np.array([t.x, t.y, t.z])
        except Exception:
            return None

    def create_obj(self, obj_id, p_type, dims, pos, quat=(0,0,0,1)):
        obj = CollisionObject()
        obj.id = obj_id
        obj.header.frame_id = self.world_frame
        obj.operation = CollisionObject.ADD
        
        obj.primitives.append(SolidPrimitive(type=p_type, dimensions=[float(d) for d in dims]))
        obj.primitive_poses.append(Pose(
            position=Point(x=float(pos[0]), y=float(pos[1]), z=float(pos[2])),
            orientation=Quaternion(x=float(quat[0]), y=float(quat[1]), z=float(quat[2]), w=float(quat[3]))
        ))
        return obj

    def update_scene(self):
        scene = PlanningScene(is_diff=True)
        new_ids = set()

        # 1. Limbs (Cylinders)
        for name, start_f, end_f, radius in LIMBS:
            p1, p2 = self.get_pos(start_f), self.get_pos(end_f)
            if p1 is None or p2 is None: continue
            
            vec = p2 - p1
            dist = np.linalg.norm(vec)
            if dist < 0.01: continue

            z_axis = vec / dist
            rot = Rotation.align_vectors([z_axis], [[0, 0, 1]])[0]
            
            scene.world.collision_objects.append(
                self.create_obj(name, SolidPrimitive.CYLINDER, [dist, radius], (p1 + p2) / 2, rot.as_quat())
            )
            new_ids.add(name)

        # 2. Torso (Box)
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

            dims = [0.2, np.linalg.norm(shoulders), np.linalg.norm(spine)]
            scene.world.collision_objects.append(self.create_obj("torso", SolidPrimitive.BOX, dims, (hip_mid + sh_mid) / 2, q_torso))
            new_ids.add("torso")

        # 3. Head (Sphere)
        hp = self.get_pos("Head_Skele")
        if hp is not None:
            scene.world.collision_objects.append(self.create_obj("head", SolidPrimitive.SPHERE, [0.12], hp))
            new_ids.add("head")

        # 4. Remove missing items
        for old_id in self.active_ids - new_ids:
            rm = CollisionObject(id=old_id, operation=CollisionObject.REMOVE)
            rm.header.frame_id = self.world_frame
            scene.world.collision_objects.append(rm)

        self.active_ids = new_ids
        if scene.world.collision_objects:
            self.scene_pub.publish(scene)

def main():
    rclpy.init()
    node = HumanCollisionPublisher()
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