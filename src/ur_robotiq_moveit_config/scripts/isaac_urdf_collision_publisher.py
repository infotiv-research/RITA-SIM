#!/usr/bin/env python3

import os
import xml.etree.ElementTree as ET
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from scipy.spatial.transform import Rotation

from geometry_msgs.msg import Point, Pose, Quaternion
from moveit_msgs.msg import CollisionObject, PlanningScene
from shape_msgs.msg import Mesh, MeshTriangle
from tf2_ros import Buffer, TransformListener

from std_msgs.msg import ColorRGBA
from moveit_msgs.msg import ObjectColor

def parse_obj(filepath, scale):
    """Parse OBJ file into a ROS Mesh."""
    verts, tris = [], []
    with open(filepath, "r", errors="ignore") as f:
        for line in f:
            p = line.split()
            if not p: continue
            
            # Get vertices and apply scale
            if p[0] == 'v':
                verts.append(Point(x=float(p[1])*scale[0], y=float(p[2])*scale[1], z=float(p[3])*scale[2]))
            
            # Get faces and triangulate
            elif p[0] == 'f':
                try:
                    idx = []
                    for val in p[1:]:
                        v_idx = int(val.split('/')[0])
                        v_idx = v_idx - 1 if v_idx > 0 else len(verts) + v_idx
                        idx.append(v_idx)
                        
                    for i in range(1, len(idx) - 1):
                        tris.append(MeshTriangle(vertex_indices=[idx[0], idx[i], idx[i+1]]))
                except ValueError:
                    pass
                    
    if not verts or not tris:
        return None
    return Mesh(vertices=verts, triangles=tris)


class IsaacMoveItPublisher(Node):
    def __init__(self):
        super().__init__("isaac_moveit_publisher")
        
        # Parameters
        assets = self.declare_parameter("assets_root", "assets/isaac_urdf_exports").value
        self.world_frame = self.declare_parameter("world_frame", "world").value
        self.assets_root = os.path.abspath(assets)
        
        # TF and Publisher
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.scene_pub = self.create_publisher(PlanningScene, "/planning_scene", 10)
        
        # State storage
        self.objects = {}       
        self.published = set()  
        
        # Init and start loop
        self.load_meshes()
        self.get_logger().info(f"Loaded {len(self.objects)} objects. Running at 30 Hz.")
        self.create_timer(1.0 / 30.0, self.publish_poses)

    def load_meshes(self):
        """Load URDFs and OBJs based on folder structure."""
        for obj_folder in os.listdir(self.assets_root):
            folder_path = os.path.join(self.assets_root, obj_folder)
            
            if not os.path.isdir(folder_path):
                continue
                
            # Folder name acts as the TF frame
            frame_name = obj_folder
            urdf_path = os.path.join(folder_path, f"{frame_name}.urdf")
            
            if not os.path.exists(urdf_path):
                continue
                
            try:
                root = ET.parse(urdf_path).getroot()
            except Exception:
                continue
            
            meshes_for_this_object = []
            
            # Parse collisions
            for col in root.findall(".//link/collision"):
                m_tag = col.find("geometry/mesh")
                if m_tag is None: continue
                
                # Find mesh file
                m_file = m_tag.get("filename", "").split("/")[-1]
                m_path = os.path.join(folder_path, "meshes", m_file)
                
                if not os.path.exists(m_path):
                    self.get_logger().warn(f"Missing mesh file: {m_path}")
                    continue
                
                # Get scale
                scale = [float(x) for x in m_tag.get("scale", "1 1 1").split()]
                if len(scale) < 3: scale = [1.0, 1.0, 1.0]
                
                # Get origin offset
                orig = col.find("origin")
                xyz = [float(x) for x in orig.get("xyz", "0 0 0").split()] if orig is not None else [0,0,0]
                rpy = [float(x) for x in orig.get("rpy", "0 0 0").split()] if orig is not None else [0,0,0]
                
                q = Rotation.from_euler("xyz", rpy).as_quat()
                pose = Pose(
                    position=Point(x=xyz[0], y=xyz[1], z=xyz[2]),
                    orientation=Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])
                )
                
                # Parse and store
                mesh = parse_obj(m_path, scale)
                if mesh:
                    meshes_for_this_object.append((mesh, pose))
                    
            if meshes_for_this_object:
                self.objects[frame_name] = meshes_for_this_object

    def publish_poses(self):
        """Publish object poses to MoveIt at 30Hz."""
        scene = PlanningScene(is_diff=True)
        
        for frame_name, mesh_data in self.objects.items():
            try:
                # Get TF transform
                tf = self.tf_buffer.lookup_transform(self.world_frame, frame_name, Time())
            except Exception:
                continue  
                
            is_new = frame_name not in self.published
            
            obj = CollisionObject()
            obj.id = frame_name
            obj.header.frame_id = self.world_frame
            
            # Use ADD for new objects, MOVE for existing ones
            obj.operation = CollisionObject.ADD if is_new else CollisionObject.MOVE
            
            obj.pose = Pose(
                position=Point(x=tf.transform.translation.x, y=tf.transform.translation.y, z=tf.transform.translation.z),
                orientation=tf.transform.rotation
            )
            
            # Attach mesh data only on initial ADD
            if is_new:
                obj.meshes = [m[0] for m in mesh_data]
                obj.mesh_poses = [m[1] for m in mesh_data]
                self.published.add(frame_name)
                self.get_logger().info(f"Adding [{frame_name}] to Planning Scene.")

                # --- Set custom color and alpha here ---
                obj_color = ObjectColor()
                obj_color.id = frame_name
                obj_color.color = ColorRGBA(r=0.0, g=1.0, b=1.0, a=1.0) 
                scene.object_colors.append(obj_color)
                
            scene.world.collision_objects.append(obj)
            
        # Publish scene
        if scene.world.collision_objects:
            self.scene_pub.publish(scene)


def main():
    rclpy.init()
    node = IsaacMoveItPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()