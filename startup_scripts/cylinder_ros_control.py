import asyncio, os, time
import carb, omni.kit.app, omni.usd, rclpy
from pxr import Gf, UsdGeom
from rclpy.node import Node
from std_msgs.msg import Bool, Int32

# === CONFIG & PRESETS ===
PRIM_PATH = "/World/Cylinder"
# Presets correspond to the number of points in the path
LOOP_PRESETS = {
    "2": [Gf.Vec3d(-1.3, 1.47, 0.82), Gf.Vec3d(-1.3, 1.47, 1.0)],
    "3": [Gf.Vec3d(-0.5, 1.47, 0.82), Gf.Vec3d(-2.1, 1.47, 0.82), Gf.Vec3d(-1.3, 1.47, 1.14)],
    "4": [Gf.Vec3d(-0.5, 1.47, 0.82), Gf.Vec3d(-2.1, 1.47, 0.82), Gf.Vec3d(-2.1, 1.47, 1.14), Gf.Vec3d(-0.5, 1.47, 1.14)],
}

class CylinderController(Node):
    def __init__(self, start_mode):
        super().__init__("cylinder_controller")
        self.active = False
        self.loop_key = start_mode
        self.loop_changed = False
        
        self.create_subscription(Bool, "/move_cylinder", self.toggle, 10)
        self.create_subscription(Int32, "/move_cylinder_loop", self.set_mode, 10)

    def toggle(self, msg): 
        self.active = msg.data
        carb.log_warn(f"Cylinder {'STARTED' if self.active else 'STOPPED'}")

    def set_mode(self, msg):
        m = str(msg.data)
        if m in LOOP_PRESETS and m != self.loop_key:
            self.loop_key = m
            self.loop_changed = True
            carb.log_info(f"Cylinder mode changed to: {m}")

async def run_loop():
    app, ctx = omni.kit.app.get_app(), omni.usd.get_context()
    
    # Handle Scene Startup
    if scene := os.environ.get("ISAAC_STARTUP_SCENE"):
        wait_steps = int(os.environ.get("ISAAC_STARTUP_SCENE_WAIT_UPDATES", 5))
        for _ in range(wait_steps): 
            await app.next_update_async()
        await ctx.open_stage_async(scene)

    # ROS 2 Initialization
    if not rclpy.ok(): 
        rclpy.init()
    
    # Get initial preset from environment or default to "2"
    raw_env = os.environ.get("CYLINDER_LOOP_PRESET", "2").strip()
    start_mode = raw_env if raw_env in LOOP_PRESETS else "2"
    
    node = CylinderController(start_mode)
    
    speed_mps = 0.15
    carb.log_info(f"Cylinder speed set to {speed_mps} m/s")

    # Wait for the USD Prim to exist in the stage
    prim = None
    while not prim and app.is_running():
        if stage := ctx.get_stage(): 
            prim = stage.GetPrimAtPath(PRIM_PATH)
        if not prim: 
            await app.next_update_async()
    
    # Setup Transform Op
    xform = UsdGeom.Xformable(prim)
    ops = xform.GetOrderedXformOps()
    op = ops[0] if ops else xform.AddTranslateOp()
    
    # State Variables
    idx, progress, last_time = 0, 0.0, time.monotonic()
    pts = LOOP_PRESETS[node.loop_key]
    speed = speed_mps

    while app.is_running():
        rclpy.spin_once(node, timeout_sec=0)
        
        current_time = time.monotonic()
        dt = current_time - last_time
        last_time = current_time

        # Handle Preset Changes
        if node.loop_changed:
            pts = LOOP_PRESETS[node.loop_key]
            idx, progress = 0, 0.0
            node.loop_changed = False

        if node.active:
            dist_to_move = speed * dt
            
            while dist_to_move > 0:
                p1, p2 = pts[idx], pts[(idx + 1) % len(pts)]
                seg_len = (p2 - p1).GetLength()
                remaining_seg = seg_len - progress
                
                step = min(dist_to_move, remaining_seg)
                progress += step
                dist_to_move -= step
                
                # If it reached the end of the segment, move to next
                if progress >= seg_len - 1e-7:
                    idx = (idx + 1) % len(pts)
                    progress = 0.0
            
            # Apply the translation to the USD Prim
            p1, p2 = pts[idx], pts[(idx + 1) % len(pts)]
            seg_len = (p2 - p1).GetLength()
            new_pos = p1 + (p2 - p1) * (progress / seg_len) if seg_len > 0 else p1
            op.Set(new_pos)

        await app.next_update_async()

    node.destroy_node()
    rclpy.shutdown()

asyncio.ensure_future(run_loop())