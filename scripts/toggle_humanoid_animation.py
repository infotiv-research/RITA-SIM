import asyncio
import builtins
import carb
import omni.anim.graph.core as ag
import omni.kit.app
import rclpy
from std_msgs.msg import Bool

CHAR_PATH = "/Characters/dab/Rokoko_Video_Character_Skele"
TOPIC_NAME = "/humanoid/play_anim"
TASK_KEY = "_humanoid_animation_ros_task"

async def spin_ros():
    if not rclpy.ok():
        rclpy.init()

    app = omni.kit.app.get_app()
    node = rclpy.create_node("humanoid_control_listener")
    
    def on_message(msg):
        character = ag.get_character(CHAR_PATH)
        
        if character is None:
            carb.log_warn(f"No Animation Graph character found at: {CHAR_PATH}")
            return
        
        play_value = bool(msg.data)
        character.set_variable("PlayAnim", play_value)
        carb.log_info(f"Setting PlayAnim to: {play_value}")

    node.create_subscription(Bool, TOPIC_NAME, on_message, 10)
    carb.log_info(f"Listening on {TOPIC_NAME}")

    try:
        while app.is_running():
            rclpy.spin_once(node, timeout_sec=0.0)
            await app.next_update_async()
    finally:
        node.destroy_node()

task = getattr(builtins, TASK_KEY, None)
if task and not task.done():
    task.cancel()

setattr(builtins, TASK_KEY, asyncio.ensure_future(spin_ros()))