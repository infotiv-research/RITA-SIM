import asyncio
import builtins
import os
import carb
import omni.anim.graph.core as ag
import omni.kit.app
import rclpy
from std_msgs.msg import Bool, String

ANIMATION_VARIABLES = {
    "dab": "PlayDab",
    "pick": "PlayPick",
}
CHARACTER_PATH_CANDIDATES = tuple(
    path
    for path in (
        os.environ.get("HUMANOID_CHARACTER_PATH", "").strip(),
        "/Characters/dab/Rokoko_Video_Character_Skele",
        "/Characters/pick/Rokoko_Video_Character_Skele",
    )
    if path
)
PLAY_TOPIC_NAME = "/humanoid/play_anim"
SELECT_TOPIC_NAME = "/humanoid/select_anim"
TASK_KEY = "_humanoid_animation_ros_task"

async def spin_ros():
    if not rclpy.ok():
        rclpy.init()

    app = omni.kit.app.get_app()
    node = rclpy.create_node("humanoid_control_listener")

    active_animation = "dab"
    active_character_path = None

    def get_character():
        nonlocal active_character_path

        for path in CHARACTER_PATH_CANDIDATES:
            character = ag.get_character(path)
            if character is not None:
                if active_character_path != path:
                    carb.log_info(f"Using humanoid character at: {path}")
                    active_character_path = path
                return character

        active_character_path = None
        return None

    def set_animation_state(selected_name, play_value):
        if selected_name not in ANIMATION_VARIABLES:
            carb.log_warn(
                f"Unknown humanoid animation '{selected_name}'. "
                f"Supported values: {', '.join(ANIMATION_VARIABLES)}"
            )
            return False

        character = get_character()
        if character is None:
            carb.log_warn(
                "No Animation Graph character found. Checked: "
                + ", ".join(CHARACTER_PATH_CANDIDATES)
            )
            return False

        for animation_name, variable_name in ANIMATION_VARIABLES.items():
            variable_value = bool(play_value) and animation_name == selected_name
            try:
                character.set_variable(variable_name, variable_value)
            except Exception as exc:
                carb.log_warn(
                    f"Failed to set '{variable_name}' on '{active_character_path}': {exc}"
                )
                return False

        return True

    def clear_animation_state():
        character = get_character()
        if character is None:
            carb.log_warn(
                "No Animation Graph character found. Checked: "
                + ", ".join(CHARACTER_PATH_CANDIDATES)
            )
            return False

        for variable_name in ANIMATION_VARIABLES.values():
            try:
                character.set_variable(variable_name, False)
            except Exception as exc:
                carb.log_warn(
                    f"Failed to clear '{variable_name}' on '{active_character_path}': {exc}"
                )
                return False

        return True

    def on_play_message(msg):
        play_value = bool(msg.data)
        if not play_value:
            if clear_animation_state():
                carb.log_info("Stopping all humanoid animations")
            return

        if set_animation_state(active_animation, True):
            carb.log_info(
                "Setting humanoid animation "
                f"'{active_animation}' to true"
            )

    def on_select_message(msg):
        nonlocal active_animation

        requested_animation = msg.data.strip().lower()
        if requested_animation in {"", "stop", "none"}:
            if clear_animation_state():
                carb.log_info("Stopping all humanoid animations")
            return

        if requested_animation not in ANIMATION_VARIABLES:
            carb.log_warn(
                f"Unknown humanoid animation '{requested_animation}'. "
                f"Supported values: {', '.join(ANIMATION_VARIABLES)}"
            )
            return

        active_animation = requested_animation
        if set_animation_state(active_animation, True):
            carb.log_info(f"Selected humanoid animation: {active_animation}")

    node.create_subscription(Bool, PLAY_TOPIC_NAME, on_play_message, 10)
    node.create_subscription(String, SELECT_TOPIC_NAME, on_select_message, 10)
    carb.log_info(
        f"Listening on {PLAY_TOPIC_NAME} (Bool) and {SELECT_TOPIC_NAME} (String)"
    )

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
