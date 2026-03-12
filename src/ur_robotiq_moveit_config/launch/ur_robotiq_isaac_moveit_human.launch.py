import importlib.util
import os

from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _load_base_launch_module():
    base_launch_path = os.path.join(
        os.path.dirname(__file__),
        "ur_robotiq_isaac_moveit.launch.py",
    )
    spec = importlib.util.spec_from_file_location(
        "ur_robotiq_isaac_moveit_base_launch",
        base_launch_path,
    )
    if spec is None or spec.loader is None:
        raise RuntimeError(f"Unable to load launch file: {base_launch_path}")

    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def generate_launch_description():
    base_launch_module = _load_base_launch_module()
    launch_description = base_launch_module.generate_launch_description()

    launch_description.add_action(
        DeclareLaunchArgument(
            "publish_human_collisions",
            default_value="true",
            description=(
                "Publish dynamic human-body collision cylinders and torso box from "
                "the *_Skele TF frames."
            ),
        )
    )
    launch_description.add_action(
        Node(
            package="ur_robotiq_moveit_config",
            executable="human_skeleton_collision_publisher.py",
            name="human_skeleton_collision_publisher",
            condition=IfCondition(LaunchConfiguration("publish_human_collisions")),
            output="screen",
            parameters=[
                {
                    "world_frame": LaunchConfiguration("environment_collision_world_frame"),
                    "publish_rate_hz": LaunchConfiguration("environment_collision_publish_rate_hz"),
                    "use_sim_time": LaunchConfiguration("use_sim_time"),
                }
            ],
        )
    )
    return launch_description
