import importlib.util
import os

from ament_index_python.packages import get_package_prefix
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _load_base_launch_module():
    base_launch_path = os.path.join(
        os.path.dirname(__file__),
        "ur_robotiq_curobo.launch.py",
    )
    spec = importlib.util.spec_from_file_location(
        "ur_robotiq_curobo_base_launch",
        base_launch_path,
    )
    if spec is None or spec.loader is None:
        raise RuntimeError(f"Unable to load launch file: {base_launch_path}")

    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _human_collision_action(context):
    installed_executable = os.path.join(
        get_package_prefix("ur_robotiq_moveit_config"),
        "lib",
        "ur_robotiq_moveit_config",
        "curobo_human_skeleton_collision_publisher.py",
    )

    parameters = {
        "world_frame": LaunchConfiguration("world_frame"),
        "publish_rate_hz": LaunchConfiguration("environment_publish_rate_hz"),
        "marker_topic": "/curobo_human_collision_markers",
        "use_sim_time": LaunchConfiguration("use_sim_time"),
    }

    if os.path.exists(installed_executable):
        return [
            Node(
                package="ur_robotiq_moveit_config",
                executable="curobo_human_skeleton_collision_publisher.py",
                name="curobo_human_skeleton_collision_publisher",
                condition=IfCondition(LaunchConfiguration("publish_human_collisions")),
                output="screen",
                parameters=[parameters],
            )
        ]

    user_ws = os.environ.get("USER_WS", "/ros2_ws")
    source_script = os.path.join(
        user_ws,
        "src",
        "ur_robotiq_moveit_config",
        "scripts",
        "curobo",
        "curobo_human_skeleton_collision_publisher.py",
    )
    if not os.path.exists(source_script):
        raise RuntimeError(
            f"Missing cuRobo human collision publisher at '{installed_executable}' "
            f"and '{source_script}'. Build ur_robotiq_moveit_config or restore the source script."
        )

    return [
        ExecuteProcess(
            condition=IfCondition(LaunchConfiguration("publish_human_collisions")),
            output="screen",
            cmd=[
                "python3",
                source_script,
                "--ros-args",
                "-r",
                "__node:=curobo_human_skeleton_collision_publisher",
                "-p",
                f"world_frame:={LaunchConfiguration('world_frame').perform(context)}",
                "-p",
                f"publish_rate_hz:={LaunchConfiguration('environment_publish_rate_hz').perform(context)}",
                "-p",
                "marker_topic:=/curobo_human_collision_markers",
                "-p",
                f"use_sim_time:={LaunchConfiguration('use_sim_time').perform(context)}",
            ],
        )
    ]


def generate_launch_description():
    base_launch_module = _load_base_launch_module()
    launch_description = base_launch_module.generate_launch_description()

    launch_description.add_action(
        DeclareLaunchArgument(
            "publish_human_collisions",
            default_value="true",
            description=(
                "Publish cuRobo humanoid collision primitives from the *_Skele TF frames."
            ),
        )
    )
    launch_description.add_action(OpaqueFunction(function=_human_collision_action))
    return launch_description
