"""
Launch file for the modular target-object pick-and-place node.

Launch this after the selected planner backend is already running:
- MoveIt backend: ur_robotiq_isaac_moveit.launch.py / ./control.sh cumotion
- curobo backend: ur_robotiq_curobo.launch.py / ./control.sh curobo
- Hybrid backend: curobo_hybrid_planning.launch.py / ./control.sh hybrid
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    default_assets_root = os.path.abspath(
        os.path.join(
            get_package_share_directory("ur_robotiq_moveit_config"),
            "..",
            "..",
            "..",
            "..",
            "assets",
            "isaac_urdf_exports",
        )
    )

    declared_arguments = [
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="true",
            description="Use simulation time.",
        ),
        DeclareLaunchArgument(
            "motion_backend",
            default_value="moveit",
            description="Motion backend: moveit, curobo_ros, or hybrid.",
        ),
        DeclareLaunchArgument(
            "planning_pipeline",
            default_value="cumotion",
            description="Pipeline hint for MoveGroup requests: cumotion or ompl.",
        ),
        DeclareLaunchArgument(
            "curobo_planner_type",
            default_value="classic",
            description="Planner type hint for curobo backend: classic, multipoint, or mpc.",
        ),
        DeclareLaunchArgument(
            "hybrid_precision_descent_enabled",
            default_value="true",
            description=(
                "When motion_backend=hybrid, use direct cuRobo MotionGen for the "
                "final grasp and drop descents."
            ),
        ),
        DeclareLaunchArgument(
            "hybrid_precision_descent_planner_type",
            default_value="classic",
            description=(
                "Direct cuRobo planner type used for hybrid precision descents."
            ),
        ),
        DeclareLaunchArgument(
            "hybrid_precision_descent_grasp_enabled",
            default_value="true",
            description="Use direct cuRobo MotionGen for the final grasp descent.",
        ),
        DeclareLaunchArgument(
            "hybrid_precision_descent_dropoff_enabled",
            default_value="true",
            description="Use direct cuRobo MotionGen for the final drop descent.",
        ),
        DeclareLaunchArgument(
            "hybrid_precision_descent_planning_time_s",
            default_value="0.0",
            description=(
                "Optional planning-time override for hybrid precision descents. "
                "Use 0 to keep the existing stage-specific budgets."
            ),
        ),
        DeclareLaunchArgument(
            "hybrid_precision_descent_num_attempts",
            default_value="0",
            description=(
                "Optional num_planning_attempts override for hybrid precision descents. "
                "Use 0 to keep the existing stage-specific budgets."
            ),
        ),
        DeclareLaunchArgument(
            "curobo_carried_object_sphere_count",
            default_value="3",
            description="Number of temporary carried-object spheres for curobo transport planning.",
        ),
        DeclareLaunchArgument(
            "move_group_replan_attempts",
            default_value="1",
            description="MoveGroup-level replan attempts per request.",
        ),
        DeclareLaunchArgument(
            "post_grasp_lift_z",
            default_value="0.08",
            description=(
                "Lift distance (meters) executed right after close and before attachment."
            ),
        ),
        DeclareLaunchArgument(
            "target_object_id",
            default_value="rubiks_cube",
            description="Planning-scene object ID to grasp/attach/suppress.",
        ),
        DeclareLaunchArgument(
            "target_object_frame",
            default_value="rubiks_cube",
            description="TF frame used to look up the grasp target pose.",
        ),
        DeclareLaunchArgument(
            "assets_root",
            default_value=default_assets_root,
            description="Path to exported Isaac object URDF assets.",
        ),
        DeclareLaunchArgument(
            "release_pose_qx",
            default_value="1.0",
            description="Release pose quaternion x component.",
        ),
        DeclareLaunchArgument(
            "release_pose_qy",
            default_value="0.0",
            description="Release pose quaternion y component.",
        ),
        DeclareLaunchArgument(
            "release_pose_qz",
            default_value="0.0",
            description="Release pose quaternion z component.",
        ),
        DeclareLaunchArgument(
            "release_pose_qw",
            default_value="0.0",
            description="Release pose quaternion w component.",
        ),
        DeclareLaunchArgument(
            "predrop_offset_z",
            default_value="0.10",
            description="Vertical offset above the release pose used before lowering to place.",
        ),
        DeclareLaunchArgument(
            "post_release_retreat_z",
            default_value="0.10",
            description="Vertical retreat distance after opening the gripper at the release pose.",
        ),
        DeclareLaunchArgument(
            "dropoff_planning_time_s",
            default_value="8.0",
            description="Planning time budget per dropoff attempt (seconds).",
        ),
        DeclareLaunchArgument(
            "dropoff_num_planning_attempts",
            default_value="8",
            description="MoveIt num_planning_attempts used for dropoff attempts.",
        ),
        DeclareLaunchArgument(
            "dropoff_position_tolerance_m",
            default_value="0.015",
            description="Position tolerance radius for dropoff pose goals (meters).",
        ),
        DeclareLaunchArgument(
            "dropoff_orientation_tolerance_rad",
            default_value="0.35",
            description="Strict orientation tolerance for dropoff pose goals (radians).",
        ),
        DeclareLaunchArgument(
            "dropoff_orientation_z_tolerance_rad",
            default_value="2.5",
            description="Strict Z-axis orientation tolerance for dropoff pose goals (radians).",
        ),
        DeclareLaunchArgument(
            "dropoff_relaxed_orientation_tolerance_rad",
            default_value="0.8",
            description="Relaxed orientation tolerance for dropoff fallback attempts (radians).",
        ),
        DeclareLaunchArgument(
            "dropoff_relaxed_orientation_z_tolerance_rad",
            default_value="3.14",
            description="Relaxed Z-axis orientation tolerance for dropoff fallback attempts (radians).",
        ),
        DeclareLaunchArgument(
            "dropoff_use_current_orientation_fallback",
            default_value="true",
            description="Use current EE orientation as an additional dropoff fallback attempt.",
        ),
        DeclareLaunchArgument(
            "dropoff_max_pose_retries",
            default_value="3",
            description="Maximum number of explicit dropoff pose planning retries.",
        ),
        DeclareLaunchArgument(
            "dropoff_debug_diagnostics",
            default_value="true",
            description=(
                "Enable verbose per-attempt diagnostics for dropoff retry failures."
            ),
        ),
        DeclareLaunchArgument(
            "cumotion_attachment_target_sphere_diameter_m",
            default_value="0.03",
            description="Target sphere diameter used to approximate attached-object geometry.",
        ),
        DeclareLaunchArgument(
            "cumotion_attachment_min_spheres",
            default_value="8",
            description="Minimum desired number of cuMotion attached-object spheres.",
        ),
        DeclareLaunchArgument(
            "cumotion_attachment_max_spheres",
            default_value="32",
            description="Maximum allowed number of cuMotion attached-object spheres.",
        ),
    ]

    pick_and_place_node = Node(
        package="ur_robotiq_moveit_config",
        executable="pick_and_place_main.py",
        name="pick_and_place",
        output="screen",
        on_exit=[EmitEvent(event=Shutdown(reason="pick_and_place node exited"))],
        parameters=[
            {
                "use_sim_time": LaunchConfiguration("use_sim_time"),
                "motion_backend": LaunchConfiguration("motion_backend"),
                "planning_pipeline": LaunchConfiguration("planning_pipeline"),
                "curobo_planner_type": LaunchConfiguration("curobo_planner_type"),
                "hybrid_precision_descent_enabled": LaunchConfiguration(
                    "hybrid_precision_descent_enabled"
                ),
                "hybrid_precision_descent_planner_type": LaunchConfiguration(
                    "hybrid_precision_descent_planner_type"
                ),
                "hybrid_precision_descent_grasp_enabled": LaunchConfiguration(
                    "hybrid_precision_descent_grasp_enabled"
                ),
                "hybrid_precision_descent_dropoff_enabled": LaunchConfiguration(
                    "hybrid_precision_descent_dropoff_enabled"
                ),
                "hybrid_precision_descent_planning_time_s": LaunchConfiguration(
                    "hybrid_precision_descent_planning_time_s"
                ),
                "hybrid_precision_descent_num_attempts": LaunchConfiguration(
                    "hybrid_precision_descent_num_attempts"
                ),
                "curobo_carried_object_sphere_count": LaunchConfiguration(
                    "curobo_carried_object_sphere_count"
                ),
                "move_group_replan_attempts": LaunchConfiguration(
                    "move_group_replan_attempts"
                ),
                "post_grasp_lift_z": LaunchConfiguration("post_grasp_lift_z"),
                "target_object_id": LaunchConfiguration("target_object_id"),
                "target_object_frame": LaunchConfiguration("target_object_frame"),
                "assets_root": LaunchConfiguration("assets_root"),
                "release_pose_qx": LaunchConfiguration("release_pose_qx"),
                "release_pose_qy": LaunchConfiguration("release_pose_qy"),
                "release_pose_qz": LaunchConfiguration("release_pose_qz"),
                "release_pose_qw": LaunchConfiguration("release_pose_qw"),
                "predrop_offset_z": LaunchConfiguration("predrop_offset_z"),
                "post_release_retreat_z": LaunchConfiguration(
                    "post_release_retreat_z"
                ),
                "dropoff_planning_time_s": LaunchConfiguration(
                    "dropoff_planning_time_s"
                ),
                "dropoff_num_planning_attempts": LaunchConfiguration(
                    "dropoff_num_planning_attempts"
                ),
                "dropoff_position_tolerance_m": LaunchConfiguration(
                    "dropoff_position_tolerance_m"
                ),
                "dropoff_orientation_tolerance_rad": LaunchConfiguration(
                    "dropoff_orientation_tolerance_rad"
                ),
                "dropoff_orientation_z_tolerance_rad": LaunchConfiguration(
                    "dropoff_orientation_z_tolerance_rad"
                ),
                "dropoff_relaxed_orientation_tolerance_rad": LaunchConfiguration(
                    "dropoff_relaxed_orientation_tolerance_rad"
                ),
                "dropoff_relaxed_orientation_z_tolerance_rad": LaunchConfiguration(
                    "dropoff_relaxed_orientation_z_tolerance_rad"
                ),
                "dropoff_use_current_orientation_fallback": LaunchConfiguration(
                    "dropoff_use_current_orientation_fallback"
                ),
                "dropoff_max_pose_retries": LaunchConfiguration(
                    "dropoff_max_pose_retries"
                ),
                "dropoff_debug_diagnostics": LaunchConfiguration(
                    "dropoff_debug_diagnostics"
                ),
                "cumotion_attachment_target_sphere_diameter_m": LaunchConfiguration(
                    "cumotion_attachment_target_sphere_diameter_m"
                ),
                "cumotion_attachment_min_spheres": LaunchConfiguration(
                    "cumotion_attachment_min_spheres"
                ),
                "cumotion_attachment_max_spheres": LaunchConfiguration(
                    "cumotion_attachment_max_spheres"
                ),
            }
        ],
    )

    return LaunchDescription(declared_arguments + [pick_and_place_node])
