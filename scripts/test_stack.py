#!/usr/bin/env python3
from __future__ import annotations

import subprocess
import sys
import threading
import time
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[1]
LOG_DIR = REPO_ROOT / "test_logs"

COMPOSE = [
    "docker",
    "compose",
    "-p",
    "rita_stack",
    "-f",
    str(REPO_ROOT / "setup/docker-compose.ros2.yaml"),
    "-f",
    str(REPO_ROOT / "setup/docker-compose.cumotion.yaml"),
    "-f",
    str(REPO_ROOT / "setup/docker-compose.isaac.yaml"),
]

ROS_READY_CMD = (
    "source /opt/ros/${ROS_DISTRO:-jazzy}/setup.bash && "
    "(source /ros2_ws/install/local_setup.bash 2>/dev/null || true) && "
    "ros2 action info /joint_trajectory_controller/follow_joint_trajectory 2>/dev/null "
    '| grep -Eq "Action servers: +1" && '
    "ros2 action info /robotiq_gripper_joint_trajectory_controller/follow_joint_trajectory 2>/dev/null "
    '| grep -Eq "Action servers: +1"'
)

ISAAC_READY_CMD = 'test "$(cat /tmp/rita_isaac_timeline_state 2>/dev/null)" = "stopped"'

CUROBO_READY_CMD = (
    "source /opt/ros/${ROS_DISTRO:-jazzy}/setup.bash && "
    "(source /ros2_ws/install/local_setup.bash 2>/dev/null || true) && "
    "ros2 param get /unified_planner startup_ready 2>/dev/null "
    '| grep -Eiq "true"'
)

MOVEIT_READY_CMD = (
    "source /opt/ros/${ROS_DISTRO:-jazzy}/setup.bash && "
    "(source /ros2_ws/install/local_setup.bash 2>/dev/null || true) && "
    "ros2 action info /move_action 2>/dev/null "
    '| grep -Eq "Action servers: +1"'
)

CUMOTION_READY_CMD = (
    MOVEIT_READY_CMD + " && "
    "ros2 action info /planner_attach_object 2>/dev/null "
    '| grep -Eq "Action servers: +1"'
)

HYBRID_READY_CMD = (
    CUROBO_READY_CMD + " && "
    "ros2 action info /move_action 2>/dev/null "
    '| grep -Eq "Action servers: +1" && '
    "ros2 action info /run_hybrid_planning 2>/dev/null "
    '| grep -Eq "Action servers: +1" && '
    "ros2 service type /plan_kinematic_path 2>/dev/null "
    '| grep -Eq "^moveit_msgs/srv/GetMotionPlan$"'
)


def compose_quiet(*args: str) -> bool:
    result = subprocess.run(
        [*COMPOSE, *args],
        cwd=REPO_ROOT,
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
        text=True,
    )
    return result.returncode == 0


def wait_until_ready(service: str, command: str, message: str) -> None:
    while not compose_quiet("exec", "-T", service, "bash", "-lc", command):
        time.sleep(2)
    print(message, flush=True)


def prepare_logs() -> int:
    LOG_DIR.mkdir(exist_ok=True)
    return 0


def prepare_start() -> int:
    prepare_logs()
    compose_quiet(
        "exec",
        "-T",
        "isaacsim",
        "bash",
        "-lc",
        "rm -f /tmp/rita_isaac_timeline_command /tmp/rita_isaac_timeline_state",
    )
    return 0


def wait_start() -> int:
    ros_thread = threading.Thread(
        target=wait_until_ready,
        args=("ros2", ROS_READY_CMD, "ros2 started"),
    )
    isaac_thread = threading.Thread(
        target=wait_until_ready,
        args=("isaacsim", ISAAC_READY_CMD, "isaac sim started"),
    )

    ros_thread.start()
    isaac_thread.start()
    ros_thread.join()
    isaac_thread.join()
    return 0


def wait_curobo() -> int:
    wait_until_ready("cumotion", CUROBO_READY_CMD, "done")
    return 0


def wait_cumotion() -> int:
    wait_until_ready("cumotion", CUMOTION_READY_CMD, "cumotion started")
    return 0


def wait_ompl() -> int:
    wait_until_ready("cumotion", MOVEIT_READY_CMD, "ompl started")
    return 0


def wait_hybrid() -> int:
    wait_until_ready("cumotion", HYBRID_READY_CMD, "hybrid started")
    return 0


def kill_non_ros() -> int:
    command = r'''
for pattern in \
  "ur_robotiq_isaac_moveit.launch.py" \
  "ur_robotiq_isaac_moveit_human.launch.py" \
  "ur_robotiq_curobo.launch.py" \
  "ur_robotiq_curobo_human.launch.py" \
  "pick_and_place.launch.py" \
  "pick_and_place_main.py" \
  "start_cumotion_planner.sh" \
  "start_curobo_planner.sh" \
  "start_hybrid_planner.sh" \
  "bootstrap_cumotion_workspace.sh" \
  "cumotion_planner_upstream_framefix.py" \
  "cumotion_planner_node" \
  "curobo_trajectory_planner" \
  "curobo_human_skeleton_collision_publisher.py" \
  "curobo_live_collision_spheres.py" \
  "isaac_urdf_collision_publisher.py" \
  "curobo_world_bridge.py" \
  "joint_state_publisher" \
  "moveit_joint_state_filter" \
  "move_group" \
  "rviz2"
do
  for pid in $(pgrep -f "$pattern" 2>/dev/null || true); do
    if [ "$pid" != "$$" ]; then
      kill "$pid" 2>/dev/null || true
    fi
  done
done
'''
    compose_quiet("exec", "-T", "cumotion", "bash", "-lc", command)
    return 0


def main(argv: list[str]) -> int:
    if not argv:
        return 1

    command = argv[0]
    if command == "prepare_logs":
        return prepare_logs()
    if command == "prepare_start":
        return prepare_start()
    if command == "wait_start":
        return wait_start()
    if command == "wait_curobo":
        return wait_curobo()
    if command == "wait_cumotion":
        return wait_cumotion()
    if command == "wait_ompl":
        return wait_ompl()
    if command == "wait_hybrid":
        return wait_hybrid()
    if command == "kill_non_ros":
        return kill_non_ros()
    return 1


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
