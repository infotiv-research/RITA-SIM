#!/bin/bash

# The purpose of ./control.sh
# - Lightweight plumbing, mainly sequential execution of commands
# - Managing parameters and environment variables
# - Simple/unified interface for non-robotic experts (AI/ML)
# - Avoid mistakes and hassle running long ROS 2 commands
# - Streamline the process of building and running the project
# - Reduce multi-terminal setup to a small set of commands

#################################################################
#          SOURCING AND SETTING ENVIRONMENT VARIABLES           #
#################################################################
#region environment variables
source "/opt/ros/${ROS_DISTRO:-humble}/setup.bash" 2>/dev/null
source install/setup.bash 2>/dev/null
export RCUTILS_LOGGING_SEVERITY_THRESHOLD="${RCUTILS_LOGGING_SEVERITY_THRESHOLD:-WARN}"
unset ROS_LOCALHOST_ONLY
export RCUTILS_CONSOLE_OUTPUT_FORMAT="[{severity}] [{name}] [{time}]: {message}"
#endregion

#################################################################
#                  CONFIGURATION VARIABLES                       #
#       here lies all variables that can be configured           #
#################################################################
#region configuration variables
CONFIG_FILE="$(dirname "$0")/config.sh"
if [ -f "$CONFIG_FILE" ]; then
    source "$CONFIG_FILE"
fi

ROS_LAUNCH_DELAY="${ROS_LAUNCH_DELAY:-3}"

echo "---[ ROS_DOMAIN_ID: ${ROS_DOMAIN_ID:-0} ]---"
#endregion

#################################################################
#                        SAFETY CHECKS                          #
#################################################################
#region safety checks
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
CURRENT_DIR="$(pwd)"

if [[ "$CURRENT_DIR" != "$SCRIPT_DIR" ]]; then
    echo "---[ Error: Please run this script from its own directory: $SCRIPT_DIR ]---"
    exit 1
fi

if [ $# -eq 0 ]; then
    echo "---[ No arguments supplied. Try: ./control.sh help ]---"
fi
#endregion

#################################################################
#                          FUNCTIONS                            #
#################################################################
#region functions
help_cmd() {
    cat <<'EOF'
Usage: ./control.sh <command>

Core commands:
  clean           Remove workspace build artifacts
  build           Clean and rebuild workspace with colcon
  source_ws       Source /opt/ros + install/setup.bash in this script context
  kill            Stop ROS 2 launch processes started by this repo

UR10 ROS2 container commands:
  robot_control   Start robot control launch
  planning        Start MoveIt/planning launch
  ur10            Start both launches

Isaac container commands:
  sim             Start Isaac Sim loaded with main_scene.usd

UR10 cuMotion container commands:
  cumotion        Start cuMotion with 7dof UR10e gantry config by default.
  pick_and_place  Start pick-and-place node.
                  Usage:
                    ./control.sh pick_and_place
                    ./control.sh pick_and_place <object_id> [object_frame]
                    ./control.sh pick_and_place _03_cracker_box _03_cracker_box
                    ./control.sh pick_and_place _03_cracker_box _03_cracker_box planning_pipeline:=ompl

Misc:
  cmd <...>       Run any command directly (passthrough)
  help            Show this help
EOF
}

source_ws() {
    source "/opt/ros/${ROS_DISTRO:-humble}/setup.bash" 2>/dev/null
    source install/setup.bash 2>/dev/null
}

kill_processes() {
    echo "---[ killing launch processes ]---"
    local patterns=(
        "ur_robotiq_isaac_control.launch.py"
        "ur_robotiq_isaac_moveit.launch.py"
        "pick_and_place.launch.py"
        "pick_and_place_main.py"
        "start_cumotion_planner.sh"
        "bootstrap_cumotion_workspace.sh"
        "cumotion_planner_upstream_framefix.py"
        "cumotion_planner_node"
        "isaac_urdf_collision_publisher.py"
        "moveit_joint_state_filter"
        "move_group"
        "rviz2"
    )

    for pattern in "${patterns[@]}"; do
        pkill -f "$pattern" 2>/dev/null || true
    done
}

cleanup_stale_robot_control_stack() {
    # Multiple concurrent control stacks create multiple /controller_manager nodes.
    # Spawners can then talk to different managers and fail nondeterministically.
    local cm_count
    cm_count="$(ros2 node list 2>/dev/null | grep -c '^/controller_manager$' || true)"
    if [[ "${cm_count:-0}" -gt 0 ]]; then
        echo "---[ detected existing /controller_manager nodes (${cm_count}); cleaning stale control stack ]---"
        pkill -f "ur_robotiq_isaac_control.launch.py" 2>/dev/null || true
        pkill -f "/controller_manager/ros2_control_node" 2>/dev/null || true
        pkill -f "/controller_manager/spawner" 2>/dev/null || true
        sleep 1
    fi
}

build() {
    clean
    echo "---[ building workspace ]---"
    source_ws
    if ! command -v colcon >/dev/null 2>&1; then
        echo "---[ Error: colcon not found in PATH ]---"
        return 1
    fi

    if ! colcon build --symlink-install; then
        echo "---[ Error: colcon build failed ]---"
        return 1
    fi

    source install/setup.bash 2>/dev/null
    echo "---[ build complete ]---"
}

clean() {
    echo "---[ cleaning workspace ]---"
    kill_processes
    rm -rf ./build ./install ./log
    echo "---[ clean complete ]---"
}

robot_control() {
    echo "---[ launching ur_robotiq_isaac_control ]---"
    source_ws
    cleanup_stale_robot_control_stack
    ros2 launch ur_robotiq_description ur_robotiq_isaac_control.launch.py
}

moveit_planning() {
    echo "---[ launching ur_robotiq_isaac_moveit ]---"
    source_ws
    ros2 launch ur_robotiq_moveit_config ur_robotiq_isaac_moveit.launch.py
}

plan_and_control() {
    echo "---[ launching both ROS 2 launch files ]---"
    source_ws
    cleanup_stale_robot_control_stack

    ros2 launch ur_robotiq_description ur_robotiq_isaac_control.launch.py &
    control_pid=$!

    cleanup() {
        kill "$control_pid" 2>/dev/null
    }
    trap cleanup EXIT INT TERM

    sleep "$ROS_LAUNCH_DELAY"
    ros2 launch ur_robotiq_moveit_config ur_robotiq_isaac_moveit.launch.py
}

isaac_sim() {
    echo "---[ starting Isaac Sim helper script ]---"
    ./startup_scripts/post_install_ros2_isaac_start.sh
}

cumotion() {
    echo "---[ starting cuMotion script ]---"
    ./startup_scripts/start_cumotion_planner.sh "$@"
}

pick_and_place() {
    echo "---[ launching pick-and-place node ]---"
    source_ws

    # Allow shorthand:
    #   ./control.sh pick_and_place <object_id> [object_frame]
    # while still supporting full ros2 launch args with := passthrough.
    if [[ $# -ge 1 && "$1" != *":="* ]]; then
        local object_id="$1"
        local object_frame="$1"
        shift
        if [[ $# -ge 1 && "$1" != *":="* ]]; then
            object_frame="$1"
            shift
        fi
        ros2 launch ur_robotiq_moveit_config pick_and_place.launch.py \
            target_object_id:="$object_id" \
            target_object_frame:="$object_frame" \
            "$@"
        return
    fi

    ros2 launch ur_robotiq_moveit_config pick_and_place.launch.py "$@"
}
#endregion

#################################################################
#                         OPERATIONS                            #
#################################################################
#region all operations
if [[ "$1" == "help" || "$1" == "--help" || "$1" == "-h" ]]; then
    help_cmd
elif [[ "$1" == "build" ]]; then
    build
elif [[ "$1" == "clean" ]]; then
    clean
elif [[ "$1" == "source_ws" ]]; then
    source_ws
elif [[ "$1" == "kill" ]]; then
    kill_processes
elif [[ "$1" == "robot_control" ]]; then
    robot_control
elif [[ "$1" == "planning" ]]; then
    moveit_planning
elif [[ "$1" == "ur10" ]]; then
    plan_and_control
elif [[ "$1" == "sim" ]]; then
    isaac_sim
elif [[ "$1" == "cumotion" ]]; then
    shift 1
    cumotion "$@"
elif [[ "$1" == "pick_and_place" ]]; then
    shift 1
    pick_and_place "$@"
elif [[ "$1" == "cmd" ]]; then
    shift 1
    echo "running ::: $*"
    exec "$@"
elif [[ -z "$1" ]]; then
    help_cmd
else
    echo "---[ Unknown command: $1 ]---"
    help_cmd
    exit 1
fi
#endregion
