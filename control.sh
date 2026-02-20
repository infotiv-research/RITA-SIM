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
  sim             Start Isaac Sim loaded with scene_with_flowrack_and_crates2.usd

UR10 cuMotion container commands:
  cumotion        Start cuMotion

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
    pkill -f ur_robotiq_isaac_control.launch.py 2>/dev/null
    pkill -f ur_robotiq_isaac_moveit.launch.py 2>/dev/null
    pkill -f move_group 2>/dev/null
    pkill -f rviz2 2>/dev/null
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
    ./startup_scripts/start_cumotion_planner.sh
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
    cumotion
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
