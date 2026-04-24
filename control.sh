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
CYLINDER_TOGGLE_STATE_FILE="${CYLINDER_TOGGLE_STATE_FILE:-/tmp/cylinder_move_state}"
CYLINDER_PATH_VALUE_FILE="${CYLINDER_PATH_VALUE_FILE:-/tmp/cylinder_path_value}"
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
  source_ws       Source /opt/ros + install/local_setup.bash in this script context
  kill            Stop ROS 2 launch processes started by this repo

UR10 ROS2 container commands:
  ros             Start ROS 2 launch
  planning        Start MoveIt/planning launch
  ur10            Start both launches

Isaac container commands:
  sim             Start Isaac Sim loaded with main_scene.usd
  sim_headless    Start Isaac Sim headless, or control the headless timeline.
                  Usage:
                    ./control.sh sim_headless        - Start Isaac Sim headless
                    ./control.sh sim_headless play   - Play the headless simulation timeline
                    ./control.sh sim_headless stop   - Stop the headless simulation timeline
  sim_cylinder    Start Isaac Sim loaded with flowrack_crates_and_robot_cylinder.usd (The same scene but scaled down and with an moving cylinder (obstacle)
                  for dynamic path planning)
  sim_humanoid    Start Isaac Sim loaded with flowrack_crates_and_robot_humanoid.usd
  build_gaussian_splats
                  Build the 3DGRUT container and convert all source .ply files under assets/ into sibling .usdz files with collision.
  cylinder        Control cylinder movement.
                  Usage:
                    ./control.sh cylinder set <number>  - Set the desired path number for cylinder movement (2, 3 or 4)
                    ./control.sh cylinder start         - Start cylinder movement using the set path number
                    ./control.sh cylinder stop          - Stop cylinder movement
  humanoid        Control humanoid animation.
                  Usage:
                    ./control.sh humanoid stop          - Stop humanoid animation
                    ./control.sh humanoid play dab      - Select and play the dab animation
                    ./control.sh humanoid play pick     - Select and play the pick animation


UR10 cuMotion container commands:
  cumotion        Start cuMotion with 7dof UR10e gantry config by default.
  curobo          Start curobo_ros with dedicated RViz workflow.
  hybrid          Start MoveIt Hybrid Planning (cuRobo MotionGen + MPC).
  hybrid_benchmark
                  Run the scripted hybrid obstacle benchmark.
                  Usage:
                    ./control.sh hybrid_benchmark --case test_1 --runs 1
  pick_and_place  Start pick-and-place node.
                  Usage:
                    ./control.sh pick_and_place
                    ./control.sh pick_and_place <object_id> [object_frame]
                    ./control.sh pick_and_place _03_cracker_box _03_cracker_box
                    ./control.sh pick_and_place _03_cracker_box _03_cracker_box planning_pipeline:=ompl
                    ./control.sh pick_and_place motion_backend:=curobo_ros
                    ./control.sh pick_and_place motion_backend:=curobo_ros curobo_planner_type:=mpc

Misc:
  cmd <...>       Run any command directly (passthrough)
  help            Show this help
EOF
}

source_ws() {
    source "/opt/ros/${ROS_DISTRO:-jazzy}/setup.bash" 2>/dev/null
    source /opt/vendor_ws/install/setup.bash 2>/dev/null
    source install/local_setup.bash 2>/dev/null
}

reset_workspace_prefix_env() {
    unset AMENT_PREFIX_PATH
    unset CMAKE_PREFIX_PATH
    unset COLCON_PREFIX_PATH
}

kill_processes() {
    echo "---[ killing launch processes ]---"
    local patterns=(
        "ur_robotiq_isaac_control.launch.py"
        "ur_robotiq_isaac_moveit.launch.py"
        "ur_robotiq_curobo.launch.py"
        "ur_robotiq_curobo_human.launch.py"
        "pick_and_place.launch.py"
        "pick_and_place_main.py"
        "start_cumotion_planner.sh"
        "start_curobo_planner.sh"
        "start_hybrid_planner.sh"
        "curobo_hybrid_planning.launch.py"
        "bootstrap_cumotion_workspace.sh"
        "cumotion_planner_upstream_framefix.py"
        "cumotion_planner_node"
        "curobo_trajectory_planner"
        "hybrid_move_action_bridge.py"
        "curobo_hybrid_planning_container"
        "curobo_human_skeleton_collision_publisher.py"
        "curobo_live_collision_spheres.py"
        "isaac_urdf_collision_publisher.py"
        "curobo_world_bridge.py"
        "joint_state_publisher"
        "moveit_joint_state_filter"
        "move_group"
        "rviz2"
    )

    for pattern in "${patterns[@]}"; do
        pkill -f "$pattern" 2>/dev/null || true
    done
}

cleanup_stale_ros_stack() {
    # Multiple concurrent ROS control stacks create multiple /controller_manager nodes.
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
    reset_workspace_prefix_env
    source_ws
    if ! command -v colcon >/dev/null 2>&1; then
        echo "---[ Error: colcon not found in PATH ]---"
        return 1
    fi

    if ! colcon build --symlink-install --cmake-args -DBUILD_TESTING=OFF -DCMAKE_BUILD_TYPE=RelWithDebInfo -DCMAKE_CXX_FLAGS="-Wno-deprecated-declarations -Wno-unused-result -Wno-reorder -Wno-cpp"; then
        echo "---[ Error: colcon build failed ]---"
        return 1
    fi

    source install/local_setup.bash 2>/dev/null
    echo "---[ build complete ]---"
}

clean() {
    echo "---[ cleaning workspace ]---"
    kill_processes
    rm -rf ./build ./install ./log
    echo "---[ clean complete ]---"
}

ros() {
    echo "---[ launching ur_robotiq_isaac_control ]---"
    source_ws
    cleanup_stale_ros_stack
    ros2 launch ur_robotiq_description ur_robotiq_isaac_control.launch.py
}

ompl() {
    echo "---[ launching ur_robotiq_isaac_moveit ]---"
    source_ws
    ros2 launch ur_robotiq_moveit_config ur_robotiq_isaac_moveit.launch.py \
        planning_pipeline:=ompl \
        launch_cumotion_planner:=false
}

isaac_sim() {
    echo "---[ starting Isaac Sim ]---"
    ./startup_scripts/post_install_ros2_isaac_start.sh
}

isaac_sim_headless() {
    "${ISAAC_SIM_PATH:-/isaac-sim}/python.sh" "${SCRIPT_DIR}/scripts/start_isaac_main_scene.py" --headless "$@"
}

isaac_sim_cylinder() {
    export CYLINDER_LOOP_PRESET="${CYLINDER_LOOP_PRESET:-2}"
    export ISAAC_STARTUP_OPEN_SCRIPT="${SCRIPT_DIR}/startup_scripts/cylinder_ros_control.py"
    
    ./startup_scripts/post_install_ros2_isaac_start.sh "assets/ur10e_robotiq2f-140/flowrack_crates_and_robot_cylinder.usd"
}

isaac_sim_humanoid() {
    echo "---[ starting Isaac Sim with humanoid ]---"
    ./startup_scripts/post_install_ros2_isaac_start.sh "assets/ur10e_robotiq2f-140/flowrack_crates_and_robot_humanoid.usd"
}

build_gaussian_splats() {
    (
        set -euo pipefail
        cd "${SCRIPT_DIR}"

        trap 'docker compose -f setup/docker-compose.3dgrut.yaml down' EXIT

        docker compose -f setup/docker-compose.3dgrut.yaml build

        find assets -type f -name '*.ply' ! -name '*_hull.ply' | while read -r ply_file; do
            ./scripts/ply_to_usdz.sh "$ply_file" "${ply_file%.ply}.usdz" --with-collision </dev/null
        done
    )
}

cylinder_action() {
    source_ws

    if [ "$1" == "set" ]; then
        if [ -z "$2" ]; then
            echo "---[ Error: Please provide a number, e.g., ./control.sh cylinder set 2 ]---"
            return 1
        fi
        echo "$2" > "$CYLINDER_PATH_VALUE_FILE"
        echo "---[ Cylinder path value set to: $2 ]---"

    elif [ "$1" == "start" ]; then
        if [ ! -f "$CYLINDER_PATH_VALUE_FILE" ]; then
            echo "---[ Error: Path value not set! Run './control.sh cylinder set <number>' first. ]---"
            return 1
        fi
        
        PATH_NUM=$(cat "$CYLINDER_PATH_VALUE_FILE")
        ros2 topic pub --once /move_cylinder_loop std_msgs/msg/Int32 "{data: $PATH_NUM}" || return 1
        ros2 topic pub --once /move_cylinder std_msgs/msg/Bool "{data: true}" || return 1
        echo "true" > "$CYLINDER_TOGGLE_STATE_FILE"
        echo "---[ Cylinder started with path value: $PATH_NUM ]---"

    elif [ "$1" == "stop" ]; then
        ros2 topic pub --once /move_cylinder std_msgs/msg/Bool "{data: false}" || return 1
        echo "false" > "$CYLINDER_TOGGLE_STATE_FILE"
        echo "---[ Cylinder stopped ]---"

    else
        echo "---[ Unknown command. Usage: ./control.sh cylinder <set <number> | start | stop> ]---"
        return 1
    fi
}

humanoid_action() {
    source_ws

    if [ "$1" == "stop" ]; then
        ros2 topic pub --once /humanoid/play_anim std_msgs/msg/Bool "{data: false}" || return 1
        echo "---[ Humanoid animation stopped ]---"

    elif [ "$1" == "play" ]; then
        if [ -z "${2:-}" ]; then
            echo "---[ Error: Please provide an animation name: dab or pick ]---"
            return 1
        fi

        case "$2" in
            dab|pick)
                ros2 topic pub --once /humanoid/select_anim std_msgs/msg/String "{data: '$2'}" || return 1
                echo "---[ Humanoid animation selected: $2 ]---"
                ;;
            *)
                echo "---[ Error: Unknown humanoid animation '$2'. Use: dab or pick ]---"
                return 1
                ;;
        esac

    else
        echo "---[ Unknown command. Usage: ./control.sh humanoid <stop | play <dab|pick>> ]---"
        return 1
    fi
}

cumotion() {
    echo "---[ starting cuMotion script ]---"
    ./startup_scripts/start_cumotion_planner.sh "$@"
}

curobo() {
    echo "---[ starting curobo script ]---"
    ./startup_scripts/start_curobo_planner.sh "$@"
}

hybrid() {
    echo "---[ starting hybrid planner (cuRobo MotionGen + MPC) ]---"
    ./startup_scripts/start_hybrid_planner.sh "$@"
}

hybrid_benchmark() {
    echo "---[ running hybrid obstacle benchmark ]---"
    source_ws
    ros2 run ur_robotiq_moveit_config hybrid_obstacle_benchmark.py -- "$@"
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
elif [[ "$1" == "ros" ]]; then
    ros
elif [[ "$1" == "sim" ]]; then
    isaac_sim
elif [[ "$1" == "sim_headless" ]]; then
    shift 1
    isaac_sim_headless "$@"
elif [[ "$1" == "sim_cylinder" ]]; then
    isaac_sim_cylinder
elif [[ "$1" == "sim_humanoid" ]]; then
    isaac_sim_humanoid
elif [[ "$1" == "build_gaussian_splats" ]]; then
    build_gaussian_splats
elif [[ "$1" == "ompl" ]]; then
    ompl
elif [[ "$1" == "cumotion" ]]; then
    shift 1
    cumotion "$@"
elif [[ "$1" == "curobo" ]]; then
    shift 1
    curobo "$@"
elif [[ "$1" == "hybrid" ]]; then
    shift 1
    hybrid "$@"
elif [[ "$1" == "hybrid_benchmark" ]]; then
    shift 1
    hybrid_benchmark "$@"
elif [[ "$1" == "pick_and_place" ]]; then
    shift 1
    pick_and_place "$@"
elif [[ "$1" == "cylinder" ]]; then
    shift 1
    cylinder_action "$@"
elif [[ "$1" == "humanoid" ]]; then
    shift 1
    humanoid_action "$@"
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
