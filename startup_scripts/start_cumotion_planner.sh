#!/bin/bash
set -euo pipefail

ROS_DISTRO="${ROS_DISTRO:-humble}"
USER_WS="${USER_WS:-/ros2_ws}"
TORCH_EXTENSIONS_DIR="${TORCH_EXTENSIONS_DIR:-${USER_WS}/.cache/torch_extensions}"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

timestamp() {
  date +"%Y-%m-%d %H:%M:%S"
}

log() {
  echo "[start_cumotion_planner][$(timestamp)] $*"
}

log "Starting cuMotion planner launcher."
log "ROS_DISTRO=${ROS_DISTRO} USER_WS=${USER_WS}"
log "TORCH_EXTENSIONS_DIR=${TORCH_EXTENSIONS_DIR}"

if [ "$#" -gt 0 ]; then
  log "User launch args: $*"
fi

mkdir -p "${TORCH_EXTENSIONS_DIR}"
export TORCH_EXTENSIONS_DIR

if [ "${CUMOTION_SKIP_BOOTSTRAP:-false}" != "true" ]; then
  if [ ! -x "${SCRIPT_DIR}/bootstrap_cumotion_workspace.sh" ]; then
    echo "ERROR: Missing executable bootstrap script at ${SCRIPT_DIR}/bootstrap_cumotion_workspace.sh"
    exit 1
  fi
  log "Running bootstrap/validation checks before launch."
  log "Note: first run can take several minutes due cuRobo CUDA JIT compilation."
  bootstrap_start_s="$(date +%s)"
  "${SCRIPT_DIR}/bootstrap_cumotion_workspace.sh"
  bootstrap_end_s="$(date +%s)"
  log "Bootstrap/validation finished in $((bootstrap_end_s - bootstrap_start_s))s."
else
  log "Skipping bootstrap because CUMOTION_SKIP_BOOTSTRAP=true."
fi

log "Sourcing ROS environments."
set +u
if [ -f "/opt/ros/${ROS_DISTRO}/setup.bash" ]; then
  source "/opt/ros/${ROS_DISTRO}/setup.bash"
  log "Sourced /opt/ros/${ROS_DISTRO}/setup.bash"
else
  log "WARNING: /opt/ros/${ROS_DISTRO}/setup.bash not found"
fi
if [ -f "${USER_WS}/install/setup.bash" ]; then
  source "${USER_WS}/install/setup.bash"
  log "Sourced ${USER_WS}/install/setup.bash"
else
  log "Workspace overlay not found at ${USER_WS}/install/setup.bash (this can be normal on first run)"
fi
set -u

launch_cmd=(
  ros2 launch ur_robotiq_moveit_config ur_robotiq_isaac_moveit.launch.py
  planning_pipeline:=cumotion
  launch_move_group:=true
  launch_cumotion_planner:=true
  cumotion_use_patched_node:=true
  collision_cache_cuboid:=200
  collision_cache_mesh:=100
  cumotion_override_moveit_scaling_factors:=true
  cumotion_time_dilation_factor:=0.25
  cumotion_max_attempts:=48
  cumotion_num_graph_seeds:=24
  cumotion_num_trajopt_seeds:=24
  cumotion_num_trajopt_time_steps:=64
  cumotion_trajopt_finetune_iters:=600
  cumotion_interpolation_dt:=0.02
  cumotion_pose_ik_retries:=2
  cumotion_reject_goal_while_busy:=true
  enable_robot_arm_beam:=false
  enable_joint_state_filter:=true
  moveit_joint_states_topic:=/moveit_joint_states
  "$@"
)

log "Launching MoveIt + cuMotion."
printf '[start_cumotion_planner][%s] Command: ' "$(timestamp)"
printf '%q ' "${launch_cmd[@]}"
echo

exec "${launch_cmd[@]}"
