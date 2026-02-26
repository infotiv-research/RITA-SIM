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

detect_gpu_vram_mb() {
  # Echoes "<total_mb>:<free_mb>" for GPU 0 when possible.
  local line total free py_total
  if command -v nvidia-smi >/dev/null 2>&1; then
    line="$(nvidia-smi --query-gpu=memory.total,memory.free --format=csv,noheader,nounits 2>/dev/null | head -n 1 || true)"
    if [ -n "${line}" ]; then
      total="$(echo "${line}" | cut -d',' -f1 | tr -dc '0-9')"
      free="$(echo "${line}" | cut -d',' -f2 | tr -dc '0-9')"
      if [ -n "${total}" ]; then
        echo "${total}:${free:-0}"
        return 0
      fi
    fi
  fi

  return 1
}

select_cumotion_profile() {
  local total_mb="$1"
  if [ "${total_mb}" -le 8192 ]; then
    echo "low"
  else
    echo "high"
  fi
}

set_profile_params() {
  local profile="$1"
  case "${profile}" in
    low)
      collision_cache_cuboid_value=100
      collision_cache_mesh_value=100
      cumotion_max_attempts_value=28
      cumotion_num_graph_seeds_value=14
      cumotion_num_trajopt_seeds_value=10
      cumotion_num_trajopt_time_steps_value=48
      cumotion_trajopt_finetune_iters_value=360
      cumotion_interpolation_dt_value=0.02
      cumotion_time_dilation_factor_value=0.30
      cumotion_pose_ik_retries_value=4
      ;;

    high)
      collision_cache_cuboid_value=200
      collision_cache_mesh_value=100
      cumotion_max_attempts_value=60
      cumotion_num_graph_seeds_value=24
      cumotion_num_trajopt_seeds_value=24
      cumotion_num_trajopt_time_steps_value=64
      cumotion_trajopt_finetune_iters_value=600
      cumotion_interpolation_dt_value=0.02
      cumotion_time_dilation_factor_value=0.25
      cumotion_pose_ik_retries_value=4
      ;;
    *)
      log "WARNING: Unknown profile '${profile}', using low."
      set_profile_params "low"
      return
      ;;
  esac
}

log "Starting cuMotion planner launcher."
log "ROS_DISTRO=${ROS_DISTRO} USER_WS=${USER_WS}"
log "TORCH_EXTENSIONS_DIR=${TORCH_EXTENSIONS_DIR}"

if [ "$#" -gt 0 ]; then
  log "User launch args: $*"
fi

mkdir -p "${TORCH_EXTENSIONS_DIR}"
export TORCH_EXTENSIONS_DIR
export PYTORCH_CUDA_ALLOC_CONF="${PYTORCH_CUDA_ALLOC_CONF:-expandable_segments:True,max_split_size_mb:64}"
log "PYTORCH_CUDA_ALLOC_CONF=${PYTORCH_CUDA_ALLOC_CONF}"

# Auto-pick a cuMotion config profile from detected VRAM.
detected_total_vram_mb=""
detected_free_vram_mb=""
selected_profile="low"
if vram_pair="$(detect_gpu_vram_mb)"; then
  detected_total_vram_mb="${vram_pair%%:*}"
  detected_free_vram_mb="${vram_pair##*:}"
fi

if [ -n "${detected_total_vram_mb}" ]; then
  selected_profile="$(select_cumotion_profile "${detected_total_vram_mb}")"
fi

if [ -n "${detected_total_vram_mb}" ] && [ -n "${detected_free_vram_mb}" ] && [ "${detected_total_vram_mb}" -gt 0 ] && [ "${detected_free_vram_mb}" -gt 0 ]; then
  free_pct=$((100 * detected_free_vram_mb / detected_total_vram_mb))
  # Downshift when most VRAM is already in use (common when Isaac Sim is loaded).
  if [ "${free_pct}" -lt 20 ] && [ "${selected_profile}" = "high" ]; then
    selected_profile="low"
  fi
fi

set_profile_params "${selected_profile}"
if [ -n "${detected_total_vram_mb}" ]; then
  log "Detected GPU0 VRAM total=${detected_total_vram_mb}MiB free=${detected_free_vram_mb}MiB -> cuMotion profile=${selected_profile}"
else
  log "Could not detect GPU VRAM -> using cuMotion profile=${selected_profile}"
fi

log "cuMotion params: cuboid=${collision_cache_cuboid_value} mesh=${collision_cache_mesh_value} max_attempts=${cumotion_max_attempts_value} graph_seeds=${cumotion_num_graph_seeds_value} trajopt_seeds=${cumotion_num_trajopt_seeds_value} time_steps=${cumotion_num_trajopt_time_steps_value} finetune_iters=${cumotion_trajopt_finetune_iters_value} interp_dt=${cumotion_interpolation_dt_value} time_dilation=${cumotion_time_dilation_factor_value} pose_ik_retries=${cumotion_pose_ik_retries_value}"

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
  collision_cache_cuboid:=${collision_cache_cuboid_value}
  collision_cache_mesh:=${collision_cache_mesh_value}
  cumotion_override_moveit_scaling_factors:=true
  cumotion_time_dilation_factor:=${cumotion_time_dilation_factor_value}
  cumotion_max_attempts:=${cumotion_max_attempts_value}
  cumotion_num_graph_seeds:=${cumotion_num_graph_seeds_value}
  cumotion_num_trajopt_seeds:=${cumotion_num_trajopt_seeds_value}
  cumotion_num_trajopt_time_steps:=${cumotion_num_trajopt_time_steps_value}
  cumotion_trajopt_finetune_iters:=${cumotion_trajopt_finetune_iters_value}
  cumotion_interpolation_dt:=${cumotion_interpolation_dt_value}
  cumotion_pose_ik_retries:=${cumotion_pose_ik_retries_value}
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
