#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
LOG_DIR="${SCRIPT_DIR}/test_logs"
ROS_DISTRO="${ROS_DISTRO:-jazzy}"
CUROBO_READY_TIMEOUT_SECONDS=0
ROS_LAUNCH_PATTERN='[u]r_robotiq_isaac_control.launch.py'
ISAAC_HEADLESS_PATTERN='[s]tart_isaac_main_scene.py'
CUROBO_PATTERN='[u]r_robotiq_curobo_human.launch.py'
CUROBO_RVIZ_PATTERN='[r]viz2 .*curobo_minimal\.rviz'
CUROBO_READY_LOG_PATTERN='cuRobo fully ready: warmup complete and all collisions are loaded.'
AUTO_CLEANUP_ENABLED=0
AUTO_CLEANUP_DONE=0
CUROBO_STREAM_PID=""
PICK_AND_PLACE_STARTED_CUROBO=0
PICK_AND_PLACE_STARTED_RVIZ=0

ROS_STACK_FILES=(
  "-f" "${SCRIPT_DIR}/setup/docker-compose.ros2.yaml"
  "-f" "${SCRIPT_DIR}/setup/docker-compose.cumotion.yaml"
)
ISAAC_STACK_FILES=(
  "-f" "${SCRIPT_DIR}/setup/docker-compose.isaac.yaml"
)

usage() {
  cat <<'EOF'
Usage:
  ./test.sh start
  ./test.sh stop
  ./test.sh pick_and_place curobo

Commands:
  start                     Start ros2, cumotion, and isaacsim containers, then launch the ROS control stack and Isaac Sim headless
  stop                      Stop launched processes, then close all three containers
  pick_and_place curobo     Use the already-running stack: play Isaac, run curobo + RViz + pick-and-place, then stop only those
EOF
}


ros_compose() {
  docker compose "${ROS_STACK_FILES[@]}" "$@"
}

isaac_compose() {
  docker compose "-f" "${SCRIPT_DIR}/setup/docker-compose.isaac.yaml" 
}

ros_service_running() {
  ros_compose ps --status running --services 2>/dev/null | grep -qx "$1"
}

ros_workspace_ready() {
  ros_compose exec -T ros2 bash -lc \
    "source /opt/ros/${ROS_DISTRO}/setup.bash >/dev/null 2>&1; \
    source /opt/vendor_ws/install/setup.bash >/dev/null 2>&1 || true; \
    source /ros2_ws/install/local_setup.bash >/dev/null 2>&1 || true; \
    ros2 pkg prefix ur_robotiq_description >/dev/null 2>&1"
}

ensure_ros_sources() {
  if ! ros_service_running "ros2"; then
    echo "Error: ros2 container is not running." >&2
    exit 1
  fi

  if ros_compose exec -T ros2 bash -lc "test -d /ros2_ws/src/serial/.git"; then
    return
  fi

  echo "Fetching missing serial package source..."
  ros_compose exec -T ros2 bash -lc \
    "git clone -b ros2 https://github.com/tylerjw/serial.git /ros2_ws/src/serial"
}

isaac_service_running() {
  isaac_compose ps --status running --services 2>/dev/null | grep -qx "$1"
}

isaac_headless_running() {
  isaac_service_running "isaacsim" && \
    isaac_compose exec -T isaacsim bash -lc "pgrep -f \"${ISAAC_HEADLESS_PATTERN}\" >/dev/null"
}

wait_until() {
  local timeout_seconds="$1"
  local description="$2"
  local check_function="$3"
  local waited_seconds=0

  while true; do
    if "${check_function}"; then
      echo "${description} is ready."
      return 0
    fi

    if (( timeout_seconds > 0 && waited_seconds >= timeout_seconds )); then
      echo "Error: timed out waiting for ${description}." >&2
      return 1
    fi

    sleep 2
    waited_seconds=$((waited_seconds + 2))

    if (( waited_seconds > 0 && waited_seconds % 30 == 0 )); then
      echo "Still waiting for ${description}... (${waited_seconds}s elapsed)"
    fi
  done
}

isaac_timeline_state_is() {
  local expected_state="$1"

  isaac_headless_running && \
    isaac_compose exec -T isaacsim bash -lc \
      "test -f /tmp/rita_isaac_timeline_state && grep -qx '${expected_state}' /tmp/rita_isaac_timeline_state"
}

ros_ready() {
  ros_service_running "ros2" && \
    ros_compose exec -T ros2 bash -lc \
      "pgrep -f \"${ROS_LAUNCH_PATTERN}\" >/dev/null && \
      source /opt/ros/${ROS_DISTRO}/setup.bash >/dev/null 2>&1; \
      source /ros2_ws/install/local_setup.bash >/dev/null 2>&1 || true; \
      ros2 action info /joint_trajectory_controller/follow_joint_trajectory 2>/dev/null | grep -Eq 'Action servers: +1' && \
      ros2 action info /robotiq_gripper_joint_trajectory_controller/follow_joint_trajectory 2>/dev/null | grep -Eq 'Action servers: +1'"
}



curobo_ready_message_seen() {
  [ -f "${LOG_DIR}/curobo.log" ] && grep -Fq "${CUROBO_READY_LOG_PATTERN}" "${LOG_DIR}/curobo.log"
}

wait_for_ros_ready() {
  echo "Waiting for ROS control action servers..."
  wait_until  120 "ROS control" ros_ready
}

wait_for_curobo_ready() {
  echo "Waiting for curobo ready message..."
  wait_until 350 "curobo ready message in log" curobo_ready_message_seen
}

start_containers() {
  mkdir -p "${LOG_DIR}"

  echo "Starting ros2 and cumotion containers..."
  ros_compose up -d ros2 cumotion

  echo "Starting isaacsim container..."
  isaac_compose up -d isaacsim
}

prepare_ros_workspace() {
  if ! ros_service_running "ros2"; then
    echo "Error: ros2 container is not running." >&2
    exit 1
  fi

  ensure_ros_sources

  if ros_workspace_ready; then
    echo "ROS workspace is ready."
    return
  fi

  : > "${LOG_DIR}/ros_build.log"
  echo "ROS workspace is not built yet. Building it now..."
  ros_compose exec -T ros2 bash -lc \
    "cd /ros2_ws && ./control.sh build" \
    2>&1 | tee -a "${LOG_DIR}/ros_build.log"
}

start_ros() {
  if ! ros_service_running "ros2"; then
    echo "Error: ros2 container is not running." >&2
    exit 1
  fi

  if ros_compose exec -T ros2 bash -lc "pgrep -f \"${ROS_LAUNCH_PATTERN}\" >/dev/null"; then
    echo "ROS control stack is already running."
    return
  fi

  echo "Starting ROS control stack..."
  ros_compose exec -d ros2 bash -lc \
    "cd /ros2_ws && mkdir -p test_logs && exec ./control.sh ros > test_logs/ros.log 2>&1"
}

start_isaac_headless() {
  if ! isaac_service_running "isaacsim"; then
    echo "Error: isaacsim container is not running." >&2
    exit 1
  fi

  if isaac_compose exec -T isaacsim bash -lc "pgrep -f \"${ISAAC_HEADLESS_PATTERN}\" >/dev/null"; then
    echo "Isaac Sim headless is already running."
    return
  fi

  echo "Starting Isaac Sim headless..."
  isaac_compose exec -d isaacsim bash -lc \
    "cd /ros2_ws && mkdir -p test_logs && exec /isaac-sim/python.sh /ros2_ws/scripts/start_isaac_main_scene.py --headless > test_logs/isaac_headless.log 2>&1"
}

start_curobo_backend() {
  if ! ros_service_running "cumotion"; then
    echo "Error: cumotion container is not running." >&2
    exit 1
  fi

  if ros_compose exec -T cumotion bash -lc "pgrep -f \"${CUROBO_PATTERN}\" >/dev/null"; then
    echo "curobo is already running."
    return
  fi

  : > "${LOG_DIR}/curobo.log"
  echo "Starting curobo in background. Logging to ${LOG_DIR}/curobo.log"
  ros_compose exec -T cumotion bash -lc \
    "cd /ros2_ws && ./control.sh curobo launch_rviz:=false" \
    > "${LOG_DIR}/curobo.log" 2>&1 &
  CUROBO_STREAM_PID="$!"
  PICK_AND_PLACE_STARTED_CUROBO=1
}

start_curobo_rviz() {
  if ! ros_service_running "cumotion"; then
    echo "Error: cumotion container is not running." >&2
    exit 1
  fi

  if ros_compose exec -T cumotion bash -lc "pgrep -f \"${CUROBO_RVIZ_PATTERN}\" >/dev/null"; then
    echo "RViz is already running."
    return
  fi

  if ! ros_compose exec -T cumotion bash -lc "test -n \"\${DISPLAY:-}\""; then
    echo "Error: DISPLAY is not set in the cumotion container, so RViz cannot be started." >&2
    exit 1
  fi

  echo "Starting RViz..."
  ros_compose exec -d cumotion bash -lc \
    "cd /ros2_ws && mkdir -p test_logs && \
    source /opt/ros/${ROS_DISTRO}/setup.bash >/dev/null 2>&1; \
    source /ros2_ws/install/local_setup.bash >/dev/null 2>&1 || true; \
    rviz_config=\"/ros2_ws/install/ur_robotiq_curobo_config/share/ur_robotiq_curobo_config/rviz/curobo_minimal.rviz\"; \
    if [ ! -f \"\${rviz_config}\" ]; then \
      rviz_config=\"/ros2_ws/src/ur_robotiq_curobo_config/rviz/curobo_minimal.rviz\"; \
    fi; \
    exec rviz2 -d \"\${rviz_config}\" > test_logs/curobo_rviz.log 2>&1"
  PICK_AND_PLACE_STARTED_RVIZ=1
}

run_pick_and_place_curobo() {
  if ! ros_service_running "cumotion"; then
    echo "Error: cumotion container is not running." >&2
    exit 1
  fi

  : > "${LOG_DIR}/pick_and_place.log"
  echo "Running pick_and_place with curobo. Live output follows..."
  ros_compose exec -T cumotion bash -lc \
    "cd /ros2_ws && \
    source /opt/ros/${ROS_DISTRO}/setup.bash >/dev/null 2>&1; \
    source /ros2_ws/install/local_setup.bash >/dev/null 2>&1 || true; \
    ./control.sh pick_and_place motion_backend:=curobo_ros" \
    2>&1 | tee -a "${LOG_DIR}/pick_and_place.log"
}

play_isaac_simulation() {
  if ! isaac_headless_running; then
    echo "Error: Isaac Sim headless is not running. Start it first with ./test.sh start or your manual workflow." >&2
    exit 1
  fi

  echo "Playing Isaac Sim timeline..."
  isaac_compose exec -T isaacsim bash -lc "printf 'play\n' > /tmp/rita_isaac_timeline_command"
  wait_until 60 "playing" "Isaac Sim timeline"
}

stop_isaac_timeline() {
  if ! isaac_headless_running; then
    return
  fi

  echo "Stopping Isaac Sim timeline..."
  isaac_compose exec -T isaacsim bash -lc "printf 'stop\n' > /tmp/rita_isaac_timeline_command"
  wait_until 60 "stopped" "Isaac Sim timeline stop" || true
}

stop_curobo_rviz() {
  if [ "${PICK_AND_PLACE_STARTED_RVIZ}" -ne 1 ]; then
    return
  fi

  if ! ros_service_running "cumotion"; then
    return
  fi

  echo "Closing RViz..."
  ros_compose exec -T cumotion bash -lc \
    "pkill -INT -f \"${CUROBO_RVIZ_PATTERN}\" 2>/dev/null || true; sleep 1; pkill -TERM -f \"${CUROBO_RVIZ_PATTERN}\" 2>/dev/null || true" || true
}

stop_curobo_backend() {
  if [ "${PICK_AND_PLACE_STARTED_CUROBO}" -ne 1 ]; then
    return
  fi

  if ! ros_service_running "cumotion"; then
    return
  fi

  echo "Closing curobo..."
  ros_compose exec -T cumotion bash -lc \
    "pkill -INT -f \"${CUROBO_PATTERN}\" 2>/dev/null || true; \
    sleep 2; \
    pkill -TERM -f \"${CUROBO_PATTERN}\" 2>/dev/null || true; \
    pkill -TERM -f 'curobo_trajectory_planner' 2>/dev/null || true; \
    pkill -TERM -f 'curobo_world_bridge.py' 2>/dev/null || true; \
    pkill -TERM -f 'curobo_human_skeleton_collision_publisher.py' 2>/dev/null || true; \
    pkill -TERM -f 'curobo_live_collision_spheres.py' 2>/dev/null || true; \
    pkill -TERM -f 'curobo_preview_joint_states.py' 2>/dev/null || true; \
    pkill -TERM -f 'joint_state_publisher' 2>/dev/null || true; \
    pkill -TERM -f 'static_transform_publisher .* preview/world' 2>/dev/null || true" || true
}

stop_ros() {
  if ! ros_service_running "ros2"; then
    return
  fi

  echo "Stopping ROS control stack and ROS launches..."
  ros_compose exec -T ros2 bash -lc "cd /ros2_ws && ./control.sh kill" || true
}

stop_cumotion_launches() {
  if ! ros_service_running "cumotion"; then
    return
  fi

  echo "Stopping cumotion launches..."
  ros_compose exec -T cumotion bash -lc "cd /ros2_ws && ./control.sh kill" || true
}

stop_isaac_headless() {
  if ! isaac_service_running "isaacsim"; then
    return
  fi

  if ! isaac_compose exec -T isaacsim bash -lc "pgrep -f \"${ISAAC_HEADLESS_PATTERN}\" >/dev/null"; then
    echo "Isaac Sim headless is not running."
    return
  fi

  echo "Stopping Isaac Sim headless..."
  isaac_compose exec -T isaacsim bash -lc \
    "pkill -INT -f \"${ISAAC_HEADLESS_PATTERN}\" 2>/dev/null || true; \
    for _ in {1..10}; do \
      if ! pgrep -f \"${ISAAC_HEADLESS_PATTERN}\" >/dev/null; then \
        exit 0; \
      fi; \
      sleep 1; \
    done; \
    pkill -TERM -f \"${ISAAC_HEADLESS_PATTERN}\" 2>/dev/null || true"
}

stop_containers() {
  echo "Closing isaacsim container..."
  isaac_compose down || true

  echo "Closing ros2 and cumotion containers..."
  ros_compose down || true
}

stop_background_streams() {
  if [ -n "${CUROBO_STREAM_PID}" ]; then
    wait "${CUROBO_STREAM_PID}" 2>/dev/null || true
    CUROBO_STREAM_PID=""
  fi
}

cleanup_pick_and_place_only() {
  stop_isaac_timeline
  stop_curobo_rviz
  stop_curobo_backend
  stop_background_streams
}

cleanup_pick_and_place() {
  local exit_code="$?"

  if [ "${AUTO_CLEANUP_ENABLED}" -ne 1 ] || [ "${AUTO_CLEANUP_DONE}" -eq 1 ]; then
    return
  fi

  AUTO_CLEANUP_DONE=1
  trap - EXIT INT TERM
  cleanup_pick_and_place_only || true
  exit "${exit_code}"
}

start_all() {
  start_containers
  prepare_ros_workspace
  start_ros
  sleep 3
  start_isaac_headless
  echo "Start sequence submitted."
  echo "Logs: ${LOG_DIR}/ros_build.log, ${LOG_DIR}/ros.log, and ${LOG_DIR}/isaac_headless.log"
}

pick_and_place_curobo_workflow() {
  local run_index total_runs=1

  AUTO_CLEANUP_ENABLED=1
  AUTO_CLEANUP_DONE=0
  PICK_AND_PLACE_STARTED_CUROBO=0
  PICK_AND_PLACE_STARTED_RVIZ=0
  trap cleanup_pick_and_place EXIT INT TERM

  play_isaac_simulation
  wait_for_ros_ready
  start_curobo_backend
  wait_for_curobo_ready
  start_curobo_rviz
  sleep 3

  echo "Logs: ${LOG_DIR}/ros.log, ${LOG_DIR}/isaac_headless.log, ${LOG_DIR}/curobo.log, ${LOG_DIR}/curobo_rviz.log, and ${LOG_DIR}/pick_and_place.log"
  for ((run_index = 1; run_index <= total_runs; run_index++)); do
    echo "Pick and place run ${run_index}/${total_runs}"
    run_pick_and_place_curobo

    if (( run_index < total_runs )); then
      stop_isaac_timeline
      play_isaac_simulation
    fi
  done
}

stop_all() {
  stop_isaac_headless
  stop_ros
  stop_cumotion_launches
  stop_containers
  stop_background_streams
  echo "Stop sequence completed."
}

main() {
  case "$1" in
    start)
      start_all
      ;;
    stop)
      stop_all
      ;;
    pick_and_place)
      case "$2" in
        curobo)
          pick_and_place_curobo_workflow
          ;;
        *)
          echo "Error: unsupported pick_and_place backend '$2'. Supported: curobo." >&2
          exit 1
          ;;
      esac
      ;;
    *)
      usage
      exit 1
      ;;
  esac
}

main "$@"
