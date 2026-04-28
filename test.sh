#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(pwd)"

COMPOSE_CMD="docker compose -p ros_stack \
  -f ${SCRIPT_DIR}/setup/docker-compose.ros2.yaml \
  -f ${SCRIPT_DIR}/setup/docker-compose.cumotion.yaml \
  -f ${SCRIPT_DIR}/setup/docker-compose.isaac.yaml"

build() {
  echo "---[ stopping test launch processes ]---"
  $COMPOSE_CMD exec -T ros2 bash -lc 'cd /ros2_ws && ./control.sh kill' || true
  $COMPOSE_CMD exec -T cumotion bash -lc 'cd /ros2_ws && ./control.sh kill' || true

  echo "---[ building workspace in ros2 container ]---"
  $COMPOSE_CMD exec -T ros2 bash -lc 'cd /ros2_ws && ./control.sh build'

  echo "---[ building cuMotion workspace packages in cumotion container ]---"
  $COMPOSE_CMD exec -T cumotion bash -lc '
    set -e
    cd /ros2_ws
    source /opt/ros/jazzy/setup.bash
    source /opt/vendor_ws/install/local_setup.bash 2>/dev/null || true
    source install/local_setup.bash 2>/dev/null || true
    pkgs="curobo_msgs curobo_ros curobo_rviz curobo_hybrid_planning_plugins ur_description robotiq_description ur_robotiq_curobo_config ur_robotiq_moveit_config ur_robotiq_servo"
    for pkg in $pkgs; do
      rm -rf build/$pkg install/$pkg log/latest_build/$pkg log/latest/$pkg
    done
    colcon build --symlink-install --packages-select $pkgs
  '

  echo "---[ build complete ]---"
}

start() {
  $COMPOSE_CMD up -d --build
  $COMPOSE_CMD start ros2 cumotion isaacsim
  python3 "${SCRIPT_DIR}/scripts/test_stack.py" prepare_start
  $COMPOSE_CMD exec -d ros2 bash -lc 'cd /ros2_ws && ./control.sh ros > test_logs/ros.log 2>&1'
  $COMPOSE_CMD exec -d isaacsim bash -lc 'cd /ros2_ws && ./control.sh sim_headless > test_logs/isaac_headless.log 2>&1'
  python3 "${SCRIPT_DIR}/scripts/test_stack.py" wait_start
}

stop() {
  $COMPOSE_CMD exec -T ros2 bash -lc 'cd /ros2_ws && ./control.sh kill' || true
  $COMPOSE_CMD exec -T cumotion bash -lc 'cd /ros2_ws && ./control.sh kill' || true
  $COMPOSE_CMD stop ros2 cumotion isaacsim
}

restart_ros() {
  python3 "${SCRIPT_DIR}/scripts/test_stack.py" prepare_logs
  $COMPOSE_CMD exec -T ros2 bash -lc 'cd /ros2_ws && ./control.sh kill' || true
  $COMPOSE_CMD exec -d ros2 bash -lc 'cd /ros2_ws && ./control.sh ros > test_logs/ros.log 2>&1'
  python3 "${SCRIPT_DIR}/scripts/test_stack.py" wait_ros
}

sim_headless() {
  $COMPOSE_CMD exec -T isaacsim bash -lc 'cd /ros2_ws && ./control.sh sim_headless "$@"' bash "$@"
}

curobo() {
  python3 "${SCRIPT_DIR}/scripts/test_stack.py" prepare_logs
  $COMPOSE_CMD exec -d cumotion bash -lc 'cd /ros2_ws && ./control.sh curobo "$@" > test_logs/curobo.log 2>&1' bash "$@"
  python3 "${SCRIPT_DIR}/scripts/test_stack.py" wait_curobo
}

cumotion() {
  python3 "${SCRIPT_DIR}/scripts/test_stack.py" prepare_logs
  $COMPOSE_CMD exec -d cumotion bash -lc 'cd /ros2_ws && ./control.sh cumotion "$@" > test_logs/cumotion.log 2>&1' bash "$@"
  python3 "${SCRIPT_DIR}/scripts/test_stack.py" wait_cumotion
}

ompl() {
  python3 "${SCRIPT_DIR}/scripts/test_stack.py" prepare_logs
  $COMPOSE_CMD exec -d cumotion bash -lc 'cd /ros2_ws && ./control.sh ompl "$@" > test_logs/ompl.log 2>&1' bash "$@"
  python3 "${SCRIPT_DIR}/scripts/test_stack.py" wait_ompl
}

hybrid() {
  python3 "${SCRIPT_DIR}/scripts/test_stack.py" prepare_logs
  $COMPOSE_CMD exec -d cumotion bash -lc 'cd /ros2_ws && ./control.sh hybrid "$@" > test_logs/hybrid.log 2>&1' bash "$@"
  python3 "${SCRIPT_DIR}/scripts/test_stack.py" wait_hybrid
}

hybrid_benchmark_run() {
  python3 "${SCRIPT_DIR}/scripts/test_stack.py" prepare_logs
  $COMPOSE_CMD exec -T cumotion bash -lc 'cd /ros2_ws && ./control.sh hybrid_benchmark "$@"' bash "$@" \
    > "${SCRIPT_DIR}/test_logs/hybrid_benchmark.log" 2>&1
}

pick_and_place_run() {
  run_number="${1:-1}"
  shift
  timeout_s="${PICK_AND_PLACE_RUN_TIMEOUT_S:-120}"
  python3 "${SCRIPT_DIR}/scripts/test_stack.py" prepare_logs
  $COMPOSE_CMD exec -T -e PICK_AND_PLACE_RUN_TIMEOUT_S="${timeout_s}" cumotion bash -lc '
    cd /ros2_ws
    timeout --kill-after=10s "${PICK_AND_PLACE_RUN_TIMEOUT_S}s" ./control.sh pick_and_place "$@"
    status=$?
    if [ "$status" -ne 0 ]; then
      pkill -f "pick_and_place.launch.py" 2>/dev/null || true
      pkill -f "pick_and_place_main.py" 2>/dev/null || true
    fi
    exit "$status"
  ' bash "$@" \
    > "${SCRIPT_DIR}/test_logs/pick_and_place_run_${run_number}.log" 2>&1
}

kill() {
  $COMPOSE_CMD exec -T cumotion bash -lc 'cd /ros2_ws && ./control.sh kill' || true
}

case "${1:-}" in
  build)
    build
    ;;
  start)
    start
    ;;
  stop)
    stop
    ;;
  restart_ros)
    restart_ros
    ;;
  sim_headless)
    shift
    sim_headless "$@"
    ;;
  curobo)
    shift
    curobo "$@"
    ;;
  cumotion)
    shift
    cumotion "$@"
    ;;
  ompl)
    shift
    ompl "$@"
    ;;
  hybrid)
    shift
    hybrid "$@"
    ;;
  hybrid_benchmark)
    shift
    python3 "${SCRIPT_DIR}/scripts/hybrid_benchmark_scenario.py" "$@"
    ;;
  hybrid_benchmark_run)
    shift
    hybrid_benchmark_run "$@"
    ;;
  pick_and_place)
    shift
    python3 "${SCRIPT_DIR}/scripts/pick_and_place_scenario.py" "$@"
    ;;
  pick_and_place_run)
    shift
    pick_and_place_run "$@"
    ;;
  kill)
    kill
    ;;
  *)
    echo "Usage: ./test.sh build | start | stop | restart_ros | sim_headless <play|stop> | curobo [args...] | cumotion [args...] | ompl [args...] | hybrid [args...] | hybrid_benchmark [--case test_1] [--runs 1] | pick_and_place <curobo|cumotion|ompl|hybrid> | kill" >&2
    exit 1
    ;;
esac
