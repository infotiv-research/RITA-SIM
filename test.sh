#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(pwd)"

COMPOSE_CMD="docker compose -p rita_stack \
  -f ${SCRIPT_DIR}/setup/docker-compose.ros2.yaml \
  -f ${SCRIPT_DIR}/setup/docker-compose.cumotion.yaml \
  -f ${SCRIPT_DIR}/setup/docker-compose.isaac.yaml"


  
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

pick_and_place_run() {
  run_number="${1:-1}"
  shift
  python3 "${SCRIPT_DIR}/scripts/test_stack.py" prepare_logs
  $COMPOSE_CMD exec -T cumotion bash -lc 'cd /ros2_ws && ./control.sh pick_and_place "$@"' bash "$@" \
    > "${SCRIPT_DIR}/test_logs/pick_and_place_run_${run_number}.log" 2>&1
}

kill() {
  $COMPOSE_CMD exec -T cumotion bash -lc 'cd /ros2_ws && ./control.sh kill' || true
}

case "${1:-}" in
  start)
    start
    ;;
  stop)
    stop
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
    echo "Usage: ./test.sh start | stop | sim_headless <play|stop> | curobo [args...] | cumotion [args...] | ompl [args...] | hybrid [args...] | pick_and_place <curobo|cumotion|ompl|hybrid> | kill" >&2
    exit 1
    ;;
esac
