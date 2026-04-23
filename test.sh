#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="/home/research/isaac-sim-rita"

COMPOSE=(
  docker compose -p rita_stack
  -f "${SCRIPT_DIR}/setup/docker-compose.ros2.yaml"
  -f "${SCRIPT_DIR}/setup/docker-compose.cumotion.yaml"
  -f "${SCRIPT_DIR}/setup/docker-compose.isaac.yaml"
)

start() {
  "${COMPOSE[@]}" start ros2 cumotion isaacsim
  python3 "${SCRIPT_DIR}/scripts/test_stack.py" prepare_start
  "${COMPOSE[@]}" exec -d ros2 bash -lc 'cd /ros2_ws && ./control.sh ros > test_logs/ros.log 2>&1'
  "${COMPOSE[@]}" exec -d isaacsim bash -lc 'cd /ros2_ws && ./control.sh sim_headless > test_logs/isaac_headless.log 2>&1'
  python3 "${SCRIPT_DIR}/scripts/test_stack.py" wait_start
}

stop() {
  "${COMPOSE[@]}" exec -T ros2 bash -lc 'cd /ros2_ws && ./control.sh kill' || true
  "${COMPOSE[@]}" exec -T cumotion bash -lc 'cd /ros2_ws && ./control.sh kill' || true
  "${COMPOSE[@]}" stop ros2 cumotion isaacsim
}

sim_headless() {
  "${COMPOSE[@]}" exec -T isaacsim bash -lc 'cd /ros2_ws && ./control.sh sim_headless "$@"' bash "$@"
}

curobo() {
  python3 "${SCRIPT_DIR}/scripts/test_stack.py" prepare_logs
  "${COMPOSE[@]}" exec -d cumotion bash -lc 'cd /ros2_ws && ./control.sh curobo "$@" > test_logs/curobo.log 2>&1' bash "$@"
  python3 "${SCRIPT_DIR}/scripts/test_stack.py" wait_curobo
}

cumotion() {
  python3 "${SCRIPT_DIR}/scripts/test_stack.py" prepare_logs
  "${COMPOSE[@]}" exec -d cumotion bash -lc 'cd /ros2_ws && ./control.sh cumotion "$@" > test_logs/cumotion.log 2>&1' bash "$@"
  python3 "${SCRIPT_DIR}/scripts/test_stack.py" wait_cumotion
}

ompl() {
  python3 "${SCRIPT_DIR}/scripts/test_stack.py" prepare_logs
  "${COMPOSE[@]}" exec -d cumotion bash -lc 'cd /ros2_ws && ./control.sh ompl "$@" > test_logs/ompl.log 2>&1' bash "$@"
  python3 "${SCRIPT_DIR}/scripts/test_stack.py" wait_ompl
}

hybrid() {
  python3 "${SCRIPT_DIR}/scripts/test_stack.py" prepare_logs
  "${COMPOSE[@]}" exec -d cumotion bash -lc 'cd /ros2_ws && ./control.sh hybrid "$@" > test_logs/hybrid.log 2>&1' bash "$@"
  python3 "${SCRIPT_DIR}/scripts/test_stack.py" wait_hybrid
}

pick_and_place_run() {
  planner="${1:-curobo}"
  run_number="${2:-1}"
  python3 "${SCRIPT_DIR}/scripts/test_stack.py" prepare_logs
  if [[ "${planner}" == "curobo" ]]; then
    args="motion_backend:=curobo_ros"
  elif [[ "${planner}" == "cumotion" ]]; then
    args="motion_backend:=moveit planning_pipeline:=cumotion"
  elif [[ "${planner}" == "hybrid" ]]; then
    args="motion_backend:=hybrid"
  else
    args="motion_backend:=moveit planning_pipeline:=ompl"
  fi
  "${COMPOSE[@]}" exec -T cumotion bash -lc 'cd /ros2_ws && ./control.sh pick_and_place '"${args}" \
    > "${SCRIPT_DIR}/test_logs/pick_and_place_run_${run_number}.log" 2>&1
}

kill() {
  "${COMPOSE[@]}" exec -T cumotion bash -lc 'cd /ros2_ws && ./control.sh kill' || true
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
