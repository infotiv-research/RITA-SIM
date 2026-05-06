#!/usr/bin/env python3
"""Reusable metrics recording and CSV helpers for simple-motion runs."""

from __future__ import annotations

import csv
import json
import subprocess
import sys
from pathlib import Path


ROOT = Path(__file__).resolve().parents[2]

DOCKER_COMPOSE = [
    "docker",
    "compose",
    "-p",
    "ros_stack",
    "-f",
    str(ROOT / "setup/docker-compose.ros2.yaml"),
    "-f",
    str(ROOT / "setup/docker-compose.cumotion.yaml"),
    "-f",
    str(ROOT / "setup/docker-compose.isaac.yaml"),
]

RUN_TIMEOUT_EXIT_CODE = 124


def read_metrics(recorder_output: str) -> dict:
    for line in reversed(recorder_output.strip().splitlines()):
        try:
            return json.loads(line)
        except json.JSONDecodeError:
            continue
    if recorder_output.strip():
        print(f"failed to parse recorder output: {recorder_output}", file=sys.stderr)
    else:
        print("recorder produced no output", file=sys.stderr)
    return {"movement": {}, "totals": {}, "success": False}


def start_metrics_recorder(stop_file: str) -> subprocess.Popen:
    recorder_command = (
        f"rm -f {stop_file} && "
        "cd /ros2_ws && "
        "source /opt/ros/${ROS_DISTRO:-jazzy}/setup.bash && "
        "(source install/local_setup.bash 2>/dev/null || true) && "
        "python3 scripts/recorders/simple_motion_metrics_recorder.py "
        f"{stop_file}"
    )
    return subprocess.Popen(
        [*DOCKER_COMPOSE, "exec", "-T", "cumotion", "bash", "-lc", recorder_command],
        cwd=ROOT,
        stdout=subprocess.PIPE,
        stderr=subprocess.DEVNULL,
        text=True,
    )


def stop_metrics_recorder(stop_file: str, recorder: subprocess.Popen) -> str:
    subprocess.run(
        [*DOCKER_COMPOSE, "exec", "-T", "cumotion", "touch", stop_file],
        cwd=ROOT,
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
    )
    try:
        recorder_output, _ = recorder.communicate(timeout=10.0)
    except subprocess.TimeoutExpired:
        recorder.kill()
        recorder_output, _ = recorder.communicate(timeout=5.0)
    return recorder_output


def format_float(value, digits: int) -> str:
    if value is None:
        return ""
    return f"{float(value):.{digits}f}"


def csv_fieldnames() -> list[str]:
    return [
        "case_name",
        "planner",
        "num_runs",
        "total_time_s",
        "planning_time_s",
        "execution_time_s",
        "tcp_movement_cm",
        "success",
        "failure_reason",
        "failure_detail",
        "moveit_error_code",
        "moveit_error_name",
        "action_status_code",
        "action_status_name",
        "backend_name",
        "backend_failure_reason",
        "backend_failure_detail",
        "backend_error_code",
        "backend_status_name",
    ]


def mark_timeout(metrics: dict) -> None:
    if not metrics.get("failure_reason"):
        metrics["failure_reason"] = "simple_motion_run_timeout"
    metrics["success"] = False


def mark_failed_run(metrics: dict) -> None:
    if not metrics.get("failure_reason"):
        metrics["failure_reason"] = "simple_motion_run_failed"
    metrics["success"] = False


def csv_row(
    case_name: str,
    planner: str,
    run_number: int,
    total_time: float,
    metrics: dict,
    success: bool,
) -> dict:
    totals = metrics.get("totals", {})
    return {
        "case_name": case_name,
        "planner": planner,
        "num_runs": str(run_number),
        "total_time_s": format_float(total_time, 3),
        "planning_time_s": format_float(totals.get("planning_time_s"), 3),
        "execution_time_s": format_float(totals.get("execution_time_s"), 3),
        "tcp_movement_cm": format_float(totals.get("tcp_movement_cm"), 2),
        "success": str(bool(success)).lower(),
        "failure_reason": str(metrics.get("failure_reason") or ""),
        "failure_detail": str(metrics.get("failure_detail") or ""),
        "moveit_error_code": "" if metrics.get("moveit_error_code") is None else str(metrics.get("moveit_error_code")),
        "moveit_error_name": str(metrics.get("moveit_error_name") or ""),
        "action_status_code": "" if metrics.get("action_status_code") is None else str(metrics.get("action_status_code")),
        "action_status_name": str(metrics.get("action_status_name") or ""),
        "backend_name": str(metrics.get("backend_name") or ""),
        "backend_failure_reason": str(metrics.get("backend_failure_reason") or ""),
        "backend_failure_detail": str(metrics.get("backend_failure_detail") or ""),
        "backend_error_code": "" if metrics.get("backend_error_code") is None else str(metrics.get("backend_error_code")),
        "backend_status_name": str(metrics.get("backend_status_name") or ""),
    }


def append_csv_row(
    csv_file: Path,
    case_name: str,
    planner: str,
    run_number: int,
    total_time: float,
    metrics: dict,
    success: bool,
) -> None:
    new_file = not csv_file.exists()
    with csv_file.open("a", newline="") as file:
        writer = csv.DictWriter(file, fieldnames=csv_fieldnames())
        if new_file:
            writer.writeheader()
        writer.writerow(
            csv_row(
                case_name,
                planner,
                run_number,
                total_time,
                metrics,
                success,
            )
        )
