#!/usr/bin/env python3
"""Reusable metrics recording and CSV helpers for pick-and-place runs."""

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
TRAJECTORY_PHASES = (
    "pre_grasp",
    "grasp",
    "post_grasp_lift",
    "pre_drop",
    "release",
    "post_release_retreat",
    "return_home",
)


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
    return {"segments": {}, "totals": {}}


def start_metrics_recorder(stop_file: str) -> subprocess.Popen:
    recorder_command = (
        f"rm -f {stop_file} && "
        "cd /ros2_ws && "
        "source /opt/ros/${ROS_DISTRO:-jazzy}/setup.bash && "
        "(source install/local_setup.bash 2>/dev/null || true) && "
        "python3 scripts/recorders/pick_and_place_metrics_recorder.py "
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


def first_incomplete_phase(metrics: dict) -> str:
    segments = metrics.get("segments", {})
    for phase in TRAJECTORY_PHASES:
        if segments.get(phase, {}).get("success") is not True:
            return phase
    return ""


def csv_fieldnames() -> list[str]:
    fields = [
        "num_runs",
        "total_time(s)",
        "total_planning_time_s",
        "total_execution_time_s",
        "total_tcp_movement_cm",
    ]
    for phase in TRAJECTORY_PHASES:
        fields.extend(
            [
                f"{phase}_planning_time_s",
                f"{phase}_execution_time_s",
                f"{phase}_tcp_movement_cm",
                f"{phase}_success",
            ]
        )
    return fields


def csv_row(run_number: int, total_time: float, metrics: dict, run_failed: bool) -> dict:
    totals = metrics.get("totals", {})
    segments = metrics.get("segments", {})
    failed_phase = metrics.get("failed_phase") or ""
    if run_failed and not failed_phase:
        failed_phase = first_incomplete_phase(metrics)
    row = {
        "num_runs": str(run_number),
        "total_time(s)": format_float(total_time, 3),
        "total_planning_time_s": format_float(totals.get("planning_time_s"), 3),
        "total_execution_time_s": format_float(totals.get("execution_time_s"), 3),
        "total_tcp_movement_cm": format_float(totals.get("tcp_movement_cm"), 2),
    }
    for phase in TRAJECTORY_PHASES:
        segment = segments.get(phase, {})
        row[f"{phase}_planning_time_s"] = format_float(
            segment.get("planning_time_s"), 3
        )
        row[f"{phase}_execution_time_s"] = format_float(
            segment.get("execution_time_s"), 3
        )
        row[f"{phase}_tcp_movement_cm"] = format_float(
            segment.get("tcp_movement_cm"), 2
        )
        if "success" in segment:
            row[f"{phase}_success"] = str(bool(segment["success"])).lower()
        elif phase == failed_phase:
            row[f"{phase}_success"] = "false"
        else:
            row[f"{phase}_success"] = ""
    return row


def mark_timeout(metrics: dict) -> None:
    if not metrics.get("failure_reason"):
        metrics["failure_reason"] = "pick_and_place_run_timeout"
    if not metrics.get("failed_phase"):
        metrics["failed_phase"] = first_incomplete_phase(metrics)
    metrics["success"] = False


def mark_failed_run(metrics: dict) -> None:
    if not metrics.get("failed_phase"):
        metrics["failed_phase"] = first_incomplete_phase(metrics)
    if not metrics.get("failure_reason"):
        metrics["failure_reason"] = "pick_and_place_run_failed"
    metrics["success"] = False


def append_csv_row(
    csv_file: Path, run_number: int, total_time: float, metrics: dict, success: bool
) -> None:
    new_file = not csv_file.exists()
    with csv_file.open("a", newline="") as file:
        writer = csv.DictWriter(file, fieldnames=csv_fieldnames())
        if new_file:
            writer.writeheader()
        writer.writerow(
            csv_row(run_number, total_time, metrics, run_failed=not success)
        )
