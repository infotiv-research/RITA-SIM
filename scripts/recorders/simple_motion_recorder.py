#!/usr/bin/env python3
"""Reusable metrics recording and CSV helpers for simple-motion runs."""

from __future__ import annotations

import csv
import json
import subprocess
import sys
import time
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path

from recorders.planner_resource_recorder import (
    PlannerResourceRecorder,
    start_planner_resource_recorder,
    stop_planner_resource_recorder,
)


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
SIMPLE_MOTION_PHASE = "movement"
SIMPLE_MOTION_PHASES = (SIMPLE_MOTION_PHASE,)
METRICS_EVENT_TOPIC = "/simple_motion/metrics_events"
JOINT_STATES_TOPIC = "/joint_states"
WORLD_FRAME = "world"
TCP_FRAME = "TCP_point"
ROTATIONAL_JOINTS = (
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "elbow_joint",
    "wrist_1_joint",
    "wrist_2_joint",
    "wrist_3_joint",
)
LINEAR_JOINTS = ("gantry_joint",)


@dataclass
class ActiveSimpleMotionRunRecording:
    run_label: str
    trajectory_stop_file: str
    joint_stop_file: str
    trajectory_recorder: subprocess.Popen
    joint_movement_recorder: subprocess.Popen
    resource_recorder: PlannerResourceRecorder


class SimpleMotionRecorder:
    def __init__(
        self,
        planner: str,
        planner_label: str,
        case_name: str,
        num_runs: int,
        data_dir: Path | None = None,
        csv_file: Path | None = None,
    ):
        self.planner = planner
        self.planner_label = planner_label
        self.case_name = case_name
        self.num_runs = num_runs
        self.data_dir = data_dir or ROOT / "test_data"
        self.csv_file = self.resolve_csv_file(csv_file)

    def start_run(self, run_label: str) -> ActiveSimpleMotionRunRecording:
        trajectory_stop_file = f"/tmp/simple_motion_trajectory_metrics_{run_label}.stop"
        joint_stop_file = f"/tmp/simple_motion_joint_movement_{run_label}.stop"
        trajectory_recorder = start_trajectory_metrics_recorder(trajectory_stop_file)
        joint_movement_recorder = start_joint_movement_recorder(joint_stop_file)
        time.sleep(0.5)
        resource_recorder = start_planner_resource_recorder(
            self.planner, DOCKER_COMPOSE, ROOT
        )
        return ActiveSimpleMotionRunRecording(
            run_label=run_label,
            trajectory_stop_file=trajectory_stop_file,
            joint_stop_file=joint_stop_file,
            trajectory_recorder=trajectory_recorder,
            joint_movement_recorder=joint_movement_recorder,
            resource_recorder=resource_recorder,
        )

    def finish_run(
        self,
        recording: ActiveSimpleMotionRunRecording,
        total_time: float,
        returncode: int,
    ) -> tuple[bool, dict]:
        resource_usage = stop_planner_resource_recorder(recording.resource_recorder)
        trajectory_output = stop_ros_json_recorder(
            recording.trajectory_stop_file,
            recording.trajectory_recorder,
        )
        joint_output = stop_ros_json_recorder(
            recording.joint_stop_file,
            recording.joint_movement_recorder,
        )

        metrics = read_metrics(trajectory_output)
        metrics["joint_movement"] = read_metrics(joint_output)
        metrics["resource_usage"] = resource_usage

        timed_out = returncode == RUN_TIMEOUT_EXIT_CODE
        if timed_out:
            mark_timeout(metrics)
        elif returncode != 0 or metrics.get("success") is not True:
            mark_failed_run(metrics)

        run_success = returncode == 0 and metrics.get("success") is True
        return run_success, metrics

    def append_run(
        self,
        run_number: int,
        total_time: float,
        metrics: dict,
        success: bool,
    ) -> None:
        append_csv_row(
            self.csv_file,
            self.case_name,
            self.planner_label,
            run_number,
            total_time,
            metrics,
            success,
        )

    def new_csv_file(self) -> Path:
        self.data_dir.mkdir(exist_ok=True)
        date = datetime.now().strftime("%Y%m%d")
        csv_file = self.data_dir / (
            f"simple_motion_{self.planner_label}_{self.case_name}_{self.num_runs}_{date}.csv"
        )

        number = 1
        while csv_file.exists():
            csv_file = self.data_dir / (
                f"simple_motion_{self.planner_label}_{self.case_name}_{self.num_runs}_{date}_{number}.csv"
            )
            number += 1

        return csv_file

    def resolve_csv_file(self, csv_file: Path | None) -> Path:
        if csv_file is None:
            return self.new_csv_file()

        csv_file.parent.mkdir(parents=True, exist_ok=True)
        return csv_file


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
    return {"segments": {}, "totals": {}, "success": False}


def start_ros_json_recorder(
    stop_file: str,
    script_path: str,
    *args: str,
) -> subprocess.Popen:
    command_args = " ".join(args)
    recorder_command = (
        f"rm -f {stop_file} && "
        "cd /ros2_ws && "
        "source /opt/ros/${ROS_DISTRO:-jazzy}/setup.bash && "
        "(source install/local_setup.bash 2>/dev/null || true) && "
        f"python3 {script_path} {stop_file}"
    )
    if command_args:
        recorder_command += f" {command_args}"
    return subprocess.Popen(
        [*DOCKER_COMPOSE, "exec", "-T", "cumotion", "bash", "-lc", recorder_command],
        cwd=ROOT,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
    )


def stop_ros_json_recorder(stop_file: str, recorder: subprocess.Popen) -> str:
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


def start_trajectory_metrics_recorder(stop_file: str) -> subprocess.Popen:
    return start_ros_json_recorder(
        stop_file,
        "scripts/recorders/trajectory_metrics_recorder.py",
        METRICS_EVENT_TOPIC,
        WORLD_FRAME,
        TCP_FRAME,
        "movement_start",
        "movement_end",
        "phase",
        SIMPLE_MOTION_PHASE,
    )


def start_joint_movement_recorder(stop_file: str) -> subprocess.Popen:
    return start_ros_json_recorder(
        stop_file,
        "scripts/recorders/joint_movement_recorder.py",
        METRICS_EVENT_TOPIC,
        JOINT_STATES_TOPIC,
        ",".join(ROTATIONAL_JOINTS),
        ",".join(LINEAR_JOINTS),
        "movement_start",
        "movement_end",
        "phase",
        SIMPLE_MOTION_PHASE,
    )


def format_float(value, digits: int) -> str:
    if value is None:
        return ""
    return f"{float(value):.{digits}f}"


def csv_fieldnames() -> list[str]:
    fields = [
        "case_name",
        "planner",
        "num_runs",
        "total_time_s",
        "planning_time_s",
        "execution_time_s",
    ]
    for phase in SIMPLE_MOTION_PHASES:
        fields.extend(
            [
                f"{phase}_planning_time_s",
                f"{phase}_execution_time_s",
                f"{phase}_tcp_movement_cm",
                f"{phase}_min_collision_clearance_m",
                f"{phase}_collision_distance_status",
                f"{phase}_success",
            ]
        )
        for joint in ROTATIONAL_JOINTS:
            fields.append(f"{phase}_{joint}_movement_deg")
        for joint in LINEAR_JOINTS:
            fields.append(f"{phase}_{joint}_movement_m")
    fields.extend(
        [
            "planner_ram_memory_min_mib",
            "planner_ram_memory_avg_mib",
            "planner_ram_memory_max_mib",
            "planner_gpu_memory_min_mib",
            "planner_gpu_memory_avg_mib",
            "planner_gpu_memory_max_mib",
            "min_collision_clearance_m",
            "collision_distance_status",
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
    )
    return fields


def mark_timeout(metrics: dict) -> None:
    if not metrics.get("failure_reason"):
        metrics["failure_reason"] = "simple_motion_run_timeout"
    metrics["success"] = False


def mark_failed_run(metrics: dict) -> None:
    if not metrics.get("failure_reason"):
        metrics["failure_reason"] = "simple_motion_run_failed"
    metrics["success"] = False


def nested_metric(summary: dict, key: str):
    if not isinstance(summary, dict):
        return None
    return summary.get(key)


def metadata_value(metrics: dict, movement: dict, key: str):
    value = metrics.get(key)
    if value is None or value == "":
        value = movement.get(key)
    return value


def csv_row(
    case_name: str,
    planner: str,
    run_number: int,
    total_time: float,
    metrics: dict,
    success: bool,
) -> dict:
    totals = metrics.get("totals", {})
    segments = metrics.get("segments", {})
    movement = segments.get(SIMPLE_MOTION_PHASE, metrics.get("movement", {}))
    joint_segments = metrics.get("joint_movement", {}).get("segments", {})
    movement_joints = joint_segments.get(SIMPLE_MOTION_PHASE, {}).get("joints", {})
    resource_usage = metrics.get("resource_usage", {})
    ram_memory = resource_usage.get("planner_ram_memory_mib", {})
    gpu_memory = resource_usage.get("planner_gpu_memory_mib", {})

    row = {
        "case_name": case_name,
        "planner": planner,
        "num_runs": str(run_number),
        "total_time_s": format_float(total_time, 3),
        "planning_time_s": format_float(
            totals.get("planning_time_s", movement.get("planning_time_s")), 3
        ),
        "execution_time_s": format_float(
            totals.get("execution_time_s", movement.get("execution_time_s")), 3
        ),
        "movement_planning_time_s": format_float(
            movement.get("planning_time_s"), 3
        ),
        "movement_execution_time_s": format_float(
            movement.get("execution_time_s"), 3
        ),
        "movement_tcp_movement_cm": format_float(
            movement.get("tcp_movement_cm"), 2
        ),
        "movement_min_collision_clearance_m": format_float(
            movement.get("min_collision_clearance_m"), 4
        ),
        "movement_collision_distance_status": str(
            movement.get("collision_distance_status") or ""
        ),
        "movement_success": (
            str(bool(movement["success"])).lower()
            if "success" in movement
            else ""
        ),
        "planner_ram_memory_min_mib": format_float(nested_metric(ram_memory, "min"), 1),
        "planner_ram_memory_avg_mib": format_float(nested_metric(ram_memory, "avg"), 1),
        "planner_ram_memory_max_mib": format_float(nested_metric(ram_memory, "max"), 1),
        "planner_gpu_memory_min_mib": format_float(nested_metric(gpu_memory, "min"), 1),
        "planner_gpu_memory_avg_mib": format_float(nested_metric(gpu_memory, "avg"), 1),
        "planner_gpu_memory_max_mib": format_float(nested_metric(gpu_memory, "max"), 1),
        "min_collision_clearance_m": format_float(
            totals.get("min_collision_clearance_m", movement.get("min_collision_clearance_m")),
            4,
        ),
        "collision_distance_status": str(
            totals.get("collision_distance_status")
            or movement.get("collision_distance_status")
            or ""
        ),
        "success": str(bool(success)).lower(),
        "failure_reason": str(metadata_value(metrics, movement, "failure_reason") or ""),
        "failure_detail": str(metadata_value(metrics, movement, "failure_detail") or ""),
        "moveit_error_code": "" if metadata_value(metrics, movement, "moveit_error_code") is None else str(metadata_value(metrics, movement, "moveit_error_code")),
        "moveit_error_name": str(metadata_value(metrics, movement, "moveit_error_name") or ""),
        "action_status_code": "" if metadata_value(metrics, movement, "action_status_code") is None else str(metadata_value(metrics, movement, "action_status_code")),
        "action_status_name": str(metadata_value(metrics, movement, "action_status_name") or ""),
        "backend_name": str(metadata_value(metrics, movement, "backend_name") or ""),
        "backend_failure_reason": str(metadata_value(metrics, movement, "backend_failure_reason") or ""),
        "backend_failure_detail": str(metadata_value(metrics, movement, "backend_failure_detail") or ""),
        "backend_error_code": "" if metadata_value(metrics, movement, "backend_error_code") is None else str(metadata_value(metrics, movement, "backend_error_code")),
        "backend_status_name": str(metadata_value(metrics, movement, "backend_status_name") or ""),
    }

    for joint in ROTATIONAL_JOINTS:
        row[f"movement_{joint}_movement_deg"] = format_float(
            movement_joints.get(joint, {}).get("movement_deg"), 2
        )
    for joint in LINEAR_JOINTS:
        row[f"movement_{joint}_movement_m"] = format_float(
            movement_joints.get(joint, {}).get("movement_m"), 4
        )

    return row


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
