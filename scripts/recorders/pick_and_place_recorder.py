#!/usr/bin/env python3
"""Reusable metrics recording and CSV helpers for pick-and-place runs."""

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
PICK_AND_PLACE_PHASES = (
    "pre_grasp",
    "grasp",
    "post_grasp_lift",
    "pre_drop",
    "release",
    "post_release_retreat",
    "return_home",
)
METRICS_EVENT_TOPIC = "/pick_and_place/metrics_events"
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
class ActivePickAndPlaceRunRecording:
    run_number: int
    metrics_stop_file: str
    joint_stop_file: str
    metrics_recorder: subprocess.Popen
    joint_movement_recorder: subprocess.Popen
    resource_recorder: PlannerResourceRecorder


class PickAndPlaceRecorder:
    def __init__(
        self,
        planner: str,
        num_runs: int,
        data_dir: Path | None = None,
        csv_file: Path | None = None,
    ):
        self.planner = planner
        self.num_runs = num_runs
        self.data_dir = data_dir or ROOT / "test_data"
        self.csv_file = self.resolve_csv_file(csv_file)

    def start_run(self, run_number: int) -> ActivePickAndPlaceRunRecording:
        metrics_stop_file = f"/tmp/trajectory_metrics_recorder_{run_number}.stop"
        joint_stop_file = f"/tmp/joint_movement_recorder_{run_number}.stop"
        metrics_recorder = start_metrics_recorder(metrics_stop_file)
        joint_movement_recorder = start_joint_movement_recorder(joint_stop_file)
        time.sleep(0.5)
        resource_recorder = start_planner_resource_recorder(
            self.planner, DOCKER_COMPOSE, ROOT
        )
        return ActivePickAndPlaceRunRecording(
            run_number=run_number,
            metrics_stop_file=metrics_stop_file,
            joint_stop_file=joint_stop_file,
            metrics_recorder=metrics_recorder,
            joint_movement_recorder=joint_movement_recorder,
            resource_recorder=resource_recorder,
        )

    def finish_run(
        self,
        recording: ActivePickAndPlaceRunRecording,
        total_time: float,
        success: bool,
    ) -> dict:
        resource_usage = stop_planner_resource_recorder(recording.resource_recorder)
        recorder_output = stop_metrics_recorder(
            recording.metrics_stop_file,
            recording.metrics_recorder,
        )
        joint_output = stop_joint_movement_recorder(
            recording.joint_stop_file,
            recording.joint_movement_recorder,
        )
        metrics = read_metrics(recorder_output)
        metrics["joint_movement"] = read_metrics(joint_output)
        metrics["resource_usage"] = resource_usage
        append_csv_row(
            self.csv_file,
            recording.run_number,
            total_time,
            metrics,
            success=success,
        )
        return metrics

    def new_csv_file(self) -> Path:
        self.data_dir.mkdir(exist_ok=True)
        date = datetime.now().strftime("%Y%m%d")
        csv_file = self.data_dir / (
            f"pick_and_place_{self.planner}_{self.num_runs}_{date}.csv"
        )

        number = 1
        while csv_file.exists():
            csv_file = self.data_dir / (
                f"pick_and_place_{self.planner}_{self.num_runs}_{date}_{number}.csv"
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
    return {"segments": {}, "totals": {}}


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


def start_metrics_recorder(stop_file: str) -> subprocess.Popen:
    return start_ros_json_recorder(
        stop_file,
        "scripts/recorders/trajectory_metrics_recorder.py",
        METRICS_EVENT_TOPIC,
        WORLD_FRAME,
        TCP_FRAME,
    )


def stop_metrics_recorder(stop_file: str, recorder: subprocess.Popen) -> str:
    return stop_ros_json_recorder(stop_file, recorder)


def start_joint_movement_recorder(stop_file: str) -> subprocess.Popen:
    return start_ros_json_recorder(
        stop_file,
        "scripts/recorders/joint_movement_recorder.py",
        METRICS_EVENT_TOPIC,
        JOINT_STATES_TOPIC,
        ",".join(ROTATIONAL_JOINTS),
        ",".join(LINEAR_JOINTS),
    )


def stop_joint_movement_recorder(stop_file: str, recorder: subprocess.Popen) -> str:
    return stop_ros_json_recorder(stop_file, recorder)


def format_float(value, digits: int) -> str:
    if value is None:
        return ""
    return f"{float(value):.{digits}f}"


def first_incomplete_phase(metrics: dict) -> str:
    segments = metrics.get("segments", {})
    for phase in PICK_AND_PLACE_PHASES:
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
        "total_min_collision_clearance_m",
        "total_collision_distance_status",
    ]
    for phase in PICK_AND_PLACE_PHASES:
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
        ]
    )
    return fields


def csv_row(run_number: int, total_time: float, metrics: dict, run_failed: bool) -> dict:
    totals = metrics.get("totals", {})
    segments = metrics.get("segments", {})
    joint_segments = metrics.get("joint_movement", {}).get("segments", {})
    ram_memory = metrics["resource_usage"]["planner_ram_memory_mib"]
    gpu_memory = metrics["resource_usage"]["planner_gpu_memory_mib"]
    failed_phase = metrics.get("failed_phase") or ""
    if run_failed and not failed_phase:
        failed_phase = first_incomplete_phase(metrics)
    row = {
        "num_runs": str(run_number),
        "total_time(s)": format_float(total_time, 3),
        "total_planning_time_s": format_float(totals.get("planning_time_s"), 3),
        "total_execution_time_s": format_float(totals.get("execution_time_s"), 3),
        "total_tcp_movement_cm": format_float(totals.get("tcp_movement_cm"), 2),
        "total_min_collision_clearance_m": format_float(
            totals.get("min_collision_clearance_m"), 4
        ),
        "total_collision_distance_status": str(
            totals.get("collision_distance_status") or ""
        ),
    }
    for phase in PICK_AND_PLACE_PHASES:
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
        row[f"{phase}_min_collision_clearance_m"] = format_float(
            segment.get("min_collision_clearance_m"), 4
        )
        row[f"{phase}_collision_distance_status"] = str(
            segment.get("collision_distance_status") or ""
        )
        if "success" in segment:
            row[f"{phase}_success"] = str(bool(segment["success"])).lower()
        elif phase == failed_phase:
            row[f"{phase}_success"] = "false"
        else:
            row[f"{phase}_success"] = ""

        phase_joints = joint_segments.get(phase, {}).get("joints", {})
        for joint in ROTATIONAL_JOINTS:
            row[f"{phase}_{joint}_movement_deg"] = format_float(
                phase_joints.get(joint, {}).get("movement_deg"), 2
            )
        for joint in LINEAR_JOINTS:
            row[f"{phase}_{joint}_movement_m"] = format_float(
                phase_joints.get(joint, {}).get("movement_m"), 4
            )
    row["planner_ram_memory_min_mib"] = format_float(ram_memory["min"], 1)
    row["planner_ram_memory_avg_mib"] = format_float(ram_memory["avg"], 1)
    row["planner_ram_memory_max_mib"] = format_float(ram_memory["max"], 1)
    row["planner_gpu_memory_min_mib"] = format_float(gpu_memory["min"], 1)
    row["planner_gpu_memory_avg_mib"] = format_float(gpu_memory["avg"], 1)
    row["planner_gpu_memory_max_mib"] = format_float(gpu_memory["max"], 1)
    return row


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
