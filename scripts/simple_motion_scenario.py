#!/usr/bin/env python3
from __future__ import annotations

import argparse
import os
import shutil
import subprocess
import sys
import time
from datetime import datetime
from pathlib import Path

from recorders.simple_motion_recorder import (
    RUN_TIMEOUT_EXIT_CODE,
    append_csv_row,
    mark_failed_run,
    mark_timeout,
    read_metrics,
    start_metrics_recorder,
    stop_metrics_recorder,
)


SCRIPT_DIR = Path(__file__).resolve().parent
REPO_ROOT = SCRIPT_DIR.parent
TEST_SH = REPO_ROOT / "test.sh"
TEST_DATA_DIR = REPO_ROOT / "test_data"
TEST_LOG_DIR = REPO_ROOT / "test_logs"

PLANNERS = ("curobo", "cumotion", "ompl", "hybrid")
OMPL_PLANNER_ALIASES = {
    "rrtstar": ("rrtStar", "RRTstarkConfigDefault"),
    "rrtconnect": ("rrtConnect", "RRTConnectkConfigDefault"),
}
SETUP_FAILURE_REASONS = {
    "failed_to_reach_start_state",
    "movement_not_recorded",
}
DEFAULT_OMPL_AFTER_PLAY_DELAY_SEC = 5.0


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Run generic simple-motion scenarios.")
    parser.add_argument("planner", choices=PLANNERS)
    parser.add_argument(
        "ompl_planner",
        nargs="?",
        help="Optional OMPL planner variant: rrtStar or rrtConnect.",
    )
    parser.add_argument("--case", default="test_1")
    parser.add_argument("--runs", type=int, default=1)
    parser.add_argument("--joint-tolerance", type=float, default=0.02)
    parser.add_argument("--csv-file", type=Path)
    parser.add_argument("--log-dir", type=Path)
    return parser


def test_sh(*args: str) -> subprocess.CompletedProcess:
    return subprocess.run([TEST_SH, *args], cwd=REPO_ROOT)


def normalize_ompl_planner(value: str | None) -> tuple[str, str] | None:
    if value is None:
        return None
    normalized = "".join(ch for ch in str(value).lower() if ch.isalnum())
    if normalized not in OMPL_PLANNER_ALIASES:
        valid = ", ".join(alias for alias, _ in OMPL_PLANNER_ALIASES.values())
        raise ValueError(f"Unknown OMPL planner '{value}'. Expected one of: {valid}")
    return OMPL_PLANNER_ALIASES[normalized]


def validate_args(args) -> None:
    if args.planner != "ompl" and args.ompl_planner is not None:
        raise ValueError("The optional planner variant is only supported after 'ompl'.")
    resolved = normalize_ompl_planner(args.ompl_planner)
    if resolved is None:
        args.ompl_planner_id = None
        return
    args.ompl_planner, args.ompl_planner_id = resolved


def planner_label(args) -> str:
    if args.planner != "ompl" or args.ompl_planner is None:
        return args.planner
    return f"{args.planner}_{args.ompl_planner}"


def new_csv_file(planner: str, case_name: str, runs: int) -> Path:
    TEST_DATA_DIR.mkdir(exist_ok=True)
    date = datetime.now().strftime("%Y%m%d")
    csv_file = TEST_DATA_DIR / f"simple_motion_{planner}_{case_name}_{runs}_{date}.csv"

    number = 1
    while csv_file.exists():
        csv_file = TEST_DATA_DIR / (
            f"simple_motion_{planner}_{case_name}_{runs}_{date}_{number}.csv"
        )
        number += 1

    return csv_file


def resolve_csv_file(args, run_count: int) -> Path:
    if args.csv_file is None:
        return new_csv_file(planner_label(args), args.case, run_count)

    args.csv_file.parent.mkdir(parents=True, exist_ok=True)
    return args.csv_file


def start_planner(planner: str) -> int:
    print(f"starting {planner}")
    if planner == "curobo":
        return test_sh(planner, "launch_rviz:=true").returncode
    return test_sh(planner).returncode


def delay_after_isaac_play(planner: str) -> None:
    if planner != "ompl":
        return

    delay_sec = float(
        os.environ.get(
            "SIMPLE_MOTION_OMPL_AFTER_PLAY_DELAY_SEC",
            DEFAULT_OMPL_AFTER_PLAY_DELAY_SEC,
        )
    )
    if delay_sec <= 0.0:
        return

    print(f"waiting {delay_sec:.1f}s after isaac sim play before OMPL planning")
    time.sleep(delay_sec)


def simple_motion_run_args(args) -> tuple[str, ...]:
    run_args = [
        "--planner",
        args.planner,
        "--case",
        args.case,
        "--joint-tolerance",
        str(args.joint_tolerance),
    ]
    if args.ompl_planner_id is not None:
        run_args.extend(["--ompl-planner-id", args.ompl_planner_id])
    return tuple(run_args)


def prepare_simple_motion_run(planner: str, run_label: str) -> int:
    print(f"resetting simulation and planner before simple_motion run {run_label}")
    stop = test_sh("sim_headless", "stop")
    if stop.returncode != 0:
        return stop.returncode

    print("stopping ros2 and planner launch processes")
    kill = test_sh("kill")
    if kill.returncode != 0:
        return kill.returncode

    print("restarting ros2 before simple_motion run")
    restart = test_sh("restart_ros")
    if restart.returncode != 0:
        return restart.returncode

    print("playing isaac sim")
    play = test_sh("sim_headless", "play")
    if play.returncode != 0:
        return play.returncode

    delay_after_isaac_play(planner)

    planner_status = start_planner(planner)
    if planner_status != 0:
        return planner_status

    return 0


def run_simple_motion_attempt(run_args: tuple[str, ...], run_label: str) -> tuple[bool, float, dict, int]:
    stop_file = f"/tmp/simple_motion_metrics_recorder_{run_label}.stop"
    recorder = start_metrics_recorder(stop_file)

    time.sleep(0.5)
    start_time = time.monotonic()
    result = test_sh("simple_motion_run", run_label, *run_args)
    total_time = time.monotonic() - start_time

    recorder_output = stop_metrics_recorder(stop_file, recorder)
    metrics = read_metrics(recorder_output)

    timed_out = result.returncode == RUN_TIMEOUT_EXIT_CODE
    if timed_out:
        mark_timeout(metrics)
    elif result.returncode != 0 or metrics.get("success") is not True:
        mark_failed_run(metrics)

    run_success = result.returncode == 0 and metrics.get("success") is True
    return run_success, total_time, metrics, result.returncode


def archive_attempt_logs(args, run_label: str) -> None:
    if args.log_dir is None:
        return

    args.log_dir.mkdir(parents=True, exist_ok=True)
    log_sources = (
        (TEST_LOG_DIR / f"simple_motion_run_{run_label}.log", f"simple_motion_run_{run_label}.log"),
        (TEST_LOG_DIR / f"{args.planner}.log", f"{args.planner}_{run_label}.log"),
        (TEST_LOG_DIR / "ros.log", f"ros_{run_label}.log"),
    )
    for source, destination_name in log_sources:
        if source.exists():
            shutil.copy2(source, args.log_dir / destination_name)


def should_retry_setup_failure(metrics: dict, returncode: int) -> bool:
    if returncode == 0:
        return False
    return str(metrics.get("failure_reason") or "") in SETUP_FAILURE_REASONS


def simple_motion_sequence(args) -> int:
    run_count = max(int(args.runs), 1)
    csv_file = resolve_csv_file(args, run_count)
    failures = 0

    try:
        run_args = simple_motion_run_args(args)
        for run_number in range(1, run_count + 1):
            prepare_status = prepare_simple_motion_run(args.planner, str(run_number))
            if prepare_status != 0:
                return prepare_status

            print(f"simple_motion run {run_number}")
            run_success, total_time, metrics, returncode = run_simple_motion_attempt(
                run_args,
                str(run_number),
            )
            archive_attempt_logs(args, str(run_number))

            if should_retry_setup_failure(metrics, returncode):
                print(
                    f"simple_motion run {run_number} failed before measured movement; "
                    "restarting once and retrying"
                )
                prepare_status = prepare_simple_motion_run(args.planner, f"{run_number}_retry_1")
                if prepare_status != 0:
                    return prepare_status
                run_success, total_time, metrics, returncode = run_simple_motion_attempt(
                    run_args,
                    f"{run_number}_retry_1",
                )
                archive_attempt_logs(args, f"{run_number}_retry_1")

            if not run_success:
                failures += 1

            append_csv_row(
                csv_file,
                args.case,
                planner_label(args),
                run_number,
                total_time,
                metrics,
                success=run_success,
            )

        return 0 if failures == 0 else 1
    finally:
        test_sh("sim_headless", "stop")
        test_sh("kill")


def main(argv: list[str]) -> int:
    parser = build_arg_parser()
    args = parser.parse_args(argv)
    try:
        validate_args(args)
    except ValueError as exc:
        parser.error(str(exc))
    return simple_motion_sequence(args)


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
