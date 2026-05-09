#!/usr/bin/env python3
from __future__ import annotations

import argparse
from pathlib import Path
import os
import shutil
import subprocess
import time

from recorders.pick_and_place_recorder import (
    PickAndPlaceRecorder,
    RUN_TIMEOUT_EXIT_CODE,
)

SCRIPT_DIR = Path(__file__).resolve().parent
REPO_ROOT = SCRIPT_DIR.parent

TEST_SH = REPO_ROOT / "test.sh"
TEST_LOG_DIR = REPO_ROOT / "test_logs"


PICK_AND_PLACE_RUNS = 2
PICK_AND_PLACE_RUN_TIMEOUT_S = float(
    os.environ.get("PICK_AND_PLACE_RUN_TIMEOUT_S", "120")
)

PLANNER_LOGS = {
    "curobo": "curobo.log",
    "cumotion": "cumotion.log",
    "hybrid": "hybrid.log",
    "ompl": "ompl.log",
}


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Run automated pick-and-place scenarios.")
    parser.add_argument("planner", nargs="?", default="curobo", choices=sorted(PLANNER_LOGS))
    parser.add_argument("--runs", type=int, default=PICK_AND_PLACE_RUNS)
    parser.add_argument("--csv-file", type=Path)
    parser.add_argument("--log-dir", type=Path)
    return parser


def test_sh(*args: str) -> subprocess.CompletedProcess:
    return subprocess.run([TEST_SH, *args], cwd=REPO_ROOT)


def pick_and_place_args(planner: str) -> tuple[str, ...]:
    planner_args = {
        "curobo": ("motion_backend:=curobo_ros",),
        "cumotion": ("motion_backend:=moveit", "planning_pipeline:=cumotion"),
        "hybrid": ("motion_backend:=hybrid",),
        "ompl": ("motion_backend:=moveit", "planning_pipeline:=ompl"),
    }
    if planner not in planner_args:
        raise ValueError(f"Unknown planner: {planner}")
    return planner_args[planner]


def planner_log_file(planner: str) -> Path:
    return TEST_LOG_DIR / PLANNER_LOGS.get(planner, f"{planner}.log")


def file_position(path: Path) -> int:
    if not path.exists():
        return 0
    return path.stat().st_size


def planner_log_chunk(log_file: Path, start_position: int) -> str:
    if not log_file.exists():
        return ""

    with log_file.open(errors="replace") as file:
        file.seek(start_position)
        return file.read()


def append_timeout_planner_log(
    csv_file: Path,
    run_number: int,
    log_text: str,
    title: str = "TIMEOUT PLANNER LOG",
) -> None:
    failure_log_file = csv_file.with_name(f"{csv_file.stem}_failure.log")
    with failure_log_file.open("a") as file:
        file.write("\n\n")
        file.write("=" * 80 + "\n")
        file.write(f"RUN {run_number} {title}\n")
        file.write("=" * 80 + "\n")
        if log_text:
            file.write(log_text)
            if not log_text.endswith("\n"):
                file.write("\n")
        else:
            file.write("(no planner log lines captured for this run)\n")


def start_planner(planner: str) -> int:
    print(f"starting {planner}")
    if planner == "curobo":
        return test_sh(planner, "launch_rviz:=false").returncode
    return test_sh(planner).returncode


def prepare_pick_and_place_run(planner: str, run_number: int) -> int:
    print(f"resetting simulation and planner before pick_and_place run {run_number}")
    stop = test_sh("sim_headless", "stop")
    if stop.returncode != 0:
        return stop.returncode

    print("stopping ros2 and planner launch processes")
    kill = test_sh("kill")
    if kill.returncode != 0:
        return kill.returncode

    print("restarting ros2 before pick_and_place run")
    restart = test_sh("restart_ros")
    if restart.returncode != 0:
        return restart.returncode

    print("playing isaac sim")
    play = test_sh("sim_headless", "play")
    if play.returncode != 0:
        return play.returncode

    return start_planner(planner)


def archive_run_logs(args, run_number: int) -> None:
    if args.log_dir is None:
        return

    args.log_dir.mkdir(parents=True, exist_ok=True)
    log_sources = (
        (TEST_LOG_DIR / f"pick_and_place_run_{run_number}.log", f"pick_and_place_run_{run_number}.log"),
        (planner_log_file(args.planner), f"{args.planner}_{run_number}.log"),
        (TEST_LOG_DIR / "ros.log", f"ros_{run_number}.log"),
    )
    for source, destination_name in log_sources:
        if source.exists():
            shutil.copy2(source, args.log_dir / destination_name)


def pick_and_place_sequence(args) -> int:
    run_count = max(int(args.runs), 1)
    failures = 0
    try:
        recorder = PickAndPlaceRecorder(
            args.planner,
            run_count,
            csv_file=args.csv_file,
        )
        run_args = pick_and_place_args(args.planner)

        for run_number in range(1, run_count + 1):
            prepare_status = prepare_pick_and_place_run(args.planner, run_number)
            if prepare_status != 0:
                return prepare_status

            print(f"pick_and_place run {run_number}")
            recording = recorder.start_run(run_number)
            planner_log = planner_log_file(args.planner)
            planner_log_start = file_position(planner_log)
            start_time = time.monotonic()
            result = test_sh("pick_and_place_run", str(run_number), *run_args)
            total_time = time.monotonic() - start_time

            timed_out = (
                result.returncode == RUN_TIMEOUT_EXIT_CODE
                or total_time >= PICK_AND_PLACE_RUN_TIMEOUT_S
            )
            run_success = result.returncode == 0 and not timed_out
            metrics = recorder.finish_run(
                recording,
                total_time,
                success=run_success,
            )
            if not run_success:
                failures += 1
                planner_log_text = planner_log_chunk(planner_log, planner_log_start)
                title = (
                    "TIMEOUT PLANNER LOG"
                    if timed_out
                    else f"FAILED PLANNER LOG exit={result.returncode}"
                )
                append_timeout_planner_log(
                    recorder.csv_file,
                    run_number,
                    planner_log_text,
                    title=title,
                )
                failure_reason = metrics.get("failure_reason") or metrics.get("failed_phase") or ""
                print(
                    f"pick_and_place run {run_number} failed: "
                    f"exit={result.returncode}, timed_out={timed_out}, "
                    f"failure={failure_reason or 'unknown'}"
                )

            archive_run_logs(args, run_number)

            print("stopping isaac sim")
            test_sh("sim_headless", "stop")

        return 0 if failures == 0 else 1
    finally:
        test_sh("sim_headless", "stop")
        test_sh("kill")


def main() -> int:
    parser = build_arg_parser()
    args = parser.parse_args()
    return pick_and_place_sequence(args)


if __name__ == "__main__":
    raise SystemExit(main())
