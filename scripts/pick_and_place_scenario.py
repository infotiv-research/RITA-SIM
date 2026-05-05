#!/usr/bin/env python3
from __future__ import annotations
from datetime import datetime
from pathlib import Path
import os
import subprocess
import sys
import time

from recorders.pick_and_place_recorder import (
    RUN_TIMEOUT_EXIT_CODE,
    append_csv_row,
    read_metrics,
    start_metrics_recorder,
    stop_metrics_recorder,
)

SCRIPT_DIR = Path(__file__).resolve().parent
REPO_ROOT = SCRIPT_DIR.parent

TEST_SH = REPO_ROOT / "test.sh"
TEST_DATA_DIR = REPO_ROOT / "test_data"
TEST_LOG_DIR = REPO_ROOT / "test_logs"


PICK_AND_PLACE_RUNS = 5
PICK_AND_PLACE_RUN_TIMEOUT_S = float(
    os.environ.get("PICK_AND_PLACE_RUN_TIMEOUT_S", "120")
)
PLANNER = sys.argv[1] if len(sys.argv) > 1 else "curobo"

PLANNER_LOGS = {
    "curobo": "curobo.log",
    "cumotion": "cumotion.log",
    "hybrid": "hybrid.log",
    "ompl": "ompl.log",
}


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


def new_csv_file() -> Path:
    TEST_DATA_DIR.mkdir(exist_ok=True)
    date = datetime.now().strftime("%Y%m%d")
    csv_file = TEST_DATA_DIR / f"pick_and_place_{PLANNER}_{PICK_AND_PLACE_RUNS}_{date}.csv"

    number = 1
    while csv_file.exists():
        csv_file = TEST_DATA_DIR / (
            f"pick_and_place_{PLANNER}_{PICK_AND_PLACE_RUNS}_{date}_{number}.csv"
        )
        number += 1

    return csv_file


def planner_log_file() -> Path:
    return TEST_LOG_DIR / PLANNER_LOGS.get(PLANNER, f"{PLANNER}.log")


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
) -> None:
    failure_log_file = csv_file.with_name(f"{csv_file.stem}_failure.log")
    with failure_log_file.open("a") as file:
        file.write("\n\n")
        file.write("=" * 80 + "\n")
        file.write(f"RUN {run_number} TIMEOUT PLANNER LOG\n")
        file.write("=" * 80 + "\n")
        if log_text:
            file.write(log_text)
            if not log_text.endswith("\n"):
                file.write("\n")
        else:
            file.write("(no planner log lines captured for this run)\n")


def start_planner() -> None:
    print(f"starting {PLANNER}")
    if PLANNER == "curobo":
        test_sh(PLANNER, "launch_rviz:=true")
    else:
        test_sh(PLANNER)


def restart_after_timeout() -> None:
    print("restarting ros2 and planner after timed-out pick_and_place run")
    test_sh("kill")
    test_sh("restart_ros")
    print("playing isaac sim")
    test_sh("sim_headless", "play")
    start_planner()


def pick_and_place_sequence() -> int:
    try:
        print("playing isaac sim")
        test_sh("sim_headless", "play")

        start_planner()
        csv_file = new_csv_file()
        run_args = pick_and_place_args(PLANNER)

        for run_number in range(1, PICK_AND_PLACE_RUNS + 1):
            if run_number > 1:
                print("playing isaac sim")
                test_sh("sim_headless", "play")

            print(f"pick_and_place run {run_number}")
            stop_file = f"/tmp/pick_and_place_metrics_recorder_{run_number}.stop"
            recorder = start_metrics_recorder(stop_file)

            time.sleep(0.5)
            planner_log = planner_log_file()
            planner_log_start = file_position(planner_log)
            start_time = time.monotonic()
            result = test_sh("pick_and_place_run", str(run_number), *run_args)
            total_time = time.monotonic() - start_time

            recorder_output = stop_metrics_recorder(stop_file, recorder)
            metrics = read_metrics(recorder_output)

            timed_out = (
                result.returncode == RUN_TIMEOUT_EXIT_CODE
                or total_time >= PICK_AND_PLACE_RUN_TIMEOUT_S
            )
            run_success = not timed_out
            append_csv_row(
                csv_file,
                run_number,
                total_time,
                metrics,
                success=run_success,
            )
            if timed_out:
                planner_log_text = planner_log_chunk(planner_log, planner_log_start)
                append_timeout_planner_log(
                    csv_file,
                    run_number,
                    planner_log_text,
                )

            print("stopping isaac sim")
            test_sh("sim_headless", "stop")

            if timed_out and run_number < PICK_AND_PLACE_RUNS:
                restart_after_timeout()

        return 0
    finally:
        test_sh("sim_headless", "stop")
        test_sh("kill")


def main() -> int:
    return pick_and_place_sequence()


if __name__ == "__main__":
    raise SystemExit(main())
