#!/usr/bin/env python3
from __future__ import annotations
from datetime import datetime
from pathlib import Path
import subprocess
import sys
import time

from recorders.pick_and_place_recorder import (
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


PICK_AND_PLACE_RUNS = 3
PLANNER = sys.argv[1] if len(sys.argv) > 1 else "curobo"


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


def start_planner() -> None:
    print(f"starting {PLANNER}")
    if PLANNER == "curobo":
        test_sh(PLANNER, "launch_rviz:=true")
    else:
        test_sh(PLANNER)


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
            start_time = time.monotonic()
            result = test_sh("pick_and_place_run", str(run_number), *run_args)
            total_time = time.monotonic() - start_time

            recorder_output = stop_metrics_recorder(stop_file, recorder)
            metrics = read_metrics(recorder_output)

            timed_out = result.returncode == RUN_TIMEOUT_EXIT_CODE
            if timed_out:
                mark_timeout(metrics)
            elif result.returncode != 0 or metrics.get("success") is not True:
                mark_failed_run(metrics)

            run_success = result.returncode == 0 and metrics.get("success") is True
            append_csv_row(
                csv_file,
                run_number,
                total_time,
                metrics,
                success=run_success,
            )

            print("stopping isaac sim")
            test_sh("sim_headless", "stop")

            if not run_success and run_number < PICK_AND_PLACE_RUNS:
                print("restarting ros2 after failed pick_and_place run")
                test_sh("restart_ros")

        return 0
    finally:
        test_sh("sim_headless", "stop")
        test_sh("kill")


def main() -> int:
    return pick_and_place_sequence()


if __name__ == "__main__":
    raise SystemExit(main())
