#!/usr/bin/env python3
from __future__ import annotations

import subprocess
import shutil
import sys
from pathlib import Path

SCRIPT_DIR = Path(__file__).resolve().parent
REPO_ROOT = SCRIPT_DIR.parent
TEST_SH = REPO_ROOT / "test.sh"
TEST_LOG_DIR = REPO_ROOT / "test_logs"


def test_sh(*args: str) -> subprocess.CompletedProcess:
    return subprocess.run([TEST_SH, *args], cwd=REPO_ROOT)


def extract_scenario_options(args: list[str]) -> tuple[int, Path | None, list[str]]:
    run_count = 1
    log_dir = None
    forwarded_args = []
    index = 0
    while index < len(args):
        arg = args[index]
        if arg == "--runs":
            if index + 1 >= len(args):
                forwarded_args.append(arg)
                index += 1
                continue
            run_count = max(int(args[index + 1]), 1)
            index += 2
            continue
        if arg.startswith("--runs="):
            run_count = max(int(arg.split("=", 1)[1]), 1)
            index += 1
            continue
        if arg == "--log-dir":
            if index + 1 >= len(args):
                forwarded_args.append(arg)
                index += 1
                continue
            log_dir = Path(args[index + 1])
            index += 2
            continue
        if arg.startswith("--log-dir="):
            log_dir = Path(arg.split("=", 1)[1])
            index += 1
            continue
        forwarded_args.append(arg)
        index += 1
    return run_count, log_dir, forwarded_args


def prepare_hybrid_benchmark_run(run_number: int) -> int:
    print(f"resetting simulation and planner before hybrid benchmark run {run_number}")
    stop = test_sh("sim_headless", "stop")
    if stop.returncode != 0:
        return stop.returncode

    print("stopping ros2 and planner launch processes")
    kill = test_sh("kill")
    if kill.returncode != 0:
        return kill.returncode

    print("restarting ros2 before hybrid benchmark run")
    restart = test_sh("restart_ros")
    if restart.returncode != 0:
        return restart.returncode

    print("playing isaac sim")
    play = test_sh("sim_headless", "play")
    if play.returncode != 0:
        return play.returncode

    print("starting hybrid")
    hybrid = test_sh("hybrid")
    if hybrid.returncode != 0:
        return hybrid.returncode

    return 0


def archive_run_logs(log_dir: Path | None, run_number: int) -> None:
    if log_dir is None:
        return

    log_dir.mkdir(parents=True, exist_ok=True)
    log_sources = (
        (TEST_LOG_DIR / "hybrid_benchmark.log", f"hybrid_benchmark_run_{run_number}.log"),
        (TEST_LOG_DIR / "hybrid.log", f"hybrid_{run_number}.log"),
        (TEST_LOG_DIR / "ros.log", f"ros_{run_number}.log"),
    )
    for source, destination_name in log_sources:
        if source.exists():
            shutil.copy2(source, log_dir / destination_name)


def hybrid_benchmark_sequence(args: list[str]) -> int:
    run_count, log_dir, forwarded_args = extract_scenario_options(args)
    failures = 0
    try:
        for run_number in range(1, run_count + 1):
            prepare_status = prepare_hybrid_benchmark_run(run_number)
            if prepare_status != 0:
                return prepare_status

            print(f"hybrid benchmark run {run_number}")
            benchmark = test_sh(
                "hybrid_benchmark_run",
                *forwarded_args,
                "--runs",
                "1",
                "--run-index-offset",
                str(run_number - 1),
            )
            if benchmark.returncode != 0:
                failures += 1
            archive_run_logs(log_dir, run_number)

        return 0 if failures == 0 else 1
    finally:
        test_sh("sim_headless", "stop")
        test_sh("kill")


def main() -> int:
    return hybrid_benchmark_sequence(sys.argv[1:])


if __name__ == "__main__":
    raise SystemExit(main())
