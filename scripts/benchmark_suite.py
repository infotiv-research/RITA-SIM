#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import shutil
import subprocess
import sys
from datetime import datetime
from pathlib import Path


SCRIPT_DIR = Path(__file__).resolve().parent
REPO_ROOT = SCRIPT_DIR.parent
TEST_SH = REPO_ROOT / "test.sh"
TEST_DATA_DIR = REPO_ROOT / "test_data"
TEST_LOG_DIR = REPO_ROOT / "test_logs"

MOVEIT_SCRIPT_DIR = REPO_ROOT / "src" / "ur_robotiq_moveit_config" / "scripts"
sys.path.insert(0, str(MOVEIT_SCRIPT_DIR))

from simple_motion.cases import SIMPLE_MOTION_CASES


SIMPLE_MOTION_PLANNER_CASES = (
    ("cumotion", ("cumotion",)),
    ("curobo", ("curobo",)),
    ("ompl_rrtStar", ("ompl", "rrtStar")),
    ("ompl_rrtConnect", ("ompl", "rrtConnect")),
    ("hybrid", ("hybrid",)),
)
PICK_AND_PLACE_PLANNERS = ("curobo", "cumotion", "ompl", "hybrid")
SIMPLE_MOTION_RUNS_PER_CASE = 2
PICK_AND_PLACE_RUNS_PER_PLANNER = 2
HYBRID_BENCHMARK_PROFILES = ("early", "medium", "late","very_late")
HYBRID_BENCHMARK_CASES = ("test_1", "test_2")
HYBRID_BENCHMARK_RUNS_PER_CASE_PROFILE = 2


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Run benchmark suites and collect outputs.")
    subparsers = parser.add_subparsers(dest="suite", required=True)

    subparsers.add_parser(
        "simple_benchmark",
        help="Run the simple-motion benchmark across all configured planners and cases.",
    )

    subparsers.add_parser(
        "hybrid_planner",
        help="Run the hybrid obstacle benchmark across all timing profiles.",
    )

    pick_and_place = subparsers.add_parser(
        "pick_and_place",
        help="Run pick-and-place across all configured planners.",
    )
    pick_and_place.add_argument("--runs", type=int, default=PICK_AND_PLACE_RUNS_PER_PLANNER)

    subparsers.add_parser("all", help="Run simple_benchmark, hybrid_planner, and pick_and_place.")
    return parser


def test_sh(*args: str) -> subprocess.CompletedProcess:
    return subprocess.run([TEST_SH, *args], cwd=REPO_ROOT)


def new_suite_dir() -> Path:
    suite_root = TEST_DATA_DIR / "benchmark_suite"
    suite_root.mkdir(parents=True, exist_ok=True)

    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    suite_dir = suite_root / timestamp

    number = 1
    while suite_dir.exists():
        suite_dir = suite_root / f"{timestamp}_{number}"
        number += 1

    suite_dir.mkdir()
    return suite_dir


def repo_relative(path: Path) -> str:
    return str(path.resolve().relative_to(REPO_ROOT.resolve()))


def write_manifest(suite_dir: Path, selected_suites: list[str]) -> None:
    manifest = {
        "suite": "benchmark_suite",
        "started_at": datetime.now().isoformat(timespec="seconds"),
        "selected_suites": selected_suites,
        "simple_benchmark": {
            "runs_per_case": SIMPLE_MOTION_RUNS_PER_CASE,
            "planners": [label for label, _ in SIMPLE_MOTION_PLANNER_CASES],
            "cases": list(SIMPLE_MOTION_CASES),
        },
        "hybrid_planner": {
            "profiles": list(HYBRID_BENCHMARK_PROFILES),
            "cases": list(HYBRID_BENCHMARK_CASES),
            "runs_per_case_profile": HYBRID_BENCHMARK_RUNS_PER_CASE_PROFILE,
        },
        "pick_and_place": {
            "runs_per_planner": PICK_AND_PLACE_RUNS_PER_PLANNER,
            "planners": list(PICK_AND_PLACE_PLANNERS),
        },
    }
    manifest_file = suite_dir / "manifest.json"
    with manifest_file.open("w", encoding="utf-8") as file:
        json.dump(manifest, file, indent=2)
        file.write("\n")


def copy_existing_logs(destination: Path, log_names: tuple[str, ...]) -> None:
    destination.mkdir(parents=True, exist_ok=True)
    for log_name in log_names:
        source = TEST_LOG_DIR / log_name
        if source.exists():
            shutil.copy2(source, destination / log_name)


def run_simple_benchmark_suite(suite_dir: Path) -> int:
    failures = 0
    case_names = list(SIMPLE_MOTION_CASES)
    total_runs = len(SIMPLE_MOTION_PLANNER_CASES) * len(case_names)

    print(
        "simple_benchmark suite: "
        f"output_dir={suite_dir / 'simple_benchmark'}, "
        f"planners={','.join(label for label, _ in SIMPLE_MOTION_PLANNER_CASES)}, "
        f"runs_per_case={SIMPLE_MOTION_RUNS_PER_CASE}, "
        f"cases={len(case_names)}, "
        f"planner_case_pairs={total_runs}"
    )

    run_index = 0
    for planner_label, planner_args in SIMPLE_MOTION_PLANNER_CASES:
        for case_name in case_names:
            run_index += 1
            print(f"[{run_index}/{total_runs}] simple_benchmark planner={planner_label} case={case_name}")
            result = test_sh(
                "simple_motion",
                *planner_args,
                "--case",
                case_name,
                "--runs",
                str(SIMPLE_MOTION_RUNS_PER_CASE),
                "--csv-file",
                str(suite_dir / "simple_benchmark" / "results" / planner_label / f"{case_name}.csv"),
                "--log-dir",
                str(suite_dir / "simple_benchmark" / "logs" / planner_label / case_name),
            )
            if result.returncode != 0:
                failures += 1
                print(f"case failed: planner={planner_label} case={case_name} (exit={result.returncode})")

    if failures:
        print(f"simple_benchmark completed with {failures} failed planner/case pair(s)")
        return 1

    print("simple_benchmark completed successfully")
    return 0


def run_hybrid_planner_suite(suite_dir: Path) -> int:
    failures = 0
    run_count = HYBRID_BENCHMARK_RUNS_PER_CASE_PROFILE
    total_runs = len(HYBRID_BENCHMARK_CASES) * len(HYBRID_BENCHMARK_PROFILES)

    print(
        "hybrid_planner suite: "
        f"cases={','.join(HYBRID_BENCHMARK_CASES)}, "
        f"profiles={','.join(HYBRID_BENCHMARK_PROFILES)}, "
        f"runs_per_case_profile={run_count}"
    )

    run_index = 0
    for case_name in HYBRID_BENCHMARK_CASES:
        for profile in HYBRID_BENCHMARK_PROFILES:
            run_index += 1
            print(f"[{run_index}/{total_runs}] hybrid_planner case={case_name} profile={profile}")
            output_dir = suite_dir / "hybrid_planner" / "results" / case_name / profile
            result = test_sh(
                "hybrid_benchmark",
                "--case",
                case_name,
                "--runs",
                str(run_count),
                "--spawn-profile",
                profile,
                "--output-dir",
                repo_relative(output_dir),
                "--log-dir",
                str(suite_dir / "hybrid_planner" / "logs" / case_name / profile),
            )
            if result.returncode != 0:
                failures += 1
                print(f"profile failed: case={case_name} profile={profile} (exit={result.returncode})")

    if failures:
        print(f"hybrid_planner completed with {failures} failed profile(s)")
        return 1

    print("hybrid_planner completed successfully")
    return 0


def run_pick_and_place_suite(suite_dir: Path, runs: int) -> int:
    failures = 0
    run_count = max(int(runs), 1)

    print(
        "pick_and_place suite: "
        f"planners={','.join(PICK_AND_PLACE_PLANNERS)}, "
        f"runs_per_planner={run_count}"
    )

    for index, planner in enumerate(PICK_AND_PLACE_PLANNERS, start=1):
        print(f"[{index}/{len(PICK_AND_PLACE_PLANNERS)}] pick_and_place planner={planner}")
        result = test_sh(
            "pick_and_place",
            planner,
            "--runs",
            str(run_count),
            "--csv-file",
            str(suite_dir / "pick_and_place" / "results" / f"{planner}.csv"),
            "--log-dir",
            str(suite_dir / "pick_and_place" / "logs" / planner),
        )
        if result.returncode != 0:
            failures += 1
            print(f"planner failed: planner={planner} (exit={result.returncode})")

    if failures:
        print(f"pick_and_place completed with {failures} failed planner(s)")
        return 1

    print("pick_and_place completed successfully")
    return 0


def selected_suite_names(args) -> list[str]:
    if args.suite == "all":
        return ["simple_benchmark", "hybrid_planner", "pick_and_place"]
    return [args.suite]


def run_selected_suites(args) -> int:
    suite_dir = new_suite_dir()
    suites = selected_suite_names(args)
    write_manifest(suite_dir, suites)
    print(f"benchmark_suite output_dir={suite_dir}")

    failures = 0
    for suite_name in suites:
        if suite_name == "simple_benchmark":
            failures += run_simple_benchmark_suite(suite_dir)
        elif suite_name == "hybrid_planner":
            failures += run_hybrid_planner_suite(suite_dir)
        elif suite_name == "pick_and_place":
            failures += run_pick_and_place_suite(
                suite_dir,
                getattr(args, "runs", PICK_AND_PLACE_RUNS_PER_PLANNER),
            )
        else:
            raise ValueError(f"Unknown suite: {suite_name}")

    if failures:
        print(f"benchmark_suite completed with {failures} failed suite(s)")
        return 1

    print("benchmark_suite completed successfully")
    return 0


def main(argv: list[str]) -> int:
    parser = build_arg_parser()
    args = parser.parse_args(argv)
    return run_selected_suites(args)


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
