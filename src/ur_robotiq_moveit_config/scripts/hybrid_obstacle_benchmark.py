#!/usr/bin/env python3
"""Run repeatable hybrid-planner obstacle benchmarks.

Typical usage:
    ros2 run ur_robotiq_moveit_config hybrid_obstacle_benchmark.py -- \
        --case test_1 --runs 5

The hybrid stack should already be running via:
    ./control.sh hybrid
"""

from __future__ import annotations

import argparse
import json
import threading
import time
from datetime import datetime
from pathlib import Path

import rclpy
from rclpy.executors import MultiThreadedExecutor

from hybrid_benchmark.constants import (
    BENCHMARK_CASES,
    DEFAULT_BENCHMARK_CASE,
    DEFAULT_OUTPUT_DIR,
    DEFAULT_SPAWN_TRIGGER_PROFILE,
    DEFAULT_SETTLE_TIME_SEC,
    SPAWN_TRIGGER_PROFILES_M,
    json_safe,
)
from hybrid_benchmark.runner import HybridObstacleBenchmark


def build_arg_parser():
    parser = argparse.ArgumentParser(description="Run scripted hybrid-planner obstacle benchmarks.")
    parser.add_argument("--case", default=DEFAULT_BENCHMARK_CASE, choices=sorted(BENCHMARK_CASES.keys()))
    parser.add_argument("--runs", type=int, default=1)
    parser.add_argument("--run-index-offset", type=int, default=0)
    parser.add_argument(
        "--spawn-profile",
        default=DEFAULT_SPAWN_TRIGGER_PROFILE,
        choices=sorted(SPAWN_TRIGGER_PROFILES_M.keys()),
        help="Named TCP-to-wall clearance trigger profile.",
    )
    parser.add_argument(
        "--spawn-clearance-m",
        type=float,
        default=None,
        help="Override the selected spawn profile with an explicit TCP-to-wall clearance in meters.",
    )
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=None,
        help="Directory where run JSON files should be written.",
    )
    return parser


def _as_int(value) -> int:
    try:
        return int(value or 0)
    except (TypeError, ValueError):
        return 0


def validate_result(result: dict) -> list[str]:
    failures = []
    obstacle = result.get("obstacle") or {}
    replanning = result.get("replanning") or {}

    if not bool(result.get("success")):
        failures.append("move_group_goal_failed")
    if bool(result.get("timed_out")):
        failures.append("timed_out")
    if not bool(obstacle.get("spawned")):
        failures.append("obstacle_not_spawned")
    if _as_int(replanning.get("path_invalidated_count")) < 1:
        failures.append("path_not_invalidated")
    if _as_int(replanning.get("replan_count")) < 1:
        failures.append("no_global_replan")
    return failures


def main() -> int:
    parser = build_arg_parser()
    args, ros_args = parser.parse_known_args()
    run_count = max(int(args.runs), 1)
    if args.spawn_clearance_m is not None and float(args.spawn_clearance_m) <= 0.0:
        parser.error("--spawn-clearance-m must be greater than zero")
    passed_runs = 0

    started_at = datetime.now()
    spawn_label = args.spawn_profile
    if args.spawn_clearance_m is not None:
        spawn_label = f"custom{float(args.spawn_clearance_m):.3f}m".replace(".", "p")
    if args.output_dir is None:
        output_dir = (
            Path(DEFAULT_OUTPUT_DIR).expanduser().resolve()
            / f"{started_at.strftime('%Y%m%d_%H%M%S')}_{args.case}_{spawn_label}_runs{run_count:02d}"
        )
    else:
        output_dir = Path(args.output_dir).expanduser()
    output_dir.mkdir(parents=True, exist_ok=True)

    rclpy.init(args=ros_args)
    node = HybridObstacleBenchmark()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    try:
        if not node.wait_until_ready(timeout_sec=30.0):
            print("FAIL: Hybrid benchmark dependencies are not ready.")
            return 1

        run_index_offset = max(int(args.run_index_offset), 0)
        for local_run_index in range(1, run_count + 1):
            run_index = run_index_offset + local_run_index
            result = node.run_single_benchmark(args, run_index)
            filename = f"hybrid_benchmark_{args.case}_{spawn_label}_run{run_index:02d}.json"
            output_path = output_dir / filename
            with output_path.open("w", encoding="utf-8") as handle:
                json.dump(json_safe(result), handle, indent=2, sort_keys=True)
                handle.write("\n")

            replanning = result.get("replanning", {})
            failures = validate_result(result)
            passed = not failures
            if passed:
                passed_runs += 1
            print(
                f"[run {run_index}] {'PASS' if passed else 'FAIL'} "
                f"case={args.case} "
                f"spawn_profile={args.spawn_profile} "
                f"success={bool(result.get('success'))} "
                f"replans={replanning.get('replan_count')} "
                f"invalidations={replanning.get('path_invalidated_count')} "
                f"output={output_path}"
                + (f" failures={','.join(failures)}" if failures else "")
            )

            if local_run_index < run_count:
                time.sleep(float(DEFAULT_SETTLE_TIME_SEC))

        print(f"Hybrid benchmark summary: {passed_runs}/{run_count} runs passed")
        return 0 if passed_runs == run_count else 1
    except Exception as exc:
        print(f"FAIL: Hybrid benchmark crashed: {exc}")
        return 1
    finally:
        executor.shutdown()
        spin_thread.join(timeout=2.0)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
