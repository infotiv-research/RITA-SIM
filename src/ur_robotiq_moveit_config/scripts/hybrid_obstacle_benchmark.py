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
    DEFAULT_SETTLE_TIME_SEC,
    json_safe,
)
from hybrid_benchmark.runner import HybridObstacleBenchmark


def build_arg_parser():
    parser = argparse.ArgumentParser(description="Run scripted hybrid-planner obstacle benchmarks.")
    parser.add_argument("--case", default=DEFAULT_BENCHMARK_CASE, choices=sorted(BENCHMARK_CASES.keys()))
    parser.add_argument("--runs", type=int, default=1)
    return parser


def main():
    parser = build_arg_parser()
    args, ros_args = parser.parse_known_args()

    output_dir = Path(DEFAULT_OUTPUT_DIR).expanduser().resolve()
    output_dir.mkdir(parents=True, exist_ok=True)

    rclpy.init(args=ros_args)
    node = HybridObstacleBenchmark()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    try:
        if not node.wait_until_ready(timeout_sec=30.0):
            raise RuntimeError("Hybrid benchmark dependencies are not ready.")

        for run_index in range(1, max(int(args.runs), 1) + 1):
            result = node.run_single_benchmark(args, run_index)
            filename = (
                f"hybrid_benchmark_{args.case}_run{run_index:02d}_"
                f"{datetime.now().strftime('%Y%m%d_%H%M%S')}.json"
            )
            output_path = output_dir / filename
            with output_path.open("w", encoding="utf-8") as handle:
                json.dump(json_safe(result), handle, indent=2, sort_keys=True)
                handle.write("\n")

            replanning = result.get("replanning", {})
            print(
                f"[run {run_index}] case={args.case} "
                f"success={bool(result.get('success'))} "
                f"replans={replanning.get('replan_count')} "
                f"invalidations={replanning.get('path_invalidated_count')} "
                f"output={output_path}"
            )

            if run_index < int(args.runs):
                time.sleep(float(DEFAULT_SETTLE_TIME_SEC))
    finally:
        executor.shutdown()
        spin_thread.join(timeout=2.0)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
