#!/usr/bin/env python3
from __future__ import annotations

import subprocess
import sys

REPO_ROOT = "/home/research/isaac-sim-rita"
TEST_SH = "/home/research/isaac-sim-rita/test.sh"
PICK_AND_PLACE_RUNS = 1
PLANNER = sys.argv[1] if len(sys.argv) > 1 else "curobo"


def test_sh(*args: str) -> None:
    subprocess.run([TEST_SH, *args], cwd=REPO_ROOT)


def pick_and_place_sequence() -> int:
    try:
        print("playing isaac sim")
        test_sh("sim_headless", "play")

        print(f"starting {PLANNER}")
        if PLANNER == "curobo":
            test_sh(PLANNER, "launch_rviz:=true")
        else:
            test_sh(PLANNER)

        for run_number in range(1, PICK_AND_PLACE_RUNS + 1):
            if run_number > 1:
                print("playing isaac sim")
                test_sh("sim_headless", "play")

            print(f"pick_and_place run {run_number}")
            test_sh("pick_and_place_run", PLANNER, str(run_number))

            print("stopping isaac sim")
            test_sh("sim_headless", "stop")

        return 0
    finally:
        test_sh("sim_headless", "stop")
        test_sh("kill")


def main() -> int:
    return pick_and_place_sequence()


if __name__ == "__main__":
    raise SystemExit(main())
