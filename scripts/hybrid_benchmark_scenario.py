#!/usr/bin/env python3
from __future__ import annotations

import subprocess
import sys
from pathlib import Path

SCRIPT_DIR = Path(__file__).resolve().parent
REPO_ROOT = SCRIPT_DIR.parent
TEST_SH = REPO_ROOT / "test.sh"


def test_sh(*args: str) -> subprocess.CompletedProcess:
    return subprocess.run([TEST_SH, *args], cwd=REPO_ROOT)


def hybrid_benchmark_sequence(args: list[str]) -> int:
    try:
        print("playing isaac sim")
        play = test_sh("sim_headless", "play")
        if play.returncode != 0:
            return play.returncode

        print("starting hybrid")
        hybrid = test_sh("hybrid")
        if hybrid.returncode != 0:
            return hybrid.returncode

        print("hybrid benchmark")
        benchmark = test_sh("hybrid_benchmark_run", *args)
        return benchmark.returncode
    finally:
        test_sh("sim_headless", "stop")
        test_sh("kill")


def main() -> int:
    return hybrid_benchmark_sequence(sys.argv[1:])


if __name__ == "__main__":
    raise SystemExit(main())
