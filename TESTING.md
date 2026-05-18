# Automation

We don't want to open all the vscode windows, instead we automate the process in `test.sh`.

## Prerequisites

- Before using `test.sh`, make sure the normal project prerequisites from [README.md](./README.md)
  are already satisfied.
- Run all commands from the repository root on the host machine.

Start the stack by running the following command:

```bash
./test.sh start
```

With the containers already running, build the workspace packages with

```bash
./test.sh build
```

once the command above executed succcessfully, run the automated pick-and-place.

```bash
./test.sh pick_and_place curobo
```

To run the automated hybrid obstacle benchmark:

```bash
./test.sh hybrid_benchmark --case test_1 --runs 1
```

The hybrid benchmark places the wall around the middle of the initial global trajectory and controls
how early it appears with spawn profiles:

```bash
./test.sh hybrid_benchmark --case test_1 --runs 1 --spawn-profile early
./test.sh hybrid_benchmark --case test_1 --runs 1 --spawn-profile medium
./test.sh hybrid_benchmark --case test_1 --runs 1 --spawn-profile late
./test.sh hybrid_benchmark --case test_1 --runs 1 --spawn-profile very_late
```

To run benchmark suites with results and archived logs collected under one timestamped folder:

```bash
./test.sh benchmark_suite simple_benchmark
./test.sh benchmark_suite hybrid_planner
./test.sh benchmark_suite pick_and_place
./test.sh benchmark_suite all
```

Suite outputs are written under [`test_data/benchmark_suite/`](./test_data/benchmark_suite/). Direct
`./test.sh hybrid_benchmark ...` runs still write JSON results under
[`benchmark_results/hybrid/`](./benchmark_results/hybrid/).

to stop the stack

```bash
./test.sh stop
```

Logs are written under [`test_logs/`](./test_logs/):

## Change Camera View in RViz

To change the camera point of view during headless tests, set the image topic to `/rgb`,
`/rgb_gripper`, or `/rgb_overhead`.

For `ompl`, `cumotion`, and `hybrid`:

```text
Displays -> Image -> Topic
```

For `curobo`:

```text
Displays -> Workspace Overview -> Topic
```
