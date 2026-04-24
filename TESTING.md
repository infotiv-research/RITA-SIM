# Automation

We don't want to open all the vscode windows, instead we automate the process in `test.sh`.

## Prerequisites

- Before using `test.sh`, make sure the normal project prerequisites from [README.md](./README.md) are already satisfied.
- Run all commands from the repository root on the host machine.

Start the stack by running the following command:

```bash
./test.sh start
```

once the command above executed succcessfully, run the automated pick-and-place.

```bash
./test.sh pick_and_place curobo
```

To run the automated hybrid obstacle benchmark:

```bash
./test.sh hybrid_benchmark --case test_1 --runs 1
```

JSON results are written under [`benchmark_results/hybrid/`](./benchmark_results/hybrid/).


to stop the stack

```bash
./test.sh stop
```

Logs are written under [`test_logs/`](./test_logs/):
