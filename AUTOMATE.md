# Automation

We don't want to open all the vscode windows, instead we automate the process in `automate.sh`.

## Prerequisites

- Before using `automate.sh`, make sure the normal project prerequisites from [README.md](./README.md) are already satisfied.
- Run all commands from the repository root on the host machine.

Start the stack by running the following command:

```bash
./automate.sh start
```

once the command above executed succcessfully, run the automated pick-and-place.

```bash
./automate.sh pick_and_place curobo
```


to stop the stack

```bash
./automate.sh stop
```

Logs are written under [`test_logs/`](./test_logs/):

