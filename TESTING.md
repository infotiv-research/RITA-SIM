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


to stop the stack

```bash
./test.sh stop
```

Logs are written under [`test_logs/`](./test_logs/):

## Change Camera View in RViz

To change the camera point of view during headless tests, set the image topic to `/rgb`, `/rgb_gripper`, or `/rgb_overhead`.

For `ompl`, `cumotion`, and `hybrid`:

```text
Displays -> Image -> Topic
```

For `curobo`:

```text
Displays -> Workspace Overview -> Topic
```
