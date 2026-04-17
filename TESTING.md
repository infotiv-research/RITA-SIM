# Testing Guide

This repository includes a host-side test runner, [`test.sh`](./test.sh), for bringing up the simulation stack and running the scripted pick-and-place validation flow.

The workflow is scenario-based rather than a unit-test suite:
- `start` boots the required containers and launches the core processes.
- `pick_and_place curobo` runs the automated pick-and-place scenario against the already running stack.
- `stop` shuts the stack down cleanly.

## Prerequisites

Before using `test.sh`, make sure the normal project prerequisites from [README.md](./README.md) are already satisfied:
- Docker is installed and works for your user account.
- NVIDIA GPU support in Docker is available for Isaac Sim.
- GUI forwarding is configured when you want RViz to open.

For Linux GUI forwarding, run this after each reboot:

```bash
xhost +local:docker
```

On a fresh checkout, `./test.sh start` builds the ROS workspace inside the `ros2` container before launching the control stack.

If the script is not executable in your checkout, fix it once with:

```bash
chmod +x ./test.sh
```

## Quick Start

Run all commands from the repository root on the host machine.

### 1. Start the stack

```bash
./test.sh start
```

This command:
- starts the `ros2` and `cumotion` containers
- builds the ROS workspace in `ros2` if it is missing
- starts the `isaacsim` container
- launches the ROS control stack with `./control.sh ros`
- launches Isaac Sim headless

`start` submits the startup sequence and returns. It does not fully execute any scenario by itself.

### 2. Run the automated pick-and-place test

```bash
./test.sh pick_and_place curobo
```

This command expects the stack from `./test.sh start` to already be running. It then:
- plays the existing Isaac Sim timeline
- waits for the ROS control action servers
- starts the curobo backend in the `cumotion` container
- waits for the curobo ready message
- starts the dedicated curobo RViz session
- runs `./control.sh pick_and_place motion_backend:=curobo_ros`

When the run finishes, the script automatically stops only the temporary test-side processes it started for this scenario:
- Isaac timeline playback
- curobo backend
- curobo RViz

The main containers remain up until you stop them explicitly.

### 3. Stop the stack

```bash
./test.sh stop
```

This command:
- stops Isaac Sim headless
- stops the ROS control stack
- stops any active cuMotion launches
- shuts down the `isaacsim`, `ros2`, and `cumotion` containers

## Logs

Logs are written under [`test_logs/`](./test_logs/):
- `test_logs/ros_build.log`
- `test_logs/ros.log`
- `test_logs/isaac_headless.log`
- `test_logs/curobo.log`
- `test_logs/curobo_rviz.log`
- `test_logs/pick_and_place.log`

## Troubleshooting

### `bash: ./test.sh: Permission denied`

The script is not executable in the current checkout:

```bash
chmod +x ./test.sh
```

### `pick_and_place curobo` exits immediately

The scenario runner expects:
- `ros2`, `cumotion`, and `isaacsim` containers to already be running
- Isaac Sim headless to already be running

Start the stack first:

```bash
./test.sh start
```

### RViz does not open

RViz requires a working `DISPLAY` inside the `cumotion` container. On Linux hosts, run:

```bash
xhost +local:docker
```

Then rerun the workflow.
