# Isaac Sim + ROS 2 (RITA)

This repository provides a dual-container workflow:
- ROS 2 container for robot control, MoveIt, and controllers.
- Isaac Sim container for simulation.

You can also run an optional third container for cuMotion planning.

Both containers (or all three with cuMotion) can run at the same time.

## Prerequisites

Make sure your machine has:
- Docker installed and working for your user account.
- Visual Studio Code installed.
- VS Code extension: **Dev Containers**.
- NVIDIA driver + GPU support in Docker (required for Isaac Sim GUI).

```
# Check Docker installation
docker --version
docker-compose --version

# Check NVIDIA drivers and Docker GPU access
nvidia-smi
docker run --rm --gpus all nvidia/cuda:11.0-base nvidia-smi
```



For Linux GUI forwarding, run this after each reboot:

```bash
xhost +
```

## Clone and Open

Clone and open the project in VS Code.

If VS Code does not prompt automatically, open command palette (`Ctrl+Shift+P`) and run:

```text
Dev Containers: Reopen in Container
```

## Quick Start

The best place to learn the available workflows, start components, and understand the command entrypoints is:

```bash
./control.sh help
```

To stop launch processes and remove workspace build artifacts (`build/`, `install/`, `log/`):

```bash
./control.sh clean
```

To clean and rebuild the workspace:

```bash
./control.sh build
```

## DevContainer Choices

You have three configurations:
- `UR10 ROS2 DevContainer`: ROS 2 development and control stack.
- `IsaacSim DevContainer`: Isaac Sim runtime and ROS bridge side.
- `UR10e cuMotion DevContainer`: cuMotion planner runtime.

Use two VS Code windows for ROS 2 + Isaac Sim, or three if using cuMotion.

## Start the ROS 2 Side

Open in container: `UR10 ROS2 DevContainer`.

Inside the container terminal start robot control:

```bash
./control.sh robot_control
```

## Start the Isaac Side

Open in container: `IsaacSim DevContainer`.

Inside the container terminal:

```bash
./control.sh sim
```

In Isaac Sim: `Start the simulation`.

## Start the cuMotion Side

Open in container: `UR10e cuMotion DevContainer`.

Inside the container terminal:

```bash
./control.sh cumotion
```

cuMotion can take a few minutes to start on first run.

Start planning only after the `cuMotion is ready for planning queries` message appears.

## Quick Checks

If Isaac Sim does not start:
- Verify NVIDIA drivers on host.
- Verify Docker GPU access:

```bash
docker run --rm --gpus all nvidia/cuda:11.0-base nvidia-smi
```

# Credits
The code in the repo is taken from [ur10e_2f140_topic_based_ros2_control](https://github.com/qdeyna/ur10e_2f140_topic_based_ros2_control) and adapted fore this project 



```
License
This project is licensed under the BSD 3-Clause License 
Portions of the code are adapted from:
- Universal Robots ROS 2 repositories (BSD-3-Clause)
- Robotiq ROS 2 repositories (BSD-3-Clause / Apache-2.0)
- ROS 2 Control Tutorial by PickNik Robotics (Apache-2.0)
```
