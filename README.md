# Isaac Sim + ROS 2 (RITA)

This repository provides a dual-container workflow:
- ROS 2 container for robot control, MoveIt, and controllers.
- Isaac Sim container for simulation.

Both containers can run at the same time.

## Prerequisites

Make sure your machine has:
- Docker installed and working for your user account.
- Visual Studio Code installed.
- VS Code extension: **Dev Containers**.
- NVIDIA driver + GPU support in Docker (required for Isaac Sim GUI).

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

## DevContainer Choices

You have two configurations:
- `UR10 ROS2 DevContainer`: ROS 2 development and control stack.
- `IsaacSim DevContainer`: Isaac Sim runtime and ROS bridge side.

Use two VS Code windows if you want both running together.

## Start the ROS 2 Side

Open in container: `UR10 ROS2 DevContainer`.

Inside the container terminal:

```bash
./post_start.sh
source install/setup.bash
```

Launch terminal 1:

```bash
ros2 launch ur_robotiq_description ur_robotiq_isaac_control.launch.py
```

Launch another terminal:

```bash
ros2 launch ur_robotiq_moveit_config ur_robotiq_isaac_moveit.launch.py
```

## Start the Isaac Side

Open in container: `IsaacSim DevContainer`.

Inside the container terminal:

```bash
./startup_scripts/post_install_ros2_isaac_start.sh
```

In Isaac Sim: `Start the simulation`.

## Quick Checks

If Isaac Sim does not start:
- Verify NVIDIA drivers on host.
- Verify Docker GPU access:

```bash
docker run --rm --gpus all nvidia/cuda:11.0-base nvidia-smi
```

If ROS 2 commands cannot find packages:

```bash
source install/setup.bash
```
