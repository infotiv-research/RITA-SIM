# Isaac Sim + ROS 2 (RITA)

This repository provides a dual-container workflow:
- ROS 2 container for robot control, MoveIt, and controllers.
- Isaac Sim container for simulation.

Both containers can run at the same time.

## 1) Prerequisites

Make sure your machine has:
- Docker installed and working for your user account.
- Visual Studio Code installed.
- VS Code extension: **Dev Containers**.
- NVIDIA driver + GPU support in Docker (required for Isaac Sim GUI).

For Linux GUI forwarding, run this after each reboot:

```bash
xhost +
```

## 2) Clone and Open

Clone and open the project in VS Code.

If VS Code does not prompt automatically, open command palette (`Ctrl+Shift+P`) and run:

```text
Dev Containers: Reopen in Container
```

## 3) DevContainer Choices

You have two configurations:
- `UR10 ROS2 DevContainer`: ROS 2 development and control stack.
- `IsaacSim DevContainer`: Isaac Sim runtime and ROS bridge side.

Use two VS Code windows if you want both running together.

## 4) Start the ROS 2 Side

Open in container: `UR10 ROS2 DevContainer`.

Inside the container terminal:

```bash
source install/setup.bash
```

Launch terminal 1:

```bash
ros2 launch ur_robotiq_description ur_robotiq_isaac_control.launch.py robot_ip:=aaa.bbb.ccc.ddd sim_isaac:=true
```

Launch terminal 2:

```bash
ros2 launch ur_robotiq_moveit_config ur_robotiq_isaac_moveit.launch.py use_fake_hardware:=true enable_joint_state_filter:=true moveit_joint_states_topic:=/moveit_joint_states
```

## 5) Start the Isaac Side

Open in container: `IsaacSim DevContainer`.

Inside the container terminal:

```bash
./script/post_install_ros2_isaac_start.sh
```

In Isaac Sim, open:

```text
assets/ur10e_robotiq2f-140/scene_with_flowrack_and_crates2.usd
```

Start the simulation.

## 6) Quick Checks

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
