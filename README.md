# RITA simulation

RITA (Robot In The Air) is a collaborative robot project developed as a use case for human-robot collaboration (HRC) within the manufacturing industry, specifically for Volvo Group Trucks Operations (GTO).

This repository has three components:
- ROS 2 container for robot control, MoveIt, and controllers.
- Isaac Sim container for simulation.
- cuMotion container planning.

[![Isaac sim pick and place with table placement](https://img.youtube.com/vi/JSuB3DH42jI/0.jpg)](https://www.youtube.com/watch?v=JSuB3DH42jI)


## Prerequisites and dependencies

Follow the [instruction](https://github.com/infotiv-research/SIMLAN/blob/main/dependencies.md) to make sure your machine has:
- Docker installed and working for your user account.
- Visual Studio Code installed.
- VS Code extension: **Dev Containers**.
- NVIDIA driver + GPU support in Docker (required for Isaac Sim GUI).

Run the commands below to check that the prerequisites (e.g docker and ocker-compose) and drivers (NVIDIA drivers and Docker GPU access) are installed correctly.
```
docker --version
docker-compose --version

nvidia-smi
docker run --rm --gpus all nvidia/cuda:11.0-base nvidia-smi
```

For Linux GUI forwarding, run this after each reboot:

```bash
xhost +local:docker
```

## DevContainers

Clone and open the project in VS Code.

If VS Code does not prompt automatically, open command palette (`Ctrl+Shift+P`) and run `Dev Containers: Reopen in Container`.

You have three configurations:
- `UR10 ROS2 DevContainer`: ROS 2 development and control stack.
- `IsaacSim DevContainer`: Isaac Sim runtime and ROS bridge side.
- `UR10e cuMotion DevContainer`: cuMotion planner runtime.


The best place to learn the available workflows, start components, and understand the command entrypoints is:

```bash
./control.sh help
```

### ROS2

> in container: `UR10 ROS2 DevContainer`.


To stop launch processes and remove workspace build artifacts (`build/`, `install/`, `log/`) and rebuild the workspace:

```bash
./control.sh clean
./control.sh build
```

Inside the container terminal start robot control:

```bash
./control.sh robot_control
```

### IsaacSim simulation

>  in container: `IsaacSim DevContainer`.

Inside the container terminal run **one of the follwing commands** and then click on  `Start the simulation` in the GUI:

```bash
./control.sh sim # simple

./control.sh sim_cylinder # Starts the scene with the moving cylinder obstacle enabled

./control.sh sim_humanoid # Starts the scene with the humanoid obstacle
```


### UR10e arm

> in container: `UR10e cuMotion DevContainer`.

Inside the container terminal execute one of the motion backend below:

```bash
./control.sh cumotion

./control.sh curobo

./control.sh ompl # CPU based
```

Keep in mind that cuMotion can take a few minutes to start on first run.

for cumotion workflow, start planning only after the `cuMotion is ready for planning queries` message appears. 
The default planner is set to cuMotion. The OMPL planner is also available and can be set in Rviz2 under the context window.



curobo workflow launches `curobo_ros`, the curobo world bridge, the trajectory forwarder used for RViz execution, and the dedicated curobo RViz configuration.


## Robot controls

Make sure that a motion backend (preferebly curobo or cumotion) is up and running before executing the pick and place control commands

[![SIMLAN demo](https://img.youtube.com/vi/3d-NdI3MTQc/0.jpg)](https://www.youtube.com/watch?v=3d-NdI3MTQc)



### Pick and Place



In the same container but new terminal:

```bash
# Default: MoveIt backend with cuMotion pipeline
./control.sh pick_and_place

# Optional: MoveIt backend with OMPL pipeline
./control.sh pick_and_place planning_pipeline:=ompl

# Optional: curobo_ros backend
./control.sh pick_and_place motion_backend:=curobo_ros

# Optional: curobo_ros backend with MPC execution
./control.sh pick_and_place motion_backend:=curobo_ros curobo_planner_type:=mpc
```

- The launch command starts `pick_and_place_main.py`.
- `motion_backend:=moveit` connects to MoveIt at `/move_action`.
- `motion_backend:=curobo_ros` requests trajectories from `/unified_planner/generate_trajectory` and executes them through the existing arm controller action.
- `motion_backend:=curobo_ros curobo_planner_type:=mpc` uses curobo MPC for each fixed-goal phase so the arm can react to moving obstacles while tracking the current phase target.
- It publishes the target object as a planning-scene collision object and toggles attach/detach during grasp and release.
- The launch argument `planning_pipeline` defaults to `cumotion` and can be set to `ompl` only for the MoveIt backend.




### Run Moving Cylinder Obstacle

[![SIMLAN demo](https://img.youtube.com/vi/3uVEBaCn-WE/0.jpg)](https://www.youtube.com/watch?v=3uVEBaCn-WE)


In the Isaac Sim container, open a new terminal to configure and control the moving cylinder:

```bash
# Set the cylinder to vertical up/down motion
./control.sh cylinder set 2

# Start the cylinder using the currently selected path
./control.sh cylinder start

# Stop the cylinder
./control.sh cylinder stop
```

Available path presets:

- `2`: Vertical up/down motion
- `3`: Triangle motion in front of the flow rack
- `4`: Square motion in front of the flow rack

Set the path first with `./control.sh cylinder set <number>`, then run `./control.sh cylinder start`.
The selected path is reused until you change it with another `set` command.



### Run Humanoid Obstacle

[![SIMLAN demo](https://img.youtube.com/vi/47wNTquTGOw/0.jpg)](https://www.youtube.com/watch?v=47wNTquTGOw)



In the Isaac Sim container, open a new terminal to control the humanoid animation:

```bash
# Play the dab animation
./control.sh humanoid play dab

# Play the pick animation
./control.sh humanoid play pick

# Stop the humanoid animation
./control.sh humanoid stop
```


## 3D Gaussian Splatting assets

The `assets/gaussian_splats/` folder have 3D Gaussian Splats that are rendered in Isaac Sim 5.1 via NVIDIA [3DGRUT](https://github.com/nv-tlabs/3dgrut). A `rita-3dgrut` container (separate from the Isaac, ROS 2, and cuMotion containers) is used to convert a captured `.ply` into a USDZ that Isaac Sim can reference. The reason for the separate container is that, as of writing, 3DGRUT requires an older CUDA version than we use in the other containers.

```bash
# 1. One-time: build the 3dgrut container
docker compose -f setup/docker-compose.3dgrut.yaml build

# 2. Convert a 3DGS .ply into a .usdz (with an embedded collision hull)
scripts/ply_to_usdz.sh \
  assets/gaussian_splats/lamp/lamp_edited.ply \
  assets/gaussian_splats/lamp/lamp_edited.usdz \
  --with-collision

# 3. When you're done converting assets, stop the container
docker compose -f setup/docker-compose.3dgrut.yaml down
```

It works as this: `scripts/ply_to_usdz.sh` bind-mounts the repo into `/workspace` and runs `threedgrut.export.scripts.ply_to_usd` inside the container. With `--with-collision`, it also calls `scripts/gen_convex_hull_mesh.py` to build a convex-hull triangle mesh from the splat positions and embeds it via `threedgrut.export.scripts.add_mesh_to_usdz` as an invisible collider, so the robot treats the lamp as a solid obstacle while still rendering the photorealistic gaussian.


# Credits

The project is done by as a part of Master Thesis within infotiv:
- Elias Wilsborn
- Marcus Olsson

Team/Tech Lead: Hamid Ebadi

The code in the repo is taken from [ur10e_2f140_topic_based_ros2_control](https://github.com/qdeyna/ur10e_2f140_topic_based_ros2_control) and adapted for this project 


This repository vendors the cuRobo ROS packages directly in `src/`:

- `src/curobo_msgs`
- `src/curobo_ros`
- `src/curobo_rviz`

These packages originated from the following upstream repositories:

- `curobo_ros`: `https://github.com/Lab-CORO/curobo_ros.git`
- `curobo_msgs`: `https://github.com/Lab-CORO/curobo_msgs.git` 
- `curobo_rviz`: `https://github.com/Lab-CORO/curobo_rviz.git`


RITA designs are Inspired by:
- [Towards an infrastructure for preparation and control of intelligent automation systems](https://research.chalmers.se/publication/528129/file/528129_Fulltext.pdf)
- [To Collaborate and Coexist with Robots in an Industrial Setting: UX-based design guidelines for cobots in Industry 4.0 and 5.0](https://www.diva-portal.org/smash/get/diva2:1882744/FULLTEXT01.pdf)
- [Perceived Safety Aspects when Collaborating with Robots in the Manufacturing Industry: Applying an HTO Methodology](https://uu.diva-portal.org/smash/get/diva2:1863125/FULLTEXT01.pdf)


This work was carried out within Artwork research projects:
- The EUREKA ITEA4 [ArtWork](https://www.vinnova.se/p/artwork---the-smart-and-connected-worker/) - The smart and connected worker financed by Vinnova under the grant number 2023-00970.

| INFOTIV AB                            | CHALMERS                               | Volvo Group                    |
| ------------------------------------- |  -------------------------------------- | ------------------------------ |
| ![](resources/logos/INFOTIV-logo.png) | ![](resources/logos/CHALMERS-logo.png) | ![](resources/logos/volvo.jpg) |


To see a complete list of contributors see the [changelog](CHANGELOG.md).


# License
This project is licensed under the [BSD 3-Clause License](LICENSE.txt) 

Portions of the code are adapted from:
- Universal Robots ROS 2 repositories (BSD-3-Clause)
- Robotiq ROS 2 repositories (BSD-3-Clause / Apache-2.0)
- ROS 2 Control Tutorial by PickNik Robotics (Apache-2.0)
