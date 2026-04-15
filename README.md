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


The best place to learn the available workflows, start components, and understand the command entrypoints is open a new terminal by going to `Terminal -> New terminal` and type:

```bash
./control.sh help
```

### ROS2

> in `UR10 ROS2 DevContainer` vscode container open a new terminal and run the following commands: 


To stop launch processes and remove workspace build artifacts (`build/`, `install/`, `log/`) and rebuild the workspace. These commands only need to be executed once:

```bash
./control.sh clean
./control.sh build
```

Start robot control:

```bash
./control.sh ros
```

**optional** : to build 3D Gaussian Splatting assets, download the [gaussian_splats package](https://www.sharepoint/....TODO) and place it in [`assets/gaussian_splats/`] folder. It contains 3D Gaussian Splats that are rendered in Isaac Sim 5.1 via NVIDIA [3DGRUT](https://github.com/nv-tlabs/3dgrut).
Run `./contro.sh build_gaussian_splats` to convert a captured `.ply` into a USDZ that Isaac Sim can reference.


### IsaacSim simulation

>  in `IsaacSim DevContainer` vscode container open a new terminal and run the following commands: 

Inside the container terminal run **one of the following commands** and then click on  `Start the simulation` in the GUI:

```bash
./control.sh sim # Simple

./control.sh sim_cylinder # Starts the scene with the moving cylinder obstacle enabled

./control.sh sim_humanoid # Starts the scene with the humanoid obstacle
```

**optional** : TODO: if you have built the gaussian splats assets, the `lamp_edited.usdz` appears in the content tab of isaac sim that can be moved into the scene.


### UR10e motion backend

> in `UR10e cuMotion DevContainer` vscode container open a new terminal and run the following commands: 

Inside the container terminal execute one of the motion backend **TODO** below:

```bash
./control.sh cumotion

./control.sh curobo

./control.sh ompl

./control.sh hybrid
```

#### cumotion
Keep in mind that cuMotion can take a few minutes to start on first run.
for cumotion workflow, start planning only after the `cuMotion is ready for planning queries` message appears. 
The default planner is set to cuMotion.
curobo workflow launches `curobo_ros`, the curobo world bridge, the trajectory forwarder used for RViz execution, and the dedicated curobo RViz configuration.

#### curobo

TODO

#### ompl
The OMPL planner is CPU only and should be set in Rviz2 under the context window.


#### hybrid  (cuRobo MotionGen + MPC)

The hybrid planner combines cuRobo MotionGen as a **global planner** (computes a full collision-free trajectory) with cuRobo MPC as a **local planner** (reactively tracks the global trajectory at high frequency). When the MPC detects an obstacle it cannot avoid, MoveIt Hybrid Planning automatically triggers a global replan.

The hybrid launch starts MoveIt (move_group + RViz), the cuRobo trajectory planner, the cuRobo world bridge, and the MoveIt Hybrid Planning components in a single command.

**Using RViz:** set a goal pose with the interactive marker and click **Plan & Execute**. The bridge intercepts the MoveGroup action, routes it through the hybrid pipeline, and streams MPC commands to the robot.

### Running Control Scenarios

Make sure that a motion backend (preferably curobo or cumotion) is up and running before executing the control commands and the **simulation is started**.

####  Pick and Place Scenario

[![SIMLAN demo](https://img.youtube.com/vi/3d-NdI3MTQc/0.jpg)](https://www.youtube.com/watch?v=3d-NdI3MTQc)

In the cuMotion container, open a new terminal and run the following commands:

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



### Dynamic Environment


In the Isaac Sim container, open a new terminal to modify the environment as below

### moving cylinder

[![SIMLAN demo](https://img.youtube.com/vi/3uVEBaCn-WE/0.jpg)](https://www.youtube.com/watch?v=3uVEBaCn-WE)



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



### moving human 

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
