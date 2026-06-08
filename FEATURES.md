# Pick and place extra scenarios

## Specify other planners like this:

```bash
# Default: MoveIt backend with cuMotion pipeline
./control.sh pick_and_place

./control.sh pick_and_place planning_pipeline:=ompl

./control.sh pick_and_place motion_backend:=curobo_ros

./control.sh pick_and_place motion_backend:=curobo_ros curobo_planner_type:=mpc

./control.sh pick_and_place motion_backend:=hybrid
```

- The launch command starts `pick_and_place_main.py`.
- `motion_backend:=moveit` connects to MoveIt at `/move_action`.
- `motion_backend:=curobo_ros` requests trajectories from `/unified_planner/generate_trajectory` and
  executes them through the existing arm controller action.
- `motion_backend:=curobo_ros curobo_planner_type:=mpc` uses curobo MPC for each fixed-goal phase so
  the arm can react to moving obstacles while tracking the current phase target.
- `motion_backend:=hybrid` routes transport motions through MoveIt Hybrid Planning
- It publishes the target object as a planning-scene collision object and toggles attach/detach
  during grasp and release.
- The launch argument `planning_pipeline` defaults to `cumotion` and can be set to `ompl` only for
  the MoveIt backend.


# moving cylinder

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

Set the path first with `./control.sh cylinder set <number>`, then run
`./control.sh cylinder start`. The selected path is reused until you change it with another `set`
command.


# simulation with human character

Start the humanoid simulation with `./control.sh sim_humanoid`.

Inside Isaac Sim, open the `Content` tab below the viewport. Go to `My Computer`, then
`/ros2_ws`, then `assets`, then `ur10e_robotiq2f-140`, and open
`main_scene_with_human_character.usd`.

In the Isaac Sim container, open a new terminal and run `./control.sh humanoid play pick` to move the human character.

If the humanoid or human character does not move into the start position, or nothing happens when you run a movement
command, reopen the USD file from the Content window.
