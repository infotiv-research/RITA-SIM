# Scripts

This folder contains the project automation scripts for benchmark scenarios, metric recording, Isaac Sim setup, asset conversion, and result processing.

## Benchmark setup

`benchmark_suite.py` defines the full benchmark pipeline. This is where you change the planners, cases, run counts, hybrid spawn profiles, and which scenario groups are included in a full benchmark run.

## Scenario files

Files ending in `_scenario.py` are scenario runners. They prepare the simulation and ROS stack, start the selected planner, run the scenario, collect logs, and call the recorders.

`simple_motion_scenario.py` is for single simple-motion benchmark cases.

`pick_and_place_scenario.py` is for pick-and-place benchmark runs.

`hybrid_benchmark_scenario.py` is for the hybrid obstacle benchmark.

If you want to change how one scenario behaves, change the matching `_scenario.py` file. If you want to change what the full benchmark pipeline runs, change `benchmark_suite.py`.

## Recorders

The `recorders/` folder contains the scripts that save benchmark data. The scenario files start them automatically. Edit these files only if the recorded values or CSV columns need to change.

## Isaac Sim and asset helpers

`combine_benchmark_results.py` reads benchmark suite outputs and creates combined result tables, summaries, and figures.

`start_isaac_main_scene.py` starts Isaac Sim and opens the main scene.

`test_stack.py` checks that the ROS, Isaac, planner, and controller stack is ready.

`toggle_humanoid_animation.py` handles humanoid animation control from ROS topics.

`convert_gazebo_model.py` converts Gazebo models into USD assets.

`create_scene_with_environment.py` helps convert a Gazebo environment for Isaac Sim by creating a USD scene that includes the converted environment objects.

`gen_convex_hull_mesh.py` creates a collision proxy mesh from a PLY splat.

`ply_to_usdz.sh` converts a PLY splat into USDZ and can add the generated collision proxy.
