#!/usr/bin/env python3
"""Launch Isaac Sim from Python and open the repo's main scene."""

from __future__ import annotations

import argparse
import importlib.util
import os
import runpy
import sys
import time
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parent.parent
ISAAC_SIM_PATH = Path(os.environ.get("ISAAC_SIM_PATH", "/isaac-sim"))
ROS_DISTRO = os.environ.get("ROS_DISTRO", "jazzy")
BRIDGE_LIB_PATH = ISAAC_SIM_PATH / "exts" / "isaacsim.ros2.bridge" / ROS_DISTRO / "lib"


def _prepare_ros_bridge_environment() -> None:
    if not BRIDGE_LIB_PATH.is_dir():
        raise FileNotFoundError(f"Isaac Sim ROS 2 bridge libs not found at {BRIDGE_LIB_PATH}")

    os.environ.setdefault("RMW_IMPLEMENTATION", "rmw_fastrtps_cpp")
    os.environ.setdefault("ROS_DOMAIN_ID", "0")
    os.environ.setdefault("ROS_LOCALHOST_ONLY", "0")
    os.environ.setdefault("FASTDDS_SHM_DEFAULT_SEGMENT_SIZE", "134217728")

    for variable_name in (
        "PYTHONPATH",
        "OLD_PYTHONPATH",
        "AMENT_PREFIX_PATH",
        "CMAKE_PREFIX_PATH",
        "COLCON_PREFIX_PATH",
    ):
        os.environ.pop(variable_name, None)

    os.environ["LD_LIBRARY_PATH"] = str(BRIDGE_LIB_PATH)


_prepare_ros_bridge_environment()


from isaacsim import SimulationApp


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Start Isaac Sim and open a USD scene from this repository.",
    )
    parser.add_argument(
        "--scene",
        default="assets/ur10e_robotiq2f-140/main_scene.usd",
        help="USD scene path, relative to the repository root by default.",
    )
    parser.add_argument(
        "--headless",
        action="store_true",
        help="Start Isaac Sim without the GUI.",
    )
    parser.add_argument(
        "--wait-updates",
        type=int,
        default=5,
        help="Number of Kit updates to wait before opening the stage.",
    )
    parser.add_argument(
        "--pause-after-startup",
        action="store_true",
        help="Pause the simulation timeline once startup completes.",
    )
    parser.add_argument(
        "--control-file",
        default="/tmp/rita_isaac_timeline_command",
        help="Path to a file used for external play/pause commands.",
    )
    parser.add_argument(
        "--state-file",
        default="/tmp/rita_isaac_timeline_state",
        help="Path to a file where the current timeline state is written.",
    )
    return parser.parse_args()


ARGS = _parse_args()
SIMULATION_APP = SimulationApp({"headless": ARGS.headless})


import asyncio

import carb
import omni.timeline
import omni.kit.app
import omni.usd
from isaacsim.core.utils.extensions import enable_extension


LOAD_EXTENSION_SCRIPT = REPO_ROOT / "startup_scripts" / "load_extension.py"
URDF_EXPORTER_SCRIPT = REPO_ROOT / "src" / "ur_robotiq_moveit_config" / "scripts" / "urdf_exporter.py"
CONTROL_FILE = Path(ARGS.control_file)
STATE_FILE = Path(ARGS.state_file)


def _resolve_scene_path(scene_arg: str) -> Path:
    scene_path = Path(scene_arg)
    if not scene_path.is_absolute():
        scene_path = REPO_ROOT / scene_path
    return scene_path.resolve()


def _run_python_script(script_path: Path) -> None:
    if not script_path.is_file():
        raise FileNotFoundError(f"Startup script not found: {script_path}")

    carb.log_info(f"Running startup script: {script_path}")
    runpy.run_path(str(script_path), run_name="__main__")


def _load_python_module(module_path: Path, module_name: str):
    if not module_path.is_file():
        raise FileNotFoundError(f"Module file not found: {module_path}")

    spec = importlib.util.spec_from_file_location(module_name, module_path)
    if spec is None or spec.loader is None:
        raise ImportError(f"Could not create import spec for {module_path}")

    module = importlib.util.module_from_spec(spec)
    sys.modules[module_name] = module
    spec.loader.exec_module(module)
    return module


async def _wait_for_world(wait_updates: int) -> object:
    app = omni.kit.app.get_app()
    usd_context = omni.usd.get_context()

    for _ in range(max(0, wait_updates)):
        await app.next_update_async()

    while app.is_running():
        stage = usd_context.get_stage()
        if stage is not None:
            world = stage.GetPrimAtPath("/World")
            if world and world.IsValid():
                world_children = [child for child in world.GetChildren() if child.IsValid()]
                if world_children:
                    return stage
        await app.next_update_async()

    raise RuntimeError("Isaac Sim closed before /World was populated.")


async def _run_urdf_export(stage: object) -> None:
    app = omni.kit.app.get_app()
    exporter_module = _load_python_module(URDF_EXPORTER_SCRIPT, "rita_urdf_exporter")

    retry_deadline = time.monotonic() + 30.0
    while app.is_running():
        try:
            converter_cls = exporter_module.load_converter_class()
        except Exception as exc:
            raise RuntimeError(f"URDF exporter initialization failed: {exc}") from exc

        if converter_cls is not None:
            exporter_module.sync_stage_urdf_exports(stage, converter_cls)
            return

        if time.monotonic() >= retry_deadline:
            raise TimeoutError("Timed out waiting for Isaac URDF exporter extension to load.")

        await app.next_update_async()

    raise RuntimeError("Isaac Sim closed before the URDF exporter became available.")


async def _open_scene(scene_path: Path, wait_updates: int) -> None:
    app = omni.kit.app.get_app()
    usd_context = omni.usd.get_context()

    for _ in range(max(0, wait_updates)):
        await app.next_update_async()

    carb.log_info(f"Opening stage: {scene_path}")
    result, err = await usd_context.open_stage_async(str(scene_path))
    if not result:
        raise RuntimeError(f"Failed to open stage '{scene_path}': {err}")

    carb.log_info(f"Stage opened: {usd_context.get_stage_url()}")


async def _start_simulation(wait_updates: int) -> None:
    app = omni.kit.app.get_app()
    _timeline_play()
    for _ in range(max(1, wait_updates)):
        await app.next_update_async()


async def _startup_sequence(scene_path: Path, wait_updates: int) -> None:
    enable_extension("isaacsim.ros2.bridge")
    SIMULATION_APP.update()
    _run_python_script(LOAD_EXTENSION_SCRIPT)
    await _open_scene(scene_path, wait_updates)
    await _start_simulation(wait_updates)
    stage = await _wait_for_world(wait_updates)
    await _run_urdf_export(stage)


def _write_timeline_state(state: str) -> None:
    STATE_FILE.write_text(state, encoding="utf-8")


def _timeline_play() -> None:
    timeline = omni.timeline.get_timeline_interface()
    timeline.play()
    _write_timeline_state("playing")


def _timeline_pause() -> None:
    timeline = omni.timeline.get_timeline_interface()
    if hasattr(timeline, "pause"):
        timeline.pause()
    else:
        timeline.stop()
    _write_timeline_state("paused")


def _timeline_stop() -> None:
    timeline = omni.timeline.get_timeline_interface()
    timeline.stop()
    _write_timeline_state("stopped")


def _apply_pending_timeline_command() -> None:
    if not CONTROL_FILE.is_file():
        return

    try:
        command = CONTROL_FILE.read_text(encoding="utf-8").strip().lower()
    except OSError:
        return

    try:
        CONTROL_FILE.unlink()
    except OSError:
        pass

    if command == "play":
        _timeline_play()
    elif command == "pause":
        _timeline_pause()
    elif command == "stop":
        _timeline_stop()


def main() -> None:
    scene_path = _resolve_scene_path(ARGS.scene)
    if not scene_path.is_file():
        raise FileNotFoundError(f"Scene file not found: {scene_path}")

    open_task = asyncio.ensure_future(_startup_sequence(scene_path, ARGS.wait_updates))

    while not open_task.done() and SIMULATION_APP.is_running():
        SIMULATION_APP.update()

    if not SIMULATION_APP.is_running():
        return

    open_task.result()

    if ARGS.pause_after_startup:
        _timeline_pause()
    else:
        _write_timeline_state("playing")

    while SIMULATION_APP.is_running():
        _apply_pending_timeline_command()
        SIMULATION_APP.update()


if __name__ == "__main__":
    try:
        main()
    finally:
        SIMULATION_APP.close()
