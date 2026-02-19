#!/usr/bin/env python3
"""Open a USD scene at startup when launched through Kit --exec."""
 
import asyncio
import os
 
import carb
import omni.kit.app
import omni.usd
 
 
async def _open_scene_on_startup() -> None:
    scene_path = os.environ.get("ISAAC_STARTUP_SCENE", "").strip()
    if not scene_path:
        carb.log_warn("ISAAC_STARTUP_SCENE is empty, skipping startup scene load.")
        return
 
    wait_updates_raw = os.environ.get("ISAAC_STARTUP_SCENE_WAIT_UPDATES", "5")
    try:
        wait_updates = max(0, int(wait_updates_raw))
    except ValueError:
        wait_updates = 5
 
    app = omni.kit.app.get_app()
    usd_context = omni.usd.get_context()
 
    for _ in range(wait_updates):
        await app.next_update_async()
 
    carb.log_warn(f"Startup scene load requested: {scene_path}")
    result, err = await usd_context.open_stage_async(scene_path)
 
    if result:
        carb.log_warn(f"Startup scene load succeeded: {usd_context.get_stage_url()}")
    else:
        carb.log_error(f"Startup scene load failed for '{scene_path}': {err}")
 
 
asyncio.ensure_future(_open_scene_on_startup())