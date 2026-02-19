#!/usr/bin/env python3
"""Create a scene composition USD that layers the existing UR10e scene
with Gazebo environment objects.

This avoids modifying the original ur10e_robotiq2f-140-topic_based.usd.
The new scene sublayers the original and adds references to environment objects.

Usage (inside Isaac Sim Docker container):
    /isaac-sim/python.sh /ros2_ws/scripts/create_scene_with_environment.py \
        --scene-dir /ros2_ws/assets/ur10e_robotiq2f-140/ \
        --base-scene ur10e_robotiq2f-140-topic_based.usd

Then open scene_with_environment.usd in Isaac Sim instead of the original.
"""

import argparse
import glob
import os
import sys

# Auto-detect Isaac Sim's USD libs when running inside the container.
_ISAAC_SIM = os.environ.get("ISAAC_PATH", "/isaac-sim")
_usd_libs = glob.glob(os.path.join(_ISAAC_SIM, "extscache", "omni.usd.libs-*"))
if _usd_libs and "pxr" not in sys.modules:
    _usd_lib_dir = sorted(_usd_libs)[-1]
    if _usd_lib_dir not in sys.path:
        sys.path.insert(0, _usd_lib_dir)

from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics


def enable_static_collisions(stage: Usd.Stage, root_path: Sdf.Path) -> int:
    """Apply collision APIs to all geometry prims under root_path."""
    root_prim = stage.GetPrimAtPath(root_path)
    if not root_prim:
        return 0

    collider_count = 0
    for prim in Usd.PrimRange(root_prim):
        if not prim.IsA(UsdGeom.Gprim):
            continue

        if not UsdPhysics.CollisionAPI(prim):
            UsdPhysics.CollisionAPI.Apply(prim)

        # For mesh geometry, force triangle-mesh collision approximation so the
        # static beam shape is respected by the robot.
        if prim.IsA(UsdGeom.Mesh):
            mesh_collision = UsdPhysics.MeshCollisionAPI.Apply(prim)
            mesh_collision.CreateApproximationAttr().Set("none")

        collider_count += 1

    return collider_count


def main():
    parser = argparse.ArgumentParser(
        description="Create a scene composition USD with Gazebo environment objects."
    )
    parser.add_argument(
        "--scene-dir",
        required=True,
        help="Path to the scene directory (e.g. assets/ur10e_robotiq2f-140/)",
    )
    parser.add_argument(
        "--base-scene",
        default="ur10e_robotiq2f-140-topic_based.usd",
        help="Filename of the base scene USD to sublayer (default: ur10e_robotiq2f-140-topic_based.usd)",
    )
    parser.add_argument(
        "--output",
        default="scene_with_environment.usd",
        help="Output filename (default: scene_with_environment.usd)",
    )
    args = parser.parse_args()

    scene_dir = os.path.abspath(args.scene_dir)
    output_path = os.path.join(scene_dir, args.output)

    print(f"Creating scene composition: {output_path}")
    print(f"  Base scene: {args.base_scene}")

    stage = Usd.Stage.CreateNew(output_path)
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)

    # Sublayer the existing scene so everything in it is inherited
    root_layer = stage.GetRootLayer()
    root_layer.subLayerPaths.append(f"./{args.base_scene}")

    # Add robot_arm_beam from Gazebo environment assets
    beam_path = Sdf.Path("/World/robot_arm_beam")
    beam_prim = stage.DefinePrim(beam_path, "Xform")
    beam_prim.GetReferences().AddReference(
        "../simlan_environment/robot_arm_beam/robot_arm_beam.usd"
    )

    # Position the beam relative to the robot (~2m in front along X)
    beam_xform = UsdGeom.Xformable(beam_prim)
    beam_xform.AddTranslateOp().Set(Gf.Vec3d(2.0, 0.0, 0.0))

    colliders = enable_static_collisions(stage, beam_path)
    print(f"  Added collision to {colliders} beam geometry prim(s)")

    # Add flow_rack from Gazebo environment assets
    flow_rack_path = Sdf.Path("/World/flow_rack")
    flow_rack_prim = stage.DefinePrim(flow_rack_path, "Xform")
    flow_rack_prim.GetReferences().AddReference(
        "../simlan_environment/flow_rack/flow_rack.usd"
    )

    # Position the flow rack near the robot workspace (tune as needed)
    flow_rack_xform = UsdGeom.Xformable(flow_rack_prim)
    flow_rack_xform.AddTranslateOp().Set(Gf.Vec3d(1.4, -0.8, 0.0))

    flow_rack_colliders = enable_static_collisions(stage, flow_rack_path)
    print(f"  Added collision to {flow_rack_colliders} flow_rack geometry prim(s)")

    # Add 5 stacking crates in a row beside each other.
    # Tune base position/spacing to suit your scene layout.
    crate_usd = "../simlan_environment/stacking_crate_isaac/stacking_crate_isaac.usd"
    crate_base_x = 1.4
    crate_base_y = -1.4
    crate_base_z = 0.0
    crate_spacing_y = 0.55

    total_crate_colliders = 0
    for i in range(5):
        crate_path = Sdf.Path(f"/World/stacking_crate_{i + 1}")
        crate_prim = stage.DefinePrim(crate_path, "Xform")
        crate_prim.GetReferences().AddReference(crate_usd)

        crate_xform = UsdGeom.Xformable(crate_prim)
        crate_xform.AddTranslateOp().Set(
            Gf.Vec3d(crate_base_x, crate_base_y + i * crate_spacing_y, crate_base_z)
        )

        total_crate_colliders += enable_static_collisions(stage, crate_path)

    print("  Added 5 stacking_crate_isaac instances")
    print(f"  Added collision to {total_crate_colliders} stacking_crate_isaac geometry prim(s)")

    root_layer.Save()
    print(f"  Created: {output_path}")
    print("\nTo use: open this file in Isaac Sim instead of the original scene.")
    print("The original scene is included via sublayer and remains unmodified.")


if __name__ == "__main__":
    main()
