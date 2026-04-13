#!/usr/bin/env python3
"""Generate a compound convex-hull triangle mesh .ply from a 3DGS splat .ply.

The input splats are first filtered by opacity (dropping low-alpha
floaters) and then partitioned into N horizontal slabs along the up axis.
A convex hull is computed per slab and all slab hulls are merged into a
single triangle-mesh .ply as disconnected components. The output is
intended as a multi-shell collision proxy for 3dgrut's
``threedgrut.export.scripts.add_mesh_to_usdz``.

A single convex hull cannot tightly wrap a non-convex shape like a
lantern: a pointed finial above a wider body forces the hull to trap a
big empty cone between the finial tip and the top edges of the body.
Stacking a few per-slab hulls along the up axis gives a much tighter fit
at the cost of a few extra triangles, with no new dependencies beyond
scipy + plyfile + numpy.

Only uses ``plyfile``, ``numpy`` and ``scipy`` so it can run inside the
existing ``rita-3dgrut`` conda environment without extra packages.
"""
from __future__ import annotations

import argparse

import numpy as np
from plyfile import PlyData, PlyElement
from scipy.spatial import ConvexHull, QhullError

# Tuning knobs — hardcoded on purpose (these are collision-proxy
# parameters tuned for the RITA gaussian-splat lamp workflow, not
# user-facing). Revisit if another splat asset behaves differently.

# Minimum post-sigmoid opacity for a splat to survive the filter.
# Chosen from a diagnostic on lamp_edited.ply: alpha >= 0.1 drops ~35%
# of splats — all of which are near-invisible floaters that still bloat
# a convex hull — while preserving every visually-meaningful Gaussian.
# Values below 0.05 leave too many floaters; values above 0.2 start
# eating actual body splats.
ALPHA_THRESHOLD = 0.1

# Number of horizontal convex-hull slabs along the up axis. Five gives a
# decent decomposition for a lantern-shaped object (base / lower body /
# upper body / neck / finial) without exploding the triangle count.
NUM_SLABS = 5

# Up-axis index in the PLY. The USDZ we produce sets upAxis = "Z", which
# matches the lamp's vertical orientation, so we slab along Z.
UP_AXIS = 2  # 0 = X, 1 = Y, 2 = Z


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("input_ply", help="Input 3DGS splat .ply")
    parser.add_argument("output_ply", help="Output triangle-mesh .ply")
    args = parser.parse_args()

    plydata = PlyData.read(args.input_ply)
    vertices = plydata["vertex"]
    xyz = np.stack(
        [vertices["x"], vertices["y"], vertices["z"]], axis=1
    ).astype(np.float32)
    opa_logit = np.asarray(vertices["opacity"], dtype=np.float32)

    # --- Step A: opacity filter (drop low-alpha floaters) -----------
    alpha = 1.0 / (1.0 + np.exp(-opa_logit))
    keep = alpha >= ALPHA_THRESHOLD
    n_total = int(keep.size)
    xyz = xyz[keep]
    n_kept = int(xyz.shape[0])
    print(
        f"opacity filter (alpha >= {ALPHA_THRESHOLD}): "
        f"kept {n_kept}/{n_total} splats "
        f"({100 * n_kept / n_total:.1f}%)"
    )
    if n_kept < NUM_SLABS * 4:
        raise SystemExit(
            f"too few splats after opacity filtering ({n_kept}) to form "
            f"{NUM_SLABS} 3D convex hulls"
        )

    # --- Step B: equal-count slabs along the up axis -----------------
    # Sorting by the up-axis and then cutting at N evenly-spaced index
    # positions gives each slab the same splat count. That auto-
    # concentrates resolution where the capture has detail (dense body
    # slabs are thin in Z; sparse finial / base slabs are wider).
    order = np.argsort(xyz[:, UP_AXIS], kind="stable")
    cut_edges = np.linspace(0, n_kept, NUM_SLABS + 1, dtype=np.int64)

    all_verts: list[np.ndarray] = []
    all_faces: list[np.ndarray] = []
    vert_offset = 0

    for i in range(NUM_SLABS):
        slab_idx = order[cut_edges[i] : cut_edges[i + 1]]
        slab = xyz[slab_idx]
        if slab.shape[0] < 4:
            print(f"  slab {i}: skipped (only {slab.shape[0]} pts)")
            continue

        try:
            hull = ConvexHull(slab)
        except QhullError as exc:
            # Co-planar / degenerate slabs can occasionally trip qhull.
            # Skipping is safe because the other slabs still cover the
            # shape; adding "QJ" joggle would mask a more serious issue.
            print(f"  slab {i}: skipped (qhull error: {exc})")
            continue

        kept_in_slab = hull.vertices
        slab_verts = slab[kept_in_slab]

        remap = np.full(slab.shape[0], -1, dtype=np.int32)
        remap[kept_in_slab] = np.arange(
            kept_in_slab.shape[0], dtype=np.int32
        )
        slab_faces = remap[hull.simplices].astype(np.int32) + vert_offset

        mn = slab.min(0)
        mx = slab.max(0)
        sz = mx - mn
        print(
            f"  slab {i}: {slab.shape[0]:>6} pts  "
            f"-> {slab_verts.shape[0]:>3}v / {slab_faces.shape[0]:>3}f  "
            f"(Z ∈ [{mn[UP_AXIS]:+.3f}, {mx[UP_AXIS]:+.3f}], "
            f"bbox {sz[0]:.2f}×{sz[1]:.2f}×{sz[2]:.2f})"
        )

        all_verts.append(slab_verts)
        all_faces.append(slab_faces)
        vert_offset += slab_verts.shape[0]

    if not all_verts:
        raise SystemExit("no valid slab hulls were produced")

    verts_xyz = np.concatenate(all_verts, axis=0)
    faces = np.concatenate(all_faces, axis=0)

    vert_array = np.empty(
        verts_xyz.shape[0],
        dtype=[("x", "f4"), ("y", "f4"), ("z", "f4")],
    )
    vert_array["x"] = verts_xyz[:, 0]
    vert_array["y"] = verts_xyz[:, 1]
    vert_array["z"] = verts_xyz[:, 2]

    face_array = np.empty(
        faces.shape[0], dtype=[("vertex_indices", "i4", (3,))]
    )
    face_array["vertex_indices"] = faces

    PlyData(
        [
            PlyElement.describe(vert_array, "vertex"),
            PlyElement.describe(face_array, "face"),
        ],
        text=False,
    ).write(args.output_ply)

    print(
        f"wrote {args.output_ply}: {verts_xyz.shape[0]} vertices, "
        f"{faces.shape[0]} faces across {NUM_SLABS} slabs"
    )


if __name__ == "__main__":
    main()
