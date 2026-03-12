"""Pure-math sphere-grid builder for kinematic object attachment.

Stateless module-level functions extracted from cumotion_attachment_ops.py.
Builds collision-sphere grids from primitive geometry (box, cylinder, sphere)
and transforms them into a target frame for cuRobo kinematic attachment.
"""

import math


# ---------------------------------------------------------------------------
# Quaternion helpers
# ---------------------------------------------------------------------------

def quaternion_multiply(q1, q2):
    """Multiply two quaternions (x, y, z, w)."""
    x1, y1, z1, w1 = q1
    x2, y2, z2, w2 = q2
    return (
        w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
        w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
        w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
        w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
    )


def quaternion_conjugate(q):
    """Conjugate of quaternion (x, y, z, w)."""
    return (-q[0], -q[1], -q[2], q[3])


def rotate_vector(vec, quat):
    """Rotate a 3-vector by quaternion (x, y, z, w)."""
    v_quat = (vec[0], vec[1], vec[2], 0.0)
    rotated = quaternion_multiply(
        quaternion_multiply(quat, v_quat),
        quaternion_conjugate(quat),
    )
    return (rotated[0], rotated[1], rotated[2])


def normalize_quaternion(qx, qy, qz, qw):
    norm = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
    if norm <= 1e-9:
        return (0.0, 0.0, 0.0, 1.0)
    inv = 1.0 / norm
    return (qx * inv, qy * inv, qz * inv, qw * inv)


# ---------------------------------------------------------------------------
# Grid helpers
# ---------------------------------------------------------------------------

def axis_centers(extent, divisions, center_offset=0.0):
    """Evenly-spaced grid centers along one axis."""
    safe_extent = max(1e-6, float(extent))
    safe_divisions = max(1, int(divisions))
    if safe_divisions == 1:
        return [float(center_offset)], safe_extent
    step = safe_extent / float(safe_divisions)
    start = (-0.5 * safe_extent) + (0.5 * step) + float(center_offset)
    return [float(start + i * step) for i in range(safe_divisions)], float(step)


def cap_grid_counts(nx, ny, nz, max_cells):
    """Limit total grid cells to max_cells by reducing largest axis first."""
    cx = max(1, int(nx))
    cy = max(1, int(ny))
    cz = max(1, int(nz))
    cap = max(1, int(max_cells))
    while (cx * cy * cz) > cap:
        if cx >= cy and cx >= cz and cx > 1:
            cx -= 1
            continue
        if cy >= cx and cy >= cz and cy > 1:
            cy -= 1
            continue
        if cz > 1:
            cz -= 1
            continue
        break
    return cx, cy, cz


# ---------------------------------------------------------------------------
# Primitive sphere builders
# ---------------------------------------------------------------------------

def _preclamp_budget(max_spheres):
    return max(64, max(1, int(max_spheres)) * 6)


def build_box_spheres(size_xyz, target_diameter, inflation, max_spheres=32,
                      center_offset=(0.0, 0.0, 0.0)):
    """Fill a box with a sphere grid. Returns list of (x, y, z, radius)."""
    sx = max(1e-6, float(size_xyz[0]))
    sy = max(1e-6, float(size_xyz[1]))
    sz = max(1e-6, float(size_xyz[2]))
    target = max(1e-4, float(target_diameter))
    nx = max(1, int(math.ceil(sx / target)))
    ny = max(1, int(math.ceil(sy / target)))
    nz = max(1, int(math.ceil(sz / target)))
    nx, ny, nz = cap_grid_counts(nx, ny, nz, _preclamp_budget(max_spheres))

    cx, cy, cz = center_offset
    xs, dx = axis_centers(sx, nx, center_offset=cx)
    ys, dy = axis_centers(sy, ny, center_offset=cy)
    zs, dz = axis_centers(sz, nz, center_offset=cz)
    radius = (
        math.sqrt((0.5 * dx) ** 2 + (0.5 * dy) ** 2 + (0.5 * dz) ** 2)
        + float(inflation)
    )
    spheres = []
    for x in xs:
        for y in ys:
            for z in zs:
                spheres.append((float(x), float(y), float(z), float(radius)))
    return spheres


def build_cylinder_spheres(height, radius, target_diameter, inflation,
                           max_spheres=32):
    """Fill a cylinder with a sphere grid."""
    h = max(1e-6, float(height))
    r = max(1e-6, float(radius))
    target = max(1e-4, float(target_diameter))
    dx_extent = 2.0 * r
    dy_extent = 2.0 * r
    dz_extent = h
    nx = max(1, int(math.ceil(dx_extent / target)))
    ny = max(1, int(math.ceil(dy_extent / target)))
    nz = max(1, int(math.ceil(dz_extent / target)))
    nx, ny, nz = cap_grid_counts(nx, ny, nz, _preclamp_budget(max_spheres))
    xs, dx = axis_centers(dx_extent, nx)
    ys, dy = axis_centers(dy_extent, ny)
    zs, dz = axis_centers(dz_extent, nz)
    cell_half_diag_xy = math.sqrt((0.5 * dx) ** 2 + (0.5 * dy) ** 2)
    radial_limit = max(0.0, r - cell_half_diag_xy)
    radius_out = (
        math.sqrt((0.5 * dx) ** 2 + (0.5 * dy) ** 2 + (0.5 * dz) ** 2)
        + float(inflation)
    )
    spheres = []
    for x in xs:
        for y in ys:
            if (x * x + y * y) > (radial_limit * radial_limit):
                continue
            for z in zs:
                spheres.append((float(x), float(y), float(z), float(radius_out)))
    if not spheres:
        spheres.append((0.0, 0.0, 0.0, float(max(r, 0.5 * h) + float(inflation))))
    return spheres


def build_sphere_spheres(radius, target_diameter, inflation, max_spheres=32):
    """Fill a sphere with a sphere grid."""
    r = max(1e-6, float(radius))
    target = max(1e-4, float(target_diameter))
    diameter = 2.0 * r
    n = max(1, int(math.ceil(diameter / target)))
    n, _, _ = cap_grid_counts(n, n, n, _preclamp_budget(max_spheres))
    xs, dx = axis_centers(diameter, n)
    ys, dy = axis_centers(diameter, n)
    zs, dz = axis_centers(diameter, n)
    cell_half_diag = math.sqrt((0.5 * dx) ** 2 + (0.5 * dy) ** 2 + (0.5 * dz) ** 2)
    if n == 1 or cell_half_diag >= r:
        return [(0.0, 0.0, 0.0, float(r + float(inflation)))]
    inscribed_limit = max(0.0, r - cell_half_diag)
    radius_out = float(cell_half_diag + float(inflation))
    spheres = []
    for x in xs:
        for y in ys:
            for z in zs:
                if (x * x + y * y + z * z) <= (inscribed_limit * inscribed_limit):
                    spheres.append((float(x), float(y), float(z), radius_out))
    if not spheres:
        spheres.append((0.0, 0.0, 0.0, float(r + float(inflation))))
    return spheres


# ---------------------------------------------------------------------------
# URDF collision parsing
# ---------------------------------------------------------------------------

def load_collision_specs_from_urdf(urdf_path):
    """Parse URDF collision geometry into sphere_builder-compatible specs.

    Returns list of dicts: {type: "box"|"cylinder"|"sphere", dimensions: tuple, pose: 7-tuple}
    The pose is (x, y, z, qx, qy, qz, qw) from the <origin> tag.
    """
    import xml.etree.ElementTree as ET

    try:
        root = ET.parse(urdf_path).getroot()
    except Exception:
        return []

    specs = []
    for collision_tag in root.findall(".//link/collision"):
        origin = collision_tag.find("origin")
        pose = _parse_origin_tuple(origin)
        geometry_tag = collision_tag.find("geometry")
        if geometry_tag is None:
            continue

        box_tag = geometry_tag.find("box")
        if box_tag is not None:
            size = _parse_float_list(box_tag.get("size", ""), 3)
            if size is not None:
                specs.append({
                    "type": "box",
                    "dimensions": (size[0], size[1], size[2]),
                    "pose": pose,
                })
            continue

        cylinder_tag = geometry_tag.find("cylinder")
        if cylinder_tag is not None:
            try:
                r = float(cylinder_tag.get("radius", "0"))
                h = float(cylinder_tag.get("length", "0"))
                specs.append({
                    "type": "cylinder",
                    "dimensions": (h, r),
                    "pose": pose,
                })
            except ValueError:
                pass
            continue

        sphere_tag = geometry_tag.find("sphere")
        if sphere_tag is not None:
            try:
                r = float(sphere_tag.get("radius", "0"))
                specs.append({
                    "type": "sphere",
                    "dimensions": (r,),
                    "pose": pose,
                })
            except ValueError:
                pass
            continue

        mesh_tag = geometry_tag.find("mesh")
        if mesh_tag is not None:
            mesh_spec = _mesh_to_box_spec(mesh_tag, urdf_path, pose)
            if mesh_spec is not None:
                specs.append(mesh_spec)

    return specs


def _parse_float_list(raw, expected_len):
    parts = str(raw).split()
    values = []
    for token in parts:
        try:
            values.append(float(token))
        except ValueError:
            continue
    if len(values) < expected_len:
        return None
    return values


def _parse_origin_tuple(origin_tag):
    """Parse <origin xyz="..." rpy="..."> into (x, y, z, qx, qy, qz, qw)."""
    if origin_tag is None:
        return (0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0)

    xyz = _parse_float_list(origin_tag.get("xyz", "0 0 0"), 3) or [0.0, 0.0, 0.0]
    rpy = _parse_float_list(origin_tag.get("rpy", "0 0 0"), 3) or [0.0, 0.0, 0.0]
    qx, qy, qz, qw = _rpy_to_quaternion(rpy[0], rpy[1], rpy[2])
    return (xyz[0], xyz[1], xyz[2], qx, qy, qz, qw)


def _rpy_to_quaternion(roll, pitch, yaw):
    """Convert roll-pitch-yaw (xyz convention) to quaternion (x, y, z, w)."""
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    return (
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    )


def _mesh_to_box_spec(mesh_tag, urdf_path, pose):
    """Approximate a mesh collision as a box using AABB via trimesh."""
    import os

    try:
        import trimesh
    except ImportError:
        return None

    mesh_filename = mesh_tag.get("filename", "").split("/")[-1]
    if not mesh_filename:
        return None

    folder_path = os.path.dirname(urdf_path)
    mesh_path = os.path.join(folder_path, "meshes", mesh_filename)
    if not os.path.exists(mesh_path):
        return None

    scale = _parse_float_list(mesh_tag.get("scale", "1 1 1"), 3) or [1.0, 1.0, 1.0]
    try:
        mesh = trimesh.load(mesh_path, force="mesh", process=False)
        if isinstance(mesh, trimesh.Scene):
            mesh = trimesh.util.concatenate(tuple(mesh.geometry.values()))
        mesh = mesh.copy()
        mesh.apply_scale(scale)
        bounds = mesh.bounds  # [[min_x, min_y, min_z], [max_x, max_y, max_z]]
        size = bounds[1] - bounds[0]
        center = (bounds[0] + bounds[1]) * 0.5
    except Exception:
        return None

    # Compose the mesh center offset with the collision origin pose
    center_pose = (float(center[0]), float(center[1]), float(center[2]),
                   0.0, 0.0, 0.0, 1.0)
    composed = compose_pose_tuples(pose, center_pose)
    return {
        "type": "box",
        "dimensions": (max(1e-6, float(size[0])),
                       max(1e-6, float(size[1])),
                       max(1e-6, float(size[2]))),
        "pose": composed,
    }


# ---------------------------------------------------------------------------
# Transform helpers
# ---------------------------------------------------------------------------

def compose_pose_tuples(parent, child):
    """Compose two 7-tuple poses (x, y, z, qx, qy, qz, qw)."""
    px, py, pz, pqx, pqy, pqz, pqw = parent
    cx, cy, cz, cqx, cqy, cqz, cqw = child
    pqx, pqy, pqz, pqw = normalize_quaternion(pqx, pqy, pqz, pqw)
    cqx, cqy, cqz, cqw = normalize_quaternion(cqx, cqy, cqz, cqw)
    rx, ry, rz = rotate_vector((cx, cy, cz), (pqx, pqy, pqz, pqw))
    oqx, oqy, oqz, oqw = normalize_quaternion(
        *quaternion_multiply((pqx, pqy, pqz, pqw), (cqx, cqy, cqz, cqw))
    )
    return (
        float(px + rx), float(py + ry), float(pz + rz),
        float(oqx), float(oqy), float(oqz), float(oqw),
    )


def transform_spheres_to_frame(pose_tuple, local_spheres):
    """Rotate+translate local spheres by pose (x, y, z, qx, qy, qz, qw)."""
    px, py, pz, qx, qy, qz, qw = pose_tuple
    qx, qy, qz, qw = normalize_quaternion(qx, qy, qz, qw)
    transformed = []
    for lx, ly, lz, radius in local_spheres:
        rx, ry, rz = rotate_vector((lx, ly, lz), (qx, qy, qz, qw))
        transformed.append((
            float(px + rx), float(py + ry), float(pz + rz), float(radius),
        ))
    return transformed


def downsample_spheres(spheres, target_count):
    """Stride-based deterministic downsampling."""
    if len(spheres) <= target_count:
        return spheres
    step = float(len(spheres)) / float(target_count)
    return [spheres[int(i * step)] for i in range(int(target_count))]


# ---------------------------------------------------------------------------
# High-level entry point
# ---------------------------------------------------------------------------

def build_attachment_spheres(collision_specs, object_pose_in_attach_frame,
                             target_diameter, inflation, min_spheres, max_spheres):
    """Build adaptive sphere grid from collision geometry specs.

    Args:
        collision_specs: list of dicts {type: "box"|"cylinder"|"sphere",
                                        dimensions: tuple, pose: 7-tuple}
        object_pose_in_attach_frame: 7-tuple (x, y, z, qx, qy, qz, qw)
        target_diameter: target sphere diameter in meters
        inflation: collision inflation in meters
        min_spheres: minimum number of spheres to attempt
        max_spheres: maximum number of output spheres

    Returns:
        list of (x, y, z, radius) in attach-frame coordinates, or None on failure.
    """
    min_spheres = max(1, int(min_spheres))
    max_spheres = max(min_spheres, int(max_spheres))
    diameter_floor = min(0.005, float(target_diameter))
    object_pose = tuple(float(v) for v in object_pose_in_attach_frame)
    spheres = []

    for _ in range(6):
        spheres = _build_once(collision_specs, object_pose, target_diameter,
                              inflation, max_spheres)
        if len(spheres) >= min_spheres or target_diameter <= diameter_floor:
            break
        target_diameter *= 0.7

    if not spheres:
        return None

    if len(spheres) > max_spheres:
        spheres = downsample_spheres(spheres, max_spheres)

    return spheres


def _build_once(collision_specs, object_pose, target_diameter, inflation,
                max_spheres):
    """Build spheres from all collision specs at a single diameter."""
    spheres = []
    for spec in collision_specs:
        spec_pose = tuple(float(v) for v in spec["pose"])
        shape_pose = compose_pose_tuples(object_pose, spec_pose)
        local = []
        stype = spec["type"]
        dims = spec["dimensions"]

        if stype == "box":
            local = build_box_spheres(dims, target_diameter, inflation,
                                      max_spheres)
        elif stype == "cylinder":
            local = build_cylinder_spheres(dims[0], dims[1], target_diameter,
                                           inflation, max_spheres)
        elif stype == "sphere":
            local = build_sphere_spheres(dims[0], target_diameter, inflation,
                                         max_spheres)

        if local:
            spheres.extend(transform_spheres_to_frame(shape_pose, local))

    return spheres
