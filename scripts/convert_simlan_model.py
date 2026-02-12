#!/usr/bin/env python3
"""Convert a SIMLAN Gazebo model (SDF + mesh files) to USD for Isaac Sim.

Usage (inside Isaac Sim Docker container):
    /isaac-sim/python.sh /ros2_ws/scripts/convert_simlan_model.py \
        --sdf-dir /ros2_ws/path/to/robot_arm_beam/ \
        --output-dir /ros2_ws/assets/simlan_environment/robot_arm_beam/

The script expects the SIMLAN model directory layout:
    <model_dir>/
        model.sdf
        meshes/
            *.dae / *.stl

It produces:
    <output_dir>/
        meshes/<mesh_name>.usd    (one per source mesh)
        <model_name>.usd          (composed asset with physics)
"""

import argparse
import glob
import os
import re
import struct
import sys
import xml.etree.ElementTree as ET

# Auto-detect Isaac Sim's USD libs when running inside the container.
# The pxr module and its .so files live in an extscache directory that
# isn't on the default PYTHONPATH / LD_LIBRARY_PATH.
_ISAAC_SIM = os.environ.get("ISAAC_PATH", "/isaac-sim")
_usd_libs = glob.glob(os.path.join(_ISAAC_SIM, "extscache", "omni.usd.libs-*"))
if _usd_libs and "pxr" not in sys.modules:
    _usd_lib_dir = sorted(_usd_libs)[-1]
    if _usd_lib_dir not in sys.path:
        sys.path.insert(0, _usd_lib_dir)
    _candidate_lib_dirs = {
        os.path.join(_usd_lib_dir, "lib"),
        os.path.join(_usd_lib_dir, "bin"),
        _usd_lib_dir,
    }
    # Some Isaac Sim builds place USD .so files in nested directories.
    for _soname in ("libgf.so", "libtf.so", "libsdf.so"):
        for _match in glob.glob(os.path.join(_usd_lib_dir, "**", _soname), recursive=True):
            _candidate_lib_dirs.add(os.path.dirname(_match))

    _existing_lib_dirs = sorted(d for d in _candidate_lib_dirs if os.path.isdir(d))
    if _existing_lib_dirs:
        os.environ["LD_LIBRARY_PATH"] = (
            ":".join(_existing_lib_dirs) + ":" + os.environ.get("LD_LIBRARY_PATH", "")
        )
        # Preload core USD libs so dependent pxr modules resolve symbols.
        try:
            import ctypes
            for _lib_dir in _existing_lib_dirs:
                for _soname in ("libgf.so", "libtf.so"):
                    _full = os.path.join(_lib_dir, _soname)
                    if os.path.isfile(_full):
                        try:
                            ctypes.CDLL(_full)
                        except OSError:
                            pass
        except OSError:
            pass

from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics, UsdShade


# ---------------------------------------------------------------------------
# SDF parsing
# ---------------------------------------------------------------------------

def parse_sdf(sdf_path):
    """Extract model properties from a SIMLAN model.sdf file.

    Returns a dict with:
        name        - model name string
        is_static   - bool
        mass        - float (kg)
        inertia     - dict with ixx, ixy, ixz, iyy, iyz, izz
        com_pose    - (tx, ty, tz, rx, ry, rz) centre-of-mass pose
        links       - list of link dicts, each containing visuals and collisions
    """
    tree = ET.parse(sdf_path)
    root = tree.getroot()
    model = root.find("model")

    result = {
        "name": model.get("name"),
        "is_static": False,
        "mass": 0.0,
        "inertia": {},
        "com_pose": (0, 0, 0, 0, 0, 0),
        "links": [],
    }

    static_el = model.find("static")
    if static_el is not None and static_el.text.strip().lower() == "true":
        result["is_static"] = True

    for link in model.findall("link"):
        link_data = {"name": link.get("name"), "visuals": [], "collisions": []}

        # Inertial
        inertial = link.find("inertial")
        if inertial is not None:
            mass_el = inertial.find("mass")
            if mass_el is not None:
                result["mass"] = float(mass_el.text)
            pose_el = inertial.find("pose")
            if pose_el is not None:
                result["com_pose"] = tuple(float(v) for v in pose_el.text.split())
            inertia_el = inertial.find("inertia")
            if inertia_el is not None:
                for comp in ("ixx", "ixy", "ixz", "iyy", "iyz", "izz"):
                    el = inertia_el.find(comp)
                    if el is not None:
                        result["inertia"][comp] = float(el.text)

        # Visuals
        for vis in link.findall("visual"):
            vdata = _parse_geometry_element(vis)
            if vdata:
                link_data["visuals"].append(vdata)

        # Collisions
        for col in link.findall("collision"):
            cdata = _parse_geometry_element(col)
            if cdata:
                link_data["collisions"].append(cdata)

        result["links"].append(link_data)

    return result


def _parse_geometry_element(element):
    """Parse a <visual> or <collision> SDF element."""
    data = {"name": element.get("name"), "pose": (0, 0, 0, 0, 0, 0), "mesh_uri": None, "scale": (1, 1, 1)}

    pose_el = element.find("pose")
    if pose_el is not None:
        data["pose"] = tuple(float(v) for v in pose_el.text.split())

    mesh = element.find("geometry/mesh")
    if mesh is not None:
        uri_el = mesh.find("uri")
        if uri_el is not None:
            # Extract just the filename from model:// URIs
            uri = uri_el.text
            if "://" in uri:
                # model://robot_arm_beam/meshes/Body.dae -> meshes/Body.dae
                parts = uri.split("/")
                # Find 'meshes' in the path and take from there
                try:
                    idx = parts.index("meshes")
                    data["mesh_uri"] = "/".join(parts[idx:])
                except ValueError:
                    data["mesh_uri"] = parts[-1]
            else:
                data["mesh_uri"] = uri

        scale_el = mesh.find("scale")
        if scale_el is not None:
            data["scale"] = tuple(float(v) for v in scale_el.text.split())

    return data


# ---------------------------------------------------------------------------
# COLLADA (.dae) parsing
# ---------------------------------------------------------------------------

DAE_NS = {"c": "http://www.collada.org/2005/11/COLLADASchema"}


def parse_dae(dae_path):
    """Parse a COLLADA .dae file and extract mesh + material data.

    Returns a dict with:
        vertices     - list of (x, y, z) tuples
        normals      - list of (nx, ny, nz) tuples
        face_vertex_indices  - flat list of vertex indices per triangle
        face_normal_indices  - flat list of normal indices per triangle
        face_vertex_counts   - list of ints (3 for each triangle)
        diffuse_color        - (r, g, b) tuple or None
        up_axis              - 'Z_UP' or 'Y_UP'
    """
    tree = ET.parse(dae_path)
    root = tree.getroot()

    result = {
        "vertices": [],
        "normals": [],
        "face_vertex_indices": [],
        "face_normal_indices": [],
        "face_vertex_counts": [],
        "diffuse_color": None,
        "up_axis": "Z_UP",
    }

    # Up axis
    up_el = root.find("c:asset/c:up_axis", DAE_NS)
    if up_el is not None:
        result["up_axis"] = up_el.text.strip()

    # Material diffuse color
    for diffuse in root.findall(".//c:lambert/c:diffuse/c:color", DAE_NS):
        vals = [float(v) for v in diffuse.text.split()]
        result["diffuse_color"] = (vals[0], vals[1], vals[2])
        break

    # Geometry
    for geom in root.findall(".//c:geometry", DAE_NS):
        mesh = geom.find("c:mesh", DAE_NS)
        if mesh is None:
            continue

        # Build a map of source id -> float_array data
        sources = {}
        for source in mesh.findall("c:source", DAE_NS):
            sid = source.get("id")
            fa = source.find("c:float_array", DAE_NS)
            if fa is not None:
                sources[sid] = [float(v) for v in fa.text.split()]

        # Identify which source is positions vs normals via <input> semantics
        pos_source_id = None
        vertices_el = mesh.find("c:vertices", DAE_NS)
        if vertices_el is not None:
            for inp in vertices_el.findall("c:input", DAE_NS):
                if inp.get("semantic") == "POSITION":
                    pos_source_id = inp.get("source").lstrip("#")

        # Parse positions
        if pos_source_id and pos_source_id in sources:
            floats = sources[pos_source_id]
            result["vertices"] = [
                (floats[i], floats[i + 1], floats[i + 2])
                for i in range(0, len(floats), 3)
            ]

        # Parse triangles
        triangles = mesh.find("c:triangles", DAE_NS)
        if triangles is None:
            continue

        # Determine input offsets
        inputs = triangles.findall("c:input", DAE_NS)
        vertex_offset = None
        normal_offset = None
        normal_source_id = None
        stride = len(inputs)  # total number of inputs = stride in <p>

        for inp in inputs:
            sem = inp.get("semantic")
            off = int(inp.get("offset"))
            if sem == "VERTEX":
                vertex_offset = off
            elif sem == "NORMAL":
                normal_offset = off
                normal_source_id = inp.get("source").lstrip("#")

        # Parse normals
        if normal_source_id and normal_source_id in sources:
            floats = sources[normal_source_id]
            result["normals"] = [
                (floats[i], floats[i + 1], floats[i + 2])
                for i in range(0, len(floats), 3)
            ]

        # Parse index data from <p>
        p_el = triangles.find("c:p", DAE_NS)
        if p_el is None:
            continue

        indices = [int(v) for v in p_el.text.split()]
        tri_count = int(triangles.get("count"))

        for t in range(tri_count):
            for v in range(3):
                base = (t * 3 + v) * stride
                if vertex_offset is not None:
                    result["face_vertex_indices"].append(indices[base + vertex_offset])
                if normal_offset is not None:
                    result["face_normal_indices"].append(indices[base + normal_offset])
            result["face_vertex_counts"].append(3)

    return result


# ---------------------------------------------------------------------------
# STL parsing
# ---------------------------------------------------------------------------

def parse_stl(stl_path):
    """Parse an STL file (ASCII or binary) and extract mesh data."""
    with open(stl_path, "rb") as f:
        data = f.read()

    # Binary STL has exact length: 80-byte header + uint32 tri count + 50 bytes/tri
    if len(data) >= 84:
        tri_count = struct.unpack("<I", data[80:84])[0]
        expected_size = 84 + tri_count * 50
        if expected_size == len(data):
            return _parse_binary_stl(data)

    # Fallback to ASCII STL
    text = data.decode("utf-8", errors="ignore")
    if text.lstrip().startswith("solid"):
        return _parse_ascii_stl(text)

    raise ValueError(f"Unsupported or invalid STL file: {stl_path}")


def _new_mesh_result():
    return {
        "vertices": [],
        "normals": [],
        "face_vertex_indices": [],
        "face_normal_indices": [],
        "face_vertex_counts": [],
        "diffuse_color": None,
        "up_axis": "Z_UP",
    }


def _parse_binary_stl(data):
    result = _new_mesh_result()

    tri_count = struct.unpack("<I", data[80:84])[0]
    offset = 84

    for _ in range(tri_count):
        nx, ny, nz = struct.unpack("<fff", data[offset:offset + 12])
        offset += 12

        tri_vertices = []
        for _ in range(3):
            x, y, z = struct.unpack("<fff", data[offset:offset + 12])
            offset += 12
            tri_vertices.append((x, y, z))

        offset += 2  # attribute byte count

        normal_idx = len(result["normals"])
        result["normals"].append((nx, ny, nz))

        for v in tri_vertices:
            result["vertices"].append(v)
            result["face_vertex_indices"].append(len(result["vertices"]) - 1)
            result["face_normal_indices"].append(normal_idx)

        result["face_vertex_counts"].append(3)

    return result


def _parse_ascii_stl(text):
    result = _new_mesh_result()

    facet_re = re.compile(
        r"facet\s+normal\s+([^\s]+)\s+([^\s]+)\s+([^\s]+)\s+outer\s+loop\s+"
        r"vertex\s+([^\s]+)\s+([^\s]+)\s+([^\s]+)\s+"
        r"vertex\s+([^\s]+)\s+([^\s]+)\s+([^\s]+)\s+"
        r"vertex\s+([^\s]+)\s+([^\s]+)\s+([^\s]+)\s+"
        r"endloop\s+endfacet",
        re.IGNORECASE | re.MULTILINE,
    )

    for match in facet_re.finditer(text):
        nx, ny, nz = float(match.group(1)), float(match.group(2)), float(match.group(3))
        v1 = (float(match.group(4)), float(match.group(5)), float(match.group(6)))
        v2 = (float(match.group(7)), float(match.group(8)), float(match.group(9)))
        v3 = (float(match.group(10)), float(match.group(11)), float(match.group(12)))

        normal_idx = len(result["normals"])
        result["normals"].append((nx, ny, nz))

        for v in (v1, v2, v3):
            result["vertices"].append(v)
            result["face_vertex_indices"].append(len(result["vertices"]) - 1)
            result["face_normal_indices"].append(normal_idx)

        result["face_vertex_counts"].append(3)

    if not result["face_vertex_counts"]:
        raise ValueError("ASCII STL parsed with zero facets")

    return result


# ---------------------------------------------------------------------------
# USD generation
# ---------------------------------------------------------------------------

def create_mesh_usd(mesh_data, output_path):
    """Create a USD file containing the mesh geometry and material.

    Args:
        mesh_data: dict from parse_dae() or parse_stl()
        output_path: path to write the .usd file
    """
    stage = Usd.Stage.CreateNew(output_path)
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)

    # Root Xform
    mesh_name = os.path.splitext(os.path.basename(output_path))[0]
    mesh_prim_name = _sanitize_prim_name(mesh_name)
    root_path = Sdf.Path(f"/{mesh_prim_name}")
    root = UsdGeom.Xform.Define(stage, root_path)
    stage.SetDefaultPrim(root.GetPrim())

    # Mesh
    mesh_prim_path = root_path.AppendChild("Mesh")
    mesh = UsdGeom.Mesh.Define(stage, mesh_prim_path)

    # Points
    points = [Gf.Vec3f(*v) for v in mesh_data["vertices"]]
    mesh.GetPointsAttr().Set(points)

    # Face vertex counts and indices
    mesh.GetFaceVertexCountsAttr().Set(mesh_data["face_vertex_counts"])
    mesh.GetFaceVertexIndicesAttr().Set(mesh_data["face_vertex_indices"])

    # Normals (per face-vertex, i.e. "faceVarying")
    if mesh_data["normals"] and mesh_data["face_normal_indices"]:
        expanded_normals = [
            Gf.Vec3f(*mesh_data["normals"][ni])
            for ni in mesh_data["face_normal_indices"]
        ]
        mesh.GetNormalsAttr().Set(expanded_normals)
        mesh.SetNormalsInterpolation(UsdGeom.Tokens.faceVarying)

    # Subdivision scheme = none (polygonal mesh)
    mesh.GetSubdivisionSchemeAttr().Set("none")

    # Material
    if mesh_data["diffuse_color"]:
        mat_path = root_path.AppendChild("Material")
        material = UsdShade.Material.Define(stage, mat_path)

        shader_path = mat_path.AppendChild("PreviewSurface")
        shader = UsdShade.Shader.Define(stage, shader_path)
        shader.CreateIdAttr("UsdPreviewSurface")

        r, g, b = mesh_data["diffuse_color"]
        shader.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(r, g, b))
        shader.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(0.8)
        shader.CreateInput("metallic", Sdf.ValueTypeNames.Float).Set(0.0)

        shader.CreateOutput("surface", Sdf.ValueTypeNames.Token)
        material.CreateSurfaceOutput().ConnectToSource(shader.ConnectableAPI(), "surface")

        # Bind material to mesh
        UsdShade.MaterialBindingAPI.Apply(mesh.GetPrim())
        UsdShade.MaterialBindingAPI(mesh.GetPrim()).Bind(material)

    stage.GetRootLayer().Save()
    print(f"  Created mesh USD: {output_path}")


def create_model_usd(sdf_data, mesh_usd_paths, output_path):
    """Create the composed model USD with references to mesh USDs and physics.

    Args:
        sdf_data: dict from parse_sdf()
        mesh_usd_paths: dict mapping mesh_uri (e.g. 'meshes/Body.dae') to
                        relative USD path (e.g. './meshes/Body.usd')
        output_path: path to write the model .usd file
    """
    stage = Usd.Stage.CreateNew(output_path)
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)

    model_name = sdf_data["name"]
    root_path = Sdf.Path(f"/{model_name}")
    root_xform = UsdGeom.Xform.Define(stage, root_path)
    stage.SetDefaultPrim(root_xform.GetPrim())

    # Process each link's visuals and collisions
    for link in sdf_data["links"]:
        # Visuals
        for vis in link["visuals"]:
            if vis["mesh_uri"] is None:
                continue

            vis_name = _sanitize_prim_name(vis["name"])
            vis_path = root_path.AppendChild(vis_name)
            vis_xform = UsdGeom.Xform.Define(stage, vis_path)

            # Apply pose offset from SDF (translation part)
            tx, ty, tz = vis["pose"][0], vis["pose"][1], vis["pose"][2]
            if tx != 0 or ty != 0 or tz != 0:
                vis_xform.AddTranslateOp().Set(Gf.Vec3d(tx, ty, tz))

            # Apply rotation if present (SDF uses roll, pitch, yaw in radians)
            rx, ry, rz = vis["pose"][3], vis["pose"][4], vis["pose"][5]
            if rx != 0 or ry != 0 or rz != 0:
                import math
                vis_xform.AddRotateXYZOp().Set(Gf.Vec3f(
                    math.degrees(rx), math.degrees(ry), math.degrees(rz)
                ))

            # Reference the mesh USD
            rel_usd_path = mesh_usd_paths.get(vis["mesh_uri"])
            if rel_usd_path:
                vis_xform.GetPrim().GetReferences().AddReference(rel_usd_path)

        # Collisions - create collision geometry matching each visual
        for col in link["collisions"]:
            if col["mesh_uri"] is None:
                continue

            col_name = _sanitize_prim_name(col["name"])
            col_path = root_path.AppendChild(col_name)
            col_xform = UsdGeom.Xform.Define(stage, col_path)

            # Apply pose offset
            tx, ty, tz = col["pose"][0], col["pose"][1], col["pose"][2]
            if tx != 0 or ty != 0 or tz != 0:
                col_xform.AddTranslateOp().Set(Gf.Vec3d(tx, ty, tz))

            rx, ry, rz = col["pose"][3], col["pose"][4], col["pose"][5]
            if rx != 0 or ry != 0 or rz != 0:
                import math
                col_xform.AddRotateXYZOp().Set(Gf.Vec3f(
                    math.degrees(rx), math.degrees(ry), math.degrees(rz)
                ))

            # Reference the same mesh USD for collision geometry
            rel_usd_path = mesh_usd_paths.get(col["mesh_uri"])
            if rel_usd_path:
                col_xform.GetPrim().GetReferences().AddReference(rel_usd_path)

            # Apply collision APIs to the referenced mesh prim
            # The mesh is at <col_xform>/<mesh_root_name>/Mesh
            mesh_basename = os.path.splitext(os.path.basename(rel_usd_path))[0]
            mesh_root_prim_name = _sanitize_prim_name(mesh_basename)
            collision_mesh_path = col_path.AppendChild(mesh_root_prim_name).AppendChild("Mesh")
            collision_mesh_prim = stage.OverridePrim(collision_mesh_path)
            UsdPhysics.CollisionAPI.Apply(collision_mesh_prim)
            UsdPhysics.MeshCollisionAPI.Apply(collision_mesh_prim)
            collision_mesh_prim.CreateAttribute(
                "physics:approximation", Sdf.ValueTypeNames.Token
            ).Set("convexHull")

            # Make this collision geometry invisible (visuals are separate)
            col_xform.GetPrim().CreateAttribute(
                "purpose", Sdf.ValueTypeNames.Token
            ).Set("guide")

    # For static objects, apply a static collider at the root
    # (no RigidBodyAPI = PhysX treats it as static/kinematic ground)
    if sdf_data["is_static"]:
        # The collision APIs on the mesh prims above are sufficient.
        # No RigidBodyAPI means the object won't move.
        pass

    stage.GetRootLayer().Save()
    print(f"  Created model USD: {output_path}")


def _sanitize_prim_name(name):
    """Convert an SDF name to a valid USD prim name."""
    # USD prim names must start with a letter or underscore, and contain
    # only alphanumeric characters and underscores
    sanitized = re.sub(r"[^a-zA-Z0-9_]", "_", name)
    if sanitized and sanitized[0].isdigit():
        sanitized = "_" + sanitized
    return sanitized


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main():
    parser = argparse.ArgumentParser(
        description="Convert a SIMLAN Gazebo model (SDF + DAE/STL) to USD for Isaac Sim."
    )
    parser.add_argument(
        "--sdf-dir",
        required=True,
        help="Path to the SIMLAN model directory containing model.sdf and meshes/",
    )
    parser.add_argument(
        "--output-dir",
        required=True,
        help="Path to the output directory for generated USD files",
    )
    args = parser.parse_args()

    sdf_dir = os.path.abspath(args.sdf_dir)
    output_dir = os.path.abspath(args.output_dir)

    sdf_path = os.path.join(sdf_dir, "model.sdf")
    if not os.path.isfile(sdf_path):
        raise FileNotFoundError(f"model.sdf not found in {sdf_dir}")

    print(f"Parsing SDF: {sdf_path}")
    sdf_data = parse_sdf(sdf_path)
    print(f"  Model: {sdf_data['name']} (static={sdf_data['is_static']}, mass={sdf_data['mass']} kg)")

    # Collect all unique mesh URIs from visuals and collisions
    mesh_uris = set()
    for link in sdf_data["links"]:
        for vis in link["visuals"]:
            if vis["mesh_uri"]:
                mesh_uris.add(vis["mesh_uri"])
        for col in link["collisions"]:
            if col["mesh_uri"]:
                mesh_uris.add(col["mesh_uri"])

    # Convert each source mesh to USD
    os.makedirs(os.path.join(output_dir, "meshes"), exist_ok=True)
    mesh_usd_paths = {}  # mesh_uri -> relative USD path

    for mesh_uri in sorted(mesh_uris):
        mesh_path = os.path.join(sdf_dir, mesh_uri)
        if not os.path.isfile(mesh_path):
            print(f"  WARNING: Mesh file not found: {mesh_path}, skipping")
            continue

        ext = os.path.splitext(mesh_path)[1].lower()
        if ext == ".dae":
            print(f"Parsing DAE: {mesh_path}")
            mesh_data = parse_dae(mesh_path)
        elif ext == ".stl":
            print(f"Parsing STL: {mesh_path}")
            mesh_data = parse_stl(mesh_path)
        else:
            print(f"  WARNING: Unsupported mesh extension '{ext}' for {mesh_path}, skipping")
            continue

        print(f"  Vertices: {len(mesh_data['vertices'])}, "
              f"Triangles: {len(mesh_data['face_vertex_counts'])}, "
              f"Diffuse: {mesh_data['diffuse_color']}")

        # Output path: meshes/<name>.usd
        mesh_basename = os.path.splitext(os.path.basename(mesh_uri))[0]
        mesh_usd_filename = f"meshes/{mesh_basename}.usd"
        mesh_usd_abs = os.path.join(output_dir, mesh_usd_filename)

        create_mesh_usd(mesh_data, mesh_usd_abs)
        mesh_usd_paths[mesh_uri] = f"./{mesh_usd_filename}"

    # Create the composed model USD
    model_usd_path = os.path.join(output_dir, f"{sdf_data['name']}.usd")
    print(f"Creating model USD: {model_usd_path}")
    create_model_usd(sdf_data, mesh_usd_paths, model_usd_path)

    print("\nDone! Generated files:")
    for root, dirs, files in os.walk(output_dir):
        for f in sorted(files):
            if f.endswith(".usd"):
                print(f"  {os.path.join(root, f)}")


if __name__ == "__main__":
    main()
