"""
urdf_exporter.py
Exports a simplified URDF by patching the original file.

Strategy
--------
1. Parse the original URDF XML with ElementTree, preserving all content.
2. For every <link> that has a collision mesh object in the Blender scene:
   a. Remove all existing <collision> child elements from that link.
   b. Export the Blender collision mesh to simplified_urdf/meshes/<link>_collision.stl
   c. Insert a fresh <collision> element pointing to the new STL.
3. Links with no Blender collision object are left completely untouched
   (their original <collision> tags stay as-is).
4. Write the patched XML to simplified_urdf/urdf/<robot_name>.urdf

Output layout
-------------
  <export_base>/
    simplified_urdf/
      urdf/
        <robot_name>.urdf
      meshes/
        <link_name>_collision.stl
        ...
"""

import bpy
import bmesh
import mathutils
import xml.etree.ElementTree as ET
import struct
import os


# ---------------------------------------------------------------------------
# STL writer (binary, no Blender operator — reliable, no selection needed)
# ---------------------------------------------------------------------------

def _write_stl(mesh_data: bpy.types.Mesh, filepath: str):
    """
    Write a Blender mesh (with modifiers already evaluated) to a binary STL.
    Vertices are written as-is (already in metres after our import bake).
    """
    bm = bmesh.new()
    bm.from_mesh(mesh_data)
    bmesh.ops.triangulate(bm, faces=bm.faces)

    with open(filepath, 'wb') as f:
        f.write(b'\0' * 80)                          # header
        f.write(struct.pack('<I', len(bm.faces)))     # triangle count
        for face in bm.faces:
            n = face.normal
            f.write(struct.pack('<fff', n.x, n.y, n.z))
            for v in face.verts:
                f.write(struct.pack('<fff', v.co.x, v.co.y, v.co.z))
            f.write(struct.pack('<H', 0))             # attribute byte count

    bm.free()


def _evaluated_mesh(obj: bpy.types.Object) -> bpy.types.Mesh:
    """Return a mesh with all modifiers (Decimate etc.) applied."""
    depsgraph = bpy.context.evaluated_depsgraph_get()
    return bpy.data.meshes.new_from_object(obj.evaluated_get(depsgraph))


# ---------------------------------------------------------------------------
# Collect objects from scene
# ---------------------------------------------------------------------------

def _collect_collision_objects() -> dict:
    """Return {link_name: obj} for every mesh tagged urdf_role=collision."""
    result = {}
    for obj in bpy.data.objects:
        if obj.type != 'MESH':
            continue
        if obj.get("urdf_role") != "collision":
            continue
        link = obj.get("urdf_link", "")
        if not link:
            continue
        if link not in result:
            result[link] = obj
    return result


def _collect_joint_limits() -> dict:
    """
    Return {joint_name: {lower, upper, velocity, effort}} for every
    joint-frame empty that has limit custom properties.
    """
    result = {}
    for obj in bpy.data.objects:
        if obj.type != 'EMPTY':
            continue
        jname = obj.get("urdf_joint_name", "")
        if not jname:
            continue
        if "urdf_lower" not in obj and "urdf_upper" not in obj:
            continue
        result[jname] = {
            "lower":    float(obj.get("urdf_lower",    0.0)),
            "upper":    float(obj.get("urdf_upper",    0.0)),
            "velocity": float(obj.get("urdf_velocity", 0.0)),
            "effort":   float(obj.get("urdf_effort",   0.0)),
        }
    return result


# ---------------------------------------------------------------------------
# XML helpers
# ---------------------------------------------------------------------------

def _remove_children_named(parent: ET.Element, tag: str):
    for child in list(parent):
        if child.tag == tag:
            parent.remove(child)


def _make_collision_element_uri(mesh_uri: str) -> ET.Element:
    """
    Build a <collision> element with the given full mesh URI:
      <collision>
        <origin xyz="0 0 0" rpy="0 0 0"/>
        <geometry>
          <mesh filename="<mesh_uri>"/>
        </geometry>
      </collision>
    """
    col_el  = ET.Element("collision")
    orig_el = ET.SubElement(col_el, "origin")
    orig_el.set("xyz", "0 0 0")
    orig_el.set("rpy", "0 0 0")
    geo_el  = ET.SubElement(col_el, "geometry")
    mesh_el = ET.SubElement(geo_el, "mesh")
    mesh_el.set("filename", mesh_uri)
    return col_el


def _indent_xml(elem, level=0):
    """In-place pretty-print indentation."""
    pad = "\n" + "  " * level
    if len(elem):
        if not elem.text or not elem.text.strip():
            elem.text = pad + "  "
        if not elem.tail or not elem.tail.strip():
            elem.tail = pad
        for child in elem:
            _indent_xml(child, level + 1)
        # last child tail
        if not child.tail or not child.tail.strip():
            child.tail = pad
    else:
        if level and (not elem.tail or not elem.tail.strip()):
            elem.tail = pad


# ---------------------------------------------------------------------------
# Main export function
# ---------------------------------------------------------------------------

def export_urdf(source_urdf_path: str, export_base: str, package_name: str) -> str:
    """
    Patch the original URDF, replacing <collision> tags for links that have
    a simplified collision mesh in the Blender scene.

    Parameters
    ----------
    source_urdf_path : str
        Absolute path to the original .urdf file.
    export_base : str
        Directory under which simplified_urdf/ will be created.
    package_name : str
        ROS package name used in package:// URIs (usually the robot name).

    Returns
    -------
    str  Path to the written .urdf file.
    """
    if not os.path.isfile(source_urdf_path):
        raise FileNotFoundError(f"Source URDF not found: {source_urdf_path}")

    # Output directories
    out_root   = os.path.join(export_base, "simplified_urdf")
    out_urdf   = os.path.join(out_root, "urdf")
    out_meshes = os.path.join(out_root, "meshes")
    os.makedirs(out_urdf,   exist_ok=True)
    os.makedirs(out_meshes, exist_ok=True)

    # Parse original URDF
    ET.register_namespace('', '')   # avoid ns0: prefixes
    tree = ET.parse(source_urdf_path)
    root = tree.getroot()

    # Collect Blender collision objects and limit overrides
    col_objs    = _collect_collision_objects()
    limit_overrides = _collect_joint_limits()
    print(f"[URDF Export] Collision objects found: {list(col_objs.keys())}")
    print(f"[URDF Export] Limit overrides: {list(limit_overrides.keys())}")

    # --- Patch <limit> elements on joints whose limits were edited in Blender ---
    for jnt_el in root.findall("joint"):
        jname = jnt_el.get("name", "")
        if jname not in limit_overrides:
            continue
        lim_el = jnt_el.find("limit")
        if lim_el is None:
            continue   # don't add limits that weren't there originally
        ov = limit_overrides[jname]
        lim_el.set("lower",    f"{ov['lower']:.6f}")
        lim_el.set("upper",    f"{ov['upper']:.6f}")
        lim_el.set("velocity", f"{ov['velocity']:.6f}")
        lim_el.set("effort",   f"{ov['effort']:.6f}")
        print(f"[URDF Export]   limit patched: {jname}")

    patched = []
    skipped = []

    for link_el in root.findall("link"):
        link_name = link_el.get("name", "")
        if link_name not in col_objs:
            skipped.append(link_name)
            continue

        obj = col_objs[link_name]

        # --- Derive the package:// URI from the original visual mesh path ---
        # Find the first <visual><geometry><mesh filename="..."> in this link.
        # Use its package:// path as the template, replacing only the filename.
        original_pkg_uri = None
        vis_mesh = link_el.find("visual/geometry/mesh")
        if vis_mesh is not None:
            original_pkg_uri = vis_mesh.get("filename", "")

        stl_filename = f"{link_name}_collision.stl"

        if original_pkg_uri and original_pkg_uri.startswith("package://"):
            # e.g. "package://robot_description/meshes/x_link.STL"
            # → keep everything up to and including the last "/" then swap filename
            prefix = original_pkg_uri.rsplit("/", 1)[0]   # "package://robot_description/meshes"
            mesh_uri = f"{prefix}/{stl_filename}"
        else:
            # Fallback: build URI from package_name as before
            mesh_uri = f"package://{package_name}/meshes/{stl_filename}"

        # Resolve the physical output path from the URI
        # Strip "package://<pkg>/" and join with out_meshes
        if mesh_uri.startswith("package://"):
            rel = mesh_uri[len("package://"):].split("/", 1)[-1]  # "meshes/x_link_collision.stl"
            stl_path = os.path.join(out_root, rel)
            os.makedirs(os.path.dirname(stl_path), exist_ok=True)
        else:
            stl_path = os.path.join(out_meshes, stl_filename)

        # Export the simplified mesh to STL
        mesh_data = _evaluated_mesh(obj)
        _write_stl(mesh_data, stl_path)
        bpy.data.meshes.remove(mesh_data)
        print(f"[URDF Export]   {link_name} → {stl_path}")

        # Remove ALL existing <collision> children from this link
        _remove_children_named(link_el, "collision")

        # Insert new <collision> pointing to the simplified STL
        col_el = _make_collision_element_uri(mesh_uri)
        link_el.append(col_el)

        patched.append(link_name)

    print(f"[URDF Export] Patched {len(patched)} links, "
          f"left {len(skipped)} links unchanged.")

    # Pretty-print and write
    _indent_xml(root)
    robot_name   = root.get("name", package_name)
    output_path  = os.path.join(out_urdf, f"{robot_name}.urdf")
    tree.write(output_path, encoding="unicode", xml_declaration=True)

    return output_path
