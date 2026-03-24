"""
urdf_exporter.py
Exports a simplified URDF by patching the original file.

What is patched
---------------
- <collision> tags: replaced with new simplified STL meshes.
- <visual>   tags: replaced with exported copies of the Blender visual meshes.
- <limit>    tags: updated if limits were edited in the Blender panel.

Everything else (joints, inertial, comments, mimic, ...) is copied verbatim.

Output layout
-------------
  <source_urdf_dir>/
    simplified_urdf/
      urdf/
        <robot_name>.urdf
      meshes/
        <link>_visual.stl
        <link>_collision.stl
        ...
"""

import bpy
import bmesh
import xml.etree.ElementTree as ET
import struct
import os


# ---------------------------------------------------------------------------
# STL writer
# ---------------------------------------------------------------------------

def _write_stl(mesh_data: bpy.types.Mesh, filepath: str):
    bm = bmesh.new()
    bm.from_mesh(mesh_data)
    bmesh.ops.triangulate(bm, faces=bm.faces)
    with open(filepath, 'wb') as f:
        f.write(b'\0' * 80)
        f.write(struct.pack('<I', len(bm.faces)))
        for face in bm.faces:
            n = face.normal
            f.write(struct.pack('<fff', n.x, n.y, n.z))
            for v in face.verts:
                f.write(struct.pack('<fff', v.co.x, v.co.y, v.co.z))
            f.write(struct.pack('<H', 0))
    bm.free()


def _evaluated_mesh(obj: bpy.types.Object) -> bpy.types.Mesh:
    depsgraph = bpy.context.evaluated_depsgraph_get()
    return bpy.data.meshes.new_from_object(obj.evaluated_get(depsgraph))


# ---------------------------------------------------------------------------
# Collect scene objects
# ---------------------------------------------------------------------------

def _collect_mesh_objects(role: str) -> dict:
    """Return {link_name: first_obj} for all meshes with the given urdf_role."""
    result = {}
    for obj in bpy.data.objects:
        if obj.type != 'MESH':
            continue
        if obj.get("urdf_role") != role:
            continue
        link = obj.get("urdf_link", "")
        if link and link not in result:
            result[link] = obj
    return result


def _collect_joint_limits() -> dict:
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


def _derive_mesh_uri(original_pkg_uri: str, new_filename: str,
                     package_name: str, fallback_subdir: str = "meshes") -> str:
    """
    Build a package:// URI for a new mesh file by replacing only the filename
    in the original URI, preserving the package name and subdirectory.
    """
    if original_pkg_uri and original_pkg_uri.startswith("package://"):
        prefix = original_pkg_uri.rsplit("/", 1)[0]
        return f"{prefix}/{new_filename}"
    return f"package://{package_name}/{fallback_subdir}/{new_filename}"


def _uri_to_physical_path(uri: str, out_root: str) -> str:
    """
    Convert a package:// URI to a physical path under out_root.
    package://robot_description/meshes/x.stl  →  out_root/meshes/x.stl
    """
    if uri.startswith("package://"):
        rel = uri[len("package://"):].split("/", 1)[-1]
        return os.path.join(out_root, rel)
    return os.path.join(out_root, "meshes", os.path.basename(uri))


def _make_mesh_element(tag: str, uri: str, origin_xyz="0 0 0", origin_rpy="0 0 0") -> ET.Element:
    """Build a <visual> or <collision> element with a mesh geometry."""
    el      = ET.Element(tag)
    orig_el = ET.SubElement(el, "origin")
    orig_el.set("xyz", origin_xyz)
    orig_el.set("rpy", origin_rpy)
    geo_el  = ET.SubElement(el, "geometry")
    mesh_el = ET.SubElement(geo_el, "mesh")
    mesh_el.set("filename", uri)
    return el


def _indent_xml(elem, level=0):
    pad = "\n" + "  " * level
    if len(elem):
        if not elem.text or not elem.text.strip():
            elem.text = pad + "  "
        if not elem.tail or not elem.tail.strip():
            elem.tail = pad
        for child in elem:
            _indent_xml(child, level + 1)
        if not child.tail or not child.tail.strip():
            child.tail = pad
    else:
        if level and (not elem.tail or not elem.tail.strip()):
            elem.tail = pad


# ---------------------------------------------------------------------------
# Export a single mesh object to STL
# ---------------------------------------------------------------------------

def _export_obj(obj, uri: str, out_root: str):
    """Evaluate modifiers, write STL to the path derived from uri."""
    phys_path = _uri_to_physical_path(uri, out_root)
    os.makedirs(os.path.dirname(phys_path), exist_ok=True)
    mesh_data = _evaluated_mesh(obj)
    _write_stl(mesh_data, phys_path)
    bpy.data.meshes.remove(mesh_data)
    print(f"[URDF Export]   → {phys_path}")


# ---------------------------------------------------------------------------
# Main export function
# ---------------------------------------------------------------------------

def export_urdf(source_urdf_path: str, export_base: str, package_name: str) -> str:
    """
    Patch the original URDF:
      - Replace <visual>    tags for links that have a visual object in Blender.
      - Replace <collision> tags for links that have a collision object in Blender.
      - Update <limit>      tags for joints whose limits were edited.

    All other content is preserved verbatim.
    """
    if not os.path.isfile(source_urdf_path):
        raise FileNotFoundError(f"Source URDF not found: {source_urdf_path}")

    out_root   = os.path.join(export_base, "simplified_urdf")
    out_urdf   = os.path.join(out_root, "urdf")
    out_meshes = os.path.join(out_root, "meshes")
    os.makedirs(out_urdf,   exist_ok=True)
    os.makedirs(out_meshes, exist_ok=True)

    ET.register_namespace('', '')
    tree = ET.parse(source_urdf_path)
    root = tree.getroot()

    vis_objs  = _collect_mesh_objects("visual")
    col_objs  = _collect_mesh_objects("collision")
    limits    = _collect_joint_limits()

    print(f"[URDF Export] Visual objects:    {list(vis_objs.keys())}")
    print(f"[URDF Export] Collision objects: {list(col_objs.keys())}")

    # ---- Patch joint limits ----
    for jnt_el in root.findall("joint"):
        jname = jnt_el.get("name", "")
        if jname not in limits:
            continue
        lim_el = jnt_el.find("limit")
        if lim_el is None:
            continue
        ov = limits[jname]
        lim_el.set("lower",    f"{ov['lower']:.6f}")
        lim_el.set("upper",    f"{ov['upper']:.6f}")
        lim_el.set("velocity", f"{ov['velocity']:.6f}")
        lim_el.set("effort",   f"{ov['effort']:.6f}")
        print(f"[URDF Export]   limit patched: {jname}")

    # ---- Patch link visual and collision ----
    for link_el in root.findall("link"):
        link_name = link_el.get("name", "")

        # Get original visual mesh URI as template for path derivation
        orig_vis_uri = ""
        vis_mesh_el = link_el.find("visual/geometry/mesh")
        if vis_mesh_el is not None:
            orig_vis_uri = vis_mesh_el.get("filename", "")

        # ---- Visual ----
        if link_name in vis_objs:
            obj = vis_objs[link_name]
            stl_name = f"{link_name}_visual.stl"
            uri      = _derive_mesh_uri(orig_vis_uri, stl_name, package_name)

            _export_obj(obj, uri, out_root)
            _remove_children_named(link_el, "visual")
            link_el.append(_make_mesh_element("visual", uri))
            print(f"[URDF Export] Visual  patched: {link_name}")

        # ---- Collision ----
        if link_name in col_objs:
            obj = col_objs[link_name]
            stl_name = f"{link_name}_collision.stl"
            # Derive from visual URI template (same package/folder)
            uri      = _derive_mesh_uri(orig_vis_uri, stl_name, package_name)

            _export_obj(obj, uri, out_root)
            _remove_children_named(link_el, "collision")
            link_el.append(_make_mesh_element("collision", uri))
            print(f"[URDF Export] Collision patched: {link_name}")

    _indent_xml(root)
    robot_name  = root.get("name", package_name)
    output_path = os.path.join(out_urdf, f"{robot_name}.urdf")
    tree.write(output_path, encoding="unicode", xml_declaration=True)

    print(f"[URDF Export] Written: {output_path}")
    return output_path
