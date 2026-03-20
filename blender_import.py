"""
blender_import.py  —  Builds a Blender scene from a parsed URDFRobot.

Key concepts
------------
Each joint gets an Empty ("joint frame", named JF_<joint_name>) that:
  - is placed at the joint origin in its parent link's frame
  - is parented to the parent link's joint frame
  - has a driver on location (prismatic) or rotation_euler (revolute/continuous)
    driven by a custom property  obj["joint_value"]

Each link mesh is parented to its incoming joint frame empty with
matrix_parent_inverse = Identity (no extra offset).  This works because
SolidWorks exports mesh data already in link-frame coordinates, and the
joint frame empty IS the link frame origin.

Mimic joints have their joint_value driven by the source joint's joint_value
(scaled by multiplier), have no slider exposed, and are not user-editable.
"""

import bpy
import bmesh
import mathutils
import math
import os
from collections import deque
from .urdf_parser import URDFRobot, Geometry, Pose


# ---------------------------------------------------------------------------
# Math
# ---------------------------------------------------------------------------

def _pose_to_matrix(pose: Pose) -> mathutils.Matrix:
    r, p, y = pose.rpy
    rot = (mathutils.Matrix.Rotation(y, 4, 'Z') @
           mathutils.Matrix.Rotation(p, 4, 'Y') @
           mathutils.Matrix.Rotation(r, 4, 'X'))
    return mathutils.Matrix.Translation(pose.xyz) @ rot


# ---------------------------------------------------------------------------
# Collections
# ---------------------------------------------------------------------------

def _ensure_collection(name, parent=None):
    col = bpy.data.collections.get(name) or bpy.data.collections.new(name)
    target = parent or bpy.context.scene.collection
    if col.name not in target.children:
        target.children.link(col)
    return col


def _move_to_collection(obj, col):
    for c in list(obj.users_collection):
        c.objects.unlink(obj)
    col.objects.link(obj)


# ---------------------------------------------------------------------------
# Mesh import
# ---------------------------------------------------------------------------

def _import_stl(filepath):
    # Import without any scale arguments — we'll read whatever scale the
    # importer set on the object and bake it ourselves.
    if hasattr(bpy.ops.wm, "stl_import"):
        bpy.ops.wm.stl_import(filepath=filepath)
    elif hasattr(bpy.ops.import_mesh, "stl"):
        bpy.ops.import_mesh.stl(filepath=filepath)
    else:
        raise RuntimeError("No STL import operator found.")


def _import_mesh_file(filepath):
    ext    = os.path.splitext(filepath)[1].lower()
    before = set(bpy.data.objects.keys())
    print(f"[URDF] Importing: {filepath}")
    try:
        if   ext == ".dae": bpy.ops.wm.collada_import(filepath=filepath, import_units=False, fix_orientation=False)
        elif ext == ".stl": _import_stl(filepath)
        elif ext == ".obj": bpy.ops.wm.obj_import(filepath=filepath)
        elif ext == ".ply": bpy.ops.import_mesh.ply(filepath=filepath)
        elif ext == ".fbx": bpy.ops.import_scene.fbx(filepath=filepath)
        else: print(f"[URDF] Unsupported format: {ext!r}"); return []
    except Exception as e:
        print(f"[URDF] Import FAILED: {e}"); import traceback; traceback.print_exc(); return []

    new = [bpy.data.objects[n] for n in set(bpy.data.objects.keys()) - before]
    mesh_objs, to_del = [], []
    for o in new:
        (mesh_objs if o.type == 'MESH' else to_del).append(o)
    for o in to_del:
        bpy.data.objects.remove(o, do_unlink=True)
    print(f"[URDF]   kept {len(mesh_objs)} mesh object(s)")
    return mesh_objs


def _geo_to_objects(geo: Geometry, name_prefix: str):
    if geo is None:
        return []
    if geo.geo_type == "mesh":
        if not geo.mesh_filename or not os.path.exists(geo.mesh_filename):
            print(f"[URDF] Mesh not found: {geo.mesh_filename!r}")
            return []
        objs = _import_mesh_file(geo.mesh_filename)
        sx, sy, sz = geo.scale

        for i, obj in enumerate(objs):
            obj.name = name_prefix if len(objs) == 1 else f"{name_prefix}_{i}"
            # The STL importer already converts mm→m internally.
            # Just bake the importer's object scale into the vertices and
            # apply any explicit URDF scale= attribute on top.
            imp_sx, imp_sy, imp_sz = obj.scale
            final = (imp_sx * sx, imp_sy * sy, imp_sz * sz)
            scale_mat = mathutils.Matrix.Diagonal(mathutils.Vector(final)).to_4x4()
            obj.data.transform(scale_mat)
            obj.data.update()
            obj.scale = (1.0, 1.0, 1.0)
        return objs
    if geo.geo_type == "box":
        obj = _prim_box(name_prefix, geo.size)
    elif geo.geo_type == "cylinder":
        obj = _prim_cylinder(name_prefix, geo.radius, geo.length)
    elif geo.geo_type == "sphere":
        obj = _prim_sphere(name_prefix, geo.sphere_radius)
    else:
        return []
    bpy.context.scene.collection.objects.link(obj)
    return [obj]


def _prim_box(name, size):
    bm = bmesh.new(); bmesh.ops.create_cube(bm, size=1.0)
    me = bpy.data.meshes.new(name); bm.to_mesh(me); bm.free()
    obj = bpy.data.objects.new(name, me); obj.scale = tuple(size); return obj

def _prim_cylinder(name, radius, length):
    bm = bmesh.new()
    bmesh.ops.create_cone(bm, cap_ends=True, cap_tris=False, segments=32,
                          radius1=radius, radius2=radius, depth=length)
    me = bpy.data.meshes.new(name); bm.to_mesh(me); bm.free()
    return bpy.data.objects.new(name, me)

def _prim_sphere(name, radius):
    bm = bmesh.new(); bmesh.ops.create_uvsphere(bm, u_segments=32, v_segments=16, radius=radius)
    me = bpy.data.meshes.new(name); bm.to_mesh(me); bm.free()
    return bpy.data.objects.new(name, me)


# ---------------------------------------------------------------------------
# Materials
# ---------------------------------------------------------------------------

def _assign_material(obj, mat_name, rgba):
    name = mat_name or f"_urdf_mat_{obj.name}"
    mat  = bpy.data.materials.get(name)
    if mat is None:
        mat = bpy.data.materials.new(name); mat.use_nodes = True
        bsdf = mat.node_tree.nodes.get("Principled BSDF")
        if bsdf:
            bsdf.inputs["Base Color"].default_value = (rgba[0], rgba[1], rgba[2], 1.0)
            bsdf.inputs["Alpha"].default_value = rgba[3] if len(rgba) > 3 else 1.0
        mat.blend_method = "BLEND" if (len(rgba) > 3 and rgba[3] < 1.0) else "OPAQUE"
    if obj.data and hasattr(obj.data, "materials"):
        obj.data.materials.clear(); obj.data.materials.append(mat)


def _assign_collision_material(obj):
    obj.display_type = 'WIRE'
    mat = bpy.data.materials.get("_URDF_Collision")
    if mat is None:
        mat = bpy.data.materials.new("_URDF_Collision"); mat.use_nodes = True
        bsdf = mat.node_tree.nodes.get("Principled BSDF")
        if bsdf:
            bsdf.inputs["Base Color"].default_value = (0.0, 1.0, 0.2, 1.0)
            bsdf.inputs["Alpha"].default_value = 0.3
        mat.blend_method = "BLEND"
    if obj.data and hasattr(obj.data, "materials"):
        obj.data.materials.clear(); obj.data.materials.append(mat)


# ---------------------------------------------------------------------------
# Simplification (public — also called by operators)
# ---------------------------------------------------------------------------

def simplify_mesh(obj, ratio=0.1):
    if obj.type != 'MESH':
        return
    for mod in list(obj.modifiers):
        if mod.type == 'DECIMATE':
            obj.modifiers.remove(mod)
    dec = obj.modifiers.new(name="URDF_Decimate", type='DECIMATE')
    dec.ratio = max(0.01, min(1.0, ratio))
    dec.use_collapse_triangulate = True


def apply_convex_hull(obj):
    if obj.type != 'MESH':
        return
    bm = bmesh.new(); bm.from_mesh(obj.data)
    result   = bmesh.ops.convex_hull(bm, input=bm.verts)
    interior = set(result.get("geom_interior", []))
    unused   = set(result.get("geom_unused",   []))
    to_del   = [g for g in (interior | unused) if isinstance(g, bmesh.types.BMFace)]
    bmesh.ops.delete(bm, geom=to_del, context='FACES')
    bm.to_mesh(obj.data); bm.free(); obj.data.update()


def apply_bounding_box(obj):
    if obj.type != 'MESH':
        return
    bb = [obj.matrix_world @ mathutils.Vector(v) for v in obj.bound_box]
    xs, ys, zs = [v.x for v in bb], [v.y for v in bb], [v.z for v in bb]
    mn = mathutils.Vector((min(xs), min(ys), min(zs)))
    mx = mathutils.Vector((max(xs), max(ys), max(zs)))
    bm = bmesh.new(); bmesh.ops.create_cube(bm, size=1.0)
    me = bpy.data.meshes.new(obj.data.name + "_bbox"); bm.to_mesh(me); bm.free()
    obj.data = me
    obj.matrix_world = mathutils.Matrix.Translation((mn + mx) / 2)
    obj.scale = tuple(mx - mn)


# ---------------------------------------------------------------------------
# Topology helpers
# ---------------------------------------------------------------------------

def _build_parent_map(robot):
    """child_link_name → URDFJoint"""
    return {j.child: j for j in robot.joints.values()}


def _topo_order(robot, parent_map):
    """Return link names in BFS order from root(s), so parents always precede children."""
    children_of = {}
    for link_name, joint in parent_map.items():
        children_of.setdefault(joint.parent, []).append(link_name)
    roots = [n for n in robot.links if n not in parent_map]
    queue = deque(roots)
    order = []
    while queue:
        link = queue.popleft()
        order.append(link)
        for child in children_of.get(link, []):
            queue.append(child)
    return order


# ---------------------------------------------------------------------------
# Driver helper
# ---------------------------------------------------------------------------

def _add_driver(target_obj, data_path, array_index, source_obj, source_prop, multiplier=1.0):
    """
    Add a scripted driver on target_obj[data_path][array_index]
    that evaluates to  source_obj[source_prop] * multiplier.
    """
    fcurve = target_obj.driver_add(data_path, array_index)
    drv = fcurve.driver
    drv.type = 'SCRIPTED'
    var = drv.variables.new()
    var.name = "v"
    var.type = 'SINGLE_PROP'
    tgt = var.targets[0]
    tgt.id_type   = 'OBJECT'
    tgt.id        = source_obj
    tgt.data_path = f'["{source_prop}"]'
    drv.expression = f"v * {multiplier}"


# ---------------------------------------------------------------------------
# Joint frame builder
# ---------------------------------------------------------------------------

def _make_joint_frame(joint, parent_frame, frame_col, mimic_source_frame=None):
    """
    Create an Empty for one joint.

    Parenting strategy
    ------------------
    The empty is placed at the joint origin RELATIVE TO ITS PARENT FRAME,
    not in world space, by setting location/rotation directly from joint.origin.
    This is the only correct approach — world-space placement + matrix_parent_inverse
    fights against the driver-modified transform of the parent frame.

    The empty's local transform = joint.origin (translation + rotation).
    The driver then adds a DELTA on top of that local transform:
      - prismatic:  adds to location along the joint axis
      - revolute/continuous: adds to rotation_euler along the joint axis

    Mimic joints
    ------------
    A mimic joint has no user-facing slider.  Its joint_value is driven by
    the source joint's joint_value * multiplier instead.
    """
    name = f"JF_{joint.name}"
    obj  = bpy.data.objects.new(name, None)
    obj.empty_display_type = 'ARROWS'
    obj.empty_display_size = 0.15
    frame_col.objects.link(obj)

    # ---- Place in parent-relative space ----
    # Set rotation mode before assigning rotation
    obj.rotation_mode = 'XYZ'

    if parent_frame is not None:
        obj.parent = parent_frame

    # Local location = joint origin translation
    obj.location = mathutils.Vector(joint.origin.xyz)
    # Local rotation = joint origin rotation (RPY → euler XYZ)
    r, p, y = joint.origin.rpy
    obj.rotation_euler = mathutils.Euler((r, p, y), 'XYZ')

    # ---- Metadata ----
    obj["urdf_joint_type"] = joint.joint_type
    obj["urdf_joint_name"] = joint.name
    obj["urdf_axis"]       = list(joint.axis)
    obj["urdf_is_mimic"]   = 1 if joint.mimic else 0

    jt  = joint.joint_type
    lim = joint.limit

    if lim:
        obj["urdf_lower"]    = lim.lower
        obj["urdf_upper"]    = lim.upper
        obj["urdf_effort"]   = lim.effort
        obj["urdf_velocity"] = lim.velocity

    # ---- Fixed joints: nothing more to do ----
    if jt == "fixed":
        return obj

    # ---- Determine joint_value range ----
    is_mimic     = joint.mimic is not None
    is_continuous = jt == "continuous"
    is_prismatic  = jt == "prismatic"
    is_revolute   = jt == "revolute"

    if is_continuous:
        # Continuous = unlimited rotation — use a wide but finite UI range
        default  = 0.0
        val_min  = -math.pi * 4
        val_max  =  math.pi * 4
    elif is_revolute:
        default  = lim.lower if lim else 0.0
        val_min  = lim.lower if lim else -math.pi
        val_max  = lim.upper if lim else  math.pi
    else:  # prismatic
        default  = lim.lower if lim else 0.0
        val_min  = lim.lower if lim else -10.0
        val_max  = lim.upper if lim else  10.0

    obj["joint_value"] = default

    # Set UI bounds on the custom property
    ui = obj.id_properties_ui("joint_value")
    ui.update(
        min=val_min, max=val_max,
        soft_min=val_min, soft_max=val_max,
        description=f"{jt} | axis {list(joint.axis)}",
    )

    ax, ay, az = joint.axis
    ox, oy, oz = joint.origin.xyz
    rr, rp, ry = joint.origin.rpy

    def _drive_axis(data_path, i, component, offset, src_obj, src_prop, multiplier):
        """Drive data_path[i] = src_obj[src_prop] * multiplier * component + offset"""
        fcurve = obj.driver_add(data_path, i)
        drv = fcurve.driver
        drv.type = 'SCRIPTED'
        var = drv.variables.new()
        var.name = "v"
        var.type = 'SINGLE_PROP'
        tgt = var.targets[0]
        tgt.id_type   = 'OBJECT'
        tgt.id        = src_obj
        tgt.data_path = f'["{src_prop}"]'
        drv.expression = f"v * {multiplier * component} + {offset}"

    if is_mimic and mimic_source_frame is not None:
        # Mimic joints: drive location/rotation DIRECTLY from the source
        # joint_value, bypassing our own joint_value property entirely.
        # This avoids Blender driver evaluation order issues where reading
        # ["joint_value"] might return a stale value.
        mult = joint.mimic.multiplier
        src  = mimic_source_frame

        if is_prismatic:
            for i, (component, offset) in enumerate(zip((ax, ay, az), (ox, oy, oz))):
                if abs(component) < 1e-6:
                    continue
                _drive_axis("location", i, component, offset, src, "joint_value", mult)
        else:
            for i, (component, offset) in enumerate(zip((ax, ay, az), (rr, rp, ry))):
                if abs(component) < 1e-6:
                    continue
                _drive_axis("rotation_euler", i, component, offset, src, "joint_value", mult)

        obj["urdf_is_mimic"] = 1

    else:
        # Non-mimic: drive from our own joint_value property
        if is_prismatic:
            for i, (component, offset) in enumerate(zip((ax, ay, az), (ox, oy, oz))):
                if abs(component) < 1e-6:
                    continue
                _drive_axis("location", i, component, offset, obj, "joint_value", 1.0)
        else:
            for i, (component, offset) in enumerate(zip((ax, ay, az), (rr, rp, ry))):
                if abs(component) < 1e-6:
                    continue
                _drive_axis("rotation_euler", i, component, offset, obj, "joint_value", 1.0)

    return obj


# ---------------------------------------------------------------------------
# Default scene cleanup
# ---------------------------------------------------------------------------

def _clear_default_scene():
    for name in ("Cube", "Camera", "Light"):
        obj = bpy.data.objects.get(name)
        if obj:
            bpy.data.objects.remove(obj, do_unlink=True)


# ---------------------------------------------------------------------------
# Main entry point
# ---------------------------------------------------------------------------

def build_scene(robot: URDFRobot,
                import_visual:    bool  = True,
                import_collision: bool  = True,
                auto_simplify:    bool  = True,
                decimate_ratio:   float = 0.1) -> dict:
    """
    Import a URDFRobot into the current Blender scene.

    Visual meshes  — imported at full resolution, never modified.
    Collision meshes — always duplicated from visual, URDF <collision> ignored.
    Returns {'joint_frames': {link_name: empty_obj}}
    """
    _clear_default_scene()

    print(f"[URDF] Robot: {robot.name!r}")
    print(f"[URDF] Links: {list(robot.links.keys())}")
    print(f"[URDF] Joints: {list(robot.joints.keys())}")

    parent_map = _build_parent_map(robot)
    topo       = _topo_order(robot, parent_map)

    master_col = _ensure_collection(f"URDF_{robot.name}")
    vis_col    = _ensure_collection(f"{robot.name}_Visual",      master_col)
    col_col    = _ensure_collection(f"{robot.name}_Collision",   master_col)
    frame_col  = _ensure_collection(f"{robot.name}_JointFrames", master_col)

    joint_frames = {}   # link_name → Empty

    # Map joint name → joint frame, for mimic lookup
    joint_frame_by_joint_name = {}

    for link_name in topo:
        if link_name not in parent_map:
            # Root link: plain empty at origin, no parent, no driver
            obj = bpy.data.objects.new(f"JF_{link_name}", None)
            obj.empty_display_type = 'ARROWS'
            obj.empty_display_size = 0.15
            frame_col.objects.link(obj)
            obj["urdf_joint_type"] = "root"
            obj["urdf_joint_name"] = link_name
            joint_frames[link_name] = obj
            continue

        joint        = parent_map[link_name]
        parent_frame = joint_frames[joint.parent]   # always exists (topo order)

        # Resolve mimic source frame if needed
        mimic_source_frame = None
        if joint.mimic:
            src_joint_name = joint.mimic.joint
            mimic_source_frame = joint_frame_by_joint_name.get(src_joint_name)
            if mimic_source_frame is None:
                print(f"[URDF] Mimic source joint {src_joint_name!r} not yet built — "
                      f"mimic for {joint.name!r} will be skipped")

        frame = _make_joint_frame(joint, parent_frame, frame_col, mimic_source_frame)
        joint_frames[link_name] = frame
        joint_frame_by_joint_name[joint.name] = frame

    # ---- Import meshes ----
    for link_name, link in robot.links.items():
        frame = joint_frames.get(link_name)
        imported_visual = []

        if import_visual:
            for vi, vis in enumerate(link.visuals):
                if vis.geometry is None:
                    continue
                objs = _geo_to_objects(vis.geometry, f"{link_name}_vis_{vi}")
                if not objs:
                    continue
                for obj in objs:
                    _move_to_collection(obj, vis_col)
                    _assign_material(obj, vis.material_name, vis.color_rgba)
                    obj["urdf_link"] = link_name
                    obj["urdf_role"] = "visual"

                    if frame is not None:
                        # Parent directly to joint frame.
                        # mesh data is in link-frame coords, joint frame IS the
                        # link origin → zero local offset needed.
                        # Apply visual origin offset if non-zero (rare in SW).
                        obj.parent = frame
                        obj.location = mathutils.Vector(vis.origin.xyz)
                        r, p, y = vis.origin.rpy
                        obj.rotation_mode = 'XYZ'
                        obj.rotation_euler = mathutils.Euler((r, p, y), 'XYZ')
                        obj.matrix_basis = obj.matrix_basis  # keep what we set

                    imported_visual.append(obj)

        if import_collision:
            for src_obj in imported_visual:
                dup      = src_obj.copy()
                dup.data = src_obj.data.copy()
                bpy.context.scene.collection.objects.link(dup)

                dup.name = src_obj.name.replace("_vis_", "_col_") + "_col"
                _move_to_collection(dup, col_col)
                _assign_collision_material(dup)
                dup["urdf_link"] = link_name
                dup["urdf_role"] = "collision"

                if frame is not None:
                    dup.parent         = frame
                    dup.location       = src_obj.location.copy()
                    dup.rotation_mode  = 'XYZ'
                    dup.rotation_euler = src_obj.rotation_euler.copy()

                if auto_simplify:
                    simplify_mesh(dup, decimate_ratio)

    col_col.hide_render = True
    bpy.context.view_layer.update()

    return {"joint_frames": joint_frames}
