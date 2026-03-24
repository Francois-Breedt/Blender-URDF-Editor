"""
blender_import.py  —  Builds a Blender scene from a parsed URDFRobot.

Collision geometry strategy
---------------------------
For each link, in order of preference:
  1. Import explicit <collision> geometry from the URDF (mesh or primitive).
  2. If none exists, duplicate the visual mesh.

Primitives (box, cylinder, sphere) are created directly via bmesh and placed
according to the collision origin — no file import needed.

The "Copy Visual → Collision" button (operator urdf.copy_visual_to_collision)
replaces all collision objects in the scene with fresh duplicates of the visual
objects and re-applies the Decimate modifier.
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
        else:
            print(f"[URDF] Unsupported format: {ext!r}")
            return []
    except Exception as e:
        print(f"[URDF] Import FAILED: {e}")
        import traceback; traceback.print_exc()
        return []

    new = [bpy.data.objects[n] for n in set(bpy.data.objects.keys()) - before]
    mesh_objs, to_del = [], []
    for o in new:
        (mesh_objs if o.type == 'MESH' else to_del).append(o)
    for o in to_del:
        bpy.data.objects.remove(o, do_unlink=True)
    print(f"[URDF]   kept {len(mesh_objs)} mesh object(s)")
    return mesh_objs


def _geo_to_objects(geo: Geometry, name_prefix: str, is_visual: bool = True):
    """
    Convert a Geometry descriptor to Blender mesh objects.
    is_visual=True  → apply 1000× bake for STL (importer returns raw mm data)
    is_visual=False → same bake needed for collision meshes that are STL files
    Primitives are returned in scene coords; callers move them to final collection.
    """
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
            imp_sx, imp_sy, imp_sz = obj.scale
            final = (imp_sx * sx, imp_sy * sy, imp_sz * sz)
            scale_mat = mathutils.Matrix.Diagonal(mathutils.Vector(final)).to_4x4()
            obj.data.transform(scale_mat)
            obj.data.update()
            obj.scale = (1.0, 1.0, 1.0)
        return objs

    # Primitive geometries — no file import, no scale bake needed
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
    obj = bpy.data.objects.new(name, me)
    obj.scale = tuple(size)
    # Apply scale into vertices immediately
    scale_mat = mathutils.Matrix.Diagonal(mathutils.Vector(size)).to_4x4()
    me.transform(scale_mat); me.update()
    obj.scale = (1.0, 1.0, 1.0)
    return obj

def _prim_cylinder(name, radius, length):
    bm = bmesh.new()
    bmesh.ops.create_cone(bm, cap_ends=True, cap_tris=False, segments=32,
                          radius1=radius, radius2=radius, depth=length)
    me = bpy.data.meshes.new(name); bm.to_mesh(me); bm.free()
    return bpy.data.objects.new(name, me)

def _prim_sphere(name, radius):
    bm = bmesh.new()
    bmesh.ops.create_uvsphere(bm, u_segments=32, v_segments=16, radius=radius)
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
# Simplification
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
    """Replace obj's mesh with its convex hull, in-place."""
    if obj.type != 'MESH':
        return
    # Evaluate modifiers first so the hull is computed on the final shape
    depsgraph = bpy.context.evaluated_depsgraph_get()
    eval_mesh = bpy.data.meshes.new_from_object(obj.evaluated_get(depsgraph))

    bm = bmesh.new()
    bm.from_mesh(eval_mesh)
    bpy.data.meshes.remove(eval_mesh)

    result = bmesh.ops.convex_hull(bm, input=bm.verts)

    # Delete everything NOT part of the hull exterior
    hull_geom = set(result.get("geom", []))
    to_del = [g for g in bm.verts if g not in hull_geom] +              [g for g in bm.edges if g not in hull_geom] +              [g for g in bm.faces if g not in hull_geom]
    bmesh.ops.delete(bm, geom=to_del, context='VERTS')

    # Write result back — remove all modifiers first so they don't conflict
    for mod in list(obj.modifiers):
        obj.modifiers.remove(mod)
    bm.to_mesh(obj.data)
    bm.free()
    obj.data.update()


def apply_bounding_box(obj):
    """Replace obj's mesh with an axis-aligned bounding box in local space."""
    if obj.type != 'MESH':
        return
    # Evaluate modifiers to get the real vertex positions
    depsgraph = bpy.context.evaluated_depsgraph_get()
    eval_mesh = bpy.data.meshes.new_from_object(obj.evaluated_get(depsgraph))

    # Compute bounds in local space (so they stay correct under the parent)
    verts = [mathutils.Vector(v.co) for v in eval_mesh.vertices]
    bpy.data.meshes.remove(eval_mesh)
    if not verts:
        return

    xs = [v.x for v in verts]; ys = [v.y for v in verts]; zs = [v.z for v in verts]
    mn = mathutils.Vector((min(xs), min(ys), min(zs)))
    mx = mathutils.Vector((max(xs), max(ys), max(zs)))
    center = (mn + mx) * 0.5
    size   = mx - mn

    # Build a unit cube and scale it to the bounding box size
    bm = bmesh.new()
    bmesh.ops.create_cube(bm, size=1.0)
    # Scale vertices directly so obj.scale stays (1,1,1)
    scale_mat = mathutils.Matrix.Diagonal(
        mathutils.Vector((size.x, size.y, size.z, 1.0)))
    bmesh.ops.transform(bm, matrix=scale_mat, verts=bm.verts)
    # Translate to center
    bmesh.ops.translate(bm, verts=bm.verts, vec=center)

    for mod in list(obj.modifiers):
        obj.modifiers.remove(mod)
    bm.to_mesh(obj.data)
    bm.free()
    obj.data.update()
    obj.scale = (1.0, 1.0, 1.0)


def apply_oriented_bounding_box(obj):
    """
    Replace obj's mesh with a minimum oriented bounding box.

    Uses PCA (SVD of the vertex point cloud) to find the rotation that
    minimises box volume, then builds a box in that orientation.
    Falls back to an axis-aligned box if numpy is unavailable.
    """
    if obj.type != 'MESH':
        return

    try:
        import numpy as np
    except ImportError:
        print("[URDF] numpy not available — falling back to axis-aligned bbox")
        apply_bounding_box(obj)
        return

    depsgraph = bpy.context.evaluated_depsgraph_get()
    eval_mesh = bpy.data.meshes.new_from_object(obj.evaluated_get(depsgraph))
    pts = np.array([v.co for v in eval_mesh.vertices], dtype=np.float64)
    bpy.data.meshes.remove(eval_mesh)

    if len(pts) < 4:
        apply_bounding_box(obj)
        return

    # PCA: centre the points, compute covariance, SVD → rotation axes
    centroid = pts.mean(axis=0)
    pts_c    = pts - centroid
    _, _, Vt = np.linalg.svd(pts_c, full_matrices=False)
    # Rows of Vt are the principal axes (largest variance first)
    R = Vt.T  # columns = principal axes, shape (3,3)

    # Ensure right-handed: if det < 0, flip the last axis
    if np.linalg.det(R) < 0:
        R[:, 2] *= -1

    # Project points onto principal axes, find min/max → OBB in local PCA space
    proj   = pts_c @ R           # (N, 3)
    mn     = proj.min(axis=0)    # (3,)
    mx     = proj.max(axis=0)    # (3,)
    center_pca = (mn + mx) * 0.5
    extents    = mx - mn          # full lengths along each axis

    # Build a unit cube, scale to extents, rotate into PCA frame, translate
    bm = bmesh.new()
    bmesh.ops.create_cube(bm, size=1.0)

    # Scale
    scale_mat = mathutils.Matrix.Diagonal(
        mathutils.Vector((extents[0], extents[1], extents[2], 1.0)))
    bmesh.ops.transform(bm, matrix=scale_mat, verts=bm.verts)

    # Rotate into PCA frame
    rot_mat = mathutils.Matrix((
        (R[0,0], R[0,1], R[0,2], 0),
        (R[1,0], R[1,1], R[1,2], 0),
        (R[2,0], R[2,1], R[2,2], 0),
        (0,      0,      0,      1),
    ))
    bmesh.ops.transform(bm, matrix=rot_mat, verts=bm.verts)

    # Translate: center_pca is in PCA frame, convert back to local frame
    center_local = mathutils.Vector(R @ center_pca + centroid)
    bmesh.ops.translate(bm, verts=bm.verts, vec=center_local)

    for mod in list(obj.modifiers):
        obj.modifiers.remove(mod)
    bm.to_mesh(obj.data)
    bm.free()
    obj.data.update()
    obj.scale = (1.0, 1.0, 1.0)


# ---------------------------------------------------------------------------
# Copy visual → collision (public, called by operator)
# ---------------------------------------------------------------------------

def copy_visual_to_collision(auto_simplify=True, decimate_ratio=0.1):
    """
    Replace every collision object in the scene with a fresh duplicate of its
    matching visual object.  Called by the URDF_OT_CopyVisualToCollision operator.
    """
    # Find the collision collection
    col_col = None
    for col in bpy.data.collections:
        if col.name.endswith("_Collision"):
            col_col = col
            break
    if col_col is None:
        print("[URDF] No Collision collection found")
        return

    # Build link_name → visual object map
    vis_by_link = {}
    for obj in bpy.data.objects:
        if obj.type == 'MESH' and obj.get("urdf_role") == "visual":
            link = obj.get("urdf_link", "")
            if link and link not in vis_by_link:
                vis_by_link[link] = obj

    # Remove all existing collision objects
    for obj in list(col_col.objects):
        bpy.data.objects.remove(obj, do_unlink=True)

    # Duplicate visual objects into collision collection
    for link_name, vis_obj in vis_by_link.items():
        dup      = vis_obj.copy()
        dup.data = vis_obj.data.copy()
        bpy.context.scene.collection.objects.link(dup)

        dup.name = vis_obj.name.replace("_vis_", "_col_") + "_col"
        _move_to_collection(dup, col_col)
        _assign_collision_material(dup)
        dup["urdf_link"] = link_name
        dup["urdf_role"] = "collision"

        # Copy parent and local transform from visual
        dup.parent         = vis_obj.parent
        dup.location       = vis_obj.location.copy()
        dup.rotation_mode  = 'XYZ'
        dup.rotation_euler = vis_obj.rotation_euler.copy()

        if auto_simplify:
            simplify_mesh(dup, decimate_ratio)

    col_col.hide_render = True
    bpy.context.view_layer.update()
    print(f"[URDF] Copied {len(vis_by_link)} visual → collision objects")


# ---------------------------------------------------------------------------
# Topology helpers
# ---------------------------------------------------------------------------

def _build_parent_map(robot):
    return {j.child: j for j in robot.joints.values()}


def _topo_order(robot, parent_map):
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

BONE_LENGTH = 0.3


def _make_joint_frame(joint, parent_frame, frame_col, mimic_source_frame=None):
    name = f"JF_{joint.name}"
    obj  = bpy.data.objects.new(name, None)
    obj.empty_display_type = 'ARROWS'
    obj.empty_display_size = 0.15
    frame_col.objects.link(obj)

    obj.rotation_mode = 'XYZ'
    if parent_frame is not None:
        obj.parent = parent_frame

    obj.location = mathutils.Vector(joint.origin.xyz)
    r, p, y = joint.origin.rpy
    obj.rotation_euler = mathutils.Euler((r, p, y), 'XYZ')

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

    if jt == "fixed":
        return obj

    is_mimic      = joint.mimic is not None
    is_continuous  = jt == "continuous"
    is_prismatic   = jt == "prismatic"
    is_revolute    = jt == "revolute"

    if is_continuous:
        default = 0.0; val_min = -math.pi * 4; val_max = math.pi * 4
    elif is_revolute:
        default = lim.lower if lim else 0.0
        val_min = lim.lower if lim else -math.pi
        val_max = lim.upper if lim else  math.pi
    else:  # prismatic
        default = lim.lower if lim else 0.0
        val_min = lim.lower if lim else -10.0
        val_max = lim.upper if lim else  10.0

    obj["joint_value"] = default
    ui = obj.id_properties_ui("joint_value")
    ui.update(min=val_min, max=val_max, soft_min=val_min, soft_max=val_max,
              description=f"{jt} | axis {list(joint.axis)}")

    ax, ay, az = joint.axis
    ox, oy, oz = joint.origin.xyz
    rr, rp, ry = joint.origin.rpy

    def _drive_axis(data_path, i, component, offset, src_obj, src_prop, multiplier):
        fcurve = obj.driver_add(data_path, i)
        drv = fcurve.driver; drv.type = 'SCRIPTED'
        var = drv.variables.new(); var.name = "v"; var.type = 'SINGLE_PROP'
        tgt = var.targets[0]
        tgt.id_type = 'OBJECT'; tgt.id = src_obj
        tgt.data_path = f'["{src_prop}"]'
        drv.expression = f"v * {multiplier * component} + {offset}"

    if is_mimic and mimic_source_frame is not None:
        mult = joint.mimic.multiplier
        src  = mimic_source_frame
        if is_prismatic:
            for i, (comp, off) in enumerate(zip((ax, ay, az), (ox, oy, oz))):
                if abs(comp) < 1e-6: continue
                _drive_axis("location", i, comp, off, src, "joint_value", mult)
        else:
            for i, (comp, off) in enumerate(zip((ax, ay, az), (rr, rp, ry))):
                if abs(comp) < 1e-6: continue
                _drive_axis("rotation_euler", i, comp, off, src, "joint_value", mult)
        obj["urdf_is_mimic"] = 1
    else:
        if is_prismatic:
            for i, (comp, off) in enumerate(zip((ax, ay, az), (ox, oy, oz))):
                if abs(comp) < 1e-6: continue
                _drive_axis("location", i, comp, off, obj, "joint_value", 1.0)
        else:
            for i, (comp, off) in enumerate(zip((ax, ay, az), (rr, rp, ry))):
                if abs(comp) < 1e-6: continue
                _drive_axis("rotation_euler", i, comp, off, obj, "joint_value", 1.0)

    return obj


# ---------------------------------------------------------------------------
# Helper: add a single geometry object to a collection, parented to frame
# ---------------------------------------------------------------------------

def _add_geo_obj(obj, col, frame, origin: Pose, link_name, role, tags=None):
    """Move obj into col, parent to frame, set local offset from origin."""
    _move_to_collection(obj, col)
    obj["urdf_link"] = link_name
    obj["urdf_role"] = role
    if tags:
        for k, v in tags.items():
            obj[k] = v
    if frame is not None:
        obj.parent = frame
        obj.location = mathutils.Vector(origin.xyz)
        r, p, y = origin.rpy
        obj.rotation_mode  = 'XYZ'
        obj.rotation_euler = mathutils.Euler((r, p, y), 'XYZ')


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

    Collision strategy (per link):
      1. Import explicit <collision> geometry from the URDF (mesh or primitive).
      2. If none found, duplicate the visual mesh as fallback.
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

    joint_frames = {}
    joint_frame_by_joint_name = {}

    for link_name in topo:
        if link_name not in parent_map:
            obj = bpy.data.objects.new(f"JF_{link_name}", None)
            obj.empty_display_type = 'ARROWS'
            obj.empty_display_size = 0.15
            frame_col.objects.link(obj)
            obj["urdf_joint_type"] = "root"
            obj["urdf_joint_name"] = link_name
            joint_frames[link_name] = obj
            continue

        joint        = parent_map[link_name]
        parent_frame = joint_frames[joint.parent]

        mimic_source_frame = None
        if joint.mimic:
            mimic_source_frame = joint_frame_by_joint_name.get(joint.mimic.joint)
            if mimic_source_frame is None:
                print(f"[URDF] Mimic source {joint.mimic.joint!r} not yet built — skipped")

        frame = _make_joint_frame(joint, parent_frame, frame_col, mimic_source_frame)
        joint_frames[link_name] = frame
        joint_frame_by_joint_name[joint.name] = frame

    # ---- Import meshes ----
    for link_name, link in robot.links.items():
        frame = joint_frames.get(link_name)

        # ---- Visual ----
        imported_visual = []   # list of (obj, Pose) — the visual origin pose

        if import_visual:
            for vi, vis in enumerate(link.visuals):
                if vis.geometry is None:
                    continue
                objs = _geo_to_objects(vis.geometry, f"{link_name}_vis_{vi}", is_visual=True)
                if not objs:
                    continue
                for obj in objs:
                    _assign_material(obj, vis.material_name, vis.color_rgba)
                    _add_geo_obj(obj, vis_col, frame, vis.origin, link_name, "visual")
                    imported_visual.append((obj, vis.origin))

        # ---- Collision ----
        if import_collision:
            imported_collision = []

            # Try explicit <collision> entries first
            for ci, col_entry in enumerate(link.collisions):
                if col_entry.geometry is None:
                    continue
                objs = _geo_to_objects(col_entry.geometry,
                                       f"{link_name}_col_{ci}", is_visual=False)
                if not objs:
                    continue
                for obj in objs:
                    _assign_collision_material(obj)
                    _add_geo_obj(obj, col_col, frame, col_entry.origin,
                                 link_name, "collision")
                    if auto_simplify and col_entry.geometry.geo_type == "mesh":
                        simplify_mesh(obj, decimate_ratio)
                    imported_collision.append(obj)

            # Fallback: duplicate visual if no collision geometry found
            if not imported_collision:
                for src_obj, vis_origin in imported_visual:
                    dup      = src_obj.copy()
                    dup.data = src_obj.data.copy()
                    bpy.context.scene.collection.objects.link(dup)
                    dup.name = src_obj.name.replace("_vis_", "_col_") + "_col"
                    _assign_collision_material(dup)
                    _add_geo_obj(dup, col_col, frame, vis_origin,
                                 link_name, "collision")
                    if auto_simplify:
                        simplify_mesh(dup, decimate_ratio)

    col_col.hide_render = True
    bpy.context.view_layer.update()

    return {"joint_frames": joint_frames}
