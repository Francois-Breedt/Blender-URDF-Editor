"""
operators.py
All Blender operators for the URDF Tools addon.
"""

import bpy
import os
from bpy.props import StringProperty, FloatProperty, BoolProperty, EnumProperty
from bpy_extras.io_utils import ImportHelper, ExportHelper

from .urdf_parser import parse_urdf
from .blender_import import (
    build_scene, simplify_mesh, apply_convex_hull, apply_bounding_box,
)
from .urdf_exporter import export_urdf


# ---------------------------------------------------------------------------
# Import operator
# ---------------------------------------------------------------------------

class URDF_OT_Import(bpy.types.Operator, ImportHelper):
    """Import a URDF robot description file"""
    bl_idname = "urdf.import"
    bl_label = "Import URDF"
    bl_options = {'REGISTER', 'UNDO'}

    filename_ext = ".urdf"
    filter_glob: StringProperty(default="*.urdf;*.xacro", options={'HIDDEN'})

    import_visual: BoolProperty(name="Import Visual Geometry", default=True)
    import_collision: BoolProperty(name="Import Collision Geometry", default=True)
    auto_simplify: BoolProperty(
        name="Auto-Simplify Collision",
        description="Apply Decimate modifier to collision meshes",
        default=True,
    )
    decimate_ratio: FloatProperty(
        name="Decimate Ratio",
        description="Fraction of faces to keep (lower = simpler)",
        min=0.01, max=1.0, default=0.1, subtype='FACTOR',
    )

    def execute(self, context):
        try:
            robot = parse_urdf(self.filepath)
        except Exception as e:
            self.report({'ERROR'}, f"Failed to parse URDF: {e}")
            return {'CANCELLED'}

        context.scene.urdf.robot_name = robot.name
        context.scene.urdf.source_urdf_path = self.filepath

        try:
            result = build_scene(
                robot,
                import_visual=self.import_visual,
                import_collision=self.import_collision,
                auto_simplify=self.auto_simplify,
                decimate_ratio=self.decimate_ratio,
            )
        except Exception as e:
            self.report({'ERROR'}, f"Failed to build scene: {e}")
            import traceback; traceback.print_exc()
            return {'CANCELLED'}

        # Mirror custom props to property group for UI
        for obj in bpy.data.objects:
            if obj.type == 'MESH':
                obj.urdf.link_name = obj.get("urdf_link", "")
                obj.urdf.role      = obj.get("urdf_role", "none")

        n_joints = len(result.get("joint_frames", {}))
        self.report({'INFO'}, f"Imported '{robot.name}': "
                              f"{len(robot.links)} links, {len(robot.joints)} joints.")
        return {'FINISHED'}

    def draw(self, context):
        layout = self.layout
        layout.prop(self, "import_visual")
        layout.prop(self, "import_collision")
        layout.separator()
        col = layout.column()
        col.enabled = self.import_collision
        col.prop(self, "auto_simplify")
        col2 = col.column()
        col2.enabled = self.auto_simplify
        col2.prop(self, "decimate_ratio", slider=True)


# ---------------------------------------------------------------------------
# Export operator
# ---------------------------------------------------------------------------

class URDF_OT_Export(bpy.types.Operator):
    """Export simplified URDF — patches original collision tags, saves to simplified_urdf/"""
    bl_idname = "urdf.export"
    bl_label = "Export URDF"
    bl_options = {'REGISTER'}

    def execute(self, context):
        props = context.scene.urdf
        source_path = props.source_urdf_path

        if not source_path or not os.path.isfile(source_path):
            self.report({'ERROR'},
                        "Original URDF path not set or file missing. "
                        "Re-import the URDF first.")
            return {'CANCELLED'}

        # Export next to the original URDF file
        export_base  = os.path.dirname(source_path)
        package_name = props.robot_name or "robot"

        try:
            output = export_urdf(
                source_urdf_path=source_path,
                export_base=export_base,
                package_name=package_name,
            )
        except Exception as e:
            self.report({'ERROR'}, f"Export failed: {e}")
            import traceback; traceback.print_exc()
            return {'CANCELLED'}

        self.report({'INFO'}, f"Exported to: {output}")
        return {'FINISHED'}

    def draw(self, context):
        layout = self.layout
        layout.prop(context.scene.urdf, "robot_name")
        layout.prop(context.scene.urdf, "source_urdf_path")


# ---------------------------------------------------------------------------
# Simplification operators
# ---------------------------------------------------------------------------

class URDF_OT_Decimate(bpy.types.Operator):
    """Apply Decimate modifier to selected collision mesh objects"""
    bl_idname = "urdf.decimate_selected"
    bl_label = "Decimate Selected"
    bl_options = {'REGISTER', 'UNDO'}

    ratio: FloatProperty(
        name="Ratio",
        min=0.01, max=1.0, default=0.1, subtype='FACTOR',
    )

    @classmethod
    def poll(cls, context):
        return any(obj.type == 'MESH' for obj in context.selected_objects)

    def execute(self, context):
        count = 0
        for obj in context.selected_objects:
            if obj.type == 'MESH':
                simplify_mesh(obj, self.ratio)
                count += 1
        self.report({'INFO'}, f"Decimate applied to {count} object(s) (ratio={self.ratio:.2f})")
        return {'FINISHED'}

    def invoke(self, context, event):
        self.ratio = context.scene.urdf.simplify_ratio
        return self.execute(context)


class URDF_OT_ConvexHull(bpy.types.Operator):
    """Replace selected meshes with their convex hull"""
    bl_idname = "urdf.convex_hull"
    bl_label = "Convex Hull"
    bl_options = {'REGISTER', 'UNDO'}

    @classmethod
    def poll(cls, context):
        return any(obj.type == 'MESH' for obj in context.selected_objects)

    def execute(self, context):
        count = 0
        for obj in context.selected_objects:
            if obj.type == 'MESH':
                apply_convex_hull(obj)
                count += 1
        self.report({'INFO'}, f"Convex hull applied to {count} object(s)")
        return {'FINISHED'}


class URDF_OT_BoundingBox(bpy.types.Operator):
    """Replace selected meshes with axis-aligned bounding boxes"""
    bl_idname = "urdf.bounding_box"
    bl_label = "Bounding Box"
    bl_options = {'REGISTER', 'UNDO'}

    @classmethod
    def poll(cls, context):
        return any(obj.type == 'MESH' for obj in context.selected_objects)

    def execute(self, context):
        count = 0
        for obj in context.selected_objects:
            if obj.type == 'MESH':
                apply_bounding_box(obj)
                count += 1
        self.report({'INFO'}, f"Bounding box applied to {count} object(s)")
        return {'FINISHED'}


class URDF_OT_ApplyModifiers(bpy.types.Operator):
    """Apply all modifiers on selected objects (makes simplification permanent)"""
    bl_idname = "urdf.apply_modifiers"
    bl_label = "Apply All Modifiers"
    bl_options = {'REGISTER', 'UNDO'}

    @classmethod
    def poll(cls, context):
        return any(obj.type == 'MESH' for obj in context.selected_objects)

    def execute(self, context):
        for obj in context.selected_objects:
            if obj.type == 'MESH':
                context.view_layer.objects.active = obj
                for mod in obj.modifiers:
                    try:
                        bpy.ops.object.modifier_apply(modifier=mod.name)
                    except Exception as e:
                        print(f"[URDF] Could not apply modifier {mod.name}: {e}")
        return {'FINISHED'}


class URDF_OT_TagSelected(bpy.types.Operator):
    """Tag selected objects with URDF link name and role"""
    bl_idname = "urdf.tag_selected"
    bl_label = "Tag Selected Objects"
    bl_options = {'REGISTER', 'UNDO'}

    link_name: StringProperty(name="Link Name", default="")
    role: EnumProperty(
        name="Role",
        items=[
            ("visual", "Visual", ""),
            ("collision", "Collision", ""),
        ],
        default="collision",
    )

    def execute(self, context):
        for obj in context.selected_objects:
            if obj.type == 'MESH':
                obj["urdf_link"] = self.link_name
                obj["urdf_role"] = self.role
                obj.urdf.link_name = self.link_name
                obj.urdf.role = self.role
        return {'FINISHED'}

    def invoke(self, context, event):
        return context.window_manager.invoke_props_dialog(self)


class URDF_OT_SelectByLink(bpy.types.Operator):
    """Select all objects belonging to a specific URDF link"""
    bl_idname = "urdf.select_by_link"
    bl_label = "Select by Link"
    bl_options = {'REGISTER', 'UNDO'}

    link_name: StringProperty(name="Link Name")

    def execute(self, context):
        bpy.ops.object.select_all(action='DESELECT')
        for obj in bpy.data.objects:
            if obj.get("urdf_link") == self.link_name:
                obj.select_set(True)
        return {'FINISHED'}

    def invoke(self, context, event):
        return context.window_manager.invoke_props_dialog(self)


class URDF_OT_ShowPolyCount(bpy.types.Operator):
    """Print polygon counts for visual vs collision objects to Info bar"""
    bl_idname = "urdf.poly_count"
    bl_label = "Report Polygon Counts"

    def execute(self, context):
        vis_total = col_total = 0
        for obj in bpy.data.objects:
            if obj.type != 'MESH':
                continue
            role = obj.get("urdf_role", "")
            count = len(obj.data.polygons)
            if role == "visual":
                vis_total += count
            elif role == "collision":
                col_total += count
        self.report(
            {'INFO'},
            f"Visual: {vis_total:,} faces | Collision: {col_total:,} faces "
            f"| Reduction: {(1 - col_total/max(vis_total,1))*100:.1f}%",
        )
        return {'FINISHED'}


class URDF_OT_ToggleLimits(bpy.types.Operator):
    """Toggle the limits section open/closed for a joint"""
    bl_idname  = "urdf.toggle_limits"
    bl_label   = "Toggle Limits"
    bl_options = set()

    joint_name: bpy.props.StringProperty()

    def execute(self, context):
        key = f"_urdf_limits_open_{self.joint_name}"
        context.scene[key] = not bool(context.scene.get(key, False))
        return {'FINISHED'}


class URDF_OT_ApplyLimits(bpy.types.Operator):
    """Apply edited position limits to the slider range and Blender drivers"""
    bl_idname  = "urdf.apply_limits"
    bl_label   = "Apply Limits"
    bl_options = {'REGISTER', 'UNDO'}

    joint_name: bpy.props.StringProperty()

    def execute(self, context):
        # Find the joint frame empty
        frame = None
        for obj in bpy.data.objects:
            if (obj.type == 'EMPTY'
                    and obj.get("urdf_joint_name") == self.joint_name):
                frame = obj
                break

        if frame is None or "joint_value" not in frame:
            self.report({'WARNING'}, f"Joint frame not found: {self.joint_name!r}")
            return {'CANCELLED'}

        lower = float(frame.get("urdf_lower", 0.0))
        upper = float(frame.get("urdf_upper", 0.0))

        if lower > upper:
            self.report({'WARNING'}, "Lower limit must be <= upper limit")
            return {'CANCELLED'}

        # 1. Update the custom property UI range so the slider clamps correctly
        ui = frame.id_properties_ui("joint_value")
        ui.update(min=lower, max=upper, soft_min=lower, soft_max=upper)

        # 2. Clamp the current value within the new limits
        cur = float(frame["joint_value"])
        frame["joint_value"] = max(lower, min(upper, cur))

        # 3. Update driver expressions to embed the new origin offsets.
        #    The driver expression for a prismatic joint is:
        #      location[i] = joint_value * component + origin_offset
        #    The origin_offset doesn't change — we only need to make sure the
        #    slider min/max is updated (done above). The driver itself reads
        #    joint_value which is now clamped, so no driver rewrite needed.

        bpy.context.view_layer.update()
        self.report({'INFO'},
                    f"{self.joint_name}: limits set to [{lower:.4f}, {upper:.4f}]")
        return {'FINISHED'}


class URDF_OT_SetLimit(bpy.types.Operator):
    """Nudge a limit field (placeholder for future direct-edit support)"""
    bl_idname  = "urdf.set_limit"
    bl_label   = "Set Limit"
    bl_options = set()

    joint_name:  bpy.props.StringProperty()
    limit_field: bpy.props.StringProperty()   # "lower" or "upper"

    def execute(self, context):
        return {'FINISHED'}


# ---------------------------------------------------------------------------
# Menu entries
# ---------------------------------------------------------------------------

def menu_import(self, context):
    self.layout.operator(URDF_OT_Import.bl_idname, text="URDF Robot (.urdf)")

def menu_export(self, context):
    self.layout.operator(URDF_OT_Export.bl_idname, text="URDF Robot (.urdf)")


# ---------------------------------------------------------------------------
# Registration
# ---------------------------------------------------------------------------

CLASSES = [
    URDF_OT_Import,
    URDF_OT_Export,
    URDF_OT_Decimate,
    URDF_OT_ConvexHull,
    URDF_OT_BoundingBox,
    URDF_OT_ApplyModifiers,
    URDF_OT_TagSelected,
    URDF_OT_SelectByLink,
    URDF_OT_ShowPolyCount,
    URDF_OT_ToggleLimits,
    URDF_OT_ApplyLimits,
    URDF_OT_SetLimit,
]

def register():
    for cls in CLASSES:
        bpy.utils.register_class(cls)

def unregister():
    for cls in reversed(CLASSES):
        bpy.utils.unregister_class(cls)
