"""
panels.py  —  Sidebar UI for URDF Tools (tab: "URDF" in 3D Viewport).
"""

import bpy
import math


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def _get_joint_frames(context):
    """Return controllable (non-mimic, non-fixed) joint-frame empties, sorted."""
    frames = []
    for obj in bpy.data.objects:
        if (obj.type == 'EMPTY'
                and "urdf_joint_name" in obj
                and "joint_value" in obj
                and not obj.get("urdf_is_mimic", 0)):
            frames.append(obj)
    frames.sort(key=lambda o: o["urdf_joint_name"])
    return frames


def _limits_open_key(frame):
    """Scene custom property key used to track whether a joint's limits are expanded."""
    return f"_urdf_limits_open_{frame['urdf_joint_name']}"


# ---------------------------------------------------------------------------
# Main panel
# ---------------------------------------------------------------------------

class URDF_PT_MainPanel(bpy.types.Panel):
    bl_label       = "URDF Tools"
    bl_idname      = "URDF_PT_main"
    bl_space_type  = 'VIEW_3D'
    bl_region_type = 'UI'
    bl_category    = 'URDF'

    def draw(self, context):
        layout = self.layout
        props  = context.scene.urdf

        box = layout.box()
        box.label(text="Robot", icon='ARMATURE_DATA')
        box.prop(props, "robot_name")

        row = layout.row(align=True)
        row.operator("urdf.import", text="Import URDF", icon='IMPORT')
        row.operator("urdf.export", text="Export URDF", icon='EXPORT')


# ---------------------------------------------------------------------------
# Joint slider panel
# ---------------------------------------------------------------------------

class URDF_PT_JointSliders(bpy.types.Panel):
    bl_label       = "Joint Values"
    bl_idname      = "URDF_PT_joint_sliders"
    bl_space_type  = 'VIEW_3D'
    bl_region_type = 'UI'
    bl_category    = 'URDF'
    bl_parent_id   = 'URDF_PT_main'

    def draw(self, context):
        layout = self.layout
        frames = _get_joint_frames(context)

        if not frames:
            layout.label(text="No joints found.", icon='INFO')
            layout.label(text="Import a URDF first.")
            return

        for frame in frames:
            jtype  = frame.get("urdf_joint_type", "fixed")
            jname  = frame.get("urdf_joint_name", frame.name)
            axis   = frame.get("urdf_axis", [1, 0, 0])
            is_prismatic = jtype == "prismatic"

            box = layout.box()

            # ---- Header row ----
            header = box.row()
            header.label(text=jname, icon='BONE_DATA')
            header.label(text=jtype)

            if jtype == "fixed":
                box.label(text="Fixed — no DOF", icon='LOCKED')
                continue

            # ---- Value slider ----
            unit_label = f"axis {list(axis)}"
            box.prop(frame, '["joint_value"]', text=unit_label, slider=True)

            # ---- Limits collapsible ----
            open_key    = _limits_open_key(frame)
            is_open     = bool(context.scene.get(open_key, False))

            limits_row  = box.row()
            icon        = 'TRIA_DOWN' if is_open else 'TRIA_RIGHT'
            op_toggle   = limits_row.operator("urdf.toggle_limits",
                                              text="Limits", icon=icon,
                                              emboss=False)
            op_toggle.joint_name = jname

            if is_open:
                lim_box = box.box()

                # Lower / Upper position limits
                if is_prismatic:
                    lim_box.label(text="Position limits (m):")
                else:
                    lim_box.label(text="Position limits (rad):")

                row_lo = lim_box.row()
                row_lo.label(text="Lower:")
                op_lo = row_lo.operator("urdf.set_limit", text="",
                                        icon='TRIA_LEFT')
                op_lo.joint_name  = jname
                op_lo.limit_field = "lower"
                row_lo.prop(frame, '["urdf_lower"]', text="")

                row_hi = lim_box.row()
                row_hi.label(text="Upper:")
                op_hi = row_hi.operator("urdf.set_limit", text="",
                                        icon='TRIA_RIGHT')
                op_hi.joint_name  = jname
                op_hi.limit_field = "upper"
                row_hi.prop(frame, '["urdf_upper"]', text="")

                apply_row = lim_box.row()
                apply_op  = apply_row.operator("urdf.apply_limits",
                                               text="Apply Position Limits",
                                               icon='CHECKMARK')
                apply_op.joint_name = jname

                lim_box.separator()

                # Velocity
                lim_box.label(text="Velocity limit:")
                lim_box.prop(frame, '["urdf_velocity"]', text="")

                # Effort
                lim_box.label(text="Effort limit:")
                lim_box.prop(frame, '["urdf_effort"]', text="")


# ---------------------------------------------------------------------------
# Simplification panel
# ---------------------------------------------------------------------------

class URDF_PT_SimplifyPanel(bpy.types.Panel):
    bl_label       = "Collision Simplification"
    bl_idname      = "URDF_PT_simplify"
    bl_space_type  = 'VIEW_3D'
    bl_region_type = 'UI'
    bl_category    = 'URDF'
    bl_parent_id   = 'URDF_PT_main'

    def draw(self, context):
        layout = self.layout
        props  = context.scene.urdf

        row = layout.row(align=True)
        row.operator("urdf.select_all_visual",    text="Select Visual",    icon='RESTRICT_SELECT_OFF')
        row.operator("urdf.select_all_collision", text="Select Collision", icon='RESTRICT_SELECT_OFF')
        layout.operator("urdf.copy_visual_to_collision",
                        text="Copy Visual → Collision", icon='COPYDOWN')
        layout.separator()
        layout.label(text="Apply to selected objects:", icon='MOD_DECIM')

        col = layout.column(align=True)
        col.operator("urdf.convex_hull",             icon='MESH_ICOSPHERE')
        col.operator("urdf.bounding_box",            icon='MESH_CUBE')
        col.operator("urdf.oriented_bounding_box",   icon='PIVOT_BOUNDBOX')

        layout.separator()
        layout.operator("urdf.apply_modifiers", icon='CHECKMARK')
        layout.operator("urdf.poly_count",      icon='INFO')


# ---------------------------------------------------------------------------
# Object tag panel
# ---------------------------------------------------------------------------

class URDF_PT_ObjectTagPanel(bpy.types.Panel):
    bl_label       = "Object URDF Tags"
    bl_idname      = "URDF_PT_object_tags"
    bl_space_type  = 'VIEW_3D'
    bl_region_type = 'UI'
    bl_category    = 'URDF'
    bl_parent_id   = 'URDF_PT_main'

    @classmethod
    def poll(cls, context):
        return (context.active_object is not None
                and context.active_object.type == 'MESH')

    def draw(self, context):
        layout = self.layout
        obj    = context.active_object
        if obj is None or obj.type != 'MESH':
            layout.label(text="Select a mesh object")
            return

        box = layout.box()
        box.label(text=obj.name, icon='MESH_DATA')
        box.prop(obj.urdf, "link_name")
        box.prop(obj.urdf, "role")

        layout.operator("urdf.tag_selected",   text="Tag Selected",   icon='OBJECT_DATA')
        layout.operator("urdf.select_by_link", text="Select by Link", icon='RESTRICT_SELECT_OFF')

        col = layout.column()
        col.enabled = False
        col.label(text=f"urdf_link: {obj.get('urdf_link', '<not set>')}")
        col.label(text=f"urdf_role: {obj.get('urdf_role', '<not set>')}")


# ---------------------------------------------------------------------------
# Registration
# ---------------------------------------------------------------------------

CLASSES = [
    URDF_PT_MainPanel,
    URDF_PT_JointSliders,
    URDF_PT_SimplifyPanel,
    URDF_PT_ObjectTagPanel,
]


def register():
    for cls in CLASSES:
        bpy.utils.register_class(cls)


def unregister():
    for cls in reversed(CLASSES):
        bpy.utils.unregister_class(cls)
