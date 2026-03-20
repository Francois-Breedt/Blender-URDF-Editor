"""
properties.py
Blender custom property groups for URDF addon settings.
"""

import bpy
from bpy.props import (
    StringProperty, FloatProperty, BoolProperty,
    EnumProperty, IntProperty,
)
from bpy.types import PropertyGroup, Object, Scene


class URDFObjectProps(PropertyGroup):
    """Per-object URDF metadata."""
    link_name: StringProperty(
        name="Link Name",
        description="URDF link this object belongs to",
        default="",
    )
    role: EnumProperty(
        name="Role",
        description="Whether this mesh is visual or collision geometry",
        items=[
            ("visual", "Visual", "Rendered visual mesh"),
            ("collision", "Collision", "Physics collision mesh"),
            ("none", "None", "Not part of URDF"),
        ],
        default="none",
    )


class URDFSceneProps(PropertyGroup):
    """Scene-level URDF import/export settings."""
    robot_name: StringProperty(
        name="Robot Name",
        description="Name of the robot (used in URDF <robot name=>)",
        default="my_robot",
    )
    source_urdf_path: StringProperty(
        name="Source URDF",
        description="Path to the original URDF file that was imported",
        subtype='FILE_PATH',
        default="",
    )

    # Import settings
    import_visual: BoolProperty(
        name="Import Visual",
        default=True,
    )
    import_collision: BoolProperty(
        name="Import Collision",
        default=True,
    )
    auto_simplify: BoolProperty(
        name="Auto-Simplify Collision",
        description="Apply Decimate modifier to collision meshes on import",
        default=True,
    )
    decimate_ratio: FloatProperty(
        name="Decimate Ratio",
        description="Target ratio of faces to keep (0.0–1.0)",
        min=0.01, max=1.0,
        default=0.1,
        subtype='FACTOR',
    )

    # Export settings
    export_dir: StringProperty(
        name="Export Directory",
        subtype='DIR_PATH',
        default="//urdf_export/",
    )
    mesh_format: EnumProperty(
        name="Mesh Format",
        items=[
            ("stl", "STL", "Binary STL — compact, widely supported"),
            ("dae", "COLLADA (.dae)", "Preserves materials and transforms"),
        ],
        default="stl",
    )
    mesh_subdir: StringProperty(
        name="Mesh Subdirectory",
        default="meshes",
    )

    # Simplification tool settings (for the N-panel buttons)
    simplify_ratio: FloatProperty(
        name="Simplify Ratio",
        min=0.01, max=1.0,
        default=0.1,
        subtype='FACTOR',
    )


def register():
    bpy.utils.register_class(URDFObjectProps)
    bpy.utils.register_class(URDFSceneProps)
    bpy.types.Object.urdf = bpy.props.PointerProperty(type=URDFObjectProps)
    bpy.types.Scene.urdf = bpy.props.PointerProperty(type=URDFSceneProps)


def unregister():
    del bpy.types.Scene.urdf
    del bpy.types.Object.urdf
    bpy.utils.unregister_class(URDFSceneProps)
    bpy.utils.unregister_class(URDFObjectProps)
