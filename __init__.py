bl_info = {
    "name": "URDF Tools",
    "author": "URDF Tools",
    "version": (1, 0, 0),
    "blender": (3, 6, 0),
    "location": "File > Import/Export and View3D > Sidebar > URDF",
    "description": "Import, simplify collision geometry, and export URDF robot files",
    "category": "Import-Export",
}

import bpy
from . import operators, panels, properties

def register():
    properties.register()
    operators.register()
    panels.register()

    bpy.types.TOPBAR_MT_file_import.append(operators.menu_import)
    bpy.types.TOPBAR_MT_file_export.append(operators.menu_export)

def unregister():
    bpy.types.TOPBAR_MT_file_import.remove(operators.menu_import)
    bpy.types.TOPBAR_MT_file_export.remove(operators.menu_export)

    panels.unregister()
    operators.unregister()
    properties.unregister()

if __name__ == "__main__":
    register()
