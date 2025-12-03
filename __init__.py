"""
Point Cloud Essentials addon entry point.

Blender scans the add-on folder itself, so we declare ``bl_info`` here and
lazy-import the implementation module only when Blender actually loads us.
"""

bl_info = {
    "name": "Point Cloud Essentials",
    "author": "Olstflow",
    "version": (0, 5, 0),
    "blender": (4, 5, 0),
    "location": "View3D > Sidebar > Point Cloud Essentials",
    "description": "Fast point cloud visualization with interactive cropping.",
    "category": "3D View",
}

if "bpy" in locals():
    import importlib
    from . import pce as _pce
    importlib.reload(_pce)
else:
    from . import pce as _pce


def register():
    _pce.register()


def unregister():
    _pce.unregister()
