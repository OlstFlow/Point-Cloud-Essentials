bl_info = {
    "name": "Point Cloud Essentials",
    "author": "Olstflow, GPT & Gemini Bros",
    "version": (0, 5, 0),
    "blender": (5, 0, 0),
    "location": "View3D > Sidebar > Point Cloud Essentials",
    "description": "Fast point cloud visualization with interactive cropping.",
    "category": "3D View",
}

             
                                                               
                                                              
                                       

import bpy
import gpu
import numpy as np
import os
import sys
import struct
import math
import uuid
import hashlib
from pathlib import Path
from datetime import datetime
from mathutils import Matrix, Vector, geometry
from types import SimpleNamespace
from contextlib import contextmanager
from bpy.app.handlers import persistent
from bpy_extras import view3d_utils
from bpy_extras.io_utils import ImportHelper
from gpu_extras.batch import batch_for_shader
from bl_ui.space_toolsystem_common import ToolSelectPanelHelper

                                                                           
                    
                                                                           

class PlyPointCloudReader:
    _supported_formats = ('binary_little_endian', 'binary_big_endian', 'ascii',)
    _types = {
        'char': 'b', 'uchar': 'B', 'int8': 'b', 'uint8': 'B',
        'short': 'h', 'ushort': 'H', 'int16': 'h', 'uint16': 'H',
        'int': 'i', 'int32': 'i', 'uint': 'I', 'uint32': 'I',
        'float': 'f', 'float32': 'f', 'float64': 'd', 'double': 'd',
    }

    def __init__(self, path):
        if not os.path.exists(path): raise OSError(f"File not found: '{path}'")
        self.path = path
        self._header()
        if self._ply_format == 'ascii': self._data_ascii()
        else: self._data_binary()
        
                      
        names = ['x', 'y', 'z']
        if 'nx' in self.points.dtype.names: names.extend(['nx', 'ny', 'nz'])
        if 'red' in self.points.dtype.names: names.extend(['red', 'green', 'blue'])
        
        actual = [n for n in names if n in self.points.dtype.names]
        self.points = self.points[actual]
        
                         
        dt_names = list(self.points.dtype.names)
        for i, n in enumerate(dt_names):
            if n.startswith('diffuse_'): dt_names[i] = n.replace('diffuse_', '')
        self.points.dtype.names = tuple(dt_names)
        
        self.has_vertices = set(('x', 'y', 'z')).issubset(self.points.dtype.names)
        self.has_colors = set(('red', 'green', 'blue')).issubset(self.points.dtype.names)

    def _header(self):
        h = []
        with open(self.path, 'rb') as f:
            for line in f:
                try: l = line.decode('ascii').strip()
                except: continue
                h.append(l)
                if l == "end_header": break
        
        if not h or h[0] != 'ply': raise TypeError("Not a PLY file")
        
        self._elements = []
        cur = None
        for l in h:
            if l.startswith('format'):
                self._ply_format = l.split(' ')[1]
                self._endianness = '<' if self._ply_format == 'binary_little_endian' else '>'
            elif l.startswith('element'):
                t, c = l.split(' ')[1:3]
                cur = {'type': t, 'count': int(c), 'props': []}
                self._elements.append(cur)
            elif l.startswith('property'):
                if cur:
                    parts = l.split(' ')
                    if parts[1] in self._types: cur['props'].append((parts[2], self._types[parts[1]]))
        
                              
        with open(self.path, 'rb') as f:
            content = f.read()
            self._header_length = content.find(b"end_header") + 11

    def _data_binary(self):
        read_from = self._header_length
        for el in self._elements:
            if el['type'] != 'vertex':
                sz = sum([struct.calcsize(t) for _, t in el['props']])
                read_from += sz * el['count']
                continue
            
            dt = np.dtype([(n, f"{self._endianness}{t}") for n, t in el['props']])
            with open(self.path, 'rb') as f:
                f.seek(read_from)
                self.points = np.fromfile(f, dtype=dt, count=el['count'])
            read_from += self.points.nbytes

    def _data_ascii(self):
                                             
                              
        with open(self.path, 'r', encoding='utf-8', errors='ignore') as f:
            skip = 0
            for line in f:
                skip += 1
                if line.strip() == 'end_header': break
        
        for el in self._elements:
            if el['type'] == 'vertex':
                dt = np.dtype([(n, t) for n, t in el['props']])
                self.points = np.genfromtxt(self.path, dtype=dt, skip_header=skip, max_rows=el['count'])
                break
            else:
                                                                                          
                skip += el['count']

                                                                           
                                          
                                                                           

              
POINTCLOUD_DRAW_HANDLER = None
SHADER_POINT = None
POINT_COUNT = 0
_SOURCE_POS = None
_SOURCE_COL = None
_CLOUD_DATA = {}
_LAYER_VIS_STATE = {}
_LAYER_SHOW_STATE = {}
_LAYER_VIS_SKIP_STATE = {}
_LAYER_VIS_SKIP_STATE = {}
_LAYER_INDEX_LOCK = False
_POST_LOAD_TIMER = None
_LAST_ACTIVE_OBJECT = None
_SCREEN_GESTURE_ID = 0
_PREV_EXC_HOOK = None
_STDERR_TEE = None
_ADDON_KEYMAPS = []
AUTO_LITE_SCENES_DIRTY = set()
AUTO_LITE_TIMER_RUNNING = False
AUTO_LITE_REBUILD_LOCK = False
AUTO_LITE_INTERNAL_UPDATE = False
ENABLE_VERBOSE_LOG = True
DEFAULT_SELECTION_COLOR = (0.0, 0.4, 1.0)
DEFAULT_SELECTION_BLEND = 1.7
DEFAULT_SELECTION_RGBA = (*DEFAULT_SELECTION_COLOR, DEFAULT_SELECTION_BLEND)
MIN_SCREEN_DRAG = 4.0
IDENTITY_MATRIX_TUPLE = tuple(Matrix.Identity(4)[i][j] for i in range(4) for j in range(4))
_EDIT_SYNC_GUARD = False


def matrix_to_tuple(mat):
    if not isinstance(mat, Matrix):
        mat = Matrix(mat)
    return tuple(float(mat[row][col]) for row in range(4) for col in range(4))


def tuple_to_matrix(values):
    if not values or len(values) != 16:
        return Matrix.Identity(4)
    try:
        rows = [values[i:i + 4] for i in range(0, 16, 4)]
        return Matrix(rows)
    except Exception:
        return Matrix.Identity(4)


def transform_points_array(points, matrix):
    if points is None or matrix is None:
        return points
    pts = np.asarray(points, dtype=np.float32)
    if pts.ndim != 2 or pts.shape[1] != 3:
        return pts
    mat = np.array(matrix, dtype=np.float32)
    if mat.shape != (4, 4):
        mat = np.array(Matrix.Identity(4), dtype=np.float32)
    ones = np.ones((len(pts), 1), dtype=np.float32)
    pts4 = np.hstack((pts, ones))
    transformed = (pts4 @ mat.T)[:, :3]
    return transformed


@persistent
def on_load_post(_):
    global _POST_LOAD_TIMER
    def _refresh():
        global _POST_LOAD_TIMER
        log_runtime_event("load_post timer fired")
        for scene in bpy.data.scenes:
            refresh_scene_layers(scene)
        _POST_LOAD_TIMER = None
        return None
    if _POST_LOAD_TIMER is None:
        _POST_LOAD_TIMER = bpy.app.timers.register(_refresh, first_interval=0.1)


def _mark_scene_auto_lite_dirty(scene):
    """
    Mark scene as needing a deferred Lite rebuild. Heavy operations will be
    executed from auto_lite_timer, not directly from depsgraph callbacks.
    """
    global AUTO_LITE_SCENES_DIRTY, AUTO_LITE_TIMER_RUNNING
    if scene is None:
        return
    try:
        AUTO_LITE_SCENES_DIRTY.add(scene)
    except Exception:
        return
    if not AUTO_LITE_TIMER_RUNNING:
        try:
            bpy.app.timers.register(_auto_lite_timer, first_interval=0.2, persistent=True)
            AUTO_LITE_TIMER_RUNNING = True
        except Exception:
            AUTO_LITE_TIMER_RUNNING = False


def schedule_full_layer_rebuild(scene, reason="auto_lite"):
    """
    Unified entry point to request a full Lite layer rebuild for a scene.
    The actual work is performed later in _auto_lite_timer.
    """
    _mark_scene_auto_lite_dirty(scene)
    log_runtime_event(f"schedule_full_layer_rebuild scene='{getattr(scene, 'name', None)}' reason='{reason}'")


def _lite_rebuild_scene_layers(scene, context=None, reason="colored"):
    """
    Full Lite rebuild used both by the experimental operator and by the
    Colored toggle when a fresh capture of all point clouds is desired.
    """
    if scene is None or not hasattr(scene, "pcv_layers"):
        return False
    ctx = context or bpy.context
    props = getattr(scene, "pcv_props", None)
    layers = getattr(scene, "pcv_layers", [])
    had_layers_before = bool(layers)
                                                                   
                                                                       
                                                                          
    saved_active_name = None
    saved_selected_names = None
    real_ctx = bpy.context
    if real_ctx and getattr(real_ctx, "scene", None) is scene:
        view_layer = getattr(real_ctx, "view_layer", None)
        if view_layer:
            try:
                active_obj = view_layer.objects.active
            except Exception:
                active_obj = getattr(real_ctx, "active_object", None)
            saved_active_name = getattr(active_obj, "name", None)
            selected = []
            try:
                for obj in view_layer.objects:
                    try:
                        if obj.select_get():
                            selected.append(obj.name)
                    except Exception:
                        continue
            except Exception:
                selected = []
            saved_selected_names = selected

                                                                
                                                               
                                      
    source_on = any(getattr(layer, "show_source_mesh", False) for layer in layers)

                                                       
    for layer in layers:
        if getattr(layer, "show_points", False):
            layer.show_points = False
            refresh_layer_visibility(layer, ctx)
    update_draw_handler_for_scene(scene)
    request_redraw()

                                                             
    for idx in reversed(range(len(scene.pcv_layers))):
        layer = scene.pcv_layers[idx]
        remove_layer_data(layer)
        scene.pcv_layers.remove(idx)

    if props:
        props.lite_enabled = False

                                                                       
    added, removed = ensure_lite_layers(scene, ctx)
    refresh_scene_layers(scene)

    layers = getattr(scene, "pcv_layers", [])
    if not len(layers):
        log_runtime_event(f"lite_rebuild_scene_layers scene='{getattr(scene, 'name', None)}' reason='{reason}' result='empty'")
        return False

                                                                                 
    for layer in layers:
        layer.show_points = True
        if source_on:
            layer.show_source_mesh = True
        refresh_layer_visibility(layer, ctx)

    update_draw_handler_for_scene(scene)
    request_redraw()

    if props:
        props.lite_enabled = True

                                                                        
                                                                  
                                                                     
                                                                   
    if real_ctx and getattr(real_ctx, "scene", None) is scene:
        view_layer = getattr(real_ctx, "view_layer", None)
        if view_layer:
            try:
                active_obj = None
                if had_layers_before and saved_selected_names is not None:
                                                                       
                    for obj in view_layer.objects:
                        try:
                            obj.select_set(False)
                        except Exception:
                            continue
                    for name in saved_selected_names:
                        obj = scene.objects.get(name)
                        if obj:
                            try:
                                obj.select_set(True)
                            except Exception:
                                continue
                    if saved_active_name:
                        active_obj = scene.objects.get(saved_active_name)
                        if active_obj:
                            try:
                                view_layer.objects.active = active_obj
                            except Exception:
                                active_obj = None
                elif not had_layers_before:
                                                                          
                                                                 
                    active_layer = get_active_layer_from_scene(scene)
                    if active_layer:
                        src = get_layer_source_object(active_layer, scene)
                        parent = getattr(src, "parent", None) if src else None
                        target = parent or src
                        if target:
                            for obj in view_layer.objects:
                                try:
                                    obj.select_set(False)
                                except Exception:
                                    continue
                            try:
                                target.select_set(True)
                            except Exception:
                                pass
                            try:
                                view_layer.objects.active = target
                                active_obj = target
                            except Exception:
                                active_obj = None
                                                                     
                                                                       
                                                                     
                if active_obj is not None:
                    set_active_layer_for_object(scene, active_obj)
            except Exception:
                pass

    log_runtime_event(
        f"lite_rebuild_scene_layers scene='{getattr(scene, 'name', None)}' reason='{reason}' "
        f"layers={len(layers)} added={added} removed={removed}"
    )
    return True


def _auto_lite_timer():
    """
    Timer worker that performs heavy Lite layer rebuilds in a safe context,
    outside of depsgraph callbacks.
    """
    global AUTO_LITE_SCENES_DIRTY, AUTO_LITE_TIMER_RUNNING
    global AUTO_LITE_REBUILD_LOCK, AUTO_LITE_INTERNAL_UPDATE

                                                        
    any_enabled = False
    for scene in bpy.data.scenes:
        props = getattr(scene, "pcv_props", None)
        if props and getattr(props, "lite_enabled", False):
            any_enabled = True
            break
    if not any_enabled:
        AUTO_LITE_SCENES_DIRTY.clear()
        AUTO_LITE_TIMER_RUNNING = False
        return None

                                                                
    wm = getattr(bpy.context, "window_manager", None)
    if wm and getattr(wm, "is_interface_locked", False):
        return 0.5

                          
    if AUTO_LITE_REBUILD_LOCK:
        return 0.2

    if not AUTO_LITE_SCENES_DIRTY:
                                                             
        return 0.2

                                        
    try:
        scene = next(iter(AUTO_LITE_SCENES_DIRTY))
    except StopIteration:
        return 0.2

    if scene is None or getattr(scene, "name", None) not in bpy.data.scenes:
        AUTO_LITE_SCENES_DIRTY.discard(scene)
        return 0.2

    props = getattr(scene, "pcv_props", None)
    if not props or not getattr(props, "lite_enabled", False):
        AUTO_LITE_SCENES_DIRTY.discard(scene)
        return 0.2

    AUTO_LITE_REBUILD_LOCK = True
    AUTO_LITE_INTERNAL_UPDATE = True
    try:
        ctx = SimpleNamespace(scene=scene, active_object=None)
        ok = _lite_rebuild_scene_layers(scene, ctx, reason="auto_lite_timer")
        AUTO_LITE_SCENES_DIRTY.discard(scene)
        if ok:
            log_runtime_event(f"auto_lite_timer_rebuild scene='{scene.name}'")
        else:
            log_runtime_event(f"auto_lite_timer_rebuild scene='{scene.name}' result='empty'")
    except Exception:
                                                 
        AUTO_LITE_SCENES_DIRTY.discard(scene)
    finally:
        AUTO_LITE_INTERNAL_UPDATE = False
        AUTO_LITE_REBUILD_LOCK = False

    return 0.2


@persistent
def on_depsgraph_update(scene, depsgraph):
    global _EDIT_SYNC_GUARD
    global _LAST_ACTIVE_OBJECT
    global AUTO_LITE_INTERNAL_UPDATE
    if _EDIT_SYNC_GUARD:
        return
                                                                      
                                                                      
                                                                    
    if not scene or not hasattr(scene, "pcv_layers"):
        return
    ctx = bpy.context
    if ctx and ctx.scene == scene:
        view_layer = getattr(ctx, "view_layer", None)
        active_container = getattr(view_layer, "objects", None)
        active_obj = active_container.active if active_container else None
        current_name = active_obj.name if active_obj else None
        if current_name != _LAST_ACTIVE_OBJECT:
            _LAST_ACTIVE_OBJECT = current_name
            if active_obj:
                set_active_layer_for_object(scene, active_obj)
                                                                          
                                                                        
                                                                          
                
        props_ctx = getattr(ctx.scene, "pcv_props", None)
        if props_ctx and getattr(props_ctx, "screen_edit_enable", False):
            ensure_workspace_screen_tool(ctx, props_ctx.screen_tool)

                                                                         
                                                                             
    props = getattr(scene, "pcv_props", None)
    if props and getattr(props, "lite_enabled", False) and getattr(props, "enable_auto_lite", False) and not AUTO_LITE_INTERNAL_UPDATE:
                                                                           
                                                                          
                                                                         
        op_id = None
        wm = getattr(ctx, "window_manager", None) if ctx else None
        if wm:
            ops = getattr(wm, "operators", None)
            if ops:
                try:
                    last_op = ops[-1]
                    op_id = getattr(last_op, "bl_idname", None)
                except Exception:
                    op_id = None
                                                                                
                                                                                
                                                             
                                                                                            
                                                                   
                                                                         
                                                                            
        existing = set()
        for layer in scene.pcv_layers:
            key = (getattr(layer, "filepath", "") or "").strip()
            if key:
                existing.add(key)
                                                                             
                                                                           
                                                                          
        if existing:
            scene_object_names = {obj.name for obj in getattr(scene, "objects", [])}
            missing_sources = any(name not in scene_object_names for name in existing)
            if missing_sources:
                schedule_full_layer_rebuild(scene, reason="join_or_delete")
        for update in getattr(depsgraph, "updates", []):
            obj = getattr(update, "id", None)
            if not isinstance(obj, bpy.types.Object):
                continue
            if getattr(obj, "type", None) != 'MESH':
                continue
            real_obj = scene.objects.get(obj.name)
            if real_obj is None:
                continue
            data = getattr(real_obj, "data", None)
            verts = getattr(data, "vertices", None) if data else None
            if not verts or len(verts) == 0:
                continue
            polys = getattr(data, "polygons", None) if data else None
            if polys is not None and len(polys) != 0:
                continue
            if real_obj.name in existing:
                continue
            schedule_full_layer_rebuild(scene, reason="auto_lite")
            break
    layers_by_source = {}
    for layer in scene.pcv_layers:
        name = (getattr(layer, "filepath", "") or "").strip()
        if name:
            layers_by_source.setdefault(name, []).append(layer)
    if not layers_by_source:
        return
    for update in getattr(depsgraph, "updates", []):
        obj = getattr(update, "id", None)
        if not isinstance(obj, bpy.types.Object):
            continue
        if obj.name not in layers_by_source:
            continue
        data = getattr(obj, "data", None)
        in_edit = bool(getattr(data, "is_editmode", False)) if data else False
        for layer in layers_by_source[obj.name]:
            layer_data = get_layer_data(layer)
            if not layer_data:
                continue
            was_in_edit = layer_data.get("was_in_edit", False)
            if in_edit:
                layer_data["was_in_edit"] = True
                continue
            if not was_in_edit:
                continue
            layer_data["was_in_edit"] = False
            if obj.type != 'MESH':
                continue
            props = getattr(scene, "pcv_props", None)
            context = SimpleNamespace(scene=scene, active_object=obj)
            try:
                _EDIT_SYNC_GUARD = True
                capture_object_to_layer(obj, layer, props, context)
            finally:
                _EDIT_SYNC_GUARD = False

def request_redraw():
    ctx = bpy.context
    wm = ctx.window_manager if ctx and ctx.window_manager else None
    if not wm:
        return
    for window in wm.windows:
        screen = window.screen
        if not screen:
            continue
        for area in screen.areas:
            if area.type == 'VIEW_3D':
                area.tag_redraw()

def mark_layer_dirty(layer):
    data = get_layer_data(layer)
    if data:
        data["dirty"] = True

def set_globals_from_layer(layer):
    global _SOURCE_POS, _SOURCE_COL
    if not layer:
        _SOURCE_POS = None
        _SOURCE_COL = None
        return
    data = get_layer_data(layer)
    if not data:
        _SOURCE_POS = None
        _SOURCE_COL = None
        return
    _SOURCE_POS = data.get("source_pos")
    _SOURCE_COL = data.get("source_col")


def layer_has_draw_data(layer):
    if not layer or not getattr(layer, "uid", ""):
        return False
    data = _CLOUD_DATA.get(layer.uid)
    return bool(data and data.get("batch"))


def any_visible_layers(scene):
    if not scene:
        return False
    layers = getattr(scene, "pcv_layers", [])
    for layer in layers:
        if layer.show_points and layer_has_draw_data(layer):
            return True
    return False


def update_draw_handler_for_scene(scene):
    if any_visible_layers(scene):
        ensure_pointcloud_draw_handler()
    else:
        remove_pointcloud_draw_handler()


def get_layer_source_object(layer, scene=None):
    if not layer:
        return None
    if scene is None:
        scene = getattr(layer, "id_data", None)
        if scene is None:
            ctx = bpy.context
            scene = getattr(ctx, "scene", None) if ctx else None
    name = (getattr(layer, "filepath", "") or "").strip()
    if not scene or not name or name not in scene.objects:
        return None
    return scene.objects.get(name)


SOURCE_DISPLAY_TYPE_KEY = "_pcv_prev_display_type"
SOURCE_SHOW_IN_FRONT_KEY = "_pcv_prev_show_in_front"


def _set_source_overlay_state(obj, enable):
    if not obj:
        return
    if enable:
        if SOURCE_DISPLAY_TYPE_KEY not in obj:
            try:
                obj[SOURCE_DISPLAY_TYPE_KEY] = str(getattr(obj, "display_type", 'TEXTURED'))
            except Exception:
                obj[SOURCE_DISPLAY_TYPE_KEY] = 'TEXTURED'
        try:
            obj.display_type = 'WIRE'
        except Exception:
            pass
        if SOURCE_SHOW_IN_FRONT_KEY not in obj:
            try:
                obj[SOURCE_SHOW_IN_FRONT_KEY] = int(bool(getattr(obj, "show_in_front", False)))
            except Exception:
                obj[SOURCE_SHOW_IN_FRONT_KEY] = 0
        try:
            obj.show_in_front = True
        except Exception:
            pass
    else:
        if SOURCE_DISPLAY_TYPE_KEY in obj:
            prev = obj.pop(SOURCE_DISPLAY_TYPE_KEY, None)
            if prev is not None:
                try:
                    obj.display_type = prev
                except Exception:
                    pass
        if SOURCE_SHOW_IN_FRONT_KEY in obj:
            prev_front = bool(obj.pop(SOURCE_SHOW_IN_FRONT_KEY, 0))
            try:
                obj.show_in_front = prev_front
            except Exception:
                pass


def toggle_source_mesh_visibility(layer, hide=True, overlay=False):
    obj = get_layer_source_object(layer)
    if not obj:
        return
    _set_source_overlay_state(obj, (not hide) and overlay)
    try:
        obj.hide_set(hide)
    except Exception:
        obj.hide_viewport = hide
    obj.hide_viewport = hide
    obj.hide_render = hide
    obj.hide_select = hide


def find_layer_collection(layer_collection, name):
    """Recursively find a layer collection by name."""
    if layer_collection is None:
        return None
    if layer_collection.name == name:
        return layer_collection
    for child in getattr(layer_collection, "children", []):
        result = find_layer_collection(child, name)
        if result:
            return result
    return None


def is_any_collection_hidden(obj, context=None):
    """
    Return True when all collections that link the object are hidden.
    This mirrors the sandbox logic where hiding a collection should
    hide both the source mesh and the GLSL overlay.
    """
    if not obj:
        return False
    collections = list(getattr(obj, "users_collection", []) or [])
    if not collections:
        return False
    ctx = context or bpy.context
    view_layer = getattr(ctx, "view_layer", None) if ctx else None
    if view_layer is None:
        base_ctx = bpy.context
        view_layer = getattr(base_ctx, "view_layer", None)
    root = getattr(view_layer, "layer_collection", None) if view_layer else None
    if root is None:
        return False
    for coll in collections:
        layer_coll = find_layer_collection(root, coll.name)
        if layer_coll and not getattr(layer_coll, "hide_viewport", False) and getattr(layer_coll, "is_visible", True):
            return False
    return True


def is_object_truly_visible(obj, context=None):
    """Check whether the source object is effectively visible in the viewport."""
    if not obj:
        return False
    if getattr(obj, "hide_viewport", False):
        return False
    ctx = context or bpy.context
    view_layer = getattr(ctx, "view_layer", None) if ctx else None
    try:
        if view_layer:
            if not obj.visible_get(view_layer=view_layer):
                return False
        else:
            if not obj.visible_get():
                return False
    except Exception:
        return False
    if is_any_collection_hidden(obj, context):
        return False
    return True


def ensure_layer_rig_object(layer, context=None, source_obj=None):
    """
    Create or return a proxy rig object that mirrors the source transform.

    - If the source already has a parent (empty/controller), that parent *is*
      treated as the rig – no extra objects are created and the hierarchy is
      not modified.
    - If there is no parent, a helper rig (`*_Rig`) is created and marked with
      `_pcv_helper_rig` so it can be treated as an internal technical object.

    This matches the UX logic from Userlogic_Lite: user drives the cloud
    through normal scene objects, while helper rigs appear only when strictly
    necessary.
    """
    if not layer:
        return None
    scene = getattr(layer, "id_data", None)
    if scene is None and context:
        scene = getattr(context, "scene", None)
    if source_obj is None:
        source_obj = get_layer_source_object(layer, scene)
    if source_obj is None:
        return None

    existing_rig = getattr(layer, "rig_object", None)

                                                                           
                                                               
    parent = getattr(source_obj, "parent", None)
    if parent and parent.name in bpy.data.objects:
        if existing_rig and existing_rig is not parent:
                                                                                
            if getattr(existing_rig, "_pcv_helper_rig", False):
                try:
                    for coll in list(existing_rig.users_collection):
                        try:
                            coll.objects.unlink(existing_rig)
                        except Exception:
                            pass
                    bpy.data.objects.remove(existing_rig)
                except Exception:
                    pass
        layer.rig_object = parent
        return parent

                                                          
    if existing_rig and existing_rig.name in bpy.data.objects:
        return existing_rig

    rig_name = f"{source_obj.name}_Rig"
    rig = bpy.data.objects.new(rig_name, None)
    rig.empty_display_type = 'PLAIN_AXES'
    try:
        base_size = float(getattr(source_obj, "dimensions", Vector()).length)
    except Exception:
        base_size = 0.0
    if base_size <= 0.0:
        base_size = 0.1
    rig.empty_display_size = max(base_size * 0.25, 0.1)

    collections = list(getattr(source_obj, "users_collection", []) or [])
    if not collections and scene:
        collections = [scene.collection]
    for coll in collections:
        try:
            if rig.name not in coll.objects:
                coll.objects.link(rig)
        except Exception:
            continue

    rig.matrix_world = source_obj.matrix_world.copy()
    rig.hide_viewport = False
    rig.hide_render = False
    rig.hide_select = True
    try:
        rig["_pcv_helper_rig"] = True
    except Exception:
        pass

    layer.rig_object = rig
    log_runtime_event(
        f"rig_created layer='{getattr(layer, 'name', None)}' rig='{rig.name}' parent='{parent.name if parent else None}'"
    )
    return rig


def sync_matrix_between_objects(from_obj, to_obj, context=None):
    """
    Copy world transform from one object to another, preserving parenting state.
    """
    if not from_obj or not to_obj:
        return
    try:
        world_mat = from_obj.matrix_world.copy()
    except Exception:
        return
    to_obj.matrix_world = world_mat
    log_runtime_event(
        f"sync_matrix from='{getattr(from_obj, 'name', None)}' to='{getattr(to_obj, 'name', None)}' "
        f"from_mat=({_format_matrix_short(world_mat)}) "
        f"to_mat=({_format_matrix_short(to_obj.matrix_world)})"
    )
    target_ctx = context or bpy.context
    if target_ctx:
        view_layer = getattr(target_ctx, "view_layer", None)
        if view_layer:
            try:
                view_layer.update()
            except Exception:
                pass


def _set_layer_driver(layer, use_source, context=None, source_obj=None):
    """
    Choose which object drives GLSL transforms. When the source mesh is visible
    we read transforms directly from it; otherwise we switch to the rig proxy
    while keeping both matrices in sync to avoid orientation flips.
    """
    if not layer:
        return None
    scene = getattr(layer, "id_data", None)
    if scene is None and context:
        scene = getattr(context, "scene", None)
    if source_obj is None:
        source_obj = get_layer_source_object(layer, scene)
    if source_obj is None:
        return None

    rig = ensure_layer_rig_object(layer, context=context, source_obj=source_obj)
    is_helper_rig = bool(rig and getattr(rig, "_pcv_helper_rig", False))
    parent_is_rig = bool(rig and getattr(source_obj, "parent", None) is rig)
    current_driver = getattr(layer, "transform_object", None)
    driver_label = "SOURCE" if use_source else "RIG"

    if use_source or not rig:
                                                                          
                                                                          
                                             
        if is_helper_rig and current_driver == rig and rig:
            sync_matrix_between_objects(rig, source_obj, context)
            log_runtime_event(
                f"driver_switch layer='{getattr(layer, 'name', None)}' rig_to_source=1"
            )
        layer.transform_object = source_obj
        log_runtime_event(
            f"driver_active layer='{getattr(layer, 'name', None)}' driver={driver_label} obj='{source_obj.name}'"
        )
        return source_obj

                                                                          
                                                                           
                                                     
    if is_helper_rig and current_driver == source_obj:
        sync_matrix_between_objects(source_obj, rig, context)
        log_runtime_event(
            f"driver_switch layer='{getattr(layer, 'name', None)}' source_to_rig=1"
        )

    layer.transform_object = rig
    if is_helper_rig:
        rig.hide_viewport = False
        rig.hide_render = False
        rig.hide_select = True
    log_runtime_event(
        f"driver_active layer='{getattr(layer, 'name', None)}' driver={driver_label} obj='{rig.name}'"
    )
    return rig


def ensure_layer_transform_driver(layer, context=None, source_obj=None):
    return _set_layer_driver(layer, True, context=context, source_obj=source_obj)


def refresh_layer_visibility(layer, context=None):
    if not layer:
        return
    scene = getattr(layer, "id_data", None)
    if scene is None and context:
        scene = getattr(context, "scene", None)
    source = get_layer_source_object(layer, scene)
    show_layer = bool(getattr(layer, "show_points", False))
    show_source_toggle = bool(getattr(layer, "show_source_mesh", False))
    show_source = bool(show_source_toggle)
    overlay = bool(show_layer and show_source_toggle)
                                                                      
                                                                    
                                                                    
                                                                    
                                                     
    use_source_driver = bool(source and show_source)
    driver_target = 'SOURCE' if use_source_driver else 'RIG'
    uid = ensure_layer_uid(layer)
    key = (show_layer, show_source, driver_target)
    prev = _LAYER_VIS_STATE.get(uid)
    _LAYER_VIS_STATE[uid] = key
    active_layer = get_active_layer_from_scene(scene) if scene else None
    if prev != key and (active_layer is None or active_layer == layer):
        src_mat = None
        rig_mat = None
        try:
            src_obj = source
            rig_obj = getattr(layer, "rig_object", None)
            src_mat = _format_matrix_short(src_obj.matrix_world) if src_obj else "none"
            rig_mat = _format_matrix_short(rig_obj.matrix_world) if rig_obj else "none"
        except Exception:
            src_mat = rig_mat = "unavailable"
        log_runtime_event(
            f"layer_visibility layer='{getattr(layer, 'name', None)}' "
            f"show_points={int(show_layer)} show_source={int(show_source)} driver={driver_target} "
            f"src_mat=({src_mat}) rig_mat=({rig_mat})"
        )
    _set_layer_driver(layer, use_source_driver, context=context, source_obj=source)
    toggle_source_mesh_visibility(layer, hide=not show_source, overlay=overlay)
    if show_source:
        data = get_layer_data(layer)
        if data and data.get("selection_mask") is not None:
            log_screen_event("clear_selection_on_show_source", layer=getattr(layer, "name", None))
            set_layer_selection(layer, None)


def show_source_update(self, context):
    refresh_layer_visibility(self, context)
    request_redraw()


def remove_layer_transform_object(layer):
    if not layer:
        return
    if hasattr(layer, "transform_object"):
        layer.transform_object = None
    if hasattr(layer, "rig_object"):
        layer.rig_object = None
    if hasattr(layer, "transform_offset"):
        layer.transform_offset = IDENTITY_MATRIX_TUPLE


def _object_world_matrix(obj, depsgraph=None, depth=0):
    if obj is None or depth > 32:
        return Matrix.Identity(4)
    if depsgraph:
        try:
            eval_obj = obj.evaluated_get(depsgraph)
            if eval_obj:
                return eval_obj.matrix_world.copy()
        except Exception:
            pass
    parent = getattr(obj, "parent", None)
    if parent:
        parent_matrix = _object_world_matrix(parent, depsgraph, depth + 1)
        try:
            parent_inv = obj.matrix_parent_inverse.copy()
        except Exception:
            parent_inv = Matrix.Identity(4)
        try:
            basis = obj.matrix_basis.copy()
        except Exception:
            try:
                basis = obj.matrix_local.copy()
            except Exception:
                basis = Matrix.Identity(4)
        return parent_matrix @ parent_inv @ basis
    try:
        return obj.matrix_basis.copy()
    except Exception:
        try:
            return obj.matrix_local.copy()
        except Exception:
            return Matrix.Identity(4)


def get_layer_transform_matrix(layer):
    ctrl = getattr(layer, "transform_object", None)
    ctx = getattr(bpy, "context", None)
    depsgraph = None
    if ctx:
        try:
            depsgraph = ctx.evaluated_depsgraph_get()
        except Exception:
            depsgraph = None
    base = Matrix.Identity(4)
    if ctrl and ctrl.name in bpy.data.objects:
        base = _object_world_matrix(ctrl, depsgraph)
    offset = Matrix.Identity(4)
    if getattr(layer, "rig_object", None) and ctrl == layer.rig_object:
        offset_data = getattr(layer, "transform_offset", None)
        offset = tuple_to_matrix(offset_data) if offset_data else Matrix.Identity(4)
    return base @ offset


def schedule_layer_refresh(context=None):
    context = context or bpy.context
    scene = getattr(context, "scene", None) if context else None
    if not scene:
        return
    for layer in scene.pcv_layers:
        data = get_layer_data(layer)
        if data:
            data["dirty"] = True
    request_redraw()


def mark_sectionbox_needs_fit(context=None):
    settings = get_sectionbox_settings(context)
    if settings:
        settings.auto_fit_done = False
    return settings


def capture_cropbox_state(settings):
    if not settings:
        return None
    return (
        tuple(settings.center),
        tuple(settings.size),
        float(settings.radius),
        float(settings.height),
    )


def restore_cropbox_state(settings, state):
    if not settings or state is None:
        return
    center, size, radius, height = state
    settings.center = center
    settings.size = size
    settings.radius = radius
    settings.height = height
    settings.auto_fit_done = True


def is_pointcloud_candidate(obj):
    if obj is None or obj.type != 'MESH':
        return False
    data = obj.data
    if not data or len(data.vertices) == 0:
        return False
    if len(data.polygons) > 0:
        return False
    return True


def detect_pointcloud_objects(scene):
    if not scene:
        return []
    return [obj for obj in scene.objects if is_pointcloud_candidate(obj)]


def ensure_cloud_parent(scene, obj, context=None):
    """
    Ensure that a point-cloud source mesh has a dedicated parent empty
    that can be used as a stable rig/controller instead of creating
    a hidden helper rig. If a parent already exists, it is reused.
    """
    if obj is None or obj.type != 'MESH':
        return None
    if getattr(obj, "parent", None):
        return obj.parent

    collections = list(getattr(obj, "users_collection", []) or [])
    if not collections and scene:
        collections = [scene.collection]

    base_name = "Cloud"
    existing = {o.name for o in scene.objects} if scene else set()
    index = 1
    while True:
        candidate = f"{base_name}.{index:03d}" if index > 0 else base_name
        if candidate not in existing:
            break
        index += 1

    empty = bpy.data.objects.new(candidate, None)
    empty.empty_display_type = 'PLAIN_AXES'
    try:
        base_size = float(getattr(obj, "dimensions", Vector()).length)
    except Exception:
        base_size = 0.0
    if base_size <= 0.0:
        base_size = 1.0
                                                                       
                                                             
    size = base_size * 0.02
    empty.empty_display_size = max(0.05, min(size, 1.0))

    for coll in collections:
        try:
            coll.objects.link(empty)
        except Exception:
            continue

                                                          
    empty.matrix_world = obj.matrix_world.copy()
    obj.parent = empty
    try:
        obj.matrix_parent_inverse = empty.matrix_world.inverted()
    except Exception:
        pass

                                                                     
                                                                      
                                                                       
                              
    base_ctx = bpy.context
    if base_ctx and getattr(base_ctx, "scene", None) is scene:
        view_layer = getattr(base_ctx, "view_layer", None)
        if view_layer:
            try:
                active_obj = view_layer.objects.active
            except Exception:
                active_obj = getattr(base_ctx, "active_object", None)
            if active_obj is obj:
                try:
                    obj.select_set(False)
                except Exception:
                    pass
                try:
                    empty.select_set(True)
                except Exception:
                    pass
                try:
                    view_layer.objects.active = empty
                except Exception:
                    pass

    return empty


def ensure_lite_layers(scene, context=None):
    if scene is None or not hasattr(scene, "pcv_layers"):
        return 0, 0
    props = getattr(scene, "pcv_props", None)
    ctx = context or SimpleNamespace(scene=scene, active_object=None)
    existing = {}
    for layer in scene.pcv_layers:
        key = (getattr(layer, "filepath", "") or "").strip()
        if key:
            existing[key] = layer
    detected = detect_pointcloud_objects(scene)
    keep = set()
    added = 0
    for obj in detected:
        ensure_cloud_parent(scene, obj, context=ctx)
        keep.add(obj.name)
        layer = existing.get(obj.name)
        if layer is None:
            layer = scene.pcv_layers.add()
            layer.name = obj.name
            layer.filepath = obj.name
            layer.show_points = True
            ensure_layer_uid(layer)
            added += 1
        capture_object_to_layer(obj, layer, props, ctx)
    removed = 0
    for idx in reversed(range(len(scene.pcv_layers))):
        layer = scene.pcv_layers[idx]
        if (getattr(layer, "filepath", "") or "").strip() not in keep:
            remove_layer_data(layer)
            scene.pcv_layers.remove(idx)
            removed += 1
    if len(scene.pcv_layers) > 0:
                                                                     
                                                                      
                                                                   
        best_index = None
        best_count = -1
        for idx, layer in enumerate(scene.pcv_layers):
            data = get_layer_data(layer)
            src_pos = data.get("source_pos") if data else None
            count = len(src_pos) if src_pos is not None else 0
            if count > best_count:
                best_count = count
                best_index = idx
        if best_index is None:
                                                              
            scene.pcv_layers_index = max(0, min(scene.pcv_layers_index, len(scene.pcv_layers) - 1))
        else:
            scene.pcv_layers_index = best_index
        sync_props_from_layer(ctx)
    return added, removed


def set_active_layer_for_object(scene, obj):
    if scene is None or obj is None or not getattr(scene, "pcv_layers", None):
        return False
    target = (obj.name or "").strip()
    if not target:
        return False
    for idx, layer in enumerate(scene.pcv_layers):
        if (getattr(layer, "filepath", "") or "").strip() == target:
            if scene.pcv_layers_index != idx:
                scene.pcv_layers_index = idx
                sync_props_from_layer(SimpleNamespace(scene=scene))
            return True
    return False


def _get_layer_display_object(layer, scene=None, context=None):
    obj = getattr(layer, "rig_object", None)
    if obj and obj.name in bpy.data.objects:
        return obj
    scene = scene or getattr(layer, "id_data", None)
    source = get_layer_source_object(layer, scene)
    if layer:
        ensure_layer_rig_object(layer, context=context, source_obj=source)
        rig = getattr(layer, "rig_object", None)
        if rig and rig.name in bpy.data.objects:
            return rig
    return source


def layer_visible_in_view(layer, context):
    """
    The Scene Collection (master collection) is special: Blender evaluates
    per-object visibility through the active View Layer. We need to respect
    whatever the user hid/excluded there so the Lite toggles stay in sync
    with the viewport hierarchy.
    """
    if layer is None:
        return True
    ctx = context or bpy.context
    scene = getattr(ctx, "scene", None) if ctx else None
    source = get_layer_source_object(layer, scene)
    display_obj = _get_layer_display_object(layer, scene=scene, context=context)
    def _clear_skip():
        uid = ensure_layer_uid(layer)
        if uid in _LAYER_VIS_SKIP_STATE:
            _LAYER_VIS_SKIP_STATE.pop(uid, None)
    def _log_skip(reason, **info):
        uid = ensure_layer_uid(layer)
        key = (reason, tuple(sorted(info.items())))
        prev = _LAYER_VIS_SKIP_STATE.get(uid)
        if prev == key:
            return
        _LAYER_VIS_SKIP_STATE[uid] = key
        active_layer = get_active_layer_from_scene(scene) if scene else None
        if active_layer is None or active_layer == layer:
            payload = " ".join(f"{k}={info[k]}" for k in sorted(info))
            log_runtime_event(
                f"layer_visible_skip layer='{getattr(layer, 'name', None)}' reason={reason} {payload}"
            )
    if display_obj is None:
        _clear_skip()
        return True

    view_layer = getattr(ctx, "view_layer", None) if ctx else None
    if view_layer is None:
        base_ctx = bpy.context
        view_layer = getattr(base_ctx, "view_layer", None)
    space = getattr(ctx, "space_data", None) if ctx else None
    viewport = space if getattr(space, "type", None) == 'VIEW_3D' else None

    visible = True
    visibility_checked = False
    if viewport is not None:
        try:
            visible = bool(display_obj.visible_in_viewport_get(viewport))
            visibility_checked = True
        except Exception:
            pass
    if not visibility_checked:
        try:
            if view_layer is not None:
                visible = bool(display_obj.visible_get(view_layer=view_layer))
            else:
                visible = bool(display_obj.visible_get())
        except Exception:
            visible = True
    if visible:
        _clear_skip()
        return True

    hidden_direct = bool(getattr(display_obj, "hide_viewport", False))
    try:
        if view_layer is not None:
            hidden_direct = hidden_direct or bool(display_obj.hide_get(view_layer=view_layer))
        else:
            hidden_direct = hidden_direct or bool(display_obj.hide_get())
    except Exception:
        hidden_direct = hidden_direct or bool(getattr(display_obj, "hide_viewport", False))

    show_source_mesh = bool(getattr(layer, "show_source_mesh", True))
    show_points = int(bool(getattr(layer, "show_points", False)))

    if hidden_direct and not show_source_mesh:
        _clear_skip()
        return True

    reason = "hidden_hierarchy" if hidden_direct else "not_visible"
    _log_skip(
        reason,
        visible=int(visible),
        hidden_direct=int(hidden_direct),
        show_source_mesh=int(show_source_mesh),
        show_points=show_points,
    )
    return False


def refresh_scene_layers(scene):
    if scene is None or not hasattr(scene, "pcv_layers"):
        return
    ensure_lite_layers(scene)
    log_runtime_event(f"refresh_scene_layers on scene '{scene.name}' with {len(scene.pcv_layers)} layers")
    fake_context = SimpleNamespace(scene=scene, active_object=None)
    any_visible = False
    for layer in scene.pcv_layers:
        if not getattr(layer, "show_points", False):
            continue
        if ensure_layer_payload(layer, fake_context):
            any_visible = True
    if any_visible:
        log_runtime_event(f"visible layers found in scene '{scene.name}', ensuring draw handler")
        update_draw_handler_for_scene(scene)
    else:
        log_runtime_event(f"no visible layers in scene '{scene.name}'")
    props = getattr(scene, "pcv_props", None)
    if props and getattr(props, "crop_enable", False):
        ensure_sectionbox_ready(fake_context)
    if props and getattr(props, "screen_edit_enable", False):
        active_layer = get_active_layer_from_scene(scene)
        if not active_layer:
            props.screen_edit_enable = False
            log_screen_event("auto_disable", reason="no_layer")
            stop_screen_selection_operator(getattr(bpy, "context", None), reason="no_layer")
        else:
            real_ctx = getattr(bpy, "context", None)
            if real_ctx:
                ensure_workspace_screen_tool(real_ctx, props.screen_tool)
    request_redraw()


def ensure_layer_payload(layer, context=None):
    if not layer:
        return False
    data = get_layer_data(layer)
    if data and data.get("source_pos") is not None:
        return True
    log_runtime_event(f"ensure_layer_payload start layer='{layer.name}' filepath='{layer.filepath}'")
    context = context or bpy.context
    scene = getattr(context, "scene", None) if context else None
    props = getattr(scene, "pcv_props", None) if scene else None
    active_obj = None
    if context:
        active_obj = context.active_object if context.active_object and context.active_object.type == 'MESH' else None
    source = (layer.filepath or "").strip()
    if not source and active_obj:
        if capture_object_to_layer(active_obj, layer, props, context):
            log_runtime_event(f"capture from active object '{active_obj.name}' for layer '{layer.name}'")
            return True
        source = (layer.filepath or "").strip()
    if scene and source and source in scene.objects:
        obj = scene.objects.get(source)
        if capture_object_to_layer(obj, layer, props, context):
            log_runtime_event(f"capture from scene object '{obj.name}' for layer '{layer.name}'")
            return True
    if source and Path(source).exists():
        ext = Path(source).suffix.lower()
        try:
            if ext == '.ply':
                reader = PlyPointCloudReader(source)
                if not reader.has_vertices:
                    return False
                pts = np.column_stack((reader.points['x'], reader.points['y'], reader.points['z']))
                colors = None
                if reader.has_colors:
                    colors = np.column_stack((reader.points['red'], reader.points['green'], reader.points['blue'])) / 255.0
                    colors = np.hstack((colors, np.ones((colors.shape[0], 1), dtype=np.float32)))
            else:
                return False
        except Exception:
            return False
        ok = update_point_cloud_data(pts, colors, None, props, layer)
        if ok:
            log_runtime_event(f"capture from file '{source}' for layer '{layer.name}'")
            ensure_layer_transform_driver(layer, context=context)
            refresh_layer_visibility(layer, context)
        return ok
    log_runtime_event(f"ensure_layer_payload failed for layer '{layer.name}'")
    return False


LOG_PATH = (Path(__file__).resolve().parent.parent / "dev" / "Terminal.md")

def reset_terminal_log():
    """
    Truncate dev/Terminal.md on addon register so each Blender session
    starts with a fresh diagnostic log.
    """
    if not ENABLE_VERBOSE_LOG:
        return
    try:
        log_path = LOG_PATH
        log_path.parent.mkdir(parents=True, exist_ok=True)
        with open(log_path, 'w', encoding='utf-8') as log_file:
            log_file.write(f"[{datetime.now().strftime('%Y-%m-%d %H:%M:%S')}] RUNTIME | log_reset\n")
    except Exception:
        pass

VERT_SRC = """
void main()
{
  f_color = color;
  vec4 world = modelMatrix * vec4(position, 1.0f);
  f_pos = world.xyz;
  gl_Position = mvpMatrix * vec4(position, 1.0f);
  gl_PointSize = params.x; 
}
"""

FRAG_SRC = """
void main()
{
  // params: x=size, y=useCrop, z=cropType, w=cropMode
  bool useCrop = (params.y > 0.5);
  float mode = params.w;
  float alphaMul = clamp(cropDataZ.x, 0.0, 1.0);
  
  if (useCrop) {
      vec3 center = cropDataX.xyz;
      vec3 invScale = cropDataY.xyz;
      bool inside = false;

      if (params.z < 1.5) {
          vec3 local = (f_pos - center) * invScale;
          inside = all(lessThanEqual(abs(local), vec3(1.0)));
      } else {
          vec2 localXY = (f_pos.xy - center.xy) * invScale.xy;
          float localZ = (f_pos.z - center.z) * invScale.z;
          inside = ((dot(localXY, localXY) <= 1.0) && (abs(localZ) <= 1.0));
      }
      
      if ((mode >= 0.0 && !inside) || (mode < 0.0 && inside)) {
          discard;
      }
  }

  if (cropDataZ.w > 0.5) {
      vec2 coord = gl_PointCoord * 2.0 - 1.0;
      if (dot(coord, coord) > 1.0) {
          discard;
      }
  }
  FragColor = vec4(f_color.rgb, f_color.a * alphaMul);
}
"""

def create_shader():
    vert_out = gpu.types.GPUStageInterfaceInfo("pcv_iface")
    vert_out.smooth('VEC4', "f_color")
    vert_out.smooth('VEC3', "f_pos")

    info = gpu.types.GPUShaderCreateInfo()
    info.push_constant('MAT4', "mvpMatrix") 
    info.push_constant('MAT4', "modelMatrix")
    info.push_constant('VEC4', "params")                                  
    info.push_constant('VEC4', "cropDataX")                        
    info.push_constant('VEC4', "cropDataY")                          
    info.push_constant('VEC4', "cropDataZ")                            
    
    info.vertex_in(0, 'VEC3', "position")
    info.vertex_in(1, 'VEC4', "color")
    info.vertex_out(vert_out)
    info.fragment_out(0, 'VEC4', "FragColor")
    
    info.vertex_source(VERT_SRC)
    info.fragment_source(FRAG_SRC)

    return gpu.shader.create_from_info(info)

def get_shader_point():
    global SHADER_POINT
    if SHADER_POINT is None:
        try: SHADER_POINT = create_shader()
        except Exception as e: print(f"Shader Error: {e}")
    return SHADER_POINT

def rebuild_point_batch(layer, props=None):
    data = get_layer_data(layer)
    if not data or data["source_pos"] is None:
        return False
    if props is None:
        scene = getattr(bpy.context, "scene", None)
        props = getattr(scene, "pcv_props", None) if scene else None
    density = layer.point_density if layer else getattr(props, "point_density", 1.0)
    density = max(0.01, min(1.0, density))
    src_pos = data["source_pos"]
    src_col = data["source_col"]
    total = len(src_pos)
    if density >= 0.999 or total <= 1:
        pos = src_pos
        col = src_col
        idx = np.arange(total, dtype=np.int32)
    else:
        target = max(1, min(total, int(math.ceil(total * density))))
        if target >= total:
            pos = src_pos
            col = src_col
            idx = np.arange(total, dtype=np.int32)
        else:
            base_uid = getattr(layer, "uid", "") or ensure_layer_uid(layer)
            seed = int(hashlib.sha256(base_uid.encode('utf-8')).hexdigest()[:16], 16) & 0xffffffff
            rng = np.random.RandomState(seed)
            idx = rng.choice(total, size=target, replace=False)
            idx.sort()
            pos = src_pos[idx]
            col = src_col[idx] if src_col is not None else None
    pos = np.asarray(pos, dtype=np.float32)
    if col is not None:
        col = np.asarray(col, dtype=np.float32)
        if col.ndim == 1:
            col = np.broadcast_to(col, (len(pos), 4))
    shader = get_shader_point()
    if not shader:
        return False
    try:
        color_data = col if col is not None else np.ones((len(pos), 4), dtype=np.float32)
        selection_mask = data.get("selection_mask")
        selection_color = np.asarray(data.get("selection_color", DEFAULT_SELECTION_RGBA), dtype=np.float32)
        if selection_mask is not None and len(selection_mask) == len(src_pos):
            sel_vis = selection_mask[idx]
            if np.any(sel_vis):
                color_data = np.array(color_data, copy=True)
                blend = selection_color[:3]
                alpha = selection_color[3] if selection_color.shape[0] > 3 else 1.0
                alpha = np.clip(alpha, 0.0, 2.0)
                mix_factor = min(alpha * 0.5, 1.0)
                color_data[sel_vis, :3] = (color_data[sel_vis, :3] * (1.0 - mix_factor)) + (blend * mix_factor)
                color_data[sel_vis, 3] = np.clip(color_data[sel_vis, 3] + min(alpha, 2.0) * 0.2, 0.0, 1.0)
        batch = batch_for_shader(shader, 'POINTS', {"position": pos, "color": color_data})
        data["positions"] = pos
        data["colors"] = color_data
        data["visible_indices"] = idx
        data["batch"] = batch
        data["point_count"] = len(pos)
        data["dirty"] = False
        global POINT_COUNT
        POINT_COUNT = len(pos)
        ensure_pointcloud_draw_handler()
        return True
    except Exception:
        return False


def encode_ids_to_colors(count):
    ids = np.arange(1, count + 1, dtype=np.int32)
    colors = np.zeros((count, 4), dtype=np.float32)
    colors[:, 0] = (ids & 0xFF) / 255.0
    colors[:, 1] = ((ids >> 8) & 0xFF) / 255.0
    colors[:, 2] = ((ids >> 16) & 0xFF) / 255.0
    colors[:, 3] = 1.0
    return colors


def _project_points_to_screen(region, rv3d, positions):
    if region is None or rv3d is None:
        return None, None
    if positions is None or len(positions) == 0:
        return None, None
    pos = np.asarray(positions, dtype=np.float32)
    if pos.ndim != 2 or pos.shape[1] != 3:
        return None, None
    ones = np.ones((pos.shape[0], 1), dtype=np.float32)
    hom = np.hstack((pos, ones))
    mvp = np.array(rv3d.perspective_matrix, dtype=np.float32)
    clip = hom @ mvp.T
    w = clip[:, 3]
    valid = np.abs(w) > 1e-8
    if not np.any(valid):
        return None, None
    ndc = np.zeros((pos.shape[0], 3), dtype=np.float32)
    ndc[valid] = clip[valid, :3] / w[valid, None]
    screen = np.zeros((pos.shape[0], 2), dtype=np.float32)
    screen[:, 0] = (ndc[:, 0] * 0.5 + 0.5) * float(region.width)
    screen[:, 1] = (ndc[:, 1] * 0.5 + 0.5) * float(region.height)
    return screen, valid


def _points_in_polygon(points, polygon):
    poly = np.asarray(polygon, dtype=np.float32)
    if poly.ndim != 2 or poly.shape[0] < 3:
        return np.zeros(len(points), dtype=bool)
    pts = np.asarray(points, dtype=np.float32)
    if pts.ndim != 2 or pts.shape[1] != 2:
        return np.zeros(len(points), dtype=bool)
    x = pts[:, 0]
    y = pts[:, 1]
    inside = np.zeros(len(points), dtype=bool)
    xj, yj = poly[-1]
    for xi, yi in poly:
        denom = (yj - yi) if abs(yj - yi) > 1e-12 else 1e-12
        cond = ((yi > y) != (yj > y)) & (x < (xj - xi) * (y - yi) / denom + xi)
        inside ^= cond
        xj, yj = xi, yi
    return inside


def _screen_selection_mask(screen, valid, region, start, end, shape='BOX', circle=None, lasso=None):
    if screen is None or valid is None or not np.any(valid):
        return np.zeros(len(screen) if screen is not None else 0, dtype=bool)
    width = float(region.width)
    height = float(region.height)
    x1, y1 = start
    x2, y2 = end
    min_x = max(0.0, min(x1, x2))
    max_x = min(width, max(x1, x2))
    min_y = max(0.0, min(y1, y2))
    max_y = min(height, max(y1, y2))
    if shape == 'CIRCLE' and circle:
        center, radius = circle
        if center is None or radius <= 0.0:
            return np.zeros(len(screen), dtype=bool)
        dx = screen[:, 0] - float(center[0])
        dy = screen[:, 1] - float(center[1])
        dist2 = dx * dx + dy * dy
        return (dist2 <= float(radius) ** 2) & valid
    if shape == 'LASSO' and lasso:
        poly = np.asarray(lasso, dtype=np.float32)
        if poly.ndim != 2 or poly.shape[0] < 3:
            return np.zeros(len(screen), dtype=bool)
                                                                        
                                                                    
        pxmin = max(0.0, float(np.min(poly[:, 0])))
        pxmax = min(width, float(np.max(poly[:, 0])))
        pymin = max(0.0, float(np.min(poly[:, 1])))
        pymax = min(height, float(np.max(poly[:, 1])))
        bbox_mask = (
            (screen[:, 0] >= pxmin) & (screen[:, 0] <= pxmax) &
            (screen[:, 1] >= pymin) & (screen[:, 1] <= pymax) &
            valid
        )
        if not np.any(bbox_mask):
            return np.zeros(len(screen), dtype=bool)
        pts = screen[bbox_mask]
        inside_local = _points_in_polygon(pts, poly)
        mask = np.zeros(len(screen), dtype=bool)
        mask[bbox_mask] = inside_local
        return mask
    xmin = min(min_x, max_x)
    xmax = max(min_x, max_x)
    ymin = min(min_y, max_y)
    ymax = max(min_y, max_y)
    return (
        (screen[:, 0] >= xmin) &
        (screen[:, 0] <= xmax) &
        (screen[:, 1] >= ymin) &
        (screen[:, 1] <= ymax) &
        valid
    )


def gpu_select_points(context, layer, start, end, use_depth=True, shape='BOX', circle=None, lasso=None):
    region = context.region
    rv3d = context.region_data
    if not region or not rv3d:
        return None
    data = get_layer_data(layer)
    if not data or data.get("source_pos") is None:
        return None
    positions = data.get("positions")
    visible_indices = data.get("visible_indices")
    if positions is None or visible_indices is None:
        return None
    matrix = get_layer_transform_matrix(layer)
    positions_world = transform_points_array(positions, matrix)
    screen, valid = _project_points_to_screen(region, rv3d, positions_world)
    if screen is None or valid is None:
        return None
    display_mask = _screen_selection_mask(screen, valid, region, start, end, shape=shape, circle=circle, lasso=lasso)
    if not np.any(display_mask):
        return None
    mapped_vis = np.asarray(visible_indices, dtype=np.int32)
    selected_vis = np.nonzero(display_mask)[0]
    src_indices = mapped_vis[selected_vis]
    if src_indices.size == 0:
        return None
    src_len = len(data["source_pos"])
    src_indices = src_indices[(src_indices >= 0) & (src_indices < src_len)]
    if src_indices.size == 0:
        return None
    mask = np.zeros(src_len, dtype=bool)
    mask[src_indices] = True
    if len(visible_indices) < src_len:
        src_world = transform_points_array(data["source_pos"], matrix)
        full_screen, full_valid = _project_points_to_screen(region, rv3d, src_world)
        full_mask = _screen_selection_mask(full_screen, full_valid, region, start, end, shape=shape, circle=circle, lasso=lasso)
        if full_mask is not None and np.any(full_mask):
            mask |= full_mask
    return mask


def get_screen_target_layers(context, primary_layer):
    """
    Return the list of layers that should participate in Screen Selection.

    - When one or more mesh objects are selected, we affect all layers whose
      source objects are in the current selection. This allows Screen tools to
      work across split clouds (multiple meshes) in one gesture.
    - When no suitable objects are selected, we fall back to the active layer
      to preserve the previous single-layer behaviour.
    """
    if not context or not getattr(context, "scene", None):
        return [primary_layer] if primary_layer else []
    scene = context.scene
    view_layer = getattr(context, "view_layer", None)
    selected = []
    if view_layer and getattr(view_layer, "objects", None):
        try:
            selected = [obj for obj in view_layer.objects.selected if obj.type == 'MESH']
        except Exception:
            selected = []
    if not selected:
        return [primary_layer] if primary_layer else []
    selected_names = {obj.name for obj in selected}
    layers = []
    for layer in getattr(scene, "pcv_layers", []):
        src_name = (getattr(layer, "filepath", "") or "").strip()
        if src_name and src_name in selected_names:
            layers.append(layer)
    if not layers and primary_layer:
        layers = [primary_layer]
                                                   
    seen = set()
    unique = []
    for layer in layers:
        uid = ensure_layer_uid(layer)
        if uid in seen:
            continue
        seen.add(uid)
        unique.append(layer)
    return unique


def apply_screen_selection(context, layer, start, end, mode, shape='BOX', circle=None, lasso=None):
    data = get_layer_data(layer)
    if not data or data.get("source_pos") is None:
        return False
    length = len(data["source_pos"])
    mask_hits = gpu_select_points(context, layer, start, end, shape=shape, circle=circle, lasso=lasso)
    if mask_hits is None:
                                                                           
        if mode == 'REPLACE':
            new_mask = np.zeros(length, dtype=bool)
            set_layer_selection(layer, new_mask)
            return True
        return False
    existing = data.get("selection_mask")
    if existing is None or len(existing) != length:
        existing = np.zeros(length, dtype=bool)
    if mode == 'ADD':
        new_mask = np.logical_or(existing, mask_hits)
    elif mode == 'SUB':
        new_mask = np.logical_and(existing, np.logical_not(mask_hits))
    elif mode == 'XOR':
        new_mask = np.logical_xor(existing, mask_hits)
    else:
        new_mask = mask_hits
    set_layer_selection(layer, new_mask)
    return True


def update_point_cloud_data(pos, col, nrm, props=None, layer=None, keep_driver_state=False):
    global _SOURCE_POS, _SOURCE_COL, POINT_COUNT
    if layer is None:
        context = bpy.context
        layer = get_active_layer(context)
        if layer is None:
            return False
    ensure_layer_uid(layer)
    layer_data = get_layer_data(layer)
    pos = np.asarray(pos, dtype=np.float32)
    if pos.ndim != 2 or pos.shape[1] != 3:
        return False
    if col is None:
        col = np.ones((len(pos), 4), dtype=np.float32)
    col = np.asarray(col, dtype=np.float32)
    if col.ndim == 1:
        col = np.broadcast_to(col, (len(pos), 4))
    if col.shape[1] == 3:
        col = np.column_stack((col, np.ones(len(pos))))
    if np.any(col > 1.0):
        col = col / 255.0
    col = col.astype(np.float32)
    layer_data["source_pos"] = pos
    layer_data["source_col"] = col
    layer_data["selection_mask"] = None
    layer_data["selection_count"] = 0
    _SOURCE_POS = pos
    _SOURCE_COL = col
    layer.state = 'READY'
                                                                            
                                                                          
                                                                            
    if not keep_driver_state:
        ensure_layer_transform_driver(layer)
        refresh_layer_visibility(layer)
        mark_sectionbox_needs_fit()
    POINT_COUNT = len(pos)
    built = rebuild_point_batch(layer, props)
    if built:
        set_globals_from_layer(layer)
        request_redraw()
    return built


def capture_object_to_layer(obj, layer, props=None, context=None):
    if obj is None or layer is None or obj.type != 'MESH':
        return False
    pts, col = extract_data(obj.data, props)
    if pts is None:
        return False
    if update_point_cloud_data(pts, col, None, props, layer):
        layer.filepath = obj.name
        if not layer.name or layer.name.startswith("Cloud"):
            layer.name = obj.name
        ensure_layer_transform_driver(layer, context=context, source_obj=obj)
        refresh_layer_visibility(layer, context)
        return True
    return False

def draw_callback():
    if SHADER_POINT is None:
        return
    context = bpy.context
    scene = getattr(context, "scene", None)
    region = getattr(context, "region_data", None)
    if not scene or not region:
        return
    props = getattr(scene, "pcv_props", None)
    if props is None:
        return
    active_layer = get_active_layer_from_scene(scene)
    view_proj = region.perspective_matrix
    zero = Vector((0.0, 0.0, 0.0, 0.0))
    gpu.state.depth_test_set('LESS_EQUAL')
    try:
        for layer in scene.pcv_layers:
            if not layer.show_points:
                continue
            source_obj = get_layer_source_object(layer, scene)
                                                                        
                                                                       
                                                                   
            if source_obj and getattr(layer, "show_source_mesh", False):
                if not is_object_truly_visible(source_obj, context):
                    continue
                                                                         
                                                                    
                                                        
            elif source_obj and is_any_collection_hidden(source_obj, context):
                continue
                                                                     
                                                                       
                                                                        
                                                                           
                                       
            if not layer_visible_in_view(layer, context):
                if not (source_obj and getattr(layer, "show_source_mesh", False) and is_object_truly_visible(source_obj, context)):
                    continue
            data = get_layer_data(layer)
            if not data or data.get("source_pos") is None:
                if not ensure_layer_payload(layer, context):
                    continue
                data = get_layer_data(layer)
            if data.get("dirty"):
                if not rebuild_point_batch(layer, props):
                    continue
                data = get_layer_data(layer)
            batch = data.get("batch")
            if batch is None:
                continue
            if layer == active_layer:
                use_crop, d_x, d_y, _, p_type = get_crop_uniforms(props, context)
                mode_factor = 1.0 if props.crop_mode == 'INSIDE' else -1.0
            else:
                use_crop, d_x, d_y, _, p_type = False, zero, zero, zero, 1.0
                mode_factor = 1.0
            params = Vector((layer.point_size, 1.0 if use_crop else 0.0, p_type, mode_factor))
            shape_flag = 1.0 if props.point_shape == 'ROUND' else 0.0
            alpha_info = Vector((layer.point_alpha, 0.0, 0.0, shape_flag))
            gpu.state.blend_set('ALPHA' if layer.point_alpha < 0.999 else 'NONE')
            gpu.state.depth_test_set('LESS_EQUAL')

            model = get_layer_transform_matrix(layer)
            mvp = view_proj @ model
            try:
                SHADER_POINT.bind()
                SHADER_POINT.uniform_float("mvpMatrix", mvp)
                SHADER_POINT.uniform_float("modelMatrix", model)
                SHADER_POINT.uniform_float("params", params)
                SHADER_POINT.uniform_float("cropDataX", d_x)
                SHADER_POINT.uniform_float("cropDataY", d_y)
                SHADER_POINT.uniform_float("cropDataZ", alpha_info)
                batch.draw(SHADER_POINT)
            except Exception:
                continue
    finally:
        gpu.state.depth_test_set('NONE')
        gpu.state.blend_set('NONE')

                                                                           
                               
                                                                           

MIN_DIMENSION = 0.001
MIN_HALF = MIN_DIMENSION * 0.5
DOT_OCCLUDED_ALPHA = 0.35
HANDLE_OFFSET = 0.01
HANDLE_HIT_SCALE = 2.2
HANDLE_VISUAL_MIN = 0.05
HANDLE_SCREEN_MIN = 0.05

_SECTIONBOX_DRAW_HANDLE = None
_SECTIONBOX_SHADER = None

def ensure_sectionbox_shader():
    global _SECTIONBOX_SHADER
    if _SECTIONBOX_SHADER is None:
        _SECTIONBOX_SHADER = gpu.shader.from_builtin("UNIFORM_COLOR")

def ensure_sectionbox_draw_handler():
    global _SECTIONBOX_DRAW_HANDLE
    if _SECTIONBOX_DRAW_HANDLE is None:
        _SECTIONBOX_DRAW_HANDLE = bpy.types.SpaceView3D.draw_handler_add(
            draw_sectionbox, (), 'WINDOW', 'POST_VIEW'
        )

def remove_sectionbox_draw_handler():
    global _SECTIONBOX_DRAW_HANDLE
    if _SECTIONBOX_DRAW_HANDLE is not None:
        bpy.types.SpaceView3D.draw_handler_remove(_SECTIONBOX_DRAW_HANDLE, 'WINDOW')
        _SECTIONBOX_DRAW_HANDLE = None

def update_gizmo_view(self, context):
    """Force redraw so gizmos follow property changes immediately."""
    if context is None:
        context = bpy.context
    if not context:
        return
    screens = []
    if context.screen:
        screens.append(context.screen)
    else:
        wm = context.window_manager if hasattr(context, "window_manager") else bpy.context.window_manager
        if wm:
            for win in wm.windows:
                if win.screen:
                    screens.append(win.screen)
    for screen in screens:
        for area in screen.areas:
            if area.type == 'VIEW_3D':
                area.tag_redraw()


class SectionBoxSettings(bpy.types.PropertyGroup):
    show: bpy.props.BoolProperty(
        name="Show Cropbox",
        default=False,
        update=update_gizmo_view,
    )
    shape: bpy.props.EnumProperty(
        name="Shape",
        items=[
            ("NONE", "None", ""),
            ("BOX", "Box", "Axis-aligned box"),
            ("CYLINDER", "Cylinder", "Vertical cylinder"),
        ],
        default="NONE",
        update=update_gizmo_view,
    )
    center: bpy.props.FloatVectorProperty(
        name="Center",
        subtype='XYZ',
        default=(0.0, 0.0, 0.0),
        update=update_gizmo_view,
    )
    size: bpy.props.FloatVectorProperty(
        name="Box Size",
        subtype='XYZ',
        default=(1.0, 1.0, 1.0),
        min=MIN_DIMENSION,
        update=update_gizmo_view,
    )
    radius: bpy.props.FloatProperty(
        name="Radius",
        default=0.5,
        min=MIN_DIMENSION,
        update=update_gizmo_view,
    )
    height: bpy.props.FloatProperty(
        name="Height",
        default=1.0,
        min=MIN_DIMENSION,
        update=update_gizmo_view,
    )
    color: bpy.props.FloatVectorProperty(
        name="Fill Color",
        subtype='COLOR',
        size=3,
        min=0.0,
        max=1.0,
        default=(0.05, 0.35, 1.0),
    )
    edge_color: bpy.props.FloatVectorProperty(
        name="Edge Color",
        subtype='COLOR',
        size=3,
        min=0.0,
        max=1.0,
        default=(0.05, 0.65, 1.0),
    )
    dot_color: bpy.props.FloatVectorProperty(
        name="Dot Color",
        subtype='COLOR',
        size=3,
        min=0.0,
        max=1.0,
        default=(0.98, 0.93, 0.55),
    )
    display_style: bpy.props.EnumProperty(
        name="Display",
        items=[("SOLID", "Solid", ""), ("WIRE", "Wire", "")],
        default="SOLID",
        update=update_gizmo_view,
    )
    hide_container: bpy.props.BoolProperty(
        name="Hide Container",
        description="Hide the cropbox mesh and gizmos",
        default=False,
        update=update_gizmo_view,
    )
    alpha_solid: bpy.props.FloatProperty(
        name="Fill Opacity",
        default=0.3,
        min=0.0,
        max=1.0,
        update=update_gizmo_view,
    )
    alpha_wire: bpy.props.FloatProperty(
        name="Wire Opacity",
        default=1.0,
        min=0.0,
        max=1.0,
        update=update_gizmo_view,
    )
    gizmo_scale_scale: bpy.props.FloatProperty(
        name="Dot Size",
        default=0.005,
        min=0.005,
        max=1.0,
        update=update_gizmo_view,
    )
    gizmo_scale_move: bpy.props.FloatProperty(
        name="Move Gizmo Size",
        default=0.5,
        min=0.05,
        max=1.5,
        update=update_gizmo_view,
    )
    symmetric_box: bpy.props.BoolProperty(
        name="Box Symmetry",
        default=False,
        update=update_gizmo_view,
    )
    symmetric_cylinder_height: bpy.props.BoolProperty(
        name="Cylinder Height Symmetry",
        default=False,
        update=update_gizmo_view,
    )
    box_cached_size: bpy.props.FloatVectorProperty(
        name="Box Cache",
        subtype='XYZ',
        size=3,
        default=(0.0, 0.0, 0.0),
    )
    box_reference_size: bpy.props.FloatVectorProperty(
        name="Box Reference",
        subtype='XYZ',
        size=3,
        default=(0.0, 0.0, 0.0),
    )
    cyl_cached_radius: bpy.props.FloatProperty(
        name="Cylinder Radius Cache",
        default=0.0,
    )
    cyl_cached_height: bpy.props.FloatProperty(
        name="Cylinder Height Cache",
        default=0.0,
    )
    cyl_reference_radius: bpy.props.FloatProperty(
        name="Cylinder Reference Radius",
        default=0.0,
    )
    auto_fit_done: bpy.props.BoolProperty(
        name="Auto Fit Performed",
        default=False,
        options={'HIDDEN'},
    )


def cache_box_size(settings):
    sz = tuple(settings.size)
    settings.box_cached_size = sz
    settings.box_reference_size = sz
    height = max(sz[2], MIN_DIMENSION)
    settings.height = height
    half_x = max(sz[0] * 0.5, MIN_DIMENSION)
    half_y = max(sz[1] * 0.5, MIN_DIMENSION)
    circ = math.hypot(half_x, half_y)
    settings.cyl_cached_radius = circ
    settings.cyl_reference_radius = circ
    settings.cyl_cached_height = height


def cache_cylinder_size(settings, set_reference=False):
    radius = max(settings.radius, MIN_DIMENSION)
    height = max(settings.height, MIN_DIMENSION)
    settings.cyl_cached_radius = radius
    settings.cyl_cached_height = height
    if set_reference or settings.cyl_reference_radius <= MIN_DIMENSION:
        settings.cyl_reference_radius = radius


def update_box_cache_from_cylinder(settings):
    base = Vector(getattr(settings, "box_reference_size", (0.0, 0.0, 0.0)))
    base_radius = max(getattr(settings, "cyl_reference_radius", 0.0), MIN_DIMENSION)
    if base.length_squared == 0.0:
        return
    scale = settings.radius / base_radius
    scaled = Vector((base.x * scale, base.y * scale, settings.height))
    settings.box_cached_size = (scaled.x, scaled.y, scaled.z)


def layer_visual_update(self, context):
    mark_layer_dirty(self)
    request_redraw()


def layer_show_update(self, context):
    ensure_layer_uid(self)
    scene = getattr(self, "id_data", None)
    if scene is None:
        scene = getattr(context, "scene", None) if context else None
    if scene is None:
        ctx = bpy.context
        scene = getattr(ctx, "scene", None) if ctx else None
    current_show = bool(self.show_points)
    uid = ensure_layer_uid(self)
    prev = _LAYER_SHOW_STATE.get(uid)
    _LAYER_SHOW_STATE[uid] = current_show
    active_layer = get_active_layer_from_scene(scene) if scene else None
    if prev != current_show and (active_layer is None or active_layer == self):
        log_runtime_event(
            f"layer_show_update layer='{getattr(self, 'name', None)}' show_points={int(current_show)}"
        )
    if self.show_points:
        ready = ensure_layer_payload(self, context)
        if not ready:
            self.show_points = False
            return
        ensure_layer_transform_driver(self, context=context)
    update_draw_handler_for_scene(scene)
    request_redraw()
    refresh_layer_visibility(self, context)
    if scene:
        active = get_active_layer_from_scene(scene)
        if active == self:
            set_globals_from_layer(self)


def layer_selection_color_update(self, context):
    layer = self
    data = get_layer_data(layer)
    if not data:
        return
    color = tuple(getattr(layer, "selection_color", DEFAULT_SELECTION_COLOR))
    strength = float(getattr(layer, "selection_color_strength", DEFAULT_SELECTION_BLEND))
    strength = max(0.0, min(2.0, strength))
    data["selection_color"] = (color[0], color[1], color[2], strength)
    mark_layer_dirty(layer)
    request_redraw()


def log_shape_state(event, settings, extra=""):
    if not ENABLE_VERBOSE_LOG:
        return
    try:
        log_path = LOG_PATH
        log_path.parent.mkdir(parents=True, exist_ok=True)
        fmt = lambda vec: tuple(round(float(v), 5) for v in vec)
        msg = (
            f"[{datetime.now().strftime('%Y-%m-%d %H:%M:%S')}] {event} | "
            f"shape={getattr(settings, 'shape', '?')} "
            f"center={fmt(getattr(settings, 'center', (0, 0, 0)))} "
            f"size={fmt(getattr(settings, 'size', (0, 0, 0)))} "
            f"radius={getattr(settings, 'radius', 0.0):.5f} "
            f"height={getattr(settings, 'height', 0.0):.5f} "
            f"box_cache={fmt(getattr(settings, 'box_cached_size', (0, 0, 0)))} "
            f"cyl_cache=(r={getattr(settings, 'cyl_cached_radius', 0.0):.5f},h={getattr(settings, 'cyl_cached_height', 0.0):.5f})"
        )
        if extra:
            msg += f" | {extra}"
        with open(log_path, 'a', encoding='utf-8') as log_file:
            log_file.write(msg + "\n")
    except Exception:
        pass


def log_runtime_event(message):
    if not ENABLE_VERBOSE_LOG:
        return
    try:
        log_path = LOG_PATH
        log_path.parent.mkdir(parents=True, exist_ok=True)
        with open(log_path, 'a', encoding='utf-8') as log_file:
            log_file.write(f"[{datetime.now().strftime('%Y-%m-%d %H:%M:%S')}] RUNTIME | {message}\n")
    except Exception:
        pass


def _format_matrix_short(mat):
    """
    Compact matrix description used for diagnostics (location, rotation, scale).
    Intended only for Cut/Split and driver debugging.
    """
    try:
        loc, rot, scale = mat.decompose()
        e = rot.to_euler('XYZ')
        return (
            f"loc=({loc.x:.3f},{loc.y:.3f},{loc.z:.3f}) "
            f"rot=({e.x:.3f},{e.y:.3f},{e.z:.3f}) "
            f"scl=({scale.x:.3f},{scale.y:.3f},{scale.z:.3f})"
        )
    except Exception:
        return "matrix=<unavailable>"


def _format_rotation_delta(mat_before, mat_after):
    """
    Compute a simple XYZ Euler rotation delta (after - before) in radians.
    Used only for high-level diagnostics in AFTER logs.
    """
    try:
        if mat_before is None or mat_after is None:
            return "n/a"
        _, r1, _ = mat_before.decompose()
        _, r2, _ = mat_after.decompose()
        e1 = r1.to_euler('XYZ')
        e2 = r2.to_euler('XYZ')
        dx = e2.x - e1.x
        dy = e2.y - e1.y
        dz = e2.z - e1.z
        return f"({dx:.3f},{dy:.3f},{dz:.3f})"
    except Exception:
        return "unavailable"


def log_points_snapshot(label, points, matrix=None, max_points=128):
    """
    Diagnostic dump of point positions for small point clouds.
    Logs local coordinates and, if matrix is provided, corresponding world coordinates.
    """
    try:
        if points is None:
            return
        pts = np.asarray(points, dtype=np.float32).reshape(-1, 3)
        count = len(pts)
        if count == 0 or count > max_points:
            return
        world_mat = None
        if matrix is not None:
            try:
                world_mat = np.array(matrix, dtype=np.float32)
                if world_mat.shape != (4, 4):
                    world_mat = None
            except Exception:
                world_mat = None
        log_path = LOG_PATH
        log_path.parent.mkdir(parents=True, exist_ok=True)
        with open(log_path, 'a', encoding='utf-8') as log_file:
            header = f"[{datetime.now().strftime('%Y-%m-%d %H:%M:%S')}] POINTS | {label} count={count}"
            log_file.write(header + "\n")
            for idx, p in enumerate(pts):
                if world_mat is not None:
                    pw = np.array([p[0], p[1], p[2], 1.0], dtype=np.float32) @ world_mat.T
                    wx, wy, wz = pw[:3]
                    log_file.write(
                        f"  idx={idx:03d} local=({p[0]:.6f},{p[1]:.6f},{p[2]:.6f}) "
                        f"world=({wx:.6f},{wy:.6f},{wz:.6f})\n"
                    )
                else:
                    log_file.write(
                        f"  idx={idx:03d} local=({p[0]:.6f},{p[1]:.6f},{p[2]:.6f})\n"
                    )
    except Exception:
        pass



def log_screen_event(event, **details):
    try:
        payload = []
        for key, value in details.items():
            if value is None:
                continue
            payload.append(f"{key}={value}")
        extra = " " + " ".join(payload) if payload else ""
        log_runtime_event(f"SCREEN | {event}{extra}")
    except Exception:
        pass


def _next_screen_gesture_id():
    global _SCREEN_GESTURE_ID
    _SCREEN_GESTURE_ID += 1
    return _SCREEN_GESTURE_ID


def _log_exception_to_file(exc_type, exc_value, exc_traceback):
    if not ENABLE_VERBOSE_LOG:
        if _PREV_EXC_HOOK:
            try:
                _PREV_EXC_HOOK(exc_type, exc_value, exc_traceback)
            except Exception:
                pass
        return
    try:
        log_path = LOG_PATH
        log_path.parent.mkdir(parents=True, exist_ok=True)
        import traceback
        with open(log_path, 'a', encoding='utf-8') as log_file:
            log_file.write(f"[{datetime.now().strftime('%Y-%m-%d %H:%M:%S')}] EXCEPTION | {exc_type.__name__}: {exc_value}\n")
            traceback.print_tb(exc_traceback, file=log_file)
    except Exception:
        pass
    if _PREV_EXC_HOOK:
        try:
            _PREV_EXC_HOOK(exc_type, exc_value, exc_traceback)
        except Exception:
            pass


def install_error_logger():
    global _PREV_EXC_HOOK
    if not ENABLE_VERBOSE_LOG:
        return
    if _PREV_EXC_HOOK is None:
        _PREV_EXC_HOOK = sys.excepthook
        sys.excepthook = _log_exception_to_file
    install_stderr_tee()


class _StdErrTee:
    def __init__(self, original):
        self.original = original

    def write(self, data):
        try:
            if self.original:
                self.original.write(data)
        except Exception:
            pass
        if not ENABLE_VERBOSE_LOG:
            return
        try:
            log_path = LOG_PATH
            log_path.parent.mkdir(parents=True, exist_ok=True)
            with open(log_path, 'a', encoding='utf-8') as f:
                f.write(data)
        except Exception:
            pass

    def flush(self):
        try:
            if self.original:
                self.original.flush()
        except Exception:
            pass


def install_stderr_tee():
    global _STDERR_TEE
    if _STDERR_TEE is None:
        try:
            _STDERR_TEE = _StdErrTee(sys.stderr)
            sys.stderr = _STDERR_TEE
        except Exception:
            _STDERR_TEE = None

class PCVCloudLayer(bpy.types.PropertyGroup):
    name: bpy.props.StringProperty(name="Layer Name", default="Point Layer")
    uid: bpy.props.StringProperty(default="")
    filepath: bpy.props.StringProperty(subtype='FILE_PATH')
    show_points: bpy.props.BoolProperty(name="Show", default=False, update=layer_show_update)
    point_size: bpy.props.FloatProperty(
        name="Point Size",
        min=1.0,
        max=20.0,
        default=3.0,
        update=layer_visual_update,
    )
    point_density: bpy.props.FloatProperty(
        name="Point Density",
        min=0.01,
        max=1.0,
        default=1.0,
        subtype='FACTOR',
        update=layer_visual_update,
    )
    point_alpha: bpy.props.FloatProperty(
        name="Point Transparency",
        min=0.0,
        max=1.0,
        default=1.0,
        update=layer_visual_update,
    )
    selection_color: bpy.props.FloatVectorProperty(
        name="Selection Color",
        subtype='COLOR',
        min=0.0,
        max=1.0,
        default=DEFAULT_SELECTION_COLOR,
        update=layer_selection_color_update,
    )
    selection_color_strength: bpy.props.FloatProperty(
        name="Selection Blend",
        subtype='FACTOR',
        min=0.0,
        max=2.0,
        default=DEFAULT_SELECTION_BLEND,
        update=layer_selection_color_update,
    )
    transform_object: bpy.props.PointerProperty(
        name="Transform Object",
        type=bpy.types.Object,
    )
    rig_object: bpy.props.PointerProperty(
        name="Rig Object",
        type=bpy.types.Object,
    )
    show_source_mesh: bpy.props.BoolProperty(
        name="Show Source Mesh",
        default=False,
        update=show_source_update,
    )
    transform_offset: bpy.props.FloatVectorProperty(
        name="Transform Offset",
        size=16,
        default=IDENTITY_MATRIX_TUPLE,
    )
    state: bpy.props.EnumProperty(
        name="State",
        items=[
            ('EMPTY', "Empty", ""),
            ('READY', "Ready", ""),
            ('ERROR', "Error", ""),
        ],
        default='EMPTY',
    )


def get_sectionbox_settings(context=None):
    if context is None:
        context = bpy.context
    if not context or not context.scene:
        return None
    return getattr(context.scene, "sectionbox_settings", None)


def get_vertical_half(settings: SectionBoxSettings) -> float:
    if settings.shape == "BOX":
        return settings.size[2] * 0.5
    if settings.shape == "CYLINDER":
        return settings.height * 0.5
    return 0.0


def get_bottom_anchor_vector(settings: SectionBoxSettings) -> Vector:
    half = get_vertical_half(settings)
    cx, cy, cz = settings.center
    return Vector((cx, cy, cz - half))


def get_view_info():
    rv3d = bpy.context.region_data
    view_loc = None
    view_distance = 1.0
    view_is_persp = False
    if rv3d is not None:
        view_distance = getattr(rv3d, "view_distance", 1.0)
        view_is_persp = rv3d.view_perspective in {'PERSP', 'CAMERA'}
        try:
            view_loc = Vector(rv3d.view_matrix.inverted().translation)
        except Exception:
            view_loc = None
    return view_loc, view_distance, view_is_persp


def compute_screen_scale(world_pos: Vector, size_param: float, view_loc: Vector, view_distance: float, view_is_persp: bool) -> float:
    if size_param <= 0.0:
        return 0.0
    if view_is_persp and view_loc is not None:
        dist = max((world_pos - view_loc).length, 1e-4)
        return size_param * dist
    return size_param * max(view_distance, 1e-4)


def build_box_world_vertices(center: Vector, size: Vector):
    cx, cy, cz = center
    sx, sy, sz = size
    hx, hy, hz = sx * 0.5, sy * 0.5, sz * 0.5

    local = [
        Vector((-hx, -hy, -hz)),
        Vector((+hx, -hy, -hz)),
        Vector((+hx, +hy, -hz)),
        Vector((-hx, +hy, -hz)),
        Vector((-hx, -hy, +hz)),
        Vector((+hx, -hy, +hz)),
        Vector((+hx, +hy, +hz)),
        Vector((-hx, +hy, +hz)),
    ]
    verts = [Vector((cx, cy, cz)) + v for v in local]

    tris = [
        (0, 1, 2), (0, 2, 3),
        (4, 5, 6), (4, 6, 7),
        (0, 1, 5), (0, 5, 4),
        (3, 2, 6), (3, 6, 7),
        (0, 3, 7), (0, 7, 4),
        (1, 2, 6), (1, 6, 5),
    ]

    lines = [
        (0, 1), (1, 2), (2, 3), (3, 0),
        (4, 5), (5, 6), (6, 7), (7, 4),
        (0, 4), (1, 5), (2, 6), (3, 7),
    ]

    return verts, tris, lines


def build_cylinder_world_vertices(center: Vector, radius: float, height: float, segments: int = 32):
    cx, cy, cz = center
    h2 = height * 0.5

    verts = []
    for ring_z in (-h2, +h2):
        for i in range(segments):
            a = 2.0 * math.pi * i / segments
            x = cx + math.cos(a) * radius
            y = cy + math.sin(a) * radius
            z = cz + ring_z
            verts.append(Vector((x, y, z)))

    bottom_center_index = len(verts)
    top_center_index = len(verts) + 1
    verts.append(Vector((cx, cy, cz - h2)))
    verts.append(Vector((cx, cy, cz + h2)))

    tris = []
    lines = []

    for i in range(segments):
        ni = (i + 1) % segments
        b0 = i
        b1 = ni
        t0 = i + segments
        t1 = ni + segments

        tris.append((b0, b1, t1))
        tris.append((b0, t1, t0))
        lines.append((b0, b1))
        lines.append((t0, t1))

    for i in range(segments):
        ni = (i + 1) % segments
        tris.append((bottom_center_index, ni, i))

    for i in range(segments):
        ni = (i + 1) % segments
        tris.append((top_center_index, i + segments, ni + segments))

    return verts, tris, lines


def draw_sectionbox():
    settings = get_sectionbox_settings()
    if not settings or not settings.show or settings.shape == "NONE" or settings.hide_container:
        return

    ensure_sectionbox_shader()
    if _SECTIONBOX_SHADER is None:
        return

    center = Vector(settings.center)

    if settings.shape == "BOX":
        size = Vector(settings.size)
        verts, tris, lines = build_box_world_vertices(center, size)
    else:
        verts, tris, lines = build_cylinder_world_vertices(center, settings.radius, settings.height)

    r, g, b = settings.color
    er, eg, eb = settings.edge_color
    a_fill = settings.alpha_solid
    a_wire = settings.alpha_wire

    gpu.state.blend_set("ALPHA")
    gpu.state.depth_test_set("LESS_EQUAL")

    draw_fill = (settings.display_style == "SOLID")

    if settings.shape == "CYLINDER":
        gpu.state.face_culling_set("BACK")
        depth_mask_original = False
    else:
        gpu.state.face_culling_set("NONE")
        depth_mask_original = True
    gpu.state.depth_mask_set(depth_mask_original)

    _SECTIONBOX_SHADER.bind()
    if draw_fill:
        gpu.state.depth_mask_set(False)
        batch_solid = batch_for_shader(_SECTIONBOX_SHADER, "TRIS", {"pos": verts}, indices=tris)
        _SECTIONBOX_SHADER.uniform_float("color", (r, g, b, a_fill))
        batch_solid.draw(_SECTIONBOX_SHADER)
        gpu.state.depth_mask_set(depth_mask_original)
    elif settings.shape == "CYLINDER":
        gpu.state.face_culling_set("NONE")
        gpu.state.depth_mask_set(True)

    batch_wire = batch_for_shader(_SECTIONBOX_SHADER, "LINES", {"pos": verts}, indices=lines)
    _SECTIONBOX_SHADER.uniform_float("color", (er, eg, eb, a_wire))
    batch_wire.draw(_SECTIONBOX_SHADER)

    dot_handles = []
    dot_size_param = max(0.0, settings.gizmo_scale_scale)
    dr, dg, db = settings.dot_color
    view_loc, view_distance, view_is_persp = get_view_info()

    if settings.shape == "BOX":
        sx, sy, sz = settings.size
        hx, hy, hz = sx * 0.5, sy * 0.5, sz * 0.5
        positions = (
            center + Vector((+hx, 0.0, 0.0)),
            center + Vector((-hx, 0.0, 0.0)),
            center + Vector((0.0, +hy, 0.0)),
            center + Vector((0.0, -hy, 0.0)),
            center + Vector((0.0, 0.0, +hz)),
            center + Vector((0.0, 0.0, -hz)),
        )
        for pos in positions:
            radius = compute_screen_scale(pos, dot_size_param, view_loc, view_distance, view_is_persp)
            if radius > 0.0:
                dot_handles.append((pos, radius))
    elif settings.shape == "CYLINDER":
        r_ = settings.radius
        h_ = settings.height * 0.5
        positions = (
            center + Vector((r_, 0.0, 0.0)),
            center + Vector((0.0, 0.0, +h_)),
            center + Vector((0.0, 0.0, -h_)),
        )
        for pos in positions:
            radius = compute_screen_scale(pos, dot_size_param, view_loc, view_distance, view_is_persp)
            if radius > 0.0:
                dot_handles.append((pos, radius))

    if dot_handles:
        gpu.state.face_culling_set("NONE")

        segments = 24
        rings = 12
        local_unit = []
        dot_tris = []

        for ri in range(rings + 1):
            v = math.pi * ri / rings
            sv = math.sin(v)
            cv = math.cos(v)
            for si in range(segments):
                u = 2.0 * math.pi * si / segments
                cu = math.cos(u)
                su = math.sin(u)
                local_unit.append(Vector((sv * cu, sv * su, cv)))

        for ri in range(rings):
            for si in range(segments):
                i0 = ri * segments + si
                i1 = ri * segments + (si + 1) % segments
                i2 = (ri + 1) * segments + si
                i3 = (ri + 1) * segments + (si + 1) % segments
                dot_tris.append((i0, i2, i1))
                dot_tris.append((i1, i2, i3))

        dot_batches = []
        for pos, radius in dot_handles:
            dot_verts = [pos + (v * radius) for v in local_unit]
            dot_batches.append(batch_for_shader(_SECTIONBOX_SHADER, "TRIS", {"pos": dot_verts}, indices=dot_tris))

        if dot_batches:
            depth_mask_original = (settings.shape != "CYLINDER")
            gpu.state.depth_mask_set(False)
            gpu.state.depth_test_set('ALWAYS')
            _SECTIONBOX_SHADER.uniform_float("color", (dr, dg, db, DOT_OCCLUDED_ALPHA))
            for batch_dot in dot_batches:
                batch_dot.draw(_SECTIONBOX_SHADER)

            gpu.state.depth_mask_set(depth_mask_original)
            gpu.state.depth_test_set('LESS_EQUAL')
            _SECTIONBOX_SHADER.uniform_float("color", (dr, dg, db, 1.0))
            for batch_dot in dot_batches:
                batch_dot.draw(_SECTIONBOX_SHADER)

    gpu.state.face_culling_set("NONE")
    gpu.state.depth_mask_set(True)
    gpu.state.blend_set("NONE")
    gpu.state.depth_test_set("LESS_EQUAL")


class SECTIONBOX_OT_toggle_show(bpy.types.Operator):
    bl_idname = "sectionbox.toggle_show"
    bl_label = "Show Cropbox"

    def execute(self, context):
        settings = get_sectionbox_settings(context)
        if not settings:
            return {'CANCELLED'}
        settings.show = not settings.show
        return {'FINISHED'}


class SECTIONBOX_OT_set_shape(bpy.types.Operator):
    bl_idname = "sectionbox.set_shape"
    bl_label = "Set Sectionbox Shape"

    shape: bpy.props.EnumProperty(
        items=[
            ("BOX", "Box", ""),
            ("CYLINDER", "Cylinder", ""),
        ]
    )

    def execute(self, context):
        settings = get_sectionbox_settings(context)
        if not settings:
            return {'CANCELLED'}

        prev_shape = settings.shape
        prev_size = Vector(settings.size)
        prev_radius = settings.radius
        prev_height = settings.height

        if prev_shape == "NONE":
            rv3d = context.region_data
            if rv3d is not None:
                settings.center = rv3d.view_location
            else:
                settings.center = context.scene.cursor.location

        if prev_shape == "BOX":
            cache_box_size(settings)
        elif prev_shape == "CYLINDER":
            cache_cylinder_size(settings)

        settings.shape = self.shape
        settings.show = True

        if self.shape == "BOX":
            cache = Vector(settings.box_cached_size)
            if cache.x > MIN_DIMENSION and cache.y > MIN_DIMENSION and cache.z > MIN_DIMENSION:
                settings.size = tuple(cache)
            elif prev_shape == "CYLINDER":
                diameter = max(prev_radius * 2.0, MIN_DIMENSION)
                settings.size = (diameter, diameter, max(prev_height, MIN_DIMENSION))
            else:
                settings.size = (1.0, 1.0, 1.0)
            cache_box_size(settings)
        elif self.shape == "CYLINDER":
            if settings.cyl_cached_radius > MIN_DIMENSION and settings.cyl_cached_height > MIN_DIMENSION:
                settings.radius = settings.cyl_cached_radius
                settings.height = settings.cyl_cached_height
            elif prev_shape == "BOX":
                half_x = max(prev_size.x * 0.5, MIN_DIMENSION)
                half_y = max(prev_size.y * 0.5, MIN_DIMENSION)
                settings.radius = float(math.hypot(half_x, half_y))
                settings.height = float(max(prev_size.z, MIN_DIMENSION))
            else:
                settings.radius = 0.5
                settings.height = 1.0
            cache_cylinder_size(settings, set_reference=True)
            update_box_cache_from_cylinder(settings)

        extra = (
            f"prev_shape={prev_shape} "
            f"prev_size={tuple(round(v, 5) for v in prev_size)} "
            f"prev_radius={prev_radius:.5f} prev_height={prev_height:.5f}"
        )
        log_shape_state(f"SWITCH {prev_shape}->{self.shape}", settings, extra)

        ensure_sectionbox_draw_handler()
        for area in context.screen.areas:
            if area.type == 'VIEW_3D':
                area.tag_redraw()

        return {'FINISHED'}


class SECTIONBOX_OT_resize_face(bpy.types.Operator):
    bl_idname = "sectionbox.resize_face"
    bl_label = "Resize Sectionbox Face"
    bl_options = {'REGISTER', 'UNDO', 'BLOCKING'}

    axis: bpy.props.IntProperty()
    direction: bpy.props.IntProperty(default=1)
    is_cylinder_radius: bpy.props.BoolProperty(default=False)

    def invoke(self, context, event):
        settings = get_sectionbox_settings(context)
        if not settings or settings.shape == "NONE":
            return {'CANCELLED'}

        region = context.region
        rv3d = context.region_data
        if region is None or rv3d is None:
            return {'CANCELLED'}

        self.settings = settings
        self.start_center = Vector(settings.center)
        self.anchor_point = self.start_center.copy()

        if self.is_cylinder_radius:
            self.start_size = settings.radius
        elif settings.shape == "BOX":
            self.start_size = settings.size[self.axis]
        else:
            self.start_size = settings.height

        if not self.is_cylinder_radius:
            half = self.start_size * 0.5
            offset = Vector((0.0, 0.0, 0.0))
            offset[self.axis] = -self.direction * half
            self.anchor_point = self.start_center + offset

        if self.is_cylinder_radius:
            self._log_event = "CYLINDER_RADIUS"
        elif settings.shape == "BOX":
            axis_name = ('X', 'Y', 'Z')[self.axis]
            self._log_event = f"BOX_AXIS_{axis_name}"
        else:
            self._log_event = "CYLINDER_HEIGHT"

        context.window_manager.modal_handler_add(self)
        return {'RUNNING_MODAL'}

    def modal(self, context, event):
        if event.type in {'RIGHTMOUSE', 'ESC'}:
            return {'CANCELLED'}

        if event.type == 'LEFTMOUSE' and event.value == 'RELEASE':
            log_shape_state(f"RESIZE_{getattr(self, '_log_event', 'UNKNOWN')}", self.settings)
            return {'FINISHED'}

        if event.type in {'MOUSEMOVE', 'INBETWEEN_MOUSEMOVE', 'LEFT_SHIFT', 'RIGHT_SHIFT', 'LEFTSHIFT', 'RIGHTSHIFT'}:
            self.update_transform(context, event)
            context.area.tag_redraw()

        return {'RUNNING_MODAL'}

    def update_transform(self, context, event):
        settings = self.settings
        region = context.region
        rv3d = context.region_data
        if not settings or region is None or rv3d is None:
            return

        axis_vec = Vector((0.0, 0.0, 0.0))
        axis_vec[self.axis] = 1.0

        coord = (event.mouse_region_x, event.mouse_region_y)
        view_vector = view3d_utils.region_2d_to_vector_3d(region, rv3d, coord)
        ray_origin = view3d_utils.region_2d_to_origin_3d(region, rv3d, coord)

        result = geometry.intersect_line_line(ray_origin, ray_origin + view_vector, self.start_center, self.start_center + axis_vec)
        if result is None:
            return
        pos_on_ray, pos_on_axis = result
        if pos_on_axis is None:
            return

        if self.is_cylinder_radius:
            dist = (pos_on_axis - self.start_center).length
            settings.radius = max(MIN_DIMENSION, dist)
            cache_cylinder_size(settings)
            update_box_cache_from_cylinder(settings)
            return

        diff_vec = pos_on_axis - self.anchor_point
        proj = diff_vec.dot(axis_vec)
        dist = proj if self.direction == 1 else -proj
        dist = max(MIN_DIMENSION, dist)

        if settings.shape == "BOX":
            base_sym = settings.symmetric_box
        else:
            base_sym = settings.symmetric_cylinder_height
        use_symmetry = bool(event.shift) or base_sym

        if use_symmetry:
            center_delta = (pos_on_axis - self.start_center).dot(axis_vec) * self.direction
            center_delta = max(MIN_HALF, center_delta)
            new_size = max(MIN_DIMENSION, center_delta * 2.0)
            new_center = self.start_center.copy()
        else:
            new_size = max(MIN_DIMENSION, dist)
            shift_vec = Vector((0.0, 0.0, 0.0))
            shift_vec[self.axis] = (new_size * 0.5) * self.direction
            new_center = self.anchor_point + shift_vec

        if settings.shape == "BOX":
            size = list(settings.size)
            size[self.axis] = new_size
            settings.size = tuple(size)
            settings.center = (new_center.x, new_center.y, new_center.z)
            cache_box_size(settings)
        else:
            settings.height = new_size
            cx, cy, _ = settings.center
            settings.center = (cx, cy, new_center.z)
            cache_cylinder_size(settings)
            update_box_cache_from_cylinder(settings)

class SECTIONBOX_OT_move_center(bpy.types.Operator):
    bl_idname = "sectionbox.move_center"
    bl_label = "Move Cropbox"
    bl_options = {'REGISTER', 'UNDO', 'BLOCKING'}

    def invoke(self, context, event):
        settings = get_sectionbox_settings(context)
        if not settings:
            return {'CANCELLED'}
        region = context.region
        rv3d = context.region_data
        if region is None or rv3d is None:
            return {'CANCELLED'}

        self.settings = settings
        self.region = region
        self.rv3d = rv3d
        self.area = context.area if context.area else None
        self.start_center = Vector(settings.center)
        self.axis_mask = Vector((1.0, 1.0, 1.0))
        self.plane_normal = rv3d.view_rotation @ Vector((0.0, 0.0, 1.0))
        self.start_hit = self._raycast(event) or self.start_center.copy()

        context.window_manager.modal_handler_add(self)
        return {'RUNNING_MODAL'}

    def modal(self, context, event):
        if event.type in {'RIGHTMOUSE', 'ESC'}:
            return {'CANCELLED'}
        if event.type == 'LEFTMOUSE' and event.value == 'RELEASE':
            log_shape_state("MOVE_CENTER", self.settings)
            return {'FINISHED'}
        if event.type in {'MOUSEMOVE', 'INBETWEEN_MOUSEMOVE'}:
            self._update_position(event)
        if event.type in {'X', 'Y', 'Z'} and event.value == 'PRESS':
            self._handle_axis_key(event)
        return {'RUNNING_MODAL'}

    def _handle_axis_key(self, event):
        axis_map = {'X': 0, 'Y': 1, 'Z': 2}
        axis = axis_map.get(event.type)
        if axis is None:
            return
        if event.shift:
            new_mask = Vector((1.0, 1.0, 1.0))
            new_mask[axis] = 0.0
        else:
            new_mask = Vector((0.0, 0.0, 0.0))
            new_mask[axis] = 1.0
        if tuple(new_mask) == tuple(self.axis_mask):
            self.axis_mask = Vector((1.0, 1.0, 1.0))
        else:
            self.axis_mask = new_mask

    def _update_position(self, event):
        hit = self._raycast(event)
        if hit is None:
            return
        delta = hit - self.start_hit
        masked = Vector((
            delta.x * self.axis_mask.x,
            delta.y * self.axis_mask.y,
            delta.z * self.axis_mask.z,
        ))
        new_center = self.start_center + masked
        self.settings.center = (new_center.x, new_center.y, new_center.z)
        if self.area:
            self.area.tag_redraw()

    def _raycast(self, event):
        coord = (event.mouse_region_x, event.mouse_region_y)
        view_vector = view3d_utils.region_2d_to_vector_3d(self.region, self.rv3d, coord)
        ray_origin = view3d_utils.region_2d_to_origin_3d(self.region, self.rv3d, coord)
        plane_co = self.start_center
        hit = geometry.intersect_line_plane(ray_origin, ray_origin + view_vector, plane_co, self.plane_normal, False)
        return Vector(hit) if hit else None


class SECTIONBOX_GGT_gizmos(bpy.types.GizmoGroup):
    bl_idname = "SECTIONBOX_GGT_gizmos"
    bl_label = "Sectionbox Gizmos"
    bl_space_type = 'VIEW_3D'
    bl_region_type = 'WINDOW'
    bl_options = {'3D', 'PERSISTENT'}

    @classmethod
    def poll(cls, context):
        s = get_sectionbox_settings(context)
        return bool(s and s.show and s.shape != "NONE")

    def setup(self, context):
        gizmos = self.gizmos
        settings = get_sectionbox_settings(context)
        scale_scale = settings.gizmo_scale_scale if settings else 0.25
        scale_move = settings.gizmo_scale_move if settings else 0.35

        self.gz_x_pos = gizmos.new("GIZMO_GT_arrow_3d")
        self.gz_x_neg = gizmos.new("GIZMO_GT_arrow_3d")
        self.gz_y_pos = gizmos.new("GIZMO_GT_arrow_3d")
        self.gz_y_neg = gizmos.new("GIZMO_GT_arrow_3d")
        self.gz_z_pos = gizmos.new("GIZMO_GT_arrow_3d")
        self.gz_z_neg = gizmos.new("GIZMO_GT_arrow_3d")

        for gz in (
            self.gz_x_pos, self.gz_x_neg,
            self.gz_y_pos, self.gz_y_neg,
            self.gz_z_pos, self.gz_z_neg,
        ):
            gz.use_draw_scale = True
            gz.scale_basis = scale_scale
            gz.draw_style = 'BOX'
            gz.color = (0.0, 0.0, 0.0)
            gz.alpha = 0.0
            gz.color_highlight = (0.0, 0.0, 0.0)
            gz.alpha_highlight = 0.0

        self.gz_height_top = gizmos.new("GIZMO_GT_arrow_3d")
        self.gz_height_top.use_draw_scale = True
        self.gz_height_top.scale_basis = scale_scale
        self.gz_height_top.draw_style = 'BOX'
        self.gz_height_top.color = (0.0, 0.0, 0.0)
        self.gz_height_top.alpha = 0.0
        self.gz_height_top.color_highlight = (0.0, 0.0, 0.0)
        self.gz_height_top.alpha_highlight = 0.0

        self.gz_height_bottom = gizmos.new("GIZMO_GT_arrow_3d")
        self.gz_height_bottom.use_draw_scale = True
        self.gz_height_bottom.scale_basis = scale_scale
        self.gz_height_bottom.draw_style = 'BOX'
        self.gz_height_bottom.color = (0.0, 0.0, 0.0)
        self.gz_height_bottom.alpha = 0.0
        self.gz_height_bottom.color_highlight = (0.0, 0.0, 0.0)
        self.gz_height_bottom.alpha_highlight = 0.0

        self.gz_radius = gizmos.new("GIZMO_GT_arrow_3d")
        self.gz_radius.use_draw_scale = True
        self.gz_radius.scale_basis = scale_scale
        self.gz_radius.draw_style = 'BOX'
        self.gz_radius.color = (0.0, 0.0, 0.0)
        self.gz_radius.alpha = 0.0
        self.gz_radius.color_highlight = (0.0, 0.0, 0.0)
        self.gz_radius.alpha_highlight = 0.0

        self.gz_move = gizmos.new("GIZMO_GT_move_3d")
        self.gz_move.use_draw_scale = True
        self.gz_move.scale_basis = scale_move
        self.gz_move.color = (1.0, 0.7, 0.0)
        self.gz_move.alpha = 0.9
        self.gz_move.color_highlight = (1.0, 1.0, 0.2)
        self.gz_move.alpha_highlight = 1.0
        self.gz_move.target_set_operator("sectionbox.move_center")

        def bind_resize_operator(gizmo, axis, direction, is_radius=False):
            op = gizmo.target_set_operator("sectionbox.resize_face")
            op.axis = axis
            op.direction = direction
            op.is_cylinder_radius = is_radius

        bind_resize_operator(self.gz_x_pos, 0, +1)
        bind_resize_operator(self.gz_x_neg, 0, -1)
        bind_resize_operator(self.gz_y_pos, 1, +1)
        bind_resize_operator(self.gz_y_neg, 1, -1)
        bind_resize_operator(self.gz_z_pos, 2, +1)
        bind_resize_operator(self.gz_z_neg, 2, -1)

        bind_resize_operator(self.gz_height_top, 2, +1)
        bind_resize_operator(self.gz_height_bottom, 2, -1)
        bind_resize_operator(self.gz_radius, 0, +1, is_radius=True)

    def draw_prepare(self, context):
        settings = get_sectionbox_settings(context)
        if not settings:
            return
        if settings.hide_container:
            for gz in self.gizmos:
                gz.hide = True
            return
        else:
            for gz in self.gizmos:
                gz.hide = False

        center = Vector(settings.center)

        scale_scale = settings.gizmo_scale_scale
        scale_move = settings.gizmo_scale_move

        self.gz_move.scale_basis = scale_move

        is_box = settings.shape == "BOX"
        is_cyl = settings.shape == "CYLINDER"
        view_loc, view_distance, view_is_persp = get_view_info()

        def calc_hit_scale(pos: Vector) -> float:
            radius = compute_screen_scale(pos, scale_scale, view_loc, view_distance, view_is_persp)
            if radius <= 0.0:
                radius = scale_scale
            radius = max(radius, scale_scale * 1.5, HANDLE_SCREEN_MIN)
            return radius * HANDLE_HIT_SCALE

        self.gz_x_pos.hide = not is_box
        self.gz_x_neg.hide = not is_box
        self.gz_y_pos.hide = not is_box
        self.gz_y_neg.hide = not is_box
        self.gz_z_pos.hide = not is_box
        self.gz_z_neg.hide = not is_box

        if is_box:
            sx, sy, sz = settings.size
            hx, hy, hz = sx * 0.5, sy * 0.5, sz * 0.5
            hx_vis = max(hx, HANDLE_VISUAL_MIN)
            hy_vis = max(hy, HANDLE_VISUAL_MIN)
            hz_vis = max(hz, HANDLE_VISUAL_MIN)
            rot_x_pos = Matrix.Rotation(math.pi * 0.5, 4, 'Y')
            rot_x_neg = Matrix.Rotation(-math.pi * 0.5, 4, 'Y')
            rot_y_pos = Matrix.Rotation(-math.pi * 0.5, 4, 'X')
            rot_y_neg = Matrix.Rotation(math.pi * 0.5, 4, 'X')
            rot_z_pos = Matrix.Identity(4)
            rot_z_neg = Matrix.Rotation(math.pi, 4, 'X')

            face_positions = (
                center + Vector((+hx_vis, 0.0, 0.0)),
                center + Vector((-hx_vis, 0.0, 0.0)),
                center + Vector((0.0, +hy_vis, 0.0)),
                center + Vector((0.0, -hy_vis, 0.0)),
                center + Vector((0.0, 0.0, +hz_vis)),
                center + Vector((0.0, 0.0, -hz_vis)),
            )
            rotations = (
                rot_x_pos, rot_x_neg,
                rot_y_pos, rot_y_neg,
                rot_z_pos, rot_z_neg,
            )
            gizmos = (
                self.gz_x_pos, self.gz_x_neg,
                self.gz_y_pos, self.gz_y_neg,
                self.gz_z_pos, self.gz_z_neg,
            )

            normals = (
                Vector((+1.0, 0.0, 0.0)),
                Vector((-1.0, 0.0, 0.0)),
                Vector((0.0, +1.0, 0.0)),
                Vector((0.0, -1.0, 0.0)),
                Vector((0.0, 0.0, +1.0)),
                Vector((0.0, 0.0, -1.0)),
            )

            for gz, base_pos, normal, rot in zip(gizmos, face_positions, normals, rotations):
                pos = base_pos + normal * HANDLE_OFFSET
                gz.matrix_basis = Matrix.Translation(pos) @ rot
                gz.scale_basis = calc_hit_scale(pos)

        self.gz_height_top.hide = not is_cyl
        self.gz_height_bottom.hide = not is_cyl
        self.gz_radius.hide = not is_cyl

        if is_cyl:
            r_val = settings.radius
            h_half = settings.height * 0.5
            vis_r = max(r_val, HANDLE_VISUAL_MIN)
            vis_h = max(h_half, HANDLE_VISUAL_MIN)
            rot_z_pos = Matrix.Identity(4)
            rot_z_neg = Matrix.Rotation(math.pi, 4, 'X')
            pos_top = center + Vector((0.0, 0.0, +vis_h)) + Vector((0.0, 0.0, HANDLE_OFFSET))
            pos_bottom = center + Vector((0.0, 0.0, -vis_h)) + Vector((0.0, 0.0, -HANDLE_OFFSET))
            pos_radius = center + Vector((vis_r, 0.0, 0.0)) + Vector((HANDLE_OFFSET, 0.0, 0.0))

            self.gz_height_top.matrix_basis = Matrix.Translation(pos_top) @ rot_z_pos
            self.gz_height_top.scale_basis = calc_hit_scale(pos_top)

            self.gz_height_bottom.matrix_basis = Matrix.Translation(pos_bottom) @ rot_z_neg
            self.gz_height_bottom.scale_basis = calc_hit_scale(pos_bottom)

            self.gz_radius.matrix_basis = Matrix.Translation(pos_radius) @ Matrix.Rotation(math.pi * 0.5, 4, 'Y')
            self.gz_radius.scale_basis = calc_hit_scale(pos_radius)

        self.gz_move.hide = (settings.shape == "NONE")
        if not self.gz_move.hide:
            bottom_anchor = get_bottom_anchor_vector(settings)
            self.gz_move.matrix_basis = Matrix.Translation(bottom_anchor)
        else:
            self.gz_move.matrix_basis = Matrix.Identity(4)


def fit_sectionbox_to_points(settings, layer=None, context=None):
    """
    Stable Fit implementation: axis-aligned bounding box based on cached
    `_SOURCE_POS` coordinates (no extra matrices). This matches the robust
    Lite behavior prior to our recent experiments.
    """
    if settings is None or _SOURCE_POS is None or len(_SOURCE_POS) == 0:
        return
    positions = np.asarray(_SOURCE_POS, dtype=np.float32)
    min_v = positions.min(axis=0)
    max_v = positions.max(axis=0)
    dims = np.maximum(max_v - min_v, MIN_DIMENSION)
    center = (min_v + max_v) * 0.5

    settings.center = tuple(center.tolist())
    settings.size = tuple(dims.tolist())
    settings.radius = float(max(dims[0], dims[1]) * 0.5)
    settings.height = float(max(dims[2], MIN_DIMENSION))
    cache_box_size(settings)
    cache_cylinder_size(settings, set_reference=True)
    settings.auto_fit_done = True


def ensure_sectionbox_ready(context):
    settings = get_sectionbox_settings(context)
    if not settings:
        return None
    ensure_sectionbox_shader()
    ensure_sectionbox_draw_handler()
    if settings.shape == "NONE":
        settings.shape = "BOX"
    if not settings.auto_fit_done:
        layer = get_active_layer(context)
        fit_sectionbox_to_points(settings, layer=layer, context=context)
    settings.show = True
    return settings


def get_crop_uniforms(props, context):
    zero = Vector((0.0, 0.0, 0.0, 0.0))
    settings = get_sectionbox_settings(context)
    if not props.crop_enable or not settings or not settings.show or settings.shape == "NONE":
        return False, zero, zero, zero, 1.0

    center = Vector(settings.center)
    center_vec = Vector((center.x, center.y, center.z, 0.0))
    
    if settings.shape == "BOX":
        size = Vector(settings.size)
        half = Vector((
            max(size.x * 0.5, MIN_DIMENSION),
            max(size.y * 0.5, MIN_DIMENSION),
            max(size.z * 0.5, MIN_DIMENSION),
        ))
        inv = Vector((1.0 / half.x, 1.0 / half.y, 1.0 / half.z))
        inv_vec = Vector((inv.x, inv.y, inv.z, 0.0))
        return True, center_vec, inv_vec, zero, 1.0

    radius = max(settings.radius, MIN_DIMENSION)
    h_half = max(settings.height * 0.5, MIN_DIMENSION)
    inv = Vector((1.0 / radius, 1.0 / radius, 1.0 / h_half))
    inv_vec = Vector((inv.x, inv.y, inv.z, 0.0))
    return True, center_vec, inv_vec, zero, 2.0


def compute_crop_mask(settings, layer=None):
    if settings is None or settings.shape == "NONE":
        return None
    positions = None
    if layer:
        data = get_layer_data(layer)
        if data:
            positions = data.get("source_pos")
    if positions is None:
        positions = _SOURCE_POS
    if positions is None or len(positions) == 0:
        return None

    matrix = get_layer_transform_matrix(layer) if layer else Matrix.Identity(4)
    pos = transform_points_array(positions, matrix)
    center = np.array(settings.center, dtype=np.float32)

    if settings.shape == "BOX":
        size = np.array(settings.size, dtype=np.float32)
        half = np.maximum(size * 0.5, MIN_DIMENSION)
        norm = np.abs((pos - center) / half)
        return np.all(norm <= 1.0, axis=1)

    radius = max(settings.radius, MIN_DIMENSION)
    height = max(settings.height, MIN_DIMENSION)
    xy = pos[:, :2] - center[:2]
    z = pos[:, 2] - center[2]
    circle = np.sum((xy / radius) ** 2, axis=1) <= 1.0
    height_mask = np.abs(z / (height * 0.5)) <= 1.0
    return circle & height_mask


def create_mesh_from_points(context, name, points, colors=None, reference=None):
    mesh = bpy.data.meshes.new(name)
    pts = np.asarray(points, dtype=np.float32).reshape(-1, 3)
                                                                          
                                                                              
                                                                          
                                                                       
                                                                       
    mesh.vertices.add(len(pts))
    mesh.vertices.foreach_set("co", pts.ravel())
    if colors is not None:
        attr = mesh.color_attributes.new(name="Color", type='FLOAT_COLOR', domain='POINT')
        attr.data.foreach_set("color", np.asarray(colors, dtype=np.float32).ravel())
    mesh.update()
    obj = bpy.data.objects.new(name, mesh)
    dst_collection = context.collection if context and context.collection else context.scene.collection
    dst_collection.objects.link(obj)
    return obj


def update_mesh_from_points(mesh_obj, points, colors=None):
    if mesh_obj is None or mesh_obj.type != 'MESH':
        return
    mesh = mesh_obj.data
    mesh.clear_geometry()
    total = len(points)
    if total == 0:
        mesh.update()
        return
                                                                             
                                                                         
                                                                              
                                                                           
                                                      
    pts = np.asarray(points, dtype=np.float32).reshape(-1, 3)
    mesh.vertices.add(len(pts))
    mesh.vertices.foreach_set("co", pts.ravel())
    if colors is not None:
        cols = np.asarray(colors, dtype=np.float32)
        if cols.ndim == 1:
            cols = np.broadcast_to(cols, (len(pts), 4))
        if cols.shape[1] == 3:
            cols = np.column_stack((cols, np.ones(len(cols))))
    else:
        cols = None
    for attr in list(mesh.color_attributes):
        mesh.color_attributes.remove(attr)
    if cols is not None:
        attr = mesh.color_attributes.new(name="Color", type='FLOAT_COLOR', domain='POINT')
        attr.data.foreach_set("color", cols.astype(np.float32).ravel())
    mesh.update()

def ensure_pointcloud_draw_handler():
    global POINTCLOUD_DRAW_HANDLER
    if POINTCLOUD_DRAW_HANDLER is None:
        POINTCLOUD_DRAW_HANDLER = bpy.types.SpaceView3D.draw_handler_add(draw_callback, (), 'WINDOW', 'POST_VIEW')

def remove_pointcloud_draw_handler():
    global POINTCLOUD_DRAW_HANDLER
    if POINTCLOUD_DRAW_HANDLER is not None:
        bpy.types.SpaceView3D.draw_handler_remove(POINTCLOUD_DRAW_HANDLER, 'WINDOW')
        POINTCLOUD_DRAW_HANDLER = None



                                                                           
                           
                                                                           

def update_ui(self, context):
    if context and getattr(context, "area", None):
        context.area.tag_redraw()


SCREEN_TOOL_UPDATE_LOCK = False
SCREEN_TOOL_ITEMS = (
    ('BOX', "Box", "Rectangle selection"),
    ('LASSO', "Lasso", "Freehand selection"),
    ('CIRCLE', "Circle", "Circle selection"),
)

@contextmanager
def suspend_screen_tool_update():
    global SCREEN_TOOL_UPDATE_LOCK
    previous = SCREEN_TOOL_UPDATE_LOCK
    SCREEN_TOOL_UPDATE_LOCK = True
    try:
        yield
    finally:
        SCREEN_TOOL_UPDATE_LOCK = previous

def screen_tool_update(self, context):
    global SCREEN_TOOL_UPDATE_LOCK
    if SCREEN_TOOL_UPDATE_LOCK:
        return
    ctx = context or getattr(bpy, "context", None)
    if not ctx or not getattr(ctx, "scene", None):
        return
    props = getattr(ctx.scene, "pcv_props", None)
    if props is None or not getattr(props, "screen_edit_enable", False):
        return
    if get_active_layer(ctx) is None:
        return
    tool = props.screen_tool if props.screen_tool in {'BOX', 'CIRCLE', 'LASSO'} else 'BOX'
    ensure_workspace_screen_tool(ctx, tool)


def get_workspace_screen_tool(context):
    ctx = context or bpy.context
    if not ctx:
        return None
    workspace = getattr(ctx, "workspace", None)
    if not workspace:
        return None
    try:
        return workspace.tools.from_space_view3d_mode(ctx.mode, create=False)
    except Exception:
        return None


def activate_view_tool(context, tool_id):
    ctx = context or bpy.context
    if not ctx:
        return False
    window, area, region = _find_view3d_area_region(ctx)
    if not area or not region:
        return False
    temp_override = getattr(ctx, "temp_override", None)
    if temp_override is None:
        base_ctx = getattr(bpy, "context", None)
        if not base_ctx:
            return False
        temp_override = base_ctx.temp_override
    override_kwargs = {"area": area, "region": region}
    if window:
        override_kwargs["window"] = window
        override_kwargs["screen"] = window.screen
    try:
        with temp_override(**override_kwargs):
            bpy.ops.wm.tool_set_by_id(name=tool_id)
    except Exception:
        return False
    return True


def ensure_workspace_screen_tool(context, shape=None):
    ctx = context or bpy.context
    if not ctx:
        return False
    tool = get_workspace_screen_tool(ctx)
    if not tool or tool.idname != "pcv.screen_select_tool":
        if not activate_view_tool(ctx, "pcv.screen_select_tool"):
            return False
        tool = get_workspace_screen_tool(ctx)
    if tool and shape in {'BOX', 'CIRCLE', 'LASSO'}:
        try:
            props = tool.operator_properties("pcv.screen_select")
            props.mode = shape
        except Exception:
            pass
    return bool(tool)


def ensure_screen_selection_mode(context, mode=None, *, source='AUTO', force=False):
    ctx = context or getattr(bpy, "context", None)
    if not ctx or not getattr(ctx, "scene", None):
        return False
    props = getattr(ctx.scene, "pcv_props", None)
    if not props:
        return False
    layer = get_active_layer(ctx)
                                                                      
                                                                   
                                       
    if not layer:
        scene = getattr(ctx, "scene", None)
        if scene and hasattr(scene, "pcv_layers"):
            ensure_lite_layers(scene, ctx)
            layer = get_active_layer(ctx)
        if not layer:
            return False
    valid_tools = {item[0] for item in SCREEN_TOOL_ITEMS} if 'SCREEN_TOOL_ITEMS' in globals() else {'BOX', 'CIRCLE', 'LASSO'}
    target = mode if mode in valid_tools else props.screen_tool
    if target not in valid_tools:
        target = 'BOX'
    changed = props.screen_tool != target
    if changed:
                                                                         
                                                                            
                                                                           
                                                            
        with suspend_screen_tool_update():
            props.screen_tool = target
    was_enabled = getattr(props, "screen_edit_enable", False)
    if not was_enabled:
        props.screen_edit_enable = True
        props.crop_enable = False
        settings = get_sectionbox_settings(ctx)
        if settings:
            settings.show = False
            update_gizmo_view(settings, ctx)
    ensure_workspace_screen_tool(ctx, target)
    return True


class PCVPropertyGroup(bpy.types.PropertyGroup):
    point_shape: bpy.props.EnumProperty(
        name="Point Shape",
        items=[('SQUARE', "Square", ""), ('ROUND', "Circle", "")],
        default='ROUND',
    )
    crop_enable: bpy.props.BoolProperty(name="Cropbox", default=False, update=update_ui)
    crop_mode: bpy.props.EnumProperty(
        name="Crop Mode",
        items=[('INSIDE', "Inside", "Show points inside the cropbox"), ('OUTSIDE', "Outside", "Show points outside the cropbox")],
        default='INSIDE',
        update=update_ui,
    )
    screen_edit_enable: bpy.props.BoolProperty(
        name="Screen Selection",
        description="Enable screen-space selection tools",
        default=False,
        update=update_ui,
    )
    screen_tool: bpy.props.EnumProperty(
        name="Screen Tool",
        items=SCREEN_TOOL_ITEMS,
        default='BOX',
        update=screen_tool_update,
    )
    screen_circle_radius: bpy.props.FloatProperty(
        name="Circle Radius",
        description="Radius of the circle selection brush (pixels)",
        default=25.0,
        min=3.0,
        max=500.0,
    )
    lite_enabled: bpy.props.BoolProperty(
        name="Lite Colored Enabled",
        description="Track global Colored (GLSL) state for Lite workflow",
        default=False,
        options={'HIDDEN'},
    )
    auto_lite_stamp: bpy.props.IntProperty(
        name="Lite Auto Stamp",
        description="Internal stamp incremented when Lite Colored is toggled ON",
        default=0,
        options={'HIDDEN'},
    )
    enable_auto_lite: bpy.props.BoolProperty(
        name="Enable Auto Lite (Experimental)",
        description="Experimental flag: when true, tries to auto-capture new point clouds while Colored is ON",
        default=False,
        options={'HIDDEN'},
    )

class PCV_OT_AddLayer(bpy.types.Operator):
    bl_idname = "pcv.layer_add"
    bl_label = "Add Layer"

    def execute(self, context):
        active = context.active_object
        base_name = active.name if active else None
        layer = add_new_layer(context.scene, name=base_name, make_active=True)
        if layer:
            sync_props_from_layer(context)
        return {'FINISHED'}

class PCV_OT_RemoveLayer(bpy.types.Operator):
    bl_idname = "pcv.layer_remove"
    bl_label = "Remove Layer"

    def execute(self, context):
        scene = context.scene
        idx = scene.pcv_layers_index
        if idx < 0 or idx >= len(scene.pcv_layers):
            return {'CANCELLED'}
        layer = scene.pcv_layers[idx]
        remove_layer_data(layer)
        toggle_source_mesh_visibility(layer, False, overlay=False)
        scene.pcv_layers.remove(idx)
        scene.pcv_layers_index = max(0, idx - 1)
        sync_props_from_layer(context)
        update_draw_handler_for_scene(scene)
        return {'FINISHED'}

def ensure_layer_uid(layer):
    if not layer.uid:
        layer.uid = str(uuid.uuid4())
    return layer.uid

def get_active_layer_from_scene(scene):
    if scene is None:
        return None
    layers = getattr(scene, "pcv_layers", None)
    if not layers or len(layers) == 0:
        return None
    raw_index = getattr(scene, "pcv_layers_index", 0)
    clamped = max(0, min(raw_index, len(layers) - 1))
    if clamped != raw_index:
        scene.pcv_layers_index = clamped
    return layers[clamped]

def sync_props_from_layer(context):
    context = context or bpy.context
    scene = getattr(context, "scene", None) if context else None
    layer = get_active_layer(context)
    if layer:
        set_globals_from_layer(layer)
    else:
        set_globals_from_layer(None)
    request_redraw()

def get_active_layer(context):
    context = context or bpy.context
    if not context or not context.scene:
        return None
    return get_active_layer_from_scene(context.scene)

def get_layer_data(layer):
    if not layer:
        return None
    uid = ensure_layer_uid(layer)
    data = _CLOUD_DATA.get(uid)
    if data is None:
        base_color = getattr(layer, "selection_color", DEFAULT_SELECTION_COLOR)
        blend_strength = float(getattr(layer, "selection_color_strength", DEFAULT_SELECTION_BLEND))
        blend_strength = max(0.0, min(2.0, blend_strength))
        data = {
            "source_pos": None,
            "source_col": None,
            "positions": None,
            "colors": None,
            "batch": None,
            "point_count": 0,
            "dirty": False,
            "visible_indices": None,
            "selection_mask": None,
            "selection_count": 0,
            "selection_color": (base_color[0], base_color[1], base_color[2], blend_strength),
            "was_in_edit": False,
        }
        _CLOUD_DATA[uid] = data
    return data

def add_new_layer(scene, name=None, make_active=True):
    if scene is None:
        return None
    layer = scene.pcv_layers.add()
    if name:
        layer.name = name
    else:
        layer.name = f"Cloud {len(scene.pcv_layers)}"
    ensure_layer_uid(layer)
    if make_active:
        scene.pcv_layers_index = len(scene.pcv_layers) - 1
    return layer

def remove_layer_data(layer):
    if layer and layer.uid in _CLOUD_DATA:
        _CLOUD_DATA.pop(layer.uid, None)
    toggle_source_mesh_visibility(layer, hide=False, overlay=False)
    remove_layer_transform_object(layer)

def get_layer_point_count(layer):
    data = get_layer_data(layer)
    if not data:
        return 0
    return data.get("point_count", 0)


def get_layer_selection(layer):
    data = get_layer_data(layer)
    if not data:
        return None, 0
    mask = data.get("selection_mask")
    count = data.get("selection_count", 0)
    if mask is None:
        count = 0
    return mask, count


def set_layer_selection(layer, mask):
    data = get_layer_data(layer)
    if not data:
        return
    src = data.get("source_pos")
    if src is None:
        return
    length = len(src)
    if mask is None:
        data["selection_mask"] = None
        data["selection_count"] = 0
    else:
        mask = np.asarray(mask, dtype=bool)
        if len(mask) != length:
            new_mask = np.zeros(length, dtype=bool)
            upto = min(length, len(mask))
            if upto > 0:
                new_mask[:upto] = mask[:upto]
            mask = new_mask
        data["selection_mask"] = mask
        data["selection_count"] = int(np.count_nonzero(mask))
    mark_layer_dirty(layer)
    request_redraw()


def clear_layer_selection(layer):
    set_layer_selection(layer, None)


def ensure_active_layer(context, create=False, name=None):
    context = context or bpy.context
    if not context or not context.scene:
        return None
    scene = context.scene
    layers = scene.pcv_layers
    if layers and len(layers) > 0 and 0 <= scene.pcv_layers_index < len(layers):
        return layers[scene.pcv_layers_index]
    if layers and len(layers) > 0:
        scene.pcv_layers_index = 0
        return layers[0]
    if create:
        layer = layers.add()
        layer.name = name or f"Cloud {len(layers)}"
        ensure_layer_uid(layer)
        scene.pcv_layers_index = len(layers) - 1
        sync_props_from_layer(context)
        return layer
    return None

def layer_index_update(self, context):
    global _LAYER_INDEX_LOCK
    if _LAYER_INDEX_LOCK:
        return
    _LAYER_INDEX_LOCK = True
    try:
        ctx = context or bpy.context
        sync_props_from_layer(ctx)
        schedule_layer_refresh(ctx)
    finally:
        _LAYER_INDEX_LOCK = False

                   

def extract_data(mesh, props):
    if not len(mesh.vertices): return None, None
    pos = np.empty(len(mesh.vertices)*3, dtype=np.float32)
    mesh.vertices.foreach_get("co", pos)
    pos = pos.reshape(-1, 3)
    
    col = None
    if mesh.color_attributes:
        idx = mesh.color_attributes.render_color_index
        if idx == -1 and len(mesh.color_attributes)>0: idx=0
        if idx != -1:
            attr = mesh.color_attributes[idx]
            if attr.domain == 'POINT':
                data_len = len(attr.data)
                if data_len == len(mesh.vertices) and data_len > 0:
                    raw = np.empty(data_len*4, dtype=np.float32)
                    try:
                        attr.data.foreach_get('color', raw)
                        col = raw.reshape(-1, 4)
                    except TypeError:
                        col = None
    return pos, col

class PCV_OT_LoadPly(bpy.types.Operator, ImportHelper):
    bl_idname = "pcv.load_ply"
    bl_label = "Load File"
    filename_ext = ".ply;.glb;.gltf"
    filter_glob: bpy.props.StringProperty(default="*.ply;*.glb;*.gltf", options={'HIDDEN'})

    def execute(self, context):
        props = context.scene.pcv_props
        fpath = self.filepath
        if not os.path.exists(fpath): return {'CANCELLED'}
        
        pos, col = None, None
        ext = os.path.splitext(fpath)[1].lower()
        
        try:
            if ext == '.ply':
                pd = PlyPointCloudReader(fpath)
                if not pd.has_vertices: return {'CANCELLED'}
                pos = np.column_stack((pd.points['x'], pd.points['y'], pd.points['z']))
                if pd.has_colors:
                    col = np.column_stack((pd.points['red'], pd.points['green'], pd.points['blue']))/255.0
                    col = np.hstack((col, np.ones((col.shape[0], 1))))
            elif ext in ('.glb', '.gltf'):
                bpy.ops.import_scene.gltf(filepath=fpath)
                all_pos = []
                for obj in context.selected_objects:
                    if obj.type == 'MESH':
                        p, c = extract_data(obj.data, props)
                        if p is not None:
                            mat = np.array(obj.matrix_world)
                            p4 = np.hstack((p, np.ones((len(p),1), dtype=np.float32)))
                            all_pos.append((p4 @ mat.T)[:, :3])
                bpy.ops.object.delete()
                if all_pos: pos = np.vstack(all_pos)
        except Exception as e:
            self.report({'ERROR'}, str(e))
            return {'CANCELLED'}

        if pos is not None:
            layer = context.scene.pcv_layers.add()
            layer.name = Path(fpath).stem or "File Cloud"
            context.scene.pcv_layers_index = len(context.scene.pcv_layers) - 1
            ensure_layer_uid(layer)
            if update_point_cloud_data(pos, col, None, props, layer):
                layer.filepath = fpath
                layer.show_points = True
                ensure_layer_transform_driver(layer, context=context)
                refresh_layer_visibility(layer, context)
                settings = get_sectionbox_settings(context)
                if settings and not settings.auto_fit_done:
                    fit_sectionbox_to_points(settings, layer=layer, context=context)
                if props.crop_enable:
                    ensure_sectionbox_ready(context)
                sync_props_from_layer(context)
        return {'FINISHED'}

class PCV_OT_RenderActive(bpy.types.Operator):
    bl_idname = "pcv.render_active"
    bl_label = "Render Points"
    bl_options = {'REGISTER', 'UNDO'}
    @classmethod
    def poll(cls, context):
        has_mesh = context.active_object and context.active_object.type == 'MESH'
        layer = get_active_layer(context)
        data = get_layer_data(layer) if layer else None
        has_layer = bool(data and data.get("batch"))
        return has_mesh or has_layer
    def execute(self, context):
        props = context.scene.pcv_props
        obj = context.active_object if context.active_object and context.active_object.type == 'MESH' else None

        layer = get_active_layer(context)
        if obj is None:
            self.report({'WARNING'}, "Select a mesh to render as points")
            return {'CANCELLED'}

        p, c = extract_data(obj.data, props)
        if p is None: return {'CANCELLED'}
        
        mat = np.array(obj.matrix_world)
        p4 = np.hstack((p, np.ones((len(p),1), dtype=np.float32)))
        p_world = (p4 @ mat.T)[:, :3]

        if layer is None:
            layer = ensure_active_layer(context, create=True, name=obj.name)
        if not layer.name or layer.name.startswith("Cloud"):
            layer.name = obj.name
        if update_point_cloud_data(p_world, c, None, props, layer):
            layer.filepath = obj.name
            layer.show_points = True
            ensure_layer_transform_driver(layer, context=context, source_obj=obj)
            refresh_layer_visibility(layer, context)
                                                                      
                                                                       
            settings = get_sectionbox_settings(context)
            if settings and not settings.auto_fit_done:
                fit_sectionbox_to_points(settings, layer=layer, context=context)
            if props.crop_enable:
                ensure_sectionbox_ready(context)
            sync_props_from_layer(context)
        return {'FINISHED'}

class PCV_OT_ToggleCropbox(bpy.types.Operator):
    bl_idname = "pcv.toggle_cropbox"
    bl_label = "Cropbox"

    def execute(self, context):
        props = context.scene.pcv_props
        settings = get_sectionbox_settings(context)
        if not settings:
            self.report({'ERROR'}, "Sectionbox settings unavailable")
            return {'CANCELLED'}
        props.crop_enable = not props.crop_enable
        settings.show = props.crop_enable
        if props.crop_enable:
            props.screen_edit_enable = False
            ensure_sectionbox_ready(context)
            layer = get_active_layer(context)
            if layer:
                log_screen_event("clear_selection_on_crop", layer=getattr(layer, "name", None))
                set_layer_selection(layer, None)
        else:
            props.crop_mode = 'INSIDE'
        update_gizmo_view(settings, context)
        return {'FINISHED'}

class PCV_OT_SplitPoints(bpy.types.Operator):
    bl_idname = "pcv.split_points"
    bl_label = "Split"
    bl_description = "Create mesh objects for inside and outside point selections"

    def execute(self, context):
        props = context.scene.pcv_props
                                                                     
                                                                         
                                                                     
                                                                        
                                                                          
        primary_layer = get_active_layer(context)
        if not primary_layer:
            self.report({'WARNING'}, "No active layer")
            return {'CANCELLED'}
        layer = primary_layer
        data = get_layer_data(layer)
        src_pos = data.get("source_pos") if data else None
        src_col = data.get("source_col") if data else None
        settings = get_sectionbox_settings(context)
        crop_state = capture_cropbox_state(settings)
        scene = context.scene
        source_obj = get_layer_source_object(layer, scene)
                                                                      
                                              
        original_driver = getattr(layer, "transform_object", None)
        original_rig = getattr(layer, "rig_object", None)
        driver_obj = getattr(layer, "transform_object", None)
        driver_name = getattr(driver_obj, "name", None)
        driver_type = "SOURCE" if driver_obj is source_obj else "RIG" if driver_obj is getattr(layer, "rig_object", None) else "OTHER"
        rig_obj = getattr(layer, "rig_object", None)
        selection_mode = "SCREEN" if getattr(props, "screen_edit_enable", False) else "CROP"
        crop_mode = getattr(props, "crop_mode", "INSIDE")
        crop_enabled = int(getattr(props, "crop_enable", False))
        shape = getattr(settings, "shape", "NONE") if settings else "NONE"
                                                         
        try:
            glsl_matrix_before = get_layer_transform_matrix(layer).copy()
            glsl_mat = _format_matrix_short(glsl_matrix_before)
        except Exception:
            glsl_matrix_before = None
            glsl_mat = "matrix=<unavailable>"
        src_parent = getattr(source_obj, "parent", None) if source_obj else None
        src_parent_name = getattr(src_parent, "name", None) if src_parent else None
        try:
            src_parent_matrix_before = src_parent.matrix_world.copy() if src_parent else None
            src_parent_mat = _format_matrix_short(src_parent_matrix_before) if src_parent else "none"
        except Exception:
            src_parent_matrix_before = None
            src_parent_mat = "matrix=<unavailable>"
        original_source_matrix = None
        original_rig_matrix = None
        if source_obj and hasattr(source_obj, "matrix_world"):
            try:
                original_source_matrix = source_obj.matrix_world.copy()
            except Exception:
                original_source_matrix = None
        if original_rig and hasattr(original_rig, "matrix_world"):
            try:
                original_rig_matrix = original_rig.matrix_world.copy()
            except Exception:
                original_rig_matrix = None
        try:
            log_runtime_event(
                "SPLIT state=BEFORE "
                f"layer='{getattr(layer, 'name', None)}' "
                f"driver={driver_type} "
                f"driver_name='{driver_name}' "
                f"source='{getattr(source_obj, 'name', None)}' "
                f"rig='{getattr(rig_obj, 'name', None)}' "
                f" selection={selection_mode} crop_mode={crop_mode} crop_enabled={crop_enabled} shape={shape} "
                f" glsl_mat=({glsl_mat}) "
                f" parent='{src_parent_name}' parent_mat=({src_parent_mat}) "
                f"src_mat=({_format_matrix_short(source_obj.matrix_world) if source_obj else 'none'}) "
                f"rig_mat=({_format_matrix_short(rig_obj.matrix_world) if rig_obj else 'none'})"
            )
        except Exception:
            pass
                                                                         
                                                                           
                                                         
        try:
            driver_obj = getattr(layer, "transform_object", None)
            if driver_obj and source_obj and driver_obj is not source_obj:
                sync_matrix_between_objects(driver_obj, source_obj, context)
        except Exception:
            pass

        if src_pos is None or len(src_pos) == 0:
            self.report({'WARNING'}, "No point cloud data to split")
            return {'CANCELLED'}

        try:
                                                                                          
            try:
                log_points_snapshot(
                    f"SPLIT BEFORE layer='{getattr(layer, 'name', None)}'",
                    src_pos,
                    matrix=get_layer_transform_matrix(layer),
                )
            except Exception:
                pass
            if props.screen_edit_enable:
                sel_mask = data.get("selection_mask") if data else None
                                                                     
                                                                     
                if (sel_mask is None or not np.any(sel_mask)):
                    candidates = get_screen_target_layers(context, primary_layer)
                    for cand in candidates:
                        cand_data = get_layer_data(cand)
                        cand_mask = cand_data.get("selection_mask") if cand_data else None
                        if cand_mask is not None and np.any(cand_mask):
                            layer = cand
                            data = cand_data
                            src_pos = data.get("source_pos")
                            src_col = data.get("source_col")
                            source_obj = get_layer_source_object(layer, scene)
                            sel_mask = cand_mask
                            break
                if sel_mask is None or not np.any(sel_mask):
                    self.report({'WARNING'}, "No screen selection")
                    return {'CANCELLED'}
                inside_points = src_pos[sel_mask]
                outside_points = src_pos[~sel_mask]
                inside_colors = src_col[sel_mask] if src_col is not None else None
                outside_colors = src_col[~sel_mask] if src_col is not None else None
            else:
                settings = get_sectionbox_settings(context)
                mask = compute_crop_mask(settings, layer=layer)
                if mask is None:
                    self.report({'WARNING'}, "Enable the cropbox to split points")
                    return {'CANCELLED'}
                inside_points = src_pos[mask]
                outside_points = src_pos[~mask]
                inside_colors = src_col[mask] if src_col is not None else None
                outside_colors = src_col[~mask] if src_col is not None else None

            created = []
            outside_has = len(outside_points) > 0
            inside_has = len(inside_points) > 0
            target_points = outside_points if outside_has else inside_points
            target_colors = outside_colors if outside_has else inside_colors

            if target_points is None or len(target_points) == 0:
                self.report({'WARNING'}, "Split produced no geometry")
                return {'CANCELLED'}

                                                                      
                                                                    
            update_point_cloud_data(target_points, target_colors, None, props, layer, keep_driver_state=True)
            update_mesh_from_points(source_obj, target_points, target_colors)
            clear_layer_selection(layer)

            def _match_parent(obj, source):
                if not obj or not source:
                    return
                obj.parent = source.parent
                try:
                    obj.matrix_parent_inverse = source.matrix_parent_inverse.copy()
                except Exception:
                    pass
                obj.matrix_world = source.matrix_world.copy()
                source_colls = list(source.users_collection) if source.users_collection else []
                if not source_colls and source.id_data:
                    source_colls = [source.id_data.collection]
                for coll in source_colls:
                    if obj.name not in coll.objects:
                        coll.objects.link(obj)
                fallback = getattr(context, "collection", None)
                if fallback and fallback not in source_colls:
                    try:
                        if obj.name in fallback.objects:
                            fallback.objects.unlink(obj)
                    except Exception:
                        pass
                obj.select_set(False)

            if inside_has and target_points is outside_points:
                base_name = source_obj.name if source_obj else "PointCloud"
                clone = create_mesh_from_points(
                    context,
                    f"{base_name}_split",
                    inside_points,
                    inside_colors,
                    reference=source_obj,
                )
                _match_parent(clone, source_obj)
                                                                        
                                                                          
                try:
                    scene = context.scene
                    if scene and hasattr(scene, "pcv_layers"):
                        new_layer = scene.pcv_layers.add()
                        ensure_layer_uid(new_layer)
                        new_layer.name = clone.name
                        new_layer.filepath = clone.name
                        new_layer.show_source_mesh = getattr(layer, "show_source_mesh", False)
                                                                                 
                                                                                
                        update_point_cloud_data(
                            inside_points,
                            inside_colors,
                            None,
                            props,
                            new_layer,
                            keep_driver_state=True,
                        )
                        ensure_layer_transform_driver(new_layer, context=context, source_obj=clone)
                                                                     
                        new_layer.show_points = getattr(layer, "show_points", True)
                        refresh_layer_visibility(new_layer, context)
                except Exception:
                    pass
                                                                
                try:
                    clone_local = getattr(clone, "rotation_euler", None)
                    source_local = getattr(source_obj, "rotation_euler", None) if source_obj else None
                    if clone_local is not None:
                        clone_local_str = (
                            f"({clone_local.x:.3f},{clone_local.y:.3f},{clone_local.z:.3f})"
                        )
                    else:
                        clone_local_str = "none"
                    if source_local is not None:
                        source_local_str = (
                            f"({source_local.x:.3f},{source_local.y:.3f},{source_local.z:.3f})"
                        )
                    else:
                        source_local_str = "none"
                    clone_world = (
                        _format_matrix_short(clone.matrix_world)
                        if hasattr(clone, "matrix_world")
                        else "matrix=<unavailable>"
                    )
                    try:
                        clone_parent_inv = _format_matrix_short(clone.matrix_parent_inverse)
                    except Exception:
                        clone_parent_inv = "matrix=<unavailable>"
                    try:
                        source_parent_inv = (
                            _format_matrix_short(source_obj.matrix_parent_inverse)
                            if source_obj
                            else "none"
                        )
                    except Exception:
                        source_parent_inv = "matrix=<unavailable>"
                    log_runtime_event(
                        "SPLIT clone_rotation "
                        f"clone='{getattr(clone, 'name', None)}' "
                        f"clone_local_rot={clone_local_str} "
                        f"source_local_rot={source_local_str} "
                        f"clone_world=({clone_world}) "
                        f"clone_parent_inv=({clone_parent_inv}) "
                        f"source_parent_inv=({source_parent_inv})"
                    )
                except Exception:
                    pass
                created.append(clone)

                                                              
            try:
                data_after = get_layer_data(layer)
                src_pos_after = data_after.get("source_pos") if data_after else None
                if src_pos_after is not None:
                    log_points_snapshot(
                        f"SPLIT AFTER layer='{getattr(layer, 'name', None)}'",
                        src_pos_after,
                        matrix=get_layer_transform_matrix(layer),
                    )
            except Exception:
                pass

            bpy.ops.object.select_all(action='DESELECT')
            if source_obj:
                source_obj.select_set(True)
                context.view_layer.objects.active = source_obj
            for obj in created:
                obj.select_set(True)

                                                                         
                                                                     
                                                 
            restore_cropbox_state(settings, crop_state)
            request_redraw()
            self.report({'INFO'}, f"Split completed. Generated {len(created)} new objects")
            return {'FINISHED'}
        finally:
            try:
                try:
                    end_driver_obj = getattr(layer, "transform_object", None)
                    end_driver_name = getattr(end_driver_obj, "name", None)
                    end_driver_type = (
                        "SOURCE"
                        if end_driver_obj is source_obj
                        else "RIG"
                        if end_driver_obj is rig_obj
                        else "OTHER"
                    )
                    try:
                        glsl_matrix_after = get_layer_transform_matrix(layer)
                        end_glsl_mat = _format_matrix_short(glsl_matrix_after)
                    except Exception:
                        glsl_matrix_after = None
                        end_glsl_mat = "matrix=<unavailable>"
                    end_parent = getattr(source_obj, "parent", None) if source_obj else None
                    end_parent_name = getattr(end_parent, "name", None) if end_parent else None
                    try:
                        end_parent_matrix = end_parent.matrix_world.copy() if end_parent else None
                        end_parent_mat = _format_matrix_short(end_parent_matrix) if end_parent else "none"
                    except Exception:
                        end_parent_matrix = None
                        end_parent_mat = "matrix=<unavailable>"
                    delta_glsl = _format_rotation_delta(glsl_matrix_before, glsl_matrix_after)
                    delta_src = _format_rotation_delta(original_source_matrix, getattr(source_obj, "matrix_world", None))
                    delta_parent = _format_rotation_delta(src_parent_matrix_before, end_parent_matrix)
                    delta_rig = _format_rotation_delta(original_rig_matrix, getattr(rig_obj, "matrix_world", None))
                    log_runtime_event(
                        "SPLIT state=AFTER "
                        f"layer='{getattr(layer, 'name', None)}' "
                        f"driver={end_driver_type} "
                        f"driver_name='{end_driver_name}' "
                        f"source='{getattr(source_obj, 'name', None)}' "
                        f"rig='{getattr(rig_obj, 'name', None)}' "
                        f" selection={selection_mode} crop_mode={crop_mode} crop_enabled={crop_enabled} shape={shape} "
                        f" glsl_mat=({end_glsl_mat}) "
                        f" parent='{end_parent_name}' parent_mat=({end_parent_mat}) "
                        f" delta_glsl={delta_glsl} delta_src={delta_src} delta_parent={delta_parent} delta_rig={delta_rig} "
                        f"src_mat=({_format_matrix_short(source_obj.matrix_world) if source_obj else 'none'}) "
                        f"rig_mat=({_format_matrix_short(rig_obj.matrix_world) if rig_obj else 'none'})"
                    )
                except Exception:
                    pass
                request_redraw()
            except Exception:
                pass

class PCV_OT_CutPoints(bpy.types.Operator):
    bl_idname = "pcv.cut_points"
    bl_label = "Cut"
    bl_description = "Cut points based on current crop mode"

    def execute(self, context):
        layer = get_active_layer(context)
        if not layer:
            self.report({'WARNING'}, "No active layer")
            return {'CANCELLED'}
        props = context.scene.pcv_props
        data = get_layer_data(layer)
        src_pos = data.get("source_pos")
        src_col = data.get("source_col")
        settings = get_sectionbox_settings(context)
        crop_state = capture_cropbox_state(settings)
        scene = context.scene
        source_obj = get_layer_source_object(layer, scene)
                                                                  
        original_driver = getattr(layer, "transform_object", None)
        original_rig = getattr(layer, "rig_object", None)
        driver_obj = original_driver
        driver_name = getattr(driver_obj, "name", None)
        rig_obj = original_rig
        driver_type = "SOURCE" if driver_obj is source_obj else "RIG" if driver_obj is rig_obj else "OTHER"
        selection_mode = "SCREEN" if getattr(props, "screen_edit_enable", False) else "CROP"
        crop_mode = getattr(props, "crop_mode", "INSIDE")
        crop_enabled = int(getattr(props, "crop_enable", False))
        shape = getattr(settings, "shape", "NONE") if settings else "NONE"
                                                         
        try:
            glsl_matrix_before = get_layer_transform_matrix(layer).copy()
            glsl_mat = _format_matrix_short(glsl_matrix_before)
        except Exception:
            glsl_matrix_before = None
            glsl_mat = "matrix=<unavailable>"
        src_parent = getattr(source_obj, "parent", None) if source_obj else None
        src_parent_name = getattr(src_parent, "name", None) if src_parent else None
        try:
            src_parent_matrix_before = src_parent.matrix_world.copy() if src_parent else None
            src_parent_mat = _format_matrix_short(src_parent_matrix_before) if src_parent else "none"
        except Exception:
            src_parent_matrix_before = None
            src_parent_mat = "matrix=<unavailable>"
        original_source_matrix = None
        original_rig_matrix = None
        if source_obj and hasattr(source_obj, "matrix_world"):
            try:
                original_source_matrix = source_obj.matrix_world.copy()
            except Exception:
                original_source_matrix = None
        if original_rig and hasattr(original_rig, "matrix_world"):
            try:
                original_rig_matrix = original_rig.matrix_world.copy()
            except Exception:
                original_rig_matrix = None
        try:
            log_runtime_event(
                "CUT state=BEFORE "
                f"layer='{getattr(layer, 'name', None)}' "
                f"driver={driver_type} "
                f"driver_name='{driver_name}' "
                f"source='{getattr(source_obj, 'name', None)}' "
                f"rig='{getattr(rig_obj, 'name', None)}' "
                f" selection={selection_mode} crop_mode={crop_mode} crop_enabled={crop_enabled} shape={shape} "
                f" glsl_mat=({glsl_mat}) "
                f" parent='{src_parent_name}' parent_mat=({src_parent_mat}) "
                f"src_mat=({_format_matrix_short(source_obj.matrix_world) if source_obj else 'none'}) "
                f"rig_mat=({_format_matrix_short(rig_obj.matrix_world) if rig_obj else 'none'})"
            )
        except Exception:
            pass
                                                                             
                                                               
        try:
            driver_obj = getattr(layer, "transform_object", None)
            if driver_obj and source_obj and driver_obj is not source_obj:
                sync_matrix_between_objects(driver_obj, source_obj, context)
        except Exception:
            pass

        if src_pos is None or len(src_pos) == 0:
            self.report({'WARNING'}, "No point cloud data to edit")
            return {'CANCELLED'}

        try:
                                                                                        
            try:
                log_points_snapshot(
                    f"CUT BEFORE layer='{getattr(layer, 'name', None)}'",
                    src_pos,
                    matrix=get_layer_transform_matrix(layer),
                )
            except Exception:
                pass
            if props.screen_edit_enable:
                sel_mask = data.get("selection_mask")
                if sel_mask is None or not np.any(sel_mask):
                    self.report({'WARNING'}, "No screen selection to cut")
                    return {'CANCELLED'}
                keep_mask = np.logical_not(sel_mask)
            else:
                settings = get_sectionbox_settings(context)
                mask = compute_crop_mask(settings, layer=layer)
                if mask is None:
                    self.report({'WARNING'}, "Enable the cropbox before deleting points")
                    return {'CANCELLED'}
                                                               
                           
                                                                                                       
                                                                                                        
                inside_mode = (props.crop_mode == 'INSIDE')
                keep_mask = mask if inside_mode else (~mask)

            keep_count = int(np.count_nonzero(keep_mask))
            if keep_count == 0:
                self.report({'WARNING'}, "Delete would remove all points")
                return {'CANCELLED'}
            new_pos = src_pos[keep_mask]
            new_col = src_col[keep_mask] if src_col is not None else None
                                                                               
            if update_point_cloud_data(new_pos, new_col, None, props, layer, keep_driver_state=True):
                clear_layer_selection(layer)
                source_obj = get_layer_source_object(layer, context.scene)
                update_mesh_from_points(source_obj, new_pos, new_col)
                                                                
                try:
                    data_after = get_layer_data(layer)
                    src_pos_after = data_after.get("source_pos") if data_after else None
                    if src_pos_after is not None:
                        log_points_snapshot(
                            f"CUT AFTER layer='{getattr(layer, 'name', None)}'",
                            src_pos_after,
                            matrix=get_layer_transform_matrix(layer),
                        )
                except Exception:
                    pass
                restore_cropbox_state(settings, crop_state)
                request_redraw()
                self.report({'INFO'}, f"Remaining points: {keep_count:,}")
                return {'FINISHED'}
            return {'CANCELLED'}
        finally:
            try:
                try:
                    end_driver_obj = getattr(layer, "transform_object", None)
                    end_driver_name = getattr(end_driver_obj, "name", None)
                    end_driver_type = (
                        "SOURCE"
                        if end_driver_obj is source_obj
                        else "RIG"
                        if end_driver_obj is rig_obj
                        else "OTHER"
                    )
                    try:
                        glsl_matrix_after = get_layer_transform_matrix(layer)
                        end_glsl_mat = _format_matrix_short(glsl_matrix_after)
                    except Exception:
                        glsl_matrix_after = None
                        end_glsl_mat = "matrix=<unavailable>"
                    end_parent = getattr(source_obj, "parent", None) if source_obj else None
                    end_parent_name = getattr(end_parent, "name", None) if end_parent else None
                    try:
                        end_parent_matrix = end_parent.matrix_world.copy() if end_parent else None
                        end_parent_mat = _format_matrix_short(end_parent_matrix) if end_parent else "none"
                    except Exception:
                        end_parent_matrix = None
                        end_parent_mat = "matrix=<unavailable>"
                    delta_glsl = _format_rotation_delta(glsl_matrix_before, glsl_matrix_after)
                    delta_src = _format_rotation_delta(original_source_matrix, getattr(source_obj, "matrix_world", None))
                    delta_parent = _format_rotation_delta(src_parent_matrix_before, end_parent_matrix)
                    delta_rig = _format_rotation_delta(original_rig_matrix, getattr(rig_obj, "matrix_world", None))
                    log_runtime_event(
                        "CUT state=AFTER "
                        f"layer='{getattr(layer, 'name', None)}' "
                        f"driver={end_driver_type} "
                        f"driver_name='{end_driver_name}' "
                        f"source='{getattr(source_obj, 'name', None)}' "
                        f"rig='{getattr(rig_obj, 'name', None)}' "
                        f" selection={selection_mode} crop_mode={crop_mode} crop_enabled={crop_enabled} shape={shape} "
                        f" glsl_mat=({end_glsl_mat}) "
                        f" parent='{end_parent_name}' parent_mat=({end_parent_mat}) "
                        f" delta_glsl={delta_glsl} delta_src={delta_src} delta_parent={delta_parent} delta_rig={delta_rig} "
                        f"src_mat=({_format_matrix_short(source_obj.matrix_world) if source_obj else 'none'}) "
                        f"rig_mat=({_format_matrix_short(rig_obj.matrix_world) if rig_obj else 'none'})"
                    )
                except Exception:
                    pass
                request_redraw()
            except Exception:
                pass


class PCV_OT_ToggleScreenSelection(bpy.types.Operator):
    bl_idname = "pcv.toggle_screen_selection"
    bl_label = "Screen"

    def execute(self, context):
        props = context.scene.pcv_props
        if not props:
            return {'CANCELLED'}
        new_state = not props.screen_edit_enable
        if new_state:
            ok = ensure_screen_selection_mode(context, props.screen_tool, source='AUTO')
            if not ok:
                self.report({'WARNING'}, "Add a point layer to use Screen Selection")
                return {'CANCELLED'}
        else:
            props.screen_edit_enable = False
            stop_screen_selection_operator(context, reason="toggle_off")
            activate_view_tool(context, "builtin.select_box")
        log_screen_event("toggle_screen", enabled=new_state, tool=props.screen_tool)
        return {'FINISHED'}


class PCV_OT_SetScreenTool(bpy.types.Operator):
    bl_idname = "pcv.set_screen_tool"
    bl_label = "Set Screen Tool"
    mode: bpy.props.EnumProperty(name="Tool", items=SCREEN_TOOL_ITEMS, default='BOX')

    def execute(self, context):
        if not ensure_screen_selection_mode(context, self.mode, source='BUTTON', force=True):
            self.report({'WARNING'}, "Add a point layer to use Screen Selection")
            return {'CANCELLED'}
        props = getattr(context.scene, "pcv_props", None)
                                                                            
                                                                          
        active = PCV_OT_ScreenSelect._active_instance
        if active and active._mode != self.mode:
            active._mode = self.mode
            active._reset_gesture()
            active._painting = False
            active._stroke_has_any = False
            area = active._area or getattr(context, "area", None)
            if area:
                area.tag_redraw()
                                                                             
                                                                            
                                                                     
        elif (props and props.screen_edit_enable) and not active:
            try:
                bpy.ops.pcv.screen_select('INVOKE_DEFAULT', mode=self.mode, source='BUTTON')
            except Exception:
                pass
        return {'FINISHED'}


class PCV_OT_ToggleLayerShow(bpy.types.Operator):
    bl_idname = "pcv.toggle_layer_show"
    bl_label = "Toggle Layer Visibility"
    bl_options = {'INTERNAL'}

    layer_index: bpy.props.IntProperty()

    @classmethod
    def poll(cls, context):
        return bool(get_active_layer(context))

    def execute(self, context):
        scene = context.scene
        if not scene or self.layer_index < 0 or self.layer_index >= len(scene.pcv_layers):
            return {'CANCELLED'}
        layer = scene.pcv_layers[self.layer_index]
        layer.show_points = not layer.show_points
        log_runtime_event(f"layer_show_toggle enabled={layer.show_points}")
        refresh_layer_visibility(layer, context)
        return {'FINISHED'}


class PCV_OT_ToggleLayerSource(bpy.types.Operator):
    bl_idname = "pcv.toggle_layer_source"
    bl_label = "Toggle Source Visibility"
    bl_options = {'INTERNAL'}

    layer_index: bpy.props.IntProperty()

    @classmethod
    def poll(cls, context):
        return bool(get_active_layer(context))

    def execute(self, context):
        scene = context.scene
        if not scene or self.layer_index < 0 or self.layer_index >= len(scene.pcv_layers):
            return {'CANCELLED'}
        layer = scene.pcv_layers[self.layer_index]
        layer.show_source_mesh = not layer.show_source_mesh
        log_runtime_event(f"layer_source_toggle enabled={layer.show_source_mesh}")
        refresh_layer_visibility(layer, context)
        return {'FINISHED'}


class PCV_OT_ScreenSelect(bpy.types.Operator):
    bl_idname = "pcv.screen_select"
    bl_label = "Screen Selection Tool"
    bl_options = {'REGISTER', 'UNDO'}

    mode: bpy.props.EnumProperty(
        name="Tool",
        items=[
            ('BOX', "Box", ""),
            ('CIRCLE', "Circle", ""),
            ('LASSO', "Lasso", ""),
        ],
        default='BOX',
    )
    source: bpy.props.EnumProperty(
        name="Source",
        items=[('MANUAL', "Manual", ""), ('AUTO', "Auto", ""), ('BUTTON', "Button", "")],
        default='MANUAL',
        options={'SKIP_SAVE'},
    )

    _start = None
    _current = None
    _lasso = None
    _handler = None
    _area = None
    _region = None
    _rv3d = None
    _mode = 'BOX'
    _selection_mode = 'REPLACE'
    _awaiting_press = True
    _painting = False
    _circle_radius = 25.0
    _stroke_has_any = False
    _nav_active = False
    _await_viewport_entry = True
    _force_exit = False
    _dragging = False
    _gesture_id = None
    _active_instance = None
    _window = None
    STATE_IDLE = "IDLE"
    STATE_GESTURE = "GESTURE"
    _state = STATE_IDLE
    _last_brush_pos = None
    _selection_history = None

    @classmethod
    def poll(cls, context):
        props = getattr(context.scene, "pcv_props", None)
        layer = get_active_layer(context)
        return bool(props and props.screen_edit_enable and layer)

    def invoke(self, context, event):
        props = getattr(context.scene, "pcv_props", None)
        active = PCV_OT_ScreenSelect._active_instance
        if active:
            active._force_exit = True
            if active._area:
                active._area.tag_redraw()
        if context.space_data.type != 'VIEW_3D':
            window, area, region = _find_view3d_area_region(context)
        else:
            window = context.window
            area = context.area
            region = context.region
        if not area or not region:
            self.report({'WARNING'}, "Run inside a 3D Viewport region")
            return {'CANCELLED'}
        if props:
            if props.screen_tool != self.mode:
                with suspend_screen_tool_update():
                    props.screen_tool = self.mode
            self._circle_radius = float(props.screen_circle_radius)
        self._area = area
        self._window = window
        if region.type != 'WINDOW':
            region = next((r for r in area.regions if r.type == 'WINDOW'), None)
        self._region = region
        if area == context.area and region == context.region:
            rv3d = context.region_data or getattr(context.space_data, "region_3d", None)
        else:
            rv3d = None
            for space in area.spaces:
                if space.type == 'VIEW_3D':
                    rv3d = space.region_3d
                    break
        self._rv3d = rv3d
        if self._region is None or self._rv3d is None:
            self.report({'WARNING'}, "Viewport region data unavailable")
            return {'CANCELLED'}
        self._mode = props.screen_tool if props and props.screen_tool in {'BOX', 'CIRCLE', 'LASSO'} else self.mode
        self._selection_mode = self._determine_mode(event)
        self._start = None
        self._current = None
        self._lasso = []
        self._awaiting_press = True
        self._painting = False
        self._stroke_has_any = False
        self._nav_active = False
        self._await_viewport_entry = False
        self._force_exit = False
        self._dragging = False
        self._gesture_id = None
        self._state = self.STATE_IDLE
        layer = get_active_layer(context)
        log_screen_event(
            "invoke",
            tool=self._mode,
            source=self.source,
            layer=getattr(layer, "name", None),
            area=getattr(self._area, "type", None),
        )
        self._handler = bpy.types.SpaceView3D.draw_handler_add(self._draw_overlay, (context,), 'WINDOW', 'POST_PIXEL')
        override_kwargs = {"area": self._area, "region": self._region}
        if window:
            override_kwargs["window"] = window
            override_kwargs["screen"] = window.screen
        base_ctx = getattr(bpy, "context", None)
        try:
            if base_ctx:
                with base_ctx.temp_override(**override_kwargs):
                    context.window_manager.modal_handler_add(self)
            else:
                context.window_manager.modal_handler_add(self)
        except Exception:
            context.window_manager.modal_handler_add(self)
        if event.type == 'LEFTMOUSE' and event.value == 'PRESS':
            coords = self._region_coords(event)
            if coords is not None:
                self._begin_gesture(context, coords, event)
        PCV_OT_ScreenSelect._active_instance = self
        return {'RUNNING_MODAL'}

    def modal(self, context, event):
        props = getattr(context.scene, "pcv_props", None)
        if not props or not props.screen_edit_enable:
            self.cancel(context)
            return {'CANCELLED'}
        if self._force_exit:
            log_screen_event("force_exit", tool=self._mode, reason="external")
            self.cancel(context)
            return {'CANCELLED'}
        if props.screen_tool != self._mode and props.screen_tool in {'BOX', 'CIRCLE', 'LASSO'}:
            self._mode = props.screen_tool
            self._reset_gesture()
            self._painting = False
            self._stroke_has_any = False
        if self._mode == 'CIRCLE':
            self._circle_radius = float(props.screen_circle_radius)
        if self._handle_navigation_passthrough(event):
            return {'PASS_THROUGH'}
        if event.type in {'MOUSEMOVE', 'INBETWEEN_MOUSEMOVE'}:
            coords = self._region_coords(event)
            if coords is None:
                log_screen_event("mouse_move_skip", reason="no_coords", evt=event.type)
                return {'RUNNING_MODAL'}
            self._current = coords
            active = self._state != self.STATE_IDLE
            if self._mode == 'CIRCLE' or active:
                area = self._area or getattr(context, "area", None)
                if area:
                    area.tag_redraw()
            if not active:
                return {'PASS_THROUGH'}
            log_screen_event("mouse_move", x=float(coords[0]), y=float(coords[1]))
            if self._mode == 'LASSO':
                if not self._lasso or math.dist(self._lasso[-1], self._current) > 2.0:
                    self._lasso.append(self._current)
                    self._dragging = True
            elif self._mode == 'CIRCLE' and self._painting:
                                                                  
                                                                   
                last = self._last_brush_pos
                if last is None or math.dist(last, self._current) > 1.0:
                    self._apply_circle_brush_step(context)
                    self._last_brush_pos = self._current
                    self._dragging = True
            elif self._mode == 'BOX' and self._start is not None:
                if not self._dragging and math.dist(self._start, self._current) > 2.0:
                    self._dragging = True
            return {'RUNNING_MODAL'}
        if self._mode == 'CIRCLE' and event.ctrl and event.type in {'WHEELUPMOUSE', 'WHEELDOWNMOUSE'}:
            delta = 2.0 if event.type == 'WHEELUPMOUSE' else -2.0
            self._adjust_circle_radius(context, delta)
            return {'RUNNING_MODAL'}
                                                                         
                                                                      
        if event.type == 'LEFTMOUSE':
            coords = self._region_coords(event)
            if coords is None:
                log_screen_event("mouse_press_skip" if event.value == 'PRESS' else "mouse_release_skip", reason="no_coords", evt=event.type)
                if event.value == 'RELEASE':
                    self._cleanup_modal(context)
                    return {'FINISHED'}
                return {'RUNNING_MODAL'}
            if event.value == 'PRESS':
                if self._begin_gesture(context, coords, event):
                    return {'RUNNING_MODAL'}
                return {'PASS_THROUGH'}
            if event.value == 'RELEASE':
                if self._state == self.STATE_IDLE:
                    self._cleanup_modal(context)
                    return {'FINISHED'}
                self._current = coords
                if self._mode == 'CIRCLE':
                    had_stroke = self._stroke_has_any
                    self._painting = False
                    self._stroke_has_any = False
                    self._dragging = False
                    layer = get_active_layer(context)
                    _, count = get_layer_selection(layer)
                    log_screen_event(
                        "circle_release",
                        tool=self._mode,
                        success=bool(had_stroke),
                        gid=self._gesture_id,
                        count=count,
                    )
                    self._gesture_id = None
                    self._awaiting_press = True
                    self._state = self.STATE_IDLE
                    self._last_brush_pos = None
                    area = self._area or getattr(context, "area", None)
                    if area:
                        area.tag_redraw()
                    log_screen_event("cycle_end", tool=self._mode, auto_restart=True)
                                                                        
                                                                    
                    return {'RUNNING_MODAL'}
                else:
                    if self._dragging:
                        applied = self._apply_drag_selection(context)
                        log_screen_event(
                            "gesture_release",
                            tool=self._mode,
                            dragging=True,
                            applied=bool(applied),
                            gid=self._gesture_id,
                        )
                    else:
                        log_screen_event(
                            "gesture_release",
                            tool=self._mode,
                            dragging=False,
                            gid=self._gesture_id,
                        )
                    self._reset_gesture()
                    self._gesture_id = None
                    self._dragging = False
                    area = self._area or getattr(context, "area", None)
                    if area:
                        area.tag_redraw()
                    log_screen_event("cycle_end", tool=self._mode, auto_restart=False)
                    self._cleanup_modal(context)
                    return {'FINISHED'}
            return {'RUNNING_MODAL'}
        if event.type in {'RIGHTMOUSE', 'ESC'}:
            self.cancel(context)
            return {'CANCELLED'}
        return {'PASS_THROUGH'}

    def cancel(self, context):
        log_screen_event(
            "cancel",
            tool=self._mode,
            force=int(self._force_exit),
            gesture=self._gesture_id,
        )
        self._gesture_id = None
        self._cleanup_modal(context)

    def _cleanup_modal(self, context):
        if PCV_OT_ScreenSelect._active_instance is self:
            PCV_OT_ScreenSelect._active_instance = None
        self._remove_handler()
        area = self._area or getattr(context, "area", None)
        if area:
            area.tag_redraw()
        self._state = self.STATE_IDLE

    def _determine_mode(self, event):
        shift = getattr(event, "shift", False)
        ctrl = getattr(event, "ctrl", False)
        if shift and ctrl:
            return 'XOR'
        if shift:
            return 'ADD'
        if ctrl:
            return 'SUB'
        return 'REPLACE'

    def _begin_gesture(self, context, coords, event):
        self._selection_mode = self._determine_mode(event)
        self._dragging = False
        self._gesture_id = _next_screen_gesture_id()
        self._state = self.STATE_GESTURE
        self._awaiting_press = False
        log_screen_event(
            "gesture_press",
            tool=self._mode,
            mode=self._selection_mode,
            source=self.source,
            x=float(coords[0]),
            y=float(coords[1]),
            gid=self._gesture_id,
        )
        if self._mode == 'CIRCLE':
            self._painting = True
            self._stroke_has_any = False
            if self._current is None:
                self._current = coords
            self._apply_circle_brush_step(context)
            self._last_brush_pos = self._current
            self._dragging = True
        else:
            self._start = coords
            self._current = coords
            self._lasso = [self._start] if self._mode == 'LASSO' else []
        area = self._area or getattr(context, "area", None)
        if area:
            area.tag_redraw()
        return True

    def _apply_drag_selection(self, context):
        layer = get_active_layer(context)
        if not layer or self._start is None or self._current is None:
            return False
        if self._mode == 'LASSO':
            lasso = list(self._lasso or [])
            if len(lasso) < 3:
                return False
            applied = apply_screen_selection(context, layer, self._start, self._current, self._selection_mode, shape='LASSO', lasso=lasso)
        else:
            dx = abs(self._current[0] - self._start[0])
            dy = abs(self._current[1] - self._start[1])
            if math.hypot(dx, dy) < MIN_SCREEN_DRAG:
                log_screen_event("gesture_small", tool=self._mode, mode=self._selection_mode)
                return False
            applied = apply_screen_selection(context, layer, self._start, self._current, self._selection_mode, shape='BOX')
        if applied:
            _, count = get_layer_selection(layer)
            self.report({'INFO'}, f"Selected {count:,} points")
            log_screen_event(
                "apply_result",
                tool=self._mode,
                mode=self._selection_mode,
                applied=True,
                count=count,
                gid=self._gesture_id,
            )
        else:
            log_screen_event(
                "apply_result",
                tool=self._mode,
                mode=self._selection_mode,
                applied=False,
                gid=self._gesture_id,
            )
        return applied

    def _circle_bounds(self):
        if self._current is None:
            return None, None
        radius = float(self._circle_radius)
        cx, cy = self._current
        start = (cx - radius, cy - radius)
        end = (cx + radius, cy + radius)
        return start, end

    def _apply_circle_brush_step(self, context):
        start, end = self._circle_bounds()
        if start is None or end is None:
            return False
        layer = get_active_layer(context)
        if not layer:
            return False
        mode = self._selection_mode
        if self._stroke_has_any and mode == 'REPLACE':
            mode = 'ADD'
        applied = apply_screen_selection(
            context,
            layer,
            start,
            end,
            mode,
            shape='CIRCLE',
            circle=(self._current, float(self._circle_radius)),
        )
        if applied:
            self._stroke_has_any = True
        return applied

    def _adjust_circle_radius(self, context, delta):
        props = getattr(context.scene, "pcv_props", None)
        if not props:
            return
        new_val = float(props.screen_circle_radius) + float(delta)
        new_val = max(3.0, min(500.0, new_val))
        props.screen_circle_radius = new_val
        self._circle_radius = new_val
        area = self._area or getattr(context, "area", None)
        if area:
            area.tag_redraw()

    def _reset_gesture(self):
        self._start = None
        self._current = None
        self._lasso = []
        self._awaiting_press = True
        self._stroke_has_any = False
        self._dragging = False
        self._state = self.STATE_IDLE
        self._last_brush_pos = None

    def _remove_handler(self):
        if self._handler is not None:
            bpy.types.SpaceView3D.draw_handler_remove(self._handler, 'WINDOW')
            self._handler = None

    def _draw_overlay(self, context):
        shader = gpu.shader.from_builtin('FLAT_COLOR')
        gpu.state.blend_set('ALPHA')
        color = (1.0, 1.0, 1.0, 0.5)
        coords = []
        if self._mode == 'CIRCLE':
            if self._current is None:
                gpu.state.blend_set('NONE')
                return
            center = self._current
            radius = float(self._circle_radius)
            steps = 48
            coords = [
                (
                    center[0] + math.cos(2 * math.pi * i / steps) * radius,
                    center[1] + math.sin(2 * math.pi * i / steps) * radius,
                )
                for i in range(steps)
            ]
        else:
            if self._start is None or self._current is None or self._awaiting_press:
                gpu.state.blend_set('NONE')
                return
            if self._mode == 'BOX':
                x1, y1 = self._start
                x2, y2 = self._current
                coords = [(x1, y1), (x1, y2), (x2, y2), (x2, y1)]
            elif self._mode == 'LASSO':
                coords = list(self._lasso or [])
        if not coords:
            gpu.state.blend_set('NONE')
            return
        dash_coords = []
        dash_len = 3.0

        def add_segment(a, b, buffer):
            ax, ay = a
            bx, by = b
            dx = bx - ax
            dy = by - ay
            dist = math.hypot(dx, dy)
            if dist <= 1e-5:
                return
            dirx = dx / dist
            diry = dy / dist
            pos = 0.0
            draw = True
            while pos < dist:
                next_pos = min(dist, pos + dash_len)
                if draw:
                    sx = ax + dirx * pos
                    sy = ay + diry * pos
                    ex = ax + dirx * next_pos
                    ey = ay + diry * next_pos
                    buffer.append((sx, sy))
                    buffer.append((ex, ey))
                draw = not draw
                pos = next_pos

        if len(coords) >= 2:
            for idx in range(len(coords) - 1):
                add_segment(coords[idx], coords[idx + 1], dash_coords)
                                                                        
                                                                          
                                                         
            if self._mode in {'BOX', 'CIRCLE', 'LASSO'} and len(coords) > 2:
                add_segment(coords[-1], coords[0], dash_coords)
        if not dash_coords:
            gpu.state.blend_set('NONE')
            return
        coords3d = [(x, y, 0.0) for x, y in dash_coords]
        color_attr = [color] * len(coords3d)
        batch = batch_for_shader(shader, 'LINES', {"pos": coords3d, "color": color_attr})
        shader.bind()
        batch.draw(shader)
        gpu.state.blend_set('NONE')

    def _handle_navigation_passthrough(self, event):
        if event.type in {'TRACKPADPAN', 'TRACKPADZOOM', 'NDOF_MOTION', 'NDOF_BUTTON'}:
            return True
        if self._mode != 'CIRCLE' and event.type in {'WHEELUPMOUSE', 'WHEELDOWNMOUSE'}:
            return True
        if event.type == 'MIDDLEMOUSE':
            if event.value == 'PRESS':
                self._nav_active = True
            elif event.value == 'RELEASE':
                self._nav_active = False
            return True
        if self._nav_active and event.type in {'MOUSEMOVE', 'INBETWEEN_MOUSEMOVE'}:
            return True
        return False

    def _region_coords(self, event):
        region = self._region
        if not region:
            log_screen_event(
                "viewport_coords_fail",
                reason="no_region",
                evt=getattr(event, "type", None),
            )
            return None
        rel_x = getattr(event, "mouse_region_x", None)
        rel_y = getattr(event, "mouse_region_y", None)
        if rel_x is not None and rel_y is not None and rel_x > -1e4 and rel_y > -1e4:
            return (rel_x, rel_y)
        abs_x = getattr(event, "mouse_x", None)
        abs_y = getattr(event, "mouse_y", None)
        if abs_x is None or abs_y is None:
            log_screen_event(
                "viewport_coords_fail",
                reason="no_absolute_coords",
                evt=getattr(event, "type", None),
            )
            return None
        return (abs_x - region.x, abs_y - region.y)

    def _event_in_viewport(self, event):
        if not self._region:
            log_screen_event(
                "viewport_check_fail",
                reason="no_region",
                evt=getattr(event, "type", None),
            )
            return False
        coords = self._region_coords(event)
        if coords is None:
            log_screen_event(
                "viewport_check_fail",
                reason="no_coords",
                evt=getattr(event, "type", None),
            )
            return False
        x, y = coords
        inside = 0 <= x <= self._region.width and 0 <= y <= self._region.height
        if not inside:
            log_screen_event(
                "viewport_check_fail",
                reason="outside_bounds",
                evt=getattr(event, "type", None),
                x=float(x),
                y=float(y),
                width=float(self._region.width),
                height=float(self._region.height),
            )
        return inside



class PCV_OT_ScreenSelectionInvert(bpy.types.Operator):
    bl_idname = "pcv.screen_selection_invert"
    bl_label = "Invert Selection"

    @classmethod
    def poll(cls, context):
        layer = get_active_layer(context)
        if not layer:
            return False
        mask, _ = get_layer_selection(layer)
        return mask is not None

    def execute(self, context):
        layer = get_active_layer(context)
        mask, _ = get_layer_selection(layer)
        if mask is None:
            self.report({'WARNING'}, "No selection to invert")
            return {'CANCELLED'}
        new_mask = np.logical_not(mask)
        set_layer_selection(layer, new_mask)
        self.report({'INFO'}, "Selection inverted")
        return {'FINISHED'}


def _find_view3d_area_region(context):
    ctx = context or bpy.context
    area = getattr(ctx, "area", None)
    region = None
    window = getattr(ctx, "window", None)
    if area and area.type == 'VIEW_3D':
        region = next((r for r in area.regions if r.type == 'WINDOW'), None)
    if area is None or region is None:
        wm = bpy.context.window_manager
        if wm:
            for win in wm.windows:
                screen = win.screen
                for candidate_area in screen.areas:
                    if candidate_area.type == 'VIEW_3D':
                        candidate_region = next((r for r in candidate_area.regions if r.type == 'WINDOW'), None)
                        if candidate_region:
                            return win, candidate_area, candidate_region
    return window, area, region


def stop_screen_selection_operator(context, reason="manual"):
    inst = PCV_OT_ScreenSelect._active_instance
    log_screen_event("stop_request", reason=reason, active=bool(inst))
    if inst:
        inst._force_exit = True
        ctx = context or bpy.context
        inst.cancel(ctx)
        request_redraw()


            

class PCV_PT_MainPanel(bpy.types.Panel):
    bl_label = "Point Cloud Essentials"
    bl_idname = "PCV_PT_MainPanel"
    bl_space_type = 'VIEW_3D'
    bl_region_type = 'UI'
    bl_category = 'PCE'

    def draw(self, context):
        layout = self.layout
        props = context.scene.pcv_props
        scene = context.scene
        layer = get_active_layer(context)
        settings = get_sectionbox_settings(context)

        col = layout.column(align=True)
        layers = list(scene.pcv_layers)
        has_layers = bool(layers)
        points_on = bool(
            (props and getattr(props, "lite_enabled", False))
            or any(getattr(l, "show_points", False) for l in layers)
        )
        sources_on = any(getattr(l, "show_source_mesh", False) for l in layers)
        row = col.row(align=True)
        point_icon = 'OUTLINER_OB_POINTCLOUD' if points_on else 'POINTCLOUD_DATA'
        point_text = "Colored"
        src_icon = 'SURFACE_NCIRCLE' if sources_on else 'CURVE_NCIRCLE'
        src_text = "Source"
        op = row.operator(
            "pcv.lite_toggle_points",
            text=point_text,
            icon=point_icon,
            depress=points_on,
        )
        op = row.operator(
            "pcv.lite_toggle_source",
            text=src_text,
            icon=src_icon,
            depress=sources_on,
        )

                                                
        if not has_layers:
            layout.separator()
            layout.label(text="Add a point-cloud mesh and enable Colored.", icon='INFO')
            return

        if not layer:
            layout.label(text="No active cloud layer.", icon='INFO')
            return

        layout.separator()
        tools_col = layout.column(align=True)
        tools_col.label(text="Edit Tools")
                                                                       
                                                                           
                                                                     
        tools_col.enabled = points_on
        row = tools_col.row(align=True)
        row.operator("pcv.toggle_cropbox", text="Cropbox", icon='MOD_BOOLEAN', depress=props.crop_enable)
        row.operator("pcv.toggle_screen_selection", text="Screen", icon='SELECT_SET', depress=props.screen_edit_enable)

        if props.crop_enable and settings:
            tools_col.separator()
            cbox = tools_col.box()
            cbox.label(text="Crop Container", icon='CUBE')
            row = cbox.row(align=True)
            op = row.operator("sectionbox.set_shape", text="Box", icon='MESH_CUBE', depress=(settings.shape == "BOX"))
            op.shape = "BOX"
            op = row.operator("sectionbox.set_shape", text="Cylinder", icon='MESH_CYLINDER', depress=(settings.shape == "CYLINDER"))
            op.shape = "CYLINDER"
            style_row = cbox.row(align=True)
            style_row.prop(settings, "display_style", expand=True)
            col = cbox.column(align=True)
            col.prop(settings, "hide_container", text="Hide Container")
            col.operator("pcv.crop_fit_selection", text="Fit to Selection", icon='SEQ_STRIP_META')

            tools_col.separator()
            tools_col.label(text="Crop Mode")
            row = tools_col.row(align=True)
            inside = row.operator("pcv.set_crop_mode", text="Inside", icon='SELECT_INTERSECT', depress=(props.crop_mode == 'INSIDE'))
            inside.mode = 'INSIDE'
            outside = row.operator("pcv.set_crop_mode", text="Outside", icon='SELECT_DIFFERENCE', depress=(props.crop_mode == 'OUTSIDE'))
            outside.mode = 'OUTSIDE'
            util_row = tools_col.row(align=True)
            util_row.operator("pcv.split_points", text="Split", icon='MOD_ARRAY')
            cut_label = "Cut Outside" if props.crop_mode == 'INSIDE' else "Cut Inside"
            util_row.operator("pcv.cut_points", text=cut_label, icon='TRASH')
        else:
            if props.screen_edit_enable:
                tools_col.separator()
                sel_box = tools_col.box()
                sel_box.label(text="Screen Selection", icon='STICKY_UVS_LOC')
                tool_row = sel_box.row(align=True)
                circle_icon_value = ToolSelectPanelHelper._icon_value_from_icon_handle("ops.generic.select_circle")
                lasso_icon_value = ToolSelectPanelHelper._icon_value_from_icon_handle("ops.generic.select_lasso")
                tool_defs = (
                    ('BOX', "Box", 'SELECT_SET', 0),
                    ('LASSO', "Lasso", 'TRACKING', lasso_icon_value),
                    ('CIRCLE', "Circle", 'OVERLAY', 0),
                )
                for mode, label, icon_name, icon_value in tool_defs:
                    op = tool_row.operator(
                        "pcv.set_screen_tool",
                        text=label,
                        icon=icon_name,
                        icon_value=icon_value,
                        depress=(props.screen_tool == mode and props.screen_edit_enable),
                    )
                    op.mode = mode
                invert_row = sel_box.row(align=True)
                invert_row.operator("pcv.screen_selection_invert", text="Invert Selection", icon='FILE_REFRESH')
                util_row = sel_box.row(align=True)
                util_row.operator("pcv.split_points", text="Split", icon='MOD_ARRAY')
                util_row.operator("pcv.cut_points", text="Cut", icon='TRASH')

class PCV_PT_CoordinatesPanel(bpy.types.Panel):
    bl_label = "Coordinates"
    bl_idname = "PCV_PT_CoordinatesPanel"
    bl_space_type = 'VIEW_3D'
    bl_region_type = 'UI'
    bl_category = 'PCE'
    bl_parent_id = "PCV_PT_MainPanel"
    bl_options = {'DEFAULT_CLOSED'}

    @classmethod
    def poll(cls, context):
        props = getattr(context.scene, "pcv_props", None)
        settings = get_sectionbox_settings(context)
        layer = get_active_layer(context)
        return bool(props and settings and props.crop_enable and layer)

    def draw(self, context):
        layout = self.layout
        scene = getattr(context, "scene", None)
        layers = list(getattr(scene, "pcv_layers", [])) if scene else []
        points_on = any(getattr(l, "show_points", False) for l in layers)
        layout.enabled = points_on
        settings = get_sectionbox_settings(context)
        if not settings:
            return
        box = layout.box()
        box.label(text="Center:")
        box.prop(settings, "center", text="")

        if settings.shape == "BOX":
            size_box = layout.box()
            size_box.label(text="Box Size (m):")
            size_box.prop(settings, "size", text="")
            size_box.prop(settings, "symmetric_box", text="Symmetric Scale")
        elif settings.shape == "CYLINDER":
            cyl_box = layout.box()
            cyl_box.label(text="Cylinder Dimensions:")
            cyl_box.prop(settings, "radius")
            cyl_box.prop(settings, "height")
            cyl_box.prop(settings, "symmetric_cylinder_height", text="Symmetric Height")
        else:
            layout.label(text="Choose a shape to edit", icon='INFO')


class PCV_PT_PointAppearancePanel(bpy.types.Panel):
    bl_label = "Point Appearance"
    bl_idname = "PCV_PT_PointAppearancePanel"
    bl_space_type = 'VIEW_3D'
    bl_region_type = 'UI'
    bl_category = 'PCE'
    bl_parent_id = "PCV_PT_MainPanel"
    bl_options = {'DEFAULT_CLOSED'}

    @classmethod
    def poll(cls, context):
        props = getattr(context.scene, "pcv_props", None)
        layer = get_active_layer(context)
        return bool(props and layer)

    def draw(self, context):
        layout = self.layout
        scene = context.scene
        layers = list(getattr(scene, "pcv_layers", [])) if scene else []
        points_on = any(getattr(l, "show_points", False) for l in layers)
        layout.enabled = points_on
        layer = get_active_layer(context)
        if not layer:
            layout.label(text="No point layer", icon='INFO')
            return
        layout.prop(layer, "point_size", slider=True)
        layout.prop(layer, "point_density", text="Point Density", slider=True)
        layout.prop(layer, "point_alpha", text="Point Transparency")


class PCV_PT_CropAppearancePanel(bpy.types.Panel):
    bl_label = "Crop Box Appearance"
    bl_idname = "PCV_PT_CropAppearancePanel"
    bl_space_type = 'VIEW_3D'
    bl_region_type = 'UI'
    bl_category = 'PCE'
    bl_parent_id = "PCV_PT_MainPanel"
    bl_options = {'DEFAULT_CLOSED'}

    @classmethod
    def poll(cls, context):
        props = getattr(context.scene, "pcv_props", None)
        settings = get_sectionbox_settings(context)
        layer = get_active_layer(context)
        return bool(props and settings and props.crop_enable and layer)

    def draw(self, context):
        layout = self.layout
        scene = getattr(context, "scene", None)
        layers = list(getattr(scene, "pcv_layers", [])) if scene else []
        points_on = any(getattr(l, "show_points", False) for l in layers)
        layout.enabled = points_on
        settings = get_sectionbox_settings(context)
        if not settings:
            layout.label(text="No cropbox")
            return
        layout.prop(settings, "color", text="Fill")
        layout.prop(settings, "edge_color", text="Edges")
        layout.prop(settings, "dot_color", text="Dots")
        layout.prop(settings, "alpha_solid")
        layout.prop(settings, "alpha_wire")
        layout.prop(settings, "gizmo_scale_scale", text="Dot Size")
        layout.prop(settings, "gizmo_scale_move", text="Move Size")


class PCV_PT_ScreenSelectionAppearancePanel(bpy.types.Panel):
    bl_label = "Screen Selection Appearance"
    bl_idname = "PCV_PT_ScreenSelectionAppearancePanel"
    bl_space_type = 'VIEW_3D'
    bl_region_type = 'UI'
    bl_category = 'PCE'
    bl_parent_id = "PCV_PT_MainPanel"
    bl_options = {'DEFAULT_CLOSED'}

    @classmethod
    def poll(cls, context):
        props = getattr(context.scene, "pcv_props", None)
        layer = get_active_layer(context)
        return bool(layer and props and props.screen_edit_enable and not props.crop_enable)

    def draw(self, context):
        layout = self.layout
        scene = getattr(context, "scene", None)
        layers = list(getattr(scene, "pcv_layers", [])) if scene else []
        points_on = any(getattr(l, "show_points", False) for l in layers)
        layout.enabled = points_on
        layer = get_active_layer(context)
        if not layer:
            layout.label(text="No point layer", icon='INFO')
            return
        layout.prop(layer, "selection_color", text="Color")
        layout.prop(layer, "selection_color_strength", text="Blend Strength")


class PCV_ScreenSelectTool(bpy.types.WorkSpaceTool):
    bl_space_type = 'VIEW_3D'
    bl_context_mode = 'OBJECT'
    bl_idname = "pcv.screen_select_tool"
    bl_label = "Point Cloud Select"
    bl_description = "Select points with box, circle, or lasso"
    bl_icon = "ops.generic.select_box"
    bl_keymap = (
        ("pcv.screen_select", {"type": 'LEFTMOUSE', "value": 'PRESS', "any": True}, None),
    )

    @classmethod
    def poll(cls, context):
        return bool(get_active_layer(context))

class PCV_PT_PointStatsPanel(bpy.types.Panel):
    bl_label = "Point Statistics"
    bl_idname = "PCV_PT_PointStatsPanel"
    bl_space_type = 'VIEW_3D'
    bl_region_type = 'UI'
    bl_category = 'PCE'
    bl_parent_id = "PCV_PT_MainPanel"
    bl_options = {'DEFAULT_CLOSED'}

    @classmethod
    def poll(cls, context):
        layer = get_active_layer(context)
        return bool(layer)

    def draw(self, context):
        layout = self.layout
        scene = getattr(context, "scene", None)
        layers = list(getattr(scene, "pcv_layers", [])) if scene else []
        points_on = any(getattr(l, "show_points", False) for l in layers)
        layout.enabled = points_on
        layer = get_active_layer(context)
        if not layer:
            layout.label(text="No active layer", icon='INFO')
            return
        count = get_layer_point_count(layer)
        box = layout.box()
        box.label(text=f"Points Rendered: {count:,}")


class PCV_OT_SetCropMode(bpy.types.Operator):
    bl_idname = "pcv.set_crop_mode"
    bl_label = "Set Crop Mode"
    mode: bpy.props.EnumProperty(
        name="Mode",
        items=[('INSIDE', "Inside", ""), ('OUTSIDE', "Outside", "")],
        default='INSIDE',
    )

    def execute(self, context):
        props = getattr(context.scene, "pcv_props", None)
        if not props:
            return {'CANCELLED'}
        if props.crop_mode != self.mode:
            props.crop_mode = self.mode
        return {'FINISHED'}


class PCV_OT_CropFitSelection(bpy.types.Operator):
    bl_idname = "pcv.crop_fit_selection"
    bl_label = "Fit Cropbox to Selection"
    bl_description = "Fit the crop container to the bounding box of selected objects"

    def execute(self, context):
        settings = get_sectionbox_settings(context)
        if not settings:
            self.report({'WARNING'}, "Cropbox settings unavailable")
            return {'CANCELLED'}
        selected = set()
        for root in context.selected_objects:
            if root.type == 'MESH':
                selected.add(root)
                                                                      
                                                                     
            for child in getattr(root, "children_recursive", []):
                if getattr(child, "type", None) == 'MESH':
                    selected.add(child)
        if not selected:
            self.report({'WARNING'}, "Select at least one mesh or controller with mesh children")
            return {'CANCELLED'}
        min_vec = Vector((float('inf'), float('inf'), float('inf')))
        max_vec = Vector((float('-inf'), float('-inf'), float('-inf')))
        for obj in selected:
            for corner in obj.bound_box:
                world = obj.matrix_world @ Vector(corner)
                min_vec = Vector((min(min_vec.x, world.x), min(min_vec.y, world.y), min(min_vec.z, world.z)))
                max_vec = Vector((max(max_vec.x, world.x), max(max_vec.y, world.y), max(max_vec.z, world.z)))
        if any(math.isinf(v) for v in min_vec) or any(math.isinf(v) for v in max_vec):
            self.report({'WARNING'}, "Failed to compute bounds")
            return {'CANCELLED'}
        center = (min_vec + max_vec) * 0.5
        size = max_vec - min_vec
        size = Vector((max(size.x, MIN_DIMENSION), max(size.y, MIN_DIMENSION), max(size.z, MIN_DIMENSION)))
        settings.center = (center.x, center.y, center.z)
        settings.size = (size.x, size.y, size.z)
        if settings.shape == "CYLINDER":
            settings.radius = max(size.x, size.y) * 0.5
            settings.height = size.z
        settings.auto_fit_done = True
        update_gizmo_view(settings, context)
        self.report({'INFO'}, "Cropbox fitted to selection")
        return {'FINISHED'}


class PCV_OT_LiteRebuildPoints(bpy.types.Operator):
    bl_idname = "pcv.lite_rebuild_points"
    bl_label = "Rebuild Colored (Experimental)"

    def execute(self, context):
        scene = context.scene
        ok = _lite_rebuild_scene_layers(scene, context, reason="operator")
        if not ok:
            self.report({'WARNING'}, "No point clouds found after rebuild")
            return {'CANCELLED'}
        layers = getattr(scene, "pcv_layers", [])
        self.report({'INFO'}, f"Rebuilt {len(layers)} point layers (exp)")
        return {'FINISHED'}


class PCV_OT_LiteTogglePoints(bpy.types.Operator):
    bl_idname = "pcv.lite_toggle_points"
    bl_label = "Toggle GLSL Points"
    action: bpy.props.EnumProperty(
        name="Action",
        items=[('AUTO', "Auto", ""), ('ON', "On", ""), ('OFF', "Off", "")],
        default='AUTO',
    )

    def execute(self, context):
        scene = context.scene
        props = getattr(scene, "pcv_props", None)

                                                                 
                                                                         
        if props and hasattr(props, "lite_enabled"):
            current = bool(props.lite_enabled)
        else:
            layers = getattr(scene, "pcv_layers", [])
            current = any(getattr(layer, "show_points", False) for layer in layers)

        if self.action == 'AUTO':
            target = not current
        elif self.action == 'ON':
            target = True
        else:
            target = False

        if target:
                                                                       
                                                                        
                                                                      
            has_layers = bool(getattr(scene, "pcv_layers", []))
            has_candidates = bool(detect_pointcloud_objects(scene))
            if not has_layers and not has_candidates:
                if props:
                    props.lite_enabled = True
                    props.enable_auto_lite = True
                return {'FINISHED'}
                                                                       
                                                                      
                                                                       
                                                 
            ok = _lite_rebuild_scene_layers(scene, context, reason="colored_toggle")
            if not ok:
                if props:
                    props.lite_enabled = False
                    props.enable_auto_lite = False
                self.report({'WARNING'}, "No point clouds")
                return {'CANCELLED'}
            if props:
                props.enable_auto_lite = True
                                                                         
                                                                           
                if getattr(props, "crop_enable", False):
                    settings = get_sectionbox_settings(context)
                    if settings is not None:
                        settings.show = True
                        ensure_sectionbox_ready(context)
                        update_gizmo_view(settings, context)
                                                                        
                                                                       
                                                            
                if getattr(props, "screen_edit_enable", False):
                    ensure_workspace_screen_tool(context, props.screen_tool)
        else:
                                                                         
                                                                            
                                                                    
                                                 
            layers = getattr(scene, "pcv_layers", [])
            for layer in layers:
                if getattr(layer, "show_points", False):
                    layer.show_points = False
                    refresh_layer_visibility(layer, context)
            update_draw_handler_for_scene(scene)
            request_redraw()
            if props:
                props.lite_enabled = False
                props.enable_auto_lite = False
                                                                     
                                                               
                settings = get_sectionbox_settings(context)
                if settings is not None and getattr(settings, "show", False):
                    settings.show = False
                    update_gizmo_view(settings, context)
                                                                       
                                                                        
                if getattr(props, "screen_edit_enable", False):
                    stop_screen_selection_operator(context, reason="colored_off")
                    activate_view_tool(context, "builtin.select_box")
        return {'FINISHED'}


class PCV_OT_LiteToggleSource(bpy.types.Operator):
    bl_idname = "pcv.lite_toggle_source"
    bl_label = "Toggle Source Visibility"
    action: bpy.props.EnumProperty(
        name="Action",
        items=[('AUTO', "Auto", ""), ('ON', "On", ""), ('OFF', "Off", "")],
        default='AUTO',
    )

    def execute(self, context):
        scene = context.scene
        layers = getattr(scene, "pcv_layers", [])
        if not len(layers):
            ensure_lite_layers(scene, context)
            layers = getattr(scene, "pcv_layers", [])
        if not len(layers):
            self.report({'WARNING'}, "No point clouds")
            return {'CANCELLED'}
        current = any(getattr(layer, "show_source_mesh", False) for layer in layers)
        if self.action == 'AUTO':
            target = not current
        elif self.action == 'ON':
            target = True
        else:
            target = False
        for layer in layers:
            if layer.show_source_mesh != target:
                layer.show_source_mesh = target
                refresh_layer_visibility(layer, context)
        return {'FINISHED'}

class PCV_OT_AdjustCircleRadius(bpy.types.Operator):
    bl_idname = "pcv.adjust_circle_radius"
    bl_label = "Adjust Circle Radius"
    delta: bpy.props.FloatProperty(default=2.0, options={'SKIP_SAVE'})

    @classmethod
    def poll(cls, context):
        props = getattr(context.scene, "pcv_props", None)
        return bool(props and props.screen_edit_enable and props.screen_tool == 'CIRCLE')

    def execute(self, context):
        props = context.scene.pcv_props
        new_val = float(props.screen_circle_radius) + float(self.delta)
        new_val = max(3.0, min(500.0, new_val))
        props.screen_circle_radius = new_val
        active = PCV_OT_ScreenSelect._active_instance
        if active and active._mode == 'CIRCLE':
            active._circle_radius = new_val
            area = active._area or getattr(context, "area", None)
            if area:
                area.tag_redraw()
        else:
            request_redraw()
        return {'FINISHED'}


class PCV_MT_PointsMenu(bpy.types.Menu):
    bl_label = "Points"
    bl_idname = "PCV_MT_points_menu"

    @classmethod
    def poll(cls, context):
        props = getattr(context.scene, "pcv_props", None)
        layer = get_active_layer(context)
        return bool(layer and props and (props.crop_enable or props.screen_edit_enable))

    def draw(self, context):
        layout = self.layout
        layout.operator("pcv.cut_points", text="Delete Points", icon='TRASH')
        layout.operator("pcv.split_points", text="Separate Points", icon='MOD_ARRAY')


class PCV_OT_ShowPointsMenu(bpy.types.Operator):
    bl_idname = "pcv.show_points_menu"
    bl_label = "Points Menu"

    @classmethod
    def poll(cls, context):
        props = getattr(context.scene, "pcv_props", None)
        layer = get_active_layer(context)
        return bool(layer and props and (props.crop_enable or props.screen_edit_enable))

    def invoke(self, context, event):
        context.window_manager.popup_menu_pie(self.draw_menu, title="Points")
        return {'FINISHED'}

    def draw_menu(self, _menu, context):
        layout = _menu.layout
        layout.operator("pcv.cut_points", text="Delete Points", icon='TRASH')
        layout.operator("pcv.split_points", text="Separate Points", icon='MOD_ARRAY')
class PCV_UL_LayerList(bpy.types.UIList):
    def draw_item(self, context, layout, data, item, icon, active_data, active_propname, index):
        if self.layout_type in {'DEFAULT', 'COMPACT'}:
            row = layout.row(align=True)
            name = row.row()
            name.prop(item, "name", text="", emboss=False)
            controls = row.row(align=True)
            show_icon = 'OUTLINER_OB_POINTCLOUD' if item.show_points else 'POINTCLOUD_DATA'
            show_btn = controls.operator(
                "pcv.toggle_layer_show",
                text="",
                icon=show_icon,
                emboss=False,
            )
            show_btn.layer_index = index
            src_icon = 'SURFACE_NCIRCLE' if item.show_source_mesh else 'CURVE_NCIRCLE'
            src_btn = controls.operator(
                "pcv.toggle_layer_source",
                text="",
                icon=src_icon,
                emboss=False,
                depress=item.show_source_mesh,
            )
            src_btn.layer_index = index
        elif self.layout_type in {'GRID'}:
            layout.alignment = 'CENTER'
            layout.label(text=str(index))

             
classes = (
    PCVCloudLayer,
    SectionBoxSettings,
    PCVPropertyGroup,
    SECTIONBOX_OT_toggle_show,
    SECTIONBOX_OT_set_shape,
    SECTIONBOX_OT_resize_face,
    SECTIONBOX_OT_move_center,
    SECTIONBOX_GGT_gizmos,
    PCV_OT_LoadPly,
    PCV_OT_RenderActive,
    PCV_OT_ToggleCropbox,
    PCV_OT_ToggleScreenSelection,
    PCV_OT_SetScreenTool,
    PCV_OT_ToggleLayerShow,
    PCV_OT_ToggleLayerSource,
    PCV_OT_ScreenSelect,
    PCV_OT_ScreenSelectionInvert,
    PCV_OT_AddLayer,
    PCV_OT_RemoveLayer,
    PCV_OT_SplitPoints,
    PCV_OT_CutPoints,
    PCV_UL_LayerList,
    PCV_PT_MainPanel,
    PCV_PT_CoordinatesPanel,
    PCV_PT_PointAppearancePanel,
    PCV_PT_CropAppearancePanel,
    PCV_PT_ScreenSelectionAppearancePanel,
    PCV_PT_PointStatsPanel,
    PCV_OT_LiteRebuildPoints,
    PCV_OT_LiteTogglePoints,
    PCV_OT_LiteToggleSource,
    PCV_OT_AdjustCircleRadius,
    PCV_OT_SetCropMode,
    PCV_OT_CropFitSelection,
    PCV_MT_PointsMenu,
    PCV_OT_ShowPointsMenu,
)

def register():
    reset_terminal_log()
    log_runtime_event("register() invoked")
    for cls in classes: bpy.utils.register_class(cls)
    try:
        bpy.utils.register_tool(PCV_ScreenSelectTool, after={"builtin.select_box"})
    except Exception:
        pass
    bpy.types.Scene.pcv_props = bpy.props.PointerProperty(type=PCVPropertyGroup)
    bpy.types.Scene.sectionbox_settings = bpy.props.PointerProperty(type=SectionBoxSettings)
    bpy.types.Scene.pcv_layers = bpy.props.CollectionProperty(type=PCVCloudLayer)
    bpy.types.Scene.pcv_layers_index = bpy.props.IntProperty(default=0, update=layer_index_update)
    ensure_sectionbox_shader()
    ensure_sectionbox_draw_handler()
    install_error_logger()
    wm = bpy.context.window_manager
    kc = wm.keyconfigs.addon if wm else None
    if kc:
        km = kc.keymaps.new(name='3D View', space_type='VIEW_3D')
        kmi = km.keymap_items.new("pcv.screen_selection_invert", type='I', value='PRESS', ctrl=True)
        _ADDON_KEYMAPS.append((km, kmi))
        kmi = km.keymap_items.new("pcv.adjust_circle_radius", type='WHEELUPMOUSE', value='PRESS', ctrl=True)
        kmi.properties.delta = 2.0
        _ADDON_KEYMAPS.append((km, kmi))
        kmi = km.keymap_items.new("pcv.adjust_circle_radius", type='WHEELDOWNMOUSE', value='PRESS', ctrl=True)
        kmi.properties.delta = -2.0
        _ADDON_KEYMAPS.append((km, kmi))
                                                                                     
        kmi = km.keymap_items.new("pcv.show_points_menu", type='X', value='PRESS')
        _ADDON_KEYMAPS.append((km, kmi))
    if on_load_post not in bpy.app.handlers.load_post:
        bpy.app.handlers.load_post.append(on_load_post)
    if on_depsgraph_update not in bpy.app.handlers.depsgraph_update_post:
        bpy.app.handlers.depsgraph_update_post.append(on_depsgraph_update)
    current_scene = getattr(bpy.context, "scene", None)
    if current_scene:
        refresh_scene_layers(current_scene)

def unregister():
    remove_pointcloud_draw_handler()
    remove_sectionbox_draw_handler()
    try:
        bpy.utils.unregister_tool(PCV_ScreenSelectTool)
    except Exception:
        pass
    wm = bpy.context.window_manager
    for km, kmi in _ADDON_KEYMAPS:
        if km and kmi:
            try:
                km.keymap_items.remove(kmi)
            except Exception:
                pass
    _ADDON_KEYMAPS.clear()
    for handler_list, handler in (
        (bpy.app.handlers.load_post, on_load_post),
        (bpy.app.handlers.depsgraph_update_post, on_depsgraph_update),
    ):
        if handler in handler_list:
            handler_list.remove(handler)
    del bpy.types.Scene.pcv_layers_index
    del bpy.types.Scene.pcv_layers
    del bpy.types.Scene.sectionbox_settings
    del bpy.types.Scene.pcv_props
    for cls in reversed(classes): bpy.utils.unregister_class(cls)

if __name__ == "__main__":
    register()
