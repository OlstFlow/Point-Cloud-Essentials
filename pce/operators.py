from .core import *                   

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


def fit_sectionbox_to_points(settings):
    if settings is None or _SOURCE_POS is None or len(_SOURCE_POS) == 0:
        return
    min_v = _SOURCE_POS.min(axis=0)
    max_v = _SOURCE_POS.max(axis=0)
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
        fit_sectionbox_to_points(settings)
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
    if reference and hasattr(reference, "matrix_world"):
        mat = reference.matrix_world.inverted()
        pts4 = np.hstack((pts, np.ones((len(pts), 1), dtype=np.float32)))
        pts = (pts4 @ np.array(mat).T)[:, :3]
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
    mat = None
    if hasattr(mesh_obj, "matrix_world"):
        try:
            mat = mesh_obj.matrix_world.inverted()
        except Exception:
            mat = None
    if mat is not None:
        pts4 = np.hstack((pts, np.ones((len(pts), 1), dtype=np.float32)))
        pts = (pts4 @ np.array(mat).T)[:, :3]
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

                                                                           
                           
                                                                           

def ensure_screen_selection_mode(context, mode=None, *, source='AUTO', force=False):
    ctx = context or getattr(bpy, "context", None)
    if not ctx or not getattr(ctx, "scene", None):
        return False
    props = getattr(ctx.scene, "pcv_props", None)
    if not props:
        return False
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
        if PCV_OT_ScreenSelect._active_instance:
            stop_screen_selection_operator(ctx, reason="shape_change")
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
                    fit_sectionbox_to_points(settings)
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
            obj.select_set(False)
            settings = get_sectionbox_settings(context)
            if settings and not settings.auto_fit_done:
                fit_sectionbox_to_points(settings)
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
        if src_pos is None or len(src_pos) == 0:
            self.report({'WARNING'}, "No point cloud data to split")
            return {'CANCELLED'}

        if props.screen_edit_enable:
            sel_mask = data.get("selection_mask")
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
            clone = create_mesh_from_points(context, f"{base_name}_split", inside_points, inside_colors, reference=source_obj)
            _match_parent(clone, source_obj)
            created.append(clone)

        bpy.ops.object.select_all(action='DESELECT')
        if source_obj:
            source_obj.select_set(True)
            context.view_layer.objects.active = source_obj
        for obj in created:
            obj.select_set(True)

        refresh_layer_visibility(layer, context)
        ensure_lite_layers(scene, context)
        refresh_scene_layers(scene)
        restore_cropbox_state(settings, crop_state)
        self.report({'INFO'}, f"Split completed. Generated {len(created)} new objects")
        return {'FINISHED'}


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
        if src_pos is None or len(src_pos) == 0:
            self.report({'WARNING'}, "No point cloud data to edit")
            return {'CANCELLED'}

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
            target_inside = props.crop_mode == 'INSIDE'
            keep_mask = (~mask) if target_inside else mask

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
            refresh_layer_visibility(layer, context)
            refresh_scene_layers(context.scene)
            restore_cropbox_state(settings, crop_state)
            self.report({'INFO'}, f"Remaining points: {keep_count:,}")
            return {'FINISHED'}
        return {'CANCELLED'}


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
        selected = [obj for obj in context.selected_objects if obj.type == 'MESH']
        if not selected:
            self.report({'WARNING'}, "Select at least one mesh object")
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
        layers = getattr(scene, "pcv_layers", [])
        if not layers:
            ensure_lite_layers(scene, context)
            layers = getattr(scene, "pcv_layers", [])
        if not layers:
            self.report({'WARNING'}, "No point clouds")
            return {'CANCELLED'}
        current = any(getattr(layer, "show_points", False) for layer in layers)
        if self.action == 'AUTO':
            target = not current
        elif self.action == 'ON':
            target = True
        else:
            target = False
        for layer in layers:
            layer.show_points = target
            refresh_layer_visibility(layer, context)
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
        if not layers:
            ensure_lite_layers(scene, context)
            layers = getattr(scene, "pcv_layers", [])
        if not layers:
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


OPERATOR_CLASSES = (
    SECTIONBOX_OT_toggle_show,
    SECTIONBOX_OT_set_shape,
    SECTIONBOX_OT_resize_face,
    SECTIONBOX_OT_move_center,
    SECTIONBOX_GGT_gizmos,
    PCV_OT_AddLayer,
    PCV_OT_RemoveLayer,
    PCV_OT_LoadPly,
    PCV_OT_RenderActive,
    PCV_OT_ToggleCropbox,
    PCV_OT_SplitPoints,
    PCV_OT_CutPoints,
    PCV_OT_ToggleScreenSelection,
    PCV_OT_SetScreenTool,
    PCV_OT_ToggleLayerShow,
    PCV_OT_ToggleLayerSource,
    PCV_OT_ScreenSelectionInvert,
    PCV_OT_SetCropMode,
    PCV_OT_CropFitSelection,
    PCV_OT_LiteTogglePoints,
    PCV_OT_LiteToggleSource,
    PCV_OT_AdjustCircleRadius,
)
