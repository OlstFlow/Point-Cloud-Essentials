from .core import *                   

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
        row = layout.row(align=True)
        row.label(text="Edit Tools")
        row = layout.row(align=True)
                                                                       
                                                      
        row.enabled = points_on
        row.operator("pcv.toggle_cropbox", text="Cropbox", icon='MOD_BOOLEAN', depress=props.crop_enable)
        row.operator("pcv.toggle_screen_selection", text="Screen", icon='SELECT_SET', depress=props.screen_edit_enable)

        if props.crop_enable and settings:
            layout.separator()
            cbox = layout.box()
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

            layout.separator()
            layout.label(text="Crop Mode")
            row = layout.row(align=True)
            inside = row.operator("pcv.set_crop_mode", text="Inside", icon='SELECT_INTERSECT', depress=(props.crop_mode == 'INSIDE'))
            inside.mode = 'INSIDE'
            outside = row.operator("pcv.set_crop_mode", text="Outside", icon='SELECT_DIFFERENCE', depress=(props.crop_mode == 'OUTSIDE'))
            outside.mode = 'OUTSIDE'
            util_row = layout.row(align=True)
            util_row.operator("pcv.split_points", text="Split", icon='MOD_ARRAY')
            util_row.operator("pcv.cut_points", text="Cut", icon='TRASH')
        else:
            if props.screen_edit_enable:
                layout.separator()
                sel_box = layout.box()
                sel_box.label(text="Screen Selection", icon='STICKY_UVS_LOC')
                tool_row = sel_box.row(align=True)
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
        scene = getattr(context, "scene", None)
        layers = list(getattr(scene, "pcv_layers", [])) if scene else []
        points_on = any(getattr(l, "show_points", False) for l in layers)
        return bool(props and settings and props.crop_enable and layer and points_on)

    def draw(self, context):
        layout = self.layout
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
        scene = getattr(context, "scene", None)
        layers = list(getattr(scene, "pcv_layers", [])) if scene else []
        points_on = any(getattr(l, "show_points", False) for l in layers)
        return bool(layer and props and props.screen_edit_enable and not props.crop_enable and points_on)

    def draw(self, context):
        layout = self.layout
        layer = get_active_layer(context)
        if not layer:
            layout.label(text="No point layer", icon='INFO')
            return
        layout.prop(layer, "selection_color", text="Color")
        layout.prop(layer, "selection_color_strength", text="Blend Strength")


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
        scene = getattr(context, "scene", None)
        layers = list(getattr(scene, "pcv_layers", [])) if scene else []
        points_on = any(getattr(l, "show_points", False) for l in layers)
        return bool(layer and points_on)

    def draw(self, context):
        layout = self.layout
        layer = get_active_layer(context)
        if not layer:
            layout.label(text="No active layer", icon='INFO')
            return
        count = get_layer_point_count(layer)
        box = layout.box()
        box.label(text=f"Points Rendered: {count:,}")


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


UI_CLASSES = (
    PCV_PT_MainPanel,
    PCV_PT_CoordinatesPanel,
    PCV_PT_PointAppearancePanel,
    PCV_PT_CropAppearancePanel,
    PCV_PT_ScreenSelectionAppearancePanel,
    PCV_PT_PointStatsPanel,
    PCV_UL_LayerList,
)

def register_workspace_tool():
    try:
        bpy.utils.register_tool(PCV_ScreenSelectTool, after={"builtin.select_box"})
    except Exception:
        pass


def unregister_workspace_tool():
    try:
        bpy.utils.unregister_tool(PCV_ScreenSelectTool)
    except Exception:
        pass
