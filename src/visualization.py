"""
visualization.py — 3D Studio Viewer for the Photogrammetry Pipeline.
"""

import datetime
import sys
import numpy as np
from pathlib import Path

try:
    import open3d as o3d
    import open3d.visualization.gui as gui
    import open3d.visualization.rendering as rendering
except ImportError:
    o3d = None

from src.utils import get_logger

logger = get_logger(__name__)

# UI Settings
PANEL_WIDTH = 420


def visualize_point_cloud(
    ply_path: Path,
    model_dir: Path = None,
) -> None:
    """
    Open the Studio Viewer. Focuses on high-quality point cloud visualization.
    """
    if o3d is None:
        logger.error("open3d is not installed. Run: pip install open3d")
        sys.exit(1)

    if not ply_path.exists():
        logger.error("PLY file not found: %s", ply_path)
        sys.exit(1)

    logger.info("Initializing Studio Viewer...")
    pcd_raw = o3d.io.read_point_cloud(str(ply_path))

    if not pcd_raw.has_colors():
        pcd_raw.paint_uniform_color([0.7, 0.7, 0.7])

    # --- Orientation Fix ---
    R_flip = pcd_raw.get_rotation_matrix_from_axis_angle([np.pi, 0, 0])
    pcd_raw.rotate(R_flip, center=(0, 0, 0))
    center = pcd_raw.get_center()
    pcd_raw.translate(-center)

    state = {
        "type": "clean",
        "vox": 0.0,
        "noise_std": 1.5,
        "pcd_clean": None,
    }

    # --- App Initialization ---
    app = gui.Application.instance
    app.initialize()

    win = app.create_window("Photogrammetry Studio", 1440, 900)
    em = win.theme.font_size

    scene_widget = gui.SceneWidget()
    scene_widget.scene = rendering.Open3DScene(win.renderer)
    scene_widget.scene.set_background([0.08, 0.08, 0.12, 1.0])
    scene_widget.set_view_controls(gui.SceneWidget.Controls.ROTATE_MODEL)

    # Point cloud material
    mat_pcd = rendering.MaterialRecord()
    mat_pcd.shader = "defaultUnlit"
    mat_pcd.point_size = 3.0 * win.scaling

    def _refresh_display():
        scene_widget.scene.clear_geometry()
        
        cloud = state["pcd_clean"] if state["type"] == "clean" else pcd_raw
        if state["vox"] > 0.001:
            cloud = cloud.voxel_down_sample(state["vox"])
        
        scene_widget.scene.add_geometry("main_cloud", cloud, mat_pcd)
        win.post_redraw()

    def _apply_filters():
        cl, _ = pcd_raw.remove_statistical_outlier(
            nb_neighbors=35, std_ratio=state["noise_std"]
        )
        state["pcd_clean"] = cl
        _refresh_display()

    # Initial load (must happen before app.run())
    _apply_filters()
    bounds = scene_widget.scene.bounding_box
    scene_widget.setup_camera(60.0, bounds, [0, 0, 0])

    # --- Sidebar UI ---
    panel = gui.Vert(int(0.5 * em), gui.Margins(em, em, em, em))

    panel.add_child(gui.Label("PHOTOGRAMMETRY PIPELINE"))
    panel.add_child(_make_sep())

    l_nav = gui.Label("NAVIGATION")
    l_nav.text_color = gui.Color(0.0, 0.6, 1.0)
    panel.add_child(l_nav)
    panel.add_child(gui.Label("Rotate: Left Click + Drag"))
    panel.add_child(gui.Label("Pan: Right Click + Drag"))
    panel.add_child(gui.Label("Zoom: Scroll Wheel / Pinch"))
    panel.add_child(_make_sep())

    l_app = gui.Label("APPEARANCE")
    l_app.text_color = gui.Color(0.0, 0.6, 1.0)
    panel.add_child(l_app)

    panel.add_child(gui.Label("Point Size"))
    size_slider = gui.Slider(gui.Slider.DOUBLE)
    size_slider.set_limits(1.0, 10.0)
    size_slider.double_value = 3.0

    def _on_size(v):
        mat_pcd.point_size = float(v) * win.scaling
        scene_widget.scene.modify_geometry_material("main_cloud", mat_pcd)
        win.post_redraw()

    size_slider.set_on_value_changed(_on_size)
    panel.add_child(size_slider)

    panel.add_child(gui.Label("Voxel Density"))
    vox_slider = gui.Slider(gui.Slider.DOUBLE)
    vox_slider.set_limits(0.0, 0.1)
    vox_slider.double_value = 0.0

    def _on_vox(v):
        state["vox"] = float(v)
        _refresh_display()

    vox_slider.set_on_value_changed(_on_vox)
    panel.add_child(vox_slider)

    panel.add_child(_make_sep())

    l_filter = gui.Label("NOISE FILTERING")
    l_filter.text_color = gui.Color(0.0, 0.6, 1.0)
    panel.add_child(l_filter)

    panel.add_child(gui.Label("Filter Strength (Lower = Aggressive)"))
    noise_slider = gui.Slider(gui.Slider.DOUBLE)
    noise_slider.set_limits(0.1, 3.0)
    noise_slider.double_value = 1.5

    def _on_noise(v):
        state["noise_std"] = float(v)
        _apply_filters()

    noise_slider.set_on_value_changed(_on_noise)
    panel.add_child(noise_slider)

    clean_toggle = gui.Checkbox("Enable Filtering")
    clean_toggle.checked = True

    def _on_clean(checked):
        state["type"] = "clean" if checked else "raw"
        _refresh_display()

    clean_toggle.set_on_checked(_on_clean)
    panel.add_child(clean_toggle)

    panel.add_child(_make_sep())

    l_act = gui.Label("ACTIONS")
    l_act.text_color = gui.Color(0.0, 0.6, 1.0)
    panel.add_child(l_act)

    def _on_screenshot():
        now = datetime.datetime.now().strftime("%H%M%S")
        path = str(ply_path.parent / f"view_{now}.png")
        scene_widget.scene.scene.render_to_image(
            lambda img: o3d.io.write_image(path, img)
        )
        logger.info("Saved: %s", path)

    snap_btn = gui.Button("Save Screenshot")
    snap_btn.set_on_clicked(_on_screenshot)
    panel.add_child(snap_btn)

    reset_btn = gui.Button("Reset Camera")
    reset_btn.set_on_clicked(lambda: _refresh_display())
    panel.add_child(reset_btn)

    def _on_layout(ctx):
        r = win.content_rect
        panel.frame = gui.Rect(r.get_right() - PANEL_WIDTH, r.y, PANEL_WIDTH, r.height)
        scene_widget.frame = gui.Rect(r.x, r.y, r.width - PANEL_WIDTH, r.height)

    win.set_on_layout(_on_layout)
    win.add_child(scene_widget)
    win.add_child(panel)

    app.run()


def _make_sep():
    s = gui.Label("--------------------------------------------")
    s.text_color = gui.Color(0.2, 0.2, 0.2)
    return s
