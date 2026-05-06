from __future__ import annotations

import argparse
import json
from pathlib import Path

import numpy as np
import open3d as o3d
import open3d.visualization.gui as gui
import open3d.visualization.rendering as rendering

from step6_compare_cad_pointcloud import default_cloud_path, resolve_cad_path
from vggt_step_common import DEFAULT_RUNS_DIR, TACTILE_DIR


DEFAULT_CAD = TACTILE_DIR / "phantom vs VGGT.stl"
DEFAULT_OUT_DIR = DEFAULT_RUNS_DIR / "cad_pi3_interactive_align"


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Interactively align STL/CAD to the latest Pi3 point cloud."
    )
    parser.add_argument("--cloud", type=Path, default=None)
    parser.add_argument("--cad", type=Path, default=DEFAULT_CAD)
    parser.add_argument("--out-dir", type=Path, default=DEFAULT_OUT_DIR)
    parser.add_argument("--cad-scale", type=float, default=0.001)
    parser.add_argument("--sample-cad-points", type=int, default=200000)
    parser.add_argument(
        "--origin",
        choices=["cloud-center", "cloud-bbox-center", "world-origin"],
        default="cloud-center",
    )
    parser.add_argument("--rx", type=float, default=0.0)
    parser.add_argument("--ry", type=float, default=0.0)
    parser.add_argument("--rz", type=float, default=0.0)
    parser.add_argument("--scale", type=float, default=1.0)
    parser.add_argument("--tx-mm", type=float, default=0.0)
    parser.add_argument("--ty-mm", type=float, default=0.0)
    parser.add_argument("--tz-mm", type=float, default=0.0)
    parser.add_argument("--translation-range-mm", type=float, default=80.0)
    parser.add_argument("--point-size", type=float, default=2.0)
    return parser.parse_args()


def euler_xyz(rx_deg: float, ry_deg: float, rz_deg: float) -> np.ndarray:
    rx, ry, rz = np.deg2rad([rx_deg, ry_deg, rz_deg])
    cx, sx = np.cos(rx), np.sin(rx)
    cy, sy = np.cos(ry), np.sin(ry)
    cz, sz = np.cos(rz), np.sin(rz)
    rx_mat = np.array([[1, 0, 0], [0, cx, -sx], [0, sx, cx]], dtype=float)
    ry_mat = np.array([[cy, 0, sy], [0, 1, 0], [-sy, 0, cy]], dtype=float)
    rz_mat = np.array([[cz, -sz, 0], [sz, cz, 0], [0, 0, 1]], dtype=float)
    return rz_mat @ ry_mat @ rx_mat


def transform_from_params(
    origin: np.ndarray,
    offset_m: np.ndarray,
    rx: float,
    ry: float,
    rz: float,
    scale: float,
) -> np.ndarray:
    transform = np.eye(4, dtype=float)
    transform[:3, :3] = euler_xyz(rx, ry, rz) * scale
    transform[:3, 3] = origin + offset_m
    return transform


def load_cloud(path: Path) -> o3d.geometry.PointCloud:
    cloud = o3d.io.read_point_cloud(str(path))
    if cloud.is_empty():
        raise RuntimeError(f"Point cloud is empty: {path}")
    if cloud.has_normals():
        normals = np.asarray(cloud.normals)
        if normals.size and np.nanmax(np.linalg.norm(normals, axis=1)) < 1e-8:
            cloud.normals = o3d.utility.Vector3dVector()
    return cloud


def load_cad_points(path: Path, cad_scale: float, sample_points: int) -> o3d.geometry.PointCloud:
    path = resolve_cad_path(path)
    mesh = o3d.io.read_triangle_mesh(str(path))
    if not mesh.is_empty() and len(mesh.triangles) > 0:
        mesh.scale(cad_scale, center=np.zeros(3))
        cad = mesh.sample_points_uniformly(number_of_points=sample_points)
    else:
        cad = o3d.io.read_point_cloud(str(path))
        if cad.is_empty():
            raise RuntimeError(f"Could not read CAD: {path}")
        cad.scale(cad_scale, center=np.zeros(3))
        if len(cad.points) > sample_points:
            cad = cad.random_down_sample(sample_points / len(cad.points))
    cad.paint_uniform_color([1.0, 0.05, 0.02])
    return cad


class AlignmentApp:
    def __init__(self, args: argparse.Namespace) -> None:
        self.args = args
        self.cloud_path = args.cloud.resolve() if args.cloud else default_cloud_path().resolve()
        self.cad_path = resolve_cad_path(args.cad).resolve()
        self.out_dir = args.out_dir.resolve()
        self.out_dir.mkdir(parents=True, exist_ok=True)

        self.cloud = load_cloud(self.cloud_path)
        self.cad_base = load_cad_points(self.cad_path, args.cad_scale, args.sample_cad_points)

        points = np.asarray(self.cloud.points, dtype=float)
        if args.origin == "cloud-center":
            self.origin = points.mean(axis=0)
        elif args.origin == "cloud-bbox-center":
            self.origin = np.asarray(self.cloud.get_axis_aligned_bounding_box().get_center(), dtype=float)
        else:
            self.origin = np.zeros(3, dtype=float)

        self.rx = float(args.rx)
        self.ry = float(args.ry)
        self.rz = float(args.rz)
        self.scale = float(args.scale)
        self.offset_m = np.array([args.tx_mm, args.ty_mm, args.tz_mm], dtype=float) / 1000.0

        self.window = gui.Application.instance.create_window("Step6 Pi3/STL Manual Alignment", 1500, 920)
        self.scene = gui.SceneWidget()
        self.scene.scene = rendering.Open3DScene(self.window.renderer)
        self.scene.scene.set_background([1.0, 1.0, 1.0, 1.0])

        self.material_cloud = rendering.MaterialRecord()
        self.material_cloud.shader = "defaultUnlit"
        self.material_cloud.point_size = float(args.point_size)

        self.material_cad = rendering.MaterialRecord()
        self.material_cad.shader = "defaultUnlit"
        self.material_cad.point_size = float(args.point_size) + 1.0

        self.material_origin = rendering.MaterialRecord()
        self.material_origin.shader = "defaultLit"

        panel = gui.Vert(8, gui.Margins(12, 12, 12, 12))
        self.info = gui.Label("")
        panel.add_child(self.info)

        self.rx_slider = self._add_slider(panel, "RX deg", -180, 180, self.rx, self._on_rx)
        self.ry_slider = self._add_slider(panel, "RY deg", -180, 180, self.ry, self._on_ry)
        self.rz_slider = self._add_slider(panel, "RZ deg", -180, 180, self.rz, self._on_rz)
        self.scale_slider = self._add_slider(panel, "Scale", 0.2, 3.0, self.scale, self._on_scale)
        t_range = float(args.translation_range_mm)
        self.tx_slider = self._add_slider(panel, "TX mm", -t_range, t_range, args.tx_mm, self._on_tx)
        self.ty_slider = self._add_slider(panel, "TY mm", -t_range, t_range, args.ty_mm, self._on_ty)
        self.tz_slider = self._add_slider(panel, "TZ mm", -t_range, t_range, args.tz_mm, self._on_tz)

        buttons = gui.Horiz(8)
        reset = gui.Button("Reset")
        reset.set_on_clicked(self._reset)
        save = gui.Button("Save Transform")
        save.set_on_clicked(self._save_transform)
        export = gui.Button("Export Combined PLY")
        export.set_on_clicked(self._export_combined)
        buttons.add_child(reset)
        buttons.add_child(save)
        buttons.add_child(export)
        panel.add_child(buttons)
        panel.add_child(gui.Label("Sliders move STL/CAD only. Save Transform writes T_cad_to_base JSON."))

        self.window.add_child(self.scene)
        self.window.add_child(panel)

        def on_layout(_: gui.LayoutContext) -> None:
            rect = self.window.content_rect
            panel_width = 360
            self.scene.frame = gui.Rect(rect.x, rect.y, rect.width - panel_width, rect.height)
            panel.frame = gui.Rect(rect.get_right() - panel_width, rect.y, panel_width, rect.height)

        self.window.set_on_layout(on_layout)
        self._init_scene()
        self._update_scene(reset_camera=True)

    def _add_slider(self, panel, label: str, lo: float, hi: float, value: float, callback):
        column = gui.Vert(2)
        text = gui.Label(f"{label}: {value:.4g}")
        slider = gui.Slider(gui.Slider.DOUBLE)
        slider.set_limits(float(lo), float(hi))
        slider.double_value = float(value)

        def on_changed(v: float) -> None:
            text.text = f"{label}: {v:.4g}"
            callback(float(v))

        slider.set_on_value_changed(on_changed)
        column.add_child(text)
        column.add_child(slider)
        panel.add_child(column)
        return slider

    def _init_scene(self) -> None:
        self.scene.scene.add_geometry("cloud", self.cloud, self.material_cloud)
        frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.06)
        frame.translate(self.origin)
        self.scene.scene.add_geometry("fixed_origin", frame, self.material_origin)
        bbox = self.cloud.get_axis_aligned_bounding_box()
        self.scene.setup_camera(60.0, bbox, bbox.get_center())

    def _current_transform(self) -> np.ndarray:
        return transform_from_params(self.origin, self.offset_m, self.rx, self.ry, self.rz, self.scale)

    def _current_cad(self) -> o3d.geometry.PointCloud:
        cad = o3d.geometry.PointCloud(self.cad_base)
        cad.transform(self._current_transform())
        cad.paint_uniform_color([1.0, 0.05, 0.02])
        return cad

    def _update_scene(self, reset_camera: bool = False) -> None:
        if self.scene.scene.has_geometry("cad"):
            self.scene.scene.remove_geometry("cad")
        self.scene.scene.add_geometry("cad", self._current_cad(), self.material_cad)
        self.info.text = (
            f"cloud: {self.cloud_path.name}\n"
            f"cad: {self.cad_path.name}\n"
            f"origin: {self.args.origin} [{self.origin[0]:.4f}, {self.origin[1]:.4f}, {self.origin[2]:.4f}]\n"
            f"offset mm=[{self.offset_m[0] * 1000:.1f}, {self.offset_m[1] * 1000:.1f}, {self.offset_m[2] * 1000:.1f}]\n"
            f"RX={self.rx:.3f}, RY={self.ry:.3f}, RZ={self.rz:.3f}, scale={self.scale:.5f}"
        )
        if reset_camera:
            bbox = self.cloud.get_axis_aligned_bounding_box()
            self.scene.setup_camera(60.0, bbox, bbox.get_center())

    def _on_rx(self, value: float) -> None:
        self.rx = value
        self._update_scene()

    def _on_ry(self, value: float) -> None:
        self.ry = value
        self._update_scene()

    def _on_rz(self, value: float) -> None:
        self.rz = value
        self._update_scene()

    def _on_scale(self, value: float) -> None:
        self.scale = value
        self._update_scene()

    def _on_tx(self, value: float) -> None:
        self.offset_m[0] = value / 1000.0
        self._update_scene()

    def _on_ty(self, value: float) -> None:
        self.offset_m[1] = value / 1000.0
        self._update_scene()

    def _on_tz(self, value: float) -> None:
        self.offset_m[2] = value / 1000.0
        self._update_scene()

    def _reset(self) -> None:
        self.rx = self.ry = self.rz = 0.0
        self.scale = 1.0
        self.offset_m[:] = 0.0
        self.rx_slider.double_value = self.rx
        self.ry_slider.double_value = self.ry
        self.rz_slider.double_value = self.rz
        self.scale_slider.double_value = self.scale
        self.tx_slider.double_value = 0.0
        self.ty_slider.double_value = 0.0
        self.tz_slider.double_value = 0.0
        self._update_scene(reset_camera=True)

    def _save_transform(self) -> None:
        payload = {
            "T_cad_to_base": self._current_transform().tolist(),
            "cad_unit_scale": float(self.args.cad_scale),
            "cad_extra_scale": float(self.scale),
            "cad_rot_deg_xyz": [float(self.rx), float(self.ry), float(self.rz)],
            "fixed_origin_base": self.origin.tolist(),
            "manual_origin_offset_m": self.offset_m.tolist(),
            "manual_origin_offset_mm": (self.offset_m * 1000.0).tolist(),
            "cloud_path": str(self.cloud_path),
            "cad_path": str(self.cad_path),
            "note": "Use with step6_compare_cad_pointcloud.py --cad-transform this_json --cad-scale <cad_unit_scale>.",
        }
        path = self.out_dir / "T_cad_to_pi3_manual.json"
        path.write_text(json.dumps(payload, indent=2), encoding="utf-8")
        print("[Interactive] saved:", path)

    def _export_combined(self) -> None:
        cloud = o3d.geometry.PointCloud(self.cloud)
        cad = self._current_cad()
        if not cloud.has_colors():
            cloud.paint_uniform_color([0.0, 0.35, 1.0])
        combined = o3d.geometry.PointCloud()
        combined.points = o3d.utility.Vector3dVector(
            np.vstack([np.asarray(cloud.points), np.asarray(cad.points)])
        )
        combined.colors = o3d.utility.Vector3dVector(
            np.vstack([np.asarray(cloud.colors), np.asarray(cad.colors)])
        )
        path = self.out_dir / "interactive_combined_cloud_cad.ply"
        o3d.io.write_point_cloud(str(path), combined)
        print("[Interactive] exported:", path)


def main() -> None:
    args = parse_args()
    gui.Application.instance.initialize()
    AlignmentApp(args)
    gui.Application.instance.run()


if __name__ == "__main__":
    main()
