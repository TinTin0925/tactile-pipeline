from __future__ import annotations

import argparse
from pathlib import Path

import numpy as np
import open3d as o3d


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Enhance point-cloud RGB colors and save a new PLY.")
    parser.add_argument("input", type=Path, help="Input PLY point cloud.")
    parser.add_argument(
        "--output",
        type=Path,
        default=None,
        help="Output PLY. Default: <input_stem>_color_enhanced.ply",
    )
    parser.add_argument("--gain", type=float, default=1.8, help="Linear brightness gain.")
    parser.add_argument("--gamma", type=float, default=0.75, help="Gamma after gain. <1 brightens.")
    parser.add_argument("--saturation", type=float, default=1.35, help="Color saturation multiplier.")
    parser.add_argument(
        "--auto-contrast",
        action="store_true",
        help="Stretch colors by 1st/99th percentiles before gain/gamma.",
    )
    parser.add_argument("--view", action="store_true", help="Open Open3D viewer after saving.")
    parser.add_argument("--point-size", type=float, default=2.0)
    return parser.parse_args()


def enhance_colors(
    colors: np.ndarray,
    gain: float,
    gamma: float,
    saturation: float,
    auto_contrast: bool,
) -> np.ndarray:
    c = np.asarray(colors, dtype=np.float64).copy()
    c = np.clip(c, 0.0, 1.0)

    if auto_contrast:
        lo = np.percentile(c, 1.0, axis=0)
        hi = np.percentile(c, 99.0, axis=0)
        scale = np.maximum(hi - lo, 1e-6)
        c = (c - lo) / scale

    c = np.clip(c * gain, 0.0, 1.0)

    if gamma > 0:
        c = np.power(c, gamma)

    if saturation != 1.0:
        gray = np.sum(c * np.array([0.299, 0.587, 0.114]), axis=1, keepdims=True)
        c = gray + (c - gray) * saturation

    return np.clip(c, 0.0, 1.0)


def main() -> None:
    args = parse_args()
    input_path = args.input.resolve()
    if not input_path.exists():
        raise FileNotFoundError(f"Input point cloud not found: {input_path}")

    output_path = args.output
    if output_path is None:
        output_path = input_path.with_name(f"{input_path.stem}_color_enhanced.ply")
    output_path = output_path.resolve()
    output_path.parent.mkdir(parents=True, exist_ok=True)

    pcd = o3d.io.read_point_cloud(str(input_path))
    if pcd.is_empty():
        raise RuntimeError(f"Point cloud is empty: {input_path}")
    if not pcd.has_colors():
        raise RuntimeError(f"Point cloud has no RGB fields: {input_path}")

    colors_before = np.asarray(pcd.colors)
    colors_after = enhance_colors(
        colors_before,
        gain=args.gain,
        gamma=args.gamma,
        saturation=args.saturation,
        auto_contrast=args.auto_contrast,
    )
    pcd.colors = o3d.utility.Vector3dVector(colors_after)
    if pcd.has_normals():
        normals = np.asarray(pcd.normals)
        if normals.size and np.nanmax(np.linalg.norm(normals, axis=1)) < 1e-8:
            pcd.normals = o3d.utility.Vector3dVector()

    ok = o3d.io.write_point_cloud(str(output_path), pcd, write_ascii=False)
    if not ok:
        raise RuntimeError(f"Failed to write: {output_path}")

    print("[Color] input:", input_path)
    print("[Color] output:", output_path)
    print("[Color] points:", len(pcd.points))
    print("[Color] before mean RGB:", np.round(colors_before.mean(axis=0) * 255.0, 2))
    print("[Color] after  mean RGB:", np.round(colors_after.mean(axis=0) * 255.0, 2))

    if args.view:
        vis = o3d.visualization.Visualizer()
        vis.create_window(window_name=f"Color Enhanced - {output_path.name}", width=1280, height=800)
        vis.add_geometry(pcd)
        render = vis.get_render_option()
        render.point_size = float(args.point_size)
        render.background_color = np.asarray([0.0, 0.0, 0.0], dtype=float)
        vis.run()
        vis.destroy_window()


if __name__ == "__main__":
    main()
