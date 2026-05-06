from __future__ import annotations

import argparse
from pathlib import Path

import numpy as np
import open3d as o3d

from vggt_step_common import DEFAULT_RUNS_DIR


def default_cloud_path() -> Path:
    refined = sorted(
        DEFAULT_RUNS_DIR.glob("refined_*/points_refined.ply"),
        key=lambda p: p.stat().st_mtime if p.exists() else 0,
        reverse=True,
    )
    if refined:
        return refined[0]

    candidates = sorted(
        DEFAULT_RUNS_DIR.glob("vggt_viser_export_*_aligned*/points.ply"),
        key=lambda p: p.stat().st_mtime if p.exists() else 0,
        reverse=True,
    )
    if candidates:
        return candidates[0]

    raw_candidates = sorted(
        DEFAULT_RUNS_DIR.glob("vggt_viser_export_*/points.ply"),
        key=lambda p: p.stat().st_mtime if p.exists() else 0,
        reverse=True,
    )
    if raw_candidates:
        return raw_candidates[0]

    return DEFAULT_RUNS_DIR / "vggt_viser_export_frames_n10_aligned" / "points.ply"


def parse_color(text: str) -> tuple[float, float, float]:
    parts = [float(x.strip()) for x in text.split(",")]
    if len(parts) != 3:
        raise argparse.ArgumentTypeError("Color must be R,G,B")
    if max(parts) > 1.0:
        parts = [v / 255.0 for v in parts]
    return tuple(max(0.0, min(1.0, v)) for v in parts)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="View a PLY point cloud with Open3D.")
    parser.add_argument("ply", nargs="?", type=Path, default=default_cloud_path())
    parser.add_argument("--point-size", type=float, default=2.0)
    parser.add_argument("--background", type=parse_color, default=parse_color("255,255,255"))
    parser.add_argument("--paint", type=parse_color, default=None, help="Override cloud color, e.g. 180,60,60")
    parser.add_argument("--estimate-normals", action="store_true")
    parser.add_argument("--voxel-size", type=float, default=0.0, help="Optional voxel downsample size.")
    parser.add_argument("--show-frame", action="store_true")
    parser.add_argument("--frame-at", choices=["center", "origin"], default="center")
    parser.add_argument("--frame-size", type=float, default=0.05)
    parser.add_argument("--info-only", action="store_true", help="Print cloud stats without opening a window.")
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    ply_path = args.ply.resolve()
    if not ply_path.exists():
        raise FileNotFoundError(f"Point cloud not found: {ply_path}")

    pcd = o3d.io.read_point_cloud(str(ply_path))
    if pcd.is_empty():
        raise RuntimeError(f"Point cloud is empty: {ply_path}")

    if pcd.has_normals():
        normals = np.asarray(pcd.normals)
        if normals.size and np.nanmax(np.linalg.norm(normals, axis=1)) < 1e-8:
            pcd.normals = o3d.utility.Vector3dVector()
            print("[View] dropped zero normals so RGB colors render correctly")

    if args.voxel_size and args.voxel_size > 0:
        pcd = pcd.voxel_down_sample(args.voxel_size)

    if args.paint is not None:
        pcd.paint_uniform_color(args.paint)

    if args.estimate_normals:
        pcd.estimate_normals(
            search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.01, max_nn=30)
        )

    pts = np.asarray(pcd.points)
    bbox = pcd.get_axis_aligned_bounding_box()
    center = pts.mean(axis=0)
    print("[View] file:", ply_path)
    print("[View] points:", len(pcd.points))
    print("[View] bounds min:", bbox.min_bound)
    print("[View] bounds max:", bbox.max_bound)
    print("[View] center:", center)
    if args.show_frame:
        print("[View] frame_at:", args.frame_at)
    if args.info_only:
        return

    geometries: list[o3d.geometry.Geometry] = [pcd]
    if args.show_frame:
        frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=args.frame_size)
        if args.frame_at == "center":
            frame.translate(center)
        geometries.append(frame)

    vis = o3d.visualization.Visualizer()
    vis.create_window(window_name=f"Point Cloud - {ply_path.name}", width=1280, height=800)
    for geom in geometries:
        vis.add_geometry(geom)

    render = vis.get_render_option()
    render.point_size = float(args.point_size)
    render.background_color = np.asarray(args.background, dtype=float)

    vis.run()
    vis.destroy_window()


if __name__ == "__main__":
    main()
