from __future__ import annotations

import argparse
import json
from pathlib import Path
from typing import Any, Iterable

import numpy as np
import open3d as o3d

from vggt_step_common import DEFAULT_RUNS_DIR, TACTILE_DIR


DEFAULT_F3D = TACTILE_DIR / "phantom+vs+VGGT.f3d"
SUPPORTED_CAD_EXTS = [".stl", ".obj", ".ply", ".off", ".gltf", ".glb"]


def newest_file(patterns: Iterable[str]) -> Path | None:
    candidates: list[Path] = []
    for pattern in patterns:
        candidates.extend(DEFAULT_RUNS_DIR.glob(pattern))
    candidates = [p for p in candidates if p.is_file()]
    if not candidates:
        return None
    return max(candidates, key=lambda p: p.stat().st_mtime)


def default_cloud_path() -> Path:
    found = newest_file(
        [
            "pi3_*/*projected_rgb.ply",
            "pi3_*/*aligned.ply",
            "refined*liver*/points_refined.ply",
            "vggt_viser_export_liver_data*_aligned/points.ply",
            "vggt_viser_export_*_aligned/points.ply",
        ]
    )
    if found is None:
        raise FileNotFoundError("No refined/aligned point cloud found under camera/runs.")
    return found


def resolve_cad_path(path: Path) -> Path:
    path = path.resolve()
    if path.suffix.lower() != ".f3d":
        return path

    alternatives = [path.with_suffix(ext) for ext in SUPPORTED_CAD_EXTS]
    alternatives = [p for p in alternatives if p.is_file()]
    if alternatives:
        return max(alternatives, key=lambda p: p.stat().st_mtime)

    raise FileNotFoundError(
        "Fusion 360 .f3d cannot be read directly by Open3D. Export the model from Fusion 360 as "
        f"one of {SUPPORTED_CAD_EXTS}, preferably STL or OBJ, next to:\n"
        f"  {path}\n"
        "Example target:\n"
        f"  {path.with_suffix('.stl')}\n"
        "Then run this Step6 script again."
    )


def ensure_transform(value: Any, name: str) -> np.ndarray:
    T = np.asarray(value, dtype=np.float64)
    if T.shape != (4, 4):
        raise ValueError(f"{name} must be 4x4, got {T.shape}")
    if not np.all(np.isfinite(T)):
        raise ValueError(f"{name} contains non-finite values")
    if not np.allclose(T[3], [0.0, 0.0, 0.0, 1.0], atol=1e-8):
        raise ValueError(f"{name} last row must be [0, 0, 0, 1]")
    return T


def load_transform(path: Path | None) -> np.ndarray:
    if path is None:
        return np.eye(4, dtype=np.float64)
    payload = json.loads(path.read_text(encoding="utf-8"))
    if isinstance(payload, list):
        return ensure_transform(payload, str(path))
    for key in ["T_cad_to_base", "T_cad_world", "transform", "T"]:
        if key in payload:
            return ensure_transform(payload[key], key)
    raise KeyError(f"No transform key found in {path}. Expected T_cad_to_base/transform/T.")


def apply_scale_and_transform(
    geom: o3d.geometry.Geometry,
    scale: float,
    transform: np.ndarray,
) -> o3d.geometry.Geometry:
    if scale != 1.0:
        geom.scale(float(scale), center=np.zeros(3))
    geom.transform(transform)
    return geom


def load_cad_as_points(cad_path: Path, sample_points: int, cad_scale: float, transform: np.ndarray) -> tuple[o3d.geometry.PointCloud, dict[str, Any]]:
    cad_path = resolve_cad_path(cad_path)
    info: dict[str, Any] = {"cad_path": str(cad_path), "cad_scale": cad_scale}

    mesh = o3d.io.read_triangle_mesh(str(cad_path))
    if not mesh.is_empty() and len(mesh.triangles) > 0:
        mesh.compute_vertex_normals()
        mesh = apply_scale_and_transform(mesh, cad_scale, transform)
        if sample_points > 0:
            cad_points = mesh.sample_points_uniformly(number_of_points=int(sample_points))
        else:
            cad_points = o3d.geometry.PointCloud()
            cad_points.points = mesh.vertices
        info["cad_type"] = "mesh"
        info["num_vertices"] = len(mesh.vertices)
        info["num_triangles"] = len(mesh.triangles)
        info["num_sampled_points"] = len(cad_points.points)
        return cad_points, info

    cad_points = o3d.io.read_point_cloud(str(cad_path))
    if cad_points.is_empty():
        raise RuntimeError(f"Could not read CAD mesh/point cloud: {cad_path}")
    cad_points = apply_scale_and_transform(cad_points, cad_scale, transform)
    if sample_points > 0 and len(cad_points.points) > sample_points:
        ratio = sample_points / max(1, len(cad_points.points))
        cad_points = cad_points.random_down_sample(ratio)
    info["cad_type"] = "point_cloud"
    info["num_sampled_points"] = len(cad_points.points)
    return cad_points, info


def load_cloud(path: Path, voxel_size: float, max_points: int) -> o3d.geometry.PointCloud:
    pcd = o3d.io.read_point_cloud(str(path))
    if pcd.is_empty():
        raise RuntimeError(f"Point cloud is empty: {path}")
    if voxel_size > 0:
        pcd = pcd.voxel_down_sample(voxel_size)
    if max_points > 0 and len(pcd.points) > max_points:
        ratio = max_points / len(pcd.points)
        pcd = pcd.random_down_sample(ratio)
    return pcd


def translation_transform(offset: np.ndarray) -> np.ndarray:
    T = np.eye(4, dtype=np.float64)
    T[:3, 3] = np.asarray(offset, dtype=np.float64).reshape(3)
    return T


def euler_xyz_transform(degrees_xyz: list[float] | tuple[float, float, float]) -> np.ndarray:
    rx, ry, rz = np.deg2rad(np.asarray(degrees_xyz, dtype=np.float64))
    cx, sx = np.cos(rx), np.sin(rx)
    cy, sy = np.cos(ry), np.sin(ry)
    cz, sz = np.cos(rz), np.sin(rz)

    Rx = np.array(
        [[1.0, 0.0, 0.0], [0.0, cx, -sx], [0.0, sx, cx]],
        dtype=np.float64,
    )
    Ry = np.array(
        [[cy, 0.0, sy], [0.0, 1.0, 0.0], [-sy, 0.0, cy]],
        dtype=np.float64,
    )
    Rz = np.array(
        [[cz, -sz, 0.0], [sz, cz, 0.0], [0.0, 0.0, 1.0]],
        dtype=np.float64,
    )

    T = np.eye(4, dtype=np.float64)
    # Apply X, then Y, then Z in CAD local coordinates.
    T[:3, :3] = Rz @ Ry @ Rx
    return T


def bbox_stats(name: str, pcd: o3d.geometry.PointCloud) -> dict[str, Any]:
    pts = np.asarray(pcd.points, dtype=np.float64)
    bbox = pcd.get_axis_aligned_bounding_box()
    extent = bbox.get_extent()
    return {
        "name": name,
        "num_points": int(len(pts)),
        "min": bbox.min_bound.tolist(),
        "max": bbox.max_bound.tolist(),
        "center": bbox.get_center().tolist(),
        "extent": extent.tolist(),
        "diag": float(np.linalg.norm(extent)),
    }


def nearest_distances(
    source: o3d.geometry.PointCloud,
    target: o3d.geometry.PointCloud,
    chunk_label: str,
) -> np.ndarray:
    # Open3D returns distance from each point in source to nearest target point.
    distances = np.asarray(source.compute_point_cloud_distance(target), dtype=np.float64)
    if distances.size == 0:
        raise RuntimeError(f"No distances computed for {chunk_label}")
    return distances


def distance_summary(distances: np.ndarray, thresholds: list[float]) -> dict[str, Any]:
    out: dict[str, Any] = {
        "count": int(distances.size),
        "mean_m": float(np.mean(distances)),
        "median_m": float(np.median(distances)),
        "rmse_m": float(np.sqrt(np.mean(distances ** 2))),
        "p90_m": float(np.percentile(distances, 90)),
        "p95_m": float(np.percentile(distances, 95)),
        "p99_m": float(np.percentile(distances, 99)),
        "max_m": float(np.max(distances)),
    }
    for threshold in thresholds:
        out[f"within_{threshold:.4f}m"] = float(np.mean(distances <= threshold))
    return out


def color_by_threshold(
    pcd: o3d.geometry.PointCloud,
    distances: np.ndarray,
    threshold: float,
    inside_color: list[float],
    outside_color: list[float],
) -> o3d.geometry.PointCloud:
    colored = o3d.geometry.PointCloud(pcd)
    colors = np.zeros((len(colored.points), 3), dtype=np.float64)
    mask = distances <= threshold
    colors[mask] = inside_color
    colors[~mask] = outside_color
    colored.colors = o3d.utility.Vector3dVector(colors)
    return colored


def maybe_icp_align(
    cloud: o3d.geometry.PointCloud,
    cad: o3d.geometry.PointCloud,
    enabled: bool,
    max_correspondence: float,
) -> tuple[o3d.geometry.PointCloud, dict[str, Any]]:
    if not enabled:
        return cad, {"enabled": False}

    result = o3d.pipelines.registration.registration_icp(
        cad,
        cloud,
        max_correspondence,
        np.eye(4),
        o3d.pipelines.registration.TransformationEstimationPointToPoint(),
    )
    aligned = o3d.geometry.PointCloud(cad)
    aligned.transform(result.transformation)
    return aligned, {
        "enabled": True,
        "fitness": float(result.fitness),
        "inlier_rmse": float(result.inlier_rmse),
        "max_correspondence_distance": float(max_correspondence),
        "T_icp_cad_to_cloud": np.asarray(result.transformation).tolist(),
    }


def prealign_source_to_target(
    source: o3d.geometry.PointCloud,
    target: o3d.geometry.PointCloud,
    align_centers: bool,
    fit_scale: bool,
) -> tuple[o3d.geometry.PointCloud, dict[str, Any]]:
    aligned = o3d.geometry.PointCloud(source)
    info: dict[str, Any] = {
        "align_centers": bool(align_centers),
        "fit_scale_to_bbox_diag": bool(fit_scale),
        "scale": 1.0,
        "translation": [0.0, 0.0, 0.0],
        "T_prealign_source_to_target": np.eye(4).tolist(),
    }
    if not align_centers and not fit_scale:
        return aligned, info

    target_bbox = target.get_axis_aligned_bounding_box()
    source_bbox = aligned.get_axis_aligned_bounding_box()
    target_center = np.asarray(target_bbox.get_center(), dtype=np.float64)
    source_center = np.asarray(source_bbox.get_center(), dtype=np.float64)

    T = np.eye(4, dtype=np.float64)

    if fit_scale:
        target_diag = float(np.linalg.norm(target_bbox.get_extent()))
        source_diag = float(np.linalg.norm(source_bbox.get_extent()))
        if source_diag <= 1e-12:
            raise ValueError("Source bbox diagonal is too small for scale fitting.")
        scale = target_diag / source_diag
        aligned.scale(scale, center=source_center)
        T_scale = np.eye(4, dtype=np.float64)
        T_scale[:3, :3] *= scale
        T_scale[:3, 3] = source_center - scale * source_center
        T = T_scale @ T
        source_bbox = aligned.get_axis_aligned_bounding_box()
        source_center = np.asarray(source_bbox.get_center(), dtype=np.float64)
        info["scale"] = scale

    if align_centers:
        translation = target_center - source_center
        aligned.translate(translation)
        T_translate = np.eye(4, dtype=np.float64)
        T_translate[:3, 3] = translation
        T = T_translate @ T
        info["translation"] = translation.tolist()

    info["T_prealign_source_to_target"] = T.tolist()
    return aligned, info


def icp_align_source_to_target(
    source: o3d.geometry.PointCloud,
    target: o3d.geometry.PointCloud,
    enabled: bool,
    max_correspondence: float,
) -> tuple[o3d.geometry.PointCloud, dict[str, Any]]:
    if not enabled:
        return source, {"enabled": False}

    result = o3d.pipelines.registration.registration_icp(
        source,
        target,
        max_correspondence,
        np.eye(4),
        o3d.pipelines.registration.TransformationEstimationPointToPoint(),
    )
    aligned = o3d.geometry.PointCloud(source)
    aligned.transform(result.transformation)
    return aligned, {
        "enabled": True,
        "fitness": float(result.fitness),
        "inlier_rmse": float(result.inlier_rmse),
        "max_correspondence_distance": float(max_correspondence),
        "T_icp_source_to_target": np.asarray(result.transformation).tolist(),
    }


def write_outputs(
    out_dir: Path,
    cloud_colored: o3d.geometry.PointCloud,
    cad_colored: o3d.geometry.PointCloud,
    report: dict[str, Any],
) -> None:
    out_dir.mkdir(parents=True, exist_ok=True)
    o3d.io.write_point_cloud(str(out_dir / "cloud_colored_by_cad_distance.ply"), cloud_colored)
    o3d.io.write_point_cloud(str(out_dir / "cad_colored_by_cloud_coverage.ply"), cad_colored)
    o3d.io.write_point_cloud(str(out_dir / "cloud_aligned_to_cad.ply"), cloud_colored)

    combined = o3d.geometry.PointCloud()
    combined.points = o3d.utility.Vector3dVector(
        np.vstack([np.asarray(cloud_colored.points), np.asarray(cad_colored.points)])
    )
    combined.colors = o3d.utility.Vector3dVector(
        np.vstack([np.asarray(cloud_colored.colors), np.asarray(cad_colored.colors)])
    )
    o3d.io.write_point_cloud(str(out_dir / "combined_cloud_cad_comparison.ply"), combined)
    (out_dir / "cad_compare_report.json").write_text(json.dumps(report, indent=2), encoding="utf-8")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Step6: compare reconstructed point cloud against CAD model.")
    parser.add_argument("--cloud", type=Path, default=None)
    parser.add_argument("--cad", type=Path, default=DEFAULT_F3D)
    parser.add_argument("--out-dir", type=Path, default=DEFAULT_RUNS_DIR / "cad_pointcloud_comparison")
    parser.add_argument("--cad-scale", type=float, default=0.001,
                        help="Scale CAD translation/vertices into meters. Use 0.001 for CAD exported in mm; use 1.0 if already in meters.")
    parser.add_argument("--cad-extra-scale", type=float, default=1.0,
                        help="Additional uniform CAD scale factor after unit conversion. Use for manual same-origin size tuning.")
    parser.add_argument("--cad-rot-deg", type=float, nargs=3, default=[0.0, 0.0, 0.0], metavar=("RX", "RY", "RZ"),
                        help="Manual CAD rotation in degrees around CAD/STL origin, applied before origin placement.")
    parser.add_argument("--cad-transform", type=Path, default=None,
                        help="Optional JSON 4x4 transform from CAD coordinates to base/point-cloud coordinates.")
    parser.add_argument("--cad-origin-at-cloud-center", action="store_true",
                        help="Place CAD/STL origin at the point-cloud mean center. Keeps CAD scale unchanged.")
    parser.add_argument("--cad-origin-at-cloud-bbox-center", action="store_true",
                        help="Place CAD/STL origin at the point-cloud bounding-box center. Keeps CAD scale unchanged.")
    parser.add_argument("--sample-cad-points", type=int, default=200000)
    parser.add_argument("--cloud-voxel-size", type=float, default=0.0)
    parser.add_argument("--max-cloud-points", type=int, default=250000)
    parser.add_argument("--threshold", type=float, default=0.003,
                        help="Main coverage/contact threshold in meters.")
    parser.add_argument("--thresholds", default="0.001,0.002,0.003,0.005,0.010",
                        help="Comma-separated thresholds in meters for report.")
    parser.add_argument("--icp", action="store_true",
                        help="Also align CAD sample to cloud with ICP before computing metrics.")
    parser.add_argument("--icp-max-correspondence", type=float, default=0.02)
    parser.add_argument("--align-cloud-to-cad", action="store_true",
                        help="Align the VGGT/cloud to CAD coordinates for shape coverage checks.")
    parser.add_argument("--align-centers", action="store_true",
                        help="Diagnostic mode: translate CAD bbox center to cloud bbox center before metrics.")
    parser.add_argument("--fit-scale-to-cloud", action="store_true",
                        help="Diagnostic mode: scale CAD bbox diagonal to cloud bbox diagonal before metrics.")
    parser.add_argument("--view", action="store_true", help="Open Open3D comparison view.")
    parser.add_argument("--point-size", type=float, default=2.0)
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    cloud_path = args.cloud.resolve() if args.cloud else default_cloud_path()
    cad_path = args.cad.resolve()
    out_dir = args.out_dir.resolve()
    thresholds = [float(x.strip()) for x in args.thresholds.split(",") if x.strip()]

    cloud = load_cloud(cloud_path, args.cloud_voxel_size, args.max_cloud_points)
    transform = load_transform(args.cad_transform.resolve() if args.cad_transform else None)
    if any(abs(v) > 1e-12 for v in args.cad_rot_deg):
        R_manual = euler_xyz_transform(args.cad_rot_deg)
        transform = R_manual @ transform
        print("[Step6] manual CAD rotation deg xyz:", args.cad_rot_deg)
    if args.cad_origin_at_cloud_center and args.cad_origin_at_cloud_bbox_center:
        raise ValueError("Use only one of --cad-origin-at-cloud-center or --cad-origin-at-cloud-bbox-center.")
    if args.cad_origin_at_cloud_center:
        cloud_center = np.asarray(cloud.points, dtype=np.float64).mean(axis=0)
        transform = translation_transform(cloud_center) @ transform
        print("[Step6] CAD origin -> cloud mean center:", cloud_center)
    if args.cad_origin_at_cloud_bbox_center:
        cloud_center = np.asarray(cloud.get_axis_aligned_bounding_box().get_center(), dtype=np.float64)
        transform = translation_transform(cloud_center) @ transform
        print("[Step6] CAD origin -> cloud bbox center:", cloud_center)

    try:
        cad_scale = float(args.cad_scale) * float(args.cad_extra_scale)
        cad, cad_info = load_cad_as_points(cad_path, args.sample_cad_points, cad_scale, transform)
        cad_info["cad_unit_scale"] = float(args.cad_scale)
        cad_info["cad_extra_scale"] = float(args.cad_extra_scale)
        cad_info["cad_rot_deg_xyz"] = [float(v) for v in args.cad_rot_deg]
    except FileNotFoundError as exc:
        print("[Step6] CAD input is not ready.")
        print(str(exc))
        raise SystemExit(1) from exc
    if args.align_cloud_to_cad:
        cloud, prealign_info = prealign_source_to_target(cloud, cad, args.align_centers, args.fit_scale_to_cloud)
        cloud, icp_info = icp_align_source_to_target(cloud, cad, args.icp, args.icp_max_correspondence)
        alignment_mode = "cloud_to_cad"
    else:
        cad, prealign_info = prealign_source_to_target(cad, cloud, args.align_centers, args.fit_scale_to_cloud)
        cad, icp_info = maybe_icp_align(cloud, cad, args.icp, args.icp_max_correspondence)
        alignment_mode = "cad_to_cloud"

    cloud_to_cad = nearest_distances(cloud, cad, "cloud_to_cad")
    cad_to_cloud = nearest_distances(cad, cloud, "cad_to_cloud")

    cloud_stats = distance_summary(cloud_to_cad, thresholds)
    cad_stats = distance_summary(cad_to_cloud, thresholds)
    coverage = float(np.mean(cad_to_cloud <= args.threshold))
    cloud_supported = float(np.mean(cloud_to_cad <= args.threshold))

    report = {
        "cloud_path": str(cloud_path),
        "cad_requested_path": str(cad_path),
        "cad_info": cad_info,
        "out_dir": str(out_dir),
        "threshold_m": float(args.threshold),
        "alignment_mode": alignment_mode,
        "coverage_cad_points_within_threshold": coverage,
        "cloud_points_supported_by_cad_within_threshold": cloud_supported,
        "cloud_bbox": bbox_stats("cloud", cloud),
        "cad_bbox_after_scale_transform_icp": bbox_stats("cad", cad),
        "cloud_to_cad_distance": cloud_stats,
        "cad_to_cloud_distance": cad_stats,
        "prealign": prealign_info,
        "icp": icp_info,
    }

    cloud_colored = color_by_threshold(
        cloud,
        cloud_to_cad,
        args.threshold,
        inside_color=[0.1, 0.35, 1.0],
        outside_color=[1.0, 0.85, 0.0],
    )
    cad_colored = color_by_threshold(
        cad,
        cad_to_cloud,
        args.threshold,
        inside_color=[0.0, 0.8, 0.25],
        outside_color=[1.0, 0.05, 0.05],
    )
    write_outputs(out_dir, cloud_colored, cad_colored, report)

    print("[Step6] cloud:", cloud_path)
    print("[Step6] cad:", cad_info["cad_path"])
    print("[Step6] out_dir:", out_dir)
    print("[Step6] alignment_mode:", alignment_mode)
    print("[Step6] threshold:", args.threshold, "m")
    print(f"[Step6] CAD coverage: {coverage * 100:.2f}%")
    print(f"[Step6] cloud supported by CAD: {cloud_supported * 100:.2f}%")
    print(f"[Step6] cloud->CAD median/RMSE: {cloud_stats['median_m'] * 1000:.2f} / {cloud_stats['rmse_m'] * 1000:.2f} mm")
    print(f"[Step6] CAD->cloud median/RMSE: {cad_stats['median_m'] * 1000:.2f} / {cad_stats['rmse_m'] * 1000:.2f} mm")
    print("[Step6] report:", out_dir / "cad_compare_report.json")

    if args.view:
        frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.05)
        vis = o3d.visualization.Visualizer()
        vis.create_window(window_name="Step6 CAD vs point cloud", width=1280, height=820)
        for geom in [cloud_colored, cad_colored, frame]:
            vis.add_geometry(geom)
        render = vis.get_render_option()
        render.point_size = args.point_size
        render.background_color = np.asarray([1.0, 1.0, 1.0])
        vis.run()
        vis.destroy_window()


if __name__ == "__main__":
    main()
