from __future__ import annotations

import argparse
from pathlib import Path

import cv2
import numpy as np
import open3d as o3d


IMAGE_EXTS = {".png", ".jpg", ".jpeg", ".bmp", ".tif", ".tiff"}


def sorted_images(folder: Path) -> list[Path]:
    def key(path: Path) -> tuple[int, int | str]:
        try:
            return (0, int(path.stem))
        except ValueError:
            return (1, path.name.lower())

    return sorted([p for p in folder.iterdir() if p.suffix.lower() in IMAGE_EXTS], key=key)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Colorize an aligned/base-space point cloud by projecting it into condition images."
    )
    parser.add_argument("cloud", type=Path, help="Aligned point cloud PLY.")
    parser.add_argument("--image-dir", type=Path, required=True, help="Condition image directory.")
    parser.add_argument("--conditions", type=Path, required=True, help="condition_robot_poses_intrinsics.npz.")
    parser.add_argument(
        "--output",
        type=Path,
        default=None,
        help="Output PLY. Default: <cloud_stem>_projected_rgb.ply",
    )
    parser.add_argument("--interval", type=int, default=2, help="Same interval used by Pi3 inference.")
    parser.add_argument("--depth-min", type=float, default=0.02)
    parser.add_argument("--depth-max", type=float, default=0.55)
    parser.add_argument("--chunk-size", type=int, default=100000)
    parser.add_argument(
        "--mode",
        choices=["nearest", "mean"],
        default="nearest",
        help="nearest uses the camera with smallest positive depth; mean averages all valid projections.",
    )
    parser.add_argument("--keep-old-if-missing", action="store_true")
    parser.add_argument("--view", action="store_true")
    parser.add_argument("--point-size", type=float, default=2.0)
    return parser.parse_args()


def load_images(image_dir: Path, interval: int) -> list[np.ndarray]:
    paths = sorted_images(image_dir)
    if interval > 1:
        paths = paths[::interval]
    images: list[np.ndarray] = []
    for path in paths:
        bgr = cv2.imread(str(path), cv2.IMREAD_COLOR)
        if bgr is None:
            raise FileNotFoundError(f"Failed to read image: {path}")
        rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
        images.append(rgb)
    if not images:
        raise RuntimeError(f"No images found in {image_dir}")
    return images


def bilinear_sample_rgb(image: np.ndarray, u: np.ndarray, v: np.ndarray) -> np.ndarray:
    h, w = image.shape[:2]
    u0 = np.floor(u).astype(np.int64)
    v0 = np.floor(v).astype(np.int64)
    u1 = np.clip(u0 + 1, 0, w - 1)
    v1 = np.clip(v0 + 1, 0, h - 1)
    u0 = np.clip(u0, 0, w - 1)
    v0 = np.clip(v0, 0, h - 1)

    du = (u - u0)[:, None]
    dv = (v - v0)[:, None]

    c00 = image[v0, u0].astype(np.float64)
    c10 = image[v0, u1].astype(np.float64)
    c01 = image[v1, u0].astype(np.float64)
    c11 = image[v1, u1].astype(np.float64)
    c0 = c00 * (1.0 - du) + c10 * du
    c1 = c01 * (1.0 - du) + c11 * du
    return (c0 * (1.0 - dv) + c1 * dv) / 255.0


def colorize_chunk(
    points: np.ndarray,
    old_colors: np.ndarray | None,
    images: list[np.ndarray],
    poses: np.ndarray,
    intrinsics: np.ndarray,
    depth_min: float,
    depth_max: float,
    mode: str,
    keep_old_if_missing: bool,
) -> tuple[np.ndarray, np.ndarray]:
    n = points.shape[0]
    if mode == "mean":
        color_sum = np.zeros((n, 3), dtype=np.float64)
        counts = np.zeros(n, dtype=np.int32)
    else:
        best_depth = np.full(n, np.inf, dtype=np.float64)
        best_color = np.zeros((n, 3), dtype=np.float64)
        counts = np.zeros(n, dtype=np.int32)

    ones = np.ones((n, 1), dtype=np.float64)
    pts_h = np.concatenate([points.astype(np.float64), ones], axis=1)

    for image, T_c2w, K in zip(images, poses, intrinsics):
        h, w = image.shape[:2]
        T_w2c = np.linalg.inv(T_c2w.astype(np.float64))
        pts_c = (T_w2c @ pts_h.T).T[:, :3]
        z = pts_c[:, 2]
        valid = (z > depth_min) & (z < depth_max)
        if not np.any(valid):
            continue

        x = pts_c[:, 0]
        y = pts_c[:, 1]
        fx, fy = float(K[0, 0]), float(K[1, 1])
        cx, cy = float(K[0, 2]), float(K[1, 2])
        u = fx * (x / z) + cx
        v = fy * (y / z) + cy
        valid &= (u >= 0) & (u <= w - 2) & (v >= 0) & (v <= h - 2)
        idx = np.flatnonzero(valid)
        if idx.size == 0:
            continue

        sampled = bilinear_sample_rgb(image, u[idx], v[idx])
        if mode == "mean":
            color_sum[idx] += sampled
            counts[idx] += 1
        else:
            closer = z[idx] < best_depth[idx]
            if np.any(closer):
                chosen = idx[closer]
                best_depth[chosen] = z[chosen]
                best_color[chosen] = sampled[closer]
                counts[chosen] = 1

    colors = np.zeros((n, 3), dtype=np.float64)
    hit = counts > 0
    if mode == "mean":
        colors[hit] = color_sum[hit] / counts[hit, None]
    else:
        colors[hit] = best_color[hit]

    if keep_old_if_missing and old_colors is not None:
        colors[~hit] = old_colors[~hit]

    return np.clip(colors, 0.0, 1.0), hit


def main() -> None:
    args = parse_args()
    cloud_path = args.cloud.resolve()
    output_path = args.output
    if output_path is None:
        output_path = cloud_path.with_name(f"{cloud_path.stem}_projected_rgb.ply")
    output_path = output_path.resolve()
    output_path.parent.mkdir(parents=True, exist_ok=True)

    pcd = o3d.io.read_point_cloud(str(cloud_path))
    if pcd.is_empty():
        raise RuntimeError(f"Point cloud is empty: {cloud_path}")

    points = np.asarray(pcd.points)
    old_colors = np.asarray(pcd.colors).copy() if pcd.has_colors() else None

    cond = np.load(args.conditions, allow_pickle=True)
    poses = np.asarray(cond["poses"], dtype=np.float64)
    intrinsics = np.asarray(cond["intrinsics"], dtype=np.float64)
    if args.interval > 1:
        poses = poses[:: args.interval]
        intrinsics = intrinsics[:: args.interval]

    images = load_images(args.image_dir, args.interval)
    count = min(len(images), len(poses), len(intrinsics))
    images = images[:count]
    poses = poses[:count]
    intrinsics = intrinsics[:count]

    print("[Colorize] cloud:", cloud_path)
    print("[Colorize] images:", len(images), args.image_dir)
    print("[Colorize] conditions:", args.conditions)
    print("[Colorize] output:", output_path)

    new_colors = np.zeros((points.shape[0], 3), dtype=np.float64)
    hit_all = np.zeros(points.shape[0], dtype=bool)

    for start in range(0, points.shape[0], args.chunk_size):
        end = min(start + args.chunk_size, points.shape[0])
        colors, hit = colorize_chunk(
            points[start:end],
            None if old_colors is None else old_colors[start:end],
            images,
            poses,
            intrinsics,
            args.depth_min,
            args.depth_max,
            args.mode,
            args.keep_old_if_missing,
        )
        new_colors[start:end] = colors
        hit_all[start:end] = hit
        print(f"[Colorize] chunk {start}:{end}, projected={int(hit.sum())}/{end - start}")

    if old_colors is not None and not args.keep_old_if_missing:
        # Keep non-projectable points visible but neutral instead of pure black.
        new_colors[~hit_all] = np.array([0.55, 0.55, 0.55], dtype=np.float64)

    pcd.colors = o3d.utility.Vector3dVector(new_colors)
    if pcd.has_normals():
        normals = np.asarray(pcd.normals)
        if normals.size and np.nanmax(np.linalg.norm(normals, axis=1)) < 1e-8:
            pcd.normals = o3d.utility.Vector3dVector()
    ok = o3d.io.write_point_cloud(str(output_path), pcd, write_ascii=False)
    if not ok:
        raise RuntimeError(f"Failed to write: {output_path}")

    print("[Colorize] projected points:", int(hit_all.sum()), "/", points.shape[0])
    print("[Colorize] mean RGB:", np.round(new_colors.mean(axis=0) * 255.0, 2))

    if args.view:
        vis = o3d.visualization.Visualizer()
        vis.create_window(window_name=f"Projected RGB - {output_path.name}", width=1280, height=800)
        vis.add_geometry(pcd)
        render = vis.get_render_option()
        render.point_size = float(args.point_size)
        render.background_color = np.asarray([1.0, 1.0, 1.0], dtype=float)
        vis.run()
        vis.destroy_window()


if __name__ == "__main__":
    main()
