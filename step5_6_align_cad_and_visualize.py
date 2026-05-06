# -*- coding: utf-8 -*-
"""
step5_6_align_cad_and_visualize.py

功能：
    1. 从当前 Pi3 图像中重新检测棋盘格角点
    2. 使用 condition_robot_poses_intrinsics.npz 中的 poses + intrinsics 计算当前 T_base_board
    3. 使用当前棋盘格中心定位 STL / CAD
    4. 默认使用已验证正确的 STL 姿态参数：
           cad_local_rot_deg = [180, 0, -90]
           stl_origin_in_board_center_mm = [-200, 0.5, 20]
    5. 计算点云到 CAD/STL 表面的最近距离误差：
           mean / RMSE / median / std / min / max / p90 / p95 / p99
    6. 自动打开 Open3D 显示 Pi3 点云 + STL + 棋盘格中心 + 棋盘格线框 + 坐标轴

OpenCV board 坐标系：
    corner[0] = board origin
    +X = corner[0] -> corner[cols - 1]
    +Y = corner[0] -> corner[(rows - 1) * cols]
    +Z = +X cross +Y
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple

import cv2
import numpy as np


# =============================================================================
# 默认路径与参数（从 run_pi3_session.py 自动推导，无需手动修改）
# =============================================================================

import sys as _sys
_sys.path.insert(0, str(Path(__file__).resolve().parent))
import run_pi3_session as _cfg

# 当前 session 的点云文件（由 run_pi3_session.session_output_path() 计算）
_SESSION_PLY  = _cfg.session_output_path()
# 对应的 pi3 输出目录（image_dir / conditions / 各输出文件都在这里）
_SESSION_DIR  = _SESSION_PLY.parent

DEFAULT_IMAGE_DIR   = _SESSION_DIR / "images"
DEFAULT_CONDITIONS  = _SESSION_DIR / "condition_robot_poses_intrinsics.npz"
DEFAULT_POINTCLOUD  = _SESSION_PLY

DEFAULT_STL = Path(
    r"E:\research\FYP\2026FYP\src\tactile_pipeline\phantom vs VGGT.stl"
)

DEFAULT_OUTPUT_JSON     = _SESSION_DIR / "cad_pose_in_base_from_current_board.json"
DEFAULT_ERROR_JSON      = _SESSION_DIR / "pointcloud_to_cad_error.json"
DEFAULT_TRANSFORMED_STL = _SESSION_DIR / "phantom_vs_pi3_aligned_in_base.stl"
DEFAULT_DEBUG_DIR       = _SESSION_DIR / "board_pose_debug"

DEFAULT_BOARD_COLS = 11
DEFAULT_BOARD_ROWS = 8
DEFAULT_SQUARE_SIZE_MM = 15.0

# 已验证正确：STL 参考点相对棋盘格 inner-corner 中心的偏移，单位 mm
DEFAULT_STL_REF_IN_BOARD_CENTER_MM = [-200.0, 0.5, 0]

# 已验证正确：STL 本地坐标系到 OpenCV board 坐标系的旋转，单位 deg
DEFAULT_CAD_LOCAL_ROT_DEG = [180.0, 0.0, -90.0]

DEFAULT_STL_UNIT = "mm"
DEFAULT_WORLD_UNIT = "m"

# 点云到 CAD 表面的误差评估配置
# 0 = 使用全部点；点云很大时可改为 200000 或 500000
DEFAULT_EVAL_SAMPLE_POINTS = 0

# 0 = 不过滤距离；如果点云包含桌面/背景，建议运行时设置 20 或 30
DEFAULT_EVAL_MAX_DISTANCE_MM = 0.0

IMAGE_EXTS = {".png", ".jpg", ".jpeg", ".bmp", ".tif", ".tiff"}


# =============================================================================
# 基础数学工具
# =============================================================================

def ensure_transform(T: Any, name: str = "T") -> np.ndarray:
    T = np.asarray(T, dtype=np.float64)
    if T.shape != (4, 4):
        raise ValueError(f"{name} must be 4x4, got {T.shape}")
    if not np.all(np.isfinite(T)):
        raise ValueError(f"{name} contains non-finite values")
    return T


def length_scale(from_unit: str, to_unit: str) -> float:
    from_unit = from_unit.lower()
    to_unit = to_unit.lower()

    if from_unit == to_unit:
        return 1.0
    if from_unit == "mm" and to_unit == "m":
        return 0.001
    if from_unit == "m" and to_unit == "mm":
        return 1000.0

    raise ValueError(f"Unsupported unit conversion: {from_unit} -> {to_unit}")


def make_transform(R_mat: np.ndarray, t_vec: np.ndarray) -> np.ndarray:
    T = np.eye(4, dtype=np.float64)
    T[:3, :3] = np.asarray(R_mat, dtype=np.float64).reshape(3, 3)
    T[:3, 3] = np.asarray(t_vec, dtype=np.float64).reshape(3)
    return T


def invert_transform(T: np.ndarray) -> np.ndarray:
    T = ensure_transform(T)
    R = T[:3, :3]
    t = T[:3, 3]

    T_inv = np.eye(4, dtype=np.float64)
    T_inv[:3, :3] = R.T
    T_inv[:3, 3] = -R.T @ t
    return T_inv


def project_to_rotation(R_raw: np.ndarray) -> np.ndarray:
    U, _, Vt = np.linalg.svd(R_raw)
    R = U @ Vt

    if np.linalg.det(R) < 0:
        U[:, -1] *= -1.0
        R = U @ Vt

    return R


def rotation_angle_deg(R_mat: np.ndarray) -> float:
    c = np.clip((np.trace(R_mat) - 1.0) / 2.0, -1.0, 1.0)
    return float(np.degrees(np.arccos(c)))


def euler_xyz_deg_to_R(rx_deg: float, ry_deg: float, rz_deg: float) -> np.ndarray:
    """
    本地旋转修正。
    使用 R = Rz @ Ry @ Rx。
    """
    rx = np.deg2rad(rx_deg)
    ry = np.deg2rad(ry_deg)
    rz = np.deg2rad(rz_deg)

    cx, sx = np.cos(rx), np.sin(rx)
    cy, sy = np.cos(ry), np.sin(ry)
    cz, sz = np.cos(rz), np.sin(rz)

    Rx = np.array(
        [
            [1.0, 0.0, 0.0],
            [0.0, cx, -sx],
            [0.0, sx, cx],
        ],
        dtype=np.float64,
    )

    Ry = np.array(
        [
            [cy, 0.0, sy],
            [0.0, 1.0, 0.0],
            [-sy, 0.0, cy],
        ],
        dtype=np.float64,
    )

    Rz = np.array(
        [
            [cz, -sz, 0.0],
            [sz, cz, 0.0],
            [0.0, 0.0, 1.0],
        ],
        dtype=np.float64,
    )

    return Rz @ Ry @ Rx


# =============================================================================
# 图像与棋盘格工具
# =============================================================================

def list_images(image_dir: Path) -> List[Path]:
    if not image_dir.is_dir():
        raise FileNotFoundError(f"Image directory not found: {image_dir}")

    def sort_key(p: Path):
        try:
            return (0, int(p.stem))
        except ValueError:
            return (1, p.name.lower())

    images = [
        p for p in sorted(image_dir.iterdir(), key=sort_key)
        if p.suffix.lower() in IMAGE_EXTS
    ]

    if not images:
        raise RuntimeError(f"No images found in: {image_dir}")

    return images


def build_object_points(
    board_cols: int,
    board_rows: int,
    square_size_mm: float,
    world_unit: str,
) -> np.ndarray:
    """
    OpenCV board object points.

    corner[0] = (0,0,0)
    corner[cols-1] = +X
    corner[(rows-1)*cols] = +Y
    """
    objp_mm = np.zeros((board_cols * board_rows, 3), dtype=np.float32)
    objp_mm[:, :2] = np.mgrid[0:board_cols, 0:board_rows].T.reshape(-1, 2)
    objp_mm *= float(square_size_mm)

    objp = objp_mm * length_scale("mm", world_unit)
    return objp.astype(np.float32)


def find_corners(gray: np.ndarray, pattern: Tuple[int, int]) -> Tuple[bool, Optional[np.ndarray]]:
    """
    返回 corners: shape=(N,1,2)
    """
    if hasattr(cv2, "findChessboardCornersSB"):
        found, corners = cv2.findChessboardCornersSB(
            gray,
            pattern,
            cv2.CALIB_CB_NORMALIZE_IMAGE,
        )
        if found and corners is not None:
            return True, corners.astype(np.float32)

    flags = cv2.CALIB_CB_ADAPTIVE_THRESH | cv2.CALIB_CB_NORMALIZE_IMAGE
    found, corners = cv2.findChessboardCorners(gray, pattern, flags)

    if not found or corners is None:
        return False, None

    criteria = (
        cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER,
        200,
        1e-6,
    )

    corners = cv2.cornerSubPix(
        gray,
        corners,
        (11, 11),
        (-1, -1),
        criteria,
    )

    return True, corners.astype(np.float32)


def save_corner_debug(
    image_path: Path,
    corners: np.ndarray,
    pattern: Tuple[int, int],
    T_base_board: Optional[np.ndarray],
    K: np.ndarray,
    rvec: np.ndarray,
    tvec: np.ndarray,
    axis_length_world: float,
    out_dir: Path,
) -> None:
    out_dir.mkdir(parents=True, exist_ok=True)

    img = cv2.imread(str(image_path), cv2.IMREAD_COLOR)
    if img is None:
        return

    vis = img.copy()
    cv2.drawChessboardCorners(vis, pattern, corners, True)

    cols, rows = pattern
    pts = corners.reshape(-1, 2)

    key_indices = {
        0: "corner[0] origin",
        cols - 1: f"corner[{cols - 1}] +X",
        (rows - 1) * cols: f"corner[{(rows - 1) * cols}] +Y",
        rows * cols - 1: f"corner[{rows * cols - 1}] diag",
    }

    for idx, text in key_indices.items():
        if 0 <= idx < pts.shape[0]:
            x, y = pts[idx]
            cv2.circle(vis, (int(round(x)), int(round(y))), 7, (0, 255, 255), -1)
            cv2.putText(
                vis,
                text,
                (int(round(x)) + 8, int(round(y)) + 18),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.55,
                (0, 255, 255),
                2,
                cv2.LINE_AA,
            )

    dist_zero = np.zeros((5, 1), dtype=np.float64)

    try:
        cv2.drawFrameAxes(
            vis,
            K.astype(np.float64),
            dist_zero,
            rvec.astype(np.float64),
            tvec.astype(np.float64),
            float(axis_length_world),
            4,
        )
    except Exception:
        pass

    if T_base_board is not None:
        line = f"T_base_board t = {T_base_board[:3, 3]}"
        cv2.putText(
            vis,
            line,
            (20, 30),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.65,
            (0, 0, 0),
            3,
            cv2.LINE_AA,
        )
        cv2.putText(
            vis,
            line,
            (20, 30),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.65,
            (255, 255, 255),
            1,
            cv2.LINE_AA,
        )

    out_path = out_dir / f"board_debug_{image_path.stem}.png"
    cv2.imwrite(str(out_path), vis)


# =============================================================================
# 从当前 Pi3 conditions 重新计算 T_base_board
# =============================================================================

def load_conditions(conditions_path: Path) -> Tuple[np.ndarray, np.ndarray, List[str]]:
    if not conditions_path.exists():
        raise FileNotFoundError(f"Conditions file not found: {conditions_path}")

    data = np.load(str(conditions_path), allow_pickle=True)

    if "poses" not in data:
        raise KeyError(f"conditions missing key 'poses': {conditions_path}")
    if "intrinsics" not in data:
        raise KeyError(f"conditions missing key 'intrinsics': {conditions_path}")

    poses = np.asarray(data["poses"], dtype=np.float64)
    intrinsics = np.asarray(data["intrinsics"], dtype=np.float64)

    if poses.ndim != 3 or poses.shape[1:] != (4, 4):
        raise ValueError(f"poses must be (N,4,4), got {poses.shape}")
    if intrinsics.ndim != 3 or intrinsics.shape[1:] != (3, 3):
        raise ValueError(f"intrinsics must be (N,3,3), got {intrinsics.shape}")
    if poses.shape[0] != intrinsics.shape[0]:
        raise ValueError("poses and intrinsics must have same number of frames")

    image_names: List[str] = []
    if "image_names" in data:
        raw_names = data["image_names"]
        for item in raw_names:
            if isinstance(item, bytes):
                image_names.append(item.decode("utf-8"))
            else:
                image_names.append(str(item))
    else:
        image_names = [f"{i:04d}.png" for i in range(poses.shape[0])]

    return poses, intrinsics, image_names


def solve_current_board_poses_from_images(
    image_dir: Path,
    conditions_path: Path,
    board_cols: int,
    board_rows: int,
    square_size_mm: float,
    world_unit: str,
    poses_are_camera_to_base: bool,
    debug_dir: Optional[Path],
    max_images: int = 0,
) -> Tuple[List[np.ndarray], List[Dict[str, Any]]]:
    """
    用当前 Pi3 图像检测棋盘格，并计算 T_base_board。

    默认假设 conditions['poses'][i] 是 T_base_camera。
    如果实际是 T_camera_base，使用 --poses-are-camera-to-base。
    """
    image_paths_all = list_images(image_dir)
    poses, intrinsics, image_names = load_conditions(conditions_path)

    image_map = {p.name: p for p in image_paths_all}

    image_paths: List[Path] = []
    for name in image_names:
        name = Path(str(name)).name
        if name in image_map:
            image_paths.append(image_map[name])

    if not image_paths:
        n = min(len(image_paths_all), poses.shape[0])
        image_paths = image_paths_all[:n]
    else:
        n = min(len(image_paths), poses.shape[0])
        image_paths = image_paths[:n]

    if max_images > 0:
        n = min(n, int(max_images))
        image_paths = image_paths[:n]

    objp = build_object_points(
        board_cols=board_cols,
        board_rows=board_rows,
        square_size_mm=square_size_mm,
        world_unit=world_unit,
    )

    pattern = (board_cols, board_rows)
    dist_zero = np.zeros((5, 1), dtype=np.float64)

    T_list: List[np.ndarray] = []
    records: List[Dict[str, Any]] = []

    print("\n" + "=" * 90)
    print("Detect current checkerboard and solve T_base_board")
    print("=" * 90)
    print(f"[image_dir]   {image_dir}")
    print(f"[conditions]  {conditions_path}")
    print(f"[images]      {len(image_paths)}")
    print(f"[pattern]     {board_cols} x {board_rows}")
    print(f"[square]      {square_size_mm} mm")
    print(f"[world_unit]  {world_unit}")
    print(f"[pose mode]   {'T_camera_base -> invert to T_base_camera' if poses_are_camera_to_base else 'T_base_camera'}")
    print("=" * 90)

    for i, image_path in enumerate(image_paths):
        img = cv2.imread(str(image_path), cv2.IMREAD_COLOR)
        if img is None:
            print(f"[skip] cannot read: {image_path.name}")
            continue

        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

        found, corners = find_corners(gray, pattern)
        if not found or corners is None:
            print(f"[skip] no checkerboard: {image_path.name}")
            records.append(
                {
                    "image": image_path.name,
                    "accepted": False,
                    "reason": "no_checkerboard",
                }
            )
            continue

        K = np.asarray(intrinsics[i], dtype=np.float64)

        ok, rvec, tvec = cv2.solvePnP(
            objp,
            corners,
            K,
            dist_zero,
            flags=cv2.SOLVEPNP_IPPE,
        )

        if not ok:
            print(f"[skip] solvePnP failed: {image_path.name}")
            records.append(
                {
                    "image": image_path.name,
                    "accepted": False,
                    "reason": "solvepnp_failed",
                }
            )
            continue

        reproj, _ = cv2.projectPoints(objp, rvec, tvec, K, dist_zero)
        reproj_err = float(cv2.norm(corners, reproj, cv2.NORM_L2) / len(reproj))

        R_camera_board, _ = cv2.Rodrigues(rvec)
        T_camera_board = make_transform(R_camera_board, tvec.reshape(3))

        T_pose = ensure_transform(poses[i], f"pose[{i}]")
        if poses_are_camera_to_base:
            T_base_camera = invert_transform(T_pose)
        else:
            T_base_camera = T_pose

        T_base_board = T_base_camera @ T_camera_board
        T_list.append(T_base_board)

        rec = {
            "image": image_path.name,
            "accepted": True,
            "reprojection_error_px": reproj_err,
            "T_camera_board": T_camera_board.tolist(),
            "T_base_camera": T_base_camera.tolist(),
            "T_base_board": T_base_board.tolist(),
            "board_translation_base_mm": (
                T_base_board[:3, 3] * length_scale(world_unit, "mm")
            ).tolist(),
        }
        records.append(rec)

        print(
            f"[ok] {image_path.name} | "
            f"reproj={reproj_err:.5f}px | "
            f"T_base_board t(mm)="
            f"{T_base_board[:3, 3] * length_scale(world_unit, 'mm')}"
        )

        if debug_dir is not None:
            axis_len = 60.0 * length_scale("mm", world_unit)
            save_corner_debug(
                image_path=image_path,
                corners=corners,
                pattern=pattern,
                T_base_board=T_base_board,
                K=K,
                rvec=rvec,
                tvec=tvec,
                axis_length_world=axis_len,
                out_dir=debug_dir,
            )

    if len(T_list) < 1:
        raise RuntimeError(
            "No valid current checkerboard poses were found. "
            "Check image_dir, pattern, square_size, and whether checkerboard is visible."
        )

    print("=" * 90)
    print(f"[current board] valid poses: {len(T_list)} / {len(image_paths)}")
    print("=" * 90)

    return T_list, records


def average_T_base_board(T_list: List[np.ndarray], world_unit: str) -> Tuple[np.ndarray, Dict[str, float]]:
    Rs = np.stack([T[:3, :3] for T in T_list], axis=0)
    ts = np.stack([T[:3, 3] for T in T_list], axis=0)

    R_mean = project_to_rotation(np.mean(Rs, axis=0))
    t_mean = np.mean(ts, axis=0)

    T_mean = make_transform(R_mean, t_mean)

    trans_err = np.linalg.norm(ts - t_mean.reshape(1, 3), axis=1)

    rot_err = []
    for R_i in Rs:
        dR = R_mean.T @ R_i
        rot_err.append(rotation_angle_deg(dR))

    rot_err = np.asarray(rot_err, dtype=np.float64)

    stats = {
        "num_samples": float(len(T_list)),
        "translation_mean_abs_world": float(np.mean(trans_err)),
        "translation_std_world": float(np.std(trans_err)),
        "translation_max_abs_world": float(np.max(trans_err)),
        "translation_mean_abs_mm": float(np.mean(trans_err) * length_scale(world_unit, "mm")),
        "translation_std_mm": float(np.std(trans_err) * length_scale(world_unit, "mm")),
        "translation_max_abs_mm": float(np.max(trans_err) * length_scale(world_unit, "mm")),
        "rotation_mean_abs_deg": float(np.mean(rot_err)),
        "rotation_std_deg": float(np.std(rot_err)),
        "rotation_max_abs_deg": float(np.max(rot_err)),
    }

    return T_mean, stats


# =============================================================================
# STL 对齐计算
# =============================================================================

def compute_board_inner_center_in_board(
    board_cols: int,
    board_rows: int,
    square_size_mm: float,
    world_unit: str,
) -> np.ndarray:
    center_mm = np.array(
        [
            (board_cols - 1) * square_size_mm * 0.5,
            (board_rows - 1) * square_size_mm * 0.5,
            0.0,
        ],
        dtype=np.float64,
    )

    return center_mm * length_scale("mm", world_unit)


def compute_reference_pose_in_base(
    T_base_board: np.ndarray,
    board_cols: int,
    board_rows: int,
    square_size_mm: float,
    stl_ref_in_board_center_mm: List[float],
    world_unit: str,
) -> Tuple[np.ndarray, np.ndarray, Dict[str, Any]]:
    T_base_board = ensure_transform(T_base_board)

    R_base_board = T_base_board[:3, :3]
    t_base_board = T_base_board[:3, 3]

    p_board_center = compute_board_inner_center_in_board(
        board_cols=board_cols,
        board_rows=board_rows,
        square_size_mm=square_size_mm,
        world_unit=world_unit,
    )

    p_center_to_ref = (
        np.asarray(stl_ref_in_board_center_mm, dtype=np.float64)
        * length_scale("mm", world_unit)
    )

    p_ref_board = p_board_center + p_center_to_ref
    p_ref_base = t_base_board + R_base_board @ p_ref_board

    debug = {
        "board_origin_in_base_mm": (t_base_board * length_scale(world_unit, "mm")).tolist(),
        "board_center_in_board_mm": (p_board_center * length_scale(world_unit, "mm")).tolist(),
        "stl_ref_offset_from_board_center_mm": [float(v) for v in stl_ref_in_board_center_mm],
        "stl_ref_in_board_mm": (p_ref_board * length_scale(world_unit, "mm")).tolist(),
        "stl_ref_in_base_mm": (p_ref_base * length_scale(world_unit, "mm")).tolist(),
    }

    return p_ref_base, p_ref_board, debug


def compute_T_stl_to_base(
    T_base_board: np.ndarray,
    p_ref_base: np.ndarray,
    stl_ref_point_in_stl_mm: List[float],
    cad_local_rot_deg: List[float],
    world_unit: str,
) -> np.ndarray:
    """
    STL 本地坐标中的 p_ref_stl 映射到 p_ref_base。

    R_total = R_base_board @ R_stl_to_board
    t_total = p_ref_base - R_total @ p_ref_stl
    """
    T_base_board = ensure_transform(T_base_board)
    R_base_board = T_base_board[:3, :3]

    rx, ry, rz = cad_local_rot_deg
    R_stl_to_board = euler_xyz_deg_to_R(rx, ry, rz)

    R_total = R_base_board @ R_stl_to_board

    p_ref_stl = (
        np.asarray(stl_ref_point_in_stl_mm, dtype=np.float64)
        * length_scale("mm", world_unit)
    )

    t_total = np.asarray(p_ref_base, dtype=np.float64).reshape(3) - R_total @ p_ref_stl

    return make_transform(R_total, t_total)


# =============================================================================
# 点云到 CAD 表面误差计算
# =============================================================================

def compute_pointcloud_to_cad_surface_error(
    pointcloud_path: Path,
    stl_path: Path,
    T_stl_to_base: np.ndarray,
    stl_unit: str,
    world_unit: str,
    sample_points: int = 0,
    max_distance_mm: float = 0.0,
    seed: int = 42,
) -> Dict[str, Any]:
    """
    计算点云到 CAD/STL 表面的最近距离误差。

    方法：
        使用 Open3D RaycastingScene.compute_distance()
        对每个点云点计算到 STL 三角网格表面的最近 unsigned distance。

    注意：
        这是 unsigned distance，只有距离大小，没有内外符号。

    返回：
        所有统计值统一用 mm 输出。
    """
    import open3d as o3d

    if not pointcloud_path.exists():
        raise FileNotFoundError(f"Point cloud not found: {pointcloud_path}")
    if not stl_path.exists():
        raise FileNotFoundError(f"STL not found: {stl_path}")

    pcd = o3d.io.read_point_cloud(str(pointcloud_path))
    if pcd is None or pcd.is_empty():
        raise RuntimeError(f"Point cloud empty or unreadable: {pointcloud_path}")

    mesh = o3d.io.read_triangle_mesh(str(stl_path))
    if mesh is None or mesh.is_empty():
        raise RuntimeError(f"STL empty or unreadable: {stl_path}")

    stl_scale = length_scale(stl_unit, world_unit)
    if abs(stl_scale - 1.0) > 1e-12:
        mesh.scale(stl_scale, center=(0.0, 0.0, 0.0))

    mesh.transform(T_stl_to_base)
    mesh.compute_vertex_normals()

    points = np.asarray(pcd.points, dtype=np.float64)
    total_points = int(points.shape[0])

    if total_points == 0:
        raise RuntimeError("Point cloud has zero points.")

    if sample_points is not None and int(sample_points) > 0 and total_points > int(sample_points):
        rng = np.random.default_rng(seed)
        keep_idx = rng.choice(total_points, size=int(sample_points), replace=False)
        points_eval = points[keep_idx]
    else:
        points_eval = points

    eval_points_before_filter = int(points_eval.shape[0])

    try:
        mesh_t = o3d.t.geometry.TriangleMesh.from_legacy(mesh)
        scene = o3d.t.geometry.RaycastingScene()
        _ = scene.add_triangles(mesh_t)

        query_points = o3d.core.Tensor(
            points_eval.astype(np.float32),
            dtype=o3d.core.Dtype.Float32,
        )

        distances_world = scene.compute_distance(query_points).numpy().astype(np.float64)

    except Exception as exc:
        raise RuntimeError(
            "Failed to compute point-to-CAD distances using Open3D RaycastingScene. "
            "Please check your Open3D version."
        ) from exc

    distances_mm = distances_world * length_scale(world_unit, "mm")
    valid = np.isfinite(distances_mm)
    distances_mm = distances_mm[valid]

    if max_distance_mm is not None and float(max_distance_mm) > 0:
        distances_mm = distances_mm[distances_mm <= float(max_distance_mm)]

    eval_points_after_filter = int(distances_mm.size)

    if eval_points_after_filter == 0:
        raise RuntimeError(
            "No points left for CAD error evaluation. "
            "Try increasing --eval-max-distance-mm or set it to 0."
        )

    stats = {
        "metric": "unsigned nearest surface distance from point cloud to CAD mesh",
        "unit": "mm",
        "pointcloud": str(pointcloud_path.resolve()),
        "stl": str(stl_path.resolve()),
        "total_pointcloud_points": total_points,
        "sample_points_requested": int(sample_points),
        "eval_points_before_filter": eval_points_before_filter,
        "eval_points_after_filter": eval_points_after_filter,
        "max_distance_filter_mm": float(max_distance_mm),
        "mean_mm": float(np.mean(distances_mm)),
        "rmse_mm": float(np.sqrt(np.mean(distances_mm ** 2))),
        "median_mm": float(np.median(distances_mm)),
        "min_mm": float(np.min(distances_mm)),
        "max_mm": float(np.max(distances_mm)),
        "std_mm": float(np.std(distances_mm)),
        "p90_mm": float(np.percentile(distances_mm, 90)),
        "p95_mm": float(np.percentile(distances_mm, 95)),
        "p99_mm": float(np.percentile(distances_mm, 99)),
    }

    print("\n" + "=" * 90)
    print("Point Cloud -> CAD Surface Error")
    print("=" * 90)
    print("[metric] unsigned nearest surface distance")
    print(f"[points] total pointcloud points      : {total_points}")
    print(f"[points] eval before filter           : {eval_points_before_filter}")
    print(f"[points] eval after filter            : {eval_points_after_filter}")
    print(f"[filter] max distance filter          : {max_distance_mm} mm")
    print("-" * 90)
    print(f"[error] mean   = {stats['mean_mm']:.6f} mm")
    print(f"[error] RMSE   = {stats['rmse_mm']:.6f} mm")
    print(f"[error] median = {stats['median_mm']:.6f} mm")
    print(f"[error] std    = {stats['std_mm']:.6f} mm")
    print(f"[error] min    = {stats['min_mm']:.6f} mm")
    print(f"[error] max    = {stats['max_mm']:.6f} mm")
    print(f"[error] p90    = {stats['p90_mm']:.6f} mm")
    print(f"[error] p95    = {stats['p95_mm']:.6f} mm")
    print(f"[error] p99    = {stats['p99_mm']:.6f} mm")
    print("=" * 90)

    return stats


def save_pointcloud_to_cad_error_json(
    error_json: Path,
    stats: Dict[str, Any],
) -> None:
    error_json.parent.mkdir(parents=True, exist_ok=True)
    error_json.write_text(
        json.dumps(stats, indent=2, ensure_ascii=False),
        encoding="utf-8",
    )
    print(f"[OK] saved pointcloud-to-CAD error json: {error_json}")


# =============================================================================
# Open3D 可视化
# =============================================================================

def load_pointcloud(pointcloud_path: Path):
    import open3d as o3d

    if not pointcloud_path.exists():
        raise FileNotFoundError(f"Point cloud not found: {pointcloud_path}")

    pcd = o3d.io.read_point_cloud(str(pointcloud_path))
    if pcd is None or pcd.is_empty():
        raise RuntimeError(f"Point cloud empty or unreadable: {pointcloud_path}")

    return pcd


def load_stl_mesh(stl_path: Path, stl_unit: str, world_unit: str):
    import open3d as o3d

    if not stl_path.exists():
        raise FileNotFoundError(f"STL not found: {stl_path}")

    mesh = o3d.io.read_triangle_mesh(str(stl_path))
    if mesh is None or mesh.is_empty():
        raise RuntimeError(f"STL empty or unreadable: {stl_path}")

    scale = length_scale(stl_unit, world_unit)
    if abs(scale - 1.0) > 1e-12:
        mesh.scale(scale, center=(0.0, 0.0, 0.0))

    mesh.compute_vertex_normals()
    mesh.paint_uniform_color([0.05, 0.45, 0.95])

    return mesh


def copy_open3d_geometry(geom):
    try:
        return geom.clone()
    except Exception:
        return type(geom)(geom)


def create_axis_frame(size: float):
    import open3d as o3d
    return o3d.geometry.TriangleMesh.create_coordinate_frame(
        size=float(size),
        origin=(0.0, 0.0, 0.0),
    )


def create_sphere_marker(position: np.ndarray, radius: float, color: List[float]):
    import open3d as o3d

    sphere = o3d.geometry.TriangleMesh.create_sphere(radius=float(radius))
    sphere.translate(np.asarray(position, dtype=np.float64).reshape(3))
    sphere.paint_uniform_color(color)
    sphere.compute_vertex_normals()

    return sphere


def make_board_outline(
    T_base_board: np.ndarray,
    board_cols: int,
    board_rows: int,
    square_size_mm: float,
    world_unit: str,
):
    import open3d as o3d

    w = (board_cols - 1) * square_size_mm * length_scale("mm", world_unit)
    h = (board_rows - 1) * square_size_mm * length_scale("mm", world_unit)

    pts_board = np.array(
        [
            [0.0, 0.0, 0.0],
            [w, 0.0, 0.0],
            [w, h, 0.0],
            [0.0, h, 0.0],
        ],
        dtype=np.float64,
    )

    pts_base = pts_board @ T_base_board[:3, :3].T + T_base_board[:3, 3]

    lines = np.array(
        [
            [0, 1],
            [1, 2],
            [2, 3],
            [3, 0],
        ],
        dtype=np.int32,
    )

    line_set = o3d.geometry.LineSet()
    line_set.points = o3d.utility.Vector3dVector(pts_base)
    line_set.lines = o3d.utility.Vector2iVector(lines)
    line_set.colors = o3d.utility.Vector3dVector(
        np.tile(np.array([[1.0, 0.8, 0.0]]), (4, 1))
    )

    return line_set


def print_geometry_info(pcd, mesh_before, mesh_after, world_unit: str) -> None:
    pcd_pts = np.asarray(pcd.points)
    v0 = np.asarray(mesh_before.vertices)
    v1 = np.asarray(mesh_after.vertices)

    print("\n" + "=" * 90)
    print("Geometry Info")
    print("=" * 90)

    print(f"[PointCloud] points = {pcd_pts.shape[0]}")
    print(f"[PointCloud] min {world_unit} = {pcd_pts.min(axis=0)}")
    print(f"[PointCloud] max {world_unit} = {pcd_pts.max(axis=0)}")
    print(f"[PointCloud] extent {world_unit} = {pcd_pts.max(axis=0) - pcd_pts.min(axis=0)}")

    print()
    print(f"[STL before transform] vertices = {v0.shape[0]}")
    print(f"[STL before transform] min {world_unit} = {v0.min(axis=0)}")
    print(f"[STL before transform] max {world_unit} = {v0.max(axis=0)}")
    print(f"[STL before transform] extent {world_unit} = {v0.max(axis=0) - v0.min(axis=0)}")

    print()
    print(f"[STL after transform] vertices = {v1.shape[0]}")
    print(f"[STL after transform] min {world_unit} = {v1.min(axis=0)}")
    print(f"[STL after transform] max {world_unit} = {v1.max(axis=0)}")
    print(f"[STL after transform] extent {world_unit} = {v1.max(axis=0) - v1.min(axis=0)}")

    print("=" * 90)


def visualize(
    pointcloud_path: Path,
    stl_path: Path,
    T_base_board: np.ndarray,
    T_stl_to_base: np.ndarray,
    p_ref_base: np.ndarray,
    board_cols: int,
    board_rows: int,
    square_size_mm: float,
    stl_unit: str,
    world_unit: str,
    point_size: float,
    show_axes: bool,
    save_transformed_stl: Optional[Path],
) -> None:
    import open3d as o3d

    pcd = load_pointcloud(pointcloud_path)

    mesh_raw = load_stl_mesh(stl_path, stl_unit, world_unit)
    mesh_aligned = copy_open3d_geometry(mesh_raw)
    mesh_aligned.transform(T_stl_to_base)

    print_geometry_info(pcd, mesh_raw, mesh_aligned, world_unit)

    if save_transformed_stl is not None:
        save_transformed_stl.parent.mkdir(parents=True, exist_ok=True)
        o3d.io.write_triangle_mesh(str(save_transformed_stl), mesh_aligned)
        print(f"[OK] saved transformed STL: {save_transformed_stl}")

    marker_radius = 0.006 if world_unit == "m" else 6.0

    board_center_board = compute_board_inner_center_in_board(
        board_cols,
        board_rows,
        square_size_mm,
        world_unit,
    )
    p_board_center_base = T_base_board[:3, 3] + T_base_board[:3, :3] @ board_center_board

    geometries = [
        pcd,
        mesh_aligned,
        create_sphere_marker(p_board_center_base, marker_radius, [1.0, 0.85, 0.0]),
        create_sphere_marker(p_ref_base, marker_radius, [1.0, 0.0, 0.0]),
        make_board_outline(T_base_board, board_cols, board_rows, square_size_mm, world_unit),
    ]

    if show_axes:
        axis_size = 0.08 if world_unit == "m" else 80.0

        base_axis = create_axis_frame(axis_size)
        geometries.append(base_axis)

        board_axis = create_axis_frame(axis_size * 0.7)
        board_axis.transform(T_base_board)
        geometries.append(board_axis)

        stl_axis = create_axis_frame(axis_size * 0.7)
        stl_axis.transform(T_stl_to_base)
        geometries.append(stl_axis)

    print("\n" + "=" * 90)
    print("Open3D Visualization")
    print("=" * 90)
    print("显示内容：")
    print("  - Pi3 彩色点云")
    print("  - 蓝色 STL")
    print("  - 黄色球：当前棋盘格 inner-corner 对角线中心")
    print("  - 红色球：STL 参考点目标位置")
    print("  - 黄色线框：当前棋盘格 inner-corner 外框")
    print("  - 坐标轴：base / board / STL")
    print()
    print("操作：")
    print("  左键拖动旋转，滚轮缩放，右键平移，Q/ESC退出")
    print("=" * 90 + "\n")

    vis = o3d.visualization.Visualizer()
    vis.create_window(
        window_name="Current-board Pi3 point cloud + STL alignment",
        width=1600,
        height=900,
        visible=True,
    )

    for g in geometries:
        vis.add_geometry(g)

    opt = vis.get_render_option()
    opt.point_size = float(point_size)
    opt.background_color = np.array([1.0, 1.0, 1.0], dtype=np.float64)

    vis.run()
    vis.destroy_window()


# =============================================================================
# 保存 JSON
# =============================================================================

def save_result_json(
    out_json: Path,
    args: argparse.Namespace,
    T_base_board: np.ndarray,
    T_stl_to_base: np.ndarray,
    p_ref_base: np.ndarray,
    board_stats: Dict[str, float],
    board_records: List[Dict[str, Any]],
    debug: Dict[str, Any],
    pointcloud_to_cad_error: Optional[Dict[str, Any]] = None,
) -> None:
    out_json.parent.mkdir(parents=True, exist_ok=True)

    payload = {
        "description": "STL aligned to robot base using checkerboard pose re-estimated from current Pi3 images.",
        "world_unit": args.world_unit,
        "stl_unit": args.stl_unit,
        "image_dir": str(Path(args.image_dir).resolve()),
        "conditions": str(Path(args.conditions).resolve()),
        "pointcloud": str(Path(args.pointcloud).resolve()),
        "stl": str(Path(args.stl).resolve()),
        "board": {
            "cols": int(args.board_cols),
            "rows": int(args.board_rows),
            "square_size_mm": float(args.square_size_mm),
            "center_mode": "inner-corner diagonal center",
            "source": "current_images_solvepnp",
        },
        "stl_ref_in_board_center_mm": [float(v) for v in args.stl_origin_in_board_center_mm],
        "stl_ref_point_in_stl_mm": [float(v) for v in args.stl_ref_point_in_stl_mm],
        "cad_local_rot_deg": [float(v) for v in args.cad_local_rot_deg],
        "T_base_board": T_base_board.tolist(),
        "T_stl_to_base": T_stl_to_base.tolist(),
        "p_ref_base": np.asarray(p_ref_base, dtype=float).tolist(),
        "board_pose_stats": board_stats,
        "board_pose_records": board_records,
        "debug": debug,
        "pointcloud_to_cad_error": pointcloud_to_cad_error,
    }

    out_json.write_text(
        json.dumps(payload, indent=2, ensure_ascii=False),
        encoding="utf-8",
    )


# =============================================================================
# CLI
# =============================================================================

def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Re-estimate current checkerboard pose from Pi3 images, align STL, evaluate error, and visualize."
    )

    parser.add_argument(
        "--image-dir",
        type=Path,
        default=DEFAULT_IMAGE_DIR,
        help="Current Pi3 image directory.",
    )

    parser.add_argument(
        "--conditions",
        type=Path,
        default=DEFAULT_CONDITIONS,
        help="condition_robot_poses_intrinsics.npz with poses and intrinsics.",
    )

    parser.add_argument(
        "--pointcloud",
        type=Path,
        default=DEFAULT_POINTCLOUD,
        help="Pi3 aligned projected RGB point cloud.",
    )

    parser.add_argument(
        "--stl",
        type=Path,
        default=DEFAULT_STL,
        help="STL file path.",
    )

    parser.add_argument(
        "--out-json",
        type=Path,
        default=DEFAULT_OUTPUT_JSON,
        help="Output JSON path.",
    )

    parser.add_argument(
        "--error-json",
        type=Path,
        default=DEFAULT_ERROR_JSON,
        help="Output JSON path for point cloud to CAD surface error.",
    )

    parser.add_argument(
        "--debug-dir",
        type=Path,
        default=DEFAULT_DEBUG_DIR,
        help="Output directory for board detection debug images.",
    )

    parser.add_argument(
        "--board-cols",
        type=int,
        default=DEFAULT_BOARD_COLS,
    )

    parser.add_argument(
        "--board-rows",
        type=int,
        default=DEFAULT_BOARD_ROWS,
    )

    parser.add_argument(
        "--square-size-mm",
        type=float,
        default=DEFAULT_SQUARE_SIZE_MM,
    )

    parser.add_argument(
        "--stl-origin-in-board-center-mm",
        type=float,
        nargs=3,
        default=DEFAULT_STL_REF_IN_BOARD_CENTER_MM,
        metavar=("X", "Y", "Z"),
        help="Target STL reference point location relative to current board center, in OpenCV board frame, mm.",
    )

    parser.add_argument(
        "--stl-ref-point-in-stl-mm",
        type=float,
        nargs=3,
        default=[0.0, 0.0, 0.0],
        metavar=("X", "Y", "Z"),
        help="Which point in STL local coordinates should be placed at the red target point. Default STL origin.",
    )

    parser.add_argument(
        "--cad-local-rot-deg",
        type=float,
        nargs=3,
        default=DEFAULT_CAD_LOCAL_ROT_DEG,
        metavar=("RX", "RY", "RZ"),
        help="Local rotation from STL axes to OpenCV board axes, degrees.",
    )

    parser.add_argument(
        "--stl-unit",
        choices=["mm", "m"],
        default=DEFAULT_STL_UNIT,
    )

    parser.add_argument(
        "--world-unit",
        choices=["m", "mm"],
        default=DEFAULT_WORLD_UNIT,
    )

    parser.add_argument(
        "--poses-are-camera-to-base",
        action="store_true",
        help="Use this only if conditions['poses'] are T_camera_base. Default assumes T_base_camera.",
    )

    parser.add_argument(
        "--max-images",
        type=int,
        default=0,
        help="Max images used for board pose estimation. 0 means all.",
    )

    parser.add_argument(
        "--point-size",
        type=float,
        default=2.0,
    )

    parser.add_argument(
        "--eval-sample-points",
        type=int,
        default=DEFAULT_EVAL_SAMPLE_POINTS,
        help="Number of point cloud points used for CAD error evaluation. 0 means all points.",
    )

    parser.add_argument(
        "--eval-max-distance-mm",
        type=float,
        default=DEFAULT_EVAL_MAX_DISTANCE_MM,
        help=(
            "Only points with point-to-CAD distance <= this value are used for statistics. "
            "0 means no filtering. Useful when the point cloud contains table/background."
        ),
    )

    parser.add_argument(
        "--no-error-eval",
        action="store_true",
        help="Disable point cloud to CAD error evaluation.",
    )

    parser.add_argument(
        "--no-axes",
        action="store_true",
    )

    parser.add_argument(
        "--no-view",
        action="store_true",
    )

    parser.add_argument(
        "--no-debug-images",
        action="store_true",
        help="Disable writing board debug images.",
    )

    parser.add_argument(
        "--save-transformed-stl",
        type=Path,
        default=DEFAULT_TRANSFORMED_STL,
    )

    return parser.parse_args()


# =============================================================================
# Main
# =============================================================================

def main() -> None:
    args = parse_args()

    print("\n" + "=" * 90)
    print("Current-board STL Alignment to Pi3 Point Cloud")
    print("=" * 90)
    print(f"[input] image_dir     = {args.image_dir}")
    print(f"[input] conditions    = {args.conditions}")
    print(f"[input] pointcloud    = {args.pointcloud}")
    print(f"[input] stl           = {args.stl}")
    print(f"[output] out_json     = {args.out_json}")
    print(f"[output] error_json   = {args.error_json}")
    print(f"[output] debug_dir    = {args.debug_dir}")
    print(f"[board] cols x rows   = {args.board_cols} x {args.board_rows}")
    print(f"[board] square mm     = {args.square_size_mm}")
    print(f"[target] STL ref relative to board center mm = {args.stl_origin_in_board_center_mm}")
    print(f"[stl] ref point in STL mm                    = {args.stl_ref_point_in_stl_mm}")
    print(f"[stl] local rotation deg                     = {args.cad_local_rot_deg}")
    print(f"[unit] stl_unit      = {args.stl_unit}")
    print(f"[unit] world_unit    = {args.world_unit}")
    print(f"[pose] poses_are_camera_to_base = {args.poses_are_camera_to_base}")
    print(f"[error eval] enabled = {not args.no_error_eval}")
    print(f"[error eval] sample_points = {args.eval_sample_points}")
    print(f"[error eval] max_distance_mm = {args.eval_max_distance_mm}")
    print("=" * 90)

    debug_dir = None if args.no_debug_images else args.debug_dir

    T_list, board_records = solve_current_board_poses_from_images(
        image_dir=args.image_dir,
        conditions_path=args.conditions,
        board_cols=int(args.board_cols),
        board_rows=int(args.board_rows),
        square_size_mm=float(args.square_size_mm),
        world_unit=str(args.world_unit),
        poses_are_camera_to_base=bool(args.poses_are_camera_to_base),
        debug_dir=debug_dir,
        max_images=int(args.max_images),
    )

    T_base_board, board_stats = average_T_base_board(T_list, world_unit=str(args.world_unit))

    p_ref_base, p_ref_board, debug = compute_reference_pose_in_base(
        T_base_board=T_base_board,
        board_cols=int(args.board_cols),
        board_rows=int(args.board_rows),
        square_size_mm=float(args.square_size_mm),
        stl_ref_in_board_center_mm=list(args.stl_origin_in_board_center_mm),
        world_unit=str(args.world_unit),
    )

    T_stl_to_base = compute_T_stl_to_base(
        T_base_board=T_base_board,
        p_ref_base=p_ref_base,
        stl_ref_point_in_stl_mm=list(args.stl_ref_point_in_stl_mm),
        cad_local_rot_deg=list(args.cad_local_rot_deg),
        world_unit=str(args.world_unit),
    )

    pointcloud_to_cad_error = None

    if not args.no_error_eval:
        pointcloud_to_cad_error = compute_pointcloud_to_cad_surface_error(
            pointcloud_path=args.pointcloud,
            stl_path=args.stl,
            T_stl_to_base=T_stl_to_base,
            stl_unit=str(args.stl_unit),
            world_unit=str(args.world_unit),
            sample_points=int(args.eval_sample_points),
            max_distance_mm=float(args.eval_max_distance_mm),
        )

        save_pointcloud_to_cad_error_json(
            error_json=args.error_json,
            stats=pointcloud_to_cad_error,
        )

    save_result_json(
        out_json=args.out_json,
        args=args,
        T_base_board=T_base_board,
        T_stl_to_base=T_stl_to_base,
        p_ref_base=p_ref_base,
        board_stats=board_stats,
        board_records=board_records,
        debug=debug,
        pointcloud_to_cad_error=pointcloud_to_cad_error,
    )

    print("\n" + "=" * 90)
    print("Alignment Result")
    print("=" * 90)

    print("\nT_base_board from current images:")
    print(np.array2string(T_base_board, precision=9, suppress_small=False))

    print("\nT_stl_to_base:")
    print(np.array2string(T_stl_to_base, precision=9, suppress_small=False))

    print("\nCurrent board center in board mm:")
    print(debug["board_center_in_board_mm"])

    print("\nSTL target reference point in board mm:")
    print(debug["stl_ref_in_board_mm"])

    print("\nSTL target reference point in base mm:")
    print(debug["stl_ref_in_base_mm"])

    print("\nBoard pose stability:")
    for k, v in board_stats.items():
        print(f"  {k}: {v}")

    print(f"\n[OK] saved json: {args.out_json}")
    if debug_dir is not None:
        print(f"[OK] debug images: {debug_dir}")
    print("=" * 90)

    if not args.no_view:
        visualize(
            pointcloud_path=args.pointcloud,
            stl_path=args.stl,
            T_base_board=T_base_board,
            T_stl_to_base=T_stl_to_base,
            p_ref_base=p_ref_base,
            board_cols=int(args.board_cols),
            board_rows=int(args.board_rows),
            square_size_mm=float(args.square_size_mm),
            stl_unit=str(args.stl_unit),
            world_unit=str(args.world_unit),
            point_size=float(args.point_size),
            show_axes=not bool(args.no_axes),
            save_transformed_stl=args.save_transformed_stl,
        )
    else:
        print("[INFO] --no-view enabled, skip visualization.")


if __name__ == "__main__":
    main()