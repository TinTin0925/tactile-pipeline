# -*- coding: utf-8 -*-
"""
0.3_camera_handeye_calib_session02_fixed_metrics.py

适配当前 session_02 数据结构：
- 从 samples.jsonl 读取每帧图像路径与机器人末端位姿 TBaseTool
- 从 camera_calibration.json 读取 K / dist
- 使用棋盘格 solvePnP + OpenCV calibrateHandEye 进行手眼标定
- 统一长度单位为米（m）

输出：
- calib_handeye_compare.npz
- summary.json

坐标约定（默认）：
- TBaseTool = ^B T_T  (base -> tool)
- solvePnP 输出 T_camera_board = ^C T_Board
- calibrateHandEye 最终统一保存：
    T_tool_camera = ^T T_C
    T_camera_tool = ^C T_T = inv(T_tool_camera)

关键判断：
- 若板子固定，则对每一帧都应满足
    T_base_board = T_base_tool @ T_tool_camera @ T_camera_board
  且 T_base_board 在所有帧之间应近似恒定。
- 下游如果需要“把 tool/base 下的几何变到 camera 坐标系”，通常应使用 T_camera_tool。
- 下游如果需要“把 camera 下的结果挂到 tool 上”，使用 T_tool_camera。
"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Any, Dict, Iterable, List, Optional, Tuple

import json
import cv2
import numpy as np


# =========================
# 路径配置（统一从 run_pi3_session 读取）
# =========================
import run_pi3_session as _cfg

SESSION_DIR       = _cfg.SOURCE_DIR.parent
SAMPLES_JSONL     = _cfg.SAMPLES_JSONL
CAMERA_CALIB_JSON = _cfg.CALIB.with_suffix(".json")
CAPTURE_DIR       = _cfg.SOURCE_DIR
OUTPUT_DIR        = _cfg.HAND_EYE.parent
CORNER_DEBUG_DIR  = OUTPUT_DIR / "corner_debug"


# =========================
# 棋盘格配置（统一用米）
# =========================
BOARD_COLS = 11
BOARD_ROWS = 8
CHESSBOARD_SIZE = (BOARD_COLS, BOARD_ROWS)
SQUARE_SIZE_M = 0.015  # 15 mm = 0.015 m
BOARD_POSE_ORIGIN = "corner00"  # "corner00" or "diagonal_center"
# corner00：原点在棋盘内角点网格的 (0,0) 角点; diagonal_center：原点在内角点矩形对角线交点（几何中心）. 该参数只改变平移参考点，不改变姿态方向（旋转矩阵不变）


# =========================
# 机器人位姿配置
# =========================
# 可选: "T_base_tool" 或 "T_tool_base"
ROBOT_POSE_CONVENTION = "T_base_tool"


# =========================
# 算法配置
# =========================
AUTO_SELECT_BEST_METHOD = True
HAND_EYE_METHOD = "TSAI"
HAND_EYE_PAIR_MODE = "all"   # "adjacent" or "all"
OUTLIER_DROP_RATIO = 0.0      # 当前先不删，便于对照原始结果
MIN_IMAGES_AFTER_DROP = 8
SAVE_CORNER_DEBUG = True


# =========================
# 数据结构
# =========================
@dataclass
class Sample:
    index: int
    image_path: Path
    T_base_tool: np.ndarray      # ^B T_T
    T_camera_board: np.ndarray   # ^C T_Board
    reproj_err_px: float


@dataclass
class MethodCompareResult:
    method: str
    axxb_rot_err_mean_deg: float
    axxb_rot_err_max_deg: float
    axxb_rot_err_min_deg: float
    axxb_trans_err_mean_m: float
    axxb_trans_err_max_m: float
    axxb_trans_err_min_m: float
    static_board_trans_std_m: float
    static_board_trans_max_dev_m: float
    static_board_rot_mean_dev_deg: float
    static_board_rot_max_dev_deg: float
    consistency_score: float
    T_tool_camera: np.ndarray
    T_camera_tool: np.ndarray


# =========================
# 工具函数
# =========================
def load_json(path: Path) -> Dict[str, Any]:
    return json.loads(path.read_text(encoding="utf-8"))


def load_jsonl(path: Path) -> List[Dict[str, Any]]:
    rows: List[Dict[str, Any]] = []
    with path.open("r", encoding="utf-8") as f:
        for line in f:
            line = line.strip()
            if not line:
                continue
            rows.append(json.loads(line))
    return rows


def get_first_key(row: Dict[str, Any], keys: Iterable[str]) -> Any:
    for k in keys:
        if k in row:
            return row[k]
    return None


def split_rt(T: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
    T = np.asarray(T, dtype=np.float64)
    if T.shape != (4, 4):
        raise ValueError(f"T must be 4x4, got {T.shape}")
    return T[:3, :3].copy(), T[:3, 3:4].copy()


def make_T(R: np.ndarray, t: np.ndarray) -> np.ndarray:
    T = np.eye(4, dtype=np.float64)
    T[:3, :3] = np.asarray(R, dtype=np.float64)
    T[:3, 3:4] = np.asarray(t, dtype=np.float64).reshape(3, 1)
    return T


def inv_T(T: np.ndarray) -> np.ndarray:
    T = np.asarray(T, dtype=np.float64)
    if T.shape != (4, 4):
        raise ValueError(f"T must be 4x4, got {T.shape}")
    R = T[:3, :3]
    t = T[:3, 3:4]
    Tinv = np.eye(4, dtype=np.float64)
    Tinv[:3, :3] = R.T
    Tinv[:3, 3:4] = -R.T @ t
    return Tinv


def rotation_angle_deg(R: np.ndarray) -> float:
    v = (np.trace(R) - 1.0) / 2.0
    v = float(np.clip(v, -1.0, 1.0))
    return float(np.degrees(np.arccos(v)))


def rotation_matrix_to_euler_xyz_deg(R: np.ndarray) -> List[float]:
    """Return XYZ Euler angles in degrees for reporting only."""
    R = np.asarray(R, dtype=np.float64)
    sy = float(np.sqrt(R[0, 0] * R[0, 0] + R[1, 0] * R[1, 0]))
    singular = sy < 1e-9

    if not singular:
        x = np.arctan2(R[2, 1], R[2, 2])
        y = np.arctan2(-R[2, 0], sy)
        z = np.arctan2(R[1, 0], R[0, 0])
    else:
        x = np.arctan2(-R[1, 2], R[1, 1])
        y = np.arctan2(-R[2, 0], sy)
        z = 0.0
    return np.degrees([x, y, z]).astype(float).tolist()


def try_find_corners(gray: np.ndarray, pattern: Tuple[int, int]) -> Tuple[bool, Optional[np.ndarray]]:
    if hasattr(cv2, "findChessboardCornersSB"):
        found, corners = cv2.findChessboardCornersSB(gray, pattern, cv2.CALIB_CB_NORMALIZE_IMAGE)
        if found:
            return True, corners
    flags = cv2.CALIB_CB_ADAPTIVE_THRESH | cv2.CALIB_CB_NORMALIZE_IMAGE
    found, corners = cv2.findChessboardCorners(gray, pattern, flags)
    if not found or corners is None:
        return False, None
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 200, 1e-6)
    corners = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
    return True, corners


def board_center_offset_in_board(board_size: Tuple[int, int], square_size_m: float) -> np.ndarray:
    cols, rows = board_size
    cx = (cols - 1) * 0.5 * float(square_size_m)
    cy = (rows - 1) * 0.5 * float(square_size_m)
    return np.array([[cx], [cy], [0.0]], dtype=np.float64)


def build_board_object_points(board_size: Tuple[int, int], square_size_m: float) -> np.ndarray:
    cols, rows = board_size
    objp = np.zeros((rows * cols, 3), dtype=np.float64)
    objp[:, :2] = np.mgrid[0:cols, 0:rows].T.reshape(-1, 2)
    objp *= float(square_size_m)
    return objp


def save_corner_debug(image_path: Path, corners: np.ndarray, pattern: Tuple[int, int], out_dir: Path) -> None:
    img = cv2.imread(str(image_path), cv2.IMREAD_COLOR)
    if img is None:
        return
    vis = img.copy()
    cv2.drawChessboardCorners(vis, pattern, corners, True)
    out_dir.mkdir(parents=True, exist_ok=True)
    cv2.imwrite(str(out_dir / f"corners_{image_path.stem}.png"), vis)


def chessboard_pose_C_T_B(
    img_bgr: np.ndarray,
    K: np.ndarray,
    dist: np.ndarray,
    board_size: Tuple[int, int],
    square_size_m: float,
) -> Tuple[Optional[np.ndarray], Optional[np.ndarray], float]:
    gray = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2GRAY)
    ok, corners = try_find_corners(gray, board_size)
    if not ok or corners is None:
        return None, None, float("inf")

    objp = build_board_object_points(board_size, square_size_m)
    ok, rvec, tvec = cv2.solvePnP(objp, corners, K, dist, flags=cv2.SOLVEPNP_ITERATIVE)
    if not ok:
        return None, corners, float("inf")

    imgpoints2, _ = cv2.projectPoints(objp, rvec, tvec, K, dist)
    corners_f64 = np.asarray(corners, dtype=np.float64)
    imgpoints2_f64 = np.asarray(imgpoints2, dtype=np.float64)
    reproj_err = float(cv2.norm(corners_f64, imgpoints2_f64, cv2.NORM_L2) / len(imgpoints2_f64))
    R, _ = cv2.Rodrigues(rvec)
    T_camera_board_corner = make_T(R, tvec.reshape(3, 1))

    if BOARD_POSE_ORIGIN == "diagonal_center":
        t_center_in_corner = board_center_offset_in_board(board_size, square_size_m)
        T_corner_to_center = make_T(np.eye(3, dtype=np.float64), t_center_in_corner)
        T_camera_board = T_camera_board_corner @ T_corner_to_center
    else:
        T_camera_board = T_camera_board_corner

    return T_camera_board, corners, reproj_err


def normalize_robot_pose(T_raw: np.ndarray, convention: str) -> np.ndarray:
    T = np.asarray(T_raw, dtype=np.float64)
    if T.shape != (4, 4):
        raise ValueError(f"Robot pose must be 4x4, got {T.shape}")
    if convention == "T_base_tool":
        return T
    if convention == "T_tool_base":
        return inv_T(T)
    raise ValueError(f"Unknown ROBOT_POSE_CONVENTION: {convention}")


def resolve_sample_image_path(image_path_raw: Any, session_dir: Path, capture_dir: Path) -> Path:
    """兼容绝对路径/相对路径/仅文件名三种格式。"""
    raw = Path(str(image_path_raw))
    candidates: List[Path] = []

    if raw.is_absolute():
        candidates.append(raw)
    else:
        candidates.append(session_dir / raw)
        candidates.append(capture_dir / raw)

    candidates.append(capture_dir / raw.name)

    for p in candidates:
        if p.exists():
            return p

    return raw if raw.is_absolute() else (session_dir / raw)


def load_samples_from_jsonl(
    samples_jsonl: Path,
    K: np.ndarray,
    dist: np.ndarray,
    board_size: Tuple[int, int],
    square_size_m: float,
    robot_pose_convention: str,
) -> Tuple[List[Sample], Dict[str, int]]:
    rows = load_jsonl(samples_jsonl)

    valid: List[Sample] = []
    num_missing_image = 0
    num_board_fail = 0
    num_bad_pose = 0

    for i, row in enumerate(rows):
        image_path_raw = get_first_key(row, ["image_path", "ImagePath"])
        T_base_tool_raw = get_first_key(row, ["T_base_tool", "TBaseTool"])
        index_raw = get_first_key(row, ["index", "Index"])

        if image_path_raw is None or T_base_tool_raw is None:
            num_bad_pose += 1
            continue

        image_path = resolve_sample_image_path(image_path_raw, SESSION_DIR, CAPTURE_DIR)
        img = cv2.imread(str(image_path), cv2.IMREAD_COLOR)
        if img is None:
            num_missing_image += 1
            continue

        T_camera_board, corners, reproj_err = chessboard_pose_C_T_B(
            img_bgr=img,
            K=K,
            dist=dist,
            board_size=board_size,
            square_size_m=square_size_m,
        )
        if T_camera_board is None:
            num_board_fail += 1
            continue

        if SAVE_CORNER_DEBUG and corners is not None:
            save_corner_debug(image_path, corners, board_size, CORNER_DEBUG_DIR)

        try:
            T_base_tool = normalize_robot_pose(np.asarray(T_base_tool_raw, dtype=np.float64), robot_pose_convention)
        except Exception:
            num_bad_pose += 1
            continue

        idx = int(index_raw) if index_raw is not None else i
        valid.append(Sample(
            index=idx,
            image_path=image_path,
            T_base_tool=T_base_tool,
            T_camera_board=T_camera_board,
            reproj_err_px=float(reproj_err),
        ))

    stats = {
        "num_rows_total": len(rows),
        "num_valid": len(valid),
        "num_missing_image": num_missing_image,
        "num_board_fail": num_board_fail,
        "num_bad_pose": num_bad_pose,
    }
    return valid, stats


def maybe_drop_outliers(samples: List[Sample]) -> Tuple[List[Sample], int]:
    if OUTLIER_DROP_RATIO <= 0.0 or len(samples) <= MIN_IMAGES_AFTER_DROP:
        return samples, 0
    samples_sorted = sorted(samples, key=lambda s: s.reproj_err_px)
    n_keep = max(MIN_IMAGES_AFTER_DROP, int(round(len(samples_sorted) * (1.0 - OUTLIER_DROP_RATIO))))
    kept = samples_sorted[:n_keep]
    return kept, len(samples_sorted) - len(kept)


def compute_consistency_score(samples: List[Sample], X_tool_camera: np.ndarray) -> float:
    """AX=XB 一致性分数。这里严格采用和 static_board 同一套方向。

    对固定板有：
        T_base_tool_i @ X @ T_camera_board_i = const
    推出：
        A @ X = X @ B
    其中：
        A = inv(T_base_tool_i) @ T_base_tool_j
        B = T_camera_board_i @ inv(T_camera_board_j)
    """
    if len(samples) < 2:
        return float("inf")

    errs: List[float] = []
    for i in range(len(samples) - 1):
        j = i + 1
        A = inv_T(samples[i].T_base_tool) @ samples[j].T_base_tool
        B = samples[i].T_camera_board @ inv_T(samples[j].T_camera_board)
        d = inv_T(A @ X_tool_camera) @ (X_tool_camera @ B)
        errs.append(float(np.linalg.norm(d[:3, 3]) + np.radians(rotation_angle_deg(d[:3, :3]))))
    return float(np.mean(errs))


def evaluate_static_board(samples: List[Sample], T_tool_camera: np.ndarray) -> Dict[str, float]:
    T_base_board_list = [s.T_base_tool @ T_tool_camera @ s.T_camera_board for s in samples]
    ts = np.array([T[:3, 3] for T in T_base_board_list], dtype=np.float64)
    t_mean = ts.mean(axis=0)
    t_dev = np.linalg.norm(ts - t_mean, axis=1)
    Rs = [T[:3, :3] for T in T_base_board_list]
    R_ref = Rs[0]
    r_dev = [rotation_angle_deg(R_ref.T @ R) for R in Rs]
    return {
        "trans_mean_dev_m": float(np.mean(t_dev)),
        "trans_max_dev_m": float(np.max(t_dev)),
        "trans_std_m": float(np.std(t_dev)),
        "rot_mean_dev_deg": float(np.mean(r_dev)),
        "rot_max_dev_deg": float(np.max(r_dev)),
    }


def compute_checkerboard_poses_in_base(samples: List[Sample], T_tool_camera: np.ndarray) -> Dict[str, Any]:
    """
    Compute checkerboard poses in world/base frame.

    Strict convention:
        ^B T_Board = ^B T_Tool @ ^T T_Camera @ ^C T_Board
    """
    T_base_board_all: List[np.ndarray] = []
    translation_m_all: List[np.ndarray] = []
    translation_mm_all: List[np.ndarray] = []
    pose_rows: List[Dict[str, Any]] = []

    for sample in samples:
        T_base_board = sample.T_base_tool @ T_tool_camera @ sample.T_camera_board
        translation_m = T_base_board[:3, 3].astype(np.float64)
        translation_mm = translation_m * 1000.0
        rvec, _ = cv2.Rodrigues(T_base_board[:3, :3])
        rvec_rad = rvec.reshape(3).astype(np.float64)
        euler_deg = rotation_matrix_to_euler_xyz_deg(T_base_board[:3, :3])

        T_base_board_all.append(T_base_board)
        translation_m_all.append(translation_m)
        translation_mm_all.append(translation_mm)
        pose_rows.append({
            "sample_index": int(sample.index),
            "image_path": str(sample.image_path),
            "reproj_err_px": float(sample.reproj_err_px),
            "T_base_board": T_base_board.tolist(),
            "translation_m": translation_m.tolist(),
            "translation_mm": translation_mm.tolist(),
            "rvec_rad": rvec_rad.tolist(),
            "euler_deg": euler_deg,
        })

    T_base_board_np = np.asarray(T_base_board_all, dtype=np.float64)
    translation_m_np = np.asarray(translation_m_all, dtype=np.float64)
    translation_mm_np = np.asarray(translation_mm_all, dtype=np.float64)

    mean_translation_m = translation_m_np.mean(axis=0)
    std_translation_m = translation_m_np.std(axis=0)
    translation_dev_m = np.linalg.norm(translation_m_np - mean_translation_m, axis=1)
    max_translation_dev_m = float(np.max(translation_dev_m))

    R_ref = T_base_board_np[0, :3, :3]
    rot_devs_deg = [
        rotation_angle_deg(R_ref.T @ T_base_board_np[i, :3, :3])
        for i in range(T_base_board_np.shape[0])
    ]

    statistics = {
        "mean_translation_m": mean_translation_m.tolist(),
        "mean_translation_mm": (mean_translation_m * 1000.0).tolist(),
        "std_translation_m": std_translation_m.tolist(),
        "std_translation_mm": (std_translation_m * 1000.0).tolist(),
        "max_translation_dev_m": max_translation_dev_m,
        "max_translation_dev_mm": max_translation_dev_m * 1000.0,
        "rot_mean_dev_deg": float(np.mean(rot_devs_deg)),
        "rot_max_dev_deg": float(np.max(rot_devs_deg)),
    }

    return {
        "poses": pose_rows,
        "statistics": statistics,
        "T_base_board_all": T_base_board_np,
        "checkerboard_translation_m_all": translation_m_np,
        "checkerboard_translation_mm_all": translation_mm_np,
    }


def print_checkerboard_poses_in_base(checkerboard_pose_result: Dict[str, Any]) -> None:
    print("\n" + "=" * 98)
    print("Checkerboard pose in world/base frame (^B T_Board)")
    print("Formula: T_base_board = T_base_tool @ T_tool_camera @ T_camera_board")
    print("=" * 98)
    for row in checkerboard_pose_result["poses"]:
        print(f"\n[sample] index = {row['sample_index']}")
        print(f"[sample] image_path = {row['image_path']}")
        print(f"[sample] reproj_err_px = {row['reproj_err_px']:.6f}")
        print(f"[sample] T_base_board =\n{np.asarray(row['T_base_board'], dtype=np.float64)}")
        print(f"[sample] translation xyz (m)  = {np.asarray(row['translation_m'], dtype=np.float64)}")
        print(f"[sample] translation xyz (mm) = {np.asarray(row['translation_mm'], dtype=np.float64)}")
        print(f"[sample] rvec_base_board (rad) = {np.asarray(row['rvec_rad'], dtype=np.float64)}")
        print(f"[sample] euler_xyz (deg) = {np.asarray(row['euler_deg'], dtype=np.float64)}")

    stats = checkerboard_pose_result["statistics"]
    print("\n" + "-" * 98)
    print("[checkerboard stats] mean_translation_m  =", np.asarray(stats["mean_translation_m"], dtype=np.float64))
    print("[checkerboard stats] mean_translation_mm =", np.asarray(stats["mean_translation_mm"], dtype=np.float64))
    print("[checkerboard stats] std_translation_m   =", np.asarray(stats["std_translation_m"], dtype=np.float64))
    print("[checkerboard stats] std_translation_mm  =", np.asarray(stats["std_translation_mm"], dtype=np.float64))
    print(f"[checkerboard stats] max_translation_dev_m  = {stats['max_translation_dev_m']:.9f}")
    print(f"[checkerboard stats] max_translation_dev_mm = {stats['max_translation_dev_mm']:.6f}")
    print(f"[checkerboard stats] rot_mean_dev_deg = {stats['rot_mean_dev_deg']:.6f}")
    print(f"[checkerboard stats] rot_max_dev_deg  = {stats['rot_max_dev_deg']:.6f}")


def compute_axxb_errors(samples: List[Sample], T_tool_camera: np.ndarray) -> Tuple[float, float, float, float, float, float]:
    """计算和当前坐标约定一致的 AX=XB 残差误差。"""
    if len(samples) < 2:
        nan = float("nan")
        return nan, nan, nan, nan, nan, nan

    rot_errs: List[float] = []
    trans_errs: List[float] = []

    pairs: List[Tuple[int, int]] = []
    if HAND_EYE_PAIR_MODE == "all":
        for i in range(len(samples) - 1):
            for j in range(i + 1, len(samples)):
                pairs.append((i, j))
    else:
        for i in range(len(samples) - 1):
            pairs.append((i, i + 1))

    for i, j in pairs:
        A = inv_T(samples[i].T_base_tool) @ samples[j].T_base_tool
        B = samples[i].T_camera_board @ inv_T(samples[j].T_camera_board)
        E = inv_T(A @ T_tool_camera) @ (T_tool_camera @ B)
        rot_errs.append(rotation_angle_deg(E[:3, :3]))
        trans_errs.append(float(np.linalg.norm(E[:3, 3])))

    return (
        float(np.mean(rot_errs)), float(np.max(rot_errs)), float(np.min(rot_errs)),
        float(np.mean(trans_errs)), float(np.max(trans_errs)), float(np.min(trans_errs)),
    )


def solve_handeye_single(samples: List[Sample], method_name: str) -> np.ndarray:
    method_map = {
        "TSAI": cv2.CALIB_HAND_EYE_TSAI,
        "PARK": cv2.CALIB_HAND_EYE_PARK,
        "HORAUD": cv2.CALIB_HAND_EYE_HORAUD,
        "ANDREFF": cv2.CALIB_HAND_EYE_ANDREFF,
        "DANIILIDIS": cv2.CALIB_HAND_EYE_DANIILIDIS,
    }
    method = method_map[method_name.upper()]

    R_g2b, t_g2b = [], []
    R_target2cam, t_target2cam = [], []
    for s in samples:
        Rg, tg = split_rt(s.T_base_tool)
        Rt, tt = split_rt(s.T_camera_board)
        R_g2b.append(Rg)
        t_g2b.append(tg)
        R_target2cam.append(Rt)
        t_target2cam.append(tt)

    R_cam2gripper, t_cam2gripper = cv2.calibrateHandEye(
        R_g2b, t_g2b, R_target2cam, t_target2cam, method=method
    )
    T_cam2gripper = make_T(R_cam2gripper, t_cam2gripper)

    # OpenCV 文档名字常让人困惑。这里统一通过一致性分数判断最终使用方向。
    X1 = inv_T(T_cam2gripper)   # 候选: ^T T_C
    X2 = T_cam2gripper          # 防御性备选
    e1 = compute_consistency_score(samples, X1)
    e2 = compute_consistency_score(samples, X2)
    return X1 if e1 <= e2 else X2


def compare_methods(samples: List[Sample]) -> List[MethodCompareResult]:
    methods = ["TSAI", "PARK", "HORAUD", "ANDREFF", "DANIILIDIS"]
    results: List[MethodCompareResult] = []
    for method in methods:
        try:
            T_tool_camera = solve_handeye_single(samples, method)
            T_camera_tool = inv_T(T_tool_camera)
            rot_mean, rot_max, rot_min, trans_mean, trans_max, trans_min = compute_axxb_errors(samples, T_tool_camera)
            static_eval = evaluate_static_board(samples, T_tool_camera)
            results.append(MethodCompareResult(
                method=method,
                axxb_rot_err_mean_deg=rot_mean,
                axxb_rot_err_max_deg=rot_max,
                axxb_rot_err_min_deg=rot_min,
                axxb_trans_err_mean_m=trans_mean,
                axxb_trans_err_max_m=trans_max,
                axxb_trans_err_min_m=trans_min,
                static_board_trans_std_m=static_eval["trans_std_m"],
                static_board_trans_max_dev_m=static_eval["trans_max_dev_m"],
                static_board_rot_mean_dev_deg=static_eval["rot_mean_dev_deg"],
                static_board_rot_max_dev_deg=static_eval["rot_max_dev_deg"],
                consistency_score=compute_consistency_score(samples, T_tool_camera),
                T_tool_camera=T_tool_camera,
                T_camera_tool=T_camera_tool,
            ))
        except Exception as e:
            print(f"[warn] {method} failed: {e}")
    return results


def choose_best_result(results: List[MethodCompareResult]) -> MethodCompareResult:
    if not results:
        raise RuntimeError("No valid hand-eye results")
    if not AUTO_SELECT_BEST_METHOD:
        for r in results:
            if r.method.upper() == HAND_EYE_METHOD.upper():
                return r
        raise RuntimeError(f"Specified method not available: {HAND_EYE_METHOD}")

    # 优先按固定棋盘稳定性选，其次按 AX=XB 残差。
    return min(
        results,
        key=lambda r: (
            r.static_board_trans_max_dev_m,
            r.static_board_rot_max_dev_deg,
            r.axxb_trans_err_mean_m,
            r.axxb_rot_err_mean_deg,
            r.consistency_score,
        ),
    )


def main() -> None:
    print("=" * 98)
    print("session_02 手眼标定（samples.jsonl + camera_calibration.json，单位=米，误差方向已修正）")
    print("=" * 98)
    print(f"[cfg] SAMPLES_JSONL = {SAMPLES_JSONL}")
    print(f"[cfg] CAMERA_CALIB_JSON = {CAMERA_CALIB_JSON}")
    print(f"[cfg] CAPTURE_DIR = {CAPTURE_DIR}")
    print(f"[cfg] OUTPUT_DIR = {OUTPUT_DIR}")
    print(f"[cfg] ROBOT_POSE_CONVENTION = {ROBOT_POSE_CONVENTION}")
    print(f"[cfg] CHESSBOARD_SIZE = {CHESSBOARD_SIZE}")
    print(f"[cfg] SQUARE_SIZE_M = {SQUARE_SIZE_M}")

    calib = load_json(CAMERA_CALIB_JSON)
    K = np.asarray(calib["K"], dtype=np.float64)
    dist = np.asarray(calib["dist"], dtype=np.float64).reshape(-1, 1)

    samples, stats = load_samples_from_jsonl(
        samples_jsonl=SAMPLES_JSONL,
        K=K,
        dist=dist,
        board_size=CHESSBOARD_SIZE,
        square_size_m=SQUARE_SIZE_M,
        robot_pose_convention=ROBOT_POSE_CONVENTION,
    )

    if len(samples) < 8:
        raise RuntimeError(f"有效样本太少: {len(samples)}，至少需要 8")

    samples, dropped = maybe_drop_outliers(samples)
    mean_pnp_reproj = float(np.mean([s.reproj_err_px for s in samples]))
    print(f"[data] stats = {stats}")
    print(f"[data] outlier_dropped = {dropped}")
    print(f"[data] used_samples = {len(samples)}")
    print(f"[data] mean_pnp_reproj_px = {mean_pnp_reproj:.6f}")

    method_results = compare_methods(samples)
    if not method_results:
        raise RuntimeError("所有手眼算法均失败")

    print("\n" + "=" * 156)
    print(
        f"{'method':<12} | {'AX=XB rot_mean(deg)':>18} {'rot_max':>12} {'rot_min':>12} | "
        f"{'AX=XB trans_mean(m)':>19} {'trans_max':>12} {'trans_min':>12} | "
        f"{'static_t_max(m)':>15} {'static_r_max(deg)':>18} | {'score':>10}"
    )
    print("-" * 156)
    for r in method_results:
        print(
            f"{r.method:<12} | {r.axxb_rot_err_mean_deg:>18.6f} {r.axxb_rot_err_max_deg:>12.6f} {r.axxb_rot_err_min_deg:>12.6f} | "
            f"{r.axxb_trans_err_mean_m:>19.6f} {r.axxb_trans_err_max_m:>12.6f} {r.axxb_trans_err_min_m:>12.6f} | "
            f"{r.static_board_trans_max_dev_m:>15.6f} {r.static_board_rot_max_dev_deg:>18.6f} | {r.consistency_score:>10.6f}"
        )
    print("=" * 156)

    best = choose_best_result(method_results)
    T_tool_camera = best.T_tool_camera
    T_camera_tool = best.T_camera_tool
    static_eval = evaluate_static_board(samples, T_tool_camera)
    checkerboard_pose_result = compute_checkerboard_poses_in_base(samples, T_tool_camera)

    OUTPUT_DIR.mkdir(parents=True, exist_ok=True)
    npz_path = OUTPUT_DIR / "calib_handeye_compare.npz"
    json_path = OUTPUT_DIR / "summary.json"

    np.savez(
        npz_path,
        K=K,
        dist=dist,
        T_tool_camera=T_tool_camera,
        T_camera_tool=T_camera_tool,
        # 兼容旧命名：tail≈tool
        T_tail_in_cam=T_tool_camera,
        T_cam_in_tail=T_camera_tool,
        T_base_board_all=checkerboard_pose_result["T_base_board_all"],
        checkerboard_translation_m_all=checkerboard_pose_result["checkerboard_translation_m_all"],
        checkerboard_translation_mm_all=checkerboard_pose_result["checkerboard_translation_mm_all"],
    )

    summary = {
        "method_selected": best.method,
        "robot_pose_convention": ROBOT_POSE_CONVENTION,
        "board_cols": BOARD_COLS,
        "board_rows": BOARD_ROWS,
        "square_size_m": SQUARE_SIZE_M,
        "samples_jsonl": str(SAMPLES_JSONL),
        "camera_calibration_json": str(CAMERA_CALIB_JSON),
        "capture_dir": str(CAPTURE_DIR),
        "stats": stats,
        "used_samples": len(samples),
        "outlier_dropped": dropped,
        "mean_pnp_reproj_px": mean_pnp_reproj,
        "interpretation": {
            "T_tool_camera": "^T T_C: 把 camera 坐标中的点/位姿变到 tool 坐标系。适合把相机结果挂到机械臂末端上。",
            "T_camera_tool": "^C T_T = inv(^T T_C): 把 tool/base 下的几何变到 camera 坐标系。点云/重建/可视化通常更常用这个方向。",
            "static_board_formula": "若棋盘固定，应满足 ^B T_Board = ^B T_Tool @ ^T T_Camera @ ^C T_Board，且在各帧中近似不变。",
        },
        "best_metrics": {
            "axxb_rot_err_mean_deg": best.axxb_rot_err_mean_deg,
            "axxb_rot_err_max_deg": best.axxb_rot_err_max_deg,
            "axxb_rot_err_min_deg": best.axxb_rot_err_min_deg,
            "axxb_trans_err_mean_m": best.axxb_trans_err_mean_m,
            "axxb_trans_err_max_m": best.axxb_trans_err_max_m,
            "axxb_trans_err_min_m": best.axxb_trans_err_min_m,
            "axxb_trans_err_mean_mm": best.axxb_trans_err_mean_m * 1000.0,
            "axxb_trans_err_max_mm": best.axxb_trans_err_max_m * 1000.0,
            "axxb_trans_err_min_mm": best.axxb_trans_err_min_m * 1000.0,
            "consistency_score": best.consistency_score,
        },
        "static_board_eval": {
            **static_eval,
            "trans_mean_dev_mm": static_eval["trans_mean_dev_m"] * 1000.0,
            "trans_max_dev_mm": static_eval["trans_max_dev_m"] * 1000.0,
            "trans_std_mm": static_eval["trans_std_m"] * 1000.0,
        },
        "checkerboard_pose_in_base": checkerboard_pose_result["poses"],
        "checkerboard_pose_statistics": checkerboard_pose_result["statistics"],
        "T_tool_camera": T_tool_camera.tolist(),
        "T_camera_tool": T_camera_tool.tolist(),
        "all_methods": [
            {
                "method": r.method,
                "axxb_rot_err_mean_deg": r.axxb_rot_err_mean_deg,
                "axxb_rot_err_max_deg": r.axxb_rot_err_max_deg,
                "axxb_rot_err_min_deg": r.axxb_rot_err_min_deg,
                "axxb_trans_err_mean_m": r.axxb_trans_err_mean_m,
                "axxb_trans_err_max_m": r.axxb_trans_err_max_m,
                "axxb_trans_err_min_m": r.axxb_trans_err_min_m,
                "axxb_trans_err_mean_mm": r.axxb_trans_err_mean_m * 1000.0,
                "axxb_trans_err_max_mm": r.axxb_trans_err_max_m * 1000.0,
                "axxb_trans_err_min_mm": r.axxb_trans_err_min_m * 1000.0,
                "static_board_trans_std_m": r.static_board_trans_std_m,
                "static_board_trans_max_dev_m": r.static_board_trans_max_dev_m,
                "static_board_rot_mean_dev_deg": r.static_board_rot_mean_dev_deg,
                "static_board_rot_max_dev_deg": r.static_board_rot_max_dev_deg,
                "consistency_score": r.consistency_score,
                "T_tool_camera": r.T_tool_camera.tolist(),
                "T_camera_tool": r.T_camera_tool.tolist(),
            }
            for r in method_results
        ],
    }
    json_path.write_text(json.dumps(summary, indent=2, ensure_ascii=False), encoding="utf-8")

    np.set_printoptions(precision=6, suppress=True)
    print(f"\n[best] method = {best.method}")
    print(f"[best] T_tool_camera (^T T_C) =\n{T_tool_camera}")
    print(f"[best] T_camera_tool (^C T_T) =\n{T_camera_tool}")
    print(f"[best] AX=XB trans mean / max (mm) = {best.axxb_trans_err_mean_m*1000.0:.3f} / {best.axxb_trans_err_max_m*1000.0:.3f}")
    print(f"[best] AX=XB rot mean / max (deg) = {best.axxb_rot_err_mean_deg:.6f} / {best.axxb_rot_err_max_deg:.6f}")
    print(f"[best] static_board trans max dev (mm) = {static_eval['trans_max_dev_m']*1000.0:.3f}")
    print(f"[best] static_board rot max dev (deg) = {static_eval['rot_max_dev_deg']:.6f}")
    print_checkerboard_poses_in_base(checkerboard_pose_result)
    print(f"[save] npz -> {npz_path}")
    print(f"[save] json -> {json_path}")


if __name__ == "__main__":
    main()