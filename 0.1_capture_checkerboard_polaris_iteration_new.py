# -*- coding: utf-8 -*-
"""
0.1_capture_checkerboard_polaris_iteration_new.py

用途：
    读取已有棋盘格图片，完成相机内参标定与结果保存

功能：
    1. 从指定图片目录读取图像
    2. 检测棋盘格角点
    3. 进行相机标定
    4. 计算重投影误差
    5. 保存标定结果到 npz/json

说明：
    这版不再包含采集相机、预览、GUI 交互逻辑，只保留标定逻辑。
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple

import cv2
import numpy as np

import run_pi3_session as _cfg


# =========================
# 默认配置（路径统一从 run_pi3_session 读取）
# =========================

DEFAULT_IMAGE_DIR = str(_cfg.SOURCE_DIR)
DEFAULT_OUTPUT_DIR = str(_cfg.SOURCE_DIR.parent / "calib_result")

# 这里沿用你脚本里原来的默认棋盘参数
CHESSBOARD_SIZE = (11, 8)   # inner corners: (cols, rows)
SQUARE_SIZE_MM = 15.0       # 单格尺寸，单位 mm


# =========================
# 工具函数
# =========================

def ensure_dir(path: str | Path) -> Path:
    path = Path(path)
    path.mkdir(parents=True, exist_ok=True)
    return path


def save_json(path: str | Path, obj: Dict[str, Any]) -> None:
    path = Path(path)
    path.write_text(json.dumps(obj, indent=2, ensure_ascii=False), encoding="utf-8")


def save_npz(path: str | Path, **kwargs: Any) -> None:
    path = Path(path)
    np.savez_compressed(str(path), **kwargs)


def list_image_files(image_dir: str | Path) -> List[Path]:
    image_dir = Path(image_dir)
    if not image_dir.exists():
        raise FileNotFoundError(f"Image directory not found: {image_dir}")

    exts = {".png", ".jpg", ".jpeg", ".bmp", ".tif", ".tiff"}
    files = [p for p in sorted(image_dir.iterdir()) if p.is_file() and p.suffix.lower() in exts]
    return files


def build_object_points(board_size: Tuple[int, int], square_size_mm: float) -> np.ndarray:
    cols, rows = board_size
    objp = np.zeros((rows * cols, 3), dtype=np.float32)
    grid = np.mgrid[0:cols, 0:rows].T.reshape(-1, 2).astype(np.float32)
    objp[:, :2] = grid * float(square_size_mm)
    return objp


def find_corners(gray: np.ndarray, board_size: Tuple[int, int]) -> Tuple[bool, Optional[np.ndarray]]:
    flags = cv2.CALIB_CB_ADAPTIVE_THRESH | cv2.CALIB_CB_NORMALIZE_IMAGE
    found, corners = cv2.findChessboardCorners(gray, board_size, flags)

    if found and corners is not None:
        criteria = (
            cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER,
            30,
            0.001,
        )
        corners = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
        return True, corners

    if hasattr(cv2, "findChessboardCornersSB"):
        found, corners = cv2.findChessboardCornersSB(gray, board_size, cv2.CALIB_CB_NORMALIZE_IMAGE)
        if found and corners is not None:
            corners = corners.astype(np.float32)
            return True, corners

    return False, None


def detect_chessboard_corners(
    img_bgr: np.ndarray,
    board_size: Tuple[int, int],
) -> Tuple[bool, Optional[np.ndarray], Optional[np.ndarray]]:
    if img_bgr is None:
        return False, None, None

    if img_bgr.ndim == 2:
        gray = img_bgr
        vis = cv2.cvtColor(img_bgr, cv2.COLOR_GRAY2BGR)
    else:
        vis = img_bgr.copy()
        gray = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2GRAY)

    found, corners = find_corners(gray, board_size)
    if not found or corners is None:
        return False, None, vis

    cv2.drawChessboardCorners(vis, board_size, corners, found)
    return True, corners, vis


def calibrate_from_images(
    image_dir: str | Path,
    board_size: Tuple[int, int],
    square_size_mm: float,
) -> Dict[str, Any]:
    image_files = list_image_files(image_dir)
    if not image_files:
        raise RuntimeError(f"No images found in: {image_dir}")

    objp = build_object_points(board_size, square_size_mm)
    object_points: List[np.ndarray] = []
    image_points: List[np.ndarray] = []
    accepted_images: List[str] = []
    rejected_images: List[str] = []

    image_size: Optional[Tuple[int, int]] = None
    preview_dir = Path(image_dir).parent / "calib_preview"
    ensure_dir(preview_dir)

    for idx, image_path in enumerate(image_files):
        img = cv2.imread(str(image_path), cv2.IMREAD_COLOR)
        if img is None:
            rejected_images.append(str(image_path))
            continue

        if image_size is None:
            h, w = img.shape[:2]
            image_size = (w, h)

        found, corners, vis = detect_chessboard_corners(img, board_size)
        if not found or corners is None:
            rejected_images.append(str(image_path))
            continue

        object_points.append(objp.copy())
        image_points.append(corners)
        accepted_images.append(str(image_path))

        preview_path = preview_dir / f"{idx:04d}_{image_path.stem}_corners.png"
        cv2.imwrite(str(preview_path), vis)

    if image_size is None:
        raise RuntimeError("Could not determine image size from input images.")

    if len(object_points) < 3:
        raise RuntimeError(
            f"Too few valid chessboard images: {len(object_points)}. "
            f"Need at least 3, recommended 10+."
        )

    calib_flags = 0
    rms, K, dist, rvecs, tvecs = cv2.calibrateCamera(
        object_points,
        image_points,
        image_size,
        None,
        None,
        flags=calib_flags,
    )

    per_view_errors: List[float] = []
    for i in range(len(object_points)):
        projected, _ = cv2.projectPoints(object_points[i], rvecs[i], tvecs[i], K, dist)
        error = cv2.norm(image_points[i], projected, cv2.NORM_L2) / len(projected)
        per_view_errors.append(float(error))

    mean_reproj_error = float(np.mean(per_view_errors)) if per_view_errors else float("inf")
    std_reproj_error = float(np.std(per_view_errors)) if per_view_errors else float("inf")

    result: Dict[str, Any] = {
        "image_dir": str(Path(image_dir).resolve()),
        "board_cols": int(board_size[0]),
        "board_rows": int(board_size[1]),
        "square_size_mm": float(square_size_mm),
        "image_size": {"width": int(image_size[0]), "height": int(image_size[1])},
        "num_images_total": len(image_files),
        "num_images_used": len(object_points),
        "num_images_rejected": len(rejected_images),
        "accepted_images": accepted_images,
        "rejected_images": rejected_images,
        "rms": float(rms),
        "mean_reprojection_error": mean_reproj_error,
        "std_reprojection_error": std_reproj_error,
        "per_view_errors": per_view_errors,
        "K": K.tolist(),
        "dist": dist.reshape(-1).tolist(),
        "rvecs": [rv.reshape(-1).tolist() for rv in rvecs],
        "tvecs": [tv.reshape(-1).tolist() for tv in tvecs],
    }

    return result


def print_result(result: Dict[str, Any]) -> None:
    print("=" * 60)
    print("Camera calibration result")
    print("=" * 60)
    print(f"Image dir: {result['image_dir']}")
    print(f"Board: {result['board_cols']} x {result['board_rows']}")
    print(f"Square size (mm): {result['square_size_mm']}")
    print(f"Image size: {result['image_size']['width']} x {result['image_size']['height']}")
    print(f"Total images: {result['num_images_total']}")
    print(f"Used images: {result['num_images_used']}")
    print(f"Rejected images: {result['num_images_rejected']}")
    print(f"RMS: {result['rms']:.6f}")
    print(f"Mean reprojection error: {result['mean_reprojection_error']:.6f}")
    print(f"Std reprojection error: {result['std_reprojection_error']:.6f}")
    print("K:")
    print(np.array(result["K"]))
    print("dist:")
    print(np.array(result["dist"]))
    print("=" * 60)


def main() -> None:
    parser = argparse.ArgumentParser(description="Calibrate camera from chessboard images.")
    parser.add_argument(
        "--image-dir",
        default=DEFAULT_IMAGE_DIR,
        help="Path to folder containing chessboard images",
    )
    parser.add_argument(
        "--output-dir",
        default=DEFAULT_OUTPUT_DIR,
        help="Directory to save calibration result",
    )
    parser.add_argument(
        "--board-cols",
        type=int,
        default=CHESSBOARD_SIZE[0],
        help="Chessboard inner corners columns",
    )
    parser.add_argument(
        "--board-rows",
        type=int,
        default=CHESSBOARD_SIZE[1],
        help="Chessboard inner corners rows",
    )
    parser.add_argument(
        "--square-size-mm",
        type=float,
        default=SQUARE_SIZE_MM,
        help="Chessboard square size in millimeters",
    )
    args = parser.parse_args()

    board_size = (args.board_cols, args.board_rows)
    output_dir = ensure_dir(args.output_dir)

    result = calibrate_from_images(
        image_dir=args.image_dir,
        board_size=board_size,
        square_size_mm=args.square_size_mm,
    )

    print_result(result)

    save_json(output_dir / "camera_calibration.json", result)
    save_npz(
        output_dir / "camera_calibration.npz",
        K=np.asarray(result["K"], dtype=np.float64),
        dist=np.asarray(result["dist"], dtype=np.float64),
        rms=np.asarray(result["rms"], dtype=np.float64),
        mean_reprojection_error=np.asarray(result["mean_reprojection_error"], dtype=np.float64),
    )

    print(f"Saved JSON: {output_dir / 'camera_calibration.json'}")
    print(f"Saved NPZ:  {output_dir / 'camera_calibration.npz'}")
    print(f"Preview images: {Path(args.image_dir).parent / 'calib_preview'}")


if __name__ == "__main__":
    main()