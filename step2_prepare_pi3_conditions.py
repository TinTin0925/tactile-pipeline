from __future__ import annotations

import argparse
import json
import shutil
from pathlib import Path
from typing import Any

import cv2
import numpy as np

from vggt_step_common import DEFAULT_RUNS_DIR, count_image_files
import run_pi3_session as _cfg


IMAGE_EXTS = {".png", ".jpg", ".jpeg", ".bmp", ".tif", ".tiff"}


def default_pi3_out_dir(source_dir: Path, num_frames: int, frame_stride: int = 1) -> Path:
    parent_tag = source_dir.parent.name if source_dir.parent.name else "images"
    stride_tag = "" if frame_stride <= 1 else f"_s{frame_stride}"
    return DEFAULT_RUNS_DIR / f"pi3_{parent_tag}_{source_dir.name}_n{num_frames}{stride_tag}"


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Step 2A: prepare Pi3X images and condition_robot_poses_intrinsics.npz."
    )
    parser.add_argument("--source-dir", type=Path, default=_cfg.SOURCE_DIR)
    parser.add_argument("--samples-jsonl", type=Path, default=_cfg.SAMPLES_JSONL)
    parser.add_argument("--hand-eye", type=Path, default=_cfg.HAND_EYE)
    parser.add_argument("--hand-eye-key", default="T_tool_camera")
    parser.add_argument("--calib", type=Path, default=_cfg.CALIB)
    parser.add_argument("--out-dir", type=Path, default=None)
    parser.add_argument("--num-frames", type=int, default=_cfg.NUM_FRAMES)
    parser.add_argument("--all-frames", action="store_true")
    parser.add_argument("--start-index", type=int, default=_cfg.START_INDEX)
    parser.add_argument("--frame-stride", type=int, default=_cfg.FRAME_STRIDE)
    parser.add_argument("--pose-translation-scale", type=float, default=1.0)
    parser.add_argument("--undistort", action=argparse.BooleanOptionalAction, default=_cfg.UNDISTORT)
    parser.add_argument("--undistort-alpha", type=float, default=_cfg.UNDISTORT_ALPHA)
    return parser.parse_args()


def load_json(path: Path) -> dict[str, Any]:
    return json.loads(path.read_text(encoding="utf-8"))


def load_npz_or_json_matrix(path: Path, key: str) -> np.ndarray:
    if path.suffix.lower() == ".npz":
        data = np.load(path, allow_pickle=True)
        if key not in data:
            raise KeyError(f"{path} does not contain key {key!r}. Available: {list(data.files)}")
        T = np.asarray(data[key], dtype=np.float64)
    else:
        data = load_json(path)
        if key not in data:
            raise KeyError(f"{path} does not contain key {key!r}. Available: {list(data.keys())}")
        T = np.asarray(data[key], dtype=np.float64)

    if T.shape != (4, 4):
        raise ValueError(f"{key} must be 4x4, got {T.shape}")
    return T


def load_camera_calibration(path: Path) -> tuple[np.ndarray, np.ndarray]:
    if path.is_dir():
        npz = path / "camera_calibration.npz"
        js = path / "camera_calibration.json"
        path = npz if npz.exists() else js

    if path.suffix.lower() == ".npz":
        data = np.load(path, allow_pickle=True)
        if "K" in data:
            K = np.asarray(data["K"], dtype=np.float64)
        elif "camera_matrix" in data:
            K = np.asarray(data["camera_matrix"], dtype=np.float64)
        else:
            raise KeyError(f"{path} must contain K or camera_matrix")

        if "dist" in data:
            dist = np.asarray(data["dist"], dtype=np.float64).reshape(-1)
        elif "dist_coeffs" in data:
            dist = np.asarray(data["dist_coeffs"], dtype=np.float64).reshape(-1)
        else:
            raise KeyError(f"{path} must contain dist or dist_coeffs")
    else:
        data = load_json(path)
        if "K" in data:
            K = np.asarray(data["K"], dtype=np.float64)
        elif "camera_matrix" in data:
            K = np.asarray(data["camera_matrix"], dtype=np.float64)
        elif "intrinsic_matrix" in data:
            K = np.asarray(data["intrinsic_matrix"], dtype=np.float64)
        else:
            raise KeyError(f"{path} must contain K/camera_matrix/intrinsic_matrix")

        if "dist" in data:
            dist = np.asarray(data["dist"], dtype=np.float64).reshape(-1)
        elif "dist_coeffs" in data:
            dist = np.asarray(data["dist_coeffs"], dtype=np.float64).reshape(-1)
        elif "distortion_coefficients" in data:
            dist = np.asarray(data["distortion_coefficients"], dtype=np.float64).reshape(-1)
        else:
            raise KeyError(f"{path} must contain dist/dist_coeffs/distortion_coefficients")

    if K.shape != (3, 3):
        raise ValueError(f"K must be 3x3, got {K.shape}")
    if dist.size < 4:
        raise ValueError(f"dist must have at least 4 values, got {dist.size}")
    return K, dist


def load_samples_pose_map(samples_jsonl: Path, pose_translation_scale: float) -> dict[str, np.ndarray]:
    pose_map: dict[str, np.ndarray] = {}
    with samples_jsonl.open("r", encoding="utf-8") as f:
        for line_idx, line in enumerate(f, start=1):
            line = line.strip()
            if not line:
                continue
            row = json.loads(line)
            image_path = row.get("ImagePath") or row.get("image_path")
            T_raw = row.get("TBaseTool") or row.get("T_base_tool")
            if image_path is None or T_raw is None:
                raise KeyError(f"Bad samples row at line {line_idx}: need ImagePath and TBaseTool")
            T = np.asarray(T_raw, dtype=np.float64)
            if T.shape != (4, 4):
                raise ValueError(f"TBaseTool for {image_path} must be 4x4, got {T.shape}")
            T = T.copy()
            T[:3, 3] *= pose_translation_scale
            pose_map[Path(image_path).name] = T
    return pose_map


def sorted_images(folder: Path) -> list[Path]:
    def key(path: Path) -> tuple[int, int | str]:
        try:
            return (0, int(path.stem))
        except ValueError:
            return (1, path.name.lower())

    return sorted([p for p in folder.iterdir() if p.suffix.lower() in IMAGE_EXTS], key=key)


def undistort_and_save(
    src: Path,
    dst: Path,
    K: np.ndarray,
    dist: np.ndarray,
    alpha: float,
    new_K_cache: dict[str, np.ndarray],
) -> np.ndarray:
    image = cv2.imread(str(src), cv2.IMREAD_COLOR)
    if image is None:
        raise FileNotFoundError(f"Failed to read image: {src}")

    h, w = image.shape[:2]
    cache_key = f"{w}x{h}"
    if cache_key not in new_K_cache:
        new_K, roi = cv2.getOptimalNewCameraMatrix(K, dist, (w, h), alpha=alpha, newImgSize=(w, h))
        new_K_cache[cache_key] = new_K.astype(np.float32)
        print(f"[Pi3-conditions] undistort image_size={w}x{h}")
        print("[Pi3-conditions] old K:\n", K)
        print("[Pi3-conditions] new K:\n", new_K)
        print("[Pi3-conditions] roi:", roi)

    new_K = new_K_cache[cache_key]
    undistorted = cv2.undistort(image, K, dist, None, new_K)
    dst.parent.mkdir(parents=True, exist_ok=True)
    ok = cv2.imwrite(str(dst), undistorted)
    if not ok:
        raise RuntimeError(f"Failed to write image: {dst}")
    return new_K


def prepare_pi3_conditions(
    source_dir: Path,
    samples_jsonl: Path,
    hand_eye: Path,
    hand_eye_key: str,
    calib: Path,
    out_dir: Path,
    num_frames: int,
    start_index: int = 0,
    frame_stride: int = 1,
    pose_translation_scale: float = 1.0,
    undistort: bool = True,
    undistort_alpha: float = 0.1,
) -> tuple[Path, Path]:
    source_dir = source_dir.resolve()
    samples_jsonl = samples_jsonl.resolve()
    hand_eye = hand_eye.resolve()
    calib = calib.resolve()
    out_dir = out_dir.resolve()
    image_dir = out_dir / "images"
    condition_npz = out_dir / "condition_robot_poses_intrinsics.npz"

    out_dir.mkdir(parents=True, exist_ok=True)
    image_dir.mkdir(parents=True, exist_ok=True)

    pose_map = load_samples_pose_map(samples_jsonl, pose_translation_scale)
    T_tool_camera = load_npz_or_json_matrix(hand_eye, hand_eye_key)
    T_tool_camera = T_tool_camera.copy()
    T_tool_camera[:3, 3] *= pose_translation_scale
    K_raw, dist = load_camera_calibration(calib)

    images = sorted_images(source_dir)
    selected = images[start_index : start_index + num_frames * frame_stride : frame_stride]
    if not selected:
        raise RuntimeError(f"No selected images from {source_dir}")

    poses: list[np.ndarray] = []
    intrinsics: list[np.ndarray] = []
    image_names: list[str] = []
    new_K_cache: dict[str, np.ndarray] = {}

    for src in selected:
        if src.name not in pose_map:
            raise KeyError(f"{src.name} not found in {samples_jsonl}")

        T_base_tool = pose_map[src.name]
        T_base_camera = T_base_tool @ T_tool_camera
        dst = image_dir / src.name
        if undistort:
            K_for_pi3 = undistort_and_save(src, dst, K_raw, dist, undistort_alpha, new_K_cache)
        else:
            shutil.copy2(src, dst)
            K_for_pi3 = K_raw

        poses.append(T_base_camera.astype(np.float32))
        intrinsics.append(K_for_pi3.astype(np.float32))
        image_names.append(src.name)

    poses_arr = np.stack(poses, axis=0).astype(np.float32)
    intrinsics_arr = np.stack(intrinsics, axis=0).astype(np.float32)
    np.savez(condition_npz, poses=poses_arr, intrinsics=intrinsics_arr, image_names=np.asarray(image_names))

    manifest = {
        "source_dir": str(source_dir),
        "samples_jsonl": str(samples_jsonl),
        "hand_eye": str(hand_eye),
        "hand_eye_key": hand_eye_key,
        "calib": str(calib),
        "out_image_dir": str(image_dir),
        "condition_npz": str(condition_npz),
        "start_index": start_index,
        "num_frames": len(image_names),
        "frame_stride": frame_stride,
        "pose_translation_scale": pose_translation_scale,
        "undistort": undistort,
        "undistort_alpha": undistort_alpha,
        "image_names": image_names,
        "raw_intrinsics": K_raw.tolist(),
        "dist_coeffs": dist.tolist(),
        "intrinsics_first": intrinsics_arr[0].tolist(),
        "pose_first": poses_arr[0].tolist(),
    }
    (out_dir / "condition_manifest.json").write_text(
        json.dumps(manifest, ensure_ascii=False, indent=2),
        encoding="utf-8",
    )

    print("[Step2-Pi3A] generated conditions")
    print("[Step2-Pi3A] images:", len(image_names))
    print("[Step2-Pi3A] image_dir:", image_dir)
    print("[Step2-Pi3A] condition:", condition_npz)
    print("[Step2-Pi3A] poses:", poses_arr.shape, poses_arr.dtype)
    print("[Step2-Pi3A] intrinsics:", intrinsics_arr.shape, intrinsics_arr.dtype)
    return image_dir, condition_npz


def main() -> None:
    args = parse_args()
    source_dir = args.source_dir.resolve()
    frame_stride = max(1, args.frame_stride)
    if args.all_frames:
        num_frames = count_image_files(source_dir)
    elif args.num_frames is not None:
        num_frames = args.num_frames
    else:
        num_frames = count_image_files(source_dir)

    out_dir = args.out_dir.resolve() if args.out_dir else default_pi3_out_dir(source_dir, num_frames, frame_stride)
    prepare_pi3_conditions(
        source_dir=source_dir,
        samples_jsonl=args.samples_jsonl,
        hand_eye=args.hand_eye,
        hand_eye_key=args.hand_eye_key,
        calib=args.calib,
        out_dir=out_dir,
        num_frames=num_frames,
        start_index=args.start_index,
        frame_stride=frame_stride,
        pose_translation_scale=args.pose_translation_scale,
        undistort=args.undistort,
        undistort_alpha=args.undistort_alpha,
    )


if __name__ == "__main__":
    main()
