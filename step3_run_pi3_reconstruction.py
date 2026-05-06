from __future__ import annotations

import argparse
import os
import subprocess
import sys
from pathlib import Path

from vggt_step_common import count_image_files
from step2_prepare_pi3_conditions import default_pi3_out_dir, prepare_pi3_conditions
import run_pi3_session as _cfg


DEFAULT_PI3_ROOT = _cfg.PI3_ROOT
DEFAULT_PI3_PYTHON = _cfg.PI3_PYTHON


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Step 3: run Pi3X reconstruction with robot-pose conditions."
    )
    parser.add_argument("--source-dir", type=Path, default=_cfg.SOURCE_DIR)
    parser.add_argument("--samples-jsonl", type=Path, default=_cfg.SAMPLES_JSONL)
    parser.add_argument("--hand-eye", type=Path, default=_cfg.HAND_EYE)
    parser.add_argument("--hand-eye-key", default="T_tool_camera")
    parser.add_argument("--calib", type=Path, default=_cfg.CALIB)
    parser.add_argument("--out-dir", type=Path, default=None)
    parser.add_argument("--num-frames", type=int, default=None)
    parser.add_argument("--all-frames", action="store_true")
    parser.add_argument("--start-index", type=int, default=_cfg.START_INDEX)
    parser.add_argument("--frame-stride", type=int, default=_cfg.FRAME_STRIDE)
    parser.add_argument("--pose-translation-scale", type=float, default=1.0)
    parser.add_argument("--undistort", action=argparse.BooleanOptionalAction, default=_cfg.UNDISTORT)
    parser.add_argument("--undistort-alpha", type=float, default=_cfg.UNDISTORT_ALPHA)
    parser.add_argument("--force-conditions", action="store_true")

    parser.add_argument("--pi3-root", type=Path, default=DEFAULT_PI3_ROOT)
    parser.add_argument("--pi3-python", type=Path, default=DEFAULT_PI3_PYTHON)
    parser.add_argument("--pi3-script", type=Path, default=None)
    parser.add_argument("--device", default=_cfg.DEVICE)
    parser.add_argument(
        "--interval",
        type=int,
        default=_cfg.INTERVAL,
        help="Pi3 inference interval. Default: 1 for <=20 prepared frames, otherwise 2 for 8GB GPU safety.",
    )
    parser.add_argument("--conf-min", type=float, default=_cfg.CONF_MIN)
    parser.add_argument("--max-points", type=int, default=_cfg.MAX_POINTS)
    parser.add_argument("--filter-depth", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--depth-min", type=float, default=_cfg.DEPTH_MIN)
    parser.add_argument("--depth-max", type=float, default=_cfg.DEPTH_MAX)
    parser.add_argument("--filter-distance", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--distance-max", type=float, default=_cfg.DISTANCE_MAX)
    parser.add_argument("--filter-mode", choices=["any", "first", "all"], default="any")
    parser.add_argument("--align-to-condition-pose", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--align-allow-scale", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--save-raw-points", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--save-path", type=Path, default=None)
    parser.add_argument(
        "--project-rgb",
        action=argparse.BooleanOptionalAction,
        default=True,
        help="After Pi3, project the aligned cloud back into condition images to write a cleaner RGB PLY.",
    )
    parser.add_argument("--view", action="store_true")
    return parser.parse_args()


def run_command(cmd: list[str], cwd: Path, env: dict[str, str] | None = None) -> None:
    print("[Step3-Pi3]", " ".join(f'"{x}"' if " " in x else x for x in cmd))
    subprocess.run(cmd, check=True, cwd=str(cwd), env=env)


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
    image_dir = out_dir / "images"
    condition_npz = out_dir / "condition_robot_poses_intrinsics.npz"

    if args.force_conditions or not condition_npz.exists():
        image_dir, condition_npz = prepare_pi3_conditions(
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

    pi3_root = args.pi3_root.resolve()
    pi3_python = args.pi3_python.resolve()
    pi3_script = args.pi3_script.resolve() if args.pi3_script else pi3_root / "example_mm_aligned.py"
    if not pi3_python.exists():
        raise FileNotFoundError(f"Pi3 Python not found: {pi3_python}")
    if not pi3_script.exists():
        raise FileNotFoundError(f"Pi3 script not found: {pi3_script}")

    interval = args.interval
    if interval is None:
        interval = 1 if num_frames <= 20 else 2
    interval = max(1, interval)

    save_path = args.save_path.resolve() if args.save_path else out_dir / f"{out_dir.name}_i{interval}_aligned.ply"
    save_path.parent.mkdir(parents=True, exist_ok=True)

    cmd = [
        str(pi3_python),
        str(pi3_script),
        "--data_path",
        str(image_dir),
        "--conditions_path",
        str(condition_npz),
        "--save_path",
        str(save_path),
        "--device",
        args.device,
        "--interval",
        str(interval),
        "--conf_min",
        str(args.conf_min),
        "--max_points",
        str(args.max_points),
    ]
    if args.align_to_condition_pose:
        cmd.append("--align_to_condition_pose")
    if args.align_allow_scale:
        cmd.append("--align_allow_scale")
    if args.save_raw_points:
        cmd.append("--save_raw_points")
    if args.filter_depth:
        cmd.extend(["--filter_depth", "--depth_min", str(args.depth_min), "--depth_max", str(args.depth_max)])
    if args.filter_distance:
        cmd.extend(["--filter_distance", "--distance_max", str(args.distance_max)])
    cmd.extend(["--filter_mode", args.filter_mode])

    env = os.environ.copy()
    env.setdefault("PYTORCH_CUDA_ALLOC_CONF", "expandable_segments:True")
    print("[Step3-Pi3] image_dir:", image_dir)
    print("[Step3-Pi3] condition:", condition_npz)
    print("[Step3-Pi3] save_path:", save_path)
    print("[Step3-Pi3] interval:", interval)
    run_command(cmd, cwd=pi3_root, env=env)

    final_path = save_path
    if args.project_rgb:
        projected_path = save_path.with_name(f"{save_path.stem}_projected_rgb.ply")
        colorize_script = Path(__file__).resolve().parent / "colorize_pointcloud_from_images.py"
        color_cmd = [
            str(pi3_python),
            str(colorize_script),
            str(save_path),
            "--image-dir",
            str(image_dir),
            "--conditions",
            str(condition_npz),
            "--output",
            str(projected_path),
            "--interval",
            str(interval),
            "--depth-min",
            str(args.depth_min),
            "--depth-max",
            str(args.depth_max),
            "--mode",
            "nearest",
        ]
        run_command(color_cmd, cwd=Path(__file__).resolve().parent)
        final_path = projected_path

    if args.view:
        view_script = Path(__file__).resolve().parent / "view_pointcloud.py"
        view_cmd = [
            sys.executable,
            str(view_script),
            str(final_path),
            "--point-size",
            "3",
            "--background",
            "255,255,255",
            "--show-frame",
        ]
        run_command(view_cmd, cwd=Path(__file__).resolve().parent)

    print("[Step3-Pi3] done")
    print("[Step3-Pi3] points:", final_path)
    print("[Step3-Pi3] raw/aligned:", save_path)
    print("[Step3-Pi3] alignment:", save_path.with_name(f"{save_path.stem}_alignment.json"))


if __name__ == "__main__":
    main()
