from __future__ import annotations

import argparse
import subprocess
import sys
from pathlib import Path

from vggt_step_common import DEFAULT_RUNS_DIR


def newest_pi3_cloud() -> Path:
    patterns = [
        "pi3_*/*projected_rgb.ply",
        "pi3_*/*aligned.ply",
        "pi3_*/*.ply",
    ]
    candidates: list[Path] = []
    for pattern in patterns:
        candidates.extend(DEFAULT_RUNS_DIR.glob(pattern))
    candidates = [p for p in candidates if p.is_file()]
    if not candidates:
        fallback = Path(
            r"E:\research\FYP\5.4\Pi3\runs\session07_condition"
            r"\session07_20_pi3x_undist_filtered_aligned_projected_rgb.ply"
        )
        if fallback.exists():
            return fallback
        raise FileNotFoundError("No Pi3 point cloud found. Run step3_run_pi3_reconstruction.py first.")
    return max(candidates, key=lambda p: p.stat().st_mtime)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Step 4: view the latest Pi3 point cloud.")
    parser.add_argument("ply", nargs="?", type=Path, default=None)
    parser.add_argument("--point-size", type=float, default=3.0)
    parser.add_argument("--background", default="255,255,255")
    parser.add_argument("--show-frame", action=argparse.BooleanOptionalAction, default=True)
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    ply = args.ply.resolve() if args.ply else newest_pi3_cloud().resolve()
    if not ply.exists():
        raise FileNotFoundError(f"Point cloud not found: {ply}")

    cmd = [
        sys.executable,
        str(Path(__file__).resolve().parent / "view_pointcloud.py"),
        str(ply),
        "--point-size",
        str(args.point_size),
        "--background",
        args.background,
    ]
    if args.show_frame:
        cmd.append("--show-frame")

    print("[Step4-View] viewing:", ply)
    subprocess.run(cmd, check=True, cwd=str(Path(__file__).resolve().parent))


if __name__ == "__main__":
    main()
