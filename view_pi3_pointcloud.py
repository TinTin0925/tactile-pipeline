from __future__ import annotations

import subprocess
import sys
from pathlib import Path


TACTILE_DIR = Path(__file__).resolve().parent
DEFAULT_PLY = Path(
    r"E:\research\FYP\5.4\Pi3\runs\session07_condition"
    r"\session07_20_pi3x_undist_filtered_aligned_projected_rgb.ply"
)


def main() -> None:
    ply = Path(sys.argv[1]).resolve() if len(sys.argv) > 1 else DEFAULT_PLY
    if not ply.exists():
        raise FileNotFoundError(f"Point cloud not found: {ply}")

    cmd = [
        sys.executable,
        str(TACTILE_DIR / "view_pointcloud.py"),
        str(ply),
        "--point-size",
        "3",
        "--background",
        "255,255,255",
        "--show-frame",
    ]
    print("[ViewPi3] running:")
    print(" ".join(f'"{x}"' if " " in x else x for x in cmd))
    subprocess.run(cmd, check=True, cwd=str(TACTILE_DIR))


if __name__ == "__main__":
    main()
