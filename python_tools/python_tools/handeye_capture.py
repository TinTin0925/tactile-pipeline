import argparse
import time
from pathlib import Path
from typing import List

import cv2
import numpy as np

from handeye_io import append_jsonl, create_run_dir
from handeye_transforms import pose6d_to_T


def get_current_robot_pose_placeholder() -> np.ndarray:
    """
    职责：占位函数，返回当前机器人 TCP 位姿 4x4。
    后续替换成：
    - 从 C# 导出的当前 pose
    - 或 Python 直接连机器人读取
    """
    # 这里先返回单位阵，方便先打通流程
    return np.eye(4, dtype=np.float64)


def capture_frame_placeholder(save_path: Path) -> float:
    """
    职责：占位函数，采一张图并返回时间戳。
    后续替换成实际相机抓拍逻辑。
    """
    # 先生成一张黑图，便于流程调试
    img = np.zeros((720, 1280, 3), dtype=np.uint8)
    cv2.imwrite(str(save_path), img)
    return time.time()


def load_joint_points_from_txt(path: str) -> List[List[float]]:
    """
    职责：读取一组关节点。
    文本格式每行 6 个角度值（单位先约定为度）。
    """
    points = []
    with open(path, "r", encoding="utf-8") as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith("#"):
                continue
            vals = [float(x) for x in line.replace(",", " ").split()]
            if len(vals) != 6:
                raise ValueError(f"Each line must contain 6 values, got: {line}")
            points.append(vals)
    return points


def main():
    parser = argparse.ArgumentParser(description="Semi-auto hand-eye dataset capture.")
    parser.add_argument("--joint-points", required=True, help="Path to joint points txt")
    parser.add_argument("--base-dir", default="runs", help="Base directory for runs")
    parser.add_argument("--dwell-sec", type=float, default=1.5, help="Wait time before capture")
    parser.add_argument("--semi-auto", action="store_true", help="Wait for Enter before each capture")
    args = parser.parse_args()

    joint_points = load_joint_points_from_txt(args.joint_points)
    run_dir = create_run_dir(args.base_dir)
    samples_jsonl = run_dir / "samples.jsonl"
    frames_dir = run_dir / "frames"

    print(f"[capture] run_dir = {run_dir}")
    print(f"[capture] total points = {len(joint_points)}")

    for i, q_deg in enumerate(joint_points):
        print("=" * 50)
        print(f"[capture] point {i}")
        print(f"[capture] target joints (deg) = {q_deg}")

        # TODO:
        # 这里后续接入真实机器人 moveJ
        # 当前先只做流程占位
        print("[capture] TODO: move robot to target joint pose")

        time.sleep(args.dwell_sec)

        if args.semi_auto:
            cmd = input("Press Enter to capture, or 'q' to quit: ").strip().lower()
            if cmd == "q":
                break

        image_path = frames_dir / f"{i:04d}.png"
        timestamp = capture_frame_placeholder(image_path)
        T_base_tool = get_current_robot_pose_placeholder()

        row = {
            "index": i,
            "timestamp": timestamp,
            "image_path": str(image_path),
            "T_base_tool": T_base_tool.tolist(),
        }
        append_jsonl(samples_jsonl, row)

        print(f"[capture] saved image = {image_path}")
        print(f"[capture] appended sample = {samples_jsonl}")

    print("[capture] finished.")


if __name__ == "__main__":
    main()