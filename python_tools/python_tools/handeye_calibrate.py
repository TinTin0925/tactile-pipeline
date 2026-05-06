import argparse
from dataclasses import dataclass
from pathlib import Path
from typing import List, Optional, Tuple

import cv2
import numpy as np

from handeye_io import load_json, load_jsonl, save_json, save_npz
from handeye_transforms import split_rt, make_T, inv_T


@dataclass
class Sample:
    """
    职责：承载一条有效样本。
    """
    index: int
    timestamp: float
    image_path: str
    T_base_tool: np.ndarray
    T_camera_board: np.ndarray


def chessboard_pose_C_T_B(
    img_bgr: np.ndarray,
    K: np.ndarray,
    dist: np.ndarray,
    board_size: Tuple[int, int],
    square_size_m: float,
) -> Optional[np.ndarray]:
    """
    职责：从棋盘格图像估计 ^C T_B。
    这是从你们现有 hand_eye_calibration_test.py 提炼出来的。
    """
    gray = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2GRAY)

    ok, corners = cv2.findChessboardCorners(gray, board_size)
    if not ok:
        return None

    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 50, 1e-6)
    corners = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)

    cols, rows = board_size
    objp = np.zeros((rows * cols, 3), dtype=np.float64)
    objp[:, :2] = np.mgrid[0:cols, 0:rows].T.reshape(-1, 2)
    objp *= square_size_m

    ok, rvec, tvec = cv2.solvePnP(objp, corners, K, dist, flags=cv2.SOLVEPNP_ITERATIVE)
    if not ok:
        return None

    R, _ = cv2.Rodrigues(rvec)
    t = tvec.reshape(3, 1)
    return make_T(R, t)


def load_valid_samples(
    samples_jsonl: str,
    intrinsics_json: str,
    board_cols: int,
    board_rows: int,
    square_size_m: float,
) -> List[Sample]:
    """
    职责：读取 samples.jsonl，并筛掉无法成功检测标定板的样本。
    """
    rows = load_jsonl(samples_jsonl)
    intr = load_json(intrinsics_json)

    K = np.asarray(intr["K"], dtype=np.float64)
    dist = np.asarray(intr["dist"], dtype=np.float64)

    board_size = (board_cols, board_rows)
    valid: List[Sample] = []

    for row in rows:
        image_path = row["image_path"]
        img = cv2.imread(image_path, cv2.IMREAD_COLOR)
        if img is None:
            continue

        T_camera_board = chessboard_pose_C_T_B(
            img_bgr=img,
            K=K,
            dist=dist,
            board_size=board_size,
            square_size_m=square_size_m,
        )
        if T_camera_board is None:
            continue

        T_base_tool = np.asarray(row["T_base_tool"], dtype=np.float64)
        valid.append(
            Sample(
                index=int(row["index"]),
                timestamp=float(row["timestamp"]),
                image_path=image_path,
                T_base_tool=T_base_tool,
                T_camera_board=T_camera_board,
            )
        )

    return valid


def compute_consistency_score(samples: List[Sample], X_tool_camera: np.ndarray) -> float:
    """
    职责：计算给定手眼矩阵下的相对运动一致性误差。
    这是把你们现有脚本里的一致性评分逻辑独立出来。
    """
    if len(samples) < 2:
        return float("inf")

    errs = []
    for i in range(len(samples) - 1):
        A = inv_T(samples[i].T_base_tool) @ samples[i + 1].T_base_tool
        B = inv_T(samples[i].T_camera_board) @ samples[i + 1].T_camera_board

        left = A @ X_tool_camera
        right = X_tool_camera @ B
        d = inv_T(left) @ right

        t = d[:3, 3]
        R = d[:3, :3]
        angle = np.arccos(np.clip((np.trace(R) - 1) / 2.0, -1.0, 1.0))
        errs.append(np.linalg.norm(t) + float(angle))

    return float(np.mean(errs))


def solve_handeye(samples: List[Sample]) -> dict:
    """
    职责：根据有效样本求解手眼矩阵，并返回结果字典。
    """
    if len(samples) < 8:
        raise RuntimeError(f"Too few valid samples for hand-eye: {len(samples)} (need >= 8, recommended 15-30)")

    R_gripper2base, t_gripper2base = [], []
    R_target2cam, t_target2cam = [], []

    for s in samples:
        Rg, tg = split_rt(s.T_base_tool)   # ^B T_T
        Rt, tt = split_rt(s.T_camera_board)  # ^C T_Board

        R_gripper2base.append(Rg)
        t_gripper2base.append(tg)
        R_target2cam.append(Rt)
        t_target2cam.append(tt)

    method = cv2.CALIB_HAND_EYE_TSAI

    R_cam2gripper, t_cam2gripper = cv2.calibrateHandEye(
        R_gripper2base, t_gripper2base,
        R_target2cam, t_target2cam,
        method=method
    )

    T_cam2gripper = make_T(R_cam2gripper, t_cam2gripper)

    # 两个候选方向
    X1 = inv_T(T_cam2gripper)   # 候选：^T T_C
    X2 = T_cam2gripper          # 候选：反向情况

    e1 = compute_consistency_score(samples, X1)
    e2 = compute_consistency_score(samples, X2)

    X = X1 if e1 <= e2 else X2
    chosen_score = min(e1, e2)

    return {
        "num_valid_samples": len(samples),
        "method": "CALIB_HAND_EYE_TSAI",
        "candidate_score_1": e1,
        "candidate_score_2": e2,
        "chosen_score": chosen_score,
        "T_tool_camera": X.tolist(),
    }


def main():
    parser = argparse.ArgumentParser(description="Solve hand-eye calibration from robot samples and images.")
    parser.add_argument("--samples-jsonl", required=True, help="Path to samples.jsonl")
    parser.add_argument("--intrinsics-json", required=True, help="Path to camera intrinsics json")
    parser.add_argument("--board-cols", type=int, default=9, help="Chessboard inner corners cols")
    parser.add_argument("--board-rows", type=int, default=6, help="Chessboard inner corners rows")
    parser.add_argument("--square-size", type=float, default=0.024, help="Chessboard square size in meters")
    parser.add_argument("--output-dir", required=True, help="Output directory")
    args = parser.parse_args()

    out_dir = Path(args.output_dir)
    out_dir.mkdir(parents=True, exist_ok=True)

    valid_samples = load_valid_samples(
        samples_jsonl=args.samples_jsonl,
        intrinsics_json=args.intrinsics_json,
        board_cols=args.board_cols,
        board_rows=args.board_rows,
        square_size_m=args.square_size,
    )

    result = solve_handeye(valid_samples)

    save_json(out_dir / "result.json", result)
    save_npz(out_dir / "result.npz", T_tool_camera=np.asarray(result["T_tool_camera"], dtype=np.float64))

    print(f"[handeye] valid_samples = {result['num_valid_samples']}")
    print(f"[handeye] chosen_score = {result['chosen_score']:.6f}")
    print(f"[handeye] result saved to {out_dir / 'result.json'}")


if __name__ == "__main__":
    main()