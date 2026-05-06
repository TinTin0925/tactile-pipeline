import argparse
from pathlib import Path
from typing import List

import numpy as np

from handeye_calibrate import load_valid_samples, compute_consistency_score
from handeye_io import load_json, save_json


def main():
    parser = argparse.ArgumentParser(description="Generate consistency report for an existing hand-eye result.")
    parser.add_argument("--samples-jsonl", required=True, help="Path to samples.jsonl")
    parser.add_argument("--intrinsics-json", required=True, help="Path to camera intrinsics json")
    parser.add_argument("--result-json", required=True, help="Path to hand-eye result.json")
    parser.add_argument("--board-cols", type=int, default=9)
    parser.add_argument("--board-rows", type=int, default=6)
    parser.add_argument("--square-size", type=float, default=0.024)
    parser.add_argument("--output-json", required=True, help="Where to save report json")
    args = parser.parse_args()

    valid_samples = load_valid_samples(
        samples_jsonl=args.samples_jsonl,
        intrinsics_json=args.intrinsics_json,
        board_cols=args.board_cols,
        board_rows=args.board_rows,
        square_size_m=args.square_size,
    )

    result = load_json(args.result_json)
    T_tool_camera = np.asarray(result["T_tool_camera"], dtype=np.float64)

    score = compute_consistency_score(valid_samples, T_tool_camera)

    report = {
        "num_valid_samples": len(valid_samples),
        "consistency_score": float(score),
        "result_json": str(Path(args.result_json).resolve()),
        "samples_jsonl": str(Path(args.samples_jsonl).resolve()),
    }

    save_json(args.output_json, report)

    print(f"[report] valid_samples = {report['num_valid_samples']}")
    print(f"[report] consistency_score = {report['consistency_score']:.6f}")
    print(f"[report] saved to {args.output_json}")


if __name__ == "__main__":
    main()