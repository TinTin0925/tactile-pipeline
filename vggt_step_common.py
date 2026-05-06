from __future__ import annotations

import importlib.util
from pathlib import Path
from types import ModuleType


TACTILE_DIR = Path(__file__).resolve().parent
SRC_DIR = TACTILE_DIR.parent
CAMERA_DIR = SRC_DIR / "camera"
VGGT_PIPELINE_PATH = CAMERA_DIR / "vggt_viser_pipeline.py"

DEFAULT_DATA_DIR = TACTILE_DIR / "data"
DEFAULT_RECON_DATA_DIR = DEFAULT_DATA_DIR / "liver_data"
DEFAULT_SOURCE_DIR = DEFAULT_RECON_DATA_DIR / "frames"
DEFAULT_POLARIS_JSONL = DEFAULT_RECON_DATA_DIR / "samples.jsonl"
DEFAULT_CALIB_DIR = DEFAULT_DATA_DIR / "calib_result"
DEFAULT_HAND_EYE = DEFAULT_DATA_DIR / "handeye_compare" / "calib_handeye_compare.npz"
DEFAULT_RUNS_DIR = CAMERA_DIR / "runs"
DEFAULT_SAFE_NUM_FRAMES = 10


def count_image_files(source_dir: Path) -> int:
    exts = {".png", ".jpg", ".jpeg", ".bmp", ".tif", ".tiff"}
    return sum(1 for p in source_dir.iterdir() if p.is_file() and p.suffix.lower() in exts)


def default_num_frames(source_dir: Path) -> int:
    n = count_image_files(source_dir)
    if n < 1:
        raise FileNotFoundError(f"No image files found in {source_dir}")
    return min(n, DEFAULT_SAFE_NUM_FRAMES)


def default_vggt_out_dir(source_dir: Path, num_frames: int, stride: int = 1) -> Path:
    tag = f"_n{num_frames}" if stride <= 1 else f"_n{num_frames}_s{stride}"
    parent_tag = source_dir.parent.name if source_dir.parent.name else "images"
    return DEFAULT_RUNS_DIR / f"vggt_viser_export_{parent_tag}_{source_dir.name}{tag}"


def load_vggt_pipeline() -> ModuleType:
    if not VGGT_PIPELINE_PATH.exists():
        raise FileNotFoundError(f"Cannot find VGGT pipeline: {VGGT_PIPELINE_PATH}")

    spec = importlib.util.spec_from_file_location("vggt_viser_pipeline", VGGT_PIPELINE_PATH)
    if spec is None or spec.loader is None:
        raise RuntimeError(f"Cannot import VGGT pipeline from {VGGT_PIPELINE_PATH}")

    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module
