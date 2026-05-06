import json
from pathlib import Path
from typing import Any, Dict, List

import numpy as np


def ensure_dir(path: Path) -> Path:
    """
    职责：确保目录存在。
    """
    path.mkdir(parents=True, exist_ok=True)
    return path


def create_run_dir(base_dir: str = "runs") -> Path:
    """
    职责：创建一次 hand-eye 运行目录。
    """
    from datetime import datetime

    base = Path(base_dir)
    ensure_dir(base)
    run_dir = base / f"handeye_{datetime.now().strftime('%Y%m%d_%H%M%S')}"
    ensure_dir(run_dir)
    ensure_dir(run_dir / "frames")
    return run_dir


def save_json(path: str | Path, obj: Dict[str, Any]) -> None:
    """
    职责：保存普通 JSON。
    """
    path = Path(path)
    path.write_text(json.dumps(obj, indent=2, ensure_ascii=False), encoding="utf-8")


def load_json(path: str | Path) -> Dict[str, Any]:
    """
    职责：读取普通 JSON。
    """
    path = Path(path)
    return json.loads(path.read_text(encoding="utf-8"))


def append_jsonl(path: str | Path, obj: Dict[str, Any]) -> None:
    """
    职责：向 JSONL 文件追加一条记录。
    """
    path = Path(path)
    with path.open("a", encoding="utf-8") as f:
        f.write(json.dumps(obj, ensure_ascii=False) + "\n")


def load_jsonl(path: str | Path) -> List[Dict[str, Any]]:
    """
    职责：读取 JSONL 文件，返回记录列表。
    """
    path = Path(path)
    rows = []
    with path.open("r", encoding="utf-8") as f:
        for line in f:
            line = line.strip()
            if not line:
                continue
            rows.append(json.loads(line))
    return rows


def save_npz(path: str | Path, **kwargs) -> None:
    """
    职责：保存 numpy 结果文件。
    """
    path = Path(path)
    np.savez(str(path), **kwargs)