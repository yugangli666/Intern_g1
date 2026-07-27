"""Small, local run logger used by the standalone scripts."""

from __future__ import annotations

import json
from datetime import datetime
from pathlib import Path
from typing import Any

import numpy as np
from PIL import Image


def _json_default(value: Any):
    if isinstance(value, np.ndarray):
        return value.tolist()
    if isinstance(value, np.generic):
        return value.item()
    if isinstance(value, Path):
        return str(value)
    raise TypeError(f"cannot serialize {type(value).__name__}")


class RunLogger:
    def __init__(self, root: str | Path, prefix: str, metadata: dict[str, Any] | None = None):
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.run_dir = Path(root) / f"{prefix}_{timestamp}"
        self.run_dir.mkdir(parents=True, exist_ok=False)
        self.rgb_dir = self.run_dir / "rgb"
        self.depth_dir = self.run_dir / "depth"
        self.rgb_dir.mkdir()
        self.depth_dir.mkdir()
        self.events = (self.run_dir / "events.jsonl").open("a", encoding="utf-8")
        (self.run_dir / "meta.json").write_text(
            json.dumps(metadata or {}, ensure_ascii=False, indent=2, default=_json_default) + "\n", encoding="utf-8"
        )
        self.index = 0

    def log(self, event: dict[str, Any], rgb: np.ndarray | None = None, depth_m: np.ndarray | None = None) -> None:
        self.index += 1
        record = {"index": self.index, "timestamp": datetime.now().isoformat(timespec="milliseconds"), **event}
        if rgb is not None:
            rgb_path = self.rgb_dir / f"{self.index:06d}.jpg"
            Image.fromarray(np.asarray(rgb, dtype=np.uint8)).convert("RGB").save(rgb_path, quality=92)
            record["rgb_path"] = str(rgb_path.relative_to(self.run_dir))
        if depth_m is not None:
            depth_path = self.depth_dir / f"{self.index:06d}.png"
            depth_mm = np.clip(np.asarray(depth_m) * 1000.0, 0, 65535).astype(np.uint16)
            Image.fromarray(depth_mm).save(depth_path)
            record["depth_path"] = str(depth_path.relative_to(self.run_dir))
        self.events.write(json.dumps(record, ensure_ascii=False, default=_json_default) + "\n")
        self.events.flush()

    def close(self) -> None:
        self.events.close()
