import json
import platform
import socket
import subprocess
import sys
from datetime import datetime
from pathlib import Path

import numpy as np
from PIL import Image


FAILURE_TYPES = {
    "target_misrecognition": "The model recognized the wrong target object or landmark.",
    "wrong_turn": "The robot turned in the wrong direction.",
    "early_stop": "The robot stopped too early before reaching the target.",
    "late_stop": "The robot stopped too late after passing the target.",
    "no_stop": "The robot did not stop when it should have stopped.",
    "collision_risk": "The robot moved too close to obstacles or had collision risk.",
    "unstable_motion": "The robot motion was unstable, shaking, drifting, or not smooth.",
    "depth_error": "The failure seems related to wrong depth perception or obstacle distance estimation.",
    "instruction_error": "The model misunderstood the language instruction.",
    "system_error": "The failure was caused by camera, network, server, ROS, DDS, or control issues.",
    "other": "The failure does not fit the above categories.",
}


def _now_text():
    return datetime.now().strftime("%Y-%m-%d %H:%M:%S")


def _json_default(value):
    if isinstance(value, np.ndarray):
        return value.tolist()
    if isinstance(value, np.generic):
        return value.item()
    if isinstance(value, Path):
        return str(value)
    return str(value)


class NavigationLogger:
    def __init__(
        self,
        log_root,
        instruction,
        model_name="InternVLA-N1-w-NavDP",
        robot="Unitree G1",
        camera="RealSense D455",
        server_url=None,
    ):
        self.enabled = True
        self.step = 0
        self.instruction = instruction
        self.model_name = model_name
        self.robot = robot
        self.camera = camera
        self.server_url = server_url
        self.start_time = _now_text()
        self.end_time = None
        self.run_id = datetime.now().strftime("run_%Y%m%d_%H%M%S")
        self.actions_file = None

        try:
            self.log_root = Path(log_root or "./logs").expanduser()
            self.run_dir = self._make_run_dir(self.log_root, self.run_id)
            self.rgb_dir = self.run_dir / "rgb"
            self.depth_dir = self.run_dir / "depth"
            self.rgb_dir.mkdir(parents=True, exist_ok=True)
            self.depth_dir.mkdir(parents=True, exist_ok=True)
            self.actions_path = self.run_dir / "actions.jsonl"
            self.meta_path = self.run_dir / "meta.json"
            self.result_path = self.run_dir / "result.txt"
            self.actions_file = self.actions_path.open("a", encoding="utf-8")
            self.meta = self._build_meta()
            self._write_meta()
            print(f"[LOG] Navigation log folder: {self.run_dir}")
        except Exception as exc:
            self.enabled = False
            self.run_dir = None
            self.rgb_dir = None
            self.depth_dir = None
            self.actions_path = None
            self.meta_path = None
            self.result_path = None
            self.meta = {}
            print(f"[LOG][WARN] Failed to initialize navigation logger: {exc}")

    def _make_run_dir(self, log_root, run_id):
        log_root.mkdir(parents=True, exist_ok=True)
        run_dir = log_root / run_id
        suffix = 1
        while run_dir.exists():
            run_dir = log_root / f"{run_id}_{suffix:02d}"
            suffix += 1
        run_dir.mkdir(parents=True, exist_ok=False)
        self.run_id = run_dir.name
        return run_dir

    def _build_meta(self):
        return {
            "run_id": self.run_id,
            "start_time": self.start_time,
            "end_time": None,
            "model_name": self.model_name,
            "robot": self.robot,
            "camera": self.camera,
            "instruction": self.instruction,
            "server_url": self.server_url,
            "git_commit": self._get_git_commit(),
            "system_info": {
                "hostname": socket.gethostname(),
                "platform": platform.platform(),
                "python_version": sys.version,
            },
            "success": None,
            "failure_type": None,
            "notes": "",
            "total_steps": 0,
        }

    def _get_git_commit(self):
        repo_root = Path(__file__).resolve().parents[2]
        try:
            result = subprocess.run(
                ["git", "rev-parse", "HEAD"],
                cwd=repo_root,
                check=True,
                capture_output=True,
                text=True,
                timeout=2,
            )
            return result.stdout.strip() or None
        except Exception:
            return None

    def _write_meta(self):
        if not self.enabled or self.meta_path is None:
            return
        try:
            with self.meta_path.open("w", encoding="utf-8") as f:
                json.dump(self.meta, f, ensure_ascii=False, indent=2, default=_json_default)
                f.write("\n")
        except Exception as exc:
            print(f"[LOG][WARN] Failed to write meta.json: {exc}")

    def log_step(
        self,
        rgb_image=None,
        depth_image=None,
        model_response=None,
        model_action=None,
        executed_action=None,
        raw_response=None,
        latency_ms=None,
    ):
        if not self.enabled:
            return

        self.step += 1
        step = self.step
        rgb_path = self._safe_save_rgb(rgb_image, step)
        depth_path = self._safe_save_depth(depth_image, step)
        record = {
            "step": step,
            "timestamp": _now_text(),
            "rgb_path": rgb_path,
            "depth_path": depth_path,
            "instruction": self.instruction,
            "model_response": model_response,
            "model_action": model_action,
            "executed_action": executed_action,
            "raw_response": raw_response,
            "latency_ms": latency_ms,
        }

        try:
            if self.actions_file is not None:
                self.actions_file.write(json.dumps(record, ensure_ascii=False, default=_json_default) + "\n")
                self.actions_file.flush()
        except Exception as exc:
            print(f"[LOG][WARN] Failed to write actions.jsonl: {exc}")

        self.meta["total_steps"] = self.step
        self._write_meta()

    def _safe_save_rgb(self, image, step):
        if image is None:
            return None
        rel_path = Path("rgb") / f"{step:06d}.jpg"
        out_path = self.run_dir / rel_path
        try:
            pil_image = self._to_pil_rgb(image)
            pil_image.save(out_path, format="JPEG", quality=95)
            return rel_path.as_posix()
        except Exception as exc:
            print(f"[LOG][WARN] Failed to save RGB image step={step}: {exc}")
            return None

    def _safe_save_depth(self, image, step):
        if image is None:
            return None
        rel_path = Path("depth") / f"{step:06d}.png"
        out_path = self.run_dir / rel_path
        try:
            depth = self._to_depth_uint16(image)
            Image.fromarray(depth).save(out_path, format="PNG")
            return rel_path.as_posix()
        except Exception as exc:
            print(f"[LOG][WARN] Failed to save depth image step={step}: {exc}")
            return None

    def _to_pil_rgb(self, image):
        if isinstance(image, Image.Image):
            return image.convert("RGB")
        arr = np.asarray(image)
        if arr.ndim == 2:
            return Image.fromarray(self._to_uint8(arr)).convert("RGB")
        if arr.ndim != 3 or arr.shape[2] < 3:
            raise ValueError(f"unsupported RGB image shape: {arr.shape}")
        arr = self._to_uint8(arr[:, :, :3])
        return Image.fromarray(arr, mode="RGB")

    def _to_uint8(self, arr):
        arr = np.asarray(arr)
        if arr.dtype == np.uint8:
            return arr
        arr = np.nan_to_num(arr.astype(np.float32), nan=0.0, posinf=255.0, neginf=0.0)
        if arr.max(initial=0) <= 1.0:
            arr = arr * 255.0
        return np.clip(arr, 0, 255).astype(np.uint8)

    def _to_depth_uint16(self, image):
        arr = np.asarray(image)
        arr = np.nan_to_num(arr, nan=0.0, posinf=0.0, neginf=0.0)
        if arr.dtype == np.uint16:
            return arr
        if np.issubdtype(arr.dtype, np.floating):
            arr = arr * 10000.0
        return np.clip(arr, 0, 65535).astype(np.uint16)

    def finalize(self, success=None, failure_type=None, notes=""):
        if not self.enabled:
            return
        if failure_type not in FAILURE_TYPES:
            failure_type = None

        self.end_time = _now_text()
        self.meta["end_time"] = self.end_time
        self.meta["success"] = success
        self.meta["failure_type"] = failure_type
        self.meta["notes"] = notes or ""
        self.meta["total_steps"] = self.step
        self._write_meta()
        self._write_result()

        try:
            if self.actions_file is not None:
                self.actions_file.close()
                self.actions_file = None
        except Exception as exc:
            print(f"[LOG][WARN] Failed to close actions.jsonl: {exc}")

    def _write_result(self):
        if self.result_path is None:
            return
        success = self.meta.get("success")
        success_text = "null" if success is None else str(bool(success)).lower()
        lines = [
            f"Run ID: {self.meta.get('run_id')}",
            f"Instruction: {self.meta.get('instruction')}",
            f"Model: {self.meta.get('model_name')}",
            f"Robot: {self.meta.get('robot')}",
            f"Camera: {self.meta.get('camera')}",
            f"Start time: {self.meta.get('start_time')}",
            f"End time: {self.meta.get('end_time')}",
            f"Total steps: {self.meta.get('total_steps')}",
            f"Success: {success_text}",
            f"Failure type: {self.meta.get('failure_type')}",
            f"Notes: {self.meta.get('notes')}",
            f"Log folder: {self.run_dir}",
            "",
        ]
        try:
            self.result_path.write_text("\n".join(lines), encoding="utf-8")
        except Exception as exc:
            print(f"[LOG][WARN] Failed to write result.txt: {exc}")
