"""Asynchronous background logger with dual-queue priority design.

Two bounded queues:
- ``_event_queue`` (high priority) — JSONL events.  log() always tries this
  first; only when it overflows is the event dropped and counted.
- ``_image_queue`` (low priority) — RGB + depth arrays.  Images are enqueued
  only after the event was successfully placed.  When the image queue is full
  the image is simply dropped (the JSONL record is still written).

Control and HTTP worker threads **never** wait for disk writes.
"""

from __future__ import annotations

import json
import queue
import threading
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


class AsyncRunLogger:
    """Non-blocking run logger with strict event-priority queueing.

    ``log()`` never blocks the caller.  When queues fill up:
    1. Images are dropped first (``dropped_image_logs``).
    2. If the event queue itself is full, the event is dropped
       (``dropped_event_count``).

    ``close()`` uses a timeout-based sentinel so it won't block forever
    even when the queue is full.
    """

    def __init__(
        self,
        root: str | Path,
        prefix: str,
        metadata: dict[str, Any] | None = None,
        queue_size: int = 256,
        image_queue_size: int | None = None,
    ):
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.run_dir = Path(root) / f"{prefix}_{timestamp}"
        self.run_dir.mkdir(parents=True, exist_ok=False)
        self.rgb_dir = self.run_dir / "rgb"
        self.depth_dir = self.run_dir / "depth"
        self.rgb_dir.mkdir()
        self.depth_dir.mkdir()

        (self.run_dir / "meta.json").write_text(
            json.dumps(metadata or {}, ensure_ascii=False, indent=2, default=_json_default) + "\n",
            encoding="utf-8",
        )

        if image_queue_size is None:
            image_queue_size = queue_size

        # ---- dual queues ----
        self._event_queue: queue.Queue = queue.Queue(maxsize=queue_size)         # (index, event_dict)
        self._image_queue: queue.Queue = queue.Queue(maxsize=image_queue_size)   # (index, rgb_array, depth_array)

        self._index = 0
        self._index_lock = threading.Lock()

        self._dropped_images = 0
        self._dropped_events = 0
        self._counter_lock = threading.Lock()

        self._closed = False
        self._events_file = (self.run_dir / "events.jsonl").open("a", encoding="utf-8")

        self._writer = threading.Thread(target=self._writer_loop, name="async-log-writer", daemon=True)
        self._writer.start()

    # -- public read-only properties ----------------------------------------

    @property
    def queue_depth(self) -> int:
        """Approximate number of JSONL events waiting to be written."""
        return self._event_queue.qsize()

    @property
    def dropped_image_logs(self) -> int:
        """Cumulative count of RGB/Depth images dropped due to queue pressure."""
        with self._counter_lock:
            return self._dropped_images

    @property
    def dropped_event_count(self) -> int:
        """Cumulative count of entire events dropped (last resort)."""
        with self._counter_lock:
            return self._dropped_events

    # -- public methods -----------------------------------------------------

    def log(
        self,
        event: dict[str, Any],
        rgb: np.ndarray | None = None,
        depth_m: np.ndarray | None = None,
    ) -> None:
        """Enqueue an event for background writing.  **Never blocks.**

        Priority order:
        1. JSONL event is enqueued on ``_event_queue``.
        2. Only after (1) succeeds, RGB/Depth images are enqueued on
           ``_image_queue`` (best-effort).
        """
        if self._closed:
            return
        with self._index_lock:
            self._index += 1
            idx = self._index

        # Step 1 — enqueue the JSONL event (high priority)
        try:
            self._event_queue.put_nowait((idx, event))
        except queue.Full:
            with self._counter_lock:
                self._dropped_events += 1
            return  # if the event couldn't be saved, images are irrelevant

        # Step 2 — try to enqueue images (low priority, best-effort)
        if rgb is not None or depth_m is not None:
            try:
                self._image_queue.put_nowait((idx, rgb, depth_m))
            except queue.Full:
                with self._counter_lock:
                    self._dropped_images += 1

    def close(self) -> None:
        """Signal the writer and wait for pending items.

        Uses a timeout-based sentinel push so a full queue cannot block
        ``close()`` forever.  The daemon writer thread will exit with the
        process if it can't be cleanly joined.
        """
        if self._closed:
            return
        self._closed = True
        try:
            self._event_queue.put(None, timeout=2.0)  # sentinel
        except queue.Full:
            pass  # queue hopelessly full — daemon thread exits with process
        self._writer.join(timeout=5.0)
        self._events_file.close()

    # -- internal -----------------------------------------------------------

    def _writer_loop(self) -> None:
        """Drain the event queue, matching images from the image queue."""
        image_buffer: dict[int, tuple[np.ndarray | None, np.ndarray | None]] = {}
        while True:
            item = self._event_queue.get()
            if item is None:          # sentinel
                break
            idx, event = item

            # Collect any available images
            while True:
                try:
                    img_idx, rgb, depth_m = self._image_queue.get_nowait()
                    image_buffer[img_idx] = (rgb, depth_m)
                except queue.Empty:
                    break

            rgb, depth_m = image_buffer.pop(idx, (None, None))

            # Clean up stale image entries (older than current index)
            stale = [k for k in image_buffer if k < idx]
            for k in stale:
                del image_buffer[k]

            try:
                self._write_one(idx, event, rgb, depth_m)
            except Exception:
                pass  # never let a write error kill the writer

    def _write_one(
        self,
        index: int,
        event: dict[str, Any],
        rgb: np.ndarray | None,
        depth_m: np.ndarray | None,
    ) -> None:
        record: dict[str, Any] = {
            "index": index,
            "timestamp": datetime.now().isoformat(timespec="milliseconds"),
            **event,
        }

        if rgb is not None:
            rgb_path = self.rgb_dir / f"{index:06d}.jpg"
            Image.fromarray(np.asarray(rgb, dtype=np.uint8)).convert("RGB").save(rgb_path, quality=92)
            record["rgb_path"] = str(rgb_path.relative_to(self.run_dir))

        if depth_m is not None:
            depth_path = self.depth_dir / f"{index:06d}.png"
            depth_mm = np.clip(np.asarray(depth_m) * 1000.0, 0, 65535).astype(np.uint16)
            Image.fromarray(depth_mm).save(depth_path)
            record["depth_path"] = str(depth_path.relative_to(self.run_dir))

        self._events_file.write(json.dumps(record, ensure_ascii=False, default=_json_default) + "\n")
        self._events_file.flush()
