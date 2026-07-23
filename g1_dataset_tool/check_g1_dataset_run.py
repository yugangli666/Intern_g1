#!/usr/bin/env python3
"""
check_g1_dataset_run.py — Validate a G1 dataset episode.

Usage:
  python3 check_g1_dataset_run.py --episode /path/to/episode_YYYYMMDD_HHMMSS
  python3 check_g1_dataset_run.py --episode /path/to/episode_YYYYMMDD_HHMMSS.inprogress

Performs 23 integrity checks on the episode directory, meta.json, frames.jsonl,
and all recorded RGB/depth image files.  Produces a human-readable report.

This tool does NOT require ROS 2 — it only needs Python 3, numpy, and OpenCV.
"""

import argparse
import json
import math
import os
import sys
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple

import cv2
import numpy as np


# ============================================================================
# Utility
# ============================================================================

_STATUS_OK = "✓"
_STATUS_WARN = "⚠"
_STATUS_FAIL = "✗"
_STATUS_INFO = "ℹ"


def _green(s: str) -> str:
    return f"\033[32m{s}\033[0m"


def _yellow(s: str) -> str:
    return f"\033[33m{s}\033[0m"


def _red(s: str) -> str:
    return f"\033[31m{s}\033[0m"


def _bold(s: str) -> str:
    return f"\033[1m{s}\033[0m"


def _status_line(status: str, msg: str) -> str:
    colour = {"✓": _green, "⚠": _yellow, "✗": _red, "ℹ": lambda x: x}.get(status, lambda x: x)
    return f"  {colour(status)}  {msg}"


# ============================================================================
# Main check function
# ============================================================================

def check_episode(episode_path: str) -> int:
    """Run all checks on an episode. Returns 0 if all pass, 1 otherwise."""
    ep_dir = Path(episode_path).expanduser().resolve()

    # Normalise: strip .inprogress suffix for display
    ep_name = ep_dir.name
    if ep_name.endswith(".inprogress"):
        ep_name_stripped = ep_name[:-len(".inprogress")]
        is_inprogress = True
    else:
        ep_name_stripped = ep_name
        is_inprogress = False

    print()
    print(_bold(f"Episode: {ep_name}"))
    print(f"  Path: {ep_dir}")
    print()

    errors = 0
    warnings = 0

    def check(condition: bool, msg: str, is_error: bool = True) -> bool:
        nonlocal errors, warnings
        if condition:
            print(_status_line(_STATUS_OK, msg))
            return True
        else:
            status = _STATUS_FAIL if is_error else _STATUS_WARN
            print(_status_line(status, msg))
            if is_error:
                errors += 1
            else:
                warnings += 1
            return False

    # ────────────────────────────────────────────────────────────────
    # 1. Episode path exists
    # ────────────────────────────────────────────────────────────────
    if not ep_dir.exists():
        print(_status_line(_STATUS_FAIL, f"Episode directory does not exist: {ep_dir}"))
        return 1
    print(_status_line(_STATUS_OK, f"Episode directory exists"))

    # ────────────────────────────────────────────────────────────────
    # 2. meta.json exists and is parseable
    # ────────────────────────────────────────────────────────────────
    meta_path = ep_dir / "meta.json"
    meta: Dict[str, Any] = {}
    if not check(meta_path.exists(), "meta.json exists"):
        print(_status_line(_STATUS_FAIL, "Cannot continue without meta.json — aborting."))
        return 1

    try:
        with open(meta_path, "r", encoding="utf-8") as f:
            meta = json.load(f)
        print(_status_line(_STATUS_OK, "meta.json is valid JSON"))
    except json.JSONDecodeError as exc:
        print(_status_line(_STATUS_FAIL, f"meta.json parse error: {exc}"))
        return 1

    sync_policy = meta.get("time_sync_policy", {}) if isinstance(meta, dict) else {}
    try:
        rgb_depth_threshold_ms = float(sync_policy.get("rgb_depth_threshold_ms", 30.0))
    except Exception:
        rgb_depth_threshold_ms = 30.0
    try:
        image_odom_threshold_ms = float(sync_policy.get("image_odom_threshold_ms", 50.0))
    except Exception:
        image_odom_threshold_ms = 50.0

    # ────────────────────────────────────────────────────────────────
    # 3. frames.jsonl exists and each line is parseable
    # ────────────────────────────────────────────────────────────────
    frames_path = ep_dir / "frames.jsonl"
    if not check(frames_path.exists(), "frames.jsonl exists"):
        print(_status_line(_STATUS_FAIL, "Cannot continue without frames.jsonl — aborting."))
        return 1

    frames: List[Dict[str, Any]] = []
    parse_errors_count = 0
    with open(frames_path, "r", encoding="utf-8") as f:
        for line_no, line in enumerate(f, 1):
            line = line.strip()
            if not line:
                continue
            try:
                frames.append(json.loads(line))
            except json.JSONDecodeError as exc:
                parse_errors_count += 1
                if parse_errors_count <= 5:
                    print(_status_line(_STATUS_WARN, f"frames.jsonl line {line_no}: parse error: {exc}"))

    all_parsed = check(parse_errors_count == 0, f"All {len(frames) + parse_errors_count} lines parseable "
                       f"({parse_errors_count} errors)")

    if len(frames) == 0:
        print(_status_line(_STATUS_FAIL, "No frames found in frames.jsonl — aborting."))
        return 1

    # ────────────────────────────────────────────────────────────────
    # 4. meta.total_frames equals JSONL line count
    # ────────────────────────────────────────────────────────────────
    meta_total = meta.get("total_frames", 0)
    check(
        meta_total == len(frames),
        f"meta.total_frames ({meta_total}) == JSONL lines ({len(frames)})",
    )

    # ────────────────────────────────────────────────────────────────
    # 5. step continuous
    # ────────────────────────────────────────────────────────────────
    steps = [f.get("step") for f in frames]
    expected_steps = list(range(steps[0], steps[0] + len(steps))) if steps else []
    step_ok = steps == expected_steps
    if step_ok:
        print(_status_line(_STATUS_OK, f"Step sequence continuous: {steps[0]}..{steps[-1]}"))
    else:
        gaps = []
        for i in range(1, len(steps)):
            if steps[i] != steps[i - 1] + 1:
                gaps.append((i, steps[i - 1], steps[i]))
        gap_strs = [f"at idx {idx}: {a}→{b}" for idx, a, b in gaps[:5]]
        print(_status_line(_STATUS_FAIL, f"Step sequence NOT continuous: {len(gaps)} gaps "
                           f"({'; '.join(gap_strs)}{'...' if len(gaps) > 5 else ''})"))
        errors += 1

    # ────────────────────────────────────────────────────────────────
    # 6. capture_time_ns monotonically increasing
    # ────────────────────────────────────────────────────────────────
    cap_times = [f.get("capture_time_ns", 0) for f in frames]
    mono_ok = all(cap_times[i] < cap_times[i + 1] for i in range(len(cap_times) - 1))
    violations = sum(1 for i in range(len(cap_times) - 1) if cap_times[i] >= cap_times[i + 1])
    check(mono_ok, f"capture_time_ns strictly monotonic ({violations} violations)")

    # ────────────────────────────────────────────────────────────────
    # 7–8. All rgb_path/depth_path exist
    # ────────────────────────────────────────────────────────────────
    rgb_missing = 0
    depth_missing = 0
    for f_entry in frames:
        rgb_p = ep_dir / f_entry.get("rgb_path", "")
        depth_p = ep_dir / f_entry.get("depth_path", "")
        if not rgb_p.exists():
            rgb_missing += 1
        if not depth_p.exists():
            depth_missing += 1
    check(rgb_missing == 0, f"All {len(frames)} RGB files exist ({rgb_missing} missing)")
    check(depth_missing == 0, f"All {len(frames)} depth files exist ({depth_missing} missing)")

    # ────────────────────────────────────────────────────────────────
    # 9. RGB readable (sample check)
    # ────────────────────────────────────────────────────────────────
    rgb_read_errors = 0
    rgb_shapes: List[Tuple[int, ...]] = []
    for f_entry in frames:
        rgb_p = ep_dir / f_entry.get("rgb_path", "")
        if rgb_p.exists():
            try:
                img = cv2.imread(str(rgb_p))
                if img is None:
                    rgb_read_errors += 1
                else:
                    rgb_shapes.append(img.shape)
            except Exception:
                rgb_read_errors += 1
    check(rgb_read_errors == 0, f"All RGB files readable ({rgb_read_errors} errors)")

    if rgb_shapes:
        shape_counts: Dict[str, int] = {}
        for s in rgb_shapes:
            key = f"{s[0]}x{s[1]}"
            shape_counts[key] = shape_counts.get(key, 0) + 1
        shape_summary = ", ".join(f"{k}: {v}" for k, v in sorted(shape_counts.items()))
        print(_status_line(_STATUS_INFO, f"RGB shapes: {shape_summary}"))

    # ────────────────────────────────────────────────────────────────
    # 10. Depth readable
    # ────────────────────────────────────────────────────────────────
    depth_read_errors = 0
    depth_dtypes: List[np.dtype] = []
    depth_shapes: List[Tuple[int, ...]] = []
    depth_m_values: List[float] = []
    depth_nonzero_fractions: List[float] = []

    depth_unit = meta.get("depth_unit_m_per_value", 0.001)

    for f_entry in frames:
        depth_p = ep_dir / f_entry.get("depth_path", "")
        if depth_p.exists():
            try:
                dimg = cv2.imread(str(depth_p), cv2.IMREAD_UNCHANGED)
                if dimg is None:
                    depth_read_errors += 1
                else:
                    depth_dtypes.append(dimg.dtype)
                    depth_shapes.append(dimg.shape)
                    # Metre-range stats
                    depth_m = dimg.astype(np.float32) * float(depth_unit)
                    depth_m_values.append(float(np.nanmean(depth_m)))
                    nz = float(np.count_nonzero(dimg) / dimg.size)
                    depth_nonzero_fractions.append(nz)
            except Exception:
                depth_read_errors += 1
    check(depth_read_errors == 0, f"All depth files readable ({depth_read_errors} errors)")

    # ────────────────────────────────────────────────────────────────
    # 11. Depth dtype is uint16
    # ────────────────────────────────────────────────────────────────
    dtype_counts: Dict[str, int] = {}
    for d in depth_dtypes:
        dtype_counts[str(d)] = dtype_counts.get(str(d), 0) + 1
    dtype_summary = ", ".join(f"{k}: {v}" for k, v in sorted(dtype_counts.items()))
    all_uint16 = all(str(d) == "uint16" for d in depth_dtypes)
    check(all_uint16, f"Depth dtype is uint16 ({dtype_summary})", is_error=not all_uint16)

    # ────────────────────────────────────────────────────────────────
    # 12. Depth is single-channel
    # ────────────────────────────────────────────────────────────────
    non_single = sum(1 for s in depth_shapes if len(s) not in (2,) and (len(s) == 3 and s[2] != 1))
    all_single = non_single == 0
    shape_summary = ", ".join(
        f"{s[0]}x{s[1]}" for s in sorted(set(tuple(ss) for ss in depth_shapes))[:5]
    )
    check(all_single, f"Depth is single-channel ({shape_summary}, {non_single} multi-channel)")

    # ────────────────────────────────────────────────────────────────
    # 13. Depth metre-range and non-zero ratio stats
    # ────────────────────────────────────────────────────────────────
    if depth_m_values:
        mean_m = float(np.mean(depth_m_values))
        min_m = float(np.min(depth_m_values))
        max_m = float(np.max(depth_m_values))
        mean_nz = float(np.mean(depth_nonzero_fractions)) * 100
        print(_status_line(
            _STATUS_INFO,
            f"Depth metre range: mean={mean_m:.3f}m  min={min_m:.3f}m  max={max_m:.3f}m  "
            f"nonzero_ratio={mean_nz:.1f}%"
        ))
        # Warn if mean depth > 10m (likely wrong units)
        if max_m > 20.0:
            print(_status_line(
                _STATUS_WARN,
                f"Max depth {max_m:.1f}m > 20m — check depth_unit_m_per_value ({depth_unit})"
            ))
            warnings += 1
        if mean_nz < 5.0:
            print(_status_line(
                _STATUS_WARN,
                f"Mean non-zero ratio {mean_nz:.1f}% very low — depth may be mostly invalid"
            ))
            warnings += 1

    # ────────────────────────────────────────────────────────────────
    # 13b. Model-input files match inference request format
    # ────────────────────────────────────────────────────────────────
    model_cfg = meta.get("model_input", {})
    model_enabled = isinstance(model_cfg, dict) and bool(model_cfg.get("enabled", False))
    has_model_paths = any(f.get("model_rgb_path") or f.get("model_depth_path") for f in frames)
    if model_enabled or has_model_paths:
        model_rgb_missing = 0
        model_depth_missing = 0
        model_rgb_read_errors = 0
        model_depth_read_errors = 0
        model_rgb_shapes: List[Tuple[int, ...]] = []
        model_depth_shapes: List[Tuple[int, ...]] = []
        model_depth_dtypes: List[np.dtype] = []
        model_depth_m_values: List[float] = []
        model_depth_nonzero_fractions: List[float] = []
        model_depth_unit = float(model_cfg.get("depth_value_m_per_unit", 0.0001)) if isinstance(model_cfg, dict) else 0.0001

        for f_entry in frames:
            model_rgb_rel = f_entry.get("model_rgb_path", "")
            model_depth_rel = f_entry.get("model_depth_path", "")
            model_rgb_p = ep_dir / model_rgb_rel
            model_depth_p = ep_dir / model_depth_rel

            if not model_rgb_rel or not model_rgb_p.exists():
                model_rgb_missing += 1
            else:
                img = cv2.imread(str(model_rgb_p))
                if img is None:
                    model_rgb_read_errors += 1
                else:
                    model_rgb_shapes.append(img.shape)

            if not model_depth_rel or not model_depth_p.exists():
                model_depth_missing += 1
            else:
                dimg = cv2.imread(str(model_depth_p), cv2.IMREAD_UNCHANGED)
                if dimg is None:
                    model_depth_read_errors += 1
                else:
                    model_depth_dtypes.append(dimg.dtype)
                    model_depth_shapes.append(dimg.shape)
                    depth_m = dimg.astype(np.float32) * model_depth_unit
                    model_depth_m_values.append(float(np.nanmean(depth_m)))
                    model_depth_nonzero_fractions.append(float(np.count_nonzero(dimg) / dimg.size))

        check(model_rgb_missing == 0, f"All {len(frames)} model-input RGB files exist ({model_rgb_missing} missing)")
        check(model_depth_missing == 0, f"All {len(frames)} model-input depth files exist ({model_depth_missing} missing)")
        check(model_rgb_read_errors == 0, f"All model-input RGB files readable ({model_rgb_read_errors} errors)")
        check(model_depth_read_errors == 0, f"All model-input depth files readable ({model_depth_read_errors} errors)")

        if model_rgb_shapes:
            shape_counts: Dict[str, int] = {}
            for s in model_rgb_shapes:
                key = f"{s[0]}x{s[1]}"
                shape_counts[key] = shape_counts.get(key, 0) + 1
            shape_summary = ", ".join(f"{k}: {v}" for k, v in sorted(shape_counts.items()))
            print(_status_line(_STATUS_INFO, f"Model-input RGB shapes: {shape_summary}"))

        if model_depth_dtypes:
            dtype_counts: Dict[str, int] = {}
            for d in model_depth_dtypes:
                dtype_counts[str(d)] = dtype_counts.get(str(d), 0) + 1
            dtype_summary = ", ".join(f"{k}: {v}" for k, v in sorted(dtype_counts.items()))
            all_model_uint16 = all(str(d) == "uint16" for d in model_depth_dtypes)
            check(all_model_uint16, f"Model-input depth dtype is uint16 ({dtype_summary})", is_error=not all_model_uint16)

        if model_depth_shapes:
            non_single_model = sum(1 for s in model_depth_shapes if len(s) not in (2,) and (len(s) == 3 and s[2] != 1))
            shape_summary = ", ".join(
                f"{s[0]}x{s[1]}" for s in sorted(set(tuple(ss) for ss in model_depth_shapes))[:5]
            )
            check(non_single_model == 0, f"Model-input depth is single-channel ({shape_summary}, {non_single_model} multi-channel)")

        if model_depth_m_values:
            mean_m = float(np.mean(model_depth_m_values))
            min_m = float(np.min(model_depth_m_values))
            max_m = float(np.max(model_depth_m_values))
            mean_nz = float(np.mean(model_depth_nonzero_fractions)) * 100
            print(_status_line(
                _STATUS_INFO,
                f"Model-input depth metre range: mean={mean_m:.3f}m  min={min_m:.3f}m  "
                f"max={max_m:.3f}m  nonzero_ratio={mean_nz:.1f}%  unit={model_depth_unit}"
            ))

    # ────────────────────────────────────────────────────────────────
    # 14. Valid sync proportion
    # ────────────────────────────────────────────────────────────────
    sync_valid = sum(1 for f in frames if f.get("valid_sync", False))
    sync_pct = sync_valid / len(frames) * 100 if frames else 0
    check(sync_pct >= 50.0, f"Valid sync frames: {sync_valid}/{len(frames)} ({sync_pct:.1f}%)",
          is_error=(sync_pct < 30.0))
    if sync_pct < 50.0:
        print(_status_line(_STATUS_WARN, "Less than 50% frames synced — check camera/odom timing"))

    # ────────────────────────────────────────────────────────────────
    # 14b. Time source statistics (if effective-time fields present)
    # ────────────────────────────────────────────────────────────────
    has_effective_times = any(
        f.get("rgb_effective_time_ns", 0) > 0 for f in frames
    )
    if has_effective_times:
        # Count time sources
        src_counts: Dict[str, Dict[str, int]] = {
            "rgb": {},
            "depth": {},
            "odom": {},
        }
        for f_entry in frames:
            for key in ("rgb", "depth", "odom"):
                src = f_entry.get(f"{key}_time_source", "unknown")
                src_counts[key][src] = src_counts[key].get(src, 0) + 1

        for label, key in [("RGB", "rgb"), ("Depth", "depth"), ("Odom", "odom")]:
            counts = src_counts[key]
            summary = ", ".join(f"{s}: {n}" for s, n in sorted(counts.items()))
            print(_status_line(_STATUS_INFO, f"{label} time_source: {summary}"))

        # Count sync bases (newer data format)
        basis_counts: Dict[str, int] = {}
        for f_entry in frames:
            basis = f_entry.get("sync_time_basis")
            if basis:
                basis_counts[basis] = basis_counts.get(basis, 0) + 1
        if basis_counts:
            basis_summary = ", ".join(f"{s}: {n}" for s, n in sorted(basis_counts.items()))
            print(_status_line(_STATUS_INFO, f"Sync time_basis: {basis_summary}"))

        # Warn if most frames are recv-based (suggesting stamp issues)
        for label, key in [("RGB", "rgb"), ("Depth", "depth"), ("Odom", "odom")]:
            counts = src_counts[key]
            total = sum(counts.values())
            recv_count = counts.get("recv", 0)
            if total > 0 and recv_count / total > 0.5:
                print(_status_line(
                    _STATUS_WARN,
                    f"{label}: {recv_count}/{total} frames ({recv_count/total*100:.0f}%) "
                    f"using recv time (stamps may be missing or frozen)"
                ))
                warnings += 1

        # Check for duplicate effective times (stale message indicator)
        for label, key in [("RGB", "rgb"), ("Depth", "depth")]:
            eff_field = f"{key}_effective_time_ns"
            eff_times = [f.get(eff_field, 0) for f in frames]
            # Count consecutive duplicates
            dup_runs = 0
            max_run = 0
            current_run = 1
            for i in range(1, len(eff_times)):
                if eff_times[i] == eff_times[i - 1] and eff_times[i] > 0:
                    current_run += 1
                else:
                    if current_run > 1:
                        dup_runs += 1
                        max_run = max(max_run, current_run)
                    current_run = 1
            if current_run > 1:
                dup_runs += 1
                max_run = max(max_run, current_run)

            if dup_runs > 0:
                print(_status_line(
                    _STATUS_WARN,
                    f"{label}: {dup_runs} duplicate-effective-time runs detected "
                    f"(max run length={max_run}) — may indicate stale/frozen messages"
                ))
                warnings += 1
            else:
                print(_status_line(_STATUS_OK, f"{label}: no duplicate effective times (messages updating)"))

        stale_counts = meta.get("stale_skip_counts", {})
        if isinstance(stale_counts, dict) and stale_counts:
            total_stale = stale_counts.get("total", 0)
            print(_status_line(
                _STATUS_INFO,
                "Stale skips: "
                f"rgb_only={stale_counts.get('rgb_only', 0)}, "
                f"depth_only={stale_counts.get('depth_only', 0)}, "
                f"both={stale_counts.get('both', 0)}, "
                f"total={total_stale}"
            ))
            if total_stale:
                warnings += 1

        # Re-check dt values using effective times if available
        if all(
            f.get("rgb_effective_time_ns", 0) > 0
            and f.get("depth_effective_time_ns", 0) > 0
            for f in frames
        ):
            eff_sync_valid = 0
            for f_entry in frames:
                rgb_eff = f_entry.get("rgb_effective_time_ns", 0)
                depth_eff = f_entry.get("depth_effective_time_ns", 0)
                odom_eff = f_entry.get("odom_effective_time_ns", 0)
                if rgb_eff <= 0 or depth_eff <= 0:
                    continue
                rgb_depth_dt = abs(rgb_eff - depth_eff) / 1e6
                image_odom_dt = (
                    abs(max(rgb_eff, depth_eff) - odom_eff) / 1e6
                    if odom_eff > 0 else float("inf")
                )
                if (
                    rgb_depth_dt <= rgb_depth_threshold_ms
                    and image_odom_dt <= image_odom_threshold_ms
                    and odom_eff > 0
                ):
                    eff_sync_valid += 1
            eff_sync_pct = eff_sync_valid / len(frames) * 100 if frames else 0
            print(_status_line(
                _STATUS_INFO,
                f"Effective-time sync: {eff_sync_valid}/{len(frames)} ({eff_sync_pct:.1f}%) "
                f"(using effective times, ≤{rgb_depth_threshold_ms:.0f}/{image_odom_threshold_ms:.0f}ms)"
            ))

        # Re-check dt values using sync_time fields if present.  This is the
        # authoritative timing basis for newer G1 data because it avoids mixing
        # image header stamps with recv-time-only SportModeState odom.
        if all(
            f.get("rgb_sync_time_ns", 0) > 0
            and f.get("depth_sync_time_ns", 0) > 0
            for f in frames
        ):
            sync_time_valid = 0
            for f_entry in frames:
                rgb_sync = f_entry.get("rgb_sync_time_ns", 0)
                depth_sync = f_entry.get("depth_sync_time_ns", 0)
                odom_sync = f_entry.get("odom_sync_time_ns", 0)
                if rgb_sync <= 0 or depth_sync <= 0:
                    continue
                rgb_depth_dt = abs(rgb_sync - depth_sync) / 1e6
                image_odom_dt = (
                    abs(max(rgb_sync, depth_sync) - odom_sync) / 1e6
                    if odom_sync > 0 else float("inf")
                )
                if (
                    rgb_depth_dt <= rgb_depth_threshold_ms
                    and image_odom_dt <= image_odom_threshold_ms
                    and odom_sync > 0
                ):
                    sync_time_valid += 1
            sync_time_pct = sync_time_valid / len(frames) * 100 if frames else 0
            print(_status_line(
                _STATUS_INFO,
                f"Sync-time sync: {sync_time_valid}/{len(frames)} ({sync_time_pct:.1f}%) "
                f"(authoritative, ≤{rgb_depth_threshold_ms:.0f}/{image_odom_threshold_ms:.0f}ms)"
            ))
    else:
        print(_status_line(_STATUS_INFO, "No effective-time fields found (older data format)"))

    # ────────────────────────────────────────────────────────────────
    # 15. Trainable frame proportion
    # ────────────────────────────────────────────────────────────────
    trainable_count = sum(1 for f in frames if f.get("trainable", False))
    trainable_pct = trainable_count / len(frames) * 100 if frames else 0
    print(_status_line(
        _STATUS_INFO,
        f"Trainable frames: {trainable_count}/{len(frames)} ({trainable_pct:.1f}%)"
    ))

    # ────────────────────────────────────────────────────────────────
    # 16. Stationary frame proportion
    # ────────────────────────────────────────────────────────────────
    stationary_count = 0
    for f_entry in frames:
        cmd = f_entry.get("command", {})
        if cmd.get("label") == "hold" and abs(cmd.get("vx", 0)) < 1e-6 and abs(cmd.get("wz", 0)) < 1e-6:
            stationary_count += 1
    stationary_pct = stationary_count / len(frames) * 100 if frames else 0
    print(_status_line(
        _STATUS_INFO,
        f"Stationary (hold) frames: {stationary_count}/{len(frames)} ({stationary_pct:.1f}%)"
    ))

    # ────────────────────────────────────────────────────────────────
    # 17. Odom valid proportion
    # ────────────────────────────────────────────────────────────────
    odom_valid = 0
    for f_entry in frames:
        odom = f_entry.get("odom", {})
        if odom.get("derived_yaw") is not None:
            odom_valid += 1
    odom_pct = odom_valid / len(frames) * 100 if frames else 0
    check(odom_pct > 0, f"Frames with valid odom yaw: {odom_valid}/{len(frames)} ({odom_pct:.1f}%)",
          is_error=(odom_pct == 0 and meta.get("collection_mode") != "dry_run"))

    # ────────────────────────────────────────────────────────────────
    # 18. Total displacement
    # ────────────────────────────────────────────────────────────────
    xs = []
    ys = []
    for f_entry in frames:
        odom = f_entry.get("odom", {})
        pos = odom.get("position", [])
        if len(pos) >= 2:
            x, y = pos[0], pos[1]
            if x is not None and y is not None:
                xs.append(x)
                ys.append(y)
    if len(xs) >= 2:
        total_disp = math.sqrt((xs[-1] - xs[0]) ** 2 + (ys[-1] - ys[0]) ** 2)
        print(_status_line(_STATUS_INFO, f"Total displacement: {total_disp:.3f} m (odom-based)"))
        # Check for odom jumps (>0.5m between consecutive frames)
        jumps = 0
        max_jump = 0.0
        for i in range(1, len(xs)):
            d = math.sqrt((xs[i] - xs[i - 1]) ** 2 + (ys[i] - ys[i - 1]) ** 2)
            if d > 0.5:
                jumps += 1
                max_jump = max(max_jump, d)
        if jumps > 0:
            print(_status_line(_STATUS_WARN, f"Odom jumps >0.5m detected: {jumps} occurrences "
                               f"(max={max_jump:.3f}m)"))
            warnings += 1
        else:
            print(_status_line(_STATUS_OK, "No odom jumps >0.5m detected"))
    else:
        print(_status_line(_STATUS_WARN, "Insufficient odom data for displacement calculation"))

    # ────────────────────────────────────────────────────────────────
    # 19. Odom jump check already done above (integrated with #18)
    # ────────────────────────────────────────────────────────────────

    # ────────────────────────────────────────────────────────────────
    # 20. success and end_reason filled reasonably
    # ────────────────────────────────────────────────────────────────
    success_val = meta.get("success")
    end_reason = meta.get("end_reason", "")
    collection_mode = meta.get("collection_mode", "unknown")

    valid_combos = {
        ("manual_demo", True, "success_terminal"): "OK - manual demo completed successfully",
        ("manual_demo", False, "failure"): "OK - manual demo marked as failure",
        ("manual_demo", False, "abort"): "OK - manual demo aborted",
        ("manual_demo", False, "exception"): "OK - manual demo ended with exception",
        ("manual_demo", False, "interrupted"): "OK - manual demo interrupted",
        ("dry_run", None, "dry_run_finished"): "OK - dry-run completed",
        ("dry_run", None, "interrupted"): "OK - dry-run interrupted",
        ("dry_run", False, "exception"): "OK - dry-run exception",
    }

    key = (collection_mode, success_val, end_reason)
    desc = valid_combos.get(key)
    if desc:
        print(_status_line(_STATUS_OK, f"success={success_val}, end_reason='{end_reason}' — {desc}"))
    else:
        print(_status_line(_STATUS_WARN, f"success={success_val}, end_reason='{end_reason}' "
                           f"(collection_mode={collection_mode}) — unexpected combination"))
        warnings += 1

    # ────────────────────────────────────────────────────────────────
    # 21. .inprogress status check
    # ────────────────────────────────────────────────────────────────
    # end_reason values that represent a properly-finalised episode
    # (directory was renamed from .inprogress → final name).
    # All of these have success is not None, triggering the rename in
    # _finalize_episode(): success=True for success_terminal;
    # success=False for failure/abort/interrupted/exception;
    # success=None for dry_run_finished (which also renames).
    _FINALIZED_END_REASONS = frozenset({
        "success_terminal",
        "dry_run_finished",
        "failure",
        "abort",
        "interrupted",
        "exception",
    })

    if is_inprogress:
        if end_reason in _FINALIZED_END_REASONS:
            print(_status_line(_STATUS_WARN, "Episode is .inprogress but end_reason suggests completion. "
                               "Directory may not have been renamed — check for crash during finalization."))
            warnings += 1
        else:
            print(_status_line(_STATUS_INFO, "Episode is .inprogress (expected for incomplete/non-terminal runs)"))
    else:
        if end_reason not in _FINALIZED_END_REASONS:
            print(_status_line(_STATUS_WARN, "Episode is NOT .inprogress but end_reason is not a recognised "
                               f"finalized state: '{end_reason}'."))
            warnings += 1
        else:
            print(_status_line(_STATUS_OK, f"Episode is properly finalised (end_reason='{end_reason}')"))

    # ────────────────────────────────────────────────────────────────
    # 22. Camera intrinsics present
    # ────────────────────────────────────────────────────────────────
    K = meta.get("camera_intrinsic", [])
    D = meta.get("camera_distortion", [])
    dm = meta.get("distortion_model", "")
    has_K = isinstance(K, list) and len(K) >= 9
    has_D = isinstance(D, list) and len(D) > 0
    if has_K and has_D:
        print(_status_line(_STATUS_OK, f"Camera intrinsics: K=9 values, D={len(D)} values, "
                           f"distortion_model='{dm}'"))
    elif has_K:
        print(_status_line(_STATUS_WARN, f"Camera intrinsics: K present but D missing"))
        warnings += 1
    elif has_D:
        print(_status_line(_STATUS_WARN, f"Camera intrinsics: D present but K missing"))
        warnings += 1
    else:
        print(_status_line(_STATUS_WARN, "Camera intrinsics missing — camera_info topic may not have been published"))
        warnings += 1

    # ────────────────────────────────────────────────────────────────
    # 23. initial_odom present
    # ────────────────────────────────────────────────────────────────
    initial_odom = meta.get("initial_odom", {})
    has_initial = isinstance(initial_odom, dict) and len(initial_odom) > 0
    if has_initial:
        # Quick peek at what's in there
        keys = list(initial_odom.keys())[:5]
        print(_status_line(_STATUS_OK, f"initial_odom present with keys: {keys}"))
    else:
        print(_status_line(_STATUS_WARN, "initial_odom is empty — no odom data was captured"))
        warnings += 1

    # ────────────────────────────────────────────────────────────────
    # Additional info
    # ────────────────────────────────────────────────────────────────
    duration_ns = meta.get("end_time_ns", 0) - meta.get("start_time_ns", 0)
    duration_s = duration_ns / 1e9 if duration_ns > 0 else 0
    actual_fps = len(frames) / duration_s if duration_s > 0 else 0

    print()
    print(_bold("Summary"))
    print(f"  Collection mode:   {collection_mode}")
    print(f"  Instruction:       {str(meta.get('instruction', ''))[:100]}")
    print(f"  Target FPS:        {meta.get('target_fps', '?')}")
    print(f"  Frames recorded:   {len(frames)}")
    print(f"  Duration:          {duration_s:.1f} s")
    print(f"  Average FPS:       {actual_fps:.1f}")
    print(f"  RGB encoding:      {meta.get('rgb_encoding', '?')}")
    print(f"  Depth encoding:    {meta.get('depth_encoding', '?')}")
    print(f"  Depth unit (m/val):{depth_unit}")
    print(f"  Image size:        {meta.get('image_width', '?')}×{meta.get('image_height', '?')}")
    print(f"  Valid sync:        {sync_valid}/{len(frames)} ({sync_pct:.1f}%)")
    print(f"  Trainable:         {trainable_count}/{len(frames)} ({trainable_pct:.1f}%)")
    print(f"  Odom valid:        {odom_valid}/{len(frames)} ({odom_pct:.1f}%)")
    print(f"  End reason:        {end_reason}")
    print(f"  Success:           {success_val}")
    print()
    print(f"  {_green(_STATUS_OK)} Passed,  {_yellow(_STATUS_WARN)} Warnings: {warnings},  "
          f"{_red(_STATUS_FAIL)} Errors: {errors}")
    print()

    return 1 if errors > 0 else 0


# ============================================================================
# CLI
# ============================================================================

def main() -> None:
    parser = argparse.ArgumentParser(
        description="Validate a G1 dataset episode.",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  python3 check_g1_dataset_run.py --episode ./dataset_runs/episode_20260625_153012
  python3 check_g1_dataset_run.py --episode ./dataset_runs/episode_20260625_153012.inprogress
""",
    )
    parser.add_argument(
        "--episode", required=True,
        help="Path to the episode directory (with or without .inprogress suffix).",
    )
    args = parser.parse_args()

    sys.exit(check_episode(args.episode))


if __name__ == "__main__":
    main()
