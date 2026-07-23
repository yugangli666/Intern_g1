#!/usr/bin/env python3
"""Replay saved VLN images through the HTTP model without ROS motion output."""

import argparse
import json
import os
import re
import time
from datetime import datetime
from pathlib import Path
from typing import Any

import cv2
import numpy as np
import requests

from direct_control_utils import grounding_summary


def _direct_client_pids() -> list[int]:
    matches = []
    for entry in Path("/proc").iterdir():
        if not entry.name.isdigit() or int(entry.name) == os.getpid():
            continue
        try:
            command = (entry / "cmdline").read_bytes().replace(b"\0", b" ").decode(
                "utf-8", errors="replace"
            )
        except (FileNotFoundError, PermissionError, ProcessLookupError):
            continue
        if "internnav_direct_control_client.py" in command:
            matches.append(int(entry.name))
    return sorted(matches)


def _slug(value: str) -> str:
    cleaned = re.sub(r"[^a-zA-Z0-9]+", "_", value).strip("_").lower()
    return cleaned[:60] or "case"


def _variants(choice: str) -> list[str]:
    return ["raw", "bottom_crop"] if choice == "both" else [choice]


def _history_modes(choice: str) -> list[str]:
    return ["isolated", "sequential"] if choice == "both" else [choice]


def _prepare_image(image: np.ndarray, variant: str, crop_ratio: float) -> np.ndarray:
    if variant == "raw":
        return image
    height, width = image.shape[:2]
    crop_height = max(1, int(round(height * (1.0 - crop_ratio))))
    return cv2.resize(image[:crop_height], (width, height), interpolation=cv2.INTER_AREA)


def _encode_observation(image: np.ndarray) -> tuple[bytes, bytes]:
    ok, rgb = cv2.imencode(".jpg", image, [cv2.IMWRITE_JPEG_QUALITY, 95])
    if not ok:
        raise RuntimeError("failed to encode RGB image")
    depth = np.zeros(image.shape[:2], dtype=np.uint16)
    ok, depth_png = cv2.imencode(".png", depth)
    if not ok:
        raise RuntimeError("failed to encode dummy depth")
    return rgb.tobytes(), depth_png.tobytes()


def _response_signature(body: dict[str, Any] | None) -> str:
    return json.dumps(body, sort_keys=True, separators=(",", ":"), ensure_ascii=False)


def _case_summary(case_id: str, records: list[dict[str, Any]]) -> dict[str, Any]:
    counts = {
        "responses": len(records),
        "errors": sum(1 for record in records if record.get("error")),
        "discrete_action": 0,
        "trajectory": 0,
        "pixel_goal": 0,
    }
    action_sequence = []
    grounding_counts: dict[str, int] = {}
    for record in records:
        response = record.get("response") or {}
        if "discrete_action" in response:
            counts["discrete_action"] += 1
            action_sequence.append(response.get("discrete_action"))
        if "trajectory" in response:
            counts["trajectory"] += 1
        if "pixel_goal" in response:
            counts["pixel_goal"] += 1
        status = record.get("grounding_status")
        if status:
            grounding_counts[status] = grounding_counts.get(status, 0) + 1
    return {
        "case_id": case_id,
        **counts,
        "action_sequence": action_sequence,
        "grounding_status_counts": grounding_counts,
        "target_locked": None,
        "target_lock_reason": "object_detector_not_configured",
    }


def _comparison(
    kind: str,
    left_id: str,
    right_id: str,
    cases: dict[str, list[dict[str, Any]]],
) -> dict[str, Any]:
    left = cases[left_id]
    right = cases[right_id]
    compared = min(len(left), len(right))
    identical = sum(
        _response_signature(left[index].get("response"))
        == _response_signature(right[index].get("response"))
        for index in range(compared)
    )
    return {
        "kind": kind,
        "left_case": left_id,
        "right_case": right_id,
        "frames_compared": compared,
        "identical_responses": identical,
        "identical_response_rate": identical / compared if compared else None,
    }


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--input-dir", required=True)
    parser.add_argument("--server-url", default="http://127.0.0.1:5801/eval_dual")
    parser.add_argument("--instruction", action="append", required=True)
    parser.add_argument(
        "--history-mode",
        choices=["isolated", "sequential", "both"],
        default="both",
    )
    parser.add_argument(
        "--image-variant", choices=["raw", "bottom_crop", "both"], default="both"
    )
    parser.add_argument("--bottom-crop-ratio", type=float, default=0.22)
    parser.add_argument("--output-dir")
    parser.add_argument("--request-timeout", type=float, default=120.0)
    parser.add_argument(
        "--exclusive-server",
        action="store_true",
        help="confirm that resetting the model cannot interfere with a live client",
    )
    return parser


def main() -> int:
    parser = build_parser()
    args = parser.parse_args()
    if not args.exclusive_server:
        parser.error("--exclusive-server is required because replay resets model history")
    if not 0.0 <= args.bottom_crop_ratio < 0.5:
        parser.error("--bottom-crop-ratio must be in [0.0, 0.5)")
    if args.request_timeout <= 0.0:
        parser.error("--request-timeout must be positive")
    live_pids = _direct_client_pids()
    if live_pids:
        parser.error(f"live direct-control client detected: {live_pids}")

    input_dir = Path(args.input_dir).expanduser().resolve()
    images = sorted(input_dir.glob("input_*.jpg"))
    if not images:
        parser.error(f"no input_*.jpg images found in {input_dir}")
    output_dir = (
        Path(args.output_dir).expanduser().resolve()
        if args.output_dir
        else Path("experiment_records")
        / f"offline_ab_{datetime.now().strftime('%Y%m%d_%H%M%S')}"
    )
    output_dir.mkdir(parents=True, exist_ok=False)
    responses_path = output_dir / "responses.jsonl"

    session = requests.Session()
    cases: dict[str, list[dict[str, Any]]] = {}
    case_meta: dict[str, dict[str, str]] = {}
    had_error = False
    for instruction_index, instruction in enumerate(args.instruction, 1):
        for history_mode in _history_modes(args.history_mode):
            for variant in _variants(args.image_variant):
                case_id = (
                    f"i{instruction_index}_{_slug(instruction)}__{history_mode}__{variant}"
                )
                case_dir = output_dir / "cases" / case_id
                case_dir.mkdir(parents=True)
                records = []
                case_meta[case_id] = {
                    "instruction": instruction,
                    "history_mode": history_mode,
                    "image_variant": variant,
                }
                for frame_index, image_path in enumerate(images):
                    image = cv2.imread(str(image_path), cv2.IMREAD_COLOR)
                    if image is None:
                        raise RuntimeError(f"failed to read {image_path}")
                    prepared = _prepare_image(image, variant, args.bottom_crop_ratio)
                    prepared_path = case_dir / image_path.name
                    if not cv2.imwrite(str(prepared_path), prepared):
                        raise RuntimeError(f"failed to save {prepared_path}")
                    rgb, depth = _encode_observation(prepared)
                    reset = history_mode == "isolated" or frame_index == 0
                    payload = {
                        "reset": reset,
                        "idx": 0 if reset else frame_index,
                        "instruction": instruction,
                        "camera_pose": np.eye(4, dtype=np.float32).tolist(),
                        "camera_pose_source": "offline_ab_identity",
                    }
                    started = time.monotonic()
                    record: dict[str, Any] = {
                        "case_id": case_id,
                        "instruction": instruction,
                        "history_mode": history_mode,
                        "image_variant": variant,
                        "frame_index": frame_index + 1,
                        "source_image": str(image_path),
                        "prepared_image": str(prepared_path.relative_to(output_dir)),
                        "reset": reset,
                    }
                    try:
                        response = session.post(
                            args.server_url,
                            files={
                                "image": ("rgb.jpg", rgb, "image/jpeg"),
                                "depth": ("depth.png", depth, "image/png"),
                            },
                            data={"json": json.dumps(payload)},
                            timeout=args.request_timeout,
                        )
                        record["latency_ms"] = round(
                            (time.monotonic() - started) * 1000.0, 3
                        )
                        response.raise_for_status()
                        body = response.json()
                        if not isinstance(body, dict):
                            raise ValueError("server response is not a JSON object")
                        record["response"] = body
                        record.update(
                            grounding_summary(
                                body, width=prepared.shape[1], height=prepared.shape[0]
                            )
                        )
                    except Exception as exc:
                        had_error = True
                        record["error"] = f"{type(exc).__name__}: {exc}"
                        record["latency_ms"] = round(
                            (time.monotonic() - started) * 1000.0, 3
                        )
                    records.append(record)
                    with responses_path.open("a", encoding="utf-8") as handle:
                        handle.write(json.dumps(record, ensure_ascii=False) + "\n")
                cases[case_id] = records

    comparisons = []
    case_ids = list(cases)
    for left_index, left_id in enumerate(case_ids):
        left_meta = case_meta[left_id]
        for right_id in case_ids[left_index + 1 :]:
            right_meta = case_meta[right_id]
            differing = [
                key
                for key in ("instruction", "history_mode", "image_variant")
                if left_meta[key] != right_meta[key]
            ]
            if len(differing) == 1:
                comparisons.append(_comparison(differing[0], left_id, right_id, cases))

    case_summaries = [_case_summary(case_id, cases[case_id]) for case_id in case_ids]
    findings = []
    raw_instruction_comparisons = [
        item
        for item in comparisons
        if item["kind"] == "instruction"
        and case_meta[item["left_case"]]["image_variant"] == "raw"
    ]
    if raw_instruction_comparisons and all(
        item["identical_response_rate"] == 1.0 for item in raw_instruction_comparisons
    ):
        findings.append(
            "Raw-image responses were identical across the tested instructions."
        )
    if any(
        item["kind"] == "history_mode" and item["identical_response_rate"] != 1.0
        for item in comparisons
    ):
        findings.append("At least one response sequence changed when history mode changed.")
    if any(
        item["kind"] == "image_variant" and item["identical_response_rate"] != 1.0
        for item in comparisons
    ):
        findings.append("At least one response sequence changed after bottom cropping.")
    trajectory_cases = [
        item["case_id"] for item in case_summaries if item["trajectory"] > 0
    ]
    if trajectory_cases:
        findings.append("Trajectory output appeared in: " + ", ".join(trajectory_cases))
    else:
        findings.append("No tested case produced a trajectory.")

    summary = {
        "created_at": datetime.now().astimezone().isoformat(),
        "input_dir": str(input_dir),
        "server_url": args.server_url,
        "image_count": len(images),
        "dummy_depth": True,
        "publishes_ros_commands": False,
        "object_detector_configured": False,
        "cases": case_summaries,
        "comparisons": comparisons,
        "findings": findings,
    }
    (output_dir / "summary.json").write_text(
        json.dumps(summary, ensure_ascii=False, indent=2) + "\n", encoding="utf-8"
    )
    print(output_dir)
    return 1 if had_error else 0


if __name__ == "__main__":
    raise SystemExit(main())
