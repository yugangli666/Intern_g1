#!/usr/bin/env python3
"""Write a measured D455 optical-frame to G1 base-frame transform.

Measure the lens centre relative to the chosen base_link origin.  Roll, pitch,
and yaw describe the camera housing deviation from a level, forward-facing
chest mount; a level camera therefore uses all zeros.
"""

from __future__ import annotations

import argparse
import math
from pathlib import Path

import numpy as np
import yaml


R_LEVEL_BASE_FROM_OPTICAL = np.array([[0.0, 0.0, 1.0], [-1.0, 0.0, 0.0], [0.0, -1.0, 0.0]])


def rotation_from_rpy(roll_deg: float, pitch_deg: float, yaw_deg: float) -> np.ndarray:
    roll, pitch, yaw = np.deg2rad([roll_deg, pitch_deg, yaw_deg])
    rx = np.array([[1, 0, 0], [0, math.cos(roll), -math.sin(roll)], [0, math.sin(roll), math.cos(roll)]])
    ry = np.array([[math.cos(pitch), 0, math.sin(pitch)], [0, 1, 0], [-math.sin(pitch), 0, math.cos(pitch)]])
    rz = np.array([[math.cos(yaw), -math.sin(yaw), 0], [math.sin(yaw), math.cos(yaw), 0], [0, 0, 1]])
    return rz @ ry @ rx @ R_LEVEL_BASE_FROM_OPTICAL


def main() -> None:
    parser = argparse.ArgumentParser(description="Create a chest-D455 base_from_optical YAML file")
    parser.add_argument("--forward-offset-m", type=float, required=True)
    parser.add_argument("--left-offset-m", type=float, default=0.0)
    parser.add_argument("--height-m", type=float, required=True)
    parser.add_argument("--roll-deg", type=float, default=0.0)
    parser.add_argument("--pitch-deg", type=float, default=0.0)
    parser.add_argument("--yaw-deg", type=float, default=0.0)
    parser.add_argument("--output", type=Path, default=Path("configs/camera_to_base.yaml"))
    args = parser.parse_args()

    transform = np.eye(4)
    transform[:3, :3] = rotation_from_rpy(args.roll_deg, args.pitch_deg, args.yaw_deg)
    transform[:3, 3] = [args.forward_offset_m, args.left_offset_m, args.height_m]
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(
        yaml.safe_dump({"base_from_optical": transform.round(8).tolist()}, sort_keys=False), encoding="utf-8"
    )
    print(f"Wrote {args.output}")


if __name__ == "__main__":
    main()
