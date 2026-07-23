#!/usr/bin/env python3
"""Summarize motion, command, and safety signals from an InternNav rosbag."""

import argparse
import json
import math
import statistics
from pathlib import Path

import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message


def yaw_from_quaternion(orientation) -> float:
    siny_cosp = 2.0 * (
        orientation.w * orientation.z + orientation.x * orientation.y
    )
    cosy_cosp = 1.0 - 2.0 * (
        orientation.y * orientation.y + orientation.z * orientation.z
    )
    return math.atan2(siny_cosp, cosy_cosp)


def _duration_template() -> dict[str, float | int | None]:
    return {
        "linear_nonzero_s": 0.0,
        "angular_nonzero_s": 0.0,
        "nonzero_s": 0.0,
        "samples": 0,
    }


def _close_duration(topic_state: dict, timestamp_ns: int) -> None:
    last_ts = topic_state.get("last_ts")
    if last_ts is None:
        topic_state["last_ts"] = timestamp_ns
        return
    dt = max(0.0, (timestamp_ns - last_ts) / 1e9)
    if topic_state.get("last_linear_nonzero"):
        topic_state["linear_nonzero_s"] += dt
    if topic_state.get("last_angular_nonzero"):
        topic_state["angular_nonzero_s"] += dt
    if topic_state.get("last_any_nonzero"):
        topic_state["nonzero_s"] += dt
    topic_state["last_ts"] = timestamp_ns


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("bag")
    parser.add_argument("--output")
    args = parser.parse_args()

    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=args.bag, storage_id="mcap"),
        rosbag2_py.ConverterOptions("", ""),
    )
    topic_types = {
        item.name: get_message(item.type) for item in reader.get_all_topics_and_types()
    }
    counts: dict[str, int] = {}
    twist_topics = ("/cmd_vel_nav", "/cmd_vel_smoothed", "/cmd_vel")
    max_twist = {topic: {"linear": 0.0, "angular": 0.0} for topic in twist_topics}
    # Dry-run preview topic: NOT a real-chassis topic. Tracked separately so the
    # preview stage can assert nonzero previews here while chassis topics stay zero.
    dry_run_topic = "/internnav/dry_run_cmd_vel"
    dry_run_stats = {"linear": 0.0, "angular": 0.0, "samples": 0, "nonzero_samples": 0}
    durations = {topic: _duration_template() for topic in (*twist_topics, "/mx_base_vel_command")}
    duration_state = {topic: {**durations[topic], "last_ts": None} for topic in durations}
    max_base_cmd = {"linear": 0.0, "angular": 0.0}
    first_pose = None
    last_pose = None
    max_displacement = 0.0
    status_states: list[str] = []
    emergency_messages = 0
    max_status_fallback_turn_deg = 0.0
    max_status_fallback_turn_actual_travel_deg = 0.0
    max_status_fallback_turn_yaw_coverage_deg = 0.0
    saw_odom_turn_coverage = False
    fault_codes: list[str] = []
    last_nav_nonzero_ns = None
    nav_to_base_delay_s: list[float] = []

    while reader.has_next():
        topic, data, timestamp_ns = reader.read_next()
        counts[topic] = counts.get(topic, 0) + 1
        message_type = topic_types.get(topic)
        if message_type is None:
            continue
        if topic not in {
            "/cmd_vel",
            "/cmd_vel_nav",
            "/cmd_vel_smoothed",
            "/mx_base_vel_command",
            "/internnav/dry_run_cmd_vel",
            "/moz1/odom_global",
            "/internnav/status",
            "/emergency_stop",
        }:
            continue
        message = deserialize_message(data, message_type)
        if topic in twist_topics:
            linear = math.hypot(message.linear.x, message.linear.y)
            angular = abs(message.angular.z)
            max_twist[topic]["linear"] = max(max_twist[topic]["linear"], linear)
            max_twist[topic]["angular"] = max(max_twist[topic]["angular"], angular)
            state = duration_state[topic]
            _close_duration(state, timestamp_ns)
            state["last_linear_nonzero"] = linear > 0.005
            state["last_angular_nonzero"] = angular > 0.005
            state["last_any_nonzero"] = linear > 0.005 or angular > 0.005
            state["samples"] += 1
            if topic == "/cmd_vel_nav" and state["last_any_nonzero"]:
                last_nav_nonzero_ns = timestamp_ns
        elif topic == "/mx_base_vel_command":
            linear = math.hypot(message.x, message.y)
            angular = abs(message.z)
            max_base_cmd["linear"] = max(max_base_cmd["linear"], linear)
            max_base_cmd["angular"] = max(max_base_cmd["angular"], angular)
            state = duration_state[topic]
            _close_duration(state, timestamp_ns)
            state["last_linear_nonzero"] = linear > 0.005
            state["last_angular_nonzero"] = angular > 0.005
            state["last_any_nonzero"] = linear > 0.005 or angular > 0.005
            state["samples"] += 1
            if state["last_any_nonzero"] and last_nav_nonzero_ns is not None:
                nav_to_base_delay_s.append((timestamp_ns - last_nav_nonzero_ns) / 1e9)
        elif topic == dry_run_topic:
            linear = math.hypot(message.linear.x, message.linear.y)
            angular = abs(message.angular.z)
            dry_run_stats["linear"] = max(dry_run_stats["linear"], linear)
            dry_run_stats["angular"] = max(dry_run_stats["angular"], angular)
            dry_run_stats["samples"] += 1
            if linear > 0.005 or angular > 0.005:
                dry_run_stats["nonzero_samples"] += 1
        elif topic == "/moz1/odom_global":
            position = message.pose.pose.position
            pose = {
                "x": float(position.x),
                "y": float(position.y),
                "yaw": yaw_from_quaternion(message.pose.pose.orientation),
            }
            if first_pose is None:
                first_pose = pose
            last_pose = pose
            max_displacement = max(
                max_displacement,
                math.hypot(pose["x"] - first_pose["x"], pose["y"] - first_pose["y"]),
            )
        elif topic == "/internnav/status":
            try:
                status = json.loads(message.data)
                state = status.get("state")
                fallback_used = float(status.get("fallback_turn_used_deg") or 0.0)
                max_status_fallback_turn_deg = max(max_status_fallback_turn_deg, fallback_used)
                actual_travel = float(
                    status.get("fallback_turn_actual_travel_deg") or 0.0
                )
                if "fallback_turn_yaw_coverage_deg" in status:
                    saw_odom_turn_coverage = True
                yaw_coverage = float(
                    status.get("fallback_turn_yaw_coverage_deg") or 0.0
                )
                max_status_fallback_turn_actual_travel_deg = max(
                    max_status_fallback_turn_actual_travel_deg, actual_travel
                )
                max_status_fallback_turn_yaw_coverage_deg = max(
                    max_status_fallback_turn_yaw_coverage_deg, yaw_coverage
                )
                fault_code = status.get("fault_code")
                if fault_code and fault_code not in fault_codes:
                    fault_codes.append(str(fault_code))
            except (json.JSONDecodeError, AttributeError, TypeError, ValueError):
                state = None
            if state and (not status_states or status_states[-1] != state):
                status_states.append(state)
        elif topic == "/emergency_stop" and message.data:
            emergency_messages += 1

    yaw_change = None
    if first_pose is not None and last_pose is not None:
        yaw_change = (last_pose["yaw"] - first_pose["yaw"] + math.pi) % (2.0 * math.pi) - math.pi

    cleaned_durations = {}
    for topic, state in duration_state.items():
        cleaned_durations[topic] = {
            "linear_nonzero_s": state["linear_nonzero_s"],
            "angular_nonzero_s": state["angular_nonzero_s"],
            "nonzero_s": state["nonzero_s"],
            "samples": state["samples"],
        }

    delay_summary = None
    if nav_to_base_delay_s:
        delay_summary = {
            "count": len(nav_to_base_delay_s),
            "min": min(nav_to_base_delay_s),
            "median": statistics.median(nav_to_base_delay_s),
            "max": max(nav_to_base_delay_s),
        }

    yaw_change_deg = math.degrees(yaw_change) if yaw_change is not None else None
    summary = {
        "bag": str(Path(args.bag).resolve()),
        "topic_counts": counts,
        "max_twist_commands": max_twist,
        "max_base_command": max_base_cmd,
        "dry_run_cmd_vel": dry_run_stats,
        "nonzero_command_durations": cleaned_durations,
        "approx_cmd_vel_nav_to_base_delay_s": delay_summary,
        "first_pose": first_pose,
        "last_pose": last_pose,
        "max_displacement_m": max_displacement,
        "yaw_change_rad": yaw_change,
        "yaw_change_deg": yaw_change_deg,
        "max_status_fallback_turn_used_deg": max_status_fallback_turn_deg,
        "max_status_fallback_turn_actual_travel_deg": (
            max_status_fallback_turn_actual_travel_deg
        ),
        "max_status_fallback_turn_yaw_coverage_deg": (
            max_status_fallback_turn_yaw_coverage_deg
        ),
        "fallback_turn_budget_source": (
            "odom_yaw_coverage" if saw_odom_turn_coverage else "legacy_nominal_tokens"
        ),
        "yaw_minus_fallback_turn_deg": (
            abs(yaw_change_deg) - max_status_fallback_turn_deg
            if yaw_change_deg is not None
            else None
        ),
        "status_states": status_states,
        "fault_codes": fault_codes,
        "emergency_messages": emergency_messages,
    }
    output = json.dumps(summary, indent=2)
    print(output)
    if args.output:
        Path(args.output).write_text(output + "\n", encoding="utf-8")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
