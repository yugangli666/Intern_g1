"""Pure helpers for converting InternNav outputs into one bounded velocity step."""

from dataclasses import asdict, dataclass
from math import atan2, copysign, isfinite, pi
from typing import Mapping, Sequence

import numpy as np


@dataclass(frozen=True)
class DirectMotionStep:
    kind: str
    source: str
    linear_x: float = 0.0
    angular_z: float = 0.0
    target_distance: float = 0.0
    target_yaw: float = 0.0
    reason: str = ""

    def as_dict(self) -> dict:
        return asdict(self)


@dataclass(frozen=True)
class FallbackTurnPlan:
    ok: bool
    reason: str = ""
    direction: float = 0.0
    angular_z: float = 0.0
    duration_s: float = 0.0
    actions_used: tuple[int, ...] = ()
    actions_raw: tuple[int, ...] = ()
    turn_used_degrees: float = 0.0
    remaining_degrees: float = 0.0

    def as_dict(self) -> dict:
        return asdict(self)


@dataclass(frozen=True)
class TurnOdomProgress:
    """Unwrapped odometry progress for a turn-only search."""

    last_yaw: float
    unwrapped_yaw: float = 0.0
    min_unwrapped_yaw: float = 0.0
    max_unwrapped_yaw: float = 0.0
    actual_travel_radians: float = 0.0

    @property
    def actual_travel_degrees(self) -> float:
        return self.actual_travel_radians * 180.0 / pi

    @property
    def yaw_coverage_degrees(self) -> float:
        return (self.max_unwrapped_yaw - self.min_unwrapped_yaw) * 180.0 / pi


def start_turn_odometry(yaw: float) -> TurnOdomProgress:
    if not isfinite(yaw):
        raise ValueError("yaw must be finite")
    return TurnOdomProgress(last_yaw=float(yaw))


def update_turn_odometry(progress: TurnOdomProgress, yaw: float) -> TurnOdomProgress:
    """Add one wrapped yaw observation and retain unwrapped search coverage."""
    if not isfinite(yaw):
        raise ValueError("yaw must be finite")
    delta = (float(yaw) - progress.last_yaw + pi) % (2.0 * pi) - pi
    unwrapped = progress.unwrapped_yaw + delta
    return TurnOdomProgress(
        last_yaw=float(yaw),
        unwrapped_yaw=unwrapped,
        min_unwrapped_yaw=min(progress.min_unwrapped_yaw, unwrapped),
        max_unwrapped_yaw=max(progress.max_unwrapped_yaw, unwrapped),
        actual_travel_radians=progress.actual_travel_radians + abs(delta),
    )


def expected_turn_yaw_sign(actions: Sequence[int]) -> float:
    """Return ROS yaw sign for a pure model turn sequence (2=left, 3=right)."""
    values = tuple(int(value) for value in actions)
    if not values or any(value not in {2, 3} for value in values):
        raise ValueError("expected a non-empty pure turn action sequence")
    return 1.0 if values[0] == 2 else -1.0


def turn_direction_mismatch(
    actions: Sequence[int], observed_yaw_delta_degrees: float, minimum_degrees: float
) -> bool:
    if minimum_degrees < 0.0:
        raise ValueError("minimum_degrees must be non-negative")
    if not isfinite(observed_yaw_delta_degrees):
        raise ValueError("observed yaw delta must be finite")
    if abs(observed_yaw_delta_degrees) < minimum_degrees:
        return False
    return observed_yaw_delta_degrees * expected_turn_yaw_sign(actions) < 0.0


def grounding_summary(response: Mapping, *, width: int, height: int) -> dict:
    """Summarize model grounding evidence without claiming object recognition."""
    if width <= 0 or height <= 0:
        raise ValueError("image dimensions must be positive")
    body = response if isinstance(response, Mapping) else {}
    raw_actions = body.get("discrete_action")
    raw_trajectory = body.get("trajectory")
    raw_pixel = body.get("pixel_goal")

    trajectory_points = len(raw_trajectory) if isinstance(raw_trajectory, list) else 0
    has_trajectory = trajectory_points > 0
    pixel_uv = None
    pixel_valid = False
    if isinstance(raw_pixel, Sequence) and not isinstance(raw_pixel, (str, bytes)) and len(raw_pixel) >= 2:
        try:
            pixel_v = float(raw_pixel[0])
            pixel_u = float(raw_pixel[1])
        except (TypeError, ValueError):
            pass
        else:
            if isfinite(pixel_u) and isfinite(pixel_v):
                pixel_uv = [int(round(pixel_u)), int(round(pixel_v))]
                pixel_valid = 0 <= pixel_u < width and 0 <= pixel_v < height

    if pixel_valid and has_trajectory:
        status = "PIXEL_AND_TRAJECTORY_PRESENT"
    elif pixel_valid:
        status = "PIXEL_GOAL_PRESENT"
    elif has_trajectory:
        status = "TRAJECTORY_PRESENT"
    else:
        status = "NO_GROUNDING_OUTPUT"

    return {
        "raw_discrete_action": raw_actions,
        "trajectory_present": has_trajectory,
        "trajectory_point_count": trajectory_points,
        "pixel_goal_raw": raw_pixel,
        "pixel_goal_uv": pixel_uv,
        "pixel_goal_valid": pixel_valid,
        "grounding_status": status,
        "target_locked": None,
        "target_lock_reason": "object_detector_not_configured",
    }


def fallback_turn_from_actions(
    actions: Sequence[int],
    *,
    turn_used_degrees: float,
    discrete_turn_degrees: float = 15.0,
    max_fallback_turn_degrees: float = 45.0,
    max_total_turn_degrees: float | None = None,
    angular_speed: float = 0.25,
) -> FallbackTurnPlan:
    """Convert G1-style discrete turn tokens into a bounded time window."""
    try:
        values = tuple(int(value) for value in actions)
    except (TypeError, ValueError):
        return FallbackTurnPlan(False, "discrete turn actions are malformed")
    if not values:
        return FallbackTurnPlan(False, "discrete turn actions are empty")
    if any(value not in {2, 3} for value in values):
        return FallbackTurnPlan(False, "not a pure turn action list", actions_raw=values)
    if discrete_turn_degrees <= 0.0:
        return FallbackTurnPlan(False, "discrete_turn_degrees must be positive", actions_raw=values)
    if max_fallback_turn_degrees < 0.0:
        return FallbackTurnPlan(False, "max_fallback_turn_degrees must be non-negative", actions_raw=values)
    total_budget = max_fallback_turn_degrees if max_total_turn_degrees is None else max_total_turn_degrees
    if total_budget < 0.0:
        return FallbackTurnPlan(False, "max_total_turn_degrees must be non-negative", actions_raw=values)
    if angular_speed <= 0.0:
        return FallbackTurnPlan(False, "angular_speed must be positive", actions_raw=values)

    remaining = max(0.0, total_budget - max(0.0, turn_used_degrees))
    segment_budget = min(max_fallback_turn_degrees, remaining)
    allowed_actions = min(len(values), int(segment_budget // discrete_turn_degrees))
    if allowed_actions <= 0:
        return FallbackTurnPlan(
            False,
            "fallback_turn_limit_reached",
            actions_raw=values,
            turn_used_degrees=turn_used_degrees,
            remaining_degrees=remaining,
        )

    direction = 1.0 if values[0] == 2 else -1.0
    used_now = allowed_actions * discrete_turn_degrees
    new_used = max(0.0, turn_used_degrees) + used_now
    duration = allowed_actions * (discrete_turn_degrees * pi / 180.0) / angular_speed
    return FallbackTurnPlan(
        True,
        direction=direction,
        angular_z=copysign(angular_speed, direction),
        duration_s=duration,
        actions_used=values[:allowed_actions],
        actions_raw=values,
        turn_used_degrees=new_used,
        remaining_degrees=max(0.0, total_budget - new_used),
    )


def _hold(source: str, reason: str) -> DirectMotionStep:
    return DirectMotionStep(kind="hold", source=source, reason=reason)


def _forward(
    source: str,
    *,
    allow_forward: bool,
    linear_speed: float,
    max_forward_distance: float,
    preview_forward: bool = False,
) -> DirectMotionStep:
    if not allow_forward:
        if preview_forward:
            # Dry-run preview: compute the nonzero linear_x that motion WOULD use,
            # but keep kind="preview_forward" so the caller never arms real motion.
            return DirectMotionStep(
                kind="preview_forward",
                source=source,
                linear_x=linear_speed,
                target_distance=max_forward_distance,
                reason="dry-run forward preview (motion locked)",
            )
        return _hold(source, "forward motion is locked")
    return DirectMotionStep(
        kind="forward",
        source=source,
        linear_x=linear_speed,
        target_distance=max_forward_distance,
    )


def _spin(source: str, direction: float, *, angular_speed: float, max_spin_degrees: float):
    target_yaw = copysign(max_spin_degrees * pi / 180.0, direction)
    return DirectMotionStep(
        kind="spin",
        source=source,
        angular_z=copysign(angular_speed, direction),
        target_yaw=target_yaw,
    )


def _trajectory_step(
    trajectory: Sequence[Sequence[float]],
    *,
    allow_forward: bool,
    linear_speed: float,
    angular_speed: float,
    max_forward_distance: float,
    max_spin_degrees: float,
    heading_deadband_degrees: float,
    lookahead_distance: float,
    max_jump: float,
    max_lateral: float,
    preview_forward: bool = False,
) -> DirectMotionStep:
    try:
        points = np.asarray(trajectory, dtype=np.float64)
    except (TypeError, ValueError):
        return _hold("trajectory", "trajectory is not numeric")
    if points.ndim != 2 or points.shape[0] == 0 or points.shape[1] < 2:
        return _hold("trajectory", "trajectory must be a non-empty Nx2 array")
    points = points[:, :2]
    if not np.all(np.isfinite(points)):
        return _hold("trajectory", "trajectory contains NaN or infinity")
    if np.linalg.norm(points[0]) > 1e-9:
        points = np.vstack((np.zeros((1, 2), dtype=np.float64), points))

    segment_lengths = np.linalg.norm(np.diff(points, axis=0), axis=1)
    cumulative = np.concatenate(([0.0], np.cumsum(segment_lengths)))
    prefix_end = int(np.searchsorted(cumulative, lookahead_distance, side="left"))
    prefix_end = min(max(prefix_end, 1), len(points) - 1)
    checked_points = points[: prefix_end + 1]
    checked_segments = segment_lengths[:prefix_end]
    if np.any(checked_segments > max_jump):
        return _hold("trajectory", "trajectory prefix contains an excessive jump")
    if np.any(np.abs(checked_points[:, 1]) > max_lateral):
        return _hold("trajectory", "trajectory prefix exceeds the lateral limit")

    target = points[prefix_end]
    if np.linalg.norm(target) <= 1e-6:
        return _hold("trajectory", "trajectory prefix has no movement")
    heading = atan2(float(target[1]), float(target[0]))
    deadband = heading_deadband_degrees * pi / 180.0
    if abs(heading) > deadband:
        return _spin(
            "trajectory_heading",
            heading,
            angular_speed=angular_speed,
            max_spin_degrees=max_spin_degrees,
        )
    if target[0] <= 0.0:
        return _hold("trajectory", "trajectory does not point forward")
    return _forward(
        "trajectory_forward",
        allow_forward=allow_forward,
        linear_speed=linear_speed,
        max_forward_distance=max_forward_distance,
        preview_forward=preview_forward,
    )


def direct_step_from_response(
    response: Mapping,
    *,
    allow_forward: bool = False,
    linear_speed: float = 0.04,
    angular_speed: float = 0.08,
    max_forward_distance: float = 0.04,
    max_spin_degrees: float = 3.0,
    heading_deadband_degrees: float = 8.0,
    lookahead_distance: float = 0.20,
    max_jump: float = 0.20,
    max_lateral: float = 0.12,
    preview_forward: bool = False,
) -> DirectMotionStep:
    """Select one short command from a model response.

    The helper deliberately does not execute a full action list or trajectory. The
    caller must stop, capture a new image, reset model history, and replan.

    ``preview_forward`` is a dry-run-only flag: when forward motion is locked
    (``allow_forward=False``) it lets the helper compute the nonzero linear
    velocity that motion WOULD use, returning a ``preview_forward`` step instead
    of a ``hold``. The caller must never arm real motion from such a step.
    """
    if not isinstance(response, Mapping):
        return _hold("response", "response is not a JSON object")
    if "discrete_action" in response:
        try:
            actions = [int(value) for value in response["discrete_action"]]
        except (TypeError, ValueError):
            return _hold("discrete_action", "discrete action is malformed")
        if not actions:
            return _hold("discrete_action", "discrete action is empty")
        if any(value in {0, 9} for value in actions):
            return DirectMotionStep(kind="stop", source="discrete_action", reason="model STOP")
        if any(value not in {1, 2, 3, 5} for value in actions):
            return _hold("discrete_action", "unknown discrete action")
        if 5 in actions:
            return _hold("discrete_action", "LOOK_DOWN has no direct motion mapping")
        if all(value == 1 for value in actions):
            return _forward(
                "discrete_forward",
                allow_forward=allow_forward,
                linear_speed=linear_speed,
                max_forward_distance=max_forward_distance,
                preview_forward=preview_forward,
            )
        if all(value in {2, 3} for value in actions):
            direction = sum(1 if value == 2 else -1 for value in actions)
            if direction == 0:
                return _hold("discrete_turn", "turn actions cancel each other")
            return _spin(
                "discrete_turn",
                direction,
                angular_speed=angular_speed,
                max_spin_degrees=max_spin_degrees,
            )
        return _hold("discrete_action", "mixed actions require a new observation")
    if "trajectory" in response:
        return _trajectory_step(
            response["trajectory"],
            allow_forward=allow_forward,
            linear_speed=linear_speed,
            angular_speed=angular_speed,
            max_forward_distance=max_forward_distance,
            max_spin_degrees=max_spin_degrees,
            heading_deadband_degrees=heading_deadband_degrees,
            lookahead_distance=lookahead_distance,
            max_jump=max_jump,
            max_lateral=max_lateral,
            preview_forward=preview_forward,
        )
    if "pixel_goal" in response:
        return _hold("pixel_goal", "pixel_goal has no direct velocity mapping")
    return _hold("response", "response contains no supported action")
