"""Pure-logic depth safety state machine and scheduler helpers.

No ROS, no numpy, no I/O — safe to import and unit-test directly.
This module is the **single source of truth** for the smooth client's
depth-safety debounce, recovery-gate, and hard-brake decisions.
"""

from __future__ import annotations

from enum import Enum, auto


class SafetyAction(Enum):
    """Action returned by DepthSafetyState.update() after each RGB-D frame."""

    NONE = auto()       # no change
    HOLD = auto()       # confirmed danger → enter safety hold
    EMERGENCY = auto()  # immediate emergency stop


class DepthSafetyState:
    """Depth-safety debounce state machine.

    Usage in the smooth client
    --------------------------
    1. **RGB-D callback** (every frame)::

           action = safety.update(clearance, frame_time)
           if action == SafetyAction.EMERGENCY:
               # immediately set emergency flag
           elif action == SafetyAction.HOLD:
               # set safety_hold flag

    2. **Control timer** (10 Hz), before accepting a local_goal::

           if safety.can_accept_goal(goal_frame_time):
               safety.accept_goal(goal_frame_time)
               # create MPC, clear hold, publish motion
           else:
               # reject — either still in hold, insufficient safe frames,
               # or the goal is stale (captured before the hold)

    3. **Control timer**, before publishing velocity::

           if safety.must_hard_brake():
               # publish v=0, w=0 directly (bypass _clip_rate)
    """

    def __init__(
        self,
        emergency_stop_m: float = 0.25,
        safety_stop_m: float = 0.45,
        safety_confirm_count: int = 3,
    ):
        if not (0.0 < emergency_stop_m < safety_stop_m):
            raise ValueError("emergency_stop_m must be in (0, safety_stop_m)")
        if safety_confirm_count < 1:
            raise ValueError("safety_confirm_count must be >= 1")

        self.emergency_stop_m = emergency_stop_m
        self.safety_stop_m = safety_stop_m
        self.safety_confirm_count = safety_confirm_count

        # ---- mutable state ----
        self.danger_count: int = 0
        """Consecutive frames with clearance in [emergency_stop_m, safety_stop_m)."""

        self.safe_after_hold_count: int = 0
        """Consecutive safe frames since entering hold (only incremented while
        safety_hold is True and clearance >= safety_stop_m)."""

        self.safety_hold: bool = False
        """True when the robot is in safety hold (emergency or confirmed danger)."""

        self._hold_frame_time: float | None = None
        """Frame timestamp when the current hold was entered.  Used to reject
        stale HTTP responses whose frame was captured before the hold."""

    # ------------------------------------------------------------------
    # Per-frame update (called from RGB-D callback, every frame)
    # ------------------------------------------------------------------

    def update(self, clearance: float | None, frame_time: float = 0.0) -> SafetyAction:
        """Ingest one depth reading.  Returns the action to take.

        Parameters
        ----------
        clearance:
            Front clearance in metres, or ``None`` if the depth ROI is invalid.
        frame_time:
            ROS-header timestamp of this frame (used as hold-entry marker).
        """
        if clearance is None:
            return SafetyAction.NONE

        # ── emergency ────────────────────────────────────────────────
        if clearance < self.emergency_stop_m:
            self.danger_count = self.safety_confirm_count  # force-trigger
            self.safe_after_hold_count = 0
            if not self.safety_hold:
                self.safety_hold = True
                self._hold_frame_time = frame_time
            return SafetyAction.EMERGENCY

        # ── danger zone ──────────────────────────────────────────────
        if clearance < self.safety_stop_m:
            self.danger_count += 1
            self.safe_after_hold_count = 0
            if self.danger_count >= self.safety_confirm_count and not self.safety_hold:
                self.safety_hold = True
                self._hold_frame_time = frame_time
                return SafetyAction.HOLD
            return SafetyAction.NONE

        # ── safe ─────────────────────────────────────────────────────
        self.danger_count = 0
        if self.safety_hold:
            self.safe_after_hold_count += 1
        return SafetyAction.NONE

    # ------------------------------------------------------------------
    # Goal-acceptance gate (called from control timer)
    # ------------------------------------------------------------------

    def can_accept_goal(self, goal_frame_time: float) -> bool:
        """Return ``True`` if a local_goal from *goal_frame_time* may be accepted.

        The goal is rejected when:
        - The robot is in emergency or confirmed-danger state.
        - The robot is in hold but hasn't seen enough safe frames yet.
        - The goal's frame was captured **before** the hold began (i.e. a
          stale HTTP response that was in-flight when the hold triggered).
        """
        # Emergency or confirmed danger → never accept
        if self.danger_count >= self.safety_confirm_count:
            return False

        # Not in hold → accept freely
        if not self.safety_hold:
            return True

        # In hold: need enough safe frames
        if self.safe_after_hold_count < self.safety_confirm_count:
            return False

        # Reject stale goals (captured before the hold)
        if self._hold_frame_time is not None and goal_frame_time <= self._hold_frame_time:
            return False

        return True

    def accept_goal(self, goal_frame_time: float) -> None:
        """Atomically clear hold state.  Call ONLY after ``can_accept_goal``
        has returned ``True``."""
        self.safety_hold = False
        self.danger_count = 0
        self.safe_after_hold_count = 0
        self._hold_frame_time = None

    # ------------------------------------------------------------------
    # Emergency braking gate (called from control timer, every tick)
    # ------------------------------------------------------------------

    def must_hard_brake(self) -> bool:
        """Return ``True`` if the velocity command must jump directly to zero,
        bypassing the normal rate-limiter.

        True when  safety_hold is active  OR  danger_count has reached the
        confirm threshold (even if the hold flag hasn't been set yet by the
        control timer — this catches the tick immediately after the RGB-D
        callback sets the counters).
        """
        return self.safety_hold or self.danger_count >= self.safety_confirm_count

    # ------------------------------------------------------------------
    # Read-only introspection
    # ------------------------------------------------------------------

    @property
    def in_hold(self) -> bool:
        return self.safety_hold

    @property
    def hold_frame_time(self) -> float | None:
        return self._hold_frame_time
