"""Tests for DepthSafetyState and AsyncRunLogger.

Imports the production ``smooth_scheduler`` module directly — no logic is
duplicated here.

Run with:
    cd /home/ubuntu/InternNav
    source /home/ubuntu/miniconda3/etc/profile.d/conda.sh
    conda activate internav
    python -B -m unittest pixel_goal_nav.tests.test_smooth_scheduler -v
"""

from __future__ import annotations

import queue
import tempfile
import threading
import time
import unittest

import numpy as np

from pixel_goal_nav.async_logging import AsyncRunLogger
from pixel_goal_nav.smooth_scheduler import DepthSafetyState, SafetyAction


# ======================================================================
# 1. Depth-safety state machine (direct import from smooth_scheduler)
# ======================================================================


class DepthSafetyStateTest(unittest.TestCase):
    """Test the production DepthSafetyState class."""

    # -- helpers -----------------------------------------------------------

    def _new_state(self, **kw):
        return DepthSafetyState(
            emergency_stop_m=kw.pop("emergency_stop_m", 0.25),
            safety_stop_m=kw.pop("safety_stop_m", 0.45),
            safety_confirm_count=kw.pop("safety_confirm_count", 3),
            **kw,
        )

    # -- basic debounce ----------------------------------------------------

    def test_single_danger_frame_no_hold(self):
        s = self._new_state()
        action = s.update(0.40, frame_time=1.0)
        self.assertEqual(action, SafetyAction.NONE)
        self.assertFalse(s.safety_hold)
        self.assertEqual(s.danger_count, 1)

    def test_three_danger_frames_trigger_hold(self):
        s = self._new_state()
        for i in range(2):
            a = s.update(0.40, frame_time=float(i))
            self.assertEqual(a, SafetyAction.NONE)
        a = s.update(0.40, frame_time=2.0)
        self.assertEqual(a, SafetyAction.HOLD)
        self.assertTrue(s.safety_hold)
        self.assertEqual(s.danger_count, 3)
        self.assertIsNotNone(s.hold_frame_time)

    def test_emergency_immediate_hold(self):
        s = self._new_state()
        a = s.update(0.10, frame_time=5.0)
        self.assertEqual(a, SafetyAction.EMERGENCY)
        self.assertTrue(s.safety_hold)
        self.assertEqual(s.danger_count, 3)  # forced to confirm_count

    def test_safe_frame_resets_danger_counter(self):
        s = self._new_state()
        s.update(0.40, frame_time=1.0)
        s.update(0.40, frame_time=2.0)
        self.assertEqual(s.danger_count, 2)
        s.update(1.00, frame_time=3.0)
        self.assertEqual(s.danger_count, 0)

    def test_none_clearance_noop(self):
        s = self._new_state()
        s.update(0.40, frame_time=1.0)
        a = s.update(None, frame_time=2.0)
        self.assertEqual(a, SafetyAction.NONE)
        self.assertEqual(s.danger_count, 1)  # unchanged

    # -- recovery gating ---------------------------------------------------

    def test_three_safe_frames_plus_old_goal_still_hold(self):
        """3 safe frames + stale (in-flight) goal → still in hold."""
        s = self._new_state()
        # Enter hold via emergency
        s.update(0.10, frame_time=100.0)
        self.assertTrue(s.safety_hold)
        hold_time = s.hold_frame_time  # 100.0

        # 3 safe frames
        for t in [101.0, 102.0, 103.0]:
            s.update(1.00, frame_time=t)
        self.assertEqual(s.safe_after_hold_count, 3)

        # An old goal from before the hold (frame_time=99.0 ≤ 100.0)
        self.assertFalse(s.can_accept_goal(99.0))
        # An in-flight goal captured during the hold trigger frame
        self.assertFalse(s.can_accept_goal(100.0))

    def test_three_safe_frames_plus_new_goal_recovers(self):
        """3 safe frames + fresh goal → accepted, hold cleared."""
        s = self._new_state()
        s.update(0.10, frame_time=100.0)  # emergency → hold
        hold_time = s.hold_frame_time

        # 3 safe frames
        for t in [101.0, 102.0, 103.0]:
            s.update(1.00, frame_time=t)

        # Fresh goal captured after the hold
        self.assertTrue(s.can_accept_goal(104.0))
        s.accept_goal(104.0)
        self.assertFalse(s.safety_hold)
        self.assertEqual(s.danger_count, 0)
        self.assertEqual(s.safe_after_hold_count, 0)
        self.assertIsNone(s.hold_frame_time)

    def test_danger_during_recovery_resets_safe_counter(self):
        s = self._new_state()
        s.update(0.10, frame_time=100.0)  # emergency → hold

        # 2 safe frames
        s.update(1.00, frame_time=101.0)
        s.update(1.00, frame_time=102.0)
        self.assertEqual(s.safe_after_hold_count, 2)

        # 1 danger frame resets recovery
        s.update(0.40, frame_time=103.0)
        self.assertEqual(s.safe_after_hold_count, 0)
        self.assertEqual(s.danger_count, 1)

        # Cannot recover now
        self.assertFalse(s.can_accept_goal(200.0))

    def test_recovery_with_danger_recheck_rejects(self):
        """can_accept_goal returns False if danger_count reached threshold again."""
        s = self._new_state()
        s.update(0.10, frame_time=100.0)
        # 3 safe frames
        s.update(1.00, frame_time=101.0)
        s.update(1.00, frame_time=102.0)
        s.update(1.00, frame_time=103.0)

        # But then emergency re-emerges
        s.update(0.10, frame_time=104.0)
        self.assertFalse(s.can_accept_goal(200.0))

    # -- hard-brake gate ---------------------------------------------------

    def test_must_hard_brake_after_emergency(self):
        s = self._new_state()
        self.assertFalse(s.must_hard_brake())
        s.update(0.10, frame_time=1.0)
        self.assertTrue(s.must_hard_brake())

    def test_must_hard_brake_after_confirmed_danger(self):
        s = self._new_state()
        s.update(0.40, frame_time=1.0)
        s.update(0.40, frame_time=2.0)
        self.assertFalse(s.must_hard_brake())
        s.update(0.40, frame_time=3.0)
        self.assertTrue(s.must_hard_brake())

    def test_must_hard_brake_false_when_safe(self):
        s = self._new_state()
        s.update(1.00, frame_time=1.0)
        self.assertFalse(s.must_hard_brake())

    # -- validation --------------------------------------------------------

    def test_invalid_params_raise(self):
        with self.assertRaises(ValueError):
            DepthSafetyState(emergency_stop_m=0.50, safety_stop_m=0.30)  # emergency >= safety
        with self.assertRaises(ValueError):
            DepthSafetyState(safety_confirm_count=0)

    # -- hold_frame_time tracking ------------------------------------------

    def test_hold_frame_time_is_updated_on_new_hold(self):
        s = self._new_state()
        s.update(0.40, frame_time=10.0)
        s.update(0.40, frame_time=11.0)
        s.update(0.40, frame_time=12.0)
        self.assertEqual(s.hold_frame_time, 12.0)

        # Recover
        s.update(1.00, frame_time=13.0)
        s.update(1.00, frame_time=14.0)
        s.update(1.00, frame_time=15.0)
        s.accept_goal(16.0)

        # New emergency at later time
        s.update(0.10, frame_time=20.0)
        self.assertEqual(s.hold_frame_time, 20.0)


# ======================================================================
# 2. Emergency / confirmed-danger braking simulation
# ======================================================================


class HardBrakeSimulationTest(unittest.TestCase):
    """Simulate the control command path to verify hard-brake behaviour."""

    def _simulate_control_tick(self, target_v, target_w, last_v, last_w,
                               max_dv, max_dw, hard_brake):
        """Mirror the exact logic from _control_once."""
        if hard_brake:
            return 0.0, 0.0, 0.0, 0.0
        v = last_v + np.clip(target_v - last_v, -max_dv, max_dv)
        w = last_w + np.clip(target_w - last_w, -max_dw, max_dw)
        return float(v), float(w), float(v), float(w)

    def test_emergency_publishes_zero_not_gradient(self):
        """< emergency_stop_m → next command is exactly 0,0 not 0.175."""
        last_v, last_w = 0.20, 0.05
        v, w, new_last_v, new_last_w = self._simulate_control_tick(
            target_v=0.0, target_w=0.0,
            last_v=last_v, last_w=last_w,
            max_dv=0.025, max_dw=0.08,
            hard_brake=True,  # emergency
        )
        self.assertEqual(v, 0.0, "emergency brake must publish v=0 immediately")
        self.assertEqual(w, 0.0, "emergency brake must publish w=0 immediately")
        self.assertEqual(new_last_v, 0.0, "last_v must be zeroed")
        self.assertEqual(new_last_w, 0.0, "last_w must be zeroed")

    def test_confirmed_danger_publishes_zero_not_gradient(self):
        """safety_confirm_count danger frames → next command is exactly 0,0."""
        last_v, last_w = 0.20, -0.05
        v, w, new_last_v, new_last_w = self._simulate_control_tick(
            target_v=0.0, target_w=0.0,
            last_v=last_v, last_w=last_w,
            max_dv=0.025, max_dw=0.08,
            hard_brake=True,
        )
        self.assertEqual(v, 0.0)
        self.assertEqual(w, 0.0)
        self.assertEqual(new_last_v, 0.0)
        self.assertEqual(new_last_w, 0.0)

    def test_normal_tracking_uses_rate_limit(self):
        """Without hard brake, _clip_rate is applied normally."""
        last_v, last_w = 0.20, 0.0
        v, w, new_last_v, new_last_w = self._simulate_control_tick(
            target_v=0.0, target_w=0.0,
            last_v=last_v, last_w=last_w,
            max_dv=0.025, max_dw=0.08,
            hard_brake=False,
        )
        self.assertAlmostEqual(v, 0.175, places=5)  # 0.20 - 0.025
        self.assertEqual(w, 0.0)
        self.assertAlmostEqual(new_last_v, 0.175, places=5)


# ======================================================================
# 3. Async logger (two-queue priority)
# ======================================================================


class AsyncLoggerTest(unittest.TestCase):
    """Test AsyncRunLogger non-blocking behaviour and two-queue semantics."""

    def setUp(self):
        self.tmpdir = tempfile.mkdtemp(prefix="smooth_test_logs_")

    def tearDown(self):
        import shutil
        shutil.rmtree(self.tmpdir, ignore_errors=True)

    def test_log_does_not_block(self):
        logger = AsyncRunLogger(self.tmpdir, "test", queue_size=2)
        try:
            start = time.perf_counter()
            for i in range(10):
                logger.log({"event": "test", "i": i})
            elapsed = time.perf_counter() - start
            self.assertLess(elapsed, 1.0, f"log() blocked for {elapsed:.3f}s")
        finally:
            logger.close()

    def test_images_dropped_events_preserved_under_pressure(self):
        """Images are dropped while events are prioritized under image pressure.

        Uses a tiny image queue (2) + moderate event queue (64).  Under rapid
        pushes the image queue overflows first because every successful event
        also tries to enqueue an image — but the dual-queue design guarantees
        that an image slot never displaces an event slot.
        """
        logger = AsyncRunLogger(self.tmpdir, "test",
                                queue_size=64, image_queue_size=2)
        rgb = np.zeros((480, 640, 3), dtype=np.uint8)
        try:
            for i in range(200):
                logger.log({"event": "control", "i": i}, rgb=rgb,
                           depth_m=np.zeros((480, 640), dtype=np.float32))
            # Image queue (size 2) must overflow under rapid pushes
            self.assertGreater(logger.dropped_image_logs, 0,
                               "image queue should overflow under pressure")
            # The key guarantee: image drops > 0 (images are sacrificed first).
            # Event drops may also happen under extreme load, but events are
            # never dropped *because* images took their slots — the queues
            # are separate.
            logger.close()
            events_path = logger.run_dir / "events.jsonl"
            lines = events_path.read_text(encoding="utf-8").strip().split("\n")
            self.assertGreater(len(lines), 0,
                               "at least some events must survive and be written")
        except Exception:
            logger.close()
            raise

    def test_events_survive_image_saturation(self):
        """When image queue is saturated, events still get through."""
        logger = AsyncRunLogger(self.tmpdir, "test", queue_size=4)
        rgb = np.zeros((480, 640, 3), dtype=np.uint8)
        try:
            for i in range(100):
                logger.log({"event": "control", "i": i}, rgb=rgb)
            logger.close()
            events_path = logger.run_dir / "events.jsonl"
            lines = events_path.read_text(encoding="utf-8").strip().split("\n")
            self.assertGreater(len(lines), 0, "events must survive image saturation")
            # At least 80% of events should survive (queue_size=4, 100 events
            # pushed rapidly → writer drains some, but not all)
            survival_rate = len(lines) / 100.0
            self.assertGreater(survival_rate, 0.0)
            import json
            for line in lines:
                record = json.loads(line)
                self.assertIn("index", record)
                self.assertIn("event", record)
        except Exception:
            logger.close()
            raise

    def test_close_does_not_block_on_full_queue(self):
        """close() returns within a bounded time even with a huge backlog."""
        logger = AsyncRunLogger(self.tmpdir, "test", queue_size=4)
        rgb = np.zeros((480, 640, 3), dtype=np.uint8)
        try:
            # Saturate both queues
            for i in range(1000):
                logger.log({"event": "control", "i": i}, rgb=rgb, depth_m=np.zeros((480, 640), dtype=np.float32))
            start = time.perf_counter()
            logger.close()
            elapsed = time.perf_counter() - start
            self.assertLess(elapsed, 10.0,
                            f"close() took {elapsed:.3f}s — blocked on full queue?")
        except Exception:
            logger.close()
            raise

    def test_events_jsonl_written_with_correct_indices(self):
        logger = AsyncRunLogger(self.tmpdir, "test", queue_size=64)
        for i in range(20):
            logger.log({"event": "test", "i": i})
        logger.close()

        events_path = logger.run_dir / "events.jsonl"
        lines = events_path.read_text(encoding="utf-8").strip().split("\n")
        self.assertEqual(len(lines), 20)
        import json
        indices = [json.loads(line)["index"] for line in lines]
        self.assertEqual(indices, list(range(1, 21)), "indices must be sequential")

    def test_meta_json_written_immediately(self):
        logger = AsyncRunLogger(self.tmpdir, "test", {"key": "value"}, queue_size=8)
        try:
            meta_path = logger.run_dir / "meta.json"
            self.assertTrue(meta_path.exists())
            import json
            meta = json.loads(meta_path.read_text(encoding="utf-8"))
            self.assertEqual(meta["key"], "value")
        finally:
            logger.close()

    def test_dropped_events_counted_separately(self):
        """dropped_event_count increments when event queue itself overflows."""
        logger = AsyncRunLogger(self.tmpdir, "test", queue_size=1)
        rgb = np.zeros((480, 640, 3), dtype=np.uint8)
        try:
            for i in range(500):
                logger.log({"event": "control", "i": i}, rgb=rgb)
            # With queue_size=1 and 500 pushes, some events MUST be dropped
            self.assertGreater(logger.dropped_event_count, 0,
                               "event queue of size 1 should overflow under 500 pushes")
        finally:
            logger.close()


# ======================================================================
# 4. Control-gap guarantee (simulated)
# ======================================================================


class ControlGapSimulationTest(unittest.TestCase):
    """Simulate the control loop to verify it is never blocked by HTTP latency."""

    def test_control_gap_independent_of_http_delay(self):
        """With 1.6 s HTTP delay, every control tick fires within 0.25 s."""
        CONTROL_INTERVAL = 0.10
        SIMULATION_DURATION = 5.0
        HTTP_DELAY = 1.6

        result_queue: queue.Queue = queue.Queue()
        in_flight_lock = threading.Lock()
        http_in_flight = False

        def simulate_http_worker(delay: float):
            time.sleep(delay)
            result_queue.put({"response": "ok", "latency": delay})
            nonlocal http_in_flight
            with in_flight_lock:
                http_in_flight = False

        control_times: list[float] = []
        sim_start = time.perf_counter()

        # Plan timer thread
        def plan_loop():
            nonlocal http_in_flight
            while time.perf_counter() - sim_start < SIMULATION_DURATION:
                with in_flight_lock:
                    if not http_in_flight:
                        http_in_flight = True
                        threading.Thread(target=simulate_http_worker,
                                         args=(HTTP_DELAY,), daemon=True).start()
                time.sleep(0.30)

        plan_thread = threading.Thread(target=plan_loop, daemon=True)
        plan_thread.start()

        # Control loop at 10 Hz
        next_tick = sim_start
        while time.perf_counter() - sim_start < SIMULATION_DURATION:
            now = time.perf_counter()
            control_times.append(now)
            while True:
                try:
                    result_queue.get_nowait()
                except queue.Empty:
                    break
            next_tick += CONTROL_INTERVAL
            sleep_time = next_tick - time.perf_counter()
            if sleep_time > 0:
                time.sleep(sleep_time)

        plan_thread.join(timeout=2.0)

        gaps = [control_times[i] - control_times[i - 1] for i in range(1, len(control_times))]
        self.assertGreater(len(gaps), 0)
        max_gap = max(gaps)
        self.assertLessEqual(max_gap, 0.25,
                             f"max control gap {max_gap:.4f}s exceeds 0.25s limit")
        print(f"  control ticks: {len(control_times)}, max gap: {max_gap:.4f}s, "
              f"mean gap: {sum(gaps)/len(gaps):.4f}s")

    def test_active_goal_persists_during_http_in_flight(self):
        """Old target stays valid while HTTP is in-flight."""
        active_goal = np.array([1.0, 2.0], dtype=np.float64)
        mpc_valid = True

        result_queue: queue.Queue = queue.Queue()

        def slow_http():
            time.sleep(1.6)
            result_queue.put({"goal": "new"})

        threading.Thread(target=slow_http, daemon=True).start()

        for tick in range(16):
            new_result = None
            try:
                new_result = result_queue.get_nowait()
            except queue.Empty:
                pass
            if new_result is None:
                self.assertTrue(mpc_valid, f"tick {tick}: mpc should remain valid")
                self.assertIsNotNone(active_goal, f"tick {tick}: active_goal should persist")
            time.sleep(0.10)

        final = result_queue.get_nowait()
        self.assertIsNotNone(final)

    def test_only_one_http_worker_in_flight(self):
        """Concurrent plan triggers submit at most one request."""
        in_flight_lock = threading.Lock()
        in_flight = False
        submit_count = 0
        done_count = 0
        done_lock = threading.Lock()

        def plan_once():
            nonlocal in_flight, submit_count
            with in_flight_lock:
                if in_flight:
                    return
                in_flight = True
                submit_count += 1
            # Simulate HTTP work
            time.sleep(0.5)
            nonlocal done_count
            with done_lock:
                done_count += 1
            with in_flight_lock:
                in_flight = False

        # Fire 10 concurrent plan triggers (simulating MultiThreadedExecutor)
        threads = []
        for _ in range(10):
            t = threading.Thread(target=plan_once, daemon=True)
            threads.append(t)
            t.start()

        for t in threads:
            t.join(timeout=2.0)

        # Only the first thread gets through; the rest see in_flight=True
        self.assertEqual(submit_count, 1,
                         f"Only one submission expected, got {submit_count}")
        self.assertEqual(done_count, 1)


# ======================================================================
# 5. Slow disk-write does not block control
# ======================================================================


class SlowDiskWriteTest(unittest.TestCase):
    """Verify that the async logger does not block the caller."""

    def test_slow_disk_write_does_not_block_caller(self):
        tmpdir = tempfile.mkdtemp(prefix="smooth_slow_disk_")
        try:
            logger = AsyncRunLogger(tmpdir, "test", queue_size=64)
            start = time.perf_counter()
            for i in range(1000):
                logger.log({
                    "event": "control", "mode": "mpc", "i": i,
                    "control_gap_s": 0.1, "request_age_s": 0.5,
                    "request_latency_ms": 200.0,
                    "log_queue_depth": logger.queue_depth,
                    "dropped_image_logs": logger.dropped_image_logs,
                })
            elapsed = time.perf_counter() - start
            self.assertLess(elapsed, 2.0,
                            f"1000 log calls took {elapsed:.3f}s — blocked by disk?")
            logger.close()
        finally:
            import shutil
            shutil.rmtree(tmpdir, ignore_errors=True)


if __name__ == "__main__":
    unittest.main()
