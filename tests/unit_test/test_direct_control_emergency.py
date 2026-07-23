import sys
from pathlib import Path
from types import SimpleNamespace


REALWORLD_DIR = Path(__file__).resolve().parents[2] / "scripts" / "realworld"
sys.path.insert(0, str(REALWORLD_DIR))

from internnav_direct_control_client import InternNavDirectControlClient  # noqa: E402


class RecordingPublisher:
    def __init__(self):
        self.messages = []

    def publish(self, message):
        self.messages.append(message)


def test_direction_mismatch_uses_existing_emergency_stop_chain():
    client = InternNavDirectControlClient.__new__(InternNavDirectControlClient)
    client.args = SimpleNamespace(stop_burst_seconds=1.0)
    client._emergency = False
    client._motion = object()
    client._mpc_active = True
    client._mpc_goal_world = object()
    client._mpc_waiting_for_fresh_odom = True
    client._mpc_arm_odom_received = 1.0
    client._fault_code = None
    client._fault_details = {}
    client._done = False
    client.command_pub = RecordingPublisher()
    client.emergency_pub = RecordingPublisher()
    client.stop_task_pub = RecordingPublisher()
    client._clear_fallback_turn = lambda reset_budget=False: None
    events = []
    client._append_event = events.append
    states = []
    client._set_state = lambda state, reason: states.append((state, reason))

    client._trigger_emergency(
        "opposite yaw",
        fault_code="TURN_DIRECTION_MISMATCH",
        details={
            "expected_yaw_sign": 1.0,
            "observed_yaw_delta_deg": -4.2,
            "command_angular_z": 0.2,
        },
    )

    assert client._emergency
    assert client._done
    assert client._fault_code == "TURN_DIRECTION_MISMATCH"
    assert len(client.command_pub.messages) == 2
    assert client.command_pub.messages[-1].linear.x == 0.0
    assert client.command_pub.messages[-1].angular.z == 0.0
    assert client.emergency_pub.messages[-1].data is True
    assert client.stop_task_pub.messages[-1].data is True
    assert events[-1]["state"] == "E_STOP"
    assert events[-1]["fault_code"] == "TURN_DIRECTION_MISMATCH"
    assert states[-1] == ("E_STOP", "opposite yaw")


def test_hybrid_tracker_switches_to_pure_pursuit_with_zero_command():
    client = InternNavDirectControlClient.__new__(InternNavDirectControlClient)
    client.args = SimpleNamespace(trajectory_tracker="hybrid")
    client._active_trajectory_tracker = "mpc"
    client._mpc_path_progress = 0.24
    client._mpc_last_command = (0.1, 0.2)
    client._mpc_last_raw_command = (0.1, 0.2)
    client._mpc_request_id = 7
    client.command_pub = RecordingPublisher()
    events = []
    client._append_event = events.append
    states = []
    client._set_state = lambda state, reason: states.append((state, reason))

    client._switch_to_pure_pursuit("mpc stalled", 12.0)

    assert client._active_trajectory_tracker == "pure_pursuit"
    assert client._trajectory_fallback_reason == "mpc stalled"
    assert client._mpc_progress_anchor == 0.24
    assert client._mpc_progress_anchor_time == 12.0
    assert client.command_pub.messages[-1].linear.x == 0.0
    assert client.command_pub.messages[-1].angular.z == 0.0
    assert events[-1]["state"] == "MPC_FALLBACK_TO_PURE_PURSUIT"
    assert states[-1] == ("PURE_PURSUIT_ACTIVE", "mpc stalled")
