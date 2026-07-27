import importlib.util
import io
import json
import sys
import threading
import time
import types
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path

import numpy as np
from PIL import Image


SERVER_PATH = Path(__file__).resolve().parents[2] / "scripts" / "realworld" / "http_internvla_server.py"


class FakeOutput:
    output_action = [1]
    output_trajectory = None
    output_pixel = None


class FakeAgent:
    def __init__(self):
        self.active = 0
        self.max_active = 0
        self.guard = threading.Lock()

    def reset(self):
        pass

    def step(self, *_args, **_kwargs):
        with self.guard:
            self.active += 1
            self.max_active = max(self.max_active, self.active)
        time.sleep(0.05)
        with self.guard:
            self.active -= 1
        return FakeOutput()


def load_server(monkeypatch):
    fake_agent_package = types.ModuleType("internnav.agent")
    fake_agent_package.__path__ = []
    fake_agent_module = types.ModuleType("internnav.agent.internvla_n1_agent_realworld")
    fake_agent_module.InternVLAN1AsyncAgent = object
    monkeypatch.setitem(sys.modules, "internnav.agent", fake_agent_package)
    monkeypatch.setitem(
        sys.modules, "internnav.agent.internvla_n1_agent_realworld", fake_agent_module
    )

    spec = importlib.util.spec_from_file_location("http_server_lock_test", SERVER_PATH)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    module.args = types.SimpleNamespace(
        instruction="walk forward", camera_intrinsic=np.eye(4), model_path="fake", device="cpu"
    )
    module.agent = FakeAgent()
    return module


def request_payload():
    rgb = io.BytesIO()
    Image.fromarray(np.zeros((8, 8, 3), dtype=np.uint8)).save(rgb, format="JPEG")
    depth = io.BytesIO()
    Image.fromarray(np.zeros((8, 8), dtype=np.uint16)).save(depth, format="PNG")
    return {
        "image": (io.BytesIO(rgb.getvalue()), "rgb.jpg"),
        "depth": (io.BytesIO(depth.getvalue()), "depth.png"),
        "json": json.dumps({"reset": False, "instruction": "walk forward"}),
    }


def test_health_and_inference_requests_are_serialized(monkeypatch):
    server = load_server(monkeypatch)
    with server.app.test_client() as client:
        health = client.get("/health")
        assert health.status_code == 200
        assert health.get_json()["status"] == "ok"

    def invoke():
        with server.app.test_client() as client:
            response = client.post("/eval_dual", data=request_payload())
            return response.status_code, response.get_json()

    with ThreadPoolExecutor(max_workers=4) as pool:
        results = list(pool.map(lambda _index: invoke(), range(4)))

    assert results == [(200, {"discrete_action": [1]})] * 4
    assert server.agent.max_active == 1

