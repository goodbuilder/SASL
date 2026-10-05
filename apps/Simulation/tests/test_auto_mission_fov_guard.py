from __future__ import annotations

import importlib.util
import sys
import time
import types
from pathlib import Path

import pytest


SCRIPT_PATH = Path(__file__).resolve().parents[1] / "scripts" / "px4_auto_mission.py"


def load_mission_module():
    pymavlink = types.ModuleType("pymavlink")
    pymavlink.mavutil = types.SimpleNamespace()
    previous = sys.modules.get("pymavlink")
    sys.modules["pymavlink"] = pymavlink
    try:
        spec = importlib.util.spec_from_file_location("px4_auto_mission_under_test", SCRIPT_PATH)
        assert spec is not None and spec.loader is not None
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        return module
    finally:
        if previous is None:
            sys.modules.pop("pymavlink", None)
        else:
            sys.modules["pymavlink"] = previous


def test_full_bbox_must_fit_inside_both_image_safe_regions() -> None:
    module = load_mission_module()

    assert module.bbox_inside_safe_fov((100, 100, 1180, 620), 1280, 720, 0.05)
    assert not module.bbox_inside_safe_fov((40, 100, 500, 620), 1280, 720, 0.05)
    assert not module.bbox_inside_safe_fov((100, 100, 1180, 700), 1280, 720, 0.05)


def test_interrupt_flag_stops_stream_until_interrupted_without_relying_on_os_delivery() -> None:
    """SIGINT delivery is unreliable in this multi-threaded process (observed

    via /proc/<pid>/status SigQ staying nonzero for minutes); the mission
    must be stoppable purely by setting the module-level flag, independent
    of whether the OS ever actually schedules the signal handler.
    """
    module = load_mission_module()
    module._interrupted.clear()
    calls = {"count": 0}

    def callback():
        calls["count"] += 1
        if calls["count"] == 3:
            module._interrupted.set()

    try:
        with pytest.raises(KeyboardInterrupt):
            module.stream_until_interrupted(1000.0, callback)
        assert calls["count"] == 3
    finally:
        module._interrupted.clear()


def test_request_interrupt_handler_sets_flag_directly() -> None:
    module = load_mission_module()
    module._interrupted.clear()
    try:
        module._request_interrupt(module.signal.SIGINT, None)
        assert module._interrupted.is_set()
        with pytest.raises(KeyboardInterrupt):
            module.raise_if_interrupted()
    finally:
        module._interrupted.clear()


def test_fov_monitor_does_not_gate_movement_without_a_fresh_pair() -> None:
    module = load_mission_module()
    monitor = module.BboxFovMonitor(
        image_width_px=1280,
        image_height_px=720,
        margin_ratio=0.05,
        timeout_s=0.5,
    )

    allowed, reason = monitor.snapshot()
    assert allowed
    assert "not gated" in reason

    monitor._latest = ((100.0, 100.0, 500.0, 500.0), None)
    monitor._latest_time = time.monotonic()
    allowed, reason = monitor.snapshot()
    assert allowed
    assert "not gated" in reason

    monitor._latest = (
        (100.0, 100.0, 500.0, 500.0),
        (100.0, 100.0, 500.0, 500.0),
    )
    monitor._latest_time = time.monotonic() - 1.0
    allowed, reason = monitor.snapshot()
    assert allowed
    assert "not gated" in reason


def test_fov_monitor_stops_only_for_known_bbox_outside_safe_region() -> None:
    module = load_mission_module()
    monitor = module.BboxFovMonitor(
        image_width_px=1280,
        image_height_px=720,
        margin_ratio=0.05,
        timeout_s=0.5,
    )
    monitor._latest = (
        (10.0, 100.0, 500.0, 500.0),
        (100.0, 100.0, 500.0, 500.0),
    )
    monitor._latest_time = time.monotonic()
    allowed, reason = monitor.snapshot()
    assert not allowed
    assert "outside" in reason


def test_actual_distance_monitor_reuses_existing_rclpy_context() -> None:
    module = load_mission_module()
    calls = {"init": 0, "shutdown": 0}

    class FakeNode:
        def create_subscription(self, *_args):
            return object()

        def create_publisher(self, *_args):
            return object()

        def destroy_node(self):
            return None

    fake_rclpy = types.ModuleType("rclpy")
    fake_rclpy.ok = lambda: True
    fake_rclpy.init = lambda args=None: calls.__setitem__("init", calls["init"] + 1)
    fake_rclpy.shutdown = lambda: calls.__setitem__("shutdown", calls["shutdown"] + 1)
    fake_rclpy.create_node = lambda _name: FakeNode()
    fake_rclpy.spin_once = lambda *_args, **_kwargs: None

    std_msgs = types.ModuleType("std_msgs")
    std_msgs_msg = types.ModuleType("std_msgs.msg")
    std_msgs_msg.Float32 = object
    std_msgs_msg.String = type("String", (), {"__init__": lambda self, data="": setattr(self, "data", data)})
    std_msgs.msg = std_msgs_msg
    previous_rclpy = sys.modules.get("rclpy")
    previous_std_msgs = sys.modules.get("std_msgs")
    previous_std_msgs_msg = sys.modules.get("std_msgs.msg")
    sys.modules["rclpy"] = fake_rclpy
    sys.modules["std_msgs"] = std_msgs
    sys.modules["std_msgs.msg"] = std_msgs_msg
    try:
        monitor = module.ActualDistanceMonitor()
        monitor.start()
        assert monitor.ready
        assert calls["init"] == 0
        monitor.stop()
        assert calls["shutdown"] == 0
    finally:
        if previous_rclpy is None:
            sys.modules.pop("rclpy", None)
        else:
            sys.modules["rclpy"] = previous_rclpy
        if previous_std_msgs is None:
            sys.modules.pop("std_msgs", None)
        else:
            sys.modules["std_msgs"] = previous_std_msgs
        if previous_std_msgs_msg is None:
            sys.modules.pop("std_msgs.msg", None)
        else:
            sys.modules["std_msgs.msg"] = previous_std_msgs_msg


def test_interrupt_land_request_uses_px4_land_mode() -> None:
    module = load_mission_module()
    calls = []

    class FakeMaster:
        def mode_mapping(self):
            return {"LAND": 9}

    previous_set_mode = module.set_mode
    module.set_mode = lambda master, mode: calls.append((master, mode))
    try:
        master = FakeMaster()
        module.request_land(master)
        assert calls == [(master, "LAND")]
    finally:
        module.set_mode = previous_set_mode
