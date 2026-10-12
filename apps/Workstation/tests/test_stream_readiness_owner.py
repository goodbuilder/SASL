"""Startup readiness is an authoritative fact owner, independent of views."""

from __future__ import annotations

import subprocess
import sys
import time
from pathlib import Path
from types import SimpleNamespace

from Workstation.camera_runtime.readiness import READINESS_FIELDS, StreamReadinessTracker
from Workstation.tests.test_panel_composition import panels
from Workstation.camera_stream_controller import CameraStreamLifecycleController


def test_disabled_video_cancels_readiness_without_reading_unused_path_settings():
    panel = SimpleNamespace(
        video_recording_enabled=False, video_recording_paths=object(), _video_recording_allocation_sizes=object()
    )
    CameraStreamLifecycleController(panel)._begin_video_start_readiness(("left",))
    assert panel._video_start_readiness is None


def test_camera_and_presentation_share_actual_readiness_state(panels):
    panel = panels()
    owner = panel.application.stream_readiness
    camera = panel.application.features["camera"].state
    presentation = panel.application.features["presentation"].state
    assert camera.stream_readiness is owner
    assert presentation.stream_readiness is owner
    for name in READINESS_FIELDS:
        assert name not in vars(camera)
        assert name not in vars(presentation)
        assert getattr(panel, name) is getattr(owner, name)
    ready, _ = owner.begin_camera(
        ("left",),
        expected_stream_generations={"left": 7},
        enable_detection=False,
        switch_to_depth=False,
        timeout_s=35.0,
    )
    assert panel._camera_start_readiness is ready
    panel._cancel_camera_start_readiness()
    assert owner._camera_start_readiness is None


def test_first_frame_generation_partial_and_timeout_contracts_are_preserved():
    owner = StreamReadinessTracker()
    expected = {"left": 7, "right": 9}
    readiness, unavailable = owner.begin_camera(
        ("left", "right"),
        expected_stream_generations=expected,
        enable_detection=True,
        switch_to_depth=False,
        timeout_s=35.0,
    )
    expected["left"] = 99
    assert unavailable == ()
    assert owner.admit_frame("left", {}, 6) is False
    assert owner.admit_frame("left", {"media_integrity_unverified": True}, 7) is True
    assert owner.evaluate_camera(readiness)[:2] == (("left",), ("right",))
    assert readiness["unverified_frame_sides"] == {"left"}
    readiness["deadline_monotonic_s"] = time.monotonic() - 1.0
    expired = owner.expire_camera(readiness["generation"], timeout_s=35.0)
    assert expired == (("right", "first decoded frame timeout after 35.0s"),)
    assert owner.evaluate_camera(readiness)[:2] == (("left",), ())
    assert owner.admit_frame("right", {}, 9) is False
    newer, _ = owner.begin_camera(
        ("right",),
        expected_stream_generations={"right": 10},
        enable_detection=True,
        switch_to_depth=False,
        timeout_s=35.0,
    )
    assert owner.evaluate_camera(readiness) is None
    assert owner.expire_camera(readiness["generation"], timeout_s=35.0) is None
    assert owner.admit_frame("right", {}, 9) is False
    assert owner.admit_frame("right", {}, 10) is True
    assert owner.evaluate_camera(newer)[0] == ("right",)


def test_video_requires_positive_growth_and_stale_finish_cannot_clear_replacement(tmp_path):
    owner = StreamReadinessTracker()
    path = tmp_path / "reserved.mp4"
    path.write_bytes(b"old")
    readiness = owner.begin_video(("left",), enabled=True, paths={"left": path}, baselines={"left": 3}, timeout_s=35.0)
    update = owner.poll_video(readiness["generation"], timeout_s=35.0)
    assert update[3] is True
    assert update[1]["left"]["state"] == "STARTING"
    path.write_bytes(b"new mp4 bytes")
    update = owner.poll_video(readiness["generation"], timeout_s=35.0)
    assert update[3] is False
    assert update[1]["left"]["state"] == "RUNNING"
    assert update[1]["right"]["state"] == "FAILED"
    assert update[1]["left"]["size_bytes"] == len(b"new mp4 bytes")
    owner.fail_pending_video("startup aborted")
    assert readiness["rows"]["left"]["state"] == "RUNNING"
    newer = owner.begin_video(
        ("left",), enabled=True, paths={"left": path}, baselines={"left": len(b"new mp4 bytes")}, timeout_s=35.0
    )
    owner.finish_video(readiness)
    assert owner._video_start_readiness is newer
    owner.cancel_camera()
    assert owner._video_start_readiness is newer
    owner.cancel_video()
    assert owner.poll_video(newer["generation"], timeout_s=35.0) is None


def test_readiness_import_does_not_load_qt_or_controller():
    apps = Path(__file__).resolve().parents[2]
    script = """
import importlib.abc, sys
sys.path.insert(0, sys.argv[1])
class Guard(importlib.abc.MetaPathFinder):
    def find_spec(self, fullname, path=None, target=None):
        if fullname.startswith(('PyQt5', 'Workstation.panel_', 'Workstation.camera_stream_controller')):
            raise AssertionError(fullname)
        return None
sys.meta_path.insert(0, Guard())
from Workstation.camera_runtime.readiness import StreamReadinessTracker
owner = StreamReadinessTracker()
assert owner._camera_start_readiness is None
"""
    subprocess.run([sys.executable, "-I", "-B", "-c", script, str(apps)], check=True, timeout=20)
