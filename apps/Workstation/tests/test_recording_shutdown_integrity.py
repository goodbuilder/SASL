"""Focused regressions for recording transactions and QObject shutdown ownership."""

from __future__ import annotations

import os
from pathlib import Path
import subprocess
import sys

import pytest
from PyQt5 import QtCore, QtWidgets


class _Label:
    def __init__(self, text: str = "") -> None:
        self.value = str(text)

    def setText(self, value: str) -> None:  # noqa: N802 - Qt test double.
        self.value = str(value)

    def text(self) -> str:
        return self.value


class _Rows:
    def __init__(self, name: str) -> None:
        self.name = str(name)
        self.discarded = False

    def discard_spool(self) -> bool:
        self.discarded = True
        return True

    def __len__(self) -> int:
        return 0


def _panel(monkeypatch: pytest.MonkeyPatch, tmp_path: Path):
    from Workstation import standalone_gui

    monkeypatch.setattr(
        standalone_gui.WirisStereoDistancePanel,
        "_initial_status",
        lambda self: None,
    )
    monkeypatch.setattr(
        standalone_gui.WirisStereoDistancePanel,
        "_auto_load_intrinsics",
        lambda self: None,
    )
    QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    return standalone_gui.WirisStereoDistancePanel(app_root=tmp_path)


def _dispose_panel(panel) -> None:
    from sasl_common.lifecycle import LifecycleCoordinator

    panel._shutdown_in_progress = False
    panel._shutdown_coordinator = LifecycleCoordinator([])
    panel.close()
    panel.deleteLater()
    application = QtWidgets.QApplication.instance()
    if application is not None:
        application.sendPostedEvents(None, QtCore.QEvent.DeferredDelete)
        application.processEvents()


def test_recording_start_exception_rolls_back_every_activated_state_and_new_rows(
    monkeypatch: pytest.MonkeyPatch,
    tmp_path: Path,
) -> None:
    panel = _panel(monkeypatch, tmp_path)
    controller = panel._recording_controller()
    old_rows = {
        "distance": panel.distance_recording_rows,
        "camera": panel.camera_recording_rows,
        "px4": panel.px4_recording_rows,
        "gnss": panel.gnss_base_recording_rows,
        "extrinsics": panel.extrinsics_comparison_recording_rows,
        "methods": panel.method_evaluation_bundle_rows,
    }
    panel.distance_recording_keys.append(("preserved",))
    panel.external_parameter_recording_rows.append({"preserved": True})
    panel._recording_pair_row_index[("preserved",)] = 7
    panel._recording_pending_native_request_rows["old-request"] = ("left", 1)
    controller._deferred_native_frame_results["old-request"] = {"preserved": True}
    panel.video_recording_paths = {"left": tmp_path / "preserved-left.mp4"}
    old_video_paths = panel.video_recording_paths
    old_snapshot = panel.video_recording_start_snapshot()

    allocated: list[_Rows] = []

    def new_rows(stream: str, **_kwargs: object) -> _Rows:
        row = _Rows(stream)
        allocated.append(row)
        return row

    events: list[dict[str, object]] = []
    monkeypatch.setattr(controller, "_new_live_rows", new_rows)
    monkeypatch.setattr(controller, "_snapshot_current_intrinsics", lambda: {"ok": True})
    monkeypatch.setattr(
        panel,
        "append_workstation_data",
        lambda stream, value, **_kwargs: events.append(
            {"stream": stream, **dict(value)}
        )
        or True,
    )
    monkeypatch.setattr(
        panel,
        "_record_px4_mag_status_snapshot",
        lambda: None,
    )
    monkeypatch.setattr(
        panel,
        "_prepare_video_recording_paths",
        lambda: setattr(
            panel,
            "video_recording_paths",
            {
                "left": tmp_path / "new-left.mp4",
                "right": tmp_path / "new-right.mp4",
            },
        ),
    )
    monkeypatch.setattr(
        panel,
        "_refresh_recording_table",
        lambda: (_ for _ in ()).throw(RuntimeError("injected start failure")),
    )

    try:
        with pytest.raises(RuntimeError, match="injected start failure"):
            controller.start_automatic_session_recording()

        assert panel._automatic_session_recording is False
        assert panel._manual_recording_active is False
        assert panel.distance_recording_enabled is False
        assert panel.video_recording_enabled is False
        assert panel.recording_session_id == ""
        assert panel.recording_session_started_unix_ns is None
        assert panel.recording_session_started_monotonic_ns is None
        assert panel.distance_recording_rows is old_rows["distance"]
        assert panel.camera_recording_rows is old_rows["camera"]
        assert panel.px4_recording_rows is old_rows["px4"]
        assert panel.gnss_base_recording_rows is old_rows["gnss"]
        assert panel.extrinsics_comparison_recording_rows is old_rows["extrinsics"]
        assert panel.method_evaluation_bundle_rows is old_rows["methods"]
        assert panel.distance_recording_keys == [("preserved",)]
        assert list(panel.external_parameter_recording_rows) == [{"preserved": True}]
        assert panel._recording_pair_row_index == {("preserved",): 7}
        assert panel._recording_pending_native_request_rows == {
            "old-request": ("left", 1)
        }
        assert controller._deferred_native_frame_results == {
            "old-request": {"preserved": True}
        }
        assert panel.video_recording_paths is old_video_paths
        assert panel.video_recording_start_snapshot() == old_snapshot
        assert allocated and all(row.discarded for row in allocated)
        assert not any(row.get("event_type") == "recording_started" for row in events)
    finally:
        _dispose_panel(panel)


def test_manual_recording_start_is_rejected_during_shutdown() -> None:
    from types import SimpleNamespace

    from Workstation.recording_controller import RecordingController

    logs: list[str] = []
    panel = SimpleNamespace(
        _automatic_session_recording=False,
        _manual_recording_active=False,
        _recording_mode="idle",
        _shutdown_in_progress=True,
        recording_session_id="",
        recording_status_label=_Label(),
        _log=logs.append,
    )

    assert RecordingController(panel)._start_session_recording() is False
    assert panel._recording_mode == "idle"
    assert panel._manual_recording_active is False
    assert panel.recording_session_id == ""
    assert "shutdown" in panel.recording_status_label.text().lower()


def test_interface_status_process_has_a_finite_runtime_boundary(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    from types import SimpleNamespace

    from Workstation.workstation_core import network

    observed: list[float | None] = []

    def run(_command: list[str], **kwargs: object) -> object:
        observed.append(kwargs.get("timeout"))
        return SimpleNamespace(returncode=0, stdout="enx-camera UP", stderr="")

    monkeypatch.setattr(network.subprocess, "run", run)

    assert network.interface_addresses("enx-camera") == "enx-camera UP"
    assert observed == [network.INTERFACE_STATUS_TIMEOUT_SECONDS]


def test_named_late_callback_timers_are_panel_owned_and_generation_guarded(
    monkeypatch: pytest.MonkeyPatch,
    tmp_path: Path,
) -> None:
    panel = _panel(monkeypatch, tmp_path)
    try:
        assert panel._rtsp_reactivation_stop_timer.parent() is panel
        assert panel._rtsp_reactivation_stop_timer.isSingleShot()
        assert panel._recording_stop_retry_timer.parent() is panel
        assert panel._recording_stop_retry_timer.isSingleShot()
        assert set(panel._eth0_probe_timers) == {"left", "right"}
        assert all(timer.parent() is panel for timer in panel._eth0_probe_timers.values())

        starts: list[tuple[str, ...]] = []
        monkeypatch.setattr(
            panel,
            "_start_rtsp_enable_worker",
            lambda *, target_labels, reactivate: starts.append(tuple(target_labels)),
        )
        panel._rtsp_reactivation_side = "left"
        panel._rtsp_reactivation_generation = 5
        panel._rtsp_reactivation_stop_generation = 4
        panel._continue_rtsp_reactivation_after_stop()
        assert starts == []

        panel._eth0_probe_tokens["left"] = 2
        panel._eth0_probe_timeout_tokens["left"] = 1
        panel._eth0_probe_timeout_fired("left")
        assert panel._eth0_probe_tokens["left"] == 2

        stop_calls: list[dict[str, object]] = []
        controller = panel._recording_controller()
        monkeypatch.setattr(
            controller,
            "_stop_session_recording",
            lambda **kwargs: stop_calls.append(dict(kwargs)),
        )
        panel.recording_session_id = "new-session"
        panel._recording_stop_retry_generation = 9
        panel._recording_stop_retry_request = {
            "generation": 9,
            "recording_session_id": "stale-session",
            "restart_streams": False,
            "final_clock_sync_snapshot": None,
        }
        controller._retry_legacy_recording_stop()
        assert stop_calls == []
        assert panel._recording_stop_retry_request is None
    finally:
        _dispose_panel(panel)


def test_direct_delete_later_drains_running_interface_qthread_without_abort(
    tmp_path: Path,
) -> None:
    repository = Path(__file__).resolve().parents[3]
    environment = os.environ.copy()
    environment["QT_QPA_PLATFORM"] = "offscreen"
    environment["PYTHONPATH"] = os.pathsep.join(
        (
            str(repository / "apps"),
            str(repository),
            str(repository / "common" / "python"),
        )
    )
    script = r'''
import tempfile
import time
from pathlib import Path

from PyQt5 import QtCore, QtWidgets
from Workstation import device_runtime_workers, standalone_gui

standalone_gui.WirisStereoDistancePanel._initial_status = lambda self: None
standalone_gui.WirisStereoDistancePanel._auto_load_intrinsics = lambda self: None
device_runtime_workers.interface_addresses = lambda _name: (time.sleep(0.20) or "UP")

app = QtWidgets.QApplication([])
with tempfile.TemporaryDirectory() as directory:
    panel = standalone_gui.WirisStereoDistancePanel(app_root=Path(directory))
    panel.destroyed.connect(lambda: print("PANEL_DESTROYED", flush=True))
    panel._start_interface_status_worker()
    deadline = time.monotonic() + 1.0
    while not panel.interface_status_thread.isRunning() and time.monotonic() < deadline:
        app.processEvents()
    panel.deleteLater()
    app.sendPostedEvents(None, QtCore.QEvent.DeferredDelete)
    app.processEvents()
'''
    completed = subprocess.run(
        [sys.executable, "-c", script],
        cwd=repository / "apps" / "Workstation",
        env=environment,
        check=False,
        capture_output=True,
        text=True,
        timeout=8,
    )

    assert completed.returncode == 0, completed.stderr
    assert "PANEL_DESTROYED" in completed.stdout
    assert "QThread: Destroyed while thread is still running" not in completed.stderr
