"""Workstation lifecycle regression tests."""

from __future__ import annotations

from PyQt5 import QtCore, QtTest

from Workstation.tests.workstation_test_support import (
    _FakeBackend,
    LifecycleCoordinator,
    os,
    Path,
    pytest,
    QtWidgets,
    SimpleNamespace,
    subprocess,
    sys,
    threading,
    time,
    WorkstationMainWindow,
)


def test_standalone_entrypoint_does_not_import_legacy_gui() -> None:
    repository = Path(__file__).resolve().parents[3]
    app_root = repository / "apps" / "StereoWIRIS"
    environment = os.environ.copy()
    environment["PYTHONPATH"] = os.pathsep.join(
        (str(app_root), str(repository / "apps"), str(repository / "common" / "python"))
    )
    environment["SASL_ROS_LIBRARY_PATH_READY"] = "1"
    script = """
import sys

import main

assert main._workstation_requested(["main.py", "--ui", "workstation"])
assert "wiris_stereo_distance.gui" not in sys.modules
assert "wiris_stereo_distance.legacy_gui" not in sys.modules
from Workstation import standalone_gui
assert standalone_gui.__file__.endswith("/standalone_gui.py")
assert "wiris_stereo_distance.gui" not in sys.modules
assert "wiris_stereo_distance.legacy_gui" not in sys.modules
assert not any(
    name == "wiris_stereo_distance" or name.startswith("wiris_stereo_distance.")
    for name in sys.modules
)
assert "rclpy" not in sys.modules
"""
    subprocess.run(
        [sys.executable, "-c", script],
        cwd=app_root,
        env=environment,
        check=True,
        capture_output=True,
        text=True,
    )

def test_class_gate_change_is_rejected_while_detection_worker_is_running(
    monkeypatch: pytest.MonkeyPatch,
    tmp_path: Path,
) -> None:
    from Workstation import standalone_gui

    monkeypatch.setattr(standalone_gui.WirisStereoDistancePanel, "_initial_status", lambda self: None)
    monkeypatch.setattr(standalone_gui.WirisStereoDistancePanel, "_auto_load_intrinsics", lambda self: None)
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    panel = standalone_gui.WirisStereoDistancePanel(app_root=tmp_path)
    panel.yolo_gate_class_checkbox.setChecked(True)
    panel.yolo_gate_class_edit.setText("person")
    panel.yolo_enabled = True

    with pytest.raises(RuntimeError, match="stop Detection"):
        panel.apply_workstation_measurement_settings(
            {"class_gate_enabled": False, "class_gate": ""}
        )

    assert panel.yolo_gate_class_checkbox.isChecked() is True
    assert panel.yolo_gate_class_edit.text() == "person"
    panel._shutdown_coordinator = LifecycleCoordinator([])
    panel.close()
    panel.deleteLater()
    app.processEvents()

def test_workstation_entrypoint_does_not_add_ros_runtime_paths() -> None:
    app_root = Path(__file__).resolve().parents[2] / "StereoWIRIS"
    environment = os.environ.copy()
    environment["PYTHONPATH"] = str(app_root)
    environment["SASL_ROS_LIBRARY_PATH_READY"] = "1"
    script = """
import sys

sys.argv = ["main.py", "--ui", "workstation"]
import main

assert main.WORKSTATION_REQUESTED is True
assert not any("/opt/ros/" in path for path in sys.path)
assert not any("workspace/main/install" in path for path in sys.path)
"""
    subprocess.run(
        [sys.executable, "-c", script],
        cwd=app_root,
        env=environment,
        check=True,
        capture_output=True,
        text=True,
    )


def test_qt_callback_exception_hook_logs_and_keeps_event_loop_alive() -> None:
    repository = Path(__file__).resolve().parents[3]
    environment = os.environ.copy()
    environment["QT_QPA_PLATFORM"] = "offscreen"
    environment["PYTHONPATH"] = os.pathsep.join(
        (str(repository / "apps"), str(repository), str(repository / "common" / "python"))
    )
    script = r'''
from PyQt5 import QtCore, QtWidgets
from Workstation.__main__ import _install_qt_exception_hook

app = QtWidgets.QApplication([])
flushes = []
ticks = []
_install_qt_exception_hook(lambda: flushes.append("flushed"))

def explode():
    raise RuntimeError("timer-sentinel")

QtCore.QTimer.singleShot(0, explode)
QtCore.QTimer.singleShot(20, lambda: ticks.append("continued"))
QtCore.QTimer.singleShot(50, app.quit)
assert app.exec_() == 0
assert ticks == ["continued"]
assert flushes == ["flushed"]
'''
    completed = subprocess.run(
        [sys.executable, "-c", script],
        cwd=repository / "apps" / "Workstation",
        env=environment,
        check=False,
        capture_output=True,
        text=True,
        timeout=5,
    )

    assert completed.returncode == 0, completed.stderr
    assert "timer-sentinel" in completed.stderr
    assert "Unhandled Workstation Qt callback exception" in completed.stderr

def test_legacy_entrypoint_does_not_import_workstation_modules() -> None:
    repository = Path(__file__).resolve().parents[3]
    app_root = repository / "apps" / "StereoWIRIS"
    environment = os.environ.copy()
    environment["PYTHONPATH"] = os.pathsep.join(
        (str(app_root), str(repository / "apps"), str(repository / "common" / "python"))
    )
    environment["SASL_ROS_LIBRARY_PATH_READY"] = "1"
    script = """
import sys

import main

assert not main._workstation_requested(["main.py"])
from wiris_stereo_distance import legacy_gui
assert legacy_gui.__file__.endswith("/legacy_gui.py")
assert "Workstation" not in sys.modules
assert not any(name.startswith("Workstation.") for name in sys.modules)
"""
    subprocess.run(
        [sys.executable, "-c", script],
        cwd=app_root,
        env=environment,
        check=True,
        capture_output=True,
        text=True,
    )

def test_workstation_runtime_paths_are_separate_from_legacy_storage() -> None:
    from wiris_stereo_distance import settings as legacy_settings
    from Workstation.runtime_paths import (
        WORKSTATION_DATA_ROOT,
        WORKSTATION_RECORDINGS_ROOT,
        WORKSTATION_SETTINGS_PATH,
    )

    assert WORKSTATION_DATA_ROOT.name == "workstation_data"
    assert WORKSTATION_DATA_ROOT.parent.name == ".local"
    assert WORKSTATION_SETTINGS_PATH == WORKSTATION_DATA_ROOT / "stereo_distance_settings.json"
    assert WORKSTATION_RECORDINGS_ROOT == WORKSTATION_DATA_ROOT / "recordings"
    assert WORKSTATION_SETTINGS_PATH != legacy_settings.DEFAULT_SETTINGS_PATH
    assert WORKSTATION_RECORDINGS_ROOT != legacy_settings.APP_ROOT / "recordings"

def test_workstation_shell_navigation_and_close() -> None:
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    backend = _FakeBackend()
    session = {
        "state": "IDLE",
        "session_id": "",
        "session_dir": "",
        "measurements": [],
    }
    backend.measurement_session_snapshot = lambda: dict(session)

    def create_session_folder() -> bool:
        backend.calls.append("session.begin")
        session.update(
            state="OPEN",
            session_id="20260822_143000_000000001",
            session_dir="/tmp/recordings/20260822_143000_000000001",
            metadata={
                "opened_local_date": "2026-08-22",
                "opened_local_time": "14:30:00",
            },
        )
        return True

    backend.begin_measurement_session = create_session_folder
    backend.close_measurement_session = lambda: True
    window = WorkstationMainWindow(backend=backend)
    window.show()
    app.processEvents()

    window.resize(1366, 768)
    app.processEvents()
    assert window.workspace_host.width() > 500
    root = window.navigation.model.item(0, 0)
    assert [root.child(row).text() for row in range(root.rowCount())] == [
        "Overview",
        "Protocol",
        "Devices",
        "Data",
        "Calibration",
        "Alignment",
        "Measurement",
        "Advanced",
        "Settings",
        "Diagnostics",
    ]

    window.navigation.select_context("devices.cameras")
    app.processEvents()
    assert window.workspace_host.currentWidget().widget() is window.camera
    assert [
        window.camera.operation_buttons[key].text()
        for key in (
            "camera.discover_host_ip",
            "camera.check_control",
            "camera.enable_rtsp",
            "camera.check_rtsp",
        )
    ] == [
        "Find network",
        "Check control",
        "Enable stream",
        "Check video",
    ]

    window.navigation.select_context("devices.time")
    app.processEvents()
    assert window.workspace_host.currentWidget().widget() is window.time_workspace
    assert window.time_workspace.sync_summary.height() == 200
    assert window.commands.invoke("time.check_sync") is True
    assert backend.calls[-1] == "time_check_sync"

    window.navigation.select_context("tasks.data")
    app.processEvents()
    assert window.workspace_host.currentWidget().widget() is window.sessions
    assert window.sessions.current_tab_id() == "current"
    assert [
        window.sessions.tabs.tabText(index)
        for index in range(window.sessions.tabs.count())
    ] == ["Status", "Capture"]
    assert window.inspector.context_label.text() == "Data"
    assert not window.session.isVisible()
    assert window.data.open_data_folder_button.objectName() == "dataWorkspaceOpenRoot"
    assert window.data.retry_storage_button.objectName() == "dataWorkspaceRetryStorage"

    window.navigation.select_context("processing.experiment")
    app.processEvents()
    assert not hasattr(window.recording, "run_setup_group")
    assert window.recording.preflight_group.isVisible()
    assert "recording.toggle" not in window.recording.operation_buttons
    assert "recording.toggle" in window.advanced.operation_buttons
    assert "session.save" not in window.advanced.operation_buttons
    assert "advanced.media.open_video" in window.advanced.operation_buttons
    assert "advanced.media.refresh" not in window.advanced.operation_buttons
    assert "advanced.media.preview" not in window.advanced.operation_buttons

    window.navigation.select_context("processing.measurement")
    app.processEvents()
    assert window.workspace_host.currentWidget().widget() is window.measurement
    assert window.measurement.findChild(QtWidgets.QWidget, "measurementPackageStart") is None
    assert window.measurement.findChild(QtWidgets.QWidget, "measurementSessionClose") is None
    assert window.measurement.findChild(
        QtWidgets.QWidget, "measurementPackageNewMeasurement"
    ) is None
    assert window.measurement.control_buttons["detection.toggle"].text() == "측정 시작"
    assert window.measurement.finalize_session_button.isHidden()
    assert not window.measurement.findChildren(QtWidgets.QLabel, "measurementResultStatus")
    assert any(
        "baseline_center_frd" in label.text()
        for label in window.measurement.findChildren(QtWidgets.QLabel)
    )
    assert [window.measurement.values[key].label.text() for key in ("range", "forward", "right", "down")] == [
        "Range",
        "Forward",
        "Right",
        "Down",
    ]
    assert window.measurement.history.model().headerData(0, 1) == "timestamp_s"
    assert set(window.measurement.previews) == {"left", "right"}
    assert not any(
        group.title() == "Frames"
        for group in window.measurement.findChildren(QtWidgets.QGroupBox)
    )
    assert window.measurement.preview_row.minimumHeight() >= 280
    assert window.measurement.preview_row.maximumHeight() == window.measurement.preview_row.minimumHeight()
    assert window.measurement.previews["left"].crop_to_fill is False
    window.resize(1600, 980)
    app.processEvents()
    app.processEvents()
    assert window.measurement.preview_row.sizePolicy().verticalPolicy() == QtWidgets.QSizePolicy.Fixed
    assert window.measurement.preview_row.height() <= 360
    assert window.measurement.previews["left"].heightForWidth(640) > 300
    assert set(window.measurement.upstream_values) >= {
        "camera.left",
        "camera.right",
        "detection",
        "synchronization",
        "gnss",
        "recording",
    }
    assert set(window.measurement.control_buttons) == {
        "camera.toggle",
        "detection.toggle",
    }
    assert window.measurement.session_status_label.text().startswith("Measurement:")
    assert window.measurement.session_details_button.isEnabled()
    assert (
        window.measurement.session_details_button.parentWidget()
        is window.measurement.session_details_group
    )
    measurement_details_layout = window.measurement.details.layout()
    assert measurement_details_layout.indexOf(window.measurement.history) < (
        measurement_details_layout.indexOf(window.measurement.session_details_group)
    )
    assert measurement_details_layout.indexOf(window.measurement.session_details_group) == (
        measurement_details_layout.count() - 1
    )
    assert set(window.measurement.quality_labels) == {
        "status",
        "gap",
        "distance",
        "mode",
        "attitude",
        "raw_point",
    }
    backend.panel.triangulation_quality_status_label = QtWidgets.QLabel("GOOD | ray pair accepted")
    backend.panel.triangulation_quality_gap_label = QtWidgets.QLabel("gap: 0.012 m")
    backend.panel.triangulation_quality_distance_label = QtWidgets.QLabel("distance: 12.300 m")
    backend.panel.triangulation_quality_mode_label = QtWidgets.QLabel("mode: two_ray_closest_midpoint")
    backend.panel.triangulation_quality_attitude_label = QtWidgets.QLabel("Attitude timing: FRESH")
    backend.panel.triangulation_quality_raw_point_label = QtWidgets.QLabel(
        "Raw baseline_center_frd point: [12.0, 0.3, -0.1, 12.0]"
    )
    window.measurement.refresh(window.status_adapter.model.snapshot())
    assert window.measurement.quality_labels["mode"].text() == "mode: two_ray_closest_midpoint"
    assert "baseline_center_frd" in window.measurement.quality_labels["raw_point"].text()
    assert "INVALID" not in window.inspector.status_value.meta.text()
    assert "source=" not in window.inspector.status_value.meta.text()
    assert all(
        "source=" not in value.meta.text()
        for value in window.inspector.context_values.values()
    )

    window.navigation.select_context("devices.pixhawk")
    app.processEvents()
    assert window.workspace_host.currentWidget().widget() is window.pixhawk
    assert window.pixhawk.attitude_model.horizontalHeaderItem(2).text() == "Roll°"
    assert window.pixhawk.receiver_model.horizontalHeaderItem(1).text() == "Fix number"
    assert "north_m" in window.pixhawk.baseline_values
    assert "p1_body_frd_forward_m" in window.pixhawk.baseline_values
    window.navigation.select_context("devices.gnss")
    app.processEvents()
    assert window.workspace_host.currentWidget().widget() is window.base_gnss
    assert window.base_gnss.summary_values["mode"].label.text() == "Mode"
    assert "gnss.ntrip" in window.status_widgets
    assert window.base_gnss.survey_values["accuracy"].label.text() == "Mean accuracy"
    assert window.base_gnss.ntrip_values["mountpoint"].label.text() == "Mountpoint"
    assert [
        window.base_gnss.operation_buttons[key].text()
        for key in (
            "gnss.base_discover",
            "gnss.base_connect",
            "gnss.base_disconnect",
            "gnss.ntrip_start",
            "gnss.ntrip_stop",
        )
    ] == [
        "Find Base",
        "Connect",
        "Disconnect",
        "Start NTRIP",
        "Stop NTRIP",
    ]

    window.navigation.select_context("processing.alignment")
    app.processEvents()
    assert window.alignment.operation_tabs.currentIndex() == 0
    assert [
        window.alignment.operation_tabs.tabText(index)
        for index in range(window.alignment.operation_tabs.count())
    ] == [
        "Mountain + GNSS",
        "Baseline Yaw90",
        "Horizon",
        "Terrain PnP",
        "Drone Rotation",
        "Drone Multi-view",
        "Combine",
    ]
    alignment_layout = window.alignment.layout()
    assert alignment_layout.indexOf(window.alignment.optical_pose_pane) == 1
    assert window.alignment.findChild(QtWidgets.QGroupBox, "alignmentDevicePrerequisites") is None
    assert not hasattr(window.alignment, "attitude_table")

    window.navigation.select_context("diagnostics.logs")
    app.processEvents()
    assert window.diagnostics_dock.isVisible()
    assert window.diagnostics.log_view.toPlainText() == "[fake] source-backed log"
    current_workspace = window.workspace_host.currentWidget()
    window.navigation.select_context("diagnostics.camera")
    app.processEvents()
    assert window.workspace_host.currentWidget() is current_workspace
    assert window.diagnostics.tabs.currentIndex() == 1
    window.navigation.select_context("diagnostics.performance")
    app.processEvents()
    assert window.diagnostics.tabs.currentIndex() == 7
    assert "fps" in window.status_widgets

    assert window.commands.invoke("camera.toggle") is True
    assert backend.calls[-1] == "camera"
    window.close()
    app.processEvents()
    window.deleteLater()
    app.processEvents()
    assert "shutdown" in backend.calls
    window.deleteLater()
    app.processEvents()

def test_optical_pose_auto_select_runs_off_ui_thread_and_rejects_duplicate_clicks() -> None:
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    backend = _FakeBackend()
    prepare_calls: list[tuple[str, dict[str, object]]] = []
    commit_calls: list[dict[str, object]] = []
    started = threading.Event()
    release = threading.Event()

    class Job:
        def run(self) -> dict[str, object]:
            started.set()
            assert release.wait(1.0)
            return {"state": "LOW_CONFIDENCE", "mode": "mountain"}

    backend.optical_pose_prepare_auto_select = lambda mode, payload: (
        prepare_calls.append((mode, dict(payload))) or Job()
    )
    backend.optical_pose_commit_auto_select = lambda suggestion: (
        commit_calls.append(dict(suggestion))
        or {"candidates": [], "auto_suggestions": []}
    )
    window = WorkstationMainWindow(backend=backend)
    window.status_adapter.stop()
    window.show()
    app.processEvents()

    command = "alignment.pose.auto_select_mountain"
    window._handle_optical_pose_command(command, {"command_id": command})
    assert started.wait(1.0)
    assert all(not button.isEnabled() for button in window.alignment.optical_pose_pane.auto_select_buttons.values())

    window._handle_optical_pose_command(command, {"command_id": command})
    assert len(prepare_calls) == 1

    release.set()
    deadline = time.monotonic() + 2.0
    while window._optical_pose_auto_thread is not None and time.monotonic() < deadline:
        app.processEvents()
        time.sleep(0.01)
    app.processEvents()

    assert len(commit_calls) == 1
    assert commit_calls[0]["mode"] == "mountain"
    assert window._optical_pose_auto_thread is None
    assert all(button.isEnabled() for button in window.alignment.optical_pose_pane.auto_select_buttons.values())

    window.close()
    app.processEvents()
    window.deleteLater()
    app.processEvents()

def test_close_requested_during_optical_pose_analysis_retries_after_worker_finishes() -> None:
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    backend = _FakeBackend()
    started = threading.Event()
    release = threading.Event()

    class Job:
        def run(self) -> dict[str, object]:
            started.set()
            assert release.wait(1.0)
            return {"state": "LOW_CONFIDENCE", "mode": "mountain"}

    backend.optical_pose_prepare_auto_select = lambda _mode, _payload: Job()
    backend.optical_pose_commit_auto_select = lambda _suggestion: {
        "candidates": [],
        "auto_suggestions": [],
    }
    window = WorkstationMainWindow(backend=backend)
    window.status_adapter.stop()
    window.show()
    app.processEvents()
    window._handle_optical_pose_command("alignment.pose.auto_select_mountain", {})
    assert started.wait(1.0)

    window.close()
    app.processEvents()
    assert window.isVisible()
    assert window._close_after_optical_pose_auto is True

    release.set()
    deadline = time.monotonic() + 2.0
    while window.isVisible() and time.monotonic() < deadline:
        app.processEvents()
        time.sleep(0.01)
    app.processEvents()

    assert not window.isVisible()
    assert window._close_after_optical_pose_auto is False
    assert "shutdown" in backend.calls
    window.deleteLater()
    app.processEvents()


def test_accumulated_drone_multiview_command_does_not_open_json_dialog(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    backend = _FakeBackend()
    solve_payloads: list[dict[str, object]] = []
    backend.optical_pose_solve_drone_rtk_multiview = lambda payload: (
        solve_payloads.append(dict(payload))
        or {"candidates": [], "drone_rtk_multiview_samples": []}
    )

    def forbidden_dialog(*_args, **_kwargs):
        raise AssertionError("accumulated Drone RTK Multi-view must not open a JSON dialog")

    monkeypatch.setattr(QtWidgets.QFileDialog, "getOpenFileName", forbidden_dialog)
    window = WorkstationMainWindow(backend=backend)
    window.status_adapter.stop()
    window._handle_optical_pose_command(
        "alignment.pose.solve_drone_rtk_multiview",
        {
            "command_id": "alignment.pose.solve_drone_rtk_multiview",
            "multiview_source": "accumulated_frozen_targets",
        },
    )

    assert solve_payloads == [
        {
            "command_id": "alignment.pose.solve_drone_rtk_multiview",
            "multiview_source": "accumulated_frozen_targets",
        }
    ]
    window.close()
    app.processEvents()
    window.deleteLater()
    app.processEvents()

def test_detection_lifecycle_starts_and_stops_automatic_session() -> None:
    from Workstation.camera_stream_controller import CameraStreamLifecycleController

    calls: list[object] = []
    panel = SimpleNamespace(
        camera_connected=False,
        yolo_enabled=False,
        _pending_camera_start=None,
        _automatic_session_recording=False,
        _session_recording_active=lambda: False,
        start_automatic_session_recording=lambda: calls.append("session.start") or True,
        _start_camera_streams=lambda **kwargs: calls.append(("streams.start", kwargs)),
        _log=lambda message: calls.append(("log", message)),
    )
    controller = CameraStreamLifecycleController(panel)

    assert controller.start_yolo() is None
    assert calls == ["session.start", ("streams.start", {"enable_detection": True, "switch_to_depth": True})]

    panel.camera_connected = True
    panel.yolo_enabled = True
    panel._automatic_session_recording = True
    panel.stop_yolo_streams = lambda: calls.append("streams.stop")
    panel.connect_cameras = lambda: calls.append("preview.reconnect")
    controller.stop_yolo_detection()

    assert calls[-2:] == ["streams.stop", "preview.reconnect"]

    blocked_calls: list[object] = []
    blocked_panel = SimpleNamespace(
        camera_connected=False,
        yolo_enabled=False,
        _pending_camera_start=None,
        _session_recording_active=lambda: False,
        start_automatic_session_recording=lambda: False,
        _start_camera_streams=lambda **kwargs: blocked_calls.append(kwargs),
        _log=lambda message: blocked_calls.append(message),
    )
    assert CameraStreamLifecycleController(blocked_panel).start_yolo() is None
    assert blocked_calls == [
        {"enable_detection": True, "switch_to_depth": True}
    ]

def test_automatic_session_controller_uses_noninteractive_start_and_stop() -> None:
    from Workstation.recording_controller import RecordingController

    class Label:
        def __init__(self) -> None:
            self.messages: list[str] = []

        def setText(self, value: str) -> None:  # noqa: N802 - Qt-compatible test double.
            self.messages.append(value)

    state = {"active": False}
    calls: list[dict[str, object]] = []
    panel = SimpleNamespace(
        recording_status_label=Label(),
        _session_save_retry_required=False,
        recording_session_id="auto-session",
        _automatic_session_recording=False,
        _session_recording_active=lambda: state["active"],
    )
    controller = RecordingController(panel)
    controller._start_session_recording = lambda **kwargs: calls.append(kwargs) or state.__setitem__("active", True)
    controller._stop_session_recording = lambda **kwargs: calls.append(kwargs) or state.__setitem__("active", False)

    assert controller.start_automatic_session_recording() is True
    assert panel._automatic_session_recording is True
    assert calls == [{"start_camera_streams": False}]

    assert controller.stop_automatic_session_recording() is True
    assert panel._automatic_session_recording is False
    assert calls[-1] == {"restart_streams": False}

def test_interface_status_worker_keeps_startup_ip_query_off_ui_path(monkeypatch) -> None:
    from Workstation import standalone_gui
    from Workstation import device_runtime_workers

    monkeypatch.setattr(device_runtime_workers, "interface_addresses", lambda name: f"{name} UP")
    results: list[tuple[str, str]] = []
    worker = standalone_gui.InterfaceStatusWorker("enx-camera")
    worker.ready.connect(lambda name, status: results.append((name, status)))
    worker.run()

    assert results == [("enx-camera", "enx-camera UP")]

def test_interface_status_worker_ignores_late_signal_after_qt_teardown(monkeypatch) -> None:
    from Workstation import standalone_gui
    from Workstation import device_runtime_workers

    monkeypatch.setattr(device_runtime_workers, "interface_addresses", lambda name: f"{name} UP")
    worker = standalone_gui.InterfaceStatusWorker("enx-camera")
    attempted_emissions: list[tuple[object, ...]] = []
    monkeypatch.setattr(
        worker,
        "_emit_if_alive",
        lambda *args: attempted_emissions.append(args) or False,
    )

    worker.run()

    assert attempted_emissions == [
        ("ready", "enx-camera", "enx-camera UP"),
        ("finished",),
    ]

def test_session_save_worker_ignores_late_signal_only_after_teardown(monkeypatch, tmp_path) -> None:
    from Workstation import standalone_gui
    from Workstation import session_workers

    saved = object()
    monkeypatch.setattr(session_workers, "save_session_files", lambda *_args, **_kwargs: saved)
    worker = standalone_gui.SessionSaveWorker(
        output_dir=tmp_path,
        metadata={},
        pair_rows=(),
        camera_rows={},
        px4_rows={},
    )
    emitted: list[object] = []
    worker.saved.connect(emitted.append)
    worker.run()
    assert emitted == [saved]

    monkeypatch.setattr(worker, "_emit_if_alive", lambda *_args: False)
    worker.run()

def test_workstation_shutdown_requests_workers_without_blocking_ui_thread(
    monkeypatch: pytest.MonkeyPatch,
    tmp_path: Path,
) -> None:
    from Workstation import standalone_gui

    monkeypatch.setattr(standalone_gui.WirisStereoDistancePanel, "_initial_status", lambda self: None)
    monkeypatch.setattr(standalone_gui.WirisStereoDistancePanel, "_auto_load_intrinsics", lambda self: None)
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    panel = standalone_gui.WirisStereoDistancePanel(app_root=tmp_path)
    stop_calls: list[bool] = []
    panel.stop_all = lambda *, wait=False: stop_calls.append(wait) or False

    assert panel.shutdown_for_close() is False
    assert stop_calls == [False]

    thread_calls: list[str] = []

    class RunningThread:
        def quit(self) -> None:
            thread_calls.append("quit")

        def isRunning(self) -> bool:  # noqa: N802 - Qt-compatible test double.
            return True

        def wait(self, _timeout_ms: int) -> bool:
            raise AssertionError("non-blocking shutdown must not call QThread.wait")

    panel.test_shutdown_thread = RunningThread()
    assert panel._shutdown_stop_thread("test_shutdown_thread") is False
    assert thread_calls == ["quit"]

    panel._shutdown_coordinator = LifecycleCoordinator([])
    panel.close()
    panel.deleteLater()
    app.processEvents()


def test_shutdown_deadline_force_terminates_an_unresponsive_worker(monkeypatch) -> None:
    from Workstation import app_lifecycle_controller

    calls: list[str] = []

    class Thread:
        def isRunning(self) -> bool:  # noqa: N802
            return True

        def requestInterruption(self) -> None:  # noqa: N802
            calls.append("interrupt")

        def quit(self) -> None:
            calls.append("quit")

        def terminate(self) -> None:
            calls.append("terminate")

        def wait(self, timeout_ms: int) -> bool:
            calls.append(f"wait:{timeout_ms}")
            return True

    panel = SimpleNamespace(
        _shutdown_coordinator=SimpleNamespace(run=lambda: False),
        _shutdown_started_monotonic_s=10.0,
        blocked_thread=Thread(),
        _log=lambda message: calls.append(message),
    )
    monkeypatch.setattr(app_lifecycle_controller.time, "monotonic", lambda: 40.1)

    assert app_lifecycle_controller.AppLifecycleController(panel).shutdown_for_close() is False
    assert "terminate" in calls
    assert any(str(value).startswith("wait:") for value in calls)
    assert panel._shutdown_resource_report["complete"] is False
    assert panel._shutdown_resource_report["running_threads"]


def test_shutdown_preserves_camera_eos_window_beyond_old_five_second_deadline(
    monkeypatch,
) -> None:
    from Workstation import app_lifecycle_controller
    from Workstation.workstation_core import camera_pane, yolo_stream

    clock = [100.0]
    forced: list[float] = []

    class Coordinator:
        def run(self) -> bool:
            # Model an MP4 EOS/finalization ACK arriving eight seconds after
            # the first close request: later than the historical 5 s global
            # deadline, but within the declared bounded camera-finalization window.
            return clock[0] >= 108.0

    panel = SimpleNamespace(
        _shutdown_coordinator=Coordinator(),
        _shutdown_started_monotonic_s=None,
        _log=lambda _message: None,
    )
    controller = app_lifecycle_controller.AppLifecycleController(panel)
    controller._force_shutdown_after_deadline = lambda: forced.append(clock[0]) or False
    monkeypatch.setattr(app_lifecycle_controller.time, "monotonic", lambda: clock[0])

    assert yolo_stream.GST_RECORDING_EOS_TIMEOUT_S == pytest.approx(9.0)
    assert camera_pane.CAMERA_STOP_COOPERATIVE_WAIT_MS == 30_000
    assert app_lifecycle_controller.SHUTDOWN_DEADLINE_S >= 30.0
    assert controller.shutdown_for_close() is False
    clock[0] = 106.0
    assert controller.shutdown_for_close() is False
    assert forced == []
    clock[0] = 108.0
    assert controller.shutdown_for_close() is True
    assert forced == []


def test_shutdown_deadline_is_measured_from_late_camera_stop_request(monkeypatch) -> None:
    from Workstation import app_lifecycle_controller

    clock = [135.9]
    forced: list[float] = []
    panel = SimpleNamespace(
        _shutdown_coordinator=SimpleNamespace(run=lambda: False),
        _shutdown_started_monotonic_s=100.0,
        left_pane=SimpleNamespace(_stop_requested_monotonic_s=106.0),
        right_pane=None,
        _log=lambda _message: None,
    )
    controller = app_lifecycle_controller.AppLifecycleController(panel)
    controller._force_shutdown_after_deadline = lambda: forced.append(clock[0]) or False
    monkeypatch.setattr(app_lifecycle_controller.time, "monotonic", lambda: clock[0])

    assert controller.shutdown_for_close() is False
    assert forced == []
    clock[0] = 136.1
    assert controller.shutdown_for_close() is False
    assert forced == [136.1]


def test_shutdown_auxiliary_gate_includes_active_oz_capture_thread() -> None:
    from Workstation.app_lifecycle_controller import AppLifecycleController

    checked: list[str] = []

    def stop_thread(attr: str) -> bool:
        checked.append(attr)
        return attr != "oz_photo_capture_thread"

    panel = SimpleNamespace(
        stop_auto_calibration_capture=lambda: True,
        _stop_calibration_preview_worker=lambda *, wait=False: True,
        stop_wiris_lrf_polling=lambda *, wait=False: True,
        _measurement_view_controller_owner=None,
        _shutdown_native_frame_stores=lambda: True,
        left_pane=None,
        right_pane=None,
        _shutdown_stop_thread=stop_thread,
    )

    assert AppLifecycleController(panel)._shutdown_stop_auxiliary_workers() is False
    assert "pnp_photo_capture_thread" in checked
    assert "oz_photo_capture_thread" in checked


def test_shutdown_waits_for_all_query_and_capture_owners_before_generic_clear():
    from Workstation.app_lifecycle_controller import AppLifecycleController

    calls = []
    def owner(label, result):
        return SimpleNamespace(request_shutdown=lambda: calls.append(label) or result)
    ftp = owner("ftp", False)
    panel = SimpleNamespace(
        application=SimpleNamespace(
            features={"calibration": SimpleNamespace(state=SimpleNamespace(ftp_transactions=ftp))},
            evidence_query_runtimes=[owner("query1", False), owner("query2", True)],
        ),
        _optical_pose_controller_owner=owner("optical", False),
        _calibration_workflow_controller_owner=owner("calibration", True),
        stop_auto_calibration_capture=lambda: True,
        _stop_calibration_preview_worker=lambda **kwargs: True,
        stop_wiris_lrf_polling=lambda **kwargs: True,
        _shutdown_stop_thread=lambda name: True,
    )
    lifecycle = AppLifecycleController(panel)
    assert not lifecycle._shutdown_stop_auxiliary_workers()
    assert calls == ["query1", "query2", "optical", "calibration", "ftp"]
    sentinel = object()
    panel.calibration_wiris_ftp_thread = sentinel
    assert not lifecycle._shutdown_stop_thread("calibration_wiris_ftp_thread")
    assert panel.calibration_wiris_ftp_thread is sentinel


def test_forced_shutdown_does_not_close_data_store_or_claim_success_for_stuck_child(
    monkeypatch,
) -> None:
    from Workstation import app_lifecycle_controller

    calls: list[str] = []

    class Process:
        def is_alive(self) -> bool:
            return True

    class Worker:
        def __init__(self) -> None:
            self._process = Process()
            self._stop_requested_monotonic_s = 0.0

        def request_stop(self) -> None:
            calls.append("request")

        def terminate_child_process(self) -> None:
            calls.append("terminate-child")

        def kill_child_process(self) -> None:
            calls.append("kill-child")

    class CameraThread:
        def isRunning(self) -> bool:  # noqa: N802
            return True

        def requestInterruption(self) -> None:  # noqa: N802
            calls.append("interrupt-camera-thread")

        def quit(self) -> None:
            calls.append("quit-camera-thread")

        def terminate(self) -> None:
            calls.append("terminate-camera-thread")

        def wait(self, _timeout_ms: int) -> bool:
            return False

    camera_thread = CameraThread()

    panel = SimpleNamespace(
        _shutdown_coordinator=SimpleNamespace(run=lambda: False),
        _shutdown_started_monotonic_s=0.0,
        left_pane=SimpleNamespace(
            worker=Worker(),
            thread=camera_thread,
            native_frame_ring=None,
            _retired_native_frame_rings=[],
        ),
        right_pane=None,
        camera_stream_thread=camera_thread,
        close_workstation_data=lambda **_kwargs: calls.append("close-data") or True,
        append_workstation_data=lambda *_args, **_kwargs: True,
        _log=calls.append,
    )
    clock = [30.1]
    monkeypatch.setattr(app_lifecycle_controller.time, "monotonic", lambda: clock[0])

    assert app_lifecycle_controller.AppLifecycleController(panel).shutdown_for_close() is False
    assert "terminate-child" in calls
    assert "kill-child" not in calls
    assert "terminate-camera-thread" not in calls
    assert "close-data" not in calls
    report = panel._shutdown_resource_report
    assert report["complete"] is False
    assert "camera-child:left" in report["live_processes"]
    clock[0] = 40.1
    assert app_lifecycle_controller.AppLifecycleController(panel).shutdown_for_close() is False
    assert "kill-child" in calls
    assert "terminate-camera-thread" not in calls
    assert "close-data" not in calls


def test_forced_shutdown_reports_clean_mock_resources_before_success(monkeypatch) -> None:
    from Workstation import app_lifecycle_controller

    class Process:
        alive = True

        def is_alive(self) -> bool:
            return self.alive

    process = Process()

    class Worker:
        _process = process
        _stop_requested_monotonic_s = 0.0

        def request_stop(self) -> None:
            pass

        def terminate_child_process(self) -> None:
            process.alive = False

        def kill_child_process(self) -> None:
            process.alive = False

    class Thread:
        running = True

        def isRunning(self) -> bool:  # noqa: N802
            return self.running

        def requestInterruption(self) -> None:  # noqa: N802
            pass

        def quit(self) -> None:
            pass

        def terminate(self) -> None:
            self.running = False

        def wait(self, _timeout_ms: int) -> bool:
            return not self.running

    thread = Thread()

    class Coordinator:
        def run(self) -> bool:
            return not process.alive and not thread.running

    panel = SimpleNamespace(
        _shutdown_coordinator=Coordinator(),
        _shutdown_started_monotonic_s=0.0,
        left_pane=SimpleNamespace(
            worker=Worker(),
            native_frame_ring=None,
            _retired_native_frame_rings=[],
        ),
        right_pane=None,
        cleanup_thread=thread,
        append_workstation_data=lambda *_args, **_kwargs: True,
        _log=lambda _message: None,
    )
    monkeypatch.setattr(app_lifecycle_controller.time, "monotonic", lambda: 30.1)

    assert app_lifecycle_controller.AppLifecycleController(panel).shutdown_for_close() is True
    assert panel._shutdown_resource_report["complete"] is True
    assert panel._shutdown_resource_report["running_threads"] == []
    assert panel._shutdown_resource_report["live_processes"] == []
    assert panel._shutdown_resource_report["open_sockets"] == []
    assert panel._shutdown_resource_report["shared_memory"] == []


def test_shutdown_resource_report_detects_open_named_socket() -> None:
    from Workstation.app_lifecycle_controller import AppLifecycleController

    class Socket:
        descriptor = 17

        def fileno(self) -> int:
            return self.descriptor

    socket = Socket()
    panel = SimpleNamespace(
        camera_control_socket=socket,
        left_pane=None,
        right_pane=None,
    )
    controller = AppLifecycleController(panel)

    report = controller._shutdown_resource_snapshot()
    assert report["complete"] is False
    assert report["open_sockets"] == ["camera_control_socket"]
    socket.descriptor = -1
    assert controller._shutdown_resource_snapshot()["complete"] is True


def test_shutdown_resource_evidence_rejection_keeps_close_pending() -> None:
    from Workstation.app_lifecycle_controller import AppLifecycleController

    accepted = {"value": False}
    panel = SimpleNamespace(
        left_pane=None,
        right_pane=None,
        append_workstation_data=lambda *_args, **_kwargs: accepted["value"],
        _log=lambda _message: None,
    )
    controller = AppLifecycleController(panel)

    assert controller._shutdown_verify_resources() is False
    assert panel._shutdown_resource_report["complete"] is True
    assert not hasattr(panel, "_shutdown_resource_report_fingerprint")

    accepted["value"] = True
    assert controller._shutdown_verify_resources() is True
    assert panel._shutdown_resource_report_fingerprint


def test_stop_all_stays_pending_after_starting_async_pixhawk_disconnect() -> None:
    from Workstation.camera_stream_controller import CameraStreamLifecycleController

    class RunningThread:
        def isRunning(self) -> bool:  # noqa: N802 - Qt-compatible test double.
            return True

    panel = SimpleNamespace(
        mavlink_pixhawk=object(),
        pixhawk_disconnection_thread=None,
        pixhawk_rtcm_thread=None,
        stop_yolo_streams=lambda *, wait=False: True,
        _stop_pixhawk_operation_workers=lambda *, wait=False: True,
        _stop_calibration_preview_worker=lambda *, wait=False: True,
    )
    for attr in (
        "camera_auto_detect_thread",
        "wiris_control_check_thread",
        "wiris_command_thread",
        "wiris_zoom_thread",
        "wiris_time_sync_thread",
        "wiris_stip_thread",
        "camera_network_discovery_thread",
        "camera_network_apply_thread",
        "interface_status_thread",
        "pixhawk_usb_discovery_thread",
        "pixhawk_host_ip_discovery_thread",
        "rtsp_check_thread",
        "rtsp_enable_thread",
    ):
        setattr(panel, attr, None)

    def disconnect() -> None:
        panel.pixhawk_disconnection_thread = RunningThread()

    panel.disconnect_mavlink_pixhawk = disconnect

    assert CameraStreamLifecycleController(panel).stop_all(wait=False) is False
    assert isinstance(panel.pixhawk_disconnection_thread, RunningThread)


def test_offscreen_shutdown_waits_for_pixhawk_disconnect_without_qthread_abort(tmp_path: Path) -> None:
    repository = Path(__file__).resolve().parents[3]
    environment = os.environ.copy()
    environment["QT_QPA_PLATFORM"] = "offscreen"
    environment["PYTHONPATH"] = os.pathsep.join(
        (str(repository / "apps"), str(repository), str(repository / "common" / "python"))
    )
    script = r'''
import tempfile
import time
from pathlib import Path

from PyQt5 import QtCore, QtWidgets

from Workstation import standalone_gui


class Subscriber:
    def __init__(self):
        self.connections = {"left": object()}

    def stop(self, _camera=None):
        time.sleep(0.05)
        self.connections.clear()


standalone_gui.WirisStereoDistancePanel._initial_status = lambda self: None
standalone_gui.WirisStereoDistancePanel._auto_load_intrinsics = lambda self: None
app = QtWidgets.QApplication([])
with tempfile.TemporaryDirectory() as directory:
    panel = standalone_gui.WirisStereoDistancePanel(app_root=Path(directory))
    panel.mavlink_pixhawk = Subscriber()
    results = []
    deadline = time.monotonic() + 3.0
    while time.monotonic() < deadline:
        results.append(panel.shutdown_for_close())
        app.processEvents()
        if results[-1]:
            break
        time.sleep(0.01)
    assert results[0] is False
    assert results[-1] is True
    assert panel.pixhawk_disconnection_thread is None
    assert not any(thread.isRunning() for thread in panel.findChildren(QtCore.QThread))
    panel.deleteLater()
    app.processEvents()
'''
    completed = subprocess.run(
        [sys.executable, "-c", script],
        cwd=repository / "apps" / "Workstation",
        env=environment,
        check=False,
        capture_output=True,
        text=True,
        timeout=10,
    )

    assert completed.returncode == 0, completed.stderr
    assert "QThread: Destroyed while thread is still running" not in completed.stderr
    assert "Fatal Python error" not in completed.stderr


def test_offscreen_shutdown_stops_active_pixhawk_timer_without_qt_warning(
    tmp_path: Path,
) -> None:
    """The transport closer must not stop a GUI-owned timer from its QThread."""

    repository = Path(__file__).resolve().parents[3]
    environment = os.environ.copy()
    environment["QT_QPA_PLATFORM"] = "offscreen"
    environment["PYTHONPATH"] = os.pathsep.join(
        (str(repository / "apps"), str(repository), str(repository / "common" / "python"))
    )
    runtime_dir = tmp_path / "qt-runtime"
    runtime_dir.mkdir(mode=0o700)
    environment["XDG_RUNTIME_DIR"] = str(runtime_dir)
    script = r'''
import tempfile
import time
from pathlib import Path

from PyQt5 import QtCore, QtWidgets

from Workstation import standalone_gui
from Workstation.workstation_core.pixhawk_attitude import DualPixhawkMavlinkSubscriber


class Mav:
    def heartbeat_send(self, *_args):
        pass


class Connection:
    target_system = 1
    target_component = 1
    mavlink = None

    def __init__(self):
        self.mav = Mav()
        self.closed = False

    def recv_match(self, **_kwargs):
        return None

    def close(self):
        self.closed = True


standalone_gui.WirisStereoDistancePanel._initial_status = lambda self: None
standalone_gui.WirisStereoDistancePanel._auto_load_intrinsics = lambda self: None
app = QtWidgets.QApplication([])
with tempfile.TemporaryDirectory() as directory:
    panel = standalone_gui.WirisStereoDistancePanel(app_root=Path(directory))
    subscriber = DualPixhawkMavlinkSubscriber(left_url="", right_url="")
    connections = {"left": Connection(), "right": Connection()}
    subscriber.connections.update(connections)
    subscriber.timer.start()
    callback_ticks = []
    subscriber.timer.timeout.connect(lambda: callback_ticks.append(time.monotonic()))
    panel.mavlink_pixhawk = subscriber

    results = []
    deadline = time.monotonic() + 3.0
    while time.monotonic() < deadline:
        results.append(panel.shutdown_for_close())
        app.processEvents(QtCore.QEventLoop.AllEvents, 10)
        if results[-1]:
            break
        time.sleep(0.005)

    assert results[0] is False
    assert results[-1] is True
    assert panel._shutdown_resource_report["complete"] is True
    assert panel.pixhawk_disconnection_thread is None
    assert all(connection.closed for connection in connections.values())
    assert not subscriber.timer.isActive()
    ticks_after_shutdown = len(callback_ticks)
    settle_deadline = time.monotonic() + 0.06
    while time.monotonic() < settle_deadline:
        app.processEvents(QtCore.QEventLoop.AllEvents, 10)
        time.sleep(0.002)
    assert len(callback_ticks) == ticks_after_shutdown

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
        timeout=10,
    )

    assert completed.returncode == 0, completed.stderr
    assert completed.stderr == ""


def test_session_save_shutdown_wait_is_non_blocking_and_retryable() -> None:
    from Workstation import standalone_gui

    class Thread:
        def isRunning(self) -> bool:
            return True

        def wait(self, *_args: object) -> bool:
            raise AssertionError("GUI shutdown must not block on session save")

    panel = SimpleNamespace(
        _session_save_thread=Thread(),
        _session_save_worker=object(),
        _session_save_shutdown_wait_started_s=None,
        _session_save_shutdown_timeout_logged=False,
        _log=lambda _message: None,
    )

    assert standalone_gui.WirisStereoDistancePanel._wait_for_session_save(panel) is False
    panel._session_save_thread = None
    assert standalone_gui.WirisStereoDistancePanel._wait_for_session_save(panel) is True


def test_active_session_save_gate_runs_before_data_close() -> None:
    from sasl_common.lifecycle import LifecycleStep
    from Workstation.recording_controller import RecordingController

    class Thread:
        def isRunning(self) -> bool:  # noqa: N802
            return True

    closes: list[str] = []
    panel = SimpleNamespace(
        _session_save_thread=Thread(),
        _session_save_worker=object(),
        _session_save_shutdown_wait_started_s=None,
        _session_save_shutdown_timeout_logged=False,
        _log=lambda _message: None,
    )
    recording = RecordingController(panel)
    coordinator = LifecycleCoordinator(
        [
            LifecycleStep("wait-session-save", recording._wait_for_session_save),
            LifecycleStep("close-data", lambda: closes.append("close") or True),
        ]
    )

    assert coordinator.run() is False
    assert closes == []
    panel._session_save_thread = None
    assert coordinator.run() is True
    assert closes == ["close"]


def test_shutdown_auxiliary_gate_retries_pinned_retired_ring() -> None:
    from Workstation.app_lifecycle_controller import AppLifecycleController

    pinned = {"value": True}

    class Pane:
        def sweep_retired_native_frame_rings(self) -> bool:
            return not pinned["value"]

    panel = SimpleNamespace(
        stop_auto_calibration_capture=lambda: True,
        _stop_calibration_preview_worker=lambda *, wait=False: True,
        stop_wiris_lrf_polling=lambda *, wait=False: True,
        _measurement_view_controller_owner=None,
        _shutdown_native_frame_stores=lambda: True,
        left_pane=Pane(),
        right_pane=Pane(),
        _shutdown_stop_thread=lambda _attr: True,
    )
    controller = AppLifecycleController(panel)

    assert controller._shutdown_stop_auxiliary_workers() is False
    pinned["value"] = False
    assert controller._shutdown_stop_auxiliary_workers() is True

def test_visible_time_workspace_recomputes_live_sync_summary_after_verification_timer_stops() -> None:
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    backend = _FakeBackend()
    backend.panel.clock_sync_status_label = QtWidgets.QLabel("old status")
    backend.panel.clock_sync_summary_text = QtWidgets.QPlainTextEdit("old detail")
    refresh_count = 0

    def update_clock_sync_display() -> str:
        nonlocal refresh_count
        refresh_count += 1
        backend.panel.clock_sync_status_label.setText(f"live status {refresh_count}")
        backend.panel.clock_sync_summary_text.setPlainText(f"live detail {refresh_count}")
        return backend.panel.clock_sync_status_label.text()

    backend.panel._update_clock_sync_display = update_clock_sync_display
    window = WorkstationMainWindow(backend=backend)
    window.status_adapter.stop()
    window._select_context("devices.time")
    refresh_count = 0

    window._refresh_visible_surfaces(window.status_adapter.model.snapshot())

    assert refresh_count == 1
    assert window.time_workspace.sync_status.text() == "live status 1"
    assert window.time_workspace.sync_summary.toPlainText() == "live detail 1"

    window._select_context("overview")
    refresh_count = 0
    window._refresh_visible_surfaces(window.status_adapter.model.snapshot())
    assert refresh_count == 0

    window.close()
    window.deleteLater()
    app.processEvents()


def test_target_drone_artifact_linkage_retries_after_terminal_stop() -> None:
    from Workstation.app_lifecycle_controller import AppLifecycleController

    artifact = {
        "artifact_id": "target-artifact-1",
        "absolute_path": "/session/raw/target.jsonl",
        "size_bytes": 42,
        "sha256": "a" * 64,
    }

    class Owner:
        phase = "RUNNING"
        stop_calls = 0

        def target_drone_stream_snapshot(self) -> dict[str, object]:
            return {
                "phase": self.phase,
                "worker_alive": self.phase == "RUNNING",
                "artifact": artifact if self.phase == "SEALED" else None,
            }

        def stop_target_drone_stream(self, *, timeout_s: float) -> dict[str, object]:
            assert timeout_s == pytest.approx(1.0)
            self.stop_calls += 1
            self.phase = "SEALED"
            return self.target_drone_stream_snapshot()

    linkage_calls: list[dict[str, object]] = []

    def register(_path: object, **kwargs: object) -> dict[str, object]:
        linkage_calls.append(dict(kwargs))
        if len(linkage_calls) == 1:
            return {
                "schema": "workstation-persistence-failure-v1",
                "status": "failed",
                "kind": "artifact_registration",
                "error": "disk-sentinel",
            }
        return {"relative_path": "raw/target.jsonl"}

    owner = Owner()
    logs: list[str] = []
    panel = SimpleNamespace(
        _target_drone_controller_owner=owner,
        register_workstation_session_artifact=register,
        _log=logs.append,
    )
    controller = AppLifecycleController(panel)

    assert controller._shutdown_stop_target_drone() is False
    assert controller._shutdown_stop_target_drone() is True
    assert owner.stop_calls == 1
    assert len(linkage_calls) == 2
    assert linkage_calls[-1]["expected_size_bytes"] == 42
    assert linkage_calls[-1]["expected_sha256"] == "a" * 64
    assert any("disk-sentinel" in message for message in logs)


def test_shutdown_save_settings_retries_when_run_evidence_flush_returns_false() -> None:
    from Workstation.app_lifecycle_controller import AppLifecycleController

    calls: list[str] = []
    panel = SimpleNamespace(
        flush_workstation_run_evidence=lambda: False,
        _save_current_settings=lambda: calls.append("settings"),
        _log=calls.append,
    )

    assert AppLifecycleController(panel)._shutdown_save_settings() is False
    assert "settings" in calls
    assert any("flush 실패" in value for value in calls)


def test_emergency_flush_distinguishes_false_from_conventional_none(capsys) -> None:
    from Workstation.__main__ import _flush_window_session_state

    none_owner = SimpleNamespace(
        flush_workstation_run_evidence=lambda: None,
        distance_recording_rows=SimpleNamespace(flush=lambda: None),
    )
    _flush_window_session_state(none_owner)
    assert capsys.readouterr().err == ""

    false_owner = SimpleNamespace(
        flush_workstation_run_evidence=lambda: False,
        distance_recording_rows=SimpleNamespace(flush=lambda: False),
    )
    _flush_window_session_state(false_owner)
    error = capsys.readouterr().err
    assert "Emergency run-evidence flush failed" in error
    assert "Emergency recording-spool flush failed" in error
