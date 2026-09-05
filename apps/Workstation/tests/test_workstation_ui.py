"""Workstation ui regression tests."""

from __future__ import annotations

from Workstation.tests.workstation_test_support import (
    BackendStatusAdapter,
    EvidenceAdapter,
    _FakeBackend,
    Path,
    pytest,
    QtCore,
    QtWidgets,
    SimpleNamespace,
    StandalonePanelBackend,
    StatusSeverity,
    time,
    WorkstationMainWindow,
)


def test_rtp_timing_tracker_matches_depay_output_by_pts_and_discards_merged_markers() -> None:
    from Workstation.workstation_core.yolo_stream import GstRtpTimingTracker, RtpFrameTiming

    tracker = GstRtpTimingTracker(SimpleNamespace(), SimpleNamespace())

    def timing(pts_s: float, sequence: int) -> RtpFrameTiming:
        return RtpFrameTiming(
            rtp_timestamp=90_000 + sequence * 4_500,
            rtp_ssrc=1234,
            rtp_marker_seq=sequence,
            rtp_pts_s=pts_s,
        )

    with tracker._lock:
        tracker._pending_rtp_frames.extend(
            (timing(1.00, 10), timing(1.05, 11), timing(1.10, 12))
        )
        matched = tracker._pop_pending_rtp_timing_locked(1_050_000_000)

    assert matched is not None
    assert matched.rtp_marker_seq == 11
    assert [row.rtp_marker_seq for row in tracker._pending_rtp_frames] == [12]

    with tracker._lock:
        matched = tracker._pop_pending_rtp_timing_locked(1_100_000_000)

    assert matched is not None
    assert matched.rtp_marker_seq == 12
    assert not tracker._pending_rtp_frames


def test_status_refresh_ignores_malformed_fps_and_recovers_next_tick() -> None:
    panel = SimpleNamespace(
        camera_stream_stats={"left": {"decoded_hz": "not-a-number"}},
    )
    adapter = BackendStatusAdapter(panel)
    snapshots: list[object] = []
    adapter.polled.connect(snapshots.append)

    adapter.refresh()
    assert adapter.model.record("fps").state == "NO DATA"

    panel.camera_stream_stats = {"left": {"decoded_hz": 19.5}}
    adapter.refresh()
    assert adapter.model.record("fps").state == "AVAILABLE"
    assert len(snapshots) == 2


def test_status_refresh_isolates_one_projection_exception(monkeypatch) -> None:
    panel = SimpleNamespace(camera_stream_stats={})
    adapter = BackendStatusAdapter(panel)
    original = adapter._refresh_camera

    def fail_left(panel, side, label, now_s):
        if side == "left":
            raise ValueError("malformed camera sentinel")
        return original(panel, side, label, now_s)

    monkeypatch.setattr(adapter, "_refresh_camera", fail_left)
    adapter.refresh()

    assert adapter.model.record("camera.right") is not None
    assert adapter.model.record("status.adapter").state == "DEGRADED"
    assert "malformed camera sentinel" in adapter.model.record("status.adapter").detail


def test_overview_shows_all_read_only_method_states_and_navigates_to_details() -> None:
    from Workstation.workspaces import OverviewWorkspace
    from Workstation.status_model import StatusModel
    from Workstation.workstation_core.measurement_methods import (
        MEASUREMENT_METHOD_IDS,
    )

    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    catalog = [
        {
            "method_id": method_id,
            "runtime_status": "REUSED" if index == 0 else "BLOCKED",
            "primary_blocker": "missing frozen input" if index else "",
            "additional_blocker_count": 2 if index == 1 else 0,
            "result_counters": {"VALID": index, "BLOCKED": 1, "FAILED": 0},
        }
        for index, method_id in enumerate(MEASUREMENT_METHOD_IDS)
    ]
    backend = SimpleNamespace(
        panel=SimpleNamespace(
            recording_session_id="session-1",
            last_distance_status="Waiting",
        ),
        measurement_methods_snapshot=lambda: {"method_catalog": catalog},
    )
    workspace = OverviewWorkspace(backend, StatusModel())
    opened: list[str] = []
    workspace.open_context.connect(opened.append)

    workspace.refresh({})
    app.processEvents()

    assert tuple(workspace.measurement_method_cards) == MEASUREMENT_METHOD_IDS
    assert "PRIMARY" in workspace.measurement_method_cards[MEASUREMENT_METHOD_IDS[0]].text()
    assert "REUSED" in workspace.measurement_method_cards[MEASUREMENT_METHOD_IDS[0]].text()
    assert "+2 more" in workspace.measurement_method_cards[MEASUREMENT_METHOD_IDS[1]].text()
    workspace.measurement_method_cards[MEASUREMENT_METHOD_IDS[3]].click()
    assert opened == ["processing.measurement"]

def test_workstation_evidence_uses_panel_data_root(tmp_path, monkeypatch: pytest.MonkeyPatch) -> None:
    from wiris_stereo_distance import settings as legacy_settings

    legacy_root = tmp_path / "legacy"
    workstation_root = tmp_path / "workstation"
    monkeypatch.setattr(legacy_settings, "APP_ROOT", legacy_root)
    session_dir = workstation_root / "recordings" / "sessions" / "workstation-session"
    session_dir.mkdir(parents=True)
    (session_dir / "session_manifest.json").write_text(
        '{"status":"finalized","artifact_roles":'
        '{"workstation-session_distance_measurements.csv":"distance_measurements_csv"}}',
        encoding="utf-8",
    )
    (session_dir / "workstation-session_distance_measurements.csv").write_text(
        "recording_time_ns,forward_m,right_m,down_m,range_m\n1,2,3,4,5\n",
        encoding="utf-8",
    )
    (session_dir / "hr_captures.jsonl").write_text(
        '{"capture_id":"hr-001","recorded_unix_ns":10,"storage":"camera_internal"}\n',
        encoding="utf-8",
    )

    adapter = EvidenceAdapter(SimpleNamespace(app_root=workstation_root))
    catalog = adapter.session_catalog()
    rows = adapter.session_artifact_rows(
        "workstation-session",
        "workstation-session_distance_measurements.csv",
    )
    hr_rows = adapter.session_artifact_rows("workstation-session", "hr_captures.jsonl")

    assert catalog and catalog[0]["path"] == str(session_dir)
    assert "hr_captures.jsonl" in catalog[0]["artifacts"]
    assert rows["rows"] == (
        {"recording_time_ns": "1", "forward_m": "2", "right_m": "3", "down_m": "4", "range_m": "5"},
    )
    assert hr_rows["rows"] == (
        {"capture_id": "hr-001", "recorded_unix_ns": 10, "storage": "camera_internal"},
    )

def test_inspector_changes_by_context_and_selected_evidence() -> None:
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    backend = _FakeBackend()
    window = WorkstationMainWindow(backend=backend)
    window.show()
    app.processEvents()

    window.navigation.select_context("devices.cameras")
    app.processEvents()
    assert not window.inspector.context_section.isHidden()
    assert not window.inspector.findChildren(QtWidgets.QLineEdit)
    assert window.inspector.output_section.isHidden()
    assert "CAM L=" in window.inspector.status_value.value.text()
    assert "CAM R=" in window.inspector.status_value.value.text()
    assert window.inspector.context_section.title() == "Camera pair"
    assert window.inspector.context_row_labels["primary"].text() == "Pair"
    assert window.inspector.context_row_labels["contract"].text() == "Image contract"
    assert window.inspector.context_row_labels["primary"].objectName() == "engineeringLabel"
    assert window.inspector.context_values["primary"].label.isHidden()

    window.navigation.select_context("devices.time")
    app.processEvents()
    assert not window.inspector.status_section.isHidden()
    assert window.inspector.context_values["contract"].value.text().startswith("WIRIS RTCP/NTP")

    window.navigation.select_context("devices.pixhawk")
    app.processEvents()
    assert not window.inspector.status_section.isHidden()
    assert "P1=" in window.inspector.status_value.value.text()
    assert "P2=" in window.inspector.status_value.value.text()
    assert window.inspector.context_section.title() == "Pixhawk pair"
    window.pixhawk.attitude_table.clicked.emit(window.pixhawk.attitude_model.index(1, 0))
    app.processEvents()
    assert window.inspector.context_section.title() == "Selection details"
    assert window.inspector.context_values["primary"].value.text() == "P2 attitude"
    assert "body FRD" in window.inspector.context_values["contract"].value.text()
    assert window.inspector.context_values["timing"].isHidden()
    window.inspector.refresh("devices.pixhawk", window.status_adapter.model)
    assert window.inspector.context_section.title() == "Selection details"
    assert window.inspector.context_values["primary"].value.text() == "P2 attitude"

    window.navigation.select_context("devices.gnss")
    app.processEvents()
    assert window.inspector.context_section.title() == "Base operation"
    assert window.inspector.context_row_labels["secondary"].text() == "Survey / NTRIP"
    assert "Base position WGS84" in window.inspector.context_values["contract"].value.text()
    assert "P1/P2=" not in window.inspector.context_values["primary"].value.text()
    window.base_gnss.route_table.clicked.emit(window.base_gnss.route_model.index(0, 0))
    app.processEvents()
    assert window.inspector.context_values["primary"].value.text() == "RTCM route · P1"
    assert "transport" in window.inspector.context_values["contract"].value.text()
    window.base_gnss.route_table.clicked.emit(window.base_gnss.route_model.index(2, 0))
    app.processEvents()
    assert window.inspector.context_values["primary"].value.text() == "RTCM route · T"

    window.navigation.select_context("processing.alignment")
    app.processEvents()
    assert "Physical adjustment only" in window.inspector.context_values["verdict"].value.text()
    assert not hasattr(window.alignment, "attitude_table")

    window.navigation.select_context("processing.measurement")
    app.processEvents()
    assert not window.inspector.output_section.isHidden()
    window.measurement.values["forward"].activated.emit()
    app.processEvents()
    assert window.inspector.context_values["primary"].value.text() == "Measurement Forward"
    assert "baseline_center_frd" in window.inspector.context_values["contract"].value.text()
    window.measurement.values["forward"].set_value("4.200", unit="m")
    window.inspector.refresh("processing.measurement", window.status_adapter.model)
    assert window.inspector.context_values["secondary"].value.text() == "4.200 m"

    window.navigation.select_context("tasks.settings")
    app.processEvents()
    assert window.inspector.status_section.isHidden()
    assert "Credentials remain environment-variable references" in window.inspector.context_values["contract"].value.text()

    window.navigation.select_context("overview")
    app.processEvents()
    assert "Overview preserves each producer contract" in window.inspector.context_values["contract"].value.text()
    window.navigation.select_context("diagnostics.system")
    app.processEvents()
    assert window._current_context == "overview"
    assert window.inspector.context_label.text() == "Overview"
    assert window.diagnostics.tabs.currentIndex() == 0

    window.navigation.select_context("diagnostics.logs")
    app.processEvents()
    assert window._current_context == "overview"
    assert window.inspector.context_label.text() == "Overview"
    assert "[fake] source-backed log" in window.diagnostics.log_view.toPlainText()

    window.navigation.select_context("diagnostics.ftp")
    app.processEvents()
    assert window._current_context == "overview"
    assert window.inspector.context_label.text() == "Overview"
    assert "[fake] FTP source-backed log" in window.diagnostics.ftp_view.toPlainText()

    window.navigation.select_context("diagnostics.gnss")
    app.processEvents()
    gnss_diagnostics = window.diagnostics.gnss_view.toPlainText()
    assert "Base operation (Base receiver / Survey-in / NTRIP / RTCM)" in gnss_diagnostics
    assert "Rover receiver projection (WGS84 geodetic; not NED baseline)" in gnss_diagnostics
    assert "Baseline source projection (NED north/east/down metres)" in gnss_diagnostics

    window.navigation.select_context("diagnostics.performance")
    app.processEvents()
    assert window._current_context == "overview"
    assert window.diagnostics.tabs.currentIndex() == 7

    window.navigation.select_context("diagnostics.measurement")
    app.processEvents()
    assert window._current_context == "overview"
    assert window.inspector.context_label.text() == "Overview"

    window.navigation.select_context("tasks.advanced")
    app.processEvents()
    assert window.inspector.open_advanced_button.isHidden()

    window.close()
    app.processEvents()
    window.deleteLater()
    app.processEvents()

def test_diagnostics_tab_preserves_workspace_inspector_selection_and_row_inspection() -> None:
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    window = WorkstationMainWindow(backend=_FakeBackend())
    window.status_adapter.stop()
    window.show()
    app.processEvents()

    window.navigation.select_context("devices.pixhawk")
    app.processEvents()
    workspace = window.workspace_host.currentWidget()
    window.pixhawk.attitude_table.clicked.emit(window.pixhawk.attitude_model.index(1, 0))
    app.processEvents()
    assert window.inspector.context_values["primary"].value.text() == "P2 attitude"

    window.navigation.select_context("diagnostics.logs")
    app.processEvents()
    assert window._current_context == "devices.pixhawk"
    assert window.workspace_host.currentWidget() is workspace
    assert window.inspector.context_label.text() == "Pixhawk"
    assert window.inspector.context_values["primary"].value.text() == "P2 attitude"
    assert window.diagnostics.tabs.currentIndex() == 4

    # A row click after Diagnostics selection must still reach Inspector.
    window.pixhawk.attitude_table.clicked.emit(window.pixhawk.attitude_model.index(0, 0))
    app.processEvents()
    assert window.inspector.context_values["primary"].value.text() == "P1 attitude"
    window._refresh_evidence_from_poll(window.status_adapter.model.snapshot())
    assert window.inspector.context_values["primary"].value.text() == "P1 attitude"

    window.close()
    app.processEvents()
    window.deleteLater()
    app.processEvents()

def test_measurement_workspace_overlays_bbox_on_connected_preview() -> None:
    from PyQt5 import QtGui

    from Workstation.status_model import StatusModel
    from Workstation.workspaces import MeasurementWorkspace
    from Workstation.workstation_core.stereo import DetectionObservation

    class FakePane(QtCore.QObject):
        frame_changed = QtCore.pyqtSignal(str, QtGui.QImage, object)
        detection_changed = QtCore.pyqtSignal(str, object)
        stats_changed = QtCore.pyqtSignal(str, object)

    class FakePanel(QtCore.QObject):
        camera_preview_cleared = QtCore.pyqtSignal()

        def __init__(self) -> None:
            super().__init__()
            self.left_pane = FakePane()
            self.right_pane = FakePane()
            self.settings = SimpleNamespace()

    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    panel = FakePanel()
    left_pane = panel.left_pane
    right_pane = panel.right_pane
    backend = SimpleNamespace(panel=panel)
    workspace = MeasurementWorkspace(backend, StatusModel())
    preview = workspace.previews["left"]
    workspace.show()
    preview.resize(400, 300)
    preview.show()
    image = QtGui.QImage(640, 480, QtGui.QImage.Format_RGB888)
    image.fill(QtGui.QColor("black"))
    left_pane.frame_changed.emit("left", image, {"timestamp_s": 1.0})
    app.processEvents()
    without_box = preview.image.pixmap().toImage().copy()

    detection = DetectionObservation(
        camera="left",
        timestamp_s=1.0,
        width=640,
        height=480,
        x1=100.0,
        y1=100.0,
        x2=300.0,
        y2=300.0,
        confidence=0.9,
        class_id=0,
        class_name="person",
    )
    left_pane.detection_changed.emit("left", detection)
    app.processEvents()
    with_box = preview.image.pixmap().toImage().copy()

    assert with_box != without_box
    assert preview._detection is detection

    left_pane.detection_changed.emit("left", None)
    app.processEvents()
    assert preview.image.pixmap().toImage().copy() == without_box

    panel.camera_preview_cleared.emit()
    app.processEvents()
    assert preview._image is None
    assert preview.image.pixmap().isNull()
    assert preview.meta.text() == "WAITING | Class: --"

    workspace.deleteLater()
    left_pane.deleteLater()
    right_pane.deleteLater()
    panel.deleteLater()
    app.processEvents()

def test_workstation_shell_smoke_uses_standalone_backend_without_devices() -> None:
    from Workstation.workstation_core.settings import AppSettings

    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    source = _FakeBackend()
    panel = QtWidgets.QWidget()
    panel.__dict__.update(source.panel.__dict__)
    calls: list[tuple[str, tuple[object, ...]]] = []

    def record(name: str):
        def handler(*args: object, **_kwargs: object) -> None:
            calls.append((name, args))

        return handler

    for method_name in (
        "toggle_camera_connection",
        "toggle_yolo_detection",
        "toggle_session_recording",
        "save_distance_history_csv",
        "auto_connect_wiris",
        "discover_usb_pixhawk_connections",
        "discover_mavlink_url_candidates",
        "discover_pc_pixhawk_ip",
        "save_current_settings_from_ui",
        "apply_workstation_measurement_settings",
        "apply_workstation_system_settings",
        "set_gnss_ntrip_session_password",
    ):
        setattr(panel, method_name, record(method_name))
    panel.workstation_settings_snapshot = lambda: {
        **dict(AppSettings().__dict__),
        "gnss_operation": AppSettings().gnss_operation.to_mapping(),
    }
    panel.workstation_wiris_command_catalog = lambda: (
        {"category": "Basic", "label": "Check", "command": "HIWS", "terminator": "\n"},
    )
    panel.shutdown_for_close = lambda: True

    backend = StandalonePanelBackend(panel)
    window = WorkstationMainWindow(backend=backend)
    window.show()
    app.processEvents()

    assert panel.parent() is window._owner_host
    assert window._owner_host.isHidden()
    assert not hasattr(window, "legacy_page")
    assert window.commands.invoke("camera.auto_connect") is True
    assert window.commands.invoke("pixhawk.discover_usb") is True
    assert window.commands.invoke("settings.save") is True
    assert [name for name, _args in calls] == [
        "auto_connect_wiris",
        "discover_usb_pixhawk_connections",
        "apply_workstation_system_settings",
        "save_current_settings_from_ui",
    ]

    window._select_context("tasks.session_details")
    app.processEvents()
    assert window.workspace_host.currentWidget().widget() is window.sessions
    assert window._current_context == "tasks.data"
    assert window.navigation.currentIndex().data(QtCore.Qt.UserRole) == "tasks.data"
    assert window.sessions.current_tab_id() == "current"
    assert window.inspector.context_label.text() == "Data"
    window.close()
    app.processEvents()
    window.deleteLater()
    app.processEvents()


def test_old_recoverable_session_and_unavailable_storage_never_disable_measurement_start() -> None:
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    backend = _FakeBackend()
    backend.panel._measurement_session_state = "RECOVERABLE"
    backend.panel._measurement_session_id = "20260823_194605_395747300"
    backend.panel.recording_session_id = "20260823_194605_395747300"
    backend.data_storage_snapshot = lambda: {
        "state": "UNAVAILABLE",
        "root": "/unwritable/measurement_data",
        "priority_pending": 0,
        "bulk_pending": 0,
        "dropped_records": 4,
        "streams": {},
        "error": "permission denied",
    }
    window = WorkstationMainWindow(backend=backend)
    window.show()
    app.processEvents()

    window._refresh_from_status(window.status_adapter.model.snapshot())
    assert window.commands.action("detection.toggle").isEnabled()
    assert window.measurement.control_buttons["detection.toggle"].isEnabled()
    assert window.commands.invoke("detection.toggle") is True
    assert "detection" in backend.calls
    assert window.navigation.select_context("tasks.data")
    app.processEvents()
    assert window.navigation.currentIndex().data(QtCore.Qt.UserRole) == "tasks.data"
    assert not any(
        button.isVisible() and "Session" in button.text()
        for button in window.data.findChildren(QtWidgets.QPushButton)
    )

    window.close()
    window.deleteLater()
    app.processEvents()


@pytest.mark.parametrize(
    ("case", "sync_state", "blocking_reason"),
    (
        ("operating-kd-missing", "not-verified", "exact operating K,D missing"),
        ("operating-zoom-mismatch", "not-verified", "operating zoom mismatch"),
        ("stereo-rt-missing", "not-verified", "stereo R,t missing"),
        ("correspondence-missing", "verified", "left/right correspondence missing"),
        ("live-attitude-gnss-missing", "verified", "live Pixhawk/GNSS missing"),
        ("stus-failed", "failed", "STUS/RTCP verification failed"),
    ),
)
def test_method_readiness_failures_remain_visible_without_disabling_global_start(
    case: str,
    sync_state: str,
    blocking_reason: str,
) -> None:
    from Workstation.workstation_core.measurement_methods import (
        MEASUREMENT_METHOD_IDS,
        MEASUREMENT_METHOD_LABELS,
    )

    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    backend = _FakeBackend()
    backend.panel.camera_sync_status = sync_state
    backend.panel.camera_sync_reason = blocking_reason
    backend.panel.last_distance_status = f"거리: {blocking_reason}"
    catalog = [
        {
            "method_id": method_id,
            "label": MEASUREMENT_METHOD_LABELS[method_id],
            "instance_id": f"blocked:{case}:{method_id}",
            "ready": False,
            "reasons": [blocking_reason],
            "primary_blocker": blocking_reason,
            "runtime_status": "BLOCKED",
            "components": {},
        }
        for method_id in MEASUREMENT_METHOD_IDS
    ]
    backend._measurement_method_snapshot = {
        "selected_method_id": "",
        "selected_instance_id": "",
        "selection_reason": "no_ready_method",
        "ready_methods": [],
        "fill_policy_locked": False,
        "method_capabilities": catalog,
        "method_catalog": catalog,
    }
    window = WorkstationMainWindow(backend=backend)
    window.show()
    backend.calls.clear()
    window.status_adapter.refresh()
    app.processEvents()

    action = window.commands.action("detection.toggle")
    button = window.measurement.control_buttons["detection.toggle"]
    assert action.isEnabled(), case
    assert button.isEnabled(), case
    assert action.text() == "측정 시작"
    assert window.commands.invoke("detection.toggle") is True
    assert backend.calls == [
        (
            "settings_apply",
            {"yolo_weights_path": "models/yolov8m.pt"},
        ),
        "detection",
    ]

    window.status_adapter.stop()
    window.close()
    window.deleteLater()
    app.processEvents()

def test_protocol_selection_is_read_only_and_recording_fallback_is_visible() -> None:
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    backend = _FakeBackend()
    window = WorkstationMainWindow(backend=backend)
    window.status_adapter.stop()
    window.show()
    app.processEvents()
    backend.calls.clear()

    assert window.navigation.select_context("processing.experiment")
    app.processEvents()
    assert backend.calls == []
    assert window._current_context == "processing.experiment"
    assert window.workspace_host.currentWidget().widget() is window.experiment
    assert window.navigation.currentIndex().data(QtCore.Qt.UserRole) == "processing.experiment"
    assert window.inspector.context_label.text() == "Protocol"

    window.measurement.session_details_button.click()
    app.processEvents()
    assert window._current_context == "tasks.data"
    assert window.workspace_host.currentWidget().widget() is window.sessions
    assert window.navigation.currentIndex().data(QtCore.Qt.UserRole) == "tasks.data"
    assert window.sessions.current_tab_id() == "current"
    assert window.inspector.context_label.text() == "Data"
    window._select_context("tasks.advanced")
    window.advanced.tabs.setCurrentIndex(3)
    app.processEvents()
    manual_button = window.advanced.operation_buttons["recording.toggle"]
    assert manual_button.isVisible()
    assert manual_button.property("commandId") == "recording.toggle"
    manual_button.click()
    assert backend.calls[-1] == "recording"

    backend.panel.recording_active = True
    backend.panel._automatic_session_recording = True
    window.status_adapter.refresh()
    app.processEvents()
    assert not window.commands.action("recording.toggle").isEnabled()
    assert not manual_button.isEnabled()

    window.close()
    app.processEvents()
    window.deleteLater()
    app.processEvents()

def test_workstation_main_cards_hide_source_but_provenance_survives_in_inspector() -> None:
    from Workstation.components import EngineeringValue, StatusIndicator
    from Workstation.status_model import RuntimeStatus, StatusSeverity

    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    status = RuntimeStatus(
        key="camera.left",
        label="CAM L",
        state="LIVE",
        severity=StatusSeverity.OK,
        detail="frame age 0.10 s",
        source="CameraPane[left]",
        clock="app_receive_monotonic",
        age_s=0.10,
    )

    main_value = EngineeringValue("Range")
    main_value.set_value("12.3", source="MeasurementAttempt/controller result", frame="baseline_center_frd")
    assert "source=" not in main_value.meta.text()
    assert "frame=" not in main_value.meta.text()
    assert "Frame: baseline_center_frd" in main_value.toolTip()
    assert "From: MeasurementAttempt/controller result" in main_value.toolTip()

    inspector_value = EngineeringValue("Evidence")
    inspector_value.set_value("frame age 0.10 s", source=status.source)
    assert "source=" not in inspector_value.meta.text()
    assert "From: CameraPane[left]" in inspector_value.toolTip()

    main_indicator = StatusIndicator("CAM L")
    main_indicator.set_status(status)
    assert main_indicator.state.text() == "LIVE"
    assert main_indicator.age.text() == "100 ms"
    assert "source=" not in main_indicator.toolTip()

    valid_value = EngineeringValue("Pair")
    valid_value.set_value("person", valid=True, age_s=0.038)
    assert "VALID" not in valid_value.meta.text()
    assert "38 ms" in valid_value.meta.text()
    assert "background" not in valid_value.meta.text()
    assert "Quality: valid" in valid_value.toolTip()

    valid_value.set_value("person", valid=True)
    assert valid_value.meta.text() == ""
    assert valid_value.meta.isHidden()
    assert "Quality: valid" in valid_value.toolTip()

    invalid_value = EngineeringValue("Pair")
    invalid_value.set_value("person", valid=False)
    assert "INVALID" in invalid_value.meta.text()
    assert "Quality: invalid" in invalid_value.toolTip()

    unknown_value = EngineeringValue("Pair")
    unknown_value.set_value("person")
    assert unknown_value.meta.text() == ""
    assert "Quality:" not in unknown_value.toolTip()

    _ = app

def test_workstation_secondary_status_text_is_compact_but_tooltip_keeps_evidence() -> None:
    from Workstation.components import StatusIndicator
    from Workstation.status_model import RuntimeStatus, StatusSeverity

    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    detail = "baseline NED=+1.000/+2.000/+3.000 m; length=3.742 m; quality=GOOD; age P1/P2=0.10/0.20s; P1→P2 @ P1 body FRD F/R/D=+2.000/-1.000/+3.000 m"
    status = RuntimeStatus(
        key="gnss",
        label="RTK / GNSS",
        state="FIX",
        severity=StatusSeverity.OK,
        detail=detail,
    )

    indicator = StatusIndicator("RTK")
    indicator.set_status(status)

    assert len(indicator.detail.text()) <= 96
    assert indicator.detail.text().endswith("…")
    assert detail in indicator.toolTip()
    _ = app

def test_status_indicator_keeps_neutral_background_across_severity_changes() -> None:
    from Workstation.components import StatusIndicator
    from Workstation.status_model import RuntimeStatus, StatusSeverity
    from Workstation.workstation_core.theme import ERROR_BG, SUCCESS_BG, WARNING_BG

    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    indicator = StatusIndicator("PAIR")
    neutral_style = indicator.styleSheet()
    for state, severity in (
        ("VALID", StatusSeverity.OK),
        ("STALE", StatusSeverity.WARN),
        ("INVALID", StatusSeverity.BAD),
    ):
        indicator.set_status(
            RuntimeStatus("pair", "PAIR", state, severity, age_s=0.042)
        )
        assert indicator.styleSheet() == neutral_style
        assert indicator.state.text() == state
        assert indicator.age.text() == "42 ms"
    assert all(color not in neutral_style for color in (SUCCESS_BG, WARNING_BG, ERROR_BG))
    _ = app

def test_status_adapter_distinguishes_fresh_stale_and_error_evidence() -> None:
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    backend = _FakeBackend()
    backend.panel.left_pane = SimpleNamespace(current_status="수신 중")
    backend.panel.right_pane = SimpleNamespace(current_status="중지됨")
    backend.panel.camera_connected = True
    backend.panel.camera_frame_times["left"] = [time.monotonic()]

    adapter = BackendStatusAdapter(backend)
    adapter.refresh()
    fresh = adapter.model.record("camera.left")
    assert fresh is not None
    assert fresh.state == "LIVE"
    assert fresh.severity == StatusSeverity.OK

    backend.panel.camera_frame_times["left"] = [time.monotonic() - 3.0]
    backend.panel.left_pane.current_status = "카메라 연결 중"
    adapter.refresh()
    stale = adapter.model.record("camera.left")
    assert stale is not None
    assert stale.state == "STALE"
    assert stale.age_s is not None and stale.age_s >= 2.9

    backend.panel.camera_connected = False
    backend.panel.camera_frame_times["left"] = []
    backend.panel.left_pane.current_status = "오류: decoder"
    adapter.refresh()
    failed = adapter.model.record("camera.left")
    assert failed is not None
    assert failed.state == "ERROR"
    assert failed.severity == StatusSeverity.BAD
    _ = app

def test_status_adapter_emits_one_snapshot_per_poll() -> None:
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    adapter = BackendStatusAdapter(_FakeBackend())
    snapshots: list[object] = []
    polls: list[object] = []
    adapter.model.changed.connect(snapshots.append)
    adapter.polled.connect(polls.append)

    adapter.refresh()

    assert len(snapshots) == 1
    assert len(snapshots[0]) == 15
    assert len(polls) == 1
    assert len(polls[0]) == 15
    adapter.refresh()
    assert len(snapshots) == 1
    assert len(polls) == 2
    _ = app

def test_stale_measurement_is_not_rendered_as_valid_in_overview_or_measurement() -> None:
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    backend = _FakeBackend()
    backend.panel.last_distance_quality = "GOOD"
    backend.panel.last_distance_status = "range=12.5 m"
    backend.panel.distance_history = [
        SimpleNamespace(
            timestamp_s=time.monotonic() - 3.0,
            distance_m=12.5,
            forward_m=12.0,
            right_m=1.0,
            down_m=0.5,
            quality="GOOD",
        )
    ]
    window = WorkstationMainWindow(backend=backend)
    window.status_adapter.stop()
    window.status_adapter.refresh()
    snapshot = window.status_adapter.model.snapshot()
    window.overview.refresh(snapshot)
    window.measurement.refresh(snapshot)

    assert snapshot["measurement"].state == "STALE"
    assert "VALID" not in window.overview.measurement_value.meta.text()
    assert window.measurement.quality_labels["status"].text().startswith("STALE")
    assert all("VALID" not in value.meta.text() for value in window.measurement.values.values())

    window.close()
    app.processEvents()
    window.deleteLater()
    app.processEvents()

def test_sessionless_data_workspace_never_scans_legacy_session_catalog(monkeypatch) -> None:
    from Workstation.status_model import StatusModel
    from Workstation.workspaces import AdvancedWorkspace, RecordingWorkspace

    calls: list[float] = []

    def catalog(_adapter: object) -> list[object]:
        calls.append(time.monotonic())
        return []

    monkeypatch.setattr(EvidenceAdapter, "session_catalog", catalog)
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    backend = _FakeBackend()
    status_model = StatusModel()

    recording = RecordingWorkspace(backend, status_model, lambda: None)
    assert len(calls) == 0
    for _ in range(10):
        recording.refresh({})
    assert len(calls) == 0
    recording.refresh_sessions(force=True)
    assert len(calls) == 0

    advanced = AdvancedWorkspace(backend, status_model)
    assert len(calls) == 0
    for _ in range(10):
        advanced.refresh({})
    assert len(calls) == 0
    assert not hasattr(advanced, "media_session_combo")
    assert not hasattr(advanced, "media_artifact_combo")

    recording.deleteLater()
    advanced.deleteLater()
    app.processEvents()


def test_sessionless_data_workspace_exposes_stable_measurement_workbook_actions(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    from PyQt5 import QtGui

    from Workstation.status_model import StatusModel
    from Workstation.workspaces import RecordingWorkspace

    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    backend = _FakeBackend()
    output = tmp_path / "20260906_143005_measurement.xlsx"
    output.write_bytes(b"fixture")
    backend._measurement_workbook_snapshot.update(
        state="READY",
        latest_path=str(output),
        last_path=str(output),
        pending_method_jobs=2,
        records=[
            {
                "measurement_id": "run-a",
                "state": "STOPPED_DRAINING",
                "pending_method_jobs": 2,
                "workbook_path": str(output),
            }
        ],
        write_count=3,
    )
    opened: list[str] = []
    monkeypatch.setattr(
        QtGui.QDesktopServices,
        "openUrl",
        lambda url: opened.append(url.toLocalFile()) or True,
    )
    workspace = RecordingWorkspace(backend, StatusModel(), lambda: None)
    initial_height = workspace.measurement_workbook_status_label.height()

    workspace._refresh_measurement_workbook()
    assert workspace.open_measurement_workbook_button.objectName() == (
        "dataWorkspaceOpenMeasurementWorkbook"
    )
    assert not hasattr(workspace, "update_latest_xlsx_button")
    assert workspace.open_measurement_workbook_button.isEnabled()
    assert "READY" in workspace.measurement_workbook_status_label.text()
    assert "pending methods=2" in workspace.measurement_workbook_status_label.text()
    assert "20260906_143005_measurement.xlsx" in workspace.measurement_workbook_status_label.text()
    assert "run-a: STOPPED_DRAINING" in workspace.measurement_workbook_status_label.toolTip()

    workspace.open_measurement_workbook_button.click()
    assert opened == [str(output)]

    long_error = "permission denied · " + "x" * 300
    backend._measurement_workbook_snapshot.update(
        state="FAILED",
        latest_path="",
        error=long_error,
    )
    workspace._refresh_measurement_workbook()
    app.processEvents()
    assert not workspace.open_measurement_workbook_button.isEnabled()
    assert workspace.measurement_workbook_status_label.height() == initial_height
    assert len(workspace.measurement_workbook_status_label.text()) < len(long_error)
    assert long_error in workspace.measurement_workbook_status_label.toolTip()

    workspace.deleteLater()
    app.processEvents()


def test_sessionless_data_workspace_shows_csv_mirror_state_and_path(tmp_path: Path) -> None:
    from Workstation.status_model import StatusModel
    from Workstation.workspaces import RecordingWorkspace

    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    backend = _FakeBackend()
    jsonl_path = tmp_path / "measurements.jsonl"
    csv_path = tmp_path / "measurements.csv"
    backend.data_storage_snapshot = lambda: {
        "state": "DEGRADED",
        "root": str(tmp_path),
        "priority_pending": 0,
        "bulk_pending": 0,
        "dropped_records": 1,
        "csv_dropped_records": 2,
        "streams": {
            "measurements": {
                "path": str(jsonl_path),
                "state": "ACTIVE",
                "dropped_records": 1,
                "last_write_utc_ns": 123,
                "last_error": "",
                "csv": {
                    "path": str(csv_path),
                    "state": "FAILED",
                    "dropped_records": 2,
                    "last_error": "permission denied",
                },
            }
        },
    }
    workspace = RecordingWorkspace(backend, StatusModel(), lambda: None)
    workspace._refresh_data_storage()
    app.processEvents()

    headers = tuple(
        workspace.artifact_model.headerData(index, QtCore.Qt.Horizontal)
        for index in range(workspace.artifact_model.columnCount())
    )
    assert headers == (
        "Type",
        "JSONL path",
        "JSONL state",
        "CSV path",
        "CSV state",
        "JSONL drop",
        "CSV drop",
        "Last write",
        "Error",
    )
    row = tuple(
        workspace.artifact_model.item(0, index).text()
        for index in range(workspace.artifact_model.columnCount())
    )
    assert row == (
        "measurements",
        str(jsonl_path),
        "ACTIVE",
        str(csv_path),
        "FAILED",
        "1",
        "2",
        "123",
        "CSV: permission denied",
    )
    assert "csv_dropped=2" in workspace.session_values["status_label"].value.text()

    workspace.deleteLater()
    app.processEvents()

def test_status_refresh_updates_only_visible_workspace_and_diagnostics() -> None:
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    window = WorkstationMainWindow(backend=_FakeBackend())
    window.show()
    app.processEvents()
    window.status_adapter.stop()
    calls: list[str] = []
    window.overview.refresh = lambda _snapshot: calls.append("overview")
    window.camera.refresh = lambda _snapshot: calls.append("camera")
    window.measurement.refresh = lambda _snapshot: calls.append("measurement")
    window.diagnostics.refresh = lambda _snapshot: calls.append("diagnostics")
    window.diagnostics_dock.hide()

    window._select_context("overview")
    calls.clear()
    window._refresh_evidence_from_poll({})
    assert calls == ["overview"]

    window._select_context("devices.cameras")
    calls.clear()
    window._refresh_evidence_from_poll({})
    assert calls == ["camera"]

    window.diagnostics_dock.show()
    app.processEvents()
    calls.clear()
    window._refresh_evidence_from_poll({})
    assert calls == ["camera", "diagnostics"]
    assert "measurement" not in calls

    window.close()
    window.deleteLater()
    app.processEvents()

def test_diagnostics_refreshes_only_the_visible_tab_and_skips_unchanged_text() -> None:
    from Workstation.status_model import StatusModel
    from Workstation.workspaces import DiagnosticsContent

    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    diagnostics = DiagnosticsContent(_FakeBackend(), StatusModel())
    calls: list[int] = []
    refresh_names = (
        "_refresh_system",
        "_refresh_camera",
        "_refresh_pixhawk",
        "_refresh_gnss",
        "_refresh_log",
        "_refresh_ftp",
        "_refresh_measurement",
        "_refresh_performance",
    )
    for index, name in enumerate(refresh_names):
        setattr(diagnostics, name, lambda _snapshot, index=index: calls.append(index))

    for index in range(diagnostics.tabs.count()):
        diagnostics.tabs.setCurrentIndex(index)
        calls.clear()
        diagnostics.refresh({})
        assert calls == [index]

    changes: list[bool] = []
    diagnostics.camera_view.textChanged.connect(lambda: changes.append(True))
    diagnostics._set_diagnostic_text(diagnostics.camera_view, "same evidence")
    diagnostics._set_diagnostic_text(diagnostics.camera_view, "same evidence")
    assert len(changes) == 1

def test_diagnostics_log_preserves_manual_scroll_and_offers_follow_latest() -> None:
    from Workstation.status_model import StatusModel
    from Workstation.workspaces import DiagnosticsContent

    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    diagnostics = DiagnosticsContent(_FakeBackend(), StatusModel())
    initial = "\n".join(f"line {index}" for index in range(80))
    diagnostics._set_log_text(initial)
    scrollbar = diagnostics.log_view.verticalScrollBar()
    scrollbar.setValue(12)
    saved_position = scrollbar.value()

    diagnostics._set_log_text(initial + "\nnew line 1\nnew line 2")
    assert scrollbar.value() == saved_position
    assert diagnostics.log_new_label.text() == "New logs: 2"
    assert not diagnostics.log_new_label.isHidden()
    assert not diagnostics.log_follow_button.isHidden()

    diagnostics.log_follow_button.click()
    assert scrollbar.value() == scrollbar.maximum()
    assert diagnostics.log_new_label.isHidden()
    assert diagnostics.log_follow_button.isHidden()

    diagnostics.deleteLater()
    app.processEvents()

def test_polled_table_updates_preserve_selected_row() -> None:
    from PyQt5 import QtGui

    from Workstation.workspaces import _sync_table_rows

    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    model = QtGui.QStandardItemModel(0, 2)
    table = QtWidgets.QTableView()
    table.setModel(model)
    table.setSelectionBehavior(QtWidgets.QAbstractItemView.SelectRows)
    _sync_table_rows(model, (("P1", "WAITING"), ("P2", "WAITING")))
    table.selectRow(0)
    app.processEvents()

    _sync_table_rows(model, (("P1", "LIVE"), ("P2", "WAITING")))

    assert model.item(0, 1).text() == "LIVE"
    assert [index.row() for index in table.selectionModel().selectedRows()] == [0]
    table.deleteLater()
    app.processEvents()

def test_data_workspace_leaves_legacy_session_files_for_file_manager_only(tmp_path, monkeypatch) -> None:
    from Workstation.workstation_core import settings as settings_module

    monkeypatch.setattr(settings_module, "APP_ROOT", tmp_path)
    session_dir = tmp_path / "recordings" / "sessions" / "session-1"
    generation_dir = session_dir / ".generations" / "generation-1"
    generation_dir.mkdir(parents=True)
    (generation_dir / "session-1_distance_measurements.csv").write_text(
        "recording_time_ns,forward_m,right_m,down_m,range_m\n1,2,3,4,5\n",
        encoding="utf-8",
    )
    (session_dir / "published").symlink_to(Path(".generations") / "generation-1")
    (session_dir / "session_manifest.json").write_text(
        '{"status":"finalized","generation_id":"generation-1",'
        '"time_axis":"recording_time_ns",'
        '"artifact_roles":{"session-1_distance_measurements.csv":"distance_measurements_csv"}}',
        encoding="utf-8",
    )
    before = (generation_dir / "session-1_distance_measurements.csv").read_bytes()

    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    window = WorkstationMainWindow(backend=_FakeBackend())
    window.show()
    app.processEvents()
    assert window.recording.session_selector.count() == 0
    assert [
        window.recording.tabs.tabText(index)
        for index in range(window.recording.tabs.count())
    ] == ["Status", "Capture"]
    assert window.recording.open_legacy_recordings_button.text() == (
        "Open Legacy Recordings Folder"
    )
    assert (generation_dir / "session-1_distance_measurements.csv").read_bytes() == before
    window.close()
    app.processEvents()
    window.deleteLater()
    app.processEvents()
