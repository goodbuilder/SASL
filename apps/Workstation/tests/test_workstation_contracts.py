"""Workstation contracts regression tests."""

from __future__ import annotations

from Workstation.tests.workstation_test_support import (
    CommandRegistry,
    CommandSpec,
    create_standalone_backend,
    _FakeBackend,
    LifecycleCoordinator,
    LifecycleStep,
    _parse_ui_mode,
    Path,
    pytest,
    QtCore,
    QtWidgets,
    SimpleNamespace,
    StandalonePanelBackend,
    WorkstationMainWindow,
)


def test_ui_mode_parser_removes_workstation_argument() -> None:
    mode, argv = _parse_ui_mode(["main.py", "--ui", "workstation", "--maximized"])

    assert mode == "workstation"
    assert argv == ["main.py", "--maximized"]
    assert _parse_ui_mode(["main.py"])[0] == "legacy"

def test_gui_compatibility_module_aliases_legacy_implementation() -> None:
    import wiris_stereo_distance.gui as gui
    import wiris_stereo_distance.legacy_gui as legacy_gui

    assert gui is legacy_gui
    assert gui.__file__.endswith("/legacy_gui.py")
    assert gui.WirisStereoDistancePanel is legacy_gui.WirisStereoDistancePanel

def test_workstation_single_instance_lock_blocks_duplicate(tmp_path: Path) -> None:
    from Workstation.__main__ import _acquire_single_instance_lock

    lock_path = tmp_path / "workstation.lock"
    first = _acquire_single_instance_lock(lock_path)
    assert first is not None
    try:
        assert _acquire_single_instance_lock(lock_path) is None
    finally:
        first.unlock()

def test_workstation_core_has_no_legacy_ros_or_image_timesync_surface() -> None:
    repo_root = Path(__file__).resolve().parents[3]
    core_root = repo_root / "apps" / "Workstation" / "workstation_core"
    forbidden_markers = (
        "wiris_stereo_distance",
        "rclpy",
        "rosidl",
        "ros2",
        "ros 2",
        "uxrce",
        "mavros",
        "photo_clock",
        "cptr",
        "icpt",
        "exif",
        "clock_ftp",
        "still-image",
        "still image",
    )
    for source_path in (*core_root.glob("*.py"), *(repo_root / "Workstation").glob("*.py")):
        if source_path.name == "__init__.py":
            continue
        source = source_path.read_text(encoding="utf-8").lower()
        assert not any(marker in source for marker in forbidden_markers), source_path

    from Workstation.workstation_core.pixhawk_attitude import DualPixhawkMavlinkSubscriber
    from Workstation.workstation_core.settings import AppSettings

    assert not hasattr(DualPixhawkMavlinkSubscriber, "start_ros2_client")
    assert not hasattr(DualPixhawkMavlinkSubscriber, "start_px4_uxrce_client_shell")
    assert not hasattr(AppSettings, "ros_domain_id")
    assert not hasattr(AppSettings, "mavlink_configure_uxrce_agent")
    assert not hasattr(AppSettings, "clock_ftp_user")

def test_legacy_ros2_and_image_timesync_surface_remains_available() -> None:
    from wiris_stereo_distance import settings as legacy_settings
    from wiris_stereo_distance.photo_clock import compare_photo_clocks
    from wiris_stereo_distance.pixhawk_attitude import DualPixhawkMavlinkSubscriber

    assert hasattr(legacy_settings.AppSettings, "ros_domain_id")
    assert hasattr(legacy_settings.AppSettings, "clock_ftp_user")
    assert hasattr(DualPixhawkMavlinkSubscriber, "start_ros2_client")
    assert callable(compare_photo_clocks)

def test_experiment_protocol_readiness_is_advisory_and_mode_specific() -> None:
    from Workstation.protocol import evaluate_recording_protocol

    snapshot = {
        key: SimpleNamespace(state="LIVE" if key.startswith(("camera.", "pixhawk.")) else state)
        for key, state in (
            ("camera.left", "LIVE"),
            ("camera.right", "LIVE"),
            ("synchronization", "SYNCED"),
            ("detection", "RUNNING"),
            ("pixhawk.left", "LIVE"),
            ("pixhawk.right", "LIVE"),
        )
    }
    calibration = {
        "active": True,
        "left": {"state": "AVAILABLE"},
        "right": {"state": "AVAILABLE"},
        "extrinsics": {"state": "AVAILABLE"},
    }
    baseline = {"state": "GOOD", "length_m": 1.2}

    pixhawk_ready = evaluate_recording_protocol(
        snapshot,
        distance_mode="pixhawk",
        calibration=calibration,
        baseline=baseline,
        require_rtk_fixed=True,
    )
    assert pixhawk_ready.state == "READY"
    assert not pixhawk_ready.blocking

    checkerboard_ready = evaluate_recording_protocol(
        snapshot,
        distance_mode="checkerboard",
        calibration=calibration,
        baseline={"state": "NO DATA", "length_m": None},
        require_rtk_fixed=True,
    )
    assert checkerboard_ready.state == "READY"

    snapshot["synchronization"] = SimpleNamespace(state="UNVERIFIED")
    blocked = evaluate_recording_protocol(
        snapshot,
        distance_mode="pixhawk",
        calibration=calibration,
        baseline=baseline,
        require_rtk_fixed=True,
    )
    assert blocked.state == "BLOCKED"
    assert {check.key for check in blocked.blocking} == {"synchronization"}

def test_experiment_step_one_requires_mode_specific_base_link_and_step_ten_tracks_detection() -> None:
    from Workstation.workspaces import ExperimentWorkspace
    from Workstation.workstation_core.gnss_operation import GnssOperationConfig, GnssOperationManager

    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    manager = GnssOperationManager(GnssOperationConfig())
    manager.configure(GnssOperationConfig())
    panel = SimpleNamespace(
        gnss_operation_manager=manager,
        gnss_base_serial_connected=False,
        gnss_network_connected=False,
        gnss_network_status_line="",
        gnss_base_rtcm_stats={},
        latest_gnss_base_survey_in={},
        gnss_base_survey_samples=(),
        gnss_coordinate_samples=(),
        latest_gps={
            "left": SimpleNamespace(fix_type=3, rtk_label="3D FIX", topic="mavlink/GPS_RAW_INT"),
            "right": SimpleNamespace(fix_type=3, rtk_label="3D FIX", topic="mavlink/GPS_RAW_INT"),
        },
    )
    alignment_snapshot: dict[str, object] = {
        "state": "WAITING",
        "sampling_active": False,
        "sample_count": 0,
        "iterations": (),
        "quality": {},
    }
    panel.workstation_physical_alignment_snapshot = lambda: alignment_snapshot
    workspace = ExperimentWorkspace(SimpleNamespace(panel=panel))
    assert [button.text() for button in workspace.step_action_buttons[1]] == ["Cameras", "Pixhawk", "Base"]
    assert [button.text() for button in workspace.step_action_buttons[4]] == ["Base", "Pixhawk"]
    opened: list[str] = []
    workspace.open_context.connect(opened.append)
    workspace.step_action_buttons[1][1].click()
    workspace.step_action_buttons[4][0].click()
    workspace.pre_run_calibration_button.click()
    assert opened == ["devices.pixhawk", "devices.gnss", "tasks.calibration"]
    snapshot = {
        "camera.left": SimpleNamespace(state="LIVE"),
        "camera.right": SimpleNamespace(state="LIVE"),
        "pixhawk.left": SimpleNamespace(state="LIVE"),
        "pixhawk.right": SimpleNamespace(state="LIVE"),
        "synchronization": SimpleNamespace(state="SYNCED"),
        "detection": SimpleNamespace(state="RUNNING"),
        "measurement": SimpleNamespace(state="WAITING"),
        "recording": SimpleNamespace(state="IDLE"),
        "gnss": SimpleNamespace(state="NO FIX"),
    }

    workspace.refresh(snapshot)
    assert workspace.step_states[1].text() == "WAIT"
    assert "BASE=DISCONNECTED" in workspace.step_details[1].text()
    assert workspace.step_states[10].text() == "RUNNING"
    assert "detection=RUNNING" in workspace.step_details[10].text()

    panel.gnss_base_serial_connected = True
    workspace.refresh(snapshot)
    assert workspace.step_states[1].text() == "READY"
    assert "BASE=CONNECTED" in workspace.step_details[1].text()

    snapshot["gnss"] = SimpleNamespace(state="STALE")
    workspace.refresh(snapshot)
    assert workspace.step_states[1].text() == "WAIT"
    snapshot["gnss"] = SimpleNamespace(state="NO FIX")

    alignment_snapshot.update(
        {
            "state": "FAIL",
            "sample_count": 20,
            "relative_rpy_deg": (2.0, 0.0, 0.0),
            "iterations": ({"iteration": 1},),
            "quality": {
                "baseline_error_m": (0.001, -0.001, 0.0),
                "relative_rpy_error_deg": (2.0, 0.0, 0.0),
                "reasons": ("relative_rpy_out_of_tolerance",),
            },
        }
    )
    workspace.refresh(snapshot)
    assert workspace.step_states[5].text() == "READY"
    assert "FRD error=" in workspace.step_details[5].text()
    assert workspace.step_states[6].text() == "CHECK"
    assert workspace.step_states[7].text() == "CHECK"

    alignment_snapshot["state"] = "PASS"
    alignment_snapshot["iterations"] = ({"iteration": 1}, {"iteration": 2})
    alignment_snapshot["quality"] = {
        "baseline_error_m": (0.001, -0.001, 0.0),
        "relative_rpy_error_deg": (0.1, -0.1, 0.2),
        "reasons": (),
    }
    workspace.refresh(snapshot)
    assert [workspace.step_states[number].text() for number in (5, 6, 7)] == ["READY", "READY", "READY"]

    panel.recording_session_id = "unsaved-session"
    workspace.refresh(snapshot)
    assert workspace.step_states[11].text() == "READY"
    assert "automatically" in workspace.step_details[11].text()
    snapshot["recording"] = SimpleNamespace(state="SAVED")
    workspace.refresh(snapshot)
    assert workspace.step_states[11].text() == "SAVED"

    workspace.deleteLater()
    app.processEvents()

def test_standalone_backend_is_a_separate_copy_and_factory_owner(monkeypatch: pytest.MonkeyPatch) -> None:
    from Workstation import standalone_gui

    assert Path(standalone_gui.__file__).name == "standalone_gui.py"

    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    panel = QtWidgets.QWidget()
    monkeypatch.setattr(standalone_gui, "WirisStereoDistancePanel", lambda **_kwargs: panel)
    backend = create_standalone_backend()

    assert isinstance(backend, StandalonePanelBackend)
    assert backend.panel is panel
    assert backend.owner_widget is panel
    backend.deleteLater()
    panel.deleteLater()
    app.processEvents()

def test_standalone_backend_facades_match_the_checked_contract() -> None:
    from Workstation.protocol import missing_backend_facades

    assert missing_backend_facades(StandalonePanelBackend) == ()

def test_lifecycle_coordinator_resumes_after_pending_step() -> None:
    calls: list[str] = []
    pending = {"value": True}

    def first() -> bool:
        calls.append("first")
        if pending["value"]:
            pending["value"] = False
            return False
        return True

    coordinator = LifecycleCoordinator(
        [LifecycleStep("first", first), LifecycleStep("second", lambda: calls.append("second"))]
    )

    assert coordinator.run() is False
    assert calls == ["first"]
    assert coordinator.run() is True
    assert calls == ["first", "first", "second"]
    assert coordinator.run() is True
    assert calls == ["first", "first", "second"]

def test_workstation_recording_gate_protects_preview_mapping_and_save() -> None:
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    backend = _FakeBackend()
    window = WorkstationMainWindow(backend=backend)
    window.show()
    app.processEvents()

    backend.panel.recording_active = True
    backend.panel.recording_session_id = "active-session"
    window.status_adapter.refresh()
    app.processEvents()

    assert not window.commands.action("camera.toggle").isEnabled()
    assert not window.commands.action("time.set_pc").isEnabled()
    assert not window.commands.action("camera.swap").isEnabled()
    assert not window.commands.action("camera.auto_connect").isEnabled()
    assert not window.commands.action("camera.reactivate_left").isEnabled()
    assert not window.commands.action("camera.reactivate_right").isEnabled()
    assert not window.commands.action("alignment.swap_pixhawks").isEnabled()
    assert not window.commands.action("pixhawk.reset_level_calibration").isEnabled()
    assert not window.commands.action("pixhawk.toggle_mavlink").isEnabled()
    assert window.commands.action("detection.toggle").isEnabled()
    with pytest.raises(KeyError):
        window.commands.action("session.save")
    assert not window.measurement.control_buttons["camera.toggle"].isEnabled()
    assert window.measurement.control_buttons["detection.toggle"].isEnabled()
    assert not window.time_workspace.operation_buttons["time.set_pc"].isEnabled()
    assert not window.camera.operation_buttons["camera.swap"].isEnabled()
    assert not window.camera.operation_buttons["camera.reactivate_left"].isEnabled()
    assert not window.camera.operation_buttons["camera.reactivate_right"].isEnabled()
    assert "alignment.swap_pixhawks" not in window.alignment.operation_buttons
    assert "pixhawk.reset_level_calibration" not in window.alignment.operation_buttons
    calls_before_disabled_invoke = list(backend.calls)
    assert window.commands.invoke("camera.swap") is False
    assert window.commands.invoke("camera.auto_connect") is False
    assert window.commands.invoke("camera.reactivate_left") is False
    assert window.commands.invoke("camera.reactivate_right") is False
    assert window.commands.invoke("alignment.swap_pixhawks") is False
    assert window.commands.invoke("pixhawk.reset_level_calibration") is False
    assert window.commands.invoke("pixhawk.toggle_mavlink") is False
    assert backend.calls == calls_before_disabled_invoke

    backend.panel.camera_connected = True
    window.status_adapter.refresh()
    app.processEvents()
    assert window.commands.action("detection.toggle").isEnabled()
    assert window.measurement.control_buttons["detection.toggle"].isEnabled()

    backend.panel.recording_active = False
    backend.panel.camera_connected = False
    backend.panel._session_save_finalized = True
    window.status_adapter.refresh()
    app.processEvents()
    assert window.commands.action("camera.toggle").isEnabled()
    assert window.commands.action("time.set_pc").isEnabled()
    with pytest.raises(KeyError):
        window.commands.action("session.save")

    backend.panel.wiris_time_sync_thread = object()
    window._refresh_from_status(window.status_adapter.model.snapshot())
    app.processEvents()
    assert not window.commands.action("recording.toggle").isEnabled()
    backend.panel.wiris_time_sync_thread = None

    window.close()
    app.processEvents()
    window.deleteLater()
    app.processEvents()


def test_measurement_command_and_button_do_not_diverge_during_manual_recording() -> None:
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    backend = _FakeBackend()
    package = {
        "state": "IDLE",
        "session_state": "OPEN",
        "session_id": "session-001",
        "measurement_id": "",
        "revision": "",
        "detection_started": False,
        "detection_completed": False,
        "artifacts": [],
    }
    session = {
        "state": "OPEN",
        "session_id": "session-001",
        "measurements": [],
    }
    backend.measurement_package_snapshot = lambda: dict(package)
    backend.measurement_session_snapshot = lambda: dict(session)
    backend.begin_measurement = lambda: True
    backend.toggle_detection = lambda: backend.calls.append("detection.rejected") or False
    window = WorkstationMainWindow(backend=backend)

    backend.panel.recording_active = True
    backend.panel.camera_connected = True
    backend.panel._automatic_session_recording = False
    window.status_adapter.refresh()
    app.processEvents()

    assert window.measurement.control_buttons["detection.toggle"].isEnabled()
    assert window.commands.action("detection.toggle").isEnabled()
    assert window.commands.invoke("detection.toggle") is False
    assert backend.calls[-1] == "detection.rejected"
    assert package["state"] == "IDLE"
    assert window.measurement.control_buttons["detection.toggle"].isEnabled()
    assert window.commands.action("detection.toggle").isEnabled()

    window.status_adapter.stop()
    window.close()
    window.deleteLater()
    app.processEvents()


def test_recording_preview_updates_in_place_and_key_value_rows_do_not_show_sample_detail(
    monkeypatch: pytest.MonkeyPatch,
    tmp_path: Path,
) -> None:
    from Workstation import standalone_gui

    monkeypatch.setattr(standalone_gui.WirisStereoDistancePanel, "_initial_status", lambda self: None)
    monkeypatch.setattr(standalone_gui.WirisStereoDistancePanel, "_auto_load_intrinsics", lambda self: None)
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    panel = standalone_gui.WirisStereoDistancePanel(app_root=tmp_path)
    table = QtWidgets.QTableWidget()
    panel._fill_recording_sheet(table, ("a", "b"), (("1", "2"), ("3", "4")))
    table.selectRow(0)
    original_item = table.item(0, 0)

    panel._fill_recording_sheet(table, ("a", "b"), (("updated", "2"), ("3", "4")))
    assert table.item(0, 0) is original_item
    assert table.item(0, 0).text() == "updated"
    assert table.selectionModel().selectedRows()[0].row() == 0

    panel.distance_recording_rows = [{"distance_m": 12.3}]
    summary_index = panel.recording_sheet_tabs.indexOf(panel.recording_sheet_tables["Summary"])
    panel.recording_sheet_tabs.setCurrentIndex(summary_index)
    summary_table = panel.recording_sheet_tables["Summary"]
    panel._fill_recording_sheet(summary_table, ("key", "value"), (("session", "test"),))
    summary_table.selectRow(0)
    panel._refresh_recording_sample_detail()
    assert panel.recording_sample_detail.toPlainText() == ""

    panel._shutdown_coordinator = LifecycleCoordinator([])
    panel.close()
    panel.deleteLater()
    table.deleteLater()
    app.processEvents()

def test_workstation_exposes_device_discovery_and_routes_tools_to_advanced() -> None:
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    backend = _FakeBackend()
    window = WorkstationMainWindow(backend=backend)
    window.show()
    app.processEvents()

    assert window.camera.operation_buttons["camera.toggle"].text() == "Preview"
    assert window.camera.operation_buttons["camera.auto_connect"].text() == "Connect cameras"
    assert window.camera.operation_buttons["camera.reactivate_left"].text() == "Reactivate CAM L"
    assert window.camera.operation_buttons["camera.reactivate_right"].text() == "Reactivate CAM R"
    assert window.camera.operation_buttons["advanced.open"].text() == "Advanced"
    assert window.camera.preview_splitter.minimumHeight() >= 320
    assert window.camera.workspace_splitter.orientation() == QtCore.Qt.Vertical
    assert window.camera.details_scroll.widgetResizable() is True
    assert window.camera.preview_pair_orientation(1600, 700) == QtCore.Qt.Horizontal
    assert window.camera.preview_pair_orientation(900, 700) == QtCore.Qt.Vertical
    window.navigation.select_context("processing.measurement")
    app.processEvents()
    assert window.measurement.preview_splitter.minimumHeight() >= 280
    assert window.measurement.preview_splitter.sizePolicy().verticalPolicy() == QtWidgets.QSizePolicy.Fixed
    assert window.measurement.upstream_group.parentWidget() is window.measurement.details
    assert window.measurement.details.layout().indexOf(window.measurement.upstream_group) == 0
    assert not hasattr(window.measurement, "workspace_splitter")
    assert not hasattr(window.measurement, "details_scroll")
    assert window.measurement.preview_pair_orientation(1600, 700) == QtCore.Qt.Horizontal
    assert window.measurement.preview_pair_orientation(900, 700) == QtCore.Qt.Vertical
    assert not hasattr(window.measurement, "active_calibration_label")
    for command_id in (
        "camera.auto_connect",
        "pixhawk.discover_usb",
        "pixhawk.discover_mavlink_candidates",
        "pixhawk.discover_host_ip",
    ):
        assert window.commands.invoke(command_id) is True

    assert backend.calls[-4:] == [
        "camera_auto_connect",
        "pixhawk_discover_usb",
        "pixhawk_discover_mavlink_candidates",
        "pixhawk_discover_host_ip",
    ]
    assert window.commands.invoke("advanced.open") is True
    assert window.workspace_host.currentWidget().widget() is window.advanced
    assert "legacy" not in window._workspace_indexes

    window.close()
    app.processEvents()
    window.deleteLater()
    app.processEvents()

def test_measurement_preview_fits_without_crop_when_widget_is_taller_than_aspect() -> None:
    from PyQt5 import QtGui

    from Workstation.workspaces import CameraPreview

    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    preview = CameraPreview("left", crop_to_fill=False)
    preview.resize(400, 300)
    preview.show()
    app.processEvents()
    image = QtGui.QImage(640, 360, QtGui.QImage.Format_RGB888)
    image.fill(QtGui.QColor("red"))
    preview.set_frame(image, {})
    app.processEvents()

    rendered = preview.image.pixmap().toImage()
    assert rendered.width() > 0 and rendered.height() > 0
    assert rendered.pixelColor(rendered.width() // 2, 1) != QtGui.QColor("red")
    assert rendered.pixelColor(rendered.width() // 2, rendered.height() // 2) == QtGui.QColor("red")

    preview.close()
    preview.deleteLater()
    app.processEvents()

def test_measurement_preview_geometry_stays_fixed_when_frames_and_labels_change() -> None:
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
    workspace = MeasurementWorkspace(SimpleNamespace(panel=panel), StatusModel())
    workspace.resize(1200, 800)
    workspace.show()
    app.processEvents()
    app.processEvents()

    def emit_pair(width: int, height: int, class_name: str, timestamp_s: float) -> None:
        for side, pane in (("left", panel.left_pane), ("right", panel.right_pane)):
            image = QtGui.QImage(width, height, QtGui.QImage.Format_RGB888)
            image.fill(QtCore.Qt.black)
            pane.frame_changed.emit(side, image, {"timestamp_s": timestamp_s})
            pane.detection_changed.emit(
                side,
                DetectionObservation(
                    camera=side,
                    timestamp_s=timestamp_s,
                    width=width,
                    height=height,
                    x1=10.0,
                    y1=10.0,
                    x2=min(100.0, width - 1.0),
                    y2=min(100.0, height - 1.0),
                    confidence=0.9,
                    class_id=0,
                    class_name=class_name,
                ),
            )
        app.processEvents()
        app.processEvents()

    emit_pair(640, 480, "person", 1.0)
    baseline = (
        workspace.preview_splitter.geometry().getRect(),
        tuple(preview.geometry().getRect() for preview in workspace.previews.values()),
        tuple(preview.image.geometry().getRect() for preview in workspace.previews.values()),
    )
    for timestamp_s, (width, height, class_name) in enumerate(
        ((1280, 720, "person"), (1672, 941, "very-long-class-name"), (640, 480, "person")),
        start=2,
    ):
        emit_pair(width, height, class_name, timestamp_s)
        current = (
            workspace.preview_splitter.geometry().getRect(),
            tuple(preview.geometry().getRect() for preview in workspace.previews.values()),
            tuple(preview.image.geometry().getRect() for preview in workspace.previews.values()),
        )
        assert current == baseline

    assert all(
        preview.meta.minimumHeight() == preview.meta.maximumHeight()
        for preview in workspace.previews.values()
    )
    workspace.deleteLater()
    panel.deleteLater()
    app.processEvents()

def test_measurement_preview_keeps_frozen_detection_frame_until_pair_is_cleared() -> None:
    from PyQt5 import QtGui

    from Workstation.status_model import StatusModel
    from Workstation.workspaces import MeasurementWorkspace
    from Workstation.workstation_core.camera_pane import CameraPane

    class FakePanel(QtCore.QObject):
        camera_preview_cleared = QtCore.pyqtSignal()

        def __init__(self) -> None:
            super().__init__()
            self.left_pane = CameraPane("left")
            self.right_pane = CameraPane("right")
            self.settings = SimpleNamespace()

    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    panel = FakePanel()
    workspace = MeasurementWorkspace(SimpleNamespace(panel=panel), StatusModel())
    workspace.show()
    workspace.previews["left"].show()
    app.processEvents()

    selected = QtGui.QImage(16, 9, QtGui.QImage.Format_RGB888)
    selected.fill(QtGui.QColor("green"))
    live = QtGui.QImage(16, 9, QtGui.QImage.Format_RGB888)
    live.fill(QtGui.QColor("blue"))

    panel.left_pane.show_selected_measurement_frame(selected)
    app.processEvents()
    assert QtGui.QColor(workspace.previews["left"]._image.pixel(0, 0)).name() == QtGui.QColor("green").name()

    panel.left_pane.detection_stream_active = True
    panel.left_pane._show_frame(live, {"timestamp_s": 1.0})
    app.processEvents()
    assert QtGui.QColor(workspace.previews["left"]._image.pixel(0, 0)).name() == QtGui.QColor("green").name()

    panel.left_pane.clear_measurement_pair_display()
    panel.left_pane._show_frame(live, {"timestamp_s": 2.0})
    app.processEvents()
    assert QtGui.QColor(workspace.previews["left"]._image.pixel(0, 0)).name() == QtGui.QColor("blue").name()

    workspace.deleteLater()
    panel.left_pane.deleteLater()
    panel.right_pane.deleteLater()
    panel.deleteLater()
    app.processEvents()

def test_command_registry_distinguishes_deferred_completion() -> None:
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    registry = CommandRegistry()
    events: list[tuple[str, str]] = []
    registry.accepted.connect(lambda command_id, _result: events.append(("accepted", command_id)))
    registry.completed.connect(lambda command_id, _result: events.append(("completed", command_id)))
    registry.register(CommandSpec("sync", "Sync", "", "test", lambda: "done"))
    registry.register(CommandSpec("async", "Async", "", "test", lambda: None))
    disabled_calls: list[str] = []
    registry.register(
        CommandSpec("disabled", "Disabled", "", "test", lambda: disabled_calls.append("called"))
    )
    registry.set_deferred_completion("async")
    registry.set_enabled("disabled", False)

    assert registry.invoke("sync") is True
    assert registry.invoke("async") is True
    assert registry.invoke("disabled") is False
    assert disabled_calls == []
    assert events == [
        ("accepted", "sync"),
        ("completed", "sync"),
        ("accepted", "async"),
    ]
    registry.deleteLater()
    app.processEvents()

def test_auto_connect_does_not_repeat_seth_after_rtsp_frame_failure(
    monkeypatch: pytest.MonkeyPatch,
    tmp_path: Path,
) -> None:
    from Workstation import standalone_gui

    monkeypatch.setattr(standalone_gui.WirisStereoDistancePanel, "_initial_status", lambda self: None)
    monkeypatch.setattr(standalone_gui.WirisStereoDistancePanel, "_auto_load_intrinsics", lambda self: None)
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    panel = standalone_gui.WirisStereoDistancePanel(app_root=tmp_path)
    panel.auto_wiris_connect_pending = True
    enabled: list[tuple[str, ...]] = []
    checks: list[str] = []
    retries: list[str] = []
    monkeypatch.setattr(panel, "enable_wiris_rtsp", lambda *, target_labels: enabled.append(target_labels))
    monkeypatch.setattr(panel, "check_cameras", lambda: checks.append("check"))
    monkeypatch.setattr(panel, "_schedule_auto_wiris_connect_retry", retries.append)

    candidate = {"host": "10.0.0.11", "port": 2250, "ok": True, "response": "OK"}
    panel._apply_auto_wiris_connect_candidates([candidate], [], full_control_scan=False)
    assert enabled == [("left",)]
    assert checks == []

    panel._apply_wiris_rtsp_enable_result(
        {
            "mode": "enable",
            "statuses": ["left=enable-sent"],
            "corrected_ports": {"left": 2240},
            "should_check": False,
        }
    )
    assert retries == ["RTSP 활성화에 성공한 카메라가 없음"]

    panel._apply_auto_wiris_connect_candidates([candidate], [], full_control_scan=False)
    assert enabled == [("left",)]
    assert checks == ["check"]
    panel._shutdown_coordinator = LifecycleCoordinator([])
    panel.close()
    app.processEvents()
    panel.deleteLater()
    app.processEvents()

def test_backend_command_failure_is_not_reported_as_completion() -> None:
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    panel = QtWidgets.QWidget()
    backend = StandalonePanelBackend(panel)
    errors: list[str] = []
    backend.command_error.connect(errors.append)
    with pytest.raises(RuntimeError, match="toggle_camera_connection"):
        backend.toggle_camera()
    assert errors and "unavailable" in errors[0]
    panel.deleteLater()
    backend.deleteLater()
    app.processEvents()

def test_phase_one_backend_facades_delegate_to_standalone_owner() -> None:
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    calls: list[str] = []
    panel = SimpleNamespace()
    method_names = (
        "discover_pc_camera_ip",
        "apply_camera_candidate",
        "check_cameras",
        "check_wiris_control",
        "enable_wiris_rtsp",
        "reactivate_wiris_rtsp",
        "swap_cameras",
        "save_current_settings_from_ui",
        "set_selected_wiris_time_from_pc",
        "refresh_clock_sync_status",
        "toggle_mavlink_connection",
        "apply_pixhawk_candidate",
        "discover_gnss_base_serial_ports",
        "apply_gnss_base_candidate",
        "connect_gnss_base_serial",
        "disconnect_gnss_base_serial",
        "start_gnss_base_survey_in",
        "stop_gnss_base_survey_in",
        "start_gnss_network",
        "stop_gnss_network",
        "capture_calibration_images",
        "run_intrinsic_calibration",
        "save_intrinsics",
        "load_intrinsics",
        "load_current_zoom_intrinsics",
        "apply_intrinsics_to_distance",
        "load_checkerboard_validation_json",
        "run_checkerboard_random_validation",
        "save_checkerboard_validation_result",
        "apply_checkerboard_validation_result",
    )
    for name in method_names:
        setattr(
            panel,
            name,
            lambda *_args, name=name: calls.append(name) or (False if name == "stop_gnss_network" else None),
        )
    backend = StandalonePanelBackend(panel)
    backend.camera_apply_candidate(1, "left")
    backend.pixhawk_apply_candidate("mavlink", 2, "right")
    backend.gnss_base_apply_candidate(2)
    assert calls[-1] == "apply_gnss_base_candidate"

    facades = (
        "camera_discover_host_ip",
        "camera_check_rtsp",
        "camera_check_control",
        "camera_enable_rtsp",
        "camera_reactivate",
        "camera_swap",
        "camera_save_settings",
        "time_set_pc",
        "time_check_sync",
        "pixhawk_toggle_mavlink",
        "gnss_base_discover",
        "gnss_base_connect",
        "gnss_base_disconnect",
        "gnss_survey_start",
        "gnss_survey_stop",
        "gnss_ntrip_start",
        "gnss_ntrip_stop",
        "calibration_capture_images",
        "calibration_run_intrinsic",
        "calibration_save_intrinsics",
        "calibration_load_intrinsics",
        "calibration_load_current_zoom_intrinsics",
        "calibration_apply_intrinsics",
        "calibration_load_validation",
        "calibration_run_validation",
        "calibration_save_validation",
        "calibration_apply_validation",
    )
    for facade in facades:
        if facade == "camera_reactivate":
            getattr(backend, facade)("left")
        else:
            getattr(backend, facade)()

    assert calls == [
        "apply_camera_candidate",
        "apply_pixhawk_candidate",
        "apply_gnss_base_candidate",
        *[
            name
            for name in method_names
            if name not in {"apply_camera_candidate", "apply_pixhawk_candidate", "apply_gnss_base_candidate"}
        ],
    ]
    assert backend.gnss_ntrip_stop() is None
    assert calls[-1] == "stop_gnss_network"
    backend.deleteLater()
    app.processEvents()

def test_advanced_backend_facades_delegate_without_creating_resource_owners() -> None:
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    calls: list[object] = []
    panel = SimpleNamespace()
    panel.workstation_wiris_command_catalog = lambda: ({"command": "HIWS"},)
    panel.workstation_send_wiris_command = lambda target, command, terminator: calls.append(
        ("wiris", target, command, terminator)
    )
    panel.workstation_send_zoom = lambda target, action, level: calls.append(
        ("zoom", target, action, level)
    )
    panel.workstation_discover_custom_mavlink_watch = lambda: calls.append("watch_mavlink")
    panel.workstation_add_custom_mavlink_watch = lambda name, message, field: calls.append(
        ("watch_add", name, message, field)
    )
    panel.workstation_remove_custom_mavlink_watch = lambda key: calls.append(("watch_remove", key))
    panel.workstation_refresh_baseline_field_browser = lambda: calls.append("baseline_scan") or {}
    panel.workstation_open_px4_shell = lambda camera: calls.append(("shell", camera))
    panel.workstation_refresh_px4_mag = lambda camera, read_params=True: calls.append(
        ("mag", camera, read_params)
    )
    panel.workstation_refresh_px4_attitude_cal = lambda camera, read_params=True: calls.append(
        ("attitude", camera, read_params)
    )
    panel.reset_px4_calibration_group = lambda camera, group: calls.append(
        ("reset_calibration", camera, group)
    )
    panel.workstation_apply_px4_mag_priorities = lambda camera, priorities: calls.append(
        ("apply_mag", camera, priorities)
    )
    panel.workstation_save_px4_mag_params = lambda camera: calls.append(("save_mag", camera))
    panel.workstation_reboot_px4 = lambda camera: calls.append(("reboot", camera))
    panel.workstation_refresh_calibration_preview = lambda: calls.append("preview") or {"images": {}}
    panel.workstation_calibration_preview_snapshot = lambda: {"images": {}}
    backend = StandalonePanelBackend(panel)

    assert backend.advanced_wiris_catalog() == ({"command": "HIWS"},)
    backend.advanced_send_wiris_command("left", "HIWS", "\n")
    backend.advanced_send_zoom("both", "set", 10)
    backend.advanced_discover_custom_mavlink_watch()
    backend.advanced_add_custom_mavlink_watch("z", "GPS_RAW_INT", "lat")
    backend.advanced_remove_custom_mavlink_watch("z")
    backend.advanced_refresh_baseline_fields()
    backend.advanced_open_px4_shell("right")
    backend.advanced_refresh_px4_mag("right")
    backend.advanced_refresh_px4_attitude_cal("right")
    backend.pixhawk_reset_calibration("right", "compass")
    backend.advanced_apply_px4_mag_priorities("right", {"CAN1": 100})
    backend.advanced_save_px4_mag_params("right")
    backend.advanced_reboot_px4("right")
    backend.advanced_refresh_calibration_preview()
    backend.advanced_calibration_preview_snapshot()

    assert calls == [
        ("wiris", "left", "HIWS", "\n"),
        ("zoom", "both", "set", 10),
        "watch_mavlink",
        ("watch_add", "z", "GPS_RAW_INT", "lat"),
        ("watch_remove", "z"),
        "baseline_scan",
        ("shell", "right"),
        ("mag", "right", True),
        ("attitude", "right", True),
        ("reset_calibration", "right", "compass"),
        ("apply_mag", "right", {"CAN1": 100}),
        ("save_mag", "right"),
        ("reboot", "right"),
        "preview",
    ]
    backend.deleteLater()
    app.processEvents()

def test_workstation_exposes_advanced_owner_tools_and_preserves_contracts() -> None:
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    backend = _FakeBackend()
    window = WorkstationMainWindow(backend=backend)
    window.show()
    app.processEvents()

    window.navigation.select_context("tasks.advanced")
    app.processEvents()
    assert window.workspace_host.currentWidget().widget() is window.advanced
    assert window.advanced.wiris_catalog_combo.count() == 1
    assert window.advanced.baseline_mavlink_model.rowCount() == 0
    assert not any("Safety contracts" in label.text() for label in window.advanced.findChildren(QtWidgets.QLabel))
    assert "OpenCV RDF" in window.advanced.preview_status_label.text()

    window.advanced.wiris_command_edit.setText("HIWS")
    assert window.commands.invoke("advanced.wiris.send") is True
    assert backend.calls[-1] == ("advanced_wiris", "both", "HIWS", "\n")

    window.advanced.zoom_level_spin.setValue(12)
    assert window.commands.invoke("advanced.zoom.set") is True
    assert backend.calls[-1] == ("advanced_zoom", "both", "set", 12)

    window.advanced.watch_name_edit.setText("local_z")
    window.advanced.watch_message_edit.setText("LOCAL_POSITION_NED")
    window.advanced.watch_field_edit.setText("z")
    assert window.commands.invoke("advanced.watch.add_mavlink") is True
    assert backend.calls[-1] == (
        "advanced_watch_add_mavlink",
        "local_z",
        "LOCAL_POSITION_NED",
        "z",
    )

    assert window.commands.invoke("advanced.baseline.scan") is True
    assert window.advanced.baseline_mavlink_model.rowCount() == 1
    assert "advanced.baseline.describe" not in window.commands._specs

    window.advanced.px4_camera_combo.setCurrentIndex(1)
    assert window.commands.invoke("advanced.px4.refresh_mag") is True
    assert backend.calls[-1] == ("advanced_px4_mag", "right", True)
    assert window.commands.invoke("advanced.px4.refresh_attitude") is True
    assert backend.calls[-1] == ("advanced_px4_attitude", "right", True)
    assert window.commands.invoke("advanced.px4.shell") is True
    assert backend.calls[-1] == ("advanced_px4_shell", "right")

    assert window.commands.invoke("advanced.preview.refresh") is True
    assert "checkerboard pixels" in window.advanced.preview_status_label.text()

    window.close()
    app.processEvents()
    window.deleteLater()
    app.processEvents()
