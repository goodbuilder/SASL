"""Workstation calibration regression tests."""

from __future__ import annotations

from Workstation.tests.workstation_test_support import (
    EvidenceAdapter,
    _FakeBackend,
    json,
    os,
    Path,
    pytest,
    QtCore,
    QtWidgets,
    SimpleNamespace,
    StandalonePanelBackend,
    time,
    WorkstationMainWindow,
)


def test_calibration_setup_settings_round_trip(tmp_path: Path) -> None:
    from Workstation.workstation_core.settings import AppSettings, load_settings, save_settings

    expected = AppSettings(
        calibration_zoom_level=24,
        calibration_inner_cols=9,
        calibration_inner_rows=6,
        calibration_square_size_mm=25.0,
        calibration_min_images=30,
        calibration_calculation_pair_count=18,
        calibration_pair_max_delta_s=0.075,
        calibration_auto_interval_s=2.5,
        calibration_root="captures/board-a",
        calibration_validation_mode="checkerboard",
        calibration_validation_root="validation/board-a",
        calibration_validation_json="profiles/intrinsics.json",
        calibration_validation_runs=5,
        calibration_validation_pair_count=12,
        calibration_validation_seed=1234,
    )
    path = tmp_path / "settings.json"

    save_settings(expected, path)
    loaded = load_settings(path)

    assert loaded.calibration_zoom_level == 24
    assert loaded.calibration_inner_cols == 9
    assert loaded.calibration_inner_rows == 6
    assert loaded.calibration_square_size_mm == pytest.approx(25.0)
    assert loaded.calibration_min_images == 30
    assert loaded.calibration_calculation_pair_count == 18
    assert loaded.calibration_pair_max_delta_s == pytest.approx(0.075)
    assert loaded.calibration_auto_interval_s == pytest.approx(2.5)
    assert loaded.calibration_root == "captures/board-a"
    assert loaded.calibration_validation_mode == "checkerboard"
    assert loaded.calibration_validation_root == "validation/board-a"
    assert loaded.calibration_validation_json == "profiles/intrinsics.json"
    assert loaded.calibration_validation_runs == 5
    assert loaded.calibration_validation_pair_count == 12
    assert loaded.calibration_validation_seed == 1234


def test_calibration_auto_interval_load_floor_is_100ms(tmp_path: Path) -> None:
    from Workstation.workstation_core.settings import load_settings

    path = tmp_path / "settings.json"
    path.write_text(
        json.dumps({"calibration_auto_interval_s": 0.01}),
        encoding="utf-8",
    )

    loaded = load_settings(path)

    assert loaded.calibration_auto_interval_s == pytest.approx(0.1)

def test_pixhawk_calibration_group_reset_is_scoped_and_uses_px4_defaults(monkeypatch: pytest.MonkeyPatch) -> None:
    from Workstation.device_runtime_controller import DeviceRuntimeController

    shell_calls: list[tuple[str, str, dict[str, object]]] = []
    panel = SimpleNamespace(
        mavlink_pixhawk=object(),
        _mavlink_open_sides=lambda: ("left",),
        px4_mag_managers={"left": SimpleNamespace(armed=False)},
        _px4_calibration_reset_pending={},
        _px4_mag_camera_label=lambda _side: "P1",
        _log=lambda _message: None,
        _start_pixhawk_shell_worker=lambda camera, command, **kwargs: shell_calls.append(
            (camera, command, kwargs)
        ) or True,
    )
    monkeypatch.setattr(
        QtWidgets.QMessageBox,
        "question",
        lambda *_args, **_kwargs: QtWidgets.QMessageBox.Yes,
    )

    assert DeviceRuntimeController(panel).reset_px4_calibration_group("left", "accelerometer") is True
    assert shell_calls[0][0] == "left"
    assert shell_calls[0][1].startswith("param reset CAL_ACC0_XOFF")
    # Only the calibration-fit offset/scale terms reset (raw sensor
    # passthrough): _ID/_PRIO/_ROT stay untouched, like level leaves
    # SENS_BOARD_ROT alone, so the slot stays bound to its device and
    # armable without forcing a QGC re-run.
    assert "CAL_ACC0_ID" not in shell_calls[0][1]
    assert "CAL_ACC0_ROT" not in shell_calls[0][1]
    assert "CAL_GYRO0_ID" not in shell_calls[0][1]
    assert "CAL_MAG0_ID" not in shell_calls[0][1]
    assert "SENS_BOARD_X_OFF" not in shell_calls[0][1]
    assert shell_calls[0][2]["operation"] == "reset_calibration:accelerometer"
    assert panel._px4_calibration_reset_pending["left"] == "accelerometer"

def test_pixhawk_calibration_reset_commands_route_to_the_selected_group() -> None:
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    backend = _FakeBackend()
    backend.panel._mavlink_open_sides = lambda: ("left",)
    backend.panel.px4_mag_managers = {"left": SimpleNamespace(armed=False)}
    window = WorkstationMainWindow(backend=backend)
    window.show()
    app.processEvents()

    command_id = "pixhawk.reset_calibration.p1.compass"
    assert window.commands.action(command_id).text() == "Reset P1 Compass / magnetometer calibration"
    assert window.commands.invoke(command_id) is True
    assert backend.calls[-1] == ("pixhawk_reset_calibration", "left", "compass")
    assert "pixhawk.reset_calibration.p1.board" not in window.commands._actions

    window.close()
    app.processEvents()
    window.deleteLater()
    app.processEvents()

def test_calibration_workspace_separates_live_and_ftp_previews_with_actions_below() -> None:
    from PyQt5 import QtGui

    from Workstation.status_model import StatusModel
    from Workstation.workspaces import CalibrationWorkspace, CameraPreviewPair
    from Workstation.workstation_core.stereo import DetectionObservation

    class FakePane(QtCore.QObject):
        frame_changed = QtCore.pyqtSignal(str, QtGui.QImage, object)
        detection_changed = QtCore.pyqtSignal(str, object)
        stats_changed = QtCore.pyqtSignal(str, object)

    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    left_pane = FakePane()
    right_pane = FakePane()
    backend = SimpleNamespace(
        panel=SimpleNamespace(left_pane=left_pane, right_pane=right_pane)
    )
    workspace = CalibrationWorkspace(
        backend,
        lambda: None,
        lambda _command_id: True,
    )
    workspace.resize(1000, 700)
    workspace.show()
    app.processEvents()

    assert set(workspace.previews) == {"left", "right"}
    assert isinstance(workspace.preview_splitter, CameraPreviewPair)
    assert workspace.preview_splitter.hasHeightForWidth()
    assert (
        workspace.preview_splitter.sizePolicy().verticalPolicy()
        == QtWidgets.QSizePolicy.Fixed
    )
    assert all(not preview.crop_to_fill for preview in workspace.previews.values())
    assert all(preview.lock_chrome_height for preview in workspace.previews.values())
    assert all(preview.lock_image_aspect for preview in workspace.previews.values())
    preview_pair_size = workspace.preview_splitter.size()
    expected_pair_height = workspace.preview_splitter.heightForWidth(preview_pair_size.width())
    assert abs(preview_pair_size.height() - expected_pair_height) <= 2
    assert workspace.preview_splitter.minimumHeight() == expected_pair_height
    app.processEvents()
    app.processEvents()
    assert abs(
        workspace.preview_splitter.sizes()[0]
        - workspace.preview_splitter.sizes()[1]
    ) <= 1
    assert workspace.preview_splitter.size() == preview_pair_size
    assert workspace.preview_splitter.parentWidget() is workspace.camera_mode_tabs.widget(0)
    assert workspace.ftp_preview_splitter.parentWidget() is workspace.camera_mode_tabs.widget(1)
    workspace.camera_mode_tabs.setCurrentIndex(1)
    app.processEvents()
    assert abs(
        workspace.ftp_preview_splitter.sizes()[0]
        - workspace.ftp_preview_splitter.sizes()[1]
    ) <= 1
    workspace.camera_mode_tabs.setCurrentIndex(0)
    app.processEvents()
    assert workspace._ftp_preview_status_text == ""
    assert not workspace.camera_mode_tabs.widget(1).findChildren(
        QtWidgets.QLabel,
        "statusStrip",
    )
    camera_container = workspace.camera_mode_tabs.parentWidget()
    assert workspace.operation_tabs.parentWidget() is camera_container
    assert camera_container is not None
    camera_layout = camera_container.layout()
    assert camera_layout is not None
    assert (
        workspace.operation_tabs.geometry().top()
        - camera_layout.contentsRect().top()
    ) <= 32
    assert camera_layout.indexOf(workspace.operation_tabs) < camera_layout.indexOf(
        workspace.camera_mode_tabs
    )
    assert camera_layout.indexOf(workspace.capture_surface) == (
        camera_layout.indexOf(workspace.camera_mode_tabs) + 1
    )
    capture_gap = (
        workspace.capture_surface.geometry().top()
        - workspace.camera_mode_tabs.geometry().bottom()
        - 1
    )
    assert 0 <= capture_gap <= 12
    assert [
        workspace.operation_tabs.tabText(index)
        for index in range(workspace.operation_tabs.count())
    ] == ["Intrinsics", "Extrinsics"]
    assert (
        workspace.intrinsics_operation_tabs.geometry().top()
        - workspace.intrinsics_contract_label.geometry().bottom()
        - 1
    ) <= 48
    for workflow_tabs in (
        workspace.intrinsics_operation_tabs,
        workspace.extrinsics_operation_tabs,
    ):
        assert [
            workflow_tabs.tabText(index)
            for index in range(workflow_tabs.count())
        ] == ["Capture", "Compute", "Profiles"]
    intrinsic_compute_ids = {
        str(button.property("commandId") or "")
        for button in workspace.intrinsics_operation_tabs.widget(1).findChildren(
            QtWidgets.QPushButton
        )
    }
    extrinsic_compute_ids = {
        str(button.property("commandId") or "")
        for button in workspace.extrinsics_operation_tabs.widget(1).findChildren(
            QtWidgets.QPushButton
        )
    }
    extrinsic_profile_ids = {
        str(button.property("commandId") or "")
        for button in workspace.extrinsics_operation_tabs.widget(2).findChildren(
            QtWidgets.QPushButton
        )
    }
    assert "calibration.run_intrinsic" in intrinsic_compute_ids
    assert "calibration.run_extrinsics" not in intrinsic_compute_ids
    assert "calibration.run_extrinsics" in extrinsic_compute_ids
    assert "calibration.run_intrinsic" not in extrinsic_compute_ids
    assert "calibration.apply_bundle" in extrinsic_profile_ids
    assert "X_right = R_left_to_right X_left" in (
        workspace.extrinsics_bundle_contract_label.text()
    )
    assert set(workspace.camera_buttons) == {"camera.toggle"}
    assert workspace.zoom_set_button.text() == "Set both"
    assert workspace.zoom_in_button.text() == "Zoom-in"
    assert workspace.zoom_out_button.text() == "Zoom-out"
    assert workspace.zoom_level_spin.minimum() == 0
    workspace.zoom_level_spin.setValue(0)
    assert workspace.zoom_level_spin.value() == 0
    assert workspace.zoom_level_spin.toolTip().startswith("WIRIS zoom index")
    assert not hasattr(workspace, "zoom_8x_button")
    assert "calibration.capture_images" in workspace.operation_buttons
    assert (
        workspace.operation_buttons[
            "calibration.capture_images_wiris_oz_capture_only"
        ].text()
        == "OZ 촬영"
    )
    assert (
        workspace.operation_buttons[
            "calibration.sync_images_wiris_oz_ftp"
        ].text()
        == "FTP 자동 전송"
    )
    assert workspace.capture_primary_button.isVisible()
    assert workspace.capture_primary_button.text() == "Capture Live Pair"
    assert workspace.operation_tabs.height() <= 260
    assert workspace.auto_capture_settings_content.isHidden()
    assert workspace.capture_response_content.isHidden()
    for workflow_tabs in (
        workspace.intrinsics_operation_tabs,
        workspace.extrinsics_operation_tabs,
    ):
        assert not workflow_tabs.widget(0).findChildren(QtWidgets.QPushButton)

    preview_top = workspace.camera_mode_tabs.geometry().top()
    capture_top = workspace.capture_surface.geometry().top()
    workspace.operation_tabs.setCurrentIndex(1)
    app.processEvents()
    assert workspace.operation_tabs.height() <= 260
    assert abs(workspace.camera_mode_tabs.geometry().top() - preview_top) <= 1
    assert abs(workspace.capture_surface.geometry().top() - capture_top) <= 1
    assert workspace.capture_primary_button.text() == "Capture Stereo Pair"
    workspace.operation_tabs.setCurrentIndex(0)
    app.processEvents()

    workspace.intrinsics_operation_tabs.setCurrentIndex(1)
    app.processEvents()
    assert workspace.extrinsics_operation_tabs.currentIndex() == 1
    assert workspace.camera_mode_tabs.isHidden()
    assert workspace.capture_surface.isHidden()
    workspace.intrinsics_operation_tabs.setCurrentIndex(0)
    assert workspace.extrinsics_operation_tabs.currentIndex() == 0
    workspace.operation_tabs.setCurrentIndex(1)
    app.processEvents()
    assert not workspace.camera_mode_tabs.isHidden()
    assert not workspace.capture_surface.isHidden()
    assert workspace.capture_primary_button.text() == "Capture Stereo Pair"
    workspace.operation_tabs.setCurrentIndex(0)
    assert set(workspace.calibration_capture_response_edits) == {"left", "right"}
    assert workspace.calibration_left_files_button.text().startswith("Left")
    assert workspace.calibration_right_files_button.text().startswith("Right")

    image = QtGui.QImage(640, 360, QtGui.QImage.Format_RGB888)
    image.fill(QtGui.QColor("black"))
    preview = workspace.previews["left"]
    left_pane.frame_changed.emit("left", image, {"timestamp_s": 1.0})
    right_pane.frame_changed.emit("right", image, {"timestamp_s": 1.0})
    app.processEvents()

    stable_pair_size = workspace.preview_splitter.size()
    stable_image_size = preview.image.size()
    for frame_index in range(20):
        left_pane.frame_changed.emit("left", image, {"timestamp_s": frame_index + 2.0})
        right_pane.frame_changed.emit("right", image, {"timestamp_s": frame_index + 2.0})
        app.processEvents()
    assert workspace.preview_splitter.size() == stable_pair_size
    assert preview.image.size() == stable_image_size
    assert abs(
        stable_pair_size.height()
        - workspace.preview_splitter.heightForWidth(stable_pair_size.width())
    ) <= 2
    assert preview.image.height() > 0
    assert preview.image.width() / preview.image.height() == pytest.approx(
        16.0 / 9.0,
        abs=0.02,
    )
    assert workspace.previews["left"].image.size() == workspace.previews["right"].image.size()

    workspace.preview_splitter.moveSplitter(300, 1)
    app.processEvents()
    assert abs(
        workspace.preview_splitter.sizes()[0]
        - workspace.preview_splitter.sizes()[1]
    ) <= 1

    assert workspace.previews["left"].meta.text().startswith("LIVE | Class: --")
    assert workspace.previews["right"].meta.text().startswith("LIVE | Class: --")
    assert preview.image.pixmap().size() == preview.image.size()
    without_box = preview.image.pixmap().toImage().copy()

    detection = DetectionObservation(
        camera="left",
        timestamp_s=1.0,
        width=640,
        height=360,
        x1=100.0,
        y1=80.0,
        x2=300.0,
        y2=260.0,
        confidence=0.9,
        class_id=0,
        class_name="person",
    )
    left_pane.detection_changed.emit("left", detection)
    app.processEvents()
    assert workspace.previews["left"]._detection is detection
    assert preview.image.pixmap().toImage().copy() != without_box
    assert "Class: person" in preview.meta.text()
    assert "receive=" not in preview.meta.text()
    assert "camera_ts=" not in preview.meta.text()

    marker_pixmap = QtGui.QPixmap(640, 360)
    marker_pixmap.fill(QtGui.QColor("black"))
    marker_pixmap = preview._draw_detection_overlay(marker_pixmap, detection)
    center_x, center_y = (round(value) for value in detection.center)
    center_ring_color = marker_pixmap.toImage().pixelColor(center_x + 4, center_y)
    assert center_ring_color.red() > center_ring_color.green()
    assert center_ring_color.red() > center_ring_color.blue()

    workspace.deleteLater()
    left_pane.deleteLater()
    right_pane.deleteLater()
    app.processEvents()

def test_calibration_workspace_shows_zoom_index_and_actual_ratio() -> None:
    from Workstation.workspaces import CalibrationWorkspace

    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    backend = _FakeBackend()
    backend.panel.actual_visible_zoom = {
        "left": {"index": 14, "ratio": 30.0},
        "right": {"index": 14, "ratio": 30.0},
    }
    workspace = CalibrationWorkspace(backend, lambda: None, lambda _command_id: True)

    workspace.refresh({})

    assert "actual=left=14 (30×), right=14 (30×)" in workspace.zoom_readback_label.text()
    workspace.deleteLater()
    app.processEvents()


def test_calibration_actual_zoom_buttons_send_both_and_show_pending_readback() -> None:
    from Workstation.workspaces import CalibrationWorkspace

    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    backend = _FakeBackend()
    workspace = CalibrationWorkspace(backend, lambda: None, lambda _command_id: True)

    backend.calls.clear()
    workspace.zoom_in_button.click()
    assert backend.calls == [("advanced_zoom", "both", "in", None)]
    assert workspace.zoom_readback_label.text() == "Requested Zoom-in for both; GZVV readback pending"

    backend.calls.clear()
    workspace.zoom_out_button.click()
    assert backend.calls == [("advanced_zoom", "both", "out", None)]
    assert workspace.zoom_readback_label.text() == "Requested Zoom-out for both; GZVV readback pending"

    workspace.deleteLater()
    app.processEvents()

def test_calibration_fit_preview_keeps_both_source_edges_and_fixed_metadata_height() -> None:
    from PyQt5 import QtGui

    from Workstation.workspaces import CameraPreview

    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    preview = CameraPreview("left", crop_to_fill=False, lock_chrome_height=True)
    preview.image.resize(240, 240)
    source = QtGui.QImage(160, 90, QtGui.QImage.Format_RGB888)
    source.fill(QtGui.QColor("black"))
    for y in range(source.height()):
        source.setPixelColor(0, y, QtGui.QColor("red"))
        source.setPixelColor(source.width() - 1, y, QtGui.QColor("blue"))

    preview.set_frame(source, {})
    preview._render()

    rendered = preview.image.pixmap().toImage()
    assert any(rendered.pixelColor(0, y).red() > 200 for y in range(rendered.height()))
    assert any(rendered.pixelColor(rendered.width() - 1, y).blue() > 200 for y in range(rendered.height()))
    metadata_height = preview.meta.height()
    preview.meta.setText("FTP STILL | " + "very-long-metadata-" * 20)
    app.processEvents()
    assert preview.meta.height() == metadata_height
    preview.deleteLater()
    app.processEvents()

def test_calibration_workspace_routes_capture_events_to_the_matching_camera_log() -> None:
    from PyQt5 import QtGui

    from Workstation.workspaces import CalibrationWorkspace

    class FakePane(QtCore.QObject):
        frame_changed = QtCore.pyqtSignal(str, QtGui.QImage, object)
        stats_changed = QtCore.pyqtSignal(str, object)

    class FakePanel(QtCore.QObject):
        calibration_capture_event = QtCore.pyqtSignal(object)
        calibration_diagnostics_changed = QtCore.pyqtSignal(object)

        def __init__(self) -> None:
            super().__init__()
            self.left_pane = FakePane()
            self.right_pane = FakePane()

    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    panel = FakePanel()
    workspace = CalibrationWorkspace(SimpleNamespace(panel=panel), lambda: None)
    panel.calibration_capture_event.emit(
        {
            "side": "left",
            "operation": "Capture FTP HR",
            "capture_id": "capture-1",
            "phase": "response",
            "command": "CPTR",
            "response": "OK",
            "state": "received",
        }
    )
    app.processEvents()
    assert ">" not in workspace.calibration_capture_response_edits["right"].toPlainText()
    assert "CPTR" in workspace.calibration_capture_response_edits["left"].toPlainText()
    assert "OK" in workspace.calibration_capture_response_edits["left"].toPlainText()

    panel.calibration_image_diagnostics = (
        {
            "side": "left",
            "path": "/tmp/left_001.png",
            "capture_key": "001",
            "reprojection_error_px": 0.5,
        },
    )
    panel.calibration_diagnostics_changed.emit(panel.calibration_image_diagnostics)
    app.processEvents()
    assert workspace.calibration_outlier_table.rowCount() == 1

    workspace.deleteLater()
    panel.deleteLater()
    app.processEvents()

def test_calibration_ftp_pair_preview_returns_to_live_unless_user_holds_it(
    tmp_path: Path,
) -> None:
    from PyQt5 import QtGui

    from Workstation.workspaces import CalibrationWorkspace

    class FakePane(QtCore.QObject):
        frame_changed = QtCore.pyqtSignal(str, QtGui.QImage, object)
        stats_changed = QtCore.pyqtSignal(str, object)

    class FakePanel(QtCore.QObject):
        calibration_capture_event = QtCore.pyqtSignal(object)

        def __init__(self) -> None:
            super().__init__()
            self.left_pane = FakePane()
            self.right_pane = FakePane()

    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    paths: dict[str, str] = {}
    colors = {"left": QtGui.QColor("red"), "right": QtGui.QColor("blue")}
    for side, color in colors.items():
        image = QtGui.QImage(32, 24, QtGui.QImage.Format_RGB888)
        image.fill(color)
        path = tmp_path / f"{side}.png"
        assert image.save(str(path))
        paths[side] = str(path)

    panel = FakePanel()
    workspace = CalibrationWorkspace(SimpleNamespace(panel=panel), lambda: None)
    assert workspace.camera_mode_tabs.tabText(0) == "Live"
    assert workspace.camera_mode_tabs.tabText(1) == "FTP"
    assert workspace.camera_mode_tabs.currentIndex() == 0
    assert workspace._ftp_preview_timer.interval() == 1000

    panel.calibration_capture_event.emit(
        {
            "phase": "ftp_pair_ready",
            "state": "ready",
            "operation": "Capture HR Still (FTP)",
            "capture_id": "ftp-001",
            "paths": paths,
        }
    )
    app.processEvents()

    assert workspace.camera_mode_tabs.currentIndex() == 1
    assert workspace._ftp_preview_timer.isActive()
    assert "ftp-001" in workspace._ftp_preview_status_text
    assert workspace.ftp_previews["left"]._image.pixelColor(0, 0) == colors["left"]
    assert workspace.ftp_previews["right"]._image.pixelColor(0, 0) == colors["right"]
    assert workspace.ftp_previews["left"].meta.text().startswith("FTP STILL")
    assert "left.png" in workspace.ftp_previews["left"].meta.text()
    assert "32×24" in workspace.ftp_previews["left"].meta.text()

    workspace._return_to_live_after_ftp_preview()
    assert workspace.camera_mode_tabs.currentIndex() == 0
    assert workspace.ftp_previews["left"]._image.pixelColor(0, 0) == colors["left"]

    panel.calibration_capture_event.emit(
        {
            "phase": "ftp_pair_ready",
            "state": "ready",
            "operation": "Capture OZ Still (FTP)",
            "capture_id": "ftp-002",
            "paths": paths,
        }
    )
    app.processEvents()
    workspace.camera_mode_tabs.tabBar().tabBarClicked.emit(1)
    assert not workspace._ftp_preview_timer.isActive()
    workspace._return_to_live_after_ftp_preview()
    assert workspace.camera_mode_tabs.currentIndex() == 1

    panel.calibration_capture_event.emit(
        {
            "phase": "ftp_pair_ready",
            "state": "ready",
            "operation": "Capture HR Still (FTP)",
            "capture_id": "ftp-003",
            "paths": paths,
        }
    )
    app.processEvents()
    assert workspace.camera_mode_tabs.currentIndex() == 1
    assert not workspace._ftp_preview_timer.isActive()
    workspace._return_to_live_after_ftp_preview()
    assert workspace.camera_mode_tabs.currentIndex() == 1

    workspace.camera_mode_tabs.setCurrentIndex(0)
    previous_left = workspace.ftp_previews["left"]._image.copy()
    panel.calibration_capture_event.emit(
        {
            "phase": "ftp_pair_ready",
            "state": "ready",
            "operation": "Capture HR Still (FTP)",
            "capture_id": "ftp-bad",
            "paths": {"left": paths["left"], "right": str(tmp_path / "missing.png")},
        }
    )
    app.processEvents()
    assert workspace.camera_mode_tabs.currentIndex() == 0
    assert workspace.ftp_previews["left"]._image == previous_left
    assert "failed" in workspace._ftp_preview_status_text.lower()

    workspace.deleteLater()
    panel.left_pane.deleteLater()
    panel.right_pane.deleteLater()
    panel.deleteLater()
    app.processEvents()

def test_calibration_controller_publishes_one_atomic_ftp_pair_ready_event() -> None:
    from PyQt5 import QtGui

    from Workstation.calibration_controller import CalibrationWorkflowController

    class FakePanel(QtCore.QObject):
        calibration_capture_event = QtCore.pyqtSignal(object)

    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    panel = FakePanel()
    events: list[dict[str, object]] = []
    panel.calibration_capture_event.connect(events.append)
    controller = CalibrationWorkflowController(panel)
    frames = {
        "left": QtGui.QImage(32, 24, QtGui.QImage.Format_RGB888),
        "right": QtGui.QImage(64, 48, QtGui.QImage.Format_RGB888),
    }
    cameras = {
        "left": {"ftp_local_path": "/tmp/left-ftp.png"},
        "right": {"ftp_local_path": "/tmp/right-ftp.png"},
    }

    controller._publish_calibration_ftp_pair_ready(
        operation="Capture HR Still (FTP)",
        capture_id="ftp-atomic-1",
        cameras=cameras,
        frames=frames,
    )
    app.processEvents()

    assert len(events) == 1
    assert events[0]["phase"] == "ftp_pair_ready"
    assert events[0]["capture_id"] == "ftp-atomic-1"
    assert events[0]["paths"] == {
        "left": "/tmp/left-ftp.png",
        "right": "/tmp/right-ftp.png",
    }
    assert events[0]["dimensions"] == {"left": [32, 24], "right": [64, 48]}
    assert all(not isinstance(value, QtGui.QImage) for value in events[0].values())

    panel.deleteLater()
    app.processEvents()

def test_calibration_error_plot_context_menu_emits_image_exclusion(monkeypatch) -> None:
    from PyQt5 import QtGui

    from Workstation.workspaces import CalibrationErrorPlot

    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    plot = CalibrationErrorPlot()
    plot.resize(640, 260)
    plot.set_diagnostics(
        (
            {
                "side": "left",
                "path": "/tmp/left_001.png",
                "capture_key": "001",
                "reprojection_error_px": 0.4,
            },
            {
                "side": "right",
                "path": "/tmp/right_001.png",
                "capture_key": "001",
                "reprojection_error_px": 2.2,
            },
        )
    )
    plot.show()
    app.processEvents()
    assert len(plot._point_positions) == 2
    point, _row = plot._point_positions[1]
    emitted: list[dict[str, object]] = []
    plot.exclude_requested.connect(emitted.append)

    def choose_intrinsic_action(menu, _global_pos):
        return next(
            action
            for action in menu.actions()
            if "Intrinsic" in action.text()
        )

    monkeypatch.setattr(QtWidgets.QMenu, "exec_", choose_intrinsic_action)
    event = QtGui.QContextMenuEvent(
        QtGui.QContextMenuEvent.Mouse,
        QtCore.QPoint(round(point.x()), round(point.y())),
        QtCore.QPoint(round(point.x()), round(point.y())),
    )
    plot.contextMenuEvent(event)
    assert emitted and emitted[0]["scope"] == "image"
    assert emitted[0]["side"] == "right"
    assert emitted[0]["path"] == "/tmp/right_001.png"

    plot.deleteLater()
    app.processEvents()

def test_calibration_image_diagnostic_keeps_unreadable_inputs_as_explicit_failures(tmp_path: Path) -> None:
    from Workstation.workstation_core.calibration import (
        CameraIntrinsics,
        CheckerboardConfig,
        analyze_calibration_image,
    )

    diagnostic = analyze_calibration_image(
        tmp_path / "left_001.png",
        CheckerboardConfig(inner_cols=8, inner_rows=6, square_size_m=0.03),
        CameraIntrinsics(
            width=1280,
            height=720,
            camera_matrix=[[900.0, 0.0, 640.0], [0.0, 901.0, 360.0], [0.0, 0.0, 1.0]],
            distortion_coefficients=[0.0],
            reprojection_error_px=0.0,
            image_count=0,
        ),
        side="left",
    )
    assert diagnostic.found is False
    assert diagnostic.reprojection_error_px is None
    assert diagnostic.error

def test_calibration_outlier_pair_exclusion_is_persisted_without_deleting_images(
    tmp_path: Path,
    monkeypatch,
) -> None:
    from PyQt5 import QtGui

    import Workstation.workspaces as workspaces_module
    from Workstation.workspaces import CalibrationWorkspace

    root = tmp_path / "session"
    (root / "left").mkdir(parents=True)
    (root / "right").mkdir(parents=True)
    image = QtGui.QImage(32, 24, QtGui.QImage.Format_RGB888)
    image.fill(QtGui.QColor("black"))
    left = root / "left" / "left_001.png"
    right = root / "right" / "right_001.png"
    assert image.save(str(left))
    assert image.save(str(right))
    evidence = SimpleNamespace(calibration_setup=lambda: {"calibration_root_resolved": str(root)})
    monkeypatch.setattr(workspaces_module, "_evidence", lambda _backend: evidence)

    class FakePane(QtCore.QObject):
        frame_changed = QtCore.pyqtSignal(str, QtGui.QImage, object)
        stats_changed = QtCore.pyqtSignal(str, object)

    panel = SimpleNamespace(left_pane=FakePane(), right_pane=FakePane())
    workspace = CalibrationWorkspace(SimpleNamespace(panel=panel), lambda: None)
    panel.calibration_image_diagnostics = (
        {
            "side": "left",
            "path": str(left),
            "capture_key": "001",
            "reprojection_error_px": 2.0,
        },
        {
            "side": "right",
            "path": str(right),
            "capture_key": "001",
            "reprojection_error_px": 2.1,
        },
    )
    workspace._handle_calibration_outlier_action(
        {"scope": "pair", "side": "left", "path": str(left), "capture_key": "001"},
        restore=False,
    )

    saved = json.loads((root / "calibration_pair_selection.json").read_text(encoding="utf-8"))
    assert saved["schema"] == "workstation-calibration-file-selection-v2"
    assert saved["root"] == "."
    assert saved["excluded_pair_keys"] == ["001"]
    assert saved["excluded_left_paths"] == ["left/left_001.png"]
    assert saved["excluded_right_paths"] == ["right/right_001.png"]
    assert left.is_file() and right.is_file()

    workspace.deleteLater()
    panel.left_pane.deleteLater()
    panel.right_pane.deleteLater()
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    app.processEvents()

def test_calibration_workspace_keeps_checkerboard_overlay_over_live_frame() -> None:
    from PyQt5 import QtGui

    from Workstation.workspaces import CalibrationWorkspace

    class FakePane(QtCore.QObject):
        frame_changed = QtCore.pyqtSignal(str, QtGui.QImage, object)
        stats_changed = QtCore.pyqtSignal(str, object)

    class FakePanel(QtCore.QObject):
        calibration_preview_changed = QtCore.pyqtSignal(object)

        def __init__(self) -> None:
            super().__init__()
            self.left_pane = FakePane()
            self.right_pane = FakePane()

    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    panel = FakePanel()
    workspace = CalibrationWorkspace(SimpleNamespace(panel=panel), lambda: None)
    workspace.show()
    app.processEvents()

    raw = QtGui.QImage(640, 360, QtGui.QImage.Format_RGB888)
    raw.fill(QtGui.QColor("black"))
    panel.left_pane.frame_changed.emit("left", raw, {"timestamp_s": 1.0})
    app.processEvents()

    overlay = QtGui.QImage(640, 360, QtGui.QImage.Format_RGB888)
    overlay.fill(QtGui.QColor("green"))
    panel.calibration_preview_changed.emit(
        {
            "camera": "left",
            "image": overlay,
            "kind": "overlay",
            "metadata": {"timestamp_s": 1.0},
        }
    )
    app.processEvents()

    assert workspace._checkerboard_overlay_active["left"] is True
    assert workspace.previews["left"]._image is not None
    assert workspace.previews["left"]._image.pixelColor(0, 0) == QtGui.QColor("green")

    raw.fill(QtGui.QColor("black"))
    panel.left_pane.frame_changed.emit("left", raw, {"timestamp_s": 2.0})
    app.processEvents()
    assert workspace.previews["left"]._image.pixelColor(0, 0) == QtGui.QColor("green")

    workspace.deleteLater()
    panel.left_pane.deleteLater()
    panel.right_pane.deleteLater()
    panel.deleteLater()
    app.processEvents()

def test_wiris_command_and_zoom_workers_keep_tcp_io_off_ui_path(monkeypatch) -> None:
    from Workstation import standalone_gui
    from Workstation import wiris_runtime_workers

    sent: list[str] = []

    def fake_send(command) -> str:
        sent.append(str(command.command))
        return "OK"

    monkeypatch.setattr(wiris_runtime_workers, "send_tcp_line", fake_send)
    command_results: list[object] = []
    command_worker = standalone_gui.WirisCommandWorker(
        (("left", "10.0.0.11", 5000, 1.0),),
        command_text="HIWS",
        terminator="\n",
    )
    command_worker.completed.connect(command_results.append)
    command_worker.run()

    zoom_results: list[object] = []
    zoom_worker = standalone_gui.WirisZoomWorker(
        (("right", "10.0.0.12", 5001),),
        template="SZIN",
        level=None,
        timeout_s=1.0,
    )
    zoom_worker.completed.connect(zoom_results.append)
    zoom_worker.run()

    assert sent == ["HIWS", "SZIN"]
    assert command_results[0]["statuses"] == {"left": "OK"}
    assert zoom_results == [[{"label": "right", "state": "sent", "command": "SZIN", "response": "OK"}]]


def test_zoom_in_result_schedules_actual_zoom_readback() -> None:
    from Workstation.wiris_control_controller import WirisControlController

    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    panel = SimpleNamespace(
        actual_visible_zoom={"left": {}, "right": {}},
        zoom_status_label=QtWidgets.QLabel(),
        _camera_name_ko=lambda label: label,
        _log=lambda _text: None,
    )
    controller = WirisControlController(panel)

    controller._apply_wiris_zoom_result(
        "both",
        "in",
        [
            {"label": "left", "state": "sent", "command": "SZIN", "response": "OK"},
            {"label": "right", "state": "sent", "command": "SZIN", "response": "OK"},
        ],
    )

    assert panel._zoom_readback_target == "both"
    assert "left: SZIN 응답=OK" in panel.zoom_status_label.text()
    assert "right: SZIN 응답=OK" in panel.zoom_status_label.text()
    panel.zoom_status_label.deleteLater()
    app.processEvents()

def test_checkerboard_pair_timing_allows_100ms_but_rejects_larger() -> None:
    from Workstation.calibration_controller import (
        CHECKERBOARD_STEREO_PAIR_MAX_DELTA_S,
        CalibrationWorkflowController,
    )

    panel = SimpleNamespace(
        camera_frame_metadata={
            "left": {"rtcp_frame_ntp_unix_s": 0.0},
            "right": {"rtcp_frame_ntp_unix_s": CHECKERBOARD_STEREO_PAIR_MAX_DELTA_S},
        },
        _camera_rtc_timestamp_s_from_metadata=lambda metadata: metadata.get("rtcp_frame_ntp_unix_s"),
    )
    controller = CalibrationWorkflowController(panel)

    pair_ok, basis, delta_s, detail = controller._checkerboard_capture_pair_timing()

    assert CHECKERBOARD_STEREO_PAIR_MAX_DELTA_S == pytest.approx(0.100)
    assert pair_ok is True
    assert basis == "rtcp-ntp"
    assert delta_s == pytest.approx(0.100)
    assert detail == "ok"

    panel.camera_frame_metadata["right"] = {
        "rtcp_frame_ntp_unix_s": CHECKERBOARD_STEREO_PAIR_MAX_DELTA_S + 0.000001
    }
    pair_ok, _basis, delta_s, detail = controller._checkerboard_capture_pair_timing()

    assert pair_ok is False
    assert delta_s == pytest.approx(0.100001)
    assert "100.0 ms" in detail


def test_checkerboard_pair_timing_uses_operator_configured_limit() -> None:
    from Workstation.calibration_controller import CalibrationWorkflowController

    panel = SimpleNamespace(
        camera_frame_metadata={
            "left": {"rtcp_frame_ntp_unix_s": 10.000},
            "right": {"rtcp_frame_ntp_unix_s": 10.040},
        },
        calibration_pair_max_delta_spin=SimpleNamespace(value=lambda: 40.0),
        _camera_rtc_timestamp_s_from_metadata=lambda metadata: metadata.get(
            "rtcp_frame_ntp_unix_s"
        ),
    )
    controller = CalibrationWorkflowController(panel)

    pair_ok, basis, delta_s, detail = controller._checkerboard_capture_pair_timing()

    assert pair_ok is True
    assert basis == "rtcp-ntp"
    assert delta_s == pytest.approx(0.040)
    assert detail == "ok"

    panel.camera_frame_metadata["right"]["rtcp_frame_ntp_unix_s"] = 10.040001
    pair_ok, _basis, delta_s, detail = controller._checkerboard_capture_pair_timing()

    assert pair_ok is False
    assert delta_s == pytest.approx(0.040001)
    assert "40.0 ms" in detail

def test_checkerboard_capture_worker_saves_and_validates_optical_pixel_images(
    monkeypatch: pytest.MonkeyPatch,
    tmp_path: Path,
) -> None:
    from Workstation import standalone_gui
    from Workstation import calibration_workers
    from Workstation.workstation_core.calibration import CheckerboardConfig

    monkeypatch.setattr(
            calibration_workers,
        "detect_checkerboard",
        lambda _path, _config: SimpleNamespace(found=True, corner_count=48, error=""),
    )
    frames = {}
    for side in ("left", "right"):
        image = standalone_gui.QtGui.QImage(32, 24, standalone_gui.QtGui.QImage.Format_RGB888)
        image.fill(127)
        frames[side] = image
    worker = standalone_gui.CheckerboardCaptureWorker(
        root=tmp_path,
        frames=frames,
        stem="001",
        config=CheckerboardConfig(inner_cols=8, inner_rows=6, square_size_m=0.03),
        opencv_error=None,
        pair_timing_ok=True,
        pair_time_basis="rtcp-ntp",
        pair_delta_s=0.002,
        pair_timing_detail="ok",
        metadata_payload={
            "frame_contract": "OpenCV image pixels; stereo timing uses RTCP/NTP camera timestamp",
            "pair_max_delta_s": 0.040,
        },
    )
    completed: list[object] = []
    worker.completed.connect(completed.append)

    worker.run()

    assert len(completed) == 1
    assert completed[0]["saved_sides"] == ("left", "right")
    assert (tmp_path / "left" / "left_001.png").is_file()
    assert (tmp_path / "right" / "right_001.png").is_file()
    metadata = json.loads(
        (tmp_path / "capture_metadata" / "capture_001.json").read_text(
            encoding="utf-8"
        )
    )
    assert "OpenCV image pixels" in metadata["frame_contract"]
    assert metadata["pair_max_delta_s"] == pytest.approx(0.040)
    assert metadata["stereo_pair_saved"] is True

def test_checkerboard_auto_capture_uses_new_synchronized_detection_pairs() -> None:
    from Workstation.calibration_controller import CalibrationWorkflowController
    from Workstation import standalone_gui

    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    image = standalone_gui.QtGui.QImage(32, 24, standalone_gui.QtGui.QImage.Format_RGB888)
    image.fill(127)
    now_s = time.monotonic()
    panel = SimpleNamespace(
        checkerboard_calibration_thread=None,
        _allow_legacy_calibration_preview_fallback=True,
        calibration_auto_interval_spin=QtWidgets.QDoubleSpinBox(),
        calibration_status_label=QtWidgets.QLabel(),
        capture_calibration_auto_button=QtWidgets.QPushButton(),
        calibration_preview_detection_results={
            "left": {
                "request_id": 1,
                "image": image,
                "found": True,
                "metadata": {"timestamp_s": now_s, "rtcp_frame_ntp_unix_s": 100.000},
            },
            "right": {
                "request_id": 2,
                "image": image,
                "found": True,
                "metadata": {"timestamp_s": now_s, "rtcp_frame_ntp_unix_s": 100.010},
            },
        },
        _checkerboard_capture_pair_timing=lambda _metadata: (True, "rtcp-ntp", 0.010, "ok"),
    )
    panel.calibration_auto_interval_spin.setRange(0.1, 10.0)
    panel.calibration_auto_interval_spin.setValue(0.1)
    controller = CalibrationWorkflowController(panel)
    capture_calls: list[dict[str, object]] = []
    controller.capture_calibration_images = lambda **kwargs: capture_calls.append(kwargs) or True

    assert controller.start_auto_calibration_capture() is True
    assert controller._auto_capture_timer.interval() == 100
    assert len(capture_calls) == 1
    assert capture_calls[0]["capture_origin"] == "auto"

    controller._auto_calibration_capture_tick()
    assert len(capture_calls) == 1

    panel.calibration_preview_detection_results["right"] = {
        "request_id": 3,
        "image": image,
        "found": True,
        "metadata": {"timestamp_s": time.monotonic(), "rtcp_frame_ntp_unix_s": 100.020},
    }
    controller._auto_calibration_capture_tick()
    assert len(capture_calls) == 2
    controller.stop_auto_calibration_capture()
    assert panel.capture_calibration_auto_button.text() == "자동 촬영 시작"
    app.processEvents()


def test_checkerboard_auto_capture_continues_until_operator_stops() -> None:
    from Workstation.calibration_controller import CalibrationWorkflowController

    panel = SimpleNamespace(
        calibration_status_label=QtWidgets.QLabel(),
        _calibration_session_root=None,
        _log=lambda _message: None,
    )
    controller = CalibrationWorkflowController(panel)
    controller._capture_origin = "auto"
    controller._auto_capture_active = True
    controller._auto_capture_saved_pairs = 199

    controller._handle_checkerboard_capture_completed(
        {
            "stem": "200",
            "saved_sides": ("left", "right"),
            "status_parts": ("left=OK", "right=OK"),
            "pair_timing_ok": True,
            "pair_time_basis": "rtcp-ntp",
            "pair_delta_s": 0.001,
        }
    )

    assert controller._auto_capture_saved_pairs == 200
    assert controller._auto_capture_active is True
    assert "Stop할 때까지 계속" in panel.calibration_status_label.text()

def test_calibration_evidence_reports_live_auto_capture_pair_count() -> None:
    controller = SimpleNamespace(_auto_capture_active=True, _auto_capture_saved_pairs=3)
    panel = SimpleNamespace(_calibration_workflow_controller_owner=controller)

    evidence = EvidenceAdapter(panel).calibration()

    assert evidence["valid_pair_count"] == 3
    assert evidence["auto_capture_active"] is True
    assert evidence["auto_capture_saved_pairs"] == 3

def test_calibration_evidence_uses_selected_validation_mode_without_result() -> None:
    class ValidationModeCombo:
        def currentData(self) -> str:
            return "checkerboard"

    panel = SimpleNamespace(
        checkerboard_validation_mode_combo=ValidationModeCombo(),
        checkerboard_validation_result_mode="pixhawk",
        active_intrinsics=None,
        last_calibration_result=None,
    )

    evidence = EvidenceAdapter(panel).calibration()

    assert evidence["validation_mode"] == "checkerboard"

def test_calibration_evidence_shows_zoom_index_and_actual_ratio() -> None:
    panel = SimpleNamespace(
        actual_visible_zoom={
            "left": {"index": 14, "ratio": 30.0},
            "right": {"index": 14, "ratio": 30.0},
        },
    )

    evidence = EvidenceAdapter(panel).calibration()

    assert evidence["zoom_readback"] == "left=14 (30×), right=14 (30×)"

def test_calibration_evidence_keeps_legacy_zoom_readback_without_valid_ratio() -> None:
    panel = SimpleNamespace(
        actual_visible_zoom={
            "left": {"index": 8},
            "right": {"index": 9, "ratio": "invalid"},
        },
    )

    evidence = EvidenceAdapter(panel).calibration()

    assert evidence["zoom_readback"] == "left=8, right=9"

def test_checkerboard_preview_detection_runs_alongside_yolo_and_sync_status() -> None:
    from Workstation import standalone_gui

    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    panel = standalone_gui.WirisStereoDistancePanel()
    panel.yolo_enabled = True
    panel.camera_connected = True
    panel.camera_sync_status = "verified"
    panel._workstation_calibration_preview_active = True
    panel._is_checkerboard_calibration_visible = lambda: False
    panel._record_gui_camera_frame = lambda *_args, **_kwargs: None
    panel._capture_recording_clock_sync_start_observation = lambda: None
    panel._update_camera_connection_status = lambda: None
    panel._record_clock_offset_sample = lambda _now_s: None
    panel._queue_calibration_preview_detection = lambda camera, image: queued.append((camera, image))
    queued: list[tuple[str, object]] = []
    image = standalone_gui.QtGui.QImage(32, 24, standalone_gui.QtGui.QImage.Format_RGB888)
    image.fill(127)

    panel._update_calibration_preview(
        "left",
        image,
        {"timestamp_s": time.monotonic(), "rtcp_frame_ntp_unix_s": 100.0},
    )

    assert queued and queued[0][0] == "left"
    panel.close()
    panel.deleteLater()
    app.processEvents()

def test_checkerboard_calibration_worker_preserves_optical_rdf_stereo_contract(monkeypatch, tmp_path) -> None:
    from Workstation import standalone_gui
    from Workstation import calibration_workers
    from Workstation.workstation_core.calibration import (
        CameraIntrinsics,
        CheckerboardConfig,
        StereoExtrinsics,
    )

    config = CheckerboardConfig(inner_cols=8, inner_rows=6, square_size_m=0.03)
    intrinsics = CameraIntrinsics(
        width=1280,
        height=720,
        camera_matrix=[[900.0, 0.0, 640.0], [0.0, 901.0, 360.0], [0.0, 0.0, 1.0]],
        distortion_coefficients=[0.0],
        reprojection_error_px=0.2,
        image_count=4,
    )
    extrinsics = StereoExtrinsics(
        rotation_left_to_right=[[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]],
        translation_left_to_right_m=[-0.3, 0.0, 0.0],
        reprojection_error_px=0.1,
        pair_count=4,
    )
    calls: list[tuple[str, tuple[Path, ...]]] = []
    quality = SimpleNamespace(
        state="PASS",
        reasons=(),
        effective_pose_count=12,
        image_size=(1280, 720),
        cx_px=640.0,
        cy_px=360.0,
        center_grid_occupied=((0, 0), (1, 1), (2, 2)),
        union_extent_normalized=(0.8, 0.8),
        subset_principal_span_px=(1.0, 1.0),
        subset_focal_relative_span=0.01,
        to_mapping=lambda: {"state": "PASS", "reason_codes": []},
    )

    def fake_solve(paths, _config):
        calls.append(("intrinsics", tuple(paths)))
        return SimpleNamespace(
            intrinsics=intrinsics,
            quality_inputs=lambda: {},
            intrinsic_standard_deviations={},
            extrinsic_standard_deviations=(),
            per_view_errors_px=(),
            effective_view_ids=tuple(str(path) for path in paths),
            observations=(),
            subset_estimates=(),
            subset_membership=(),
            subset_failures=(),
            subset_summary={},
            calibration_flags=0,
        )

    monkeypatch.setattr(
        calibration_workers,
        "calibrate_camera_with_diagnostics",
        fake_solve,
    )
    monkeypatch.setattr(
        calibration_workers,
        "calibrate_camera_from_images",
        lambda paths, _config: calls.append(("intrinsics", tuple(paths))) or intrinsics,
    )
    monkeypatch.setattr(
        calibration_workers,
        "evaluate_camera_calibration_quality",
        lambda **_kwargs: quality,
    )
    monkeypatch.setattr(
        calibration_workers,
        "calibrate_stereo_extrinsics_from_image_pairs",
        lambda left, right, _config, _left, _right: calls.append(
            ("stereo", (*tuple(left), *tuple(right)))
        )
        or extrinsics,
    )
    left_paths = [tmp_path / f"left_{index}.png" for index in range(4)]
    right_paths = [tmp_path / f"right_{index}.png" for index in range(4)]
    worker = standalone_gui.CheckerboardCalibrationWorker(
        mode="checkerboard",
        left_paths=left_paths,
        right_paths=right_paths,
        config=config,
    )
    completed: list[object] = []
    progress: list[dict[str, object]] = []
    worker.completed.connect(completed.append)
    worker.progress.connect(progress.append)

    worker.run()

    assert len(completed) == 1
    result = completed[0]
    assert result.extrinsics is extrinsics
    assert "opencv_optical:RDF" in result.extrinsics.frame_contract
    assert result.extrinsics.translation_left_to_right_m == [-0.3, 0.0, 0.0]
    assert [kind for kind, _paths in calls] == ["intrinsics", "intrinsics", "stereo"]
    assert {item["phase"] for item in progress} >= {
        "left_intrinsic",
        "right_intrinsic",
        "stereo_extrinsic",
    }
    assert progress[-1]["percent"] == pytest.approx(100.0)

    calls.clear()
    validation_worker = standalone_gui.CheckerboardRandomValidationWorker(
        mode="checkerboard",
        pairs=[(str(index), left_paths[index], right_paths[index]) for index in range(4)],
        config=config,
        runs=3,
        subset_count=3,
        seed=7,
    )
    validation_results: list[object] = []
    validation_worker.completed.connect(validation_results.append)

    validation_worker.run()

    assert len(validation_results) == 1
    assert len(validation_results[0]) == 3
    assert all(row["result"].extrinsics is extrinsics for row in validation_results[0])
    assert [kind for kind, _paths in calls].count("stereo") == 3

def test_calibration_pair_limit_selects_evenly_spread_pairs() -> None:
    from Workstation.calibration_controller import CalibrationWorkflowController

    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    limit_spin = QtWidgets.QSpinBox()
    limit_spin.setRange(0, 10000)
    limit_spin.setValue(4)
    panel = SimpleNamespace(calib_calculation_pairs_spin=limit_spin)
    controller = CalibrationWorkflowController(panel)
    pairs = [(f"{index:03d}", Path(f"left_{index:03d}.png"), Path(f"right_{index:03d}.png")) for index in range(10)]

    selected = controller._select_calibration_pairs(pairs)

    assert len(selected) == 4
    assert selected[0][0] == "000"
    assert selected[-1][0] == "009"
    assert [item[0] for item in selected] == sorted(item[0] for item in selected)
    limit_spin.setValue(0)
    assert controller._select_calibration_pairs(pairs) == pairs

def test_workstation_calibration_and_validation_run_off_ui_thread(monkeypatch, tmp_path) -> None:
    from Workstation import standalone_gui
    from Workstation import calibration_workers
    from Workstation.workstation_core.calibration import CameraIntrinsics, StereoExtrinsics

    monkeypatch.setattr(standalone_gui, "APP_ROOT", tmp_path)
    monkeypatch.setattr(standalone_gui.WirisStereoDistancePanel, "_initial_status", lambda self: None)
    monkeypatch.setattr(standalone_gui.WirisStereoDistancePanel, "_auto_load_intrinsics", lambda self: None)
    monkeypatch.setattr(standalone_gui, "opencv_available", lambda: (True, None))
    monkeypatch.setattr(standalone_gui, "save_settings", lambda _settings: None)
    root = tmp_path / "custom-calibration"
    for side in ("left", "right"):
        directory = root / side
        directory.mkdir(parents=True)
        for index in range(4):
            (directory / f"{side}_{index:03d}.png").write_text("fixture", encoding="utf-8")
    metadata_root = root / "capture_metadata"
    metadata_root.mkdir(parents=True)
    for index in range(4):
        (metadata_root / f"capture_{index:03d}.json").write_text(
            json.dumps(
                {
                    "capture_zoom_level": 8,
                    "capture_left_zoom_level": 8,
                    "capture_right_zoom_level": 8,
                    "capture_zoom_source": "GZVV",
                }
            ),
            encoding="utf-8",
        )

    def fake_intrinsics(paths, _config):
        return CameraIntrinsics(
            width=1280,
            height=720,
            camera_matrix=[[900.0, 0.0, 640.0], [0.0, 901.0, 360.0], [0.0, 0.0, 1.0]],
            distortion_coefficients=[0.0],
            reprojection_error_px=0.2,
            image_count=len(paths),
        )

    def fake_extrinsics(left_paths, _right_paths, _config, _left, _right):
        return StereoExtrinsics(
            rotation_left_to_right=[[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]],
            translation_left_to_right_m=[-0.3, 0.0, 0.0],
            reprojection_error_px=0.1,
            pair_count=len(left_paths),
        )

    quality = SimpleNamespace(
        state="PASS",
        reasons=(),
        effective_pose_count=20,
        image_size=(1280, 720),
        cx_px=640.0,
        cy_px=360.0,
        center_grid_occupied=((0, 0), (1, 1), (2, 2)),
        union_extent_normalized=(0.8, 0.8),
        subset_principal_span_px=(1.0, 1.0),
        subset_focal_relative_span=0.01,
        to_mapping=lambda: {"state": "PASS", "reason_codes": []},
    )

    def fake_solve(paths, config):
        intrinsics = fake_intrinsics(paths, config)
        return SimpleNamespace(
            intrinsics=intrinsics,
            quality_inputs=lambda: {},
            intrinsic_standard_deviations={},
            extrinsic_standard_deviations=(),
            per_view_errors_px=(),
            effective_view_ids=tuple(str(path) for path in paths),
            observations=tuple(paths),
            subset_estimates=(),
            subset_membership=(),
            subset_failures=(),
            subset_summary={},
            calibration_flags=0,
        )

    monkeypatch.setattr(calibration_workers, "calibrate_camera_with_diagnostics", fake_solve)
    monkeypatch.setattr(calibration_workers, "calibrate_camera_from_images", fake_intrinsics)
    monkeypatch.setattr(
        calibration_workers,
        "evaluate_camera_calibration_quality",
        lambda **_kwargs: quality,
    )
    monkeypatch.setattr(calibration_workers, "calibrate_stereo_extrinsics_from_image_pairs", fake_extrinsics)
    monkeypatch.setattr(
        calibration_workers,
        "detect_checkerboard",
        lambda _path, _config: SimpleNamespace(found=True, corner_count=48, error=""),
    )
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    panel = standalone_gui.WirisStereoDistancePanel()
    # This legacy worker test exercises a caller-owned custom calibration
    # directory. Session-owned routing has separate contract coverage.
    panel._measurement_session_hierarchy_enabled = False
    panel.calib_dir_edit.setText(str(root))
    panel.checkerboard_validation_root_edit.setText(str(root))
    panel.checkerboard_validation_mode_combo.setCurrentIndex(
        panel.checkerboard_validation_mode_combo.findData("checkerboard")
    )

    image = standalone_gui.QtGui.QImage(32, 24, standalone_gui.QtGui.QImage.Format_RGB888)
    image.fill(127)
    panel.left_pane.current_image = lambda: image
    panel.right_pane.current_image = lambda: image
    native_now = time.monotonic()
    for side, pane, rtcp in (
        ("left", panel.left_pane, 100.000),
        ("right", panel.right_pane, 100.002),
    ):
        pane._store_measurement_frame(
            native_now,
            image,
            {
                "timestamp_s": native_now,
                "rtcp_frame_ntp_unix_s": rtcp,
                "native_width": image.width(),
                "native_height": image.height(),
                "native_lossless_source": True,
                "overlay_applied": False,
                "source_native_frame_id": f"native-{side}",
            },
        )
    panel.camera_frame_metadata["left"] = {"rtcp_frame_ntp_unix_s": 100.000}
    panel.camera_frame_metadata["right"] = {"rtcp_frame_ntp_unix_s": 100.002}
    panel.actual_visible_zoom["left"] = {"index": 8, "ratio": 8.0}
    panel.actual_visible_zoom["right"] = {"index": 8, "ratio": 8.0}
    panel.capture_calibration_images()
    assert panel.checkerboard_calibration_thread is not None
    deadline = time.monotonic() + 2.0
    while panel.checkerboard_calibration_thread is not None and time.monotonic() < deadline:
        app.processEvents()
        QtCore.QThread.msleep(5)
    assert panel.checkerboard_calibration_thread is None
    assert (root / "left" / "left_004.png").is_file()
    capture_metadata = json.loads(
        (root / "capture_metadata" / "capture_004.json").read_text(encoding="utf-8")
    )
    assert capture_metadata["frame_grid_by_side"]["left"] == {
        "frame_id": "native-left",
        "source_native_width": 32,
        "source_native_height": 24,
        "saved_width": 32,
        "saved_height": 24,
        "preview_width": 0,
        "preview_height": 0,
        "resize_scale_xy": [1.0, 1.0],
        "crop_xywh": None,
        "overlay_applied": False,
        "native_lossless_source": True,
    }
    assert panel.capture_calibration_button.isEnabled()

    panel.run_intrinsic_calibration()
    assert panel.checkerboard_calibration_thread is not None
    assert EvidenceAdapter(panel).calibration()["state"] == "CALCULATING"
    deadline = time.monotonic() + 2.0
    while panel.checkerboard_calibration_thread is not None and time.monotonic() < deadline:
        app.processEvents()
        QtCore.QThread.msleep(5)
    assert panel.checkerboard_calibration_thread is None
    assert panel.checkerboard_validation_result is not None
    assert EvidenceAdapter(panel).calibration()["state"] == "AVAILABLE"
    assert EvidenceAdapter(panel).calibration()["valid_pair_count"] == 5
    assert [row["run"] for row in EvidenceAdapter(panel).calibration()["validation_run_results"]] == ["Full"]
    assert "T_left_to_right_m" in panel.calibration_result_text.toPlainText()

    panel.checkerboard_validation_random_runs_spin.setValue(2)
    panel.checkerboard_validation_random_pair_count_spin.setValue(3)
    panel.run_checkerboard_random_validation()
    assert panel.checkerboard_calibration_thread is not None
    deadline = time.monotonic() + 2.0
    while panel.checkerboard_calibration_thread is not None and time.monotonic() < deadline:
        app.processEvents()
        QtCore.QThread.msleep(5)
    assert panel.checkerboard_calibration_thread is None
    assert "Random stereo-pair validation" in panel.calibration_result_text.toPlainText()
    run_results = EvidenceAdapter(panel).calibration()["validation_run_results"]
    assert [row["run"] for row in run_results] == ["Full", "Random 1", "Random 2"]
    assert all(row["state"] == "OK" for row in run_results)

    panel.close()
    panel.deleteLater()
    app.processEvents()

def test_measurement_and_calibration_setup_facades_delegate_to_owner() -> None:
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    calls: list[tuple[str, object]] = []
    panel = SimpleNamespace(
        apply_workstation_measurement_settings=lambda payload: calls.append(("measurement", payload)),
        save_current_settings_from_ui=lambda: calls.append(("save", None)),
        save_workstation_calibration_setup=lambda: calls.append(("save", None)),
        apply_workstation_calibration_setup=lambda payload: calls.append(("calibration", payload)),
        scan_checkerboard_validation_files=lambda: calls.append(("scan", None)),
        refresh_baseline_field_browser=lambda: calls.append(("alignment_scan", None)),
        start_baseline_sampling=lambda: calls.append(("alignment_start", None)),
        stop_baseline_sampling=lambda: calls.append(("alignment_stop", None)),
        save_alignment_profile=lambda: calls.append(("alignment_save", None)),
        load_alignment_profile=lambda: calls.append(("alignment_load", None)),
        swap_pixhawks=lambda: calls.append(("alignment_swap", None)),
    )
    backend = StandalonePanelBackend(panel)
    measurement = {"distance_mode": "pixhawk", "focal_px": 900.0}
    setup = {"inner_cols": 13, "inner_rows": 8, "square_size_mm": 40.0}
    backend.apply_measurement_settings(measurement)
    backend.save_measurement_settings()
    backend.calibration_apply_setup(setup)
    backend.calibration_scan_validation()
    backend.calibration_save_setup()
    backend.alignment_scan_sources()
    backend.alignment_start_sampling()
    backend.alignment_stop_sampling()
    backend.alignment_save_profile()
    backend.alignment_load_profile()
    backend.alignment_swap_pixhawks()
    assert calls == [
        ("measurement", measurement),
        ("save", None),
        ("calibration", setup),
        ("scan", None),
        ("save", None),
        ("alignment_scan", None),
        ("alignment_start", None),
        ("alignment_stop", None),
        ("alignment_save", None),
        ("alignment_load", None),
        ("alignment_swap", None),
    ]
    backend.deleteLater()
    app.processEvents()

def test_workstation_uses_source_backed_alignment_calibration_and_recording_pages() -> None:
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    backend = _FakeBackend()
    window = WorkstationMainWindow(backend=backend)
    window.show()
    app.processEvents()

    for context, workspace in (
        ("devices.pixhawk", window.pixhawk),
        ("devices.gnss", window.base_gnss),
        ("processing.alignment", window.alignment),
        ("tasks.calibration", window.calibration),
        ("tasks.advanced", window.advanced),
    ):
        window.navigation.select_context(context)
        app.processEvents()
        assert window.workspace_host.currentWidget().widget() is workspace

    assert not hasattr(window.alignment, "attitude_model")
    assert not hasattr(window.alignment, "gnss_model")
    assert window.alignment.operation_tabs is window.alignment.optical_pose_pane.method_tabs
    assert window.calibration.intrinsic_model.rowCount() == 2
    window._select_context("tasks.session_details")
    app.processEvents()
    assert window._current_context == "tasks.session"
    assert window.navigation.currentIndex().data(QtCore.Qt.UserRole) == "tasks.session"
    assert window.sessions.current_tab_id() == "review"
    assert window.recording.artifact_model.rowCount() >= 1
    assert window.recording.protocol_state_label.text().startswith("BLOCKED")
    assert any("Both camera streams" in label.text() for label in window.recording.protocol_check_labels.values())
    assert any("Checkerboard stereo R,t" in label.text() for label in window.recording.protocol_check_labels.values())
    assert not any("Numeric GNSS fix_type" in label.text() for label in window.recording.protocol_check_labels.values())
    assert any("Physical alignment evidence" in label.text() for label in window.recording.protocol_check_labels.values())
    assert any("Final WIRIS time-sync checkpoint" in label.text() for label in window.recording.protocol_check_labels.values())
    assert any(
        "NED" in label.text()
        for label in window.alignment.findChildren(QtWidgets.QLabel)
    )

    assert [
        window.calibration.operation_tabs.tabText(index)
        for index in range(window.calibration.operation_tabs.count())
    ] == ["Intrinsics", "Extrinsics"]
    backend.calls.clear()
    window.calibration.zoom_query_button.click()
    assert (
        window.calibration.operation_buttons["calibration.capture_images"].text()
        == "Capture Live Pair"
    )
    assert (
        window.calibration.operation_buttons[
            "calibration.capture_images_wiris_ftp"
        ].text()
        == "Capture HR Still (FTP)"
    )
    assert (
        window.calibration.operation_buttons[
            "calibration.capture_images_wiris_ftp_oz"
        ].text()
        == "Capture OZ Still (FTP)"
    )
    assert (
        window.commands._specs["calibration.sync_images_wiris_oz_ftp"].text
        == "FTP 자동 전송"
    )
    assert not window.calibration.operation_buttons["calibration.run_extrinsics"].isEnabled()
    for command_id in (
        "calibration.save_extrinsics",
        "calibration.load_extrinsics",
        "calibration.apply_bundle",
    ):
        window.calibration.operation_buttons[command_id].click()
    assert ("advanced_zoom", "both", "query", None) in backend.calls
    assert "detection.toggle" not in window.calibration.camera_buttons
    assert "calibration.load_current_zoom_intrinsics" not in window.commands._specs
    assert "calibration.apply_intrinsics" not in window.commands._specs
    assert "calibration.run_validation" not in window.commands._specs
    assert "calibration.scan_validation" not in window.commands._specs
    advanced_zoom_button = window.calibration.findChildren(QtWidgets.QPushButton)[-1]
    assert advanced_zoom_button.text() == "Advanced Zoom"
    advanced_zoom_button.click()
    assert window.workspace_host.currentWidget().widget() is window.advanced
    assert window.advanced.tabs.currentIndex() == 0
    assert "calibration_run_extrinsics" not in backend.calls
    assert "calibration_save_extrinsics" in backend.calls
    assert "calibration_load_extrinsics" in backend.calls
    assert "calibration_apply_bundle" in backend.calls

    window.recording.pnp_capture_button.click()
    assert "hr_capture" in backend.calls
    assert window.recording.pnp_capture_button.text() == "Capture HR Still"
    assert window.recording.oz_capture_button.text() == "Capture OZ Still"

    window.measurement.refresh({})
    assert not any(
        "Calibration bundle:" in label.text()
        for label in window.measurement.findChildren(QtWidgets.QLabel)
    )
    window.close()
    app.processEvents()
    window.deleteLater()
    app.processEvents()

def test_load_intrinsics_opens_file_picker_and_applies_selected_profile(
    monkeypatch: pytest.MonkeyPatch,
    tmp_path: Path,
) -> None:
    from Workstation import standalone_gui
    from Workstation.workstation_core.calibration import (
        CameraIntrinsics,
        CheckerboardConfig,
        StereoIntrinsicsResult,
        load_intrinsics_json,
        save_intrinsics_json,
    )

    monkeypatch.setattr(standalone_gui.WirisStereoDistancePanel, "_initial_status", lambda self: None)
    monkeypatch.setattr(standalone_gui.WirisStereoDistancePanel, "_auto_load_intrinsics", lambda self: None)
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    panel = standalone_gui.WirisStereoDistancePanel(app_root=tmp_path)
    panel._measurement_session_state = "OPEN"
    panel._measurement_session_id = "20260822_130000_000000001"
    panel.recording_session_id = panel._measurement_session_id
    panel.zoom_level_spin.setValue(4)
    camera = CameraIntrinsics(
        width=1920,
        height=1080,
        camera_matrix=[[1000.0, 0.0, 960.0], [0.0, 1001.0, 540.0], [0.0, 0.0, 1.0]],
        distortion_coefficients=[0.0, 0.0, 0.0, 0.0, 0.0],
        reprojection_error_px=0.2,
        image_count=12,
    )
    result = StereoIntrinsicsResult(
        left=camera,
        right=camera,
        board=CheckerboardConfig(inner_cols=9, inner_rows=6, square_size_m=0.025),
    )
    selected_path = tmp_path / "selected_intrinsics.json"
    save_intrinsics_json(selected_path, result, metadata={"zoom_level": 4})
    monkeypatch.setattr(
        standalone_gui.QtWidgets.QFileDialog,
        "getOpenFileName",
        lambda *_args, **_kwargs: (str(selected_path), "Intrinsics JSON (*.json)"),
    )

    panel.load_intrinsics()

    assert panel.active_intrinsics == result
    immutable = Path(panel._active_intrinsics_profile_path)
    assert immutable.is_file()
    assert "profiles/intrinsics" in immutable.as_posix()
    assert load_intrinsics_json(immutable) == result
    assert panel.manual_focal_checkbox.isChecked() is False
    assert panel.calibration_status_label.text().startswith(
        "Loaded and applied intrinsics: selected_intrinsics.json | Session copy="
    )
    session_copies = list(
        (tmp_path / "recordings" / panel.recording_session_id / "calibration").glob(
            "*/loaded_intrinsics/*-selected_intrinsics.json"
        )
    )
    assert len(session_copies) == 1
    assert session_copies[0].read_bytes() == selected_path.read_bytes()
    panel.deleteLater()
    app.processEvents()

def test_save_intrinsics_uses_capture_zoom_for_filename(
    monkeypatch: pytest.MonkeyPatch,
    tmp_path: Path,
) -> None:
    from Workstation import standalone_gui
    from Workstation.workstation_core.calibration import (
        CameraIntrinsics,
        CheckerboardConfig,
        StereoIntrinsicsResult,
    )

    monkeypatch.setattr(standalone_gui.WirisStereoDistancePanel, "_initial_status", lambda self: None)
    monkeypatch.setattr(standalone_gui.WirisStereoDistancePanel, "_auto_load_intrinsics", lambda self: None)
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    panel = standalone_gui.WirisStereoDistancePanel(app_root=tmp_path)
    root = tmp_path / "captures"
    root.mkdir()
    panel.calib_dir_edit.setText(str(root))
    camera = CameraIntrinsics(
        width=1920,
        height=1080,
        camera_matrix=[[1000.0, 0.0, 960.0], [0.0, 1001.0, 540.0], [0.0, 0.0, 1.0]],
        distortion_coefficients=[0.0, 0.0, 0.0, 0.0, 0.0],
        reprojection_error_px=0.2,
        image_count=12,
    )
    panel.last_calibration_result = StereoIntrinsicsResult(
        left=camera,
        right=camera,
        board=CheckerboardConfig(inner_cols=9, inner_rows=6, square_size_m=0.025),
    )
    panel.last_calibration_zoom_level = 8
    panel.zoom_level_spin.setValue(1)
    monkeypatch.setattr(panel, "_save_current_settings", lambda *_args, **_kwargs: None)

    panel.save_intrinsics()

    assert (root / "intrinsics_zoom_08.json").is_file()
    assert not (root / "intrinsics_zoom_01.json").exists()
    metadata = json.loads((root / "intrinsics_zoom_08.json").read_text(encoding="utf-8"))["metadata"]
    assert metadata["capture_zoom_level"] == 8
    assert metadata["zoom_source"] == "capture_metadata"
    panel.deleteLater()
    app.processEvents()

def test_workstation_exposes_measurement_and_calibration_setup_controls() -> None:
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    backend = _FakeBackend()
    window = WorkstationMainWindow(backend=backend)
    window.show()
    app.processEvents()

    assert window.measurement.operation_buttons["measurement.apply_settings"].isEnabled()
    assert window.measurement.operation_buttons["measurement.save_settings"].isEnabled()
    assert window.time_workspace.operation_buttons["time.apply_settings"].isEnabled()
    assert window.time_workspace.operation_buttons["time.save_settings"].isEnabled()
    assert window.calibration.operation_buttons["calibration.apply_setup"].isEnabled()
    assert "calibration.scan_validation" not in window.calibration.operation_buttons
    assert window.calibration.operation_buttons["calibration.save_setup"].isEnabled()
    assert "alignment.scan_sources" not in window.alignment.operation_buttons
    assert "alignment.start_sampling" not in window.alignment.operation_buttons
    assert "alignment.stop_sampling" not in window.alignment.operation_buttons
    assert "alignment.save_profile" not in window.alignment.operation_buttons
    assert "alignment.load_profile" not in window.alignment.operation_buttons
    assert "alignment.swap_pixhawks" not in window.alignment.operation_buttons
    measurement_payload = window.measurement.settings_payload()
    time_payload = window.time_workspace.settings_payload()
    assert "distance_mode" not in measurement_payload
    assert {
        "intrinsics_source",
        "focal_px",
        "yolo_input_size_px",
    }.isdisjoint(window.measurement.settings_widgets)
    assert {
        "intrinsics_source",
        "focal_px",
        "yolo_input_size_px",
    }.isdisjoint(measurement_payload)
    assert "yolo_input_size" in window.settings_workspace.settings_widgets
    assert window.measurement.measurement_method_combo.objectName() == "measurementMethodCombo"
    assert measurement_payload["yolo_confidence"] > 0
    assert "camera_sync_rtcp_pair_max_delta_ms" not in measurement_payload
    assert "camera_sync_rtcp_pair_max_delta_ms" in time_payload
    calibration_payload = window.calibration.setup_payload()
    assert calibration_payload["inner_cols"] >= 2
    assert calibration_payload["square_size_mm"] > 0
    assert calibration_payload["validation_root"] == calibration_payload["calibration_root"]
    assert calibration_payload["pair_max_delta_ms"] == pytest.approx(100.0)
    assert calibration_payload["auto_interval_s"] == pytest.approx(1.0)
    assert "auto_target_pairs" not in calibration_payload
    assert "auto_target_pairs" not in window.calibration.setup_widgets
    assert calibration_payload["calculation_pair_count"] == 20
    assert window.calibration.setup_widgets["auto_interval_s"].minimum() == pytest.approx(0.1)
    window.calibration.setup_widgets["pair_max_delta_ms"].setValue(35.0)
    backend.calls.clear()
    assert window.commands.invoke("calibration.capture_images") is True
    assert backend.calls[-2] == (
        "calibration_apply_setup",
        {"pair_max_delta_ms": pytest.approx(35.0)},
    )
    assert backend.calls[-1] == "calibration_capture_images"
    window.calibration.setup_widgets["calculation_pair_count"].setValue(12)
    window.calibration.setup_widgets["auto_interval_s"].setValue(2.5)
    assert window.commands.invoke("calibration.auto_capture") is True
    assert backend.calls[-2][0] == "calibration_apply_setup"
    assert backend.calls[-2][1]["auto_interval_s"] == pytest.approx(2.5)
    assert "auto_target_pairs" not in backend.calls[-2][1]
    assert backend.calls[-2][1]["calculation_pair_count"] == 12
    assert backend.calls[-1] == "calibration_toggle_auto_capture"
    window.calibration.setup_widgets["validation_mode"].setCurrentIndex(
        window.calibration.setup_widgets["validation_mode"].findData("checkerboard")
    )
    window.calibration.apply_selected_calibration_session = lambda: True
    window.calibration._calibration_session_explicit_root = "/tmp/test-photo-set"
    assert window.commands.invoke("calibration.run_intrinsic") is True
    assert backend.calls[-2][0] == "calibration_apply_setup"
    assert backend.calls[-2][1]["validation_mode"] == "checkerboard"
    assert backend.calls[-1] == "calibration_run_intrinsic"
    assert backend.last_calibration_input_request["root"] == "/tmp/test-photo-set"
    assert backend.last_calibration_input_request["mode"] in {
        "pixhawk",
        "checkerboard",
    }
    assert window.commands.invoke("calibration.run_validation") is False
    assert window.commands.invoke("measurement.apply_settings") is True
    assert window.commands.invoke("measurement.save_settings") is True
    assert window.commands.invoke("time.apply_settings") is True
    assert window.commands.invoke("time.save_settings") is True
    assert window.commands.invoke("calibration.apply_setup") is True
    assert window.commands.invoke("calibration.scan_validation") is False
    assert window.commands.invoke("calibration.save_setup") is True
    assert window.commands.invoke("alignment.scan_sources") is True
    assert window.commands.invoke("alignment.start_sampling") is True
    assert window.commands.invoke("alignment.stop_sampling") is True
    assert window.commands.invoke("alignment.save_profile") is True
    assert window.commands.invoke("alignment.load_profile") is True
    assert window.commands.invoke("alignment.swap_pixhawks") is True
    extended_calls = backend.calls[-15:]
    assert [call[0] if isinstance(call, tuple) else call for call in extended_calls] == [
        "measurement_apply_settings",
        "measurement_apply_settings",
        "measurement_save_settings",
        "measurement_apply_settings",
        "measurement_apply_settings",
        "measurement_save_settings",
        "calibration_apply_setup",
        "calibration_apply_setup",
        "calibration_save_setup",
        "alignment_scan_sources",
        "alignment_start_sampling",
        "alignment_stop_sampling",
        "alignment_save_profile",
        "alignment_load_profile",
        "alignment_swap_pixhawks",
    ]
    window.close()
    app.processEvents()
    window.deleteLater()
    app.processEvents()

def test_calibration_compute_requires_explicit_photo_session_selection(
    monkeypatch: pytest.MonkeyPatch,
    tmp_path: Path,
) -> None:
    from PyQt5 import QtGui

    from Workstation import workspaces

    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    root = tmp_path / "saved-calibration"
    for side in ("left", "right"):
        directory = root / side
        directory.mkdir(parents=True)
        image = QtGui.QImage(32, 24, QtGui.QImage.Format_RGB888)
        image.fill(QtGui.QColor("black"))
        assert image.save(str(directory / f"{side}_pair-001.png"))
    monkeypatch.setattr(workspaces, "DEFAULT_CALIBRATION_ROOT", str(root))

    backend = _FakeBackend()
    window = WorkstationMainWindow(backend=backend)
    window.show()
    app.processEvents()

    workspace = window.calibration
    assert workspace.calibration_session_combo.currentIndex() == 0
    assert workspace.calibration_session_combo.currentData() is None
    assert workspace.calibration_selected_files_table.rowCount() == 0
    assert not workspace.operation_buttons["calibration.run_intrinsic"].isEnabled()
    assert not workspace.operation_buttons["calibration.run_extrinsics"].isEnabled()
    backend.calls.clear()
    assert window.commands.invoke("calibration.run_intrinsic") is False
    assert "calibration_run_intrinsic" not in backend.calls

    workspace.calibration_session_combo.setCurrentIndex(1)
    app.processEvents()

    assert workspace.calibration_session_combo.currentData() == str(root)
    assert workspace.calibration_selected_files_table.rowCount() == 1
    assert workspace.operation_buttons["calibration.run_intrinsic"].isEnabled()
    assert workspace.operation_buttons["calibration.run_extrinsics"].isEnabled()
    workspace._refresh_calibration_session_selector(force=True)
    assert workspace.calibration_session_combo.currentData() == str(root)
    assert workspace.calibration_selected_files_table.rowCount() == 1
    assert window.commands.invoke("calibration.run_intrinsic") is True
    assert backend.calls[-1] == "calibration_run_intrinsic"
    assert backend.last_calibration_input_request["root"] == str(root)
    assert backend.last_calibration_input_request["pair_limit"] == 20
    applied_setup = next(
        call[1]
        for call in reversed(backend.calls)
        if isinstance(call, tuple) and call[0] == "calibration_apply_setup"
    )
    assert applied_setup["calibration_root"] == str(root)
    assert applied_setup["validation_root"] == str(root)

    workspace.calibration_session_combo.setCurrentIndex(0)
    app.processEvents()
    assert workspace.calibration_selected_files_table.rowCount() == 0
    assert not workspace.operation_buttons["calibration.run_intrinsic"].isEnabled()
    assert not workspace.operation_buttons["calibration.run_extrinsics"].isEnabled()

    window.close()
    app.processEvents()
    window.deleteLater()
    app.processEvents()

def test_calibration_workspace_exposes_validation_controls() -> None:
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    backend = _FakeBackend()
    window = WorkstationMainWindow(backend=backend)
    window.status_adapter.stop()
    window.calibration.refresh(window.status_adapter.model.snapshot())

    assert [
        window.calibration.operation_tabs.tabText(index)
        for index in range(window.calibration.operation_tabs.count())
    ] == ["Intrinsics", "Extrinsics"]
    # The checkerboard random-validation workflow (load/run/save/apply) was
    # fully implemented in calibration_controller.py/standalone_gui.py from
    # early on, but never had a way to reach it from this navigation-based
    # UI -- only from the legacy panel's own hidden widgets. These controls
    # are the payload-in/result-out surface added to close that gap.
    assert hasattr(window.calibration, "checkerboard_validation_root_edit")
    assert hasattr(window.calibration, "checkerboard_validation_status_label")
    assert hasattr(window.calibration, "checkerboard_validation_random_button")

    window.close()
    window.deleteLater()
    app.processEvents()

def test_calibration_pair_picker_sync_is_cached_until_capture_directory_changes(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    from PyQt5 import QtGui

    import Workstation.workspaces as workspaces_module
    from Workstation.workspaces import CalibrationWorkspace

    class CountingListWidget(QtWidgets.QListWidget):
        def __init__(self) -> None:
            super().__init__()
            self.clear_calls = 0

        def clear(self) -> None:
            self.clear_calls += 1
            super().clear()

    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    root = tmp_path / "calibration"
    (root / "left").mkdir(parents=True)
    (root / "right").mkdir(parents=True)
    image = QtGui.QImage(16, 9, QtGui.QImage.Format_RGB888)
    image.fill(QtGui.QColor("black"))
    assert image.save(str(root / "left" / "left_001.png"))
    assert image.save(str(root / "right" / "right_001.png"))

    evidence = SimpleNamespace(calibration_setup=lambda: {"calibration_root_resolved": str(root)})
    monkeypatch.setattr(workspaces_module, "_evidence", lambda _backend: evidence)

    workspace = CalibrationWorkspace.__new__(CalibrationWorkspace)
    workspace.backend = SimpleNamespace()
    workspace.calibration_pair_picker_list = CountingListWidget()
    workspace.calibration_manual_selection_checkbox = QtWidgets.QCheckBox()
    workspace.calibration_show_all_pairs_checkbox = QtWidgets.QCheckBox()
    workspace.calibration_show_all_pairs_checkbox.setChecked(True)
    workspace.calibration_pair_picker_summary = QtWidgets.QLabel()
    workspace._calibration_pair_picker_signature = None
    workspace._calibration_pair_picker_thumbnail_generation = 0
    workspace._calibration_pair_picker_thumbnail_paths = []
    workspace._calibration_pair_picker_thumbnail_index = 0
    workspace._calibration_pair_picker_items = {}
    workspace._calibration_pair_picker_visible_keys = set()
    workspace._calibration_pair_picker_session_started_unix_ns = 0

    workspace._sync_calibration_pair_picker()
    workspace._sync_calibration_pair_picker()
    assert workspace.calibration_pair_picker_list.count() == 1
    assert workspace.calibration_pair_picker_list.clear_calls == 1

    assert image.save(str(root / "left" / "left_002.png"))
    assert image.save(str(root / "right" / "right_002.png"))
    os.utime(root / "left", None)
    os.utime(root / "right", None)
    workspace._sync_calibration_pair_picker()
    assert workspace.calibration_pair_picker_list.count() == 2
    assert workspace.calibration_pair_picker_list.clear_calls == 2

    workspace.calibration_pair_picker_list.deleteLater()
    workspace.calibration_manual_selection_checkbox.deleteLater()
    app.processEvents()

def test_calibration_pair_picker_defaults_to_session_pairs_and_can_show_all(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    from PyQt5 import QtGui

    import Workstation.workspaces as workspaces_module
    from Workstation.workspaces import CalibrationWorkspace

    class FakeListWidget(QtWidgets.QListWidget):
        pass

    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    root = tmp_path / "calibration"
    (root / "left").mkdir(parents=True)
    (root / "right").mkdir(parents=True)
    image = QtGui.QImage(16, 9, QtGui.QImage.Format_RGB888)
    image.fill(QtGui.QColor("black"))
    for index in range(55):
        key = f"{index:03d}"
        assert image.save(str(root / "left" / f"left_{key}.png"))
        assert image.save(str(root / "right" / f"right_{key}.png"))

    evidence = SimpleNamespace(calibration_setup=lambda: {"calibration_root_resolved": str(root)})
    monkeypatch.setattr(workspaces_module, "_evidence", lambda _backend: evidence)
    workspace = CalibrationWorkspace.__new__(CalibrationWorkspace)
    workspace.backend = SimpleNamespace()
    workspace.calibration_pair_picker_list = FakeListWidget()
    workspace.calibration_manual_selection_checkbox = QtWidgets.QCheckBox()
    workspace.calibration_show_all_pairs_checkbox = QtWidgets.QCheckBox()
    workspace.calibration_pair_picker_summary = QtWidgets.QLabel()
    workspace._calibration_pair_picker_signature = None
    workspace._calibration_pair_picker_thumbnail_generation = 0
    workspace._calibration_pair_picker_thumbnail_paths = []
    workspace._calibration_pair_picker_thumbnail_index = 0
    workspace._calibration_pair_picker_items = {}
    workspace._calibration_pair_picker_visible_keys = set()
    session_started_ns = 2_000_000_000
    workspace._calibration_pair_picker_session_started_unix_ns = session_started_ns

    for index in range(55):
        timestamp_ns = 1_000_000_000 if index < 5 else 3_000_000_000
        key = f"{index:03d}"
        os.utime(root / "left" / f"left_{key}.png", ns=(timestamp_ns, timestamp_ns))
        os.utime(root / "right" / f"right_{key}.png", ns=(timestamp_ns, timestamp_ns))

    workspace._sync_calibration_pair_picker()
    assert workspace.calibration_pair_picker_list.count() == 50
    assert "앱 실행 후 표시: 50 / 전체 55 pair" == workspace.calibration_pair_picker_summary.text()

    workspace.calibration_show_all_pairs_checkbox.setChecked(True)
    workspace._handle_calibration_pair_picker_filter_changed(True)
    assert workspace.calibration_pair_picker_list.count() == 55
    assert "전체 표시: 55 / 전체 55 pair" == workspace.calibration_pair_picker_summary.text()

    workspace._calibration_pair_picker_thumbnail_generation += 1
    workspace.calibration_pair_picker_list.deleteLater()
    workspace.calibration_manual_selection_checkbox.deleteLater()
    workspace.calibration_show_all_pairs_checkbox.deleteLater()
    workspace.calibration_pair_picker_summary.deleteLater()
    app.processEvents()

def test_explicit_calibration_file_selection_is_handed_to_intrinsic_and_stereo_pair_inputs(
    tmp_path: Path,
) -> None:
    from Workstation.calibration_controller import CalibrationWorkflowController

    root = tmp_path / "session"
    (root / "left").mkdir(parents=True)
    (root / "right").mkdir(parents=True)
    left_paths = [root / "left" / "left_002.png", root / "left" / "left_004.png"]
    right_paths = [root / "right" / "right_002.png", root / "right" / "right_004.png"]
    for path in (*left_paths, *right_paths):
        path.write_bytes(b"selected")

    selection = {
        "manual_mode": True,
        "left_paths": [str(left_paths[1]), str(left_paths[0])],
        "right_paths": [str(right_paths[1]), str(right_paths[0])],
        "excluded_left_paths": [str(left_paths[1])],
        "excluded_right_paths": [str(right_paths[1])],
    }
    panel = SimpleNamespace(
        _load_calibration_pair_selection=lambda: selection,
        _calibration_dirs=lambda: (root, root / "left", root / "right"),
    )
    controller = CalibrationWorkflowController.__new__(CalibrationWorkflowController)
    controller.panel = panel

    selected_left, selected_right, explicit = controller._selected_calibration_file_lists(
        root,
        [],
        [],
    )
    assert explicit is True
    assert selected_left == [left_paths[0].resolve()]
    assert selected_right == [right_paths[0].resolve()]
    pairs = controller._selected_calibration_pairs(selected_left, selected_right)
    assert [(left.name, right.name) for _key, left, right in pairs] == [
        ("left_002.png", "right_002.png"),
    ]
