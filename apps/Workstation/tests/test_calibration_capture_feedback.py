from __future__ import annotations

import json
import time
from pathlib import Path
from types import SimpleNamespace

from PyQt5 import QtCore, QtGui, QtWidgets

from Workstation.calibration_controller import CalibrationWorkflowController
from Workstation.calibration_sessions import scan_calibration_capture_inventory
from Workstation.evidence import EvidenceAdapter
from Workstation.protocol import missing_backend_facades
from Workstation.standalone_backend import StandalonePanelBackend
from Workstation.workspaces import CalibrationWorkspace


def _image(path: Path) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_bytes(b"calibration-image")


def _metadata(root: Path, key: str, payload: object) -> None:
    path = root / "capture_metadata" / f"capture_{key}.json"
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(payload), encoding="utf-8")


def test_capture_inventory_reports_empty_missing_root(tmp_path: Path) -> None:
    inventory = scan_calibration_capture_inventory(tmp_path / "not-created")

    assert inventory.state == "EMPTY"
    assert inventory.left_image_count == 0
    assert inventory.right_image_count == 0
    assert inventory.stereo_candidate_count == 0
    assert inventory.left_intrinsics_only_count == 0
    assert inventory.right_intrinsics_only_count == 0
    assert inventory.evidence_warning_pair_count == 0
    assert inventory.error == ""


def test_capture_inventory_matches_persisted_pair_usage(tmp_path: Path) -> None:
    root = tmp_path / "capture-set"
    for key in ("001", "002", "003", "004", "005"):
        _image(root / "left" / f"left_{key}.png")
    for key in ("001", "002", "004", "005"):
        _image(root / "right" / f"right_{key}.png")

    _metadata(root, "001", {"stereo_pair_saved": True})
    _metadata(root, "002", {"stereo_pair_saved": False})
    _metadata(root, "004", {"stereo_pair_saved": "yes"})
    # 005 deliberately has no metadata and therefore follows the existing
    # legacy-compatible stereo rule.

    inventory = scan_calibration_capture_inventory(root)

    assert inventory.state == "READY"
    assert inventory.left_image_count == 5
    assert inventory.right_image_count == 4
    assert inventory.stereo_candidate_count == 2
    assert inventory.left_intrinsics_only_count == 3
    assert inventory.right_intrinsics_only_count == 2
    assert inventory.legacy_unverified_pair_count == 1
    assert inventory.invalid_metadata_pair_count == 1
    assert inventory.evidence_warning_pair_count == 2
    assert inventory.to_mapping()["root"] == str(root)


def test_capture_inventory_supports_current_calibration_image_suffixes(
    tmp_path: Path,
) -> None:
    root = tmp_path / "photo-set"
    _image(root / "left" / "left_a.jpg")
    _image(root / "right" / "right_a.jpg")
    _metadata(root, "a", {"stereo_pair_saved": True})

    inventory = scan_calibration_capture_inventory(root)

    assert inventory.left_image_count == 1
    assert inventory.right_image_count == 1
    assert inventory.stereo_candidate_count == 1


def test_capture_inventory_does_not_present_unreadable_root_as_zero(
    tmp_path: Path,
) -> None:
    root = tmp_path / "not-a-directory"
    root.write_text("file", encoding="utf-8")

    inventory = scan_calibration_capture_inventory(root)

    assert inventory.state == "UNAVAILABLE"
    assert inventory.left_image_count is None
    assert inventory.right_image_count is None
    assert inventory.stereo_candidate_count is None
    assert "not a directory" in inventory.error


def test_capture_inventory_reports_stat_permission_failure(
    monkeypatch,
    tmp_path: Path,
) -> None:
    root = tmp_path / "denied"
    root.mkdir()
    original_stat = Path.stat

    def denied_stat(path: Path, *args, **kwargs):
        if path == root:
            raise PermissionError("capture root denied")
        return original_stat(path, *args, **kwargs)

    monkeypatch.setattr(Path, "stat", denied_stat)

    inventory = scan_calibration_capture_inventory(root)

    assert inventory.state == "UNAVAILABLE"
    assert inventory.left_image_count is None
    assert "capture root denied" in inventory.error


class _Text:
    def __init__(self, value: str) -> None:
        self.value = value

    def text(self) -> str:
        return self.value


class _Value:
    def __init__(self, value: int) -> None:
        self._value = value

    def value(self) -> int:
        return self._value


def _controller(root: Path) -> CalibrationWorkflowController:
    now_s = time.monotonic()
    panel = SimpleNamespace(
        _calibration_dirs=lambda: (root, root / "left", root / "right"),
        calibration_preview_detection_results={
            "left": {
                "request_id": 10,
                "found": True,
                "corners": [(1.0, 2.0)] * 80,
                "status": "checkerboard: 80 corners",
                "metadata": {"timestamp_s": now_s - 0.25},
            },
            "right": {
                "request_id": 11,
                "found": False,
                "corners": [],
                "status": "checkerboard: not found",
                "metadata": {"timestamp_s": now_s - 0.20},
            },
        },
        latest_calibration_preview_images={},
        calibration_status_label=_Text("자동 촬영 대기: 새 pose를 기다리는 중"),
        calib_min_images_spin=_Value(20),
    )
    controller = CalibrationWorkflowController(panel)
    panel._calibration_workflow_controller_owner = controller
    return controller


def test_capture_snapshot_combines_detection_run_and_persisted_set(
    tmp_path: Path,
) -> None:
    root = tmp_path / "2026-08-29_17-00-00"
    _image(root / "left" / "left_001.png")
    _image(root / "right" / "right_001.png")
    _metadata(root, "001", {"stereo_pair_saved": True})
    controller = _controller(root)

    controller._capture_feedback_begin(
        root=root,
        capture_id="001",
        origin="manual",
    )
    controller._capture_feedback_complete(
        capture_id="001",
        origin="manual",
        saved_sides=("left", "right"),
        stereo_pair_saved=True,
    )
    snapshot = controller.workstation_calibration_capture_snapshot()

    assert snapshot["schema"] == "workstation-calibration-capture-status-v1"
    assert snapshot["capture_set_id"] == root.name
    assert snapshot["detection"]["left"]["state"] == "FOUND"
    assert snapshot["detection"]["left"]["corner_count"] == 80
    assert snapshot["detection"]["right"]["state"] == "NOT_FOUND"
    assert snapshot["run"] == {
        "inspection_request_count": 1,
        "stereo_saved_count": 1,
        "intrinsics_only_saved_count": 0,
        "failed_count": 0,
        "inflight_count": 0,
    }
    assert snapshot["capture_set"]["left_image_count"] == 1
    assert snapshot["capture_set"]["right_image_count"] == 1
    assert snapshot["capture_set"]["stereo_candidate_count"] == 1
    assert snapshot["minimum_images_per_side"] == 20
    assert snapshot["last"]["message"].startswith("자동 촬영 대기")


def test_capture_snapshot_marks_old_detection_stale(tmp_path: Path) -> None:
    controller = _controller(tmp_path / "capture")
    controller.panel.calibration_preview_detection_results["left"]["metadata"] = {
        "timestamp_s": time.monotonic() - 2.5
    }

    snapshot = controller.workstation_calibration_capture_snapshot()

    assert snapshot["detection"]["left"]["state"] == "STALE"
    assert snapshot["detection"]["left"]["age_s"] >= 2.0


def test_capture_run_counters_reset_only_when_capture_root_changes(
    tmp_path: Path,
) -> None:
    first = tmp_path / "first"
    second = tmp_path / "second"
    controller = _controller(first)
    controller._capture_feedback_begin(root=first, capture_id="001", origin="auto")
    controller._capture_feedback_complete(
        capture_id="001",
        origin="auto",
        saved_sides=("left",),
        stereo_pair_saved=False,
    )

    first_snapshot = controller.workstation_calibration_capture_snapshot()
    assert first_snapshot["run"]["inspection_request_count"] == 1
    assert first_snapshot["run"]["intrinsics_only_saved_count"] == 1

    controller.panel._calibration_dirs = lambda: (
        second,
        second / "left",
        second / "right",
    )
    second_snapshot = controller.workstation_calibration_capture_snapshot()

    assert second_snapshot["run"]["inspection_request_count"] == 0
    assert second_snapshot["run"]["intrinsics_only_saved_count"] == 0


def test_start_auto_capture_resets_run_feedback_but_preserves_capture_set(
    tmp_path: Path,
) -> None:
    _app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    root = tmp_path / "AutoCapture"
    _image(root / "left" / "left_001.png")
    _image(root / "right" / "right_001.png")
    _metadata(root, "001", {"stereo_pair_saved": True})
    controller = _controller(root)
    controller._capture_feedback_begin(root=root, capture_id="001", origin="auto")
    controller._capture_feedback_complete(
        capture_id="001",
        origin="auto",
        saved_sides=("left", "right"),
        stereo_pair_saved=True,
    )
    controller._auto_capture_saved_pairs = 9
    controller.panel.checkerboard_calibration_thread = None
    controller.panel.calibration_auto_interval_spin = QtWidgets.QDoubleSpinBox()
    controller.panel.calibration_auto_interval_spin.setRange(0.1, 10.0)
    controller.panel.calibration_auto_interval_spin.setValue(1.0)
    controller.panel.capture_calibration_auto_button = QtWidgets.QPushButton()
    controller.panel.calibration_status_label = QtWidgets.QLabel()

    assert controller.start_auto_calibration_capture() is True
    snapshot = controller.workstation_calibration_capture_snapshot()

    assert snapshot["run"] == {
        "inspection_request_count": 0,
        "stereo_saved_count": 0,
        "intrinsics_only_saved_count": 0,
        "failed_count": 0,
        "inflight_count": 0,
    }
    assert snapshot["last"]["state"] == "WAITING"
    assert snapshot["last"]["origin"] == "auto"
    assert snapshot["capture_set"]["left_image_count"] == 1
    assert snapshot["capture_set"]["right_image_count"] == 1
    assert snapshot["capture_set"]["stereo_candidate_count"] == 1
    assert controller._auto_capture_saved_pairs == 0
    controller.stop_auto_calibration_capture()


def test_rejected_auto_restart_does_not_reset_visible_run(tmp_path: Path) -> None:
    root = tmp_path / "AutoCapture"
    controller = _controller(root)
    controller._capture_feedback_begin(root=root, capture_id="001", origin="auto")
    controller._capture_feedback_complete(
        capture_id="001",
        origin="auto",
        saved_sides=("left",),
        stereo_pair_saved=False,
    )
    controller.panel.checkerboard_calibration_thread = object()
    controller.panel.calibration_status_label = QtWidgets.QLabel()

    assert controller.start_auto_calibration_capture() is False
    snapshot = controller.workstation_calibration_capture_snapshot()

    assert snapshot["run"]["inspection_request_count"] == 1
    assert snapshot["run"]["intrinsics_only_saved_count"] == 1


def test_capture_waiting_status_does_not_increment_run_counter(
    tmp_path: Path,
) -> None:
    controller = _controller(tmp_path / "capture")

    snapshot = controller.workstation_calibration_capture_snapshot()

    assert snapshot["run"]["inspection_request_count"] == 0
    assert snapshot["run"]["failed_count"] == 0


def test_capture_snapshot_is_public_backend_and_evidence_contract() -> None:
    capture_status = {
        "schema": "workstation-calibration-capture-status-v1",
        "capture_set_id": "set-a",
    }
    panel = SimpleNamespace(
        workstation_calibration_capture_snapshot=lambda: dict(capture_status),
    )
    backend = StandalonePanelBackend(panel)

    assert backend.workstation_calibration_capture_snapshot() == capture_status
    assert EvidenceAdapter(panel).calibration()["capture_status"] == capture_status
    assert missing_backend_facades(StandalonePanelBackend) == ()


class _WorkspacePanel(QtCore.QObject):
    calibration_preview_changed = QtCore.pyqtSignal(object)
    calibration_capture_event = QtCore.pyqtSignal(object)
    calibration_diagnostics_changed = QtCore.pyqtSignal(object)

    def __init__(self) -> None:
        super().__init__()
        self.settings = SimpleNamespace()


class _WorkspaceBackend:
    def __init__(self, snapshot: dict[str, object]) -> None:
        self.panel = _WorkspacePanel()
        self.snapshot = snapshot

    def workstation_calibration_capture_snapshot(self) -> dict[str, object]:
        return dict(self.snapshot)


def _capture_snapshot_for_ui() -> dict[str, object]:
    return {
        "schema": "workstation-calibration-capture-status-v1",
        "capture_root": "/tmp/set-a",
        "capture_set_id": "set-a",
        "auto_capture_active": True,
        "detection": {
            "left": {
                "state": "FOUND",
                "corner_count": 80,
                "age_s": 0.25,
                "detail": "checkerboard: 80 corners",
            },
            "right": {
                "state": "NOT_FOUND",
                "corner_count": 0,
                "age_s": 0.20,
                "detail": "checkerboard: not found",
            },
        },
        "run": {
            "inspection_request_count": 14,
            "stereo_saved_count": 11,
            "intrinsics_only_saved_count": 2,
            "failed_count": 1,
            "inflight_count": 0,
        },
        "capture_set": {
            "state": "READY",
            "left_image_count": 13,
            "right_image_count": 13,
            "stereo_candidate_count": 11,
            "left_intrinsics_only_count": 2,
            "right_intrinsics_only_count": 2,
            "evidence_warning_pair_count": 0,
            "error": "",
        },
        "minimum_images_per_side": 20,
        "last": {
            "state": "WAITING",
            "capture_id": "014",
            "origin": "auto",
            "message": "자동 촬영 대기: 중복 자세 — 위치·거리·기울기를 변경하세요",
        },
    }


def test_primary_capture_tab_shows_overlay_and_source_backed_counts() -> None:
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    backend = _WorkspaceBackend(_capture_snapshot_for_ui())
    workspace = CalibrationWorkspace(backend, lambda: None, lambda _command: True)
    workspace.resize(1000, 700)
    workspace.show()
    app.processEvents()

    workspace.apply_calibration_capture_snapshot(backend.snapshot)

    assert set(workspace.simple_calibration_previews) == {"left", "right"}
    assert workspace.simple_calibration_preview_splitter.isVisible()
    assert workspace.simple_calibration_board_indicators["left"].state.text() == "인식 80"
    assert workspace.simple_calibration_board_indicators["right"].state.text() == "미인식"
    assert workspace.simple_calibration_run_status.text() == (
        "이번 촬영: 검사 14 · Stereo 저장 11 · K,D 전용 2 · 실패 1"
    )
    assert "L 13/20" in workspace.simple_calibration_set_status.text()
    assert "Stereo 저장 후보 11" in workspace.simple_calibration_set_status.text()
    assert "중복 자세" in workspace.simple_calibration_capture_status.toolTip()

    overlay = QtGui.QImage(640, 360, QtGui.QImage.Format_RGB888)
    overlay.fill(QtGui.QColor("red"))
    backend.panel.calibration_preview_changed.emit(
        {"camera": "left", "image": overlay, "kind": "overlay"}
    )
    app.processEvents()
    assert workspace.simple_calibration_previews["left"]._image is not None
    assert (
        workspace.simple_calibration_previews["left"]._image.pixelColor(0, 0)
        == QtGui.QColor("red")
    )

    workspace.deleteLater()
    backend.panel.deleteLater()
    app.processEvents()


def test_primary_capture_feedback_keeps_long_status_in_tooltip() -> None:
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    snapshot = _capture_snapshot_for_ui()
    long_message = "자동 촬영 대기: " + "매우 긴 복구 설명 " * 30
    snapshot["last"] = {"message": long_message}
    backend = _WorkspaceBackend(snapshot)
    workspace = CalibrationWorkspace(backend, lambda: None, lambda _command: True)
    workspace.resize(720, 600)
    workspace.show()
    app.processEvents()

    workspace.apply_calibration_capture_snapshot(snapshot)

    assert workspace.simple_calibration_capture_status.toolTip() == long_message.strip()
    assert workspace.simple_calibration_capture_status.wordWrap() is False
    assert len(workspace.simple_calibration_capture_status.text()) < len(long_message)

    workspace.deleteLater()
    backend.panel.deleteLater()
    app.processEvents()


def test_primary_capture_controls_and_feedback_precede_preview_at_1366x768() -> None:
    from Workstation.main_window import WorkstationMainWindow
    from Workstation.tests.workstation_test_support import _FakeBackend

    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    window = WorkstationMainWindow(backend=_FakeBackend())
    window.resize(1366, 768)
    window.show()
    window.navigation.select_context("tasks.calibration")
    app.processEvents()

    scroll = window.workspace_host.currentWidget()
    viewport_bottom = scroll.viewport().mapTo(
        window,
        scroll.viewport().rect().bottomLeft(),
    ).y()
    capture_button = window.calibration.simple_calibration_buttons[
        "calibration.capture_images"
    ][0]
    for widget in (
        capture_button,
        window.calibration.simple_calibration_run_status,
        window.calibration.simple_calibration_set_status,
        window.calibration.simple_calibration_capture_status,
    ):
        assert widget.mapTo(window, widget.rect().bottomLeft()).y() <= viewport_bottom
    last_bottom = window.calibration.simple_calibration_capture_status.mapTo(
        window,
        window.calibration.simple_calibration_capture_status.rect().bottomLeft(),
    ).y()
    preview_top = window.calibration.simple_calibration_preview_splitter.mapTo(
        window,
        window.calibration.simple_calibration_preview_splitter.rect().topLeft(),
    ).y()
    assert preview_top > last_bottom
    assert all(
        preview.focusPolicy() == QtCore.Qt.NoFocus
        for preview in window.calibration.simple_calibration_previews.values()
    )

    window.deleteLater()
    app.processEvents()
