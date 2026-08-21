from __future__ import annotations

import pytest
from pathlib import Path
from types import SimpleNamespace

from PyQt5 import QtGui, QtWidgets

from Workstation.calibration_controller import CalibrationWorkflowController
from Workstation.optical_pose_controller import OpticalPoseController


def _save_test_image(path: Path) -> None:
    image = QtGui.QImage(4, 3, QtGui.QImage.Format_RGB888)
    image.fill(64)
    assert image.save(str(path))


def _camera_records(tmp_path: Path, *, left_ok: bool = True, right_ok: bool = True) -> dict[str, object]:
    def _one(side: str, ok: bool) -> dict[str, object]:
        if not ok:
            return {"state": "timeout", "error": "ICPT completion timeout", "ftp_downloaded": False}
        path = tmp_path / f"{side}.png"
        _save_test_image(path)
        return {"state": "completed", "ftp_downloaded": True, "ftp_local_path": str(path)}

    return {"left": _one("left", left_ok), "right": _one("right", right_ok)}


# --- Calibration -------------------------------------------------------


def _calibration_panel(**overrides: object) -> SimpleNamespace:
    base = dict(
        calibration_wiris_ftp_thread=None,
        calibration_wiris_ftp_oz_thread=None,
        checkerboard_calibration_thread=None,
        settings=SimpleNamespace(wiris_ftp_enabled=True),
        calibration_status_label=QtWidgets.QLabel(),
        capture_calibration_button=QtWidgets.QPushButton(),
        run_calibration_button=QtWidgets.QPushButton(),
        _camera_photo_target_infos=lambda: (
            ("left", "10.0.0.1", 2240, 0.5),
            ("right", "10.0.0.2", 2240, 0.5),
        ),
        _calibration_dirs=lambda: (Path("/tmp/calib-root"), Path("/tmp/calib-root/left"), Path("/tmp/calib-root/right")),
        _log=lambda _message: None,
    )
    base.update(overrides)
    return SimpleNamespace(**base)


def test_calibration_wiris_ftp_capture_refuses_when_ftp_disabled() -> None:
    panel = _calibration_panel(settings=SimpleNamespace(wiris_ftp_enabled=False))
    controller = CalibrationWorkflowController(panel)

    assert controller.capture_calibration_images_wiris_ftp() is False
    assert panel.calibration_wiris_ftp_thread is None


def test_calibration_wiris_ftp_capture_refuses_when_already_busy() -> None:
    panel = _calibration_panel(calibration_wiris_ftp_thread=object())
    controller = CalibrationWorkflowController(panel)

    assert controller.capture_calibration_images_wiris_ftp() is False


def test_calibration_wiris_ftp_capture_refuses_without_both_camera_hosts() -> None:
    panel = _calibration_panel(_camera_photo_target_infos=lambda: (("left", "10.0.0.1", 2240, 0.5),))
    controller = CalibrationWorkflowController(panel)

    assert controller.capture_calibration_images_wiris_ftp() is False


def test_default_calibration_capture_uses_one_timestamped_session_root(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    import Workstation.calibration_controller as calibration_module

    default_root = tmp_path / "Workstation" / "calibration"
    monkeypatch.setattr(calibration_module, "DEFAULT_CALIBRATION_ROOT", str(default_root))
    logs: list[str] = []
    panel = SimpleNamespace(
        app_root=tmp_path / "runtime-data",
        calib_dir_edit=QtWidgets.QLineEdit(str(default_root)),
        _calibration_session_root=None,
        _log=logs.append,
    )
    controller = CalibrationWorkflowController(panel)

    first = controller._ensure_calibration_session_root()
    second = controller._ensure_calibration_session_root()

    assert first is not None
    assert first == second
    assert first.parent == default_root
    assert first.is_dir()
    assert len(first.name) == len("YYYY-MM-DD_HH-MM-SS")
    assert logs and str(first) in logs[-1]

    auto = controller._ensure_calibration_session_root("auto")
    assert auto is not None
    assert auto.name == "AutoCapture"
    assert auto.parent.name == "calib_capture"
    assert auto.parent.parent.name.startswith(first.name)
    assert auto.parent.parent.parent == default_root
    assert auto != first
    assert controller._ensure_calibration_session_root("auto") == auto


def test_calibration_wiris_ftp_completion_saves_via_existing_capture_path(tmp_path: Path) -> None:
    calls: list[dict[str, object]] = []
    panel = _calibration_panel(
        capture_calibration_images=lambda **kwargs: calls.append(kwargs) or True,
    )
    controller = CalibrationWorkflowController(panel)

    controller._handle_calibration_wiris_ftp_capture_completed(
        {"capture_id": "calib-001", "cameras": _camera_records(tmp_path)}
    )

    assert len(calls) == 1
    assert calls[0]["capture_origin"] == "ftp"
    frames = calls[0]["frames"]
    assert set(frames) == {"left", "right"}
    assert not frames["left"].isNull()
    assert not frames["right"].isNull()
    metadata = calls[0]["metadata_by_camera"]
    assert metadata["left"]["timestamp_reference"] == "unknown"
    assert metadata["right"]["timestamp_reference"] == "unknown"
    assert metadata["left"]["capture_timestamp_known"] is False
    assert "timestamp_s" not in metadata["left"]


def test_missing_capture_zoom_metadata_never_falls_back_to_current_spinner(
    tmp_path: Path,
) -> None:
    def fail_if_read() -> int:
        raise AssertionError("current UI zoom is not capture evidence")

    panel = _calibration_panel(
        zoom_level_spin=SimpleNamespace(value=fail_if_read),
    )
    controller = CalibrationWorkflowController(panel)

    zoom, detail = controller._capture_zoom_level_for_paths(
        tmp_path,
        (tmp_path / "left_001.png", tmp_path / "right_001.png"),
    )

    assert zoom is None
    assert "capture-time zoom readback is missing" in detail


def test_apply_calibration_without_capture_zoom_fails_before_reading_spinner() -> None:
    health: list[tuple[str, str]] = []

    def fail_if_read() -> int:
        raise AssertionError("current UI zoom is not capture evidence")

    panel = _calibration_panel(
        checkerboard_validation_result=SimpleNamespace(extrinsics=None),
        checkerboard_validation_result_mode="pixhawk",
        checkerboard_validation_status_label=QtWidgets.QLabel(),
        last_calibration_zoom_level=None,
        zoom_level_spin=SimpleNamespace(value=fail_if_read),
        _set_health_label=lambda _label, text, level: health.append(
            (str(text), str(level))
        ),
    )
    controller = CalibrationWorkflowController(panel)
    controller._calibration_quality_allows_action = lambda *_args, **_kwargs: True  # type: ignore[method-assign]

    controller.apply_checkerboard_validation_result()

    assert health[-1][1] == "bad"
    assert "readback" in health[-1][0]


def test_calibration_wiris_ftp_completion_with_one_camera_failed_does_not_save(tmp_path: Path) -> None:
    calls: list[dict[str, object]] = []
    panel = _calibration_panel(
        capture_calibration_images=lambda **kwargs: calls.append(kwargs) or True,
    )
    controller = CalibrationWorkflowController(panel)

    controller._handle_calibration_wiris_ftp_capture_completed(
        {"capture_id": "calib-002", "cameras": _camera_records(tmp_path, right_ok=False)}
    )

    assert calls == []
    assert "failed" in panel.calibration_status_label.text().lower()
    assert panel.capture_calibration_button.isEnabled()
    assert panel.run_calibration_button.isEnabled()


def test_calibration_wiris_ftp_failure_surfaces_ftp_error_detail(tmp_path: Path) -> None:
    panel = _calibration_panel(
        capture_calibration_images=lambda **kwargs: True,
    )
    controller = CalibrationWorkflowController(panel)
    left_path = tmp_path / "left.png"
    _save_test_image(left_path)

    controller._handle_calibration_wiris_ftp_capture_completed(
        {
            "capture_id": "calib-ftp-error",
            "cameras": {
                "left": {
                    "state": "completed",
                    "stored_on_camera": True,
                    "ftp_downloaded": True,
                    "ftp_local_path": str(left_path),
                },
                "right": {
                    "state": "completed",
                    "stored_on_camera": True,
                    "ftp_downloaded": False,
                    "ftp_error": "timed out",
                },
            },
        }
    )

    assert "timed out" in panel.calibration_status_label.text()


def test_calibration_wiris_ftp_worker_cleared_on_finish() -> None:
    panel = _calibration_panel(
        calibration_wiris_ftp_thread=object(),
        calibration_wiris_ftp_worker=object(),
    )
    controller = CalibrationWorkflowController(panel)

    controller._clear_calibration_wiris_ftp_capture_worker()

    assert panel.calibration_wiris_ftp_thread is None
    assert panel.calibration_wiris_ftp_worker is None


# --- Calibration OZ variant -----------------------------------------------


def test_calibration_wiris_ftp_oz_capture_refuses_when_hr_capture_busy() -> None:
    panel = _calibration_panel(calibration_wiris_ftp_thread=object())
    controller = CalibrationWorkflowController(panel)

    assert controller.capture_calibration_images_wiris_ftp_oz() is False


def test_calibration_wiris_ftp_hr_capture_refuses_when_oz_capture_busy() -> None:
    panel = _calibration_panel(calibration_wiris_ftp_oz_thread=object())
    controller = CalibrationWorkflowController(panel)

    assert controller.capture_calibration_images_wiris_ftp() is False


def test_calibration_wiris_ftp_oz_completion_saves_via_existing_capture_path(tmp_path: Path) -> None:
    calls: list[dict[str, object]] = []
    panel = _calibration_panel(
        capture_calibration_images=lambda **kwargs: calls.append(kwargs) or True,
    )
    controller = CalibrationWorkflowController(panel)

    controller._handle_calibration_wiris_ftp_oz_capture_completed(
        {"capture_id": "calib-oz-001", "cameras": _camera_records(tmp_path)}
    )

    assert len(calls) == 1
    assert calls[0]["capture_origin"] == "ftp_oz"
    metadata = calls[0]["metadata_by_camera"]
    assert metadata["left"]["timestamp_reference"] == "unknown"
    assert metadata["right"]["timestamp_reference"] == "unknown"


def test_calibration_wiris_ftp_oz_worker_cleared_on_finish() -> None:
    panel = _calibration_panel(
        calibration_wiris_ftp_oz_thread=object(),
        calibration_wiris_ftp_oz_worker=object(),
    )
    controller = CalibrationWorkflowController(panel)

    controller._clear_calibration_wiris_ftp_oz_capture_worker()

    assert panel.calibration_wiris_ftp_oz_thread is None
    assert panel.calibration_wiris_ftp_oz_worker is None


def test_calibration_ftp_oz_sync_pair_keeps_capture_clock_unknown() -> None:
    calls: list[dict[str, object]] = []
    frames = {
        "left": QtGui.QImage(4, 3, QtGui.QImage.Format_RGB888),
        "right": QtGui.QImage(4, 3, QtGui.QImage.Format_RGB888),
    }
    metadata = {
        side: {
            "timestamp_reference": "unknown",
            "capture_timestamp_known": False,
            "ftp_capture_id": "oz-sync-001",
        }
        for side in ("left", "right")
    }
    panel = _calibration_panel(
        checkerboard_calibration_thread=SimpleNamespace(
            finished=SimpleNamespace(connect=lambda _callback: None)
        ),
        capture_calibration_images=lambda **kwargs: calls.append(kwargs) or True,
        _process_next_ftp_oz_sync_pair=lambda: None,
    )
    controller = CalibrationWorkflowController(panel)
    controller._pending_ftp_oz_sync_frames = [("oz-sync-001", frames, metadata)]

    controller._process_next_ftp_oz_sync_pair()

    assert calls[0]["capture_origin"] == "ftp_oz_sync"
    assert calls[0]["metadata_by_camera"] is metadata
    assert calls[0]["metadata_by_camera"]["left"]["timestamp_reference"] == "unknown"


# --- Alignment (Optical Pose) -------------------------------------------


def _optical_pose_panel(**overrides: object) -> SimpleNamespace:
    base = dict(
        optical_pose_wiris_ftp_thread=None,
        optical_pose_wiris_ftp_oz_thread=None,
        settings=SimpleNamespace(wiris_ftp_enabled=True),
        recording_session_id="session-align",
        _camera_photo_target_infos=lambda: (
            ("left", "10.0.0.1", 2240, 0.5),
            ("right", "10.0.0.2", 2240, 0.5),
        ),
        _recording_session_dir=lambda stamp: Path("/tmp") / stamp,
        _log=lambda _message: None,
    )
    base.update(overrides)
    return SimpleNamespace(**base)


def test_optical_pose_wiris_ftp_capture_refuses_when_ftp_disabled() -> None:
    panel = _optical_pose_panel(settings=SimpleNamespace(wiris_ftp_enabled=False))
    controller = OpticalPoseController(panel)

    with pytest.raises(RuntimeError):
        controller.capture_workstation_optical_pose_burst_wiris_ftp()


def test_optical_pose_wiris_ftp_capture_refuses_when_already_busy() -> None:
    panel = _optical_pose_panel(optical_pose_wiris_ftp_thread=object())
    controller = OpticalPoseController(panel)

    with pytest.raises(RuntimeError):
        controller.capture_workstation_optical_pose_burst_wiris_ftp()


def test_optical_pose_wiris_ftp_capture_refuses_without_both_camera_hosts() -> None:
    panel = _optical_pose_panel(_camera_photo_target_infos=lambda: ())
    controller = OpticalPoseController(panel)

    with pytest.raises(RuntimeError):
        controller.capture_workstation_optical_pose_burst_wiris_ftp()


def test_optical_pose_wiris_ftp_completion_imports_burst_with_no_file_dialog(
    monkeypatch: pytest.MonkeyPatch, tmp_path: Path
) -> None:
    def _forbidden_dialog(*_args: object, **_kwargs: object) -> None:
        raise AssertionError("QFileDialog must not be used by the WIRIS FTP import path")

    monkeypatch.setattr(QtWidgets.QFileDialog, "getOpenFileName", _forbidden_dialog)

    panel = _optical_pose_panel()
    controller = OpticalPoseController(panel)
    imported: list[dict[str, object]] = []
    controller.import_workstation_optical_pose_burst = lambda payload=None: (
        imported.append(dict(payload or {})) or {"candidates": []}
    )
    controller._pending_wiris_ftp_payload = {
        "dof": {"roll": True},
        "_ftp_request_gnss_snapshot": {
            "left": {"latitude_deg": 37.0, "longitude_deg": 127.0, "altitude_m": 100.0},
            "right": {"latitude_deg": 37.00001, "longitude_deg": 127.00002, "altitude_m": 100.4},
            "drone": None,
        },
    }

    controller._handle_optical_pose_wiris_ftp_capture_completed(
        {"capture_id": "align-001", "cameras": _camera_records(tmp_path)}
    )

    assert len(imported) == 1
    assert imported[0]["dof"] == {"roll": True}
    assert imported[0]["_ftp_request_gnss_snapshot"]["left"]["latitude_deg"] == 37.0
    sources = imported[0]["sources"]
    assert set(sources) == {"left", "right"}
    assert Path(sources["left"]["path"]).is_file()
    assert Path(sources["right"]["path"]).is_file()
    assert sources["left"]["metadata"]["timestamp_reference"] == "unknown"
    assert sources["left"]["metadata"]["capture_timestamp_known"] is False
    assert "zoom_index" not in sources["left"]


def test_optical_pose_wiris_ftp_completion_with_one_camera_failed_does_not_import(
    tmp_path: Path,
) -> None:
    panel = _optical_pose_panel()
    controller = OpticalPoseController(panel)
    imported: list[dict[str, object]] = []
    controller.import_workstation_optical_pose_burst = lambda payload=None: imported.append(
        dict(payload or {})
    )

    controller._handle_optical_pose_wiris_ftp_capture_completed(
        {"capture_id": "align-002", "cameras": _camera_records(tmp_path, left_ok=False)}
    )

    assert imported == []


def test_optical_pose_wiris_ftp_capture_failed_does_not_raise() -> None:
    panel = _optical_pose_panel()
    controller = OpticalPoseController(panel)

    controller._handle_optical_pose_wiris_ftp_capture_failed("boom")


def test_optical_pose_wiris_ftp_worker_cleared_on_finish() -> None:
    panel = _optical_pose_panel(
        optical_pose_wiris_ftp_thread=object(),
        optical_pose_wiris_ftp_worker=object(),
    )
    controller = OpticalPoseController(panel)

    controller._clear_optical_pose_wiris_ftp_capture_worker()

    assert panel.optical_pose_wiris_ftp_thread is None
    assert panel.optical_pose_wiris_ftp_worker is None


# --- Alignment OZ variant --------------------------------------------------


def test_optical_pose_wiris_ftp_oz_capture_refuses_when_hr_capture_busy() -> None:
    panel = _optical_pose_panel(optical_pose_wiris_ftp_thread=object())
    controller = OpticalPoseController(panel)

    with pytest.raises(RuntimeError):
        controller.capture_workstation_optical_pose_burst_wiris_ftp_oz()


def test_optical_pose_wiris_ftp_hr_capture_refuses_when_oz_capture_busy() -> None:
    panel = _optical_pose_panel(optical_pose_wiris_ftp_oz_thread=object())
    controller = OpticalPoseController(panel)

    with pytest.raises(RuntimeError):
        controller.capture_workstation_optical_pose_burst_wiris_ftp()


def test_optical_pose_wiris_ftp_oz_completion_imports_burst(tmp_path: Path) -> None:
    panel = _optical_pose_panel()
    controller = OpticalPoseController(panel)
    imported: list[dict[str, object]] = []
    controller.import_workstation_optical_pose_burst = lambda payload=None: (
        imported.append(dict(payload or {})) or {"candidates": []}
    )

    controller._handle_optical_pose_wiris_ftp_oz_capture_completed(
        {"capture_id": "align-oz-001", "cameras": _camera_records(tmp_path)}
    )

    assert len(imported) == 1
    sources = imported[0]["sources"]
    assert Path(sources["left"]["path"]).is_file()
    assert Path(sources["right"]["path"]).is_file()
    assert sources["right"]["metadata"]["timestamp_reference"] == "unknown"


def test_optical_pose_wiris_ftp_oz_worker_cleared_on_finish() -> None:
    panel = _optical_pose_panel(
        optical_pose_wiris_ftp_oz_thread=object(),
        optical_pose_wiris_ftp_oz_worker=object(),
    )
    controller = OpticalPoseController(panel)

    controller._clear_optical_pose_wiris_ftp_oz_capture_worker()

    assert panel.optical_pose_wiris_ftp_oz_thread is None
    assert panel.optical_pose_wiris_ftp_oz_worker is None


# --- Real panel binding ---------------------------------------------------
#
# The controller code above calls self.panel._camera_photo_target_infos()
# and self.panel._wiris_ftp_worker_kwargs(...): both are only reachable at
# runtime if their names are registered in a *_METHODS tuple that
# bind_controller_delegates() attaches to WirisStereoDistancePanel. A
# SimpleNamespace-stubbed panel (as used above) can't catch a missing
# registration, so assert directly against the real panel class.


def test_new_wiris_ftp_methods_are_bound_on_the_real_panel() -> None:
    from Workstation import standalone_gui

    for name in (
        "_camera_photo_target_infos",
        "_wiris_ftp_worker_kwargs",
        "capture_calibration_images_wiris_ftp",
        "_handle_calibration_wiris_ftp_capture_completed",
        "_handle_calibration_wiris_ftp_capture_failed",
        "_clear_calibration_wiris_ftp_capture_worker",
        "capture_workstation_optical_pose_burst_wiris_ftp",
        "_handle_optical_pose_wiris_ftp_capture_completed",
        "_handle_optical_pose_wiris_ftp_capture_failed",
        "_clear_optical_pose_wiris_ftp_capture_worker",
        "capture_calibration_images_wiris_ftp_oz",
        "_handle_calibration_wiris_ftp_oz_capture_completed",
        "_handle_calibration_wiris_ftp_oz_capture_failed",
        "_clear_calibration_wiris_ftp_oz_capture_worker",
        "capture_workstation_optical_pose_burst_wiris_ftp_oz",
        "_handle_optical_pose_wiris_ftp_oz_capture_completed",
        "_handle_optical_pose_wiris_ftp_oz_capture_failed",
        "_clear_optical_pose_wiris_ftp_oz_capture_worker",
    ):
        assert name in standalone_gui.WirisStereoDistancePanel.__dict__, (
            f"{name} is not bound onto WirisStereoDistancePanel -- add it to the "
            "relevant *_METHODS tuple"
        )
