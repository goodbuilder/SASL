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


def test_calibration_wiris_ftp_oz_worker_cleared_on_finish() -> None:
    panel = _calibration_panel(
        calibration_wiris_ftp_oz_thread=object(),
        calibration_wiris_ftp_oz_worker=object(),
    )
    controller = CalibrationWorkflowController(panel)

    controller._clear_calibration_wiris_ftp_oz_capture_worker()

    assert panel.calibration_wiris_ftp_oz_thread is None
    assert panel.calibration_wiris_ftp_oz_worker is None


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
    controller._pending_wiris_ftp_payload = {"dof": {"roll": True}}

    controller._handle_optical_pose_wiris_ftp_capture_completed(
        {"capture_id": "align-001", "cameras": _camera_records(tmp_path)}
    )

    assert len(imported) == 1
    assert imported[0]["dof"] == {"roll": True}
    sources = imported[0]["sources"]
    assert set(sources) == {"left", "right"}
    assert Path(sources["left"]["path"]).is_file()
    assert Path(sources["right"]["path"]).is_file()


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
