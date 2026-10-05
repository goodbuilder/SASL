from __future__ import annotations

import hashlib
import json
from pathlib import Path
from types import SimpleNamespace

import pytest
from PyQt5 import QtCore, QtWidgets

from sasl_common.lifecycle import LifecycleCoordinator
from Workstation.runtime_paths import (
    import_workstation_asset_file,
    portable_workstation_data_path_text,
    workstation_settings_portability_errors,
)
from Workstation.workstation_core.settings import AppSettings, save_settings


def test_portable_path_normalizes_relative_and_owned_absolute_paths(tmp_path: Path) -> None:
    root = tmp_path / "data"
    root.mkdir()

    assert portable_workstation_data_path_text(
        "calibration/./session/intrinsics.json",
        data_root=root,
        field_name="intrinsics_path",
    ) == "calibration/session/intrinsics.json"
    assert portable_workstation_data_path_text(
        root / "models" / "yolov8m.pt",
        data_root=root,
        field_name="yolo_weights_path",
    ) == "models/yolov8m.pt"


@pytest.mark.parametrize(
    "value",
    (
        "/home/operator/models/yolo.pt",
        "../outside/model.pt",
        r"C:\Users\operator\model.pt",
    ),
)
def test_portable_path_rejects_machine_absolute_and_traversal(
    tmp_path: Path,
    value: str,
) -> None:
    with pytest.raises(ValueError, match="must"):
        portable_workstation_data_path_text(
            value,
            data_root=tmp_path / "data",
            field_name="yolo_weights_path",
        )


def test_portable_path_rejects_symlink_escape(tmp_path: Path) -> None:
    root = tmp_path / "data"
    external = tmp_path / "external"
    root.mkdir()
    external.mkdir()
    (root / "escape").symlink_to(external, target_is_directory=True)

    with pytest.raises(ValueError, match="SASL_WORKSTATION_DATA_ROOT"):
        portable_workstation_data_path_text(
            "escape/model.pt",
            data_root=root,
            field_name="yolo_weights_path",
        )


def test_external_asset_import_is_content_addressed_verified_and_idempotent(
    tmp_path: Path,
) -> None:
    root = tmp_path / "data"
    source = tmp_path / "downloads" / "profile.json"
    source.parent.mkdir()
    source.write_bytes(b'{"profile":"sentinel"}\n')
    digest = hashlib.sha256(source.read_bytes()).hexdigest()

    first = import_workstation_asset_file(source, category="calibration", data_root=root)
    second = import_workstation_asset_file(source, category="calibration", data_root=root)

    assert first == root / "calibration" / "imported" / digest / source.name
    assert second == first
    assert first.read_bytes() == source.read_bytes()
    assert portable_workstation_data_path_text(
        first,
        data_root=root,
        field_name="intrinsics_path",
    ) == f"calibration/imported/{digest}/{source.name}"


def test_existing_conflicting_import_target_fails_closed(tmp_path: Path) -> None:
    root = tmp_path / "data"
    source = tmp_path / "profile.json"
    source.write_bytes(b"approved")
    digest = hashlib.sha256(source.read_bytes()).hexdigest()
    target = root / "models" / "imported" / digest / source.name
    target.parent.mkdir(parents=True)
    target.write_bytes(b"tampered")

    with pytest.raises(ValueError, match="conflicts"):
        import_workstation_asset_file(source, category="models", data_root=root)


def test_settings_portability_checks_only_asset_path_fields(tmp_path: Path) -> None:
    settings = SimpleNamespace(
        intrinsics_path="calibration/intrinsics.json",
        yolo_weights_path="/home/operator/yolo.pt",
        calibration_root="calibration",
        calibration_validation_root="calibration",
        calibration_validation_json="",
        measurement_data_root="",
        gnss_base_serial_port="/dev/ttyACM0",
        wiris_ftp_remote_dir="/",
    )

    errors = workstation_settings_portability_errors(settings, data_root=tmp_path / "data")

    assert len(errors) == 1
    assert errors[0].startswith("yolo_weights_path")


def test_blank_measurement_root_is_a_valid_portable_default(tmp_path: Path) -> None:
    settings = SimpleNamespace(measurement_data_root="")

    assert workstation_settings_portability_errors(
        settings,
        data_root=tmp_path / "data",
    ) == []


def test_panel_settings_save_normalizes_owned_paths_and_rejects_external_root(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
    workstation_qapplication: object,
) -> None:
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
    data_root = tmp_path / "data"
    calibration_root = data_root / "calibration" / "session-a"
    model_path = data_root / "models" / "model.pt"
    calibration_root.mkdir(parents=True)
    model_path.parent.mkdir(parents=True)
    model_path.write_bytes(b"model")
    settings_path = data_root / "stereo_distance_settings.json"
    save_settings(
        AppSettings(
            intrinsics_path="calibration/session-a/intrinsics.json",
            yolo_weights_path="models/model.pt",
            calibration_root="calibration/session-a",
            calibration_validation_root="calibration/session-a",
            calibration_validation_json="calibration/session-a/validation.json",
        ),
        settings_path,
    )
    panel = standalone_gui.WirisStereoDistancePanel(
        app_root=data_root,
        settings_path=settings_path,
    )
    try:
        panel.weights_edit.setText(str(model_path))
        panel.calib_dir_edit.setText(str(calibration_root))
        panel.checkerboard_validation_root_edit.setText(str(calibration_root))
        panel.checkerboard_validation_json_edit.setText(
            str(calibration_root / "validation.json")
        )

        panel._save_current_settings(calibration_root / "intrinsics.json")

        payload = json.loads(settings_path.read_text(encoding="utf-8"))
        assert payload["intrinsics_path"] == "calibration/session-a/intrinsics.json"
        assert payload["yolo_weights_path"] == "models/model.pt"
        assert payload["calibration_root"] == "calibration/session-a"
        assert payload["calibration_validation_root"] == "calibration/session-a"
        assert payload["calibration_validation_json"] == (
            "calibration/session-a/validation.json"
        )
        assert str(tmp_path) not in settings_path.read_text(encoding="utf-8")
        original_bytes = settings_path.read_bytes()

        panel.calib_dir_edit.setText(str(tmp_path / "external-calibration"))
        with pytest.raises(ValueError, match="calibration_root must stay under"):
            panel._save_current_settings()
        assert settings_path.read_bytes() == original_bytes
    finally:
        panel._shutdown_coordinator = LifecycleCoordinator([])
        panel.close()
        panel.deleteLater()
        application = QtWidgets.QApplication.instance()
        if application is not None:
            application.sendPostedEvents(None, QtCore.QEvent.DeferredDelete)
            application.processEvents()
