from __future__ import annotations

from pathlib import Path
from types import SimpleNamespace

import pytest
from PyQt5 import QtCore, QtWidgets

from sasl_common.lifecycle import LifecycleCoordinator
from Workstation.workstation_core.calibration import (
    CameraIntrinsics,
    CheckerboardConfig,
    StereoIntrinsicsResult,
    intrinsics_content_sha256,
    save_intrinsics_json,
)
from Workstation.workstation_core.settings import AppSettings, load_settings, save_settings


def _intrinsics_result(*, focal_px: float) -> StereoIntrinsicsResult:
    camera = CameraIntrinsics(
        width=1920,
        height=1080,
        camera_matrix=[
            [focal_px, 0.0, 960.0],
            [0.0, focal_px + 1.0, 540.0],
            [0.0, 0.0, 1.0],
        ],
        distortion_coefficients=[0.0, 0.0, 0.0, 0.0, 0.0],
        reprojection_error_px=0.2,
        image_count=12,
    )
    return StereoIntrinsicsResult(
        left=camera,
        right=camera,
        board=CheckerboardConfig(inner_cols=9, inner_rows=6, square_size_m=0.025),
    )


def _save_profile(
    path: Path,
    result: StereoIntrinsicsResult,
    *,
    zoom_level: int | None,
) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    metadata = {} if zoom_level is None else {"zoom_level": zoom_level}
    save_intrinsics_json(path, result, metadata=metadata)


@pytest.fixture
def autoload_panel_factory(
    monkeypatch: pytest.MonkeyPatch,
    tmp_path: Path,
    workstation_qapplication: object,
):
    from Workstation import standalone_gui

    original_auto_load = standalone_gui.WirisStereoDistancePanel._auto_load_intrinsics
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
    panels: list[QtWidgets.QWidget] = []
    sequence = 0

    def create(settings: AppSettings):
        nonlocal sequence
        sequence += 1
        app_root = tmp_path / f"data-{sequence}"
        app_root.mkdir()
        settings_path = tmp_path / f"settings-{sequence}.json"
        save_settings(settings, settings_path)
        panel = standalone_gui.WirisStereoDistancePanel(
            app_root=app_root,
            settings_path=settings_path,
        )
        panels.append(panel)
        return panel, lambda: original_auto_load(panel)

    yield create

    for panel in panels:
        panel._shutdown_coordinator = LifecycleCoordinator([])
        panel.close()
        panel.deleteLater()
    application = QtWidgets.QApplication.instance()
    if application is not None:
        application.sendPostedEvents(None, QtCore.QEvent.DeferredDelete)
        application.processEvents()


def test_auto_load_prefers_exact_external_intrinsics_path(autoload_panel_factory, tmp_path: Path) -> None:
    storage_root = tmp_path / "storage"
    storage_root.mkdir()
    exact_path = tmp_path / "external" / "approved-intrinsics.json"
    expected = _intrinsics_result(focal_px=1100.0)
    _save_profile(exact_path, expected, zoom_level=4)
    panel, auto_load = autoload_panel_factory(
        AppSettings(
            intrinsics_path=str(exact_path),
            calibration_root=str(storage_root),
            calibration_zoom_level=4,
            auto_load_intrinsics=True,
        )
    )

    auto_load()

    assert panel.active_intrinsics == expected
    assert panel.last_calibration_zoom_level == 4
    assert panel._active_intrinsics_profile_path == str(exact_path)
    assert str(exact_path) in panel.zoom_intrinsics_path_label.text()
    assert intrinsics_content_sha256(panel.active_intrinsics) == intrinsics_content_sha256(expected)


def test_auto_load_rejects_existing_exact_profile_with_zoom_mismatch(
    autoload_panel_factory,
    tmp_path: Path,
) -> None:
    storage_root = tmp_path / "storage"
    exact_path = tmp_path / "external" / "wrong-zoom.json"
    _save_profile(exact_path, _intrinsics_result(focal_px=900.0), zoom_level=3)
    storage_profile = storage_root / "intrinsics_zoom_04.json"
    _save_profile(
        storage_profile,
        _intrinsics_result(focal_px=1400.0),
        zoom_level=4,
    )
    panel, auto_load = autoload_panel_factory(
        AppSettings(
            intrinsics_path=str(exact_path),
            calibration_root=str(storage_root),
            calibration_zoom_level=4,
            auto_load_intrinsics=True,
        )
    )

    auto_load()

    assert panel.active_intrinsics is None
    assert panel._active_intrinsics_profile_path == ""
    assert "zoom mismatch" in panel.calibration_status_label.text()
    assert str(exact_path) in panel.calibration_status_label.text()


def test_auto_load_does_not_replace_missing_explicit_path_with_storage_profile(
    autoload_panel_factory,
    tmp_path: Path,
) -> None:
    storage_root = tmp_path / "storage"
    exact_path = tmp_path / "external" / "missing.json"
    storage_profile = storage_root / "intrinsics_zoom_04.json"
    expected = _intrinsics_result(focal_px=1250.0)
    _save_profile(storage_profile, expected, zoom_level=4)
    panel, auto_load = autoload_panel_factory(
        AppSettings(
            intrinsics_path=str(exact_path),
            calibration_root=str(storage_root),
            calibration_zoom_level=4,
            auto_load_intrinsics=True,
        )
    )

    auto_load()

    assert panel.active_intrinsics is None
    assert panel._active_intrinsics_profile_path == ""
    assert "storage fallback is disabled" in panel.calibration_status_label.text()
    assert str(exact_path) in panel.calibration_status_label.text()


def test_auto_load_falls_back_from_missing_storage_alias_to_exact_zoom_file(
    autoload_panel_factory,
    tmp_path: Path,
) -> None:
    storage_root = tmp_path / "storage"
    exact_alias = storage_root / "intrinsics.json"
    storage_profile = storage_root / "intrinsics_zoom_04.json"
    expected = _intrinsics_result(focal_px=1250.0)
    _save_profile(storage_profile, expected, zoom_level=4)
    panel, auto_load = autoload_panel_factory(
        AppSettings(
            intrinsics_path=str(exact_alias),
            calibration_root=str(storage_root),
            calibration_zoom_level=4,
            auto_load_intrinsics=True,
        )
    )

    auto_load()

    assert panel.active_intrinsics == expected
    assert panel._active_intrinsics_profile_path == str(storage_profile)
    assert str(storage_profile) in panel.zoom_intrinsics_path_label.text()


def test_auto_load_rejects_malformed_exact_profile_without_storage_fallback(
    autoload_panel_factory,
    tmp_path: Path,
) -> None:
    storage_root = tmp_path / "storage"
    exact_path = tmp_path / "external" / "malformed.json"
    exact_path.parent.mkdir()
    exact_path.write_text("{not-json", encoding="utf-8")
    _save_profile(
        storage_root / "intrinsics_zoom_04.json",
        _intrinsics_result(focal_px=1400.0),
        zoom_level=4,
    )
    panel, auto_load = autoload_panel_factory(
        AppSettings(
            intrinsics_path=str(exact_path),
            calibration_root=str(storage_root),
            calibration_zoom_level=4,
            auto_load_intrinsics=True,
        )
    )

    auto_load()

    assert panel.active_intrinsics is None
    assert panel._active_intrinsics_profile_path == ""
    assert "invalid intrinsic profile" in panel.calibration_status_label.text()
    assert str(exact_path) in panel.calibration_status_label.text()


def test_auto_load_rejects_exact_profile_without_zoom_metadata(
    autoload_panel_factory,
    tmp_path: Path,
) -> None:
    storage_root = tmp_path / "storage"
    exact_path = tmp_path / "external" / "unknown-zoom.json"
    _save_profile(exact_path, _intrinsics_result(focal_px=1100.0), zoom_level=None)
    _save_profile(
        storage_root / "intrinsics_zoom_04.json",
        _intrinsics_result(focal_px=1400.0),
        zoom_level=4,
    )
    panel, auto_load = autoload_panel_factory(
        AppSettings(
            intrinsics_path=str(exact_path),
            calibration_root=str(storage_root),
            calibration_zoom_level=4,
            auto_load_intrinsics=True,
        )
    )

    auto_load()

    assert panel.active_intrinsics is None
    assert panel._active_intrinsics_profile_path == ""
    assert "metadata.zoom_level" in panel.calibration_status_label.text()


def test_manually_selected_external_profile_is_auto_loaded_after_restart(
    monkeypatch: pytest.MonkeyPatch,
    tmp_path: Path,
    workstation_qapplication: object,
) -> None:
    from Workstation import standalone_gui

    original_auto_load = standalone_gui.WirisStereoDistancePanel._auto_load_intrinsics
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
    app_root = tmp_path / "data"
    app_root.mkdir()
    storage_root = tmp_path / "storage"
    storage_root.mkdir()
    settings_path = tmp_path / "settings.json"
    save_settings(
        AppSettings(
            calibration_root=str(storage_root),
            calibration_zoom_level=4,
            auto_load_intrinsics=True,
        ),
        settings_path,
    )
    exact_path = tmp_path / "external" / "operator-selected.json"
    expected = _intrinsics_result(focal_px=1325.0)
    _save_profile(exact_path, expected, zoom_level=4)
    monkeypatch.setattr(
        standalone_gui.QtWidgets.QFileDialog,
        "getOpenFileName",
        lambda *_args, **_kwargs: (str(exact_path), "Intrinsics JSON (*.json)"),
    )
    first = standalone_gui.WirisStereoDistancePanel(
        app_root=app_root,
        settings_path=settings_path,
    )
    second = None
    try:
        first.load_intrinsics()
        assert load_settings(settings_path).intrinsics_path == str(exact_path)

        second = standalone_gui.WirisStereoDistancePanel(
            app_root=app_root,
            settings_path=settings_path,
        )
        original_auto_load(second)

        assert second.active_intrinsics == expected
        assert second._active_intrinsics_profile_path == str(exact_path)
        assert str(exact_path) in second.zoom_intrinsics_path_label.text()
    finally:
        for panel in (first, second):
            if panel is None:
                continue
            panel._shutdown_coordinator = LifecycleCoordinator([])
            panel.close()
            panel.deleteLater()
        application = QtWidgets.QApplication.instance()
        if application is not None:
            application.sendPostedEvents(None, QtCore.QEvent.DeferredDelete)
            application.processEvents()


def test_saving_new_calibration_replaces_previous_persisted_exact_profile(
    autoload_panel_factory,
    tmp_path: Path,
) -> None:
    storage_root = tmp_path / "storage"
    storage_root.mkdir()
    previous_path = tmp_path / "external" / "previous.json"
    _save_profile(previous_path, _intrinsics_result(focal_px=900.0), zoom_level=4)
    panel, auto_load = autoload_panel_factory(
        AppSettings(
            intrinsics_path=str(previous_path),
            calibration_root=str(storage_root),
            calibration_zoom_level=4,
            auto_load_intrinsics=True,
        )
    )
    panel.last_calibration_result = _intrinsics_result(focal_px=1500.0)
    panel.last_calibration_zoom_level = 4

    panel.save_intrinsics()

    zoom_output = storage_root / "intrinsics_zoom_04.json"
    assert zoom_output.is_file()
    assert load_settings(panel.settings_path).intrinsics_path == str(zoom_output)

    panel.last_calibration_result = None
    panel.last_calibration_zoom_level = None
    panel.active_intrinsics = None
    panel._active_intrinsics_profile_path = ""
    auto_load()

    assert panel.active_intrinsics is not None
    assert panel._active_intrinsics_profile_path == str(zoom_output)


def test_auto_load_restores_previous_runtime_state_when_apply_gate_rejects(
    autoload_panel_factory,
    monkeypatch: pytest.MonkeyPatch,
    tmp_path: Path,
) -> None:
    storage_root = tmp_path / "storage"
    storage_root.mkdir()
    exact_path = tmp_path / "external" / "candidate.json"
    _save_profile(exact_path, _intrinsics_result(focal_px=1500.0), zoom_level=4)
    panel, auto_load = autoload_panel_factory(
        AppSettings(
            intrinsics_path=str(exact_path),
            calibration_root=str(storage_root),
            calibration_zoom_level=4,
            auto_load_intrinsics=True,
        )
    )
    previous = _intrinsics_result(focal_px=800.0)
    previous_path = tmp_path / "previous-active.json"
    panel.last_calibration_result = previous
    panel.last_calibration_zoom_level = 4
    panel.active_intrinsics = previous
    panel._active_intrinsics_profile_path = str(previous_path)
    panel._active_intrinsics_profile_hash = intrinsics_content_sha256(previous)
    monkeypatch.setattr(panel, "apply_intrinsics_to_distance", lambda: False)

    auto_load()

    assert panel.last_calibration_result == previous
    assert panel.last_calibration_zoom_level == 4
    assert panel.active_intrinsics == previous
    assert panel._active_intrinsics_profile_path == str(previous_path)
    assert panel._active_intrinsics_profile_hash == intrinsics_content_sha256(previous)


def test_auto_load_contains_unexpected_profile_resolution_exception(
    autoload_panel_factory,
    monkeypatch: pytest.MonkeyPatch,
    tmp_path: Path,
) -> None:
    storage_root = tmp_path / "storage"
    storage_root.mkdir()
    exact_path = tmp_path / "external" / "candidate.json"
    panel, auto_load = autoload_panel_factory(
        AppSettings(
            intrinsics_path=str(exact_path),
            calibration_root=str(storage_root),
            calibration_zoom_level=4,
            auto_load_intrinsics=True,
        )
    )

    def fail_resolution(_zoom_level: int):
        raise OSError("simulated filesystem failure")

    monkeypatch.setattr(panel, "_auto_load_intrinsics_candidate", fail_resolution)

    auto_load()

    assert panel.active_intrinsics is None
    assert "unexpected intrinsic profile resolution failure" in panel.calibration_status_label.text()
    assert "simulated filesystem failure" in panel.calibration_status_label.text()


def test_manual_unknown_zoom_profile_warns_that_restart_auto_load_will_reject() -> None:
    from Workstation import standalone_gui

    messages: list[str] = []
    panel = SimpleNamespace(
        zoom_level_spin=SimpleNamespace(value=lambda: 4),
        calibration_status_label=SimpleNamespace(setText=messages.append),
        _log=messages.append,
    )

    allowed = standalone_gui.WirisStereoDistancePanel._confirm_intrinsics_profile_mismatch(
        panel,
        Path("unknown-zoom.json"),
        _intrinsics_result(focal_px=1100.0),
        None,
    )

    assert allowed is True
    assert any("다음 실행의 자동 로드는 차단" in message for message in messages)
