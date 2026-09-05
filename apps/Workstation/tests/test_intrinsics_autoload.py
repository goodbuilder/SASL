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
    load_intrinsics_json,
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
    storage_root.mkdir(parents=True)
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
    immutable = Path(panel._active_intrinsics_profile_path)
    assert immutable.parent == storage_root / "profiles" / "intrinsics"
    assert load_intrinsics_json(immutable) == expected
    assert str(immutable) in panel.zoom_intrinsics_path_label.text()
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
    immutable = Path(panel._active_intrinsics_profile_path)
    assert immutable.parent == storage_root / "profiles" / "intrinsics"
    assert load_intrinsics_json(immutable) == expected
    assert str(immutable) in panel.zoom_intrinsics_path_label.text()


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
    storage_root = app_root / "calibration" / "session-a"
    storage_root.mkdir(parents=True)
    settings_path = tmp_path / "settings.json"
    save_settings(
        AppSettings(
            calibration_root="calibration/session-a",
            calibration_validation_root="calibration/session-a",
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
    first._measurement_session_state = "OPEN"
    first._measurement_session_id = "20260822_160000_000000001"
    first.recording_session_id = first._measurement_session_id
    second = None
    try:
        first.load_intrinsics()
        persisted_path = load_settings(settings_path).intrinsics_path
        assert not Path(persisted_path).is_absolute()
        assert persisted_path.startswith("calibration/imported/")
        imported_path = app_root / persisted_path
        assert imported_path.read_bytes() == exact_path.read_bytes()

        second = standalone_gui.WirisStereoDistancePanel(
            app_root=app_root,
            settings_path=settings_path,
        )
        original_auto_load(second)

        assert second.active_intrinsics == expected
        immutable = Path(second._active_intrinsics_profile_path)
        assert immutable.parent == storage_root / "profiles" / "intrinsics"
        assert load_intrinsics_json(immutable) == expected
        assert str(immutable) in second.zoom_intrinsics_path_label.text()
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
    storage_root = tmp_path / "data-1" / "calibration"
    previous_path = tmp_path / "external" / "previous.json"
    _save_profile(previous_path, _intrinsics_result(focal_px=900.0), zoom_level=4)
    panel, auto_load = autoload_panel_factory(
        AppSettings(
            intrinsics_path=str(previous_path),
            calibration_root="calibration",
            calibration_validation_root="calibration",
            calibration_zoom_level=4,
            auto_load_intrinsics=True,
        )
    )
    panel.last_calibration_result = _intrinsics_result(focal_px=1500.0)
    panel.last_calibration_zoom_level = 4

    panel.save_intrinsics()

    zoom_output = storage_root / "intrinsics_zoom_04.json"
    assert zoom_output.is_file()
    persisted = load_settings(panel.settings_path).intrinsics_path
    assert persisted.startswith("calibration/profiles/intrinsics/intrinsics-")
    assert persisted.endswith(".json")

    panel.last_calibration_result = None
    panel.last_calibration_zoom_level = None
    panel.active_intrinsics = None
    panel._active_intrinsics_profile_path = ""
    auto_load()

    assert panel.active_intrinsics is not None
    assert panel._active_intrinsics_profile_path == str(
        panel.app_root / load_settings(panel.settings_path).intrinsics_path
    )


def test_source_owned_default_calibration_saves_and_imports_portable_profile(
    autoload_panel_factory,
    tmp_path: Path,
) -> None:
    source_root = tmp_path / "workstation-source" / "calibration"
    panel, _auto_load = autoload_panel_factory(
        AppSettings(
            calibration_root="calibration",
            calibration_validation_root="calibration",
            calibration_zoom_level=4,
        )
    )
    panel.default_calibration_root = source_root
    panel._calibration_session_root = None
    panel._calibration_session_roots = {}
    panel.calib_dir_edit.setText("calibration")
    panel.last_calibration_result = _intrinsics_result(focal_px=1500.0)
    panel.last_calibration_zoom_level = 4

    panel.save_intrinsics()

    assert (source_root / "intrinsics_zoom_04.json").is_file()
    assert (source_root / "profiles" / "intrinsics").is_dir()
    persisted = load_settings(panel.settings_path)
    assert persisted.calibration_root == "calibration"
    assert persisted.intrinsics_path.startswith("calibration/imported/")
    assert (panel.app_root / persisted.intrinsics_path).is_file()


def test_save_intrinsics_rejects_external_calibration_root_without_writing(
    autoload_panel_factory,
    tmp_path: Path,
) -> None:
    external_root = tmp_path / "external-calibration"
    panel, _auto_load = autoload_panel_factory(
        AppSettings(
            calibration_root=str(external_root),
            calibration_validation_root="calibration",
            calibration_zoom_level=4,
        )
    )
    panel.last_calibration_result = _intrinsics_result(focal_px=1500.0)
    panel.last_calibration_zoom_level = 4

    panel.save_intrinsics()

    assert not external_root.exists()
    assert "outside the portable data root" in panel.calibration_status_label.text()


def test_save_intrinsics_keeps_active_matrix_and_provenance_until_apply(
    autoload_panel_factory,
    tmp_path: Path,
) -> None:
    panel, _auto_load = autoload_panel_factory(
        AppSettings(
            calibration_root="calibration",
            calibration_validation_root="calibration",
            calibration_zoom_level=4,
        )
    )
    previous = _intrinsics_result(focal_px=800.0)
    previous_path = tmp_path / "active-intrinsics.json"
    _save_profile(previous_path, previous, zoom_level=4)
    previous_hash = intrinsics_content_sha256(previous)
    panel.active_intrinsics = previous
    panel._active_intrinsics_profile_path = str(previous_path)
    panel._active_intrinsics_profile_hash = previous_hash
    panel._active_intrinsics_source_zoom_level = 4
    panel.last_calibration_result = _intrinsics_result(focal_px=1500.0)
    panel.last_calibration_zoom_level = 4

    panel.save_intrinsics()

    assert panel.active_intrinsics == previous
    assert panel._active_intrinsics_profile_path == str(previous_path)
    assert panel._active_intrinsics_profile_hash == previous_hash
    assert panel._active_intrinsics_source_zoom_level == 4
    candidate_path = Path(panel._candidate_intrinsics_profile_path)
    assert candidate_path.is_file()
    assert candidate_path != previous_path
    assert load_intrinsics_json(candidate_path) == panel.last_calibration_result


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
    panel._candidate_intrinsics_profile_path = "memory:previous-candidate"
    monkeypatch.setattr(panel, "apply_intrinsics_to_distance", lambda: False)

    auto_load()

    assert panel.last_calibration_result == previous
    assert panel.last_calibration_zoom_level == 4
    assert panel.active_intrinsics == previous
    assert panel._active_intrinsics_profile_path == str(previous_path)
    assert panel._active_intrinsics_profile_hash == intrinsics_content_sha256(previous)
    assert panel._candidate_intrinsics_profile_path == "memory:previous-candidate"


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
