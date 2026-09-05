from __future__ import annotations

from pathlib import Path
from types import SimpleNamespace

import pytest

from Workstation.calibration_controller import CalibrationWorkflowController
from Workstation.optical_pose_controller import OpticalPoseController
from Workstation.workstation_core.measurement_session import (
    MeasurementSession,
    verify_measurement_session,
)


class _TextValue:
    def __init__(self, value: str = "") -> None:
        self._value = value

    def text(self) -> str:
        return self._value


def _session_panel(tmp_path: Path, *, state: str = "OPEN") -> SimpleNamespace:
    session_id = "20260822_120102_000000001"
    return SimpleNamespace(
        app_root=tmp_path,
        calib_dir_edit=_TextValue(""),
        _measurement_session_hierarchy_enabled=True,
        _measurement_session_state=state,
        _measurement_session_id=session_id,
        recording_session_id=session_id,
        _measurement_session_dir=lambda selected: tmp_path / "recordings" / selected,
        _calibration_session_root=None,
        _calibration_session_roots={},
        _log=lambda _message: None,
    )


def test_calibration_run_is_created_inside_open_session(tmp_path: Path) -> None:
    panel = _session_panel(tmp_path)
    controller = CalibrationWorkflowController(panel)

    calibration_root = controller._ensure_calibration_session_root("manual")

    assert calibration_root is not None
    assert calibration_root.parent == (
        tmp_path / "recordings" / panel.recording_session_id / "calibration"
    )
    assert (calibration_root / "calibration_session.json").is_file()


def test_pre_session_calibration_is_allowed_but_alignment_still_requires_session(
    tmp_path: Path,
) -> None:
    panel = _session_panel(tmp_path, state="IDLE")

    calibration_root = CalibrationWorkflowController(panel)._ensure_calibration_session_root(
        "manual"
    )
    assert calibration_root == tmp_path / "calibration"
    assert (calibration_root / "calibration_session.json").is_file()
    with pytest.raises(RuntimeError, match="Open Session"):
        OpticalPoseController(panel)._require_open_session_id()


def test_sessionless_default_marker_uses_panel_owned_calibration_root(
    tmp_path: Path,
) -> None:
    default_root = tmp_path / "workstation-source" / "calibration"
    panel = SimpleNamespace(
        app_root=tmp_path / "data-root",
        default_calibration_root=default_root,
        calib_dir_edit=_TextValue("calibration"),
        _measurement_session_hierarchy_enabled=False,
        _calibration_session_root=None,
        _calibration_session_roots={},
        _log=lambda _message: None,
    )
    controller = CalibrationWorkflowController(panel)

    assert controller._calibration_dirs()[0] == default_root.resolve()
    created = controller._ensure_calibration_session_root("manual")

    assert created is not None
    assert created.parent == default_root.resolve()
    assert (created / "calibration_session.json").is_file()


def test_pre_session_loaded_intrinsics_are_not_blocked_by_session_archiving(
    tmp_path: Path,
) -> None:
    panel = _session_panel(tmp_path, state="IDLE")
    source = tmp_path / "external_intrinsics.json"
    source.write_bytes(b'{"schema":"fixture"}\n')

    assert CalibrationWorkflowController(panel)._archive_loaded_intrinsics(source) is None


def test_loaded_intrinsics_are_copied_into_session(tmp_path: Path) -> None:
    panel = _session_panel(tmp_path)
    source = tmp_path / "external_intrinsics.json"
    source.write_bytes(b'{"schema":"fixture"}\n')
    controller = CalibrationWorkflowController(panel)

    archived = controller._archive_loaded_intrinsics(source)

    assert archived is not None
    assert archived.read_bytes() == source.read_bytes()
    assert archived.parent.name == "loaded_intrinsics"
    assert archived.is_relative_to(tmp_path / "recordings" / panel.recording_session_id)


def test_session_manifest_inventories_calibration_as_shared_evidence(tmp_path: Path) -> None:
    panel = _session_panel(tmp_path)
    session_root = panel._measurement_session_dir(panel.recording_session_id)
    session = MeasurementSession.create(
        session_root,
        session_id=panel.recording_session_id,
    )
    controller = CalibrationWorkflowController(panel)
    calibration_root = controller._ensure_calibration_session_root("manual")
    assert calibration_root is not None
    (calibration_root / "intrinsics.json").write_bytes(b'{"schema":"fixture"}\n')

    manifest = session.seal()

    shared_paths = {row["relative_path"] for row in manifest["shared_artifacts"]}
    assert any(path.startswith("calibration/") for path in shared_paths)
    assert verify_measurement_session(session_root).valid


def test_calibration_cache_is_isolated_by_parent_session(tmp_path: Path) -> None:
    panel = _session_panel(tmp_path)
    controller = CalibrationWorkflowController(panel)
    first = controller._ensure_calibration_session_root("manual")
    assert first is not None

    panel._measurement_session_id = "20260822_140000_000000002"
    panel.recording_session_id = panel._measurement_session_id
    second = controller._ensure_calibration_session_root("manual")

    assert second is not None
    assert second != first
    assert second.is_relative_to(tmp_path / "recordings" / panel.recording_session_id)


def test_measurement_creation_readiness_is_not_owned_by_alignment_helper() -> None:
    """Geometry readiness is evaluated after child creation, not by the workspace."""

    from Workstation import workspaces

    assert not hasattr(workspaces, "_measurement_alignment_ready")
