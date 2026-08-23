from __future__ import annotations

from types import SimpleNamespace

from PyQt5 import QtWidgets

from Workstation.workspaces import CalibrationWorkspace


GATED_COMMANDS = (
    "calibration.apply_bundle",
    "calibration.validation.apply",
)

EVIDENCE_SAVE_COMMANDS = (
    "calibration.save_intrinsics",
    "calibration.save_extrinsics",
    "calibration.validation.save",
)


class _QualityBackend:
    def __init__(self, snapshot: dict[str, object]) -> None:
        self.panel = SimpleNamespace(settings=SimpleNamespace())
        self.snapshot = snapshot

    def calibration_quality_snapshot(self) -> dict[str, object]:
        return dict(self.snapshot)


def _workspace(backend: object) -> CalibrationWorkspace:
    workspace = CalibrationWorkspace(backend, lambda: None, lambda _command: True)
    workspace.checkerboard_validation_outer_group.setChecked(True)
    return workspace


def _dispose(workspace: CalibrationWorkspace) -> None:
    workspace.deleteLater()
    application = QtWidgets.QApplication.instance()
    assert application is not None
    application.processEvents()


def test_calibration_quality_pass_displays_source_metrics_and_coverage_grid() -> None:
    backend = _QualityBackend(
        {
            "state": "PASS",
            "source": "calibration-controller/quality-v1",
            "effective_pose_count": 24,
            "coverage": {
                "fraction": 0.82,
                "occupied_cells": 9,
                "required_cells": 7,
            },
            "coverage_grid_3x3": [[3, 2, 1], [4, 8, 5], [1, 2, 3]],
            "principal_point_offsets_px": {
                "left": {"cx": 1.25, "cy": -0.75},
                "right": {"cx": 0.5, "cy": -0.25},
            },
            "subset_stability_px": {"cx": 0.12, "cy": 0.15, "focal": 0.8},
            "uncertainty": {
                "left_principal_stddev_px": [1.2, 0.8],
                "left_max_per_view_error_px": 0.42,
            },
            "reasons": (),
        }
    )
    workspace = _workspace(backend)

    assert workspace.calibration_quality_values["state"].value.text() == "PASS"
    assert workspace.calibration_quality_values["effective_pose_count"].value.text() == "24"
    assert "fraction=0.8200" in workspace.calibration_quality_values["coverage"].value.text()
    assert "left=" in workspace.calibration_quality_values["principal_offsets"].value.text()
    assert "focal=0.8000" in workspace.calibration_quality_values["subset_stability"].value.text()
    assert "left_max_per_view_error_px=0.4200" in (
        workspace.calibration_quality_values["uncertainty"].value.text()
    )
    assert workspace.calibration_center_coverage_table.item(1, 1).text() == "8"
    assert "calibration-controller/quality-v1" in (
        workspace.calibration_quality_values["coverage"].toolTip()
    )
    assert all(workspace.operation_buttons[command].isEnabled() for command in GATED_COMMANDS)
    assert workspace.calibration_quality_warning_checkbox.isHidden()

    _dispose(workspace)


def test_calibration_quality_warning_requires_explicit_confirmation_and_resets() -> None:
    backend = _QualityBackend(
        {
            "state": "WARNING",
            "source": "calibration-controller/quality-v1",
            "quality_id": "quality-a",
            "effective_pose_count": 13,
            "reasons": ({"code": "coverage_low", "message": "bottom-right sparse"},),
            "coverage_grid_3x3": {
                "top_left": {"count": 2, "covered": True},
                "center": {"count": 5, "covered": True},
                "bottom_right": {"count": 0, "covered": False},
            },
        }
    )
    workspace = _workspace(backend)

    assert workspace.calibration_quality_values["state"].value.text() == "WARNING"
    assert "coverage_low: bottom-right sparse" in (
        workspace.calibration_quality_values["reasons"].value.text()
    )
    assert not workspace.calibration_quality_warning_checkbox.isHidden()
    assert all(not workspace.operation_buttons[command].isEnabled() for command in GATED_COMMANDS)

    assert workspace.confirm_calibration_quality_warning(True) is True
    assert workspace.calibration_quality_warning_checkbox.isChecked()
    assert all(workspace.operation_buttons[command].isEnabled() for command in GATED_COMMANDS)
    assert "acknowledged" in workspace.calibration_quality_gate_label.text()

    backend.snapshot.update(
        quality_id="quality-b",
        reasons=("subset focal stability near limit",),
    )
    workspace.refresh_calibration_quality()
    assert not workspace.calibration_quality_warning_checkbox.isChecked()
    assert all(not workspace.operation_buttons[command].isEnabled() for command in GATED_COMMANDS)

    _dispose(workspace)


def test_calibration_quality_fail_allows_attempt_save_but_blocks_apply() -> None:
    backend = _QualityBackend(
        {
            "quality_state": "FAIL",
            "producer": "calibration-quality-gate",
            "quality_reasons": (
                "effective_pose_count_below_minimum",
                "principal_offset_out_of_range",
            ),
            "metrics": {"effective_pose_count": 5},
        }
    )
    workspace = _workspace(backend)

    assert workspace.calibration_quality_values["state"].value.text() == "FAIL"
    assert all(not workspace.operation_buttons[command].isEnabled() for command in GATED_COMMANDS)
    assert all(workspace.operation_buttons[command].isEnabled() for command in EVIDENCE_SAVE_COMMANDS)
    assert "Save preserves an attempt" in workspace.calibration_quality_gate_label.text()
    assert not workspace.operation_buttons["calibration.run_intrinsic"].isEnabled()
    assert not workspace.operation_buttons["calibration.run_extrinsics"].isEnabled()
    assert "Select a Photo set" in workspace.operation_buttons[
        "calibration.run_intrinsic"
    ].toolTip()
    assert "Apply is blocked" in workspace.calibration_quality_gate_label.text()
    assert "principal_offset_out_of_range" in workspace.operation_buttons[
        "calibration.apply_bundle"
    ].toolTip()
    assert workspace.confirm_calibration_quality_warning(True) is False

    _dispose(workspace)


def test_legacy_calibration_without_quality_is_unverified_not_pass() -> None:
    backend = SimpleNamespace(panel=SimpleNamespace(settings=SimpleNamespace()))
    workspace = _workspace(backend)

    assert workspace.calibration_quality_values["state"].value.text() == "UNVERIFIED"
    assert "legacy" in workspace.calibration_quality_values["reasons"].value.text().lower()
    assert "not presented as PASS" in workspace.calibration_quality_gate_label.text()
    assert all(workspace.operation_buttons[command].isEnabled() for command in GATED_COMMANDS)
    assert workspace.calibration_quality_action_allowed("calibration.apply_bundle") is True
    assert workspace.calibration_quality_warning_checkbox.isHidden()

    _dispose(workspace)
