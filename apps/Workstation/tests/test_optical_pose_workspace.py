from __future__ import annotations

from types import SimpleNamespace

import pytest
from PyQt5 import QtCore, QtGui, QtWidgets

from Workstation.annotation_image_view import FrozenAnnotationPayload, SourcePixel
from Workstation.evidence import EvidenceAdapter
from Workstation.status_model import StatusModel
from Workstation.workspaces import AlignmentWorkspace, OpticalPosePane, RecordingWorkspace


def _app() -> QtWidgets.QApplication:
    return QtWidgets.QApplication.instance() or QtWidgets.QApplication([])


def test_optical_pose_defaults_rotation_dof_and_keeps_position_fixed() -> None:
    app = _app()
    pane = OpticalPosePane()

    assert pane.dof_payload() == {
        "roll": True,
        "pitch": True,
        "yaw": False,
        "north": False,
        "east": False,
        "down": False,
    }
    assert not pane.dof_checkboxes["yaw"].isEnabled()
    assert "NO FROZEN BURST" in pane.frozen_preview_labels["left"].text()
    assert "NO FROZEN BURST" in pane.frozen_preview_labels["right"].text()
    assert pane.apply_payload() is None

    pane.deleteLater()
    app.processEvents()


def test_horizon_disables_yaw_and_manual_mountain_restores_it() -> None:
    app = _app()
    pane = OpticalPosePane()

    pane.manual_mode_buttons["ridge"].click()
    assert pane.annotation_mode_combo.currentData() == "ridge"
    assert pane.dof_checkboxes["yaw"].isEnabled()
    assert pane.dof_payload()["yaw"] is True

    pane.manual_mode_buttons["horizon"].click()
    assert pane.annotation_mode_combo.currentData() == "horizon"
    assert not pane.dof_checkboxes["yaw"].isEnabled()
    assert pane.dof_payload()["yaw"] is False

    # The automatic actions remain available for both manual annotation modes.
    assert pane.solver_buttons["alignment.pose.solve_horizon"].text() == "Solve Horizon Roll/Pitch"
    assert pane.solver_buttons["alignment.pose.solve_gnss_mountain"].text() == "Solve GNSS + Mountain R,t"
    annotation_layout = pane.annotation_clear_button.parentWidget().layout().itemAt(0).layout()
    assert annotation_layout.indexOf(pane.capture_frozen_burst_button) == annotation_layout.indexOf(
        pane.annotation_clear_button
    ) + 1
    assert "alignment.pose.import_terrain" not in pane.solver_buttons
    assert "alignment.pose.solve_terrain" not in pane.solver_buttons
    candidates_group = pane.findChild(QtWidgets.QGroupBox, "opticalPoseCandidatesGroup")
    method_tabs = pane.findChild(QtWidgets.QTabWidget, "opticalPoseMethodTabs")
    assert candidates_group is not None
    assert method_tabs is not None
    assert [method_tabs.tabText(index) for index in range(method_tabs.count())] == [
        "Horizon R/P",
        "Mountain + GNSS",
        "Drone RTK Yaw",
        "Existing calibration",
        "Combine candidates",
    ]
    assert pane.layout().indexOf(candidates_group) > pane.layout().indexOf(method_tabs.parentWidget())

    pane.set_candidates(
        [
            {
                "candidate_id": "gnss-mountain-1",
                "method": "gnss-constrained-mountain-uv",
                "observable_dof": {
                    "roll_deg": True,
                    "pitch_deg": True,
                    "yaw_deg": True,
                    "north_m": True,
                    "east_m": True,
                    "down_m": True,
                },
            }
        ]
    )
    assert pane.dof_payload() == {
        "roll": True,
        "pitch": True,
        "yaw": True,
        "north": True,
        "east": True,
        "down": True,
    }

    pane.deleteLater()
    app.processEvents()


def test_existing_calibration_is_reference_and_combine_requires_explicit_sources() -> None:
    app = _app()
    pane = OpticalPosePane()

    existing_text = pane.method_tabs.widget(3).findChildren(QtWidgets.QLabel)[0].text()
    assert "Reference only" in existing_text
    assert "Calibration" in existing_text

    combine_button = pane.solver_buttons["alignment.pose.solve_combined"]
    assert combine_button.isEnabled() is False
    assert pane.combine_candidate_ids() == ()

    pane.set_candidates(
        [
            {
                "candidate_id": "horizon-1",
                "method": "flat-horizon-roll-pitch",
                "observable_dof": {"roll_deg": True, "pitch_deg": True},
            },
            {
                "candidate_id": "drone-1",
                "method": "drone-rtk-target-yaw",
                "observable_dof": {"yaw_deg": True},
            },
        ]
    )
    assert pane.combine_candidate_list.count() == 2
    assert combine_button.isEnabled() is False

    pane.combine_candidate_list.item(0).setCheckState(QtCore.Qt.Checked)
    assert pane.combine_candidate_ids() == ("horizon-1",)
    assert combine_button.isEnabled() is False

    pane.combine_candidate_list.item(1).setCheckState(QtCore.Qt.Checked)
    assert pane.combine_candidate_ids() == ("horizon-1", "drone-1")
    assert combine_button.isEnabled() is True
    assert pane.command_payload("alignment.pose.solve_combined")["candidate_ids"] == [
        "horizon-1",
        "drone-1",
    ]

    pane.deleteLater()
    app.processEvents()


def test_low_quality_candidate_remains_selectable_and_emits_explicit_payload() -> None:
    app = _app()
    pane = OpticalPosePane()
    emitted: list[tuple[str, object]] = []
    pane.command_requested.connect(
        lambda command_id, payload: emitted.append((command_id, payload))
    )

    pane.set_candidates(
        [
            {
                "candidate_id": "pose-low-quality",
                "method": "terrain-pnp",
                "quality": "LOW",
                "residual_px": 99.0,
                "observable_dof": {"roll": True, "pitch": True, "yaw": True},
            }
        ]
    )

    assert pane.candidate_table.item(0, 0).data(QtCore.Qt.UserRole) == "pose-low-quality"
    assert pane.selected_candidate_id() == "pose-low-quality"
    assert pane.apply_button.isEnabled()
    assert pane.apply_payload() == {
        "candidate_id": "pose-low-quality",
        "dof": {
            "roll": True,
            "pitch": True,
            "yaw": False,
            "north": False,
            "east": False,
            "down": False,
        },
    }

    pane.apply_button.click()
    assert emitted == [
        (
            "alignment.pose.apply_candidate",
            pane.apply_payload(),
        )
    ]

    pane.deleteLater()
    app.processEvents()


def test_candidate_refresh_preserves_selection_by_candidate_id_after_reorder() -> None:
    app = _app()
    pane = OpticalPosePane()
    first = {
        "candidate_id": "pose-a",
        "method": "horizon",
        "quality": "GOOD",
    }
    second = {
        "candidate_id": "pose-b",
        "method": "drone-rtk-yaw",
        "quality": "LOW",
    }

    pane.set_candidates([first, second])
    pane.candidate_table.selectRow(1)
    pane.candidate_table.setCurrentCell(1, 0)
    assert pane.selected_candidate_id() == "pose-b"

    pane.set_candidates([second, first])

    assert pane.selected_candidate_id() == "pose-b"
    assert pane.candidate_table.currentRow() == 0
    assert all(
        pane.candidate_table.item(0, column).data(QtCore.Qt.UserRole) == "pose-b"
        for column in range(pane.candidate_table.columnCount())
    )

    pane.deleteLater()
    app.processEvents()


def test_alignment_isolates_optical_pose_page_and_preserves_physical_index() -> None:
    app = _app()
    backend = SimpleNamespace(panel=SimpleNamespace())
    workspace = AlignmentWorkspace(
        backend,
        StatusModel(),
        lambda: None,
        lambda _command_id: True,
    )

    assert [
        workspace.operation_tabs.tabText(index)
        for index in range(workspace.operation_tabs.count())
    ] == ["R,t methods"]
    assert workspace.operation_tabs.widget(0) is workspace.optical_pose_scroll
    assert workspace.optical_pose_scroll.widget() is workspace.optical_pose_pane
    assert (
        workspace.optical_pose_scroll.verticalScrollBarPolicy()
        == QtCore.Qt.ScrollBarAsNeeded
    )
    assert workspace.findChild(QtWidgets.QGroupBox, "alignmentDevicePrerequisites") is None
    assert workspace.optical_pose_pane.method_tabs.count() == 5
    workspace.select_tab_for_context("processing.alignment")
    assert workspace.operation_tabs.currentIndex() == 0

    workspace.deleteLater()
    app.processEvents()


def test_frozen_native_annotations_follow_capture_and_solver_commands() -> None:
    app = _app()
    pane = OpticalPosePane()
    left = QtGui.QImage(640, 480, QtGui.QImage.Format_RGB888)
    right = QtGui.QImage(800, 600, QtGui.QImage.Format_RGB888)
    left.fill(QtGui.QColor(QtCore.Qt.darkGray))
    right.fill(QtGui.QColor(QtCore.Qt.lightGray))
    pane.apply_snapshot(
        {
            "frozen_frames": {
                "left": {
                    "image": left,
                    "frame_id": "burst-007-left",
                    "sha256": "a" * 64,
                    "width": 640,
                    "height": 480,
                    "overlay_applied": False,
                },
                "right": {
                    "image": right,
                    "frame_id": "burst-007-right",
                    "sha256": "b" * 64,
                    "width": 800,
                    "height": 600,
                    "overlay_applied": False,
                },
            }
        }
    )
    pane.annotation_views["left"].add_source_point((20.5, 101.25), mode="horizon")
    pane.annotation_views["left"].add_source_point((610.5, 98.75), mode="horizon")
    pane.annotation_views["right"].add_source_point((30.0, 130.0), mode="horizon")
    pane.annotation_views["right"].add_source_point((760.0, 125.0), mode="horizon")

    emitted: list[tuple[str, dict[str, object]]] = []
    pane.command_requested.connect(
        lambda command_id, payload: emitted.append((command_id, payload))
    )
    pane.solver_buttons["alignment.pose.capture_burst"].click()
    pane.solver_buttons["alignment.pose.solve_horizon"].click()

    assert [command for command, _payload in emitted] == [
        "alignment.pose.capture_burst",
        "alignment.pose.solve_horizon",
    ]
    for command, payload in emitted:
        assert payload["command_id"] == command
        annotations = payload["annotations"]
        assert isinstance(annotations, dict)
        assert isinstance(annotations["left"], FrozenAnnotationPayload)
        assert isinstance(annotations["right"], FrozenAnnotationPayload)
        assert annotations["left"].frame_id == "burst-007-left"
        assert annotations["left"].horizon_line == (
            SourcePixel(20.5, 101.25),
            SourcePixel(610.5, 98.75),
        )

    # UI resizing changes only the fixed-16:9 center-crop transform, never native points.
    pane.annotation_views["left"].resize(333, 777)
    assert pane.annotation_payloads()["left"].horizon_line == (
        SourcePixel(20.5, 101.25),
        SourcePixel(610.5, 98.75),
    )

    ridge_index = pane.annotation_mode_combo.findData("ridge")
    pane.annotation_mode_combo.setCurrentIndex(ridge_index)
    pane.annotation_views["left"].add_source_point((10, 200))
    pane.annotation_views["left"].add_source_point((300, 180))
    pane.annotation_clear_button.click()
    exported = pane.annotation_payloads()["left"]
    assert exported.ridge_polyline == ()
    assert len(exported.horizon_line) == 2

    pane.deleteLater()
    app.processEvents()


def test_optical_pose_frozen_reference_columns_are_real_16_9_viewports() -> None:
    app = _app()
    pane = OpticalPosePane()
    pane.resize(1200, 800)
    pane.show()
    app.processEvents()

    for view in pane.annotation_views.values():
        assert view.width() > 0
        assert view.height() > 0
        assert view.width() / view.height() == pytest.approx(16.0 / 9.0, abs=0.02)
        assert view.height() == view.heightForWidth(view.width())

    assert pane.frozen_preview_layout.direction() == QtWidgets.QBoxLayout.TopToBottom
    assert pane.annotation_views["left"].width() > pane.width() / 2
    assert pane.annotation_views["right"].width() > pane.width() / 2

    pane.close()
    pane.deleteLater()
    app.processEvents()


def test_hr_purpose_event_selection_and_import_emit_typed_owner_payloads(monkeypatch) -> None:
    app = _app()
    monkeypatch.setattr(EvidenceAdapter, "session_catalog", lambda _self: [])
    panel = SimpleNamespace(
        pnp_photo_capture_records=[],
        pnp_photo_capture_thread=None,
        oz_photo_capture_records=[],
        oz_photo_capture_thread=None,
    )
    workspace = RecordingWorkspace(
        SimpleNamespace(panel=panel),
        StatusModel(),
        lambda: None,
    )
    emitted: list[tuple[str, dict[str, object]]] = []
    workspace.hr_command_requested.connect(
        lambda command_id, payload: emitted.append((command_id, payload))
    )

    workspace.pnp_capture_purpose_combo.setCurrentIndex(
        workspace.pnp_capture_purpose_combo.findData("post")
    )
    workspace.pnp_capture_button.click()
    assert emitted[-1] == (
        "recording.hr.capture",
        {
            "purpose": "post",
            "storage": "camera_internal_then_exact_file_import",
        },
    )

    workspace.oz_capture_button.click()
    assert emitted[-1] == (
        "recording.oz.capture",
        {
            "purpose": "post",
            "storage": "camera_internal_separate_from_rtsp",
            "capture_channel": "oz",
        },
    )

    workspace._sync_hr_capture_records(
        [
            {
                "capture_id": "hr-20260813-001",
                "purpose": "post",
                "state": "completed",
                "cameras": {
                    "left": {"state": "completed"},
                    "right": {"state": "completed"},
                },
            }
        ]
    )
    assert workspace._selected_hr_capture_id() == "hr-20260813-001"
    workspace.hr_import_right_button.click()
    command, payload = emitted[-1]
    assert command == "recording.hr.import"
    assert payload == {
        "capture_id": "hr-20260813-001",
        "purpose": "post",
        "side": "right",
        "file_selection": "owner_dialog_required",
        "byte_contract": "exact source bytes; owner computes SHA-256 and decoded resolution",
    }
    assert "path" not in payload
    assert "sha256" not in payload

    workspace.deleteLater()
    app.processEvents()
