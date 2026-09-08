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


def test_optical_pose_has_no_editable_candidate_dof_controls() -> None:
    app = _app()
    pane = OpticalPosePane()

    assert not hasattr(pane, "dof_group")
    assert not hasattr(pane, "dof_checkboxes")
    assert not hasattr(pane, "dof_payload")
    assert pane.apply_button.text() == "Apply candidate"
    assert pane.reset_button.text() == "Reset Alignment"
    assert "NO FROZEN BURST" in pane.frozen_preview_labels["left"].text()
    assert "NO FROZEN BURST" in pane.frozen_preview_labels["right"].text()
    assert pane.apply_payload() is None

    pane.deleteLater()
    app.processEvents()


def test_import_pair_emits_only_paths_without_zoom_prompt(
    monkeypatch: pytest.MonkeyPatch,
    tmp_path,
) -> None:
    app = _app()
    pane = OpticalPosePane()
    left = tmp_path / "left.png"
    right = tmp_path / "right.png"
    left.write_bytes(b"left")
    right.write_bytes(b"right")
    selections = iter(((str(left), "Images"), (str(right), "Images")))
    monkeypatch.setattr(
        QtWidgets.QFileDialog,
        "getOpenFileName",
        lambda *_args, **_kwargs: next(selections),
    )

    def forbidden_zoom_dialog(*_args, **_kwargs):
        raise AssertionError("frozen-image Import must not request a zoom index")

    monkeypatch.setattr(QtWidgets.QInputDialog, "getInt", forbidden_zoom_dialog)
    captured: list[tuple[str, object]] = []
    pane.command_requested.connect(
        lambda command_id, payload: captured.append((command_id, payload))
    )

    pane._import_frozen_burst()

    assert captured == [
        (
            "alignment.pose.import_burst",
            {
                "sources": {
                    "left": {"path": str(left)},
                    "right": {"path": str(right)},
                }
            },
        )
    ]
    pane.deleteLater()
    app.processEvents()


def test_alignment_reset_is_an_explicit_owner_command() -> None:
    app = _app()
    pane = OpticalPosePane()
    captured: list[tuple[str, object]] = []
    pane.command_requested.connect(
        lambda command_id, payload: captured.append((command_id, payload))
    )

    pane.reset_button.click()

    assert captured == [("alignment.pose.reset", {})]
    pane.deleteLater()
    app.processEvents()


def test_annotation_modes_do_not_expose_candidate_apply_dof_controls() -> None:
    app = _app()
    pane = OpticalPosePane()

    pane.annotation_mode_combo.setCurrentIndex(pane.annotation_mode_combo.findData("ridge"))
    assert pane.annotation_mode_combo.currentData() == "ridge"

    pane.annotation_mode_combo.setCurrentIndex(pane.annotation_mode_combo.findData("horizon"))
    assert pane.annotation_mode_combo.currentData() == "horizon"

    # The automatic actions remain available for both manual annotation modes.
    assert pane.solver_buttons["alignment.pose.solve_horizon"].text() == "Solve Horizon Roll/Pitch"
    assert pane.solver_buttons["alignment.pose.solve_gnss_mountain"].text() == "Solve GNSS + Mountain R,t"
    assert pane.mountain_hide_auto_lines_checkbox.isChecked()
    assert pane.capture_frozen_burst_button.text() == "Capture Live Pair"
    assert pane.annotation_clear_button.text() == "Clear"
    assert pane.import_frozen_burst_button.text() == "Import Pair"
    assert pane.capture_frozen_burst_wiris_ftp_button.text() == "Capture HR Still (FTP)"
    assert pane.capture_frozen_burst_wiris_ftp_oz_button.text() == "Capture OZ Still (FTP)"
    assert "Roll/Pitch" in pane.annotation_mode_combo.toolTip()
    pane.annotation_mode_combo.setCurrentIndex(pane.annotation_mode_combo.findData("ridge"))
    assert "at least four paired points" in pane.annotation_mode_combo.toolTip()
    pane.annotation_mode_combo.setCurrentIndex(pane.annotation_mode_combo.findData("target"))
    assert "single-target" in pane.annotation_mode_combo.toolTip()
    pane.annotation_mode_combo.setCurrentIndex(pane.annotation_mode_combo.findData("horizon"))
    annotation_layout = pane.annotation_clear_button.parentWidget().layout().itemAt(0).layout()
    assert annotation_layout.indexOf(pane.capture_frozen_burst_button) == annotation_layout.indexOf(
        pane.annotation_clear_button
    ) + 1
    assert pane.solver_buttons["alignment.pose.import_terrain"].text() == "Import GeoTIFF"
    assert pane.solver_buttons["alignment.pose.solve_terrain"].text() == "Solve Terrain PnP"
    candidates_group = pane.findChild(QtWidgets.QGroupBox, "opticalPoseCandidatesGroup")
    method_tabs = pane.findChild(QtWidgets.QTabWidget, "alignmentOperationTabs")
    assert candidates_group is not None
    assert method_tabs is not None
    assert [method_tabs.tabText(index) for index in range(method_tabs.count())] == [
        "Horizon",
        "Mountain + GNSS",
        "Terrain PnP",
        "Drone Rotation",
        "Drone Multi-view",
        "Combine",
    ]
    assert "alignment.pose.initialize_telemetry" not in pane.solver_buttons
    assert pane.solver_buttons["alignment.pose.solve_drone_rtk_multiview"].text() == (
        "Solve Drone RTK Multi-view R,t"
    )
    assert set(pane.method_information_labels) == {
        "Horizon",
        "Mountain + GNSS",
        "Terrain PnP",
        "Drone Rotation",
        "Drone Multi-view",
        "Combine",
    }
    assert candidates_group.parentWidget() is pane.candidate_review_widget
    assert method_tabs.parentWidget() is pane.method_column

    pane.deleteLater()
    app.processEvents()


def test_mountain_auto_point_exclusion_is_paired_and_sent_for_acceptance() -> None:
    app = _app()
    pane = OpticalPosePane()
    image = QtGui.QImage(640, 480, QtGui.QImage.Format_RGB888)
    image.fill(QtGui.QColor("#333333"))
    for side in ("left", "right"):
        pane.annotation_views[side].load_frozen_native_frame(
            image,
            frame_id=f"{side}-frame",
            image_sha256=("1" if side == "left" else "2") * 64,
        )
        pane.annotation_views[side].set_mode("ridge")
    suggestion = {
        "suggestion_id": "mountain-1",
        "mode": "mountain",
        "state": "LOW_CONFIDENCE",
        "eligible": False,
        "copy_eligible": True,
        "failure_reason": "match_confidence_below_acceptance_threshold",
        "sides": {
            "left": {
                "frame_id": "left-frame",
                "image_sha256": "1" * 64,
                "source_width_px": 640,
                "source_height_px": 480,
                "points": [(10.0, 20.0), (120.0, 210.0), (260.0, 55.0), (430.0, 300.0), (620.0, 110.0)],
                "point_confidences": [0.9, 0.8, 0.7, 0.6, 0.5],
            },
            "right": {
                "frame_id": "right-frame",
                "image_sha256": "2" * 64,
                "source_width_px": 640,
                "source_height_px": 480,
                "points": [(15.0, 18.0), (125.0, 208.0), (265.0, 53.0), (435.0, 298.0), (625.0, 108.0)],
                "point_confidences": [0.9, 0.8, 0.7, 0.6, 0.5],
            },
        },
    }
    pane.set_auto_suggestions([suggestion])

    assert pane.use_suggestion_button.isEnabled()
    assert "LOW_CONFIDENCE" in pane.auto_suggestion_status_label.text()
    assert "COPY READY" in pane.auto_suggestion_status_label.text()

    pane._on_suggestion_point_right_clicked("left", "ridge", 1)

    expected_left = tuple(
        SourcePixel(*point)
        for point in ((10.0, 20.0), (260.0, 55.0), (430.0, 300.0), (620.0, 110.0))
    )
    expected_right = tuple(
        SourcePixel(*point)
        for point in ((15.0, 18.0), (265.0, 53.0), (435.0, 298.0), (625.0, 108.0))
    )
    assert pane.annotation_views["left"].suggestion_points("ridge") == expected_left
    assert pane.annotation_views["right"].suggestion_points("ridge") == expected_right
    assert pane.command_payload("alignment.pose.use_suggestion")["excluded_point_indices"] == [1]
    assert "excluded pairs 1" in pane.auto_suggestion_metrics_label.text()

    # A periodic owner snapshot must not restore the operator-excluded pair.
    pane.set_auto_suggestions([suggestion])
    assert pane.annotation_views["left"].suggestion_points("ridge") == expected_left
    assert pane.use_suggestion_button.isEnabled()

    pane._on_suggestion_point_right_clicked("left", "ridge", 0)
    assert not pane.use_suggestion_button.isEnabled()
    assert "COPY BLOCKED" in pane.auto_suggestion_status_label.text()
    pane.deleteLater()
    app.processEvents()


def test_live_rt_replaces_reference_registration_and_combine_requires_explicit_sources() -> None:
    app = _app()
    pane = OpticalPosePane()

    assert "References" not in [pane.method_tabs.tabText(i) for i in range(pane.method_tabs.count())]
    assert not any("Register" in button.text() for button in pane.findChildren(QtWidgets.QPushButton))
    assert pane.live_rt_table.rowCount() == 2
    combine_buttons = {
        button.text(): button
        for button in pane.method_tabs.widget(5).findChildren(QtWidgets.QPushButton)
    }
    assert "Build combined candidate" in combine_buttons

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

    # Each checked candidate defaults to contributing every axis it
    # actually observed, and an axis it never observed is not offered.
    assert pane.combine_candidate_dofs() == {
        "horizon-1": ["roll_deg", "pitch_deg"],
        "drone-1": ["yaw_deg"],
    }
    assert pane.combine_dof_table.rowCount() == 2
    disabled_cell = pane.combine_dof_table.item(0, 3)  # horizon-1's Yaw column
    assert not (disabled_cell.flags() & QtCore.Qt.ItemIsUserCheckable)
    assert pane.command_payload("alignment.pose.solve_combined")["candidate_dofs"] == {
        "horizon-1": ["roll_deg", "pitch_deg"],
        "drone-1": ["yaw_deg"],
    }

    pane.deleteLater()
    app.processEvents()


def test_combine_overlapping_axes_block_build_until_unchecked() -> None:
    app = _app()
    pane = OpticalPosePane()

    pane.set_candidates(
        [
            {
                "candidate_id": "horizon-1",
                "method": "flat-horizon-roll-pitch",
                "observable_dof": {"roll_deg": True, "pitch_deg": True},
            },
            {
                "candidate_id": "checkerboard-1",
                "method": "reference/checkerboard",
                "observable_dof": {
                    "roll_deg": True,
                    "pitch_deg": True,
                    "yaw_deg": True,
                    "north_m": True,
                    "east_m": True,
                    "down_m": True,
                },
            },
        ]
    )
    combine_button = pane.solver_buttons["alignment.pose.solve_combined"]
    pane.combine_candidate_list.item(0).setCheckState(QtCore.Qt.Checked)
    pane.combine_candidate_list.item(1).setCheckState(QtCore.Qt.Checked)

    # Both candidates default to claiming Roll/Pitch -- overlap blocks the build.
    assert combine_button.isEnabled() is False
    assert "roll_deg" in pane.combine_candidate_status.text()

    # Uncheck Checkerboard's Roll and Pitch cells (row 1, columns 1 and 2),
    # leaving it contributing only translation -- resolves the overlap.
    pane.combine_dof_table.item(1, 1).setCheckState(QtCore.Qt.Unchecked)
    pane.combine_dof_table.item(1, 2).setCheckState(QtCore.Qt.Unchecked)
    pane.combine_dof_table.item(1, 3).setCheckState(QtCore.Qt.Unchecked)

    assert combine_button.isEnabled() is True
    assert pane.combine_candidate_dofs() == {
        "horizon-1": ["roll_deg", "pitch_deg"],
        "checkerboard-1": ["north_m", "east_m", "down_m"],
    }

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
                "optimized_dof": {
                    "roll_deg": True,
                    "pitch_deg": True,
                    "yaw_deg": True,
                    "north_m": True,
                    "east_m": True,
                    "down_m": True,
                },
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

    assert pane.candidate_table.item(0, 0).data(QtCore.Qt.UserRole) == "pose-low-quality"
    assert pane.selected_candidate_id() == "pose-low-quality"
    assert pane.apply_button.isEnabled()
    assert pane.apply_payload() == {"candidate_id": "pose-low-quality"}
    assert "Apply DOF=roll_deg,pitch_deg,yaw_deg,north_m,east_m,down_m" in (
        pane.candidate_status_label.text()
    )

    pane.apply_button.click()
    assert emitted == [
        (
            "alignment.pose.apply_candidate",
            pane.apply_payload(),
        )
    ]

    pane.deleteLater()
    app.processEvents()


def test_candidate_apply_is_blocked_when_an_owned_dof_is_unobservable() -> None:
    app = _app()
    pane = OpticalPosePane()
    pane.set_candidates(
        [
            {
                "candidate_id": "full-but-rank-deficient",
                "method": "drone-rtk-multiview-r-t",
                "optimized_dof": {
                    "roll_deg": True,
                    "pitch_deg": True,
                    "yaw_deg": True,
                    "north_m": True,
                    "east_m": True,
                    "down_m": True,
                },
                "observable_dof": {
                    "roll_deg": True,
                    "pitch_deg": True,
                    "yaw_deg": True,
                    "north_m": True,
                    "east_m": True,
                    "down_m": False,
                },
            }
        ]
    )

    assert not pane.apply_button.isEnabled()
    assert "BLOCKED unobservable=down_m" in pane.candidate_status_label.text()

    pane.deleteLater()
    app.processEvents()


def test_candidate_detail_panel_shows_provenance_and_warnings_on_selection() -> None:
    app = _app()
    pane = OpticalPosePane()

    pane.set_candidates(
        [
            {
                "candidate_id": "gnss-mountain-uv-1",
                "method": "gnss-constrained-mountain-uv",
                "input_hashes": {
                    "lever_arm_status": "UNMODELED",
                    "left_anchor_pose_sha256": "a" * 64,
                },
                "quality_metrics": {
                    "sampson_rms_normalized": 1.2345e-4,
                    "inlier_count": 8.0,
                    "outlier_count": 2.0,
                },
                "relative_pose_rdf": {
                    "rotation_right_from_left_rdf": [
                        [1.0, 0.0, 0.0],
                        [0.0, 1.0, 0.0],
                        [0.0, 0.0, 1.0],
                    ],
                    "translation_right_from_left_rdf_m": [0.1, -0.2, 0.3],
                    "right_origin_in_left_rdf_m": [0.1, -0.2, 0.3],
                },
                "warnings": ["lever arm UNMODELED: assumption, not a measurement"],
                "observable_dof": {"roll": True, "pitch": True, "yaw": True},
            }
        ]
    )

    text = pane.candidate_detail_text.toPlainText()
    assert "relative_frame: OpenCV optical RDF" in text
    assert "R_right_from_left_rdf:" in text
    assert "[+1.000000000, +0.000000000, +0.000000000]" in text
    assert "t_right_from_left_rdf_m: [+0.100000000, -0.200000000, +0.300000000]" in text
    assert "right_origin_in_left_rdf_m: [+0.100000000, -0.200000000, +0.300000000]" in text
    assert "lever_arm_status: UNMODELED" in text
    assert "left_anchor_pose_sha256: " + "a" * 64 in text
    assert "sampson_rms_normalized:" in text
    assert "inlier_count: 8" in text
    assert "lever arm UNMODELED" in text

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
    ] == [
        "Mountain + GNSS",
        "Baseline Yaw90",
        "Horizon",
        "Terrain PnP",
        "Drone Rotation",
        "Drone Multi-view",
        "Combine",
    ]
    assert workspace.operation_tabs is workspace.optical_pose_pane.method_tabs
    assert workspace.optical_pose_scroll is None
    assert not hasattr(workspace, "lrf_group")
    assert not hasattr(workspace, "lrf_indicators")
    assert workspace.findChild(QtWidgets.QGroupBox, "alignmentDevicePrerequisites") is None
    assert workspace.optical_pose_pane.method_tabs.count() == 7
    assert all(
        view.wheel_zoom_requires_control is False
        for view in workspace.optical_pose_pane.annotation_views.values()
    )
    workspace.select_tab_for_context("processing.alignment")
    assert workspace.operation_tabs.currentIndex() == 0
    assert workspace.operation_tabs.tabText(0) == "Mountain + GNSS"
    workspace.open_baseline_yaw90()
    assert workspace.operation_tabs.currentIndex() == 1
    assert workspace.operation_tabs.currentWidget() is workspace.baseline_yaw90_pane

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
        assert "dof" not in payload
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


def test_promoted_method_tabs_switch_annotation_mode_without_losing_points() -> None:
    app = _app()
    pane = OpticalPosePane()
    image = QtGui.QImage(640, 480, QtGui.QImage.Format_RGB888)
    image.fill(QtGui.QColor(QtCore.Qt.darkGray))
    pane.apply_snapshot(
        {
            "frozen_frames": {
                side: {
                    "image": image,
                    "frame_id": f"burst-{side}",
                    "sha256": ("a" if side == "left" else "b") * 64,
                    "width": 640,
                    "height": 480,
                }
                for side in ("left", "right")
            }
        }
    )
    pane.annotation_views["left"].add_source_point((20.0, 100.0), mode="horizon")
    pane.annotation_views["left"].add_source_point((600.0, 100.0), mode="horizon")
    commands: list[str] = []
    pane.command_requested.connect(lambda command, _payload: commands.append(command))

    pane.method_tabs.setCurrentIndex(pane.method_tabs.indexOf(pane.method_tabs.widget(1)))
    assert pane.method_tabs.currentIndex() == 1
    assert pane.annotation_mode_combo.currentData() == "ridge"
    assert len(pane.annotation_payloads()["left"].horizon_line) == 2

    pane.method_tabs.setCurrentIndex(3)
    assert pane.annotation_mode_combo.currentData() == "target"
    assert len(pane.annotation_payloads()["left"].horizon_line) == 2

    pane.method_tabs.setCurrentIndex(4)
    assert not pane.annotation_mode_combo.isEnabled()
    assert not pane.annotation_views["left"].isEnabled()
    pane.method_tabs.setCurrentIndex(0)
    assert pane.annotation_mode_combo.currentData() == "horizon"
    assert pane.annotation_views["left"].isEnabled()
    assert len(pane.annotation_payloads()["left"].horizon_line) == 2
    assert "L/R=2/0" in pane.method_information_labels["Horizon"].text()
    assert commands == []

    pane.deleteLater()
    app.processEvents()


def test_multiview_capture_keeps_target_marks_until_clear_current_annotation() -> None:
    app = _app()
    pane = OpticalPosePane()
    first_left = QtGui.QImage(640, 480, QtGui.QImage.Format_RGB888)
    first_right = QtGui.QImage(640, 480, QtGui.QImage.Format_RGB888)
    second_left = QtGui.QImage(640, 480, QtGui.QImage.Format_RGB888)
    second_right = QtGui.QImage(640, 480, QtGui.QImage.Format_RGB888)
    first_left.fill(QtGui.QColor("#333333"))
    first_right.fill(QtGui.QColor("#333333"))
    second_left.fill(QtGui.QColor("#444444"))
    second_right.fill(QtGui.QColor("#444444"))

    def snapshot(left: QtGui.QImage, right: QtGui.QImage, suffix: str) -> dict[str, object]:
        return {
            "frozen_frames": {
                "left": {
                    "image": left,
                    "frame_id": f"{suffix}-left",
                    "sha256": (suffix + "a") * 32,
                    "width": 640,
                    "height": 480,
                    "overlay_applied": False,
                },
                "right": {
                    "image": right,
                    "frame_id": f"{suffix}-right",
                    "sha256": (suffix + "b") * 32,
                    "width": 640,
                    "height": 480,
                    "overlay_applied": False,
                },
            }
        }

    pane.apply_snapshot(snapshot(first_left, first_right, "burst-1"))
    pane.annotation_mode_combo.setCurrentIndex(pane.annotation_mode_combo.findData("target"))
    pane.annotation_views["left"].add_source_point((100.0, 120.0), mode="target")
    pane.annotation_views["right"].add_source_point((200.0, 220.0), mode="target")

    pane.apply_snapshot(snapshot(second_left, second_right, "burst-2"))

    assert pane.annotation_views["left"].annotation_points("target") == (SourcePixel(100.0, 120.0),)
    assert pane.annotation_views["right"].annotation_points("target") == (SourcePixel(200.0, 220.0),)
    assert pane.annotation_payloads()["left"].frame_id == "burst-1-left"
    assert pane.annotation_payloads()["right"].frame_id == "burst-1-right"

    pane.annotation_clear_button.click()
    assert pane.annotation_views["left"].annotation_points("target") == ()
    assert pane.annotation_views["right"].annotation_points("target") == ()
    assert pane.annotation_payloads()["left"].frame_id == "burst-2-left"
    assert pane.annotation_payloads()["right"].frame_id == "burst-2-right"

    pane.deleteLater()
    app.processEvents()


def test_multiview_capture_clears_a_target_pair_after_it_was_accumulated() -> None:
    app = _app()
    pane = OpticalPosePane()
    first = QtGui.QImage(640, 480, QtGui.QImage.Format_RGB888)
    second = QtGui.QImage(640, 480, QtGui.QImage.Format_RGB888)
    first.fill(QtGui.QColor("#333333"))
    second.fill(QtGui.QColor("#444444"))

    def frames(image: QtGui.QImage, suffix: str) -> dict[str, object]:
        return {
            side: {
                "image": image,
                "frame_id": f"{suffix}-{side}",
                "sha256": (("a" if side == "left" else "b") + suffix) * 16,
                "width": 640,
                "height": 480,
                "overlay_applied": False,
            }
            for side in ("left", "right")
        }

    pane.apply_snapshot({"frozen_frames": frames(first, "burst-1")})
    pane.annotation_mode_combo.setCurrentIndex(pane.annotation_mode_combo.findData("target"))
    pane.annotation_views["left"].add_source_point((100.0, 120.0), mode="target")
    pane.annotation_views["right"].add_source_point((200.0, 220.0), mode="target")
    pane.apply_snapshot(
        {
            "drone_rtk_multiview_samples": [
                {
                    "sample_id": "sample-1",
                    "capture_observation_id": "obs-1",
                    "burst_id": "burst-1",
                    "state": "READY",
                    "annotations": {
                        side: {"frame_id": f"burst-1-{side}"}
                        for side in ("left", "right")
                    },
                }
            ],
            "frozen_frames": frames(second, "burst-2"),
        }
    )

    assert pane.annotation_views["left"].annotation_points("target") == ()
    assert pane.annotation_views["right"].annotation_points("target") == ()
    assert pane.annotation_payloads()["left"].frame_id == "burst-2-left"
    assert pane.annotation_payloads()["right"].frame_id == "burst-2-right"

    pane.deleteLater()
    app.processEvents()


def test_multiview_capture_preserves_target_pair_when_sample_is_blocked() -> None:
    app = _app()
    pane = OpticalPosePane()
    first = QtGui.QImage(640, 480, QtGui.QImage.Format_RGB888)
    second = QtGui.QImage(640, 480, QtGui.QImage.Format_RGB888)
    first.fill(QtGui.QColor("#333333"))
    second.fill(QtGui.QColor("#444444"))

    def frames(image: QtGui.QImage, suffix: str) -> dict[str, object]:
        return {
            side: {
                "image": image,
                "frame_id": f"{suffix}-{side}",
                "sha256": (("a" if side == "left" else "b") + suffix) * 16,
                "width": 640,
                "height": 480,
                "overlay_applied": False,
            }
            for side in ("left", "right")
        }

    pane.apply_snapshot({"frozen_frames": frames(first, "burst-1")})
    pane.annotation_mode_combo.setCurrentIndex(pane.annotation_mode_combo.findData("target"))
    pane.annotation_views["left"].add_source_point((100.0, 120.0), mode="target")
    pane.annotation_views["right"].add_source_point((200.0, 220.0), mode="target")
    pane.apply_snapshot(
        {
            "drone_rtk_multiview_samples": [
                {
                    "sample_id": "sample-blocked",
                    "burst_id": "burst-1",
                    "state": "BLOCKED",
                    "annotations": {
                        side: {"frame_id": f"burst-1-{side}"}
                        for side in ("left", "right")
                    },
                }
            ],
            "frozen_frames": frames(second, "burst-2"),
        }
    )

    assert pane.annotation_views["left"].annotation_points("target") == (
        SourcePixel(100.0, 120.0),
    )
    assert pane.annotation_views["right"].annotation_points("target") == (
        SourcePixel(200.0, 220.0),
    )
    assert pane.annotation_payloads()["left"].frame_id == "burst-1-left"
    assert pane.annotation_payloads()["right"].frame_id == "burst-1-right"

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

    assert pane.frozen_preview_layout.direction() == QtWidgets.QBoxLayout.LeftToRight
    assert pane.method_review_splitter.orientation() == QtCore.Qt.Horizontal
    assert pane.annotation_views["left"].width() < pane.width() / 2
    assert pane.annotation_views["right"].width() < pane.width() / 2

    pane.resize(900, 800)
    pane._update_optical_pose_layout_mode()
    app.processEvents()
    assert pane.frozen_preview_layout.direction() == QtWidgets.QBoxLayout.TopToBottom
    assert pane.method_review_splitter.orientation() == QtCore.Qt.Vertical

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


def _drone_rtk_observation(
    observation_id: str,
    *,
    sequence: int,
    state: str,
    drone: dict | None,
    left: dict | None,
    right: dict | None,
    sync_left_s: float | None,
    sync_right_s: float | None,
    left_frame_timestamp_s: float,
    right_frame_timestamp_s: float,
) -> dict:
    return {
        "schema": "optical-pose-drone-rtk-capture-observation-v1",
        "observation_id": observation_id,
        "sequence": sequence,
        "session_id": "session-001",
        "burst_id": f"burst-{sequence:03d}",
        "state": state,
        "gnss": {"drone": drone, "left": left, "right": right},
        "frames": {
            "left": {
                "frame_id": f"frame-l-{sequence}",
                "sha256": "a" * 64,
                "timestamp_s": left_frame_timestamp_s,
                "timestamp_reference": "app-receive",
            },
            "right": {
                "frame_id": f"frame-r-{sequence}",
                "sha256": "b" * 64,
                "timestamp_s": right_frame_timestamp_s,
                "timestamp_reference": "app-receive",
            },
        },
        "synchronization": {
            "state": "SYNC_OK" if sync_left_s is not None else "SYNC_UNVERIFIED",
            "left_frame_to_drone_gnss_s": sync_left_s,
            "right_frame_to_drone_gnss_s": sync_right_s,
            "reason": "" if sync_left_s is not None else "no Drone GNSS sample was available at capture",
        },
        "created_utc_ns": 0,
    }


def _drone_rtk_gnss_row(*, role: str, latitude_deg: float, longitude_deg: float) -> dict:
    return {
        "role": role,
        "latitude_deg": latitude_deg,
        "longitude_deg": longitude_deg,
        "altitude_m": 100.0,
        "altitude_ellipsoid_m": 120.0,
        "fix_type": 6,
        "rtk_label": "RTK_FIXED",
        "satellites_used": 22,
        "eph_m": 0.02,
        "epv_m": 0.03,
        "timestamp_s": 100.008,
        "has_position": True,
    }


def test_drone_rtk_observation_history_and_detail_selection() -> None:
    app = _app()
    pane = OpticalPosePane()

    ready = _drone_rtk_observation(
        "obs-001",
        sequence=0,
        state="EVIDENCE_READY",
        drone=_drone_rtk_gnss_row(role="drone", latitude_deg=37.111111111, longitude_deg=127.111111111),
        left=_drone_rtk_gnss_row(role="left", latitude_deg=37.222222222, longitude_deg=127.222222222),
        right=_drone_rtk_gnss_row(role="right", latitude_deg=37.333333333, longitude_deg=127.333333333),
        sync_left_s=0.008,
        sync_right_s=0.004,
        left_frame_timestamp_s=100.0,
        right_frame_timestamp_s=100.004,
    )
    incomplete = _drone_rtk_observation(
        "obs-002",
        sequence=1,
        state="INCOMPLETE",
        drone=None,
        left=None,
        right=None,
        sync_left_s=None,
        sync_right_s=None,
        left_frame_timestamp_s=200.0,
        right_frame_timestamp_s=200.004,
    )

    pane.apply_snapshot({"drone_rtk_capture_observations": [ready, incomplete]})

    table = pane.observation_history_table
    assert table.rowCount() == 2
    assert table.item(0, 0).text() == "001"
    assert table.item(0, 1).text() == "EVIDENCE_READY"
    assert table.item(0, 2).text() == "RTK_FIXED"  # Drone RTK
    assert table.item(0, 3).text() == "RTK_FIXED"  # P1 RTK
    assert table.item(0, 4).text() == "RTK_FIXED"  # P2 RTK
    assert table.item(0, 5).text() == "+8 ms"
    assert table.item(0, 6).text() == "+4 ms"
    assert table.item(0, 7).text() == "NOT ANNOTATED"
    assert table.item(1, 0).text() == "002"
    assert table.item(1, 1).text() == "INCOMPLETE"
    assert table.item(1, 2).text() == "NO DATA"
    assert table.item(1, 5).text() == "--"
    assert table.item(1, 6).text() == "--"

    # Default selection is the most recent capture (row 1 / obs-002), and the
    # Current Observation summary always tracks the latest capture too.
    assert pane.observation_detail_values["observation_id"].value.text() == "obs-002"
    assert pane.observation_detail_values["drone_latitude"].value.text() == "--"
    assert pane.current_observation_summary_label.text() == "#002 · INCOMPLETE"
    assert pane.current_observation_values["drone_rtk"].value.text() == "NO DATA"

    # Selecting row 0 must replace every stale row-1 value, not merge with it.
    table.selectRow(0)
    table.setCurrentCell(0, 0)
    assert pane.observation_detail_values["observation_id"].value.text() == "obs-001"
    assert pane.observation_detail_values["observation_state"].value.text() == "EVIDENCE_READY"
    assert pane.observation_detail_values["drone_latitude"].value.text().startswith("37.111111111")
    assert pane.observation_detail_values["p1_latitude"].value.text().startswith("37.222222222")
    assert pane.observation_detail_values["p2_latitude"].value.text().startswith("37.333333333")
    assert pane.observation_detail_values["left_frame_id"].value.text() == "frame-l-0"
    assert pane.observation_detail_values["sync_state"].value.text() == "SYNC_OK"
    assert pane.observation_detail_values["sync_delta_left"].value.text() == "+8 ms"

    # Current Observation stays on the latest capture (obs-002) even though
    # the detail panel below is now inspecting obs-001.
    assert pane.current_observation_summary_label.text() == "#002 · INCOMPLETE"

    # Re-applying the identical snapshot (a routine poll) must preserve the
    # operator's row-0 selection instead of resetting it.
    pane.apply_snapshot({"drone_rtk_capture_observations": [ready, incomplete]})
    assert pane.observation_detail_values["observation_id"].value.text() == "obs-001"
    assert pane.observation_detail_values["drone_latitude"].value.text().startswith("37.111111111")

    pane.deleteLater()
    app.processEvents()


def test_drone_rtk_multiview_sample_projects_target_and_readiness_state() -> None:
    app = _app()
    pane = OpticalPosePane()
    observation = _drone_rtk_observation(
        "obs-001",
        sequence=0,
        state="EVIDENCE_READY",
        drone=_drone_rtk_gnss_row(role="drone", latitude_deg=37.1, longitude_deg=127.1),
        left=_drone_rtk_gnss_row(role="left", latitude_deg=37.2, longitude_deg=127.2),
        right=_drone_rtk_gnss_row(role="right", latitude_deg=37.3, longitude_deg=127.3),
        sync_left_s=0.008,
        sync_right_s=0.004,
        left_frame_timestamp_s=100.0,
        right_frame_timestamp_s=100.004,
    )
    sample = {
        "sample_id": "sample-001",
        "capture_observation_id": "obs-001",
        "burst_id": observation["burst_id"],
        "state": "READY",
        "annotations": {
            "left": {"frame_id": observation["frames"]["left"]["frame_id"]},
            "right": {"frame_id": observation["frames"]["right"]["frame_id"]},
        },
    }

    pane.apply_snapshot(
        {
            "drone_rtk_multiview_samples": [sample],
            "drone_rtk_capture_observations": [observation],
        }
    )

    assert pane.observation_history_table.item(0, 7).text() == "READY"
    assert pane.current_observation_values["target"].value.text() == "READY"
    assert "READY/total=1/1" in pane.method_information_labels["Drone Multi-view"].text()
    payload = pane.command_payload("alignment.pose.solve_drone_rtk_multiview")
    assert payload["multiview_source"] == "accumulated_frozen_targets"

    pane.deleteLater()
    app.processEvents()


def test_blocked_multiview_sample_does_not_mark_its_frame_collected() -> None:
    app = _app()
    pane = OpticalPosePane()
    observation = _drone_rtk_observation(
        "obs-002",
        sequence=0,
        state="EVIDENCE_READY",
        drone=_drone_rtk_gnss_row(role="drone", latitude_deg=37.1, longitude_deg=127.1),
        left=_drone_rtk_gnss_row(role="left", latitude_deg=37.2, longitude_deg=127.2),
        right=_drone_rtk_gnss_row(role="right", latitude_deg=37.3, longitude_deg=127.3),
        sync_left_s=0.008,
        sync_right_s=0.004,
        left_frame_timestamp_s=100.0,
        right_frame_timestamp_s=100.004,
    )
    sample = {
        "sample_id": "sample-002",
        "capture_observation_id": "obs-002",
        "burst_id": observation["burst_id"],
        "state": "BLOCKED",
        "annotations": {
            "left": {"frame_id": observation["frames"]["left"]["frame_id"]},
            "right": {"frame_id": observation["frames"]["right"]["frame_id"]},
        },
    }

    pane.apply_snapshot(
        {
            "drone_rtk_multiview_samples": [sample],
            "drone_rtk_capture_observations": [observation],
        }
    )

    assert pane._drone_rtk_multiview_collected_frame_ids["left"] == set()
    assert pane._drone_rtk_multiview_collected_frame_ids["right"] == set()

    pane.deleteLater()
    app.processEvents()
