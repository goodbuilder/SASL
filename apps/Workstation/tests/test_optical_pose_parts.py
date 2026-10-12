"""State ownership and the unchanged optical-pose component handoffs."""

from __future__ import annotations

import ast
from pathlib import Path

import pytest
from PyQt5 import QtCore, QtGui, QtWidgets

from Workstation.workspace_views.optical_pose import OpticalPosePane, _COMPONENT_FIELDS
from Workstation.workspace_views.optical_pose_parts.annotations import FrozenAnnotations
from Workstation.workspace_views.optical_pose_parts.candidates import CandidateReview
from Workstation.workspace_views.optical_pose_parts.live_reference import LiveRelativePose
from Workstation.workspace_views.optical_pose_parts.observations import RtkObservations
from Workstation.workspace_views.optical_pose_parts.suggestions import SuggestionReview


@pytest.fixture
def host():
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    widget = QtWidgets.QWidget()
    yield widget
    widget.deleteLater()
    QtCore.QCoreApplication.sendPostedEvents(None, QtCore.QEvent.DeferredDelete)
    app.processEvents()
    QtCore.QCoreApplication.sendPostedEvents(None, QtCore.QEvent.DeferredDelete)


def _buttons(parent):
    def create(command_id, label):
        button = QtWidgets.QPushButton(label, parent)
        button.setProperty("commandId", command_id)
        return button

    return create


def _image():
    image = QtGui.QImage(640, 480, QtGui.QImage.Format_RGB888)
    image.fill(QtGui.QColor("#244466"))
    return image


def _frame(side, *, suffix="first"):
    return {
        "image": _image(),
        "frame_id": f"{side}-{suffix}",
        "image_sha256": ("1" if side == "left" else "2") * 64,
    }


def test_public_pane_fields_are_aliases_to_their_actual_component_owners(host):
    pane = OpticalPosePane(host)
    for component_name, names in _COMPONENT_FIELDS.items():
        component = getattr(pane, component_name)
        assert component.parent() is pane
        for name in names:
            assert name not in vars(pane)
            if hasattr(component, name):
                assert getattr(pane, name) is getattr(component, name)
    assert pane.suggestions.annotations is pane.annotations
    assert pane.candidate_group.parentWidget() is pane.candidate_review_widget
    assert pane.method_tabs.parentWidget() is pane.method_column
    assert list(pane.solver_buttons) == [
        "alignment.pose.capture_burst",
        "alignment.pose.capture_burst_wiris_ftp",
        "alignment.pose.capture_burst_wiris_ftp_oz",
        "alignment.pose.auto_select_horizon",
        "alignment.pose.auto_select_mountain",
        "alignment.pose.auto_select_cloud",
        "alignment.pose.use_suggestion",
        "alignment.pose.solve_drone_rtk_multiview",
        "alignment.pose.solve_horizon",
        "alignment.pose.solve_gnss_mountain",
        "alignment.pose.solve_cloud_rotation",
        "alignment.pose.import_terrain",
        "alignment.pose.solve_terrain",
        "alignment.pose.solve_drone_yaw",
        "alignment.pose.solve_combined",
    ]
    commands = []
    pane.command_requested.connect(lambda command, payload: commands.append((command, payload)))
    pane.reset_button.click()
    assert commands == [("alignment.pose.reset", {})]


def test_suggestion_review_uses_only_the_explicit_native_pair_and_rejects_stale_identity(host):
    annotations = FrozenAnnotations(host, make_solver_button=_buttons(host), collected_frame_ids=lambda side: ())
    review = SuggestionReview(annotations, host, make_solver_button=_buttons(host))
    for side in ("left", "right"):
        annotations.set_frozen_preview_state(side, _frame(side))
    suggestion = {
        "suggestion_id": "review-horizon",
        "mode": "horizon",
        "state": "LOW_CONFIDENCE",
        "eligible": False,  # Detector quality remains advisory for structurally valid copy.
        "sides": {
            side: {
                "frame_id": f"{side}-first",
                "image_sha256": ("1" if side == "left" else "2") * 64,
                "source_width_px": 640,
                "source_height_px": 480,
                "points": [(20.0, 210.0), (620.0, 230.0)],
            }
            for side in ("left", "right")
        },
    }
    review.set_auto_suggestions([suggestion])
    assert review.use_suggestion_button.isEnabled()
    assert all(not payload.horizon_line for payload in annotations.annotation_payloads().values())
    review._apply_accepted_auto_suggestion(suggestion)
    accepted = annotations.annotation_payloads()
    assert accepted["left"].frame_id == "left-first"
    assert [(point.u, point.v) for point in accepted["left"].horizon_line] == [(20.0, 210.0), (620.0, 230.0)]
    assert review._auto_suggestion_provenance == "AUTO_ACCEPTED"
    annotations.set_frozen_preview_state("right", _frame("right", suffix="replacement"))
    review.set_auto_suggestions([suggestion])
    assert not review.use_suggestion_button.isEnabled()
    assert "frame identity changed" in review.auto_suggestion_status_label.text()
    assert review._accepted_annotation_snapshots == {}


def test_candidate_component_preserves_selected_identity_and_explicit_dof_choices(host):
    review = CandidateReview(host, make_solver_button=_buttons(host))
    review.create_combine_button()
    page = QtWidgets.QWidget(host)
    review.populate_combine_page(QtWidgets.QVBoxLayout(page))
    axes = ("roll_deg", "pitch_deg", "yaw_deg", "north_m", "east_m", "down_m")

    def candidate(name, owned):
        mask = {axis: axis in owned for axis in axes}
        return {"candidate_id": name, "method": name, "applicable": True, "optimized_dof": mask, "observable_dof": mask}

    horizon = candidate("horizon", {"roll_deg", "pitch_deg"})
    yaw = candidate("drone-yaw", {"yaw_deg"})
    review.set_candidates([horizon, yaw])
    review.candidate_table.setCurrentCell(1, 0)
    for index in range(2):
        review.combine_candidate_list.item(index).setCheckState(QtCore.Qt.Checked)
    assert review.combine_button.isEnabled()
    review.combine_dof_table.item(0, 2).setCheckState(QtCore.Qt.Unchecked)
    assert review.combine_candidate_dofs() == {"horizon": ["roll_deg"], "drone-yaw": ["yaw_deg"]}
    review.set_candidates([yaw, horizon])
    assert review.selected_candidate_id() == "drone-yaw"
    assert review.combine_candidate_ids() == ("drone-yaw", "horizon")
    assert review.combine_candidate_dofs() == {"drone-yaw": ["yaw_deg"], "horizon": ["roll_deg"]}
    assert review.apply_payload() == {"candidate_id": "drone-yaw"}


def test_frozen_observation_selection_and_collected_frames_are_separate_from_live_rt(host):
    observations = RtkObservations(host)
    rows = [
        {"observation_id": f"obs-{index}", "burst_id": f"burst-{index}", "sequence": index, "state": "READY"}
        for index in range(2)
    ]
    observations.set_drone_rtk_observations(rows)
    observations.observation_history_table.setCurrentCell(0, 0)
    observations.set_drone_rtk_multiview_samples(
        [
            {
                "capture_observation_id": "obs-0",
                "burst_id": "burst-0",
                "state": "READY",
                "annotations": {"left": {"frame_id": "left-collected"}},
            },
            {
                "capture_observation_id": "obs-1",
                "burst_id": "burst-1",
                "state": "BLOCKED",
                "annotations": {"right": {"frame_id": "right-unverified"}},
            },
        ]
    )
    observations.set_drone_rtk_observations([*rows, {"observation_id": "obs-2", "sequence": 2}])
    assert observations._selected_drone_rtk_observation_id() == "obs-0"
    assert observations.collected_frame_ids("left") == frozenset({"left-collected"})
    assert observations.collected_frame_ids("right") == frozenset()
    live = LiveRelativePose((("live", "Live reference"),), host)
    live.set_live_rt_snapshot(
        {
            "rows": [
                {
                    "method_id": "live",
                    "state": "LIVE",
                    "rotation_right_from_left_rdf": [[1, 0, 0], [0, 1, 0], [0, 0, 1]],
                    "translation_right_from_left_rdf_m": [1, -2, 3],
                }
            ]
        }
    )
    assert live.live_rt_table.item(0, 2).text() == "R +1.000000\nD -2.000000\nF +3.000000"
    assert observations._selected_drone_rtk_observation_id() == "obs-0"
    assert observations._drone_rtk_observation_rows["obs-0"] == rows[0]


def _assert_part_boundary(source):
    dependencies = set()
    for node in ast.walk(ast.parse(source)):
        if isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef)):
            assert node.name not in {"__getattr__", "__setattr__"}
        if isinstance(node, ast.Attribute) and isinstance(node.value, ast.Name) and node.value.id == "self":
            assert node.attr not in {"panel", "backend", "window", "workspace"}
        if isinstance(node, ast.ImportFrom):
            assert node.module not in {"Workstation.workspaces", "Workstation.workspace_views.optical_pose"}
            assert all(alias.name != "*" for alias in node.names)
            if node.level:
                assert node.level == 1
                dependencies.add(node.module)
    return dependencies


def test_optical_parts_have_one_way_dependencies_and_reject_workspace_back_import():
    directory = Path(__file__).resolve().parents[1] / "workspace_views" / "optical_pose_parts"
    graph = {path.stem: _assert_part_boundary(path.read_text()) for path in directory.glob("*.py")}
    assert graph == {
        "__init__": set(),
        "layouts": set(),
        "annotations": {"layouts"},
        "suggestions": {"annotations"},
        "candidates": set(),
        "methods": {"layouts"},
        "observations": set(),
        "live_reference": set(),
    }
    with pytest.raises(AssertionError):
        _assert_part_boundary("from Workstation.workspace_views.optical_pose import OpticalPosePane")
    with pytest.raises(AssertionError):
        _assert_part_boundary("class Incorrect:\n    def read(self): return self.workspace.candidate_table")
