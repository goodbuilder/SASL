"""Optical-pose layout and command composition over state-owning view components.

Native annotations, suggestions, candidates, frozen RTK history and live R,t
have distinct owners. Named legacy properties expose the same widget/state
objects; internal components never receive this pane or a backend proxy.
"""

from __future__ import annotations

import time as time  # Existing UI clock-injection boundary; shared stdlib module.
from collections.abc import Mapping
from typing import Sequence
from PyQt5 import QtCore, QtGui, QtWidgets
from Workstation.ui.annotation_image_view import AnnotationMode

from .optical_pose_parts.annotations import FrozenAnnotations
from .optical_pose_parts.suggestions import SuggestionReview
from .optical_pose_parts.candidates import CandidateReview
from .optical_pose_parts.methods import MethodControls
from .optical_pose_parts.observations import (
    RtkObservations as RtkObservations,
    _format_drone_rtk_sync_delta_ms as _format_drone_rtk_sync_delta_ms,
    _format_drone_rtk_degrees as _format_drone_rtk_degrees,
    _format_drone_rtk_metres as _format_drone_rtk_metres,
    _format_drone_rtk_seconds as _format_drone_rtk_seconds,
    _drone_rtk_gnss_label as _drone_rtk_gnss_label,
)
from .optical_pose_parts.live_reference import LiveRelativePose
from .optical_pose_parts.layouts import (
    _FrozenPreviewColumn as _FrozenPreviewColumn,
    _FrozenPreviewGroup as _FrozenPreviewGroup,
    _CurrentPageTabWidget as _CurrentPageTabWidget,
)


class OpticalPosePane(QtWidgets.QWidget):
    """Compose the established optical-pose UI and emit unchanged command payloads."""

    command_requested = QtCore.pyqtSignal(str, object)
    CANDIDATE_HEADERS = CandidateReview.CANDIDATE_HEADERS
    _CANDIDATE_DETAIL_INPUT_HASH_KEYS = CandidateReview._CANDIDATE_DETAIL_INPUT_HASH_KEYS
    _CANDIDATE_DETAIL_QUALITY_KEYS = CandidateReview._CANDIDATE_DETAIL_QUALITY_KEYS
    DRONE_RTK_OBSERVATION_HEADERS = RtkObservations.DRONE_RTK_OBSERVATION_HEADERS

    def __init__(self, parent: QtWidgets.QWidget | None = None) -> None:
        super().__init__(parent)
        self._optical_pose_layout_width_hint = 0
        layout = QtWidgets.QVBoxLayout(self)
        layout.setContentsMargins(6, 6, 6, 6)
        layout.setSpacing(6)
        self.solver_buttons = {}

        def make_solver_button(command_id, label):
            button = QtWidgets.QPushButton(label)
            button.setProperty("commandId", command_id)
            button.clicked.connect(
                lambda _checked=False, command_id=command_id: self.command_requested.emit(
                    command_id, self.command_payload(command_id)
                )
            )
            self.solver_buttons[command_id] = button
            return button

        self.observations = RtkObservations(self)
        self.annotations = FrozenAnnotations(
            self, make_solver_button=make_solver_button, collected_frame_ids=self.observations.collected_frame_ids
        )
        self.suggestions = SuggestionReview(self.annotations, self, make_solver_button=make_solver_button)
        self.candidates = CandidateReview(self, make_solver_button=make_solver_button)
        self.methods = MethodControls(
            self,
            make_solver_button=make_solver_button,
            create_combine_button=self.candidates.create_combine_button,
            populate_combine_page=self.candidates.populate_combine_page,
        )
        from Workstation.controllers.alignment_rt_controller import LIVE_RT_METHODS

        self.live_reference = LiveRelativePose(LIVE_RT_METHODS, self)
        for component in (self.annotations, self.suggestions, self.candidates):
            component.command_requested.connect(self.command_requested.emit)
        for component in (self.suggestions, self.candidates, self.observations, self.methods):
            component.information_changed.connect(self._refresh_method_information)
        self.methods.mountain_lines_changed.connect(self.annotations._set_mountain_auto_lines_hidden)
        self.annotations._set_mountain_auto_lines_hidden(True)
        layout.addWidget(self.methods.readiness_group)
        layout.addWidget(self.annotations.preview_group)
        contract = QtWidgets.QLabel(
            "Frames: image pixels/OpenCV optical RDF · pose PX4 virtual FRD→world NED · "
            "GNSS WGS84 · candidate application is immutable and owner-controlled"
        )
        contract.setObjectName("compactContract")
        contract.setWordWrap(True)
        self.contract_label = contract

        method_column = QtWidgets.QWidget()
        method_column.setObjectName("alignmentMethodColumn")
        method_column_layout = QtWidgets.QVBoxLayout(method_column)
        method_column_layout.setContentsMargins(0, 0, 0, 0)
        method_column_layout.setSpacing(6)
        method_column_layout.addWidget(self.method_tabs)
        method_column_layout.addWidget(self.auto_group)
        method_column_layout.addStretch(1)
        self.method_column = method_column

        candidate_review = QtWidgets.QWidget()
        candidate_review.setObjectName("alignmentCandidateReview")
        candidate_review_layout = QtWidgets.QVBoxLayout(candidate_review)
        candidate_review_layout.setContentsMargins(0, 0, 0, 0)
        candidate_review_layout.setSpacing(6)
        candidate_review_layout.addWidget(self.candidate_group)
        candidate_review_layout.addWidget(self.observations_group)
        candidate_review_layout.addWidget(self.contract_label)
        candidate_review_layout.addStretch(1)
        self.candidate_review_widget = candidate_review

        lower_splitter = QtWidgets.QSplitter(QtCore.Qt.Horizontal)
        lower_splitter.setObjectName("alignmentMethodReviewSplitter")
        lower_splitter.setChildrenCollapsible(False)
        lower_splitter.setHandleWidth(8)
        lower_splitter.addWidget(method_column)
        lower_splitter.addWidget(candidate_review)
        lower_splitter.setStretchFactor(0, 3)
        lower_splitter.setStretchFactor(1, 2)
        lower_splitter.setSizes([720, 480])
        self.method_review_splitter = lower_splitter
        layout.addWidget(self.live_reference.live_rt_group)
        layout.addWidget(lower_splitter)
        self.methods.method_tabs.currentChanged.connect(self._handle_method_tab_changed)
        self._handle_method_tab_changed(self.methods.method_tabs.currentIndex())

    def _refresh_method_information(self):
        observation_count, samples = self.observations.information_snapshot()
        self.methods.refresh_information(
            self.annotations.annotation_payloads(), self.candidates.information_rows(), observation_count, samples
        )

    def _handle_method_tab_changed(self, index: int) -> None:
        label = self.method_tabs.tabText(index) if index >= 0 else ""
        setup = label == "Baseline Yaw90"
        was_setup = bool(getattr(self, "_baseline_setup_visible", False))
        if setup and not was_setup:
            self._baseline_saved_splitter_sizes = self.method_review_splitter.sizes()
            parent = self.parentWidget()
            while parent is not None and not isinstance(parent, QtWidgets.QScrollArea):
                parent = parent.parentWidget()
            self._baseline_saved_scroll = parent.verticalScrollBar().value() if parent is not None else 0
        self._baseline_setup_visible = setup
        # Keep the installation tables in their existing first viewport. The
        # live transform readout follows the setup page, and precedes solver
        # review on the other tabs.
        root_layout = self.layout()
        root_layout.removeWidget(self.live_rt_group)
        if setup:
            root_layout.addWidget(self.live_rt_group)
        else:
            root_layout.insertWidget(root_layout.indexOf(self.method_review_splitter), self.live_rt_group)
        self.readiness_group.setVisible(not setup)
        self.preview_group.setVisible(not setup)
        self.candidate_review_widget.setVisible(not setup)
        if setup:
            self.auto_group.hide()
            return
        if was_setup:
            self.method_review_splitter.setSizes(self._baseline_saved_splitter_sizes)
            parent = self.parentWidget()
            while parent is not None and not isinstance(parent, QtWidgets.QScrollArea):
                parent = parent.parentWidget()
            if parent is not None:
                scrollbar = parent.verticalScrollBar()
                value = getattr(self, "_baseline_saved_scroll", 0)
                QtCore.QTimer.singleShot(0, lambda: scrollbar.setValue(value))
        annotation_modes = {
            "Horizon": AnnotationMode.HORIZON.value,
            "Mountain + GNSS": AnnotationMode.RIDGE.value,
            "Terrain PnP": AnnotationMode.RIDGE.value,
            "Drone Rotation": AnnotationMode.TARGET.value,
        }
        mode = annotation_modes.get(label)
        editable = mode is not None
        self.annotation_mode_combo.setEnabled(editable)
        self.annotation_clear_button.setEnabled(editable)
        for view in self.annotation_views.values():
            view.setEnabled(editable)
        if mode is not None:
            mode_index = self.annotation_mode_combo.findData(mode)
            if mode_index >= 0:
                self.annotation_mode_combo.setCurrentIndex(mode_index)
            self._set_annotation_mode(self.annotation_mode_combo.currentIndex())

        show_horizon_auto = label == "Horizon"
        show_mountain_auto = label == "Mountain + GNSS"
        self.auto_group.setVisible(show_horizon_auto or show_mountain_auto)
        horizon_button = self.auto_select_buttons.get("alignment.pose.auto_select_horizon")
        mountain_button = self.auto_select_buttons.get("alignment.pose.auto_select_mountain")
        cloud_button = self.auto_select_buttons.get("alignment.pose.auto_select_cloud")
        if horizon_button is not None:
            horizon_button.setVisible(show_horizon_auto)
        if mountain_button is not None:
            mountain_button.setVisible(show_mountain_auto)
        if cloud_button is not None:
            cloud_button.setVisible(show_mountain_auto)
        self.observations_group.setVisible(label in {"Drone Rotation", "Drone Multi-view"})
        self._refresh_method_information()

    def _update_optical_pose_layout_mode(self) -> None:
        if getattr(self, "_baseline_setup_visible", False):
            return
        width_hint = self._optical_pose_layout_width_hint or self.width()
        preview_width = min(
            self.preview_group.contentsRect().width(),
            max(0, width_hint - 12),
        )
        margins = self.frozen_preview_layout.contentsMargins()
        usable_width = max(0, preview_width - margins.left() - margins.right())
        horizontal_previews = (usable_width - self.frozen_preview_layout.spacing()) // 2 >= 560
        preview_direction = (
            QtWidgets.QBoxLayout.LeftToRight if horizontal_previews else QtWidgets.QBoxLayout.TopToBottom
        )
        if self.frozen_preview_layout.direction() != preview_direction:
            self.frozen_preview_layout.setDirection(preview_direction)
            self.preview_group.updateGeometry()

        splitter_orientation = QtCore.Qt.Horizontal if width_hint >= 1200 else QtCore.Qt.Vertical
        if self.method_review_splitter.orientation() != splitter_orientation:
            self.method_review_splitter.setOrientation(splitter_orientation)
            self.method_review_splitter.setSizes(
                [720, 480] if splitter_orientation == QtCore.Qt.Horizontal else [420, 360]
            )

    def resizeEvent(self, event: QtGui.QResizeEvent) -> None:  # noqa: N802 - Qt override.
        self._optical_pose_layout_width_hint = event.size().width()
        super().resizeEvent(event)
        self._update_optical_pose_layout_mode()

    def showEvent(self, event: QtGui.QShowEvent) -> None:  # noqa: N802 - Qt override.
        super().showEvent(event)
        QtCore.QTimer.singleShot(0, self._update_optical_pose_layout_mode)

    def command_payload(self, command_id: str) -> dict[str, object]:
        payload: dict[str, object] = {
            "command_id": str(command_id),
            "annotation_mode": str(self.annotation_mode_combo.currentData() or AnnotationMode.HORIZON.value),
            "annotations": self.annotation_payloads(),
            "annotation_contract": (
                "frozen native OpenCV pixels: u right, v down; identity is frame_id + image_sha256 + native dimensions"
            ),
            "mountain_contract": (
                "same-order paired left/right frozen native Mountain u,v; "
                "numeric GNSS baseline; no GeoTIFF or terrain height"
            ),
        }
        if command_id == "alignment.pose.solve_combined":
            payload["candidate_ids"] = list(self.combine_candidate_ids())
            payload["candidate_dofs"] = self.combine_candidate_dofs()
        if command_id in {
            "alignment.pose.auto_select_horizon",
            "alignment.pose.auto_select_mountain",
            "alignment.pose.auto_select_cloud",
        }:
            payload["auto_mode"] = command_id.rsplit("_", 1)[-1]
            payload["frozen_frames_required"] = True
        if command_id == "alignment.pose.use_suggestion":
            payload.update(self.suggestions.acceptance_payload())
        if command_id == "alignment.pose.solve_drone_rtk_multiview":
            payload["multiview_source"] = "accumulated_frozen_targets"
        return payload

    def apply_snapshot(self, snapshot: Mapping[str, object]) -> None:
        readiness = snapshot.get("readiness")
        if isinstance(readiness, Mapping):
            self.set_readiness(readiness)
        candidates = snapshot.get("candidates")
        if isinstance(candidates, (tuple, list)):
            self.set_candidates(candidates)
        self.set_active_profile(snapshot.get("active_profile_id"))
        drone_rtk_multiview_samples = snapshot.get("drone_rtk_multiview_samples")
        if isinstance(drone_rtk_multiview_samples, (tuple, list)):
            self.set_drone_rtk_multiview_samples(drone_rtk_multiview_samples)
        drone_rtk_observations = snapshot.get("drone_rtk_capture_observations")
        if isinstance(drone_rtk_observations, (tuple, list)):
            self.set_drone_rtk_observations(drone_rtk_observations)
        # Text-only owner state remains supported, while ``frozen_frames`` is
        # the explicit QImage-bearing snapshot used by the annotation editor.
        for key in ("frozen_previews", "frozen_frames"):
            frozen = snapshot.get(key)
            if isinstance(frozen, Mapping):
                for side in ("left", "right"):
                    if side in frozen:
                        self.set_frozen_preview_state(side, frozen[side])
        auto_suggestions = snapshot.get("auto_suggestions")
        if isinstance(auto_suggestions, (tuple, list)):
            self.set_auto_suggestions(auto_suggestions)
        accepted = snapshot.get("accepted_auto_suggestion")
        if isinstance(accepted, Mapping):
            self._apply_accepted_auto_suggestion(accepted)

    def _selected_drone_rtk_observation_id(self) -> str:
        return self.observations._selected_drone_rtk_observation_id()

    def set_drone_rtk_multiview_samples(self, samples: object) -> None:
        return self.observations.set_drone_rtk_multiview_samples(samples)

    def _drone_rtk_target_state(self, observation: Mapping[str, object]) -> str:
        return self.observations._drone_rtk_target_state(observation)

    def set_drone_rtk_observations(self, observations: object) -> None:
        return self.observations.set_drone_rtk_observations(observations)

    def _refresh_drone_rtk_observation_detail(self) -> None:
        return self.observations._refresh_drone_rtk_observation_detail()

    def _apply_current_drone_rtk_observation_summary(self, observation: object) -> None:
        return self.observations._apply_current_drone_rtk_observation_summary(observation)

    _apply_gnss_detail_rows = staticmethod(RtkObservations._apply_gnss_detail_rows)

    _apply_frame_detail_rows = staticmethod(RtkObservations._apply_frame_detail_rows)

    def _apply_selected_drone_rtk_observation_detail(self, observation: object) -> None:
        return self.observations._apply_selected_drone_rtk_observation_detail(observation)

    def annotation_payloads(self) -> dict[str, object]:
        return self.annotations.annotation_payloads()

    def _import_frozen_burst(self) -> None:
        return self.annotations._import_frozen_burst()

    def set_frozen_preview_state(self, side: str, detail: object) -> None:
        return self.annotations.set_frozen_preview_state(side, detail)

    def _set_mountain_auto_lines_hidden(self, hidden: bool) -> None:
        return self.annotations._set_mountain_auto_lines_hidden(hidden)

    def _set_annotation_mode(self, _index: int = -1) -> None:
        return self.annotations._set_annotation_mode(_index)

    def _clear_current_annotations(self) -> None:
        return self.annotations._clear_current_annotations()

    _auto_points = staticmethod(SuggestionReview._auto_points)

    _auto_confidences = staticmethod(SuggestionReview._auto_confidences)

    def _clear_auto_overlays(self) -> None:
        return self.suggestions._clear_auto_overlays()

    def _on_suggestion_point_right_clicked(self, side: str, mode: str, visible_index: int) -> None:
        return self.suggestions._on_suggestion_point_right_clicked(side, mode, visible_index)

    def _filtered_auto_points(
        self, suggestion_id: str, side: str, mode: AnnotationMode, row: Mapping[str, object]
    ) -> tuple[tuple[tuple[float, float], ...], tuple[float, ...]]:
        return self.suggestions._filtered_auto_points(suggestion_id, side, mode, row)

    def _mountain_suggestion_copy_status(self, suggestion_id: str, sides: Mapping[str, object]) -> tuple[bool, str]:
        return self.suggestions._mountain_suggestion_copy_status(suggestion_id, sides)

    def _horizon_suggestion_copy_status(self, sides: Mapping[str, object]) -> tuple[bool, str]:
        return self.suggestions._horizon_suggestion_copy_status(sides)

    def set_auto_suggestion_busy(self, busy: bool, mode: str = "") -> None:
        return self.suggestions.set_auto_suggestion_busy(busy, mode)

    def _reset_auto_acceptance(self) -> None:
        return self.suggestions._reset_auto_acceptance()

    def set_auto_suggestion_error(self, command_id: str, message: str) -> None:
        return self.suggestions.set_auto_suggestion_error(command_id, message)

    def set_auto_suggestions(self, suggestions: object) -> None:
        return self.suggestions.set_auto_suggestions(suggestions)

    def _apply_accepted_auto_suggestion(self, suggestion: Mapping[str, object]) -> None:
        return self.suggestions._apply_accepted_auto_suggestion(suggestion)

    def _on_annotation_changed(self, side: str, payload: object) -> None:
        return self.suggestions._on_annotation_changed(side, payload)

    _candidate_text = staticmethod(CandidateReview._candidate_text)

    _candidate_observability = staticmethod(CandidateReview._candidate_observability)

    _candidate_owned_dofs = staticmethod(CandidateReview._candidate_owned_dofs)

    def selected_candidate_id(self) -> str:
        return self.candidates.selected_candidate_id()

    def apply_payload(self) -> dict[str, object] | None:
        return self.candidates.apply_payload()

    def revert_payload(self) -> dict[str, object]:
        return self.candidates.revert_payload()

    def set_active_profile(self, profile_id: object) -> None:
        return self.candidates.set_active_profile(profile_id)

    def set_reference_profile(self, calibration: Mapping[str, object]) -> None:
        return self.candidates.set_reference_profile(calibration)

    _candidate_rt_sources = staticmethod(CandidateReview._candidate_rt_sources)

    def set_candidates(self, candidates: object) -> None:
        return self.candidates.set_candidates(candidates)

    def _selected_candidate_applicable(self) -> bool:
        return self.candidates._selected_candidate_applicable()

    _candidate_relative_pose_text = staticmethod(CandidateReview._candidate_relative_pose_text)

    _candidate_detail_text = staticmethod(CandidateReview._candidate_detail_text)

    def _refresh_candidate_actions(self) -> None:
        return self.candidates._refresh_candidate_actions()

    def combine_candidate_ids(self) -> tuple[str, ...]:
        return self.candidates.combine_candidate_ids()

    def _sync_combine_candidate_list(self, candidates: Sequence[Mapping[str, object]], selected_ids: set[str]) -> None:
        return self.candidates._sync_combine_candidate_list(candidates, selected_ids)

    def combine_candidate_dofs(self) -> dict[str, list[str]]:
        return self.candidates.combine_candidate_dofs()

    def _sync_combine_dof_table(self) -> None:
        return self.candidates._sync_combine_dof_table()

    def _on_combine_candidates_changed(self, *_args: object) -> None:
        return self.candidates._on_combine_candidates_changed(*_args)

    def _refresh_combine_action(self, *_args: object) -> None:
        return self.candidates._refresh_combine_action(*_args)

    def _request_apply(self) -> None:
        return self.candidates._request_apply()

    def _request_revert(self) -> None:
        return self.candidates._request_revert()

    def _request_reset(self) -> None:
        return self.candidates._request_reset()

    def set_readiness(self, readiness: Mapping[str, object]) -> None:
        return self.methods.set_readiness(readiness)

    def set_live_rt_snapshot(self, snapshot: Mapping[str, object], *, error: str = "") -> None:
        return self.live_reference.set_live_rt_snapshot(snapshot, error=error)


def _component_field(component, name):
    return property(
        lambda pane: getattr(getattr(pane, component), name),
        lambda pane, value: setattr(getattr(pane, component), name, value),
    )


# Finite compatibility surface used by AlignmentWorkspace and established UI tests.
# Values are never copied; components above are their only storage owners.
_COMPONENT_FIELDS = {
    "observations": (
        "_drone_rtk_multiview_collected_frame_ids",
        "_drone_rtk_multiview_sample_by_burst",
        "_drone_rtk_multiview_sample_by_observation",
        "_drone_rtk_multiview_samples",
        "_drone_rtk_observation_order",
        "_drone_rtk_observation_rows",
        "current_observation_summary_label",
        "current_observation_values",
        "observation_detail_values",
        "observation_history_table",
        "observations_group",
    ),
    "annotations": (
        "_auto_accept_in_progress",
        "_manual_mode_tooltips",
        "annotation_clear_button",
        "annotation_mode_combo",
        "annotation_views",
        "capture_frozen_burst_button",
        "capture_frozen_burst_wiris_ftp_button",
        "capture_frozen_burst_wiris_ftp_oz_button",
        "frozen_preview_labels",
        "frozen_preview_layout",
        "frozen_preview_panels",
        "import_frozen_burst_button",
        "preview_group",
    ),
    "suggestions": (
        "_accepted_annotation_snapshots",
        "_accepted_suggestion_id",
        "_active_auto_suggestion_id",
        "_auto_suggestion_error_active",
        "_auto_suggestion_excluded_indices",
        "_auto_suggestion_provenance",
        "_auto_suggestions",
        "_visible_auto_suggestion_indices",
        "auto_group",
        "auto_select_buttons",
        "auto_suggestion_metrics_label",
        "auto_suggestion_status_label",
        "use_suggestion_button",
    ),
    "candidates": (
        "_active_profile_id",
        "_candidate_rows",
        "apply_button",
        "candidate_detail_text",
        "candidate_group",
        "candidate_status_label",
        "candidate_table",
        "combine_candidate_list",
        "combine_candidate_status",
        "combine_dof_table",
        "reference_value",
        "reset_button",
        "revert_button",
    ),
    "methods": (
        "method_detail_labels",
        "method_information_labels",
        "method_tabs",
        "mountain_hide_auto_lines_checkbox",
        "readiness_group",
        "readiness_values",
    ),
    "live_reference": ("_live_rt_rows", "live_rt_group", "live_rt_table"),
}
for _component, _names in _COMPONENT_FIELDS.items():
    for _name in _names:
        setattr(OpticalPosePane, _name, _component_field(_component, _name))
del _component, _names, _name
