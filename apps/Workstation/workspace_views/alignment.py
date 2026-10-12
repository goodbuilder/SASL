"""Alignment workspace composing optical pose and baseline setup panes."""

from __future__ import annotations

from typing import Callable
from PyQt5 import QtCore, QtWidgets
from Workstation.ui.components import WorkspaceHeader
from Workstation.application.status_model import RuntimeStatus, StatusModel
from .optical_pose import OpticalPosePane


class AlignmentWorkspace(QtWidgets.QWidget):
    """Post-Devices workspace for measuring and comparing camera R,t."""

    inspection_requested = QtCore.pyqtSignal(object)
    optical_pose_command_requested = QtCore.pyqtSignal(str, object)
    open_measurement_requested = QtCore.pyqtSignal()
    inspection_clear_requested = QtCore.pyqtSignal(str)

    def __init__(
        self,
        backend: object,
        status_model: StatusModel,
        open_advanced: Callable[[], None],
        invoke_command: Callable[[str], bool] | None = None,
        parent: QtWidgets.QWidget | None = None,
    ) -> None:
        super().__init__(parent)
        self.backend = backend
        self.status_model = status_model
        # This page is hosted by a horizontally non-scrolling QScrollArea.
        # Wide method tabs and evidence tables have a large size hint, but
        # that hint must not keep the page wider than the central viewport
        # when the Inspector dock opens.
        self.setMinimumWidth(0)
        self.setSizePolicy(
            QtWidgets.QSizePolicy.Ignored,
            QtWidgets.QSizePolicy.Expanding,
        )
        layout = QtWidgets.QVBoxLayout(self)
        layout.setContentsMargins(8, 8, 8, 8)
        layout.setSpacing(6)
        layout.addWidget(
            WorkspaceHeader("Alignment")
        )

        # Alignment owns R,t evidence only. Device connection, Survey-in, and
        # their command surfaces stay in Devices; this page compares geometry
        # candidates below.
        self.operation_tabs = self._build_operation_tabs(invoke_command)
        # The Alignment page itself lives in the workspace host's outer scroll
        # area.  Common readiness/frozen evidence stays above the promoted
        # method tabs owned by OpticalPosePane.
        self.optical_pose_pane.setSizePolicy(
            QtWidgets.QSizePolicy.Ignored,
            QtWidgets.QSizePolicy.Expanding,
        )
        layout.addWidget(self.optical_pose_pane, 1)
    def _build_operation_tabs(
        self,
        _invoke_command: Callable[[str], bool] | None = None,
    ) -> QtWidgets.QTabWidget:
        self.operation_buttons: dict[str, QtWidgets.QPushButton] = {}
        self.optical_pose_pane = OpticalPosePane()
        operation_tabs = self.optical_pose_pane.method_tabs
        operation_tabs.tabBar().moveTab(1, 0)  # Mountain + GNSS first in Alignment.
        from Workstation.ui.baseline_yaw90_pane import BaselineYaw90Pane

        self.baseline_yaw90_pane = BaselineYaw90Pane(self.backend, _invoke_command)
        self.baseline_yaw90_pane.inspection_requested.connect(self.inspection_requested.emit)
        self.baseline_yaw90_pane.open_measurement.connect(self.open_measurement_requested.emit)
        self.baseline_yaw90_index = operation_tabs.insertTab(1, self.baseline_yaw90_pane, "Baseline Yaw90")
        operation_tabs.setCurrentIndex(0)
        self._last_setup_tab = False
        self._optical_scroll_position = 0
        operation_tabs.currentChanged.connect(self._setup_tab_changed)
        self.optical_pose_pane.command_requested.connect(
            self.optical_pose_command_requested.emit
        )
        self.operation_buttons.update(self.optical_pose_pane.solver_buttons)
        self.operation_buttons["alignment.pose.apply_candidate"] = (
            self.optical_pose_pane.apply_button
        )
        self.operation_buttons["alignment.pose.revert"] = (
            self.optical_pose_pane.revert_button
        )
        self.operation_buttons["alignment.pose.reset"] = (
            self.optical_pose_pane.reset_button
        )
        self.optical_pose_scroll = None
        return operation_tabs

    def _setup_tab_changed(self, index: int) -> None:
        setup = index == self.baseline_yaw90_index
        if self._last_setup_tab and not setup:
            self.inspection_clear_requested.emit("baseline_yaw90")
        parent = self.parentWidget()
        while parent is not None and not isinstance(parent, QtWidgets.QScrollArea):
            parent = parent.parentWidget()
        if parent is not None:
            scrollbar = parent.verticalScrollBar()
            if setup and not self._last_setup_tab:
                scrollbar.setValue(0)
        self._last_setup_tab = setup
        if setup:
            self.baseline_yaw90_pane.refresh(self.status_model.snapshot())

    def open_baseline_yaw90(self) -> None:
        self.operation_tabs.setCurrentIndex(self.baseline_yaw90_index)

    def refresh(self, snapshot: dict[str, RuntimeStatus]) -> None:
        if self.operation_tabs.currentIndex() == self.baseline_yaw90_index:
            self.baseline_yaw90_pane.refresh(snapshot)
        try:
            reader = getattr(self.backend, "alignment_live_rt_snapshot", None)
            if callable(reader):
                live = reader()
            else:
                from Workstation.controllers.alignment_rt_controller import alignment_live_rt_snapshot
                live = alignment_live_rt_snapshot(getattr(self.backend, "panel", self.backend))
            self.optical_pose_pane.set_live_rt_snapshot(live)
        except (AttributeError, TypeError, ValueError, RuntimeError) as exc:
            self.optical_pose_pane.set_live_rt_snapshot({}, error=str(exc))

    def select_tab_for_context(self, _context_id: str) -> None:
        """Alignment has one promoted method surface; preserve its local tab."""

        return
