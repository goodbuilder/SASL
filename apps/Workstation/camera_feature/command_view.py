"""Owned WIRIS command/zoom widgets; actions remain backend commands."""

from dataclasses import dataclass
from typing import Callable
from PyQt5 import QtCore, QtWidgets
from Workstation.wiris_workers.wiris_sdk_catalog import WIRIS_COMMAND_CATALOG
from Workstation.workstation_core.wiris_zoom import MIN_ZOOM_LEVEL, MAX_ZOOM_LEVEL, LINE_TERMINATOR, NUL_TERMINATOR


@dataclass(frozen=True)
class CameraCommandActions:
    send_zoom: Callable
    selected_zoom_target: Callable
    _refresh_zoom_intrinsics_tab: Callable
    _apply_selected_wiris_command: Callable
    send_selected_wiris_command: Callable
    _configure_collapsible_group: Callable

    def __post_init__(self):
        if not all(callable(value) for value in vars(self).values()):
            raise TypeError("Camera command widgets require explicit callbacks")


COMMAND_VIEW_FIELDS = (
    "wiris_command_combo",
    "wiris_command_edit",
    "wiris_command_log",
    "wiris_command_send_button",
    "wiris_command_status_label",
    "wiris_command_target_combo",
    "wiris_command_terminator_combo",
    "zoom_in_button",
    "zoom_level_spin",
    "zoom_out_button",
    "zoom_set_button",
    "zoom_status_label",
    "zoom_stop_button",
    "zoom_target_combo",
)


class CameraCommandView(QtCore.QObject):
    def __init__(self, initial_zoom, actions: CameraCommandActions, *, parent=None):
        super().__init__(parent)
        zoom_group = QtWidgets.QGroupBox("WIRIS 줌 명령 제어")
        zoom_group.setObjectName("checkerboardZoomControlGroup")
        zoom_group_layout = QtWidgets.QVBoxLayout(zoom_group)
        self.zoom_level_spin = QtWidgets.QSpinBox()
        self.zoom_level_spin.setRange(MIN_ZOOM_LEVEL, MAX_ZOOM_LEVEL)
        self.zoom_level_spin.setValue(initial_zoom)
        self.zoom_level_spin.valueChanged.connect(lambda _value: actions._refresh_zoom_intrinsics_tab())
        self.zoom_target_combo = QtWidgets.QComboBox()
        self.zoom_target_combo.addItem("양쪽 카메라", "both")
        self.zoom_target_combo.addItem("왼쪽 카메라", "left")
        self.zoom_target_combo.addItem("오른쪽 카메라", "right")
        self.zoom_status_label = QtWidgets.QLabel("대기 중")

        action_row = QtWidgets.QGridLayout()
        action_row.setHorizontalSpacing(6)
        action_row.setVerticalSpacing(6)
        self.zoom_in_button = QtWidgets.QPushButton("확대")
        self.zoom_out_button = QtWidgets.QPushButton("축소")
        self.zoom_stop_button = QtWidgets.QPushButton("정지")
        self.zoom_set_button = QtWidgets.QPushButton("레벨 적용")
        self.zoom_in_button.clicked.connect(
            lambda _checked=False: actions.send_zoom(actions.selected_zoom_target(), "in")
        )
        self.zoom_out_button.clicked.connect(
            lambda _checked=False: actions.send_zoom(actions.selected_zoom_target(), "out")
        )
        self.zoom_stop_button.clicked.connect(
            lambda _checked=False: actions.send_zoom(actions.selected_zoom_target(), "stop")
        )
        self.zoom_set_button.clicked.connect(
            lambda _checked=False: actions.send_zoom(actions.selected_zoom_target(), "set")
        )
        action_row.addWidget(QtWidgets.QLabel("대상"), 0, 0)
        action_row.addWidget(self.zoom_target_combo, 0, 1, 1, 2)
        action_row.addWidget(QtWidgets.QLabel("레벨"), 0, 3)
        action_row.addWidget(self.zoom_level_spin, 0, 4)
        action_row.addWidget(self.zoom_set_button, 0, 5)
        action_row.addWidget(self.zoom_in_button, 1, 0, 1, 2)
        action_row.addWidget(self.zoom_out_button, 1, 2, 1, 2)
        action_row.addWidget(self.zoom_stop_button, 1, 4, 1, 2)
        action_row.setRowMinimumHeight(0, 42)
        action_row.setRowMinimumHeight(1, 42)
        action_row.setColumnStretch(1, 1)
        action_row.setColumnStretch(2, 1)
        action_row.setColumnStretch(4, 1)
        zoom_group_layout.addLayout(action_row)
        zoom_group_layout.addWidget(self.zoom_status_label)
        actions._configure_collapsible_group(zoom_group, checked=True)

        command_group = QtWidgets.QGroupBox("WIRIS SDK 명령 선택")
        command_form = QtWidgets.QGridLayout(command_group)
        command_form.setHorizontalSpacing(8)
        command_form.setColumnStretch(1, 1)
        command_form.setColumnStretch(2, 1)
        command_form.setColumnStretch(3, 1)
        self.wiris_command_target_combo = QtWidgets.QComboBox()
        self.wiris_command_target_combo.addItem("양쪽 카메라", "both")
        self.wiris_command_target_combo.addItem("왼쪽 카메라", "left")
        self.wiris_command_target_combo.addItem("오른쪽 카메라", "right")
        self.wiris_command_combo = QtWidgets.QComboBox()
        for category, label, command, terminator, description in WIRIS_COMMAND_CATALOG:
            display = f"{category} / {label}"
            self.wiris_command_combo.addItem(display, (command, terminator, description))
        self.wiris_command_combo.currentIndexChanged.connect(actions._apply_selected_wiris_command)
        self.wiris_command_edit = QtWidgets.QLineEdit()
        self.wiris_command_edit.setPlaceholderText("SDK 명령 예: HIWS")
        self.wiris_command_terminator_combo = QtWidgets.QComboBox()
        self.wiris_command_terminator_combo.addItem("LINE", LINE_TERMINATOR)
        self.wiris_command_terminator_combo.addItem("NULL", NUL_TERMINATOR)
        self.wiris_command_send_button = QtWidgets.QPushButton("명령 전송")
        self.wiris_command_send_button.clicked.connect(actions.send_selected_wiris_command)
        self.wiris_command_status_label = QtWidgets.QLabel("명령을 선택하면 아래 입력칸에 삽입됩니다.")
        self.wiris_command_status_label.setObjectName("statusStrip")
        self.wiris_command_status_label.setWordWrap(True)
        self.wiris_command_log = QtWidgets.QPlainTextEdit()
        self.wiris_command_log.setReadOnly(True)
        self.wiris_command_log.setMaximumBlockCount(400)
        command_form.addWidget(QtWidgets.QLabel("대상"), 0, 0)
        command_form.addWidget(self.wiris_command_target_combo, 0, 1)
        command_form.addWidget(QtWidgets.QLabel("명령"), 1, 0)
        command_form.addWidget(self.wiris_command_combo, 1, 1, 1, 3)
        command_form.addWidget(QtWidgets.QLabel("편집"), 2, 0)
        command_form.addWidget(self.wiris_command_edit, 2, 1, 1, 3)
        command_form.addWidget(QtWidgets.QLabel("종료 문자"), 3, 0)
        command_form.addWidget(self.wiris_command_terminator_combo, 3, 1)
        command_form.addWidget(self.wiris_command_send_button, 3, 2)
        command_form.addWidget(self.wiris_command_status_label, 4, 0, 1, 4)
        self.zoom_group = zoom_group
        self.command_group = command_group

        if isinstance(parent, QtWidgets.QWidget):
            self.zoom_group.setParent(parent)
            self.command_group.setParent(parent)
            self.wiris_command_log.setParent(parent)

    def install(self, layout):
        layout.addWidget(self.command_group)
        layout.addWidget(self.wiris_command_log, 1)
