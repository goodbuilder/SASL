"""Pixhawk connection widgets, candidate selection and layout owned by Diagnostics.

Only captured settings and explicit intent callbacks cross this view boundary.
P1/P2 roles, MAVLink addresses/bauds and stored telemetry labels retain their
existing meaning; this module performs no position or attitude conversion.
"""

from __future__ import annotations
from collections import deque
from dataclasses import dataclass, field
from typing import Callable
from PyQt5 import QtCore, QtWidgets
from Workstation.panel.panel_support import STARTUP_PIXHAWK_STATUS


@dataclass(frozen=True)
class PixhawkConnectionSettings:
    drone_pixhawk_bootstrap_baud: object
    drone_pixhawk_bootstrap_url: object
    left_mavlink_baud: object
    left_mavlink_url: object
    left_pixhawk_bootstrap_baud: object
    left_pixhawk_bootstrap_url: object
    left_pixhawk_eth_ip: object
    mavlink_stream_rate: object
    pixhawk_ethernet_mode: object
    pixhawk_host_address: object
    right_mavlink_baud: object
    right_mavlink_url: object
    right_pixhawk_bootstrap_baud: object
    right_pixhawk_bootstrap_url: object
    right_pixhawk_eth_ip: object


@dataclass(frozen=True)
class PixhawkConnectionCommands:
    discover_usb_pixhawk_connections: Callable
    discover_mavlink_url_candidates: Callable
    apply_selected_pixhawk_candidate: Callable
    swap_pixhawks: Callable
    discover_pc_pixhawk_ip: Callable
    toggle_mavlink_connection: Callable
    bootstrap_ethernet_mavlink_from_usb: Callable
    open_px4_shell: Callable
    apply_persistent_pixhawk_eth: Callable
    reboot_pixhawk_from_mavlink: Callable
    _refresh_pixhawk_ethernet_mode_controls: Callable

    def __post_init__(self):
        if not all(callable(value) for value in vars(self).values()):
            raise TypeError("Pixhawk connection view requires explicit command callbacks")


@dataclass
class PixhawkCandidateModel:
    rows: list = field(default_factory=list)
    by_source: dict = field(default_factory=lambda: {"pc": [], "network": [], "mavlink": []})
    revision: int = 0
    status: str = "NOT RUN"


def swap_spinbox_value(left: QtWidgets.QAbstractSpinBox, right: QtWidgets.QAbstractSpinBox) -> object:
    left_value = left.value()
    right_value = right.value()
    right.setValue(left_value)
    return right_value


def swap_line_edit_text(left: QtWidgets.QLineEdit, right: QtWidgets.QLineEdit) -> str:
    left_text = left.text()
    right_text = right.text()
    right.setText(left_text)
    return right_text


def sample_hz(timestamps: deque[float]) -> float:
    if len(timestamps) < 2:
        return 0.0
    elapsed_s = timestamps[-1] - timestamps[0]
    if elapsed_s <= 0.0:
        return 0.0
    return (len(timestamps) - 1) / elapsed_s


def build_mavlink_baud_spin() -> QtWidgets.QSpinBox:
    spin = QtWidgets.QSpinBox()
    spin.setRange(1_200, 4_000_000)
    spin.setValue(57_600)
    spin.setSingleStep(9_600)
    return spin


PIXHAWK_CONNECTION_VIEW_FIELDS = (
    "apply_pixhawk_candidate_host_button",
    "apply_pixhawk_candidate_left_button",
    "apply_pixhawk_candidate_right_button",
    "clear_pixhawk_candidate_button",
    "clear_pixhawk_pc_candidate_button",
    "detect_pixhawk_host_ip_button",
    "drone_pixhawk_bootstrap_baud_spin",
    "drone_pixhawk_bootstrap_url_edit",
    "left_mavlink_baud_spin",
    "left_mavlink_bootstrap_button",
    "left_mavlink_button",
    "left_mavlink_url_edit",
    "left_persist_eth_button",
    "left_pixhawk_bootstrap_baud_spin",
    "left_pixhawk_bootstrap_url_edit",
    "left_pixhawk_eth_ip_edit",
    "left_pixhawk_workflow_group",
    "left_px4_shell_button",
    "left_reboot_pixhawk_button",
    "mavlink_configure_streams_checkbox",
    "mavlink_pixhawk_candidate_table",
    "mavlink_stream_rate_spin",
    "pc_pixhawk_candidate_table",
    "pixhawk_advanced_diagnostics_group",
    "pixhawk_candidate_group",
    "pixhawk_candidate_table",
    "pixhawk_common_setup_group",
    "pixhawk_connection_actions_group",
    "pixhawk_connection_label",
    "pixhawk_connection_scroll",
    "pixhawk_ethernet_mode_combo",
    "pixhawk_host_address_edit",
    "pixhawk_log",
    "pixhawk_log_splitter",
    "pixhawk_mavlink_candidate_group",
    "pixhawk_mavlink_rtcm_group",
    "pixhawk_mavlink_url_candidate_button",
    "pixhawk_pair_settings_group",
    "pixhawk_pc_candidate_group",
    "pixhawk_usb_discover_button",
    "right_mavlink_baud_spin",
    "right_mavlink_bootstrap_button",
    "right_mavlink_button",
    "right_mavlink_url_edit",
    "right_persist_eth_button",
    "right_pixhawk_bootstrap_baud_spin",
    "right_pixhawk_bootstrap_url_edit",
    "right_pixhawk_eth_ip_edit",
    "right_pixhawk_workflow_group",
    "right_px4_shell_button",
    "right_reboot_pixhawk_button",
    "swap_pixhawk_button",
)
PIXHAWK_CANDIDATE_VIEW_FIELDS = (
    "_pixhawk_candidate_rows",
    "_pixhawk_candidate_rows_by_source",
    "pixhawk_discovery_revision",
    "workstation_pixhawk_discovery_status",
)


def _candidate_field(name):
    return property(lambda view: getattr(view.model, name), lambda view, value: setattr(view.model, name, value))


class PixhawkConnectionView(QtCore.QObject):
    _pixhawk_candidate_rows = _candidate_field("rows")
    _pixhawk_candidate_rows_by_source = _candidate_field("by_source")
    pixhawk_discovery_revision = _candidate_field("revision")
    workstation_pixhawk_discovery_status = _candidate_field("status")

    def __init__(
        self,
        settings: PixhawkConnectionSettings,
        commands: PixhawkConnectionCommands,
        *,
        build_toggle,
        log,
        parent=None,
    ):
        super().__init__(parent)
        self.commands = commands
        self._log = log
        self.model = PixhawkCandidateModel()
        self.pixhawk_host_address_edit = QtWidgets.QLineEdit(settings.pixhawk_host_address)
        self.pixhawk_usb_discover_button = QtWidgets.QPushButton("Scan USB Pixhawks")
        self.pixhawk_usb_discover_button.clicked.connect(self.commands.discover_usb_pixhawk_connections)
        self.pixhawk_mavlink_url_candidate_button = QtWidgets.QPushButton("Generate Ethernet MAVLink Candidates")
        self.pixhawk_mavlink_url_candidate_button.clicked.connect(self.commands.discover_mavlink_url_candidates)

        def build_pixhawk_candidate_table() -> QtWidgets.QTableWidget:
            table = QtWidgets.QTableWidget(0, 5)
            table.setHorizontalHeaderLabels(["종류", "값", "Baud", "상태", "인터페이스"])
            table.verticalHeader().setVisible(False)
            table.setSelectionBehavior(QtWidgets.QAbstractItemView.SelectRows)
            table.setSelectionMode(QtWidgets.QAbstractItemView.SingleSelection)
            table.setEditTriggers(QtWidgets.QAbstractItemView.NoEditTriggers)
            table.setHorizontalScrollBarPolicy(QtCore.Qt.ScrollBarAlwaysOff)
            table.setSizePolicy(QtWidgets.QSizePolicy.Ignored, QtWidgets.QSizePolicy.Fixed)
            table.setFixedHeight(240)
            table.itemSelectionChanged.connect(self._refresh_pixhawk_candidate_actions)
            header = table.horizontalHeader()
            header.setMinimumSectionSize(32)
            header.setSectionResizeMode(QtWidgets.QHeaderView.Stretch)
            header.setStretchLastSection(False)
            return table

        self.pc_pixhawk_candidate_table = build_pixhawk_candidate_table()
        self.mavlink_pixhawk_candidate_table = build_pixhawk_candidate_table()
        self.pixhawk_candidate_table = self.mavlink_pixhawk_candidate_table
        self.clear_pixhawk_candidate_button = QtWidgets.QPushButton("Clear")
        self.clear_pixhawk_candidate_button.clicked.connect(
            lambda _checked=False: self.clear_pixhawk_candidate_history("mavlink")
        )
        self.clear_pixhawk_candidate_button.setEnabled(False)
        self.clear_pixhawk_candidate_button.setToolTip("Pixhawk 탐색 후보 내역을 지웁니다.")
        self.clear_pixhawk_pc_candidate_button = QtWidgets.QPushButton("Clear")
        self.clear_pixhawk_pc_candidate_button.clicked.connect(
            lambda _checked=False: self.clear_pixhawk_candidate_history("pc")
        )
        self.clear_pixhawk_pc_candidate_button.setEnabled(False)
        self.clear_pixhawk_pc_candidate_button.setToolTip("Pixhawk PC 후보 내역을 지웁니다.")
        self.apply_pixhawk_candidate_host_button = QtWidgets.QPushButton("Apply to Host")
        self.apply_pixhawk_candidate_host_button.setEnabled(False)
        self.apply_pixhawk_candidate_host_button.clicked.connect(
            lambda _checked=False: self.commands.apply_selected_pixhawk_candidate("host")
        )
        self.apply_pixhawk_candidate_left_button = QtWidgets.QPushButton("Apply to P1")
        self.apply_pixhawk_candidate_left_button.setEnabled(False)
        self.apply_pixhawk_candidate_left_button.clicked.connect(
            lambda _checked=False: self.commands.apply_selected_pixhawk_candidate("left")
        )
        self.apply_pixhawk_candidate_right_button = QtWidgets.QPushButton("Apply to P2")
        self.apply_pixhawk_candidate_right_button.setEnabled(False)
        self.apply_pixhawk_candidate_right_button.clicked.connect(
            lambda _checked=False: self.commands.apply_selected_pixhawk_candidate("right")
        )
        for button in (
            self.pixhawk_usb_discover_button,
            self.pixhawk_mavlink_url_candidate_button,
            self.clear_pixhawk_candidate_button,
            self.clear_pixhawk_pc_candidate_button,
            self.apply_pixhawk_candidate_host_button,
            self.apply_pixhawk_candidate_left_button,
            self.apply_pixhawk_candidate_right_button,
        ):
            button.setMinimumWidth(0)
            button.setSizePolicy(QtWidgets.QSizePolicy.Ignored, QtWidgets.QSizePolicy.Fixed)
        self._pixhawk_candidate_rows: list[dict[str, object]] = []
        self._pixhawk_candidate_rows_by_source: dict[str, list[dict[str, object]]] = {
            "pc": [],
            "network": [],
            "mavlink": [],
        }
        self.pixhawk_discovery_revision = 0
        self.workstation_pixhawk_discovery_status = "NOT RUN"
        self.swap_pixhawk_button = QtWidgets.QPushButton("Swap P1 / P2 Roles")
        self.swap_pixhawk_button.clicked.connect(self.commands.swap_pixhawks)
        self.detect_pixhawk_host_ip_button = QtWidgets.QPushButton("Discover PC IP")
        self.detect_pixhawk_host_ip_button.clicked.connect(self.commands.discover_pc_pixhawk_ip)
        self.left_mavlink_button = QtWidgets.QPushButton("Connect P1 MAVLink")
        self.left_mavlink_button.clicked.connect(lambda _checked=False: self.commands.toggle_mavlink_connection("left"))
        self.right_mavlink_button = QtWidgets.QPushButton("Connect P2 MAVLink")
        self.right_mavlink_button.clicked.connect(
            lambda _checked=False: self.commands.toggle_mavlink_connection("right")
        )
        self.left_mavlink_bootstrap_button = QtWidgets.QPushButton("Start Ethernet MAVLink via USB (P1)")
        self.left_mavlink_bootstrap_button.clicked.connect(
            lambda _checked=False: self.commands.bootstrap_ethernet_mavlink_from_usb("left")
        )
        self.right_mavlink_bootstrap_button = QtWidgets.QPushButton("Start Ethernet MAVLink via USB (P2)")
        self.right_mavlink_bootstrap_button.clicked.connect(
            lambda _checked=False: self.commands.bootstrap_ethernet_mavlink_from_usb("right")
        )
        self.left_px4_shell_button = QtWidgets.QPushButton("P1 Shell")
        self.left_px4_shell_button.setEnabled(False)
        self.left_px4_shell_button.clicked.connect(lambda: self.commands.open_px4_shell("left"))
        self.left_persist_eth_button = QtWidgets.QPushButton("ETH 저장")
        self.left_persist_eth_button.setEnabled(False)
        self.left_persist_eth_button.clicked.connect(lambda: self.commands.apply_persistent_pixhawk_eth("left"))
        self.left_reboot_pixhawk_button = QtWidgets.QPushButton("P1 재부팅")
        self.left_reboot_pixhawk_button.setEnabled(False)
        self.left_reboot_pixhawk_button.clicked.connect(
            lambda _checked=False: self.commands.reboot_pixhawk_from_mavlink("left")
        )
        self.right_px4_shell_button = QtWidgets.QPushButton("P2 Shell")
        self.right_px4_shell_button.setEnabled(False)
        self.right_px4_shell_button.clicked.connect(lambda: self.commands.open_px4_shell("right"))
        self.right_persist_eth_button = QtWidgets.QPushButton("ETH 저장")
        self.right_persist_eth_button.setEnabled(False)
        self.right_persist_eth_button.clicked.connect(lambda: self.commands.apply_persistent_pixhawk_eth("right"))
        self.right_reboot_pixhawk_button = QtWidgets.QPushButton("P2 재부팅")
        self.right_reboot_pixhawk_button.setEnabled(False)
        self.right_reboot_pixhawk_button.clicked.connect(
            lambda _checked=False: self.commands.reboot_pixhawk_from_mavlink("right")
        )
        self.left_pixhawk_eth_ip_edit = QtWidgets.QLineEdit(settings.left_pixhawk_eth_ip)
        self.right_pixhawk_eth_ip_edit = QtWidgets.QLineEdit(settings.right_pixhawk_eth_ip)
        self.left_mavlink_url_edit = QtWidgets.QLineEdit(settings.left_mavlink_url)
        self.right_mavlink_url_edit = QtWidgets.QLineEdit(settings.right_mavlink_url)
        self.left_mavlink_baud_spin = build_mavlink_baud_spin()
        self.left_mavlink_baud_spin.setValue(settings.left_mavlink_baud)
        self.right_mavlink_baud_spin = build_mavlink_baud_spin()
        self.right_mavlink_baud_spin.setValue(settings.right_mavlink_baud)
        self.left_pixhawk_bootstrap_url_edit = QtWidgets.QLineEdit(settings.left_pixhawk_bootstrap_url)
        self.right_pixhawk_bootstrap_url_edit = QtWidgets.QLineEdit(settings.right_pixhawk_bootstrap_url)
        self.drone_pixhawk_bootstrap_url_edit = QtWidgets.QLineEdit(settings.drone_pixhawk_bootstrap_url)
        self.left_pixhawk_bootstrap_baud_spin = build_mavlink_baud_spin()
        self.left_pixhawk_bootstrap_baud_spin.setValue(settings.left_pixhawk_bootstrap_baud)
        self.right_pixhawk_bootstrap_baud_spin = build_mavlink_baud_spin()
        self.right_pixhawk_bootstrap_baud_spin.setValue(settings.right_pixhawk_bootstrap_baud)
        self.drone_pixhawk_bootstrap_baud_spin = build_mavlink_baud_spin()
        self.drone_pixhawk_bootstrap_baud_spin.setValue(settings.drone_pixhawk_bootstrap_baud)
        self.mavlink_configure_streams_checkbox = build_toggle(
            "Legacy auto MAV_2 setup (disabled)",
            False,
        )
        self.mavlink_configure_streams_checkbox.setEnabled(False)
        self.mavlink_stream_rate_spin = QtWidgets.QSpinBox()
        self.mavlink_stream_rate_spin.setRange(1_000, 10_000_000)
        self.mavlink_stream_rate_spin.setValue(settings.mavlink_stream_rate)
        self.pixhawk_ethernet_mode_combo = QtWidgets.QComboBox()
        self.pixhawk_ethernet_mode_combo.addItem("MAVLink만", "mavlink")
        mode_index = self.pixhawk_ethernet_mode_combo.findData(settings.pixhawk_ethernet_mode)
        self.pixhawk_ethernet_mode_combo.setCurrentIndex(max(0, mode_index))
        self.pixhawk_ethernet_mode_combo.currentIndexChanged.connect(
            lambda _index: self.commands._refresh_pixhawk_ethernet_mode_controls()
        )
        self.pixhawk_usb_discover_button.setText("Scan USB Pixhawks")
        self.pixhawk_mavlink_url_candidate_button.setText("Generate Ethernet MAVLink Candidates")
        self.detect_pixhawk_host_ip_button.setText("Discover PC IP")
        self.clear_pixhawk_candidate_button.setText("Clear")
        self.apply_pixhawk_candidate_host_button.setText("Apply to Host")
        self.apply_pixhawk_candidate_left_button.setText("Apply to P1")
        self.apply_pixhawk_candidate_right_button.setText("Apply to P2")
        self.swap_pixhawk_button.setText("Swap P1 / P2 Roles")
        self.left_mavlink_button.setText("Connect P1 MAVLink")
        self.right_mavlink_button.setText("Connect P2 MAVLink")
        self.left_mavlink_bootstrap_button.setText("Start Ethernet MAVLink via USB (P1)")
        self.right_mavlink_bootstrap_button.setText("Start Ethernet MAVLink via USB (P2)")
        self.left_px4_shell_button.setText("P1 Shell")
        self.right_px4_shell_button.setText("P2 Shell")
        self.left_persist_eth_button.setText("ETH 저장")
        self.right_persist_eth_button.setText("ETH 저장")
        self.left_reboot_pixhawk_button.setText("P1 재부팅")
        self.right_reboot_pixhawk_button.setText("P2 재부팅")

        if isinstance(parent, QtWidgets.QWidget):
            # Include controls such as the hidden Drone USB settings which do
            # not join the legacy layout; their Qt lifetime still belongs here.
            for name in PIXHAWK_CONNECTION_VIEW_FIELDS:
                widget = getattr(self, name, None)
                if isinstance(widget, QtWidgets.QWidget) and widget.parentWidget() is None:
                    widget.setParent(parent)

    def build_layout(self, pixhawk_connection_layout, pixhawk_form, pixhawk_group):
        pixhawk_common_group = QtWidgets.QGroupBox("0 공통 네트워크")
        pixhawk_common_group.setObjectName("pixhawkCommonSetupGroup")
        pixhawk_common_form = QtWidgets.QGridLayout(pixhawk_common_group)
        pixhawk_common_form.setHorizontalSpacing(8)
        pixhawk_common_form.setVerticalSpacing(8)
        pixhawk_common_form.setContentsMargins(12, 24, 12, 12)
        pixhawk_common_form.setColumnStretch(1, 1)
        pixhawk_common_form.addWidget(QtWidgets.QLabel("픽스호크 Host IP"), 0, 0)
        pixhawk_common_form.addWidget(self.pixhawk_host_address_edit, 0, 1)
        self.swap_pixhawk_button.setMinimumWidth(128)
        self.swap_pixhawk_button.setSizePolicy(QtWidgets.QSizePolicy.Ignored, QtWidgets.QSizePolicy.Fixed)
        pixhawk_common_form.addWidget(self.swap_pixhawk_button, 1, 1, QtCore.Qt.AlignRight)
        self.pixhawk_common_setup_group = pixhawk_common_group
        pixhawk_form.addWidget(pixhawk_common_group, 0, 0, 1, 6)

        pixhawk_candidate_group = QtWidgets.QGroupBox("1 연결 후보")
        pixhawk_candidate_group.setObjectName("pixhawkCandidateGroup")
        pixhawk_candidate_layout = QtWidgets.QVBoxLayout(pixhawk_candidate_group)
        pixhawk_candidate_layout.setSpacing(10)
        pixhawk_candidate_layout.setContentsMargins(12, 24, 12, 12)

        pc_candidate_group = QtWidgets.QGroupBox("PC / Host IP")
        pc_candidate_group.setObjectName("pixhawkPcCandidateGroup")
        pc_candidate_group.setSizePolicy(QtWidgets.QSizePolicy.Ignored, QtWidgets.QSizePolicy.Preferred)
        pc_candidate_group.setMinimumHeight(320)
        pc_candidate_layout = QtWidgets.QVBoxLayout(pc_candidate_group)
        pc_candidate_layout.setSpacing(6)
        pc_candidate_layout.setContentsMargins(8, 18, 8, 8)
        pc_candidate_layout.addWidget(self.detect_pixhawk_host_ip_button)
        pc_candidate_layout.addWidget(self.pc_pixhawk_candidate_table)
        pc_candidate_actions = QtWidgets.QHBoxLayout()
        pc_candidate_actions.setSpacing(6)
        pc_candidate_actions.addWidget(self.clear_pixhawk_pc_candidate_button, 1)
        pc_candidate_actions.addWidget(self.apply_pixhawk_candidate_host_button, 1)
        pc_candidate_layout.addLayout(pc_candidate_actions)

        mavlink_candidate_group = QtWidgets.QGroupBox("MAVLink / USB 제어")
        mavlink_candidate_group.setObjectName("pixhawkMavlinkCandidateGroup")
        mavlink_candidate_group.setSizePolicy(QtWidgets.QSizePolicy.Ignored, QtWidgets.QSizePolicy.Preferred)
        mavlink_candidate_group.setMinimumHeight(350)
        mavlink_candidate_layout = QtWidgets.QVBoxLayout(mavlink_candidate_group)
        mavlink_candidate_layout.setSpacing(6)
        mavlink_candidate_layout.setContentsMargins(8, 18, 8, 8)
        mavlink_candidate_discovery_actions = QtWidgets.QHBoxLayout()
        mavlink_candidate_discovery_actions.setSpacing(6)
        mavlink_candidate_discovery_actions.addWidget(self.pixhawk_usb_discover_button, 1)
        mavlink_candidate_discovery_actions.addWidget(self.pixhawk_mavlink_url_candidate_button, 1)
        mavlink_candidate_layout.addLayout(mavlink_candidate_discovery_actions)
        mavlink_candidate_layout.addWidget(self.mavlink_pixhawk_candidate_table)
        mavlink_candidate_actions = QtWidgets.QHBoxLayout()
        mavlink_candidate_actions.setSpacing(6)
        mavlink_candidate_actions.addWidget(self.clear_pixhawk_candidate_button, 1)
        mavlink_candidate_actions.addWidget(self.apply_pixhawk_candidate_left_button, 1)
        mavlink_candidate_actions.addWidget(self.apply_pixhawk_candidate_right_button, 1)
        mavlink_candidate_layout.addLayout(mavlink_candidate_actions)

        pixhawk_candidate_layout.addWidget(pc_candidate_group)
        pixhawk_candidate_layout.addWidget(mavlink_candidate_group)
        self.pixhawk_pc_candidate_group = pc_candidate_group
        self.pixhawk_mavlink_candidate_group = mavlink_candidate_group
        self.pixhawk_candidate_group = pixhawk_candidate_group
        pixhawk_form.addWidget(pixhawk_candidate_group, 1, 0, 1, 6)

        def build_pixhawk_mavlink_side_group(
            title: str,
            object_name: str,
            eth_ip_edit: QtWidgets.QLineEdit,
            mavlink_url_edit: QtWidgets.QLineEdit,
            baud_spin: QtWidgets.QSpinBox,
            bootstrap_url_edit: QtWidgets.QLineEdit,
            bootstrap_baud_spin: QtWidgets.QSpinBox,
        ) -> QtWidgets.QGroupBox:
            side_group = QtWidgets.QGroupBox(title)
            side_group.setObjectName(object_name)
            side_form = QtWidgets.QGridLayout(side_group)
            side_form.setHorizontalSpacing(8)
            side_form.setVerticalSpacing(10)
            side_form.setContentsMargins(12, 24, 12, 12)
            side_form.setColumnStretch(0, 0)
            side_form.setColumnStretch(1, 1)
            for row, label_text, field_widget in (
                (0, "MAVLink URL", mavlink_url_edit),
                (1, "MAVLink Baud", baud_spin),
                (2, "USB 제어 포트", bootstrap_url_edit),
                (3, "USB Baud", bootstrap_baud_spin),
                (4, "ETH IP", eth_ip_edit),
            ):
                label = QtWidgets.QLabel(label_text)
                label.setAlignment(QtCore.Qt.AlignRight | QtCore.Qt.AlignVCenter)
                side_form.addWidget(label, row, 0)
                side_form.addWidget(field_widget, row, 1)
            return side_group

        pixhawk_mavlink_group = QtWidgets.QGroupBox("2 Rover MAVLink / RTCM")
        pixhawk_mavlink_group.setObjectName("pixhawkMavlinkRtcmGroup")
        pixhawk_mavlink_layout = QtWidgets.QVBoxLayout(pixhawk_mavlink_group)
        pixhawk_mavlink_layout.setSpacing(8)
        pixhawk_mavlink_layout.setContentsMargins(12, 24, 12, 12)
        pixhawk_mavlink_options = QtWidgets.QGridLayout()
        pixhawk_mavlink_options.setHorizontalSpacing(8)
        pixhawk_mavlink_options.setVerticalSpacing(8)
        pixhawk_mavlink_options.setColumnStretch(1, 1)
        pixhawk_mavlink_options.addWidget(QtWidgets.QLabel("MAV 속도"), 0, 0)
        pixhawk_mavlink_options.addWidget(self.mavlink_stream_rate_spin, 0, 1)
        pixhawk_mavlink_options.addWidget(self.mavlink_configure_streams_checkbox, 0, 2, 1, 2)
        pixhawk_mavlink_options.addWidget(QtWidgets.QLabel("Ethernet 서비스"), 1, 0)
        pixhawk_mavlink_options.addWidget(self.pixhawk_ethernet_mode_combo, 1, 1, 1, 3)
        pixhawk_mavlink_pair_layout = QtWidgets.QHBoxLayout()
        pixhawk_mavlink_pair_layout.setSpacing(8)
        left_pixhawk_group = build_pixhawk_mavlink_side_group(
            "Pixhawk 1 MAVLink",
            "leftPixhawkMavlinkWorkflowGroup",
            self.left_pixhawk_eth_ip_edit,
            self.left_mavlink_url_edit,
            self.left_mavlink_baud_spin,
            self.left_pixhawk_bootstrap_url_edit,
            self.left_pixhawk_bootstrap_baud_spin,
        )
        right_pixhawk_group = build_pixhawk_mavlink_side_group(
            "Pixhawk 2 MAVLink",
            "rightPixhawkMavlinkWorkflowGroup",
            self.right_pixhawk_eth_ip_edit,
            self.right_mavlink_url_edit,
            self.right_mavlink_baud_spin,
            self.right_pixhawk_bootstrap_url_edit,
            self.right_pixhawk_bootstrap_baud_spin,
        )
        pixhawk_mavlink_pair_layout.addWidget(left_pixhawk_group, 1)
        pixhawk_mavlink_pair_layout.addWidget(right_pixhawk_group, 1)
        pixhawk_mavlink_actions = QtWidgets.QHBoxLayout()
        pixhawk_mavlink_actions.setSpacing(6)
        for button in (
            self.left_mavlink_button,
            self.right_mavlink_button,
            self.left_mavlink_bootstrap_button,
            self.right_mavlink_bootstrap_button,
        ):
            button.setMinimumWidth(128)
            button.setSizePolicy(QtWidgets.QSizePolicy.Ignored, QtWidgets.QSizePolicy.Fixed)
            pixhawk_mavlink_actions.addWidget(button, 1)
        pixhawk_mavlink_layout.addLayout(pixhawk_mavlink_options)
        pixhawk_mavlink_layout.addLayout(pixhawk_mavlink_pair_layout)
        pixhawk_mavlink_layout.addLayout(pixhawk_mavlink_actions)
        self.left_pixhawk_workflow_group = left_pixhawk_group
        self.right_pixhawk_workflow_group = right_pixhawk_group
        self.pixhawk_mavlink_rtcm_group = pixhawk_mavlink_group
        self.pixhawk_pair_settings_group = pixhawk_mavlink_group
        pixhawk_form.addWidget(pixhawk_mavlink_group, 2, 0, 1, 6)

        self.pixhawk_connection_actions_group = pixhawk_mavlink_group

        pixhawk_advanced_group = QtWidgets.QGroupBox("4 고급 / 진단")
        pixhawk_advanced_group.setObjectName("pixhawkAdvancedDiagnosticsGroup")
        pixhawk_advanced_actions = QtWidgets.QHBoxLayout(pixhawk_advanced_group)
        pixhawk_advanced_actions.setSpacing(6)
        pixhawk_advanced_actions.setContentsMargins(12, 24, 12, 12)
        for button in (
            self.left_px4_shell_button,
            self.right_px4_shell_button,
            self.left_persist_eth_button,
            self.right_persist_eth_button,
            self.left_reboot_pixhawk_button,
            self.right_reboot_pixhawk_button,
        ):
            pixhawk_advanced_actions.addWidget(button, 1)
        self.pixhawk_advanced_diagnostics_group = pixhawk_advanced_group
        pixhawk_form.addWidget(pixhawk_advanced_group, 3, 0, 1, 6)
        self.pixhawk_connection_label = QtWidgets.QLabel(STARTUP_PIXHAWK_STATUS)
        self.pixhawk_connection_label.setObjectName("statusStrip")
        pixhawk_form.addWidget(self.pixhawk_connection_label, 4, 0, 1, 6)

        pixhawk_log_group = QtWidgets.QGroupBox("Pixhawk 로그")
        pixhawk_log_layout = QtWidgets.QVBoxLayout(pixhawk_log_group)
        self.pixhawk_log = QtWidgets.QPlainTextEdit()
        self.pixhawk_log.setReadOnly(True)
        self.pixhawk_log.setMaximumBlockCount(600)
        pixhawk_log_layout.addWidget(self.pixhawk_log)
        pixhawk_scroll = QtWidgets.QScrollArea()
        pixhawk_scroll.setWidgetResizable(True)
        pixhawk_scroll.setFrameShape(QtWidgets.QFrame.NoFrame)
        pixhawk_scroll.setWidget(pixhawk_group)
        self.pixhawk_connection_scroll = pixhawk_scroll
        pixhawk_splitter = QtWidgets.QSplitter(QtCore.Qt.Vertical)
        pixhawk_splitter.addWidget(pixhawk_scroll)
        pixhawk_splitter.addWidget(pixhawk_log_group)
        pixhawk_splitter.setStretchFactor(0, 2)
        pixhawk_splitter.setStretchFactor(1, 1)
        pixhawk_splitter.setChildrenCollapsible(False)
        pixhawk_splitter.setSizes([620, 120])
        self.pixhawk_log_splitter = pixhawk_splitter
        pixhawk_connection_layout.addWidget(pixhawk_splitter, 1)

    def _pixhawk_candidate_table_for_source(self, source: str) -> QtWidgets.QTableWidget:
        if source == "pc":
            return self.pc_pixhawk_candidate_table
        return self.mavlink_pixhawk_candidate_table

    def _pixhawk_candidate_source_for_kind(self, kind: str) -> str:
        if kind == "PC_IP":
            return "pc"
        if kind == "NETWORK":
            return "network"
        return "mavlink"

    def _sync_legacy_pixhawk_candidate_rows(self) -> None:
        self._pixhawk_candidate_rows = [
            *self._pixhawk_candidate_rows_by_source.get("pc", []),
            *self._pixhawk_candidate_rows_by_source.get("network", []),
            *self._pixhawk_candidate_rows_by_source.get("mavlink", []),
        ]

    def _set_pixhawk_candidate_rows(self, rows: list[dict[str, object]], *, source: str | None = None) -> None:
        if source is None:
            grouped: dict[str, list[dict[str, object]]] = {
                "pc": [],
                "network": [],
                "mavlink": [],
            }
            for entry in rows:
                grouped[self._pixhawk_candidate_source_for_kind(str(entry.get("kind", "")))].append(entry)
            self._pixhawk_candidate_rows_by_source = grouped
            for key, source_rows in grouped.items():
                self._render_pixhawk_candidate_rows(key, source_rows)
        else:
            self._pixhawk_candidate_rows_by_source[source] = rows
            self._render_pixhawk_candidate_rows(source, rows)
        self._sync_legacy_pixhawk_candidate_rows()
        if self._pixhawk_candidate_rows:
            self.workstation_pixhawk_discovery_status = f"FOUND {len(self._pixhawk_candidate_rows)} ENDPOINTS"
        self._refresh_pixhawk_candidate_actions()

    def _render_pixhawk_candidate_rows(self, source: str, rows: list[dict[str, object]]) -> None:
        table = self._pixhawk_candidate_table_for_source(source)
        table.setRowCount(len(rows))
        for row, entry in enumerate(rows):
            for column, key in enumerate(("kind", "value", "baud", "status", "interface")):
                item = QtWidgets.QTableWidgetItem(str(entry.get(key, "")))
                table.setItem(row, column, item)
        table.resizeRowsToContents()

    def _append_pixhawk_candidate_rows(self, rows: list[dict[str, object]], *, source: str | None = None) -> None:
        if source is None:
            for entry in rows:
                entry_source = self._pixhawk_candidate_source_for_kind(str(entry.get("kind", "")))
                current = self._pixhawk_candidate_rows_by_source.get(entry_source, [])
                self._set_pixhawk_candidate_rows([*current, entry], source=entry_source)
            return
        current = self._pixhawk_candidate_rows_by_source.get(source, [])
        self._set_pixhawk_candidate_rows([*current, *rows], source=source)

    def clear_pixhawk_candidate_history(self, source: str | None = None) -> None:
        if source is not None:
            if not self._pixhawk_candidate_rows_by_source.get(source):
                return
            self._set_pixhawk_candidate_rows([], source=source)
            self._log("[픽스호크] 탐색 후보 내역을 지웠습니다.")
            return
        if not self._pixhawk_candidate_rows_by_source.get("mavlink"):
            return
        self._set_pixhawk_candidate_rows([], source="mavlink")
        self._log("[픽스호크] 탐색 후보 내역을 지웠습니다.")

    def _refresh_pixhawk_candidate_actions(self) -> None:
        pc_selected = self.pc_pixhawk_candidate_table.currentRow() >= 0
        mavlink_row = self.mavlink_pixhawk_candidate_table.currentRow()
        mavlink_rows = self._pixhawk_candidate_rows_by_source.get("mavlink", [])
        mavlink_kind = ""
        if 0 <= mavlink_row < len(mavlink_rows):
            mavlink_kind = str(mavlink_rows[mavlink_row].get("kind", ""))
        self.clear_pixhawk_pc_candidate_button.setEnabled(bool(self._pixhawk_candidate_rows_by_source.get("pc")))
        self.clear_pixhawk_candidate_button.setEnabled(bool(mavlink_rows))
        self.clear_pixhawk_candidate_button.setToolTip("Pixhawk MAVLink/USB 후보 내역을 지웁니다.")
        self.apply_pixhawk_candidate_host_button.setEnabled(pc_selected)
        self.apply_pixhawk_candidate_host_button.setToolTip(
            "선택한 PC IP 후보를 Pixhawk Host IP에 적용합니다."
            if pc_selected
            else "PC_IP 후보를 선택하면 Host에 적용할 수 있습니다."
        )
        side_enabled = mavlink_kind in {"USB", "MAVLink"}
        for button in (self.apply_pixhawk_candidate_left_button, self.apply_pixhawk_candidate_right_button):
            button.setEnabled(side_enabled)
            if mavlink_kind == "USB":
                button.setToolTip(
                    "선택한 USB 후보를 P1/P2 USB 부트스트랩 포트에 적용합니다. Ethernet MAVLink URL은 유지됩니다."
                )
            elif mavlink_kind == "MAVLink":
                button.setToolTip(
                    "선택한 MAVLink URL 후보를 P1/P2 MAVLink URL에 적용합니다. 연결은 적용 후 직접 누르세요."
                )
            else:
                button.setToolTip("USB 또는 MAVLink 후보를 선택하면 P1/P2에 적용할 수 있습니다.")

    def _mavlink_url_candidate_rows(self) -> list[dict[str, object]]:
        host_ip = self.pixhawk_host_address_edit.text().strip() or "0.0.0.0"
        left_eth_ip = self.left_pixhawk_eth_ip_edit.text().strip().split("/", 1)[0]
        right_eth_ip = self.right_pixhawk_eth_ip_edit.text().strip().split("/", 1)[0]
        rows = [
            {
                "kind": "MAVLink",
                "value": "udpin:0.0.0.0:14550",
                "baud": "",
                "status": "Ethernet UDP listen",
                "interface": host_ip,
            },
            {
                "kind": "MAVLink",
                "value": "udpin:0.0.0.0:14551",
                "baud": "",
                "status": "Ethernet UDP listen",
                "interface": host_ip,
            },
        ]
        for side_label, eth_ip, udp_port in (
            ("P1", left_eth_ip, 14550),
            ("P2", right_eth_ip, 14551),
        ):
            if not eth_ip:
                continue
            rows.extend(
                [
                    {
                        "kind": "MAVLink",
                        "value": f"tcp:{eth_ip}:5760",
                        "baud": "",
                        "status": f"{side_label} Ethernet TCP",
                        "interface": host_ip,
                    },
                    {
                        "kind": "MAVLink",
                        "value": f"udpout:{eth_ip}:{udp_port}",
                        "baud": "",
                        "status": f"{side_label} Ethernet UDP out",
                        "interface": host_ip,
                    },
                ]
            )
        return rows

    def _selected_pixhawk_candidate(self, source: str | None = None) -> dict[str, object] | None:
        if source is None:
            source = "mavlink"
        table = self._pixhawk_candidate_table_for_source(source)
        rows = self._pixhawk_candidate_rows_by_source.get(source, [])
        row = table.currentRow()
        if row < 0 or row >= len(rows):
            self._log("[픽스호크] 탐색 후보를 먼저 선택하세요.")
            return None
        return rows[row]
