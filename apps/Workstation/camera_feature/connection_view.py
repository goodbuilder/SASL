"""Camera endpoint/decoder controls own their widgets and captured settings.

Only explicit user-intent callbacks leave this view; device/network ownership
and stream generation remain at the runtime. Native pixel/clock policy labels
and existing control ranges are unchanged.
"""

from dataclasses import dataclass
from typing import Callable
from PyQt5 import QtCore, QtWidgets
from Workstation.panel.panel_controls import NoWheelSpinBox
from Workstation.panel.panel_support import STARTUP_CAMERA_STATUS
from Workstation.workstation_core.network import DEFAULT_CAMERA_URLS, DEFAULT_HOST_ADDRESS, DEFAULT_INTERFACE
from Workstation.workstation_core.media_stream.policy import (
    DECODER_AUTO,
    DECODER_NVIDIA,
    DECODER_SOFTWARE,
    RTSP_TRANSPORT_TCP,
    RTSP_TRANSPORT_UDP,
    STREAM_BACKEND_GSTREAMER,
    STREAM_BACKEND_OPENCV,
    STREAM_BACKEND_OPENCV_AUTO,
    STREAM_BACKEND_PYAV,
)


@dataclass(frozen=True)
class CameraConnectionSettings:
    camera_host_address: object
    interface_name: object
    left_control_host: object
    left_control_port: object
    left_rtsp_url: object
    left_stip_ip: object
    right_control_host: object
    right_control_port: object
    right_rtsp_url: object
    right_stip_ip: object
    rtsp_decoder: object
    rtsp_latency_ms: object
    rtsp_transport: object
    stip_reboot: object
    stip_subnet_mask: object
    stream_backend: object
    wiris_control_port: object
    wiris_control_timeout_s: object


@dataclass(frozen=True)
class CameraConnectionActions:
    _handle_decoder_mode_changed: Callable
    apply_control_settings: Callable
    apply_pc_camera_ip: Callable
    apply_rtsp_url: Callable
    apply_wiris_stip: Callable
    auto_connect_wiris: Callable
    check_cameras: Callable
    check_wiris_control: Callable
    detect_camera_control: Callable
    detect_camera_rtsp: Callable
    discover_pc_camera_ip: Callable
    enable_wiris_rtsp: Callable
    swap_cameras: Callable
    toggle_camera_connection: Callable

    def __post_init__(self):
        if not all(callable(value) for value in vars(self).values()):
            raise TypeError("Camera connection controls require explicit action callbacks")


CONNECTION_CONTROL_FIELDS = (
    "apply_pc_ip_button",
    "auto_wiris_connect_button",
    "camera_connect_button",
    "check_button",
    "check_control_button",
    "decoder_combo",
    "detect_control_button",
    "detect_pc_ip_button",
    "detect_rtsp_button",
    "enable_rtsp_button",
    "host_address_edit",
    "interface_edit",
    "left_control_apply_button",
    "left_control_port_spin",
    "left_rtsp_apply_button",
    "left_rtsp_ip_edit",
    "left_stip_button",
    "left_stip_ip_edit",
    "left_url_edit",
    "left_zoom_host_edit",
    "right_control_apply_button",
    "right_control_port_spin",
    "right_detect_control_button",
    "right_detect_rtsp_button",
    "right_rtsp_apply_button",
    "right_rtsp_ip_edit",
    "right_stip_button",
    "right_stip_ip_edit",
    "right_url_edit",
    "right_zoom_host_edit",
    "rtsp_latency_spin",
    "rtsp_transport_combo",
    "stip_mask_edit",
    "stip_reboot_checkbox",
    "stream_backend_combo",
    "swap_camera_button",
    "zoom_port_spin",
    "zoom_timeout_spin",
)

CONNECTION_VIEW_FIELDS = CONNECTION_CONTROL_FIELDS + (
    "wiris_pair_settings_group",
    "camera_connection_label",
    "wiris_log",
    "camera_connection_splitter",
)


class CameraConnectionControls(QtCore.QObject):
    def __init__(
        self,
        settings: CameraConnectionSettings,
        actions: CameraConnectionActions,
        *,
        rtsp_host_text,
        build_toggle,
        set_combo,
        parent=None,
    ):
        super().__init__(parent)
        self.actions = actions
        self.interface_edit = QtWidgets.QLineEdit(settings.interface_name or DEFAULT_INTERFACE)
        self.host_address_edit = QtWidgets.QLineEdit(settings.camera_host_address or DEFAULT_HOST_ADDRESS)
        self.left_url_edit = QtWidgets.QLineEdit(settings.left_rtsp_url or DEFAULT_CAMERA_URLS[0])
        self.right_url_edit = QtWidgets.QLineEdit(settings.right_rtsp_url or DEFAULT_CAMERA_URLS[1])
        self.left_rtsp_ip_edit = QtWidgets.QLineEdit(rtsp_host_text(self.left_url_edit.text()).split(":", 1)[0])
        self.right_rtsp_ip_edit = QtWidgets.QLineEdit(rtsp_host_text(self.right_url_edit.text()).split(":", 1)[0])
        self.left_rtsp_apply_button = QtWidgets.QPushButton("Apply Video URL")
        self.left_rtsp_apply_button.clicked.connect(lambda _checked=False: self.actions.apply_rtsp_url("left"))
        self.right_rtsp_apply_button = QtWidgets.QPushButton("Apply Video URL")
        self.right_rtsp_apply_button.clicked.connect(lambda _checked=False: self.actions.apply_rtsp_url("right"))
        self.left_control_apply_button = QtWidgets.QPushButton("Apply Control Port")
        self.left_control_apply_button.clicked.connect(
            lambda _checked=False: self.actions.apply_control_settings("left")
        )
        self.right_control_apply_button = QtWidgets.QPushButton("Apply Control Port")
        self.right_control_apply_button.clicked.connect(
            lambda _checked=False: self.actions.apply_control_settings("right")
        )
        self.left_zoom_host_edit = QtWidgets.QLineEdit(settings.left_control_host)
        self.right_zoom_host_edit = QtWidgets.QLineEdit(settings.right_control_host)
        self.left_stip_ip_edit = QtWidgets.QLineEdit(settings.left_stip_ip)
        self.right_stip_ip_edit = QtWidgets.QLineEdit(settings.right_stip_ip)
        self.stip_mask_edit = QtWidgets.QLineEdit(settings.stip_subnet_mask)
        self.stip_reboot_checkbox = build_toggle("STIP 후 재부팅", settings.stip_reboot)
        self.zoom_port_spin = NoWheelSpinBox()
        self.zoom_port_spin.setRange(1, 65535)
        self.zoom_port_spin.setValue(settings.wiris_control_port)
        self.left_control_port_spin = NoWheelSpinBox()
        self.left_control_port_spin.setRange(1, 65535)
        self.left_control_port_spin.setValue(settings.left_control_port)
        self.right_control_port_spin = NoWheelSpinBox()
        self.right_control_port_spin.setRange(1, 65535)
        self.right_control_port_spin.setValue(settings.right_control_port)
        self.zoom_timeout_spin = QtWidgets.QDoubleSpinBox()
        self.zoom_timeout_spin.setRange(0.1, 10.0)
        self.zoom_timeout_spin.setSingleStep(0.1)
        self.zoom_timeout_spin.setValue(settings.wiris_control_timeout_s)
        self.rtsp_latency_spin = QtWidgets.QSpinBox()
        self.rtsp_latency_spin.setRange(0, 5000)
        self.rtsp_latency_spin.setSingleStep(25)
        self.rtsp_latency_spin.setValue(settings.rtsp_latency_ms)
        self.decoder_combo = QtWidgets.QComboBox()
        self.decoder_combo.addItem("Auto", DECODER_AUTO)
        self.decoder_combo.addItem("NVIDIA", DECODER_NVIDIA)
        self.decoder_combo.addItem("Software (avdec_h264)", DECODER_SOFTWARE)
        set_combo(self.decoder_combo, settings.rtsp_decoder)
        self.decoder_combo.currentIndexChanged.connect(self.actions._handle_decoder_mode_changed)
        self.stream_backend_combo = QtWidgets.QComboBox()
        self.stream_backend_combo.addItem("GStreamer", STREAM_BACKEND_GSTREAMER)
        self.stream_backend_combo.addItem("OpenCV/FFmpeg", STREAM_BACKEND_OPENCV)
        self.stream_backend_combo.addItem("OpenCV Auto", STREAM_BACKEND_OPENCV_AUTO)
        self.stream_backend_combo.addItem("PyAV", STREAM_BACKEND_PYAV)
        set_combo(self.stream_backend_combo, settings.stream_backend)
        self.rtsp_transport_combo = QtWidgets.QComboBox()
        self.rtsp_transport_combo.addItem("UDP", RTSP_TRANSPORT_UDP)
        self.rtsp_transport_combo.addItem("TCP", RTSP_TRANSPORT_TCP)
        set_combo(self.rtsp_transport_combo, settings.rtsp_transport)
        self.detect_control_button = QtWidgets.QPushButton("Scan Control Port")
        self.detect_control_button.clicked.connect(lambda _checked=False: self.actions.detect_camera_control("left"))
        self.right_detect_control_button = QtWidgets.QPushButton("Scan Control Port")
        self.right_detect_control_button.clicked.connect(
            lambda _checked=False: self.actions.detect_camera_control("right")
        )
        self.detect_rtsp_button = QtWidgets.QPushButton("Scan Video URL")
        self.detect_rtsp_button.clicked.connect(self.actions.detect_camera_rtsp)
        self.right_detect_rtsp_button = QtWidgets.QPushButton("Scan Video URL")
        self.right_detect_rtsp_button.clicked.connect(self.actions.detect_camera_rtsp)
        self.auto_wiris_connect_button = QtWidgets.QPushButton("Auto-Connect Cameras")
        self.auto_wiris_connect_button.clicked.connect(self.actions.auto_connect_wiris)
        self.detect_pc_ip_button = QtWidgets.QPushButton("Discover PC IP")
        self.detect_pc_ip_button.clicked.connect(self.actions.discover_pc_camera_ip)
        self.apply_pc_ip_button = QtWidgets.QPushButton("Apply PC IP")
        self.apply_pc_ip_button.clicked.connect(self.actions.apply_pc_camera_ip)
        self.swap_camera_button = QtWidgets.QPushButton("Swap Camera Sides")
        self.swap_camera_button.clicked.connect(self.actions.swap_cameras)
        self.check_button = QtWidgets.QPushButton("Verify Video Stream")
        self.check_button.clicked.connect(self.actions.check_cameras)
        self.check_control_button = QtWidgets.QPushButton("Verify Camera Control")
        self.check_control_button.clicked.connect(lambda _checked=False: self.actions.check_wiris_control())
        self.enable_rtsp_button = QtWidgets.QPushButton("Enable Video Stream")
        self.enable_rtsp_button.clicked.connect(self.actions.enable_wiris_rtsp)
        self.left_stip_button = QtWidgets.QPushButton("STIP 적용")
        self.left_stip_button.clicked.connect(lambda _checked=False: self.actions.apply_wiris_stip("left"))
        self.right_stip_button = QtWidgets.QPushButton("STIP 적용")
        self.right_stip_button.clicked.connect(lambda _checked=False: self.actions.apply_wiris_stip("right"))
        self.camera_connect_button = QtWidgets.QPushButton("Connect Camera Preview")
        self.camera_connect_button.clicked.connect(self.actions.toggle_camera_connection)

        if isinstance(parent, QtWidgets.QWidget):
            for name in CONNECTION_CONTROL_FIELDS:
                getattr(self, name).setParent(parent)

    def build_layout(self, camera_form, camera_network_group, camera_layout, *, candidate_group, sequence_label):
        network_actions = QtWidgets.QHBoxLayout()
        network_actions.setSpacing(6)
        for button in (self.detect_pc_ip_button, self.apply_pc_ip_button):
            network_actions.addWidget(button, 1)
        camera_form.addLayout(network_actions, 1, 0, 1, 8)

        wiris_pair_group = QtWidgets.QGroupBox("좌/우 WIRIS 설정")
        wiris_pair_group.setObjectName("wirisPairSettingsGroup")
        wiris_pair_form = QtWidgets.QGridLayout(wiris_pair_group)
        wiris_pair_form.setHorizontalSpacing(8)
        wiris_pair_form.setVerticalSpacing(8)
        wiris_pair_form.setColumnStretch(0, 0)
        wiris_pair_form.setColumnStretch(1, 1)
        wiris_pair_form.setColumnStretch(2, 1)
        wiris_pair_form.addWidget(QtWidgets.QLabel("항목"), 0, 0)
        wiris_pair_form.addWidget(QtWidgets.QLabel("왼쪽 WIRIS"), 0, 1)
        wiris_pair_form.addWidget(QtWidgets.QLabel("오른쪽 WIRIS"), 0, 2)
        for row, label_text, left_widget, right_widget in (
            (1, "제어 IP", self.left_zoom_host_edit, self.right_zoom_host_edit),
            (2, "Telnet port", self.left_control_port_spin, self.right_control_port_spin),
            (4, "RTSP URL", self.left_url_edit, self.right_url_edit),
            (5, "RTSP IP", self.left_rtsp_ip_edit, self.right_rtsp_ip_edit),
        ):
            label = QtWidgets.QLabel(label_text)
            label.setAlignment(QtCore.Qt.AlignRight | QtCore.Qt.AlignVCenter)
            wiris_pair_form.addWidget(label, row, 0)
            wiris_pair_form.addWidget(left_widget, row, 1)
            wiris_pair_form.addWidget(right_widget, row, 2)
        left_control_actions = QtWidgets.QHBoxLayout()
        left_control_actions.setSpacing(6)
        left_control_actions.addWidget(self.detect_control_button, 1)
        left_control_actions.addWidget(self.left_control_apply_button, 1)
        right_control_actions = QtWidgets.QHBoxLayout()
        right_control_actions.setSpacing(6)
        right_control_actions.addWidget(self.right_detect_control_button, 1)
        right_control_actions.addWidget(self.right_control_apply_button, 1)
        telnet_action_label = QtWidgets.QLabel("Telnet 작업")
        telnet_action_label.setAlignment(QtCore.Qt.AlignRight | QtCore.Qt.AlignVCenter)
        wiris_pair_form.addWidget(telnet_action_label, 3, 0)
        wiris_pair_form.addLayout(left_control_actions, 3, 1)
        wiris_pair_form.addLayout(right_control_actions, 3, 2)
        left_stream_actions = QtWidgets.QHBoxLayout()
        left_stream_actions.setSpacing(6)
        left_stream_actions.addWidget(self.detect_rtsp_button, 1)
        left_stream_actions.addWidget(self.left_rtsp_apply_button, 1)
        right_stream_actions = QtWidgets.QHBoxLayout()
        right_stream_actions.setSpacing(6)
        right_stream_actions.addWidget(self.right_detect_rtsp_button, 1)
        right_stream_actions.addWidget(self.right_rtsp_apply_button, 1)
        rtsp_action_label = QtWidgets.QLabel("RTSP 작업")
        rtsp_action_label.setAlignment(QtCore.Qt.AlignRight | QtCore.Qt.AlignVCenter)
        wiris_pair_form.addWidget(rtsp_action_label, 6, 0)
        wiris_pair_form.addLayout(left_stream_actions, 6, 1)
        wiris_pair_form.addLayout(right_stream_actions, 6, 2)
        self.wiris_pair_settings_group = wiris_pair_group
        verification_group = QtWidgets.QGroupBox("검증 / 활성화")
        verification_actions = QtWidgets.QHBoxLayout(verification_group)
        verification_actions.setSpacing(6)
        for button in (
            self.check_control_button,
            self.enable_rtsp_button,
            self.check_button,
            self.camera_connect_button,
        ):
            verification_actions.addWidget(button, 1)
        auto_group = QtWidgets.QGroupBox("자동 / 보조")
        auto_actions = QtWidgets.QHBoxLayout(auto_group)
        auto_actions.setSpacing(6)
        for button in (self.auto_wiris_connect_button, self.swap_camera_button):
            auto_actions.addWidget(button, 1)
        advanced_ip_group = QtWidgets.QGroupBox("IP Address / IP Mask 변경")
        advanced_ip_form = QtWidgets.QGridLayout(advanced_ip_group)
        advanced_ip_form.setHorizontalSpacing(8)
        advanced_ip_form.setVerticalSpacing(8)
        advanced_ip_form.setColumnStretch(1, 1)
        advanced_ip_form.setColumnStretch(4, 1)
        advanced_ip_form.addWidget(QtWidgets.QLabel("IP Mask"), 0, 0)
        advanced_ip_form.addWidget(self.stip_mask_edit, 0, 1)
        self.stip_reboot_checkbox.setSizePolicy(QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Fixed)
        advanced_ip_form.addWidget(self.stip_reboot_checkbox, 0, 2, 1, 3)
        advanced_ip_form.addWidget(QtWidgets.QLabel("Left IP Address"), 1, 0)
        advanced_ip_form.addWidget(self.left_stip_ip_edit, 1, 1)
        advanced_ip_form.addWidget(self.left_stip_button, 1, 2)
        advanced_ip_form.addWidget(QtWidgets.QLabel("Right IP Address"), 1, 3)
        advanced_ip_form.addWidget(self.right_stip_ip_edit, 1, 4)
        advanced_ip_form.addWidget(self.right_stip_button, 1, 5)
        camera_layout.addWidget(camera_network_group)
        camera_layout.addWidget(sequence_label)
        camera_layout.addWidget(wiris_pair_group)
        camera_layout.addWidget(candidate_group)
        camera_layout.addWidget(verification_group)
        camera_layout.addWidget(auto_group)
        camera_layout.addWidget(advanced_ip_group)
        self.camera_connection_label = QtWidgets.QLabel(STARTUP_CAMERA_STATUS)
        self.camera_connection_label.setObjectName("statusStrip")
        camera_layout.addWidget(self.camera_connection_label)

    def build_container(self, camera_group, camera_layout_outer):
        camera_scroll = QtWidgets.QScrollArea()
        camera_scroll.setWidgetResizable(True)
        camera_scroll.setFrameShape(QtWidgets.QFrame.NoFrame)
        camera_scroll.setWidget(camera_group)
        wiris_log_group = QtWidgets.QGroupBox("WIRIS 로그")
        wiris_log_layout = QtWidgets.QVBoxLayout(wiris_log_group)
        self.wiris_log = QtWidgets.QPlainTextEdit()
        self.wiris_log.setReadOnly(True)
        self.wiris_log.setMaximumBlockCount(600)
        wiris_log_layout.addWidget(self.wiris_log)
        camera_splitter = QtWidgets.QSplitter(QtCore.Qt.Vertical)
        camera_splitter.addWidget(camera_scroll)
        camera_splitter.addWidget(wiris_log_group)
        camera_splitter.setStretchFactor(0, 3)
        camera_splitter.setStretchFactor(1, 1)
        camera_splitter.setChildrenCollapsible(False)
        camera_splitter.setSizes([560, 180])
        self.camera_connection_splitter = camera_splitter
        camera_layout_outer.addWidget(camera_splitter, 1)
