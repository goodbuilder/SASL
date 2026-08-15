from __future__ import annotations

from collections.abc import Callable, Mapping
from typing import Any

from PyQt5 import QtWidgets

from Workstation.workstation_core.settings import AppSettings

from .components import WorkspaceHeader


def _mapping(value: object) -> dict[str, Any]:
    return dict(value) if isinstance(value, Mapping) else {}


class SettingsWorkspace(QtWidgets.QWidget):
    """Single Workstation surface for settings previously exposed only by Legacy."""

    def __init__(
        self,
        backend: object,
        invoke_command: Callable[[str], bool] | None = None,
        parent: QtWidgets.QWidget | None = None,
    ) -> None:
        super().__init__(parent)
        self.backend = backend
        self.settings_widgets: dict[str, QtWidgets.QWidget] = {}
        self._snapshot = self._read_snapshot()

        layout = QtWidgets.QVBoxLayout(self)
        layout.setContentsMargins(8, 8, 8, 8)
        layout.setSpacing(6)
        layout.addWidget(
            WorkspaceHeader(
                "Settings",
            )
        )

        self.tabs = QtWidgets.QTabWidget()
        self.tabs.setDocumentMode(True)
        self._build_camera_tab()
        self._build_yolo_tab()
        self._build_pixhawk_tab()
        self._build_runtime_tab()
        self._build_gnss_tab()
        layout.addWidget(self.tabs, 1)

        actions = QtWidgets.QHBoxLayout()
        self.operation_buttons: dict[str, QtWidgets.QPushButton] = {}
        for command_id, label in (
            ("settings.apply", "Apply"),
            ("settings.save", "Save"),
        ):
            button = QtWidgets.QPushButton(label)
            button.setProperty("commandId", command_id)
            if invoke_command is None:
                button.setEnabled(False)
            else:
                button.clicked.connect(lambda _checked=False, command=command_id: invoke_command(command))
            self.operation_buttons[command_id] = button
            actions.addWidget(button)
        actions.addStretch(1)
        layout.addLayout(actions)

    def _read_snapshot(self) -> dict[str, Any]:
        reader = getattr(self.backend, "workstation_settings_snapshot", None)
        payload: object = None
        if callable(reader):
            try:
                payload = reader()
            except Exception:
                payload = None
        if not isinstance(payload, Mapping):
            defaults = AppSettings()
            payload = dict(defaults.__dict__)
            payload["gnss_operation"] = defaults.gnss_operation.to_mapping()
        else:
            payload = dict(payload)
        gnss = payload.get("gnss_operation")
        if hasattr(gnss, "to_mapping"):
            payload["gnss_operation"] = gnss.to_mapping()
        else:
            payload["gnss_operation"] = _mapping(gnss)
        return payload

    def _value(self, key: str, default: object = "") -> object:
        value = self._snapshot.get(key, default)
        return default if value is None else value

    def _gnss(self) -> dict[str, Any]:
        return _mapping(self._snapshot.get("gnss_operation"))

    def _gnss_value(self, section: str, key: str, default: object = "") -> object:
        return _mapping(self._gnss().get(section)).get(key, default)

    def _field_value(self, key: str) -> object:
        direct = self._snapshot.get(key)
        if direct is not None:
            return direct
        mapping = {
            "gnss_site_label": self._gnss().get("site_label"),
            "gnss_site_address": self._gnss().get("site_address"),
            "gnss_operation_mode": self._gnss().get("operation_mode"),
            "gnss_readiness_gate_enabled": self._gnss().get("readiness_gate_enabled"),
            "gnss_require_drone_rtk_fixed": self._gnss().get("require_drone_rtk_fixed"),
            "gnss_route_left_enabled": self._gnss_value("routing", "left_enabled"),
            "gnss_route_right_enabled": self._gnss_value("routing", "right_enabled"),
            "gnss_route_drone_enabled": self._gnss_value("routing", "drone_enabled"),
            "gnss_network_caster_host": self._gnss_value("network", "caster_host"),
            "gnss_network_caster_port": self._gnss_value("network", "caster_port"),
            "gnss_network_mountpoint": self._gnss_value("network", "mountpoint"),
            "gnss_network_username_env": self._gnss_value("network", "username_env"),
            "gnss_network_password_env": self._gnss_value("network", "password_env"),
            "gnss_network_tls": self._gnss_value("network", "tls"),
            "gnss_network_reconnect_interval_s": self._gnss_value("network", "reconnect_interval_s"),
            "gnss_network_connect_timeout_s": self._gnss_value("network", "connect_timeout_s"),
            "gnss_network_read_timeout_s": self._gnss_value("network", "read_timeout_s"),
            "gnss_network_gga_policy": self._gnss_value("network", "gga_policy"),
            "gnss_readiness_minimum_fixed_duration_s": self._gnss_value(
                "readiness", "minimum_fixed_duration_s"
            ),
            "gnss_readiness_minimum_fix_type": self._gnss_value(
                "readiness", "minimum_fix_type"
            ),
            "gnss_readiness_maximum_correction_age_s": self._gnss_value(
                "readiness", "maximum_correction_age_s"
            ),
            "gnss_readiness_maximum_gps_sample_age_s": self._gnss_value(
                "readiness", "maximum_gps_sample_age_s"
            ),
            "gnss_readiness_maximum_horizontal_accuracy_m": self._gnss_value(
                "readiness", "maximum_horizontal_accuracy_m"
            ),
            "gnss_readiness_maximum_vertical_accuracy_m": self._gnss_value(
                "readiness", "maximum_vertical_accuracy_m"
            ),
            "gnss_readiness_maximum_left_right_timestamp_delta_ms": self._gnss_value(
                "readiness", "maximum_left_right_timestamp_delta_ms"
            ),
            "gnss_readiness_maximum_baseline_std_m": self._gnss_value(
                "readiness", "maximum_baseline_std_m"
            ),
            "gnss_fixed_coordinate_profile_path": self._gnss_value("fixed_base", "coordinate_profile_path"),
            "gnss_fixed_acquisition_duration_s": self._gnss_value("fixed_base", "acquisition_duration_s"),
            "gnss_fixed_minimum_fixed_duration_s": self._gnss_value("fixed_base", "minimum_fixed_duration_s"),
            "gnss_fixed_minimum_fixed_ratio": self._gnss_value("fixed_base", "minimum_fixed_ratio"),
            "gnss_fixed_max_horizontal_scatter_m": self._gnss_value("fixed_base", "maximum_horizontal_scatter_m"),
            "gnss_fixed_max_vertical_scatter_m": self._gnss_value("fixed_base", "maximum_vertical_scatter_m"),
            "gnss_fixed_permit_profile_reuse": self._gnss_value("fixed_base", "permit_profile_reuse"),
            "gnss_fixed_height_type": self._gnss_value("fixed_base", "height_type"),
        }
        return mapping.get(key, "")

    def _text(self, key: str, *, password: bool = False, placeholder: str = "") -> QtWidgets.QLineEdit:
        widget = QtWidgets.QLineEdit(str(self._field_value(key) or ""))
        if password:
            widget.setEchoMode(QtWidgets.QLineEdit.Password)
        if placeholder:
            widget.setPlaceholderText(placeholder)
        self.settings_widgets[key] = widget
        return widget

    def _integer(self, key: str, minimum: int, maximum: int, suffix: str = "") -> QtWidgets.QSpinBox:
        widget = QtWidgets.QSpinBox()
        widget.setRange(minimum, maximum)
        try:
            widget.setValue(int(self._field_value(key)))
        except (TypeError, ValueError):
            widget.setValue(minimum)
        widget.setSuffix(suffix)
        self.settings_widgets[key] = widget
        return widget

    def _number(
        self,
        key: str,
        minimum: float,
        maximum: float,
        decimals: int = 3,
        suffix: str = "",
    ) -> QtWidgets.QDoubleSpinBox:
        widget = QtWidgets.QDoubleSpinBox()
        widget.setRange(minimum, maximum)
        widget.setDecimals(decimals)
        try:
            widget.setValue(float(self._field_value(key)))
        except (TypeError, ValueError):
            widget.setValue(minimum)
        widget.setSuffix(suffix)
        self.settings_widgets[key] = widget
        return widget

    def _toggle(self, key: str, label: str) -> QtWidgets.QCheckBox:
        widget = QtWidgets.QCheckBox(label)
        widget.setChecked(bool(self._field_value(key)))
        self.settings_widgets[key] = widget
        return widget

    def _combo(self, key: str, items: tuple[tuple[str, str], ...]) -> QtWidgets.QComboBox:
        widget = QtWidgets.QComboBox()
        for label, value in items:
            widget.addItem(label, value)
        current = str(self._field_value(key) or "")
        index = widget.findData(current)
        widget.setCurrentIndex(max(0, index))
        self.settings_widgets[key] = widget
        return widget

    @staticmethod
    def _group(title: str, rows: list[tuple[str, QtWidgets.QWidget]]) -> QtWidgets.QGroupBox:
        group = QtWidgets.QGroupBox(title)
        form = QtWidgets.QFormLayout(group)
        form.setFieldGrowthPolicy(QtWidgets.QFormLayout.AllNonFixedFieldsGrow)
        form.setHorizontalSpacing(10)
        form.setVerticalSpacing(6)
        for label, widget in rows:
            form.addRow(label, widget)
        return group

    def _tab(self, title: str) -> QtWidgets.QVBoxLayout:
        page = QtWidgets.QWidget()
        outer = QtWidgets.QVBoxLayout(page)
        outer.setContentsMargins(6, 6, 6, 6)
        scroll = QtWidgets.QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setFrameShape(QtWidgets.QFrame.NoFrame)
        content = QtWidgets.QWidget()
        content_layout = QtWidgets.QVBoxLayout(content)
        content_layout.setContentsMargins(2, 2, 2, 2)
        content_layout.setSpacing(8)
        scroll.setWidget(content)
        outer.addWidget(scroll)
        self.tabs.addTab(page, title)
        return content_layout

    def _build_camera_tab(self) -> None:
        layout = self._tab("Camera / WIRIS")
        layout.addWidget(
            self._group(
                "Camera network",
                [
                    ("Local interface", self._text("interface_name", placeholder="enx...")),
                    ("PC camera-network address", self._text("camera_host_address", placeholder="10.0.0.10/24")),
                    ("Left RTSP URL", self._text("left_rtsp_url")),
                    ("Right RTSP URL", self._text("right_rtsp_url")),
                ],
            )
        )
        layout.addWidget(
            self._group(
                "WIRIS control / STIP",
                [
                    ("Left control host", self._text("left_control_host")),
                    ("Right control host", self._text("right_control_host")),
                    ("WIRIS control port", self._integer("wiris_control_port", 1, 65535)),
                    ("Left control port", self._integer("left_control_port", 1, 65535)),
                    ("Right control port", self._integer("right_control_port", 1, 65535)),
                    ("Control timeout", self._number("wiris_control_timeout_s", 0.1, 60.0, 2, " s")),
                    ("Left STIP IP", self._text("left_stip_ip")),
                    ("Right STIP IP", self._text("right_stip_ip")),
                    ("STIP subnet mask", self._text("stip_subnet_mask")),
                    ("Reboot after STIP", self._toggle("stip_reboot", "Send REBT after STIP/STMS")),
                ],
            )
        )
        layout.addWidget(
            self._group(
                "WIRIS FTP (HR/OZ capture download)",
                [
                    ("Enable FTP download", self._toggle("wiris_ftp_enabled", "Download captured stills after HR/OZ capture")),
                    ("FTP port", self._integer("wiris_ftp_port", 1, 65535)),
                    ("FTP user", self._text("wiris_ftp_user")),
                    ("FTP password", self._text("wiris_ftp_password", password=True)),
                    ("FTP remote dir", self._text("wiris_ftp_remote_dir", placeholder="/")),
                    ("FTP timeout", self._number("wiris_ftp_timeout_s", 0.5, 60.0, 1, " s")),
                ],
            )
        )
        layout.addStretch(1)

    def _build_yolo_tab(self) -> None:
        layout = self._tab("YOLO / RTSP")
        layout.addWidget(
            self._group(
                "YOLO runtime",
                [
                    ("Weights path", self._text("yolo_weights_path")),
                    ("Inference device", self._combo("yolo_device", (("CPU", "cpu"), ("CUDA", "cuda:0")))),
                    ("Preview width", self._integer("preview_width", 320, 3840, " px")),
                    ("RTSP latency", self._integer("rtsp_latency_ms", 0, 5000, " ms")),
                    ("YOLO event queue", self._integer("yolo_event_queue_size", 1, 10000)),
                    ("Decoded frame queue", self._integer("yolo_frame_queue_size", 1, 1000)),
                ],
            )
        )
        layout.addWidget(
            self._group(
                "Stream backend",
                [
                    ("Decoder", self._combo("rtsp_decoder", (("Auto", "auto"), ("NVIDIA", "nvidia"), ("Software", "software")))),
                    (
                        "Backend",
                        self._combo(
                            "stream_backend",
                            (("GStreamer", "gstreamer"), ("OpenCV/FFmpeg", "opencv"), ("OpenCV Auto", "opencv_auto"), ("PyAV", "pyav")),
                        ),
                    ),
                    ("RTSP transport", self._combo("rtsp_transport", (("UDP", "udp"), ("TCP interleaved", "tcp")))),
                ],
            )
        )
        layout.addStretch(1)

    def _build_pixhawk_tab(self) -> None:
        layout = self._tab("PX4 / MAVLink")
        layout.addWidget(
            self._group(
                "MAVLink / host",
                [
                    ("Pixhawk host address", self._text("pixhawk_host_address")),
                ],
            )
        )
        layout.addWidget(
            self._group(
                "Pixhawk endpoints",
                [
                    ("Left Ethernet IP", self._text("left_pixhawk_eth_ip")),
                    ("Right Ethernet IP", self._text("right_pixhawk_eth_ip")),
                    ("Left MAVLink URL", self._text("left_mavlink_url")),
                    ("Right MAVLink URL", self._text("right_mavlink_url")),
                    ("Left MAVLink baud", self._integer("left_mavlink_baud", 1, 10_000_000)),
                    ("Right MAVLink baud", self._integer("right_mavlink_baud", 1, 10_000_000)),
                    ("Drone Telemetry URL", self._text("drone_telemetry_url", placeholder="udpin:0.0.0.0:14552")),
                    ("Drone Telemetry baud", self._integer("drone_telemetry_baud", 1, 10_000_000)),
                    (
                        "Drone RTCM source",
                        self._combo(
                            "drone_telemetry_rtcm_source",
                            (("Base GNSS", "base"), ("Network NTRIP", "ntrip")),
                        ),
                    ),
                    (
                        "Forward RTCM to drone",
                        self._toggle(
                            "drone_telemetry_rtcm_forward",
                            "Send the selected RTCM source to the connected drone Pixhawk",
                        ),
                    ),
                    ("Left bootstrap URL", self._text("left_pixhawk_bootstrap_url")),
                    ("Right bootstrap URL", self._text("right_pixhawk_bootstrap_url")),
                    ("Left bootstrap baud", self._integer("left_pixhawk_bootstrap_baud", 1, 10_000_000)),
                    ("Right bootstrap baud", self._integer("right_pixhawk_bootstrap_baud", 1, 10_000_000)),
                ],
            )
        )
        layout.addWidget(
            self._group(
                "PX4 transport setup",
                [
                    ("Ethernet mode", self._combo("pixhawk_ethernet_mode", (("MAVLink", "mavlink"),))),
                    ("Configure MAVLink streams", self._toggle("mavlink_configure_streams", "Apply stream parameters after MAVLink connect")),
                    ("MAVLink stream rate", self._integer("mavlink_stream_rate", 1, 10_000_000)),
                ],
            )
        )
        layout.addStretch(1)

    def _build_runtime_tab(self) -> None:
        layout = self._tab("Runtime / Calibration")
        layout.addWidget(
            self._group(
                "Intrinsics / measurement mode",
                [
                    ("Intrinsics path", self._text("intrinsics_path")),
                    ("Auto-load intrinsics", self._toggle("auto_load_intrinsics", "Load the active zoom profile at startup")),
                    ("Manual focal override", self._toggle("manual_focal_override", "Use the manual focal fallback")),
                    ("Triangulation enabled", self._toggle("use_triangulation", "Run 3D triangulation")),
                    ("Display mode", self._combo("measurement_display_mode", (("Detection", "detection"), ("Preview", "preview")))),
                    ("Use RTK baseline", self._toggle("use_rtk_baseline", "Use GPS/RTK baseline when selected")),
                    ("Require RTK fixed", self._toggle("require_rtk_fixed", "Block measurement until RTK Fixed")),
                ],
            )
        )
        layout.addStretch(1)

    def _refresh_gnss_mode_controls(self) -> None:
        operation_mode = self.settings_widgets.get("gnss_operation_mode")
        if not isinstance(operation_mode, QtWidgets.QComboBox):
            return

        mode = str(operation_mode.currentData() or "LOCAL_SURVEY_IN")
        local_survey_in = mode == "LOCAL_SURVEY_IN"
        network_assisted_fixed_base = mode == "NETWORK_ASSISTED_FIXED_BASE"
        network_rtk_direct = mode == "NETWORK_RTK_DIRECT"

        for key in ("gnss_base_serial_port", "gnss_base_baud"):
            widget = self.settings_widgets.get(key)
            if widget is not None:
                widget.setEnabled(local_survey_in or network_assisted_fixed_base)

        for key in ("gnss_base_survey_in_min_duration_s", "gnss_base_survey_in_accuracy_m"):
            widget = self.settings_widgets.get(key)
            if widget is not None:
                widget.setEnabled(local_survey_in)

        for key in (
            "gnss_network_caster_host",
            "gnss_network_caster_port",
            "gnss_network_mountpoint",
            "gnss_network_username_env",
            "gnss_network_password_env",
            "gnss_network_tls",
            "gnss_network_reconnect_interval_s",
            "gnss_network_connect_timeout_s",
            "gnss_network_read_timeout_s",
            "gnss_network_gga_policy",
        ):
            widget = self.settings_widgets.get(key)
            if widget is not None:
                widget.setEnabled(not local_survey_in)

        for key in (
            "gnss_fixed_coordinate_profile_path",
            "gnss_fixed_acquisition_duration_s",
            "gnss_fixed_minimum_fixed_duration_s",
            "gnss_fixed_minimum_fixed_ratio",
            "gnss_fixed_max_horizontal_scatter_m",
            "gnss_fixed_max_vertical_scatter_m",
            "gnss_fixed_permit_profile_reuse",
            "gnss_fixed_height_type",
        ):
            widget = self.settings_widgets.get(key)
            if widget is not None:
                widget.setEnabled(network_assisted_fixed_base)

        # Keep this explicit so a future mode addition cannot accidentally leave
        # the base-only controls active in Network RTK direct mode.
        if network_rtk_direct:
            for key in (
                "gnss_base_serial_port",
                "gnss_base_baud",
                "gnss_base_survey_in_min_duration_s",
                "gnss_base_survey_in_accuracy_m",
                "gnss_fixed_coordinate_profile_path",
                "gnss_fixed_acquisition_duration_s",
                "gnss_fixed_minimum_fixed_duration_s",
                "gnss_fixed_minimum_fixed_ratio",
                "gnss_fixed_max_horizontal_scatter_m",
                "gnss_fixed_max_vertical_scatter_m",
                "gnss_fixed_permit_profile_reuse",
                "gnss_fixed_height_type",
            ):
                widget = self.settings_widgets.get(key)
                if widget is not None:
                    widget.setEnabled(False)

    def _build_gnss_tab(self) -> None:
        layout = self._tab("GNSS / RTK")
        site_label = self._text("gnss_site_label", placeholder="Per-experiment site name")
        site_label.setToolTip(
            "실험 장소 또는 측점을 사람이 식별하기 위한 이름입니다. "
            "GNSS 보정과 거리 계산에는 사용되지 않고 세션 메타데이터로 저장됩니다."
        )
        site_address = self._text(
            "gnss_site_address",
            placeholder="Per-experiment address or field note",
        )
        site_address.setToolTip(
            "현장 주소, 설치 위치, 조사 조건 등 기록용 주소/메모입니다. "
            "GNSS 보정과 측정 계산에는 영향을 주지 않습니다."
        )
        operation_mode = self._combo(
            "gnss_operation_mode",
            (
                ("Local Survey-in", "LOCAL_SURVEY_IN"),
                ("Network assisted fixed base", "NETWORK_ASSISTED_FIXED_BASE"),
                ("Network RTK direct", "NETWORK_RTK_DIRECT"),
            ),
        )
        operation_mode.setToolTip(
            "Local Survey-in은 Base가 직접 보정 기준을 만들고, Network assisted fixed base는 "
            "Base가 NTRIP를 수신하며, Network RTK direct는 Base 없이 Network NTRIP를 사용합니다."
        )
        operation_mode.currentIndexChanged.connect(
            lambda _index: self._refresh_gnss_mode_controls()
        )
        layout.addWidget(
            self._group(
                "Base / operation",
                [
                    ("Experiment site / point", site_label),
                    ("Experiment address / note", site_address),
                    ("Operation mode", operation_mode),
                    (
                        "Base serial port",
                        self._text(
                            "gnss_base_serial_port",
                            placeholder="Required for Local Survey-in / assisted fixed base",
                        ),
                    ),
                    ("Base baud", self._integer("gnss_base_baud", 1, 10_000_000)),
                    ("Survey-in minimum duration", self._integer("gnss_base_survey_in_min_duration_s", 1, 86_400, " s")),
                    ("Survey-in accuracy", self._number("gnss_base_survey_in_accuracy_m", 0.001, 1000.0, 3, " m")),
                    ("GNSS readiness gate", self._toggle("gnss_readiness_gate_enabled", "Require GNSS readiness before operation")),
                ],
            )
        )
        layout.addWidget(
            self._group(
                "RTCM routing / Drone readiness",
                [
                    ("P1 Pixhawk RTCM route", self._toggle("gnss_route_left_enabled", "Send correction frames to P1")),
                    ("P2 Pixhawk RTCM route", self._toggle("gnss_route_right_enabled", "Send correction frames to P2")),
                    ("Drone RTCM route", self._toggle("gnss_route_drone_enabled", "Enable the Drone correction route and readiness target")),
                    ("Drone RTK Fixed requirement", self._toggle("gnss_require_drone_rtk_fixed", "Require Drone RTK Fixed for GNSS readiness")),
                ],
            )
        )
        layout.addWidget(
            self._group(
                "NTRIP caster (any provider)",
                [
                    (
                        "Caster host",
                        self._text("gnss_network_caster_host", placeholder="caster.example.com (hostname only)"),
                    ),
                    ("Caster port", self._integer("gnss_network_caster_port", 1, 65535)),
                    ("Mountpoint", self._text("gnss_network_mountpoint", placeholder="MOUNTPOINT")),
                    (
                        "Username environment variable",
                        self._text("gnss_network_username_env", placeholder="NTRIP_USERNAME"),
                    ),
                    (
                        "Password environment variable",
                        self._text("gnss_network_password_env", placeholder="NTRIP_PASSWORD"),
                    ),
                    ("TLS", self._toggle("gnss_network_tls", "Use TLS for NTRIP")),
                    ("Reconnect interval", self._number("gnss_network_reconnect_interval_s", 0.0, 3600.0, 2, " s")),
                    ("Connect timeout", self._number("gnss_network_connect_timeout_s", 0.1, 3600.0, 2, " s")),
                    ("Read timeout", self._number("gnss_network_read_timeout_s", 0.1, 3600.0, 2, " s")),
                    ("GGA policy", self._combo("gnss_network_gga_policy", (("Base GNSS (assisted fixed base)", "BASE_GNSS"), ("System center", "SYSTEM_CENTER"), ("Left GNSS", "LEFT_GNSS"), ("Manual / disabled", "MANUAL"), ("Per rover session", "PER_ROVER_SESSION")))),
                ],
            )
        )
        layout.addWidget(
            self._group(
                "Readiness limits (WGS84 / quality inputs)",
                [
                    ("Minimum numeric fix_type", self._integer("gnss_readiness_minimum_fix_type", 0, 255)),
                    ("Minimum fixed duration", self._number("gnss_readiness_minimum_fixed_duration_s", 0.0, 86_400.0, 2, " s")),
                    ("Maximum correction age", self._number("gnss_readiness_maximum_correction_age_s", 0.0, 3600.0, 2, " s")),
                    ("Maximum GPS sample age", self._number("gnss_readiness_maximum_gps_sample_age_s", 0.0, 3600.0, 2, " s")),
                    ("Maximum horizontal accuracy", self._number("gnss_readiness_maximum_horizontal_accuracy_m", 0.0, 1000.0, 4, " m")),
                    ("Maximum vertical accuracy", self._number("gnss_readiness_maximum_vertical_accuracy_m", 0.0, 1000.0, 4, " m")),
                    ("Maximum L/R timestamp delta", self._number("gnss_readiness_maximum_left_right_timestamp_delta_ms", 0.0, 10_000.0, 2, " ms")),
                    ("Maximum baseline std", self._number("gnss_readiness_maximum_baseline_std_m", 0.0, 1000.0, 4, " m")),
                ],
            )
        )
        layout.addWidget(
            self._group(
                "Fixed base profile",
                [
                    ("Coordinate profile path", self._text("gnss_fixed_coordinate_profile_path")),
                    ("Acquisition duration", self._number("gnss_fixed_acquisition_duration_s", 0.1, 86_400.0, 2, " s")),
                    ("Minimum fixed duration", self._number("gnss_fixed_minimum_fixed_duration_s", 0.1, 86_400.0, 2, " s")),
                    ("Minimum fixed ratio", self._number("gnss_fixed_minimum_fixed_ratio", 0.001, 1.0, 4)),
                    ("Maximum horizontal scatter", self._number("gnss_fixed_max_horizontal_scatter_m", 0.0001, 1000.0, 4, " m")),
                    ("Maximum vertical scatter", self._number("gnss_fixed_max_vertical_scatter_m", 0.0001, 1000.0, 4, " m")),
                    ("Permit profile reuse", self._toggle("gnss_fixed_permit_profile_reuse", "Reuse an existing fixed-base profile")),
                    ("Height type", self._combo("gnss_fixed_height_type", (("Ellipsoid", "ELLIPSOID"), ("MSL", "MSL"), ("Unknown", "UNKNOWN")))),
                ],
            )
        )
        layout.addStretch(1)
        self._refresh_gnss_mode_controls()

    def _widget_value(self, widget: QtWidgets.QWidget) -> object:
        if isinstance(widget, QtWidgets.QComboBox):
            return widget.currentData()
        if isinstance(widget, QtWidgets.QAbstractButton):
            return widget.isChecked()
        if isinstance(widget, QtWidgets.QLineEdit):
            return widget.text().strip()
        if isinstance(widget, (QtWidgets.QSpinBox, QtWidgets.QDoubleSpinBox)):
            return widget.value()
        return None

    def settings_payload(self) -> dict[str, object]:
        payload = {key: self._widget_value(widget) for key, widget in self.settings_widgets.items()}
        gnss = _mapping(self._snapshot.get("gnss_operation"))
        readiness = _mapping(gnss.get("readiness"))
        local_base = _mapping(gnss.get("local_base"))
        network = _mapping(gnss.get("network"))
        fixed_base = _mapping(gnss.get("fixed_base"))
        routing = _mapping(gnss.get("routing"))
        gnss["operation_mode"] = payload.get("gnss_operation_mode")
        gnss["site_label"] = payload.get("gnss_site_label", "")
        gnss["site_address"] = payload.get("gnss_site_address", "")
        gnss["require_drone_rtk_fixed"] = bool(payload.get("gnss_require_drone_rtk_fixed"))
        gnss["readiness_gate_enabled"] = bool(payload.get("gnss_readiness_gate_enabled"))
        local_base.update(
            {
                "device": payload.get("gnss_base_serial_port", ""),
                "baud": payload.get("gnss_base_baud", 115200),
                "survey_in_min_duration_s": payload.get("gnss_base_survey_in_min_duration_s", 300),
                "survey_in_accuracy_limit_m": payload.get("gnss_base_survey_in_accuracy_m", 0.5),
            }
        )
        network.update(
            {
                "caster_host": payload.get("gnss_network_caster_host", ""),
                "caster_port": payload.get("gnss_network_caster_port", 2101),
                "mountpoint": payload.get("gnss_network_mountpoint", ""),
                "username_env": payload.get("gnss_network_username_env", "NTRIP_USERNAME"),
                "password_env": payload.get("gnss_network_password_env", "NTRIP_PASSWORD"),
                "tls": bool(payload.get("gnss_network_tls")),
                "reconnect_interval_s": payload.get("gnss_network_reconnect_interval_s", 5.0),
                "connect_timeout_s": payload.get("gnss_network_connect_timeout_s", 10.0),
                "read_timeout_s": payload.get("gnss_network_read_timeout_s", 10.0),
                "gga_policy": payload.get("gnss_network_gga_policy", "SYSTEM_CENTER"),
            }
        )
        readiness.update(
            {
                "minimum_fix_type": payload.get("gnss_readiness_minimum_fix_type", 3),
                "minimum_fixed_duration_s": payload.get("gnss_readiness_minimum_fixed_duration_s", 30.0),
                "maximum_correction_age_s": payload.get("gnss_readiness_maximum_correction_age_s", 3.0),
                "maximum_gps_sample_age_s": payload.get("gnss_readiness_maximum_gps_sample_age_s", 1.0),
                "maximum_horizontal_accuracy_m": payload.get("gnss_readiness_maximum_horizontal_accuracy_m", 0.03),
                "maximum_vertical_accuracy_m": payload.get("gnss_readiness_maximum_vertical_accuracy_m", 0.05),
                "maximum_left_right_timestamp_delta_ms": payload.get("gnss_readiness_maximum_left_right_timestamp_delta_ms", 50.0),
                "maximum_baseline_std_m": payload.get("gnss_readiness_maximum_baseline_std_m", 0.02),
            }
        )
        fixed_base.update(
            {
                "coordinate_profile_path": payload.get("gnss_fixed_coordinate_profile_path", ""),
                "acquisition_duration_s": payload.get("gnss_fixed_acquisition_duration_s", 180.0),
                "minimum_fixed_duration_s": payload.get("gnss_fixed_minimum_fixed_duration_s", 60.0),
                "minimum_fixed_ratio": payload.get("gnss_fixed_minimum_fixed_ratio", 0.95),
                "maximum_horizontal_scatter_m": payload.get("gnss_fixed_max_horizontal_scatter_m", 0.03),
                "maximum_vertical_scatter_m": payload.get("gnss_fixed_max_vertical_scatter_m", 0.05),
                "permit_profile_reuse": bool(payload.get("gnss_fixed_permit_profile_reuse")),
                "height_type": payload.get("gnss_fixed_height_type", "UNKNOWN"),
            }
        )
        left_enabled = bool(payload.get("gnss_route_left_enabled"))
        right_enabled = bool(payload.get("gnss_route_right_enabled"))
        drone_enabled = bool(payload.get("gnss_route_drone_enabled"))
        if left_enabled and right_enabled:
            target = "both"
        elif left_enabled:
            target = "left"
        elif right_enabled:
            target = "right"
        else:
            target = "disabled"
        routing.update(
            {
                "left_enabled": left_enabled,
                "right_enabled": right_enabled,
                "drone_enabled": drone_enabled,
                # NTRIP-to-Base routing is inherent in the assisted fixed-base
                # operation; it is not an independent operator choice.
                "field_base_enabled": payload.get("gnss_operation_mode")
                == "NETWORK_ASSISTED_FIXED_BASE",
            }
        )
        gnss.update({"readiness": readiness, "local_base": local_base, "network": network, "fixed_base": fixed_base, "routing": routing})
        # Keep the legacy flat target available to the owner bridge and older
        # settings files while the visible UI uses explicit P1/P2 route flags.
        payload["gnss_base_rtcm_injection_target"] = target
        payload["gnss_operation"] = gnss
        return payload

    def _set_widget_value(self, key: str, value: object) -> None:
        widget = self.settings_widgets.get(key)
        if widget is None or value is None:
            return
        if isinstance(widget, QtWidgets.QComboBox):
            index = widget.findData(value)
            if index >= 0:
                widget.setCurrentIndex(index)
        elif isinstance(widget, QtWidgets.QAbstractButton):
            widget.setChecked(bool(value))
        elif isinstance(widget, QtWidgets.QLineEdit):
            widget.setText(str(value))
        elif isinstance(widget, QtWidgets.QDoubleSpinBox):
            try:
                widget.setValue(float(value))
            except (TypeError, ValueError):
                return
        elif isinstance(widget, QtWidgets.QSpinBox):
            try:
                widget.setValue(int(value))
            except (TypeError, ValueError):
                return

    def refresh_from_backend(self) -> None:
        self._snapshot = self._read_snapshot()
        for key in self.settings_widgets:
            self._set_widget_value(key, self._field_value(key))
