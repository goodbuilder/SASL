"""Alignment feature: owned state/views and explicit dependency ports.

Coordinate values retain their producer contracts (NED, body FRD, optical RDF,
and baseline-center FRD). This ownership change performs no axis conversion.
"""
from __future__ import annotations

from Workstation.panel.panel_feature_runtime import PanelFeature
import time
from PyQt5 import (
    QtCore,
    QtWidgets,
)
from Workstation.panel.panel_support import angle_diff_deg
from Workstation.ui.telemetry_plots import AttitudeHistoryPlot, RtkAlignmentPlot, TelemetryHistoryPlot, TelemetryHistoryPoint
from Workstation.workstation_core.alignment_managers import (
    AlignmentProfileManager,
    AlignmentValidator,
    CoordinateSystemManager,
)
from Workstation.workstation_core.attitude_geometry import (
    relative_attitude,
    world_ned_vector_to_body_frd,
)
from Workstation.workstation_core.baseline_alignment import (
    BaselineQuality,
    BaselineVector,
    baseline_from_gps,
    evaluate_baseline_quality,
)
from Workstation.workstation_core.physical_alignment import (
    PhysicalAlignmentQuality,
    PhysicalAlignmentSample,
    evaluate_physical_alignment,
    make_physical_alignment_sample,
)
from Workstation.workstation_core.pixhawk_attitude import (
    discover_mavlink_messages,
    magnetometer_value_units,
    mavlink_graphable_fields_for_message,
)
from Workstation.workstation_core.px4_mag_manager import (
    BOARD_INTERNAL_MAG_ID,
    GPS1_MAG_ID,
    GPS2_MAG_ID,
    RM3100_BACKUP_ID,
    RM3100_IDS,
    RM3100_PRIMARY_ID,
    current_pixhawk_6x_mag_params,
    identify_mag_device,
)
from Workstation.workstation_core.rtk_geometry import (
    BodyFrdBaselineProjection,
    GpsPositionState,
    project_ned_baseline_to_p1_body_frd,
)
from collections import (
    deque,
)
from pathlib import (
    Path,
)


class AlignmentFeature(PanelFeature):
    feature_name = 'alignment'
    dependency_names = (
        '_alignment_view_camera_translation_right_in_left_m',
        '_baseline_camera_translation_in_side_rdf_m',
        '_baseline_forward_angle_level',
        '_baseline_forward_geometry_from_camera_translation',
        '_baseline_forward_geometry_text',
        '_build_gnss_operation_config',
        '_build_px4_attitude_cal_panel',
        '_build_px4_mag_manager_panel',
        '_build_single_pixhawk_sensor_tab',
        '_gps_baseline_usable_for_translation',
        '_gps_fix_level',
        '_gps_fix_ready_for_baseline',
        '_pixhawk_gnss_source',
        '_refresh_optical_alignment_view',
        '_refresh_rtk_alignment_view',
        '_refresh_rtk_baseline',
        '_save_current_settings',
        '_set_baseline_forward_metric_labels',
        '_set_health_label',
        '_set_label_text_if_changed',
        'app_root',
        'attitude_compare_labels',
        'attitude_monitor',
        'attitude_plots',
        'baseline_attitude_labels',
        'baseline_calc_labels',
        'baseline_gps_card_labels',
        'baseline_gps_rate_history',
        'baseline_left_gps_combo',
        'baseline_length_plot',
        'baseline_mavlink_message_table',
        'baseline_right_gps_combo',
        'baseline_sampling_active',
        'baseline_sampling_started_s',
        'baseline_scan_topics_button',
        'baseline_source_summary_labels',
        'baseline_start_sampling_button',
        'baseline_stop_sampling_button',
        'baseline_tabs',
        'f9p_compare_labels',
        'f9p_quality_plots',
        'f9p_sats_plots',
        'f9p_slot_labels',
        'gnss_heading_labels',
        'gps_sensor_labels',
        'latest_baseline_vector',
        'latest_can_mag_sensors',
        'latest_extrinsics',
        'latest_gps',
        'latest_gps_sensors',
        'latest_mag_sensors',
        'latest_p1_body_frd_baseline',
        'latest_rtk_baseline',
        'latest_vehicle_mag_device_ids',
        'magnetometer_monitor',
        'optical_expected_dy_combo',
        'optical_expected_dy_spin',
        'optical_target_distance_spin',
        'pixhawk_sensor_summary_labels',
        'px4_mag_managers',
        'rm3100_compare_labels',
        'rm3100_delta_plots',
        'rm3100_norm_plots',
        'rm3100_slot_labels',
        'set_pixhawk_gnss_source',
        'signal_attitude_age_plot',
        'signal_attitude_labels',
        'signal_gps_eph_plot',
        'signal_gps_position_labels',
        'signal_gps_sats_plot',
        'signal_gps_sensor_labels',
        'workstation_time_sync_checkpoints',
    )
    writable_dependencies = frozenset(('attitude_compare_labels', 'attitude_plots', 'baseline_attitude_labels', 'baseline_calc_labels', 'baseline_gps_card_labels', 'baseline_gps_rate_history', 'baseline_left_gps_combo', 'baseline_length_plot', 'baseline_mavlink_message_table', 'baseline_right_gps_combo', 'baseline_sampling_active', 'baseline_sampling_started_s', 'baseline_scan_topics_button', 'baseline_source_summary_labels', 'baseline_start_sampling_button', 'baseline_stop_sampling_button', 'baseline_tabs', 'f9p_compare_labels', 'f9p_quality_plots', 'f9p_sats_plots', 'f9p_slot_labels', 'gnss_heading_labels', 'latest_baseline_vector', 'latest_p1_body_frd_baseline', 'optical_expected_dy_combo', 'optical_expected_dy_spin', 'optical_target_distance_spin', 'pixhawk_sensor_summary_labels', 'rm3100_compare_labels', 'rm3100_delta_plots', 'rm3100_norm_plots', 'rm3100_slot_labels', 'signal_attitude_age_plot', 'signal_attitude_labels', 'signal_gps_eph_plot', 'signal_gps_position_labels', 'signal_gps_sats_plot', 'signal_gps_sensor_labels'))

    def initialize_state(self, *, auto_start=False, auto_start_pixhawk=False, app_root=None, settings_path=None):
        self.coordinate_system_manager = CoordinateSystemManager()
        self.alignment_validator = AlignmentValidator()
        self.alignment_profile_manager = AlignmentProfileManager()
        self.loaded_alignment_profile: dict[str, object] | None = None

    def _build_alignment_workflow_header(self, layout: QtWidgets.QVBoxLayout) -> None:
        group = QtWidgets.QGroupBox("정렬 작업 순서")
        grid = QtWidgets.QGridLayout(group)
        grid.setHorizontalSpacing(8)
        grid.setVerticalSpacing(6)
        self.alignment_step_buttons: dict[str, QtWidgets.QPushButton] = {}
        self.alignment_step_status_labels: dict[str, QtWidgets.QLabel] = {}
        steps = (
            ("sensor", "1 센서", "GNSS/MAG 인식"),
            ("source", "2 소스", "선택/품질 확인"),
            ("baseline", "3 Baseline", "GNSS 벡터 계산"),
            ("align", "4 정렬", "Forward 90deg 확인"),
        )
        for column, (key, title, initial_status) in enumerate(steps):
            button = QtWidgets.QPushButton(title)
            button.setCheckable(True)
            button.setProperty("role", "toggle")
            button.clicked.connect(lambda _checked=False, step=key: self._set_alignment_step(step))
            status_label = QtWidgets.QLabel(initial_status)
            status_label.setObjectName("statChip")
            status_label.setWordWrap(True)
            grid.addWidget(button, 0, column)
            grid.addWidget(status_label, 1, column)
            grid.setColumnStretch(column, 1)
            self.alignment_step_buttons[key] = button
            self.alignment_step_status_labels[key] = status_label
        layout.addWidget(group)
        self._refresh_alignment_workflow_status()

    def _alignment_step_index(self, step: str) -> int | None:
        attr_by_step = {
            "sensor": "signal_alignment_tab_index",
            "source": "source_quality_alignment_tab_index",
            "baseline": "baseline_alignment_tab_index",
            "align": "baseline_attitude_alignment_tab_index",
        }
        attr = attr_by_step.get(step)
        if attr is None or not hasattr(self, attr):
            return None
        return int(getattr(self, attr))

    def _set_alignment_step(self, step: str) -> None:
        index = self._alignment_step_index(step)
        if index is not None and hasattr(self, "alignment_tabs"):
            self.alignment_tabs.setCurrentIndex(index)

    def _handle_alignment_step_changed(self, index: int) -> None:
        for step, button in getattr(self, "alignment_step_buttons", {}).items():
            step_index = self._alignment_step_index(step)
            button.setChecked(step_index == index)
        self._refresh_alignment_workflow_status()

    def _set_alignment_step_status(self, step: str, text: str, level: str) -> None:
        label = getattr(self, "alignment_step_status_labels", {}).get(step)
        if label is not None:
            self._set_health_label(label, text, level)

    def _refresh_alignment_workflow_status(self) -> None:
        if not hasattr(self, "alignment_step_status_labels"):
            return

        selected_left_gps, selected_right_gps = self._selected_baseline_gps_states()
        gps_ready = [
            self._gps_fix_ready_for_baseline(selected_left_gps),
            self._gps_fix_ready_for_baseline(selected_right_gps),
        ]
        gps_count = sum(1 for ready in gps_ready if ready)
        mag_count = sum(
            1
            for side in ("left", "right")
            for slot in ("CAN1", "CAN2")
            if self._mag_slot_state(side, slot) is not None
        )
        if gps_count == 2:
            sensor_text = f"GPS OK / MAG {mag_count}/4"
            sensor_level = "good" if mag_count >= 4 else "warn"
        elif gps_count or mag_count:
            sensor_text = f"GPS {gps_count}/2 / MAG {mag_count}/4"
            sensor_level = "warn"
        else:
            sensor_text = "센서 대기"
            sensor_level = "idle"
        self._set_alignment_step_status("sensor", sensor_text, sensor_level)

        selected_mag_count = sum(1 for value in self.latest_vehicle_mag_device_ids.values() if value is not None)
        source_level = "good" if gps_count == 2 and selected_mag_count >= 2 else "warn" if gps_count or selected_mag_count else "idle"
        self._set_alignment_step_status("source", f"GNSS {gps_count}/2 · MAG selected {selected_mag_count}/2", source_level)

        baseline = self.latest_rtk_baseline
        if baseline is None:
            self._set_alignment_step_status("baseline", "baseline 대기", "idle")
        else:
            usable, reason = self._gps_baseline_usable_for_translation(baseline)
            self._set_alignment_step_status(
                "baseline",
                f"{baseline.baseline_m:.3f} m" if usable else reason,
                "good" if usable else "bad",
            )

        geometry = self._baseline_forward_geometry_from_camera_translation(
            None if baseline is None else self._alignment_view_camera_translation_right_in_left_m(baseline)
        )
        if geometry is None:
            self._set_alignment_step_status("align", "자세/baseline 대기", "idle")
        else:
            angle_deg, _effective_m, _length_m, _forward_m = geometry
            self._set_alignment_step_status("align", f"{angle_deg:.1f} deg", self._baseline_forward_angle_level(angle_deg))

    def _build_pixhawk_sensor_health_tabs(
        self,
        summary_layout: QtWidgets.QVBoxLayout,
        pixhawk_1_layout: QtWidgets.QVBoxLayout,
        pixhawk_2_layout: QtWidgets.QVBoxLayout,
        yaw_layout: QtWidgets.QVBoxLayout,
    ) -> None:
        summary_group = QtWidgets.QGroupBox("픽스호크 센서 상태")
        summary_form = QtWidgets.QGridLayout(summary_group)
        self.pixhawk_sensor_summary_labels: dict[str, dict[str, QtWidgets.QLabel]] = {"left": {}, "right": {}}
        summary_form.addWidget(QtWidgets.QLabel("픽스호크"), 0, 0)
        summary_form.addWidget(QtWidgets.QLabel("RM3100 CAN"), 0, 1)
        summary_form.addWidget(QtWidgets.QLabel("F9P GNSS"), 0, 2)
        summary_form.addWidget(QtWidgets.QLabel("GNSS Heading"), 0, 3)
        for row, (camera, title) in enumerate((("left", "Pixhawk 1"), ("right", "Pixhawk 2")), start=1):
            summary_form.addWidget(QtWidgets.QLabel(title), row, 0)
            for column, key in enumerate(("rm3100", "f9p", "heading"), start=1):
                label = QtWidgets.QLabel("대기 중")
                label.setObjectName("statusStrip")
                summary_form.addWidget(label, row, column)
                self.pixhawk_sensor_summary_labels[camera][key] = label
        summary_layout.addWidget(summary_group)
        summary_layout.addStretch(1)

        self.rm3100_slot_labels: dict[str, dict[str, QtWidgets.QLabel]] = {"left": {}, "right": {}}
        self.f9p_slot_labels: dict[str, dict[str, QtWidgets.QLabel]] = {"left": {}, "right": {}}
        self.rm3100_compare_labels: dict[str, QtWidgets.QLabel] = {}
        self.f9p_compare_labels: dict[str, QtWidgets.QLabel] = {}
        self.gnss_heading_labels: dict[str, QtWidgets.QLabel] = {}
        self.rm3100_norm_plots: dict[str, TelemetryHistoryPlot] = {}
        self.rm3100_delta_plots: dict[str, TelemetryHistoryPlot] = {}
        self.f9p_sats_plots: dict[str, TelemetryHistoryPlot] = {}
        self.f9p_quality_plots: dict[str, TelemetryHistoryPlot] = {}

        self._build_single_pixhawk_sensor_tab(pixhawk_1_layout, "left", "Pixhawk 1")
        self._build_single_pixhawk_sensor_tab(pixhawk_2_layout, "right", "Pixhawk 2")
        self._build_px4_mag_manager_panel(yaw_layout, panel_key="sensor_tab")
        yaw_layout.addStretch(1)

    def _build_alignment_signal_tab(self, layout: QtWidgets.QVBoxLayout) -> None:
        layout.setSpacing(8)
        status_group = QtWidgets.QGroupBox("GPS / Baseline 입력 신호")
        status_form = QtWidgets.QGridLayout(status_group)
        status_form.setHorizontalSpacing(8)
        status_form.setVerticalSpacing(6)
        status_form.setColumnStretch(1, 1)
        self.signal_attitude_labels: dict[str, QtWidgets.QLabel] = {}
        self.signal_gps_position_labels: dict[str, QtWidgets.QLabel] = {}
        self.signal_gps_sensor_labels: dict[str, QtWidgets.QLabel] = {}
        for row, (camera, title) in enumerate((("left", "P1"), ("right", "P2"))):
            status_form.addWidget(QtWidgets.QLabel(title), row, 0)
            attitude_label = QtWidgets.QLabel("Attitude: 대기 중")
            position_label = QtWidgets.QLabel("GPS position: 대기 중")
            sensor_label = QtWidgets.QLabel("GPS sensor: 대기 중")
            for label in (attitude_label, position_label, sensor_label):
                label.setObjectName("statusStrip")
                label.setWordWrap(True)
                label.setTextInteractionFlags(QtCore.Qt.TextSelectableByMouse)
            status_form.addWidget(attitude_label, row, 1)
            status_form.addWidget(position_label, row, 2)
            status_form.addWidget(sensor_label, row, 3)
            self.signal_attitude_labels[camera] = attitude_label
            self.signal_gps_position_labels[camera] = position_label
            self.signal_gps_sensor_labels[camera] = sensor_label
        layout.addWidget(status_group)

        self.signal_gps_sats_plot = TelemetryHistoryPlot("GPS 위성 수 수신 확인 (MAVLink)", "sats")
        self.signal_gps_sats_plot.set_series_labels(
            {
                "left_GPS1": "P1 GPS1",
                "left_GPS2": "P1 GPS2",
                "right_GPS1": "P2 GPS1",
                "right_GPS2": "P2 GPS2",
            }
        )
        self.signal_gps_eph_plot = TelemetryHistoryPlot("GPS eph 수신 확인 (MAVLink)", "m")
        self.signal_gps_eph_plot.set_series_labels(
            {
                "left_GPS1": "P1 GPS1",
                "left_GPS2": "P1 GPS2",
                "right_GPS1": "P2 GPS1",
                "right_GPS2": "P2 GPS2",
            }
        )
        self.signal_attitude_age_plot = TelemetryHistoryPlot("Pixhawk 자세 age", "ms")
        self.signal_attitude_age_plot.set_series_labels({"left": "P1 attitude", "right": "P2 attitude"})
        graph_tabs = QtWidgets.QTabWidget()
        graph_tabs.setDocumentMode(True)
        gps_tab = QtWidgets.QWidget()
        gps_layout = QtWidgets.QHBoxLayout(gps_tab)
        gps_layout.setContentsMargins(0, 0, 0, 0)
        gps_layout.setSpacing(8)
        gps_layout.addWidget(self.signal_gps_sats_plot, 1)
        gps_layout.addWidget(self.signal_gps_eph_plot, 1)
        attitude_tab = QtWidgets.QWidget()
        attitude_layout = QtWidgets.QVBoxLayout(attitude_tab)
        attitude_layout.setContentsMargins(0, 0, 0, 0)
        attitude_layout.addWidget(self.signal_attitude_age_plot, 1)
        graph_tabs.addTab(gps_tab, "GPS 그래프")
        graph_tabs.addTab(attitude_tab, "자세 그래프")
        layout.addWidget(graph_tabs, 1)

    def _init_baseline_alignment_state(self) -> None:
        self.baseline_sampling_active = False
        self.baseline_sampling_started_s: float | None = None
        self.baseline_samples: list[tuple[BaselineVector, GpsPositionState, GpsPositionState]] = []
        self.latest_baseline_quality: BaselineQuality | None = None
        self.physical_alignment_samples: list[PhysicalAlignmentSample] = []
        self.latest_physical_alignment_quality: PhysicalAlignmentQuality | None = None
        self.alignment_iteration_history: list[dict[str, object]] = []
        self.alignment_iteration_number = 0
        self.latest_baseline_vector: BaselineVector | None = None
        self.baseline_gps_rate_history: dict[str, deque[float]] = {"left": deque(maxlen=120), "right": deque(maxlen=120)}
        self.baseline_source_summary_labels: list[QtWidgets.QLabel] = []

    def _build_alignment_sensor_recognition_tab(self, layout: QtWidgets.QVBoxLayout) -> None:
        layout.setSpacing(8)
        self._build_baseline_gps_recognition_view(layout)

    def _build_alignment_source_quality_tab(self, layout: QtWidgets.QVBoxLayout) -> None:
        layout.setSpacing(8)
        scroll_area = QtWidgets.QScrollArea()
        scroll_area.setWidgetResizable(True)
        scroll_area.setFrameShape(QtWidgets.QFrame.NoFrame)
        content = QtWidgets.QWidget()
        content_layout = QtWidgets.QVBoxLayout(content)
        content_layout.setContentsMargins(0, 0, 0, 0)
        content_layout.setSpacing(8)
        scroll_area.setWidget(content)
        layout.addWidget(scroll_area, 1)
        self._build_baseline_source_selection_view(content_layout)
        self._build_baseline_quality_view(content_layout)

    def _build_baseline_alignment_tab(self, layout: QtWidgets.QVBoxLayout) -> None:
        layout.setSpacing(8)
        self._init_baseline_alignment_state()
        self.baseline_tabs = QtWidgets.QTabWidget()
        self.baseline_tabs.setDocumentMode(True)
        gps_tab = QtWidgets.QWidget()
        calc_tab = QtWidgets.QWidget()
        quality_tab = QtWidgets.QWidget()
        gps_layout = QtWidgets.QVBoxLayout(gps_tab)
        calc_layout = QtWidgets.QVBoxLayout(calc_tab)
        quality_layout = QtWidgets.QVBoxLayout(quality_tab)
        for child_layout in (gps_layout, calc_layout, quality_layout):
            child_layout.setContentsMargins(6, 6, 6, 6)
            child_layout.setSpacing(8)
        self._build_baseline_gps_recognition_view(gps_layout)
        self._build_baseline_calculation_view(calc_layout)
        self._build_baseline_quality_view(quality_layout)
        self.baseline_tabs.addTab(gps_tab, "GPS 인식")
        self.baseline_tabs.addTab(calc_tab, "Baseline 계산")
        self.baseline_tabs.addTab(quality_tab, "신뢰성 평가")
        layout.addWidget(self.baseline_tabs, 1)
        self.refresh_baseline_field_browser()
        self._refresh_baseline_alignment_ui()

    def _build_alignment_overview_tab(self, layout: QtWidgets.QVBoxLayout) -> None:
        group = QtWidgets.QGroupBox("정렬 상태 개요")
        grid = QtWidgets.QGridLayout(group)
        self.alignment_overview_labels: dict[str, QtWidgets.QLabel] = {}
        items = (
            ("left_pixhawk", "Left Pixhawk 상태"),
            ("right_pixhawk", "Right Pixhawk 상태"),
            ("left_gps1", "Left GPS1 상태"),
            ("left_gps2", "Left GPS2 상태"),
            ("right_gps1", "Right GPS1 상태"),
            ("right_gps2", "Right GPS2 상태"),
            ("left_can1", "Left CAN1 Magnetometer 상태"),
            ("left_can2", "Left CAN2 Magnetometer 상태"),
            ("right_can1", "Right CAN1 Magnetometer 상태"),
            ("right_can2", "Right CAN2 Magnetometer 상태"),
            ("baseline_length", "Baseline Length"),
            ("baseline_std", "Baseline Std"),
            ("baseline_quality", "Baseline Quality"),
            ("alignment_quality", "Alignment Quality Badge"),
        )
        for index, (key, title) in enumerate(items):
            row = index // 2
            column = (index % 2) * 2
            label = QtWidgets.QLabel("대기 중")
            label.setObjectName("statusStrip")
            label.setWordWrap(True)
            grid.addWidget(QtWidgets.QLabel(title), row, column)
            grid.addWidget(label, row, column + 1)
            self.alignment_overview_labels[key] = label
        layout.addWidget(group, 1)

    def _build_alignment_magnetometer_tab(
        self,
        layout: QtWidgets.QVBoxLayout,
        *,
        include_manager: bool = True,
        include_plot: bool = True,
        expand_manager: bool = False,
    ) -> None:
        scroll_area = QtWidgets.QScrollArea()
        scroll_area.setWidgetResizable(True)
        scroll_area.setFrameShape(QtWidgets.QFrame.NoFrame)
        scroll_area.setHorizontalScrollBarPolicy(QtCore.Qt.ScrollBarAlwaysOff)
        content = QtWidgets.QWidget()
        content.setSizePolicy(QtWidgets.QSizePolicy.Ignored, QtWidgets.QSizePolicy.Preferred)
        content_layout = QtWidgets.QVBoxLayout(content)
        content_layout.setContentsMargins(0, 0, 0, 0)
        content_layout.setSpacing(8)
        scroll_area.setWidget(content)
        layout.addWidget(scroll_area, 1)
        layout = content_layout

        status_group = QtWidgets.QGroupBox("Magnetometer 역할별 상태")
        status_group.setMinimumHeight(300)
        status_layout = QtWidgets.QVBoxLayout(status_group)
        status_layout.setContentsMargins(8, 8, 8, 8)
        status_layout.setSpacing(8)
        headers = ("Device Type", "X", "Y", "Z", "벡터 크기", "선택", "상태", "수신률", "출처")
        self.alignment_mag_labels: dict[str, dict[str, QtWidgets.QLabel]] = {}
        self.alignment_mag_role_tabs = QtWidgets.QTabWidget()
        self.alignment_mag_role_tabs.setDocumentMode(True)
        self.alignment_mag_role_tabs.setSizePolicy(QtWidgets.QSizePolicy.Ignored, QtWidgets.QSizePolicy.Preferred)
        for side, side_label in (("left", "Left"), ("right", "Right")):
            side_group = QtWidgets.QGroupBox(f"{side_label} Pixhawk")
            side_group.setSizePolicy(QtWidgets.QSizePolicy.Ignored, QtWidgets.QSizePolicy.Preferred)
            grid = QtWidgets.QGridLayout(side_group)
            grid.setHorizontalSpacing(8)
            grid.setVerticalSpacing(6)
            for column, header in enumerate(headers):
                header_label = QtWidgets.QLabel(header)
                header_label.setWordWrap(True)
                grid.addWidget(header_label, 0, column)
                grid.setColumnMinimumWidth(column, 0)
                grid.setColumnStretch(column, 1)
            row = 1
            for slot in ("BOARD", "CAN1", "CAN2", "GPS1_MAG", "GPS2_MAG"):
                key = f"{side}_{slot}"
                self.alignment_mag_labels[key] = {}
                for column, field in enumerate(
                    (
                        "device_type",
                        "x",
                        "y",
                        "z",
                        "vector_norm",
                        "selected",
                        "health",
                        "rate",
                        "source",
                    )
                ):
                    label = QtWidgets.QLabel("대기 중")
                    label.setObjectName("statusStrip")
                    label.setWordWrap(True)
                    label.setTextInteractionFlags(QtCore.Qt.TextSelectableByMouse)
                    label.setMinimumWidth(0)
                    label.setSizePolicy(QtWidgets.QSizePolicy.Ignored, QtWidgets.QSizePolicy.Preferred)
                    grid.addWidget(label, row, column)
                    self.alignment_mag_labels[key][field] = label
                row += 1
            self.alignment_mag_role_tabs.addTab(side_group, f"{side_label} Pixhawk")
        status_layout.addWidget(self.alignment_mag_role_tabs)
        layout.addWidget(status_group)
        compare_group = QtWidgets.QGroupBox("Heading / Yaw 비교")
        compare_form = QtWidgets.QGridLayout(compare_group)
        compare_form.setColumnStretch(1, 1)
        self.alignment_mag_compare_labels: dict[str, QtWidgets.QLabel] = {}
        for row, (key, title) in enumerate(
            (
                ("left_can_diff", "Left 주/보조 자기장 차이"),
                ("right_can_diff", "Right 주/보조 자기장 차이"),
                ("left_right_diff", "좌/우 선택 상태 비교"),
            )
        ):
            label = QtWidgets.QLabel("대기 중")
            label.setObjectName("statusStrip")
            label.setWordWrap(True)
            label.setTextInteractionFlags(QtCore.Qt.TextSelectableByMouse)
            compare_form.addWidget(QtWidgets.QLabel(title), row, 0)
            compare_form.addWidget(label, row, 1)
            self.alignment_mag_compare_labels[key] = label
        layout.addWidget(compare_group)

        if include_manager:
            self._build_px4_mag_manager_panel(layout, collapsible=not expand_manager)
            self._build_px4_attitude_cal_panel(layout, collapsible=not expand_manager)
        if include_plot:
            self.alignment_mag_heading_plot = RtkAlignmentPlot()
            self.alignment_mag_heading_plot.setMinimumHeight(160)
            layout.addWidget(self.alignment_mag_heading_plot, 1)
        layout.addStretch(1)

    def _build_alignment_attitude_tab(self, layout: QtWidgets.QVBoxLayout) -> None:
        scroll_area = QtWidgets.QScrollArea()
        scroll_area.setWidgetResizable(True)
        scroll_area.setFrameShape(QtWidgets.QFrame.NoFrame)
        content = QtWidgets.QWidget()
        content_layout = QtWidgets.QVBoxLayout(content)
        content_layout.setContentsMargins(0, 0, 0, 0)
        content_layout.setSpacing(8)
        scroll_area.setWidget(content)
        layout.addWidget(scroll_area, 1)
        layout = content_layout

        group = QtWidgets.QGroupBox("Pixhawk EKF Attitude")
        grid = QtWidgets.QGridLayout(group)
        grid.setVerticalSpacing(6)
        grid.setColumnStretch(1, 1)
        self.alignment_attitude_labels: dict[str, QtWidgets.QLabel] = {}
        for row, (key, title) in enumerate(
            (
                ("left_rpy", "Left Roll/Pitch/Yaw"),
                ("left_q", "Left Quaternion"),
                ("right_rpy", "Right Roll/Pitch/Yaw"),
                ("right_q", "Right Quaternion"),
                ("yaw_diff", "Yaw Difference"),
                ("pitch_diff", "Pitch Difference"),
                ("roll_diff", "Roll Difference"),
            )
        ):
            label = QtWidgets.QLabel("대기 중")
            label.setObjectName("statusStrip")
            label.setWordWrap(True)
            grid.addWidget(QtWidgets.QLabel(title), row, 0)
            grid.addWidget(label, row, 1)
            self.alignment_attitude_labels[key] = label
        group.setSizePolicy(QtWidgets.QSizePolicy.Preferred, QtWidgets.QSizePolicy.Minimum)

        compare_group = QtWidgets.QGroupBox("자세 축 비교")
        compare_form = QtWidgets.QGridLayout(compare_group)
        compare_form.setColumnStretch(1, 1)
        compare_form.setColumnStretch(2, 1)
        compare_form.setColumnStretch(3, 1)
        compare_form.addWidget(QtWidgets.QLabel("축"), 0, 0)
        compare_form.addWidget(QtWidgets.QLabel("Left"), 0, 1)
        compare_form.addWidget(QtWidgets.QLabel("Right"), 0, 2)
        compare_form.addWidget(QtWidgets.QLabel("Difference"), 0, 3)
        self.attitude_compare_labels = {}
        for row, axis in enumerate(("Roll", "Pitch", "Yaw"), start=1):
            left_label = QtWidgets.QLabel("대기 중")
            right_label = QtWidgets.QLabel("대기 중")
            delta_label = QtWidgets.QLabel("대기 중")
            for label in (left_label, right_label, delta_label):
                label.setObjectName("statusStrip")
            compare_form.addWidget(QtWidgets.QLabel(axis), row, 0)
            compare_form.addWidget(left_label, row, 1)
            compare_form.addWidget(right_label, row, 2)
            compare_form.addWidget(delta_label, row, 3)
            self.attitude_compare_labels[axis.lower()] = (left_label, right_label, delta_label)
        attitude_summary_row = QtWidgets.QHBoxLayout()
        attitude_summary_row.setSpacing(8)
        attitude_summary_row.addWidget(group, 2)
        attitude_summary_row.addWidget(compare_group, 1)
        layout.addLayout(attitude_summary_row)

        self.alignment_optical_axis_plot = RtkAlignmentPlot()
        self.alignment_optical_axis_plot.setMinimumHeight(180)
        layout.addWidget(self.alignment_optical_axis_plot, 1)
        self.attitude_plots = {
            "roll": AttitudeHistoryPlot("Roll"),
            "pitch": AttitudeHistoryPlot("Pitch"),
            "yaw": AttitudeHistoryPlot("Yaw"),
        }
        attitude_history_tabs = QtWidgets.QTabWidget()
        attitude_history_tabs.setDocumentMode(True)
        for axis, plot in self.attitude_plots.items():
            plot.setMinimumHeight(180)
            tab = QtWidgets.QWidget()
            tab_layout = QtWidgets.QVBoxLayout(tab)
            tab_layout.setContentsMargins(0, 0, 0, 0)
            tab_layout.addWidget(plot)
            attitude_history_tabs.addTab(tab, axis.title())
        layout.addWidget(attitude_history_tabs, 1)

    def _build_alignment_coordinate_tab(self, layout: QtWidgets.QVBoxLayout) -> None:
        group = QtWidgets.QGroupBox("좌표계 계약")
        form = QtWidgets.QGridLayout(group)
        self.alignment_global_frame_combo = QtWidgets.QComboBox()
        self.alignment_global_frame_combo.addItems(["ENU", "NED"])
        self.alignment_camera_frame_combo = QtWidgets.QComboBox()
        self.alignment_camera_frame_combo.addItems(["OpenCV optical RDF"])
        self.alignment_origin_combo = QtWidgets.QComboBox()
        self.alignment_origin_combo.addItems(["Left Camera", "Midpoint"])
        self.alignment_transform_text = QtWidgets.QPlainTextEdit("0 1 0\n0 0 1\n1 0 0")
        self.alignment_transform_text.setMaximumHeight(90)
        for widget in (self.alignment_global_frame_combo, self.alignment_camera_frame_combo, self.alignment_origin_combo):
            widget.currentIndexChanged.connect(self._refresh_coordinate_system_settings)
        self.alignment_transform_text.textChanged.connect(self._refresh_coordinate_system_settings)
        form.addWidget(QtWidgets.QLabel("Global Frame"), 0, 0)
        form.addWidget(self.alignment_global_frame_combo, 0, 1)
        form.addWidget(QtWidgets.QLabel("Camera Frame"), 1, 0)
        form.addWidget(self.alignment_camera_frame_combo, 1, 1)
        form.addWidget(QtWidgets.QLabel("Origin"), 2, 0)
        form.addWidget(self.alignment_origin_combo, 2, 1)
        form.addWidget(QtWidgets.QLabel("Pixhawk FRD -> Camera RDF"), 3, 0)
        form.addWidget(self.alignment_transform_text, 3, 1)
        layout.addWidget(group)
        self.alignment_tf_tree_label = QtWidgets.QLabel("TF Tree: 대기 중")
        self.alignment_tf_tree_label.setObjectName("statusStrip")
        self.alignment_tf_tree_label.setWordWrap(True)
        layout.addWidget(self.alignment_tf_tree_label)
        layout.addStretch(1)
        self._refresh_coordinate_system_settings()

    def _build_alignment_validation_tab(self, layout: QtWidgets.QVBoxLayout) -> None:
        control = QtWidgets.QHBoxLayout()
        self.alignment_validation_distance_spin = QtWidgets.QDoubleSpinBox()
        self.alignment_validation_distance_spin.setRange(1.0, 10000.0)
        self.alignment_validation_distance_spin.setValue(1852.0)
        self.alignment_validation_distance_spin.setSuffix(" m")
        self.alignment_validation_distance_spin.valueChanged.connect(lambda _value: self._refresh_alignment_validation())
        control.addWidget(QtWidgets.QLabel("Distance"))
        control.addWidget(self.alignment_validation_distance_spin)
        control.addStretch(1)
        layout.addLayout(control)
        group = QtWidgets.QGroupBox("정렬 품질 검증")
        grid = QtWidgets.QGridLayout(group)
        self.alignment_validation_labels: dict[str, QtWidgets.QLabel] = {}
        for row, (key, title) in enumerate(
            (
                ("baseline_length", "Stored vs Current Length Difference"),
                ("baseline_vector", "Stored vs Current Vector Difference"),
                ("yaw", "Stored vs Current Yaw Difference"),
                ("pitch", "Stored vs Current Pitch Difference"),
                ("roll", "Stored vs Current Roll Difference"),
                ("baseline_error", "Baseline Error Contribution"),
                ("yaw_error", "Yaw Error Contribution"),
                ("pitch_error", "Pitch Error Contribution"),
                ("status", "Alignment Status"),
            )
        ):
            label = QtWidgets.QLabel("대기 중")
            label.setObjectName("statusStrip")
            label.setWordWrap(True)
            grid.addWidget(QtWidgets.QLabel(title), row, 0)
            grid.addWidget(label, row, 1)
            self.alignment_validation_labels[key] = label
        layout.addWidget(group)
        layout.addStretch(1)

    def _build_alignment_profile_tab(self, layout: QtWidgets.QVBoxLayout) -> None:
        actions = QtWidgets.QHBoxLayout()
        self.alignment_profile_save_button = QtWidgets.QPushButton("Save")
        self.alignment_profile_load_button = QtWidgets.QPushButton("Load")
        self.alignment_profile_compare_button = QtWidgets.QPushButton("Compare")
        self.alignment_profile_export_button = QtWidgets.QPushButton("Export")
        self.alignment_profile_save_button.clicked.connect(self.save_alignment_profile)
        self.alignment_profile_load_button.clicked.connect(self.load_alignment_profile)
        self.alignment_profile_compare_button.clicked.connect(self._refresh_alignment_validation)
        self.alignment_profile_export_button.clicked.connect(self.save_alignment_profile)
        for button in (
            self.alignment_profile_save_button,
            self.alignment_profile_load_button,
            self.alignment_profile_compare_button,
            self.alignment_profile_export_button,
        ):
            actions.addWidget(button)
        actions.addStretch(1)
        layout.addLayout(actions)
        self.alignment_profile_status_label = QtWidgets.QLabel("Alignment Profile: 대기 중")
        self.alignment_profile_status_label.setObjectName("statusStrip")
        self.alignment_profile_status_label.setWordWrap(True)
        layout.addWidget(self.alignment_profile_status_label)
        self.alignment_profile_preview = QtWidgets.QPlainTextEdit()
        self.alignment_profile_preview.setReadOnly(True)
        layout.addWidget(self.alignment_profile_preview, 1)

    def _build_baseline_gps_recognition_view(self, layout: QtWidgets.QVBoxLayout) -> None:
        row = QtWidgets.QHBoxLayout()
        row.setSpacing(8)
        self.baseline_gps_card_labels: dict[str, dict[str, QtWidgets.QLabel]] = {"left": {}, "right": {}}
        fields = (
            ("connection", "연결 상태"),
            ("gps1", "GPS1 인식"),
            ("gps2", "GPS2 인식"),
            ("fix", "fix_type"),
            ("rtk", "RTK 상태"),
            ("sats", "satellites_visible"),
            ("hdop", "HDOP"),
            ("vdop", "VDOP"),
            ("h_acc", "h_acc"),
            ("v_acc", "v_acc"),
            ("lla", "lat / lon / alt"),
            ("rate", "message rate"),
            ("age", "last update age"),
            ("timestamp", "time_usec / timestamp"),
        )
        for side, title in (("left", "좌측 Pixhawk"), ("right", "우측 Pixhawk")):
            group = QtWidgets.QGroupBox(title)
            form = QtWidgets.QGridLayout(group)
            form.setColumnStretch(1, 1)
            for row_index, (key, label_text) in enumerate(fields):
                label = QtWidgets.QLabel("대기 중")
                label.setObjectName("statusStrip")
                label.setWordWrap(True)
                label.setTextInteractionFlags(QtCore.Qt.TextSelectableByMouse)
                form.addWidget(QtWidgets.QLabel(label_text), row_index, 0)
                form.addWidget(label, row_index, 1)
                self.baseline_gps_card_labels[side][key] = label
            row.addWidget(group, 1)
        layout.addLayout(row, 2)

        detail_group = QtWidgets.QGroupBox("GPS1/GPS2 MAVLink Field 상태")
        detail_form = QtWidgets.QGridLayout(detail_group)
        detail_form.setColumnStretch(1, 1)
        detail_form.setColumnStretch(3, 1)
        for column, title in ((0, "좌측 Pixhawk"), (2, "우측 Pixhawk")):
            detail_form.addWidget(QtWidgets.QLabel(title), 0, column, 1, 2)
        for column, side in ((0, "left"), (2, "right")):
            for row_index, slot in enumerate(("GPS1", "GPS2"), start=1):
                label = QtWidgets.QLabel("대기 중")
                label.setObjectName("statusStrip")
                label.setWordWrap(True)
                label.setTextInteractionFlags(QtCore.Qt.TextSelectableByMouse)
                detail_form.addWidget(QtWidgets.QLabel(slot), row_index, column)
                detail_form.addWidget(label, row_index, column + 1)
                self.f9p_slot_labels[side][slot] = label
                self.gps_sensor_labels[side][slot] = label
            compare_label = QtWidgets.QLabel("GPS1/GPS2 품질 비교: 대기 중")
            compare_label.setObjectName("statusStrip")
            compare_label.setWordWrap(True)
            compare_label.setTextInteractionFlags(QtCore.Qt.TextSelectableByMouse)
            detail_form.addWidget(QtWidgets.QLabel("비교"), 3, column)
            detail_form.addWidget(compare_label, 3, column + 1)
            self.f9p_compare_labels[side] = compare_label
        layout.addWidget(detail_group)

    def _build_baseline_field_browser_view(self, layout: QtWidgets.QVBoxLayout) -> None:
        actions = QtWidgets.QHBoxLayout()
        self.baseline_scan_topics_button = QtWidgets.QPushButton("MAVLink field 스캔")
        self.baseline_scan_topics_button.clicked.connect(self.refresh_baseline_field_browser)
        actions.addWidget(self.baseline_scan_topics_button)
        actions.addStretch(1)
        layout.addLayout(actions)

        self.baseline_mavlink_message_table = QtWidgets.QTableWidget(0, 2)
        self.baseline_mavlink_message_table.setHorizontalHeaderLabels(["Message", "Fields"])
        self.baseline_mavlink_message_table.verticalHeader().setVisible(False)
        self.baseline_mavlink_message_table.setEditTriggers(QtWidgets.QAbstractItemView.NoEditTriggers)
        self.baseline_mavlink_message_table.horizontalHeader().setSectionResizeMode(1, QtWidgets.QHeaderView.Stretch)
        mav_group = QtWidgets.QGroupBox("MAVLink field reference")
        mav_layout = QtWidgets.QVBoxLayout(mav_group)
        mav_layout.addWidget(self.baseline_mavlink_message_table)
        layout.addWidget(mav_group, 1)

    def _build_baseline_source_selection_view(self, layout: QtWidgets.QVBoxLayout) -> None:
        control_group = QtWidgets.QGroupBox("Baseline 입력 선택")
        form = QtWidgets.QGridLayout(control_group)
        self.baseline_left_gps_combo = QtWidgets.QComboBox()
        self.baseline_right_gps_combo = QtWidgets.QComboBox()
        for side, combo in (("left", self.baseline_left_gps_combo), ("right", self.baseline_right_gps_combo)):
            combo.addItem("GPS1", "GPS1")
            combo.addItem("GPS2", "GPS2")
            selected = self._pixhawk_gnss_source(side)
            selected_index = combo.findData(selected)
            if selected_index >= 0:
                combo.setCurrentIndex(selected_index)
            combo.currentIndexChanged.connect(lambda _index: self._handle_baseline_source_changed())
        form.addWidget(QtWidgets.QLabel("좌측 카메라 기준 GPS"), 0, 0)
        form.addWidget(self.baseline_left_gps_combo, 0, 1)
        form.addWidget(QtWidgets.QLabel("우측 카메라 기준 GPS"), 0, 2)
        form.addWidget(self.baseline_right_gps_combo, 0, 3)
        label = QtWidgets.QLabel("선택 소스: P1 GPS1 -> P2 GPS1")
        label.setObjectName("statusStrip")
        label.setWordWrap(True)
        label.setSizePolicy(QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Fixed)
        label.setMaximumHeight(42)
        form.addWidget(label, 1, 0, 1, 4)
        if not hasattr(self, "baseline_source_summary_labels"):
            self.baseline_source_summary_labels = []
        self.baseline_source_summary_labels.append(label)
        layout.addWidget(control_group)
        self._refresh_baseline_source_summary()

    def _build_baseline_calculation_view(
        self,
        layout: QtWidgets.QVBoxLayout,
        *,
        include_source_controls: bool = True,
        include_alignment_plot: bool = True,
    ) -> None:
        if include_source_controls:
            self._build_baseline_source_selection_view(layout)
        else:
            label = QtWidgets.QLabel("선택 소스: P1 GPS1 -> P2 GPS1")
            label.setObjectName("statusStrip")
            label.setWordWrap(True)
            label.setSizePolicy(QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Fixed)
            label.setMaximumHeight(42)
            layout.addWidget(label)
            if not hasattr(self, "baseline_source_summary_labels"):
                self.baseline_source_summary_labels = []
            self.baseline_source_summary_labels.append(label)
            self._refresh_baseline_source_summary()
        summary_group = QtWidgets.QGroupBox("Baseline 계산 결과")
        summary_group.setSizePolicy(QtWidgets.QSizePolicy.Preferred, QtWidgets.QSizePolicy.Maximum)
        summary_form = QtWidgets.QGridLayout(summary_group)
        self.baseline_calc_labels: dict[str, QtWidgets.QLabel] = {}
        for row, (key, label_text) in enumerate(
            (
                ("enu", "baseline vector ENU [E,N,U]"),
                ("ned", "baseline vector NED [N,E,D]"),
                ("p1_body_frd", "P1→P2 in P1 body FRD [F,R,D]"),
                ("length", "baseline length"),
                ("azimuth", "azimuth"),
                ("elevation", "elevation"),
                ("altitude_delta", "left-right altitude difference"),
                ("timestamp_delta", "left-right timestamp delta"),
            )
        ):
            label = QtWidgets.QLabel("대기 중")
            label.setObjectName("statusStrip")
            label.setWordWrap(True)
            label.setSizePolicy(QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Fixed)
            label.setMaximumHeight(42)
            summary_form.addWidget(QtWidgets.QLabel(label_text), row, 0)
            summary_form.addWidget(label, row, 1)
            self.baseline_calc_labels[key] = label
        layout.addWidget(summary_group)
        explanation_group = QtWidgets.QGroupBox("계산 기준")
        explanation_layout = QtWidgets.QVBoxLayout(explanation_group)
        explanation_label = QtWidgets.QLabel(
            "Baseline = P2 GNSS - P1 GNSS입니다.\n"
            "ENU [E,N,U]는 East/North/Up 순서이고, NED [N,E,D]는 North/East/Down 순서입니다. "
            "Down은 -Up입니다.\n"
            "baseline length는 선택된 좌우 GNSS 사이의 3D 거리입니다. "
            "azimuth는 북쪽 기준 수평 방향각, elevation은 수평면 대비 기울기입니다.\n"
            "altitude difference는 P2 altitude - P1 altitude입니다. "
            "timestamp delta는 P1/P2 GNSS 샘플 시간 차이이며, 클수록 baseline 신뢰도가 낮습니다.\n"
            "P1→P2 in P1 body FRD는 같은 raw NED 벡터를 P1 Pixhawk 자세의 역회전으로 "
            "[Forward, Right, Down] 순서로 다시 표현한 파생값입니다. "
            "카메라 optical RDF나 public baseline_center_frd 측정값이 아닙니다.\n"
            "이 탭은 GNSS 위치 기반 baseline만 계산합니다. "
            "Forward와 baseline의 직각 여부는 Baseline / 자세 정렬 탭에서 확인합니다."
        )
        explanation_label.setObjectName("statusStrip")
        explanation_label.setWordWrap(True)
        explanation_label.setTextInteractionFlags(QtCore.Qt.TextSelectableByMouse)
        explanation_layout.addWidget(explanation_label)
        layout.addWidget(explanation_group)
        layout.addStretch(1)

        if include_alignment_plot:
            self._build_baseline_attitude_alignment_tab(layout)

    def _build_baseline_attitude_alignment_tab(self, layout: QtWidgets.QVBoxLayout) -> None:
        metrics_group = QtWidgets.QGroupBox("Baseline / 자세 정렬 지표")
        metrics_form = QtWidgets.QGridLayout(metrics_group)
        metrics_form.setColumnStretch(1, 1)
        self.baseline_attitude_labels: dict[str, QtWidgets.QLabel] = {}
        for row, (key, title) in enumerate(
            (
                ("rdf_forward_angle", "P1 RDF Forward vs Baseline (deg)"),
                ("effective_baseline", "P1 effective baseline (RDF Right/Down)"),
                ("forward_component", "P1 Forward component"),
                ("p1_body_frd", "P1→P2 in P1 body FRD [F,R,D]"),
                ("right_rdf_forward_angle", "P2 RDF Forward vs Baseline (deg)"),
                ("right_effective_baseline", "P2 effective baseline (RDF Right/Down)"),
                ("right_forward_component", "P2 Forward component"),
                ("left_rpy", "P1 Roll / Pitch / Yaw"),
                ("right_rpy", "P2 Roll / Pitch / Yaw"),
                ("yaw_diff", "P1-P2 Yaw difference"),
            )
        ):
            label = QtWidgets.QLabel("대기 중")
            label.setObjectName("statusStrip")
            label.setWordWrap(True)
            label.setTextInteractionFlags(QtCore.Qt.TextSelectableByMouse)
            metrics_form.addWidget(QtWidgets.QLabel(title), row, 0)
            metrics_form.addWidget(label, row, 1)
            self.baseline_attitude_labels[key] = label
        layout.addWidget(metrics_group)

        self.rtk_alignment_plot = RtkAlignmentPlot()
        self.rtk_alignment_detail_label = QtWidgets.QLabel("정렬 성분: GPS baseline 대기 중")
        self.rtk_alignment_detail_label.setObjectName("statusStrip")
        self.rtk_alignment_detail_label.setWordWrap(True)
        layout.addWidget(self.rtk_alignment_detail_label)
        layout.addWidget(self.rtk_alignment_plot, 1)

    def _build_optical_alignment_tab(self, layout: QtWidgets.QVBoxLayout) -> None:
        header_group = QtWidgets.QGroupBox("Optical 정렬 상태")
        header_layout = QtWidgets.QGridLayout(header_group)
        header_layout.setColumnStretch(1, 1)
        self.optical_alignment_status_label = QtWidgets.QLabel("Optical alignment: YOLO target pair 대기 중")
        self.optical_alignment_status_label.setObjectName("statusStrip")
        self.optical_alignment_status_label.setWordWrap(True)
        self.optical_alignment_status_label.setTextInteractionFlags(QtCore.Qt.TextSelectableByMouse)
        self.optical_alignment_note_label = QtWidgets.QLabel(
            "Alignment preview only: 계산 파라미터를 변경하지 않습니다. 30x에서 물리 R을 조절하고 8x에서 재확인하세요."
        )
        self.optical_alignment_note_label.setObjectName("statusStrip")
        self.optical_alignment_note_label.setWordWrap(True)
        header_layout.addWidget(self.optical_alignment_status_label, 0, 0, 1, 4)
        header_layout.addWidget(self.optical_alignment_note_label, 1, 0, 1, 4)
        layout.addWidget(header_group)

        controls_group = QtWidgets.QGroupBox("기대값 기준")
        controls_layout = QtWidgets.QGridLayout(controls_group)
        controls_layout.setColumnStretch(5, 1)
        self.optical_expected_dy_combo = QtWidgets.QComboBox()
        self.optical_expected_dy_combo.addItem("Rectified 0 px", "rectified")
        self.optical_expected_dy_combo.addItem("Raw intrinsics cy 차이", "raw_intrinsics")
        self.optical_expected_dy_combo.addItem("Manual", "manual")
        self.optical_expected_dy_combo.setToolTip(
            "dy residual 기준입니다. Rectified는 0 px, Raw는 left cy - right cy, Manual은 입력값을 사용합니다."
        )
        self.optical_expected_dy_spin = QtWidgets.QDoubleSpinBox()
        self.optical_expected_dy_spin.setRange(-5000.0, 5000.0)
        self.optical_expected_dy_spin.setDecimals(3)
        self.optical_expected_dy_spin.setSuffix(" px")
        self.optical_expected_dy_spin.setValue(0.0)
        self.optical_target_distance_spin = QtWidgets.QDoubleSpinBox()
        self.optical_target_distance_spin.setRange(0.1, 100000.0)
        self.optical_target_distance_spin.setDecimals(2)
        self.optical_target_distance_spin.setValue(100.0)
        self.optical_target_distance_spin.setSuffix(" m")
        self.optical_target_distance_spin.setToolTip(
            "Yaw residual 계산용 target 거리입니다. expected disparity = fx * baseline / target distance입니다."
        )
        controls_layout.addWidget(QtWidgets.QLabel("expected dy"), 0, 0)
        controls_layout.addWidget(self.optical_expected_dy_combo, 0, 1)
        controls_layout.addWidget(QtWidgets.QLabel("manual dy"), 0, 2)
        controls_layout.addWidget(self.optical_expected_dy_spin, 0, 3)
        controls_layout.addWidget(QtWidgets.QLabel("target distance"), 0, 4)
        controls_layout.addWidget(self.optical_target_distance_spin, 0, 5)
        layout.addWidget(controls_group)

        self.optical_alignment_labels: dict[str, QtWidgets.QLabel] = {}
        top_row = QtWidgets.QHBoxLayout()
        top_row.setSpacing(8)
        observation_group = QtWidgets.QGroupBox("좌/우 Target 관측")
        observation_form = QtWidgets.QGridLayout(observation_group)
        for row, (key, title) in enumerate(
            (
                ("left_center", "L center"),
                ("right_center", "R center"),
                ("bbox_delta", "bbox dx / dy"),
                ("bbox_quality", "bbox conf / class"),
                ("camera_time", "camera pair Δt"),
            )
        ):
            self._add_optical_alignment_label(observation_form, row, key, title)
        top_row.addWidget(observation_group, 1)

        vertical_group = QtWidgets.QGroupBox("Vertical / Pitch 정렬")
        vertical_form = QtWidgets.QGridLayout(vertical_group)
        for row, (key, title) in enumerate(
            (
                ("expected_dy", "expected dy"),
                ("observed_dy", "observed dy"),
                ("dy_residual", "dy residual"),
                ("vertical_error", "vertical error"),
            )
        ):
            self._add_optical_alignment_label(vertical_form, row, key, title)
        top_row.addWidget(vertical_group, 1)
        layout.addLayout(top_row)

        bottom_row = QtWidgets.QHBoxLayout()
        bottom_row.setSpacing(8)
        yaw_group = QtWidgets.QGroupBox("Horizontal / Yaw 정렬")
        yaw_form = QtWidgets.QGridLayout(yaw_group)
        for row, (key, title) in enumerate(
            (
                ("observed_dx", "observed dx"),
                ("expected_disparity", "expected disparity"),
                ("yaw_residual", "yaw residual"),
                ("yaw_error", "yaw error"),
            )
        ):
            self._add_optical_alignment_label(yaw_form, row, key, title)
        bottom_row.addWidget(yaw_group, 1)

        distance_group = QtWidgets.QGroupBox("Live 거리 영향")
        distance_form = QtWidgets.QGridLayout(distance_group)
        for row, (key, title) in enumerate(
            (
                ("range", "YOLO Range"),
                ("ray_gap", "ray gap"),
                ("baseline", "baseline"),
                ("triangulation", "triangulation"),
                ("attitude", "attitude status"),
            )
        ):
            self._add_optical_alignment_label(distance_form, row, key, title)
        bottom_row.addWidget(distance_group, 1)
        layout.addLayout(bottom_row)
        layout.addStretch(1)

        self.optical_expected_dy_combo.currentIndexChanged.connect(
            lambda _index: self._refresh_optical_alignment_view()
        )
        self.optical_expected_dy_spin.valueChanged.connect(lambda _value: self._refresh_optical_alignment_view())
        self.optical_target_distance_spin.valueChanged.connect(lambda _value: self._refresh_optical_alignment_view())
        self._refresh_optical_alignment_view()

    def _add_optical_alignment_label(
        self,
        form: QtWidgets.QGridLayout,
        row: int,
        key: str,
        title: str,
    ) -> None:
        label = QtWidgets.QLabel("대기 중")
        label.setObjectName("statusStrip")
        label.setWordWrap(True)
        label.setTextInteractionFlags(QtCore.Qt.TextSelectableByMouse)
        form.addWidget(QtWidgets.QLabel(title), row, 0)
        form.addWidget(label, row, 1)
        self.optical_alignment_labels[key] = label

    def _build_baseline_quality_view(self, layout: QtWidgets.QVBoxLayout) -> None:
        actions = QtWidgets.QHBoxLayout()
        self.baseline_sample_duration_spin = QtWidgets.QDoubleSpinBox()
        self.baseline_sample_duration_spin.setRange(10.0, 30.0)
        self.baseline_sample_duration_spin.setValue(10.0)
        self.baseline_sample_duration_spin.setSuffix(" s")
        self.baseline_start_sampling_button = QtWidgets.QPushButton("Start Sampling")
        self.baseline_stop_sampling_button = QtWidgets.QPushButton("Stop Sampling")
        self.baseline_start_sampling_button.clicked.connect(self.start_baseline_sampling)
        self.baseline_stop_sampling_button.clicked.connect(self.stop_baseline_sampling)
        actions.addWidget(QtWidgets.QLabel("수집 시간"))
        actions.addWidget(self.baseline_sample_duration_spin)
        actions.addWidget(self.baseline_start_sampling_button)
        actions.addWidget(self.baseline_stop_sampling_button)
        actions.addStretch(1)
        layout.addLayout(actions)

        self.baseline_quality_badge = QtWidgets.QLabel("Quality: 대기 중")
        self.baseline_quality_badge.setObjectName("distanceStatus")
        layout.addWidget(self.baseline_quality_badge)
        stats_group = QtWidgets.QGroupBox("신뢰성 평가")
        stats_form = QtWidgets.QGridLayout(stats_group)
        self.baseline_quality_labels: dict[str, QtWidgets.QLabel] = {}
        for row, (key, label_text) in enumerate(
            (
                ("length_mean", "baseline length mean"),
                ("length_std", "baseline length std"),
                ("enu_mean", "ENU vector mean"),
                ("enu_std", "ENU vector std"),
                ("rtk_ratio", "RTK Fixed ratio"),
                ("message_rate", "GPS message rate"),
                ("timestamp_delta", "left-right timestamp delta"),
                ("outliers", "outlier count"),
                ("jumps", "baseline jump count"),
                ("acc", "mean h_acc / v_acc"),
            )
        ):
            label = QtWidgets.QLabel("대기 중")
            label.setObjectName("statusStrip")
            label.setWordWrap(True)
            stats_form.addWidget(QtWidgets.QLabel(label_text), row, 0)
            stats_form.addWidget(label, row, 1)
            self.baseline_quality_labels[key] = label
        layout.addWidget(stats_group)
        self.baseline_length_plot = TelemetryHistoryPlot("Baseline length time-series", "m")
        layout.addWidget(self.baseline_length_plot, 1)

    def _handle_baseline_source_changed(self) -> None:
        if hasattr(self, "baseline_left_gps_combo"):
            self.set_pixhawk_gnss_source(
                "left",
                str(self.baseline_left_gps_combo.currentData() or "GPS1"),
                persist=False,
            )
            self.set_pixhawk_gnss_source(
                "right",
                str(self.baseline_right_gps_combo.currentData() or "GPS1"),
                persist=False,
            )
            self._save_current_settings()
        self._refresh_baseline_source_summary()
        self._refresh_rtk_baseline()
        self._refresh_baseline_alignment_ui()

    def _refresh_baseline_source_summary(self) -> None:
        if not hasattr(self, "baseline_source_summary_labels"):
            self.baseline_source_summary_labels = []
        left = "GPS1"
        right = "GPS1"
        if hasattr(self, "baseline_left_gps_combo"):
            left = str(self.baseline_left_gps_combo.currentData() or "GPS1")
            right = str(self.baseline_right_gps_combo.currentData() or "GPS1")
        text = f"선택 소스: P1 {left} -> P2 {right}"
        for label in list(getattr(self, "baseline_source_summary_labels", [])):
            self._set_label_text_if_changed(label, text)

    def refresh_baseline_field_browser(self) -> None:
        if not hasattr(self, "baseline_mavlink_message_table"):
            return
        mavlink_rows = {
            str(item.get("name") or ""): ", ".join(
                mavlink_graphable_fields_for_message(str(item.get("name") or ""))
            )
            for item in discover_mavlink_messages()
            if str(item.get("name") or "")
        }
        self.baseline_mavlink_message_table.setRowCount(len(mavlink_rows))
        for row, (message, fields) in enumerate(mavlink_rows.items()):
            self.baseline_mavlink_message_table.setItem(row, 0, QtWidgets.QTableWidgetItem(message))
            self.baseline_mavlink_message_table.setItem(row, 1, QtWidgets.QTableWidgetItem(fields))

    def _selected_baseline_gps_state(self, side: str) -> GpsPositionState | None:
        if hasattr(self, "baseline_left_gps_combo"):
            combo = self.baseline_left_gps_combo if side == "left" else self.baseline_right_gps_combo
            slot = str(combo.currentData() or "GPS1")
        else:
            slot = "GPS1"
        if slot == "GPS1":
            return self.latest_gps.get(side)
        # GPS2 position is only usable when the producer emits raw GPS2 lat/lon into latest_gps.
        state = self.latest_gps.get(f"{side}_GPS2")
        return state if isinstance(state, GpsPositionState) else None

    def _selected_baseline_gps_states(self) -> tuple[GpsPositionState | None, GpsPositionState | None]:
        return self._selected_baseline_gps_state("left"), self._selected_baseline_gps_state("right")

    def _current_baseline_vector(self) -> tuple[BaselineVector, GpsPositionState, GpsPositionState] | None:
        left, right = self._selected_baseline_gps_states()
        if left is None or right is None:
            return None
        vector = baseline_from_gps(left, right)
        if vector is None or vector.timestamp_delta_ms > 200.0:
            return None
        return vector, left, right

    def _refresh_p1_body_frd_baseline(self) -> BodyFrdBaselineProjection | None:
        """Refresh the explicit P1→P2 body-FRD projection without replacing raw NED."""

        baseline = self.latest_rtk_baseline
        p1_attitude = self.latest_extrinsics.get("left")
        projection = None
        if baseline is not None and p1_attitude is not None:
            projection = project_ned_baseline_to_p1_body_frd(
                (baseline.north_m, baseline.east_m, baseline.down_m),
                p1_attitude.q,
                attitude_timestamp_s=p1_attitude.timestamp_s,
                attitude_age_s=max(0.0, time.monotonic() - p1_attitude.timestamp_s),
            )
        self.latest_p1_body_frd_baseline = projection
        self._set_p1_body_frd_baseline_labels(projection)
        return projection

    def _set_p1_body_frd_baseline_labels(
        self,
        projection: BodyFrdBaselineProjection | None,
    ) -> None:
        if projection is None:
            if hasattr(self, "baseline_calc_labels"):
                self._set_label_text_if_changed(self.baseline_calc_labels["p1_body_frd"], "P1 자세 대기 중")
            if hasattr(self, "baseline_attitude_labels"):
                self._set_health_label(
                    self.baseline_attitude_labels["p1_body_frd"],
                    "P1 자세 대기 중",
                    "warn",
                )
            return
        value_text = (
            f"[F={projection.forward_m:+.4f}, R={projection.right_m:+.4f}, "
            f"D={projection.down_m:+.4f}] m"
        )
        if projection.attitude_age_s is not None:
            value_text += f" | attitude age={projection.attitude_age_s * 1000.0:.0f} ms"
        if hasattr(self, "baseline_calc_labels"):
            self._set_label_text_if_changed(self.baseline_calc_labels["p1_body_frd"], value_text)
        if hasattr(self, "baseline_attitude_labels"):
            baseline = self.latest_rtk_baseline
            level = "good" if baseline is not None and baseline.fix_quality == "GOOD" else "warn"
            self._set_health_label(self.baseline_attitude_labels["p1_body_frd"], value_text, level)

    def _refresh_baseline_alignment_ui(self) -> None:
        self._refresh_baseline_gps_cards()
        current = self._current_baseline_vector()
        if current is None:
            self.latest_baseline_vector = None
            self.latest_p1_body_frd_baseline = None
            if hasattr(self, "baseline_calc_labels"):
                for label in self.baseline_calc_labels.values():
                    self._set_health_label(label, "대기 중", "idle")
            if hasattr(self, "baseline_attitude_labels"):
                for label in self.baseline_attitude_labels.values():
                    self._set_health_label(label, "대기 중", "idle")
            if hasattr(self, "rtk_alignment_detail_label"):
                self._set_label_text_if_changed(
                    self.rtk_alignment_detail_label,
                    "정렬 성분: 좌/우 GPS 위치 또는 timestamp 동기화 대기 중",
                )
            self._set_p1_body_frd_baseline_labels(None)
            return
        vector, left, right = current
        self.latest_baseline_vector = vector
        if hasattr(self, "baseline_calc_labels"):
            self._set_label_text_if_changed(
                self.baseline_calc_labels["enu"],
                f"[{vector.east_m:+.4f}, {vector.north_m:+.4f}, {vector.up_m:+.4f}] m",
            )
            self._set_label_text_if_changed(
                self.baseline_calc_labels["ned"],
                f"[{vector.north_m:+.4f}, {vector.east_m:+.4f}, {-vector.up_m:+.4f}] m",
            )
            self._set_label_text_if_changed(self.baseline_calc_labels["length"], f"{vector.length_m:.4f} m")
            self._set_label_text_if_changed(self.baseline_calc_labels["azimuth"], f"{vector.azimuth_deg:.2f} deg")
            self._set_label_text_if_changed(self.baseline_calc_labels["elevation"], f"{vector.elevation_deg:.2f} deg")
            self._set_label_text_if_changed(
                self.baseline_calc_labels["altitude_delta"], f"{vector.altitude_delta_m:+.4f} m"
            )
            self._set_label_text_if_changed(
                self.baseline_calc_labels["timestamp_delta"], f"{vector.timestamp_delta_ms:.1f} ms"
            )
        if hasattr(self, "rtk_alignment_plot"):
            # The Measurement controller is the sole owner of
            # latest_rtk_baseline.  Alignment derives display-only ENU/angle
            # values without replacing that canonical estimate.
            estimate = self.latest_rtk_baseline
            if estimate is None:
                self.latest_p1_body_frd_baseline = None
                self.rtk_alignment_plot.set_alignment(
                    None,
                    left_attitude=self.latest_extrinsics.get("left"),
                    right_attitude=self.latest_extrinsics.get("right"),
                )
                self._set_label_text_if_changed(
                    self.rtk_alignment_detail_label,
                    "정렬 성분: canonical GPS/RTK baseline 계산 대기 중",
                )
            else:
                self._refresh_p1_body_frd_baseline()
                camera_translation = self._alignment_view_camera_translation_right_in_left_m(estimate)
                right_camera_translation = self._baseline_camera_translation_in_side_rdf_m(estimate, "right")
                self.rtk_alignment_plot.set_alignment(
                    estimate,
                    camera_translation,
                    self.latest_extrinsics.get("left"),
                    self.latest_extrinsics.get("right"),
                )
                geometry = self._baseline_forward_geometry_from_camera_translation(camera_translation)
                right_geometry = self._baseline_forward_geometry_from_camera_translation(right_camera_translation)
                if hasattr(self, "baseline_attitude_labels"):
                    self._set_baseline_forward_metric_labels(prefix="left", geometry=geometry)
                    self._set_baseline_forward_metric_labels(prefix="right", geometry=right_geometry)
                right_geometry_text = self._baseline_forward_geometry_text(right_camera_translation).replace(
                    "P1 RDF Forward", "P2 RDF Forward"
                )
                self._set_label_text_if_changed(
                    self.rtk_alignment_detail_label,
                    f"ENU E/N/U={vector.east_m:+.3f}/{vector.north_m:+.3f}/{vector.up_m:+.3f} m | "
                    f"NED N/E/D={vector.north_m:+.3f}/{vector.east_m:+.3f}/{-vector.up_m:+.3f} m | "
                    f"{self._baseline_forward_geometry_text(camera_translation)} | {right_geometry_text}",
                )
        else:
            self._refresh_p1_body_frd_baseline()
        if hasattr(self, "baseline_calc_labels"):
            self._set_p1_body_frd_baseline_labels(self.latest_p1_body_frd_baseline)
        self._refresh_alignment_overview()
        self._refresh_alignment_validation()

    def _refresh_baseline_gps_cards(self) -> None:
        if not hasattr(self, "baseline_gps_card_labels"):
            return
        now_s = time.monotonic()
        for side, labels in self.baseline_gps_card_labels.items():
            gps = self._selected_baseline_gps_state(side)
            sensors = self.latest_gps_sensors.get(side, {})
            connected = gps is not None or bool(sensors)
            self._set_health_label(labels["connection"], "connected" if connected else "waiting", "good" if connected else "idle")
            for slot_key, label_key in (("GPS1", "gps1"), ("GPS2", "gps2")):
                state = sensors.get(slot_key)
                level = "good" if state is not None else "idle"
                text = "인식" if state is not None else "대기"
                if state is not None and state.fix_type is not None:
                    text += f" fix={state.fix_type}"
                self._set_health_label(labels[label_key], text, level)
            if gps is None:
                for key in ("fix", "rtk", "sats", "hdop", "vdop", "h_acc", "v_acc", "lla", "rate", "age", "timestamp"):
                    labels[key].setText("대기 중")
                continue
            age_ms = max(0.0, now_s - gps.timestamp_s) * 1000.0
            rate = self._baseline_message_rate(side)
            labels["fix"].setText("--" if gps.fix_type is None else str(gps.fix_type))
            self._set_health_label(labels["rtk"], gps.rtk_label, self._gps_fix_level(gps.fix_type, age_ms=age_ms))
            labels["sats"].setText("--" if gps.satellites_used is None else str(gps.satellites_used))
            labels["hdop"].setText("--" if gps.hdop is None else f"{gps.hdop:.2f}")
            labels["vdop"].setText("--" if gps.vdop is None else f"{gps.vdop:.2f}")
            labels["h_acc"].setText("--" if gps.eph_m is None else f"{gps.eph_m:.3f} m")
            labels["v_acc"].setText("--" if gps.epv_m is None else f"{gps.epv_m:.3f} m")
            if gps.has_position:
                labels["lla"].setText(f"{gps.latitude_deg:.8f}, {gps.longitude_deg:.8f}, {gps.altitude_m:.3f} m")
            else:
                labels["lla"].setText("position 대기")
            labels["rate"].setText(f"{rate:.2f} Hz")
            labels["age"].setText(f"{age_ms:.0f} ms")
            labels["timestamp"].setText("--" if gps.px4_timestamp_s is None else f"{gps.px4_timestamp_s:.6f} s")

    def _refresh_alignment_overview(self) -> None:
        if not hasattr(self, "alignment_overview_labels"):
            return
        labels = self.alignment_overview_labels
        for side, key in (("left", "left_pixhawk"), ("right", "right_pixhawk")):
            connected = side in self.latest_gps or side in self.latest_extrinsics
            self._set_health_label(labels[key], "connected" if connected else "waiting", "good" if connected else "idle")
        for side in ("left", "right"):
            for slot in ("GPS1", "GPS2"):
                state = self.latest_gps_sensors.get(side, {}).get(slot)
                level = "good" if state is not None else "idle"
                text = "인식" if state is not None else "대기"
                self._set_health_label(labels[f"{side}_{slot.lower()}"], text, level)
            for slot in ("CAN1", "CAN2"):
                state = self._mag_slot_state(side, slot)
                level = "good" if state is not None else "idle"
                text = "인식" if state is not None else "대기"
                self._set_health_label(labels[f"{side}_{slot.lower()}"], text, level)
        vector = self.latest_baseline_vector
        quality = self.latest_baseline_quality
        labels["baseline_length"].setText("대기 중" if vector is None else f"{vector.length_m:.4f} m")
        labels["baseline_std"].setText("대기 중" if quality is None else f"{quality.baseline_length_std_m:.4f} m")
        self._set_health_label(
            labels["baseline_quality"],
            "대기 중" if quality is None else quality.grade,
            "idle" if quality is None else "good" if quality.grade == "GOOD" else "warn" if quality.grade == "WARNING" else "bad",
        )
        grade = "GOOD" if quality is not None and quality.grade == "GOOD" else "WARNING" if vector is not None else "BAD"
        self._set_health_label(labels["alignment_quality"], grade, "good" if grade == "GOOD" else "warn" if grade == "WARNING" else "bad")

    def _magnetometer_role_default_device_id(self, slot: str) -> int | None:
        return {
            "BOARD": BOARD_INTERNAL_MAG_ID,
            "CAN1": RM3100_PRIMARY_ID,
            "CAN2": RM3100_BACKUP_ID,
            "GPS1_MAG": GPS1_MAG_ID,
            "GPS2_MAG": GPS2_MAG_ID,
        }.get(slot)

    def _magnetometer_device_type(self, slot: str, device_id: int | None) -> str:
        resolved_id = device_id if device_id is not None else self._magnetometer_role_default_device_id(slot)
        return identify_mag_device(resolved_id).device_type

    def _magnetometer_pending_text(self, camera: str) -> str:
        if self.latest_vehicle_mag_device_ids.get(camera) is not None:
            return "selected telemetry만 수신 · 역할 raw sample 대기"
        return "MAG sample 대기"

    def _refresh_alignment_magnetometer_ui(self) -> None:
        if not hasattr(self, "alignment_mag_labels"):
            return
        for side in ("left", "right"):
            for slot in ("BOARD", "CAN1", "CAN2", "GPS1_MAG", "GPS2_MAG"):
                key = f"{side}_{slot}"
                labels = self.alignment_mag_labels[key]
                state = self._mag_slot_state(side, slot)
                if state is None:
                    text = self._magnetometer_pending_text(side)
                    labels["device_type"].setText(self._magnetometer_device_type(slot, None))
                    for field in ("x", "y", "z", "vector_norm"):
                        labels[field].setText("--")
                    for field in ("selected", "health", "rate", "source"):
                        labels[field].setText(text)
                    continue
                device_id, device_source = self._mag_state_display_device_id(side, state)
                actual_device_id = self.latest_vehicle_mag_device_ids.get(side)
                labels["device_type"].setText(self._magnetometer_device_type(slot, device_id))
                if device_id is None:
                    selected_text = "확인 불가"
                elif actual_device_id is None:
                    selected_text = "선택 확인 대기"
                elif actual_device_id == device_id:
                    selected_text = "실제 선택"
                else:
                    selected_text = "미선택"
                labels["selected"].setText(selected_text)
                units = self._mag_state_value_units(state)
                for field, value in (
                    ("x", getattr(state, "x", None)),
                    ("y", getattr(state, "y", None)),
                    ("z", getattr(state, "z", None)),
                    ("vector_norm", getattr(state, "magnitude", None)),
                ):
                    labels[field].setText(self._mag_state_value_text(value, units))
                complete = all(
                    getattr(state, field, None) is not None
                    for field in ("x", "y", "z", "magnitude")
                )
                labels["health"].setText("정상" if complete else "부분 수신")
                labels["rate"].setText("live")
                labels["source"].setText(
                    {
                        "telemetry": "MAVLink telemetry",
                        "PX4 Shell": "PX4 Shell sensor_mag",
                        "PX4 param inferred": "PX4 추정",
                        "unknown": "확인 불가",
                    }.get(device_source, device_source)
                )
        for side in ("left", "right"):
            difference, units = self.magnetometer_monitor.can_difference_display(side)
            self.alignment_mag_compare_labels[f"{side}_can_diff"].setText(
                "--" if difference is None else self._mag_state_value_text(difference, units)
            )
        self.alignment_mag_compare_labels["left_right_diff"].setText("heading source 대기")

    @staticmethod
    def _mag_state_value_units(state: object) -> str:
        return magnetometer_value_units(getattr(state, "topic", ""))

    @staticmethod
    def _mag_state_value_text(value: object, units: str) -> str:
        if value is None:
            return "--"
        text = f"{float(value):.3f}"
        return f"{text} {units}" if units else text

    def _mag_slot_state(self, camera: str, slot: str) -> object | None:
        """Prefer a dedicated CAN-mag sample over the legacy SCALED_IMU-labeled one.

        ``latest_can_mag_sensors`` only ever holds genuine CAN_MAG/DEBUG_VECT
        readings, while ``latest_mag_sensors`` reuses the same "CAN1"/"CAN2"
        slot names for onboard-IMU-derived samples; the dedicated dict must
        win whenever it has data so real CAN telemetry is never hidden behind
        a stale/absent legacy sample.
        """

        if slot in ("CAN1", "CAN2"):
            dedicated = self.latest_can_mag_sensors.get(camera, {}).get(slot)
            if dedicated is not None:
                return dedicated
        return self.latest_mag_sensors.get(camera, {}).get(slot)

    def _mag_state_display_device_id(self, camera: str, state: object) -> tuple[int | None, str]:
        device_id = getattr(state, "device_id", None)
        if device_id is not None:
            topic = str(getattr(state, "topic", ""))
            if topic.lower().startswith("px4-shell/sensor_mag/"):
                return int(device_id), "PX4 Shell"
            return int(device_id), "telemetry"
        topic = str(getattr(state, "topic", ""))
        if topic.startswith("mavlink/SCALED_IMU"):
            inferred = self._inferred_rm3100_device_id_for_slot(camera, str(getattr(state, "sensor_slot", "")))
            if inferred is not None:
                return inferred, "PX4 param inferred"
        return None, "unknown"

    def _inferred_rm3100_device_id_for_slot(self, camera: str, sensor_slot: str) -> int | None:
        manager = self.px4_mag_managers.get(camera) if hasattr(self, "px4_mag_managers") else None
        params = None
        if manager is not None:
            store = getattr(manager, "store", None)
            values = getattr(store, "values", None)
            if isinstance(values, dict):
                params = values
        if params is None:
            params = current_pixhawk_6x_mag_params()
        rm_ids = []
        for value in params.values():
            try:
                candidate = int(value)
            except (TypeError, ValueError):
                continue
            if candidate in RM3100_IDS and candidate not in rm_ids:
                rm_ids.append(candidate)
        if sensor_slot == "CAN1":
            return rm_ids[0] if rm_ids else RM3100_PRIMARY_ID
        if sensor_slot == "CAN2":
            if len(rm_ids) >= 2:
                return rm_ids[1]
            return RM3100_BACKUP_ID
        return None

    def _refresh_alignment_attitude_ui(self) -> None:
        if not hasattr(self, "alignment_attitude_labels") and not hasattr(self, "baseline_attitude_labels"):
            return
        for side, prefix in (("left", "left"), ("right", "right")):
            state = self.latest_extrinsics.get(side)
            rpy_text = "대기 중"
            q_text = "대기 중"
            if state is not None:
                rpy_text = f"R={state.roll_deg:+.3f} P={state.pitch_deg:+.3f} Y={state.yaw_deg:+.3f} deg"
                q_text = ", ".join(f"{value:.6f}" for value in state.q)
            if hasattr(self, "alignment_attitude_labels"):
                if state is None:
                    self.alignment_attitude_labels[f"{prefix}_rpy"].setText("대기 중")
                    self.alignment_attitude_labels[f"{prefix}_q"].setText("대기 중")
                else:
                    self.alignment_attitude_labels[f"{prefix}_rpy"].setText(rpy_text)
                    self.alignment_attitude_labels[f"{prefix}_q"].setText(q_text)
            if hasattr(self, "baseline_attitude_labels"):
                key = "left_rpy" if side == "left" else "right_rpy"
                self.baseline_attitude_labels[key].setText(rpy_text)
        left = self.latest_extrinsics.get("left")
        right = self.latest_extrinsics.get("right")
        yaw_text = "대기 중"
        pitch_text = "대기 중"
        roll_text = "대기 중"
        if left is not None and right is not None:
            yaw_text = f"{angle_diff_deg(left.yaw_deg, right.yaw_deg):+.3f} deg"
            pitch_text = f"{right.pitch_deg - left.pitch_deg:+.3f} deg"
            roll_text = f"{right.roll_deg - left.roll_deg:+.3f} deg"
        if hasattr(self, "alignment_attitude_labels"):
            self.alignment_attitude_labels["yaw_diff"].setText(yaw_text)
            self.alignment_attitude_labels["pitch_diff"].setText(pitch_text)
            self.alignment_attitude_labels["roll_diff"].setText(roll_text)
        if hasattr(self, "baseline_attitude_labels"):
            self.baseline_attitude_labels["yaw_diff"].setText(yaw_text)
        self._refresh_rtk_alignment_view()

    def _refresh_coordinate_system_settings(self) -> None:
        if not hasattr(self, "alignment_global_frame_combo"):
            return
        self.coordinate_system_manager.update(
            global_frame=self.alignment_global_frame_combo.currentText(),
            camera_frame=self.alignment_camera_frame_combo.currentText(),
            origin=self.alignment_origin_combo.currentText(),
        )
        self.alignment_tf_tree_label.setText(f"TF Tree: {self.coordinate_system_manager.tf_tree_text()}")

    def _refresh_alignment_validation(self) -> None:
        if not hasattr(self, "alignment_validation_labels"):
            return
        stored_baseline = None
        if self.loaded_alignment_profile:
            vector = (
                self.loaded_alignment_profile.get("alignment_profile", {})
                .get("baseline", {})
                .get("vector_m")
            )
            if isinstance(vector, list) and len(vector) == 3:
                stored_baseline = (float(vector[0]), float(vector[1]), float(vector[2]))
        result = self.alignment_validator.validate(
            stored_baseline_enu_m=stored_baseline,
            current_baseline=self.latest_baseline_vector,
            stored_rpy_deg=(0.0, 0.0, 0.0),
            current_rpy_delta_deg=self.attitude_monitor.deltas(),
            distance_m=self.alignment_validation_distance_spin.value() if hasattr(self, "alignment_validation_distance_spin") else 1852.0,
        )
        labels = self.alignment_validation_labels
        labels["baseline_length"].setText("--" if result.baseline_length_difference_m is None else f"{result.baseline_length_difference_m:.4f} m")
        labels["baseline_vector"].setText("--" if result.baseline_vector_difference_m is None else f"{result.baseline_vector_difference_m:.4f} m")
        labels["yaw"].setText("--" if result.yaw_difference_deg is None else f"{result.yaw_difference_deg:.3f} deg")
        labels["pitch"].setText("--" if result.pitch_difference_deg is None else f"{result.pitch_difference_deg:.3f} deg")
        labels["roll"].setText("--" if result.roll_difference_deg is None else f"{result.roll_difference_deg:.3f} deg")
        labels["baseline_error"].setText("--" if result.baseline_error_contribution_m is None else f"{result.baseline_error_contribution_m:.3f} m")
        labels["yaw_error"].setText("--" if result.yaw_error_contribution_m is None else f"{result.yaw_error_contribution_m:.3f} m")
        labels["pitch_error"].setText("--" if result.pitch_error_contribution_m is None else f"{result.pitch_error_contribution_m:.3f} m")
        self._set_health_label(labels["status"], result.status, "good" if result.status == "PASS" else "bad")

    def _baseline_message_rate(self, side: str) -> float:
        history = self.baseline_gps_rate_history.get(side)
        if not history or len(history) < 2:
            return 0.0
        duration = max(history[-1] - history[0], 1e-6)
        return (len(history) - 1) / duration

    def start_baseline_sampling(self) -> None:
        self.baseline_sampling_active = True
        self.baseline_sampling_started_s = time.monotonic()
        self.baseline_samples.clear()
        self.physical_alignment_samples.clear()
        self.latest_baseline_quality = None
        self.latest_physical_alignment_quality = None
        if hasattr(self, "baseline_length_plot"):
            self.baseline_length_plot.points.clear()
            self.baseline_length_plot.update()
        self.baseline_quality_badge.setText("Quality: sampling")
        self._set_health_label(self.baseline_quality_badge, "Quality: sampling", "warn")

    def stop_baseline_sampling(self) -> None:
        self.baseline_sampling_active = False
        self._finalize_baseline_quality()

    def _append_baseline_sample_if_active(self) -> None:
        if not getattr(self, "baseline_sampling_active", False):
            return
        current = self._current_baseline_vector()
        if current is None:
            return
        self.baseline_samples.append(current)
        vector, left, right = current
        left_attitude = self.latest_extrinsics.get("left")
        right_attitude = self.latest_extrinsics.get("right")
        if left_attitude is not None and right_attitude is not None:
            try:
                self.physical_alignment_samples.append(
                    make_physical_alignment_sample(
                        timestamp_s=time.monotonic(),
                        baseline_ned_m=vector.ned_m,
                        p1_body_to_world_q=left_attitude.q,
                        p2_body_to_world_q=right_attitude.q,
                        left_fix_type=left.fix_type,
                        right_fix_type=right.fix_type,
                    )
                )
            except (TypeError, ValueError):
                # Keep raw baseline quality usable even when one attitude
                # sample is malformed or temporarily unavailable.
                pass
        if hasattr(self, "baseline_length_plot"):
            self.baseline_length_plot.add_point(TelemetryHistoryPoint(time.monotonic(), "length", vector.length_m))
        duration = self.baseline_sample_duration_spin.value() if hasattr(self, "baseline_sample_duration_spin") else 10.0
        started = self.baseline_sampling_started_s or time.monotonic()
        if time.monotonic() - started >= duration:
            self.stop_baseline_sampling()

    def _finalize_baseline_quality(self) -> None:
        duration = 0.0
        if self.baseline_sampling_started_s is not None:
            duration = max(0.0, time.monotonic() - self.baseline_sampling_started_s)
        quality = evaluate_baseline_quality(self.baseline_samples, duration_s=duration)
        self.latest_baseline_quality = quality
        self._render_baseline_quality(quality)
        minimum_fix_type = 3
        minimum_fixed_ratio = 0.0
        try:
            readiness = self._build_gnss_operation_config().readiness
            minimum_fix_type = readiness.minimum_fix_type
        except (AttributeError, TypeError, ValueError):
            pass
        physical_quality = evaluate_physical_alignment(
            self.physical_alignment_samples,
            target_baseline_frd_m=self._physical_alignment_reference_baseline_frd_m(),
            minimum_fix_type=minimum_fix_type,
            minimum_fixed_ratio=minimum_fixed_ratio,
        )
        self.latest_physical_alignment_quality = physical_quality
        if physical_quality is not None:
            self.alignment_iteration_number += 1
            self.alignment_iteration_history.append(
                {
                    "iteration": self.alignment_iteration_number,
                    "timestamp_s": time.monotonic(),
                    **physical_quality.to_mapping(),
                }
            )
            self.alignment_iteration_history = self.alignment_iteration_history[-20:]

    def _physical_alignment_reference_baseline_frd_m(self) -> tuple[float, float, float] | None:
        """Convert a loaded ENU profile reference into P1 body FRD explicitly."""

        profile = getattr(self, "loaded_alignment_profile", None)
        if not isinstance(profile, dict):
            return None
        alignment = profile.get("alignment_profile")
        if not isinstance(alignment, dict):
            return None
        baseline = alignment.get("baseline")
        attitude = alignment.get("left_attitude")
        if not isinstance(baseline, dict) or not isinstance(attitude, dict):
            return None
        if str(baseline.get("frame", "")).upper() != "ENU":
            return None
        vector = baseline.get("vector_m")
        quaternion = attitude.get("quaternion_wxyz")
        if not isinstance(vector, (tuple, list)) or len(vector) != 3:
            return None
        if not isinstance(quaternion, (tuple, list)) or len(quaternion) != 4:
            return None
        try:
            # The persisted alignment profile is explicitly ENU.  Convert it
            # to local NED before applying the P1 body-FRD inverse attitude.
            east, north, up = (float(value) for value in vector)
            return world_ned_vector_to_body_frd(
                (north, east, -up),
                tuple(float(value) for value in quaternion),
            )
        except (TypeError, ValueError):
            return None

    def workstation_physical_alignment_snapshot(self) -> dict[str, object]:
        """Return Workstation alignment evidence without applying corrections."""

        selected_left_gps, selected_right_gps = self._selected_baseline_gps_states()
        baseline = self.latest_rtk_baseline
        projection = self.latest_p1_body_frd_baseline
        left = self.latest_extrinsics.get("left")
        right = self.latest_extrinsics.get("right")
        relative_rpy = None
        if left is not None and right is not None:
            try:
                relative = relative_attitude(left.q, right.q)
                relative_rpy = (relative.roll_deg, relative.pitch_deg, relative.yaw_deg)
            except (TypeError, ValueError):
                relative_rpy = None
        quality = self.latest_physical_alignment_quality
        if self.baseline_sampling_active:
            state = "SAMPLING"
        elif quality is not None:
            state = quality.state
        elif relative_rpy is not None and baseline is not None:
            state = "LIVE"
        else:
            state = "WAITING"
        return {
            "state": state,
            "sampling_active": bool(self.baseline_sampling_active),
            "sample_count": len(self.physical_alignment_samples),
            "current_baseline_ned_m": (
                None
                if baseline is None
                else (baseline.north_m, baseline.east_m, baseline.down_m)
            ),
            "current_baseline_frd_m": (
                None
                if projection is None
                else (projection.forward_m, projection.right_m, projection.down_m)
            ),
            "relative_rpy_deg": relative_rpy,
            "fix_type": {
                "left": getattr(selected_left_gps, "fix_type", None),
                "right": getattr(selected_right_gps, "fix_type", None),
            },
            "quality": {} if quality is None else quality.to_mapping(),
            "reference_baseline_frd_m": self._physical_alignment_reference_baseline_frd_m(),
            "iterations": tuple(self.alignment_iteration_history[-20:]),
            "time_sync_checkpoints": tuple(self.workstation_time_sync_checkpoints[-20:]),
            "physical_adjustment_only": True,
            "coordinate_contract": (
                "raw GPS/RTK NED [north,east,down] preserved; P1→P2 FRD "
                "[forward,right,down] is an attitude-derived observation; "
                "relative attitude is P2 relative P1 [roll,pitch,yaw]"
            ),
        }

    def _render_baseline_quality(self, quality: BaselineQuality | None) -> None:
        if not hasattr(self, "baseline_quality_labels"):
            return
        if quality is None:
            self._set_health_label(self.baseline_quality_badge, "Quality: 샘플 대기", "idle")
            return
        level = "good" if quality.grade == "GOOD" else "warn" if quality.grade == "WARNING" else "bad"
        self._set_health_label(self.baseline_quality_badge, f"Quality: {quality.grade}", level)
        self.baseline_quality_labels["length_mean"].setText(f"{quality.baseline_length_mean_m:.4f} m")
        self.baseline_quality_labels["length_std"].setText(f"{quality.baseline_length_std_m:.4f} m")
        self.baseline_quality_labels["enu_mean"].setText(
            f"[{quality.enu_mean_m[0]:+.4f}, {quality.enu_mean_m[1]:+.4f}, {quality.enu_mean_m[2]:+.4f}] m"
        )
        self.baseline_quality_labels["enu_std"].setText(
            f"[{quality.enu_std_m[0]:.4f}, {quality.enu_std_m[1]:.4f}, {quality.enu_std_m[2]:.4f}] m"
        )
        self.baseline_quality_labels["rtk_ratio"].setText(f"{quality.rtk_fixed_ratio * 100.0:.1f}%")
        self.baseline_quality_labels["message_rate"].setText(f"{quality.gps_message_rate_hz:.2f} Hz")
        self.baseline_quality_labels["timestamp_delta"].setText(f"{quality.timestamp_delta_ms:.1f} ms")
        self.baseline_quality_labels["outliers"].setText(str(quality.outlier_count))
        self.baseline_quality_labels["jumps"].setText(str(quality.baseline_jump_count))
        h_acc = "--" if quality.mean_h_acc_m is None else f"{quality.mean_h_acc_m:.3f} m"
        v_acc = "--" if quality.mean_v_acc_m is None else f"{quality.mean_v_acc_m:.3f} m"
        self.baseline_quality_labels["acc"].setText(f"{h_acc} / {v_acc}")

    def save_alignment_profile(self) -> None:
        selected, _filter = QtWidgets.QFileDialog.getSaveFileName(
            self.widget,
            "Save Alignment Profile",
            str(self.app_root / "alignment_profile.yaml"),
            "YAML (*.yaml *.yml)",
        )
        if not selected:
            return
        payload = self.alignment_profile_manager.build_payload(
            baseline=self.latest_baseline_vector,
            baseline_quality=self.latest_baseline_quality,
            left_attitude=self.latest_extrinsics.get("left"),
            right_attitude=self.latest_extrinsics.get("right"),
            coordinate_settings=self.coordinate_system_manager.settings,
            gps_mapping={
                "left_camera": str(self.baseline_left_gps_combo.currentData() or "GPS1") if hasattr(self, "baseline_left_gps_combo") else "GPS1",
                "right_camera": str(self.baseline_right_gps_combo.currentData() or "GPS1") if hasattr(self, "baseline_right_gps_combo") else "GPS1",
            },
            magnetometer_mapping={
                "left_can1": "CAN1",
                "left_can2": "CAN2",
                "right_can1": "CAN1",
                "right_can2": "CAN2",
            },
            physical_alignment=self.workstation_physical_alignment_snapshot(),
        )
        text = self.alignment_profile_manager.dump(payload)
        path = Path(selected)
        path.write_text(text, encoding="utf-8")
        self.alignment_profile_preview.setPlainText(text)
        self.alignment_profile_status_label.setText(f"Alignment Profile 저장: {path}")

    def load_alignment_profile(self) -> None:
        selected, _filter = QtWidgets.QFileDialog.getOpenFileName(
            self.widget,
            "Load Alignment Profile",
            str(self.app_root),
            "YAML (*.yaml *.yml)",
        )
        if not selected:
            return
        path = Path(selected)
        try:
            payload = self.alignment_profile_manager.load(path.read_text(encoding="utf-8"))
        except Exception as exc:
            QtWidgets.QMessageBox.warning(self.widget, "Alignment Profile", str(exc))
            return
        self.loaded_alignment_profile = payload
        self.alignment_profile_preview.setPlainText(path.read_text(encoding="utf-8"))
        self.alignment_profile_status_label.setText(f"Alignment Profile 불러옴: {path}")
        self._refresh_alignment_validation()

