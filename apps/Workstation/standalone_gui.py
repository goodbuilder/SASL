from __future__ import annotations

import argparse
import csv
import hashlib
import html as html_lib
import faulthandler
import json
import math
import os
import queue
import random
import re
import signal
import struct
import sys
import threading
import time
import uuid
from collections.abc import Sequence
from collections import deque
from dataclasses import dataclass
from datetime import datetime, timezone
from decimal import Decimal, InvalidOperation, ROUND_HALF_UP
from ipaddress import IPv4Address
from math import isfinite
from pathlib import Path
from statistics import median
from typing import TYPE_CHECKING, Any
from urllib.parse import urlparse

from PyQt5 import QtCore, QtGui, QtWidgets

from Workstation.controller_delegation import ControllerDelegateBinding, bind_controller_delegates

from Workstation.workstation_core.calibration import (
    CheckerboardConfig,
    StereoExtrinsicsProfile,
    StereoIntrinsicsResult,
    calibrate_camera_from_images,
    calibrate_stereo_extrinsics_from_image_pairs,
    detect_checkerboard,
    opencv_available,
    load_intrinsics_json,
    save_intrinsics_json,
    validate_intrinsics_result,
    without_stereo_extrinsics,
)

from Workstation.workstation_core.attitude_geometry import (
    quaternion_conjugate,
    quaternion_multiply,
    relative_attitude,
    relative_rotation_angle_deg,
    world_ned_vector_to_body_frd,
)
from Workstation.workstation_core.camera_pane import CameraPane
from Workstation.workstation_core.coordinate_contract import (
    BODY_FRAME,
    CAMERA_FRAME,
    CAMERA_MOUNT_PROFILE,
    LEGACY_XYZ_ALIAS_CONTRACT,
    MEASUREMENT_AXIS_CONTRACT,
    MEASUREMENT_FRAME,
    PUBLIC_MEASUREMENT_VECTOR_CONTRACT,
    WORLD_FRAME,
)
from Workstation.workstation_core.measurement_widgets import (
    ClockOffsetHistoryPlot,
    ClockOffsetPoint,
    DistanceHistoryPlot,
    DistanceHistoryPoint,
    Triangulation3DView,
    triangulation_3d_scene_points,
)
from Workstation.workstation_core.measurement_attempt import (
    AttitudeSnapshot,
    DetectionSnapshot,
    GeometryConfigSnapshot,
    GpsSnapshot,
    MeasurementAttempt,
    StereoIntrinsicsSnapshot,
    build_checkerboard_baseline_snapshot,
    build_pixhawk_baseline_snapshot,
    detection_pair_key,
)
from Workstation.workstation_core.measurement_methods import MEASUREMENT_METHOD_LABELS
from Workstation.workstation_core.measurement_controller import calculate_measurement_geometry
from Workstation.workstation_core.fixed_base_initializer import FixedBaseInitializer
from Workstation.workstation_core.gnss_network import NtripClient, format_nmea_gga, gga_from_position, gga_from_coordinate_sample
from Workstation.workstation_core.gnss_operation import (
    CorrectionSourceIdentity,
    evaluate_gnss_readiness,
    GnssReadinessInputs,
    GnssOperationConfig,
    GnssOperationManager,
    GnssOperationMode,
    GnssReadinessSnapshot,
    GgaPolicy,
    GnssStatusGroup,
    HeightType,
    NetworkConfig,
    RoverCorrectionStatus,
    RoverReadinessInput,
    STATE_STATUS_GROUP,
)
from Workstation.workstation_core.gnss_profile import (
    GnssCoordinateSample,
    geodetic_to_ecef_m,
)
from Workstation.workstation_core.network import (
    DEFAULT_CAMERA_URLS,
    DEFAULT_HOST_ADDRESS,
    DEFAULT_INTERFACE,
    apply_interface_address,
    check_tcp_port,
    discover_host_interface_candidates,
    discover_rtsp_candidates,
    discover_tcp_port_candidates,
    discover_tcp_port_range_candidates,
    interface_addresses,
    interface_has_address,
    interface_link_exists,
    local_ipv4_interfaces,
    nm_connection_ipv4_addresses,
    nm_device_connection_name,
    resolve_nm_interface_address,
    rtsp_host_port,
    select_host_for_camera,
    tcp_port_scan_sequence,
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
from Workstation.workstation_core.alignment_managers import (
    AlignmentProfileManager,
    AlignmentValidator,
    AttitudeMonitor,
    BaselineManager,
    CoordinateSystemManager,
    GNSSMonitor,
    MagnetometerMonitor,
)
from Workstation.workstation_core.pixhawk_attitude import (
    CameraExtrinsicState,
    CustomTopicSample,
    DEFAULT_USB_MAVLINK_BAUDRATES,
    DualPixhawkMavlinkSubscriber,
    MavlinkShellCommandResult,
    camera_label_ko,
    discover_mavlink_messages,
    mavlink_graphable_fields_for_message,
    magnetometer_value_units,
    parse_px4_sensor_mag_shell_output,
    probe_usb_mavlink_devices,
    usb_mavlink_device_candidates,
)
from Workstation.workstation_core.px4_mag_manager import (
    BOARD_INTERNAL_MAG_ID,
    GPS1_MAG_ID,
    GPS2_MAG_ID,
    MAG_PARAM_NAMES,
    MAG_PRIORITY_LEVELS,
    MAG_SLOTS,
    MockParameterStore,
    Px4MagManager,
    RM3100_BACKUP_ID,
    RM3100_BACKUP_IDS,
    RM3100_IDS,
    RM3100_PRIMARY_ID,
    RM3100_PRIMARY_IDS,
    build_mag_status,
    current_pixhawk_6x_mag_params,
    identify_mag_device,
    mag_priority_display,
)
from Workstation.workstation_core.px4_attitude_cal_manager import (
    ATTITUDE_CAL_PARAM_NAMES,
    Px4AttitudeCalManager,
    Px4AttitudeCalStatus,
    build_attitude_cal_status,
    current_pixhawk_6x_attitude_cal_params,
)
from Workstation.workstation_core.rtk_geometry import (
    BodyFrdBaselineProjection,
    GpsPositionState,
    RtkBaselineEstimate,
    estimate_rtk_baseline,
    ned_delta_m,
    project_ned_baseline_to_p1_body_frd,
)
from Workstation.workstation_core.recording_session import (
    SavedSessionFiles,
    SpoolRows,
    compact_distance_measurement_rows,
    format_video_paths,
    raw_video_paths,
    resolve_session_dir,
    save_session_files,
    session_id_now,
    session_sheet_previews,
    write_recovery_metadata,
)
from Workstation.workstation_core.settings import (
    DEFAULT_INTRINSICS_PATH,
    AppSettings,
    load_settings as _load_shared_settings,
    resolve_settings_path,
    save_settings as _save_shared_settings,
)
from Workstation.workstation_core.stereo import (
    DetectionObservation,
    correspondence_rejection_reason,
    detection_timestamp_for_basis,
)
from Workstation.workstation_core.theme import (
    ACCENT,
    ACCENT_ACTIVE,
    ACCENT_HOVER,
    APP_BG,
    CHART_AXIS,
    CHART_BG,
    CHART_GRID,
    CHART_MUTED,
    CHART_TEXT,
    ERROR,
    ERROR_BG,
    ERROR_TEXT,
    FONT_FAMILY,
    INFO_BG,
    INVERSE_BG,
    INVERSE_PANEL_BG,
    INVERSE_TEXT,
    INVERSE_TEXT_MUTED,
    PANEL_ALT_BG,
    PANEL_ALT_STRONG,
    PANEL_BG,
    PANEL_BORDER,
    PANEL_BORDER_STRONG,
    SERIES_DISTANCE,
    SERIES_HIGHLIGHT,
    SERIES_LEFT,
    SERIES_RIGHT,
    SERIES_Z,
    SUCCESS,
    SUCCESS_BG,
    SUCCESS_TEXT,
    TEXT_MAIN,
    TEXT_MUTED,
    TEXT_SUBTLE,
    WARNING,
    WARNING_BG,
    WARNING_TEXT,
)
from Workstation.workstation_core.sync import (
    ATTITUDE_STALE_AGE_S,
    ATTITUDE_STALE_PAIR_DELTA_S,
    ATTITUDE_TIMING_POLICY,
    ATTITUDE_VERY_STALE_AGE_S,
    ATTITUDE_VERY_STALE_PAIR_DELTA_S,
    TimedSampleBuffer,
    classify_attitude_timing,
    match_nearest_pair,
)
from Workstation.workstation_core.triangulation import (
    RayTriangulationResult,
    body_frd_vector_to_camera,
    camera_translation_right_in_left_camera,
    invert_left_to_right_camera_transform,
)
from Workstation.workstation_core.yolo_stream import (
    DECODER_AUTO,
    DECODER_NVIDIA,
    DECODER_SOFTWARE,
    DEFAULT_PREVIEW_DISPLAY_RATE_HZ,
    DEFAULT_RTSP_LATENCY_MS,
    DEFAULT_YOLO_WEIGHTS,
    build_gst_pipeline_text,
    decoder_candidates,
    STREAM_BACKEND_GSTREAMER,
    STREAM_BACKEND_OPENCV,
    STREAM_BACKEND_OPENCV_AUTO,
    STREAM_BACKEND_PYAV,
    RTSP_TRANSPORT_TCP,
    RTSP_TRANSPORT_UDP,
    normalize_decoder_mode,
    normalize_rtsp_transport,
    normalize_stream_backend,
)
from sasl_common.lifecycle import LifecycleCoordinator, LifecycleStep
from Workstation.evidence import mavlink_link_provenance
from Workstation.runtime_paths import (
    WORKSTATION_DATA_ROOT,
    WORKSTATION_SETTINGS_PATH,
    resolve_workstation_data_path,
)


# Keep the copied owner API stable while routing every relative calibration,
# clock, alignment, recording, and settings path into Workstation-owned state.
APP_ROOT = WORKSTATION_DATA_ROOT


def load_settings(path: str | Path | None = None) -> AppSettings:
    return _load_shared_settings(WORKSTATION_SETTINGS_PATH if path is None else path)


def save_settings(settings: AppSettings, path: str | Path | None = None) -> None:
    _save_shared_settings(settings, WORKSTATION_SETTINGS_PATH if path is None else path)

from Workstation.workstation_core.wiris_zoom import (
    DEFAULT_ZOOM_CONTROL_PORT,
    DEFAULT_ZOOM_IN_TEMPLATE,
    DEFAULT_ZOOM_OUT_TEMPLATE,
    DEFAULT_ZOOM_SET_TEMPLATE,
    DEFAULT_ZOOM_STOP_TEMPLATE,
    GET_ETHERNET_MODE_COMMAND,
    LINE_TERMINATOR,
    MAX_ZOOM_LEVEL,
    MIN_ZOOM_LEVEL,
    NUL_TERMINATOR,
    REBOOT_COMMAND,
    SET_ETHERNET_MODE_COMMAND,
    SET_IP_ADDRESS_TEMPLATE,
    SET_SUBNET_MASK_TEMPLATE,
    VISIBLE_ZOOM_FX_ESTIMATES,
    build_wiris_command,
    build_zoom_command,
    send_tcp_line,
)
from Workstation.wiris_sdk_catalog import (
    WIRIS_COMMAND_CATALOG,
    WIRIS_DANGEROUS_COMMAND_PREFIXES,
)


def angle_diff_deg(a_deg: float, b_deg: float) -> float:
    """Return wrapped angular difference a-b in [-180, 180) degrees."""
    return (float(a_deg) - float(b_deg) + 180.0) % 360.0 - 180.0


from Workstation.workstation_core.mavlink_parameter_store import (
    MavlinkAttitudeParameterStore,
    MavlinkParameterStore,
)


FORM_LABEL_MIN_WIDTH = 138
FORM_BUTTON_MIN_WIDTH = 112
NETWORK_FIELD_MIN_WIDTHS = {
    "interface": 120,
    "host": 140,
    "port": 88,
    "timeout": 104,
    "mask": 140,
}
APP_WINDOW_TITLE = "StereoWIRIS"
STARTUP_CAMERA_STATUS = "카메라: 대기 중 - 장비 연결 후 영상 수신 확인 또는 카메라 Preview 연결"
STARTUP_PIXHAWK_STATUS = "픽스호크: 대기 중 - 장비 연결 후 MAVLink 연결"
STARTUP_READY_MESSAGE = (
    "[연결] 카메라/픽스호크 미연결 상태에서도 앱을 사용할 수 있습니다. "
    "장비 연결 후 영상 수신 확인, 카메라 Preview 연결 또는 MAVLink 연결을 누르세요."
)
YOLO_CPU_DEVICE = "cpu"
YOLO_CUDA_DEVICE = "cuda:0"
WIRIS_VISIBLE_ZOOM_FX_ESTIMATES = VISIBLE_ZOOM_FX_ESTIMATES
MAX_MANUAL_FOCAL_PX = 100_000.0
DETECTION_PAIR_MAX_AGE_S = 1.0
RUNTIME_CAMERA_STATUS_LABEL_HEIGHT_PX = 32
RUNTIME_CAMERA_STATUS_GROUP_HEIGHT_PX = 48
RUNTIME_CAMERA_LAYOUT_BREAKPOINT_PX = 1060
# Switching runtime_header_layout's direction changes its own preferred
# height/width, which immediately changes how much of runtime_layout's
# remaining stretch space the camera panes get. A single breakpoint flips
# back and forth every resize pass near RUNTIME_CAMERA_LAYOUT_BREAKPOINT_PX,
# visibly jittering the camera preview size. This margin makes the two
# transition points distinct so the direction only changes once per crossing.
RUNTIME_CAMERA_LAYOUT_HYSTERESIS_PX = 48
VISIBLE_STREAM_GOP_GET_COMMAND = "GEGS VISIBLE"
RECORDING_PREVIEW_LIMIT = 256
RECORDING_KEY_CACHE_LIMIT = 1024
# WIRIS cameras expose the SDK and RTSP services a short time after power-up.
# Keep auto-connect responsive while allowing that normal boot window to settle.
CAMERA_AUTO_CONNECT_MAX_ATTEMPTS = 6
CAMERA_AUTO_CONNECT_RETRY_DELAY_MS = 1_500


def _format_visible_zoom_fx_estimates() -> str:
    rows = [
        "WIRIS OZ 1280x720, HFOV 63.7deg 기준 명목 추정값입니다.",
        "정확한 측정은 해당 zoom index에서 체커보드 보정 후 적용하세요.",
        "",
        "index / ratio / estimated fx",
    ]
    rows.extend(
        f"{index:>2} / {ratio:>4.1f}x / {fx_px:>5,d} px"
        for index, ratio, fx_px in WIRIS_VISIBLE_ZOOM_FX_ESTIMATES
    )
    return "\n".join(rows)


WIRIS_VISIBLE_ZOOM_FX_TOOLTIP = _format_visible_zoom_fx_estimates()

from Workstation.workstation_core.runtime_camera_status import (
    camera_display_name,
    camera_resolution_text,
    format_camera_stream_stats,
    format_runtime_camera_ntp_compact,
    format_runtime_camera_stats,
    format_runtime_camera_status,
    format_runtime_camera_timestamp,
    format_runtime_camera_timestamp_compact,
    format_runtime_yolo_device_text,
    format_signed_ms_compact,
    format_unix_timestamp_compact,
    infer_runtime_camera_timestamp_reference,
)


from Workstation.telemetry_plots import (
    AttitudeHistoryPlot,
    AttitudeHistoryPoint,
    Px4ShellDialog,
    RtkAlignmentPlot,
    TelemetryHistoryPlot,
    TelemetryHistoryPoint,
)


def _set_px4_priority_combo_value(combo: QtWidgets.QComboBox, value: int | float | None) -> None:
    if value is None:
        return
    raw = int(value)
    index = combo.findData(raw)
    if index < 0:
        combo.addItem(f"Custom ({raw})", raw)
        index = combo.count() - 1
    combo.setCurrentIndex(index)
    combo.setToolTip(f"PX4 CAL_MAG*_PRIO: {mag_priority_display(raw)}")


def _build_px4_priority_combo(value: int) -> QtWidgets.QComboBox:
    combo = QtWidgets.QComboBox()
    for raw, label in MAG_PRIORITY_LEVELS:
        combo.addItem(label, raw)
    _set_px4_priority_combo_value(combo, value)
    combo.currentIndexChanged.connect(
        lambda _index, combo=combo: combo.setToolTip(
            f"PX4 CAL_MAG*_PRIO: {mag_priority_display(combo.currentData())}"
        )
    )
    return combo


def _px4_priority_combo_value(combo: QtWidgets.QComboBox) -> int:
    value = combo.currentData()
    return 0 if value is None else int(value)


def _control_candidate_value(candidate: object, key: str, default: object | None = None) -> object | None:
    if isinstance(candidate, dict):
        return candidate.get(key, default)
    return getattr(candidate, key, default)


def _preferred_wiris_control_ports(control_port: int | None) -> tuple[int, ...]:
    ports: list[int] = []
    for port in (control_port, DEFAULT_ZOOM_CONTROL_PORT):
        try:
            normalized = int(port or DEFAULT_ZOOM_CONTROL_PORT)
        except (TypeError, ValueError):
            continue
        if 1 <= normalized <= 65535 and normalized not in ports:
            ports.append(normalized)
    return tuple(ports or (DEFAULT_ZOOM_CONTROL_PORT,))


def _full_wiris_control_port_sequence(control_port: int | None) -> tuple[int, ...]:
    return tcp_port_scan_sequence(*_preferred_wiris_control_ports(control_port))


def _merge_tcp_control_candidates(*groups: list[object]) -> list[object]:
    merged: list[object] = []
    seen: set[tuple[str, int]] = set()
    for group in groups:
        for candidate in group:
            host = str(_control_candidate_value(candidate, "host", "") or "")
            try:
                port = int(_control_candidate_value(candidate, "port", 0) or 0)
            except (TypeError, ValueError):
                continue
            key = (host, port)
            if not host or key in seen:
                continue
            seen.add(key)
            merged.append(candidate)
    return merged


def _verify_wiris_control_candidates(candidates: list[object]) -> list[dict[str, object]]:
    verified: list[dict[str, object]] = []
    for candidate in candidates:
        host = str(_control_candidate_value(candidate, "host", "") or "")
        try:
            port = int(_control_candidate_value(candidate, "port", 0) or 0)
        except (TypeError, ValueError):
            continue
        interface = _control_candidate_value(candidate, "interface")
        try:
            command = build_wiris_command(
                host=host,
                port=port,
                command="HIWS",
                timeout_s=0.8,
                terminator=NUL_TERMINATOR,
            )
            response = send_tcp_line(command)
        except Exception as exc:
            verified.append(
                {
                    "host": host,
                    "port": port,
                    "interface": interface,
                    "ok": False,
                    "response": str(exc),
                }
            )
            continue
        verified.append(
            {
                "host": host,
                "port": port,
                "interface": interface,
                "ok": response.strip().upper() == "OK",
                "response": response,
            }
        )
    return verified


from Workstation.session_workers import SessionSaveWorker


class CameraAutoDetectWorker(QtCore.QObject):
    detected = QtCore.pyqtSignal(object)
    failed = QtCore.pyqtSignal(str)
    finished = QtCore.pyqtSignal()

    def __init__(
        self,
        *,
        mode: str,
        control_port: int | None = None,
        host_hints: tuple[str, ...] = (),
        target_side: str | None = None,
        full_control_scan: bool = True,
    ) -> None:
        super().__init__()
        self.mode = mode
        self.control_port = control_port
        self.host_hints = tuple(host_hints)
        self.target_side = target_side
        self.full_control_scan = bool(full_control_scan)

    def _discover_wiris_control(self, arp_scan_reports: list[object]) -> tuple[list[object], list[dict[str, object]]]:
        preferred_ports = _preferred_wiris_control_ports(self.control_port)
        preferred_candidates = discover_tcp_port_range_candidates(
            preferred_ports,
            timeout=0.2,
            max_hosts_per_network=0,
            fallback_scan_if_empty=False,
            stop_after=None,
            arp_scan_reports=arp_scan_reports,
            host_hints=self.host_hints,
        )
        preferred_verified = _verify_wiris_control_candidates(preferred_candidates)
        if len([candidate for candidate in preferred_verified if candidate.get("ok")]) >= 2:
            return list(preferred_candidates), preferred_verified

        full_scan_candidates: list[object] = []
        if self.full_control_scan:
            full_scan_candidates = discover_tcp_port_range_candidates(
                _full_wiris_control_port_sequence(self.control_port),
                timeout=0.08,
                max_hosts_per_network=0,
                fallback_scan_if_empty=False,
                stop_after=None,
                arp_scan_reports=None,
                host_hints=self.host_hints,
            )

        fallback_candidates: list[object] = []
        if len([candidate for candidate in preferred_verified if candidate.get("ok")]) < 2:
            for port in preferred_ports:
                fallback_candidates = _merge_tcp_control_candidates(
                    fallback_candidates,
                    discover_tcp_port_candidates(
                        int(port),
                        timeout=0.2,
                        max_hosts_per_network=0,
                        fallback_scan_if_empty=True,
                        stop_after=None,
                        arp_scan_reports=None,
                        host_hints=self.host_hints,
                    ),
                )

        control_candidates = _merge_tcp_control_candidates(
            list(preferred_candidates),
            list(full_scan_candidates),
            fallback_candidates,
        )
        return control_candidates, _verify_wiris_control_candidates(control_candidates)

    @QtCore.pyqtSlot()
    def run(self) -> None:
        try:
            arp_scan_reports = []
            host_interfaces = discover_host_interface_candidates()
            if self.mode == "auto_connect":
                control, verified = self._discover_wiris_control(arp_scan_reports)
                result = {
                    "mode": "auto_connect",
                    "full_control_scan": self.full_control_scan,
                    "host_interfaces": host_interfaces,
                    "control": control,
                    "verified_control": verified,
                    "arp_scan": arp_scan_reports,
                }
            elif self.mode == "control":
                control, verified = self._discover_wiris_control(arp_scan_reports)
                result = {
                    "mode": "control",
                    "full_control_scan": self.full_control_scan,
                    "target_side": self.target_side,
                    "host_interfaces": host_interfaces,
                    "control": control,
                    "verified_control": verified,
                    "arp_scan": arp_scan_reports,
                }
            else:
                result = {
                    "mode": "rtsp",
                    "host_interfaces": host_interfaces,
                    "rtsp": discover_rtsp_candidates(
                        timeout=0.2,
                        max_hosts_per_network=0,
                        fallback_scan_if_empty=True,
                        stop_after=None,
                        arp_scan_reports=arp_scan_reports,
                        host_hints=self.host_hints,
                    ),
                    "arp_scan": arp_scan_reports,
                }
        except Exception as exc:
            self.failed.emit(str(exc))
        else:
            self.detected.emit(result)
        finally:
            self.finished.emit()


class WirisControlCheckWorker(QtCore.QObject):
    """Probe WIRIS control ports without blocking the Qt event loop."""

    checked = QtCore.pyqtSignal(object)
    failed = QtCore.pyqtSignal(str)
    finished = QtCore.pyqtSignal()

    def __init__(self, targets: Sequence[tuple[str, str, int]], *, timeout_s: float) -> None:
        super().__init__()
        self.targets = tuple((str(label), str(host), int(port)) for label, host, port in targets)
        self.timeout_s = max(0.1, float(timeout_s))

    @QtCore.pyqtSlot()
    def run(self) -> None:
        results: list[dict[str, object]] = []
        try:
            for label, host, port in self.targets:
                if not host.strip():
                    results.append({"label": label, "host": host, "port": port, "state": "missing-host"})
                    continue
                try:
                    is_open = bool(check_tcp_port(host.strip(), port, timeout=self.timeout_s))
                except Exception as exc:
                    results.append(
                        {
                            "label": label,
                            "host": host,
                            "port": port,
                            "state": "error",
                            "error": str(exc),
                        }
                    )
                else:
                    results.append(
                        {
                            "label": label,
                            "host": host,
                            "port": port,
                            "state": "control-open" if is_open else "control-closed",
                        }
                    )
        except Exception as exc:
            self.failed.emit(str(exc))
        else:
            self.checked.emit(results)
        finally:
            self.finished.emit()






from Workstation.device_runtime_workers import (
    PixhawkUsbDiscoveryWorker,
    CameraNetworkDiscoveryWorker,
    CameraNetworkAddressApplyWorker,
    InterfaceStatusWorker,
    PixhawkHostIpDiscoveryWorker,
    PixhawkConnectionWorker,
    PixhawkDisconnectWorker,
    PixhawkStreamSetupWorker,
    PixhawkUsbBootstrapWorker,
    PixhawkPersistentConfigWorker,
    PixhawkRtcmInjectionWorker,
    PixhawkShellCommandWorker,
    PixhawkParameterWorker,
)
from Workstation.device_runtime_controller import (
    DEVICE_RUNTIME_CONTROLLER_METHODS,
    DeviceRuntimeController,
)


from Workstation.wiris_runtime_workers import (
    RTSP_ENABLE_PORT_WAIT_INTERVAL_MS,
    RTSP_ENABLE_PORT_WAIT_MAX_ATTEMPTS,
    RTSP_ENABLE_SETTLE_DELAY_MS,
    WIRIS_STUS_MAX_VALID_RTT_S,
    WIRIS_STUS_PARSE_MARGIN_S,
    WIRIS_STUS_RTT_PROBE_COMMAND,
    WIRIS_STUS_RTT_PROBE_COUNT,
    RtspStreamCheckWorker,
    WirisClockCommandWorker,
    WirisCommandWorker,
    WirisLrfPollWorker,
    WirisRtspEnableWorker,
    WirisStipWorker,
    WirisZoomWorker,
)
from Workstation.wiris_control_controller import (
    WIRIS_CONTROL_METHODS,
    WirisControlController,
)
from Workstation.app_lifecycle_controller import (
    APP_LIFECYCLE_METHODS,
    AppLifecycleController,
)
from Workstation.camera_time_sync_controller import (
    CAMERA_RTCP_MAX_PC_OFFSET_S,
    CAMERA_RTCP_PAIR_MAX_DELTA_S,
    CAMERA_SYNC_FRAME_HISTORY_WINDOW_S,
    CAMERA_SYNC_HISTORY_MAX_AGE_S,
    CAMERA_SYNC_MAX_CLOCK_RATE_ERROR,
    CAMERA_SYNC_MAX_SR_AGE_S,
    CAMERA_SYNC_RECEIVE_PAIR_MAX_DELTA_S,
    CAMERA_SYNC_REQUIRED_SR_COUNT,
    CAMERA_SYNC_STATUS_FAILED,
    CAMERA_SYNC_STATUS_NOT_VERIFIED,
    CAMERA_SYNC_STATUS_RECONNECTING,
    CAMERA_SYNC_STATUS_VERIFIED,
    CAMERA_SYNC_STATUS_WARMING,
    CAMERA_SYNC_WARMUP_TIMEOUT_S,
    CAMERA_TIME_SYNC_METHODS,
    CameraTimeSyncController,
)
from Workstation.camera_stream_controller import (
    CAMERA_STREAM_LIFECYCLE_METHODS,
    CameraStreamLifecycleController,
)
from Workstation.measurement_package_controller import (
    MEASUREMENT_PACKAGE_METHODS,
    MeasurementPackageController,
)


from Workstation.workstation_core.gnss_protocol import (
    GNSS_BASE_LOW_CNO_DBHZ,
    GNSS_BASE_LOW_ELEVATION_DEG,
    GNSS_BASE_SURVEY_IMPROVING_DELTA_M,
    GNSS_BASE_SURVEY_STAGNANT_DELTA_M,
    GNSS_BASE_SURVEY_TREND_WINDOW_S,
    Rtcm3StreamParser,
    UbxStreamParser,
    gnss_base_serial_port_candidates,
    preferred_gnss_base_serial_port,
    parse_nmea_gga_satellite_count,
    parse_rtcm3_message_type,
    parse_ubx_ack,
    parse_ubx_base_satellite_count,
    parse_ubx_cfg_tmode3,
    parse_ubx_nav_pvt_coordinate_sample,
    parse_ubx_nav_sat_quality,
    parse_ubx_nav_svin,
    probe_gnss_base_serial_ports,
    ubx_cfg_tmode3_disabled_frame,
    ubx_cfg_tmode3_fixed_frame,
    ubx_cfg_tmode3_poll_frame,
    ubx_cfg_tmode3_survey_in_frame,
    ubx_frame,
    ubx_nav_pvt_poll_frame,
    ubx_nav_svin_poll_frame,
    ubx_poll_frame,
)

from Workstation.gnss_runtime_workers import (
    GnssBaseSerialWorker,
    NtripNetworkWorker,
    GnssBaseSerialDetectWorker,
    RtcmDataPlaneWorker,
)
from Workstation.gnss_runtime_controller import (
    GNSS_RUNTIME_CONTROLLER_METHODS,
    GnssRuntimeController,
)


from Workstation.calibration_workers import (
    CheckerboardCaptureWorker,
    CheckerboardCalibrationWorker,
    CheckerboardRandomValidationWorker,
    CheckerboardPreviewWorker,
)
from Workstation.calibration_controller import (
    CALIBRATION_WORKFLOW_METHODS,
    CHECKERBOARD_STEREO_PAIR_MAX_DELTA_S,
    CalibrationWorkflowController,
)
from Workstation.recording_controller import (
    RECORDING_CONTROLLER_METHODS,
    RecordingController,
    new_measurement_attempt_generation_id as _new_measurement_attempt_generation_id,
)
from Workstation.run_evidence_controller import (
    RUN_EVIDENCE_CONTROLLER_METHODS,
    RunEvidenceController,
)
from Workstation.optical_pose_controller import (
    OPTICAL_POSE_CONTROLLER_METHODS,
    OpticalPoseController,
)
from Workstation.target_drone_controller import (
    TARGET_DRONE_CONTROLLER_METHODS,
    TargetDroneController,
)
from Workstation.drone_telemetry_controller import (
    DRONE_TELEMETRY_CONTROLLER_METHODS,
    DroneTelemetryController,
    normalize_drone_rtcm_source,
)
from Workstation.measurement_view_controller import (
    MEASUREMENT_VIEW_CONTROLLER_METHODS,
    MeasurementViewController,
)
from Workstation.sensor_presentation_controller import (
    SENSOR_PRESENTATION_METHODS,
    SensorPresentationController,
)
from Workstation.settings_controller import (
    SETTINGS_CONTROLLER_METHODS,
    WorkstationSettingsController,
)


class TopicBrowserDialog(QtWidgets.QDialog):
    def __init__(self, topics: list[dict[str, str]], parent=None, *, title: str = "MAVLink Message Browser") -> None:
        super().__init__(parent)
        self.setWindowTitle(title)
        self.resize(900, 560)
        self._topics = topics
        self._filtered_topics = list(topics)
        layout = QtWidgets.QVBoxLayout(self)
        self.search_edit = QtWidgets.QLineEdit()
        self.search_edit.setPlaceholderText("토픽, 메시지, 타입 검색")
        self.search_edit.textChanged.connect(self._refresh)
        layout.addWidget(self.search_edit)
        self.table = QtWidgets.QTableWidget(0, 2)
        self.table.setHorizontalHeaderLabels(["토픽", "타입"])
        self.table.setSelectionBehavior(QtWidgets.QAbstractItemView.SelectRows)
        self.table.setSelectionMode(QtWidgets.QAbstractItemView.SingleSelection)
        self.table.setEditTriggers(QtWidgets.QAbstractItemView.NoEditTriggers)
        self.table.verticalHeader().setVisible(False)
        header = self.table.horizontalHeader()
        header.setSectionResizeMode(0, QtWidgets.QHeaderView.Stretch)
        header.setSectionResizeMode(1, QtWidgets.QHeaderView.ResizeToContents)
        layout.addWidget(self.table, 1)
        buttons = QtWidgets.QHBoxLayout()
        select_button = QtWidgets.QPushButton("토픽 선택")
        select_button.clicked.connect(self.accept)
        cancel_button = QtWidgets.QPushButton("취소")
        cancel_button.clicked.connect(self.reject)
        buttons.addStretch(1)
        buttons.addWidget(select_button)
        buttons.addWidget(cancel_button)
        layout.addLayout(buttons)
        self.table.itemDoubleClicked.connect(lambda _item: self.accept())
        self._refresh()

    def _refresh(self) -> None:
        query = self.search_edit.text().strip().lower()
        self._filtered_topics = [
            topic
            for topic in self._topics
            if not query or query in topic.get("name", "").lower() or query in topic.get("type", "").lower()
        ]
        self.table.setRowCount(len(self._filtered_topics))
        for row, topic in enumerate(self._filtered_topics):
            self.table.setItem(row, 0, QtWidgets.QTableWidgetItem(topic.get("name", "")))
            self.table.setItem(row, 1, QtWidgets.QTableWidgetItem(topic.get("type", "")))
        if self._filtered_topics:
            self.table.selectRow(0)

    def selected_topic(self) -> dict[str, str] | None:
        row = self.table.currentRow()
        if row < 0 or row >= len(self._filtered_topics):
            return None
        return dict(self._filtered_topics[row])


class FieldSelectionDialog(QtWidgets.QDialog):
    def __init__(self, topic_name: str, topic_type: str, fields: list[str], parent=None) -> None:
        super().__init__(parent)
        self.setWindowTitle(f"Select Fields: {topic_name}")
        self.resize(720, 560)
        self._fields = fields
        layout = QtWidgets.QVBoxLayout(self)
        summary = QtWidgets.QLabel(f"{topic_name}\n{topic_type}")
        summary.setTextInteractionFlags(QtCore.Qt.TextSelectableByMouse)
        layout.addWidget(summary)
        self.search_edit = QtWidgets.QLineEdit()
        self.search_edit.setPlaceholderText("필드 검색")
        self.search_edit.textChanged.connect(self._refresh)
        layout.addWidget(self.search_edit)
        self.list_widget = QtWidgets.QListWidget()
        self.list_widget.setSelectionMode(QtWidgets.QAbstractItemView.ExtendedSelection)
        layout.addWidget(self.list_widget, 1)
        buttons = QtWidgets.QHBoxLayout()
        add_button = QtWidgets.QPushButton("선택 필드 추가")
        add_button.clicked.connect(self.accept)
        cancel_button = QtWidgets.QPushButton("취소")
        cancel_button.clicked.connect(self.reject)
        buttons.addStretch(1)
        buttons.addWidget(add_button)
        buttons.addWidget(cancel_button)
        layout.addLayout(buttons)
        self.list_widget.itemDoubleClicked.connect(lambda _item: self.accept())
        self._refresh()

    def _refresh(self) -> None:
        query = self.search_edit.text().strip().lower()
        self.list_widget.clear()
        for field in self._fields:
            if query and query not in field.lower():
                continue
            self.list_widget.addItem(field)
        if self.list_widget.count():
            self.list_widget.setCurrentRow(0)

    def selected_fields(self) -> list[str]:
        return [item.text() for item in self.list_widget.selectedItems()]


class NoWheelSpinBox(QtWidgets.QSpinBox):
    def wheelEvent(self, event) -> None:
        event.ignore()


class WirisStereoDistancePanel(QtWidgets.QWidget):
    calibration_preview_requested = QtCore.pyqtSignal(object)
    calibration_preview_changed = QtCore.pyqtSignal(object)
    camera_preview_cleared = QtCore.pyqtSignal()
    wiris_command_response = QtCore.pyqtSignal(object)
    calibration_capture_event = QtCore.pyqtSignal(object)
    calibration_diagnostics_changed = QtCore.pyqtSignal(object)
    measurement_package_changed = QtCore.pyqtSignal(object)

    def event(self, event: QtCore.QEvent) -> bool:  # noqa: N802 - Qt override.
        # A number of embedded/test workflows use deleteLater() instead of a
        # top-level close event.  Stop the QObject-owned preview thread before
        # Qt destroys the parent, otherwise QThread can be destroyed while its
        # event loop is still running.
        if event.type() == QtCore.QEvent.DeferredDelete:
            self._stop_calibration_preview_worker()
        return super().event(event)

    def __del__(self) -> None:
        # This is a last-resort guard for wrappers collected before Qt delivers
        # DeferredDelete.  It deliberately touches only the calibration preview
        # worker; session/settings shutdown remains explicit in closeEvent.
        try:
            self._stop_calibration_preview_worker()
        except Exception:
            pass

    def __init__(
        self,
        *,
        auto_start: bool = False,
        auto_start_pixhawk: bool = False,
        app_root: str | Path | None = None,
        settings_path: str | Path | None = None,
    ) -> None:
        super().__init__()
        self.app_root = Path(APP_ROOT if app_root is None else app_root).expanduser().resolve()
        self.settings_path = Path(
            self.app_root / WORKSTATION_SETTINGS_PATH.name
            if settings_path is None
            else settings_path
        ).expanduser().resolve()
        self.setMinimumSize(900, 820)
        self.latest_left: DetectionObservation | None = None
        self.latest_right: DetectionObservation | None = None
        self.ftp_log_lines: deque[str] = deque(maxlen=600)
        self.latest_calibration_preview_images: dict[str, QtGui.QImage] = {}
        self.calibration_preview_overlay_images: dict[str, QtGui.QImage] = {}
        self.calibration_preview_detection_results: dict[str, dict[str, object]] = {}
        self.calibration_preview_times = {
            "left": 0.0,
            "right": 0.0,
        }
        self.calibration_preview_request_ids = {
            "left": 0,
            "right": 0,
        }
        self._calibration_preview_next_request_id = 0
        # Calibration preview is an optional visual aid.  Keep its worker
        # lazy so constructing a panel for settings/evidence/tests does not
        # create a long-lived QThread that outlives a deleteLater() cycle.
        self._calibration_preview_worker: CheckerboardPreviewWorker | None = None
        self._calibration_preview_thread: QtCore.QThread | None = None
        self.checkerboard_calibration_worker: QtCore.QObject | None = None
        self.checkerboard_calibration_thread: QtCore.QThread | None = None
        self.calibration_wiris_ftp_worker: QtCore.QObject | None = None
        self.calibration_wiris_ftp_thread: QtCore.QThread | None = None
        self.calibration_wiris_ftp_oz_worker: QtCore.QObject | None = None
        self.calibration_wiris_ftp_oz_thread: QtCore.QThread | None = None
        self.calibration_wiris_oz_capture_worker: QtCore.QObject | None = None
        self.calibration_wiris_oz_capture_thread: QtCore.QThread | None = None
        self.calibration_wiris_oz_sync_worker: QtCore.QObject | None = None
        self.calibration_wiris_oz_sync_thread: QtCore.QThread | None = None
        self.calibration_wiris_oz_delete_worker: QtCore.QObject | None = None
        self.calibration_wiris_oz_delete_thread: QtCore.QThread | None = None
        self.calibration_wiris_oz_format_worker: QtCore.QObject | None = None
        self.calibration_wiris_oz_format_thread: QtCore.QThread | None = None
        self.calibration_wiris_oz_gilc_worker: QtCore.QObject | None = None
        self.calibration_wiris_oz_gilc_thread: QtCore.QThread | None = None
        self.wiris_image_format_worker: QtCore.QObject | None = None
        self.wiris_image_format_thread: QtCore.QThread | None = None
        self.wiris_image_format_status: dict[str, object] = {}
        self._workstation_calibration_preview_active = False
        self.destroyed.connect(self._stop_calibration_preview_worker)
        self.detection_buffers = {
            "left": TimedSampleBuffer[DetectionObservation](maxlen=120),
            "right": TimedSampleBuffer[DetectionObservation](maxlen=120),
        }
        self.latest_extrinsics: dict[str, CameraExtrinsicState] = {}
        self.latest_gps: dict[str, GpsPositionState] = {}
        self.custom_watch_specs: list[dict[str, str]] = []
        self.latest_rtk_baseline: RtkBaselineEstimate | None = None
        self.latest_p1_body_frd_baseline: BodyFrdBaselineProjection | None = None
        self.calibration_capture_index = 0
        self.last_calibration_result: StereoIntrinsicsResult | None = None
        self.calibration_image_diagnostics: tuple[dict[str, object], ...] = ()
        self.calibration_diagnostics_root = ""
        self.checkerboard_validation_result: StereoIntrinsicsResult | None = None
        self.checkerboard_validation_result_mode = "pixhawk"
        self.checkerboard_validation_summary: dict[str, object] = {}
        self.checkerboard_validation_run_results: list[dict[str, object]] = []
        self.checkerboard_validation_result_saved = False
        self.calibration_progress: dict[str, object] = {
            "phase": "idle",
            "processed": 0,
            "total": 0,
            "percent": 0.0,
            "message": "대기",
        }
        self.last_capture_zoom_level: int | None = None
        self.last_calibration_zoom_level: int | None = None
        self.active_intrinsics: StereoIntrinsicsResult | None = None
        self._active_intrinsics_profile_path = ""
        self.selected_extrinsics_profile: StereoExtrinsicsProfile | None = None
        # Independent geometry-comparison slots: unlike selected_extrinsics_profile,
        # these never overwrite each other, so Measurement-tab comparison rows
        # can always show the last checkerboard result and the last applied
        # PnP result side by side.
        self.checkerboard_extrinsics_profile: StereoExtrinsicsProfile | None = None
        self.optical_pose_extrinsics_profile: StereoExtrinsicsProfile | None = None
        self._selected_extrinsics_profile_path = ""
        self._selected_extrinsics_profile_hash = ""
        self._active_intrinsics_profile_hash = ""
        self._active_intrinsics_source_zoom_level: int | None = None
        self._active_extrinsics_profile_path = ""
        self._active_extrinsics_profile_hash = ""
        self._active_extrinsics_source_zoom_level: int | None = None
        self._active_calibration_selection_mode = ""
        self.actual_visible_zoom: dict[str, dict[str, object]] = {
            "left": {},
            "right": {},
        }
        self._zoom_readback_target: str | None = None
        self.zoom_intrinsics_compare_paths: list[Path] = []
        self.attitude_buffers = {
            "left": TimedSampleBuffer[CameraExtrinsicState](maxlen=1000),
            "right": TimedSampleBuffer[CameraExtrinsicState](maxlen=1000),
        }
        self.mavlink_pixhawk: DualPixhawkMavlinkSubscriber | None = None
        self._eth0_probe_buffers: dict[str, str] = {}
        self._eth0_probe_tokens: dict[str, int] = {}
        self._eth0_probe_counter = 0
        self.px4_shell_dialogs: dict[str, Px4ShellDialog] = {}
        self.camera_connected = False
        self.yolo_enabled = False
        self._camera_start_request_generation = 0
        self._pending_camera_start: tuple[bool, bool] | None = None
        self.camera_auto_detect_thread: QtCore.QThread | None = None
        self.camera_auto_detect_worker: CameraAutoDetectWorker | None = None
        self._camera_auto_connect_attempt = 0
        self._camera_auto_connect_retry_generation = 0
        # Once a camera accepted SETH TRUE, do not send the same activation
        # command again merely because RTSP frame verification is delayed.
        # Some WIRIS firmware restarts the RTSP service on SETH TRUE and can
        # become unresponsive when that command is repeated in a retry loop.
        self._auto_wiris_enabled_cameras: set[tuple[str, str]] = set()
        self._rtsp_reactivation_side: str | None = None
        self._rtsp_reactivation_enable_detection = False
        self._rtsp_reactivation_generation = 0
        self.wiris_control_check_thread: QtCore.QThread | None = None
        self.wiris_control_check_worker: WirisControlCheckWorker | None = None
        self.wiris_command_thread: QtCore.QThread | None = None
        self.wiris_command_worker: WirisCommandWorker | None = None
        self.pnp_photo_capture_thread: QtCore.QThread | None = None
        self.pnp_photo_capture_worker: QtCore.QObject | None = None
        self.pnp_photo_capture_records: list[dict[str, object]] = []
        self.oz_photo_capture_thread: QtCore.QThread | None = None
        self.oz_photo_capture_worker: QtCore.QObject | None = None
        self.oz_photo_capture_records: list[dict[str, object]] = []
        self.optical_pose_wiris_ftp_thread: QtCore.QThread | None = None
        self.optical_pose_wiris_ftp_worker: QtCore.QObject | None = None
        self.optical_pose_wiris_ftp_oz_thread: QtCore.QThread | None = None
        self.optical_pose_wiris_ftp_oz_worker: QtCore.QObject | None = None
        self._pnp_session_prepared = False
        self.wiris_lrf_thread: QtCore.QThread | None = None
        self.wiris_lrf_worker: WirisLrfPollWorker | None = None
        self.wiris_zoom_thread: QtCore.QThread | None = None
        self.wiris_zoom_worker: WirisZoomWorker | None = None
        self.wiris_time_sync_thread: QtCore.QThread | None = None
        self.wiris_time_sync_worker: WirisClockCommandWorker | None = None
        self.wiris_stip_thread: QtCore.QThread | None = None
        self.wiris_stip_worker: WirisStipWorker | None = None
        self.camera_network_discovery_thread: QtCore.QThread | None = None
        self.camera_network_discovery_worker: CameraNetworkDiscoveryWorker | None = None
        self.camera_network_apply_thread: QtCore.QThread | None = None
        self.camera_network_apply_worker: CameraNetworkAddressApplyWorker | None = None
        self.interface_status_thread: QtCore.QThread | None = None
        self.interface_status_worker: InterfaceStatusWorker | None = None
        self.pixhawk_usb_discovery_thread: QtCore.QThread | None = None
        self.pixhawk_usb_discovery_worker: PixhawkUsbDiscoveryWorker | None = None
        self.pixhawk_host_ip_discovery_thread: QtCore.QThread | None = None
        self.pixhawk_host_ip_discovery_worker: PixhawkHostIpDiscoveryWorker | None = None
        self.pixhawk_connection_thread: QtCore.QThread | None = None
        self.pixhawk_connection_worker: PixhawkConnectionWorker | None = None
        self.pixhawk_disconnection_thread: QtCore.QThread | None = None
        self.pixhawk_disconnection_worker: PixhawkDisconnectWorker | None = None
        self.pixhawk_stream_setup_thread: QtCore.QThread | None = None
        self.pixhawk_stream_setup_worker: PixhawkStreamSetupWorker | None = None
        self.pixhawk_usb_bootstrap_thread: QtCore.QThread | None = None
        self.pixhawk_usb_bootstrap_worker: PixhawkUsbBootstrapWorker | None = None
        self.pixhawk_usb_bootstrap_status: dict[str, dict[str, object]] = {
            "left": {"state": "IDLE", "detail": "대기", "updated_s": time.monotonic()},
            "right": {"state": "IDLE", "detail": "대기", "updated_s": time.monotonic()},
        }
        self.pixhawk_persistent_config_thread: QtCore.QThread | None = None
        self.pixhawk_persistent_config_worker: PixhawkPersistentConfigWorker | None = None
        self.pixhawk_rtcm_thread: QtCore.QThread | None = None
        self.pixhawk_rtcm_worker: PixhawkRtcmInjectionWorker | None = None
        self.pixhawk_shell_threads: dict[str, QtCore.QThread] = {}
        self.pixhawk_shell_workers: dict[str, PixhawkShellCommandWorker] = {}
        self.pixhawk_parameter_threads: dict[str, QtCore.QThread] = {}
        self.pixhawk_parameter_workers: dict[str, PixhawkParameterWorker] = {}
        self.rtsp_check_thread: QtCore.QThread | None = None
        self.rtsp_check_worker: RtspStreamCheckWorker | None = None
        self.rtsp_enable_thread: QtCore.QThread | None = None
        self.rtsp_enable_worker: WirisRtspEnableWorker | None = None
        self.gnss_base_serial_thread: QtCore.QThread | None = None
        self.gnss_base_serial_worker: GnssBaseSerialWorker | None = None
        self.gnss_base_serial_connected = False
        self.gnss_network_thread: QtCore.QThread | None = None
        self.gnss_network_worker: NtripNetworkWorker | None = None
        self.rtcm_pipeline_thread: QtCore.QThread | None = None
        self.rtcm_pipeline_worker: RtcmDataPlaneWorker | None = None
        self._rtcm_pipeline_base_worker: GnssBaseSerialWorker | None = None
        self._rtcm_pipeline_injection_worker: PixhawkRtcmInjectionWorker | None = None
        self._rtcm_package_signal_connected = False
        self.gnss_network_connected = False
        self.gnss_network_phase = "DISCONNECTED"
        self.gnss_network_stats: object | None = None
        self.gnss_network_status_line = ""
        self.gnss_network_last_error = ""
        self.gnss_network_reconnect_count = 0
        self.gnss_network_rtcm_parser = Rtcm3StreamParser()
        self.gnss_coordinate_samples: deque[GnssCoordinateSample] = deque(maxlen=2400)
        # Latest Base position is a WGS84 geodetic sample used only for the
        # assisted-mode NTRIP GGA hint.  It is cleared between serial/session
        # lifecycles so a previous experiment's coordinate cannot be reused.
        self.latest_gnss_base_position: GnssCoordinateSample | None = None
        self.gnss_fixed_base_profile = None
        self.gnss_fixed_base_live_profile = None
        self.gnss_fixed_base_pending_profile = None
        self.gnss_fixed_base_command_sent = False
        self.gnss_fixed_base_reconfigure_expected: dict[str, object] | None = None
        self.gnss_fixed_base_reconfigure_ack = False
        self.gnss_fixed_base_reconfigure_readback = False
        self.gnss_fixed_base_reconfigure_rtcm = False
        self.gnss_fixed_base_reconfigure_deadline_s: float | None = None
        self.gnss_active_operation_config: GnssOperationConfig | None = None
        self.gnss_rover_fixed_since: dict[str, float | None] = {"left": None, "right": None, "drone": None}
        self.gnss_operation_gate_active = False
        self.gnss_base_detect_thread: QtCore.QThread | None = None
        self.gnss_base_detect_worker: GnssBaseSerialDetectWorker | None = None
        self._gnss_base_candidate_rows: list[dict[str, object]] = []
        self.workstation_gnss_base_discovery_status = "NOT RUN"
        self.latest_gnss_base_survey_in: dict[str, object] | None = None
        self.gnss_base_survey_samples: deque[tuple[float, int, float, bool, bool]] = deque(maxlen=240)
        self.gnss_base_rtcm_stats = self._new_gnss_base_rtcm_stats()
        self.gnss_base_satellite_count: int | None = None
        self.gnss_base_satellite_count_source: str = ""
        self.gnss_base_satellite_quality: dict[str, object] | None = None
        self.auto_wiris_connect_pending = False
        self._startup_camera_sequence_pending = False
        self.auto_start = auto_start
        self.auto_start_pixhawk = auto_start_pixhawk
        self.settings = load_settings(self.settings_path)
        self.drone_telemetry_url = self.settings.drone_telemetry_url
        self.drone_telemetry_baud = self.settings.drone_telemetry_baud
        self.drone_telemetry_rtcm_forward = self.settings.drone_telemetry_rtcm_forward
        self.drone_telemetry_rtcm_source = normalize_drone_rtcm_source(
            self.settings.drone_telemetry_rtcm_source
        )
        self.drone_telemetry_connection_state = "DISCONNECTED"
        self.drone_telemetry_last_error = ""
        self.drone_telemetry_connection_thread: QtCore.QThread | None = None
        self.drone_telemetry_connection_worker: PixhawkConnectionWorker | None = None
        self.drone_telemetry_usb_discovery_thread: QtCore.QThread | None = None
        self.drone_telemetry_usb_discovery_worker: PixhawkUsbDiscoveryWorker | None = None
        self.drone_telemetry_usb_discovery_state = ""
        self._workstation_setting_overrides: dict[str, object] = {}
        self.gnss_operation_manager = GnssOperationManager(self.settings.gnss_operation)
        self.distance_history: deque[DistanceHistoryPoint] = deque(maxlen=600)
        self.geometry_comparison_rows: tuple[dict[str, object], ...] = ()
        self.distance_recording_rows: SpoolRows | list[dict[str, object]] = SpoolRows(
            preview_limit=RECORDING_PREVIEW_LIMIT,
            supports_updates=True,
        )
        self.distance_recording_keys: list[tuple[object, ...]] = []
        self.external_parameter_recording_rows: deque[dict[str, object]] = deque(maxlen=256)
        self.camera_recording_rows: dict[str, SpoolRows] = {
            "left": SpoolRows(),
            "right": SpoolRows(),
        }
        self.px4_recording_rows: dict[str, SpoolRows] = {
            "left": SpoolRows(),
            "right": SpoolRows(),
        }
        self.gnss_base_recording_rows: SpoolRows = SpoolRows()
        self.extrinsics_comparison_recording_rows: SpoolRows = SpoolRows()
        self._session_save_thread: QtCore.QThread | None = None
        self._session_save_worker: SessionSaveWorker | None = None
        self._session_save_retry_required = False
        self._session_save_finalized = False
        # True only when the current session was opened by Detection's
        # automatic measurement lifecycle. Manual/advanced evidence capture
        # remains compatible with the existing explicit session controls.
        self._automatic_session_recording = False
        self._recording_mode = "manual"
        # The Workstation UI uses the explicit Begin Session -> Detection ->
        # Finalize package contract.  Bare controller test doubles and legacy
        # non-UI callers omit this flag and retain the automatic-session
        # compatibility path.
        self._measurement_package_explicit_lifecycle = True
        self._measurement_session_hierarchy_enabled = True
        self._measurement_session = None
        self._measurement_session_id = ""
        self._measurement_session_state = "IDLE"
        self._measurement_id = ""
        self._measurement_app_run_id = (
            f"run-{datetime.now(timezone.utc):%Y%m%dT%H%M%S.%fZ}-{uuid.uuid4().hex[:8]}"
        )
        self._measurement_package = None
        self._measurement_package_state = "IDLE"
        self._measurement_package_detail = "Begin Session to collect evidence"
        self._measurement_package_error = ""
        self._measurement_package_detection_started = False
        self._measurement_package_detection_completed = False
        self._measurement_package_finalize_requested = False
        self._measurement_package_finalize_retry_scheduled = False
        self._measurement_package_registered_paths: set[tuple[str, str]] = set()
        self._measurement_package_inventory_errors: list[str] = []
        self._measurement_package_revision = 0
        self._measurement_package_calibration_collected = False
        self._recovered_session_metadata: dict[str, object] | None = None
        self.distance_recording_enabled = False
        self.distance_recording_started_monotonic_s: float | None = None
        self.recording_session_id = ""
        self._measurement_attempt_generation_id = _new_measurement_attempt_generation_id()
        self.recording_session_started_wall_s: float | None = None
        self.recording_session_started_monotonic_s: float | None = None
        self.recording_session_started_unix_ns: int | None = None
        self.recording_session_started_monotonic_ns: int | None = None
        self.recording_session_anchor_uncertainty_ns: int | None = None
        self._recording_clock_sync_start_observation: dict[str, object] | None = None
        self._recording_clock_sync_end_observation: dict[str, object] | None = None
        self._last_wiris_pc_utc_sync: dict[str, object] | None = None
        self.camera_sync_status = CAMERA_SYNC_STATUS_NOT_VERIFIED
        self.camera_sync_reason = "STUS 동기화 후 RTCP/NTP 검증이 필요합니다."
        # Workstation protocol checkpoints are evidence only.  They do not
        # alter the existing camera sync state or measurement timestamps.
        self.workstation_time_sync_checkpoints: list[dict[str, object]] = []
        self._camera_sync_transport_override: str | None = None
        self._camera_sync_deadline_monotonic_s: float | None = None
        self._camera_sync_stus_sr_baseline: dict[str, int] = {}
        self._camera_sync_timer = QtCore.QTimer(self)
        self._camera_sync_timer.setInterval(250)
        self._camera_sync_timer.timeout.connect(self._poll_camera_sync_state)
        self.recording_reference_attitudes: dict[str, AttitudeSnapshot] = {}
        self.recording_reference_gps: dict[str, GpsSnapshot] = {}
        self.recording_origin_gps: GpsSnapshot | None = None
        self.recording_reference_baseline_ned_m: tuple[float, float, float] | None = None
        self._recording_pair_row_index: dict[tuple[object, ...], int] = {}
        self.video_recording_enabled = False
        self.video_recording_paths: dict[str, Path] = {}
        self._last_distance_history_key: tuple[object, ...] | None = None
        self._measurement_attempt_counter = 0
        self._last_measurement_attempt_key: tuple[object, ...] | None = None
        self._last_measurement_attempt: MeasurementAttempt | None = None
        self._last_correspondence_rejection_reason = ""
        self.camera_frame_times = {
            "left": deque(maxlen=120),
            "right": deque(maxlen=120),
        }
        self.camera_frame_metadata = {
            "left": {},
            "right": {},
        }
        self.camera_frame_metadata_history: dict[str, deque[dict[str, object]]] = {
            "left": deque(maxlen=240),
            "right": deque(maxlen=240),
        }
        self._last_clock_offset_sample_s = 0.0
        self.selected_runtime_frame_metadata = {
            "left": {},
            "right": {},
        }
        self.selected_runtime_frame_pair_delta_s: float | None = None
        self.selected_runtime_frame_pair_basis = ""
        self.selected_runtime_frame_pair_camera_rtc_delta_s: float | None = None
        self._runtime_pair_uses_app_fallback = False
        self.camera_stream_stats = {
            "left": {},
            "right": {},
        }
        self.active_stream_decoders: dict[str, str | None] = {
            "left": None,
            "right": None,
        }
        self.pixhawk_attitude_times = {
            "left": deque(maxlen=240),
            "right": deque(maxlen=240),
        }
        self.attitude_history: dict[str, deque[AttitudeHistoryPoint]] = {
            "roll": deque(maxlen=600),
            "pitch": deque(maxlen=600),
            "yaw": deque(maxlen=600),
        }
        self.gnss_monitor = GNSSMonitor()
        self.baseline_manager = BaselineManager(self.gnss_monitor)
        self.magnetometer_monitor = MagnetometerMonitor()
        self.attitude_monitor = AttitudeMonitor()
        self.coordinate_system_manager = CoordinateSystemManager()
        self.alignment_validator = AlignmentValidator()
        self.alignment_profile_manager = AlignmentProfileManager()
        self.loaded_alignment_profile: dict[str, object] | None = None
        self.latest_mag_sensors: dict[str, dict[str, object]] = {"left": {}, "right": {}}
        self.latest_can_mag_sensors: dict[str, dict[str, object]] = {"left": {}, "right": {}}
        self.latest_vehicle_mag_device_ids: dict[str, int | None] = {"left": None, "right": None}
        self.px4_mag_managers = {
            "left": Px4MagManager(MockParameterStore(current_pixhawk_6x_mag_params())),
            "right": Px4MagManager(MockParameterStore(current_pixhawk_6x_mag_params())),
        }
        self._px4_mag_status_cache: dict[str, object] = {}
        self._px4_attitude_cal_status_cache: dict[str, Px4AttitudeCalStatus] = {}
        self._px4_calibration_parameter_error: dict[str, str] = {}
        self._px4_calibration_reset_error: dict[str, str] = {}
        self._px4_calibration_reset_pending: dict[str, str] = {}
        self._px4_calibration_auto_refreshed_sides: set[str] = set()
        self.px4_mag_manager = self.px4_mag_managers["left"]
        self.px4_attitude_cal_managers = {
            "left": Px4AttitudeCalManager(MockParameterStore(current_pixhawk_6x_attitude_cal_params())),
            "right": Px4AttitudeCalManager(MockParameterStore(current_pixhawk_6x_attitude_cal_params())),
        }
        self.latest_gps_sensors: dict[str, dict[str, object]] = {"left": {}, "right": {}}
        self.latest_gnss_relative: dict[str, object] = {}
        self.latest_wiris_lrf: dict[str, dict[str, object]] = {
            "left": {},
            "right": {},
        }
        self.last_rtsp_check_status: dict[str, str] = {"left": "unchecked", "right": "unchecked"}
        self.last_distance_quality = "WARNING"
        self.last_distance_status = "거리: 좌/우 검출 매칭 대기 중"

        root = QtWidgets.QVBoxLayout(self)
        root.setContentsMargins(8, 8, 8, 8)
        root.setSpacing(6)
        self.tabs = QtWidgets.QTabWidget()
        self.tabs.setDocumentMode(True)
        root.addWidget(self.tabs, 1)
        self._apply_visual_style()

        setup_tab = QtWidgets.QWidget()
        setup_layout = QtWidgets.QVBoxLayout(setup_tab)
        time_tab = QtWidgets.QWidget()
        time_layout = QtWidgets.QVBoxLayout(time_tab)
        alignment_tab = QtWidgets.QWidget()
        alignment_layout = QtWidgets.QVBoxLayout(alignment_tab)
        alignment_overview_tab = QtWidgets.QWidget()
        alignment_overview_layout = QtWidgets.QVBoxLayout(alignment_overview_tab)
        rtk_tab = QtWidgets.QWidget()
        rtk_layout = QtWidgets.QVBoxLayout(rtk_tab)
        mag_management_tab = QtWidgets.QWidget()
        mag_management_layout = QtWidgets.QVBoxLayout(mag_management_tab)
        alignment_mag_tab = QtWidgets.QWidget()
        alignment_mag_layout = QtWidgets.QVBoxLayout(alignment_mag_tab)
        alignment_attitude_tab = QtWidgets.QWidget()
        alignment_attitude_layout = QtWidgets.QVBoxLayout(alignment_attitude_tab)
        optical_alignment_tab = QtWidgets.QWidget()
        optical_alignment_layout = QtWidgets.QVBoxLayout(optical_alignment_tab)
        alignment_coordinate_tab = QtWidgets.QWidget()
        alignment_coordinate_layout = QtWidgets.QVBoxLayout(alignment_coordinate_tab)
        alignment_validation_tab = QtWidgets.QWidget()
        alignment_validation_layout = QtWidgets.QVBoxLayout(alignment_validation_tab)
        alignment_profile_tab = QtWidgets.QWidget()
        alignment_profile_layout = QtWidgets.QVBoxLayout(alignment_profile_tab)
        gnss_base_tab = QtWidgets.QWidget()
        gnss_base_layout = QtWidgets.QVBoxLayout(gnss_base_tab)
        signal_alignment_tab = QtWidgets.QWidget()
        signal_alignment_layout = QtWidgets.QVBoxLayout(signal_alignment_tab)
        rm3100_alignment_tab = QtWidgets.QWidget()
        rm3100_alignment_layout = QtWidgets.QVBoxLayout(rm3100_alignment_tab)
        yaw_alignment_tab = QtWidgets.QWidget()
        yaw_alignment_layout = QtWidgets.QVBoxLayout(yaw_alignment_tab)
        calibration_tab = QtWidgets.QWidget()
        calibration_layout = QtWidgets.QVBoxLayout(calibration_tab)
        checkerboard_calibration_tab = QtWidgets.QWidget()
        checkerboard_calibration_layout = QtWidgets.QVBoxLayout(checkerboard_calibration_tab)
        checkerboard_validation_tab = QtWidgets.QWidget()
        checkerboard_validation_layout = QtWidgets.QVBoxLayout(checkerboard_validation_tab)
        zoom_intrinsics_tab = QtWidgets.QWidget()
        zoom_intrinsics_layout = QtWidgets.QVBoxLayout(zoom_intrinsics_tab)
        camera_params_tab = QtWidgets.QWidget()
        camera_params_layout = QtWidgets.QVBoxLayout(camera_params_tab)
        intrinsic_parameters_tab = QtWidgets.QWidget()
        intrinsic_parameters_layout = QtWidgets.QVBoxLayout(intrinsic_parameters_tab)
        extrinsic_parameters_tab = QtWidgets.QWidget()
        extrinsic_parameters_layout = QtWidgets.QVBoxLayout(extrinsic_parameters_tab)
        triangulation_quality_tab = QtWidgets.QWidget()
        triangulation_quality_layout = QtWidgets.QVBoxLayout(triangulation_quality_tab)
        yolo_tab = QtWidgets.QWidget()
        yolo_layout_outer = QtWidgets.QVBoxLayout(yolo_tab)
        measurement_tab = QtWidgets.QWidget()
        measurement_layout = QtWidgets.QVBoxLayout(measurement_tab)
        stereo_tab = QtWidgets.QWidget()
        stereo_layout = QtWidgets.QVBoxLayout(stereo_tab)
        runtime_tab = QtWidgets.QWidget()
        runtime_layout = QtWidgets.QVBoxLayout(runtime_tab)
        save_tab = QtWidgets.QWidget()
        save_layout = QtWidgets.QVBoxLayout(save_tab)
        advanced_tab = QtWidgets.QWidget()
        advanced_layout = QtWidgets.QVBoxLayout(advanced_tab)
        wiris_command_panel = QtWidgets.QWidget()
        wiris_command_layout = QtWidgets.QVBoxLayout(wiris_command_panel)
        baseline_field_browser_tab = QtWidgets.QWidget()
        baseline_field_browser_layout = QtWidgets.QVBoxLayout(baseline_field_browser_tab)
        px4_mag_advanced_tab = QtWidgets.QWidget()
        px4_mag_advanced_layout = QtWidgets.QVBoxLayout(px4_mag_advanced_tab)
        for layout in (
            setup_layout,
            time_layout,
            alignment_layout,
            alignment_overview_layout,
            rtk_layout,
            mag_management_layout,
            alignment_mag_layout,
            alignment_attitude_layout,
            optical_alignment_layout,
            alignment_coordinate_layout,
            alignment_validation_layout,
            alignment_profile_layout,
            signal_alignment_layout,
            rm3100_alignment_layout,
            yaw_alignment_layout,
            calibration_layout,
            checkerboard_calibration_layout,
            checkerboard_validation_layout,
            zoom_intrinsics_layout,
            camera_params_layout,
            intrinsic_parameters_layout,
            extrinsic_parameters_layout,
            yolo_layout_outer,
            measurement_layout,
            stereo_layout,
            runtime_layout,
            save_layout,
            advanced_layout,
            wiris_command_layout,
            baseline_field_browser_layout,
            px4_mag_advanced_layout,
        ):
            layout.setContentsMargins(6, 6, 6, 6)
            layout.setSpacing(6)

        self.tabs.addTab(setup_tab, "연결")
        self.tabs.addTab(time_tab, "시간")
        self.tabs.addTab(calibration_tab, "보정")
        self.tabs.addTab(camera_params_tab, "파라미터")
        self.tabs.addTab(alignment_tab, "정렬")
        self.tabs.addTab(yolo_tab, "YOLO")
        self.tabs.addTab(measurement_tab, "측정")
        self.tabs.addTab(save_tab, "저장·분석")
        self.tabs.addTab(advanced_tab, "고급")
        self.time_tab_index = self.tabs.indexOf(time_tab)
        self.setup_tab_index = self.tabs.indexOf(setup_tab)
        self.alignment_tab_index = self.tabs.indexOf(alignment_tab)
        self.calibration_tab_index = self.tabs.indexOf(calibration_tab)
        self.parameter_tab_index = self.tabs.indexOf(camera_params_tab)
        self.camera_parameters_tab_index = self.parameter_tab_index
        self.measurement_parameter_tab_index = self.parameter_tab_index
        self.depth_tab_index = self.tabs.indexOf(measurement_tab)
        self.results_tab_index = self.tabs.indexOf(save_tab)
        self.pixhawk_detail_tab_index = self.tabs.indexOf(alignment_tab)
        self.rtk_detail_tab_index = self.tabs.indexOf(alignment_tab)
        self.advanced_tab_index = self.tabs.indexOf(advanced_tab)
        self.tabs.currentChanged.connect(self._handle_tab_changed)

        calibration_tabs = QtWidgets.QTabWidget()
        calibration_tabs.setDocumentMode(True)
        calibration_tabs.addTab(checkerboard_calibration_tab, "체커보드")
        calibration_tabs.addTab(checkerboard_validation_tab, "체커보드 검증/계산")
        calibration_tabs.addTab(zoom_intrinsics_tab, "줌 내부 파라미터")
        calibration_layout.addWidget(calibration_tabs, 1)
        self.calibration_tabs = calibration_tabs
        self.checkerboard_calibration_tab_index = calibration_tabs.indexOf(checkerboard_calibration_tab)
        self.checkerboard_validation_tab_index = calibration_tabs.indexOf(checkerboard_validation_tab)
        self.zoom_intrinsics_tab_index = calibration_tabs.indexOf(zoom_intrinsics_tab)

        parameter_tabs = QtWidgets.QTabWidget()
        parameter_tabs.setDocumentMode(True)
        parameter_tabs.addTab(intrinsic_parameters_tab, "내부")
        parameter_tabs.addTab(extrinsic_parameters_tab, "외부/기하")
        parameter_tabs.addTab(stereo_tab, "계산 설정")
        camera_params_layout.addWidget(parameter_tabs, 1)
        self.parameter_tabs = parameter_tabs
        self.intrinsic_parameters_tab_index = parameter_tabs.indexOf(intrinsic_parameters_tab)
        self.extrinsic_parameters_tab_index = parameter_tabs.indexOf(extrinsic_parameters_tab)
        self.stereo_settings_tab_index = parameter_tabs.indexOf(stereo_tab)

        alignment_tabs = QtWidgets.QTabWidget()
        alignment_tabs.setDocumentMode(True)
        alignment_tabs.addTab(alignment_overview_tab, "센서 인식")
        alignment_tabs.addTab(rtk_tab, "소스 / 품질")
        alignment_tabs.addTab(gnss_base_tab, "GNSS Base")
        alignment_tabs.addTab(mag_management_tab, "MAG 관리")
        alignment_tabs.addTab(alignment_mag_tab, "Baseline 계산")
        alignment_tabs.addTab(alignment_attitude_tab, "Baseline / 자세 정렬")
        alignment_tabs.addTab(optical_alignment_tab, "Optical 정렬")
        alignment_layout.addWidget(alignment_tabs, 1)
        self.alignment_tabs = alignment_tabs
        self.alignment_overview_tab_index = alignment_tabs.indexOf(alignment_overview_tab)
        self.signal_alignment_tab_index = self.alignment_overview_tab_index
        self.source_quality_alignment_tab_index = alignment_tabs.indexOf(rtk_tab)
        self.gnss_base_alignment_tab_index = alignment_tabs.indexOf(gnss_base_tab)
        self.rm3100_alignment_tab_index = alignment_tabs.indexOf(mag_management_tab)
        self.baseline_alignment_tab_index = alignment_tabs.indexOf(alignment_mag_tab)
        self.yaw_alignment_tab_index = alignment_tabs.indexOf(alignment_attitude_tab)
        self.baseline_attitude_alignment_tab_index = self.yaw_alignment_tab_index
        self.optical_alignment_tab_index = alignment_tabs.indexOf(optical_alignment_tab)
        self.coordinate_alignment_tab_index = -1
        self.validation_alignment_tab_index = -1
        self.alignment_profile_tab_index = -1

        advanced_tabs = QtWidgets.QTabWidget()
        advanced_tabs.setDocumentMode(True)
        advanced_tabs.currentChanged.connect(lambda _index: self._handle_tab_changed(self.tabs.currentIndex()))
        advanced_layout.addWidget(advanced_tabs, 1)
        self.advanced_tabs = advanced_tabs

        measurement_tabs = QtWidgets.QTabWidget()
        measurement_tabs.setDocumentMode(True)
        measurement_tabs.addTab(runtime_tab, "실시간")
        measurement_tabs.addTab(triangulation_quality_tab, "측정 품질")
        measurement_layout.addWidget(measurement_tabs, 1)
        self.measurement_tabs = measurement_tabs
        self.live_measurement_tab_index = measurement_tabs.indexOf(runtime_tab)
        self.triangulation_quality_tab_index = measurement_tabs.indexOf(triangulation_quality_tab)
        measurement_tabs.currentChanged.connect(lambda _index: self._handle_tab_changed(self.tabs.currentIndex()))

        connection_tabs = QtWidgets.QTabWidget()
        connection_tabs.setDocumentMode(True)
        camera_tab = QtWidgets.QWidget()
        camera_layout_outer = QtWidgets.QVBoxLayout(camera_tab)
        pixhawk_connection_tab = QtWidgets.QWidget()
        pixhawk_connection_layout = QtWidgets.QVBoxLayout(pixhawk_connection_tab)
        for layout in (camera_layout_outer, yolo_layout_outer, pixhawk_connection_layout):
            layout.setContentsMargins(0, 0, 0, 0)
            layout.setSpacing(6)
        connection_tabs.addTab(camera_tab, "WIRIS")
        connection_tabs.addTab(pixhawk_connection_tab, "Pixhawk")
        self.setup_tabs = connection_tabs
        self.pixhawk_connection_tab_index = connection_tabs.indexOf(pixhawk_connection_tab)
        camera_group = QtWidgets.QGroupBox("카메라 / WIRIS")
        camera_layout = QtWidgets.QVBoxLayout(camera_group)
        camera_layout.setSpacing(10)
        camera_network_group = QtWidgets.QGroupBox("카메라망 / NIC")
        camera_form = QtWidgets.QGridLayout(camera_network_group)
        self.camera_network_form = camera_form
        camera_form.setHorizontalSpacing(8)
        camera_form.setVerticalSpacing(10)
        yolo_group = QtWidgets.QGroupBox("YOLO / 객체 검출")
        yolo_form = QtWidgets.QGridLayout(yolo_group)
        yolo_form.setHorizontalSpacing(8)
        yolo_form.setVerticalSpacing(8)
        pixhawk_group = QtWidgets.QGroupBox("픽스호크 / PX4")
        pixhawk_form = QtWidgets.QGridLayout(pixhawk_group)
        pixhawk_form.setHorizontalSpacing(8)
        pixhawk_form.setVerticalSpacing(8)
        self.pixhawk_settings_group = pixhawk_group
        self.interface_edit = QtWidgets.QLineEdit(self.settings.interface_name or DEFAULT_INTERFACE)
        self.host_address_edit = QtWidgets.QLineEdit(self.settings.camera_host_address or DEFAULT_HOST_ADDRESS)
        self.pixhawk_host_address_edit = QtWidgets.QLineEdit(self.settings.pixhawk_host_address)
        self.left_url_edit = QtWidgets.QLineEdit(self.settings.left_rtsp_url or DEFAULT_CAMERA_URLS[0])
        self.right_url_edit = QtWidgets.QLineEdit(self.settings.right_rtsp_url or DEFAULT_CAMERA_URLS[1])
        self.left_rtsp_ip_edit = QtWidgets.QLineEdit(self._rtsp_url_host_text(self.left_url_edit.text()).split(":", 1)[0])
        self.right_rtsp_ip_edit = QtWidgets.QLineEdit(
            self._rtsp_url_host_text(self.right_url_edit.text()).split(":", 1)[0]
        )
        self.left_rtsp_apply_button = QtWidgets.QPushButton("Apply Video URL")
        self.left_rtsp_apply_button.clicked.connect(lambda _checked=False: self.apply_rtsp_url("left"))
        self.right_rtsp_apply_button = QtWidgets.QPushButton("Apply Video URL")
        self.right_rtsp_apply_button.clicked.connect(lambda _checked=False: self.apply_rtsp_url("right"))
        self.left_control_apply_button = QtWidgets.QPushButton("Apply Control Port")
        self.left_control_apply_button.clicked.connect(lambda _checked=False: self.apply_control_settings("left"))
        self.right_control_apply_button = QtWidgets.QPushButton("Apply Control Port")
        self.right_control_apply_button.clicked.connect(lambda _checked=False: self.apply_control_settings("right"))
        self.left_zoom_host_edit = QtWidgets.QLineEdit(self.settings.left_control_host)
        self.right_zoom_host_edit = QtWidgets.QLineEdit(self.settings.right_control_host)
        self.left_stip_ip_edit = QtWidgets.QLineEdit(self.settings.left_stip_ip)
        self.right_stip_ip_edit = QtWidgets.QLineEdit(self.settings.right_stip_ip)
        self.stip_mask_edit = QtWidgets.QLineEdit(self.settings.stip_subnet_mask)
        self.stip_reboot_checkbox = self._build_toggle_button("STIP 후 재부팅", self.settings.stip_reboot)
        self.zoom_port_spin = NoWheelSpinBox()
        self.zoom_port_spin.setRange(1, 65535)
        self.zoom_port_spin.setValue(self.settings.wiris_control_port)
        self.left_control_port_spin = NoWheelSpinBox()
        self.left_control_port_spin.setRange(1, 65535)
        self.left_control_port_spin.setValue(self.settings.left_control_port)
        self.right_control_port_spin = NoWheelSpinBox()
        self.right_control_port_spin.setRange(1, 65535)
        self.right_control_port_spin.setValue(self.settings.right_control_port)
        self.zoom_timeout_spin = QtWidgets.QDoubleSpinBox()
        self.zoom_timeout_spin.setRange(0.1, 10.0)
        self.zoom_timeout_spin.setSingleStep(0.1)
        self.zoom_timeout_spin.setValue(self.settings.wiris_control_timeout_s)
        self.weights_edit = QtWidgets.QLineEdit(self.settings.yolo_weights_path or DEFAULT_YOLO_WEIGHTS)
        self.device_edit = QtWidgets.QLineEdit(self._normalized_yolo_device(self.settings.yolo_device))
        self.device_edit.hide()
        self.device_combo = QtWidgets.QComboBox()
        self.device_combo.addItem("CPU", YOLO_CPU_DEVICE)
        self.device_combo.addItem("CUDA", YOLO_CUDA_DEVICE)
        self.device_combo.setAccessibleName("추론 장치")
        self._set_yolo_device_combo(self._normalized_yolo_device(self.settings.yolo_device))
        self.device_combo.currentIndexChanged.connect(self._refresh_yolo_device_toggle)
        self.input_size_spin = QtWidgets.QSpinBox()
        self.input_size_spin.setRange(160, 1920)
        self.input_size_spin.setSingleStep(32)
        self.input_size_spin.setValue(self.settings.yolo_input_size)
        self.conf_spin = QtWidgets.QDoubleSpinBox()
        self.conf_spin.setRange(0.01, 1.0)
        self.conf_spin.setSingleStep(0.05)
        self.conf_spin.setValue(self.settings.yolo_conf_threshold)
        self.iou_spin = QtWidgets.QDoubleSpinBox()
        self.iou_spin.setRange(0.01, 1.0)
        self.iou_spin.setSingleStep(0.05)
        self.iou_spin.setValue(self.settings.yolo_iou_threshold)
        self.yolo_gate_class_checkbox = self._build_toggle_button(
            "대상 클래스 제한",
            self.settings.yolo_gate_class_enabled,
        )
        self.yolo_gate_class_edit = QtWidgets.QLineEdit(self.settings.yolo_gate_class_text)
        self.yolo_gate_class_edit.setPlaceholderText("예: 0 또는 person")
        self.yolo_gate_matching_checkbox = self._build_toggle_button(
            "시간 기준",
            self.settings.yolo_gate_matching_enabled,
        )
        self.yolo_gate_time_delta_checkbox = self._build_toggle_button(
            "최대 Δt",
            self.settings.yolo_gate_time_delta_enabled,
        )
        self.yolo_gate_vertical_delta_checkbox = self._build_toggle_button(
            "vertical Δ",
            self.settings.yolo_gate_vertical_delta_enabled,
        )
        self.yolo_gate_vertical_delta_spin = QtWidgets.QDoubleSpinBox()
        self.yolo_gate_vertical_delta_spin.setRange(0.0, 5000.0)
        self.yolo_gate_vertical_delta_spin.setValue(self.settings.yolo_gate_max_vertical_delta_px)
        self.yolo_gate_vertical_delta_spin.setSuffix(" px")
        self.yolo_gate_time_delta_spin = QtWidgets.QDoubleSpinBox()
        self.yolo_gate_time_delta_spin.setRange(0.0, 10.0)
        self.yolo_gate_time_delta_spin.setDecimals(3)
        self.yolo_gate_time_delta_spin.setValue(self.settings.yolo_gate_max_time_delta_s)
        self.yolo_gate_time_delta_spin.setSuffix(" s")
        self.yolo_gate_time_basis_combo = QtWidgets.QComboBox()
        self.yolo_gate_time_basis_combo.addItem("RTCP/NTP 필수", "camera_rtc_required")
        self.yolo_gate_time_basis_combo.addItem("RTCP/NTP 우선", "camera_rtcp")
        self.yolo_gate_time_basis_combo.addItem("app 수신 시간", "detection")
        time_basis_index = self.yolo_gate_time_basis_combo.findData(self.settings.yolo_gate_time_basis)
        self.yolo_gate_time_basis_combo.setCurrentIndex(max(0, time_basis_index))
        self.preview_width_spin = QtWidgets.QSpinBox()
        self.preview_width_spin.setRange(320, 3840)
        self.preview_width_spin.setSingleStep(160)
        self.preview_width_spin.setValue(self.settings.preview_width)
        self.rtsp_latency_spin = QtWidgets.QSpinBox()
        self.rtsp_latency_spin.setRange(0, 5000)
        self.rtsp_latency_spin.setSingleStep(25)
        self.rtsp_latency_spin.setValue(self.settings.rtsp_latency_ms)
        self.yolo_event_queue_size_spin = QtWidgets.QSpinBox()
        self.yolo_event_queue_size_spin.setRange(1, 10000)
        self.yolo_event_queue_size_spin.setValue(self.settings.yolo_event_queue_size)
        self.yolo_frame_queue_size_spin = QtWidgets.QSpinBox()
        self.yolo_frame_queue_size_spin.setRange(1, 1000)
        self.yolo_frame_queue_size_spin.setValue(self.settings.yolo_frame_queue_size)
        self.decoder_combo = QtWidgets.QComboBox()
        self.decoder_combo.addItem("Auto", DECODER_AUTO)
        self.decoder_combo.addItem("NVIDIA", DECODER_NVIDIA)
        self.decoder_combo.addItem("Software (avdec_h264)", DECODER_SOFTWARE)
        self._set_combo_data(self.decoder_combo, self.settings.rtsp_decoder)
        self.stream_backend_combo = QtWidgets.QComboBox()
        self.stream_backend_combo.addItem("GStreamer", STREAM_BACKEND_GSTREAMER)
        self.stream_backend_combo.addItem("OpenCV/FFmpeg", STREAM_BACKEND_OPENCV)
        self.stream_backend_combo.addItem("OpenCV Auto", STREAM_BACKEND_OPENCV_AUTO)
        self.stream_backend_combo.addItem("PyAV", STREAM_BACKEND_PYAV)
        self._set_combo_data(self.stream_backend_combo, self.settings.stream_backend)
        self.rtsp_transport_combo = QtWidgets.QComboBox()
        self.rtsp_transport_combo.addItem("UDP", RTSP_TRANSPORT_UDP)
        self.rtsp_transport_combo.addItem("TCP", RTSP_TRANSPORT_TCP)
        self._set_combo_data(self.rtsp_transport_combo, self.settings.rtsp_transport)
        self.detect_control_button = QtWidgets.QPushButton("Scan Control Port")
        self.detect_control_button.clicked.connect(lambda _checked=False: self.detect_camera_control("left"))
        self.right_detect_control_button = QtWidgets.QPushButton("Scan Control Port")
        self.right_detect_control_button.clicked.connect(lambda _checked=False: self.detect_camera_control("right"))
        self.detect_rtsp_button = QtWidgets.QPushButton("Scan Video URL")
        self.detect_rtsp_button.clicked.connect(self.detect_camera_rtsp)
        self.right_detect_rtsp_button = QtWidgets.QPushButton("Scan Video URL")
        self.right_detect_rtsp_button.clicked.connect(self.detect_camera_rtsp)
        self.auto_wiris_connect_button = QtWidgets.QPushButton("Auto-Connect Cameras")
        self.auto_wiris_connect_button.clicked.connect(self.auto_connect_wiris)
        self.detect_pc_ip_button = QtWidgets.QPushButton("Discover PC IP")
        self.detect_pc_ip_button.clicked.connect(self.discover_pc_camera_ip)
        self.apply_pc_ip_button = QtWidgets.QPushButton("Apply PC IP")
        self.apply_pc_ip_button.clicked.connect(self.apply_pc_camera_ip)
        self.camera_candidate_table = QtWidgets.QTableWidget(0, 6)
        self.camera_candidate_table.setHorizontalHeaderLabels(["종류", "주소", "포트", "상태", "인터페이스", "PC IP"])
        self.camera_candidate_table.verticalHeader().setVisible(False)
        self.camera_candidate_table.setSelectionBehavior(QtWidgets.QAbstractItemView.SelectRows)
        self.camera_candidate_table.setSelectionMode(QtWidgets.QAbstractItemView.SingleSelection)
        self.camera_candidate_table.setEditTriggers(QtWidgets.QAbstractItemView.NoEditTriggers)
        self.camera_candidate_table.setMinimumHeight(140)
        self.connection_sequence_label = QtWidgets.QLabel(
            "1 PC IP 적용  |  2 Telnet port 탐색  |  3 영상 송출 활성화  |  4 RTSP port/주소 탐색  |  5 영상 수신 확인  |  6 카메라 Preview 연결"
        )
        self.connection_sequence_label.setObjectName("statusStrip")
        self.connection_sequence_label.setWordWrap(True)
        self.apply_candidate_nic_button = QtWidgets.QPushButton("Apply Selected NIC")
        self.apply_candidate_nic_button.clicked.connect(self.apply_selected_camera_candidate_nic)
        self.apply_candidate_left_button = QtWidgets.QPushButton("Apply to Left")
        self.apply_candidate_left_button.clicked.connect(lambda _checked=False: self.apply_selected_camera_candidate("left"))
        self.apply_candidate_right_button = QtWidgets.QPushButton("Apply to Right")
        self.apply_candidate_right_button.clicked.connect(lambda _checked=False: self.apply_selected_camera_candidate("right"))
        self.clear_camera_candidate_button = QtWidgets.QPushButton("Clear")
        self.clear_camera_candidate_button.clicked.connect(self.clear_camera_candidate_history)
        self.clear_camera_candidate_button.setEnabled(False)
        self.clear_camera_candidate_button.setToolTip("WIRIS 탐색 후보 내역을 지웁니다.")
        self.camera_candidate_table.itemSelectionChanged.connect(self._refresh_camera_candidate_actions)
        self._camera_candidate_rows: list[dict[str, object]] = []
        self.workstation_camera_discovery_status = "NOT RUN"
        self.pixhawk_usb_discover_button = QtWidgets.QPushButton("Scan USB Pixhawks")
        self.pixhawk_usb_discover_button.clicked.connect(self.discover_usb_pixhawk_connections)
        self.pixhawk_mavlink_url_candidate_button = QtWidgets.QPushButton(
            "Generate Ethernet MAVLink Candidates"
        )
        self.pixhawk_mavlink_url_candidate_button.clicked.connect(self.discover_mavlink_url_candidates)
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
        self.clear_pixhawk_pc_candidate_button.clicked.connect(lambda _checked=False: self.clear_pixhawk_candidate_history("pc"))
        self.clear_pixhawk_pc_candidate_button.setEnabled(False)
        self.clear_pixhawk_pc_candidate_button.setToolTip("Pixhawk PC 후보 내역을 지웁니다.")
        self.apply_pixhawk_candidate_host_button = QtWidgets.QPushButton("Apply to Host")
        self.apply_pixhawk_candidate_host_button.setEnabled(False)
        self.apply_pixhawk_candidate_host_button.clicked.connect(
            lambda _checked=False: self.apply_selected_pixhawk_candidate("host")
        )
        self.apply_pixhawk_candidate_left_button = QtWidgets.QPushButton("Apply to P1")
        self.apply_pixhawk_candidate_left_button.setEnabled(False)
        self.apply_pixhawk_candidate_left_button.clicked.connect(
            lambda _checked=False: self.apply_selected_pixhawk_candidate("left")
        )
        self.apply_pixhawk_candidate_right_button = QtWidgets.QPushButton("Apply to P2")
        self.apply_pixhawk_candidate_right_button.setEnabled(False)
        self.apply_pixhawk_candidate_right_button.clicked.connect(
            lambda _checked=False: self.apply_selected_pixhawk_candidate("right")
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
            "mavlink": [],
        }
        self.workstation_pixhawk_discovery_status = "NOT RUN"
        self.swap_camera_button = QtWidgets.QPushButton("Swap Camera Sides")
        self.swap_camera_button.clicked.connect(self.swap_cameras)
        self.swap_pixhawk_button = QtWidgets.QPushButton("Swap P1 / P2 Roles")
        self.swap_pixhawk_button.clicked.connect(self.swap_pixhawks)
        self.check_button = QtWidgets.QPushButton("Verify Video Stream")
        self.check_button.clicked.connect(self.check_cameras)
        self.check_control_button = QtWidgets.QPushButton("Verify Camera Control")
        self.check_control_button.clicked.connect(lambda _checked=False: self.check_wiris_control())
        self.enable_rtsp_button = QtWidgets.QPushButton("Enable Video Stream")
        self.enable_rtsp_button.clicked.connect(self.enable_wiris_rtsp)
        self.left_stip_button = QtWidgets.QPushButton("STIP 적용")
        self.left_stip_button.clicked.connect(lambda _checked=False: self.apply_wiris_stip("left"))
        self.right_stip_button = QtWidgets.QPushButton("STIP 적용")
        self.right_stip_button.clicked.connect(lambda _checked=False: self.apply_wiris_stip("right"))
        self.camera_connect_button = QtWidgets.QPushButton("Connect Camera Preview")
        self.camera_connect_button.clicked.connect(self.toggle_camera_connection)
        self.detect_pixhawk_host_ip_button = QtWidgets.QPushButton("Discover PC IP")
        self.detect_pixhawk_host_ip_button.clicked.connect(self.discover_pc_pixhawk_ip)
        self.left_mavlink_button = QtWidgets.QPushButton("Connect P1 MAVLink")
        self.left_mavlink_button.clicked.connect(lambda _checked=False: self.toggle_mavlink_connection("left"))
        self.right_mavlink_button = QtWidgets.QPushButton("Connect P2 MAVLink")
        self.right_mavlink_button.clicked.connect(lambda _checked=False: self.toggle_mavlink_connection("right"))
        self.left_mavlink_bootstrap_button = QtWidgets.QPushButton(
            "Start Ethernet MAVLink via USB (P1)"
        )
        self.left_mavlink_bootstrap_button.clicked.connect(
            lambda _checked=False: self.bootstrap_ethernet_mavlink_from_usb("left")
        )
        self.right_mavlink_bootstrap_button = QtWidgets.QPushButton(
            "Start Ethernet MAVLink via USB (P2)"
        )
        self.right_mavlink_bootstrap_button.clicked.connect(
            lambda _checked=False: self.bootstrap_ethernet_mavlink_from_usb("right")
        )
        self.left_px4_shell_button = QtWidgets.QPushButton("P1 Shell")
        self.left_px4_shell_button.setEnabled(False)
        self.left_px4_shell_button.clicked.connect(lambda: self.open_px4_shell("left"))
        self.left_persist_eth_button = QtWidgets.QPushButton("ETH 저장")
        self.left_persist_eth_button.setEnabled(False)
        self.left_persist_eth_button.clicked.connect(lambda: self.apply_persistent_pixhawk_eth("left"))
        self.left_reboot_pixhawk_button = QtWidgets.QPushButton("P1 재부팅")
        self.left_reboot_pixhawk_button.setEnabled(False)
        self.left_reboot_pixhawk_button.clicked.connect(lambda _checked=False: self.reboot_pixhawk_from_mavlink("left"))
        self.right_px4_shell_button = QtWidgets.QPushButton("P2 Shell")
        self.right_px4_shell_button.setEnabled(False)
        self.right_px4_shell_button.clicked.connect(lambda: self.open_px4_shell("right"))
        self.right_persist_eth_button = QtWidgets.QPushButton("ETH 저장")
        self.right_persist_eth_button.setEnabled(False)
        self.right_persist_eth_button.clicked.connect(lambda: self.apply_persistent_pixhawk_eth("right"))
        self.right_reboot_pixhawk_button = QtWidgets.QPushButton("P2 재부팅")
        self.right_reboot_pixhawk_button.setEnabled(False)
        self.right_reboot_pixhawk_button.clicked.connect(lambda _checked=False: self.reboot_pixhawk_from_mavlink("right"))
        self.left_pixhawk_eth_ip_edit = QtWidgets.QLineEdit(self.settings.left_pixhawk_eth_ip)
        self.right_pixhawk_eth_ip_edit = QtWidgets.QLineEdit(self.settings.right_pixhawk_eth_ip)
        self.left_mavlink_url_edit = QtWidgets.QLineEdit(self.settings.left_mavlink_url)
        self.right_mavlink_url_edit = QtWidgets.QLineEdit(self.settings.right_mavlink_url)
        self.left_mavlink_baud_spin = self._build_mavlink_baud_spin()
        self.left_mavlink_baud_spin.setValue(self.settings.left_mavlink_baud)
        self.right_mavlink_baud_spin = self._build_mavlink_baud_spin()
        self.right_mavlink_baud_spin.setValue(self.settings.right_mavlink_baud)
        self.left_pixhawk_bootstrap_url_edit = QtWidgets.QLineEdit(self.settings.left_pixhawk_bootstrap_url)
        self.right_pixhawk_bootstrap_url_edit = QtWidgets.QLineEdit(self.settings.right_pixhawk_bootstrap_url)
        self.left_pixhawk_bootstrap_baud_spin = self._build_mavlink_baud_spin()
        self.left_pixhawk_bootstrap_baud_spin.setValue(self.settings.left_pixhawk_bootstrap_baud)
        self.right_pixhawk_bootstrap_baud_spin = self._build_mavlink_baud_spin()
        self.right_pixhawk_bootstrap_baud_spin.setValue(self.settings.right_pixhawk_bootstrap_baud)
        self.mavlink_configure_streams_checkbox = self._build_toggle_button(
            "MAVLink stream 설정",
            self.settings.mavlink_configure_streams,
        )
        self.mavlink_stream_rate_spin = QtWidgets.QSpinBox()
        self.mavlink_stream_rate_spin.setRange(1_000, 10_000_000)
        self.mavlink_stream_rate_spin.setValue(self.settings.mavlink_stream_rate)
        self.pixhawk_ethernet_mode_combo = QtWidgets.QComboBox()
        self.pixhawk_ethernet_mode_combo.addItem("MAVLink만", "mavlink")
        mode_index = self.pixhawk_ethernet_mode_combo.findData(
            self.settings.pixhawk_ethernet_mode
        )
        self.pixhawk_ethernet_mode_combo.setCurrentIndex(max(0, mode_index))
        self.pixhawk_ethernet_mode_combo.currentIndexChanged.connect(
            lambda _index: self._refresh_pixhawk_ethernet_mode_controls()
        )
        self.browse_weights_button = QtWidgets.QPushButton("파일 선택")
        self.browse_weights_button.clicked.connect(self.browse_yolo_weights)
        self.start_button = QtWidgets.QPushButton("YOLO 측정 시작")
        self.start_button.clicked.connect(self.toggle_yolo_detection)
        self.detect_control_button.setText("Scan Control Port")
        self.right_detect_control_button.setText("Scan Control Port")
        self.detect_rtsp_button.setText("Scan Video URL")
        self.right_detect_rtsp_button.setText("Scan Video URL")
        self.auto_wiris_connect_button.setText("Auto-Connect Cameras")
        self.detect_pc_ip_button.setText("Discover PC IP")
        self.apply_pc_ip_button.setText("Apply PC IP")
        self.apply_candidate_nic_button.setText("Apply Selected NIC")
        self.apply_candidate_left_button.setText("Apply to Left")
        self.apply_candidate_right_button.setText("Apply to Right")
        self.clear_camera_candidate_button.setText("Clear")
        self.left_control_apply_button.setText("Apply Control Port")
        self.right_control_apply_button.setText("Apply Control Port")
        self.left_rtsp_apply_button.setText("Apply Video URL")
        self.right_rtsp_apply_button.setText("Apply Video URL")
        self.pixhawk_usb_discover_button.setText("Scan USB Pixhawks")
        self.pixhawk_mavlink_url_candidate_button.setText("Generate Ethernet MAVLink Candidates")
        self.detect_pixhawk_host_ip_button.setText("Discover PC IP")
        self.clear_pixhawk_candidate_button.setText("Clear")
        self.apply_pixhawk_candidate_host_button.setText("Apply to Host")
        self.apply_pixhawk_candidate_left_button.setText("Apply to P1")
        self.apply_pixhawk_candidate_right_button.setText("Apply to P2")
        self.swap_camera_button.setText("Swap Camera Sides")
        self.swap_pixhawk_button.setText("Swap P1 / P2 Roles")
        self.check_button.setText("Verify Video Stream")
        self.check_control_button.setText("Verify Camera Control")
        self.enable_rtsp_button.setText("Enable Video Stream")
        self.camera_connect_button.setText("Connect Camera Preview")
        self.left_mavlink_button.setText("Connect P1 MAVLink")
        self.right_mavlink_button.setText("Connect P2 MAVLink")
        self.left_mavlink_bootstrap_button.setText("Start Ethernet MAVLink via USB (P1)")
        self.right_mavlink_bootstrap_button.setText("Start Ethernet MAVLink via USB (P2)")
        self.start_button.setText("YOLO 측정 시작")
        self.left_stip_button.setText("STIP 적용")
        self.right_stip_button.setText("STIP 적용")
        self.left_px4_shell_button.setText("P1 Shell")
        self.right_px4_shell_button.setText("P2 Shell")
        self.left_persist_eth_button.setText("ETH 저장")
        self.right_persist_eth_button.setText("ETH 저장")
        self.left_reboot_pixhawk_button.setText("P1 재부팅")
        self.right_reboot_pixhawk_button.setText("P2 재부팅")
        self.browse_weights_button.setText("파일 선택")
        self._refresh_yolo_device_toggle()

        for widget, width in (
            (self.interface_edit, NETWORK_FIELD_MIN_WIDTHS["interface"]),
            (self.host_address_edit, NETWORK_FIELD_MIN_WIDTHS["host"]),
            (self.zoom_port_spin, NETWORK_FIELD_MIN_WIDTHS["port"]),
            (self.left_control_port_spin, NETWORK_FIELD_MIN_WIDTHS["port"]),
            (self.right_control_port_spin, NETWORK_FIELD_MIN_WIDTHS["port"]),
            (self.zoom_timeout_spin, NETWORK_FIELD_MIN_WIDTHS["timeout"]),
            (self.stip_mask_edit, NETWORK_FIELD_MIN_WIDTHS["mask"]),
        ):
            widget.setProperty("compactNetworkField", True)
            widget.setMinimumWidth(width)
            widget.setSizePolicy(QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Fixed)

        for label_text, widget, label_column, field_column in (
            ("인터페이스", self.interface_edit, 0, 1),
            ("PC IP", self.host_address_edit, 2, 3),
            ("스캔 기본 포트", self.zoom_port_spin, 4, 5),
            ("타임아웃", self.zoom_timeout_spin, 6, 7),
        ):
            label = QtWidgets.QLabel(label_text)
            label.setProperty("compactNetworkLabel", True)
            camera_form.addWidget(label, 0, label_column)
            camera_form.addWidget(widget, 0, field_column)
            camera_form.setColumnStretch(field_column, 1)
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
        candidate_group = QtWidgets.QGroupBox("탐색 후보")
        candidate_layout = QtWidgets.QVBoxLayout(candidate_group)
        candidate_layout.addWidget(self.camera_candidate_table)
        candidate_actions = QtWidgets.QHBoxLayout()
        for button in (
            self.apply_candidate_nic_button,
            self.apply_candidate_left_button,
            self.apply_candidate_right_button,
            self.clear_camera_candidate_button,
        ):
            candidate_actions.addWidget(button, 1)
        candidate_layout.addLayout(candidate_actions)
        verification_group = QtWidgets.QGroupBox("검증 / 활성화")
        verification_actions = QtWidgets.QHBoxLayout(verification_group)
        verification_actions.setSpacing(6)
        for button in (self.check_control_button, self.enable_rtsp_button, self.check_button, self.camera_connect_button):
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
        camera_layout.addWidget(self.connection_sequence_label)
        camera_layout.addWidget(wiris_pair_group)
        camera_layout.addWidget(candidate_group)
        camera_layout.addWidget(verification_group)
        camera_layout.addWidget(auto_group)
        camera_layout.addWidget(advanced_ip_group)
        self.camera_connection_label = QtWidgets.QLabel(STARTUP_CAMERA_STATUS)
        self.camera_connection_label.setObjectName("statusStrip")
        camera_layout.addWidget(self.camera_connection_label)

        weights_row = QtWidgets.QHBoxLayout()
        weights_row.addWidget(self.weights_edit, 1)
        weights_row.addWidget(self.browse_weights_button)
        yolo_form.addWidget(QtWidgets.QLabel("가중치"), 0, 0)
        yolo_form.addLayout(weights_row, 0, 1, 1, 5)
        yolo_form.addWidget(QtWidgets.QLabel("추론 장치"), 1, 0)
        yolo_form.addWidget(self.device_combo, 1, 1, 1, 5)
        yolo_advanced_group = QtWidgets.QGroupBox("고급 스트림 / YOLO 설정")
        yolo_advanced_group.setObjectName("yoloAdvancedSettingsGroup")
        yolo_advanced_layout = QtWidgets.QVBoxLayout(yolo_advanced_group)
        yolo_param_grid = QtWidgets.QGridLayout()
        yolo_param_grid.setHorizontalSpacing(8)
        yolo_param_grid.setVerticalSpacing(6)
        for index, (label_text, widget) in enumerate(
            (
                ("입력 크기(px)", self.input_size_spin),
                ("신뢰도 임계값", self.conf_spin),
                ("IoU 임계값", self.iou_spin),
                ("미리보기 폭(px)", self.preview_width_spin),
                ("RTP 지연(ms)", self.rtsp_latency_spin),
                ("Event 큐", self.yolo_event_queue_size_spin),
                ("Preview 큐", self.yolo_frame_queue_size_spin),
                ("디코더", self.decoder_combo),
                ("수신 백엔드", self.stream_backend_combo),
                ("RTSP 전송", self.rtsp_transport_combo),
            )
        ):
            row = index // 3
            label_column = (index % 3) * 2
            field_column = label_column + 1
            label = QtWidgets.QLabel(label_text)
            label.setBuddy(widget)
            label.setProperty("compactAdvancedLabel", True)
            widget.setAccessibleName(label_text)
            yolo_param_grid.addWidget(label, row, label_column)
            yolo_param_grid.addWidget(widget, row, field_column)
            yolo_param_grid.setColumnStretch(field_column, 1)
        yolo_advanced_layout.addLayout(yolo_param_grid)
        yolo_form.addWidget(yolo_advanced_group, 2, 0, 1, 6)

        yolo_gate_group = QtWidgets.QGroupBox("측정 Gate")
        yolo_gate_layout = QtWidgets.QGridLayout(yolo_gate_group)
        yolo_gate_layout.setHorizontalSpacing(8)
        yolo_gate_layout.setVerticalSpacing(6)
        yolo_gate_layout.setColumnStretch(1, 1)
        yolo_gate_layout.setColumnStretch(3, 1)
        yolo_gate_layout.addWidget(self.yolo_gate_class_checkbox, 0, 0)
        yolo_gate_layout.addWidget(self.yolo_gate_class_edit, 0, 1, 1, 3)
        yolo_gate_layout.addWidget(self.yolo_gate_matching_checkbox, 1, 0)
        yolo_gate_layout.addWidget(self.yolo_gate_time_basis_combo, 1, 1)
        yolo_gate_layout.addWidget(self.yolo_gate_time_delta_checkbox, 1, 2)
        yolo_gate_layout.addWidget(self.yolo_gate_time_delta_spin, 1, 3)
        yolo_gate_layout.addWidget(self.yolo_gate_vertical_delta_checkbox, 1, 4)
        yolo_gate_layout.addWidget(self.yolo_gate_vertical_delta_spin, 1, 5)
        self.yolo_gate_status_label = QtWidgets.QLabel("Gate OFF: 기존 YOLO 후보 선택/매칭을 사용합니다.")
        self.yolo_gate_status_label.setObjectName("statusStrip")
        self.yolo_gate_status_label.setWordWrap(True)
        yolo_gate_layout.addWidget(self.yolo_gate_status_label, 2, 0, 1, 7)
        yolo_form.addWidget(yolo_gate_group, 3, 0, 1, 6)
        for checkbox in (
            self.yolo_gate_class_checkbox,
            self.yolo_gate_matching_checkbox,
            self.yolo_gate_time_delta_checkbox,
            self.yolo_gate_vertical_delta_checkbox,
        ):
            checkbox.toggled.connect(lambda _checked=False: self._refresh_yolo_gate_controls())
            checkbox.toggled.connect(lambda _checked=False: self._refresh_distance())
        self.yolo_gate_vertical_delta_spin.valueChanged.connect(lambda _value: self._refresh_distance())
        self.yolo_gate_time_delta_spin.valueChanged.connect(lambda _value: self._refresh_distance())
        self.yolo_gate_time_basis_combo.currentIndexChanged.connect(lambda _index: self._refresh_distance())
        self._refresh_yolo_gate_controls()

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
        self.camera_yolo_log = QtWidgets.QPlainTextEdit()
        self.camera_yolo_log.setReadOnly(True)
        self.camera_yolo_log.setMaximumBlockCount(600)
        yolo_splitter = QtWidgets.QSplitter(QtCore.Qt.Vertical)
        yolo_splitter.addWidget(yolo_group)
        yolo_splitter.addWidget(self.camera_yolo_log)
        yolo_splitter.setStretchFactor(0, 2)
        yolo_splitter.setStretchFactor(1, 1)
        yolo_splitter.setChildrenCollapsible(False)
        yolo_splitter.setSizes([360, 220])
        self.yolo_log_splitter = yolo_splitter
        yolo_layout_outer.addWidget(yolo_splitter, 1)
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

        stereo = QtWidgets.QGroupBox("3D 거리 계산 설정")
        stereo_form = QtWidgets.QGridLayout(stereo)
        stereo_form.setHorizontalSpacing(8)
        stereo_form.setColumnStretch(1, 1)
        stereo_form.setColumnStretch(3, 1)
        stereo_form.setColumnStretch(5, 1)
        self.focal_spin = QtWidgets.QDoubleSpinBox()
        self.focal_spin.setRange(1.0, MAX_MANUAL_FOCAL_PX)
        self.focal_spin.setValue(900.0)
        self.focal_spin.setSuffix(" px")
        self.focal_spin.valueChanged.connect(lambda _value: self._refresh_camera_parameters_tab())
        self.manual_focal_checkbox = self._build_toggle_button(
            "수동 초점거리 사용",
            self.settings.manual_focal_override,
        )
        self.manual_focal_checkbox.setToolTip(
            "체커보드가 없을 때 추정 fx(px)를 직접 입력합니다.\n"
            "체커보드 보정값을 쓰려면 이 옵션을 끄고 보정 결과를 적용하세요.\n\n"
            + WIRIS_VISIBLE_ZOOM_FX_TOOLTIP
        )
        self.manual_focal_checkbox.toggled.connect(self._set_focal_override_enabled)
        self.intrinsics_source_combo = QtWidgets.QComboBox()
        self.intrinsics_source_combo.addItem("체커보드 보정값", "checkerboard")
        self.intrinsics_source_combo.addItem("수동 fallback", "manual")
        self._set_combo_data(
            self.intrinsics_source_combo,
            "manual" if self.settings.manual_focal_override else "checkerboard",
        )
        self.intrinsics_source_combo.currentIndexChanged.connect(self._handle_intrinsics_source_changed)
        self.intrinsics_status_label = QtWidgets.QLabel("내부 파라미터: 대기 중")
        self.intrinsics_status_label.setObjectName("statusStrip")
        self.intrinsics_status_label.setWordWrap(True)
        self.baseline_spin = QtWidgets.QDoubleSpinBox()
        self.baseline_spin.setRange(0.001, 100.0)
        self.baseline_spin.setDecimals(3)
        self.baseline_spin.setValue(0.300)
        self.baseline_spin.setSuffix(" m")
        self.baseline_spin.valueChanged.connect(lambda _value: self._refresh_camera_parameters_tab())
        self.gps_baseline_spins: dict[str, dict[str, QtWidgets.QDoubleSpinBox]] = {}
        self.distance_label = QtWidgets.QLabel("거리: 좌/우 검출 매칭 대기 중")
        self.distance_label.setObjectName("distanceStatus")
        self.distance_label.setWordWrap(True)
        self.distance_mode_combo = QtWidgets.QComboBox()
        self.distance_mode_combo.setObjectName("distanceModeCombo")
        self.distance_mode_combo.setAccessibleName("외부 파라미터 계산 방식")
        self.distance_mode_combo.setToolTip(
            "3D ray 거리 계산에 사용할 외부 파라미터 출처를 고릅니다.\n"
            "Pixhawk 자세 + GPS/RTK baseline 또는 체커보드 stereo R/T를 사용합니다."
        )
        self.distance_mode_combo.addItem("Pixhawk 자세 + GPS/RTK baseline", "pixhawk")
        self.distance_mode_combo.addItem("체커보드 Stereo 외부 파라미터", "checkerboard")
        initial_distance_mode = str(self.settings.triangulation_extrinsics_source or "pixhawk")
        if initial_distance_mode == "calibration":
            initial_distance_mode = "checkerboard"
        if initial_distance_mode not in {"pixhawk", "checkerboard"}:
            initial_distance_mode = "pixhawk"
        self.distance_mode_combo.setCurrentIndex(self.distance_mode_combo.findData(initial_distance_mode))
        self._distance_mode_last_source = initial_distance_mode
        self.distance_mode_combo.currentIndexChanged.connect(self._handle_distance_mode_changed)
        self.distance_parameters_label = QtWidgets.QLabel("계산 파라미터: 대기 중")
        self.distance_parameters_label.setObjectName("statusStrip")
        self.distance_parameters_label.setWordWrap(True)
        self.camera_sync_history_window_spin = QtWidgets.QDoubleSpinBox()
        self.camera_sync_history_window_spin.setRange(0.1, 5.0)
        self.camera_sync_history_window_spin.setSingleStep(0.1)
        self.camera_sync_history_window_spin.setDecimals(1)
        self.camera_sync_history_window_spin.setValue(self.settings.camera_sync_history_window_s)
        self.camera_sync_history_window_spin.setSuffix(" s")
        self.camera_sync_receive_pair_max_delta_spin = QtWidgets.QDoubleSpinBox()
        self.camera_sync_receive_pair_max_delta_spin.setRange(0.0, 2000.0)
        self.camera_sync_receive_pair_max_delta_spin.setSingleStep(25.0)
        self.camera_sync_receive_pair_max_delta_spin.setDecimals(0)
        self.camera_sync_receive_pair_max_delta_spin.setValue(
            self.settings.camera_sync_receive_pair_max_delta_s * 1000.0
        )
        self.camera_sync_receive_pair_max_delta_spin.setSuffix(" ms")
        self.camera_sync_rtcp_pair_max_delta_spin = QtWidgets.QDoubleSpinBox()
        self.camera_sync_rtcp_pair_max_delta_spin.setRange(0.0, 1000.0)
        self.camera_sync_rtcp_pair_max_delta_spin.setSingleStep(5.0)
        self.camera_sync_rtcp_pair_max_delta_spin.setDecimals(0)
        self.camera_sync_rtcp_pair_max_delta_spin.setValue(
            self.settings.camera_sync_rtcp_pair_max_delta_s * 1000.0
        )
        self.camera_sync_rtcp_pair_max_delta_spin.setSuffix(" ms")
        self.camera_sync_max_sr_age_spin = QtWidgets.QDoubleSpinBox()
        self.camera_sync_max_sr_age_spin.setRange(0.1, 60.0)
        self.camera_sync_max_sr_age_spin.setSingleStep(0.5)
        self.camera_sync_max_sr_age_spin.setDecimals(1)
        self.camera_sync_max_sr_age_spin.setValue(self.settings.camera_sync_max_sr_age_s)
        self.camera_sync_max_sr_age_spin.setSuffix(" s")
        self.camera_sync_max_pc_offset_spin = QtWidgets.QDoubleSpinBox()
        self.camera_sync_max_pc_offset_spin.setRange(0.0, 5000.0)
        self.camera_sync_max_pc_offset_spin.setSingleStep(10.0)
        self.camera_sync_max_pc_offset_spin.setDecimals(0)
        self.camera_sync_max_pc_offset_spin.setValue(self.settings.camera_sync_max_pc_offset_s * 1000.0)
        self.camera_sync_max_pc_offset_spin.setSuffix(" ms")
        self.camera_sync_clock_rate_error_spin = QtWidgets.QDoubleSpinBox()
        self.camera_sync_clock_rate_error_spin.setRange(0.0, 10.0)
        self.camera_sync_clock_rate_error_spin.setSingleStep(0.1)
        self.camera_sync_clock_rate_error_spin.setDecimals(2)
        self.camera_sync_clock_rate_error_spin.setValue(self.settings.camera_sync_max_clock_rate_error * 100.0)
        self.camera_sync_clock_rate_error_spin.setSuffix(" %")
        self.camera_sync_required_sr_count_spin = QtWidgets.QSpinBox()
        self.camera_sync_required_sr_count_spin.setRange(1, 20)
        self.camera_sync_required_sr_count_spin.setValue(self.settings.camera_sync_required_sr_count)
        self.camera_sync_required_sr_count_spin.setSuffix(" 회")
        self.camera_sync_warmup_timeout_spin = QtWidgets.QDoubleSpinBox()
        self.camera_sync_warmup_timeout_spin.setRange(1.0, 120.0)
        self.camera_sync_warmup_timeout_spin.setSingleStep(1.0)
        self.camera_sync_warmup_timeout_spin.setDecimals(0)
        self.camera_sync_warmup_timeout_spin.setValue(self.settings.camera_sync_warmup_timeout_s)
        self.camera_sync_warmup_timeout_spin.setSuffix(" s")
        self.camera_sync_settings_group = QtWidgets.QGroupBox("카메라 RTCP 동기화 Gate")
        self.camera_sync_settings_group.setObjectName("cameraSyncSettingsGroup")
        camera_sync_settings_form = QtWidgets.QGridLayout(self.camera_sync_settings_group)
        camera_sync_settings_form.setHorizontalSpacing(8)
        camera_sync_settings_form.setVerticalSpacing(6)
        camera_sync_settings_form.setColumnStretch(1, 1)
        camera_sync_settings_form.setColumnStretch(3, 1)
        camera_sync_fields = (
            ("Frame history", self.camera_sync_history_window_spin),
            ("수신 skew 허용", self.camera_sync_receive_pair_max_delta_spin),
            ("RTCP/NTP Δt", self.camera_sync_rtcp_pair_max_delta_spin),
            ("SR stale", self.camera_sync_max_sr_age_spin),
            ("SR-PC offset", self.camera_sync_max_pc_offset_spin),
            ("Clock rate 오차", self.camera_sync_clock_rate_error_spin),
            ("최소 SR 수", self.camera_sync_required_sr_count_spin),
            ("Warmup timeout", self.camera_sync_warmup_timeout_spin),
        )
        for index, (label_text, widget) in enumerate(camera_sync_fields):
            row = index // 2
            column = (index % 2) * 2
            label = QtWidgets.QLabel(label_text)
            label.setBuddy(widget)
            widget.setAccessibleName(label_text)
            camera_sync_settings_form.addWidget(label, row, column)
            camera_sync_settings_form.addWidget(widget, row, column + 1)
        self.current_parameters_group = QtWidgets.QGroupBox("현재 적용값")
        current_parameters_layout = QtWidgets.QVBoxLayout(self.current_parameters_group)
        current_parameters_layout.addWidget(self.distance_parameters_label)

        self.intrinsics_settings_group = QtWidgets.QGroupBox("기본: 내부 파라미터")
        intrinsics_form = QtWidgets.QGridLayout(self.intrinsics_settings_group)
        intrinsics_form.setHorizontalSpacing(8)
        intrinsics_form.setColumnStretch(1, 1)
        self.manual_focal_label = QtWidgets.QLabel("초점거리")
        intrinsics_form.addWidget(QtWidgets.QLabel("출처"), 0, 0)
        intrinsics_form.addWidget(self.intrinsics_source_combo, 0, 1)
        intrinsics_form.addWidget(self.manual_focal_label, 1, 0)
        intrinsics_form.addWidget(self.focal_spin, 1, 1)
        intrinsics_form.addWidget(self.intrinsics_status_label, 2, 0, 1, 2)
        self.manual_focal_checkbox.setVisible(False)
        self._configure_collapsible_group(self.intrinsics_settings_group, checked=True)

        self.extrinsics_settings_group = QtWidgets.QGroupBox("기본: 외부 파라미터 / Baseline")
        extrinsics_form = QtWidgets.QGridLayout(self.extrinsics_settings_group)
        extrinsics_form.setHorizontalSpacing(8)
        extrinsics_form.setColumnStretch(1, 1)
        self.distance_mode_label = QtWidgets.QLabel("계산 방식")
        self.baseline_label = QtWidgets.QLabel("Baseline")
        self.mount_assumption_label = QtWidgets.QLabel(
            "장착 가정: GPS/RTK baseline only. "
            "수동 baseline 입력은 사용하지 않고 좌/우 Pixhawk GPS baseline과 자세만 계산에 사용합니다."
        )
        self.mount_assumption_label.setObjectName("statusStrip")
        self.mount_assumption_label.setWordWrap(True)
        extrinsics_form.addWidget(self.distance_mode_label, 0, 0)
        extrinsics_form.addWidget(self.distance_mode_combo, 0, 1)
        self.baseline_label.setVisible(False)
        self.baseline_spin.setVisible(False)
        extrinsics_form.addWidget(self.mount_assumption_label, 1, 0, 1, 2)

        stereo_parameter_splitter = QtWidgets.QSplitter(QtCore.Qt.Horizontal)
        stereo_parameter_splitter.setObjectName("stereoParameterSplitter")
        stereo_parameter_splitter.setChildrenCollapsible(False)
        stereo_parameter_splitter.setHandleWidth(8)
        stereo_parameter_splitter.setOpaqueResize(True)
        self.stereo_parameter_splitter = stereo_parameter_splitter

        intrinsics_panel = QtWidgets.QWidget()
        intrinsics_panel_layout = QtWidgets.QVBoxLayout(intrinsics_panel)
        intrinsics_panel_layout.setContentsMargins(0, 0, 0, 0)
        intrinsics_panel_layout.setSpacing(0)
        intrinsics_panel_layout.addWidget(self.intrinsics_settings_group)

        extrinsics_panel = QtWidgets.QWidget()
        extrinsics_panel_layout = QtWidgets.QVBoxLayout(extrinsics_panel)
        extrinsics_panel_layout.setContentsMargins(0, 0, 0, 0)
        extrinsics_panel_layout.setSpacing(0)
        extrinsics_panel_layout.addWidget(self.extrinsics_settings_group)

        stereo_parameter_splitter.addWidget(intrinsics_panel)
        stereo_parameter_splitter.addWidget(extrinsics_panel)
        stereo_parameter_splitter.setStretchFactor(0, 1)
        stereo_parameter_splitter.setStretchFactor(1, 1)
        stereo_parameter_splitter.setSizes([1, 1])

        stereo_form.addWidget(self.current_parameters_group, 0, 0, 1, 6)
        stereo_form.addWidget(stereo_parameter_splitter, 1, 0, 1, 6)
        stereo_form.addWidget(self.distance_label, 2, 0, 1, 6)
        for widget in (
            self.camera_sync_history_window_spin,
            self.camera_sync_receive_pair_max_delta_spin,
            self.camera_sync_rtcp_pair_max_delta_spin,
            self.camera_sync_max_sr_age_spin,
            self.camera_sync_max_pc_offset_spin,
            self.camera_sync_clock_rate_error_spin,
            self.camera_sync_required_sr_count_spin,
            self.camera_sync_warmup_timeout_spin,
        ):
            widget.valueChanged.connect(lambda _value: self._refresh_distance())
        stereo_layout.addWidget(stereo)
        stereo_layout.addStretch(1)

        self.left_gps_label = QtWidgets.QLabel("픽스호크 1 GPS/RTK: 미연결")
        self.right_gps_label = QtWidgets.QLabel("픽스호크 2 GPS/RTK: 미연결")
        self.rtk_baseline_label = QtWidgets.QLabel("GPS baseline: 대기 중")
        self.rtk_fixed_status_label = QtWidgets.QLabel("RTK Fixed: 좌/우 GPS 대기 중")
        self.rtk_summary_labels: dict[str, QtWidgets.QLabel] = {}
        for key, text in (
            ("baseline", "Baseline: 대기 중"),
            ("camera", "Camera: 자세 대기 중"),
            ("status", "Status: GPS 대기 중"),
        ):
            label = QtWidgets.QLabel(text)
            label.setObjectName("statChip")
            label.setWordWrap(True)
            self.rtk_summary_labels[key] = label
        for rtk_label in (self.left_gps_label, self.right_gps_label, self.rtk_baseline_label, self.rtk_fixed_status_label):
            rtk_label.setObjectName("statusStrip")
            rtk_label.setWordWrap(True)
        setup_layout.addWidget(connection_tabs, 1)
        pixhawk_tabs = QtWidgets.QTabWidget()
        pixhawk_summary_tab = QtWidgets.QWidget()
        QtWidgets.QVBoxLayout(pixhawk_summary_tab)
        pixhawk_1_tab = QtWidgets.QWidget()
        QtWidgets.QVBoxLayout(pixhawk_1_tab)
        pixhawk_2_tab = QtWidgets.QWidget()
        QtWidgets.QVBoxLayout(pixhawk_2_tab)
        pixhawk_attitude_tab = QtWidgets.QWidget()
        QtWidgets.QVBoxLayout(pixhawk_attitude_tab)
        custom_watch_tab = QtWidgets.QWidget()
        custom_watch_layout = QtWidgets.QVBoxLayout(custom_watch_tab)
        pixhawk_tabs.addTab(pixhawk_summary_tab, "요약")
        pixhawk_tabs.addTab(pixhawk_1_tab, "픽스호크 1")
        pixhawk_tabs.addTab(pixhawk_2_tab, "픽스호크 2")
        self.pixhawk_tabs = pixhawk_tabs
        advanced_tabs.addTab(wiris_command_panel, "WIRIS SDK")
        advanced_tabs.addTab(custom_watch_tab, "필드 감시")
        advanced_tabs.addTab(baseline_field_browser_tab, "Baseline 필드")
        advanced_tabs.addTab(px4_mag_advanced_tab, "PX4 MAG")
        self._handle_alignment_step_changed(alignment_tabs.currentIndex())
        self.advanced_wiris_command_tab_index = advanced_tabs.indexOf(wiris_command_panel)
        self.advanced_custom_watch_tab_index = advanced_tabs.indexOf(custom_watch_tab)
        self.advanced_baseline_field_tab_index = advanced_tabs.indexOf(baseline_field_browser_tab)
        self.advanced_px4_mag_tab_index = advanced_tabs.indexOf(px4_mag_advanced_tab)
        self.wiris_command_tab_index = self.advanced_wiris_command_tab_index

        custom_watch_group = QtWidgets.QGroupBox("사용자 지정 MAVLink 감시")
        custom_watch_group_layout = QtWidgets.QVBoxLayout(custom_watch_group)
        custom_watch_form = QtWidgets.QGridLayout()
        self.custom_watch_name_edit = QtWidgets.QLineEdit("local_z")
        self.custom_watch_message_edit = QtWidgets.QLineEdit("LOCAL_POSITION_NED")
        self.custom_watch_field_edit = QtWidgets.QLineEdit("z")
        self.custom_watch_add_button = QtWidgets.QPushButton("감시 추가")
        self.custom_watch_add_button.clicked.connect(self.add_custom_mavlink_watch_from_ui)
        self.custom_watch_mavlink_discover_button = QtWidgets.QPushButton("MAVLink 필드 찾기")
        self.custom_watch_mavlink_discover_button.clicked.connect(self.discover_custom_mavlink_watch)
        self.custom_watch_remove_combo = QtWidgets.QComboBox()
        self.custom_watch_remove_combo.setMinimumWidth(260)
        self.custom_watch_remove_button = QtWidgets.QPushButton("감시 제거")
        self.custom_watch_remove_button.clicked.connect(self.remove_selected_custom_topic_watch)
        custom_watch_form.addWidget(QtWidgets.QLabel("이름"), 0, 0)
        custom_watch_form.addWidget(self.custom_watch_name_edit, 0, 1)
        custom_watch_form.addWidget(QtWidgets.QLabel("MAVLink message"), 1, 0)
        custom_watch_form.addWidget(self.custom_watch_message_edit, 1, 1)
        custom_watch_form.addWidget(QtWidgets.QLabel("필드"), 2, 0)
        custom_watch_form.addWidget(self.custom_watch_field_edit, 2, 1)
        custom_watch_buttons = QtWidgets.QHBoxLayout()
        custom_watch_buttons.addWidget(self.custom_watch_mavlink_discover_button)
        custom_watch_buttons.addWidget(self.custom_watch_add_button)
        custom_watch_buttons.addWidget(self.custom_watch_remove_combo)
        custom_watch_buttons.addWidget(self.custom_watch_remove_button)
        custom_watch_buttons.addStretch(1)
        custom_watch_form.addLayout(custom_watch_buttons, 3, 0, 1, 2)
        custom_watch_group_layout.addLayout(custom_watch_form)
        self.custom_watch_plot = TelemetryHistoryPlot("사용자 지정 MAVLink 숫자 필드", "값")
        self.custom_watch_plot.setMinimumHeight(360)
        custom_watch_group_layout.addWidget(self.custom_watch_plot, 1)
        custom_watch_layout.addWidget(custom_watch_group, 1)

        self.mag_sensor_labels: dict[str, dict[str, QtWidgets.QLabel]] = {"left": {}, "right": {}}
        self.gps_sensor_labels: dict[str, dict[str, QtWidgets.QLabel]] = {"left": {}, "right": {}}
        self.mag_sensor_plots: dict[str, TelemetryHistoryPlot] = {}
        self.gps_sensor_plots: dict[str, TelemetryHistoryPlot] = {}
        self.pixhawk_sensor_summary_labels: dict[str, dict[str, QtWidgets.QLabel]] = {"left": {}, "right": {}}
        self.rm3100_slot_labels: dict[str, dict[str, QtWidgets.QLabel]] = {"left": {}, "right": {}}
        self.f9p_slot_labels: dict[str, dict[str, QtWidgets.QLabel]] = {"left": {}, "right": {}}
        self.rm3100_compare_labels: dict[str, QtWidgets.QLabel] = {}
        self.f9p_compare_labels: dict[str, QtWidgets.QLabel] = {}
        self.gnss_heading_labels: dict[str, QtWidgets.QLabel] = {}
        self.rm3100_norm_plots: dict[str, TelemetryHistoryPlot] = {}
        self.rm3100_delta_plots: dict[str, TelemetryHistoryPlot] = {}
        self.f9p_sats_plots: dict[str, TelemetryHistoryPlot] = {}
        self.f9p_quality_plots: dict[str, TelemetryHistoryPlot] = {}
        self._build_baseline_field_browser_view(baseline_field_browser_layout)
        self._init_baseline_alignment_state()
        self._build_alignment_sensor_recognition_tab(alignment_overview_layout)
        self._build_alignment_source_quality_tab(rtk_layout)
        self._build_gnss_base_tab(gnss_base_layout)
        self._build_alignment_magnetometer_tab(
            mag_management_layout,
            include_manager=True,
            include_plot=False,
            expand_manager=True,
        )
        self._build_px4_mag_detail_panel(px4_mag_advanced_layout)
        self._build_baseline_calculation_view(alignment_mag_layout, include_source_controls=False, include_alignment_plot=False)
        self._build_baseline_attitude_alignment_tab(alignment_attitude_layout)
        self._build_optical_alignment_tab(optical_alignment_layout)
        self.refresh_baseline_field_browser()
        self._refresh_baseline_alignment_ui()

        zoom_group = QtWidgets.QGroupBox("WIRIS 줌 명령 제어")
        zoom_group.setObjectName("checkerboardZoomControlGroup")
        zoom_group_layout = QtWidgets.QVBoxLayout(zoom_group)
        self.zoom_level_spin = QtWidgets.QSpinBox()
        self.zoom_level_spin.setRange(MIN_ZOOM_LEVEL, MAX_ZOOM_LEVEL)
        self.zoom_level_spin.setValue(self.settings.calibration_zoom_level)
        self.zoom_level_spin.valueChanged.connect(lambda _value: self._refresh_zoom_intrinsics_tab())
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
        self.zoom_in_button.clicked.connect(lambda _checked=False: self.send_zoom(self.selected_zoom_target(), "in"))
        self.zoom_out_button.clicked.connect(lambda _checked=False: self.send_zoom(self.selected_zoom_target(), "out"))
        self.zoom_stop_button.clicked.connect(lambda _checked=False: self.send_zoom(self.selected_zoom_target(), "stop"))
        self.zoom_set_button.clicked.connect(lambda _checked=False: self.send_zoom(self.selected_zoom_target(), "set"))
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
        self._configure_collapsible_group(zoom_group, checked=True)

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
        self.wiris_command_combo.currentIndexChanged.connect(self._apply_selected_wiris_command)
        self.wiris_command_edit = QtWidgets.QLineEdit()
        self.wiris_command_edit.setPlaceholderText("SDK 명령 예: HIWS")
        self.wiris_command_terminator_combo = QtWidgets.QComboBox()
        self.wiris_command_terminator_combo.addItem("LINE", LINE_TERMINATOR)
        self.wiris_command_terminator_combo.addItem("NULL", NUL_TERMINATOR)
        self.wiris_command_send_button = QtWidgets.QPushButton("명령 전송")
        self.wiris_command_send_button.clicked.connect(self.send_selected_wiris_command)
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
        wiris_command_layout.addWidget(command_group)
        wiris_command_layout.addWidget(self.wiris_command_log, 1)
        self._apply_selected_wiris_command()

        calibration_group = QtWidgets.QGroupBox("체커보드 캡처")
        calibration_group.setObjectName("checkerboardIntrinsicCalibrationGroup")
        calib_form = QtWidgets.QGridLayout(calibration_group)
        calib_form.setHorizontalSpacing(6)
        calib_form.setVerticalSpacing(6)
        calib_form.setColumnStretch(5, 1)
        self.calib_cols_spin = QtWidgets.QSpinBox()
        self.calib_cols_spin.setRange(2, 30)
        self.calib_cols_spin.setValue(self.settings.calibration_inner_cols)
        self.calib_cols_spin.valueChanged.connect(self._refresh_all_calibration_previews)
        self.calib_rows_spin = QtWidgets.QSpinBox()
        self.calib_rows_spin.setRange(2, 30)
        self.calib_rows_spin.setValue(self.settings.calibration_inner_rows)
        self.calib_rows_spin.valueChanged.connect(self._refresh_all_calibration_previews)
        self.calib_square_spin = QtWidgets.QDoubleSpinBox()
        self.calib_square_spin.setRange(1.0, 500.0)
        self.calib_square_spin.setValue(self.settings.calibration_square_size_mm)
        self.calib_square_spin.setSuffix(" mm")
        self.calib_min_images_spin = QtWidgets.QSpinBox()
        self.calib_min_images_spin.setRange(3, 100)
        self.calib_min_images_spin.setValue(self.settings.calibration_min_images)
        self.calib_calculation_pairs_spin = QtWidgets.QSpinBox()
        self.calib_calculation_pairs_spin.setRange(0, 10000)
        self.calib_calculation_pairs_spin.setValue(self.settings.calibration_calculation_pair_count)
        self.calib_calculation_pairs_spin.setSpecialValueText("전체 pair")
        self.calib_calculation_pairs_spin.setSuffix(" pair")
        for spin, width in (
            (self.calib_cols_spin, 72),
            (self.calib_rows_spin, 72),
            (self.calib_square_spin, 96),
            (self.calib_calculation_pairs_spin, 96),
        ):
            spin.setProperty("compactFormField", True)
            spin.setMinimumWidth(width)
            spin.setSizePolicy(QtWidgets.QSizePolicy.Fixed, QtWidgets.QSizePolicy.Fixed)
        initial_intrinsics = self._intrinsics_storage_base_path()
        calibration_root = self.settings.calibration_root.strip() or str(initial_intrinsics.parent)
        self.calib_dir_edit = QtWidgets.QLineEdit(calibration_root)
        self.calib_dir_edit.setProperty("compactFormField", True)
        self.calib_dir_edit.setMinimumWidth(260)
        self.capture_calibration_button = QtWidgets.QPushButton("Capture Live Pair")
        self.capture_calibration_button.setToolTip(
            "Capture the newest decoded native pair from the live RTSP streams; raw RTP packets are not captured."
        )
        self.capture_calibration_button.clicked.connect(self.capture_calibration_images)
        self.calibration_auto_interval_spin = QtWidgets.QDoubleSpinBox()
        self.calibration_auto_interval_spin.setRange(0.5, 10.0)
        self.calibration_auto_interval_spin.setSingleStep(0.5)
        self.calibration_auto_interval_spin.setValue(self.settings.calibration_auto_interval_s)
        self.calibration_auto_interval_spin.setSuffix(" s")
        self.calibration_auto_target_spin = QtWidgets.QSpinBox()
        self.calibration_auto_target_spin.setRange(1, 200)
        self.calibration_auto_target_spin.setValue(self.settings.calibration_auto_target_pairs)
        self.calibration_auto_target_spin.setSuffix(" pair")
        self.capture_calibration_auto_button = QtWidgets.QPushButton("자동 촬영 시작")
        self.capture_calibration_auto_button.clicked.connect(self.toggle_auto_calibration_capture)
        self.save_intrinsics_button = QtWidgets.QPushButton("저장")
        self.save_intrinsics_button.clicked.connect(self.save_intrinsics)
        self.apply_intrinsics_button = QtWidgets.QPushButton("계산에 적용")
        self.apply_intrinsics_button.clicked.connect(self.apply_intrinsics_to_distance)
        self.calibration_status_label = QtWidgets.QLabel("좌/우 라이브 페어에서 체커보드가 보이는 프레임을 캡처하세요.")
        self.calibration_status_label.setObjectName("statusStrip")
        self.calibration_status_label.setWordWrap(True)
        self.capture_calibration_button.setText("Capture Live Pair")
        self.save_intrinsics_button.setText("저장")
        self.apply_intrinsics_button.setText("계산에 적용")
        self.calibration_result_text = QtWidgets.QPlainTextEdit()
        self.calibration_result_text.setReadOnly(True)
        self.calibration_result_text.setMaximumBlockCount(200)
        calibration_preview_row = QtWidgets.QHBoxLayout()
        self.calibration_left_preview = self._build_calibration_preview_label("left preview")
        self.calibration_right_preview = self._build_calibration_preview_label("right preview")
        calibration_preview_row.addWidget(self.calibration_left_preview, 1)
        calibration_preview_row.addWidget(self.calibration_right_preview, 1)

        compact_calibration_labels = [
            QtWidgets.QLabel(text)
            for text in ("내부 열", "내부 행", "격자 크기", "저장 경로")
        ]
        for label in compact_calibration_labels:
            label.setProperty("compactFormLabel", True)
        cols_label, rows_label, square_label, dir_label = compact_calibration_labels

        calib_form.addWidget(cols_label, 0, 0)
        calib_form.addWidget(self.calib_cols_spin, 0, 1)
        calib_form.addWidget(rows_label, 0, 2)
        calib_form.addWidget(self.calib_rows_spin, 0, 3)
        calib_form.addWidget(square_label, 0, 4)
        calib_form.addWidget(self.calib_square_spin, 0, 5)
        calib_form.addWidget(dir_label, 1, 0)
        calib_form.addWidget(self.calib_dir_edit, 1, 1, 1, 5)
        calib_form.addWidget(self.capture_calibration_button, 1, 6)
        auto_interval_label = QtWidgets.QLabel("자동 간격")
        auto_interval_label.setProperty("compactFormLabel", True)
        auto_target_label = QtWidgets.QLabel("목표 pair")
        auto_target_label.setProperty("compactFormLabel", True)
        calib_form.addWidget(auto_interval_label, 2, 0)
        calib_form.addWidget(self.calibration_auto_interval_spin, 2, 1)
        calib_form.addWidget(auto_target_label, 2, 2)
        calib_form.addWidget(self.calibration_auto_target_spin, 2, 3)
        calib_form.addWidget(self.capture_calibration_auto_button, 2, 4, 1, 3)

        validation_source_group = QtWidgets.QGroupBox("입력 / 계산 목적")
        validation_source_form = QtWidgets.QGridLayout(validation_source_group)
        validation_source_form.setHorizontalSpacing(6)
        validation_source_form.setVerticalSpacing(6)
        validation_source_form.setColumnStretch(1, 1)
        validation_source_form.setColumnStretch(4, 1)
        self.checkerboard_validation_mode_combo = QtWidgets.QComboBox()
        self.checkerboard_validation_mode_combo.addItem("Pixhawk 자세 + GPS/RTK baseline", "pixhawk")
        self.checkerboard_validation_mode_combo.addItem("체커보드 Stereo 외부 파라미터", "checkerboard")
        self._set_combo_data(
            self.checkerboard_validation_mode_combo,
            self.settings.calibration_validation_mode,
        )
        self.checkerboard_validation_mode_combo.setToolTip(
            "Pixhawk 모드는 내부 파라미터만 계산하고, 체커보드 모드는 내부 파라미터와 stereo R/T를 함께 계산합니다."
        )
        validation_root = (
            self.settings.calibration_validation_root.strip()
            or self.calib_dir_edit.text()
        )
        self.checkerboard_validation_root_edit = QtWidgets.QLineEdit(validation_root)
        self.checkerboard_validation_root_edit.setMinimumWidth(260)
        self.checkerboard_validation_choose_root_button = QtWidgets.QPushButton("폴더 선택")
        self.checkerboard_validation_choose_root_button.clicked.connect(self.choose_checkerboard_validation_root)
        self.checkerboard_validation_scan_button = QtWidgets.QPushButton("스캔")
        self.checkerboard_validation_scan_button.clicked.connect(self.scan_checkerboard_validation_files)
        validation_json = (
            self.settings.calibration_validation_json.strip()
            or str(self._intrinsics_storage_path())
        )
        self.checkerboard_validation_json_edit = QtWidgets.QLineEdit(validation_json)
        self.checkerboard_validation_json_edit.setMinimumWidth(260)
        self.checkerboard_validation_choose_json_button = QtWidgets.QPushButton("JSON 선택")
        self.checkerboard_validation_choose_json_button.clicked.connect(self.choose_checkerboard_validation_json)
        self.checkerboard_validation_load_json_button = QtWidgets.QPushButton("기존 JSON 불러오기")
        self.checkerboard_validation_load_json_button.clicked.connect(self.load_checkerboard_validation_json)
        validation_source_form.addWidget(QtWidgets.QLabel("계산 방식"), 0, 0)
        validation_source_form.addWidget(self.checkerboard_validation_mode_combo, 0, 1, 1, 2)
        validation_source_form.addWidget(QtWidgets.QLabel("사진셋 root"), 1, 0)
        validation_source_form.addWidget(self.checkerboard_validation_root_edit, 1, 1, 1, 3)
        validation_source_form.addWidget(self.checkerboard_validation_choose_root_button, 1, 4)
        validation_source_form.addWidget(self.checkerboard_validation_scan_button, 1, 5)
        validation_source_form.addWidget(QtWidgets.QLabel("기존 intrinsics JSON"), 2, 0)
        validation_source_form.addWidget(self.checkerboard_validation_json_edit, 2, 1, 1, 3)
        validation_source_form.addWidget(self.checkerboard_validation_choose_json_button, 2, 4)
        validation_source_form.addWidget(self.checkerboard_validation_load_json_button, 2, 5)
        self.checkerboard_validation_scan_label = QtWidgets.QLabel("스캔 대기 중")
        self.checkerboard_validation_scan_label.setObjectName("statusStrip")
        self.checkerboard_validation_scan_label.setWordWrap(True)
        validation_source_form.addWidget(self.checkerboard_validation_scan_label, 3, 0, 1, 6)
        self.checkerboard_validation_mode_combo.currentIndexChanged.connect(
            lambda _index: self.scan_checkerboard_validation_files()
        )
        self.checkerboard_validation_root_edit.editingFinished.connect(self.scan_checkerboard_validation_files)

        validation_action_group = QtWidgets.QGroupBox("계산 / 검증")
        validation_action_form = QtWidgets.QGridLayout(validation_action_group)
        validation_action_form.setHorizontalSpacing(6)
        validation_action_form.setVerticalSpacing(6)
        validation_action_form.setColumnStretch(9, 1)
        self.calib_min_images_spin.setProperty("compactFormField", True)
        self.calib_min_images_spin.setMinimumWidth(72)
        self.calib_min_images_spin.setSizePolicy(QtWidgets.QSizePolicy.Fixed, QtWidgets.QSizePolicy.Fixed)
        self.calib_calculation_pairs_spin.setProperty("compactFormField", True)
        self.calib_calculation_pairs_spin.setMinimumWidth(96)
        self.calib_calculation_pairs_spin.setSizePolicy(QtWidgets.QSizePolicy.Fixed, QtWidgets.QSizePolicy.Fixed)
        self.checkerboard_validation_random_runs_spin = QtWidgets.QSpinBox()
        self.checkerboard_validation_random_runs_spin.setRange(1, 20)
        self.checkerboard_validation_random_runs_spin.setValue(self.settings.calibration_validation_runs)
        self.checkerboard_validation_random_pair_count_spin = QtWidgets.QSpinBox()
        self.checkerboard_validation_random_pair_count_spin.setRange(0, 10000)
        self.checkerboard_validation_random_pair_count_spin.setValue(self.settings.calibration_validation_pair_count)
        self.checkerboard_validation_random_pair_count_spin.setSpecialValueText("자동")
        self.checkerboard_validation_seed_spin = QtWidgets.QSpinBox()
        self.checkerboard_validation_seed_spin.setRange(0, 2_147_483_647)
        self.checkerboard_validation_seed_spin.setValue(self.settings.calibration_validation_seed)
        self.checkerboard_validation_seed_spin.setSpecialValueText("자동")
        self.run_calibration_button = QtWidgets.QPushButton("전체 사진으로 계산")
        self.run_calibration_button.clicked.connect(self.run_intrinsic_calibration)
        self.checkerboard_validation_random_button = QtWidgets.QPushButton("랜덤 검증")
        self.checkerboard_validation_random_button.clicked.connect(self.run_checkerboard_random_validation)
        self.checkerboard_validation_save_button = QtWidgets.QPushButton("계산 결과 저장")
        self.checkerboard_validation_save_button.clicked.connect(self.save_checkerboard_validation_result)
        self.checkerboard_validation_apply_button = QtWidgets.QPushButton("계산에 적용")
        self.checkerboard_validation_apply_button.clicked.connect(self.apply_checkerboard_validation_result)
        validation_action_form.addWidget(QtWidgets.QLabel("최소 이미지"), 0, 0)
        validation_action_form.addWidget(self.calib_min_images_spin, 0, 1)
        validation_action_form.addWidget(QtWidgets.QLabel("계산 pair"), 0, 2)
        validation_action_form.addWidget(self.calib_calculation_pairs_spin, 0, 3)
        validation_action_form.addWidget(QtWidgets.QLabel("반복"), 0, 4)
        validation_action_form.addWidget(self.checkerboard_validation_random_runs_spin, 0, 5)
        validation_action_form.addWidget(QtWidgets.QLabel("회당 pair"), 0, 6)
        validation_action_form.addWidget(self.checkerboard_validation_random_pair_count_spin, 0, 7)
        validation_action_form.addWidget(QtWidgets.QLabel("seed"), 0, 8)
        validation_action_form.addWidget(self.checkerboard_validation_seed_spin, 0, 9)
        validation_action_form.addWidget(self.run_calibration_button, 1, 0, 1, 2)
        validation_action_form.addWidget(self.checkerboard_validation_random_button, 1, 2, 1, 2)
        validation_action_form.addWidget(self.checkerboard_validation_save_button, 1, 4, 1, 2)
        validation_action_form.addWidget(self.checkerboard_validation_apply_button, 1, 6, 1, 2)
        self.checkerboard_validation_status_label = QtWidgets.QLabel("계산 결과 없음")
        self.checkerboard_validation_status_label.setObjectName("statusStrip")
        self.checkerboard_validation_status_label.setWordWrap(True)
        validation_action_form.addWidget(self.checkerboard_validation_status_label, 2, 0, 1, 10)
        self.calibration_progress_bar = QtWidgets.QProgressBar()
        self.calibration_progress_bar.setRange(0, 100)
        self.calibration_progress_bar.setValue(0)
        self.calibration_progress_bar.setFormat("대기 · %p%")
        self.calibration_progress_bar.setTextVisible(True)
        validation_action_form.addWidget(self.calibration_progress_bar, 3, 0, 1, 10)

        calibration_top_stack = QtWidgets.QVBoxLayout()
        calibration_top_stack.setSpacing(6)
        calibration_top_stack.addWidget(zoom_group)
        calibration_top_stack.addWidget(calibration_group)
        calibration_top_stack.addWidget(self.calibration_status_label)
        checkerboard_calibration_layout.addLayout(calibration_top_stack)
        checkerboard_calibration_layout.addLayout(calibration_preview_row, 2)
        checkerboard_validation_layout.addWidget(validation_source_group)
        checkerboard_validation_layout.addWidget(validation_action_group)
        checkerboard_validation_layout.addWidget(self.calibration_result_text, 1)

        zoom_intrinsics_split = QtWidgets.QHBoxLayout()
        zoom_intrinsics_split.setSpacing(8)
        zoom_intrinsics_view_group = QtWidgets.QGroupBox("내부 파라미터")
        zoom_intrinsics_view_layout = QtWidgets.QHBoxLayout(zoom_intrinsics_view_group)
        self.zoom_left_intrinsics_text = self._build_parameter_text()
        self.zoom_right_intrinsics_text = self._build_parameter_text()
        zoom_intrinsics_view_layout.addWidget(self.zoom_left_intrinsics_text, 1)
        zoom_intrinsics_view_layout.addWidget(self.zoom_right_intrinsics_text, 1)
        zoom_intrinsics_action_group = QtWidgets.QGroupBox("저장 / 적용")
        zoom_intrinsics_action_layout = QtWidgets.QVBoxLayout(zoom_intrinsics_action_group)
        self.zoom_intrinsics_status_label = QtWidgets.QLabel("줌: --")
        self.zoom_intrinsics_status_label.setObjectName("statusStrip")
        self.zoom_intrinsics_status_label.setWordWrap(True)
        self.zoom_intrinsics_path_label = QtWidgets.QLabel("파일: --")
        self.zoom_intrinsics_path_label.setObjectName("statusStrip")
        self.zoom_intrinsics_path_label.setWordWrap(True)
        zoom_intrinsics_action_layout.addWidget(self.zoom_intrinsics_status_label)
        zoom_intrinsics_action_layout.addWidget(self.zoom_intrinsics_path_label)
        zoom_intrinsics_action_layout.addWidget(self.save_intrinsics_button)
        zoom_intrinsics_action_layout.addWidget(self.apply_intrinsics_button)
        zoom_intrinsics_action_layout.addStretch(1)
        zoom_intrinsics_split.addWidget(zoom_intrinsics_view_group, 1)
        zoom_intrinsics_split.addWidget(zoom_intrinsics_action_group)
        zoom_intrinsics_layout.addLayout(zoom_intrinsics_split, 1)

        zoom_intrinsics_compare_group = QtWidgets.QGroupBox("저장 파일 비교")
        zoom_intrinsics_compare_layout = QtWidgets.QVBoxLayout(zoom_intrinsics_compare_group)
        zoom_intrinsics_compare_actions = QtWidgets.QHBoxLayout()
        self.add_zoom_intrinsics_file_button = QtWidgets.QPushButton("추가")
        self.add_zoom_intrinsics_file_button.clicked.connect(self.add_zoom_intrinsics_comparison_files)
        self.remove_zoom_intrinsics_file_button = QtWidgets.QPushButton("제거")
        self.remove_zoom_intrinsics_file_button.clicked.connect(self.remove_selected_zoom_intrinsics_comparison_files)
        self.apply_zoom_intrinsics_file_button = QtWidgets.QPushButton("선택 적용")
        self.apply_zoom_intrinsics_file_button.clicked.connect(self.apply_selected_zoom_intrinsics_comparison_file)
        zoom_intrinsics_compare_actions.addWidget(self.add_zoom_intrinsics_file_button)
        zoom_intrinsics_compare_actions.addWidget(self.remove_zoom_intrinsics_file_button)
        zoom_intrinsics_compare_actions.addWidget(self.apply_zoom_intrinsics_file_button)
        zoom_intrinsics_compare_actions.addStretch(1)
        self.zoom_intrinsics_compare_list = QtWidgets.QListWidget()
        self.zoom_intrinsics_compare_list.setObjectName("zoomIntrinsicsCompareList")
        self.zoom_intrinsics_compare_list.setSelectionMode(QtWidgets.QAbstractItemView.ExtendedSelection)
        self.zoom_intrinsics_compare_list.setDragDropMode(QtWidgets.QAbstractItemView.InternalMove)
        self.zoom_intrinsics_compare_list.setDefaultDropAction(QtCore.Qt.MoveAction)
        self.zoom_intrinsics_compare_list.setWordWrap(True)
        self.zoom_intrinsics_compare_list.setTextElideMode(QtCore.Qt.ElideNone)
        self.zoom_intrinsics_compare_list.setMinimumHeight(220)
        self.zoom_intrinsics_compare_list.itemDoubleClicked.connect(
            lambda _item: self.apply_selected_zoom_intrinsics_comparison_file()
        )
        zoom_intrinsics_compare_layout.addLayout(zoom_intrinsics_compare_actions)
        zoom_intrinsics_compare_layout.addWidget(self.zoom_intrinsics_compare_list, 1)
        zoom_intrinsics_layout.addWidget(zoom_intrinsics_compare_group, 1)

        intrinsic_group = QtWidgets.QGroupBox("내부 파라미터")
        intrinsic_layout = QtWidgets.QHBoxLayout(intrinsic_group)
        self.left_intrinsics_text = self._build_parameter_text()
        self.right_intrinsics_text = self._build_parameter_text()
        intrinsic_layout.addWidget(self.left_intrinsics_text, 1)
        intrinsic_layout.addWidget(self.right_intrinsics_text, 1)
        extrinsic_group = QtWidgets.QGroupBox("외부 파라미터 / 스테레오 기하")
        extrinsic_layout = QtWidgets.QHBoxLayout(extrinsic_group)
        self.left_extrinsics_text = self._build_parameter_text()
        self.right_extrinsics_text = self._build_parameter_text()
        self.stereo_geometry_text = self._build_parameter_text()
        extrinsic_layout.addWidget(self.left_extrinsics_text, 1)
        extrinsic_layout.addWidget(self.right_extrinsics_text, 1)
        extrinsic_layout.addWidget(self.stereo_geometry_text, 1)
        intrinsic_parameters_layout.addWidget(intrinsic_group, 1)
        extrinsic_parameters_layout.addWidget(extrinsic_group, 1)

        video_row = QtWidgets.QHBoxLayout()
        self.left_pane = CameraPane("left")
        self.right_pane = CameraPane("right")
        self.left_pane.detection_changed.connect(self._update_detection)
        self.right_pane.detection_changed.connect(self._update_detection)
        self.left_pane.frame_changed.connect(self._update_calibration_preview)
        self.right_pane.frame_changed.connect(self._update_calibration_preview)
        self.left_pane.stats_changed.connect(self._update_camera_stream_stats)
        self.right_pane.stats_changed.connect(self._update_camera_stream_stats)
        self.left_pane.state_changed.connect(self._log_state)
        self.right_pane.state_changed.connect(self._log_state)
        video_row.addWidget(self.left_pane, 1)
        video_row.addWidget(self.right_pane, 1)
        runtime_controls = QtWidgets.QGridLayout()
        runtime_controls.setHorizontalSpacing(6)
        runtime_controls.setVerticalSpacing(6)
        runtime_controls.addWidget(self.start_button, 0, 0)
        self.record_button = QtWidgets.QPushButton("세션 기록 시작")
        self.record_button.clicked.connect(self.toggle_session_recording)
        self.clear_history_button = QtWidgets.QPushButton("그래프/기록 초기화")
        self.clear_history_button.clicked.connect(self.clear_distance_history)
        self.record_button.setText("세션 기록 시작")
        self.clear_history_button.setText("그래프/기록 초기화")
        runtime_action_buttons = (
            self.start_button,
            self.record_button,
            self.clear_history_button,
        )
        runtime_action_width = max(button.sizeHint().width() for button in runtime_action_buttons)
        for button in runtime_action_buttons:
            button.setFixedWidth(runtime_action_width)
        runtime_controls.addWidget(self.record_button, 0, 1)
        runtime_controls.addWidget(self.clear_history_button, 0, 2)
        for column in range(3):
            runtime_controls.setColumnStretch(column, 1)
        self.runtime_camera_settings_group = QtWidgets.QGroupBox("실시간 카메라 / NTP / Pair 상태")
        self.runtime_camera_settings_group.setObjectName("runtimeCameraSettingsGroup")
        runtime_camera_settings_layout = QtWidgets.QVBoxLayout(self.runtime_camera_settings_group)
        runtime_camera_settings_layout.setSpacing(4)
        self.runtime_header_layout = QtWidgets.QBoxLayout(QtWidgets.QBoxLayout.LeftToRight)
        self.runtime_header_layout.setContentsMargins(0, 0, 0, 0)
        self.runtime_header_layout.setSpacing(8)
        self.runtime_header_layout.addLayout(runtime_controls, 2)
        self.runtime_camera_status_group = QtWidgets.QGroupBox()
        self.runtime_camera_status_group.setObjectName("runtimeCameraStatusGroup")
        self.runtime_camera_status_group.setFixedHeight(RUNTIME_CAMERA_STATUS_GROUP_HEIGHT_PX)
        self.left_runtime_stats_label = QtWidgets.QLabel(
            "왼쪽: -- | -- | 대기 중 | live: missing", self
        )
        self.right_runtime_stats_label = QtWidgets.QLabel(
            "오른쪽: -- | -- | 대기 중 | live: missing", self
        )
        self.runtime_pair_status_label = QtWidgets.QLabel("Pair: 대기 | pair: missing", self)
        for stats_label in (self.left_runtime_stats_label, self.right_runtime_stats_label, self.runtime_pair_status_label):
            stats_label.setMinimumWidth(320)
            stats_label.setFixedHeight(RUNTIME_CAMERA_STATUS_LABEL_HEIGHT_PX)
            stats_label.setSizePolicy(QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Fixed)
            stats_label.setAlignment(QtCore.Qt.AlignCenter)
            stats_label.setObjectName("statChip")
            stats_label.setTextInteractionFlags(QtCore.Qt.TextSelectableByMouse)
            stats_label.setWordWrap(True)
            stats_label.hide()
        self.runtime_camera_summary_label = QtWidgets.QLabel("카메라: 좌 NTP -- | 우 NTP -- | Pair 대기")
        self.runtime_camera_summary_label.setObjectName("runtimeCameraSummary")
        self.runtime_camera_summary_label.setAlignment(QtCore.Qt.AlignLeft | QtCore.Qt.AlignVCenter)
        self.runtime_camera_summary_label.setTextInteractionFlags(QtCore.Qt.TextSelectableByMouse)
        self.runtime_camera_summary_label.setWordWrap(False)
        self.runtime_camera_summary_label.setSizePolicy(
            QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Fixed
        )
        self.runtime_camera_summary_label.setFixedHeight(RUNTIME_CAMERA_STATUS_LABEL_HEIGHT_PX)
        self.runtime_camera_summary_label.setFont(QtGui.QFontDatabase.systemFont(QtGui.QFontDatabase.FixedFont))
        self.runtime_camera_status_layout = QtWidgets.QHBoxLayout(self.runtime_camera_status_group)
        self.runtime_camera_status_layout.setContentsMargins(8, 2, 8, 6)
        self.runtime_camera_status_layout.addWidget(self.runtime_camera_summary_label)
        self.runtime_camera_status_group.setSizePolicy(QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Fixed)
        runtime_camera_settings_layout.addLayout(self.runtime_header_layout)
        self.runtime_header_layout.addWidget(self.runtime_camera_status_group, 3)
        self._configure_collapsible_group(self.runtime_camera_settings_group, checked=True)
        runtime_layout.addWidget(self.runtime_camera_settings_group)
        runtime_layout.addLayout(video_row, 10)
        self.runtime_distance_graph_group = QtWidgets.QGroupBox("3D 위치 / 거리 그래프")
        self.runtime_distance_graph_group.setObjectName("runtimeDistanceGraphGroup")
        distance_plot_layout = QtWidgets.QVBoxLayout(self.runtime_distance_graph_group)
        self.distance_plot = DistanceHistoryPlot()
        self.distance_plot.point_deleted.connect(self._remove_distance_history_point)
        distance_plot_layout.addWidget(self.distance_plot)
        self._configure_collapsible_group(self.runtime_distance_graph_group, checked=True)
        runtime_layout.addWidget(self.runtime_distance_graph_group, 1)
        self._update_runtime_camera_settings_layout_mode(self.width())

        quality_summary_group = QtWidgets.QGroupBox("현재 측정 품질")
        quality_summary_form = QtWidgets.QGridLayout(quality_summary_group)
        quality_summary_form.setHorizontalSpacing(8)
        quality_summary_form.setColumnStretch(1, 1)
        quality_summary_form.setColumnStretch(3, 1)
        self.triangulation_quality_status_label = QtWidgets.QLabel("3D ray triangulation 대기 중")
        self.triangulation_quality_status_label.setObjectName("statusStrip")
        self.triangulation_quality_status_label.setWordWrap(True)
        self.triangulation_quality_status_label.setTextInteractionFlags(QtCore.Qt.TextSelectableByMouse)
        self.triangulation_quality_gap_label = QtWidgets.QLabel("gap: --")
        self.triangulation_quality_distance_label = QtWidgets.QLabel("distance: --")
        self.triangulation_quality_mode_label = QtWidgets.QLabel("mode: --")
        self.triangulation_quality_attitude_label = QtWidgets.QLabel("Attitude timing: --")
        self.triangulation_quality_attitude_label.setObjectName("statusStrip")
        self.triangulation_quality_attitude_label.setWordWrap(True)
        self.triangulation_quality_attitude_label.setTextInteractionFlags(QtCore.Qt.TextSelectableByMouse)
        self.triangulation_quality_attitude_label.setToolTip(
            "표시 전용 비차단 진단: sample age > 2 s 또는 pair Δt > 1 s이면 STALE, "
            "sample age > 10 s 또는 pair Δt > 5 s이면 VERY_STALE입니다."
        )
        for label in (
            self.triangulation_quality_gap_label,
            self.triangulation_quality_distance_label,
            self.triangulation_quality_mode_label,
        ):
            label.setObjectName("statChip")
            label.setAlignment(QtCore.Qt.AlignCenter)
            label.setTextInteractionFlags(QtCore.Qt.TextSelectableByMouse)
        quality_summary_form.addWidget(self.triangulation_quality_status_label, 0, 0, 1, 4)
        quality_summary_form.addWidget(self.triangulation_quality_gap_label, 1, 0)
        quality_summary_form.addWidget(self.triangulation_quality_distance_label, 1, 1)
        quality_summary_form.addWidget(self.triangulation_quality_mode_label, 1, 2, 1, 2)
        quality_summary_form.addWidget(self.triangulation_quality_attitude_label, 2, 0, 1, 4)
        triangulation_quality_layout.addWidget(quality_summary_group)

        self.triangulation_quality_plot = Triangulation3DView()
        triangulation_quality_layout.addWidget(self.triangulation_quality_plot, 1)
        point_group = QtWidgets.QGroupBox("3D point")
        point_layout = QtWidgets.QGridLayout(point_group)
        point_layout.setColumnStretch(1, 1)
        self.triangulation_quality_raw_point_label = QtWidgets.QLabel("Raw baseline_center_frd point: --")
        self.triangulation_quality_raw_point_label.setObjectName("statusStrip")
        self.triangulation_quality_raw_point_label.setTextInteractionFlags(QtCore.Qt.TextSelectableByMouse)
        point_layout.addWidget(self.triangulation_quality_raw_point_label, 0, 0, 1, 2)
        triangulation_quality_layout.addWidget(point_group)
        self._clear_triangulation_quality("3D ray triangulation 대기 중")

        save_group = QtWidgets.QGroupBox("세션 파일 / 설정")
        save_form = QtWidgets.QGridLayout(save_group)
        self.save_history_button = QtWidgets.QPushButton("세션 파일 저장")
        self.save_history_button.clicked.connect(self.save_distance_history_csv)
        self.save_history_button.setToolTip(
            "현재 측정을 typed Parquet 원본으로 확정하고 세션 통합 review.xlsx를 갱신합니다."
        )
        self.save_settings_button = QtWidgets.QPushButton("전체 설정 저장")
        self.save_settings_button.clicked.connect(self.save_current_settings_from_ui)
        self.save_history_button.setText("세션 파일 저장")
        self.save_settings_button.setText("전체 설정 저장")
        self.session_summary_label = QtWidgets.QLabel("아직 기록된 매칭 프레임쌍이 없습니다.")
        self.session_summary_label.setObjectName("statusStrip")
        self.session_summary_label.setTextInteractionFlags(QtCore.Qt.TextSelectableByMouse)
        self.session_summary_label.setWordWrap(True)
        self.recording_status_label = QtWidgets.QLabel("세션 기록: 중지")
        self.recording_status_label.setObjectName("statusStrip")
        self.recording_status_label.setWordWrap(True)
        self.recording_status_label.setTextInteractionFlags(QtCore.Qt.TextSelectableByMouse)
        save_form.addWidget(self.save_history_button, 0, 0)
        save_form.addWidget(self.save_settings_button, 0, 1)
        save_form.addWidget(self.session_summary_label, 1, 0, 1, 2)
        save_form.addWidget(self.recording_status_label, 2, 0, 1, 2)
        save_layout.addWidget(save_group)

        self.recording_table_group = QtWidgets.QGroupBox("저장될 세션 review.xlsx 미리보기")
        recording_table_layout = QtWidgets.QVBoxLayout(self.recording_table_group)
        self.recording_sheet_tabs = QtWidgets.QTabWidget()
        self.recording_sheet_tabs.setDocumentMode(True)
        self.recording_sheet_tables: dict[str, QtWidgets.QTableWidget] = {}
        for sheet_name in self._recording_sheet_names():
            table = QtWidgets.QTableWidget(0, 0)
            table.setObjectName("recordingSampleTable")
            table.setEditTriggers(QtWidgets.QAbstractItemView.NoEditTriggers)
            table.setSelectionBehavior(QtWidgets.QAbstractItemView.SelectRows)
            table.setSelectionMode(QtWidgets.QAbstractItemView.SingleSelection)
            table.setAlternatingRowColors(True)
            table.verticalHeader().setVisible(False)
            table.horizontalHeader().setSectionResizeMode(QtWidgets.QHeaderView.ResizeToContents)
            table.itemSelectionChanged.connect(self._refresh_recording_sample_detail)
            self.recording_sheet_tables[sheet_name] = table
            self.recording_sheet_tabs.addTab(table, sheet_name)
        self.recording_sheet_tabs.currentChanged.connect(
            lambda _index: self._refresh_active_recording_sheet()
        )
        recording_table_layout.addWidget(self.recording_sheet_tabs, 2)
        self.recording_sample_detail = QtWidgets.QPlainTextEdit()
        self.recording_sample_detail.setObjectName("recordingSampleDetail")
        self.recording_sample_detail.setReadOnly(True)
        self.recording_sample_detail.setMaximumBlockCount(300)
        self.recording_sample_detail.setPlaceholderText("테이블에서 샘플을 선택하면 전체 내부 기록 필드가 표시됩니다.")
        recording_table_layout.addWidget(self.recording_sample_detail, 1)
        save_layout.addWidget(self.recording_table_group, 1)

        self._build_clock_sync_tab(time_layout)
        self._polish_controls()
        self._refresh_session_toggle_buttons()
        self._install_tooltips()
        self._set_focal_override_enabled(self.manual_focal_checkbox.isChecked())
        self._update_distance_settings_visibility()
        self._refresh_camera_parameters_tab()
        self._refresh_operation_header()
        self._shutdown_coordinator = LifecycleCoordinator(
            [
                LifecycleStep("stop-target-drone", lambda: self._shutdown_stop_target_drone()),
                LifecycleStep("stop-gnss-network", lambda: self.stop_gnss_network()),
                LifecycleStep("stop-gnss-base-serial", lambda: self._shutdown_stop_gnss_base_serial()),
                LifecycleStep("flush-stop-rtcm-pipeline", lambda: self._stop_rtcm_pipeline()),
                LifecycleStep("stop-drone-telemetry", lambda: self._shutdown_stop_drone_telemetry()),
                LifecycleStep("stop-runtime-workers", lambda: self.stop_all(wait=False)),
                LifecycleStep("stop-auxiliary-workers", lambda: self._shutdown_stop_auxiliary_workers()),
                LifecycleStep("wait-session-save", lambda: self._wait_for_session_save()),
                LifecycleStep("save-settings", lambda: self._shutdown_save_settings()),
            ]
        )
        QtCore.QTimer.singleShot(0, self._initial_status)
        QtCore.QTimer.singleShot(0, self._auto_load_intrinsics)

    def _build_toggle_button(self, text: str, checked: bool = False) -> QtWidgets.QPushButton:
        button = QtWidgets.QPushButton(text)
        button.setCheckable(True)
        button.setChecked(bool(checked))
        button.setProperty("role", "toggle")
        button.setAccessibleName(text)
        return button

    def _pixhawk_ethernet_mode(self) -> str:
        mode = str(self.pixhawk_ethernet_mode_combo.currentData() or "mavlink")
        return mode if mode == "mavlink" else "mavlink"

    def _refresh_pixhawk_ethernet_mode_controls(self) -> None:
        # Workstation has a MAVLink-only Pixhawk transport surface.
        self.pixhawk_ethernet_mode_combo.setCurrentIndex(0)

    def _set_yolo_device_combo(self, device: str) -> None:
        index = self.device_combo.findData(self._normalized_yolo_device(device))
        if index >= 0:
            self.device_combo.setCurrentIndex(index)

    def _yolo_device_uses_cuda(self, value: str) -> bool:
        return self._normalized_yolo_device(value) == YOLO_CUDA_DEVICE

    def _normalized_yolo_device(self, value: str) -> str:
        normalized = value.strip().lower()
        if normalized in {"cuda", "cuda:0", "0", "gpu"}:
            return YOLO_CUDA_DEVICE
        return YOLO_CPU_DEVICE

    def _refresh_yolo_device_toggle(self, _checked: bool | None = None) -> None:
        if not hasattr(self, "device_combo"):
            return
        device = self._normalized_yolo_device(str(self.device_combo.currentData() or YOLO_CPU_DEVICE))
        self.device_edit.setText(device)
        self.device_combo.setAccessibleName(f"추론 장치: {device}")

    def _refresh_yolo_gate_controls(self) -> None:
        if not hasattr(self, "yolo_gate_class_checkbox"):
            return
        class_enabled = self.yolo_gate_class_checkbox.isChecked()
        time_basis_enabled = self.yolo_gate_matching_checkbox.isChecked()
        time_delta_enabled = self.yolo_gate_time_delta_checkbox.isChecked()
        vertical_delta_enabled = self.yolo_gate_vertical_delta_checkbox.isChecked()
        class_target = self.yolo_gate_class_edit.text().strip()
        self.yolo_gate_class_edit.setEnabled(class_enabled)
        for widget in (
            self.yolo_gate_time_basis_combo,
        ):
            widget.setEnabled(time_basis_enabled)
        self.yolo_gate_time_delta_spin.setEnabled(time_delta_enabled)
        self.yolo_gate_vertical_delta_spin.setEnabled(vertical_delta_enabled)
        enabled = []
        if class_enabled:
            enabled.append("class" if class_target else "class(target missing)")
        if time_basis_enabled:
            enabled.append("time-basis")
        if time_delta_enabled:
            enabled.append("time-delta")
        if vertical_delta_enabled:
            enabled.append("vertical-delta")
        text = "Gate OFF: 기존 YOLO 후보 선택/매칭을 사용합니다." if not enabled else f"Gate ON: {', '.join(enabled)}"
        self.yolo_gate_status_label.setText(text)

    def _yolo_matching_gate_enabled(self) -> bool:
        return bool(
            hasattr(self, "yolo_gate_matching_checkbox")
            and self.yolo_gate_matching_checkbox.isChecked()
        )

    def _yolo_time_delta_gate_enabled(self) -> bool:
        return bool(
            hasattr(self, "yolo_gate_time_delta_checkbox")
            and self.yolo_gate_time_delta_checkbox.isChecked()
        )

    def _yolo_vertical_delta_gate_enabled(self) -> bool:
        return bool(
            hasattr(self, "yolo_gate_vertical_delta_checkbox")
            and self.yolo_gate_vertical_delta_checkbox.isChecked()
        )

    def _yolo_pair_max_delta_s(self) -> float:
        if self._yolo_time_delta_gate_enabled():
            return self.yolo_gate_time_delta_spin.value()
        return float("inf")

    def _yolo_time_basis(self) -> str:
        if self._camera_sync_gate_active():
            return "camera_rtc_required"
        if self._yolo_matching_gate_enabled() and hasattr(self, "yolo_gate_time_basis_combo"):
            return str(self.yolo_gate_time_basis_combo.currentData() or "camera_rtc_required")
        return "detection"

    def _yolo_match_timestamp_s(self, observation: DetectionObservation) -> float | None:
        return detection_timestamp_for_basis(observation, self._effective_yolo_time_basis())

    def _effective_yolo_time_basis(self) -> str:
        if self._camera_sync_gate_active():
            return "camera_rtc_required"
        if self._yolo_time_basis() == "camera_rtcp" and self._runtime_pair_uses_app_fallback:
            return "detection"
        return self._yolo_time_basis()

    def _camera_sync_gate_active(self) -> bool:
        """Require verified RTCP/NTP once a live camera stream is running."""

        return bool(self.camera_connected)

    def _camera_rtc_timestamp_s_from_metadata(self, metadata: dict) -> float | None:
        value = metadata.get("rtcp_frame_ntp_unix_s")
        if value is None:
            return None
        try:
            parsed = float(value)
        except (TypeError, ValueError):
            return None
        if not math.isfinite(parsed):
            return None
        return parsed

    def _selected_pair_sync_summary(self, left: DetectionObservation, right: DetectionObservation) -> tuple[str, float | None]:
        if self._runtime_pair_uses_app_fallback:
            return "app fallback", None
        left_rtc_s = self._camera_rtc_timestamp_s_from_metadata(left.timestamp_metadata)
        right_rtc_s = self._camera_rtc_timestamp_s_from_metadata(right.timestamp_metadata)
        camera_rtc_delta_s = None
        if left_rtc_s is not None and right_rtc_s is not None:
            camera_rtc_delta_s = abs(left_rtc_s - right_rtc_s)
        time_basis = self._yolo_time_basis()
        if time_basis == "camera_rtc_required":
            basis = "RTCP/NTP required" if camera_rtc_delta_s is not None else "RTCP/NTP missing"
        elif time_basis == "camera_rtcp":
            basis = "RTCP/NTP preferred" if camera_rtc_delta_s is not None else "app fallback"
        else:
            basis = "app"
        return basis, camera_rtc_delta_s

    def _yolo_vertical_max_delta_px(self) -> float:
        if self._yolo_vertical_delta_gate_enabled():
            return self.yolo_gate_vertical_delta_spin.value()
        return float("inf")

    def _set_button_role(self, button: QtWidgets.QPushButton, role: str) -> None:
        if button.property("role") == role:
            return
        button.setProperty("role", role)
        button.style().unpolish(button)
        button.style().polish(button)

    def _refresh_session_toggle_buttons(self) -> None:
        if not hasattr(self, "camera_connect_button"):
            return
        self.camera_connect_button.setText(
            "Disconnect Camera Preview" if self.camera_connected else "Connect Camera Preview"
        )
        self._set_button_role(self.camera_connect_button, "danger" if self.camera_connected else "primary")

        self.start_button.setText("YOLO 측정 중지" if self.yolo_enabled else "YOLO 측정 시작")
        self._set_button_role(self.start_button, "danger" if self.yolo_enabled else "primary")

        self.record_button.setText("세션 기록 중지" if self._session_recording_active() else "세션 기록 시작")
        self._set_button_role(self.record_button, "danger" if self._session_recording_active() else "success")

        connected_mavlink_sides = self._mavlink_open_sides()
        self.left_mavlink_button.setText(
            "Disconnect P1 MAVLink" if "left" in connected_mavlink_sides else "Connect P1 MAVLink"
        )
        self._set_button_role(self.left_mavlink_button, "danger" if "left" in connected_mavlink_sides else "primary")
        self.right_mavlink_button.setText(
            "Disconnect P2 MAVLink" if "right" in connected_mavlink_sides else "Connect P2 MAVLink"
        )
        self._set_button_role(self.right_mavlink_button, "danger" if "right" in connected_mavlink_sides else "primary")

    def _install_tooltips(self) -> None:
        tooltips = {
            self.interface_edit: "카메라 네트워크 도달성을 확인할 때 사용할 로컬 네트워크 인터페이스입니다.",
            self.host_address_edit: "WIRIS 카메라망에서 PC가 사용할 IP입니다. 카메라 IP가 아니며 Pixhawk host 주소와도 별개입니다.",
            self.pixhawk_host_address_edit: "Pixhawk MAVLink가 사용할 PC 주소입니다. 예: 10.0.0.10",
            self.left_url_edit: "왼쪽 카메라 스트림의 전체 RTSP URL입니다. 카메라 IP, 포트, 경로를 포함합니다.",
            self.right_url_edit: "오른쪽 카메라 스트림의 전체 RTSP URL입니다. 카메라 IP, 포트, 경로를 포함합니다.",
            self.left_rtsp_ip_edit: "왼쪽 RTSP URL에 적용할 OSD IP Address 값입니다. Telnet 제어 host는 이 버튼으로 바꾸지 않습니다.",
            self.right_rtsp_ip_edit: "오른쪽 RTSP URL에 적용할 OSD IP Address 값입니다. Telnet 제어 host는 이 버튼으로 바꾸지 않습니다.",
            self.left_rtsp_apply_button: "왼쪽 RTSP URL의 host만 OSD IP Address 값으로 바꿉니다. RTSP port는 URL 안의 포트를 유지합니다.",
            self.right_rtsp_apply_button: "오른쪽 RTSP URL의 host만 OSD IP Address 값으로 바꿉니다. RTSP port는 URL 안의 포트를 유지합니다.",
            self.weights_edit: (
                "거리 추정 전 객체 검출에 사용할 YOLO 모델 파일입니다. .pt/.onnx/.engine 등 Ultralytics가 읽을 수 있는 "
                "weight를 지정하며, 모델의 학습 클래스와 입력 전처리도 이 파일에 따라 결정됩니다. "
                "파일을 바꾸면 다음 `YOLO 측정 시작` 때 새 모델을 로드하고, 모델 로드 실패 시 카메라 Preview만 유지합니다."
            ),
            self.browse_weights_button: ".pt, .onnx, .engine 같은 YOLO weight 파일을 선택합니다.",
            self.device_combo: (
                "YOLO 추론을 실행할 장치입니다. `CPU`는 PyTorch `cpu`, `CUDA`는 `cuda:0`을 사용합니다. "
                "CUDA를 선택해도 드라이버/모델 추론이 실패하면 worker가 가능한 경우 CPU로 재시도합니다. "
                "이 값은 RTSP 수신/디코더 장치가 아니라 객체 추론 장치만 결정합니다."
            ),
            self.input_size_spin: (
                "Ultralytics에 전달할 YOLO 추론 입력 크기(px)입니다. 디코드된 원본 프레임을 이 크기로 letterbox/resize한 뒤 "
                "추론하며, GUI Preview 크기는 바꾸지 않습니다. 값을 키우면 작은 객체를 찾는 데 유리할 수 있지만 "
                "GPU 메모리·추론 시간·지연이 증가하고 YOLO Hz가 낮아질 수 있습니다. 값을 줄이면 처리량은 좋아지지만 "
                "작은 객체가 사라질 수 있습니다. worker는 모델 stride에 맞게 내부적으로 크기를 보정할 수 있습니다."
            ),
            self.conf_spin: (
                "검출 후보를 유지할 최소 confidence(0.01~1.0)입니다. YOLO가 계산한 class confidence가 이 값보다 낮은 "
                "박스는 좌/우 매칭 전에 버립니다. 낮추면 놓치는 객체는 줄지만 오검출이 늘고, 높이면 오검출은 줄지만 "
                "희미하거나 작은 객체를 놓칠 수 있습니다. 좌/우 동기화의 Δt 기준이나 NMS IoU와는 별도 값입니다."
            ),
            self.iou_spin: (
                "NMS(non-maximum suppression)에 사용하는 박스 겹침 기준(IoU)입니다. 같은 class의 두 박스가 겹칠 때 "
                "높은 confidence 박스를 남길지 판단합니다. 낮추면 겹친 박스를 더 공격적으로 제거하고, 높이면 겹친 "
                "후보를 더 오래 남깁니다. 이 값은 좌/우 frame의 시간 차이 또는 RTCP/NTP 동기화 판정이 아닙니다."
            ),
            self.preview_width_spin: (
                "Qt GUI Preview에 복사할 최대 너비(px)입니다. 1920으로 두면 현재 1080p WIRIS 프레임을 추가 축소 없이 "
                "표시할 수 있습니다. 이 값은 Preview 복사/렌더링 비용만 바꾸며 YOLO 검출 입력 크기와 다릅니다. "
                "YOLO는 Preview 이미지가 아니라 원본 디코딩 프레임을 입력으로 사용합니다."
            ),
            self.rtsp_latency_spin: (
                "GStreamer `rtpjitterbuffer`가 RTP 패킷을 기다리는 버퍼 시간(ms)입니다. 이것은 YOLO 추론 지연이 아니라 "
                "RTSP 수신 경로의 지터 흡수 설정입니다. 낮추면 최신 프레임 지연은 줄지만 패킷 순서 흔들림/손실에 민감해지고, "
                "높추면 네트워크 끊김에는 강해질 수 있지만 오래된 프레임과 전체 처리 지연이 늘어납니다. "
                "GStreamer backend에서 주로 적용되며 RTCP/NTP 오프셋 추이의 transit delay와 같은 값이 아닙니다."
            ),
            self.yolo_event_queue_size_spin: (
                "YOLO worker에서 GUI로 보내는 detection, measurement frame, stats, status 이벤트 큐의 최대 크기입니다. "
                "작게 두면 이벤트 backlog로 인한 UI 지연을 줄일 수 있지만, 처리 속도가 UI보다 빠를 때 오래된 이벤트가 "
                "버려질 수 있습니다. YOLO 모델의 정확도나 RTSP 디코딩 속도 자체를 바꾸지는 않습니다."
            ),
            self.yolo_frame_queue_size_spin: (
                "RTSP reader에서 YOLO worker로 전달하는 decoded frame 큐 크기입니다. 작을수록 최신 frame을 우선해 실시간 "
                "지연을 낮추지만, worker가 느리면 중간 frame을 버립니다. 크게 하면 burst를 흡수할 수 있지만 오래된 frame이 "
                "추론될 가능성과 지연 누적이 커집니다. 수신 Hz는 reader 기준이고 YOLO Hz는 추론 완료 기준이라 서로 다를 수 있습니다."
            ),
            self.decoder_combo: (
                "RTSP H.264 디코더 선택입니다. `NVIDIA`는 nvh264dec, `Software`는 avdec_h264를 사용하고, `Auto`는 NVIDIA를 "
                "먼저 시도한 뒤 생성 실패 시 소프트웨어로 전환합니다. 디코더 선택은 YOLO 정확도보다 CPU/GPU 사용량, "
                "프레임 수신 안정성, 지연에 영향을 줍니다."
            ),
            self.stream_backend_combo: (
                "RTSP 수신/디코드 백엔드입니다. `GStreamer`는 jitter buffer, decoder, frame drop을 세밀하게 제어하는 기본 경로이고, "
                "`OpenCV/FFmpeg`는 단순한 대체 경로, `OpenCV Auto`는 OpenCV가 사용 가능한 backend를 선택합니다. "
                "`PyAV`는 av 패키지가 있을 때 FFmpeg를 직접 사용합니다. backend를 바꾸면 RTP/RTCP timestamp와 지연 특성이 달라질 수 있습니다."
            ),
            self.rtsp_transport_combo: (
                "RTSP media transport입니다. `UDP`는 지연 누적이 적지만 방화벽/패킷 손실에 영향을 받고, `TCP` interleaving은 "
                "RTSP control channel로 RTP/RTCP를 함께 받아 Sender Report 유실을 줄이는 대신 수신 queue 지연이 누적될 수 있습니다. "
                "카메라 시간 검증에서는 RTCP Sender Report가 필요하므로 TCP를 기본값으로 사용합니다."
            ),
            self.yolo_gate_class_checkbox: (
                "대상 class gate를 켭니다. 켜면 아래 입력한 class id 또는 class 이름과 일치하는 YOLO 검출만 좌/우 매칭 후보로 사용합니다. "
                "예: `0` 또는 `person`. YOLO 모델의 실제 class names와 맞아야 하며, 검출 confidence/NMS 이후에 적용됩니다."
            ),
            self.yolo_gate_class_edit: "대상 class id 또는 class 이름입니다. `0`, `1`, `person`처럼 입력하며 대상 class gate가 켜져 있을 때만 적용됩니다.",
            self.yolo_gate_matching_checkbox: (
                "시간 기준 매칭 gate를 켭니다. 켜면 아래 time basis를 사용해 좌/우 detection 후보를 시간 순서와 timestamp 차이로 매칭합니다. "
                "실시간 카메라가 연결되면 안전을 위해 RTCP/NTP 필수 기준이 우선 적용되며, 이 toggle은 RTCP/NTP가 없는 단순 offline/app 매칭을 강제로 우회하지 않습니다."
            ),
            self.yolo_gate_time_basis_combo: (
                "좌/우 detection pair를 고를 시간 기준입니다. `RTCP/NTP 필수`는 두 frame에 RTCP/NTP 시각이 없으면 거부하고, "
                "`RTCP/NTP 우선`은 가능하면 RTCP/NTP를 쓰며 fallback을 허용하고, `app 수신 시간`은 공통 monotonic 수신 시각을 사용합니다. "
                "실시간 카메라 연결 중에는 절대 UTC/카메라 동기화 보호를 위해 RTCP/NTP 필수 기준이 우선됩니다."
            ),
            self.yolo_gate_time_delta_checkbox: (
                "좌/우 detection pair의 최대 시간 차이 gate를 켭니다. 아래 최대 Δt보다 timestamp 차이가 큰 후보는 매칭하지 않습니다. "
                "이 값은 YOLO pair 선택 기준이며, `시간 > Sync limits`의 RTCP/NTP 동기화 Gate(기본 50ms)나 `RTP 지연(ms)`와는 별개입니다."
            ),
            self.yolo_gate_time_delta_spin: "YOLO 좌/우 pair 최대 Δt(s)입니다. 최대 Δt gate가 켜져 있을 때 이 값보다 오래 떨어진 detection pair를 제외합니다.",
            self.yolo_gate_vertical_delta_checkbox: (
                "좌/우 bbox 중심의 세로 위치 차이 gate를 켭니다. 카메라 정렬이 양호하면 같은 대상의 중심 y 차이가 작아야 하므로, "
                "아래 vertical Δ보다 큰 pair를 제외해 잘못된 좌/우 매칭을 줄입니다."
            ),
            self.yolo_gate_vertical_delta_spin: "좌/우 bbox 중심 y 좌표의 최대 차이(px)입니다. vertical Δ gate가 켜져 있을 때 이 값보다 큰 pair를 제외합니다.",
            self.detect_pc_ip_button: "선택한 인터페이스의 활성 NetworkManager 연결에서 카메라망 IP를 찾아 PC IP 입력칸에 채웁니다.",
            self.apply_pc_ip_button: "선택한 로컬 인터페이스에 PC 카메라망 IP를 적용해 카메라 NIC로 통신하게 합니다.",
            self.auto_wiris_connect_button: "WIRIS Telnet port 전체 스캔, HIWS 확인, Ethernet/RTSP 활성화, 프레임 확인, 프리뷰 연결을 순서대로 시도합니다.",
            self.detect_control_button: "후보 WIRIS IP Address의 TCP 1-65535 포트를 스캔하고 HIWS=OK 응답을 받은 Telnet port를 적용합니다.",
            self.right_detect_control_button: "후보 WIRIS IP Address의 TCP 1-65535 포트를 스캔하고 HIWS=OK 응답을 받은 Telnet port를 적용합니다.",
            self.left_control_apply_button: "왼쪽 OSD IP Address/Telnet port를 저장하고 RTSP URL host와 IP Address 입력을 같은 값으로 맞춥니다.",
            self.right_control_apply_button: "오른쪽 OSD IP Address/Telnet port를 저장하고 RTSP URL host와 IP Address 입력을 같은 값으로 맞춥니다.",
            self.detect_rtsp_button: "로컬 카메라망에서 RTSP port를 찾아 좌/우 RTSP URL을 채우고 프레임 수신을 확인합니다.",
            self.right_detect_rtsp_button: "로컬 카메라망에서 RTSP port를 찾아 좌/우 RTSP URL을 채우고 프레임 수신을 확인합니다.",
            self.check_control_button: "각 WIRIS IP Address가 설정된 Telnet port에서 명령 연결을 받는지 확인합니다.",
            self.enable_rtsp_button: "설정된 WIRIS IP Address/Telnet port에 Ethernet Mode 명령을 보내 RTSP 서버를 엽니다.",
            self.left_stip_ip_edit: "왼쪽 WIRIS에 쓸 새 OSD IP Address입니다. 대상 카메라가 명확할 때만 사용하세요.",
            self.right_stip_ip_edit: "오른쪽 WIRIS에 쓸 새 OSD IP Address입니다. 대상 카메라가 명확할 때만 사용하세요.",
            self.stip_mask_edit: "STIP 이후 STMS로 함께 쓸 OSD IP Mask입니다.",
            self.stip_reboot_checkbox: "STIP/STMS 후 REBT를 보내 새 IP Address로 WIRIS를 재시작합니다.",
            self.left_stip_button: (
                "IP Address / IP Mask 변경 그룹의 Left IP Address를 왼쪽 WIRIS에 씁니다.\n"
                "완료되면 왼쪽 RTSP URL의 host도 같이 갱신됩니다."
            ),
            self.right_stip_button: (
                "IP Address / IP Mask 변경 그룹의 Right IP Address를 오른쪽 WIRIS에 씁니다.\n"
                "완료되면 오른쪽 RTSP URL의 host도 같이 갱신됩니다."
            ),
            self.pixhawk_usb_discover_button: "OS에 보이는 USB Pixhawk 포트와 MAVLink heartbeat/권한 상태를 후보 표에 표시합니다. 자동 할당은 하지 않습니다.",
            self.pixhawk_mavlink_url_candidate_button: "Ethernet MAVLink UDP 기본 URL 후보를 표에 추가합니다. PX4가 해당 UDP 포트로 MAVLink를 보내고 있어야 연결됩니다.",
            self.detect_pixhawk_host_ip_button: "P1/P2 Pixhawk ETH IP 대역에 맞는 이 PC의 IPv4를 찾아 Pixhawk Host IP에 채웁니다.",
            self.swap_camera_button: "좌/우 카메라 RTSP URL과 IP Address/Telnet port를 서로 바꿉니다. 실행 중인 스트림은 새 매핑으로 재시작됩니다.",
            self.swap_pixhawk_button: "P1/P2의 MAVLink URL, USB 제어 포트, ETH IP 매핑을 서로 바꿉니다. 실행 중인 MAVLink 링크는 새 매핑으로 재시작됩니다.",
            self.check_button: "좌/우 RTSP 호스트의 RTSP 포트가 열려 있는지 확인합니다.",
            self.camera_connect_button: "좌/우 RTSP Preview를 연결하거나, 연결 중이면 카메라 Preview와 활성 YOLO 검출을 중지합니다.",
            self.left_mavlink_button: "Pixhawk 1 MAVLink 세션을 연결하거나, 연결 중이면 P1 MAVLink 링크만 중지합니다.",
            self.right_mavlink_button: "Pixhawk 2 MAVLink 세션을 연결하거나, 연결 중이면 P2 MAVLink 링크만 중지합니다.",
            self.left_px4_shell_button: "활성 MAVLink 연결을 통해 Pixhawk 1 PX4 shell 창을 엽니다.",
            self.right_px4_shell_button: "활성 MAVLink 연결을 통해 Pixhawk 2 PX4 shell 창을 엽니다.",
            self.left_persist_eth_button: "왼쪽 Pixhawk eth0/MAVLink UDP 설정을 즉시 적용하고 extras.txt에 저장합니다.",
            self.right_persist_eth_button: "오른쪽 Pixhawk eth0/MAVLink UDP 설정을 즉시 적용하고 extras.txt에 저장합니다.",
            self.left_reboot_pixhawk_button: "활성 MAVLink 연결로 Pixhawk 1에 MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN을 보냅니다.",
            self.right_reboot_pixhawk_button: "활성 MAVLink 연결로 Pixhawk 2에 MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN을 보냅니다.",
            self.camera_connection_label: "좌/우 카메라 Preview 연결 상태와 측정된 프레임 속도를 표시합니다.",
            self.runtime_camera_settings_group: "실시간 카메라 화면 위의 실행 버튼, 카메라별 live 상태, 좌우 pair 상태를 접거나 펼칩니다.",
            self.runtime_camera_status_group: "좌측/우측 카메라 상태와 좌우 계산 pair 상태를 한 줄로 요약해 보여주는 상태 그룹입니다.",
            self.runtime_camera_summary_label: "좌측/우측 카메라의 현재 수신 상태와 각 카메라의 RTCP Sender Report 기반 NTP 시각, 계산에 선택된 좌우 pair 동기화 상태입니다. 상세 시각은 상태 툴팁에서 확인할 수 있습니다.",
            self.left_runtime_stats_label: (
                "측정 탭 왼쪽 카메라의 최신 해상도/Hz, YOLO Hz, live 시각입니다.\n"
                "수신 Hz: 스트림 reader가 디코딩한 프레임 속도입니다. 측정 모드에서도 YOLO 추론 완료 속도와 분리됩니다.\n"
                "YOLO Hz: 실제 YOLO 추론이 완료된 속도입니다. -- 는 아직 통계가 없고, 0.0 Hz 는 통계는 있으나 최근 추론 완료가 없다는 뜻입니다.\n"
                "CUDA/CPU: 설정값이 아니라 YOLO worker가 마지막 추론에 실제 사용한 장치입니다. CPU fallback은 CUDA 추론 실패 후 CPU로 전환된 상태입니다.\n"
                "실시간/지연: 최근 스트림 통계나 프레임이 2초 안에 갱신됐는지 표시합니다.\n"
                "GNSS-UTC: timestamp basis가 명시적으로 GNSS-UTC일 때만 UTC로 표시합니다.\n"
                "source timestamp: RTCP/NTP가 없을 때만 표시하는 디코더/소스 시각입니다.\n"
                "missing: RTCP/NTP와 source timestamp 모두 없는 상태입니다.\n"
                "live: 최신 수신 프레임의 RTCP Sender Report 기반 RTCP/NTP 시각입니다. WIRIS GDTI와의 UTC 일치는 미검증입니다."
            ),
            self.right_runtime_stats_label: (
                "측정 탭 오른쪽 카메라의 최신 해상도/Hz, YOLO Hz, live 시각입니다.\n"
                "수신 Hz: 스트림 reader가 디코딩한 프레임 속도입니다. 측정 모드에서도 YOLO 추론 완료 속도와 분리됩니다.\n"
                "YOLO Hz: 실제 YOLO 추론이 완료된 속도입니다. -- 는 아직 통계가 없고, 0.0 Hz 는 통계는 있으나 최근 추론 완료가 없다는 뜻입니다.\n"
                "CUDA/CPU: 설정값이 아니라 YOLO worker가 마지막 추론에 실제 사용한 장치입니다. CPU fallback은 CUDA 추론 실패 후 CPU로 전환된 상태입니다.\n"
                "실시간/지연: 최근 스트림 통계나 프레임이 2초 안에 갱신됐는지 표시합니다.\n"
                "GNSS-UTC: timestamp basis가 명시적으로 GNSS-UTC일 때만 UTC로 표시합니다.\n"
                "source timestamp: RTCP/NTP가 없을 때만 표시하는 디코더/소스 시각입니다.\n"
                "missing: RTCP/NTP와 source timestamp 모두 없는 상태입니다.\n"
                "live: 최신 수신 프레임의 RTCP Sender Report 기반 RTCP/NTP 시각입니다. WIRIS GDTI와의 UTC 일치는 미검증입니다."
            ),
            self.runtime_pair_status_label: "좌우 계산/동기화 frame pair의 time basis, pair 시각, 좌우 delta를 표시합니다.",
            self.pixhawk_connection_label: "Pixhawk 자세 구독 상태와 메시지 수신 속도를 표시합니다.",
            self.left_pixhawk_eth_ip_edit: "왼쪽 Pixhawk에 적용하고 저장할 eth0 고정 IP입니다. 예: 10.0.0.2",
            self.right_pixhawk_eth_ip_edit: "오른쪽 Pixhawk에 적용하고 저장할 eth0 고정 IP입니다. 예: 10.0.0.3",
            self.left_mavlink_url_edit: "Pixhawk 1 MAVLink URL입니다. 예: udpin:0.0.0.0:14550, /dev/ttyACM0, tcp:192.168.0.3:5760",
            self.right_mavlink_url_edit: "Pixhawk 2 MAVLink URL입니다. 예: udpin:0.0.0.0:14551, /dev/ttyACM1, tcp:10.41.10.2:5760",
            self.left_mavlink_baud_spin: "Pixhawk 1 MAVLink URL이 시리얼 장치일 때 사용할 baud rate입니다.",
            self.right_mavlink_baud_spin: "Pixhawk 2 MAVLink URL이 시리얼 장치일 때 사용할 baud rate입니다.",
            self.mavlink_configure_streams_checkbox: "MAVLink 연결 후 MAV_SYS_ID 설정과 Pixhawk Host IP 방향 MAVLink stream 시작 명령을 보냅니다.",
            self.mavlink_stream_rate_spin: "PX4 shell로 MAVLink stream을 설정할 때 사용할 mavlink start -r 값입니다.",
            self.custom_watch_name_edit: "사용자 지정 MAVLink 필드 감시의 표시 이름입니다.",
            self.custom_watch_message_edit: "감시할 MAVLink message 이름입니다. 예: ATTITUDE, GPS_RAW_INT",
            self.custom_watch_field_edit: "읽을 메시지 필드 경로입니다. 예: z, q[0], position[2]. 숫자 필드는 그래프로 표시됩니다.",
            self.custom_watch_add_button: "사용자 지정 MAVLink 감시를 추가합니다.",
            self.custom_watch_mavlink_discover_button: "ATTITUDE.roll 같은 MAVLink 메시지/필드를 골라 활성 MAVLink Pixhawk 연결에서 그래프로 표시합니다.",
            self.custom_watch_remove_combo: "제거할 기존 사용자 지정 감시를 선택합니다.",
            self.custom_watch_remove_button: "선택한 사용자 지정 감시를 제거하고 그래프 series를 지웁니다.",
            self.custom_watch_plot: "사용자 지정 숫자 감시값의 이력 그래프입니다.",
            self.rtk_baseline_label: "실시간 baseline 길이, N/E/D 벡터, 샘플 age, GPS/RTK 품질 경고를 표시합니다.",
            self.rtk_alignment_detail_label: "안테나 baseline과 카메라 기준 Right/Down/Forward 성분을 분리해 표시합니다.",
            self.rtk_alignment_plot: "GPS 안테나 baseline과 카메라 기준 baseline 성분을 시각화합니다.",
            self.left_gps_label: "Pixhawk 1의 최신 GPS/RTK 상태입니다.",
            self.right_gps_label: "Pixhawk 2의 최신 GPS/RTK 상태입니다.",
            self.left_zoom_host_edit: "왼쪽 WIRIS의 OSD IP Address입니다. Telnet/SDK 명령과 RTSP URL host의 기준값입니다.",
            self.right_zoom_host_edit: "오른쪽 WIRIS의 OSD IP Address입니다. Telnet/SDK 명령과 RTSP URL host의 기준값입니다.",
            self.zoom_port_spin: "Telnet port 탐색을 시작할 때 먼저 확인하는 스캔 기본 포트입니다. 좌/우 실제 명령은 각 WIRIS의 Telnet port를 사용합니다.",
            self.zoom_timeout_spin: "WIRIS SDK/줌 명령 TCP 요청 timeout입니다. 왼쪽 카메라 SDK 응답 지연을 포함해야 합니다.",
            self.zoom_target_combo: "줌 명령을 보낼 카메라 대상을 선택합니다.",
            self.zoom_in_button: "기본 Zoom In 명령(SZIN)을 보냅니다.",
            self.zoom_out_button: "기본 Zoom Out 명령(SZOT)을 보냅니다.",
            self.zoom_stop_button: "기본 Zoom Stop 명령(SGFA)을 보냅니다.",
            self.zoom_set_button: "선택한 절대 줌 레벨을 기본 명령(SZVN)으로 보냅니다.",
            self.zoom_level_spin: "WIRIS 가시광 줌 레벨입니다.\n\n" + WIRIS_VISIBLE_ZOOM_FX_TOOLTIP,
            self.wiris_command_target_combo: "선택한 SDK 명령을 왼쪽, 오른쪽, 양쪽 WIRIS 중 어디로 보낼지 선택합니다.",
            self.wiris_command_combo: "공식 WIRIS Ethernet Stream SDK 명령 preset입니다.",
            self.wiris_command_edit: "전송할 SDK 명령 텍스트입니다. <IPv4> 같은 placeholder는 전송 전에 바꾸세요.",
            self.wiris_command_terminator_combo: "명령 구분자입니다. 소프트웨어에서는 NULL을, telnet 방식에서는 LINE을 주로 사용합니다.",
            self.wiris_command_send_button: "편집한 SDK 명령을 선택한 WIRIS 대상으로 전송합니다.",
            self.clock_wiris_time_target_combo: "PC UTC를 설정할 WIRIS RTC 대상을 선택합니다.",
            self.wiris_set_pc_time_button: (
                "선택한 WIRIS 대상의 RTC를 PC 현재 UTC로 설정합니다. "
                "카메라별 HIWS 제어 채널 RTT를 측정해 RTT/2와 파싱 여유를 더한 STUS 목표시각을 사용하고, 측정값을 로그에 남깁니다."
            ),
            self.clock_sync_check_button: (
                "최근 스트림 frame의 RTCP/NTP 기반 시각을 PC UTC와 비교합니다. "
                "프레임 수신 age를 보정해 해당 프레임 시점의 PC UTC offset을 계산하며, WIRIS GDTI와의 일치는 별도 검증 대상입니다."
            ),
            self.wiris_command_log: "카메라 SDK 명령 응답 로그입니다.",
            self.calib_cols_spin: "체커보드 내부 코너의 열 개수입니다.",
            self.calib_rows_spin: "체커보드 내부 코너의 행 개수입니다.",
            self.calib_square_spin: "내부 파라미터 보정에 사용할 체커보드 한 칸 크기입니다.",
            self.calib_min_images_spin: "카메라별 권장 유효 체커보드 프레임 최소 개수입니다.",
            self.calib_calculation_pairs_spin: (
                "내부 파라미터 계산에 실제로 사용할 matching pair 수입니다. "
                "0(전체 pair)이면 입력된 모든 pair를 사용합니다."
            ),
            self.calib_dir_edit: "보정 이미지와 intrinsics.json / intrinsics_zoom_##.json을 저장/로드할 디렉터리입니다.",
            self.capture_calibration_button: "현재 좌/우 카메라 프레임 중 체커보드가 검출된 쪽만 내부 파라미터 보정 이미지로 저장합니다.",
            self.calibration_auto_interval_spin: "자동 촬영 시도 간격입니다. 보드 검출·좌우 시각 동기·중복 검사를 통과한 pair만 저장합니다.",
            self.calibration_auto_target_spin: "자동 촬영을 멈출 유효한 좌/우 체커보드 pair 개수입니다.",
            self.capture_calibration_auto_button: "좌·우 모두 체커보드가 검출되고 RTCP/NTP 시각 차이가 100ms 이하인 pair를 자동으로 저장합니다.",
            self.run_calibration_button: "캡처된 체커보드 이미지로 OpenCV 내부 파라미터 보정을 실행합니다.",
            self.calibration_progress_bar: "내부 파라미터 및 stereo 계산에서 실제 검사한 이미지/pair 기준 진행률입니다.",
            self.save_intrinsics_button: "촬영 당시 양쪽 카메라 GZVV readback으로 확인된 줌 레벨의 내부 파라미터를 intrinsics_zoom_##.json으로 저장하고 latest alias intrinsics.json도 갱신합니다.",
            self.apply_intrinsics_button: "현재 WIRIS 줌 레벨의 intrinsics_zoom_##.json 또는 막 계산한 결과를 거리 측정에 적용합니다.",
            self.add_zoom_intrinsics_file_button: "비교 목록에 저장된 intrinsics JSON 파일을 추가합니다. 파일 자체는 복사하거나 변경하지 않습니다.",
            self.remove_zoom_intrinsics_file_button: "선택한 intrinsics JSON 파일을 비교 목록에서만 제거합니다. 저장된 파일은 삭제하지 않습니다.",
            self.apply_zoom_intrinsics_file_button: "비교 목록에서 선택한 intrinsics JSON 파일을 거리 계산에 바로 적용합니다.",
            self.zoom_intrinsics_compare_list: "저장된 내부 파라미터 파일들을 스크롤로 비교합니다. 항목은 드래그해서 순서를 바꿀 수 있습니다.",
            self.intrinsics_source_combo: "거리 계산에 사용할 내부 파라미터 출처입니다. 체커보드 보정값 또는 수동 초점거리 fallback 중 하나만 선택합니다.",
            self.intrinsics_status_label: "현재 내부 파라미터 적용 상태와 초점거리 출처를 표시합니다.",
            self.distance_mode_combo: "3D ray 거리 계산은 Pixhawk 자세 + GPS/RTK baseline 또는 체커보드 stereo R/T를 사용합니다.",
            self.distance_parameters_label: "현재 거리 계산에 쓰이는 초점거리, baseline/translation, 위치/자세 출처를 요약합니다.",
            self.calibration_left_preview: "체커보드 검출 overlay가 표시되는 왼쪽 보정 미리보기입니다.",
            self.calibration_right_preview: "체커보드 검출 overlay가 표시되는 오른쪽 보정 미리보기입니다.",
            self.focal_spin: (
                "거리 계산에 사용할 초점거리 fx(px) 참고값입니다.\n"
                "3D ray 계산은 적용된 내부 파라미터를 우선 사용하며, 체커보드가 없으면 "
                "`수동 초점거리 사용`을 켠 뒤 추정값을 입력하세요.\n\n"
                + WIRIS_VISIBLE_ZOOM_FX_TOOLTIP
            ),
            self.manual_focal_checkbox: (
                "로드된 내부 파라미터 대신 초점거리를 수동으로 편집할 수 있게 합니다.\n"
                "체커보드가 없을 때만 켜고, 체커보드 보정값을 쓰려면 끄세요.\n\n"
                + WIRIS_VISIBLE_ZOOM_FX_TOOLTIP
            ),
            self.baseline_spin: "거리 계산에 항상 사용하는 수동 카메라 baseline입니다. Baseline 탭의 보조 수동 계산 값을 여기에 적용할 수 있습니다.",
            self.distance_label: "현재 거리 추정값, 계산 방식, 동기화 품질, 경고를 자세히 표시합니다.",
            self.start_button: "YOLO 검출과 3D 거리 측정을 시작하거나, 측정 중이면 Pixhawk 연결은 유지한 채 카메라 Preview로 돌아갑니다.",
            self.record_button: (
                "세션 기록을 시작하거나 중지합니다. 켜면 좌/우 영상 MP4와 측정·카메라·"
                "Pixhawk·GNSS 타임라인을 같은 session id에 기록합니다."
            ),
            self.clear_history_button: "거리 그래프와 아직 저장하지 않은 메모리 기록을 지웁니다.",
            self.runtime_distance_graph_group: "baseline_center_frd Forward/Right/Down/Range 이력 그래프 영역을 접거나 펼칩니다.",
            self.distance_plot: (
                "3D ray 샘플의 baseline_center_frd Forward/Right/Down/Range 이력 그래프입니다. "
                "점 색상은 동기화 품질을 따릅니다.\n"
                "왼쪽 클릭: 점의 Forward/Right/Down과 midpoint 기준 Range를 표시합니다.\n"
                "오른쪽 클릭: 해당 점을 삭제합니다."
            ),
            self.save_history_button: "현재 측정을 typed Parquet과 context.json으로 확정하고 세션 통합 review.xlsx를 갱신합니다.",
            self.save_settings_button: "현재 내부 파라미터 경로와 전체 런타임 설정을 저장합니다.",
            self.recording_status_label: "메모리 내 세션 기록 상태와 매칭 프레임쌍 수입니다.",
            self.recording_sheet_tabs: "저장될 세션 review.xlsx의 주요 시트 구성으로 현재 기록을 미리 보여줍니다.",
            self.recording_sample_detail: "선택한 매칭 프레임쌍의 전체 내부 기록 필드를 표시합니다.",
            self.wiris_log: "WIRIS, 카메라 Preview 연결, 보정, 시간 확인 메시지 로그입니다.",
            self.camera_yolo_log: "YOLO, 거리 측정, 저장 메시지 로그입니다.",
            self.pixhawk_log: "Pixhawk, PX4, MAVLink 메시지 로그입니다.",
        }
        for widget, tooltip in tooltips.items():
            widget.setToolTip(tooltip)
        if hasattr(self, "rm3100_slot_labels"):
            for labels in self.rm3100_slot_labels.values():
                for slot, label in labels.items():
                    label.setToolTip(
                        f"{slot}에 배정된 최신 RM3100 샘플입니다. device_id는 Pixhawk별 최초 발견 순서로 매핑됩니다."
                    )
        if hasattr(self, "f9p_slot_labels"):
            for labels in self.f9p_slot_labels.values():
                for slot, label in labels.items():
                    label.setToolTip(
                        f"{slot}에 배정된 최신 F9P GNSS 샘플입니다. device_id는 Pixhawk별 최초 발견 순서로 매핑됩니다."
                    )
        if hasattr(self, "rm3100_norm_plots"):
            for plot in self.rm3100_norm_plots.values():
                plot.setToolTip("해당 Pixhawk의 RM3100 CAN1/CAN2 자기장 크기 overlay 그래프입니다.")
        if hasattr(self, "rm3100_delta_plots"):
            for plot in self.rm3100_delta_plots.values():
                plot.setToolTip("해당 Pixhawk의 RM3100 CAN1/CAN2 크기 차이와 벡터 각도 차이 그래프입니다.")
        if hasattr(self, "f9p_sats_plots"):
            for plot in self.f9p_sats_plots.values():
                plot.setToolTip("해당 Pixhawk의 F9P GPS1/GPS2 위성 수 overlay 그래프입니다.")
        if hasattr(self, "f9p_quality_plots"):
            for plot in self.f9p_quality_plots.values():
                plot.setToolTip("해당 Pixhawk의 F9P GPS1/GPS2 수평 정확도 overlay 그래프입니다.")

    def _refresh_operation_header(self) -> None:
        self._refresh_results_summary()

    def _refresh_results_summary(self) -> None:
        if not hasattr(self, "session_summary_label"):
            return
        rows = self._recording_preview_rows()
        if not rows:
            latest = self.distance_history[-1].distance_m if self.distance_history else None
            latest_text = "" if latest is None else f"\n최근 표시 거리: {latest:.2f} m"
            self.session_summary_label.setText(f"아직 기록된 매칭 프레임쌍이 없습니다.{latest_text}")
            return
        distances = []
        qualities: dict[str, int] = {}
        for row in rows:
            quality = str(row.get("quality", "unknown"))
            qualities[quality] = qualities.get(quality, 0) + 1
            try:
                distances.append(float(row.get("distance_m", 0.0)))
            except (TypeError, ValueError):
                pass
        latest_text = f"{distances[-1]:.2f} m" if distances else "--"
        average_text = f"{sum(distances) / len(distances):.2f} m" if distances else "--"
        quality_text = ", ".join(f"{name} {count}" for name, count in sorted(qualities.items()))
        self.session_summary_label.setText(
            f"매칭 프레임쌍: {len(self.distance_recording_rows)}개"
            f" (최근 {len(rows)}개 요약)\n"
            f"품질 분포: {quality_text or '없음'}\n"
            f"최신 거리: {latest_text} | 평균 거리: {average_text}"
        )

    def _apply_visual_style(self) -> None:
        base_font = QtGui.QFont("IBM Plex Sans")
        base_font.setPointSize(10)
        self.setFont(base_font)
        self.setStyleSheet(
            f"""
            QWidget {{
                font-family: {FONT_FAMILY};
                background: {APP_BG};
                color: {TEXT_MAIN};
                selection-background-color: {ACCENT};
                selection-color: #ffffff;
            }}
            QTabWidget::pane {{
                border: 1px solid {PANEL_BORDER};
                background: {PANEL_BG};
                top: -1px;
            }}
            QTabBar::tab {{
                background: {PANEL_ALT_BG};
                color: {TEXT_MUTED};
                border: 1px solid {PANEL_BORDER};
                border-bottom: none;
                padding: 7px 12px;
                min-height: 22px;
            }}
            QTabBar::tab:selected {{
                background: {PANEL_BG};
                color: {TEXT_MAIN};
                border-top: 3px solid {ACCENT};
            }}
            QGroupBox {{
                background: {PANEL_BG};
                border: 1px solid {PANEL_BORDER};
                border-radius: 2px;
                margin-top: 10px;
                padding: 10px 10px 8px 10px;
                font-weight: 600;
            }}
            QGroupBox::title {{
                subcontrol-origin: margin;
                left: 8px;
                padding: 0 4px;
                color: {TEXT_MAIN};
            }}
            QLabel {{
                background: transparent;
            }}
            QLineEdit, QPlainTextEdit, QTableWidget, QSpinBox, QDoubleSpinBox, QComboBox {{
                background: {PANEL_ALT_BG};
                color: {TEXT_MAIN};
                border: 1px solid {PANEL_BORDER_STRONG};
                border-radius: 0px;
                padding: 5px 7px;
                min-height: 22px;
            }}
            QHeaderView::section {{
                background: {PANEL_ALT_STRONG};
                color: {TEXT_MAIN};
                border: 1px solid {PANEL_BORDER};
                padding: 4px 6px;
                font-weight: 600;
            }}
            QTableWidget::item:selected {{
                background: {ACCENT};
                color: #ffffff;
            }}
            QPlainTextEdit {{
                font-family: monospace;
            }}
            QLineEdit:focus, QPlainTextEdit:focus, QTableWidget:focus, QSpinBox:focus, QDoubleSpinBox:focus, QComboBox:focus {{
                border-color: {ACCENT};
            }}
            QPushButton {{
                background: {PANEL_BG};
                color: {ACCENT};
                border: 1px solid {PANEL_BORDER_STRONG};
                border-radius: 0px;
                padding: 7px 10px;
                min-height: 28px;
                font-weight: 500;
            }}
            QPushButton:hover {{
                background: {INFO_BG};
                border-color: {ACCENT};
                color: {ACCENT_ACTIVE};
            }}
            QPushButton:disabled {{
                background: {PANEL_ALT_BG};
                color: {TEXT_SUBTLE};
                border-color: {PANEL_BORDER};
            }}
            QPushButton[role="primary"] {{
                background: {ACCENT};
                border-color: {ACCENT};
                color: #ffffff;
            }}
            QPushButton[role="primary"]:hover {{
                background: {ACCENT_HOVER};
                border-color: {ACCENT_HOVER};
            }}
            QPushButton[role="danger"] {{
                background: {ERROR};
                border-color: {ERROR};
                color: #ffffff;
            }}
            QPushButton[role="success"] {{
                background: {SUCCESS};
                border-color: {SUCCESS};
                color: #ffffff;
            }}
            QPushButton[role="toggle"] {{
                background: {PANEL_BG};
                border-color: {PANEL_BORDER_STRONG};
                color: {TEXT_MUTED};
            }}
            QPushButton[role="toggle"]:hover {{
                background: {INFO_BG};
                border-color: {ACCENT};
                color: {ACCENT_ACTIVE};
            }}
            QPushButton[role="toggle"]:checked {{
                background: {INFO_BG};
                border-color: {ACCENT};
                color: {ACCENT_ACTIVE};
            }}
            QPushButton[role="toggle"]:checked:hover {{
                background: {PANEL_ALT_STRONG};
                border-color: {ACCENT_HOVER};
            }}
            QCheckBox {{
                spacing: 7px;
                background: transparent;
            }}
            QLabel#statusStrip, QLabel#statChip {{
                background: {PANEL_ALT_BG};
                border: 1px solid {PANEL_BORDER};
                border-left: 3px solid {ACCENT};
                border-radius: 0px;
                color: {TEXT_MAIN};
                padding: 5px 7px;
            }}
            QLabel#distanceStatus {{
                background: {PANEL_ALT_BG};
                border: 1px solid {ACCENT};
                border-radius: 0px;
                padding: 7px 10px;
                font-size: 18px;
                font-weight: 700;
            }}
            QGroupBox#cameraPane {{
                margin-top: 16px;
            }}
            QLabel#cameraPreview, QLabel#calibrationPreview {{
                background: {INVERSE_BG};
                border: 1px solid {PANEL_BORDER_STRONG};
                border-radius: 0px;
                color: {INVERSE_TEXT_MUTED};
            }}
            QLabel#cameraMeta {{
                color: {TEXT_MUTED};
                padding: 2px 4px;
                font-weight: 500;
            }}
            QScrollBar:vertical, QScrollBar:horizontal {{
                background: {PANEL_ALT_BG};
                border: none;
                margin: 0;
            }}
            QScrollBar::handle {{
                background: {PANEL_BORDER_STRONG};
                border-radius: 0px;
                min-height: 24px;
                min-width: 24px;
            }}
            """
        )

    def _polish_controls(self) -> None:
        self._polish_form_layouts()
        primary_buttons = (
            self.detect_control_button,
            self.right_detect_control_button,
            self.left_control_apply_button,
            self.right_control_apply_button,
            self.auto_wiris_connect_button,
            self.detect_rtsp_button,
            self.right_detect_rtsp_button,
            self.left_rtsp_apply_button,
            self.right_rtsp_apply_button,
            self.browse_weights_button,
            self.camera_connect_button,
            self.left_mavlink_button,
            self.right_mavlink_button,
            self.start_button,
            self.capture_calibration_button,
            self.capture_calibration_auto_button,
            self.run_calibration_button,
            self.save_intrinsics_button,
            self.save_settings_button,
        )
        danger_buttons = ()
        success_buttons = (
            self.save_history_button,
            self.record_button,
            self.apply_intrinsics_button,
        )
        for button in primary_buttons:
            button.setProperty("role", "primary")
        for button in danger_buttons:
            button.setProperty("role", "danger")
        for button in success_buttons:
            button.setProperty("role", "success")
        for button in self.findChildren(QtWidgets.QPushButton):
            button.setCursor(QtCore.Qt.PointingHandCursor)
            button.setMinimumWidth(max(button.minimumWidth(), button.sizeHint().width() + 6, FORM_BUTTON_MIN_WIDTH))
            button.setMinimumHeight(max(button.minimumHeight(), button.sizeHint().height(), 38))
            button.style().unpolish(button)
            button.style().polish(button)
        for line_edit in self.findChildren(QtWidgets.QLineEdit):
            if line_edit.property("compactNetworkField") or line_edit.property("compactFormField"):
                continue
            line_edit.setMinimumWidth(max(line_edit.minimumWidth(), 220))
        for combo in self.findChildren(QtWidgets.QComboBox):
            combo.setMinimumWidth(max(combo.minimumWidth(), 170))
        for spin in self.findChildren(QtWidgets.QAbstractSpinBox):
            if spin.property("compactNetworkField") or spin.property("compactFormField"):
                continue
            spin.setMinimumWidth(max(spin.minimumWidth(), 120))
        self._polish_button_rows()
        self._reserve_pixhawk_pair_settings_height()
        self._reserve_pixhawk_candidate_group_height()

    def _reserve_pixhawk_pair_settings_height(self) -> None:
        group = getattr(self, "pixhawk_pair_settings_group", None)
        if group is None:
            return
        layout = group.layout()
        if layout is not None:
            layout.activate()
        group.setMinimumHeight(max(group.minimumHeight(), group.sizeHint().height() + 16))

    def _reserve_pixhawk_candidate_group_height(self) -> None:
        group = getattr(self, "pixhawk_candidate_group", None)
        if group is None:
            return
        for table in (
            getattr(self, "pc_pixhawk_candidate_table", None),
            getattr(self, "mavlink_pixhawk_candidate_table", None),
        ):
            if table is not None:
                table.setFixedHeight(240)
        layout = group.layout()
        if layout is not None:
            layout.activate()
        group.setMinimumHeight(max(group.minimumHeight(), group.sizeHint().height()))

    def _polish_form_layouts(self) -> None:
        layouts: list[QtWidgets.QLayout] = []
        root_layout = self.layout()
        if root_layout is not None:
            layouts.append(root_layout)
        layouts.extend(self.findChildren(QtWidgets.QLayout))
        for layout in dict.fromkeys(layouts):
            if isinstance(layout, QtWidgets.QGridLayout):
                self._polish_grid_form_layout(layout)
            elif isinstance(layout, QtWidgets.QHBoxLayout):
                self._polish_inline_form_layout(layout)

    def _iter_layouts(self, layout: QtWidgets.QLayout, seen: set[int] | None = None):
        if seen is None:
            seen = set()
        layout_id = id(layout)
        if layout_id in seen:
            return
        seen.add(layout_id)
        yield layout
        for index in range(layout.count()):
            item = layout.itemAt(index)
            child_layout = item.layout()
            if child_layout is not None:
                yield from self._iter_layouts(child_layout, seen)
            child_widget = item.widget()
            if child_widget is not None and child_widget.layout() is not None:
                yield from self._iter_layouts(child_widget.layout(), seen)

    def _polish_grid_form_layout(self, layout: QtWidgets.QGridLayout) -> None:
        for index in range(layout.count()):
            item = layout.itemAt(index)
            widget = item.widget()
            if not isinstance(widget, QtWidgets.QLabel) or not self._is_form_label(widget):
                continue
            row, column, _row_span, _column_span = layout.getItemPosition(index)
            next_item = layout.itemAtPosition(row, column + 1)
            if next_item is None or not self._layout_item_has_form_field(next_item):
                continue
            self._polish_form_label(widget)
            label_min_width = widget.minimumWidth()
            layout.setColumnMinimumWidth(column, max(layout.columnMinimumWidth(column), label_min_width))
            layout.setColumnStretch(column + 1, max(layout.columnStretch(column + 1), 1))

    def _polish_inline_form_layout(self, layout: QtWidgets.QHBoxLayout) -> None:
        for index in range(layout.count() - 1):
            item = layout.itemAt(index)
            next_item = layout.itemAt(index + 1)
            widget = item.widget()
            if not isinstance(widget, QtWidgets.QLabel) or not self._is_form_label(widget):
                continue
            if not self._layout_item_has_form_field(next_item):
                continue
            self._polish_form_label(widget)

    def _layout_item_has_form_field(self, item: QtWidgets.QLayoutItem) -> bool:
        widget = item.widget()
        if widget is not None:
            return isinstance(
                widget,
                (
                    QtWidgets.QLineEdit,
                    QtWidgets.QComboBox,
                    QtWidgets.QAbstractSpinBox,
                ),
            )
        layout = item.layout()
        if layout is None:
            return False
        for index in range(layout.count()):
            if self._layout_item_has_form_field(layout.itemAt(index)):
                return True
        return False

    def _is_form_label(self, label: QtWidgets.QLabel) -> bool:
        if not label.text().strip():
            return False
        if label.objectName() in {
            "statusStrip",
            "statChip",
            "distanceStatus",
            "cameraPreview",
            "calibrationPreview",
            "cameraMeta",
        }:
            return False
        if label.pixmap() is not None and not label.pixmap().isNull():
            return False
        return True

    def _polish_form_label(self, label: QtWidgets.QLabel) -> None:
        if (
            label.property("compactNetworkLabel")
            or label.property("compactAdvancedLabel")
            or label.property("compactFormLabel")
        ):
            label.setMinimumWidth(max(label.minimumWidth(), label.sizeHint().width() + 8))
        else:
            label.setMinimumWidth(max(label.minimumWidth(), FORM_LABEL_MIN_WIDTH))
        label.setAlignment(QtCore.Qt.AlignRight | QtCore.Qt.AlignVCenter)

    def _polish_button_rows(self) -> None:
        layouts: list[QtWidgets.QLayout] = []
        root_layout = self.layout()
        if root_layout is not None:
            layouts.append(root_layout)
        layouts.extend(self.findChildren(QtWidgets.QLayout))
        for layout in dict.fromkeys(layouts):
            if isinstance(layout, QtWidgets.QGridLayout):
                self._polish_grid_button_rows(layout)
            elif isinstance(layout, QtWidgets.QHBoxLayout):
                self._polish_hbox_button_row(layout)

    def _polish_grid_button_rows(self, layout: QtWidgets.QGridLayout) -> None:
        row_buttons: dict[int, list[tuple[int, int, QtWidgets.QPushButton]]] = {}
        rows_with_form_fields: set[int] = set()
        for index in range(layout.count()):
            item = layout.itemAt(index)
            widget = item.widget()
            if isinstance(widget, (QtWidgets.QLineEdit, QtWidgets.QComboBox, QtWidgets.QAbstractSpinBox)):
                row, _column, _row_span, _column_span = layout.getItemPosition(index)
                rows_with_form_fields.add(row)
                continue
            if not isinstance(widget, QtWidgets.QPushButton):
                continue
            row, column, _row_span, column_span = layout.getItemPosition(index)
            row_buttons.setdefault(row, []).append((column, column_span, widget))
        for row, buttons in row_buttons.items():
            if row in rows_with_form_fields:
                continue
            if len(buttons) < 2:
                continue
            for column, column_span, button in buttons:
                self._make_button_expand(button)
                for stretched_column in range(column, column + column_span):
                    layout.setColumnStretch(stretched_column, max(layout.columnStretch(stretched_column), 1))

    def _polish_hbox_button_row(self, layout: QtWidgets.QHBoxLayout) -> None:
        button_indexes: list[tuple[int, QtWidgets.QPushButton]] = []
        non_button_widgets = 0
        for index in range(layout.count()):
            widget = layout.itemAt(index).widget()
            if isinstance(widget, QtWidgets.QPushButton):
                button_indexes.append((index, widget))
            elif widget is not None and widget.isVisible():
                non_button_widgets += 1
        if len(button_indexes) < 2 or non_button_widgets:
            return
        for index, button in button_indexes:
            self._make_button_expand(button)
            layout.setStretch(index, 1)

    def _make_button_expand(self, button: QtWidgets.QPushButton) -> None:
        button.setSizePolicy(QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Fixed)

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

        gps_ready = [
            self._gps_fix_ready_for_baseline(self.latest_gps.get("left")),
            self._gps_fix_ready_for_baseline(self.latest_gps.get("right")),
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
        self._build_px4_mag_manager_panel(yaw_layout)
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

    def _build_gnss_base_tab(self, layout: QtWidgets.QVBoxLayout) -> None:
        layout.setSpacing(8)
        self.gnss_base_status_labels: dict[str, QtWidgets.QLabel] = {}
        self.gnss_base_status_title_labels: dict[str, QtWidgets.QLabel] = {}
        self.gnss_base_quality_labels: dict[str, QtWidgets.QLabel] = {}
        scroll_area = QtWidgets.QScrollArea()
        scroll_area.setObjectName("gnssBaseScrollArea")
        scroll_area.setWidgetResizable(True)
        scroll_area.setFrameShape(QtWidgets.QFrame.NoFrame)
        content = QtWidgets.QWidget()
        content_layout = QtWidgets.QVBoxLayout(content)
        content_layout.setContentsMargins(0, 0, 0, 0)
        content_layout.setSpacing(8)

        operation_config = self.settings.gnss_operation
        self.gnss_site_label_edit = QtWidgets.QLineEdit(operation_config.site_label)
        self.gnss_site_label_edit.setPlaceholderText("이번 실험의 장소/지점명")
        self.gnss_site_address_edit = QtWidgets.QLineEdit(operation_config.site_address)
        self.gnss_site_address_edit.setPlaceholderText("이번 실험의 주소 또는 현장 설명")
        self.gnss_operation_mode_combo = QtWidgets.QComboBox()
        self.gnss_operation_mode_combo.addItem("현장 Base Survey-in", GnssOperationMode.LOCAL_SURVEY_IN.value)
        self.gnss_operation_mode_combo.addItem(
            "NTRIP 보조 Fixed Base", GnssOperationMode.NETWORK_ASSISTED_FIXED_BASE.value
        )
        self.gnss_operation_mode_combo.addItem("Network RTK 직접", GnssOperationMode.NETWORK_RTK_DIRECT.value)
        self._set_combo_data(self.gnss_operation_mode_combo, operation_config.operation_mode.value)
        self.gnss_operation_mode_combo.currentIndexChanged.connect(
            lambda _index: self._handle_gnss_operation_mode_changed()
        )
        self.gnss_ntrip_host_edit = QtWidgets.QLineEdit(operation_config.network.caster_host)
        self.gnss_ntrip_host_edit.setPlaceholderText("caster.example.com (hostname only)")
        self.gnss_ntrip_port_spin = QtWidgets.QSpinBox()
        self.gnss_ntrip_port_spin.setRange(1, 65535)
        self.gnss_ntrip_port_spin.setValue(operation_config.network.caster_port)
        self.gnss_ntrip_mountpoint_edit = QtWidgets.QLineEdit(operation_config.network.mountpoint)
        self.gnss_ntrip_mountpoint_edit.setPlaceholderText("MOUNTPOINT")
        self.gnss_ntrip_username_edit = QtWidgets.QLineEdit(operation_config.network.username)
        self.gnss_ntrip_username_edit.setPlaceholderText("NTRIP username")
        # Session-only: never pre-filled from a stored password, and never
        # read back into GnssOperationConfig/settings JSON. See
        # gnss_runtime_controller._resolve_ntrip_credentials for how this
        # value is used (and workstation_core/ntrip_credentials.py for where
        # "Remember" actually persists it).
        self.gnss_ntrip_password_edit = QtWidgets.QLineEdit()
        self.gnss_ntrip_password_edit.setEchoMode(QtWidgets.QLineEdit.Password)
        self.gnss_ntrip_password_edit.setPlaceholderText("NTRIP password")
        self.gnss_ntrip_remember_checkbox = QtWidgets.QCheckBox("Remember password securely")
        self.gnss_ntrip_credential_status_label = QtWidgets.QLabel("--")
        self.gnss_ntrip_forget_password_button = QtWidgets.QPushButton("Forget saved password")
        self.gnss_ntrip_forget_password_button.clicked.connect(self.forget_gnss_ntrip_password)
        self.gnss_ntrip_host_edit.editingFinished.connect(self._refresh_ntrip_credential_status)
        self.gnss_ntrip_mountpoint_edit.editingFinished.connect(self._refresh_ntrip_credential_status)
        self.gnss_ntrip_username_edit.editingFinished.connect(self._refresh_ntrip_credential_status)
        # Legacy indirection, kept only so settings files written before this
        # UI existed keep authenticating.  New setups should leave these
        # alone and use the Username/Password fields above instead.
        self.gnss_ntrip_username_env_edit = QtWidgets.QLineEdit(operation_config.network.username_env)
        self.gnss_ntrip_password_env_edit = QtWidgets.QLineEdit(operation_config.network.password_env)
        self.gnss_ntrip_gga_policy_combo = QtWidgets.QComboBox()
        self.gnss_ntrip_gga_policy_combo.addItem("Base GNSS (보조 Fixed Base)", GgaPolicy.BASE_GNSS.value)
        self.gnss_ntrip_gga_policy_combo.addItem("좌우 중심", GgaPolicy.SYSTEM_CENTER.value)
        self.gnss_ntrip_gga_policy_combo.addItem("Left GNSS", GgaPolicy.LEFT_GNSS.value)
        self.gnss_ntrip_gga_policy_combo.addItem("수동/미사용", GgaPolicy.MANUAL.value)
        self.gnss_ntrip_gga_policy_combo.addItem("세션별 rover", GgaPolicy.PER_ROVER_SESSION.value)
        self._set_combo_data(self.gnss_ntrip_gga_policy_combo, operation_config.network.gga_policy.value)
        self.gnss_fixed_height_combo = QtWidgets.QComboBox()
        self.gnss_fixed_height_combo.addItem("WGS84 ellipsoid height", HeightType.ELLIPSOID.value)
        self.gnss_fixed_height_combo.addItem("MSL (고정 좌표 전 변환 필요)", HeightType.MSL.value)
        self.gnss_fixed_height_combo.addItem("Unknown / not validated", HeightType.UNKNOWN.value)
        self._set_combo_data(self.gnss_fixed_height_combo, operation_config.fixed_base.height_type.value)
        self.gnss_fixed_acquisition_duration_spin = QtWidgets.QSpinBox()
        self.gnss_fixed_acquisition_duration_spin.setRange(1, 86400)
        self.gnss_fixed_acquisition_duration_spin.setValue(int(operation_config.fixed_base.acquisition_duration_s))
        self.gnss_fixed_min_duration_spin = QtWidgets.QSpinBox()
        self.gnss_fixed_min_duration_spin.setRange(1, 86400)
        self.gnss_fixed_min_duration_spin.setValue(int(operation_config.fixed_base.minimum_fixed_duration_s))
        self.gnss_rover_min_fix_type_spin = QtWidgets.QSpinBox()
        self.gnss_rover_min_fix_type_spin.setRange(0, 255)
        self.gnss_rover_min_fix_type_spin.setValue(int(operation_config.readiness.minimum_fix_type))
        self.gnss_base_serial_port_edit = QtWidgets.QLineEdit(
            preferred_gnss_base_serial_port(self.settings.gnss_base_serial_port)
        )
        self.gnss_base_baud_spin = QtWidgets.QSpinBox()
        self.gnss_base_baud_spin.setRange(1200, 4000000)
        self.gnss_base_baud_spin.setValue(int(self.settings.gnss_base_baud))
        self.gnss_base_survey_duration_spin = QtWidgets.QSpinBox()
        self.gnss_base_survey_duration_spin.setRange(1, 86400)
        self.gnss_base_survey_duration_spin.setValue(int(self.settings.gnss_base_survey_in_min_duration_s))
        self.gnss_base_survey_accuracy_spin = QtWidgets.QDoubleSpinBox()
        self.gnss_base_survey_accuracy_spin.setDecimals(3)
        self.gnss_base_survey_accuracy_spin.setRange(0.001, 100.0)
        self.gnss_base_survey_accuracy_spin.setSingleStep(0.1)
        self.gnss_base_survey_accuracy_spin.setValue(float(self.settings.gnss_base_survey_in_accuracy_m))
        self.gnss_base_rtcm_target_combo = QtWidgets.QComboBox()
        self.gnss_base_rtcm_target_combo.addItem("비활성", "disabled")
        self.gnss_base_rtcm_target_combo.addItem("좌/우 Pixhawk", "both")
        self.gnss_base_rtcm_target_combo.addItem("Left Pixhawk", "left")
        self.gnss_base_rtcm_target_combo.addItem("Right Pixhawk", "right")
        self._set_combo_data(self.gnss_base_rtcm_target_combo, self.settings.gnss_base_rtcm_injection_target)

        def add_setting(form: QtWidgets.QGridLayout, row: int, title: str, widget: QtWidgets.QWidget) -> None:
            form.addWidget(QtWidgets.QLabel(title), row, 0)
            form.addWidget(widget, row, 1)
            form.setColumnStretch(1, 1)

        self.gnss_common_config_group = QtWidgets.QGroupBox("GNSS 운용")
        common_form = QtWidgets.QGridLayout(self.gnss_common_config_group)
        add_setting(common_form, 0, "GNSS 운용 모드", self.gnss_operation_mode_combo)
        add_setting(common_form, 1, "실험 장소/지점명", self.gnss_site_label_edit)
        add_setting(common_form, 2, "실험 주소/현장 설명", self.gnss_site_address_edit)
        add_setting(common_form, 3, "RTCM 주입 대상", self.gnss_base_rtcm_target_combo)
        add_setting(common_form, 4, "Rover 최소 fix_type (3=3D, 6=Fixed)", self.gnss_rover_min_fix_type_spin)

        self.gnss_base_config_group = QtWidgets.QGroupBox("Base Serial 설정")
        base_config_form = QtWidgets.QGridLayout(self.gnss_base_config_group)
        add_setting(base_config_form, 0, "Base serial port", self.gnss_base_serial_port_edit)
        add_setting(base_config_form, 1, "Baudrate", self.gnss_base_baud_spin)

        self.gnss_survey_config_group = QtWidgets.QGroupBox("Local Survey-in 설정")
        survey_config_form = QtWidgets.QGridLayout(self.gnss_survey_config_group)
        add_setting(survey_config_form, 0, "Survey-in 최소 시간(s)", self.gnss_base_survey_duration_spin)
        add_setting(survey_config_form, 1, "Survey-in 목표 정확도(m)", self.gnss_base_survey_accuracy_spin)

        self.gnss_network_config_group = QtWidgets.QGroupBox("임의 NTRIP caster 설정")
        network_config_form = QtWidgets.QGridLayout(self.gnss_network_config_group)
        add_setting(network_config_form, 0, "NTRIP caster host", self.gnss_ntrip_host_edit)
        add_setting(network_config_form, 1, "NTRIP port", self.gnss_ntrip_port_spin)
        add_setting(network_config_form, 2, "NTRIP mountpoint", self.gnss_ntrip_mountpoint_edit)
        add_setting(network_config_form, 3, "NTRIP username", self.gnss_ntrip_username_edit)
        add_setting(network_config_form, 4, "NTRIP password", self.gnss_ntrip_password_edit)
        add_setting(network_config_form, 5, "", self.gnss_ntrip_remember_checkbox)
        add_setting(network_config_form, 6, "Credential status", self.gnss_ntrip_credential_status_label)
        add_setting(network_config_form, 7, "", self.gnss_ntrip_forget_password_button)
        add_setting(network_config_form, 8, "NTRIP GGA 정책", self.gnss_ntrip_gga_policy_combo)

        self.gnss_network_legacy_env_group = QtWidgets.QGroupBox(
            "레거시 환경변수 인증 (선택 사항 -- 위 Username/Password가 비어 있을 때만 fallback으로 사용)"
        )
        network_legacy_env_form = QtWidgets.QGridLayout(self.gnss_network_legacy_env_group)
        add_setting(network_legacy_env_form, 0, "NTRIP username 환경변수", self.gnss_ntrip_username_env_edit)
        add_setting(network_legacy_env_form, 1, "NTRIP password 환경변수", self.gnss_ntrip_password_env_edit)

        self.gnss_fixed_config_group = QtWidgets.QGroupBox("Fixed Base 설정")
        fixed_config_form = QtWidgets.QGridLayout(self.gnss_fixed_config_group)
        add_setting(fixed_config_form, 0, "Fixed Base 높이 기준", self.gnss_fixed_height_combo)
        add_setting(fixed_config_form, 1, "Fixed 좌표 수집 시간(s)", self.gnss_fixed_acquisition_duration_spin)
        add_setting(fixed_config_form, 2, "Fixed 상태 최소 시간(s)", self.gnss_fixed_min_duration_spin)

        self.gnss_base_connect_button = QtWidgets.QPushButton("Connect Base GNSS")
        self.gnss_base_connect_button.clicked.connect(self.connect_gnss_base_serial)
        self.gnss_base_disconnect_button = QtWidgets.QPushButton("Disconnect Base GNSS")
        self.gnss_base_disconnect_button.clicked.connect(self.disconnect_gnss_base_serial)
        self.gnss_base_disconnect_button.setEnabled(False)
        self.gnss_base_survey_start_button = QtWidgets.QPushButton("Start Survey-in")
        self.gnss_base_survey_start_button.clicked.connect(self.start_gnss_base_survey_in)
        self.gnss_base_survey_start_button.setEnabled(False)
        self.gnss_base_survey_stop_button = QtWidgets.QPushButton("Stop Survey-in")
        self.gnss_base_survey_stop_button.clicked.connect(self.stop_gnss_base_survey_in)
        self.gnss_base_survey_stop_button.setEnabled(False)
        self.gnss_network_start_button = QtWidgets.QPushButton("Start NTRIP Corrections")
        self.gnss_network_start_button.clicked.connect(self.start_gnss_network)
        self.gnss_network_stop_button = QtWidgets.QPushButton("Stop NTRIP Corrections")
        self.gnss_network_stop_button.clicked.connect(self.stop_gnss_network)
        self.gnss_network_stop_button.setEnabled(False)
        self.gnss_base_discover_button = QtWidgets.QPushButton("Scan Base GNSS Ports")
        self.gnss_base_discover_button.clicked.connect(self.discover_gnss_base_serial_ports)
        self.gnss_base_apply_candidate_button = QtWidgets.QPushButton("Apply Selected Port")
        self.gnss_base_apply_candidate_button.clicked.connect(self.apply_selected_gnss_base_candidate)
        self.gnss_base_apply_candidate_button.setEnabled(False)
        button_row = QtWidgets.QHBoxLayout()
        button_row.addWidget(self.gnss_base_discover_button)
        button_row.addWidget(self.gnss_base_connect_button)
        button_row.addWidget(self.gnss_base_disconnect_button)
        button_row.addWidget(self.gnss_base_survey_start_button)
        button_row.addWidget(self.gnss_base_survey_stop_button)
        button_row.addWidget(self.gnss_network_start_button)
        button_row.addWidget(self.gnss_network_stop_button)
        button_row.addWidget(self.gnss_base_apply_candidate_button)
        button_row.addStretch(1)
        self.gnss_base_control_group = QtWidgets.QGroupBox("GNSS 운용 제어")
        self.gnss_base_control_group.setLayout(button_row)

        self.gnss_base_candidate_table = QtWidgets.QTableWidget(0, 5)
        self.gnss_base_candidate_table.setHorizontalHeaderLabels(["Port", "Target", "Baud", "상태", "UBX 메시지"])
        self.gnss_base_candidate_table.verticalHeader().setVisible(False)
        self.gnss_base_candidate_table.setSelectionBehavior(QtWidgets.QAbstractItemView.SelectRows)
        self.gnss_base_candidate_table.setSelectionMode(QtWidgets.QAbstractItemView.SingleSelection)
        self.gnss_base_candidate_table.setEditTriggers(QtWidgets.QAbstractItemView.NoEditTriggers)
        self.gnss_base_candidate_table.setMinimumHeight(110)
        self.gnss_base_candidate_table.itemSelectionChanged.connect(self._refresh_gnss_base_candidate_actions)
        gnss_candidate_header = self.gnss_base_candidate_table.horizontalHeader()
        gnss_candidate_header.setSectionResizeMode(QtWidgets.QHeaderView.Stretch)
        self.gnss_base_candidate_group = QtWidgets.QGroupBox("Base 포트 후보")
        candidate_layout = QtWidgets.QVBoxLayout(self.gnss_base_candidate_group)
        candidate_layout.addWidget(self.gnss_base_candidate_table)

        content_layout.addWidget(self.gnss_common_config_group)
        content_layout.addWidget(self.gnss_base_config_group)
        content_layout.addWidget(self.gnss_survey_config_group)
        content_layout.addWidget(self.gnss_network_config_group)
        content_layout.addWidget(self.gnss_network_legacy_env_group)
        content_layout.addWidget(self.gnss_fixed_config_group)
        content_layout.addWidget(self.gnss_base_control_group)
        content_layout.addWidget(self.gnss_base_candidate_group)

        base_group = QtWidgets.QGroupBox("GNSS Base 연결")
        base_group.setObjectName("gnssBaseConnectionGroup")
        base_group.setMinimumHeight(190)
        base_grid = QtWidgets.QGridLayout(base_group)
        base_items = (
            ("connection", "Base 연결", "대기 중", "idle"),
            ("operation", "측정 준비 상태", "대기 중", "idle"),
            ("network", "NTRIP", "미연결", "idle"),
            ("survey_in", "Survey-in", "미연결", "idle"),
            ("base_satellites", "Base 위성 수", "대기 중", "idle"),
            ("rtcm_rx", "Raw 수신", "없음", "idle"),
            ("rtcm_injection", "RTCM 주입", "비활성", "idle"),
        )
        for row, (key, title, text, level) in enumerate(base_items):
            label = QtWidgets.QLabel(text)
            label.setObjectName("statusStrip")
            label.setWordWrap(True)
            title_label = QtWidgets.QLabel(title)
            base_grid.addWidget(title_label, row, 0)
            base_grid.addWidget(label, row, 1)
            self.gnss_base_status_labels[key] = label
            self.gnss_base_status_title_labels[key] = title_label
            self._set_health_label(label, text, level)

        base_fixed_group = QtWidgets.QGroupBox("Base RTK-Fixed 진행")
        base_fixed_group.setObjectName("gnssBaseFixedProgressGroup")
        base_fixed_grid = QtWidgets.QGridLayout(base_fixed_group)
        base_fixed_label = QtWidgets.QLabel("대기 중")
        base_fixed_label.setObjectName("statusStrip")
        base_fixed_label.setWordWrap(True)
        base_fixed_grid.addWidget(QtWidgets.QLabel("Base Fixed 상태"), 0, 0)
        base_fixed_grid.addWidget(base_fixed_label, 0, 1)
        self.gnss_base_status_labels["base_fixed"] = base_fixed_label
        self._set_health_label(base_fixed_label, "대기 중", "idle")
        base_position_label = QtWidgets.QLabel("대기 중")
        base_position_label.setObjectName("statusStrip")
        base_position_label.setWordWrap(True)
        base_fixed_grid.addWidget(QtWidgets.QLabel("Base 위치"), 1, 0)
        base_fixed_grid.addWidget(base_position_label, 1, 1)
        self.gnss_base_status_labels["base_position"] = base_position_label
        self._set_health_label(base_position_label, "대기 중", "idle")
        base_fixed_plot_row = QtWidgets.QHBoxLayout()
        self.gnss_base_fixed_state_plot = TelemetryHistoryPlot("Base RTK Fix 상태 (Float→Fixed)", "fix")
        self.gnss_base_fixed_state_plot.setMinimumHeight(180)
        self.gnss_base_fixed_scatter_plot = TelemetryHistoryPlot("Fixed 좌표 수렴 (scatter H/V)", "m")
        self.gnss_base_fixed_scatter_plot.setMinimumHeight(180)
        base_fixed_plot_row.addWidget(self.gnss_base_fixed_state_plot, 1)
        base_fixed_plot_row.addWidget(self.gnss_base_fixed_scatter_plot, 1)
        base_fixed_grid.addLayout(base_fixed_plot_row, 2, 0, 1, 2)
        self.gnss_base_fixed_progress_group = base_fixed_group

        rtcm_group = QtWidgets.QGroupBox("RTCM 보정 데이터")
        rtcm_grid = QtWidgets.QGridLayout(rtcm_group)
        rtcm_items = (
            ("rtcm_receive", "수신", "RTCM 대기 중", "idle"),
            ("rtcm_messages", "메시지", "type 대기 중", "idle"),
            ("rtcm_injection_stats", "주입", "대상 대기 중", "idle"),
            ("rtcm_bandwidth", "대역폭", "packet 대기 중", "idle"),
        )
        for row, (key, title, text, level) in enumerate(rtcm_items):
            label = QtWidgets.QLabel(text)
            label.setObjectName("statusStrip")
            label.setWordWrap(True)
            rtcm_grid.addWidget(QtWidgets.QLabel(title), row, 0)
            rtcm_grid.addWidget(label, row, 1)
            self.gnss_base_status_labels[key] = label
            self._set_health_label(label, text, level)

        rover_group = QtWidgets.QGroupBox("Rover RTK 상태")
        rover_grid = QtWidgets.QGridLayout(rover_group)
        rover_items = (
            ("left_rover", "Left rover", "GPS/RTK 대기 중"),
            ("right_rover", "Right rover", "GPS/RTK 대기 중"),
            ("baseline", "GPS/RTK baseline", "대기 중"),
        )
        for row, (key, title, text) in enumerate(rover_items):
            label = QtWidgets.QLabel(text)
            label.setObjectName("statusStrip")
            label.setWordWrap(True)
            rover_grid.addWidget(QtWidgets.QLabel(title), row, 0)
            rover_grid.addWidget(label, row, 1)
            self.gnss_base_status_labels[key] = label
            self._set_health_label(label, text, "idle")

        survey_group = QtWidgets.QGroupBox("Survey-in 진행")
        survey_group.setObjectName("gnssBaseSurveyProgressGroup")
        survey_group.setMinimumHeight(320)
        survey_layout = QtWidgets.QVBoxLayout(survey_group)
        survey_summary_row = QtWidgets.QHBoxLayout()
        self.gnss_base_survey_progress_bar = QtWidgets.QProgressBar()
        self.gnss_base_survey_progress_bar.setRange(0, 100)
        self.gnss_base_survey_progress_bar.setValue(0)
        self.gnss_base_survey_progress_bar.setFormat("not running | time 0% | accuracy --")
        self.gnss_base_survey_progress_bar.setMinimumHeight(24)
        self.gnss_base_survey_progress_bar.setSizePolicy(QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Fixed)
        self.gnss_base_survey_accuracy_label = QtWidgets.QLabel("accuracy 대기 중")
        self.gnss_base_survey_accuracy_label.setObjectName("statusStrip")
        self.gnss_base_survey_accuracy_label.setWordWrap(True)
        survey_summary_row.addWidget(self.gnss_base_survey_progress_bar, 2)
        survey_summary_row.addWidget(self.gnss_base_survey_accuracy_label, 1)
        survey_layout.addLayout(survey_summary_row)
        self.gnss_base_survey_convergence_label = QtWidgets.QLabel("수렴: 대기 중 | ETA: -- | 최근 개선: --")
        self.gnss_base_survey_convergence_label.setObjectName("statusStrip")
        self.gnss_base_survey_convergence_label.setWordWrap(True)
        survey_layout.addWidget(self.gnss_base_survey_convergence_label)

        survey_plot_row = QtWidgets.QHBoxLayout()
        self.gnss_base_survey_accuracy_plot = TelemetryHistoryPlot("Survey-in accuracy", "m")
        self.gnss_base_survey_accuracy_plot.setMinimumHeight(180)
        self.gnss_base_survey_observation_plot = TelemetryHistoryPlot("Survey-in observations", "obs")
        self.gnss_base_survey_observation_plot.setMinimumHeight(180)
        survey_plot_row.addWidget(self.gnss_base_survey_accuracy_plot, 1)
        survey_plot_row.addWidget(self.gnss_base_survey_observation_plot, 1)
        survey_layout.addLayout(survey_plot_row)

        quality_group = QtWidgets.QGroupBox("Survey-in 품질 진단")
        self.gnss_survey_quality_group = quality_group
        quality_grid = QtWidgets.QGridLayout(quality_group)
        quality_items = (
            ("satellites", "위성", "NAV-SAT 대기 중"),
            ("signal", "신호", "C/N0 대기 중"),
            ("elevation", "고도각", "elevation 대기 중"),
            ("constellations", "구성", "constellation 대기 중"),
        )
        for row, (key, title, text) in enumerate(quality_items):
            label = QtWidgets.QLabel(text)
            label.setObjectName("statusStrip")
            label.setWordWrap(True)
            quality_grid.addWidget(QtWidgets.QLabel(title), row, 0)
            quality_grid.addWidget(label, row, 1)
            self.gnss_base_quality_labels[key] = label
            self._set_health_label(label, text, "idle")

        note = QtWidgets.QLabel(
            "현장 Base Survey-in, NTRIP 보조 Fixed Base, Network RTK 직접 모드를 지원합니다. "
            "NTRIP 비밀번호는 설정 파일에 저장하지 않고 지정한 환경변수에서 읽습니다."
        )
        note.setWordWrap(True)
        note.setObjectName("statusStrip")

        content_layout.addWidget(base_group)
        content_layout.addWidget(base_fixed_group)
        content_layout.addWidget(rtcm_group)
        content_layout.addWidget(rover_group)
        self.gnss_survey_progress_group = survey_group
        content_layout.addWidget(survey_group)
        content_layout.addWidget(quality_group)
        content_layout.addWidget(note)
        content_layout.addStretch(1)
        scroll_area.setWidget(content)
        layout.addWidget(scroll_area, 1)
        self._refresh_gnss_base_tab()
        self._handle_gnss_operation_mode_changed()

    def _new_gnss_base_rtcm_stats(self) -> dict[str, object]:
        return {
            "start_s": time.monotonic(),
            "last_frame_s": None,
            "frames": 0,
            "bytes": 0,
            "max_frame_bytes": 0,
            "message_types": {},
            "injection": {
                "left": {"frames": 0, "bytes": 0, "packets": 0, "fail": 0},
                "right": {"frames": 0, "bytes": 0, "packets": 0, "fail": 0},
                "drone": {"frames": 0, "bytes": 0, "packets": 0, "fail": 0},
            },
        }

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
            ("eph", "eph"),
            ("epv", "epv"),
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
        for combo in (self.baseline_left_gps_combo, self.baseline_right_gps_combo):
            combo.addItem("GPS1", "GPS1")
            combo.addItem("GPS2", "GPS2")
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
            self.gnss_monitor.left_source = str(self.baseline_left_gps_combo.currentData() or "GPS1")
            self.gnss_monitor.right_source = str(self.baseline_right_gps_combo.currentData() or "GPS1")
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
            estimate = RtkBaselineEstimate(
                baseline_m=vector.length_m,
                north_m=vector.north_m,
                east_m=vector.east_m,
                down_m=-vector.up_m,
                left_age_s=max(0.0, time.monotonic() - left.timestamp_s),
                right_age_s=max(0.0, time.monotonic() - right.timestamp_s),
                fix_quality="GOOD" if left.fix_type == 6 and right.fix_type == 6 else "DEGRADED",
            )
            self.latest_rtk_baseline = estimate
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
            gps = self.latest_gps.get(side)
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
                for key in ("fix", "rtk", "sats", "eph", "epv", "h_acc", "v_acc", "lla", "rate", "age", "timestamp"):
                    labels[key].setText("대기 중")
                continue
            age_ms = max(0.0, now_s - gps.timestamp_s) * 1000.0
            rate = self._baseline_message_rate(side)
            labels["fix"].setText("--" if gps.fix_type is None else str(gps.fix_type))
            self._set_health_label(labels["rtk"], gps.rtk_label, self._gps_fix_level(gps.fix_type, age_ms=age_ms))
            labels["sats"].setText("--" if gps.satellites_used is None else str(gps.satellites_used))
            labels["eph"].setText("--" if gps.eph_m is None else f"{gps.eph_m:.3f} m")
            labels["epv"].setText("--" if gps.epv_m is None else f"{gps.epv_m:.3f} m")
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



























































    def _clear_gnss_base_serial_worker(self, thread: QtCore.QThread) -> None:
        if self.gnss_base_serial_thread is thread:
            self.gnss_base_serial_thread = None
            self.gnss_base_serial_worker = None
            self.gnss_base_serial_connected = False
            owner = getattr(self, "_gnss_runtime_controller_owner", None)
            if owner is not None:
                owner._wire_rtcm_pipeline_base_worker(None)
                owner._sync_rtcm_pipeline_config()
            if hasattr(self, "gnss_base_connect_button"):
                self.gnss_base_connect_button.setEnabled(True)
            if hasattr(self, "gnss_base_disconnect_button"):
                self.gnss_base_disconnect_button.setEnabled(False)
            if hasattr(self, "gnss_base_survey_start_button"):
                self.gnss_base_survey_start_button.setEnabled(False)
            if hasattr(self, "gnss_base_survey_stop_button"):
                self.gnss_base_survey_stop_button.setEnabled(False)
        thread.deleteLater()

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
                "left": getattr(self.latest_gps.get("left"), "fix_type", None),
                "right": getattr(self.latest_gps.get("right"), "fix_type", None),
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
            self,
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
            self,
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
            QtWidgets.QMessageBox.warning(self, "Alignment Profile", str(exc))
            return
        self.loaded_alignment_profile = payload
        self.alignment_profile_preview.setPlainText(path.read_text(encoding="utf-8"))
        self.alignment_profile_status_label.setText(f"Alignment Profile 불러옴: {path}")
        self._refresh_alignment_validation()

    def _build_px4_mag_manager_panel(self, layout: QtWidgets.QVBoxLayout, *, collapsible: bool = True) -> None:
        group = QtWidgets.QGroupBox("PX4 Magnetometer 역할 / Priority 관리")
        group.setSizePolicy(QtWidgets.QSizePolicy.Ignored, QtWidgets.QSizePolicy.Preferred)
        group_layout = QtWidgets.QVBoxLayout(group)

        self.px4_mag_tables: dict[str, QtWidgets.QTableWidget] = {}
        self.px4_mag_status_labels: dict[str, QtWidgets.QLabel] = {}
        self.px4_mag_refresh_buttons: dict[str, QtWidgets.QPushButton] = {}
        self.px4_mag_apply_manual_buttons: dict[str, QtWidgets.QPushButton] = {}
        self.px4_mag_save_buttons: dict[str, QtWidgets.QPushButton] = {}
        self.px4_mag_reboot_buttons: dict[str, QtWidgets.QPushButton] = {}
        # Kept under the historical ``*_spins`` attribute name for backend
        # compatibility; the controls now expose PX4's semantic priority enum.
        self.px4_mag_priority_spins: dict[str, dict[str, QtWidgets.QComboBox]] = {}

        self.px4_mag_tab_widget = QtWidgets.QTabWidget()
        for camera, title in (("left", "Pixhawk 1"), ("right", "Pixhawk 2")):
            tab = QtWidgets.QWidget()
            tab_layout = QtWidgets.QVBoxLayout(tab)
            self._build_single_px4_mag_manager_tab(tab_layout, camera)
            self.px4_mag_tab_widget.addTab(tab, title)
        group_layout.addWidget(self.px4_mag_tab_widget)
        layout.addWidget(group)
        if collapsible:
            self._configure_collapsible_group(group, checked=False)
        self.refresh_px4_mag_manager()

    def _build_single_px4_mag_manager_tab(self, layout: QtWidgets.QVBoxLayout, camera: str) -> None:
        control_row = QtWidgets.QHBoxLayout()
        refresh_button = QtWidgets.QPushButton("새로고침")
        apply_manual_button = QtWidgets.QPushButton("수동 Priority 적용")
        save_button = QtWidgets.QPushButton("파라미터 저장")
        reboot_button = QtWidgets.QPushButton("PX4 재부팅")
        for button in (
            refresh_button,
            apply_manual_button,
            save_button,
            reboot_button,
        ):
            control_row.addWidget(button)
        control_row.addStretch(1)
        layout.addLayout(control_row)

        status_label = QtWidgets.QLabel("PX4 MAG 상태: 대기 중")
        status_label.setObjectName("statusStrip")
        status_label.setWordWrap(True)
        status_label.setTextInteractionFlags(QtCore.Qt.TextSelectableByMouse)
        layout.addWidget(status_label)

        manual_group = QtWidgets.QGroupBox("수동 Priority 입력")
        manual_layout = QtWidgets.QGridLayout(manual_group)
        manual_layout.setColumnStretch(1, 1)
        default_priorities = {
            "BOARD": 0,
            "CAN1": 100,
            "CAN2": 90,
            "GPS1_MAG": 0,
            "GPS2_MAG": 0,
        }
        self.px4_mag_priority_spins[camera] = {}
        for row, role in enumerate(("BOARD", "CAN1", "CAN2", "GPS1_MAG", "GPS2_MAG")):
            spin = _build_px4_priority_combo(default_priorities[role])
            spin.setToolTip(
                f"{role}가 등록된 CAL_MAG 슬롯의 PRIO 값을 설정합니다. "
                f"현재 선택: {mag_priority_display(default_priorities[role])}"
            )
            manual_layout.addWidget(QtWidgets.QLabel(role), row, 0)
            manual_layout.addWidget(spin, row, 1)
            self.px4_mag_priority_spins[camera][role] = spin
        manual_note = QtWidgets.QLabel(
            "PX4 값: Uninitialized=-1, Disabled=0, Min=1, Low=25, Medium=50, High=75, Max=100. "
            "기존의 비표준 값은 Custom으로 보존합니다. MAVLink heartbeat + PARAM readback 상태에서만 적용됩니다."
        )
        manual_note.setObjectName("statusStrip")
        manual_note.setWordWrap(True)
        manual_layout.addWidget(manual_note, len(default_priorities), 0, 1, 2)
        layout.addWidget(manual_group)

        self.px4_mag_refresh_buttons[camera] = refresh_button
        self.px4_mag_apply_manual_buttons[camera] = apply_manual_button
        self.px4_mag_save_buttons[camera] = save_button
        self.px4_mag_reboot_buttons[camera] = reboot_button
        self.px4_mag_status_labels[camera] = status_label

        refresh_button.clicked.connect(
            lambda _checked=False, side=camera: self.refresh_px4_mag_manager(side, read_params=True)
        )
        apply_manual_button.clicked.connect(lambda _checked=False, side=camera: self.apply_manual_mag_priorities_from_panel(side))
        save_button.clicked.connect(lambda _checked=False, side=camera: self.save_px4_mag_params_from_panel(side))
        reboot_button.clicked.connect(lambda _checked=False, side=camera: self.reboot_px4_from_panel(side))

    def _build_px4_attitude_cal_panel(self, layout: QtWidgets.QVBoxLayout, *, collapsible: bool = True) -> None:
        group = QtWidgets.QGroupBox("PX4 가속도계 / 자이로 / 보드 회전 보정 상태 (QGC Sensor Calibration)")
        group.setSizePolicy(QtWidgets.QSizePolicy.Ignored, QtWidgets.QSizePolicy.Preferred)
        group_layout = QtWidgets.QVBoxLayout(group)

        self.px4_attitude_cal_status_labels: dict[str, QtWidgets.QLabel] = {}
        self.px4_attitude_cal_refresh_buttons: dict[str, QtWidgets.QPushButton] = {}

        tab_widget = QtWidgets.QTabWidget()
        for camera, title in (("left", "Pixhawk 1"), ("right", "Pixhawk 2")):
            tab = QtWidgets.QWidget()
            tab_layout = QtWidgets.QVBoxLayout(tab)
            self._build_single_px4_attitude_cal_tab(tab_layout, camera)
            tab_widget.addTab(tab, title)
        self.px4_attitude_cal_tab_widget = tab_widget
        group_layout.addWidget(tab_widget)
        layout.addWidget(group)
        if collapsible:
            self._configure_collapsible_group(group, checked=False)
        self.refresh_px4_attitude_cal_status()

    def _build_single_px4_attitude_cal_tab(self, layout: QtWidgets.QVBoxLayout, camera: str) -> None:
        control_row = QtWidgets.QHBoxLayout()
        refresh_button = QtWidgets.QPushButton("새로고침")
        control_row.addWidget(refresh_button)
        control_row.addStretch(1)
        layout.addLayout(control_row)

        status_label = QtWidgets.QLabel("PX4 자세 보정 상태: 대기 중")
        status_label.setObjectName("statusStrip")
        status_label.setWordWrap(True)
        status_label.setTextInteractionFlags(QtCore.Qt.TextSelectableByMouse)
        layout.addWidget(status_label)

        note = QtWidgets.QLabel(
            "가속도계/자이로/보드 회전(레벨 호라이즌) 보정은 QGC Sensor Setup에서 수행합니다. "
            "이 패널은 PX4에 저장된 보정 결과를 읽기 전용으로 표시하며, 여기서 값을 수정하지는 않습니다."
        )
        note.setObjectName("statusStrip")
        note.setWordWrap(True)
        layout.addWidget(note)

        self.px4_attitude_cal_refresh_buttons[camera] = refresh_button
        self.px4_attitude_cal_status_labels[camera] = status_label

        refresh_button.clicked.connect(
            lambda _checked=False, side=camera: self.refresh_px4_attitude_cal_status(side, read_params=True)
        )

    def _build_px4_mag_detail_panel(self, layout: QtWidgets.QVBoxLayout) -> None:
        group = QtWidgets.QGroupBox("PX4 MAG 고급 상세: CAL_MAG 슬롯 / Priority")
        group_layout = QtWidgets.QVBoxLayout(group)
        note = QtWidgets.QLabel(
            "일반 MAG 관리에는 역할 상태와 Priority 수정만 표시합니다. "
            "CAL_MAG 슬롯 매핑과 PX4 PARAM readback은 이 고급 화면에서 확인합니다."
        )
        note.setObjectName("statusStrip")
        note.setWordWrap(True)
        group_layout.addWidget(note)

        self.px4_mag_tab_widget = QtWidgets.QTabWidget()
        for camera, title in (("left", "Pixhawk 1"), ("right", "Pixhawk 2")):
            tab = QtWidgets.QWidget()
            tab_layout = QtWidgets.QVBoxLayout(tab)
            table = QtWidgets.QTableWidget(0, 10)
            table.setHorizontalHeaderLabels(
                [
                    "Slot",
                    "Device ID",
                    "Device Type",
                    "Interface",
                    "RM3100",
                    "Priority",
                    "Enabled",
                    "Expected Selected",
                    "Actual Selected",
                    "Status",
                ]
            )
            table.setEditTriggers(QtWidgets.QAbstractItemView.NoEditTriggers)
            table.setSelectionBehavior(QtWidgets.QAbstractItemView.SelectRows)
            table.setAlternatingRowColors(True)
            table.setWordWrap(True)
            table.setHorizontalScrollBarPolicy(QtCore.Qt.ScrollBarAlwaysOff)
            table.verticalHeader().setVisible(False)
            table.verticalHeader().setSectionResizeMode(QtWidgets.QHeaderView.ResizeToContents)
            table.horizontalHeader().setSectionResizeMode(QtWidgets.QHeaderView.Stretch)
            tab_layout.addWidget(table)
            self.px4_mag_tables[camera] = table
            self.px4_mag_tab_widget.addTab(tab, title)
        group_layout.addWidget(self.px4_mag_tab_widget)
        layout.addWidget(group)
        self.refresh_px4_mag_manager()

    def _build_single_pixhawk_sensor_tab(self, layout: QtWidgets.QVBoxLayout, camera: str, title: str) -> None:
        layout.setSpacing(8)

        top_row = QtWidgets.QHBoxLayout()
        top_row.setSpacing(8)

        rm_group = QtWidgets.QGroupBox(f"{title} · RM3100 CAN")
        rm_form = QtWidgets.QGridLayout(rm_group)
        rm_form.setContentsMargins(8, 10, 8, 8)
        rm_form.setHorizontalSpacing(8)
        rm_form.setVerticalSpacing(6)
        rm_source_label = QtWidgets.QLabel("소스: MAVLink SCALED_IMU* / MAVLink sensor telemetry")
        rm_source_label.setObjectName("statusStrip")
        rm_form.addWidget(rm_source_label, 0, 0, 1, 2)
        rm_form.addWidget(QtWidgets.QLabel("센서"), 1, 0)
        rm_form.addWidget(QtWidgets.QLabel("최근 샘플"), 1, 1)
        for row, slot in enumerate(("CAN1", "CAN2"), start=2):
            label = QtWidgets.QLabel("대기 중")
            label.setObjectName("statusStrip")
            rm_form.addWidget(QtWidgets.QLabel(f"RM3100 · {slot}"), row, 0)
            rm_form.addWidget(label, row, 1)
            self.rm3100_slot_labels[camera][slot] = label
            self.mag_sensor_labels[camera][slot] = label
        rm3100_compare_label = QtWidgets.QLabel("CAN1/CAN2 비교: 대기 중")
        rm3100_compare_label.setObjectName("statusStrip")
        rm_form.addWidget(QtWidgets.QLabel("비교"), 4, 0)
        rm_form.addWidget(rm3100_compare_label, 4, 1)
        self.rm3100_compare_labels[camera] = rm3100_compare_label
        top_row.addWidget(rm_group, 1)

        gnss_group = QtWidgets.QGroupBox(f"{title} · F9P GNSS")
        gnss_form = QtWidgets.QGridLayout(gnss_group)
        gnss_form.setContentsMargins(8, 10, 8, 8)
        gnss_form.setHorizontalSpacing(8)
        gnss_form.setVerticalSpacing(6)
        gnss_source_label = QtWidgets.QLabel("소스: MAVLink GPS / GNSS telemetry")
        gnss_source_label.setObjectName("statusStrip")
        gnss_form.addWidget(gnss_source_label, 0, 0, 1, 2)
        gnss_form.addWidget(QtWidgets.QLabel("수신기"), 1, 0)
        gnss_form.addWidget(QtWidgets.QLabel("최근 샘플"), 1, 1)
        for row, slot in enumerate(("GPS1", "GPS2"), start=2):
            label = QtWidgets.QLabel("대기 중")
            label.setObjectName("statusStrip")
            label.setWordWrap(True)
            label.setTextInteractionFlags(QtCore.Qt.TextSelectableByMouse)
            gnss_form.addWidget(QtWidgets.QLabel(f"F9P · {slot}"), row, 0)
            gnss_form.addWidget(label, row, 1)
            self.f9p_slot_labels[camera][slot] = label
            self.gps_sensor_labels[camera][slot] = label
        f9p_compare_label = QtWidgets.QLabel("GPS1/GPS2 품질 비교: 대기 중")
        f9p_compare_label.setObjectName("statusStrip")
        f9p_compare_label.setWordWrap(True)
        f9p_compare_label.setTextInteractionFlags(QtCore.Qt.TextSelectableByMouse)
        gnss_form.addWidget(QtWidgets.QLabel("비교"), 4, 0)
        gnss_form.addWidget(f9p_compare_label, 4, 1)
        self.f9p_compare_labels[camera] = f9p_compare_label
        gnss_heading_label = QtWidgets.QLabel("GNSS Heading: /fmu/out/sensor_gnss_relative 대기 중")
        gnss_heading_label.setObjectName("statusStrip")
        gnss_form.addWidget(QtWidgets.QLabel("Moving Base"), 5, 0)
        gnss_form.addWidget(gnss_heading_label, 5, 1)
        self.gnss_heading_labels[camera] = gnss_heading_label
        layout.addLayout(top_row)

        rm3100_norm_plot = TelemetryHistoryPlot(f"{title} 자기장 크기 안정성 (CAN1/CAN2)", "G")
        rm3100_delta_plot = TelemetryHistoryPlot(f"{title} CAN1/CAN2 자기장 불일치", "G/deg")
        f9p_sats_plot = TelemetryHistoryPlot(f"{title} GPS 위성 수 (GPS1/GPS2)", "sats")
        f9p_quality_plot = TelemetryHistoryPlot(f"{title} GPS 수평 정확도 eph (GPS1/GPS2)", "m")
        for plot in (rm3100_norm_plot, rm3100_delta_plot, f9p_sats_plot, f9p_quality_plot):
            plot.setMinimumHeight(145)
        self.rm3100_norm_plots[camera] = rm3100_norm_plot
        self.rm3100_delta_plots[camera] = rm3100_delta_plot
        self.f9p_sats_plots[camera] = f9p_sats_plot
        self.f9p_quality_plots[camera] = f9p_quality_plot

        rm_graph_layout = QtWidgets.QHBoxLayout()
        rm_graph_layout.setSpacing(8)
        rm_graph_layout.addWidget(rm3100_norm_plot, 1)
        rm_graph_layout.addWidget(rm3100_delta_plot, 1)
        layout.addLayout(rm_graph_layout, 1)

        f9p_detail_group = QtWidgets.QGroupBox(f"상세: {title} · F9P GNSS")
        f9p_detail_layout = QtWidgets.QVBoxLayout(f9p_detail_group)
        f9p_detail_layout.addWidget(gnss_group)
        f9p_graph_layout = QtWidgets.QHBoxLayout()
        f9p_graph_layout.setContentsMargins(0, 0, 0, 0)
        f9p_graph_layout.setSpacing(8)
        f9p_graph_layout.addWidget(f9p_sats_plot, 1)
        f9p_graph_layout.addWidget(f9p_quality_plot, 1)
        f9p_detail_layout.addLayout(f9p_graph_layout)
        self._configure_collapsible_group(f9p_detail_group, checked=False)
        layout.addWidget(f9p_detail_group)

    def _build_clock_sync_tab(self, layout: QtWidgets.QVBoxLayout) -> None:
        """Build the Workstation camera-clock and RTCP/NTP evidence surface.

        Workstation timing evidence includes the WIRIS RTC command, editable
        RTCP/NTP sync limits, and live RTCP/NTP frame metadata.
        """

        layout.setSpacing(8)

        rtc_group = QtWidgets.QGroupBox("WIRIS RTC / 스트림 동기화")
        rtc_layout = QtWidgets.QGridLayout(rtc_group)
        rtc_layout.setHorizontalSpacing(8)
        rtc_layout.setVerticalSpacing(6)
        rtc_layout.setColumnStretch(1, 1)
        rtc_layout.setColumnStretch(3, 1)
        self.clock_wiris_time_target_combo = QtWidgets.QComboBox()
        self.clock_wiris_time_target_combo.addItem("양쪽 카메라", "both")
        self.clock_wiris_time_target_combo.addItem("왼쪽 카메라", "left")
        self.clock_wiris_time_target_combo.addItem("오른쪽 카메라", "right")
        self.wiris_set_pc_time_button = QtWidgets.QPushButton("PC UTC로 시간 설정")
        self.wiris_set_pc_time_button.clicked.connect(
            lambda _checked=False: self.set_selected_wiris_time_from_pc(
                str(self.clock_wiris_time_target_combo.currentData() or "both")
            )
        )
        self.clock_sync_check_button = QtWidgets.QPushButton("RTCP/NTP 동기화 확인")
        self.clock_sync_check_button.clicked.connect(self.refresh_clock_sync_status)
        self.clock_sync_status_label = QtWidgets.QLabel("RTCP/NTP frame timestamp 대기 중")
        self.clock_sync_status_label.setObjectName("statusStrip")
        self.clock_sync_status_label.setWordWrap(True)
        self.clock_sync_status_label.setTextInteractionFlags(QtCore.Qt.TextSelectableByMouse)
        self.clock_sync_summary_text = QtWidgets.QPlainTextEdit()
        self.clock_sync_summary_text.setReadOnly(True)
        self.clock_sync_summary_text.setMaximumBlockCount(80)
        self.clock_sync_summary_text.setFixedHeight(104)
        self.clock_sync_summary_text.setPlaceholderText(
            "YOLO/preview stream에서 RTCP Sender Report timestamp가 들어오면 좌/우 RTC offset이 표시됩니다."
        )
        rtc_layout.addWidget(QtWidgets.QLabel("시간 설정 대상"), 0, 0)
        rtc_layout.addWidget(self.clock_wiris_time_target_combo, 0, 1)
        rtc_layout.addWidget(self.wiris_set_pc_time_button, 0, 2)
        rtc_layout.addWidget(self.clock_sync_check_button, 0, 3)
        rtc_layout.addWidget(self.clock_sync_status_label, 1, 0, 1, 4)
        rtc_layout.addWidget(self.clock_sync_summary_text, 2, 0, 1, 4)

        self.clock_offset_plot = ClockOffsetHistoryPlot()
        history_group = QtWidgets.QGroupBox("RTCP/NTP 오프셋 추이")
        history_layout = QtWidgets.QVBoxLayout(history_group)
        history_layout.addWidget(self.clock_offset_plot)
        layout.addWidget(rtc_group)
        layout.addWidget(self.camera_sync_settings_group)
        layout.addWidget(history_group, 1)

    def _camera_time_sync_controller(self):
        owner = getattr(self, "_camera_time_sync_controller_owner", None)
        if owner is None:
            owner = CameraTimeSyncController(self)
            self._camera_time_sync_controller_owner = owner
        return owner

    def yolo_kwargs(self) -> dict:
        weights_path = resolve_workstation_data_path(
            self.weights_edit.text().strip() or DEFAULT_YOLO_WEIGHTS
        )
        return {
            "weights_path": str(weights_path),
            "device": self.device_edit.text().strip(),
            "input_size": self.input_size_spin.value(),
            "conf_threshold": self.conf_spin.value(),
            "iou_threshold": self.iou_spin.value(),
            "class_gate_enabled": self.yolo_gate_class_checkbox.isChecked(),
            "class_gate": self.yolo_gate_class_edit.text().strip(),
            "max_rate_hz": 0.0,
            "display_rate_hz": DEFAULT_PREVIEW_DISPLAY_RATE_HZ,
            "preview_max_width": self.preview_width_spin.value(),
            "rtsp_latency_ms": self.rtsp_latency_spin.value(),
            "event_queue_size": self.yolo_event_queue_size_spin.value(),
            "frame_queue_size": self.yolo_frame_queue_size_spin.value(),
            "decoder": self.decoder_combo.currentData(),
            "stream_backend": self.stream_backend_combo.currentData(),
            "rtsp_transport": (
                self._camera_sync_transport_override
                or self.rtsp_transport_combo.currentData()
            ),
        }

    def _video_recording_kwargs(self, camera: str) -> dict[str, object]:
        if not self.video_recording_enabled:
            return {}
        path = self.video_recording_paths.get(camera)
        if path is None:
            return {}
        return {
            "record_video_path": str(path),
            "record_video_fps": 30.0,
        }

    def _recording_session_dir(self, stamp: str | None = None) -> Path:
        selected = stamp or self.recording_session_id
        root = self._measurement_session_dir(selected)
        if (
            bool(getattr(self, "_measurement_session_hierarchy_enabled", False))
            and str(getattr(self, "_measurement_session_id", "") or "") == str(selected or "")
            and str(getattr(self, "_measurement_id", "") or "")
        ):
            return root / "measurements" / str(self._measurement_id)
        return root

    def _measurement_session_dir(self, session_id: str | None = None) -> Path:
        selected = session_id or self._measurement_session_id or self.recording_session_id
        return resolve_session_dir(
            self.app_root / "recordings",
            str(selected or ""),
        )

    def _measurement_sessions_root(self) -> Path:
        return self.app_root / "recordings"

    def _prepare_video_recording_paths(self) -> None:
        output_dir = self._recording_session_dir(self.recording_session_id)
        self.video_recording_paths = {
            "left": output_dir / "video" / "raw_left.mp4",
            "right": output_dir / "video" / "raw_right.mp4",
        }
        output_dir.mkdir(parents=True, exist_ok=True)

    def _format_video_recording_paths(self) -> str:
        return format_video_paths(self.video_recording_paths)

    def _session_recording_active(self) -> bool:
        return self.distance_recording_enabled or self.video_recording_enabled

    def browse_yolo_weights(self) -> None:
        current_path = resolve_workstation_data_path(
            self.weights_edit.text().strip() or DEFAULT_YOLO_WEIGHTS
        )
        start_dir = current_path.parent if current_path.parent.exists() else Path.home()
        selected, _filter = QtWidgets.QFileDialog.getOpenFileName(
            self,
            "Select YOLO weights",
            str(start_dir),
            "Model weights (*.pt *.onnx *.engine *.torchscript);;All files (*)",
        )
        if not selected:
            return
        self.weights_edit.setText(selected)
        self._log(f"[YOLO] weights 파일 선택: {selected}")

    def _format_age(self, age_s: float) -> str:
        if not isfinite(age_s):
            return "n/a"
        if age_s < 1.0:
            return f"{age_s * 1000.0:.0f}ms"
        return f"{age_s:.2f}s"

    def _rtsp_url_host_text(self, url: str) -> str:
        if not url:
            return "missing"
        try:
            host, port = rtsp_host_port(url)
        except Exception:
            return "invalid-url"
        return f"{host}:{port}"

    def _rtsp_url_host_only(self, url: str) -> str:
        try:
            host, _port = rtsp_host_port(url.strip())
        except Exception:
            return ""
        return host

    def detect_camera_control(self, side: str | None = None) -> None:
        self._start_camera_detection("control", target_side=side)

    def detect_camera_rtsp(self) -> None:
        self._start_camera_detection("rtsp")

    def auto_connect_wiris(self) -> bool | None:
        if self._session_recording_active():
            self._log("[카메라] 세션 기록 중에는 WIRIS 자동 연결을 시작할 수 없습니다.")
            return False
        self._camera_auto_connect_retry_generation += 1
        self._camera_auto_connect_attempt = 0
        self._auto_wiris_enabled_cameras.clear()
        self.auto_wiris_connect_pending = True
        self._start_camera_detection("auto_connect")
        return None

    def _start_startup_camera_sequence(self) -> None:
        """Enable configured WIRIS streams without a blocking network-wide scan."""

        if self._session_recording_active():
            self._log("[카메라] --start: 세션 기록 중이어서 startup 카메라 연결을 건너뜁니다.")
            return
        self._startup_camera_sequence_pending = True
        self.auto_wiris_connect_pending = True
        self._camera_auto_connect_attempt = 1
        self._auto_wiris_enabled_cameras.clear()
        self._log(
            "[카메라] --start: 설정된 Left/Right Control Host만 사용해 "
            "WIRIS 활성화와 RTSP 프레임 확인을 시작합니다. "
            "서비스가 준비되지 않으면 자동 탐색으로 fallback합니다."
        )
        self._start_rtsp_enable_worker(target_labels=("left", "right"), reactivate=False)

    def _camera_host_hints(self) -> tuple[str, ...]:
        hints: list[str] = []
        for text in (
            self.left_zoom_host_edit.text(),
            self.right_zoom_host_edit.text(),
            self.left_url_edit.text(),
            self.right_url_edit.text(),
            *DEFAULT_CAMERA_URLS,
        ):
            value = str(text).strip()
            if not value:
                continue
            if value.startswith("rtsp://"):
                try:
                    host, _port = rtsp_host_port(value)
                except Exception:
                    continue
                value = host
            try:
                IPv4Address(value)
            except ValueError:
                continue
            if value not in hints:
                hints.append(value)
        return tuple(hints)

    def _start_camera_detection(self, mode: str, *, target_side: str | None = None) -> None:
        if self.camera_auto_detect_thread is not None:
            return
        attempt_text = ""
        if mode == "auto_connect":
            self._camera_auto_connect_attempt += 1
            attempt_text = (
                f" (시도 {self._camera_auto_connect_attempt}/{CAMERA_AUTO_CONNECT_MAX_ATTEMPTS})"
            )
        full_control_scan = mode != "auto_connect" or self._camera_auto_connect_attempt <= 1
        self.workstation_camera_discovery_status = f"SEARCHING {mode.upper()}"
        self._log(
            f"[연결] 카메라 {mode} 탐색은 설정된 인터페이스를 사용합니다. "
            "필요하면 Discover Camera Network로 후보 NIC/IP를 먼저 확인하세요."
        )
        if mode == "auto_connect":
            self._log(
                "[카메라] WIRIS 자동 연결을 시작합니다"
                f"{attempt_text}: 설정/기본 제어 포트, HIWS 검증, "
                "RTSP 활성화, 프레임 확인을 순차 실행합니다."
            )
            self._log("[arp] scan: 카메라망 후보 탐색을 시작합니다.")
        elif mode == "control":
            self._log("[카메라] 카메라 네트워크에서 WIRIS 제어 포트를 전체 스캔하고 HIWS로 검증합니다.")
            self._log("[arp] scan: 카메라망 후보 탐색을 시작합니다.")
        else:
            self._log("[카메라] 카메라 네트워크와 RTSP 후보를 탐색합니다.")
        self.auto_wiris_connect_button.setEnabled(False)
        self.detect_control_button.setEnabled(False)
        self.right_detect_control_button.setEnabled(False)
        self.detect_rtsp_button.setEnabled(False)
        self.right_detect_rtsp_button.setEnabled(False)
        self.camera_auto_detect_thread = QtCore.QThread(self)
        self.camera_auto_detect_worker = CameraAutoDetectWorker(
            mode=mode,
            control_port=self.zoom_port_spin.value(),
            host_hints=self._camera_host_hints(),
            target_side=target_side,
            full_control_scan=full_control_scan,
        )
        self.camera_auto_detect_worker.moveToThread(self.camera_auto_detect_thread)
        self.camera_auto_detect_thread.started.connect(self.camera_auto_detect_worker.run)
        self.camera_auto_detect_worker.detected.connect(self._apply_camera_detection)
        self.camera_auto_detect_worker.failed.connect(self._camera_auto_detect_failed)
        self.camera_auto_detect_worker.finished.connect(self.camera_auto_detect_thread.quit)
        self.camera_auto_detect_worker.finished.connect(self.camera_auto_detect_worker.deleteLater)
        self.camera_auto_detect_thread.finished.connect(self.camera_auto_detect_thread.deleteLater)
        self.camera_auto_detect_thread.finished.connect(self._clear_camera_auto_detect_worker)
        self.camera_auto_detect_thread.start()

    def _cancel_auto_wiris_connect_retry(self) -> None:
        self._camera_auto_connect_retry_generation += 1
        self._camera_auto_connect_attempt = 0
        self.auto_wiris_connect_pending = False
        self._startup_camera_sequence_pending = False

    def _schedule_auto_wiris_connect_retry(self, reason: str) -> None:
        if not self.auto_wiris_connect_pending:
            return
        if self._startup_camera_sequence_pending:
            # Startup must remain responsive.  A failed configured-host
            # attempt is reported for manual recovery instead of escalating
            # into a repository-wide TCP/HIWS scan while the operator is
            # waiting for the window to become usable.
            self.auto_wiris_connect_pending = False
            self._startup_camera_sequence_pending = False
            self.workstation_camera_discovery_status = "STARTUP CONFIGURED HOSTS UNAVAILABLE"
            self.camera_connection_label.setText(
                "WIRIS startup: configured camera hosts unavailable; use Connect cameras"
            )
            self._log(
                f"[카메라] startup configured-host 연결 실패: {reason}. "
                "전체 자동 탐색은 생략했습니다. Connect cameras로 수동 탐색하세요."
            )
            self._refresh_operation_header()
            return
        attempt = max(0, int(self._camera_auto_connect_attempt))
        if attempt >= CAMERA_AUTO_CONNECT_MAX_ATTEMPTS:
            self.auto_wiris_connect_pending = False
            self._startup_camera_sequence_pending = False
            self._log(
                "[카메라] WIRIS 자동 연결을 중단했습니다: "
                f"{CAMERA_AUTO_CONNECT_MAX_ATTEMPTS}회 시도 후에도 카메라 서비스가 준비되지 않았습니다. "
                "전원/Ethernet Mode/IP를 확인하세요."
            )
            self.camera_connection_label.setText("WIRIS auto connect: camera service unavailable")
            self._refresh_operation_header()
            return

        self._camera_auto_connect_retry_generation += 1
        generation = self._camera_auto_connect_retry_generation
        next_attempt = attempt + 1
        delay_s = CAMERA_AUTO_CONNECT_RETRY_DELAY_MS / 1000.0
        self.workstation_camera_discovery_status = (
            f"RETRYING AUTO_CONNECT {next_attempt}/{CAMERA_AUTO_CONNECT_MAX_ATTEMPTS}"
        )
        self.camera_connection_label.setText(
            f"WIRIS auto connect: waiting for camera service "
            f"({next_attempt}/{CAMERA_AUTO_CONNECT_MAX_ATTEMPTS})"
        )
        self._log(
            f"[카메라] WIRIS 자동 연결 재시도 예약: {reason}; "
            f"{delay_s:.1f}초 후 {next_attempt}/{CAMERA_AUTO_CONNECT_MAX_ATTEMPTS}회 시도"
        )

        def retry() -> None:
            if generation != self._camera_auto_connect_retry_generation:
                return
            if not self.auto_wiris_connect_pending:
                return
            if (
                self.camera_auto_detect_thread is not None
                or self.rtsp_enable_thread is not None
                or self.rtsp_check_thread is not None
            ):
                QtCore.QTimer.singleShot(100, retry)
                return
            self._start_camera_detection("auto_connect")

        QtCore.QTimer.singleShot(CAMERA_AUTO_CONNECT_RETRY_DELAY_MS, retry)

    def _apply_camera_detection(self, result: object) -> None:
        if isinstance(result, dict):
            mode = str(result.get("mode") or "rtsp")
            target_side = result.get("target_side")
            host_interfaces = list(result.get("host_interfaces") or [])
            rtsp_candidates = list(result.get("rtsp") or [])
            control_candidates = list(result.get("control") or [])
            verified_control_candidates = list(result.get("verified_control") or [])
            arp_scan_reports = list(result.get("arp_scan") or [])
            full_control_scan = bool(result.get("full_control_scan", False))
        else:
            mode = "rtsp"
            target_side = None
            host_interfaces = []
            rtsp_candidates = list(result or [])
            control_candidates = []
            verified_control_candidates = []
            arp_scan_reports = []
            full_control_scan = False
        self._log_arp_scan_reports(arp_scan_reports)
        self._set_camera_candidate_rows(host_interfaces, rtsp_candidates, control_candidates, verified_control_candidates)
        if mode == "auto_connect" and self._session_recording_active():
            self._cancel_auto_wiris_connect_retry()
            self.camera_connection_label.setText("WIRIS auto connect: recording is active")
            self._log("[카메라] 탐색 중 세션 기록이 시작되어 자동 후보 적용과 프리뷰 연결을 건너뛰었습니다.")
            return
        if not (host_interfaces or rtsp_candidates or control_candidates or verified_control_candidates):
            self.workstation_camera_discovery_status = "FOUND 0 ENDPOINTS"
        if host_interfaces:
            self._apply_host_interface_candidates(host_interfaces)
        if mode == "auto_connect":
            self._apply_auto_wiris_connect_candidates(
                verified_control_candidates,
                control_candidates,
                full_control_scan=full_control_scan,
            )
        elif mode == "control":
            self._apply_control_candidates(verified_control_candidates, control_candidates, target_side=target_side)
        elif not rtsp_candidates:
            self._log("[연결] RTSP 후보를 찾지 못했습니다. 카메라 전원, IP 대역, RTSP 포트를 확인하세요.")
            self.camera_connection_label.setText("camera RTSP: auto detect found no candidates")
        else:
            self._apply_rtsp_candidates(rtsp_candidates)

    def _set_camera_candidate_rows(
        self,
        host_interfaces: list[object],
        rtsp_candidates: list[object],
        control_candidates: list[object],
        verified_control_candidates: list[object],
    ) -> None:
        rows: list[dict[str, object]] = []
        for candidate in host_interfaces:
            interface = getattr(candidate, "interface", None)
            if interface is None:
                continue
            neighbor_hosts = tuple(getattr(candidate, "neighbor_hosts", ()) or ())
            status = f"neighbors={', '.join(neighbor_hosts)}" if neighbor_hosts else "candidate"
            rows.append(
                {
                    "kind": "NIC",
                    "address": getattr(interface, "name", ""),
                    "port": "",
                    "status": status,
                    "interface": interface,
                    "host_ip": str(getattr(interface, "address", "")),
                    "candidate": candidate,
                }
            )
        for candidate in rtsp_candidates:
            interface = getattr(candidate, "interface", None)
            rows.append(
                {
                    "kind": "RTSP",
                    "address": str(getattr(candidate, "url", "")),
                    "host": str(getattr(candidate, "host", "")),
                    "port": str(getattr(candidate, "port", "")),
                    "status": "port open",
                    "interface": interface,
                    "host_ip": str(getattr(interface, "address", "")) if interface is not None else "",
                    "candidate": candidate,
                }
            )
        verified_keys: set[tuple[str, int]] = set()
        for candidate in verified_control_candidates:
            host = str(_control_candidate_value(candidate, "host", "") or "")
            try:
                port = int(_control_candidate_value(candidate, "port", 0) or 0)
            except (TypeError, ValueError):
                port = 0
            verified_keys.add((host, port))
            interface = _control_candidate_value(candidate, "interface")
            response = str(_control_candidate_value(candidate, "response", "") or "")
            ok = bool(_control_candidate_value(candidate, "ok"))
            rows.append(
                {
                    "kind": "WIRIS",
                    "address": host,
                    "host": host,
                    "port": str(port) if port else "",
                    "status": f"HIWS {'OK' if ok else 'FAIL'} {response}".strip(),
                    "interface": interface,
                    "host_ip": str(getattr(interface, "address", "")) if interface is not None else "",
                    "candidate": candidate,
                }
            )
        for candidate in control_candidates:
            host = str(_control_candidate_value(candidate, "host", "") or "")
            try:
                port = int(_control_candidate_value(candidate, "port", 0) or 0)
            except (TypeError, ValueError):
                port = 0
            if (host, port) in verified_keys:
                continue
            interface = _control_candidate_value(candidate, "interface")
            rows.append(
                {
                    "kind": "WIRIS",
                    "address": host,
                    "host": host,
                    "port": str(port) if port else "",
                    "status": "TCP open",
                    "interface": interface,
                    "host_ip": str(getattr(interface, "address", "")) if interface is not None else "",
                    "candidate": candidate,
                }
            )
        self._set_camera_candidate_table_rows(rows)

    def _set_camera_candidate_table_rows(self, rows: list[dict[str, object]]) -> None:
        self._camera_candidate_rows = rows
        if rows:
            self.workstation_camera_discovery_status = f"FOUND {len(rows)} ENDPOINTS"
        self.camera_candidate_table.setRowCount(len(rows))
        for row, entry in enumerate(rows):
            for column, key in enumerate(("kind", "address", "port", "status", "interface", "host_ip")):
                value = entry.get(key, "")
                if key == "interface" and value:
                    value = getattr(value, "name", "")
                item = QtWidgets.QTableWidgetItem(str(value))
                self.camera_candidate_table.setItem(row, column, item)
        self.camera_candidate_table.resizeColumnsToContents()
        self._refresh_camera_candidate_actions()

    def clear_camera_candidate_history(self) -> None:
        if not self._camera_candidate_rows:
            return
        self._set_camera_candidate_table_rows([])
        self._log("[카메라] WIRIS 탐색 후보 내역을 지웠습니다.")

    def _refresh_camera_candidate_actions(self) -> None:
        row = self.camera_candidate_table.currentRow()
        kind = ""
        has_selection = 0 <= row < len(self._camera_candidate_rows)
        if has_selection:
            kind = str(self._camera_candidate_rows[row].get("kind", ""))
        self.clear_camera_candidate_button.setEnabled(bool(self._camera_candidate_rows))
        self.clear_camera_candidate_button.setToolTip("WIRIS 탐색 후보 내역을 지웁니다.")
        self.apply_candidate_nic_button.setEnabled(kind == "NIC")
        self.apply_candidate_nic_button.setToolTip(
            "선택한 NIC 후보를 카메라망 인터페이스와 PC IP에 적용합니다."
            if kind == "NIC"
            else "NIC 후보를 선택하면 적용할 수 있습니다."
        )
        side_enabled = kind in {"RTSP", "WIRIS"}
        for button in (self.apply_candidate_left_button, self.apply_candidate_right_button):
            button.setEnabled(side_enabled)
            if kind == "RTSP":
                button.setToolTip("선택한 RTSP 후보를 WIRIS 영상 주소에 적용합니다.")
            elif kind == "WIRIS":
                button.setToolTip("선택한 WIRIS 제어 후보를 Control Host와 RTSP host에 적용합니다.")
            else:
                button.setToolTip("RTSP 또는 WIRIS 후보를 선택하면 Left/Right에 적용할 수 있습니다.")

    def _selected_camera_candidate(self) -> dict[str, object] | None:
        row = self.camera_candidate_table.currentRow()
        if row < 0 or row >= len(self._camera_candidate_rows):
            self._log("[카메라] 탐색 후보를 먼저 선택하세요.")
            return None
        return self._camera_candidate_rows[row]

    def apply_selected_camera_candidate_nic(self) -> None:
        entry = self._selected_camera_candidate()
        if entry is None:
            return
        interface = entry.get("interface")
        if interface is None:
            self._log("[연결] 선택 후보에 적용 가능한 NIC 정보가 없습니다.")
            return
        self.interface_edit.setText(str(getattr(interface, "name", "")))
        self.host_address_edit.setText(str(getattr(interface, "address", "")))
        self._log(f"[연결] 선택 후보를 NIC로 적용했습니다: {getattr(interface, 'name', '')} {getattr(interface, 'address', '')}")
        self._refresh_operation_header()

    def apply_selected_camera_candidate(self, side: str) -> None:
        entry = self._selected_camera_candidate()
        if entry is None:
            return
        kind = str(entry.get("kind", ""))
        if kind == "NIC":
            self.apply_selected_camera_candidate_nic()
            return
        host = str(entry.get("host") or entry.get("address") or "").strip()
        if not host:
            self._log("[카메라] 선택 후보에 적용 가능한 host가 없습니다.")
            return
        if kind == "RTSP":
            url = str(entry.get("address") or "")
            url_edit, ip_edit = self._rtsp_widgets_for_camera(side)
            url_edit.setText(url)
            ip_edit.setText(host)
        else:
            if entry.get("candidate") is not None:
                self._apply_control_candidate_to_side(side, entry["candidate"])
            elif side == "left":
                self.left_zoom_host_edit.setText(host)
                self.left_stip_ip_edit.setText(host)
            else:
                self.right_zoom_host_edit.setText(host)
                self.right_stip_ip_edit.setText(host)
            self._sync_rtsp_url_from_control_host(side, host)
            try:
                port = int(str(entry.get("port") or "0"))
            except ValueError:
                port = 0
            if 1 <= port <= 65535:
                self._set_control_port_for_camera(side, port)
        interface = entry.get("interface")
        if interface is not None:
            self.interface_edit.setText(str(getattr(interface, "name", "")))
            self.host_address_edit.setText(str(getattr(interface, "address", "")))
        self._log(f"[카메라] 선택 후보를 {self._camera_name_ko(side)}에 적용했습니다: {kind} {host}")
        self._refresh_operation_header()

    def apply_camera_candidate(self, row: int, target: str) -> bool:
        """Apply a discovered camera endpoint selected by the Workstation surface."""

        if self._session_recording_active():
            self._log("[카메라] 세션 기록 중에는 카메라 네트워크/Left/Right 후보를 적용할 수 없습니다.")
            return False
        if isinstance(row, bool) or not isinstance(row, int) or not 0 <= row < len(self._camera_candidate_rows):
            self._log("[카메라] 적용할 탐색 후보가 올바르지 않습니다.")
            return False
        if target not in {"nic", "left", "right"}:
            self._log(f"[카메라] 지원하지 않는 후보 적용 대상입니다: {target}")
            return False
        self.camera_candidate_table.selectRow(row)
        if target == "nic":
            self.apply_selected_camera_candidate_nic()
        else:
            self.apply_selected_camera_candidate(target)
        return True

    def _log_arp_scan_reports(self, reports: list[object]) -> None:
        if not reports:
            self._log("[연결] arp-scan 결과 없음: 설치되지 않았거나 스캔 가능한 로컬 IPv4 /24 대역이 없습니다.")
            return
        for report in reports:
            if not getattr(report, "available", False):
                error = getattr(report, "error", "") or "unavailable"
                self._log(f"[연결] arp-scan 사용 불가: {error}")
                continue
            interface = getattr(report, "interface", None)
            interface_name = getattr(interface, "name", "unknown")
            network = getattr(report, "network", "")
            hosts = tuple(getattr(report, "hosts", ()) or ())
            returncode = getattr(report, "returncode", None)
            error = getattr(report, "error", "")
            if returncode not in (0, None):
                detail = f": {error}" if error else ""
                self._log(f"[연결] arp-scan 실패: {interface_name} {network} rc={returncode}{detail}")
                continue
            host_text = ", ".join(hosts) if hosts else "none"
            self._log(f"[연결] arp-scan: {interface_name} {network}, hosts={host_text}")

    def _apply_host_interface_candidates(self, candidates: list[object]) -> None:
        selected = candidates[0]
        interface = selected.interface
        self.interface_edit.setText(interface.name)
        self.host_address_edit.setText(str(interface.address))
        neighbor_hosts = tuple(getattr(selected, "neighbor_hosts", ()) or ())
        neighbor_text = f", neighbors={', '.join(neighbor_hosts)}" if neighbor_hosts else ""
        self._log(f"[연결] Host IP 자동 선택: {interface.name} {interface.address}{neighbor_text}")
        for index, candidate in enumerate(candidates[1:], start=2):
            candidate_interface = candidate.interface
            candidate_neighbors = tuple(getattr(candidate, "neighbor_hosts", ()) or ())
            candidate_neighbor_text = (
                f", neighbors={', '.join(candidate_neighbors)}" if candidate_neighbors else ""
            )
            self._log(
                f"[연결] Host IP 후보 {index}: "
                f"{candidate_interface.name} {candidate_interface.address}{candidate_neighbor_text}"
            )

    def _rtsp_url_host(self, url_edit: QtWidgets.QLineEdit) -> str | None:
        try:
            host, _port = rtsp_host_port(url_edit.text().strip())
        except Exception:
            return None
        return host

    def _normalized_rtsp_identity(self, url: str) -> tuple[str, int, str] | None:
        try:
            host, port = rtsp_host_port(url.strip())
        except Exception:
            return None
        path = ""
        try:
            path = urlparse(url.strip()).path or ""
        except Exception:
            path = ""
        return (host, int(port), path)

    def _rtsp_urls_are_same_stream(self) -> bool:
        left_identity = self._normalized_rtsp_identity(self.left_url_edit.text())
        right_identity = self._normalized_rtsp_identity(self.right_url_edit.text())
        return left_identity is not None and left_identity == right_identity

    def _preferred_rtsp_side_for_candidate(self, candidate: object) -> str | None:
        candidate_host = str(_control_candidate_value(candidate, "host", "") or "").strip()
        if not candidate_host:
            return None
        scores: list[tuple[int, str]] = []
        for side, control_edit, url_edit in (
            ("left", self.left_zoom_host_edit, self.left_url_edit),
            ("right", self.right_zoom_host_edit, self.right_url_edit),
        ):
            score = 0
            if control_edit.text().strip() == candidate_host:
                score += 2
            if self._rtsp_url_host(url_edit) == candidate_host:
                score += 1
            if score:
                scores.append((score, side))
        if not scores:
            return None
        scores.sort(key=lambda item: (item[0], 1 if item[1] == "right" else 0), reverse=True)
        if len(scores) > 1 and scores[0][0] == scores[1][0]:
            return None
        return scores[0][1]

    def _apply_rtsp_candidates(self, candidates: list[object]) -> None:
        selected = candidates[:2]
        self._log(f"[카메라] RTSP 후보 {len(candidates)}개 발견")
        for index, candidate in enumerate(selected, start=1):
            interface_text = ""
            if getattr(candidate, "interface", None) is not None:
                interface = candidate.interface
                interface_text = f", interface={interface.name}, host={interface.address}"
            self._log(
                f"[카메라] RTSP 후보 {index}: {candidate.url} "
                f"(host={candidate.host}, port={candidate.port}{interface_text})"
            )
        host_interface = getattr(selected[0], "interface", None)
        if host_interface is None:
            host_interface = select_host_for_camera(selected[0].host)
        if host_interface is not None:
            self.interface_edit.setText(host_interface.name)
            self.host_address_edit.setText(str(host_interface.address))
            self._log(f"[연결] Host IP 자동 선택: {host_interface.name} {host_interface.address}")
        if len(selected) == 1:
            candidate = selected[0]
            preferred_side = self._preferred_rtsp_side_for_candidate(candidate)
            if preferred_side == "left":
                self.left_url_edit.setText(candidate.url)
                self.left_rtsp_ip_edit.setText(candidate.host)
                self.left_zoom_host_edit.setText(candidate.host)
                self._log(
                    "[카메라] RTSP 후보가 1개만 발견되어 Left에만 자동 입력했습니다. "
                    "Right는 기존 URL을 유지합니다."
                )
            elif preferred_side == "right":
                self.right_url_edit.setText(candidate.url)
                self.right_rtsp_ip_edit.setText(candidate.host)
                self.right_zoom_host_edit.setText(candidate.host)
                self._log(
                    "[카메라] RTSP 후보가 1개만 발견되어 Right에만 자동 입력했습니다. "
                    "Left는 기존 URL을 유지합니다."
                )
            else:
                self._log(
                    "[카메라] RTSP 후보가 1개만 발견되었지만 좌/우 매칭이 불확실해서 "
                    "기존 URL을 유지했습니다."
                )
        else:
            self.left_url_edit.setText(selected[0].url)
            self.left_rtsp_ip_edit.setText(selected[0].host)
            self.left_zoom_host_edit.setText(selected[0].host)
            self.right_url_edit.setText(selected[1].url)
            self.right_rtsp_ip_edit.setText(selected[1].host)
            self.right_zoom_host_edit.setText(selected[1].host)
            self._log("[카메라] RTSP 후보 2개를 Left/Right에 자동 입력했습니다.")
        self.check_cameras()

    def _log_verified_control_candidates(self, verified: list[object]) -> None:
        for candidate in verified:
            host = str(_control_candidate_value(candidate, "host", "") or "")
            response = str(_control_candidate_value(candidate, "response", "") or "")
            ok_text = "OK" if _control_candidate_value(candidate, "ok") else "FAIL"
            self._log(
                f"[카메라] WIRIS SDK 후보 검증: "
                f"{host}:{_control_candidate_value(candidate, 'port')} HIWS={ok_text} 응답={response or '<empty>'}"
            )

    def _ok_control_candidates(self, verified: list[object]) -> list[object]:
        return [candidate for candidate in verified if bool(_control_candidate_value(candidate, "ok"))]

    def _candidate_control_port(self, candidate: object) -> int | None:
        try:
            port = int(_control_candidate_value(candidate, "port", 0) or 0)
        except (TypeError, ValueError):
            return None
        return port if 1 <= port <= 65535 else None

    def _apply_discovered_control_port(self, side: str, candidate: object) -> None:
        port = self._candidate_control_port(candidate)
        if port is None:
            return
        self._set_control_port_for_camera(side, port)
        self._log(f"[카메라] {self._camera_name_ko(side)} WIRIS 제어 포트를 발견된 값으로 설정했습니다: {port}")

    def _apply_control_candidate_to_side(self, side: str, candidate: object) -> None:
        host = str(_control_candidate_value(candidate, "host", "") or "")
        if side == "left":
            self.left_zoom_host_edit.setText(host)
            self.left_stip_ip_edit.setText(host)
        else:
            self.right_zoom_host_edit.setText(host)
            self.right_stip_ip_edit.setText(host)
        self._apply_discovered_control_port(side, candidate)
        interface = _control_candidate_value(candidate, "interface")
        if interface is not None:
            self.interface_edit.setText(interface.name)
            self.host_address_edit.setText(str(interface.address))
            self._log(f"[연결] Host IP 자동 선택: {interface.name} {interface.address}")

    def _apply_discovered_control_port_hint(self, candidates: list[object]) -> None:
        ports: list[int] = []
        for candidate in candidates:
            port = self._candidate_control_port(candidate)
            if port is None:
                continue
            if port not in ports:
                ports.append(port)
        if not ports:
            return
        selected_port = ports[0]
        if self.zoom_port_spin.value() != selected_port:
            self.zoom_port_spin.setValue(selected_port)
            self._log(f"[카메라] WIRIS 스캔 기본 포트를 발견된 값으로 설정했습니다: {selected_port}")
        if len(ports) > 1:
            self._log("[카메라] 좌/우 WIRIS 제어 포트가 서로 달라 각 카메라별 포트에 따로 적용합니다.")

    def _apply_control_candidates(
        self,
        verified: list[object],
        fallback: list[object] | None = None,
        *,
        target_side: object | None = None,
    ) -> None:
        fallback = list(fallback or [])
        if verified:
            self._log_verified_control_candidates(verified)
        candidates = self._ok_control_candidates(verified) if verified else list(fallback)
        if not candidates:
            if fallback:
                self._log("[카메라] 열린 TCP 포트 후보는 있으나 HIWS=OK 응답을 받은 WIRIS를 찾지 못했습니다.")
            else:
                self._log("[카메라] WIRIS 제어 포트 후보를 찾지 못했습니다. IP 대역, 전원, 제어 포트를 확인하세요.")
            self.camera_connection_label.setText("camera control: detect found no candidates")
            return
        selected = candidates[:2]
        self._apply_discovered_control_port_hint(selected)
        self._log(f"[카메라] WIRIS 제어 후보 {len(candidates)}개 발견")
        for index, candidate in enumerate(selected, start=1):
            interface_text = ""
            interface = _control_candidate_value(candidate, "interface")
            if interface is not None:
                interface_text = f", interface={interface.name}, host={interface.address}"
            self._log(
                f"[카메라] WIRIS 제어 후보 {index}: "
                f"{_control_candidate_value(candidate, 'host')}:{_control_candidate_value(candidate, 'port')}{interface_text}"
            )
        if target_side in {"left", "right"}:
            self._apply_control_candidate_to_side(str(target_side), selected[0])
            self._log(f"[카메라] WIRIS 제어 후보를 {self._camera_name_ko(str(target_side))}에만 입력했습니다.")
        elif len(selected) == 1:
            self._apply_control_candidate_to_side("left", selected[0])
            self._log("[카메라] WIRIS 제어 후보가 1개만 발견되어 Left Control Host에만 자동 입력했습니다.")
        else:
            self._apply_control_candidate_to_side("left", selected[0])
            self._apply_control_candidate_to_side("right", selected[1])
            self._log("[카메라] WIRIS 제어 후보 2개를 Left/Right Control Host에 자동 입력했습니다.")
        self.check_wiris_control()
        self._refresh_operation_header()

    def _apply_auto_wiris_connect_candidates(
        self,
        verified: list[object],
        fallback: list[object],
        *,
        full_control_scan: bool = False,
    ) -> None:
        ok_candidates = self._ok_control_candidates(verified)
        self._log_verified_control_candidates(verified)
        if not ok_candidates:
            if fallback:
                reason = "열린 포트에서 HIWS=OK 응답 없음"
                self._log("[카메라] 열린 TCP 포트 후보는 있으나 HIWS=OK 응답을 받은 WIRIS를 찾지 못했습니다.")
            else:
                reason = "WIRIS 제어 포트가 아직 열리지 않음"
                self._log("[카메라] WIRIS 제어 포트 후보를 찾지 못했습니다. PC IP 대역 또는 카메라 IP를 확인하세요.")
            if verified:
                open_ports = ", ".join(
                    f"{_control_candidate_value(candidate, 'host')}:{_control_candidate_value(candidate, 'port')}"
                    for candidate in verified
                )
                self._log(
                    "[카메라] 열린 포트의 HIWS 검증 결과: "
                    f"{open_ports}; WIRIS SDK 응답 없음"
                )
            elif full_control_scan:
                self._log(
                    "[카메라] 설정/기본 포트와 전체 TCP 1-65535 스캔 모두에서 "
                    "WIRIS SDK 포트를 찾지 못했습니다."
                )
            self._schedule_auto_wiris_connect_retry(reason)
            self._refresh_operation_header()
            return

        self._camera_auto_connect_retry_generation += 1
        selected = ok_candidates[:2]
        self._apply_discovered_control_port_hint(selected)
        self._apply_control_candidate_to_side("left", selected[0])
        left_host = str(_control_candidate_value(selected[0], "host", "") or "")
        self.left_zoom_host_edit.setText(left_host)
        self.left_stip_ip_edit.setText(left_host)
        self._sync_rtsp_url_from_control_host("left", left_host)
        if len(selected) > 1:
            self._apply_control_candidate_to_side("right", selected[1])
            right_host = str(_control_candidate_value(selected[1], "host", "") or "")
            self.right_zoom_host_edit.setText(right_host)
            self.right_stip_ip_edit.setText(right_host)
            self._sync_rtsp_url_from_control_host("right", right_host)
        if len(selected) == 1:
            self._log("[카메라] HIWS=OK WIRIS가 1개만 발견되어 Left Control Host에 입력했습니다.")
        else:
            self._log("[카메라] HIWS=OK WIRIS 2개를 Left/Right Control Host에 입력했습니다.")
        selected_sides = ("left",) if len(selected) == 1 else ("left", "right")
        activation_targets: list[str] = []
        for side, candidate in zip(selected_sides, selected):
            host = str(_control_candidate_value(candidate, "host", "") or "").strip()
            camera_key = (side, host)
            if camera_key in self._auto_wiris_enabled_cameras:
                self._log(
                    f"[카메라] {self._camera_name_ko(side)}는 이미 SETH TRUE가 승인되어 "
                    "RTSP 프레임만 다시 확인합니다."
                )
                continue
            activation_targets.append(side)

        if activation_targets:
            self.camera_connection_label.setText("WIRIS auto connect: control verified, enabling RTSP")
            self.enable_wiris_rtsp(target_labels=tuple(activation_targets))
        else:
            self.camera_connection_label.setText("WIRIS auto connect: control verified, checking RTSP")
            self.check_cameras()

    def _pixhawk_candidate_table_for_source(self, source: str) -> QtWidgets.QTableWidget:
        if source == "pc":
            return self.pc_pixhawk_candidate_table
        return self.mavlink_pixhawk_candidate_table

    def _pixhawk_candidate_source_for_kind(self, kind: str) -> str:
        if kind == "PC_IP":
            return "pc"
        return "mavlink"

    def _sync_legacy_pixhawk_candidate_rows(self) -> None:
        self._pixhawk_candidate_rows = [
            *self._pixhawk_candidate_rows_by_source.get("pc", []),
            *self._pixhawk_candidate_rows_by_source.get("mavlink", []),
        ]

    def _set_pixhawk_candidate_rows(self, rows: list[dict[str, object]], *, source: str | None = None) -> None:
        if source is None:
            grouped: dict[str, list[dict[str, object]]] = {"pc": [], "mavlink": []}
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
                button.setToolTip("선택한 USB 후보를 P1/P2 USB 부트스트랩 포트에 적용합니다. Ethernet MAVLink URL은 유지됩니다.")
            elif mavlink_kind == "MAVLink":
                button.setToolTip("선택한 MAVLink URL 후보를 P1/P2 MAVLink URL에 적용합니다. 연결은 적용 후 직접 누르세요.")
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

    def discover_mavlink_url_candidates(self) -> None:
        rows = self._mavlink_url_candidate_rows()
        self._append_pixhawk_candidate_rows(rows, source="mavlink")
        self.workstation_pixhawk_discovery_status = f"FOUND {len(rows)} NETWORK ENDPOINTS"
        for row in rows:
            self._log(
                f"[픽스호크] MAVLink URL 후보: {row['value']} "
                f"status={row['status']} host={row['interface']}"
            )

    def discover_usb_pixhawk_connections(self) -> None:
        if self.pixhawk_usb_discovery_thread is not None:
            self._log("[픽스호크] USB Pixhawk 연결 탐색이 이미 진행 중입니다.")
            return
        self.pixhawk_usb_discover_button.setEnabled(False)
        self.workstation_pixhawk_discovery_status = "SEARCHING USB"
        self._log("[픽스호크] USB Pixhawk 연결 탐색을 시작합니다. 후보는 자동 할당하지 않습니다.")
        self.pixhawk_usb_discovery_thread = QtCore.QThread(self)
        self.pixhawk_usb_discovery_worker = PixhawkUsbDiscoveryWorker(timeout_s=0.8)
        self.pixhawk_usb_discovery_worker.moveToThread(self.pixhawk_usb_discovery_thread)
        self.pixhawk_usb_discovery_thread.started.connect(self.pixhawk_usb_discovery_worker.run)
        self.pixhawk_usb_discovery_worker.discovered.connect(self._apply_pixhawk_usb_discovery)
        self.pixhawk_usb_discovery_worker.failed.connect(self._pixhawk_usb_discovery_failed)
        self.pixhawk_usb_discovery_worker.finished.connect(self.pixhawk_usb_discovery_thread.quit)
        self.pixhawk_usb_discovery_worker.finished.connect(self.pixhawk_usb_discovery_worker.deleteLater)
        self.pixhawk_usb_discovery_thread.finished.connect(self.pixhawk_usb_discovery_thread.deleteLater)
        self.pixhawk_usb_discovery_thread.finished.connect(self._clear_pixhawk_usb_discovery)
        self.pixhawk_usb_discovery_thread.start()

    def _apply_pixhawk_usb_discovery(self, result: object) -> None:
        data = dict(result or {}) if isinstance(result, dict) else {}
        candidates = list(data.get("candidates") or [])
        found = list(data.get("found") or [])
        rows: list[dict[str, object]] = []
        found_by_device = {device: baud for device, baud in found}
        for device in candidates:
            baud = found_by_device.get(device, "")
            if baud:
                status = "MAVLink heartbeat"
            else:
                status = self._usb_pixhawk_probe_status(device)
            rows.append({"kind": "USB", "value": device, "baud": baud, "status": status})
            if baud:
                rows.append(
                    {
                        "kind": "MAVLink",
                        "value": device,
                        "baud": baud,
                        "status": "USB serial direct",
                    }
                )
        if not rows:
            self.workstation_pixhawk_discovery_status = "FOUND 0 USB ENDPOINTS"
            self._log("[픽스호크] USB Pixhawk 장치가 보이지 않습니다: /dev/ttyACM*, /dev/ttyUSB* 없음")
            return
        self._append_pixhawk_candidate_rows(rows, source="mavlink")
        self.workstation_pixhawk_discovery_status = f"FOUND {len(rows)} USB ENDPOINTS"
        for row in rows:
            self._log(
                f"[픽스호크] USB 후보: {row['value']} "
                f"baud={row['baud'] or '-'} status={row['status']}"
            )

    def _pixhawk_usb_discovery_failed(self, error: str) -> None:
        self.workstation_pixhawk_discovery_status = f"ERROR · {error}"
        self._log(f"[픽스호크] USB Pixhawk 연결 탐색 실패: {error}")

    def _clear_pixhawk_usb_discovery(self) -> None:
        self.pixhawk_usb_discovery_thread = None
        self.pixhawk_usb_discovery_worker = None
        self.pixhawk_usb_discover_button.setEnabled(True)

    def _usb_pixhawk_probe_status(self, device: str) -> str:
        try:
            with open(device, "rb", buffering=0):
                pass
        except PermissionError:
            return "permission denied: add user to dialout"
        except OSError as exc:
            return f"open failed: {exc}"
        return "no heartbeat"

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

    def apply_pixhawk_candidate(self, source: str, row: int, target: str) -> bool:
        """Apply a discovered endpoint selected by the Workstation surface."""

        if self._session_recording_active():
            self._log("[픽스호크] 세션 기록 중에는 Pixhawk Host/P1/P2 후보를 적용할 수 없습니다.")
            return False
        if source not in {"pc", "mavlink"} or target not in {"host", "left", "right"}:
            self._log("[픽스호크] 후보 source 또는 적용 대상이 올바르지 않습니다.")
            return False
        rows = self._pixhawk_candidate_rows_by_source.get(source, [])
        if isinstance(row, bool) or not isinstance(row, int) or not 0 <= row < len(rows):
            self._log("[픽스호크] 적용할 탐색 후보가 올바르지 않습니다.")
            return False
        self._pixhawk_candidate_table_for_source(source).selectRow(row)
        self.apply_selected_pixhawk_candidate(target, source=source)
        return True

    def _camera_auto_detect_failed(self, error: str) -> None:
        self.workstation_camera_discovery_status = f"ERROR · {error}"
        self._log(f"[카메라] 자동 탐색 실패: {error}")
        self.camera_connection_label.setText(f"camera RTSP: auto detect failed ({error})")
        self.last_rtsp_check_status = {"left": "check-failed", "right": "check-failed"}
        if self.auto_wiris_connect_pending:
            self._schedule_auto_wiris_connect_retry(f"탐색 오류: {error}")
        self._refresh_operation_header()

    def _clear_camera_auto_detect_worker(self) -> None:
        self.camera_auto_detect_thread = None
        self.camera_auto_detect_worker = None
        self.auto_wiris_connect_button.setEnabled(True)
        self.detect_control_button.setEnabled(True)
        self.right_detect_control_button.setEnabled(True)
        self.detect_rtsp_button.setEnabled(True)
        self.right_detect_rtsp_button.setEnabled(True)

    def swap_cameras(self) -> bool | None:
        if self._session_recording_active():
            self._log("[카메라] 세션 기록 중에는 Left/Right 카메라 매핑을 교체할 수 없습니다.")
            return False
        cameras_running = self.camera_connected or bool(self.left_pane.worker or self.right_pane.worker)
        detection_was_enabled = self.yolo_enabled
        if cameras_running:
            self.stop_yolo_streams()
        self.left_url_edit.setText(self._swap_line_edit_text(self.left_url_edit, self.right_url_edit))
        self.left_rtsp_ip_edit.setText(self._swap_line_edit_text(self.left_rtsp_ip_edit, self.right_rtsp_ip_edit))
        self.left_zoom_host_edit.setText(self._swap_line_edit_text(self.left_zoom_host_edit, self.right_zoom_host_edit))
        self._log("[카메라] Left/Right 카메라 매핑을 교체했습니다.")
        if cameras_running:
            self._start_camera_streams(enable_detection=detection_was_enabled, switch_to_depth=False)
        return None

    def swap_pixhawks(self) -> bool | None:
        if self._session_recording_active():
            self._log("[픽스호크] 세션 기록 중에는 P1/P2 Pixhawk 매핑을 교체할 수 없습니다.")
            return False
        mavlink_connected_sides = self._mavlink_open_sides()
        for side in ("left", "right"):
            if side in mavlink_connected_sides:
                self.disconnect_mavlink_pixhawk(side)
        self.left_mavlink_url_edit.setText(self._swap_line_edit_text(self.left_mavlink_url_edit, self.right_mavlink_url_edit))
        self.left_pixhawk_eth_ip_edit.setText(
            self._swap_line_edit_text(self.left_pixhawk_eth_ip_edit, self.right_pixhawk_eth_ip_edit)
        )
        self.left_mavlink_baud_spin.setValue(
            int(self._swap_spinbox_value(self.left_mavlink_baud_spin, self.right_mavlink_baud_spin))
        )
        self.left_pixhawk_bootstrap_url_edit.setText(
            self._swap_line_edit_text(self.left_pixhawk_bootstrap_url_edit, self.right_pixhawk_bootstrap_url_edit)
        )
        self.left_pixhawk_bootstrap_baud_spin.setValue(
            int(self._swap_spinbox_value(self.left_pixhawk_bootstrap_baud_spin, self.right_pixhawk_bootstrap_baud_spin))
        )
        self._log("[픽스호크] Pixhawk 1/2 MAVLink 매핑을 교체했습니다.")
        for side in ("left", "right"):
            if side in mavlink_connected_sides:
                self.connect_mavlink_pixhawk(side)
        return None

    def _swap_spinbox_value(self, left: QtWidgets.QAbstractSpinBox, right: QtWidgets.QAbstractSpinBox) -> object:
        left_value = left.value()
        right_value = right.value()
        right.setValue(left_value)
        return right_value

    def _swap_line_edit_text(self, left: QtWidgets.QLineEdit, right: QtWidgets.QLineEdit) -> str:
        left_text = left.text()
        right_text = right.text()
        right.setText(left_text)
        return right_text

    def _finish_auto_wiris_connect(self) -> None:
        startup_sequence = self._startup_camera_sequence_pending
        self._cancel_auto_wiris_connect_retry()
        if startup_sequence:
            self._log(
                "[카메라] 시작 시퀀스 완료: WIRIS Ethernet/RTSP 활성화와 좌/우 프레임 확인 후 "
                "카메라 스트림을 시작합니다."
            )
            self.start_yolo()
            return
        self._log("[카메라] WIRIS 자동 연결: 좌/우 RTSP 프레임 확인 완료. 카메라 프리뷰를 연결합니다.")
        self.connect_cameras()

    def check_cameras(self) -> None:
        if self.rtsp_check_thread is not None:
            self._log("[카메라] RTSP 스트림 확인이 이미 진행 중입니다.")
            return
        self._log("[연결] RTSP 확인은 설정된 카메라망 인터페이스와 URL을 사용합니다.")
        urls = {
            "left": self.left_url_edit.text().strip(),
            "right": self.right_url_edit.text().strip(),
        }
        self._log("[카메라] RTSP 포트와 실제 프레임 수신을 확인합니다.")
        self.check_button.setEnabled(False)
        self.camera_connection_label.setText("camera RTSP: checking stream frames...")
        self.last_rtsp_check_status = {"left": "checking", "right": "checking"}
        self._refresh_operation_header()
        self.rtsp_check_thread = QtCore.QThread(self)
        self.rtsp_check_worker = RtspStreamCheckWorker(
            urls,
            latency_ms=self.rtsp_latency_spin.value(),
            decoder=self.decoder_combo.currentData(),
            backend=self.stream_backend_combo.currentData(),
            transport=self.rtsp_transport_combo.currentData(),
            timeout_s=4.0,
        )
        self.rtsp_check_worker.moveToThread(self.rtsp_check_thread)
        self.rtsp_check_thread.started.connect(self.rtsp_check_worker.run)
        self.rtsp_check_worker.checked.connect(self._apply_rtsp_stream_check)
        self.rtsp_check_worker.failed.connect(self._rtsp_stream_check_failed)
        self.rtsp_check_worker.finished.connect(self.rtsp_check_thread.quit)
        self.rtsp_check_worker.finished.connect(self.rtsp_check_worker.deleteLater)
        self.rtsp_check_thread.finished.connect(self.rtsp_check_thread.deleteLater)
        self.rtsp_check_thread.finished.connect(self._clear_rtsp_check_worker)
        self.rtsp_check_thread.start()

    def check_wiris_control(self) -> None:
        if self.wiris_control_check_thread is not None:
            self._log("[카메라] WIRIS 제어 확인이 이미 진행 중입니다.")
            return
        self._log("[연결] WIRIS 제어 확인은 설정된 Control Host/port를 사용합니다.")
        targets = []
        for label, host in self.zoom_hosts("both"):
            normalized_host = host.strip()
            if normalized_host:
                self._log_rtsp_control_host_mismatch(label, normalized_host)
            targets.append((label, normalized_host, self._control_port_for_camera(label)))
        self.check_control_button.setEnabled(False)
        self.camera_connection_label.setText("camera control: checking...")
        self.wiris_control_check_thread = QtCore.QThread(self)
        self.wiris_control_check_worker = WirisControlCheckWorker(
            targets,
            timeout_s=self.zoom_timeout_spin.value(),
        )
        self.wiris_control_check_worker.moveToThread(self.wiris_control_check_thread)
        self.wiris_control_check_thread.started.connect(self.wiris_control_check_worker.run)
        self.wiris_control_check_worker.checked.connect(self._apply_wiris_control_check)
        self.wiris_control_check_worker.failed.connect(self._wiris_control_check_failed)
        self.wiris_control_check_worker.finished.connect(self.wiris_control_check_thread.quit)
        self.wiris_control_check_worker.finished.connect(self.wiris_control_check_worker.deleteLater)
        self.wiris_control_check_thread.finished.connect(self.wiris_control_check_thread.deleteLater)
        self.wiris_control_check_thread.finished.connect(self._clear_wiris_control_check_worker)
        self.wiris_control_check_thread.start()

    def _apply_wiris_control_check(self, results: object) -> None:
        statuses: list[str] = []
        for result in list(results or []):
            if not isinstance(result, dict):
                continue
            label = str(result.get("label") or "")
            host = str(result.get("host") or "")
            port = result.get("port", "")
            state = str(result.get("state") or "error")
            camera_name = self._camera_name_ko(label)
            statuses.append(f"{label}={state}")
            if state == "missing-host":
                self._log(f"[카메라] {camera_name} WIRIS 제어 host가 비어 있습니다.")
            elif state == "control-open":
                self._log(f"[카메라] {camera_name} WIRIS 제어 포트 열림: {host}:{port}")
            elif state == "control-closed":
                self._log(f"[카메라] {camera_name} WIRIS 제어 포트 닫힘/오류: {host}:{port}")
            else:
                self._log(f"[카메라] {camera_name} WIRIS 제어 확인 오류: {result.get('error', state)}")
        self.camera_connection_label.setText(f"camera control: {', '.join(statuses)}")
        self._refresh_operation_header()

    def _wiris_control_check_failed(self, error: str) -> None:
        self._log(f"[카메라] WIRIS 제어 확인 실패: {error}")
        self.camera_connection_label.setText(f"camera control: failed ({error})")
        self._refresh_operation_header()

    def _clear_wiris_control_check_worker(self) -> None:
        self.wiris_control_check_thread = None
        self.wiris_control_check_worker = None
        self.check_control_button.setEnabled(True)

    def _ethernet_mode_candidate_ports(self, camera: str) -> list[int]:
        ports = [
            self._control_port_for_camera(camera),
            DEFAULT_ZOOM_CONTROL_PORT,
            self.zoom_port_spin.value(),
        ]
        unique_ports = []
        for port in ports:
            port = int(port)
            if 1 <= port <= 65535 and port not in unique_ports:
                unique_ports.append(port)
        return unique_ports

    def _start_rtsp_enable_worker(self, *, target_labels: tuple[str, ...], reactivate: bool) -> None:
        hosts = {label: host.strip() for label, host in self.zoom_hosts("both")}
        candidate_ports = {label: self._ethernet_mode_candidate_ports(label) for label in ("left", "right")}
        urls = {
            "left": self.left_url_edit.text().strip(),
            "right": self.right_url_edit.text().strip(),
        }
        self.enable_rtsp_button.setEnabled(False)
        status_text = "camera RTSP reactivate: enabling safely..." if reactivate else "camera RTSP enable: enabling..."
        self.camera_connection_label.setText(status_text)
        self.rtsp_enable_thread = QtCore.QThread(self)
        self.rtsp_enable_worker = WirisRtspEnableWorker(
            hosts=hosts,
            candidate_ports=candidate_ports,
            urls=urls,
            timeout_s=self.zoom_timeout_spin.value(),
            target_labels=target_labels,
            reactivate=reactivate,
        )
        self.rtsp_enable_worker.moveToThread(self.rtsp_enable_thread)
        self.rtsp_enable_thread.started.connect(self.rtsp_enable_worker.run)
        self.rtsp_enable_worker.enabled.connect(self._apply_wiris_rtsp_enable_result)
        self.rtsp_enable_worker.failed.connect(self._wiris_rtsp_enable_failed)
        self.rtsp_enable_worker.finished.connect(self.rtsp_enable_thread.quit)
        self.rtsp_enable_worker.finished.connect(self.rtsp_enable_worker.deleteLater)
        self.rtsp_enable_thread.finished.connect(self.rtsp_enable_thread.deleteLater)
        self.rtsp_enable_thread.finished.connect(self._clear_rtsp_enable_worker)
        self.rtsp_enable_thread.start()

    def enable_wiris_rtsp(self, *, target_labels: tuple[str, ...] = ("left", "right")) -> None:
        if self.rtsp_enable_thread is not None:
            self._log("[카메라] RTSP 송출 활성화가 이미 진행 중입니다.")
            return
        if self._rtsp_reactivation_side is not None:
            self._log("[카메라] 카메라별 RTSP 재활성화가 진행 중입니다.")
            return
        self._log("[연결] RTSP 활성화는 설정된 Control Host/port와 영상 URL을 사용합니다.")
        self._start_rtsp_enable_worker(target_labels=target_labels, reactivate=False)

    def reactivate_wiris_rtsp(self, camera: str) -> bool | None:
        if camera not in {"left", "right"}:
            self._log(f"[카메라] 알 수 없는 재활성화 대상입니다: {camera}")
            return False
        if self._session_recording_active():
            self._log("[카메라] 세션 기록 중에는 카메라 RTSP 재활성화를 실행할 수 없습니다.")
            return False
        if self.rtsp_enable_thread is not None or self.rtsp_check_thread is not None:
            self._log("[카메라] 다른 RTSP 확인/활성화 작업이 진행 중입니다.")
            return False
        host = dict(self.zoom_hosts("both")).get(camera, "").strip()
        if not host:
            self._log(f"[카메라] {self._camera_name_ko(camera)} Control Host가 비어 있어 재활성화할 수 없습니다.")
            self.camera_connection_label.setText(f"camera RTSP reactivate: {camera}=missing-host")
            return False
        if self.auto_wiris_connect_pending:
            self._cancel_auto_wiris_connect_retry()

        self._rtsp_reactivation_side = camera
        self._rtsp_reactivation_enable_detection = bool(self.yolo_enabled)
        self._rtsp_reactivation_generation += 1
        generation = self._rtsp_reactivation_generation
        camera_name = self._camera_name_ko(camera)
        self._log(
            f"[카메라] {camera_name} RTSP 재활성화를 시작합니다: "
            "스트림 정리 → GETH 확인 → SETH TRUE → 프레임 확인 → Preview 재연결 "
            "(제어 서비스 보호를 위해 SETH FALSE 생략)"
        )
        self.camera_connection_label.setText(f"camera RTSP reactivate: {camera}=stopping")
        pane = self.left_pane if camera == "left" else self.right_pane
        pane.stop(wait=False)

        def start_when_stopped() -> None:
            if generation != self._rtsp_reactivation_generation or self._rtsp_reactivation_side != camera:
                return
            if pane.worker is not None or pane.thread is not None:
                pane.stop(wait=False)
                QtCore.QTimer.singleShot(100, start_when_stopped)
                return
            self._start_rtsp_enable_worker(target_labels=(camera,), reactivate=True)

        start_when_stopped()
        return None

    def _apply_wiris_rtsp_enable_result(self, result: object) -> None:
        data = dict(result or {})
        for label, port in dict(data.get("corrected_ports") or {}).items():
            self._set_control_port_for_camera(str(label), int(port))
            self.zoom_port_spin.setValue(int(port))
            self._save_current_settings()
        for message in list(data.get("logs") or []):
            self._log(str(message))
        statuses = [str(status) for status in list(data.get("statuses") or [])]
        mode = str(data.get("mode") or "enable")
        if mode == "enable" and self.auto_wiris_connect_pending:
            for status in statuses:
                label, separator, state = status.partition("=")
                if not separator or state != "enable-sent" or label not in {"left", "right"}:
                    continue
                host = dict(self.zoom_hosts("both")).get(label, "").strip()
                if host:
                    self._auto_wiris_enabled_cameras.add((label, host))
                    self._log(
                        f"[카메라] {self._camera_name_ko(label)} SETH TRUE 승인 상태를 기억합니다. "
                        "RTSP 실패 재시도에서는 SETH TRUE를 반복 전송하지 않습니다."
                    )
        self.camera_connection_label.setText(
            f"camera RTSP {'reactivate' if mode == 'reactivate' else 'enable'}: {', '.join(statuses)}"
        )
        if mode == "reactivate" and self._rtsp_reactivation_side is not None:
            camera = self._rtsp_reactivation_side
            expected_status = f"{camera}=reactivated-sent"
            if expected_status not in statuses:
                self._finish_camera_rtsp_reactivation(False, f"{camera}=reactivation-failed")
            elif data.get("should_check"):
                self._log(f"[카메라] {self._camera_name_ko(camera)} RTSP 재활성화 후 실제 프레임을 확인합니다.")
                self.check_cameras()
            else:
                self._finish_camera_rtsp_reactivation(False, f"{camera}=no-stream")
            return
        if data.get("should_check"):
            self.check_cameras()
        elif self.auto_wiris_connect_pending:
            self._schedule_auto_wiris_connect_retry("RTSP 활성화에 성공한 카메라가 없음")

    def _wiris_rtsp_enable_failed(self, error: str) -> None:
        self._log(f"[카메라] RTSP 송출 활성화 실패: {error}")
        self.camera_connection_label.setText(f"camera RTSP enable: failed ({error})")
        if self._rtsp_reactivation_side is not None:
            self._finish_camera_rtsp_reactivation(False, error)
            return
        if self.auto_wiris_connect_pending:
            self._schedule_auto_wiris_connect_retry(f"RTSP 활성화 오류: {error}")

    def _clear_rtsp_enable_worker(self) -> None:
        self.rtsp_enable_thread = None
        self.rtsp_enable_worker = None
        self.enable_rtsp_button.setEnabled(True)

    def _finish_camera_rtsp_reactivation(self, success: bool, detail: str = "") -> None:
        camera = self._rtsp_reactivation_side
        if camera is None:
            return
        enable_detection = self._rtsp_reactivation_enable_detection
        self._rtsp_reactivation_side = None
        self._rtsp_reactivation_enable_detection = False
        self._rtsp_reactivation_generation += 1
        camera_name = self._camera_name_ko(camera)
        if not success:
            self._log(f"[카메라] {camera_name} RTSP 재활성화 실패: {detail or 'frame check failed'}")
            self.camera_connection_label.setText(f"camera RTSP reactivate: {camera}=failed")
            self._refresh_operation_header()
            return

        pane = self.left_pane if camera == "left" else self.right_pane
        url = self.left_url_edit.text().strip() if camera == "left" else self.right_url_edit.text().strip()
        self.camera_frame_times[camera].clear()
        self.camera_frame_metadata[camera] = {}
        self.camera_frame_metadata_history[camera].clear()
        self.camera_stream_stats[camera] = {}
        self.active_stream_decoders[camera] = None
        if enable_detection:
            self.tabs.setCurrentIndex(self.depth_tab_index)
        self._camera_sync_transport_override = None
        self._set_camera_sync_state(
            CAMERA_SYNC_STATUS_NOT_VERIFIED,
            f"{camera_name} RTSP가 재활성화되어 RTCP/NTP 검증이 필요합니다.",
        )
        started = pane.start_stream(
            url,
            enable_detection=enable_detection,
            **self.yolo_kwargs(),
            **self._video_recording_kwargs(camera),
        )
        if not started:
            self._log(f"[카메라] {camera_name} RTSP 재활성화 후 Preview 재연결을 시작하지 못했습니다.")
            self.camera_connection_label.setText(f"camera RTSP reactivate: {camera}=preview-start-failed")
            self._refresh_operation_header()
            return
        self.camera_connected = bool(self.left_pane.worker or self.right_pane.worker)
        self.yolo_enabled = bool(enable_detection)
        self._log(f"[카메라] {camera_name} RTSP 프레임 확인 완료. 해당 Preview를 다시 연결했습니다.")
        self._refresh_session_toggle_buttons()
        self._refresh_operation_header()

    def _rtsp_widgets_for_camera(self, camera: str) -> tuple[QtWidgets.QLineEdit, QtWidgets.QLineEdit]:
        if camera == "left":
            return self.left_url_edit, self.left_rtsp_ip_edit
        return self.right_url_edit, self.right_rtsp_ip_edit

    def _sync_rtsp_url_from_control_host(self, camera: str, host: str) -> None:
        url_edit, ip_edit = self._rtsp_widgets_for_camera(camera)
        old_host = self._rtsp_url_host_only(url_edit.text())
        self._replace_rtsp_url_host(url_edit, host)
        ip_edit.setText(host)
        if old_host != host:
            camera_name = self._camera_name_ko(camera)
            self._log(f"[카메라] {camera_name} RTSP URL host 자동 갱신: {old_host or 'invalid'} -> {host}")

    def _log_rtsp_control_host_mismatch(self, camera: str, control_host: str) -> None:
        url_edit, _ip_edit = self._rtsp_widgets_for_camera(camera)
        rtsp_host = self._rtsp_url_host_only(url_edit.text())
        if not rtsp_host or rtsp_host == control_host:
            return
        camera_name = self._camera_name_ko(camera)
        self._log(
            f"[카메라] 경고: {camera_name} OSD IP Address({control_host})와 "
            f"RTSP URL host({rtsp_host})가 다릅니다. RTSP 변경은 RTSP port / 영상 그룹의 IP Address 적용을 사용하세요."
        )

    def apply_control_settings(self, camera: str) -> None:
        camera_name = self._camera_name_ko(camera)
        host_edit = self.left_zoom_host_edit if camera == "left" else self.right_zoom_host_edit
        host = host_edit.text().strip()
        port = self._control_port_for_camera(camera)
        if not host:
            self._log(f"[카메라] {camera_name} IP Address/Telnet port 적용 실패: IP Address가 비어 있습니다.")
            self.camera_connection_label.setText(f"camera control: {camera}=missing-host")
            self._refresh_operation_header()
            return
        self._sync_rtsp_url_from_control_host(camera, host)
        self._save_current_settings()
        self._log(f"[카메라] {camera_name} IP Address/Telnet port 적용: {host}:{port}")
        self.camera_connection_label.setText(f"camera control: {camera}=applied {host}:{port}")
        self._refresh_operation_header()

    def apply_rtsp_url(self, camera: str) -> None:
        camera_name = self._camera_name_ko(camera)
        url_edit, ip_edit = self._rtsp_widgets_for_camera(camera)
        url = url_edit.text().strip()
        new_ip = ip_edit.text().strip()
        try:
            IPv4Address(new_ip)
            current_host, port = rtsp_host_port(url)
        except Exception as exc:
            self._log(f"[카메라] {camera_name} RTSP 입력 오류: {exc}")
            self.camera_connection_label.setText(f"camera RTSP: {camera}=invalid-input")
            return
        if current_host == new_ip:
            self._log(f"[카메라] {camera_name} RTSP URL 적용 건너뜀: 현재 URL host와 같습니다 ({new_ip}).")
            self.camera_connection_label.setText(f"camera RTSP: {camera}=unchanged")
            return
        self._replace_rtsp_url_host(url_edit, new_ip)
        self._log(f"[카메라] {camera_name} RTSP URL 적용: {current_host} -> {new_ip} (port={port})")
        self.camera_connection_label.setText(f"camera RTSP: {camera}={current_host}->{new_ip}")
        self._refresh_operation_header()

    def _replace_rtsp_url_host(self, url_edit: QtWidgets.QLineEdit, new_host: str) -> None:
        url = url_edit.text().strip()
        try:
            _old_host, port = rtsp_host_port(url)
        except Exception:
            url_edit.setText(f"rtsp://{new_host}:8554/visible")
            return
        suffix = "/visible"
        if url.startswith("rtsp://") and "/" in url.removeprefix("rtsp://"):
            suffix = "/" + url.removeprefix("rtsp://").split("/", 1)[1]
        url_edit.setText(f"rtsp://{new_host}:{port}{suffix}")

    def _apply_rtsp_stream_check(self, results: object) -> None:
        statuses = []
        next_status = {}
        for label in ("left", "right"):
            result = dict((results or {}).get(label, {}))
            camera_name = self._camera_name_ko(label)
            if not result.get("port_open"):
                error = result.get("error", "port closed")
                self._log(f"[카메라] {camera_name} RTSP 포트 닫힘/오류: {error}")
                statuses.append(f"{label}=port-closed")
                next_status[label] = "port-closed"
                continue
            if not result.get("frame_ok"):
                error = result.get("error", "no frame")
                decoder = result.get("decoder", self.decoder_combo.currentText())
                self._log(f"[카메라] {camera_name} RTSP 포트 열림, 프레임 수신 실패: {error} (decoder={decoder})")
                statuses.append(f"{label}=open-no-frame")
                next_status[label] = "open-no-frame"
                continue
            pts = result.get("pts_s")
            pts_text = "PTS=n/a" if pts is None else f"PTS={float(pts):.3f}s"
            decoder = result.get("decoder", self.decoder_combo.currentText())
            backend = result.get("backend", self.stream_backend_combo.currentText())
            self._set_active_stream_decoder(label, str(decoder))
            self._log(
                f"[카메라] {camera_name} RTSP 스트림 정상: "
                f"{result.get('width')}x{result.get('height')}, {pts_text}, "
                f"firstFrame={float(result.get('elapsed_s', 0.0)):.2f}s, backend={backend}, decoder={decoder}"
            )
            statuses.append(f"{label}=frame-ok")
            next_status[label] = "frame-ok"
        self.camera_connection_label.setText(f"camera RTSP stream: {', '.join(statuses)}")
        self.last_rtsp_check_status = next_status
        self._refresh_operation_header()
        if self._rtsp_reactivation_side is not None:
            camera = self._rtsp_reactivation_side
            if next_status.get(camera) == "frame-ok":
                self._finish_camera_rtsp_reactivation(True)
            else:
                self._finish_camera_rtsp_reactivation(False, str(next_status.get(camera) or "frame check failed"))
            return
        if self.auto_wiris_connect_pending:
            if set(next_status) == {"left", "right"} and all(
                status == "frame-ok" for status in next_status.values()
            ):
                self._finish_auto_wiris_connect()
            else:
                self._log("[카메라] WIRIS 자동 연결: RTSP 프레임 확인 실패. 제어 주소, Ethernet Mode, 영상 주소를 확인하세요.")
                self._schedule_auto_wiris_connect_retry("RTSP 프레임 확인 실패")

    def _rtsp_stream_check_failed(self, error: str) -> None:
        self._log(f"[카메라] RTSP 스트림 확인 실패: {error}")
        self.camera_connection_label.setText(f"camera RTSP stream: check failed ({error})")
        self.last_rtsp_check_status = {"left": "check-failed", "right": "check-failed"}
        if self._rtsp_reactivation_side is not None:
            self._finish_camera_rtsp_reactivation(False, error)
            return
        if self.auto_wiris_connect_pending:
            self._schedule_auto_wiris_connect_retry(f"RTSP 확인 오류: {error}")
        self._refresh_operation_header()

    def _clear_rtsp_check_worker(self) -> None:
        self.rtsp_check_thread = None
        self.rtsp_check_worker = None
        self.check_button.setEnabled(True)

    def _sample_hz(self, timestamps: deque[float]) -> float:
        if len(timestamps) < 2:
            return 0.0
        elapsed_s = timestamps[-1] - timestamps[0]
        if elapsed_s <= 0.0:
            return 0.0
        return (len(timestamps) - 1) / elapsed_s

    def _update_camera_connection_status(self) -> None:
        now_s = time.monotonic()
        parts = []
        for camera in ("left", "right"):
            stats = self.camera_stream_stats.get(camera) or {}
            if stats and now_s - float(stats.get("timestamp_s", 0.0)) < 2.0:
                state = "connected"
                parts.append(format_camera_stream_stats(camera, state, stats))
                continue
            timestamps = self.camera_frame_times[camera]
            if not timestamps:
                parts.append(f"{camera_display_name(camera)}=대기")
                continue
            age_s = now_s - timestamps[-1]
            state = "connected" if age_s < 2.0 else "stale"
            state_text = "연결" if state == "connected" else "지연"
            parts.append(f"{camera_display_name(camera)}={state_text} {self._sample_hz(timestamps):.1f} Hz")
        self.camera_connection_label.setText(f"카메라: {', '.join(parts)}")
        self._update_runtime_camera_stats_labels(now_s)
        self._refresh_operation_header()

    def _update_runtime_camera_settings_layout_mode(self, width: int | None = None) -> None:
        if not hasattr(self, "runtime_header_layout") or not hasattr(self, "runtime_camera_settings_group"):
            return
        current_width = self.runtime_camera_settings_group.width() if width is None else width
        current_direction = self.runtime_header_layout.direction()
        # Hysteresis prevents flip-flopping right at the breakpoint: toggling
        # direction changes the header's own sizeHint, which changes how much
        # vertical space is left for the camera panes below it (video_row
        # holds nearly all of runtime_layout's stretch). Without a dead zone,
        # each resize pass near the breakpoint could cross back over it,
        # making the camera preview visibly jitter in size.
        if current_direction == QtWidgets.QBoxLayout.LeftToRight:
            should_stack = current_width < RUNTIME_CAMERA_LAYOUT_BREAKPOINT_PX - RUNTIME_CAMERA_LAYOUT_HYSTERESIS_PX
            direction = QtWidgets.QBoxLayout.TopToBottom if should_stack else QtWidgets.QBoxLayout.LeftToRight
        else:
            should_unstack = current_width >= RUNTIME_CAMERA_LAYOUT_BREAKPOINT_PX + RUNTIME_CAMERA_LAYOUT_HYSTERESIS_PX
            direction = QtWidgets.QBoxLayout.LeftToRight if should_unstack else QtWidgets.QBoxLayout.TopToBottom
        if direction != current_direction:
            self.runtime_header_layout.setDirection(direction)

    def _format_runtime_pair_status(self) -> tuple[str, str]:
        if self.selected_runtime_frame_pair_delta_s is None:
            text = "Pair: 대기 | pair: missing"
            return text, text
        left_metadata = self.selected_runtime_frame_metadata.get("left") or {}
        right_metadata = self.selected_runtime_frame_metadata.get("right") or {}
        left_pair = format_runtime_camera_timestamp_compact(left_metadata)
        right_pair = format_runtime_camera_timestamp_compact(right_metadata)
        delta_ms = self.selected_runtime_frame_pair_delta_s * 1000.0
        parts = [
            f"Pair: {self.selected_runtime_frame_pair_basis or 'matched'}",
            f"pair L {left_pair}",
            f"R {right_pair}",
            f"Δ {delta_ms:.1f} ms",
        ]
        if self.selected_runtime_frame_pair_camera_rtc_delta_s is not None:
            parts.append(f"RTCP/NTPΔ {self.selected_runtime_frame_pair_camera_rtc_delta_s * 1000.0:.1f} ms")
        text = " | ".join(parts)
        tooltip = (
            f"{text}\n"
            f"left pair 전체 시각: {format_runtime_camera_timestamp(left_metadata)}\n"
            f"right pair 전체 시각: {format_runtime_camera_timestamp(right_metadata)}"
        )
        return text, tooltip

    def _format_runtime_camera_summary(self) -> tuple[str, str]:
        def compact_camera_status(camera: str, label: QtWidgets.QLabel) -> str:
            text = label.text()
            prefix = "L" if camera == "left" else "R"
            if "대기" in text:
                state = "대기"
            elif "지연" in text:
                state = "지연"
            else:
                state = "연결"
            stats = self.camera_stream_stats.get(camera) or {}
            resolution = camera_resolution_text(stats).removeprefix(" @ ") or "--"
            decoded_hz = float(stats.get("decoded_hz", 0.0)) if stats else 0.0
            live_metadata = self.camera_frame_metadata.get(camera) or {}
            ntp_text = format_runtime_camera_ntp_compact(live_metadata)
            text = f"{prefix} {state:<2} {resolution:>9} {decoded_hz:4.1f}Hz NTP:{ntp_text}"
            if stats.get("enable_detection"):
                inferred_hz = float(stats.get("inferred_hz", 0.0))
                text += f" Y:{inferred_hz:4.1f}Hz"
            return text

        left_text = compact_camera_status("left", self.left_runtime_stats_label)
        right_text = compact_camera_status("right", self.right_runtime_stats_label)
        pair_text, pair_tooltip = self._format_runtime_pair_status()
        if self.selected_runtime_frame_pair_delta_s is None:
            pair_summary = "Pair 대기            "
        else:
            basis = (self.selected_runtime_frame_pair_basis or "matched")[:18]
            if self.selected_runtime_frame_pair_camera_rtc_delta_s is not None:
                delta_ms = self.selected_runtime_frame_pair_camera_rtc_delta_s * 1000.0
            else:
                delta_ms = self.selected_runtime_frame_pair_delta_s * 1000.0
            pair_summary = f"Pair {basis:<18} Δ{delta_ms:+7.1f}ms"
        summary = f"{left_text} | {right_text} | {pair_summary}"

        clock_lines = []
        left_metadata = self.camera_frame_metadata.get("left") or {}
        right_metadata = self.camera_frame_metadata.get("right") or {}
        for camera_prefix, metadata in (("L", left_metadata), ("R", right_metadata)):
            ntp_text = format_unix_timestamp_compact(metadata.get("rtcp_frame_ntp_unix_s"))
            pc_utc_text = format_unix_timestamp_compact(metadata.get("rtcp_frame_received_wall_s"))
            delta_text = format_signed_ms_compact(metadata.get("rtcp_frame_receive_delay_s"))
            clock_lines.append(f"{camera_prefix} NTP: {ntp_text} | PC UTC: {pc_utc_text} | Δt(PC-NTP): {delta_text}")
        left_ntp_s = left_metadata.get("rtcp_frame_ntp_unix_s")
        right_ntp_s = right_metadata.get("rtcp_frame_ntp_unix_s")
        if isinstance(left_ntp_s, (int, float)) and isinstance(right_ntp_s, (int, float)):
            ntp_delta_text = format_signed_ms_compact(float(right_ntp_s) - float(left_ntp_s))
        else:
            ntp_delta_text = "--"
        clock_lines.append(f"NTP Δt(R-L): {ntp_delta_text}")

        tooltip = (
            f"{summary}\n"
            f"{self.left_runtime_stats_label.toolTip()}\n"
            f"{self.right_runtime_stats_label.toolTip()}\n"
            f"{pair_tooltip}\n"
            + "\n".join(clock_lines)
        )
        return summary, tooltip

    def _update_runtime_camera_stats_labels(self, now_s: float | None = None) -> None:
        now_s = time.monotonic() if now_s is None else now_s
        for camera, label in (
            ("left", self.left_runtime_stats_label),
            ("right", self.right_runtime_stats_label),
        ):
            stats = self.camera_stream_stats.get(camera) or {}
            timestamps = self.camera_frame_times[camera]
            stats_age_s = now_s - float(stats.get("timestamp_s", 0.0)) if stats else float("inf")
            frame_age_s = now_s - timestamps[-1] if timestamps else float("inf")
            has_recent_stats = stats_age_s < 2.0
            has_recent_frame = frame_age_s < 2.0
            live_metadata = self.camera_frame_metadata.get(camera) or {}
            status_text = (
                f"{format_runtime_camera_stats(camera, stats, self._sample_hz(timestamps), stale=not (has_recent_stats or has_recent_frame))}"
                f" | live: {format_runtime_camera_timestamp_compact(live_metadata)}"
            )
            tooltip_text = f"{status_text}\nlive 전체 시각: {format_runtime_camera_timestamp(live_metadata)}"
            label.setText(status_text)
            label.setToolTip(tooltip_text)
        if hasattr(self, "runtime_pair_status_label"):
            pair_text, pair_tooltip = self._format_runtime_pair_status()
            self.runtime_pair_status_label.setText(pair_text)
            self.runtime_pair_status_label.setToolTip(pair_tooltip)
        if hasattr(self, "runtime_camera_summary_label"):
            summary_text, summary_tooltip = self._format_runtime_camera_summary()
            self.runtime_camera_summary_label.setText(summary_text)
            self.runtime_camera_summary_label.setToolTip(summary_tooltip)

    def _update_camera_stream_stats(self, camera: str, payload: object) -> None:
        stats = dict(payload or {})
        stats["timestamp_s"] = time.monotonic()
        self.camera_stream_stats[camera] = stats
        self._update_camera_connection_status()

    def _update_pixhawk_connection_status(self) -> None:
        now_s = time.monotonic()
        parts = []
        for camera in ("left", "right"):
            timestamps = self.pixhawk_attitude_times[camera]
            if not timestamps:
                parts.append(f"{camera_display_name(camera)}=대기")
                continue
            age_s = now_s - timestamps[-1]
            state = "connected" if age_s < 1.0 else "stale"
            state_text = "연결" if state == "connected" else "지연"
            parts.append(f"{camera_display_name(camera)}={state_text} {self._sample_hz(timestamps):.1f} Hz")
        self.pixhawk_connection_label.setText(f"픽스호크 자세: {', '.join(parts)}")
        self._refresh_operation_header()

    def _set_focal_override_enabled(self, enabled: bool) -> None:
        self.focal_spin.setEnabled(bool(enabled))
        if hasattr(self, "intrinsics_source_combo"):
            desired = "manual" if enabled else "checkerboard"
            if self.intrinsics_source_combo.currentData() != desired:
                self._set_combo_data(self.intrinsics_source_combo, desired)
        self._update_distance_settings_visibility()
        self._refresh_camera_parameters_tab()

    def _handle_intrinsics_source_changed(self, _index: int) -> None:
        manual = self._intrinsics_source() == "manual"
        if self.manual_focal_checkbox.isChecked() != manual:
            self.manual_focal_checkbox.setChecked(manual)
            return
        self._update_distance_settings_visibility()
        self._refresh_camera_parameters_tab()

    def _handle_distance_mode_changed(self, _index: int) -> None:
        mode = self._distance_mode()
        self._distance_mode_last_source = mode
        self._update_distance_settings_visibility()
        self._refresh_camera_parameters_tab()
        self._refresh_optical_alignment_view()

    def _handle_measurement_display_mode_changed(self, _index: int) -> None:
        if self._measurement_display_mode() == "preview":
            self._clear_measurement_pair_display()
        else:
            self._refresh_distance()
        self._refresh_camera_parameters_tab()

    def _intrinsics_source(self) -> str:
        if not hasattr(self, "intrinsics_source_combo"):
            return "manual" if self.manual_focal_checkbox.isChecked() else "checkerboard"
        source = str(self.intrinsics_source_combo.currentData() or "checkerboard")
        return "manual" if source == "manual" else "checkerboard"

    def _update_distance_settings_visibility(self) -> None:
        if not hasattr(self, "intrinsics_source_combo"):
            return
        manual_intrinsics = self._intrinsics_source() == "manual"
        for widget in (self.manual_focal_label, self.focal_spin):
            widget.setVisible(manual_intrinsics)

        for widget in (self.distance_mode_label, self.distance_mode_combo):
            widget.setVisible(True)
        for widget in (self.baseline_label, self.baseline_spin):
            widget.setVisible(False)

        if self._distance_mode() == "checkerboard":
            self.mount_assumption_label.setText(
                "장착 가정: 체커보드로 계산한 고정 좌/우 카메라 기하를 사용합니다. "
                "체커보드 R/T는 좌측 optical RDF에서 우측 optical RDF로 변환해 적용합니다."
            )
        else:
            self.mount_assumption_label.setText(
                "장착 가정: GPS/RTK baseline only. "
                "수동 baseline 입력은 사용하지 않고 좌/우 Pixhawk GPS baseline과 자세만 계산에 사용합니다."
            )

        source_text = "수동 fallback" if manual_intrinsics else "체커보드 보정값"
        if self.active_intrinsics is not None:
            left_fx = self.active_intrinsics.left.camera_matrix[0][0]
            right_fx = self.active_intrinsics.right.camera_matrix[0][0]
            detail = f"적용됨, fx 평균={(left_fx + right_fx) / 2.0:.3f} px"
        elif self.last_calibration_result is not None:
            detail = "불러옴/계산됨, 아직 적용 전"
        else:
            detail = "대기 중"
        self.intrinsics_status_label.setText(f"내부 파라미터: {source_text} | {detail}")

    def _configure_collapsible_group(
        self, group: QtWidgets.QGroupBox, *, checked: bool = True, collapsed_height: int = 34
    ) -> None:
        group.setCheckable(True)
        group.setChecked(bool(checked))

        def set_group_visible(current: bool, target: QtWidgets.QGroupBox = group) -> None:
            self._set_group_children_visible(target, current)
            target.setMaximumHeight(16777215 if current else collapsed_height)

        group.toggled.connect(set_group_visible)
        set_group_visible(bool(checked))

    def _set_group_children_visible(self, group: QtWidgets.QGroupBox, visible: bool) -> None:
        for child in group.findChildren(QtWidgets.QWidget):
            child.setVisible(visible)

    def _build_calibration_preview_label(self, text: str) -> QtWidgets.QLabel:
        label = QtWidgets.QLabel(text)
        label.setObjectName("calibrationPreview")
        label.setMinimumSize(320, 180)
        label.setAlignment(QtCore.Qt.AlignCenter)
        label.setSizePolicy(QtWidgets.QSizePolicy.Ignored, QtWidgets.QSizePolicy.Ignored)
        return label

    def _build_parameter_text(self) -> QtWidgets.QPlainTextEdit:
        editor = QtWidgets.QPlainTextEdit()
        editor.setReadOnly(True)
        editor.setLineWrapMode(QtWidgets.QPlainTextEdit.NoWrap)
        editor.setMinimumHeight(180)
        font = QtGui.QFontDatabase.systemFont(QtGui.QFontDatabase.FixedFont)
        editor.setFont(font)
        return editor

    def _format_matrix(self, matrix: list[list[float]]) -> str:
        return "\n".join("  [" + ", ".join(f"{value:12.6f}" for value in row) + "]" for row in matrix)

    def _intrinsics_source_label(self) -> str:
        if self.active_intrinsics is not None:
            return "active calibration result"
        if self.last_calibration_result is not None:
            return "latest calibration result (not applied)"
        return "not loaded"

    def _focal_source_label(self) -> str:
        if self._intrinsics_source() == "manual":
            return f"manual focal spinbox ({self.focal_spin.value():.6f} px)"
        if self.active_intrinsics is not None:
            left_fx = self.active_intrinsics.left.camera_matrix[0][0]
            right_fx = self.active_intrinsics.right.camera_matrix[0][0]
            return f"calibrated intrinsics average fx ({(left_fx + right_fx) / 2.0:.6f} px)"
        if self.last_calibration_result is not None:
            return f"latest calibration result not applied; spinbox fallback ({self.focal_spin.value():.6f} px)"
        return f"spinbox fallback ({self.focal_spin.value():.6f} px)"

    def _baseline_source_label(self, baseline_source: str, baseline_m: float) -> str:
        if baseline_source == "gps-rtk-waiting":
            return "waiting for live GPS/RTK baseline"
        if baseline_source == "gps-rtk":
            return f"live GPS/RTK baseline ({baseline_m:.6f} m)"
        if baseline_source == "gps-rtk-blocked":
            return "live GPS/RTK baseline blocked"
        if baseline_source.startswith("RTK"):
            return f"{baseline_source} from Pixhawk/GPS"
        return baseline_source

    def _extrinsics_source_label(self, extrinsic: CameraExtrinsicState | None) -> str:
        if extrinsic is None:
            return "none (Pixhawk disconnected or waiting)"
        topic = extrinsic.topic.strip()
        if topic.startswith("mavlink/"):
            return f"MAVLink ({topic})"
        return f"MAVLink ({topic})" if topic else "MAVLink telemetry"

    def _distance_mode(self) -> str:
        if not hasattr(self, "distance_mode_combo"):
            return "pixhawk"
        mode = str(self.distance_mode_combo.currentData() or "pixhawk")
        return mode if mode in {"pixhawk", "checkerboard"} else "pixhawk"

    def _measurement_display_mode(self) -> str:
        value = self._workstation_setting_overrides.get("measurement_display_mode", "detection")
        return str(value) if value in {"detection", "preview"} else "detection"

    def _distance_mode_label(self) -> str:
        selected = str(getattr(self, "_measurement_selected_method_id", "") or "")
        if selected in MEASUREMENT_METHOD_LABELS:
            return str(MEASUREMENT_METHOD_LABELS[selected])
        if self._distance_mode() == "checkerboard":
            return "체커보드 Stereo 외부 파라미터"
        return "Pixhawk 자세 + GPS/RTK baseline"

    def _triangulation_extrinsics_source(self) -> str:
        return self._distance_mode()

    def _measurement_path_label(self) -> str:
        if self.active_intrinsics is None:
            return "3D ray fallback: 내부 파라미터 대기 중"
        selected = str(getattr(self, "_measurement_selected_method_id", "") or "")
        if selected in MEASUREMENT_METHOD_LABELS:
            return f"3D ray 거리: {MEASUREMENT_METHOD_LABELS[selected]} 사용"
        if self._distance_mode() == "checkerboard":
            if self._checkerboard_translation_right_in_left_m() is None:
                return "3D ray fallback: 체커보드 stereo 외부 파라미터 대기 중"
            return "3D ray 거리: 체커보드 Stereo 외부 파라미터 사용"
        left_ext = self.latest_extrinsics.get("left")
        right_ext = self.latest_extrinsics.get("right")
        if left_ext is None or right_ext is None:
            return "3D ray fallback: Pixhawk 자세 대기 중"
        if self._latest_translation_right_in_left_m_for_diagnostics() is None:
            return "3D ray fallback: GPS/RTK baseline 대기 중"
        return "3D ray 거리: Pixhawk 자세 + GPS/RTK baseline 사용"

    def _triangulation_extrinsics_source_label(self) -> str:
        selected = str(getattr(self, "_measurement_selected_method_id", "") or "")
        if selected in MEASUREMENT_METHOD_LABELS:
            return str(MEASUREMENT_METHOD_LABELS[selected])
        if self._distance_mode() == "checkerboard":
            return "체커보드 Stereo 외부 파라미터"
        return "Pixhawk 자세 + GPS/RTK baseline"

    def _distance_parameters_summary(self) -> str:
        path = self._measurement_path_label()
        focal = self._focal_source_label()
        baseline_m, baseline_source, _warnings = self._active_baseline_m()
        geometry = f"baseline={self._baseline_source_label(baseline_source, baseline_m)}"
        external = f"장착={self._mount_assumption_status_label()}"
        return (
            f"계산 파라미터: {path} | 초점거리={focal} | "
            f"위치/자세 출처={self._triangulation_extrinsics_source_label()} | {geometry} | {external}"
        )

    def _refresh_distance_parameters_label(self) -> None:
        if hasattr(self, "distance_parameters_label"):
            self.distance_parameters_label.setText(self._distance_parameters_summary())

    def _format_intrinsics_text(
        self,
        camera: str,
        intrinsics,
        status: str,
        source_label: str | None = None,
    ) -> str:
        title = "Left Camera Intrinsics" if camera == "left" else "Right Camera Intrinsics"
        resolved_source = source_label or self._intrinsics_source_label()
        if intrinsics is None:
            return (
                f"{title}\n"
                f"status: not loaded\n"
                f"source: {resolved_source}\n\n"
                "먼저 내부 파라미터 보정 결과를 불러오거나 실행하세요."
            )
        return (
            f"{title}\n"
            f"status: {status}\n"
            f"source: {resolved_source}\n"
            f"image_size: {intrinsics.width} x {intrinsics.height}\n"
            f"valid_images: {intrinsics.image_count}\n"
            f"reprojection_error: {intrinsics.reprojection_error_px:.6f} px\n\n"
            f"camera_matrix K:\n{self._format_matrix(intrinsics.camera_matrix)}\n\n"
            f"fx: {intrinsics.camera_matrix[0][0]:.6f}\n"
            f"fy: {intrinsics.camera_matrix[1][1]:.6f}\n"
            f"cx: {intrinsics.camera_matrix[0][2]:.6f}\n"
            f"cy: {intrinsics.camera_matrix[1][2]:.6f}\n\n"
            "distortion_coefficients:\n"
            + ", ".join(f"{value:.9g}" for value in intrinsics.distortion_coefficients)
        )

    def _current_zoom_intrinsics_result(self) -> tuple[StereoIntrinsicsResult | None, str, str, Path | None]:
        zoom_level = self.zoom_level_spin.value()
        if self.last_calibration_result is not None and self.last_calibration_zoom_level == zoom_level:
            active_path_text = str(getattr(self, "_active_intrinsics_profile_path", "") or "")
            active_path = (
                Path(active_path_text)
                if active_path_text and not active_path_text.startswith("memory:")
                else None
            )
            active_name = active_path.name if active_path is not None else "in-memory result"
            return (
                self.last_calibration_result,
                f"active calibration result (zoom {zoom_level:02d})",
                f"active calibration result for zoom {zoom_level:02d} ({active_name})",
                active_path,
            )
        zoom_path = self._intrinsics_storage_path(zoom_level)
        if zoom_path.exists():
            try:
                return (
                    load_intrinsics_json(zoom_path),
                    f"saved zoom calibration ({zoom_path.name})",
                    f"zoom {zoom_level:02d} file",
                    zoom_path,
                )
            except Exception as exc:
                return None, f"failed to load {zoom_path.name}: {exc}", f"zoom {zoom_level:02d} file", zoom_path
        latest_path = self._intrinsics_storage_base_path()
        if latest_path.exists():
            try:
                latest_zoom_level = self._intrinsics_file_zoom_level(latest_path)
                latest_zoom_label = (
                    f", zoom {latest_zoom_level:02d}" if latest_zoom_level is not None else ""
                )
                return (
                    load_intrinsics_json(latest_path),
                    f"latest alias fallback ({latest_path.name}{latest_zoom_label})",
                    f"latest alias fallback ({latest_path.name}{latest_zoom_label})",
                    latest_path,
                )
            except Exception as exc:
                return None, f"failed to load {latest_path.name}: {exc}", f"latest alias fallback ({latest_path.name})", latest_path
        return None, f"not loaded for zoom {zoom_level:02d}", f"zoom {zoom_level:02d}", None

    def _configured_intrinsics_profile_path(self) -> Path | None:
        raw_path = str(
            self._workstation_setting("intrinsics_path", self.settings.intrinsics_path) or ""
        )
        if not raw_path.strip():
            return None
        return resolve_settings_path(raw_path, base_dir=self.app_root)

    def _load_intrinsics_profile_for_exact_zoom(
        self,
        path: Path,
        zoom_level: int,
    ) -> tuple[StereoIntrinsicsResult | None, str]:
        if not path.is_file():
            return None, f"intrinsic profile is not a regular file: {path}"
        try:
            result = load_intrinsics_json(path)
        except Exception as exc:
            return None, f"invalid intrinsic profile: {path}: {exc}"
        metadata_zoom_level = self._intrinsics_file_zoom_level(path)
        if metadata_zoom_level is None:
            return None, f"intrinsic profile has no valid metadata.zoom_level: {path}"
        if metadata_zoom_level != int(zoom_level):
            return (
                None,
                "intrinsic profile zoom mismatch: "
                f"configured={metadata_zoom_level:02d}, operating={int(zoom_level):02d}, path={path}",
            )
        return result, ""

    def _auto_load_intrinsics_candidate(
        self,
        zoom_level: int,
    ) -> tuple[StereoIntrinsicsResult | None, Path | None, str]:
        """Resolve the exact persisted profile without silently changing operator intent.

        An existing configured profile is authoritative.  If it is malformed or belongs to a
        different zoom, automatic loading fails closed instead of selecting another file.  The
        generated per-zoom/base storage paths are compatibility fallbacks only when the missing
        configured path is the calibration root's base alias.  A missing explicit external path
        never falls through to unrelated storage bytes.
        """

        configured_path = self._configured_intrinsics_profile_path()
        zoom_path = self._intrinsics_storage_path(zoom_level)
        base_path = self._intrinsics_storage_base_path()
        if configured_path is not None and configured_path.exists():
            result, error = self._load_intrinsics_profile_for_exact_zoom(
                configured_path,
                zoom_level,
            )
            return result, configured_path, error

        if configured_path is not None and configured_path != base_path:
            return (
                None,
                configured_path,
                f"configured intrinsic profile is missing: {configured_path}; "
                "automatic storage fallback is disabled for an explicit profile path",
            )

        candidates: list[Path] = []
        for candidate in (zoom_path, base_path):
            if candidate not in candidates:
                candidates.append(candidate)
        for candidate in candidates:
            if not candidate.exists():
                continue
            result, error = self._load_intrinsics_profile_for_exact_zoom(candidate, zoom_level)
            return result, candidate, error

        if configured_path is not None:
            return (
                None,
                configured_path,
                f"configured intrinsic profile is missing: {configured_path}; "
                f"no zoom {int(zoom_level):02d} storage profile exists at {zoom_path}",
            )
        return (
            None,
            zoom_path,
            f"no configured intrinsic profile and no zoom {int(zoom_level):02d} "
            f"storage profile exists at {zoom_path}",
        )

    def _load_current_zoom_intrinsics_for_apply(self) -> StereoIntrinsicsResult | None:
        zoom_level = self.zoom_level_spin.value()
        zoom_path = self._intrinsics_storage_path(zoom_level)
        if zoom_path.exists():
            return load_intrinsics_json(zoom_path)
        if self.last_calibration_result is not None and self.last_calibration_zoom_level == zoom_level:
            return self.last_calibration_result
        latest_path = self._intrinsics_storage_base_path()
        if latest_path.exists() and self._intrinsics_file_zoom_level(latest_path) == zoom_level:
            return load_intrinsics_json(latest_path)
        return None

    def _intrinsics_file_zoom_level(self, path: Path) -> int | None:
        try:
            payload = json.loads(Path(path).read_text())
        except Exception:
            return None
        metadata = payload.get("metadata") or {}
        try:
            zoom_level = metadata.get("zoom_level")
            return None if zoom_level is None else int(zoom_level)
        except (TypeError, ValueError):
            return None

    def _intrinsics_file_metadata(self, path: Path) -> dict:
        try:
            payload = json.loads(Path(path).read_text())
        except Exception:
            return {}
        metadata = payload.get("metadata") or {}
        return metadata if isinstance(metadata, dict) else {}

    def _format_zoom_intrinsics_comparison_text(self, path: Path, result: StereoIntrinsicsResult) -> str:
        metadata = self._intrinsics_file_metadata(path)
        zoom_value = metadata.get("zoom_level")
        zoom_text = "--" if zoom_value is None else f"{int(zoom_value):02d}"
        timestamp_text = ""
        try:
            timestamp_s = float(metadata.get("timestamp_s"))
            timestamp_text = datetime.fromtimestamp(timestamp_s).strftime("%Y-%m-%d %H:%M:%S")
        except (TypeError, ValueError, OSError):
            timestamp_text = "unknown time"
        left = result.left
        right = result.right
        lines = [
            f"{path.name} | zoom {zoom_text} | {timestamp_text}",
            str(path),
            (
                "Left  "
                f"{left.width}x{left.height} fx={left.camera_matrix[0][0]:.3f} "
                f"fy={left.camera_matrix[1][1]:.3f} cx={left.camera_matrix[0][2]:.3f} "
                f"cy={left.camera_matrix[1][2]:.3f} err={left.reprojection_error_px:.4f}px "
                f"n={left.image_count}"
            ),
            (
                "Right "
                f"{right.width}x{right.height} fx={right.camera_matrix[0][0]:.3f} "
                f"fy={right.camera_matrix[1][1]:.3f} cx={right.camera_matrix[0][2]:.3f} "
                f"cy={right.camera_matrix[1][2]:.3f} err={right.reprojection_error_px:.4f}px "
                f"n={right.image_count}"
            ),
        ]
        return "\n".join(lines)

    def add_zoom_intrinsics_comparison_files(self) -> None:
        start_path = self._intrinsics_storage_base_path().parent
        selected, _filter = QtWidgets.QFileDialog.getOpenFileNames(
            self,
            "Add intrinsics JSON files",
            str(start_path),
            "Intrinsics JSON (*.json);;All files (*)",
        )
        added = 0
        for raw_path in selected:
            if self._add_zoom_intrinsics_comparison_path(Path(raw_path)):
                added += 1
        if added:
            self.calibration_status_label.setText(f"비교 목록에 intrinsic 파일 {added}개를 추가했습니다.")

    def remove_selected_zoom_intrinsics_comparison_files(self) -> None:
        if not hasattr(self, "zoom_intrinsics_compare_list"):
            return
        rows = sorted({index.row() for index in self.zoom_intrinsics_compare_list.selectedIndexes()}, reverse=True)
        for row in rows:
            item = self.zoom_intrinsics_compare_list.takeItem(row)
            if item is None:
                continue
            raw_path = item.data(QtCore.Qt.UserRole)
            try:
                self.zoom_intrinsics_compare_paths.remove(Path(str(raw_path)))
            except ValueError:
                pass
        if rows:
            self.calibration_status_label.setText(f"비교 목록에서 intrinsic 파일 {len(rows)}개를 제거했습니다.")

    def apply_selected_zoom_intrinsics_comparison_file(self) -> None:
        if not hasattr(self, "zoom_intrinsics_compare_list"):
            return
        item = self.zoom_intrinsics_compare_list.currentItem()
        if item is None:
            selected = self.zoom_intrinsics_compare_list.selectedItems()
            item = selected[0] if selected else None
        if item is None:
            self.calibration_status_label.setText("적용할 비교 목록 파일을 선택하세요.")
            return
        path = Path(str(item.data(QtCore.Qt.UserRole)))
        try:
            result = load_intrinsics_json(path)
        except Exception as exc:
            self.calibration_status_label.setText(f"비교 파일 적용 실패: {path} ({exc})")
            return
        zoom_level = self._intrinsics_file_zoom_level(path)
        self.last_calibration_result = result
        self.calibration_image_diagnostics = tuple(result.image_diagnostics)
        self.last_calibration_zoom_level = zoom_level
        self._active_intrinsics_profile_path = str(path)
        if self.apply_intrinsics_to_distance() is False:
            self._refresh_zoom_intrinsics_tab()
            self._refresh_operation_header()
            return
        if zoom_level is not None:
            self.zoom_level_spin.setValue(int(zoom_level))
            zoom_text = f"zoom {int(zoom_level):02d}"
        else:
            zoom_text = "zoom unknown"
        self.calibration_status_label.setText(f"Applied comparison intrinsics ({zoom_text}): {path}")
        self._log(f"[캘리브레이션] 비교 목록 intrinsic 파일을 적용했습니다 ({zoom_text}): {path}")
        self._refresh_zoom_intrinsics_tab()
        self._refresh_operation_header()

    def _add_zoom_intrinsics_comparison_path(self, path: Path) -> bool:
        if not hasattr(self, "zoom_intrinsics_compare_list"):
            return False
        resolved = path.expanduser()
        try:
            resolved = resolved.resolve()
        except OSError:
            pass
        if any(existing == resolved for existing in self.zoom_intrinsics_compare_paths):
            return False
        try:
            result = load_intrinsics_json(resolved)
        except Exception as exc:
            self.calibration_status_label.setText(f"비교 파일 로드 실패: {resolved} ({exc})")
            return False
        item = QtWidgets.QListWidgetItem(self._format_zoom_intrinsics_comparison_text(resolved, result))
        item.setData(QtCore.Qt.UserRole, str(resolved))
        item.setSizeHint(QtCore.QSize(0, 118))
        item.setFlags(item.flags() | QtCore.Qt.ItemIsDragEnabled | QtCore.Qt.ItemIsDropEnabled)
        self.zoom_intrinsics_compare_list.addItem(item)
        self.zoom_intrinsics_compare_paths.append(resolved)
        self.zoom_intrinsics_compare_list.clearSelection()
        self.zoom_intrinsics_compare_list.setCurrentItem(item)
        return True

    def _refresh_zoom_intrinsics_tab(self) -> None:
        if not hasattr(self, "zoom_left_intrinsics_text"):
            return
        result, status, source_label, path = self._current_zoom_intrinsics_result()
        zoom_level = self.zoom_level_spin.value()
        self.zoom_intrinsics_status_label.setText(f"줌 {zoom_level:02d} | 상태: {status}")
        path_text = f"파일: {path}" if path is not None else f"파일: {self._intrinsics_storage_path(zoom_level)}"
        self.zoom_intrinsics_path_label.setText(path_text)
        self.zoom_left_intrinsics_text.setPlainText(
            self._format_intrinsics_text("left", None if result is None else result.left, status, source_label)
        )
        self.zoom_right_intrinsics_text.setPlainText(
            self._format_intrinsics_text("right", None if result is None else result.right, status, source_label)
        )

    def _format_live_pixhawk_extrinsic_text(
        self,
        camera: str,
        extrinsic: CameraExtrinsicState | None,
        *,
        title_prefix: str = "",
    ) -> str:
        title = f"{title_prefix}{'Left Live Extrinsic' if camera == 'left' else 'Right Live Extrinsic'}"
        if extrinsic is None:
            return (
                f"{title}\n"
                "status: waiting\n\n"
                f"source: {self._extrinsics_source_label(None)}\n\n"
                "실시간 카메라 자세를 보려면 MAVLink로 픽스호크를 연결하세요."
            )
        age_ms = max(0.0, time.monotonic() - extrinsic.timestamp_s) * 1000.0
        px4_timestamp = "n/a" if extrinsic.px4_timestamp_s is None else f"{extrinsic.px4_timestamp_s:.6f}"
        return (
            f"{title}\n"
            f"status: live\n"
            f"source: {self._extrinsics_source_label(extrinsic)}\n"
            f"age: {age_ms:.0f} ms\n"
            f"px4_timestamp: {px4_timestamp}\n\n"
            f"roll:  {extrinsic.roll_deg:.6f} deg\n"
            f"pitch: {extrinsic.pitch_deg:.6f} deg\n"
            f"yaw:   {extrinsic.yaw_deg:.6f} deg\n\n"
            "quaternion q(w,x,y,z):\n"
            + ", ".join(f"{value:.9g}" for value in extrinsic.q)
        )

    def _format_checkerboard_effective_extrinsics_text(
        self,
        camera: str,
        live_extrinsic: CameraExtrinsicState | None,
    ) -> str:
        result = self.active_intrinsics
        extrinsics = None if result is None else result.extrinsics
        title = "Left Effective Extrinsic" if camera == "left" else "Right Effective Extrinsic"
        live_source = self._extrinsics_source_label(live_extrinsic)
        diagnostic = (
            "\n\nLive Pixhawk diagnostic only\n"
            f"source: {live_source}\n"
            "used_for_triangulation: false"
        )
        if extrinsics is None:
            return (
                f"{title}\n"
                "mode: checkerboard stereo R/T\n"
                "status: waiting for applied checkerboard stereo extrinsics\n"
                "used_for_triangulation: pending\n"
                f"frame_contract: {CAMERA_FRAME}\n"
                f"{diagnostic}"
            )
        rotation_right_to_left, translation_right_in_left_m = invert_left_to_right_camera_transform(
            extrinsics.rotation_left_to_right,
            extrinsics.translation_left_to_right_m,
        )
        if camera == "left":
            return (
                f"{title}\n"
                "mode: checkerboard stereo R/T\n"
                "status: effective reference camera\n"
                "source: applied checkerboard stereo extrinsics\n"
                "used_for_triangulation: true\n"
                f"frame_contract: {extrinsics.frame_contract}\n"
                "origin_in_left_rdf_m: 0.000000000, 0.000000000, 0.000000000\n"
                "rotation_left_to_left:\n"
                + self._format_matrix([[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]])
                + diagnostic
            )
        baseline_m = math.sqrt(sum(value * value for value in translation_right_in_left_m))
        return (
            f"{title}\n"
            "mode: checkerboard stereo R/T\n"
            "status: effective right camera in left RDF\n"
            "source: applied checkerboard stereo extrinsics\n"
            "used_for_triangulation: true\n"
            f"frame_contract: {extrinsics.frame_contract}\n"
            f"baseline_m: {baseline_m:.9f}\n"
            "translation_right_in_left_m: "
            + ", ".join(f"{value:.9f}" for value in translation_right_in_left_m)
            + "\n"
            "rotation_right_to_left:\n"
            + self._format_matrix(rotation_right_to_left)
            + diagnostic
        )

    def _format_extrinsics_text(self, camera: str, extrinsic: CameraExtrinsicState | None) -> str:
        if self._distance_mode() == "checkerboard":
            return self._format_checkerboard_effective_extrinsics_text(camera, extrinsic)
        return self._format_live_pixhawk_extrinsic_text(camera, extrinsic)

    def _format_stereo_geometry_text(self) -> str:
        active = self.active_intrinsics
        if active is not None:
            intrinsics_state = "active"
        elif self.last_calibration_result is not None:
            intrinsics_state = "latest loaded/calculated, not applied"
        else:
            intrinsics_state = "not loaded"
        baseline_m, baseline_source, warnings = self._active_baseline_m()
        translation = self._latest_translation_right_in_left_m_for_diagnostics()
        translation_text = "대기 중"
        if translation is not None:
            translation_text = f"{translation[0]:.6f}, {translation[1]:.6f}, {translation[2]:.6f} m"
        rtk = self.latest_rtk_baseline
        rtk_text = "대기 중"
        if rtk is not None:
            rtk_text = (
                f"{rtk.baseline_m:.6f} m, N/E/D={rtk.north_m:.6f}/"
                f"{rtk.east_m:.6f}/{rtk.down_m:.6f} m, quality={rtk.fix_quality}"
            )
        warning_text = "none" if not warnings else "; ".join(warnings)
        return (
            "Stereo Geometry\n"
            f"intrinsics: {intrinsics_state}\n"
            f"intrinsics_source: {self._intrinsics_source_label()}\n"
            f"distance_method: {self._measurement_path_label()}\n"
            f"distance_mode: {self._distance_mode_label()}\n"
            "use_3d_distance: true\n"
            f"pose_source: {self._triangulation_extrinsics_source_label()}\n"
            f"world_frame: {WORLD_FRAME}\n"
            f"body_frame: {BODY_FRAME}\n"
            f"camera_frame: {CAMERA_FRAME}\n"
            f"camera_mount_profile: {CAMERA_MOUNT_PROFILE}\n"
            "mount_assumption_model: gps-rtk-baseline-only-v1\n"
            f"mount_assumption_status: {self._mount_assumption_status_label()}\n"
            f"manual_focal_reference: {self.focal_spin.value():.6f} px\n"
            f"focal_source: {self._focal_source_label()}\n"
            f"baseline_used: {baseline_m:.6f} m\n"
            f"baseline_source: {baseline_source}\n"
            f"baseline_source_detail: {self._baseline_source_label(baseline_source, baseline_m)}\n"
            f"parameter_summary: {self._distance_parameters_summary()}\n"
            f"baseline_warnings: {warning_text}\n\n"
            f"rtk_baseline: {rtk_text}\n"
            f"translation_right_in_left: {translation_text}\n"
            f"{self._format_checkerboard_stereo_extrinsics_summary()}\n\n"
            f"{self._format_mount_assumption_summary()}\n\n"
            f"max_detection_dt: {self._yolo_pair_max_delta_s():.6f} s\n"
            f"max_vertical_delta: {self._yolo_vertical_max_delta_px():.6f} px\n"
            "same_class_required: true (stereo correspondence invariant)\n"
            f"target_class_gate_enabled: {self.yolo_gate_class_checkbox.isChecked()}\n"
            f"target_class_gate: {self.yolo_gate_class_edit.text().strip() or 'none'}\n"
            "attitude_gate: false\n"
            f"attitude_timing_policy: {ATTITUDE_TIMING_POLICY}\n"
            f"attitude_stale_age_threshold_s: {ATTITUDE_STALE_AGE_S:.3f}\n"
            f"attitude_very_stale_age_threshold_s: {ATTITUDE_VERY_STALE_AGE_S:.3f}\n"
            f"attitude_stale_pair_delta_threshold_s: {ATTITUDE_STALE_PAIR_DELTA_S:.3f}\n"
            f"attitude_very_stale_pair_delta_threshold_s: {ATTITUDE_VERY_STALE_PAIR_DELTA_S:.3f}"
        )

    def _format_checkerboard_stereo_extrinsics_summary(self) -> str:
        result = self.active_intrinsics or self.last_calibration_result
        extrinsics = None if result is None else result.extrinsics
        if extrinsics is None:
            return "checkerboard_stereo_extrinsics: not computed"
        baseline_m = math.sqrt(sum(value * value for value in extrinsics.translation_left_to_right_m))
        rotation_rows = [
            "  [" + ", ".join(f"{value: .9f}" for value in row) + "]"
            for row in extrinsics.rotation_left_to_right
        ]
        return (
            "checkerboard_stereo_extrinsics: computed\n"
            "checkerboard_stereo_usage: selectable live triangulation source in parameter calculation settings\n"
            f"checkerboard_frame_contract: {extrinsics.frame_contract}\n"
            f"checkerboard_pair_count: {extrinsics.pair_count}\n"
            f"checkerboard_reprojection_error_px: {extrinsics.reprojection_error_px:.6f}\n"
            f"checkerboard_translation_left_to_right_m: "
            + ", ".join(f"{value:.9f}" for value in extrinsics.translation_left_to_right_m)
            + "\n"
            f"checkerboard_baseline_m: {baseline_m:.9f}\n"
            "checkerboard_rotation_left_to_right:\n"
            + "\n".join(rotation_rows)
        )

    def _refresh_camera_parameters_tab(self) -> None:
        if not hasattr(self, "left_intrinsics_text"):
            return
        self._update_distance_settings_visibility()
        result = self.active_intrinsics or self.last_calibration_result
        status = "active" if self.active_intrinsics is not None else "latest loaded/calculated, not applied"
        self.left_intrinsics_text.setPlainText(self._format_intrinsics_text("left", None if result is None else result.left, status))
        self.right_intrinsics_text.setPlainText(self._format_intrinsics_text("right", None if result is None else result.right, status))
        self.left_extrinsics_text.setPlainText(self._format_extrinsics_text("left", self.latest_extrinsics.get("left")))
        self.right_extrinsics_text.setPlainText(self._format_extrinsics_text("right", self.latest_extrinsics.get("right")))
        self.stereo_geometry_text.setPlainText(self._format_stereo_geometry_text())
        self._refresh_distance_parameters_label()
        self._refresh_zoom_intrinsics_tab()

    def _update_calibration_preview(self, camera: str, image: QtGui.QImage, metadata: object | None = None) -> None:
        now_s = time.monotonic()
        if isinstance(metadata, dict):
            frame_metadata = dict(metadata)
        elif metadata is None:
            frame_metadata = {}
        else:
            frame_metadata = {"timestamp_s": float(metadata)}
        raw_frame_timestamp_s = frame_metadata.get("timestamp_s", now_s)
        if raw_frame_timestamp_s is None:
            raw_frame_timestamp_s = now_s
        frame_timestamp_s = float(raw_frame_timestamp_s)
        frame_metadata.setdefault("timestamp_s", frame_timestamp_s)
        frame_metadata.setdefault("source_timestamp_s", None)
        frame_metadata.setdefault("timestamp_source", "app-receive")
        frame_metadata.setdefault("timestamp_reference", infer_runtime_camera_timestamp_reference(frame_metadata))
        self._record_gui_camera_frame(camera, image, frame_metadata)
        if not self.yolo_enabled:
            self.left_pane.clear_measurement_pair_display()
            self.right_pane.clear_measurement_pair_display()
            self._clear_selected_runtime_measurement_pair(update_labels=False)
        self.camera_frame_metadata[camera] = frame_metadata
        self.camera_frame_metadata_history[camera].append(dict(frame_metadata))
        self.camera_frame_times[camera].append(frame_timestamp_s)
        self._capture_recording_clock_sync_start_observation()
        self._update_camera_connection_status()
        self._record_clock_offset_sample(now_s)
        if self._is_clock_sync_tab_visible():
            self._update_clock_sync_display()
        self.latest_calibration_preview_images[camera] = image.copy()
        self.calibration_preview_changed.emit(
            {
                "camera": camera,
                "image": image.copy(),
                "kind": "raw",
                "metadata": dict(frame_metadata),
            }
        )
        if not (
            self._is_checkerboard_calibration_visible()
            or self._workstation_calibration_preview_active
        ):
            return
        if now_s - self.calibration_preview_times[camera] < 0.5:
            return
        self.calibration_preview_times[camera] = now_s
        self._refresh_calibration_preview(camera)

    def _recent_detection_observations(self, camera: str, *, now_s: float | None = None) -> list[DetectionObservation]:
        if not self.yolo_enabled and not self.camera_connected:
            return list(self.detection_buffers[camera])
        now_s = time.monotonic() if now_s is None else now_s
        return [
            detection
            for detection in self.detection_buffers[camera]
            if 0.0 <= now_s - float(detection.timestamp_s) <= DETECTION_PAIR_MAX_AGE_S
        ]

    def _matched_detection_pair(self):
        now_s = time.monotonic()
        if self._camera_sync_gate_active() and self.camera_sync_status != CAMERA_SYNC_STATUS_VERIFIED:
            self._runtime_pair_uses_app_fallback = False
            self._last_correspondence_rejection_reason = (
                f"camera_time_sync_{self.camera_sync_status}"
            )
            return None
        left_samples = self._recent_detection_observations("left", now_s=now_s)
        right_samples = self._recent_detection_observations("right", now_s=now_s)
        self._runtime_pair_uses_app_fallback = False
        pair = match_nearest_pair(
            left_samples,
            right_samples,
            max_delta_s=self._yolo_pair_max_delta_s(),
            timestamp=self._yolo_match_timestamp_s,
        )
        if pair is not None or self._yolo_time_basis() != "camera_rtcp":
            return self._validated_correspondence_pair(pair, left_samples, right_samples)

        # The WIRIS units can emit valid RTCP/NTP fields while their two
        # camera clocks drift apart. Keep RTCP as the preferred diagnostic
        # basis, but use the common app receive monotonic clock for live
        # pairing so a long-running session does not stop producing pairs.
        pair = match_nearest_pair(
            left_samples,
            right_samples,
            max_delta_s=self._yolo_pair_max_delta_s(),
            timestamp=lambda sample: float(sample.timestamp_s),
        )
        self._runtime_pair_uses_app_fallback = pair is not None
        return self._validated_correspondence_pair(pair, left_samples, right_samples)

    def _validated_correspondence_pair(self, pair, left_samples, right_samples):
        if pair is None:
            if not left_samples:
                self._last_correspondence_rejection_reason = "missing_left_detection"
            elif not right_samples:
                self._last_correspondence_rejection_reason = "missing_right_detection"
            else:
                self._last_correspondence_rejection_reason = ""
            return None
        reason = correspondence_rejection_reason(pair.left, pair.right)
        self._last_correspondence_rejection_reason = "" if reason is None else reason
        return pair if reason is None else None

    def _recent_detection_pair_available(self) -> bool:
        pair = self._matched_detection_pair()
        if pair is None:
            return False
        return True

    def _refresh_calibration_preview(self, camera: str) -> None:
        image = self.latest_calibration_preview_images.get(camera)
        if image is None:
            return
        if camera == "left":
            label = self.calibration_left_preview
        elif camera == "right":
            label = self.calibration_right_preview
        else:
            return
        waiting = self._calibration_overlay_image(
            image,
            found=False,
            corners=[],
            status="checkerboard: detecting...",
        )
        self._set_calibration_preview_pixmap(camera, label, waiting)
        self._queue_calibration_preview_detection(camera, image)

    def _refresh_all_calibration_previews(self, *_args) -> None:
        self._refresh_calibration_preview("left")
        self._refresh_calibration_preview("right")

    def _handle_tab_changed(self, index: int) -> None:
        if index == self.parameter_tab_index:
            self._refresh_camera_parameters_tab()
        if index == self.calibration_tab_index and self._is_checkerboard_calibration_visible():
            self.calibration_preview_times["left"] = 0.0
            self.calibration_preview_times["right"] = 0.0
            self._refresh_all_calibration_previews()
        if hasattr(self, "triangulation_quality_plot"):
            self.triangulation_quality_plot.set_tab_visible(self._is_triangulation_quality_visible())
        if index == self.time_tab_index:
            self._update_clock_sync_display()
        if index == self.results_tab_index:
            self._refresh_active_recording_sheet()

    def _is_triangulation_quality_visible(self) -> bool:
        return (
            self.tabs.currentIndex() == self.depth_tab_index
            and hasattr(self, "measurement_tabs")
            and self.measurement_tabs.currentIndex() == self.triangulation_quality_tab_index
        )

    def _is_checkerboard_calibration_visible(self) -> bool:
        return (
            self.tabs.currentIndex() == self.calibration_tab_index
            and hasattr(self, "calibration_tabs")
            and self.calibration_tabs.currentIndex() == self.checkerboard_calibration_tab_index
        )

    def _calibration_overlay_image(
        self,
        image: QtGui.QImage,
        *,
        found: bool,
        corners: list[tuple[float, float]],
        status: str,
    ) -> QtGui.QImage:
        annotated = image.convertToFormat(QtGui.QImage.Format_RGB888).copy()
        painter = QtGui.QPainter(annotated)
        painter.setRenderHint(QtGui.QPainter.Antialiasing)
        if found and corners:
            pen = QtGui.QPen(QtGui.QColor(SUCCESS), 3)
            painter.setPen(pen)
            painter.setBrush(QtGui.QBrush(QtGui.QColor(SUCCESS)))
            columns, rows = self._checkerboard_config().pattern_size
            if len(corners) == columns * rows:
                for row in range(rows):
                    start = row * columns
                    for column in range(columns - 1):
                        painter.drawLine(
                            QtCore.QPointF(*corners[start + column]),
                            QtCore.QPointF(*corners[start + column + 1]),
                        )
                for column in range(columns):
                    for row in range(rows - 1):
                        painter.drawLine(
                            QtCore.QPointF(*corners[row * columns + column]),
                            QtCore.QPointF(*corners[(row + 1) * columns + column]),
                        )
            for x, y in corners:
                painter.drawEllipse(QtCore.QPointF(x, y), 5.0, 5.0)
        else:
            painter.setPen(QtGui.QPen(QtGui.QColor(ERROR), 4))
            painter.drawRect(2, 2, max(0, annotated.width() - 4), max(0, annotated.height() - 4))
        painter.fillRect(0, 0, annotated.width(), 28, QtGui.QColor(0, 0, 0, 170))
        painter.setPen(QtGui.QPen(QtGui.QColor(230, 230, 230), 1))
        painter.drawText(8, 20, status)
        painter.end()
        return annotated

    def _queue_calibration_preview_detection(self, camera: str, image: QtGui.QImage) -> int:
        self._ensure_calibration_preview_worker()
        self._calibration_preview_next_request_id += 1
        request_id = self._calibration_preview_next_request_id
        self.calibration_preview_request_ids[camera] = request_id
        self.calibration_preview_requested.emit(
            {
                "camera": camera,
                "request_id": request_id,
                "image": image.convertToFormat(QtGui.QImage.Format_RGB888).copy(),
                "pattern_size": self._checkerboard_config().pattern_size,
                "metadata": dict(self.camera_frame_metadata.get(camera) or {}),
            }
        )
        return request_id

    def _ensure_calibration_preview_worker(self) -> None:
        thread = getattr(self, "_calibration_preview_thread", None)
        worker = getattr(self, "_calibration_preview_worker", None)
        if thread is not None and worker is not None and thread.isRunning():
            return
        if thread is not None:
            self._stop_calibration_preview_worker()

        worker = CheckerboardPreviewWorker()
        thread = QtCore.QThread(self)
        worker.moveToThread(thread)
        self.calibration_preview_requested.connect(worker.submit)
        worker.result_ready.connect(self._handle_calibration_preview_result)
        thread.finished.connect(worker.deleteLater)
        self._calibration_preview_worker = worker
        self._calibration_preview_thread = thread
        thread.start()

    def _handle_calibration_preview_result(self, payload: object) -> None:
        if not isinstance(payload, dict):
            return
        camera = str(payload.get("camera") or "")
        if camera not in {"left", "right"}:
            return
        request_id = int(payload.get("request_id") or 0)
        if request_id != self.calibration_preview_request_ids.get(camera):
            return
        image = payload.get("image")
        if not isinstance(image, QtGui.QImage):
            return
        metadata = payload.get("metadata")
        self.calibration_preview_detection_results[camera] = {
            "request_id": request_id,
            "image": image.copy(),
            "found": bool(payload.get("found")),
            "corners": list(payload.get("corners") or []),
            "status": str(payload.get("status") or "checkerboard: not found"),
            "metadata": dict(metadata) if isinstance(metadata, dict) else {},
        }
        if not (
            self._is_checkerboard_calibration_visible()
            or self._workstation_calibration_preview_active
        ):
            return
        corners = payload.get("corners")
        overlay = self._calibration_overlay_image(
            image,
            found=bool(payload.get("found")),
            corners=list(corners or []),
            status=str(payload.get("status") or "checkerboard: not found"),
        )
        label = self.calibration_left_preview if camera == "left" else self.calibration_right_preview
        self._set_calibration_preview_pixmap(camera, label, overlay)

    def _set_calibration_preview_pixmap(
        self,
        camera: str,
        label: QtWidgets.QLabel,
        image: QtGui.QImage,
    ) -> None:
        self.calibration_preview_overlay_images[camera] = image.copy()
        pixmap = QtGui.QPixmap.fromImage(image)
        label.setPixmap(pixmap.scaled(label.size(), QtCore.Qt.KeepAspectRatio, QtCore.Qt.SmoothTransformation))
        self.calibration_preview_changed.emit(
            {
                "camera": camera,
                "image": image.copy(),
                "kind": "overlay",
            }
        )

    def _rescale_calibration_preview_pixmaps(self) -> None:
        for camera, label in (
            ("left", self.calibration_left_preview),
            ("right", self.calibration_right_preview),
        ):
            image = self.calibration_preview_overlay_images.get(camera)
            if image is None:
                continue
            pixmap = QtGui.QPixmap.fromImage(image)
            label.setPixmap(pixmap.scaled(label.size(), QtCore.Qt.KeepAspectRatio, QtCore.Qt.SmoothTransformation))

    def resizeEvent(self, event) -> None:  # noqa: N802 - Qt override.
        self._update_runtime_camera_settings_layout_mode(event.size().width())
        self._rescale_calibration_preview_pixmaps()
        super().resizeEvent(event)
















    def _distance_history_key(self, point: DistanceHistoryPoint) -> tuple[object, ...]:
        return point.sample_key or (point.timestamp_s, point.distance_m, point.method)

    def _recording_controller(self):
        owner = getattr(self, "_recording_controller_owner", None)
        if owner is None:
            owner = RecordingController(self)
            self._recording_controller_owner = owner
        return owner

    def _run_evidence_controller(self):
        owner = getattr(self, "_run_evidence_controller_owner", None)
        if owner is None:
            owner = RunEvidenceController(self)
            self._run_evidence_controller_owner = owner
        return owner

    def _optical_pose_controller(self):
        owner = getattr(self, "_optical_pose_controller_owner", None)
        if owner is None:
            owner = OpticalPoseController(self)
            self._optical_pose_controller_owner = owner
        return owner

    def _target_drone_controller(self):
        owner = getattr(self, "_target_drone_controller_owner", None)
        if owner is None:
            owner = TargetDroneController(self)
            self._target_drone_controller_owner = owner
        return owner

    def _drone_telemetry_controller(self):
        owner = getattr(self, "_drone_telemetry_controller_owner", None)
        if owner is None:
            owner = DroneTelemetryController(self)
            self._drone_telemetry_controller_owner = owner
        return owner

    def _calibration_workflow_controller(self):
        owner = getattr(self, "_calibration_workflow_controller_owner", None)
        if owner is None:
            owner = CalibrationWorkflowController(self)
            self._calibration_workflow_controller_owner = owner
        return owner

    def _device_runtime_controller(self):
        owner = getattr(self, "_device_runtime_controller_owner", None)
        if owner is None:
            owner = DeviceRuntimeController(self)
            self._device_runtime_controller_owner = owner
        return owner

    def _gnss_runtime_controller(self):
        owner = getattr(self, "_gnss_runtime_controller_owner", None)
        if owner is None:
            owner = GnssRuntimeController(self)
            self._gnss_runtime_controller_owner = owner
        return owner

    def _measurement_view_controller(self):
        owner = getattr(self, "_measurement_view_controller_owner", None)
        if owner is None:
            owner = MeasurementViewController(self)
            self._measurement_view_controller_owner = owner
        return owner

    def _sensor_presentation_controller(self):
        owner = getattr(self, "_sensor_presentation_controller_owner", None)
        if owner is None:
            owner = SensorPresentationController(self)
            self._sensor_presentation_controller_owner = owner
        return owner

    def _camera_stream_lifecycle_controller(self):
        owner = getattr(self, "_camera_stream_lifecycle_controller_owner", None)
        if owner is None:
            owner = CameraStreamLifecycleController(self)
            self._camera_stream_lifecycle_controller_owner = owner
        return owner

    def _measurement_package_controller(self):
        owner = getattr(self, "_measurement_package_controller_owner", None)
        if owner is None:
            owner = MeasurementPackageController(self)
            self._measurement_package_controller_owner = owner
        return owner

    def _wiris_control_controller(self):
        owner = getattr(self, "_wiris_control_controller_owner", None)
        if owner is None:
            owner = WirisControlController(self)
            self._wiris_control_controller_owner = owner
        return owner

    def _app_lifecycle_controller(self):
        owner = getattr(self, "_app_lifecycle_controller_owner", None)
        if owner is None:
            owner = AppLifecycleController(self)
            self._app_lifecycle_controller_owner = owner
        return owner

    def _workstation_settings_controller(self):
        owner = getattr(self, "_workstation_settings_controller_owner", None)
        if owner is None:
            owner = WorkstationSettingsController(self)
            self._workstation_settings_controller_owner = owner
        return owner

    # BEGIN GENERATED PANEL DELEGATE STUBS - see tools/gen_panel_delegate_stubs.py
    #
    # These declarations are inert at runtime (TYPE_CHECKING is False when the
    # module is imported); they exist only so IDEs/type checkers can see the
    # ~410 methods that `_CONTROLLER_DELEGATE_BINDINGS` attaches via setattr,
    # which are otherwise invisible to static analysis. Do not hand-edit -
    # regenerate with `python3 tools/gen_panel_delegate_stubs.py` after
    # changing any `*_METHODS` registry.
    if TYPE_CHECKING:
        def _set_camera_sync_state(self, *args: Any, **kwargs: Any) -> Any: ...
        def _camera_sync_widget_value(self, *args: Any, **kwargs: Any) -> Any: ...
        def _camera_sync_history_window_s(self, *args: Any, **kwargs: Any) -> Any: ...
        def _camera_sync_receive_pair_max_delta_s(self, *args: Any, **kwargs: Any) -> Any: ...
        def _camera_sync_rtcp_pair_max_delta_s(self, *args: Any, **kwargs: Any) -> Any: ...
        def _camera_sync_max_sr_age_s(self, *args: Any, **kwargs: Any) -> Any: ...
        def _camera_sync_max_pc_offset_s(self, *args: Any, **kwargs: Any) -> Any: ...
        def _camera_sync_max_clock_rate_error(self, *args: Any, **kwargs: Any) -> Any: ...
        def _camera_sync_required_sr_count(self, *args: Any, **kwargs: Any) -> Any: ...
        def _camera_sync_warmup_timeout_s(self, *args: Any, **kwargs: Any) -> Any: ...
        def _camera_sync_readiness(self, *args: Any, **kwargs: Any) -> Any: ...
        def _poll_camera_sync_state(self, *args: Any, **kwargs: Any) -> Any: ...
        def _begin_camera_sync_after_stus(self, *args: Any, **kwargs: Any) -> Any: ...
        def _clock_sync_side_summary(self, *args: Any, **kwargs: Any) -> Any: ...
        def _rtcp_sender_report_count(self, *args: Any, **kwargs: Any) -> Any: ...
        def _clock_sync_pair_metadata(self, *args: Any, **kwargs: Any) -> Any: ...
        def _camera_time_sync_metrics(self, *args: Any, **kwargs: Any) -> Any: ...
        def _camera_time_sync_snapshot(self, *args: Any, **kwargs: Any) -> Any: ...
        def _capture_recording_clock_sync_start_observation(self, *args: Any, **kwargs: Any) -> Any: ...
        def _camera_time_sync_drift_metadata(self, *args: Any, **kwargs: Any) -> Any: ...
        def _format_clock_sync_status(self, *args: Any, **kwargs: Any) -> Any: ...
        def _record_clock_offset_sample(self, *args: Any, **kwargs: Any) -> Any: ...
        def _update_clock_sync_display(self, *args: Any, **kwargs: Any) -> Any: ...
        def _is_clock_sync_tab_visible(self, *args: Any, **kwargs: Any) -> Any: ...
        def refresh_clock_sync_status(self, *args: Any, **kwargs: Any) -> Any: ...
        def _refresh_recording_status_label(self, *args: Any, **kwargs: Any) -> Any: ...
        def _recording_table_value(self, *args: Any, **kwargs: Any) -> Any: ...
        def _recording_preview_rows(self, *args: Any, **kwargs: Any) -> Any: ...
        def _recording_sheet_names(self, *args: Any, **kwargs: Any) -> Any: ...
        def _recording_sheet_kind(self, *args: Any, **kwargs: Any) -> Any: ...
        def _active_recording_sheet_name(self, *args: Any, **kwargs: Any) -> Any: ...
        def _append_recording_table_row(self, *args: Any, **kwargs: Any) -> Any: ...
        def _refresh_active_recording_sheet(self, *args: Any, **kwargs: Any) -> Any: ...
        def _fill_recording_sheet(self, *args: Any, **kwargs: Any) -> Any: ...
        def _refresh_recording_table(self, *args: Any, **kwargs: Any) -> Any: ...
        def _refresh_recording_sample_detail(self, *args: Any, **kwargs: Any) -> Any: ...
        def toggle_session_recording(self, *args: Any, **kwargs: Any) -> Any: ...
        def start_automatic_session_recording(self, *args: Any, **kwargs: Any) -> Any: ...
        def stop_automatic_session_recording(self, *args: Any, **kwargs: Any) -> Any: ...
        def _start_session_recording(self, *args: Any, **kwargs: Any) -> Any: ...
        def _recover_retryable_recording_session(self, *args: Any, **kwargs: Any) -> Any: ...
        def _stop_session_recording(self, *args: Any, **kwargs: Any) -> Any: ...
        def capture_pnp_hr_photos(self, *args: Any, **kwargs: Any) -> Any: ...
        def capture_pnp_oz_photos(self, *args: Any, **kwargs: Any) -> Any: ...
        def import_hr_files(self, *args: Any, **kwargs: Any) -> Any: ...
        def _camera_photo_target_infos(self, *args: Any, **kwargs: Any) -> Any: ...
        def _wiris_ftp_worker_kwargs(self, *args: Any, **kwargs: Any) -> Any: ...
        def _append_pnp_photo_capture_record(self, *args: Any, **kwargs: Any) -> Any: ...
        def _append_oz_photo_capture_record(self, *args: Any, **kwargs: Any) -> Any: ...
        def _handle_pnp_photo_capture_completed(self, *args: Any, **kwargs: Any) -> Any: ...
        def _handle_pnp_photo_capture_failed(self, *args: Any, **kwargs: Any) -> Any: ...
        def _clear_pnp_photo_capture_worker(self, *args: Any, **kwargs: Any) -> Any: ...
        def _handle_oz_photo_capture_completed(self, *args: Any, **kwargs: Any) -> Any: ...
        def _handle_oz_photo_capture_failed(self, *args: Any, **kwargs: Any) -> Any: ...
        def _clear_oz_photo_capture_worker(self, *args: Any, **kwargs: Any) -> Any: ...
        def toggle_distance_recording(self, *args: Any, **kwargs: Any) -> Any: ...
        def toggle_video_recording(self, *args: Any, **kwargs: Any) -> Any: ...
        def _pair_recording_base_row(self, *args: Any, **kwargs: Any) -> Any: ...
        def _upsert_pair_recording_row(self, *args: Any, **kwargs: Any) -> Any: ...
        def _record_unresolved_pair(self, *args: Any, **kwargs: Any) -> Any: ...
        def _record_blocked_measurement_pair(self, *args: Any, **kwargs: Any) -> Any: ...
        def _append_distance_history(self, *args: Any, **kwargs: Any) -> Any: ...
        def _append_extrinsics_comparison_snapshot(self, *args: Any, **kwargs: Any) -> Any: ...
        def _distance_history_class_name(self, *args: Any, **kwargs: Any) -> Any: ...
        def _fmt_record_float(self, *args: Any, **kwargs: Any) -> Any: ...
        def _fmt_record_bool(self, *args: Any, **kwargs: Any) -> Any: ...
        def _recording_elapsed_s(self, *args: Any, **kwargs: Any) -> Any: ...
        def _session_utc_ns_for_monotonic_ns(self, *args: Any, **kwargs: Any) -> Any: ...
        def _recording_side(self, *args: Any, **kwargs: Any) -> Any: ...
        def _raw_timeline_row(self, *args: Any, **kwargs: Any) -> Any: ...
        def _record_gui_camera_frame(self, *args: Any, **kwargs: Any) -> Any: ...
        def _record_external_timeline_row(self, *args: Any, **kwargs: Any) -> Any: ...
        def _record_bbox_parameters(self, *args: Any, **kwargs: Any) -> Any: ...
        def _detection_utc_unix_s(self, *args: Any, **kwargs: Any) -> Any: ...
        def _unix_seconds_to_ns(self, *args: Any, **kwargs: Any) -> Any: ...
        def _record_measurement_sync_timestamp_parameters(self, *args: Any, **kwargs: Any) -> Any: ...
        def _record_frame_timestamp_parameters(self, *args: Any, **kwargs: Any) -> Any: ...
        def _record_live_telemetry_snapshot_parameters(self, *args: Any, **kwargs: Any) -> Any: ...
        def _record_reference_delta_parameters(self, *args: Any, **kwargs: Any) -> Any: ...
        def _record_current_intrinsic_parameters(self, *args: Any, **kwargs: Any) -> Any: ...
        def _record_intrinsic_parameters(self, *args: Any, **kwargs: Any) -> Any: ...
        def _record_current_checkerboard_extrinsics_parameters(self, *args: Any, **kwargs: Any) -> Any: ...
        def _record_checkerboard_extrinsics_parameters(self, *args: Any, **kwargs: Any) -> Any: ...
        def _record_external_parameters(self, *args: Any, **kwargs: Any) -> Any: ...
        def _record_extrinsic_state(self, *args: Any, **kwargs: Any) -> Any: ...
        def _record_gps_state(self, *args: Any, **kwargs: Any) -> Any: ...
        def _record_gnss_relative_state(self, *args: Any, **kwargs: Any) -> Any: ...
        def save_distance_history_csv(self, *args: Any, **kwargs: Any) -> Any: ...
        def _handle_session_saved(self, *args: Any, **kwargs: Any) -> Any: ...
        def _handle_session_save_failed(self, *args: Any, **kwargs: Any) -> Any: ...
        def _handle_session_save_thread_finished(self, *args: Any, **kwargs: Any) -> Any: ...
        def _wait_for_session_save(self, *args: Any, **kwargs: Any) -> Any: ...
        def _record_px4_mag_status_rows(self, *args: Any, **kwargs: Any) -> Any: ...
        def _record_px4_mag_status_snapshot(self, *args: Any, **kwargs: Any) -> Any: ...
        def _recording_camera_timing_diagnostics(self, *args: Any, **kwargs: Any) -> Any: ...
        def _recording_session_metadata(self, *args: Any, **kwargs: Any) -> Any: ...
        def workstation_ulog_snapshot(self, *args: Any, **kwargs: Any) -> Any: ...
        def import_workstation_ulog(self, *args: Any, **kwargs: Any) -> Any: ...
        def _remove_distance_history_point(self, *args: Any, **kwargs: Any) -> Any: ...
        def apply_workstation_run_setup(self, *args: Any, **kwargs: Any) -> Any: ...
        def workstation_run_setup_snapshot(self, *args: Any, **kwargs: Any) -> Any: ...
        def ensure_workstation_run_evidence(self, *args: Any, **kwargs: Any) -> Any: ...
        def append_workstation_run_event(self, *args: Any, **kwargs: Any) -> Any: ...
        def import_workstation_session_artifact(self, *args: Any, **kwargs: Any) -> Any: ...
        def register_workstation_session_artifact(self, *args: Any, **kwargs: Any) -> Any: ...
        def workstation_optical_pose_snapshot(self, *args: Any, **kwargs: Any) -> Any: ...
        def register_workstation_optical_pose_candidate(self, *args: Any, **kwargs: Any) -> Any: ...
        def apply_workstation_optical_pose_candidate(self, *args: Any, **kwargs: Any) -> Any: ...
        def revert_workstation_optical_pose(self, *args: Any, **kwargs: Any) -> Any: ...
        def capture_workstation_optical_pose_burst(self, *args: Any, **kwargs: Any) -> Any: ...
        def import_workstation_optical_pose_burst(self, *args: Any, **kwargs: Any) -> Any: ...
        def import_workstation_optical_pose_terrain(self, *args: Any, **kwargs: Any) -> Any: ...
        def solve_workstation_optical_pose_horizon(self, *args: Any, **kwargs: Any) -> Any: ...
        def solve_workstation_optical_pose_terrain(self, *args: Any, **kwargs: Any) -> Any: ...
        def solve_workstation_optical_pose_gnss_mountain(self, *args: Any, **kwargs: Any) -> Any: ...
        def solve_workstation_optical_pose_drone_yaw(self, *args: Any, **kwargs: Any) -> Any: ...
        def solve_workstation_optical_pose_drone_rtk_multiview(self, *args: Any, **kwargs: Any) -> Any: ...
        def solve_workstation_optical_pose_combined(self, *args: Any, **kwargs: Any) -> Any: ...
        def prepare_workstation_optical_pose_auto_selection(self, *args: Any, **kwargs: Any) -> Any: ...
        def commit_workstation_optical_pose_auto_selection(self, *args: Any, **kwargs: Any) -> Any: ...
        def auto_select_workstation_optical_pose_horizon(self, *args: Any, **kwargs: Any) -> Any: ...
        def auto_select_workstation_optical_pose_mountain(self, *args: Any, **kwargs: Any) -> Any: ...
        def accept_workstation_optical_pose_suggestion(self, *args: Any, **kwargs: Any) -> Any: ...
        def record_workstation_optical_pose_annotation(self, *args: Any, **kwargs: Any) -> Any: ...
        def workstation_optical_pose_candidate_poses(self, *args: Any, **kwargs: Any) -> Any: ...
        def register_workstation_optical_pose_reference_candidate(self, *args: Any, **kwargs: Any) -> Any: ...
        def set_workstation_optical_pose_mountain_ridge_margin(self, *args: Any, **kwargs: Any) -> Any: ...
        def set_workstation_optical_pose_mountain_max_points(self, *args: Any, **kwargs: Any) -> Any: ...
        def clear_workstation_optical_pose_auto_suggestions(self, *args: Any, **kwargs: Any) -> Any: ...
        def capture_workstation_optical_pose_burst_wiris_ftp(self, *args: Any, **kwargs: Any) -> Any: ...
        def _handle_optical_pose_wiris_ftp_capture_completed(self, *args: Any, **kwargs: Any) -> Any: ...
        def _handle_optical_pose_wiris_ftp_capture_failed(self, *args: Any, **kwargs: Any) -> Any: ...
        def _clear_optical_pose_wiris_ftp_capture_worker(self, *args: Any, **kwargs: Any) -> Any: ...
        def capture_workstation_optical_pose_burst_wiris_ftp_oz(self, *args: Any, **kwargs: Any) -> Any: ...
        def _handle_optical_pose_wiris_ftp_oz_capture_completed(self, *args: Any, **kwargs: Any) -> Any: ...
        def _handle_optical_pose_wiris_ftp_oz_capture_failed(self, *args: Any, **kwargs: Any) -> Any: ...
        def _clear_optical_pose_wiris_ftp_oz_capture_worker(self, *args: Any, **kwargs: Any) -> Any: ...
        def configure_target_drone_stream(self, *args: Any, **kwargs: Any) -> Any: ...
        def connect_target_drone_stream(self, *args: Any, **kwargs: Any) -> Any: ...
        def start_target_drone_stream(self, *args: Any, **kwargs: Any) -> Any: ...
        def drain_target_drone_stream(self, *args: Any, **kwargs: Any) -> Any: ...
        def stop_target_drone_stream(self, *args: Any, **kwargs: Any) -> Any: ...
        def target_drone_stream_snapshot(self, *args: Any, **kwargs: Any) -> Any: ...
        def target_drone_spool_artifact(self, *args: Any, **kwargs: Any) -> Any: ...
        def configure_drone_telemetry(self, *args: Any, **kwargs: Any) -> Any: ...
        def connect_drone_telemetry(self, *args: Any, **kwargs: Any) -> Any: ...
        def disconnect_drone_telemetry(self, *args: Any, **kwargs: Any) -> Any: ...
        def save_drone_telemetry_settings(self, *args: Any, **kwargs: Any) -> Any: ...
        def drone_telemetry_snapshot(self, *args: Any, **kwargs: Any) -> Any: ...
        def discover_drone_telemetry_usb(self, *args: Any, **kwargs: Any) -> Any: ...
        def _app_relative_path_text(self, *args: Any, **kwargs: Any) -> Any: ...
        def _checkerboard_validation_mode(self, *args: Any, **kwargs: Any) -> Any: ...
        def _checkerboard_validation_root(self, *args: Any, **kwargs: Any) -> Any: ...
        def _capture_zoom_snapshot(self, *args: Any, **kwargs: Any) -> Any: ...
        def _capture_zoom_level_for_paths(self, *args: Any, **kwargs: Any) -> Any: ...
        def _calibration_image_paths_for_root(self, *args: Any, **kwargs: Any) -> Any: ...
        def _calibration_stereo_pair_paths_for_root(self, *args: Any, **kwargs: Any) -> Any: ...
        def _checkerboard_validation_scan_counts(self, *args: Any, **kwargs: Any) -> Any: ...
        def scan_checkerboard_validation_files(self, *args: Any, **kwargs: Any) -> Any: ...
        def choose_checkerboard_validation_root(self, *args: Any, **kwargs: Any) -> Any: ...
        def choose_checkerboard_validation_json(self, *args: Any, **kwargs: Any) -> Any: ...
        def _checkerboard_validation_set_result(self, *args: Any, **kwargs: Any) -> Any: ...
        def _checkerboard_result_baseline_m(self, *args: Any, **kwargs: Any) -> Any: ...
        def _rotation_delta_deg_from_matrices(self, *args: Any, **kwargs: Any) -> Any: ...
        def _format_checkerboard_validation_result_summary(self, *args: Any, **kwargs: Any) -> Any: ...
        def load_checkerboard_validation_json(self, *args: Any, **kwargs: Any) -> Any: ...
        def _checkerboard_random_subset_count(self, *args: Any, **kwargs: Any) -> Any: ...
        def _calibration_calculation_pair_limit(self, *args: Any, **kwargs: Any) -> Any: ...
        def _select_calibration_pairs(self, *args: Any, **kwargs: Any) -> Any: ...
        def _checkerboard_intrinsic_delta_px(self, *args: Any, **kwargs: Any) -> Any: ...
        def _checkerboard_validation_stddev(self, *args: Any, **kwargs: Any) -> Any: ...
        def _calibration_quality_allows_action(self, *args: Any, **kwargs: Any) -> Any: ...
        def workstation_calibration_quality_snapshot(self, *args: Any, **kwargs: Any) -> Any: ...
        def run_checkerboard_random_validation(self, *args: Any, **kwargs: Any) -> Any: ...
        def _handle_random_validation_failed(self, *args: Any, **kwargs: Any) -> Any: ...
        def _handle_random_validation_completed(self, *args: Any, **kwargs: Any) -> Any: ...
        def _checkerboard_validation_result_row(self, *args: Any, **kwargs: Any) -> Any: ...
        def _checkerboard_validation_result_data(self, *args: Any, **kwargs: Any) -> Any: ...
        def save_checkerboard_validation_result(self, *args: Any, **kwargs: Any) -> Any: ...
        def apply_checkerboard_validation_result(self, *args: Any, **kwargs: Any) -> Any: ...
        def apply_intrinsics_to_distance(self, *args: Any, **kwargs: Any) -> Any: ...
        def _checkerboard_config(self, *args: Any, **kwargs: Any) -> Any: ...
        def _calibration_dirs(self, *args: Any, **kwargs: Any) -> Any: ...
        def _intrinsics_storage_base_path(self, *args: Any, **kwargs: Any) -> Any: ...
        def _intrinsics_storage_path(self, *args: Any, **kwargs: Any) -> Any: ...
        def _extrinsics_storage_path(self, *args: Any, **kwargs: Any) -> Any: ...
        def _calibration_image_paths(self, *args: Any, **kwargs: Any) -> Any: ...
        def _next_calibration_capture_stem(self, *args: Any, **kwargs: Any) -> Any: ...
        def _checkerboard_capture_pair_timing(self, *args: Any, **kwargs: Any) -> Any: ...
        def capture_calibration_images(self, *args: Any, **kwargs: Any) -> Any: ...
        def toggle_auto_calibration_capture(self, *args: Any, **kwargs: Any) -> Any: ...
        def start_auto_calibration_capture(self, *args: Any, **kwargs: Any) -> Any: ...
        def stop_auto_calibration_capture(self, *args: Any, **kwargs: Any) -> Any: ...
        def _handle_checkerboard_capture_completed(self, *args: Any, **kwargs: Any) -> Any: ...
        def _handle_checkerboard_capture_failed(self, *args: Any, **kwargs: Any) -> Any: ...
        def run_intrinsic_calibration(self, *args: Any, **kwargs: Any) -> Any: ...
        def run_stereo_extrinsics_calibration(self, *args: Any, **kwargs: Any) -> Any: ...
        def _handle_stereo_extrinsics_completed(self, *args: Any, **kwargs: Any) -> Any: ...
        def save_stereo_extrinsics_profile(self, *args: Any, **kwargs: Any) -> Any: ...
        def load_stereo_extrinsics_profile(self, *args: Any, **kwargs: Any) -> Any: ...
        def apply_calibration_bundle(self, *args: Any, **kwargs: Any) -> Any: ...
        def _handle_calibration_progress(self, *args: Any, **kwargs: Any) -> Any: ...
        def _handle_intrinsic_calibration_completed(self, *args: Any, **kwargs: Any) -> Any: ...
        def _handle_intrinsic_calibration_failed(self, *args: Any, **kwargs: Any) -> Any: ...
        def _clear_checkerboard_calibration_worker(self, *args: Any, **kwargs: Any) -> Any: ...
        def workstation_checkerboard_validation_snapshot(self, *args: Any, **kwargs: Any) -> Any: ...
        def configure_workstation_checkerboard_validation(self, *args: Any, **kwargs: Any) -> Any: ...
        def load_workstation_checkerboard_validation_json(self, *args: Any, **kwargs: Any) -> Any: ...
        def run_workstation_checkerboard_validation(self, *args: Any, **kwargs: Any) -> Any: ...
        def save_workstation_checkerboard_validation_result(self, *args: Any, **kwargs: Any) -> Any: ...
        def apply_workstation_checkerboard_validation_result(self, *args: Any, **kwargs: Any) -> Any: ...
        def capture_calibration_images_wiris_ftp(self, *args: Any, **kwargs: Any) -> Any: ...
        def _handle_calibration_wiris_ftp_capture_completed(self, *args: Any, **kwargs: Any) -> Any: ...
        def _handle_calibration_wiris_ftp_capture_failed(self, *args: Any, **kwargs: Any) -> Any: ...
        def _clear_calibration_wiris_ftp_capture_worker(self, *args: Any, **kwargs: Any) -> Any: ...
        def capture_calibration_images_wiris_ftp_oz(self, *args: Any, **kwargs: Any) -> Any: ...
        def _handle_calibration_wiris_ftp_oz_capture_completed(self, *args: Any, **kwargs: Any) -> Any: ...
        def _handle_calibration_wiris_ftp_oz_capture_failed(self, *args: Any, **kwargs: Any) -> Any: ...
        def _clear_calibration_wiris_ftp_oz_capture_worker(self, *args: Any, **kwargs: Any) -> Any: ...
        def capture_calibration_images_wiris_oz_capture_only(self, *args: Any, **kwargs: Any) -> Any: ...
        def _handle_calibration_wiris_oz_format_preset_completed(self, *args: Any, **kwargs: Any) -> Any: ...
        def _handle_calibration_wiris_oz_format_preset_failed(self, *args: Any, **kwargs: Any) -> Any: ...
        def _clear_calibration_wiris_oz_format_worker(self, *args: Any, **kwargs: Any) -> Any: ...
        def _start_calibration_wiris_oz_capture(self, *args: Any, **kwargs: Any) -> Any: ...
        def _handle_calibration_wiris_oz_capture_only_completed(self, *args: Any, **kwargs: Any) -> Any: ...
        def _handle_calibration_wiris_oz_capture_only_failed(self, *args: Any, **kwargs: Any) -> Any: ...
        def _clear_calibration_wiris_oz_capture_only_worker(self, *args: Any, **kwargs: Any) -> Any: ...
        def sync_calibration_images_wiris_oz_ftp(self, *args: Any, **kwargs: Any) -> Any: ...
        def _handle_calibration_wiris_oz_gilc_completed(self, *args: Any, **kwargs: Any) -> Any: ...
        def _handle_calibration_wiris_oz_gilc_failed(self, *args: Any, **kwargs: Any) -> Any: ...
        def _clear_calibration_wiris_oz_gilc_worker(self, *args: Any, **kwargs: Any) -> Any: ...
        def _handle_calibration_wiris_oz_sync_completed(self, *args: Any, **kwargs: Any) -> Any: ...
        def _handle_calibration_wiris_oz_sync_failed(self, *args: Any, **kwargs: Any) -> Any: ...
        def _clear_calibration_wiris_oz_sync_worker(self, *args: Any, **kwargs: Any) -> Any: ...
        def _process_next_ftp_oz_sync_pair(self, *args: Any, **kwargs: Any) -> Any: ...
        def _start_calibration_wiris_oz_delete(self, *args: Any, **kwargs: Any) -> Any: ...
        def _handle_calibration_wiris_oz_delete_completed(self, *args: Any, **kwargs: Any) -> Any: ...
        def _handle_calibration_wiris_oz_delete_failed(self, *args: Any, **kwargs: Any) -> Any: ...
        def _clear_calibration_wiris_oz_delete_worker(self, *args: Any, **kwargs: Any) -> Any: ...
        def _calib_oz_ledger_path(self, *args: Any, **kwargs: Any) -> Any: ...
        def _load_calib_oz_ledger(self, *args: Any, **kwargs: Any) -> Any: ...
        def _save_calib_oz_ledger(self, *args: Any, **kwargs: Any) -> Any: ...
        def _calibration_pair_selection_path(self, *args: Any, **kwargs: Any) -> Any: ...
        def _load_calibration_pair_selection(self, *args: Any, **kwargs: Any) -> Any: ...
        def _choose_calibration_pairs(self, *args: Any, **kwargs: Any) -> Any: ...
        def toggle_mavlink_connection(self, *args: Any, **kwargs: Any) -> Any: ...
        def _stop_pixhawk_operation_workers(self, *args: Any, **kwargs: Any) -> Any: ...
        def reset_px4_calibration_group(self, *args: Any, **kwargs: Any) -> Any: ...
        def discover_pc_camera_ip(self, *args: Any, **kwargs: Any) -> Any: ...
        def _apply_camera_network_discovery(self, *args: Any, **kwargs: Any) -> Any: ...
        def _camera_network_discovery_failed(self, *args: Any, **kwargs: Any) -> Any: ...
        def _clear_camera_network_discovery(self, *args: Any, **kwargs: Any) -> Any: ...
        def discover_pc_pixhawk_ip(self, *args: Any, **kwargs: Any) -> Any: ...
        def _apply_pixhawk_host_ip_discovery(self, *args: Any, **kwargs: Any) -> Any: ...
        def _pixhawk_host_ip_discovery_failed(self, *args: Any, **kwargs: Any) -> Any: ...
        def _clear_pixhawk_host_ip_discovery(self, *args: Any, **kwargs: Any) -> Any: ...
        def _append_pixhawk_host_ip_candidates(self, *args: Any, **kwargs: Any) -> Any: ...
        def apply_pc_camera_ip(self, *args: Any, **kwargs: Any) -> Any: ...
        def _apply_camera_network_address_result(self, *args: Any, **kwargs: Any) -> Any: ...
        def _camera_network_apply_failed(self, *args: Any, **kwargs: Any) -> Any: ...
        def _clear_camera_network_apply(self, *args: Any, **kwargs: Any) -> Any: ...
        def _update_mavlink_optional_timeline(self, *args: Any, **kwargs: Any) -> Any: ...
        def _mavlink_connected_sides(self, *args: Any, **kwargs: Any) -> Any: ...
        def _mavlink_open_sides(self, *args: Any, **kwargs: Any) -> Any: ...
        def _refresh_mavlink_side_actions(self, *args: Any, **kwargs: Any) -> Any: ...
        def _mavlink_target_sides(self, *args: Any, **kwargs: Any) -> Any: ...
        def _sync_mavlink_subscriber_settings(self, *args: Any, **kwargs: Any) -> Any: ...
        def _ensure_mavlink_subscriber(self, *args: Any, **kwargs: Any) -> Any: ...
        def _handle_mavlink_status_changed(self, *args: Any, **kwargs: Any) -> Any: ...
        def _configured_mavlink_sides(self, *args: Any, **kwargs: Any) -> Any: ...
        def _mavlink_setup_target_sides(self, *args: Any, **kwargs: Any) -> Any: ...
        def _apply_mavlink_pixhawk_setup(self, *args: Any, **kwargs: Any) -> Any: ...
        def _start_pixhawk_stream_setup_worker(self, *args: Any, **kwargs: Any) -> Any: ...
        def _handle_mavlink_stream_setup_required(self, *args: Any, **kwargs: Any) -> Any: ...
        def _clear_pixhawk_stream_setup_worker(self, *args: Any, **kwargs: Any) -> Any: ...
        def bootstrap_ethernet_mavlink_from_usb(self, *args: Any, **kwargs: Any) -> Any: ...
        def _apply_pixhawk_usb_bootstrap_result(self, *args: Any, **kwargs: Any) -> Any: ...
        def _clear_pixhawk_usb_bootstrap_worker(self, *args: Any, **kwargs: Any) -> Any: ...
        def _temporary_mavlink_heartbeat(self, *args: Any, **kwargs: Any) -> Any: ...
        def _ethernet_mavlink_bootstrap_command(self, *args: Any, **kwargs: Any) -> Any: ...
        def connect_mavlink_pixhawk(self, *args: Any, **kwargs: Any) -> Any: ...
        def _apply_pixhawk_connection_result(self, *args: Any, **kwargs: Any) -> Any: ...
        def _pixhawk_connection_failed(self, *args: Any, **kwargs: Any) -> Any: ...
        def _clear_pixhawk_connection_worker(self, *args: Any, **kwargs: Any) -> Any: ...
        def disconnect_mavlink_pixhawk(self, *args: Any, **kwargs: Any) -> Any: ...
        def apply_selected_pixhawk_candidate(self, *args: Any, **kwargs: Any) -> Any: ...
        def apply_persistent_pixhawk_eth(self, *args: Any, **kwargs: Any) -> Any: ...
        def _apply_pixhawk_persistent_config_result(self, *args: Any, **kwargs: Any) -> Any: ...
        def _clear_pixhawk_persistent_config_worker(self, *args: Any, **kwargs: Any) -> Any: ...
        def reboot_pixhawk_from_mavlink(self, *args: Any, **kwargs: Any) -> Any: ...
        def _persistent_pixhawk_eth_command(self, *args: Any, **kwargs: Any) -> Any: ...
        def _persistent_pixhawk_extras_text(self, *args: Any, **kwargs: Any) -> Any: ...
        def _persistent_pixhawk_net_cfg_text(self, *args: Any, **kwargs: Any) -> Any: ...
        def _persistent_pixhawk_eth_setup(self, *args: Any, **kwargs: Any) -> Any: ...
        def _start_pixhawk_shell_worker(self, *args: Any, **kwargs: Any) -> Any: ...
        def _apply_pixhawk_shell_worker_result(self, *args: Any, **kwargs: Any) -> Any: ...
        def _pixhawk_shell_worker_failed(self, *args: Any, **kwargs: Any) -> Any: ...
        def _clear_pixhawk_shell_worker(self, *args: Any, **kwargs: Any) -> Any: ...
        def reset_px4_level_calibration(self, *args: Any, **kwargs: Any) -> Any: ...
        def _start_pixhawk_parameter_worker(self, *args: Any, **kwargs: Any) -> Any: ...
        def _apply_pixhawk_parameter_worker_result(self, *args: Any, **kwargs: Any) -> Any: ...
        def _pixhawk_parameter_worker_failed(self, *args: Any, **kwargs: Any) -> Any: ...
        def _clear_pixhawk_parameter_worker(self, *args: Any, **kwargs: Any) -> Any: ...
        def _refresh_gnss_base_tab(self, *args: Any, **kwargs: Any) -> Any: ...
        def _gnss_selected_operation_mode(self, *args: Any, **kwargs: Any) -> Any: ...
        def _gnss_active_operation_config(self, *args: Any, **kwargs: Any) -> Any: ...
        def _set_gnss_operation_controls_locked(self, *args: Any, **kwargs: Any) -> Any: ...
        def _build_gnss_operation_config(self, *args: Any, **kwargs: Any) -> Any: ...
        def _gnss_configure_manager(self, *args: Any, **kwargs: Any) -> Any: ...
        def _gnss_status_label_key_for_state(self, *args: Any, **kwargs: Any) -> Any: ...
        def _gnss_transition(self, *args: Any, **kwargs: Any) -> Any: ...
        def _handle_gnss_operation_mode_changed(self, *args: Any, **kwargs: Any) -> Any: ...
        def _gnss_source_identity(self, *args: Any, **kwargs: Any) -> Any: ...
        def _gnss_gga_provider(self, *args: Any, **kwargs: Any) -> Any: ...
        def _resolve_ntrip_credentials(self, *args: Any, **kwargs: Any) -> Any: ...
        def _refresh_ntrip_credential_status(self, *args: Any, **kwargs: Any) -> Any: ...
        def forget_gnss_ntrip_password(self, *args: Any, **kwargs: Any) -> Any: ...
        def set_gnss_ntrip_session_password(self, *args: Any, **kwargs: Any) -> Any: ...
        def start_gnss_network(self, *args: Any, **kwargs: Any) -> Any: ...
        def stop_gnss_network(self, *args: Any, **kwargs: Any) -> Any: ...
        def _handle_gnss_network_connected(self, *args: Any, **kwargs: Any) -> Any: ...
        def _handle_gnss_network_stats(self, *args: Any, **kwargs: Any) -> Any: ...
        def _handle_gnss_network_chunk(self, *args: Any, **kwargs: Any) -> Any: ...
        def _handle_gnss_network_failed(self, *args: Any, **kwargs: Any) -> Any: ...
        def _handle_gnss_network_reconnecting(self, *args: Any, **kwargs: Any) -> Any: ...
        def _handle_gnss_network_diagnostic(self, *args: Any, **kwargs: Any) -> Any: ...
        def _clear_gnss_network_worker(self, *args: Any, **kwargs: Any) -> Any: ...
        def _record_gnss_base_position_row(self, *args: Any, **kwargs: Any) -> Any: ...
        def _handle_gnss_base_position(self, *args: Any, **kwargs: Any) -> Any: ...
        def _fail_gnss_fixed_base_reconfiguration(self, *args: Any, **kwargs: Any) -> Any: ...
        def _handle_gnss_base_ubx_message(self, *args: Any, **kwargs: Any) -> Any: ...
        def _maybe_activate_gnss_fixed_base(self, *args: Any, **kwargs: Any) -> Any: ...
        def _refresh_gnss_operation_readiness(self, *args: Any, **kwargs: Any) -> Any: ...
        def connect_gnss_base_serial(self, *args: Any, **kwargs: Any) -> Any: ...
        def disconnect_gnss_base_serial(self, *args: Any, **kwargs: Any) -> Any: ...
        def start_gnss_base_survey_in(self, *args: Any, **kwargs: Any) -> Any: ...
        def stop_gnss_base_survey_in(self, *args: Any, **kwargs: Any) -> Any: ...
        def discover_gnss_base_serial_ports(self, *args: Any, **kwargs: Any) -> Any: ...
        def _handle_gnss_base_candidates_detected(self, *args: Any, **kwargs: Any) -> Any: ...
        def _handle_gnss_base_candidate_detect_failed(self, *args: Any, **kwargs: Any) -> Any: ...
        def _clear_gnss_base_detect_worker(self, *args: Any, **kwargs: Any) -> Any: ...
        def _set_gnss_base_candidate_rows(self, *args: Any, **kwargs: Any) -> Any: ...
        def _refresh_gnss_base_candidate_actions(self, *args: Any, **kwargs: Any) -> Any: ...
        def apply_selected_gnss_base_candidate(self, *args: Any, **kwargs: Any) -> Any: ...
        def apply_gnss_base_candidate(self, *args: Any, **kwargs: Any) -> Any: ...
        def _handle_gnss_base_serial_connected(self, *args: Any, **kwargs: Any) -> Any: ...
        def _handle_gnss_base_serial_stats(self, *args: Any, **kwargs: Any) -> Any: ...
        def _handle_gnss_base_serial_failed(self, *args: Any, **kwargs: Any) -> Any: ...
        def _handle_gnss_base_serial_disconnected(self, *args: Any, **kwargs: Any) -> Any: ...
        def _handle_gnss_base_satellite_count(self, *args: Any, **kwargs: Any) -> Any: ...
        def _set_gnss_base_quality_waiting(self, *args: Any, **kwargs: Any) -> Any: ...
        def _reset_gnss_base_rtcm_stats(self, *args: Any, **kwargs: Any) -> Any: ...
        def _format_rtcm_message_types(self, *args: Any, **kwargs: Any) -> Any: ...
        def _refresh_gnss_base_rtcm_stats(self, *args: Any, **kwargs: Any) -> Any: ...
        def _handle_gnss_base_satellite_quality(self, *args: Any, **kwargs: Any) -> Any: ...
        def _ensure_rtcm_pipeline(self, *args: Any, **kwargs: Any) -> Any: ...
        def _sync_rtcm_pipeline_config(self, *args: Any, **kwargs: Any) -> Any: ...
        def _sync_rtcm_evidence_context(self, *args: Any, **kwargs: Any) -> Any: ...
        def _close_rtcm_evidence_context_and_drain(self, *args: Any, **kwargs: Any) -> Any: ...
        def _apply_rtcm_pipeline_snapshot(self, *args: Any, **kwargs: Any) -> Any: ...
        def _handle_rtcm_pipeline_evidence_started(self, *args: Any, **kwargs: Any) -> Any: ...
        def _handle_rtcm_pipeline_integrity_failed(self, *args: Any, **kwargs: Any) -> Any: ...
        def _stop_rtcm_pipeline(self, *args: Any, **kwargs: Any) -> Any: ...
        def _gnss_base_rtcm_target_sides(self, *args: Any, **kwargs: Any) -> Any: ...
        def _record_gnss_base_rtcm_frame(self, *args: Any, **kwargs: Any) -> Any: ...
        def _record_gnss_base_rtcm_injection(self, *args: Any, **kwargs: Any) -> Any: ...
        def _handle_gnss_base_rtcm_frame(self, *args: Any, **kwargs: Any) -> Any: ...
        def _ensure_pixhawk_rtcm_worker(self, *args: Any, **kwargs: Any) -> Any: ...
        def _apply_pixhawk_rtcm_result(self, *args: Any, **kwargs: Any) -> Any: ...
        def _clear_pixhawk_rtcm_worker(self, *args: Any, **kwargs: Any) -> Any: ...
        def _handle_gnss_base_command_sent(self, *args: Any, **kwargs: Any) -> Any: ...
        def _handle_gnss_base_survey_in(self, *args: Any, **kwargs: Any) -> Any: ...
        def _update_gnss_base_survey_visualization(self, *args: Any, **kwargs: Any) -> Any: ...
        def _format_eta_seconds(self, *args: Any, **kwargs: Any) -> Any: ...
        def _update_gnss_base_survey_convergence(self, *args: Any, **kwargs: Any) -> Any: ...
        def workstation_measurement_methods_snapshot(self, *args: Any, **kwargs: Any) -> Any: ...
        def select_workstation_measurement_method(self, *args: Any, **kwargs: Any) -> Any: ...
        def configure_workstation_measurement_method_fill(self, *args: Any, **kwargs: Any) -> Any: ...
        def _measurement_method_statuses(self, *args: Any, **kwargs: Any) -> Any: ...
        def _select_measurement_method_for_attempt(self, *args: Any, **kwargs: Any) -> Any: ...
        def _record_no_ready_measurement_pair(self, *args: Any, **kwargs: Any) -> Any: ...
        def _update_detection(self, *args: Any, **kwargs: Any) -> Any: ...
        def _checkerboard_translation_right_in_left_m(self, *args: Any, **kwargs: Any) -> Any: ...
        def _active_baseline_m(self, *args: Any, **kwargs: Any) -> Any: ...
        def _gps_fix_ready_for_baseline(self, *args: Any, **kwargs: Any) -> Any: ...
        def _gps_baseline_usable_for_translation(self, *args: Any, **kwargs: Any) -> Any: ...
        def _rtk_fixed_status_text(self, *args: Any, **kwargs: Any) -> Any: ...
        def _refresh_rtk_baseline(self, *args: Any, **kwargs: Any) -> Any: ...
        def _set_rtk_summary_chip(self, *args: Any, **kwargs: Any) -> Any: ...
        def _baseline_forward_geometry_from_camera_translation(self, *args: Any, **kwargs: Any) -> Any: ...
        def _baseline_camera_translation_in_side_rdf_m(self, *args: Any, **kwargs: Any) -> Any: ...
        def _set_baseline_forward_metric_labels(self, *args: Any, **kwargs: Any) -> Any: ...
        def _baseline_forward_angle_level(self, *args: Any, **kwargs: Any) -> Any: ...
        def _baseline_forward_geometry_text(self, *args: Any, **kwargs: Any) -> Any: ...
        def _refresh_rtk_alignment_view(self, *args: Any, **kwargs: Any) -> Any: ...
        def _optical_alignment_intrinsics_result(self, *args: Any, **kwargs: Any) -> Any: ...
        def _optical_alignment_mean_focal_px(self, *args: Any, **kwargs: Any) -> Any: ...
        def _optical_alignment_expected_dy_px(self, *args: Any, **kwargs: Any) -> Any: ...
        def _optical_alignment_level_from_mrad(self, *args: Any, **kwargs: Any) -> Any: ...
        def _optical_alignment_overall_level(self, *args: Any, **kwargs: Any) -> Any: ...
        def _optical_alignment_level_text(self, *args: Any, **kwargs: Any) -> Any: ...
        def _format_optical_px(self, *args: Any, **kwargs: Any) -> Any: ...
        def _format_optical_mrad(self, *args: Any, **kwargs: Any) -> Any: ...
        def _refresh_optical_alignment_view(self, *args: Any, **kwargs: Any) -> Any: ...
        def _build_measurement_attempt(self, *args: Any, **kwargs: Any) -> Any: ...
        def _translation_right_in_left_m(self, *args: Any, **kwargs: Any) -> Any: ...
        def _latest_translation_right_in_left_m_for_diagnostics(self, *args: Any, **kwargs: Any) -> Any: ...
        def _alignment_view_camera_translation_right_in_left_m(self, *args: Any, **kwargs: Any) -> Any: ...
        def _capture_measurement_gnss_contract(self, *args: Any, **kwargs: Any) -> Any: ...
        def _measurement_gnss_record_metadata(self, *args: Any, **kwargs: Any) -> Any: ...
        def _refresh_distance(self, *args: Any, **kwargs: Any) -> Any: ...
        def _geometry_comparison_rows(self, *args: Any, **kwargs: Any) -> Any: ...
        def _show_measurement_pair(self, *args: Any, **kwargs: Any) -> Any: ...
        def _clear_measurement_pair_display(self, *args: Any, **kwargs: Any) -> Any: ...
        def _clear_selected_runtime_measurement_pair(self, *args: Any, **kwargs: Any) -> Any: ...
        def _detection_wait_detail(self, *args: Any, **kwargs: Any) -> Any: ...
        def _detection_wait_status(self, *args: Any, **kwargs: Any) -> Any: ...
        def _stereo_geometry_wait_detail(self, *args: Any, **kwargs: Any) -> Any: ...
        def _triangulation_wait_detail(self, *args: Any, **kwargs: Any) -> Any: ...
        def _format_distance_label_text(self, *args: Any, **kwargs: Any) -> Any: ...
        def _format_quality_point(self, *args: Any, **kwargs: Any) -> Any: ...
        def _format_attitude_timing_age(self, *args: Any, **kwargs: Any) -> Any: ...
        def _attitude_timing_health_level(self, *args: Any, **kwargs: Any) -> Any: ...
        def _format_attitude_timing_text(self, *args: Any, **kwargs: Any) -> Any: ...
        def _clear_triangulation_quality(self, *args: Any, **kwargs: Any) -> Any: ...
        def _update_triangulation_quality(self, *args: Any, **kwargs: Any) -> Any: ...
        def _set_distance_text(self, *args: Any, **kwargs: Any) -> Any: ...
        def _distance_style(self, *args: Any, **kwargs: Any) -> Any: ...
        def _update_extrinsic(self, *args: Any, **kwargs: Any) -> Any: ...
        def _update_gps(self, *args: Any, **kwargs: Any) -> Any: ...
        def _update_mag_sensor(self, *args: Any, **kwargs: Any) -> Any: ...
        def _update_gps_sensor(self, *args: Any, **kwargs: Any) -> Any: ...
        def _refresh_f9p_compare_label(self, *args: Any, **kwargs: Any) -> Any: ...
        def _update_gnss_relative(self, *args: Any, **kwargs: Any) -> Any: ...
        def _freshest_sensor_slots(self, *args: Any, **kwargs: Any) -> Any: ...
        def _set_health_label(self, *args: Any, **kwargs: Any) -> Any: ...
        def _set_label_text_if_changed(self, *args: Any, **kwargs: Any) -> Any: ...
        def _telemetry_source_label(self, *args: Any, **kwargs: Any) -> Any: ...
        def _gps_position_summary_text(self, *args: Any, **kwargs: Any) -> Any: ...
        def _gps_fix_level(self, *args: Any, **kwargs: Any) -> Any: ...
        def _gps_sensor_status_text(self, *args: Any, **kwargs: Any) -> Any: ...
        def _vector_angle_deg(self, *args: Any, **kwargs: Any) -> Any: ...
        def _fix_type_label(self, *args: Any, **kwargs: Any) -> Any: ...
        def _refresh_pixhawk_sensor_health(self, *args: Any, **kwargs: Any) -> Any: ...
        def _refresh_one_pixhawk_sensor_health(self, *args: Any, **kwargs: Any) -> Any: ...
        def _update_relative_attitude_status(self, *args: Any, **kwargs: Any) -> Any: ...
        def _attitude_delta_level(self, *args: Any, **kwargs: Any) -> Any: ...
        def toggle_camera_connection(self, *args: Any, **kwargs: Any) -> Any: ...
        def toggle_yolo_detection(self, *args: Any, **kwargs: Any) -> Any: ...
        def connect_cameras(self, *args: Any, **kwargs: Any) -> Any: ...
        def start_yolo(self, *args: Any, **kwargs: Any) -> Any: ...
        def stop_yolo_detection(self, *args: Any, **kwargs: Any) -> Any: ...
        def _start_camera_streams(self, *args: Any, **kwargs: Any) -> Any: ...
        def _schedule_camera_start_retry(self, *args: Any, **kwargs: Any) -> Any: ...
        def _cancel_pending_camera_start(self, *args: Any, **kwargs: Any) -> Any: ...
        def stop_yolo_streams(self, *args: Any, **kwargs: Any) -> Any: ...
        def _stop_camera_panes(self, *args: Any, **kwargs: Any) -> Any: ...
        def _clear_calibration_previews(self, *args: Any, **kwargs: Any) -> Any: ...
        def _clear_detection_state(self, *args: Any, **kwargs: Any) -> Any: ...
        def stop_all(self, *args: Any, **kwargs: Any) -> Any: ...
        def begin_measurement_session(self, *args: Any, **kwargs: Any) -> Any: ...
        def measurement_session_snapshot(self, *args: Any, **kwargs: Any) -> Any: ...
        def measurement_package_snapshot(self, *args: Any, **kwargs: Any) -> Any: ...
        def activate_measurement_package_source(self, *args: Any, **kwargs: Any) -> Any: ...
        def begin_measurement_detection(self, *args: Any, **kwargs: Any) -> Any: ...
        def cancel_measurement_detection_start(self, *args: Any, **kwargs: Any) -> Any: ...
        def complete_measurement_detection(self, *args: Any, **kwargs: Any) -> Any: ...
        def finalize_measurement_package(self, *args: Any, **kwargs: Any) -> Any: ...
        def _continue_measurement_package_finalization(self, *args: Any, **kwargs: Any) -> Any: ...
        def _handle_measurement_rtcm_evidence_drained(self, *args: Any, **kwargs: Any) -> Any: ...
        def begin_measurement(self, *args: Any, **kwargs: Any) -> Any: ...
        def begin_next_measurement(self, *args: Any, **kwargs: Any) -> Any: ...
        def close_measurement_session(self, *args: Any, **kwargs: Any) -> Any: ...
        def begin_measurement_revision(self, *args: Any, **kwargs: Any) -> Any: ...
        def verify_measurement_package(self, *args: Any, **kwargs: Any) -> Any: ...
        def verify_session(self, *args: Any, **kwargs: Any) -> Any: ...
        def begin_session(self, *args: Any, **kwargs: Any) -> Any: ...
        def end_measurement(self, *args: Any, **kwargs: Any) -> Any: ...
        def seal_session(self, *args: Any, **kwargs: Any) -> Any: ...
        def close_session(self, *args: Any, **kwargs: Any) -> Any: ...
        def create_revision(self, *args: Any, **kwargs: Any) -> Any: ...
        def verify_package(self, *args: Any, **kwargs: Any) -> Any: ...
        def zoom_command_text(self, *args: Any, **kwargs: Any) -> Any: ...
        def selected_zoom_target(self, *args: Any, **kwargs: Any) -> Any: ...
        def zoom_hosts(self, *args: Any, **kwargs: Any) -> Any: ...
        def _control_port_spin_for_camera(self, *args: Any, **kwargs: Any) -> Any: ...
        def _control_port_for_camera(self, *args: Any, **kwargs: Any) -> Any: ...
        def _set_control_port_for_camera(self, *args: Any, **kwargs: Any) -> Any: ...
        def send_zoom(self, *args: Any, **kwargs: Any) -> Any: ...
        def _apply_wiris_zoom_result(self, *args: Any, **kwargs: Any) -> Any: ...
        def _wiris_zoom_failed(self, *args: Any, **kwargs: Any) -> Any: ...
        def _clear_wiris_zoom_worker(self, *args: Any, **kwargs: Any) -> Any: ...
        def workstation_wiris_command_catalog(self, *args: Any, **kwargs: Any) -> Any: ...
        def workstation_send_wiris_command(self, *args: Any, **kwargs: Any) -> Any: ...
        def workstation_send_zoom(self, *args: Any, **kwargs: Any) -> Any: ...
        def _apply_selected_wiris_command(self, *args: Any, **kwargs: Any) -> Any: ...
        def _append_wiris_command_log(self, *args: Any, **kwargs: Any) -> Any: ...
        def _wiris_command_needs_confirmation(self, *args: Any, **kwargs: Any) -> Any: ...
        def _stus_command_from_unix_time(self, *args: Any, **kwargs: Any) -> Any: ...
        def set_selected_wiris_time_from_pc(self, *args: Any, **kwargs: Any) -> Any: ...
        def _apply_wiris_time_sync_result(self, *args: Any, **kwargs: Any) -> Any: ...
        def _wiris_time_sync_failed(self, *args: Any, **kwargs: Any) -> Any: ...
        def _clear_wiris_time_sync_worker(self, *args: Any, **kwargs: Any) -> Any: ...
        def send_selected_wiris_command(self, *args: Any, **kwargs: Any) -> Any: ...
        def _start_wiris_command_worker(self, *args: Any, **kwargs: Any) -> Any: ...
        def _apply_wiris_command_result(self, *args: Any, **kwargs: Any) -> Any: ...
        def _wiris_command_failed(self, *args: Any, **kwargs: Any) -> Any: ...
        def _clear_wiris_command_worker(self, *args: Any, **kwargs: Any) -> Any: ...
        def query_wiris_lrf_once(self, *args: Any, **kwargs: Any) -> Any: ...
        def _apply_wiris_lrf_result(self, *args: Any, **kwargs: Any) -> Any: ...
        def _wiris_lrf_failed(self, *args: Any, **kwargs: Any) -> Any: ...
        def stop_wiris_lrf_polling(self, *args: Any, **kwargs: Any) -> Any: ...
        def _clear_wiris_lrf_worker(self, *args: Any, **kwargs: Any) -> Any: ...
        def apply_wiris_stip(self, *args: Any, **kwargs: Any) -> Any: ...
        def _apply_wiris_stip_result(self, *args: Any, **kwargs: Any) -> Any: ...
        def _wiris_stip_failed(self, *args: Any, **kwargs: Any) -> Any: ...
        def _clear_wiris_stip_worker(self, *args: Any, **kwargs: Any) -> Any: ...
        def _image_format_target_infos(self, *args: Any, **kwargs: Any) -> Any: ...
        def _start_wiris_image_format_worker(self, *args: Any, **kwargs: Any) -> Any: ...
        def refresh_wiris_image_format_status(self, *args: Any, **kwargs: Any) -> Any: ...
        def apply_oz_only_image_format(self, *args: Any, **kwargs: Any) -> Any: ...
        def set_wiris_image_format(self, *args: Any, **kwargs: Any) -> Any: ...
        def _handle_wiris_image_format_status_completed(self, *args: Any, **kwargs: Any) -> Any: ...
        def _handle_wiris_image_format_preset_completed(self, *args: Any, **kwargs: Any) -> Any: ...
        def _handle_wiris_image_format_set_completed(self, *args: Any, **kwargs: Any) -> Any: ...
        def _wiris_image_format_failed(self, *args: Any, **kwargs: Any) -> Any: ...
        def _clear_wiris_image_format_worker(self, *args: Any, **kwargs: Any) -> Any: ...
        def _shutdown_save_settings(self, *args: Any, **kwargs: Any) -> Any: ...
        def _shutdown_stop_thread(self, *args: Any, **kwargs: Any) -> Any: ...
        def _shutdown_stop_auxiliary_workers(self, *args: Any, **kwargs: Any) -> Any: ...
        def _shutdown_stop_gnss_base_serial(self, *args: Any, **kwargs: Any) -> Any: ...
        def _shutdown_stop_target_drone(self, *args: Any, **kwargs: Any) -> Any: ...
        def _shutdown_stop_drone_telemetry(self, *args: Any, **kwargs: Any) -> Any: ...
        def shutdown_for_close(self, *args: Any, **kwargs: Any) -> Any: ...
        def closeEvent(self, *args: Any, **kwargs: Any) -> Any: ...
        def _stop_calibration_preview_worker(self, *args: Any, **kwargs: Any) -> Any: ...
        def save_current_settings_from_ui(self, *args: Any, **kwargs: Any) -> Any: ...
        def save_workstation_calibration_setup(self, *args: Any, **kwargs: Any) -> Any: ...
        def workstation_settings_snapshot(self, *args: Any, **kwargs: Any) -> Any: ...
        def apply_workstation_system_settings(self, *args: Any, **kwargs: Any) -> Any: ...
        def apply_workstation_measurement_settings(self, *args: Any, **kwargs: Any) -> Any: ...
        def apply_workstation_calibration_setup(self, *args: Any, **kwargs: Any) -> Any: ...
        def workstation_calibration_preview_snapshot(self, *args: Any, **kwargs: Any) -> Any: ...
        def _save_current_settings(self, *args: Any, **kwargs: Any) -> Any: ...
    # END GENERATED PANEL DELEGATE STUBS

    def clear_distance_history(self) -> None:
        if self._session_save_thread is not None:
            self.recording_status_label.setText("세션 기록: 저장 중에는 기록을 초기화할 수 없습니다")
            return
        self.distance_history.clear()
        self.distance_recording_rows.clear()
        self.distance_recording_keys.clear()
        self.external_parameter_recording_rows.clear()
        self.camera_recording_rows["left"].clear()
        self.camera_recording_rows["right"].clear()
        self.px4_recording_rows["left"].clear()
        self.px4_recording_rows["right"].clear()
        self.gnss_base_recording_rows.clear()
        self.extrinsics_comparison_recording_rows.clear()
        if self.distance_recording_enabled and self.recording_session_id:
            spool_root = self._recording_session_dir(self.recording_session_id) / ".spool"
            if isinstance(self.distance_recording_rows, SpoolRows):
                self.distance_recording_rows.bind(spool_root / "pair_rows.jsonl")
            for side, rows in self.camera_recording_rows.items():
                rows.bind(spool_root / f"camera_{side}.jsonl")
            for side, rows in self.px4_recording_rows.items():
                rows.bind(spool_root / f"px4_{side}.jsonl")
            self.gnss_base_recording_rows.bind(spool_root / "gnss_base.jsonl")
            self.extrinsics_comparison_recording_rows.bind(spool_root / "extrinsics_comparison.jsonl")
        self._recording_pair_row_index.clear()
        if not self.distance_recording_enabled:
            self.distance_recording_started_monotonic_s = None
        self._last_distance_history_key = None
        self.distance_plot.clear()
        self._refresh_recording_table()
        self._refresh_recording_status_label()
        self._log("[Depth] 거리 그래프와 미저장 기록을 초기화했습니다.")
        self._refresh_operation_header()

    def _workstation_setting(self, name: str, default: object = None) -> object:
        value = self._workstation_setting_overrides.get(name, default)
        return default if value is None else value















    def discover_custom_mavlink_watch(self) -> None:
        messages = discover_mavlink_messages()
        dialog = TopicBrowserDialog(messages, parent=self, title="MAVLink Message Browser")
        if dialog.exec_() != QtWidgets.QDialog.Accepted:
            return
        selected_message = dialog.selected_topic()
        if not selected_message:
            return
        message_name = selected_message.get("name", "").strip().upper()
        if not message_name:
            return
        fields = mavlink_graphable_fields_for_message(message_name)
        if not fields:
            self._log(f"[픽스호크] 그래프 가능한 MAVLink field를 찾지 못했습니다: {message_name}")
            return
        field_dialog = FieldSelectionDialog(message_name, "MAVLink", fields, parent=self)
        if field_dialog.exec_() != QtWidgets.QDialog.Accepted:
            return
        selected_fields = field_dialog.selected_fields()
        if not selected_fields:
            self._log("[픽스호크] 선택한 MAVLink field가 없습니다.")
            return
        for field in selected_fields:
            key = f"mavlink:{message_name}.{field}"
            self._add_custom_mavlink_watch(
                key=key,
                topic=message_name,
                field=field,
                message=message_name,
            )
        self._log(f"[픽스호크] MAVLink Custom watch {len(selected_fields)}개 추가: {message_name}")

    def _add_custom_mavlink_watch(
        self,
        *,
        key: str,
        topic: str,
        field: str,
        message: str | None = None,
    ) -> None:
        spec = {"key": key, "topic": topic, "field": field, "source": "mavlink"}
        if message:
            spec["message"] = message
        if spec not in self.custom_watch_specs:
            self.custom_watch_specs.append(spec)
        if self.custom_watch_remove_combo.findData(key) < 0:
            self.custom_watch_remove_combo.addItem(key, key)
        if self.mavlink_pixhawk is not None:
            self.mavlink_pixhawk.add_custom_watch(spec)

    def remove_custom_mavlink_watch(self, key: str) -> None:
        removed_specs = [spec for spec in self.custom_watch_specs if str(spec.get("key", "")) == key]
        if not removed_specs:
            self._log(f"[픽스호크] 제거할 Custom watch가 없습니다: {key}")
            return
        self.custom_watch_specs = [spec for spec in self.custom_watch_specs if str(spec.get("key", "")) != key]
        combo_index = self.custom_watch_remove_combo.findData(key)
        if combo_index >= 0:
            self.custom_watch_remove_combo.removeItem(combo_index)
        self.custom_watch_plot.remove_series(key)
        self.custom_watch_plot.remove_series(f"left:{key}")
        self.custom_watch_plot.remove_series(f"right:{key}")
        if self.mavlink_pixhawk is not None:
            self.mavlink_pixhawk.remove_custom_watch(key)
        self._log(f"[픽스호크] Custom watch 제거: {key}")

    def remove_selected_custom_topic_watch(self) -> None:
        key = self.custom_watch_remove_combo.currentData()
        if key is None:
            self._log("[픽스호크] 제거할 Custom watch가 없습니다.")
            return
        self.remove_custom_mavlink_watch(str(key))

    def add_custom_mavlink_watch_from_ui(self) -> None:
        message = self.custom_watch_message_edit.text().strip().upper()
        field = self.custom_watch_field_edit.text().strip()
        name = self.custom_watch_name_edit.text().strip() or f"mavlink:{message}.{field}"
        if not message or not field:
            self._log("[픽스호크] MAVLink watch 추가 실패: message/field가 비어 있습니다.")
            return
        self._add_custom_mavlink_watch(
            key=name,
            topic=message,
            field=field,
            message=message,
        )
        self._log(f"[픽스호크] MAVLink watch 추가: {name} = {message}.{field}")

    def _update_custom_watch(self, sample: CustomTopicSample) -> None:
        if sample.numeric_value is not None:
            self.custom_watch_plot.add_point(
                TelemetryHistoryPoint(sample.timestamp_s, sample.key, sample.numeric_value)
            )


















    def _usb_bootstrap_baud_candidates(self, selected_baud: int) -> tuple[int, ...]:
        ordered = [int(selected_baud)]
        ordered.extend(int(baud) for baud in DEFAULT_USB_MAVLINK_BAUDRATES)
        return tuple(dict.fromkeys(baud for baud in ordered if baud > 0))

    def _set_usb_bootstrap_baud(self, camera: str, baud: int) -> None:
        if camera == "left":
            self.left_pixhawk_bootstrap_baud_spin.setValue(int(baud))
        else:
            self.right_pixhawk_bootstrap_baud_spin.setValue(int(baud))
















    def open_px4_shell(self, camera: str) -> None:
        if self.mavlink_pixhawk is None or camera not in self.mavlink_pixhawk.connections:
            self._log(f"[픽스호크] {camera_label_ko(camera)} PX4 Shell을 열 수 없습니다: MAVLink 연결이 없습니다.")
            return
        dialog = self.px4_shell_dialogs.get(camera)
        if dialog is None:
            dialog = Px4ShellDialog(
                f"{camera_label_ko(camera)} PX4 Shell",
                lambda command, camera=camera: self._send_px4_shell_command(camera, command),
                lambda _checked=False, camera=camera: self._read_px4_sensor_mag_shell(camera),
            )
            dialog.destroyed.connect(lambda _obj=None, camera=camera: self.px4_shell_dialogs.pop(camera, None))
            self.px4_shell_dialogs[camera] = dialog
        dialog.show()
        dialog.raise_()
        dialog.activateWindow()
        self._log(f"[픽스호크] {camera_label_ko(camera)} PX4 Shell 창을 열었습니다.")

    def _send_px4_shell_command(self, camera: str, command: str) -> None:
        if self.mavlink_pixhawk is None:
            self._log(f"[픽스호크] {camera_label_ko(camera)} PX4 Shell 명령 실패: MAVLink 연결이 없습니다.")
            return
        self._start_pixhawk_shell_worker(camera, command)

    def _read_px4_sensor_mag_shell(self, camera: str) -> None:
        """Read sensor_mag instances through the existing Ethernet Shell path."""
        commands = "; ".join(f"listener sensor_mag -i {instance} -n 1" for instance in range(5))
        self._start_pixhawk_shell_worker(
            camera,
            commands,
            operation="sensor_mag_shell_watch",
            timeout_s=4.0,
            on_complete=lambda result, camera=camera: self._apply_px4_sensor_mag_shell_result(camera, result),
        )

    def _apply_px4_sensor_mag_shell_result(self, camera: str, result: dict[str, object]) -> None:
        output = str(result.get("output") or "")
        states = parse_px4_sensor_mag_shell_output(output, camera=camera)
        if not states:
            self._log(
                f"[픽스호크] {camera_label_ko(camera)} sensor_mag Shell 결과에서 CAN device_id를 찾지 못했습니다."
            )
            return
        can_states = self.latest_can_mag_sensors.setdefault(camera, {})
        for state in states:
            can_states[state.sensor_slot] = state
        self._log(
            f"[픽스호크] {camera_label_ko(camera)} Shell sensor_mag 감시 갱신: "
            + ", ".join(f"{state.sensor_slot} device_id={state.device_id}" for state in states)
        )









    def _px4_mag_camera_label(self, camera: str) -> str:
        if camera == "left":
            return "Pixhawk 1"
        if camera == "right":
            return "Pixhawk 2"
        if camera == "drone":
            return "Drone Pixhawk"
        return str(camera)

    def _px4_mag_manager_has_data_source(self, camera: str) -> bool:
        if self.latest_vehicle_mag_device_ids.get(camera) is not None:
            return True
        return camera in self._mavlink_connected_sides()

    def _sync_px4_mag_manager_store(self, camera: str) -> bool:
        manager = self.px4_mag_managers.get(camera) if hasattr(self, "px4_mag_managers") else None
        if manager is None:
            return False
        mavlink = getattr(self, "mavlink_pixhawk", None)
        if mavlink is not None and camera in self._mavlink_connected_sides():
            current_store = getattr(manager, "store", None)
            if not (
                isinstance(current_store, MavlinkParameterStore)
                and current_store.subscriber is mavlink
                and current_store.camera == camera
            ):
                manager.store = MavlinkParameterStore(mavlink, camera)
            return True
        current_store = getattr(manager, "store", None)
        if isinstance(current_store, MavlinkParameterStore):
            manager.store = MockParameterStore(current_pixhawk_6x_mag_params())
        return False

    def _set_px4_mag_manager_button_state(
        self,
        camera: str,
        *,
        refresh_enabled: bool,
        actions_enabled: bool,
        tooltip: str = "",
    ) -> None:
        refresh_button = self.px4_mag_refresh_buttons.get(camera)
        if refresh_button is not None:
            refresh_button.setEnabled(refresh_enabled)
            refresh_button.setToolTip("" if refresh_enabled else tooltip)
        for button in (
            self.px4_mag_apply_manual_buttons.get(camera),
            self.px4_mag_save_buttons.get(camera),
            self.px4_mag_reboot_buttons.get(camera),
        ):
            if button is None:
                continue
            button.setEnabled(actions_enabled)
            button.setToolTip("" if actions_enabled else tooltip)

    def _clear_px4_mag_manager_ui(self, camera: str) -> None:
        if not hasattr(self, "px4_mag_tables"):
            return
        table = self.px4_mag_tables.get(camera)
        status_label = self.px4_mag_status_labels.get(camera)
        if table is not None:
            table.setRowCount(0)
        if status_label is not None:
            status_label.setText(
                f"{self._px4_mag_camera_label(camera)} | 픽스호크 MAG 관리: MAVLink 수신 대기 중\n"
                "MAVLink에서 실제 MAG device_id를 수신하면 관리 내용이 표시됩니다."
            )
        self._set_px4_mag_manager_button_state(
            camera,
            refresh_enabled=False,
            actions_enabled=False,
            tooltip="MAVLink MAG 수신 후 사용 가능",
        )

    def _clear_px4_mag_manager_connection_state(self, camera: str | None = None) -> None:
        sides = ("left", "right") if camera is None else (camera,)
        for side in sides:
            self.latest_vehicle_mag_device_ids[side] = None
            self._px4_mag_status_cache.pop(side, None)
            self._px4_calibration_parameter_error.pop(side, None)
            for key in tuple(self._px4_calibration_reset_error):
                if key.startswith(f"{side}:"):
                    self._px4_calibration_reset_error.pop(key, None)
            self._px4_calibration_reset_pending.pop(side, None)
            self._clear_px4_mag_manager_ui(side)

    def _render_px4_mag_mavlink_pending_status(self, camera: str) -> None:
        status_label = self.px4_mag_status_labels.get(camera) if hasattr(self, "px4_mag_status_labels") else None
        if status_label is not None:
            actual = self.latest_vehicle_mag_device_ids.get(camera)
            status_label.setText(
                f"{self._px4_mag_camera_label(camera)} | Actual selected MAG: "
                f"{'N/A' if actual is None else actual}\n"
                "MAVLink MAG 수신 중. PARAM readback은 새로고침 버튼을 눌렀을 때 실행합니다."
            )
        self._set_px4_mag_manager_button_state(
            camera,
            refresh_enabled=True,
            actions_enabled=False,
            tooltip="먼저 새로고침으로 CAL_MAG PARAM readback을 완료하세요.",
        )

    def refresh_px4_mag_manager(self, camera: str | None = None, *, read_params: bool = False) -> None:
        if not hasattr(self, "px4_mag_managers"):
            return
        cameras = ("left", "right") if camera is None else (camera,)
        for side in cameras:
            manager = self.px4_mag_managers.get(side)
            if manager is None:
                continue
            if not self._px4_mag_manager_has_data_source(side):
                self._clear_px4_mag_manager_ui(side)
                continue
            using_real_params = self._sync_px4_mag_manager_store(side)
            manager.actual_selected_device_id = self.latest_vehicle_mag_device_ids.get(side)
            if using_real_params and not read_params:
                self._render_px4_mag_mavlink_pending_status(side)
                continue
            if using_real_params and read_params:
                self._start_pixhawk_parameter_worker(side, "mag_refresh")
                continue
            status = manager.refresh()
            if using_real_params:
                self._render_px4_mag_status(side, status)
            else:
                self._render_px4_mag_mavlink_pending_status(side)
            self._record_px4_mag_status_snapshot(side)

    def _sync_px4_attitude_cal_store(self, camera: str) -> bool:
        manager = self.px4_attitude_cal_managers.get(camera) if hasattr(self, "px4_attitude_cal_managers") else None
        if manager is None:
            return False
        mavlink = getattr(self, "mavlink_pixhawk", None)
        if mavlink is not None and camera in self._mavlink_connected_sides():
            current_store = getattr(manager, "store", None)
            if not (
                isinstance(current_store, MavlinkAttitudeParameterStore)
                and current_store.subscriber is mavlink
                and current_store.camera == camera
            ):
                manager.store = MavlinkAttitudeParameterStore(mavlink, camera)
            return True
        current_store = getattr(manager, "store", None)
        if isinstance(current_store, MavlinkAttitudeParameterStore):
            manager.store = MockParameterStore(current_pixhawk_6x_attitude_cal_params())
        return False

    def _clear_px4_attitude_cal_ui(self, camera: str) -> None:
        label = getattr(self, "px4_attitude_cal_status_labels", {}).get(camera)
        if label is not None:
            label.setText(
                f"{self._px4_mag_camera_label(camera)} | PX4 자세 보정 상태: 프로토콜 수신 대기 중\n"
                "MAVLink 연결 후 새로고침을 누르면 가속도계/자이로/보드 회전 보정값을 읽습니다."
            )
        button = getattr(self, "px4_attitude_cal_refresh_buttons", {}).get(camera)
        if button is not None:
            button.setEnabled(False)
            button.setToolTip("MAVLink 연결 후 사용 가능")

    def _clear_px4_attitude_cal_connection_state(self, camera: str | None = None) -> None:
        sides = ("left", "right") if camera is None else (camera,)
        for side in sides:
            self._px4_attitude_cal_status_cache.pop(side, None)
            self._clear_px4_attitude_cal_ui(side)

    def _render_px4_attitude_cal_pending_status(self, camera: str) -> None:
        label = self.px4_attitude_cal_status_labels.get(camera) if hasattr(self, "px4_attitude_cal_status_labels") else None
        if label is not None:
            label.setText(
                f"{self._px4_mag_camera_label(camera)} | MAVLink 연결됨. "
                "새로고침을 눌러 CAL_ACC/CAL_GYRO/SENS_BOARD PARAM을 읽으세요."
            )
        button = self.px4_attitude_cal_refresh_buttons.get(camera) if hasattr(self, "px4_attitude_cal_refresh_buttons") else None
        if button is not None:
            button.setEnabled(True)
            button.setToolTip("")

    @staticmethod
    def _format_attitude_cal_value(value: float | None) -> str:
        return "N/A" if value is None else f"{value:.5f}"

    def _format_attitude_cal_status_text(self, camera: str, status: Px4AttitudeCalStatus) -> str:
        lines = [f"{self._px4_mag_camera_label(camera)} | PX4 자세 보정 상태"]
        for slot in status.accel_slots:
            if slot.device_id is None:
                continue
            offset_text = ", ".join(self._format_attitude_cal_value(v) for v in slot.offset_m_s2)
            scale_text = ", ".join(self._format_attitude_cal_value(v) for v in slot.scale)
            lines.append(
                f"ACC{slot.slot}: id={slot.device_id} prio={slot.priority} "
                f"calibrated={'Y' if slot.calibrated else 'N'} offset(m/s^2)=({offset_text}) scale=({scale_text})"
            )
        for slot in status.gyro_slots:
            if slot.device_id is None:
                continue
            offset_text = ", ".join(self._format_attitude_cal_value(v) for v in slot.offset_rad_s)
            lines.append(
                f"GYRO{slot.slot}: id={slot.device_id} prio={slot.priority} "
                f"calibrated={'Y' if slot.calibrated else 'N'} offset(rad/s)=({offset_text})"
            )
        board = status.board_rotation
        level_text = ", ".join(
            "N/A" if v is None else f"{self._format_attitude_cal_value(v)}deg" for v in board.level_offset_deg
        )
        lines.append(f"BOARD_ROT: {board.rotation_label} level_offset=({level_text})")
        if status.warning_text:
            lines.append(status.warning_text)
        return "\n".join(lines)

    def _render_px4_attitude_cal_status(
        self, camera: str, status: Px4AttitudeCalStatus, *, using_real_params: bool
    ) -> None:
        self._px4_attitude_cal_status_cache[camera] = status
        label = self.px4_attitude_cal_status_labels.get(camera) if hasattr(self, "px4_attitude_cal_status_labels") else None
        if label is not None:
            label.setText(self._format_attitude_cal_status_text(camera, status))
        button = self.px4_attitude_cal_refresh_buttons.get(camera) if hasattr(self, "px4_attitude_cal_refresh_buttons") else None
        if button is not None:
            button.setEnabled(using_real_params)
            button.setToolTip("" if using_real_params else "MAVLink 연결 후 사용 가능")

    def refresh_px4_attitude_cal_status(self, camera: str | None = None, *, read_params: bool = False) -> None:
        if not hasattr(self, "px4_attitude_cal_managers"):
            return
        cameras = ("left", "right") if camera is None else (camera,)
        for side in cameras:
            manager = self.px4_attitude_cal_managers.get(side)
            if manager is None:
                continue
            if not self._px4_mag_manager_has_data_source(side):
                self._clear_px4_attitude_cal_ui(side)
                continue
            using_real_params = self._sync_px4_attitude_cal_store(side)
            if using_real_params and not read_params:
                self._render_px4_attitude_cal_pending_status(side)
                continue
            if using_real_params and read_params:
                self._start_pixhawk_parameter_worker(side, "attitude_refresh")
                continue
            status = manager.refresh()
            self._render_px4_attitude_cal_status(side, status, using_real_params=using_real_params)

    def _px4_mag_role_for_device_id(self, device_id: int | None) -> str | None:
        if device_id == BOARD_INTERNAL_MAG_ID:
            return "BOARD"
        if device_id in RM3100_PRIMARY_IDS:
            return "CAN1"
        if device_id in RM3100_BACKUP_IDS:
            return "CAN2"
        if device_id == GPS1_MAG_ID:
            return "GPS1_MAG"
        if device_id == GPS2_MAG_ID:
            return "GPS2_MAG"
        return None

    def apply_manual_mag_priorities_from_panel(self, camera: str = "left") -> None:
        manager = self.px4_mag_managers[camera]
        label = self._px4_mag_camera_label(camera)
        if not self._px4_mag_manager_has_data_source(camera):
            self._log(f"[PX4 MAG] {label} 수동 Priority 적용 차단: MAVLink MAG 수신 없음")
            self._clear_px4_mag_manager_ui(camera)
            return
        if manager.armed:
            self._log(f"[PX4 MAG] {label} 수동 Priority 적용 차단: Disarm required")
            self.refresh_px4_mag_manager(camera, read_params=True)
            return
        using_mavlink_params = self._sync_px4_mag_manager_store(camera)
        if not using_mavlink_params:
            self._log(
                f"[PX4 MAG] {label} 수동 Priority 적용 차단: "
                "MAVLink PARAM 연결 후 다시 시도하세요."
            )
            self.refresh_px4_mag_manager(camera, read_params=True)
            return
        spins = self.px4_mag_priority_spins.get(camera, {})
        desired = {role: _px4_priority_combo_value(spin) for role, spin in spins.items()}
        if using_mavlink_params:
            if self._start_pixhawk_parameter_worker(
                camera,
                "mag_apply",
                priorities=desired,
            ):
                return
            self._log(f"[PX4 MAG] {label} 수동 Priority 적용이 이미 진행 중입니다.")
            return
        try:
            status = manager.refresh()
            applied: list[str] = []
            missing = set(desired)
            for slot in status.slots:
                role = self._px4_mag_role_for_device_id(slot.device_id)
                if role is None or role not in desired:
                    continue
                name = f"CAL_MAG{slot.slot}_PRIO"
                manager.store.set_param(name, desired[role])
                applied.append(f"{role}:{name}={desired[role]}")
                missing.discard(role)
            if not applied:
                raise RuntimeError("적용 가능한 BOARD/CAN/GPS MAG 슬롯을 찾지 못했습니다.")
            status = manager.refresh()
        except Exception as exc:
            self._log(f"[PX4 MAG] {label} 수동 Priority 적용 실패: {exc}")
            self.refresh_px4_mag_manager(camera, read_params=True)
            return
        self._log(f"[PX4 MAG] {label} 수동 Priority 적용: {', '.join(applied)}")
        if missing:
            self._log(f"[PX4 MAG] {label} 수동 Priority 미적용 role: {', '.join(sorted(missing))}")
        self._render_px4_mag_status(camera, status)

    def save_px4_mag_params_from_panel(self, camera: str = "left") -> None:
        manager = self.px4_mag_managers[camera]
        label = self._px4_mag_camera_label(camera)
        if not self._px4_mag_manager_has_data_source(camera):
            self._log(f"[PX4 MAG] {label} param save 차단: MAVLink MAG 수신 없음")
            self._clear_px4_mag_manager_ui(camera)
            return
        if manager.armed:
            self._log(f"[PX4 MAG] {label} param save 차단: Disarm required")
        else:
            if self.mavlink_pixhawk is not None and camera in self._mavlink_connected_sides():
                if self._start_pixhawk_shell_worker(
                    camera,
                    "param save",
                    timeout_s=2.0,
                    on_complete=lambda _result, camera=camera: self._start_pixhawk_parameter_worker(
                        camera, "mag_refresh"
                    ),
                ):
                    return
            else:
                self._log(f"[PX4 MAG] {label} QGC 또는 PX4 shell에서 `param save` 후 reboot 필요")
        self.refresh_px4_mag_manager(camera, read_params=True)

    def reboot_px4_from_panel(self, camera: str = "left") -> None:
        manager = self.px4_mag_managers[camera]
        label = self._px4_mag_camera_label(camera)
        if not self._px4_mag_manager_has_data_source(camera):
            self._log(f"[PX4 MAG] {label} reboot 차단: MAVLink MAG 수신 없음")
            self._clear_px4_mag_manager_ui(camera)
            return
        if manager.armed:
            self._log(f"[PX4 MAG] {label} reboot 차단: Disarm required")
            self.refresh_px4_mag_manager(camera)
            return
        if self.mavlink_pixhawk is None or camera not in self._mavlink_connected_sides():
            self._log(f"[PX4 MAG] {label} PX4 재부팅: QGC 또는 PX4 shell에서 수행 필요")
            self.refresh_px4_mag_manager(camera)
            return
        result = QtWidgets.QMessageBox.question(
            self,
            f"Reboot {label}",
            f"{label} Pixhawk에 MAVLink 재부팅 명령을 보냅니다.\n계속할까요?",
            QtWidgets.QMessageBox.Yes | QtWidgets.QMessageBox.No,
            QtWidgets.QMessageBox.No,
        )
        if result == QtWidgets.QMessageBox.Yes:
            self._start_pixhawk_shell_worker(
                camera,
                "MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN",
                operation="reboot",
                timeout_s=1.0,
            )
        else:
            self._log(f"[PX4 MAG] {label} MAVLink 재부팅 명령을 취소했습니다.")
        self.refresh_px4_mag_manager(camera)

    def _render_px4_mag_status(self, camera: str, status) -> None:
        self._px4_mag_status_cache[camera] = status
        if not hasattr(self, "px4_mag_tables"):
            return
        priority_controls = self.px4_mag_priority_spins.get(camera, {})
        for slot in status.slots:
            role = self._px4_mag_role_for_device_id(slot.device_id)
            if role is None or role not in priority_controls or slot.priority is None:
                continue
            _set_px4_priority_combo_value(priority_controls[role], slot.priority)
        table = self.px4_mag_tables.get(camera)
        status_label = self.px4_mag_status_labels.get(camera)
        if status_label is None:
            return
        if table is not None:
            table.setRowCount(len(status.slots))
            for row, slot in enumerate(status.slots):
                values = [
                    f"CAL_MAG{slot.slot}",
                    "N/A" if slot.device_id is None else str(slot.device_id),
                    slot.device_type,
                    slot.interface,
                    "true" if slot.is_rm3100 else "false",
                    mag_priority_display(slot.priority),
                    "true" if slot.enabled else "false",
                    "true" if slot.expected_selected else "false",
                    "true" if slot.actual_selected else "false",
                    slot.status,
                ]
                for column, value in enumerate(values):
                    item = QtWidgets.QTableWidgetItem(value)
                    if slot.is_rm3100:
                        item.setForeground(QtGui.QColor(SUCCESS_TEXT))
                    if slot.actual_selected:
                        item.setForeground(QtGui.QColor(ACCENT))
                    elif slot.expected_selected:
                        item.setForeground(QtGui.QColor(WARNING_TEXT))
                    item.setFlags(QtCore.Qt.ItemIsEnabled | QtCore.Qt.ItemIsSelectable)
                    table.setItem(row, column, item)
            table.resizeRowsToContents()
            table.horizontalHeader().setSectionResizeMode(QtWidgets.QHeaderView.Stretch)
        expected = status.expected_selected_device_id
        actual = status.actual_selected_device_id
        warning = status.warning_text or "No warnings."
        label = self._px4_mag_camera_label(camera)
        text = (
            f"{label} | Expected selected MAG: {'N/A' if expected is None else expected} | "
            f"Actual selected MAG: {'N/A' if actual is None else actual} | "
            "Save/Reboot: QGC 또는 PX4 shell에서 `param save` 후 reboot 필요\n"
            f"{warning}"
        )
        status_label.setText(text)
        disable_actions = bool(status.armed)
        self._set_px4_mag_manager_button_state(
            camera,
            refresh_enabled=True,
            actions_enabled=not disable_actions,
            tooltip="Disarm required" if disable_actions else "",
        )

    def _append_px4_shell_output(self, camera: str, text: str) -> None:
        self._handle_eth0_probe_output(camera, text)
        dialog = self.px4_shell_dialogs.get(camera)
        if dialog is not None:
            dialog.append_output(text)

    def _request_mavlink_eth0_status(self, target_sides: Sequence[str]) -> None:
        if self.mavlink_pixhawk is None:
            return
        connected_sides = self._mavlink_connected_sides()
        for camera in target_sides:
            if camera not in connected_sides:
                continue
            self._eth0_probe_counter += 1
            token = self._eth0_probe_counter
            self._eth0_probe_tokens[camera] = token
            self._eth0_probe_buffers[camera] = ""
            label = camera_label_ko(camera)
            if not self._start_pixhawk_shell_worker(camera, "ifconfig eth0", timeout_s=2.0):
                self._eth0_probe_tokens.pop(camera, None)
                self._eth0_probe_buffers.pop(camera, None)
                continue
            configured_ip = (
                self.left_pixhawk_eth_ip_edit.text().strip()
                if camera == "left"
                else self.right_pixhawk_eth_ip_edit.text().strip()
            )
            self._log(
                f"[픽스호크] {label} 실제 eth0 확인 요청: ifconfig eth0 "
                f"(GUI 설정값={configured_ip or '<empty>'})"
            )
            QtCore.QTimer.singleShot(2500, lambda camera=camera, token=token: self._finish_eth0_probe_if_pending(camera, token))

    def _handle_eth0_probe_output(self, camera: str, text: str) -> None:
        if camera not in self._eth0_probe_tokens:
            return
        buffer = self._eth0_probe_buffers.get(camera, "") + text
        self._eth0_probe_buffers[camera] = buffer
        lower = buffer.lower()
        if "eth0" not in lower:
            return
        match = re.search(r"(?:inet addr:|inet\s+)(\d{1,3}(?:\.\d{1,3}){3})", buffer)
        if match:
            ip_text = match.group(1)
            try:
                IPv4Address(ip_text)
            except ValueError:
                return
            self._clear_eth0_probe(camera)
            self._log(f"[픽스호크] {camera_label_ko(camera)} 실제 eth0 IP 확인: {ip_text}")
            return
        if any(marker in lower for marker in ("no such device", "not found", "invalid interface", "does not exist")):
            self._clear_eth0_probe(camera)
            self._log(f"[픽스호크] {camera_label_ko(camera)} 실제 eth0 확인 실패: eth0 인터페이스를 찾지 못했습니다.")

    def _finish_eth0_probe_if_pending(self, camera: str, token: int) -> None:
        if self._eth0_probe_tokens.get(camera) != token:
            return
        self._clear_eth0_probe(camera)
        self._log(
            f"[픽스호크] {camera_label_ko(camera)} 실제 eth0 확인 실패: "
            "PX4 shell 응답이 없어 MAVLink만으로 현재 eth0 IP를 확인할 수 없습니다."
        )

    def _clear_eth0_probe(self, camera: str) -> None:
        self._eth0_probe_tokens.pop(camera, None)
        self._eth0_probe_buffers.pop(camera, None)

    def _pixhawk_host_ip_for_mavlink(self, *, extra_hints: Sequence[str] = ()) -> str:
        text = self.pixhawk_host_address_edit.text().strip()
        if "/" in text:
            text = text.split("/", 1)[0]
        if not text:
            raise ValueError("Pixhawk Host IP가 비어 있어 Ethernet MAVLink를 시작할 수 없습니다.")
        try:
            IPv4Address(text)
        except ValueError as exc:
            raise ValueError(f"Pixhawk Host IP 형식이 올바르지 않습니다: {text}") from exc
        # Host-interface discovery is an explicit asynchronous Workstation
        # command.  Do not rescan NetworkManager from a PX4 command handler;
        # the selected candidate is already the operator's input here.
        return text

    def _active_mavlink_remote_hosts(self) -> tuple[str, ...]:
        subscriber = self.mavlink_pixhawk
        if subscriber is None:
            return ()
        return self._mavlink_remote_hosts_for_sides(subscriber, tuple(getattr(subscriber, "connections", {}).keys()))

    def _mavlink_remote_hosts_for_sides(
        self,
        subscriber: DualPixhawkMavlinkSubscriber,
        sides: Sequence[str],
    ) -> tuple[str, ...]:
        hosts: list[str] = []
        side_set = set(sides)
        for connection in getattr(subscriber, "connections", {}).values():
            side = next(
                (
                    key
                    for key, candidate in getattr(subscriber, "connections", {}).items()
                    if candidate is connection
                ),
                "",
            )
            if side_set and side not in side_set:
                continue
            clients = getattr(connection, "clients", None)
            if not clients:
                continue
            for client in clients:
                if isinstance(client, tuple) and client:
                    hosts.append(str(client[0]))
        return tuple(dict.fromkeys(hosts))

    def _build_mavlink_baud_spin(self) -> QtWidgets.QSpinBox:
        spin = QtWidgets.QSpinBox()
        spin.setRange(1_200, 4_000_000)
        spin.setValue(57_600)
        spin.setSingleStep(9_600)
        return spin

    def _mount_assumption_status_label(self) -> str:
        if self._distance_mode() == "checkerboard":
            return "checkerboard stereo extrinsics"
        return "GPS/RTK baseline only"

    def _format_mount_assumption_summary(self) -> str:
        if self._distance_mode() == "checkerboard":
            return (
                "mount_assumption:\n"
                "  model: checkerboard-stereo-extrinsics-v1\n"
                "  calculation: fixed_left_to_right_optical_rdf_transform_only"
            )
        return (
            "mount_assumption:\n"
            "  model: gps-rtk-baseline-only-v1\n"
            "  calculation: live_gps_rtk_baseline_and_pixhawk_attitude_only"
        )

    def _set_combo_data(self, combo: QtWidgets.QComboBox, value: str) -> None:
        index = combo.findData(value)
        if index >= 0:
            combo.setCurrentIndex(index)

    def _mavlink_stream_port(self, url: str, *, default: int) -> int:
        text = url.strip().rsplit(":", 1)[-1]
        if "/" in text:
            text = text.split("/", 1)[0]
        try:
            return int(text)
        except ValueError:
            return int(default)











    # Workstation advanced-tool bridges.  The methods below are deliberately
    # small: they validate the public arguments, then hand control back to
    # the existing panel widgets/managers.  They do not create a second
    # camera, Pixhawk, preview worker, or session writer.



    def workstation_discover_custom_mavlink_watch(self) -> None:
        self.discover_custom_mavlink_watch()

    def workstation_add_custom_mavlink_watch(self, name: str, message: str, field: str) -> str:
        message = str(message or "").strip().upper()
        field = str(field or "").strip()
        name = str(name or "").strip() or f"mavlink:{message}.{field}"
        if not message or not field:
            raise ValueError("MAVLink custom watch requires message and field")
        self._add_custom_mavlink_watch(
            key=name,
            topic=message,
            field=field,
            message=message,
        )
        self._log(f"[픽스호크] Workstation MAVLink watch 추가: {name} = {message}.{field}")
        return name

    def workstation_remove_custom_mavlink_watch(self, key: str) -> None:
        key = str(key or "").strip()
        if not key:
            raise ValueError("custom watch key is empty")
        self.remove_custom_mavlink_watch(key)

    def workstation_refresh_baseline_field_browser(self) -> dict[str, object]:
        mavlink_rows = tuple(
            (
                str(item.get("name") or ""),
                ", ".join(mavlink_graphable_fields_for_message(str(item.get("name") or ""))),
            )
            for item in discover_mavlink_messages()
            if str(item.get("name") or "")
        )
        return {
            "mavlink_messages": mavlink_rows,
            "mavlink_headers": ("message", "fields"),
            "axis_contract": "GPS/RTK source fields remain producer-labelled; no NED/ENU/camera-frame conversion",
        }

    def workstation_open_px4_shell(self, camera: str) -> None:
        camera = str(camera or "")
        if camera not in {"left", "right"}:
            raise ValueError(f"Unknown Pixhawk side: {camera}")
        self.open_px4_shell(camera)

    def workstation_refresh_px4_mag(self, camera: str, *, read_params: bool = True) -> None:
        camera = str(camera or "")
        if camera not in {"left", "right"}:
            raise ValueError(f"Unknown Pixhawk side: {camera}")
        if read_params and self._start_pixhawk_parameter_worker(camera, "mag_refresh"):
            return
        self.refresh_px4_mag_manager(camera, read_params=bool(read_params))

    def workstation_refresh_px4_attitude_cal(self, camera: str, *, read_params: bool = True) -> None:
        camera = str(camera or "")
        if camera not in {"left", "right"}:
            raise ValueError(f"Unknown Pixhawk side: {camera}")
        if read_params and self._start_pixhawk_parameter_worker(camera, "attitude_refresh"):
            return
        self.refresh_px4_attitude_cal_status(camera, read_params=bool(read_params))

    def workstation_refresh_px4_calibration_params(self, camera: str, *, read_params: bool = True) -> None:
        """Read the QGC calibration parameter set for one Pixhawk."""

        camera = str(camera or "")
        if camera not in {"left", "right"}:
            raise ValueError(f"Unknown Pixhawk side: {camera}")
        if read_params and self._start_pixhawk_parameter_worker(camera, "calibration_refresh"):
            return
        self._log(
            f"[픽스호크] {self._px4_mag_camera_label(camera)} QGC calibration PARAM readback을 시작할 수 없습니다."
        )

    def workstation_apply_px4_mag_priorities(self, camera: str, priorities: dict[str, int]) -> None:
        camera = str(camera or "")
        if camera not in {"left", "right"}:
            raise ValueError(f"Unknown Pixhawk side: {camera}")
        spins = self.px4_mag_priority_spins.get(camera, {})
        for role, value in dict(priorities).items():
            spin = spins.get(str(role))
            if spin is None:
                raise ValueError(f"Unknown PX4 MAG role: {role}")
            _set_px4_priority_combo_value(spin, int(value))
        if self._start_pixhawk_parameter_worker(
            camera,
            "mag_apply",
            priorities={str(role): int(value) for role, value in dict(priorities).items()},
        ):
            return
        self.apply_manual_mag_priorities_from_panel(camera)

    def workstation_save_px4_mag_params(self, camera: str) -> None:
        camera = str(camera or "")
        if camera not in {"left", "right"}:
            raise ValueError(f"Unknown Pixhawk side: {camera}")
        self._start_pixhawk_shell_worker(
            camera,
            "param save",
            timeout_s=2.0,
            on_complete=lambda _result, camera=camera: self._start_pixhawk_parameter_worker(
                camera, "mag_refresh"
            ),
        )

    def workstation_reboot_px4(self, camera: str) -> None:
        camera = str(camera or "")
        if camera not in {"left", "right"}:
            raise ValueError(f"Unknown Pixhawk side: {camera}")
        manager = self.px4_mag_managers.get(camera)
        if manager is None or manager.armed:
            self._log(f"[PX4 MAG] {self._px4_mag_camera_label(camera)} 재부팅 차단: Disarm required")
            return
        if self.mavlink_pixhawk is None or camera not in self._mavlink_open_sides():
            self._log(f"[PX4 MAG] {self._px4_mag_camera_label(camera)} 재부팅 차단: MAVLink 연결 없음")
            return
        result = QtWidgets.QMessageBox.question(
            self,
            f"Reboot {self._px4_mag_camera_label(camera)}",
            f"{self._px4_mag_camera_label(camera)} Pixhawk에 MAVLink 재부팅 명령을 보냅니다.\n계속할까요?",
            QtWidgets.QMessageBox.Yes | QtWidgets.QMessageBox.No,
            QtWidgets.QMessageBox.No,
        )
        if result != QtWidgets.QMessageBox.Yes:
            self._log(f"[PX4 MAG] {self._px4_mag_camera_label(camera)} 재부팅을 취소했습니다.")
            return
        self._start_pixhawk_shell_worker(
            camera,
            "MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN",
            operation="reboot",
            timeout_s=1.0,
        )

    def workstation_refresh_calibration_preview(self) -> dict[str, object]:
        self._workstation_calibration_preview_active = True
        self.calibration_preview_times["left"] = 0.0
        self.calibration_preview_times["right"] = 0.0
        self._refresh_all_calibration_previews()
        return self.workstation_calibration_preview_snapshot()





























































































































































    def _auto_load_intrinsics(self) -> None:
        if not bool(self._workstation_setting("auto_load_intrinsics", self.settings.auto_load_intrinsics)):
            return
        zoom_level = self.zoom_level_spin.value()
        try:
            result, source_path, error = self._auto_load_intrinsics_candidate(zoom_level)
        except Exception as exc:
            result = None
            source_path = None
            error = f"unexpected intrinsic profile resolution failure: {exc}"
        if result is None:
            detail = error or f"no valid profile for zoom {zoom_level:02d}"
            self.calibration_status_label.setText(
                f"Failed to auto-load intrinsics for zoom {zoom_level:02d}: {detail}"
            )
            self._log(f"[캘리브레이션] intrinsic 자동 로드를 건너뜁니다: {detail}")
            self._refresh_operation_header()
            self._refresh_zoom_intrinsics_tab()
            return
        state_names = (
            "last_calibration_result",
            "last_calibration_zoom_level",
            "_active_intrinsics_profile_path",
            "active_intrinsics",
            "_active_intrinsics_source_zoom_level",
            "_active_intrinsics_profile_hash",
            "selected_extrinsics_profile",
            "checkerboard_extrinsics_profile",
            "_selected_extrinsics_profile_path",
            "_selected_extrinsics_profile_hash",
            "_active_extrinsics_profile_path",
            "_active_extrinsics_profile_hash",
            "_active_extrinsics_source_zoom_level",
            "_active_calibration_selection_mode",
        )
        previous_state = {name: getattr(self, name) for name in state_names}

        def restore_previous_state() -> None:
            for name, value in previous_state.items():
                setattr(self, name, value)

        try:
            self.last_calibration_result = result
            self.last_calibration_zoom_level = zoom_level
            self._active_intrinsics_profile_path = str(source_path)
            if self.apply_intrinsics_to_distance() is False:
                restore_previous_state()
                self._log(
                    f"[캘리브레이션] intrinsic 자동 적용이 quality/grid gate에서 차단됐습니다 "
                    f"(zoom={zoom_level:02d})"
                )
                self._refresh_operation_header()
                self._refresh_zoom_intrinsics_tab()
                return
            self.calibration_status_label.setText(
                f"Auto-loaded intrinsics for zoom {zoom_level:02d}: {source_path}"
            )
            self._log(
                f"[캘리브레이션] intrinsic을 자동으로 불러왔습니다 "
                f"(zoom={zoom_level:02d}, path={source_path})"
            )
        except Exception as exc:
            restore_previous_state()
            self.calibration_status_label.setText(f"Failed to auto-load intrinsics: {exc}")
            self._log(f"[캘리브레이션] intrinsic 자동 로드 실패: {exc}")
        self._refresh_operation_header()
        self._refresh_zoom_intrinsics_tab()

    def save_intrinsics(self, *, quality_warning_confirmed: bool = False) -> None:
        if self.last_calibration_result is None:
            self.calibration_status_label.setText("Run calibration before saving intrinsics.")
            return
        quality_reader = getattr(self, "_calibration_quality_state", None)
        if callable(quality_reader):
            quality_state, quality_reasons = quality_reader(self.last_calibration_result)
            if quality_state == "FAIL" or (
                quality_state == "WARNING" and not quality_warning_confirmed
            ):
                qualifier = (
                    "FAIL"
                    if quality_state == "FAIL"
                    else "WARNING requires explicit confirmation"
                )
                self.calibration_status_label.setText(
                    f"Cannot save intrinsics: calibration quality {qualifier} ("
                    + ", ".join(quality_reasons)
                    + ")"
                )
                return
            if quality_state == "WARNING":
                summary = dict(getattr(self, "checkerboard_validation_summary", {}) or {})
                summary["quality_override"] = {
                    "action": "save_intrinsics",
                    "confirmed": True,
                    "confirmed_utc_ns": time.time_ns(),
                    "reason_codes": list(quality_reasons),
                }
                self.checkerboard_validation_summary = summary
        root, _left_dir, _right_dir = self._calibration_dirs()
        root.mkdir(parents=True, exist_ok=True)
        zoom_level = self.last_calibration_zoom_level
        if zoom_level is None:
            self.calibration_status_label.setText(
                "Cannot save intrinsics: capture-time camera zoom readback is unavailable."
            )
            self._log(
                "[캘리브레이션] intrinsic 저장을 중단했습니다. "
                "촬영 당시 양쪽 카메라 GZVV readback이 필요합니다."
            )
            return
        zoom_output = self._intrinsics_storage_path(zoom_level)
        latest_output = self._intrinsics_storage_base_path()
        metadata = {
            "note": "Generated by StereoWIRIS Camera Calibration tab",
            "timestamp_s": time.time(),
            "zoom_level": zoom_level,
            "capture_zoom_level": zoom_level,
            "zoom_source": "capture_metadata",
            "zoom_target": str(self.zoom_target_combo.currentData() or "both"),
        }
        # K/D and R/T have independent lifecycles.  Keep the generic JSON
        # loader backward-compatible with older embedded files, but new
        # per-zoom intrinsic saves intentionally exclude stereo extrinsics.
        intrinsics_only = without_stereo_extrinsics(self.last_calibration_result)
        save_intrinsics_json(
            zoom_output,
            intrinsics_only,
            metadata=metadata,
        )
        if zoom_output != latest_output:
            save_intrinsics_json(latest_output, intrinsics_only, metadata=metadata)
        self.last_calibration_zoom_level = zoom_level
        self.calibration_status_label.setText(f"Saved intrinsics for zoom {zoom_level:02d}: {zoom_output}")
        self._add_zoom_intrinsics_comparison_path(zoom_output)
        self._save_current_settings(zoom_output)
        self._log(f"[캘리브레이션] intrinsic 파일을 저장했습니다 (zoom={zoom_level:02d}): {zoom_output}")
        self._refresh_operation_header()
        self._refresh_zoom_intrinsics_tab()

    def _validate_intrinsics_result(self, result: StereoIntrinsicsResult) -> tuple[str, ...]:
        return validate_intrinsics_result(result)

    def _confirm_intrinsics_profile_mismatch(
        self,
        path: Path,
        result: StereoIntrinsicsResult,
        metadata_zoom_level: int | None,
    ) -> bool:
        warnings: list[str] = []
        current_zoom_level = self.zoom_level_spin.value()
        if metadata_zoom_level is None:
            warnings.append(
                "파일 metadata.zoom_level 없음; 수동 검토 적용만 가능하며 다음 실행의 자동 로드는 차단됨"
            )
        elif metadata_zoom_level != current_zoom_level:
            warnings.append(
                f"파일 zoom_level={metadata_zoom_level:02d}, 현재 WIRIS zoom={current_zoom_level:02d}"
            )
        left_size = (result.left.width, result.left.height)
        right_size = (result.right.width, result.right.height)
        if left_size != right_size:
            warnings.append(f"좌/우 calibration 해상도가 다름: left={left_size}, right={right_size}")
        if warnings:
            warning_text = "; ".join(warnings)
            self.calibration_status_label.setText(
                f"Profile warning (non-blocking): {warning_text}"
            )
            self._log(
                f"[캘리브레이션] intrinsic profile mismatch warning; "
                f"manual apply remains allowed: {path}: {warning_text}"
            )
        return True

    def load_intrinsics(self) -> None:
        initial_path = self._intrinsics_storage_path(self.zoom_level_spin.value())
        selected, _selected_filter = QtWidgets.QFileDialog.getOpenFileName(
            self,
            "Load intrinsics",
            str(initial_path if initial_path.exists() else initial_path.parent),
            "Intrinsics JSON (*.json);;All files (*)",
        )
        if not selected:
            return
        path = Path(selected)
        try:
            result = load_intrinsics_json(path)
            errors = self._validate_intrinsics_result(result)
            if errors:
                raise ValueError("; ".join(errors))
            metadata_zoom_level = self._intrinsics_file_zoom_level(path)
        except Exception as exc:
            self.calibration_status_label.setText(f"Failed to load intrinsics: {exc}")
            self._log(f"[캘리브레이션] intrinsic 파일 검증 실패: {path}: {exc}")
            return
        if not self._confirm_intrinsics_profile_mismatch(path, result, metadata_zoom_level):
            self._log(f"[캘리브레이션] intrinsic 적용을 취소했습니다: {path}")
            return
        self.last_calibration_result = result
        self.calibration_image_diagnostics = tuple(result.image_diagnostics)
        self.last_calibration_zoom_level = (
            self.zoom_level_spin.value() if metadata_zoom_level is None else metadata_zoom_level
        )
        self._active_intrinsics_profile_path = str(path)
        if self.apply_intrinsics_to_distance() is False:
            self._log(f"[캘리브레이션] intrinsic 적용이 quality/grid gate에서 차단됐습니다: {path}")
            self._refresh_zoom_intrinsics_tab()
            self._refresh_operation_header()
            return
        self._save_current_settings(path)
        self.calibration_status_label.setText(f"Loaded and applied intrinsics: {path.name}")
        self._add_zoom_intrinsics_comparison_path(path)
        self._log(f"[캘리브레이션] intrinsic 파일을 선택해 적용했습니다: {path}")
        self._refresh_zoom_intrinsics_tab()
        self._refresh_operation_header()

    def load_current_zoom_intrinsics(self) -> None:
        zoom_level = self.zoom_level_spin.value()
        result = self._load_current_zoom_intrinsics_for_apply()
        if result is None:
            path = self._intrinsics_storage_path(zoom_level)
            self.calibration_status_label.setText(f"Failed to load intrinsics for zoom {zoom_level:02d}: {path}")
            self._refresh_operation_header()
            self._refresh_zoom_intrinsics_tab()
            return
        self.last_calibration_result = result
        self.calibration_image_diagnostics = tuple(result.image_diagnostics)
        self.last_calibration_zoom_level = zoom_level
        zoom_path = self._intrinsics_storage_path(zoom_level)
        self._active_intrinsics_profile_path = str(
            zoom_path if zoom_path.exists() else self._intrinsics_storage_base_path()
        )
        if self.apply_intrinsics_to_distance() is False:
            self._log(
                f"[캘리브레이션] 현재 zoom intrinsic 적용이 quality/grid gate에서 차단됐습니다 "
                f"(zoom={zoom_level:02d})"
            )
            self._refresh_zoom_intrinsics_tab()
            self._refresh_operation_header()
            return
        self._save_current_settings()
        self.calibration_status_label.setText(f"Applied intrinsics for zoom {zoom_level:02d}")
        self._log(f"[캘리브레이션] intrinsic 파일을 적용했습니다 (zoom={zoom_level:02d})")
        self._refresh_zoom_intrinsics_tab()
        self._refresh_operation_header()

    def _initial_status(self) -> None:
        self._start_interface_status_worker()
        self.camera_connection_label.setText(STARTUP_CAMERA_STATUS)
        self.pixhawk_connection_label.setText(STARTUP_PIXHAWK_STATUS)
        self._log(STARTUP_READY_MESSAGE)
        self._recover_retryable_recording_session()
        if self.auto_start_pixhawk:
            QtCore.QTimer.singleShot(500, self._auto_start_pixhawk)
        if self.auto_start:
            self._log("[카메라] --start: WIRIS 활성화와 RTSP 프레임 확인 후 GStreamer를 시작합니다.")
            self._start_startup_camera_sequence()

    def _start_interface_status_worker(self) -> None:
        if self.interface_status_thread is not None:
            return
        thread = QtCore.QThread(self)
        worker = InterfaceStatusWorker(self.interface_edit.text().strip())
        worker.moveToThread(thread)
        thread.started.connect(worker.run)
        worker.ready.connect(self._apply_interface_status)
        worker.failed.connect(self._interface_status_failed)
        worker.finished.connect(thread.quit)
        worker.finished.connect(worker.deleteLater)
        thread.finished.connect(thread.deleteLater)
        thread.finished.connect(self._clear_interface_status_worker)
        self.interface_status_thread = thread
        self.interface_status_worker = worker
        thread.start()

    def _apply_interface_status(self, interface_name: str, status: str) -> None:
        self._log(f"[연결] 네트워크 인터페이스 상태 ({interface_name}):\n{status}")

    def _interface_status_failed(self, error: str) -> None:
        self._log(f"[연결] 네트워크 인터페이스 상태 확인 실패: {error}")

    def _clear_interface_status_worker(self) -> None:
        self.interface_status_thread = None
        self.interface_status_worker = None

    def _auto_start_pixhawk(self) -> None:
        self.tabs.setCurrentIndex(self.alignment_tab_index)
        self.alignment_tabs.setCurrentIndex(self.baseline_alignment_tab_index)
        if self.mavlink_pixhawk is None:
            self.connect_mavlink_pixhawk()

    def _log_state(self, camera: str, state: str) -> None:
        self._log(f"[카메라] {self._camera_name_ko(camera)} 스트림 상태: {state}")
        decoder = self._extract_active_decoder_from_state(state)
        if decoder is not None:
            self._set_active_stream_decoder(camera, decoder)

    def _camera_name_ko(self, camera: str) -> str:
        return camera_display_name(camera)

    def _extract_active_decoder_from_state(self, state: str) -> str | None:
        prefix = "GStreamer 디코더: "
        if state.startswith(prefix):
            decoder = state.removeprefix(prefix).strip()
            return decoder or None
        return None

    def _set_active_stream_decoder(self, camera: str, decoder: str | None) -> None:
        if camera not in self.active_stream_decoders:
            return
        normalized = (decoder or "").strip() or None
        if self.active_stream_decoders.get(camera) == normalized:
            return
        self.active_stream_decoders[camera] = normalized

    def _log(self, text: str) -> None:
        package_state = str(
            getattr(self, "_measurement_package_state", "IDLE") or "IDLE"
        ).upper()
        if package_state in {"COLLECTING", "POST_CAPTURE", "VERIFYING", "RECOVERABLE"}:
            session_id = str(getattr(self, "recording_session_id", "") or "")
            if session_id:
                try:
                    from Workstation.workstation_core.package_provenance import redact_secrets

                    log_path = self._recording_session_dir(session_id) / "audit" / "application_log.jsonl"
                    log_path.parent.mkdir(parents=True, exist_ok=True)
                    record = {
                        "schema": "workstation-application-log-v1",
                        "utc_ns": time.time_ns(),
                        "message": redact_secrets(str(text)),
                    }
                    with log_path.open("ab") as handle:
                        handle.write(
                            json.dumps(
                                record,
                                ensure_ascii=False,
                                allow_nan=False,
                                sort_keys=True,
                                separators=(",", ":"),
                            ).encode("utf-8")
                            + b"\n"
                        )
                        handle.flush()
                        os.fsync(handle.fileno())
                except (OSError, TypeError, ValueError):
                    # Evidence-log failure is surfaced by package inventory;
                    # it must never recursively break the operator log path.
                    pass
        targets: list[QtWidgets.QPlainTextEdit] = []
        is_ftp_message = "FTP" in text.upper()
        is_wiris_message = (
            text.startswith("[카메라]")
            or text.startswith("[캘리브레이션]")
            or text.startswith("[시계]")
            or text.startswith("[연결]")
            or text.startswith("[arp]")
            or "WIRIS" in text
        )
        is_pixhawk_message = text.startswith("[픽스호크]") or (
            not is_wiris_message and ("Pixhawk" in text or "PX4" in text or "MAVLink" in text)
        )
        if is_pixhawk_message:
            targets.append(self.pixhawk_log)
        if is_wiris_message:
            targets.append(self.wiris_log)
        if is_ftp_message:
            self.ftp_log_lines.append(text)
        if text.startswith("[YOLO]") or text.startswith("[Depth]") or text.startswith("[저장]"):
            targets.append(self.camera_yolo_log)
        if not targets:
            targets.append(self.wiris_log)
        for target in dict.fromkeys(targets):
            target.appendPlainText(text)
        if is_wiris_message:
            try:
                print(text, flush=True)
            except OSError:
                # A GUI launched without a live terminal must keep its UI/log
                # path alive even if stdout was closed by the launcher.
                pass



# Every controller module whose methods are exposed directly on
# WirisStereoDistancePanel. `accessor_name` must name a lazy-init method
# already defined on the class (see the `_x_controller` accessor block
# above) that caches the real controller instance; `bind_controller_delegates`
# falls back to an uncached `controller_cls(panel)` construction when that
# accessor isn't present (e.g. a bare test double standing in for the panel).
_CONTROLLER_DELEGATE_BINDINGS: tuple[ControllerDelegateBinding, ...] = (
    ControllerDelegateBinding(CameraTimeSyncController, "_camera_time_sync_controller", CAMERA_TIME_SYNC_METHODS),
    ControllerDelegateBinding(RecordingController, "_recording_controller", RECORDING_CONTROLLER_METHODS),
    ControllerDelegateBinding(RunEvidenceController, "_run_evidence_controller", RUN_EVIDENCE_CONTROLLER_METHODS),
    ControllerDelegateBinding(OpticalPoseController, "_optical_pose_controller", OPTICAL_POSE_CONTROLLER_METHODS),
    ControllerDelegateBinding(TargetDroneController, "_target_drone_controller", TARGET_DRONE_CONTROLLER_METHODS),
    ControllerDelegateBinding(
        DroneTelemetryController, "_drone_telemetry_controller", DRONE_TELEMETRY_CONTROLLER_METHODS
    ),
    ControllerDelegateBinding(
        CalibrationWorkflowController, "_calibration_workflow_controller", CALIBRATION_WORKFLOW_METHODS
    ),
    ControllerDelegateBinding(
        DeviceRuntimeController, "_device_runtime_controller", DEVICE_RUNTIME_CONTROLLER_METHODS
    ),
    ControllerDelegateBinding(GnssRuntimeController, "_gnss_runtime_controller", GNSS_RUNTIME_CONTROLLER_METHODS),
    ControllerDelegateBinding(
        MeasurementViewController, "_measurement_view_controller", MEASUREMENT_VIEW_CONTROLLER_METHODS
    ),
    ControllerDelegateBinding(
        SensorPresentationController, "_sensor_presentation_controller", SENSOR_PRESENTATION_METHODS
    ),
    ControllerDelegateBinding(
        CameraStreamLifecycleController, "_camera_stream_lifecycle_controller", CAMERA_STREAM_LIFECYCLE_METHODS
    ),
    ControllerDelegateBinding(
        MeasurementPackageController,
        "_measurement_package_controller",
        MEASUREMENT_PACKAGE_METHODS,
    ),
    ControllerDelegateBinding(WirisControlController, "_wiris_control_controller", WIRIS_CONTROL_METHODS),
    ControllerDelegateBinding(AppLifecycleController, "_app_lifecycle_controller", APP_LIFECYCLE_METHODS),
    ControllerDelegateBinding(
        WorkstationSettingsController, "_workstation_settings_controller", SETTINGS_CONTROLLER_METHODS
    ),
)

for _controller_delegate_binding in _CONTROLLER_DELEGATE_BINDINGS:
    bind_controller_delegates(WirisStereoDistancePanel, _controller_delegate_binding)


def _configure_gl_surface_format() -> None:
    """Request a Compatibility-profile GL 2.1 context before QApplication exists.

    pyqtgraph's opengl.GLViewWidget misparses some Mesa version strings (e.g.
    ``4.6 (Compatibility Profile) Mesa ...``) and refuses to initialize unless
    the default surface format is pinned explicitly first. Skipped entirely if
    PyOpenGL isn't installed, so this stays a no-op when the 3D quality view
    will fall back to its placeholder anyway.
    """

    try:
        import OpenGL  # noqa: F401
    except Exception:
        return
    try:
        surface_format = QtGui.QSurfaceFormat()
        surface_format.setVersion(2, 1)
        surface_format.setProfile(QtGui.QSurfaceFormat.CompatibilityProfile)
        QtGui.QSurfaceFormat.setDefaultFormat(surface_format)
    except Exception:
        pass


def _parse_ui_mode(argv: list[str]) -> tuple[str, list[str]]:
    parser = argparse.ArgumentParser(add_help=False)
    parser.add_argument("--ui", choices=("workstation",), default="workstation")
    parsed, remaining = parser.parse_known_args(argv[1:])
    return str(parsed.ui), [argv[0], *remaining]


def main() -> int:
    os.environ.setdefault("QT_AUTO_SCREEN_SCALE_FACTOR", "1")
    try:
        faulthandler.register(signal.SIGUSR1)
    except Exception:
        pass
    _ui_mode, cleaned_argv = _parse_ui_mode(list(sys.argv))
    sys.argv[:] = cleaned_argv
    auto_start = "--start" in sys.argv
    if auto_start:
        sys.argv.remove("--start")
    auto_start_pixhawk = "--start-pixhawk" in sys.argv or "--connect-pixhawk" in sys.argv
    if "--start-pixhawk" in sys.argv:
        sys.argv.remove("--start-pixhawk")
    if "--connect-pixhawk" in sys.argv:
        sys.argv.remove("--connect-pixhawk")
    maximized = "--maximized" in sys.argv
    if maximized:
        sys.argv.remove("--maximized")
    _configure_gl_surface_format()
    app = QtWidgets.QApplication(sys.argv)
    from Workstation.main_window import WorkstationMainWindow

    window = WorkstationMainWindow(
        auto_start=auto_start,
        auto_start_pixhawk=auto_start_pixhawk,
    )
    if maximized:
        window.showMaximized()
    else:
        window.show()
    return app.exec_()
