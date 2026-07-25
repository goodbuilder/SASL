from __future__ import annotations

import csv
import ftplib
import html as html_lib
import faulthandler
import json
import math
import os
import queue
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
from urllib.parse import urlparse

from PyQt5 import QtCore, QtGui, QtWidgets

from wiris_stereo_distance.calibration import (
    CheckerboardConfig,
    StereoIntrinsicsResult,
    calibrate_camera_from_images,
    calibrate_stereo_extrinsics_from_image_pairs,
    detect_checkerboard,
    opencv_available,
    load_intrinsics_json,
    save_intrinsics_json,
)

from wiris_stereo_distance.attitude_geometry import (
    quaternion_conjugate,
    quaternion_multiply,
    relative_attitude,
    relative_rotation_angle_deg,
)
from wiris_stereo_distance.camera_pane import CameraPane
from wiris_stereo_distance.coordinate_contract import (
    BODY_FRAME,
    CAMERA_FRAME,
    CAMERA_MOUNT_PROFILE,
    LEGACY_XYZ_ALIAS_CONTRACT,
    MEASUREMENT_AXIS_CONTRACT,
    MEASUREMENT_FRAME,
    PUBLIC_MEASUREMENT_VECTOR_CONTRACT,
    WORLD_FRAME,
)
from wiris_stereo_distance.measurement_widgets import (
    DistanceHistoryPlot,
    DistanceHistoryPoint,
    TriangulationQualityPlot,
)
from wiris_stereo_distance.measurement_attempt import (
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
from wiris_stereo_distance.measurement_controller import calculate_measurement_geometry
from wiris_stereo_distance.network import (
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
from wiris_stereo_distance.baseline_alignment import (
    BaselineQuality,
    BaselineVector,
    baseline_from_gps,
    evaluate_baseline_quality,
)
from wiris_stereo_distance.alignment_managers import (
    AlignmentProfileManager,
    AlignmentValidator,
    AttitudeMonitor,
    BaselineManager,
    CoordinateSystemManager,
    GNSSMonitor,
    MagnetometerMonitor,
)
from wiris_stereo_distance.photo_clock import (
    ClockFolderResult,
    ClockOffsetResult,
    PhotoMetadataError,
    compare_photo_clocks,
    compare_photo_folders,
    format_offset_seconds,
    format_photo_timestamp,
)
from wiris_stereo_distance.pixhawk_attitude import (
    CameraExtrinsicState,
    CustomTopicSample,
    DEFAULT_USB_MAVLINK_BAUDRATES,
    DualPixhawkAttitudeSubscriber,
    DualPixhawkMavlinkSubscriber,
    camera_label_ko,
    describe_ros2_message_fields,
    discover_mavlink_messages,
    discover_pixhawk_topics,
    discover_ros2_topic_types,
    mavlink_graphable_fields_for_message,
    probe_usb_mavlink_devices,
    sensor_gps_topics,
    sensor_mag_topics,
    usb_mavlink_device_candidates,
    _ros2_topic_has_publisher,
)
from wiris_stereo_distance.px4_mag_manager import (
    BOARD_INTERNAL_MAG_ID,
    GPS1_MAG_ID,
    GPS2_MAG_ID,
    MAG_SLOTS,
    MockParameterStore,
    Px4MagManager,
    RM3100_BACKUP_ID,
    RM3100_BACKUP_IDS,
    RM3100_IDS,
    RM3100_PRIMARY_ID,
    RM3100_PRIMARY_IDS,
    current_pixhawk_6x_mag_params,
)
from wiris_stereo_distance.rtk_geometry import GpsPositionState, RtkBaselineEstimate, estimate_rtk_baseline, ned_delta_m
from wiris_stereo_distance.recording_session import (
    SavedSessionFiles,
    SpoolRows,
    discover_retryable_sessions,
    format_video_paths,
    load_retryable_session,
    raw_video_paths,
    save_session_files,
    session_dir,
    session_id_now,
    write_recovery_metadata,
)
from wiris_stereo_distance.settings import (
    APP_ROOT,
    DEFAULT_CLOCK_DOWNLOAD_DIR,
    DEFAULT_INTRINSICS_PATH,
    AppSettings,
    load_settings,
    resolve_settings_path,
    save_settings,
)
from wiris_stereo_distance.stereo import (
    DetectionObservation,
    correspondence_rejection_reason,
    detection_timestamp_for_basis,
)
from wiris_stereo_distance.theme import (
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
from wiris_stereo_distance.sync import (
    ATTITUDE_STALE_AGE_S,
    ATTITUDE_STALE_PAIR_DELTA_S,
    ATTITUDE_TIMING_POLICY,
    ATTITUDE_VERY_STALE_AGE_S,
    ATTITUDE_VERY_STALE_PAIR_DELTA_S,
    TimedSampleBuffer,
    classify_attitude_timing,
    match_nearest_pair,
)
from wiris_stereo_distance.triangulation import (
    RayTriangulationResult,
    body_frd_vector_to_camera,
    camera_translation_right_in_left_camera,
    invert_left_to_right_camera_transform,
    world_ned_vector_to_body_frd,
)
from wiris_stereo_distance.xrce_agent import ensure_micro_xrce_agent
from wiris_stereo_distance.yolo_stream import (
    DECODER_AUTO,
    DECODER_NVIDIA,
    DECODER_SOFTWARE,
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

from wiris_stereo_distance.wiris_zoom import (
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
    build_wiris_command,
    build_zoom_command,
    send_tcp_line,
)


def _new_measurement_attempt_generation_id(session_id: str = "") -> str:
    """Return a collision-resistant generation identity independent of wall-clock resolution."""

    return f"{session_id or 'runtime'}-{uuid.uuid4().hex}"


def angle_diff_deg(a_deg: float, b_deg: float) -> float:
    """Return wrapped angular difference a-b in [-180, 180) degrees."""
    return (float(a_deg) - float(b_deg) + 180.0) % 360.0 - 180.0


class MavlinkParameterStore:
    def __init__(self, subscriber: DualPixhawkMavlinkSubscriber, camera: str) -> None:
        self.subscriber = subscriber
        self.camera = camera

    def get_param(self, name: str) -> int | float | None:
        return self.subscriber.read_param_int(self.camera, name, timeout_s=1.5)

    def set_param(self, name: str, value: int | float) -> None:
        expected = int(value)
        actual = self.subscriber.set_param_int(self.camera, name, expected, timeout_s=2.0)
        if actual != expected:
            raise RuntimeError(f"{name} readback mismatch: expected={expected}, actual={actual}")


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
STARTUP_PIXHAWK_STATUS = "픽스호크: 대기 중 - 장비 연결 후 ROS2 연결 또는 MAVLink 연결"
STARTUP_READY_MESSAGE = (
    "[연결] 카메라/픽스호크 미연결 상태에서도 앱을 사용할 수 있습니다. "
    "장비 연결 후 영상 수신 확인, 카메라 Preview 연결, ROS2 연결 또는 MAVLink 연결을 누르세요."
)
YOLO_CPU_DEVICE = "cpu"
YOLO_CUDA_DEVICE = "cuda:0"
WIRIS_VISIBLE_ZOOM_FX_ESTIMATES = (
    (0, 1.0, 1030),
    (1, 1.2, 1236),
    (2, 1.5, 1545),
    (3, 2.0, 2060),
    (4, 3.0, 3091),
    (5, 4.0, 4121),
    (6, 5.0, 5151),
    (7, 6.0, 6181),
    (8, 8.0, 8242),
    (9, 10.0, 10302),
    (10, 12.0, 12362),
    (11, 16.0, 16483),
    (12, 20.0, 20604),
    (13, 25.0, 25755),
    (14, 30.0, 30906),
)
MAX_MANUAL_FOCAL_PX = 100_000.0
RTSP_ENABLE_SETTLE_DELAY_MS = 4000
RTSP_ENABLE_PORT_WAIT_INTERVAL_MS = 1000
RTSP_ENABLE_PORT_WAIT_MAX_ATTEMPTS = 12
DETECTION_PAIR_MAX_AGE_S = 1.0
CHECKERBOARD_STEREO_PAIR_MAX_DELTA_S = 0.050
CAMERA_RTCP_PAIR_MAX_DELTA_S = 0.050
CAMERA_RTCP_MAX_PC_OFFSET_S = 0.100
CAMERA_SYNC_HISTORY_MAX_AGE_S = 2.0
CAMERA_SYNC_RECEIVE_PAIR_MAX_DELTA_S = 0.200
RUNTIME_CAMERA_STATUS_LABEL_HEIGHT_PX = 32
RUNTIME_CAMERA_STATUS_GROUP_HEIGHT_PX = 48
VISIBLE_STREAM_GOP_GET_COMMAND = "GEGS VISIBLE"
WIRIS_FTP_IMAGE_EXTENSIONS = (".jpg", ".jpeg", ".tif", ".tiff")
RECORDED_SAMPLE_TABLE_COLUMNS = (
    ("timestamp_s", "시간(s)"),
    ("distance_m", "YOLO Range(m)"),
    ("method", "방식"),
    ("quality", "품질"),
    ("attitude_timing_status", "자세 시각"),
    ("disparity_px", "Disparity(px)"),
    ("camera_delta_s", "좌우Δt(s)"),
    ("vertical_delta_px", "세로Δ(px)"),
    ("baseline_source", "Baseline 출처"),
    ("baseline_m", "Baseline(m)"),
    ("relative_roll_deg", "상대 Roll(deg)"),
    ("relative_pitch_deg", "상대 Pitch(deg)"),
    ("relative_yaw_deg", "상대 Yaw(deg)"),
    ("left_roll_deg", "L Roll(deg)"),
    ("left_pitch_deg", "L Pitch(deg)"),
    ("left_yaw_deg", "L Yaw(deg)"),
    ("right_roll_deg", "R Roll(deg)"),
    ("right_pitch_deg", "R Pitch(deg)"),
    ("right_yaw_deg", "R Yaw(deg)"),
    ("rtk_fix_quality", "RTK 품질"),
    ("left_gps_satellites_used", "L Sats"),
    ("left_gps_eph_m", "L eph(m)"),
    ("left_gps_epv_m", "L epv(m)"),
    ("right_gps_satellites_used", "R Sats"),
    ("right_gps_eph_m", "R eph(m)"),
    ("right_gps_epv_m", "R epv(m)"),
    ("intrinsics_zoom_level", "줌"),
    ("intrinsics_path", "Intrinsics"),
    ("baseline_center_forward_m", "Center Forward(m)"),
    ("baseline_center_right_m", "Center Right(m)"),
    ("baseline_center_down_m", "Center Down(m)"),
    ("baseline_center_range_m", "Center Range(m)"),
    ("left_cx", "L cx(px)"),
    ("right_cx", "R cx(px)"),
    ("bbox_mid_timestamp_s", "BBox mid t(s)"),
    ("bbox_mid_utc_unix_s", "BBox UTC(s)"),
    ("camera_time_sync_status", "카메라 시간 상태"),
    ("rtcp_pair_delta_s", "RTCP 좌우Δt(s)"),
    ("rtcp_pc_offset_max_ms", "RTCP-PC 최대Δ(ms)"),
)
RECORDING_PREVIEW_LIMIT = 256
RECORDING_KEY_CACHE_LIMIT = 1024


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

WIRIS_COMMAND_CATALOG = (
    ("Activation", "Check SDK activation", "IACT", LINE_TERMINATOR, "SDK 활성화 상태를 TRUE/FALSE로 읽습니다."),
    ("Activation", "Activate SDK", "ACTV <ACTIVATION-CODE>", LINE_TERMINATOR, "활성화 코드로 SDK를 활성화합니다. 전송 전에 실제 코드로 바꾸세요."),
    ("Activation", "Get Ethernet Mode", "GETH", LINE_TERMINATOR, "Ethernet/RTSP 모드 상태를 TRUE/FALSE로 읽습니다."),
    ("Activation", "Set Ethernet Mode ON", "SETH TRUE", LINE_TERMINATOR, "Ethernet/RTSP 모드를 켜서 영상 송출을 시작합니다."),
    ("Activation", "Set Ethernet Mode OFF", "SETH FALSE", LINE_TERMINATOR, "Ethernet/RTSP 모드를 끕니다."),
    ("Activation", "Set SDK Port", "SSDK <port>", LINE_TERMINATOR, "SDK 제어 TCP 포트를 변경합니다."),
    ("Basic", "Check connection", "HIWS", LINE_TERMINATOR, "연결 확인 명령입니다. 정상 응답은 OK입니다."),
    ("Basic", "Set delimiter NULL", "SDLM NULL", LINE_TERMINATOR, "명령 구분자를 NULL로 설정합니다."),
    ("Basic", "Set delimiter LINE", "SDLM LINE", LINE_TERMINATOR, "명령 구분자를 줄바꿈 문자로 설정합니다."),
    ("Basic", "Get delimiter", "GDLM", LINE_TERMINATOR, "현재 명령 구분자 설정을 읽습니다."),
    ("Basic", "Get serial number", "GSRN", LINE_TERMINATOR, "카메라 시리얼 번호를 읽습니다."),
    ("Basic", "Get article number", "GATN", LINE_TERMINATOR, "카메라 article 번호를 읽습니다."),
    ("Basic", "Get firmware version", "GFWV", LINE_TERMINATOR, "카메라 펌웨어 버전을 읽습니다."),
    ("Basic", "Get CPU temperature", "GTCU", LINE_TERMINATOR, "CPU 온도를 읽습니다."),
    ("Basic", "Get internal core temperature", "GTIC", LINE_TERMINATOR, "카메라 내부 core 온도를 읽습니다."),
    ("Basic", "Get inside temperature", "GTIN", LINE_TERMINATOR, "카메라 내부 온도를 읽습니다."),
    ("Basic", "Get fan power", "GFPW", LINE_TERMINATOR, "팬 출력 상태를 읽습니다."),
    ("Date/Time", "Get date/time", "GDTI", LINE_TERMINATOR, "현재 카메라 날짜/시간을 읽습니다."),
    (
        "Date/Time",
        "Set Unix time",
        "STUS <unix_seconds> <microseconds>",
        LINE_TERMINATOR,
        "카메라 시간을 Unix epoch 초+마이크로초로 설정합니다. 재부팅 후 유지되지 않을 수 있습니다.",
    ),
    (
        "Date/Time",
        "Set date/time text",
        "SDTI yyyy/MM/dd-hh:mm:ss",
        LINE_TERMINATOR,
        "카메라 시간을 날짜 문자열로 설정합니다. 재부팅 후 유지되지 않을 수 있습니다.",
    ),
    ("Menu", "Move up", "MOVE UP", LINE_TERMINATOR, "카메라 메뉴에서 위로 이동합니다."),
    ("Menu", "Move down", "MOVE DOWN", LINE_TERMINATOR, "카메라 메뉴에서 아래로 이동합니다."),
    ("Menu", "Move OK", "MOVE OK", LINE_TERMINATOR, "카메라 메뉴에서 현재 항목을 확인합니다."),
    ("Menu", "Move cancel", "MOVE CANCEL", LINE_TERMINATOR, "카메라 메뉴에서 취소하거나 뒤로 이동합니다."),
    ("GPS", "Get GPS coordinates", "GGPS", LINE_TERMINATOR, "사용 가능할 때 카메라 GPS 좌표를 읽습니다."),
    ("Appearance", "Set main camera thermal", "SMCA THERMO", LINE_TERMINATOR, "열화상 카메라를 메인 카메라로 설정합니다."),
    ("Appearance", "Set main camera visible", "SMCA VISIBLE", LINE_TERMINATOR, "가시광 카메라를 메인 카메라로 설정합니다."),
    ("Appearance", "Get main camera", "GMCA", LINE_TERMINATOR, "현재 메인 카메라 설정을 읽습니다."),
    ("Appearance", "Set GPS info position", "SGPI POSITION", LINE_TERMINATOR, "화면 GPS 정보 표시를 위치 정보로 설정합니다."),
    ("Appearance", "Set GPS info altitude/speed", "SGPI ALTITUDE_SPEED", LINE_TERMINATOR, "화면 GPS 정보 표시를 고도/속도 정보로 설정합니다."),
    ("Appearance", "Get GPS info mode", "GGPI", LINE_TERMINATOR, "현재 GPS 정보 표시 모드를 읽습니다."),
    ("Zoom", "Zoom in", "SZIN", LINE_TERMINATOR, "선택한 카메라를 줌 인합니다."),
    ("Zoom", "Zoom out", "SZOT", LINE_TERMINATOR, "선택한 카메라를 줌 아웃합니다."),
    ("Zoom", "Stop focus/zoom action", "SGFA", LINE_TERMINATOR, "지원되는 경우 진행 중인 포커스/줌 동작을 중지합니다."),
    ("Zoom", "Set simultaneous zoom ON", "SZSM TRUE", LINE_TERMINATOR, "열화상/가시광 카메라를 동시에 줌하도록 설정합니다."),
    ("Zoom", "Set simultaneous zoom OFF", "SZSM FALSE", LINE_TERMINATOR, "열화상/가시광 동시 줌을 끕니다."),
    ("Zoom", "Get simultaneous zoom", "GZSM", LINE_TERMINATOR, "동시 줌 설정 상태를 읽습니다."),
    ("Zoom", "Set thermal zoom", "SZTN <index>", LINE_TERMINATOR, "열화상 줌 값을 설정합니다. <index>를 지원 값으로 바꾸세요."),
    ("Zoom", "Get thermal zoom", "GZTV", LINE_TERMINATOR, "현재 열화상 줌 값을 읽습니다."),
    ("Zoom", "Get thermal zoom list", "GZTL", LINE_TERMINATOR, "지원되는 열화상 줌 값 목록을 읽습니다."),
    ("Zoom", "Set visible zoom", "SZVN <index>", LINE_TERMINATOR, "가시광 줌 값을 설정합니다. <index>를 지원 값으로 바꾸세요."),
    ("Zoom", "Get visible zoom", "GZVV", LINE_TERMINATOR, "현재 가시광 줌 값을 읽습니다."),
    ("Capture/Record", "Capture image", "CPTR", LINE_TERMINATOR, "카메라 자체 이미지 캡처를 트리거합니다."),
    ("Capture/Record", "Is capturing", "ICPT", LINE_TERMINATOR, "카메라 캡처 진행 상태를 읽습니다."),
    ("Capture/Record", "Recording start", "RCRS", LINE_TERMINATOR, "카메라 자체 녹화를 시작합니다."),
    ("Capture/Record", "Recording finish", "RCRF", LINE_TERMINATOR, "카메라 자체 녹화를 종료합니다."),
    ("Capture/Record", "Is recording", "IRCR", LINE_TERMINATOR, "카메라 녹화 상태를 읽습니다."),
    ("Capture/Record", "Set periodic capture", "SPIC <period>", LINE_TERMINATOR, "주기 캡처 값을 설정합니다. <period>를 SDK 지원 값으로 바꾸세요."),
    ("Capture/Record", "Get periodic capture", "GPIC", LINE_TERMINATOR, "현재 주기 캡처 설정을 읽습니다."),
    ("Stream", "Get thermal resolution", "GTRE", LINE_TERMINATOR, "열화상 스트림 해상도를 읽습니다."),
    ("Stream", "Set encoder bitrate", "SEBR VISIBLE <bitrate>", LINE_TERMINATOR, "가시광 스트림/녹화 인코더 bitrate를 설정합니다."),
    ("Stream", "Get encoder bitrate", "GEBR VISIBLE", LINE_TERMINATOR, "가시광 인코더 bitrate 설정을 읽습니다."),
    ("Stream", "Set GOP size", "SEGS VISIBLE <gop>", LINE_TERMINATOR, "가시광 인코더 GOP 크기를 설정합니다."),
    ("Stream", "Get GOP size", "GEGS VISIBLE", LINE_TERMINATOR, "가시광 인코더 GOP 크기를 읽습니다."),
    ("Stream", "Set IDR interval", "SEIR VISIBLE <idr>", LINE_TERMINATOR, "가시광 인코더 IDR 간격을 설정합니다."),
    ("Stream", "Get IDR interval", "GEIR VISIBLE", LINE_TERMINATOR, "가시광 인코더 IDR 간격을 읽습니다."),
    ("Stream", "Set quantization quality", "SEQZ VISIBLE <qp>", LINE_TERMINATOR, "가시광 인코더 QP 값을 설정합니다. 범위는 0-51입니다."),
    ("Stream", "Get quantization quality", "GEQZ VISIBLE", LINE_TERMINATOR, "가시광 인코더 QP 설정을 읽습니다."),
    ("Trigger", "Set trigger correction", "STRG CORRECTION", LINE_TERMINATOR, "외부 trigger 입력을 보정 동작으로 설정합니다."),
    ("Trigger", "Set trigger capture", "STRG CAPTURE", LINE_TERMINATOR, "외부 trigger 입력을 이미지 캡처 동작으로 설정합니다."),
    ("Trigger", "Set trigger record", "STRG RECORD", LINE_TERMINATOR, "외부 trigger 입력을 녹화 제어 동작으로 설정합니다."),
    ("Trigger", "Disable trigger action", "STRG NA", LINE_TERMINATOR, "외부 trigger 동작을 사용하지 않도록 설정합니다."),
    ("Trigger", "Get trigger action", "GTRG", LINE_TERMINATOR, "현재 외부 trigger 동작 설정을 읽습니다."),
    ("System", "Set IP address", "STIP <IPv4>", LINE_TERMINATOR, "카메라 IP 주소를 변경합니다."),
    ("System", "Set subnet mask", "STMS <mask>", LINE_TERMINATOR, "카메라 서브넷 마스크를 변경합니다."),
    ("System", "Get subnet mask", "GTMS", LINE_TERMINATOR, "현재 서브넷 마스크를 읽습니다."),
    ("System", "Set gateway", "STGW <gateway>", LINE_TERMINATOR, "카메라 gateway 주소를 변경합니다."),
    ("System", "Get gateway", "GTGW", LINE_TERMINATOR, "현재 gateway 주소를 읽습니다."),
    ("System", "Manual gateway ON", "SMGW TRUE", LINE_TERMINATOR, "수동 gateway 설정을 사용합니다."),
    ("System", "Manual gateway OFF", "SMGW FALSE", LINE_TERMINATOR, "수동 gateway 설정을 끕니다."),
    ("System", "Get manual gateway", "GMGW", LINE_TERMINATOR, "수동 gateway 사용 여부를 읽습니다."),
    ("System", "Reboot", "REBT", LINE_TERMINATOR, "카메라를 재부팅합니다."),
    ("Custom", "Custom SDK command", "", LINE_TERMINATOR, "SDK 명령을 직접 입력합니다."),
)
WIRIS_DANGEROUS_COMMAND_PREFIXES = (
    "ACTV",
    "SSDK",
    "SETH FALSE",
    "STIP",
    "STMS",
    "STGW",
    "SMGW",
    "STUS",
    "SDTI",
    "REBT",
    "RCRS",
    "RCRF",
)


def camera_resolution_text(stats: dict) -> str:
    try:
        width = int(stats.get("width", 0))
        height = int(stats.get("height", 0))
    except (TypeError, ValueError):
        return ""
    if width <= 0 or height <= 0:
        return ""
    return f" @ {width}x{height}"


def camera_display_name(camera: str) -> str:
    return {"left": "왼쪽", "right": "오른쪽"}.get(camera, camera)


def format_camera_stream_stats(camera: str, state: str, stats: dict) -> str:
    camera_name = camera_display_name(camera)
    state_text = {"connected": "연결", "stale": "지연", "waiting": "대기"}.get(state, state)
    decoded_hz = float(stats.get("decoded_hz", 0.0))
    inferred_hz = float(stats.get("inferred_hz", 0.0))
    resolution = camera_resolution_text(stats)
    resolution_text = resolution.removeprefix(" @ ")
    if stats.get("enable_detection"):
        receive_rate_text = f"{resolution_text} 수신 {decoded_hz:.1f} Hz".strip()
        return f"{camera_name}={state_text} {receive_rate_text}, YOLO {inferred_hz:.1f} Hz"
    receive_rate_text = f"{resolution_text} 수신 {decoded_hz:.1f} Hz".strip()
    return f"{camera_name}={state_text} {receive_rate_text}"


def format_runtime_camera_stats(camera: str, stats: dict, fallback_hz: float, *, stale: bool) -> str:
    label = camera_display_name(camera)
    resolution = camera_resolution_text(stats) or " @ --"
    has_stats = bool(stats)
    decoded_hz = float(stats.get("decoded_hz", 0.0)) if has_stats else 0.0
    if has_stats:
        stream_hz_text = f"{resolution.removeprefix(' @ ')} {decoded_hz:.1f} Hz".strip()
        yolo_hz_text = f"{float(stats.get('inferred_hz', 0.0)):.1f} Hz"
    else:
        stream_hz_text = resolution.removeprefix(" @ ") or "--"
        yolo_hz_text = "--"
    state_text = " | 지연" if stale else ""
    return (
        f"{label}: {stream_hz_text} | "
        f"{yolo_hz_text}{format_runtime_yolo_device_text(stats)}{state_text}"
    )


def format_runtime_camera_status(camera: str, stats: dict, metadata: dict, fallback_hz: float, *, stale: bool) -> str:
    return f"{format_runtime_camera_stats(camera, stats, fallback_hz, stale=stale)} | 시각: {format_runtime_camera_timestamp(metadata)}"


def format_runtime_yolo_device_text(stats: dict) -> str:
    if not stats or not stats.get("enable_detection"):
        return ""
    actual_device = str(stats.get("yolo_actual_device") or "").strip()
    if not actual_device:
        return " (장치 대기)"
    normalized = actual_device.lower()
    if normalized == "cpu":
        return " (CPU fallback)" if stats.get("yolo_cpu_fallback") else " (CPU)"
    if normalized.startswith("cuda"):
        return " (CUDA)"
    return f" ({actual_device})"


def infer_runtime_camera_timestamp_reference(metadata: dict) -> str:
    reference = (
        str(metadata.get("timestamp_reference") or "")
        .strip()
        .lower()
        .replace("_", "-")
        .replace("/", "-")
        .replace(" ", "-")
    )
    if reference:
        return reference
    if metadata.get("rtcp_timing_invalid_reason"):
        return "rtcp-invalid"
    if metadata.get("rtcp_frame_ntp_unix_s") is not None:
        return "rtcp-ntp"
    if metadata.get("source_timestamp_s") is not None:
        return "source"
    return "app-receive"


def format_runtime_camera_timestamp(metadata: dict) -> str:
    frame_ntp_unix_s = metadata.get("rtcp_frame_ntp_unix_s")
    timestamp_reference = infer_runtime_camera_timestamp_reference(metadata)
    if timestamp_reference == "rtcp-invalid":
        return "RTCP invalid (SR-PC offset)"
    if frame_ntp_unix_s is not None:
        try:
            utc_text = datetime.fromtimestamp(float(frame_ntp_unix_s), tz=timezone.utc).isoformat(timespec="microseconds")
        except (TypeError, ValueError, OverflowError, OSError):
            utc_text = ""
        if utc_text:
            if timestamp_reference == "gnss-utc":
                return f"UTC {utc_text.replace('+00:00', 'Z')}"
            return f"RTCP NTP {utc_text.replace('+00:00', 'Z')}"
    source_timestamp_s = metadata.get("source_timestamp_s")
    if source_timestamp_s is not None:
        source_source = str(metadata.get("timestamp_source") or "source")
        return f"src {float(source_timestamp_s):.6f} s [{source_source}]"
    return "missing"


def format_runtime_camera_timestamp_compact(metadata: dict) -> str:
    frame_ntp_unix_s = metadata.get("rtcp_frame_ntp_unix_s")
    timestamp_reference = infer_runtime_camera_timestamp_reference(metadata)
    if timestamp_reference == "rtcp-invalid":
        return "RTCP invalid"
    if frame_ntp_unix_s is not None:
        try:
            utc_dt = datetime.fromtimestamp(float(frame_ntp_unix_s), tz=timezone.utc)
        except (TypeError, ValueError, OverflowError, OSError):
            utc_dt = None
        if utc_dt is not None:
            prefix = "UTC" if timestamp_reference == "gnss-utc" else "RTCP"
            return f"{prefix} {utc_dt.strftime('%H:%M:%S.%f')[:-3]}Z"
    source_timestamp_s = metadata.get("source_timestamp_s")
    if source_timestamp_s is not None:
        source_source = str(metadata.get("timestamp_source") or "source")
        return f"src {float(source_timestamp_s):.3f}s [{source_source}]"
    return "missing"


def format_runtime_camera_ntp_compact(metadata: dict) -> str:
    frame_ntp_unix_s = metadata.get("rtcp_frame_ntp_unix_s")
    if frame_ntp_unix_s is None:
        return "--"
    try:
        utc_dt = datetime.fromtimestamp(float(frame_ntp_unix_s), tz=timezone.utc)
    except (TypeError, ValueError, OverflowError, OSError):
        return "invalid"
    return f"{utc_dt.strftime('%H:%M:%S.%f')[:-3]}Z"


@dataclass(frozen=True)
class AttitudeHistoryPoint:
    timestamp_s: float
    left_deg: float | None = None
    right_deg: float | None = None


class AttitudeHistoryPlot(QtWidgets.QWidget):
    def __init__(self, axis_name: str) -> None:
        super().__init__()
        self.axis_name = axis_name
        self.points: deque[AttitudeHistoryPoint] = deque(maxlen=600)
        self.setMinimumHeight(150)
        self.setSizePolicy(QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Expanding)
        self.setStyleSheet(f"background: {CHART_BG}; border: 1px solid {PANEL_BORDER};")

    def add_point(self, point: AttitudeHistoryPoint) -> None:
        self.points.append(point)
        self.update()

    @staticmethod
    def auto_y_bounds(values: list[float]) -> tuple[float, float]:
        min_value = min(values)
        max_value = max(values)
        if min_value == max_value:
            padding = max(0.0001, abs(min_value) * 0.05)
            return min_value - padding, max_value + padding
        return min_value, max_value

    def paintEvent(self, event) -> None:  # noqa: N802 - Qt override.
        super().paintEvent(event)
        painter = QtGui.QPainter(self)
        painter.setRenderHint(QtGui.QPainter.Antialiasing)
        painter.fillRect(self.rect(), QtGui.QColor(CHART_BG))
        rect = self.rect().adjusted(50, 18, -18, -34)
        painter.setPen(QtGui.QPen(QtGui.QColor(CHART_AXIS), 1))
        painter.drawRect(rect)

        if len(self.points) < 2:
            painter.setPen(QtGui.QColor(CHART_MUTED))
            painter.drawText(self.rect(), QtCore.Qt.AlignCenter, f"{self.axis_name}: 픽스호크 샘플 대기 중")
            painter.end()
            return

        values = [
            value
            for point in self.points
            for value in (point.left_deg, point.right_deg)
            if value is not None and isfinite(value)
        ]
        if not values:
            painter.setPen(QtGui.QColor(CHART_MUTED))
            painter.drawText(self.rect(), QtCore.Qt.AlignCenter, f"{self.axis_name}: 픽스호크 샘플 대기 중")
            painter.end()
            return
        min_value, max_value = self.auto_y_bounds(values)
        start_s = self.points[0].timestamp_s
        end_s = max(self.points[-1].timestamp_s, start_s + 1.0)

        def map_xy(timestamp_s: float, value: float) -> QtCore.QPointF:
            x_ratio = (timestamp_s - start_s) / (end_s - start_s)
            y_ratio = (value - min_value) / (max_value - min_value)
            return QtCore.QPointF(rect.left() + x_ratio * rect.width(), rect.bottom() - y_ratio * rect.height())

        painter.setPen(QtGui.QPen(QtGui.QColor(CHART_GRID), 1))
        mid_y = rect.center().y()
        painter.drawLine(rect.left(), mid_y, rect.right(), mid_y)
        for i in range(1, 4):
            y = rect.top() + rect.height() * i / 4
            painter.drawLine(rect.left(), int(y), rect.right(), int(y))

        painter.setPen(QtGui.QColor(CHART_TEXT))
        painter.drawText(rect.left(), rect.top() - 4, self.axis_name)

        series = (
            ("Pixhawk 1", SERIES_LEFT, lambda point: point.left_deg),
            ("Pixhawk 2", SERIES_RIGHT, lambda point: point.right_deg),
        )
        painter.save()
        painter.setClipRect(rect)
        for name, color, getter in series:
            painter.setPen(QtGui.QPen(QtGui.QColor(color), 2))
            path = QtGui.QPainterPath()
            path_started = False
            for point in self.points:
                value = getter(point)
                if value is None or not isfinite(value):
                    path_started = False
                    continue
                mapped = map_xy(point.timestamp_s, value)
                if not path_started:
                    path.moveTo(mapped)
                    path_started = True
                else:
                    path.lineTo(mapped)
            painter.drawPath(path)
        painter.restore()

        painter.setPen(QtGui.QColor(CHART_TEXT))
        painter.drawText(8, rect.top() + 6, f"{max_value:.4f} deg")
        painter.drawText(8, rect.bottom(), f"{min_value:.4f} deg")
        legend_x = rect.left()
        for name, color, _getter in series:
            painter.setPen(QtGui.QColor(color))
            painter.drawText(legend_x, self.height() - 10, name)
            legend_x += 58
        painter.setPen(QtGui.QColor(CHART_TEXT))
        painter.drawText(rect.right() - 130, self.height() - 10, f"{end_s - start_s:.1f}s window")
        painter.end()


class Px4ShellDialog(QtWidgets.QDialog):
    def __init__(self, title: str, send_callback) -> None:
        super().__init__()
        self.send_callback = send_callback
        self.setWindowTitle(title)
        self.resize(760, 420)
        layout = QtWidgets.QVBoxLayout(self)
        self.output = QtWidgets.QPlainTextEdit()
        self.output.setReadOnly(True)
        self.output.setStyleSheet(
            f"background: {INVERSE_BG}; color: {INVERSE_TEXT}; font-family: monospace; "
            f"border: 1px solid {PANEL_BORDER_STRONG};"
        )
        command_row = QtWidgets.QHBoxLayout()
        self.command_edit = QtWidgets.QLineEdit()
        self.command_edit.setPlaceholderText("PX4 shell 명령")
        self.command_edit.returnPressed.connect(self.send_command)
        self.send_button = QtWidgets.QPushButton("전송")
        self.send_button.clicked.connect(self.send_command)
        command_row.addWidget(self.command_edit, 1)
        command_row.addWidget(self.send_button)
        layout.addWidget(self.output, 1)
        layout.addLayout(command_row)

    def append_output(self, text: str) -> None:
        self.output.moveCursor(QtGui.QTextCursor.End)
        self.output.insertPlainText(text)
        self.output.moveCursor(QtGui.QTextCursor.End)

    def send_command(self) -> None:
        command = self.command_edit.text().strip()
        if not command:
            return
        self.append_output(f"\n> {command}\n")
        self.command_edit.clear()
        self.send_callback(command)


@dataclass(frozen=True)
class TelemetryHistoryPoint:
    timestamp_s: float
    series: str
    value: float


class TelemetryHistoryPlot(QtWidgets.QWidget):
    def __init__(self, title: str, y_unit: str) -> None:
        super().__init__()
        self.title = title
        self.y_unit = y_unit
        self.points: deque[TelemetryHistoryPoint] = deque(maxlen=600)
        self.series_labels: dict[str, str] = {
            "Δnorm": "Delta Norm",
            "angle": "Angle",
        }
        self.series_order: dict[str, int] = {
            "Δnorm": 50,
            "angle": 51,
        }
        self.setMouseTracking(True)
        self.setMinimumHeight(200)
        self.setSizePolicy(QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Expanding)
        self.setStyleSheet(f"background: {CHART_BG}; border: 1px solid {PANEL_BORDER};")

    def add_point(self, point: TelemetryHistoryPoint) -> None:
        self.points.append(point)
        self.update()

    def set_series_labels(self, labels: dict[str, str]) -> None:
        self.series_labels.update(labels)
        self.update()

    def remove_series(self, series: str) -> None:
        self.points = deque((point for point in self.points if point.series != series), maxlen=self.points.maxlen)
        self.update()

    def _plot_rect(self) -> QtCore.QRect:
        return self.rect().adjusted(54, 24, -18, -34)

    def _series_names(self) -> list[str]:
        return sorted({point.series for point in self.points}, key=lambda name: (self.series_order.get(name, 1000), name))

    def _series_color_map(self) -> dict[str, str]:
        colors = [SERIES_LEFT, SERIES_RIGHT, SERIES_Z, SERIES_DISTANCE]
        return {
            series_name: colors[index % len(colors)]
            for index, series_name in enumerate(self._series_names())
        }

    def _series_label(self, series_name: str) -> str:
        return self.series_labels.get(series_name, series_name)

    def _value_bounds(self) -> tuple[float, float]:
        values = [point.value for point in self.points]
        min_value = min(values)
        max_value = max(values)
        if max_value - min_value < 1e-6:
            min_value -= 1.0
            max_value += 1.0
        return min_value, max_value

    def _time_bounds(self) -> tuple[float, float]:
        start_s = self.points[0].timestamp_s
        end_s = max(self.points[-1].timestamp_s, start_s + 1.0)
        return start_s, end_s

    def _map_point(
        self,
        point: TelemetryHistoryPoint,
        rect: QtCore.QRect,
        start_s: float,
        end_s: float,
        min_value: float,
        max_value: float,
    ) -> QtCore.QPointF:
        x_ratio = (point.timestamp_s - start_s) / (end_s - start_s)
        y_ratio = (point.value - min_value) / (max_value - min_value)
        return QtCore.QPointF(rect.left() + x_ratio * rect.width(), rect.bottom() - y_ratio * rect.height())

    def _format_hover_text(self, point: TelemetryHistoryPoint) -> str:
        return f"{self._series_label(point.series)}\nt={point.timestamp_s:.3f}s\nvalue={point.value:.3f} {self.y_unit}"

    def _nearest_point_at(self, position: QtCore.QPoint) -> TelemetryHistoryPoint | None:
        if len(self.points) < 2:
            return None
        rect = self._plot_rect()
        if not rect.adjusted(-8, -8, 8, 8).contains(position):
            return None
        min_value, max_value = self._value_bounds()
        start_s, end_s = self._time_bounds()
        best_point: TelemetryHistoryPoint | None = None
        best_distance_sq = 14.0 * 14.0
        for point in self.points:
            mapped = self._map_point(point, rect, start_s, end_s, min_value, max_value)
            dx = mapped.x() - position.x()
            dy = mapped.y() - position.y()
            distance_sq = dx * dx + dy * dy
            if distance_sq <= best_distance_sq:
                best_distance_sq = distance_sq
                best_point = point
        return best_point

    def mouseMoveEvent(self, event) -> None:  # noqa: N802 - Qt override.
        point = self._nearest_point_at(event.pos())
        if point is None:
            QtWidgets.QToolTip.hideText()
            return
        QtWidgets.QToolTip.showText(event.globalPos(), self._format_hover_text(point), self)

    def leaveEvent(self, event) -> None:  # noqa: N802 - Qt override.
        QtWidgets.QToolTip.hideText()
        super().leaveEvent(event)

    def _draw_legend(self, painter: QtGui.QPainter, rect: QtCore.QRect, series_colors: dict[str, str]) -> None:
        if not series_colors:
            return
        font = painter.font()
        font.setPointSize(8)
        painter.setFont(font)
        metrics = QtGui.QFontMetrics(font)
        entries = [(self._series_label(series), color) for series, color in series_colors.items()]
        row_h = max(12, metrics.height())
        width = min(
            max(metrics.horizontalAdvance(label) for label, _color in entries) + 24,
            max(74, rect.width() // 2),
        )
        height = row_h * len(entries) + 8
        legend = QtCore.QRect(rect.right() - width - 6, rect.top() + 6, width, height)
        painter.fillRect(legend, QtGui.QColor(244, 244, 244, 220))
        painter.setPen(QtGui.QPen(QtGui.QColor(CHART_AXIS), 1))
        painter.drawRect(legend)
        y = legend.top() + 5 + metrics.ascent()
        for label, color in entries:
            painter.setPen(QtGui.QColor(color))
            painter.drawText(legend.left() + 7, y, "●")
            painter.setPen(QtGui.QColor(CHART_TEXT))
            painter.drawText(legend.left() + 20, y, label)
            y += row_h

    def paintEvent(self, event) -> None:  # noqa: N802 - Qt override.
        super().paintEvent(event)
        painter = QtGui.QPainter(self)
        painter.setRenderHint(QtGui.QPainter.Antialiasing)
        painter.fillRect(self.rect(), QtGui.QColor(CHART_BG))
        rect = self._plot_rect()
        painter.setPen(QtGui.QPen(QtGui.QColor(CHART_AXIS), 1))
        painter.drawRect(rect)
        painter.setPen(QtGui.QColor(CHART_TEXT))
        painter.drawText(10, 18, self.title)

        if len(self.points) < 2:
            painter.drawText(self.rect(), QtCore.Qt.AlignCenter, "샘플 대기 중")
            painter.end()
            return

        min_value, max_value = self._value_bounds()
        start_s, end_s = self._time_bounds()

        painter.setPen(QtGui.QPen(QtGui.QColor(CHART_GRID), 1))
        for i in range(1, 4):
            y = rect.top() + rect.height() * i / 4
            painter.drawLine(rect.left(), int(y), rect.right(), int(y))

        series_colors = self._series_color_map()
        for series_name in self._series_names():
            series_points = [point for point in self.points if point.series == series_name]
            if len(series_points) < 2:
                continue
            painter.setPen(QtGui.QPen(QtGui.QColor(series_colors[series_name]), 2))
            path = QtGui.QPainterPath(
                self._map_point(series_points[0], rect, start_s, end_s, min_value, max_value)
            )
            for point in series_points[1:]:
                path.lineTo(self._map_point(point, rect, start_s, end_s, min_value, max_value))
            painter.drawPath(path)

        painter.setPen(QtGui.QColor(CHART_TEXT))
        painter.drawText(8, rect.top() + 6, f"{max_value:.2f} {self.y_unit}")
        painter.drawText(8, rect.bottom(), f"{min_value:.2f} {self.y_unit}")
        painter.setPen(QtGui.QColor(CHART_MUTED))
        painter.drawText(rect.left(), rect.bottom() + 18, f"t={start_s:.3f}s")
        painter.drawText(rect.right() - 86, rect.bottom() + 18, f"t={end_s:.3f}s")
        self._draw_legend(painter, rect, series_colors)
        painter.end()


class RtkAlignmentPlot(QtWidgets.QWidget):
    def __init__(self) -> None:
        super().__init__()
        self.baseline: RtkBaselineEstimate | None = None
        self.camera_translation_m: tuple[float, float, float] | None = None
        self.left_attitude: CameraExtrinsicState | None = None
        self.right_attitude: CameraExtrinsicState | None = None
        self.setMinimumHeight(260)
        self.setSizePolicy(QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Expanding)
        self.setStyleSheet(f"background: {CHART_BG}; border: 1px solid {PANEL_BORDER};")

    def set_alignment(
        self,
        baseline: RtkBaselineEstimate | None,
        camera_translation_m: tuple[float, float, float] | None = None,
        left_attitude: CameraExtrinsicState | None = None,
        right_attitude: CameraExtrinsicState | None = None,
    ) -> None:
        self.baseline = baseline
        self.camera_translation_m = camera_translation_m
        self.left_attitude = left_attitude
        self.right_attitude = right_attitude
        self.update()

    def paintEvent(self, event) -> None:  # noqa: N802 - Qt override.
        super().paintEvent(event)
        painter = QtGui.QPainter(self)
        painter.setRenderHint(QtGui.QPainter.Antialiasing)
        painter.fillRect(self.rect(), QtGui.QColor(CHART_BG))
        painter.setPen(QtGui.QColor(CHART_TEXT))
        painter.drawText(12, 22, "GPS baseline 정렬 시각화")

        content = self.rect().adjusted(14, 34, -14, -18)
        if self.baseline is None:
            painter.drawText(content, QtCore.Qt.AlignCenter, "GPS baseline 대기 중")
            painter.end()
            return

        if self.camera_translation_m is None:
            self._draw_gps_only_panel(painter, content)
        else:
            gap = 12
            panel_width = int((content.width() - gap) * 0.62)
            alignment_rect = QtCore.QRect(content.left(), content.top(), panel_width, content.height())
            component_rect = QtCore.QRect(
                alignment_rect.right() + gap,
                content.top(),
                content.right() - alignment_rect.right() - gap,
                content.height(),
            )
            self._draw_forward_alignment_panel(painter, alignment_rect, self.camera_translation_m)
            self._draw_component_panel(painter, component_rect, self.camera_translation_m)
        painter.end()

    def _draw_forward_alignment_panel(
        self,
        painter: QtGui.QPainter,
        rect: QtCore.QRect,
        camera_translation_m: tuple[float, float, float],
    ) -> None:
        right_m, down_m, forward_m = camera_translation_m
        length_m = math.sqrt(right_m * right_m + down_m * down_m + forward_m * forward_m)
        if length_m <= 1e-9:
            self._draw_vector_panel(
                painter,
                rect,
                title="Top view: Forward vs Baseline",
                x_label="Right",
                y_label="Forward",
                x_m=0.0,
                y_m=0.0,
                footer="baseline 대기 중",
                color=SERIES_RIGHT,
            )
            return
        angle_deg = math.degrees(math.acos(max(-1.0, min(1.0, forward_m / length_m))))
        effective_m = math.sqrt(right_m * right_m + down_m * down_m)
        level = self._angle_level(angle_deg)
        status_color = {"good": SUCCESS, "warn": WARNING, "bad": ERROR}[level]

        painter.setBrush(QtCore.Qt.NoBrush)
        painter.setPen(QtGui.QPen(QtGui.QColor(CHART_AXIS), 1))
        painter.drawRect(rect)
        painter.setPen(QtGui.QColor(CHART_TEXT))
        painter.drawText(rect.left() + 8, rect.top() + 18, "상단 보기: P1 RDF Forward vs P1→P2 Baseline")

        plot = rect.adjusted(42, 44, -42, -70)
        center = QtCore.QPointF(plot.center())
        scale_value = max(abs(right_m), abs(forward_m), 0.05)
        scale = 0.42 * min(plot.width(), plot.height()) / scale_value
        baseline_end = QtCore.QPointF(center.x() + right_m * scale, center.y() - forward_m * scale)
        forward_end = QtCore.QPointF(center.x(), center.y() - min(plot.height() * 0.36, 150.0))
        right_end = QtCore.QPointF(center.x() + min(plot.width() * 0.36, 170.0), center.y())

        painter.setPen(QtGui.QPen(QtGui.QColor(CHART_GRID), 1, QtCore.Qt.DashLine))
        painter.drawLine(plot.left(), int(center.y()), plot.right(), int(center.y()))
        painter.drawLine(int(center.x()), plot.top(), int(center.x()), plot.bottom())

        target_rect = QtCore.QRectF(center.x() + 14, center.y() - 44, 40, 40)
        painter.setPen(QtGui.QPen(QtGui.QColor(SUCCESS), 2))
        painter.drawArc(target_rect, 180 * 16, 90 * 16)
        painter.drawText(int(center.x() + 58), int(center.y() - 30), "목표 90°")

        self._draw_arrow(painter, center, forward_end, QtGui.QColor(SERIES_Z))
        painter.setPen(QtGui.QColor(SERIES_Z))
        painter.drawText(forward_end + QtCore.QPointF(8, 4), "Forward +Z")

        self._draw_arrow(painter, center, right_end, QtGui.QColor(CHART_MUTED))
        painter.setPen(QtGui.QColor(CHART_MUTED))
        painter.drawText(right_end + QtCore.QPointF(-48, 18), "Right")

        self._draw_arrow(painter, center, baseline_end, QtGui.QColor(SERIES_RIGHT))
        painter.setBrush(QtGui.QColor(SERIES_RIGHT))
        painter.setPen(QtCore.Qt.NoPen)
        painter.drawEllipse(baseline_end, 5, 5)
        painter.setPen(QtGui.QColor(SERIES_RIGHT))
        painter.drawText(baseline_end + QtCore.QPointF(8, -8), "P1→P2 baseline")

        badge = QtCore.QRect(rect.left() + 8, rect.bottom() - 48, 118, 28)
        painter.fillRect(badge, QtGui.QColor(status_color))
        painter.setPen(QtGui.QColor("#ffffff"))
        painter.drawText(badge, QtCore.Qt.AlignCenter, level.upper())
        painter.setPen(QtGui.QColor(CHART_TEXT))
        painter.drawText(
            badge.right() + 12,
            badge.top() + 19,
            f"angle={angle_deg:.2f}°  effective={effective_m:.3f}/{length_m:.3f} m  forward={forward_m:+.3f} m",
        )

    def _angle_level(self, angle_deg: float) -> str:
        if 80.0 <= angle_deg <= 100.0:
            return "good"
        if 70.0 <= angle_deg <= 110.0:
            return "warn"
        return "bad"

    def _draw_component_panel(
        self,
        painter: QtGui.QPainter,
        rect: QtCore.QRect,
        camera_translation_m: tuple[float, float, float],
    ) -> None:
        right_m, down_m, forward_m = camera_translation_m
        length_m = math.sqrt(right_m * right_m + down_m * down_m + forward_m * forward_m)
        effective_m = math.sqrt(right_m * right_m + down_m * down_m)
        painter.setBrush(QtCore.Qt.NoBrush)
        painter.setPen(QtGui.QPen(QtGui.QColor(CHART_AXIS), 1))
        painter.drawRect(rect)
        painter.setPen(QtGui.QColor(CHART_TEXT))
        painter.drawText(rect.left() + 8, rect.top() + 18, "P1 RDF 기준 baseline 성분")

        rows = (
            ("Right", right_m, SERIES_RIGHT),
            ("Down", down_m, SERIES_LEFT),
            ("Forward", forward_m, ERROR if abs(forward_m) > max(0.05, length_m * 0.15) else SUCCESS),
            ("Effective", effective_m, SERIES_DISTANCE),
        )
        max_value = max(*(abs(value) for _name, value, _color in rows), 0.05)
        bar_left = rect.left() + 88
        bar_right = rect.right() - 18
        zero_x = int((bar_left + bar_right) / 2)
        top = rect.top() + 52
        row_h = 38
        painter.setPen(QtGui.QPen(QtGui.QColor(CHART_GRID), 1))
        painter.drawLine(zero_x, top - 12, zero_x, top + row_h * len(rows) - 4)
        for index, (name, value, color) in enumerate(rows):
            y = top + index * row_h
            painter.setPen(QtGui.QColor(CHART_TEXT))
            painter.drawText(rect.left() + 10, y + 17, name)
            half_width = (bar_right - bar_left) * 0.46
            bar_len = int((abs(value) / max_value) * half_width)
            if value >= 0:
                bar_rect = QtCore.QRect(zero_x, y, bar_len, 20)
            else:
                bar_rect = QtCore.QRect(zero_x - bar_len, y, bar_len, 20)
            painter.fillRect(bar_rect, QtGui.QColor(color))
            painter.setPen(QtGui.QColor(CHART_TEXT))
            painter.drawText(bar_right - 86, y + 17, f"{value:+.3f} m")
        painter.setPen(QtGui.QColor(CHART_MUTED))
        painter.drawText(
            rect.adjusted(10, 0, -10, -10),
            QtCore.Qt.AlignLeft | QtCore.Qt.AlignBottom,
            "Forward≈0, Effective≈전체 baseline이면 좋습니다.",
        )

    def _format_relative_attitude_line(self) -> str:
        if self.left_attitude is None or self.right_attitude is None:
            return "상대 ΔR/P/Y: 대기 중"
        relative = relative_attitude(self.left_attitude.q, self.right_attitude.q)
        return f"상대 ΔR/P/Y: {relative.roll_deg:+.2f}° / {relative.pitch_deg:+.2f}° / {relative.yaw_deg:+.2f}°"

    def _attitude_arrow_components(self, state: CameraExtrinsicState) -> tuple[float, float, float]:
        yaw = max(-1.0, min(1.0, state.yaw_deg / 45.0))
        pitch = max(-1.0, min(1.0, state.pitch_deg / 45.0))
        return yaw, pitch, state.roll_deg

    def _draw_camera_attitude_arrows(self, painter: QtGui.QPainter, rect: QtCore.QRect) -> None:
        states = (
            ("L", self.left_attitude, SERIES_LEFT),
            ("R", self.right_attitude, SERIES_Z),
        )
        if all(state is None for _label, state, _color in states):
            return
        overlay_width = min(238, max(150, rect.width() - 24))
        overlay = QtCore.QRect(rect.right() - overlay_width - 10, rect.top() + 28, overlay_width, 92)
        painter.fillRect(overlay, QtGui.QColor(244, 244, 244, 232))
        painter.setPen(QtGui.QPen(QtGui.QColor(CHART_AXIS), 1))
        painter.drawRect(overlay)
        painter.setPen(QtGui.QColor(CHART_TEXT))
        painter.drawText(overlay.left() + 8, overlay.top() + 16, "자세 화살표")
        for index, (label, state, color) in enumerate(states):
            origin = QtCore.QPointF(overlay.left() + 58 + index * 92, overlay.top() + 58)
            self._draw_single_attitude_glyph(painter, origin, label, state, QtGui.QColor(color))
        painter.setPen(QtGui.QColor(CHART_MUTED))
        painter.drawText(overlay.left() + 8, overlay.bottom() - 8, "광축: yaw/pitch, 원호: roll")

    def _draw_single_attitude_glyph(
        self,
        painter: QtGui.QPainter,
        origin: QtCore.QPointF,
        label: str,
        state: CameraExtrinsicState | None,
        color: QtGui.QColor,
    ) -> None:
        painter.setPen(QtGui.QPen(QtGui.QColor(CHART_AXIS), 1))
        painter.drawLine(QtCore.QPointF(origin.x() - 24, origin.y()), QtCore.QPointF(origin.x() + 24, origin.y()))
        painter.drawLine(QtCore.QPointF(origin.x(), origin.y() - 30), QtCore.QPointF(origin.x(), origin.y() + 8))
        painter.setPen(QtGui.QColor(CHART_TEXT))
        painter.drawText(int(origin.x() - 5), int(origin.y() + 24), label)
        if state is None:
            painter.setPen(QtGui.QColor(CHART_MUTED))
            painter.drawText(int(origin.x() - 16), int(origin.y() - 10), "대기")
            return
        yaw, pitch, roll = self._attitude_arrow_components(state)
        end = QtCore.QPointF(origin.x() + yaw * 24.0, origin.y() - 28.0 - pitch * 18.0)
        self._draw_arrow(painter, origin, end, color)
        self._draw_roll_arc(painter, origin, roll, color)

    def _draw_arrow(self, painter: QtGui.QPainter, start: QtCore.QPointF, end: QtCore.QPointF, color: QtGui.QColor) -> None:
        painter.setPen(QtGui.QPen(color, 2))
        painter.drawLine(start, end)
        angle = math.atan2(end.y() - start.y(), end.x() - start.x())
        head_len = 7.0
        for offset in (math.radians(150), -math.radians(150)):
            point = QtCore.QPointF(
                end.x() + math.cos(angle + offset) * head_len,
                end.y() + math.sin(angle + offset) * head_len,
            )
            painter.drawLine(end, point)

    def _draw_roll_arc(self, painter: QtGui.QPainter, origin: QtCore.QPointF, roll_deg: float, color: QtGui.QColor) -> None:
        radius = 14
        rect = QtCore.QRectF(origin.x() - radius, origin.y() - radius, radius * 2, radius * 2)
        span_deg = max(-120.0, min(120.0, roll_deg))
        if abs(span_deg) < 0.5:
            span_deg = 18.0
        painter.setPen(QtGui.QPen(color, 1))
        painter.drawArc(rect, 35 * 16, int(span_deg * 16))

    def _draw_gps_only_panel(self, painter: QtGui.QPainter, rect: QtCore.QRect) -> None:
        if self.baseline is None:
            return
        up_m = -self.baseline.down_m
        self._draw_vector_panel(
            painter,
            rect,
            title="GPS 전용 E/U",
            x_label="E",
            y_label="U",
            x_m=self.baseline.east_m,
            y_m=up_m,
            footer=f"E={self.baseline.east_m:.3f} m, U={up_m:.3f} m, N={self.baseline.north_m:.3f} m",
            color=SERIES_RIGHT,
        )

    def _draw_vector_panel(
        self,
        painter: QtGui.QPainter,
        rect: QtCore.QRect,
        *,
        title: str,
        x_label: str,
        y_label: str,
        x_m: float,
        y_m: float,
        footer: str,
        color: str,
    ) -> None:
        painter.setPen(QtGui.QPen(QtGui.QColor(CHART_AXIS), 1))
        painter.drawRect(rect)
        painter.setPen(QtGui.QColor(CHART_TEXT))
        painter.drawText(rect.left() + 8, rect.top() + 18, title)

        plot = rect.adjusted(32, 34, -24, -42)
        center = QtCore.QPointF(plot.center())
        scale_value = max(abs(x_m), abs(y_m), 0.05)
        scale = 0.45 * min(plot.width(), plot.height()) / scale_value
        end = QtCore.QPointF(center.x() + x_m * scale, center.y() - y_m * scale)

        painter.setPen(QtGui.QPen(QtGui.QColor(CHART_GRID), 1))
        painter.drawLine(plot.left(), int(center.y()), plot.right(), int(center.y()))
        painter.drawLine(int(center.x()), plot.top(), int(center.x()), plot.bottom())
        painter.setPen(QtGui.QColor(CHART_MUTED))
        painter.drawText(plot.right() - 44, int(center.y()) - 6, x_label)
        painter.drawText(int(center.x()) + 6, plot.top() + 14, y_label)

        painter.setPen(QtGui.QPen(QtGui.QColor(color), 3))
        painter.drawLine(center, end)
        painter.setBrush(QtGui.QColor(color))
        painter.drawEllipse(end, 4, 4)
        painter.setPen(QtGui.QColor(CHART_TEXT))
        painter.drawText(rect.left() + 8, rect.bottom() - 18, footer)


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


class SessionSaveWorker(QtCore.QObject):
    saved = QtCore.pyqtSignal(object)
    failed = QtCore.pyqtSignal(str)
    finished = QtCore.pyqtSignal()

    def __init__(
        self,
        *,
        output_dir: Path,
        metadata: dict[str, object],
        pair_rows: Sequence[dict[str, object]],
        camera_rows: dict[str, SpoolRows],
        px4_rows: dict[str, SpoolRows],
    ) -> None:
        super().__init__()
        self.output_dir = output_dir
        self.metadata = metadata
        self.pair_rows = pair_rows
        self.camera_rows = camera_rows
        self.px4_rows = px4_rows

    @QtCore.pyqtSlot()
    def run(self) -> None:
        try:
            saved = save_session_files(
                self.output_dir,
                metadata=self.metadata,
                pair_rows=self.pair_rows,
                camera_rows=self.camera_rows,
                px4_rows=self.px4_rows,
            )
        except Exception as exc:
            self.failed.emit(str(exc))
        else:
            self.saved.emit(saved)
        finally:
            self.finished.emit()


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
    ) -> None:
        super().__init__()
        self.mode = mode
        self.control_port = control_port
        self.host_hints = tuple(host_hints)
        self.target_side = target_side

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
                    "host_interfaces": host_interfaces,
                    "control": control,
                    "verified_control": verified,
                    "arp_scan": arp_scan_reports,
                }
            elif self.mode == "control":
                control, verified = self._discover_wiris_control(arp_scan_reports)
                result = {
                    "mode": "control",
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


class PixhawkAutoDetectWorker(QtCore.QObject):
    detected = QtCore.pyqtSignal(object)
    failed = QtCore.pyqtSignal(str)
    finished = QtCore.pyqtSignal()

    def __init__(self, *, domain_id: int) -> None:
        super().__init__()
        self.domain_id = domain_id

    @QtCore.pyqtSlot()
    def run(self) -> None:
        try:
            result = discover_pixhawk_topics(current_domain_id=self.domain_id, timeout_s=1.5)
        except Exception as exc:
            self.failed.emit(str(exc))
        else:
            self.detected.emit(result)
        finally:
            self.finished.emit()


class WirisRtspEnableWorker(QtCore.QObject):
    enabled = QtCore.pyqtSignal(object)
    failed = QtCore.pyqtSignal(str)
    finished = QtCore.pyqtSignal()

    def __init__(
        self,
        *,
        hosts: dict[str, str],
        candidate_ports: dict[str, list[int]],
        urls: dict[str, str],
        timeout_s: float,
    ) -> None:
        super().__init__()
        self.hosts = hosts
        self.candidate_ports = candidate_ports
        self.urls = urls
        self.timeout_s = timeout_s

    @QtCore.pyqtSlot()
    def run(self) -> None:
        try:
            result = self._enable_rtsp()
        except Exception as exc:
            self.failed.emit(str(exc))
        else:
            self.enabled.emit(result)
        finally:
            self.finished.emit()

    def _enable_rtsp(self) -> dict[str, object]:
        logs: list[str] = []
        statuses: list[str] = []
        corrected_ports: dict[str, int] = {}
        for label in ("left", "right"):
            host = self.hosts.get(label, "").strip()
            camera_name = camera_display_name(label)
            if not host:
                statuses.append(f"{label}=missing-host")
                logs.append(f"[카메라] {camera_name} WIRIS RTSP 활성화 건너뜀: 제어 host가 비어 있습니다.")
                continue
            port, before, corrected, failure_detail = self._resolve_ethernet_mode_port(label, host)
            if port is None:
                statuses.append(f"{label}=unsupported-port")
                logs.append(f"[카메라] {camera_name} Ethernet Mode 지원 포트를 찾지 못했습니다: {failure_detail}")
                logs.append(
                    f"[카메라] {camera_name} RTSP/Ethernet Mode 활성화 실패: "
                    f"{GET_ETHERNET_MODE_COMMAND} 응답={before}"
                )
                continue
            if corrected:
                corrected_ports[label] = port
                logs.append(
                    f"[카메라] {camera_name} Telnet port 자동 보정: "
                    f"{corrected} -> {port} ({GET_ETHERNET_MODE_COMMAND}={before})"
                )
            try:
                set_command = build_wiris_command(
                    host=host,
                    port=port,
                    command=SET_ETHERNET_MODE_COMMAND,
                    timeout_s=self.timeout_s,
                )
                response = send_tcp_line(set_command)
            except Exception as exc:
                statuses.append(f"{label}=enable-failed")
                logs.append(f"[카메라] {camera_name} RTSP/Ethernet Mode 활성화 실패: {exc}")
                continue
            if not self._set_ethernet_response_ok(response):
                statuses.append(f"{label}=enable-rejected")
                logs.append(
                    f"[카메라] {camera_name} RTSP/Ethernet Mode 활성화 실패: "
                    f"{SET_ETHERNET_MODE_COMMAND} 응답={response or 'empty'}"
                )
                continue
            statuses.append(f"{label}=enable-sent")
            logs.append(
                f"[카메라] {camera_name} RTSP/Ethernet Mode 활성화: "
                f"{GET_ETHERNET_MODE_COMMAND}={before or 'n/a'}, {SET_ETHERNET_MODE_COMMAND} 응답={response or 'sent'}"
            )

        should_check = any(status.endswith("=enable-sent") for status in statuses)
        closed: list[str] = []
        if should_check:
            logs.append(
                f"[카메라] RTSP port가 열릴 때까지 최대 {RTSP_ENABLE_PORT_WAIT_MAX_ATTEMPTS}초 대기한 뒤 프레임 수신을 확인합니다."
            )
            closed = self._wait_for_rtsp_ports(logs)
        else:
            logs.append("[카메라] RTSP 활성화에 성공한 카메라가 없어 프레임 확인을 건너뜁니다.")
        return {
            "statuses": statuses,
            "logs": logs,
            "corrected_ports": corrected_ports,
            "should_check": should_check,
            "closed": closed,
        }

    def _resolve_ethernet_mode_port(self, camera: str, host: str) -> tuple[int | None, str, int | None, str]:
        failures = []
        ports = self.candidate_ports.get(camera, [])
        original_port = ports[0] if ports else None
        for port in ports:
            try:
                command = build_wiris_command(
                    host=host,
                    port=port,
                    command=GET_ETHERNET_MODE_COMMAND,
                    timeout_s=self.timeout_s,
                )
                response = send_tcp_line(command)
            except Exception as exc:
                failures.append(f"{port}=failed:{exc}")
                continue
            if self._ethernet_mode_response_ok(response):
                corrected = original_port if original_port is not None and port != original_port else None
                return port, response, corrected, ", ".join(failures)
            failures.append(f"{port}={response or 'empty'}")
        return None, failures[-1] if failures else "no-candidate", None, ", ".join(failures)

    def _wait_for_rtsp_ports(self, logs: list[str]) -> list[str]:
        closed: list[str] = []
        QtCore.QThread.msleep(RTSP_ENABLE_SETTLE_DELAY_MS)
        for attempt in range(RTSP_ENABLE_PORT_WAIT_MAX_ATTEMPTS + 1):
            closed = self._closed_rtsp_ports()
            if not closed:
                logs.append("[카메라] RTSP port 열림 확인 완료. 프레임 수신 확인을 시작합니다.")
                return []
            if attempt >= RTSP_ENABLE_PORT_WAIT_MAX_ATTEMPTS:
                logs.append("[카메라] RTSP port 대기 초과: " + ", ".join(closed))
                return closed
            logs.append("[카메라] RTSP port 대기 중: " + ", ".join(closed))
            QtCore.QThread.msleep(RTSP_ENABLE_PORT_WAIT_INTERVAL_MS)
        return closed

    def _closed_rtsp_ports(self) -> list[str]:
        closed = []
        for camera, url in self.urls.items():
            try:
                host, port = rtsp_host_port(url.strip())
            except Exception as exc:
                closed.append(f"{camera}=invalid-url:{exc}")
                continue
            if not check_tcp_port(host, port, timeout=0.5):
                closed.append(f"{camera}={host}:{port}")
        return closed

    def _ethernet_mode_response_ok(self, response: str) -> bool:
        return response.strip().upper() in {"TRUE", "FALSE"}

    def _set_ethernet_response_ok(self, response: str) -> bool:
        return response.strip().upper() in {"OK", "TRUE"}


class RtspStreamCheckWorker(QtCore.QObject):
    checked = QtCore.pyqtSignal(object)
    failed = QtCore.pyqtSignal(str)
    finished = QtCore.pyqtSignal()

    def __init__(
        self,
        urls: dict[str, str],
        *,
        latency_ms: int,
        decoder: str,
        backend: str,
        transport: str,
        timeout_s: float = 4.0,
    ) -> None:
        super().__init__()
        self.urls = urls
        self.latency_ms = latency_ms
        self.decoder = normalize_decoder_mode(decoder)
        self.backend = normalize_stream_backend(backend)
        self.transport = normalize_rtsp_transport(transport)
        self.timeout_s = timeout_s

    @QtCore.pyqtSlot()
    def run(self) -> None:
        if self.backend in {STREAM_BACKEND_OPENCV, STREAM_BACKEND_OPENCV_AUTO}:
            self._run_opencv()
            return
        if self.backend == STREAM_BACKEND_PYAV:
            self._run_pyav()
            return
        self._run_gstreamer()

    def _run_opencv(self) -> None:
        try:
            import cv2
        except Exception as exc:
            self.failed.emit(f"OpenCV unavailable: {exc}")
            self.finished.emit()
            return
        results = {}
        for label, url in self.urls.items():
            results[label] = self._check_one_opencv(cv2, url)
        self.checked.emit(results)
        self.finished.emit()

    def _check_one_opencv(self, cv2, url: str) -> dict[str, object]:
        backend_name = "opencv_auto" if self.backend == STREAM_BACKEND_OPENCV_AUTO else "opencv"
        api_preference = cv2.CAP_ANY if self.backend == STREAM_BACKEND_OPENCV_AUTO else cv2.CAP_FFMPEG
        started_s = time.monotonic()
        try:
            host, port = rtsp_host_port(url)
        except Exception as exc:
            return {"port_open": False, "frame_ok": False, "error": f"invalid url: {exc}", "backend": backend_name}
        if not check_tcp_port(host, port, timeout=min(self.timeout_s, 1.0)):
            return {"port_open": False, "frame_ok": False, "error": f"{host}:{port} closed", "backend": backend_name}
        os.environ["OPENCV_FFMPEG_CAPTURE_OPTIONS"] = (
            f"rtsp_transport;{self.transport}|stimeout;2000000|max_delay;500000"
        )
        cap = cv2.VideoCapture(url, api_preference)
        try:
            cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        except Exception:
            pass
        try:
            while time.monotonic() - started_s < self.timeout_s:
                ok, frame = cap.read()
                if ok and frame is not None:
                    height, width = frame.shape[:2]
                    return {
                        "port_open": True,
                        "frame_ok": True,
                        "width": width,
                        "height": height,
                        "elapsed_s": time.monotonic() - started_s,
                        "decoder": "opencv-auto" if self.backend == STREAM_BACKEND_OPENCV_AUTO else "opencv-ffmpeg",
                        "backend": backend_name,
                    }
                QtCore.QThread.msleep(20)
        finally:
            cap.release()
        return {
            "port_open": True,
            "frame_ok": False,
            "error": f"no frame within {self.timeout_s:.1f}s",
            "backend": backend_name,
        }

    def _run_pyav(self) -> None:
        try:
            import av
        except Exception as exc:
            self.failed.emit(f"PyAV unavailable: {exc}")
            self.finished.emit()
            return
        results = {}
        for label, url in self.urls.items():
            results[label] = self._check_one_pyav(av, url)
        self.checked.emit(results)
        self.finished.emit()

    def _check_one_pyav(self, av, url: str) -> dict[str, object]:
        started_s = time.monotonic()
        try:
            host, port = rtsp_host_port(url)
        except Exception as exc:
            return {"port_open": False, "frame_ok": False, "error": f"invalid url: {exc}", "backend": "pyav"}
        if not check_tcp_port(host, port, timeout=min(self.timeout_s, 1.0)):
            return {"port_open": False, "frame_ok": False, "error": f"{host}:{port} closed", "backend": "pyav"}
        options = {
            "rtsp_transport": self.transport,
            "stimeout": "2000000",
            "max_delay": "500000",
            "fflags": "nobuffer",
            "flags": "low_delay",
        }
        container = None
        try:
            container = av.open(url, mode="r", options=options)
            video_stream = next((stream for stream in container.streams if stream.type == "video"), None)
            if video_stream is None:
                return {"port_open": True, "frame_ok": False, "error": "no video stream", "backend": "pyav"}
            for packet in container.demux(video_stream):
                for frame in packet.decode():
                    pts_s = None
                    if frame.pts is not None and frame.time_base is not None:
                        pts_s = float(frame.pts * frame.time_base)
                    return {
                        "url": url,
                        "port_open": True,
                        "frame_ok": True,
                        "width": int(frame.width),
                        "height": int(frame.height),
                        "pts_s": pts_s,
                        "elapsed_s": time.monotonic() - started_s,
                        "decoder": "pyav",
                        "backend": "pyav",
                    }
                if time.monotonic() - started_s >= self.timeout_s:
                    break
            return {
                "url": url,
                "port_open": True,
                "frame_ok": False,
                "error": f"no frame within {self.timeout_s:.1f}s",
                "decoder": "pyav",
                "backend": "pyav",
            }
        except Exception as exc:
            return {"url": url, "port_open": True, "frame_ok": False, "error": str(exc), "decoder": "pyav", "backend": "pyav"}
        finally:
            if container is not None:
                container.close()

    def _run_gstreamer(self) -> None:
        try:
            import gi

            gi.require_version("Gst", "1.0")
            from gi.repository import Gst
        except Exception as exc:
            self.failed.emit(f"GStreamer unavailable: {exc}")
            self.finished.emit()
            return

        Gst.init(None)
        results = {}
        for label, url in self.urls.items():
            results[label] = self._check_one(Gst, url)
        self.checked.emit(results)
        self.finished.emit()

    def _check_one(self, Gst, url: str) -> dict[str, object]:
        try:
            host, port = rtsp_host_port(url)
            port_open = check_tcp_port(host, port, timeout=1.0)
        except Exception as exc:
            return {"url": url, "port_open": False, "frame_ok": False, "error": str(exc)}
        if not port_open:
            return {"url": url, "port_open": False, "frame_ok": False, "error": f"{host}:{port} closed"}

        last_error = ""
        candidates = decoder_candidates(self.decoder)
        for decoder in candidates:
            pipeline = None
            try:
                pipeline_text = build_gst_pipeline_text(
                    url,
                    latency_ms=self.latency_ms,
                    decoder=decoder,
                    transport=self.transport,
                )
                pipeline = Gst.parse_launch(pipeline_text)
                sink = pipeline.get_by_name("sink")
                bus = pipeline.get_bus()
                if sink is None or bus is None:
                    raise RuntimeError("pipeline missing appsink or bus")
                state_result = pipeline.set_state(Gst.State.PLAYING)
                if state_result == Gst.StateChangeReturn.FAILURE:
                    raise RuntimeError(f"{decoder} failed to enter PLAYING state")
                return self._wait_for_rtsp_frame(Gst, pipeline, sink, bus, url, decoder)
            except Exception as exc:
                last_error = str(exc)
                if pipeline is not None:
                    pipeline.set_state(Gst.State.NULL)
                if self.decoder == DECODER_AUTO and decoder != candidates[-1]:
                    continue
                break
        return {"url": url, "port_open": True, "frame_ok": False, "error": last_error or "decoder unavailable"}

    def _wait_for_rtsp_frame(self, Gst, pipeline, sink, bus, url: str, decoder: str) -> dict[str, object]:
        try:
            start_s = time.monotonic()
            while time.monotonic() - start_s < self.timeout_s:
                message = bus.pop_filtered(Gst.MessageType.ERROR | Gst.MessageType.EOS)
                if message is not None:
                    if message.type == Gst.MessageType.ERROR:
                        error, debug = message.parse_error()
                        detail = error.message if debug is None else f"{error.message} ({debug})"
                    else:
                        detail = "EOS before frame"
                    return {"url": url, "port_open": True, "frame_ok": False, "error": detail, "decoder": decoder}

                sample = sink.emit("try-pull-sample", 200_000_000)
                if sample is None:
                    continue
                buffer = sample.get_buffer()
                pts_s = None if buffer.pts == Gst.CLOCK_TIME_NONE else buffer.pts / 1e9
                caps = sample.get_caps().get_structure(0)
                return {
                    "url": url,
                    "port_open": True,
                    "frame_ok": True,
                    "width": int(caps.get_value("width")),
                    "height": int(caps.get_value("height")),
                    "pts_s": pts_s,
                    "elapsed_s": time.monotonic() - start_s,
                    "decoder": decoder,
                }
            return {
                "url": url,
                "port_open": True,
                "frame_ok": False,
                "error": f"no frame within {self.timeout_s:.1f}s",
                "decoder": decoder,
            }
        except Exception as exc:
            return {"url": url, "port_open": True, "frame_ok": False, "error": str(exc), "decoder": decoder}
        finally:
            pipeline.set_state(Gst.State.NULL)


def _ubx_checksum(data: bytes) -> tuple[int, int]:
    ck_a = 0
    ck_b = 0
    for value in data:
        ck_a = (ck_a + value) & 0xFF
        ck_b = (ck_b + ck_a) & 0xFF
    return ck_a, ck_b


def ubx_frame(message_class: int, message_id: int, payload: bytes) -> bytes:
    header = bytes((message_class & 0xFF, message_id & 0xFF)) + len(payload).to_bytes(2, "little")
    ck_a, ck_b = _ubx_checksum(header + payload)
    return b"\xb5\x62" + header + payload + bytes((ck_a, ck_b))


def ubx_cfg_tmode3_survey_in_frame(*, min_duration_s: int, accuracy_m: float) -> bytes:
    min_duration_s = max(1, int(min_duration_s))
    accuracy_0_1mm = max(1, int(round(float(accuracy_m) / 0.0001)))
    payload = struct.pack(
        "<BBHlllbbbBIII8s",
        0,
        0,
        1,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        min_duration_s,
        accuracy_0_1mm,
        b"\x00" * 8,
    )
    return ubx_frame(0x06, 0x71, payload)


def ubx_cfg_tmode3_disabled_frame() -> bytes:
    payload = struct.pack(
        "<BBHlllbbbBIII8s",
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        b"\x00" * 8,
    )
    return ubx_frame(0x06, 0x71, payload)


def ubx_poll_frame(message_class: int, message_id: int) -> bytes:
    return ubx_frame(message_class, message_id, b"")


def ubx_nav_svin_poll_frame() -> bytes:
    return ubx_poll_frame(0x01, 0x3B)


def parse_ubx_nav_svin(payload: bytes) -> dict[str, object] | None:
    if len(payload) < 40:
        return None
    (
        _version,
        _reserved0,
        _itow_ms,
        duration_s,
        mean_x_cm,
        mean_y_cm,
        mean_z_cm,
        mean_x_hp_0_1mm,
        mean_y_hp_0_1mm,
        mean_z_hp_0_1mm,
        _reserved1,
        mean_acc_0_1mm,
        observations,
        valid,
        active,
        _reserved2,
    ) = struct.unpack_from("<B3sLLlllbbbBLLBBH", payload, 0)
    return {
        "duration_s": int(duration_s),
        "mean_accuracy_m": float(mean_acc_0_1mm) * 0.0001,
        "observations": int(observations),
        "valid": bool(valid),
        "active": bool(active),
        "mean_x_m": float(mean_x_cm) * 0.01 + float(mean_x_hp_0_1mm) * 0.0001,
        "mean_y_m": float(mean_y_cm) * 0.01 + float(mean_y_hp_0_1mm) * 0.0001,
        "mean_z_m": float(mean_z_cm) * 0.01 + float(mean_z_hp_0_1mm) * 0.0001,
    }


def parse_ubx_base_satellite_count(message_class: int, message_id: int, payload: bytes) -> int | None:
    if message_class != 0x01:
        return None
    if message_id == 0x07 and len(payload) >= 24:
        return int(payload[23])
    if message_id == 0x35 and len(payload) >= 8:
        return int(payload[5])
    return None


def parse_nmea_gga_satellite_count(line: str) -> tuple[int, int] | None:
    text = line.strip()
    if not text.startswith(("$GNGGA", "$GPGGA", "$GAGGA", "$GLGGA", "$GBGGA")):
        return None
    if "*" in text:
        body, checksum_text = text[1:].split("*", 1)
        checksum = 0
        for char in body:
            checksum ^= ord(char)
        try:
            expected = int(checksum_text[:2], 16)
        except ValueError:
            return None
        if checksum != expected:
            return None
        fields = body.split(",")
    else:
        fields = text[1:].split(",")
    if len(fields) <= 7:
        return None
    try:
        fix_quality = int(fields[6] or "0")
        satellites = int(fields[7] or "0")
    except ValueError:
        return None
    return satellites, fix_quality


def gnss_base_serial_port_candidates() -> list[str]:
    candidates: list[str] = []
    for pattern in ("/dev/serial/by-id/*", "/dev/ttyACM*", "/dev/ttyUSB*"):
        for path in sorted(Path("/").glob(pattern.removeprefix("/"))):
            text = str(path)
            if text not in candidates:
                candidates.append(text)
    return candidates


def probe_gnss_base_serial_ports(candidates: Sequence[str], *, baud: int, timeout_s: float = 0.6) -> list[dict[str, object]]:
    rows: list[dict[str, object]] = []
    try:
        import serial  # type: ignore[import-not-found]
    except Exception as exc:
        return [
            {
                "port": str(candidate),
                "target": os.path.realpath(str(candidate)),
                "baud": int(baud),
                "status": f"pyserial 필요: {exc}",
                "ubx": False,
                "messages": "",
            }
            for candidate in candidates
        ]
    for candidate in candidates:
        port = str(candidate)
        parser = UbxStreamParser()
        nmea_buffer = ""
        messages_seen: set[str] = set()
        total_bytes = 0
        status = "no data"
        ubx_detected = False
        try:
            with serial.Serial(port, int(baud), timeout=0.1) as serial_obj:
                deadline = time.monotonic() + max(0.1, float(timeout_s))
                while time.monotonic() < deadline:
                    chunk = serial_obj.read(4096)
                    if not chunk:
                        continue
                    total_bytes += len(chunk)
                    nmea_buffer += chunk.decode("ascii", "ignore")
                    if "\n" in nmea_buffer:
                        lines = nmea_buffer.splitlines()
                        nmea_buffer = "" if nmea_buffer.endswith(("\n", "\r")) else lines.pop() if lines else nmea_buffer
                        for line in lines:
                            gga = parse_nmea_gga_satellite_count(line)
                            if gga is not None:
                                satellites, fix_quality = gga
                                messages_seen.add(f"NMEA-GGA sats={satellites} fix={fix_quality}")
                    for message_class, message_id, payload in parser.feed(chunk):
                        ubx_detected = True
                        if message_class == 0x01 and message_id == 0x3B:
                            messages_seen.add("NAV-SVIN")
                        elif message_class == 0x01 and message_id == 0x07:
                            sat_count = parse_ubx_base_satellite_count(message_class, message_id, payload)
                            messages_seen.add("NAV-PVT" if sat_count is None else f"NAV-PVT sats={sat_count}")
                        elif message_class == 0x01 and message_id == 0x35:
                            sat_count = parse_ubx_base_satellite_count(message_class, message_id, payload)
                            messages_seen.add("NAV-SAT" if sat_count is None else f"NAV-SAT sats={sat_count}")
                        else:
                            messages_seen.add(f"UBX {message_class:02X}/{message_id:02X}")
                    if messages_seen:
                        break
            if messages_seen:
                status = "GNSS data detected"
            elif total_bytes:
                status = f"raw data only ({total_bytes} bytes)"
        except Exception as exc:
            status = str(exc)
        rows.append(
            {
                "port": port,
                "target": os.path.realpath(port),
                "baud": int(baud),
                "status": status,
                "ubx": ubx_detected,
                "messages": ", ".join(sorted(messages_seen)),
            }
        )
    return rows


class UbxStreamParser:
    def __init__(self) -> None:
        self._buffer = bytearray()

    def feed(self, data: bytes) -> list[tuple[int, int, bytes]]:
        self._buffer.extend(data)
        messages: list[tuple[int, int, bytes]] = []
        while True:
            sync_index = self._buffer.find(b"\xb5\x62")
            if sync_index < 0:
                if len(self._buffer) > 1:
                    del self._buffer[:-1]
                break
            if sync_index:
                del self._buffer[:sync_index]
            if len(self._buffer) < 8:
                break
            payload_len = int.from_bytes(self._buffer[4:6], "little")
            frame_len = 8 + payload_len
            if len(self._buffer) < frame_len:
                break
            frame = bytes(self._buffer[:frame_len])
            del self._buffer[:frame_len]
            ck_a, ck_b = _ubx_checksum(frame[2:-2])
            if ck_a != frame[-2] or ck_b != frame[-1]:
                continue
            messages.append((frame[2], frame[3], frame[6:-2]))
        return messages


class Rtcm3StreamParser:
    def __init__(self) -> None:
        self._buffer = bytearray()

    def feed(self, data: bytes) -> list[bytes]:
        self._buffer.extend(data)
        frames: list[bytes] = []
        while True:
            sync_index = self._buffer.find(b"\xd3")
            if sync_index < 0:
                self._buffer.clear()
                break
            if sync_index:
                del self._buffer[:sync_index]
            if len(self._buffer) < 3:
                break
            if self._buffer[1] & 0xFC:
                del self._buffer[0]
                continue
            payload_len = ((self._buffer[1] & 0x03) << 8) | self._buffer[2]
            frame_len = 3 + payload_len + 3
            if len(self._buffer) < frame_len:
                break
            frames.append(bytes(self._buffer[:frame_len]))
            del self._buffer[:frame_len]
        return frames


class GnssBaseSerialWorker(QtCore.QObject):
    connected = QtCore.pyqtSignal(str, int)
    stats_changed = QtCore.pyqtSignal(int, float, float)
    survey_in_changed = QtCore.pyqtSignal(object)
    satellite_count_changed = QtCore.pyqtSignal(int, str)
    rtcm_frame_received = QtCore.pyqtSignal(object)
    command_sent = QtCore.pyqtSignal(str, int)
    failed = QtCore.pyqtSignal(str)
    disconnected = QtCore.pyqtSignal(str)
    finished = QtCore.pyqtSignal()

    def __init__(self, *, port: str, baud: int) -> None:
        super().__init__()
        self.port = port
        self.baud = int(baud)
        self._stop_requested = False
        self._write_queue: queue.Queue[tuple[str, bytes]] = queue.Queue()

    @QtCore.pyqtSlot()
    def run(self) -> None:
        serial_obj = None
        total_bytes = 0
        started_s = time.monotonic()
        last_emit_s = started_s
        last_rx_s: float | None = None
        last_nav_svin_poll_s = 0.0
        ubx_parser = UbxStreamParser()
        rtcm_parser = Rtcm3StreamParser()
        nmea_buffer = ""
        try:
            try:
                import serial  # type: ignore[import-not-found]
            except Exception as exc:
                self.failed.emit(f"pyserial 필요: {exc}")
                return
            serial_obj = serial.Serial(self.port, self.baud, timeout=0.2)
            self.connected.emit(self.port, self.baud)
            while not self._stop_requested:
                self._drain_writes(serial_obj)
                now_s = time.monotonic()
                if now_s - last_nav_svin_poll_s >= 2.0:
                    serial_obj.write(ubx_nav_svin_poll_frame())
                    try:
                        serial_obj.flush()
                    except Exception:
                        pass
                    last_nav_svin_poll_s = now_s
                chunk = serial_obj.read(4096)
                if chunk:
                    total_bytes += len(chunk)
                    last_rx_s = now_s
                    for rtcm_frame in rtcm_parser.feed(chunk):
                        self.rtcm_frame_received.emit(rtcm_frame)
                    nmea_buffer += chunk.decode("ascii", "ignore")
                    if "\n" in nmea_buffer:
                        lines = nmea_buffer.splitlines()
                        nmea_buffer = "" if nmea_buffer.endswith(("\n", "\r")) else lines.pop() if lines else nmea_buffer
                        for line in lines:
                            gga = parse_nmea_gga_satellite_count(line)
                            if gga is not None:
                                satellites, fix_quality = gga
                                self.satellite_count_changed.emit(satellites, f"NMEA-GGA fix={fix_quality}")
                    for message_class, message_id, payload in ubx_parser.feed(chunk):
                        if message_class == 0x01 and message_id == 0x3B:
                            survey_in = parse_ubx_nav_svin(payload)
                            if survey_in is not None:
                                self.survey_in_changed.emit(survey_in)
                        satellite_count = parse_ubx_base_satellite_count(message_class, message_id, payload)
                        if satellite_count is not None:
                            source = "NAV-PVT" if message_id == 0x07 else "NAV-SAT"
                            self.satellite_count_changed.emit(satellite_count, source)
                if now_s - last_emit_s >= 0.5:
                    elapsed_s = max(1e-6, now_s - started_s)
                    rate_bps = total_bytes / elapsed_s
                    age_s = -1.0 if last_rx_s is None else max(0.0, now_s - last_rx_s)
                    self.stats_changed.emit(total_bytes, rate_bps, age_s)
                    last_emit_s = now_s
            self.disconnected.emit("disconnected")
        except Exception as exc:
            self.failed.emit(str(exc))
        finally:
            if serial_obj is not None:
                try:
                    serial_obj.close()
                except Exception:
                    pass
            self.finished.emit()

    @QtCore.pyqtSlot()
    def request_stop(self) -> None:
        self._stop_requested = True

    @QtCore.pyqtSlot(str, bytes)
    def send_bytes(self, label: str, payload: bytes) -> None:
        self._write_queue.put((label, bytes(payload)))

    def _drain_writes(self, serial_obj) -> None:
        while True:
            try:
                label, payload = self._write_queue.get_nowait()
            except queue.Empty:
                return
            serial_obj.write(payload)
            try:
                serial_obj.flush()
            except Exception:
                pass
            self.command_sent.emit(label, len(payload))


class GnssBaseSerialDetectWorker(QtCore.QObject):
    detected = QtCore.pyqtSignal(object)
    failed = QtCore.pyqtSignal(str)
    finished = QtCore.pyqtSignal()

    def __init__(self, *, baud: int) -> None:
        super().__init__()
        self.baud = int(baud)

    @QtCore.pyqtSlot()
    def run(self) -> None:
        try:
            candidates = gnss_base_serial_port_candidates()
            rows = probe_gnss_base_serial_ports(candidates, baud=self.baud, timeout_s=0.6)
        except Exception as exc:
            self.failed.emit(str(exc))
        else:
            self.detected.emit(rows)
        finally:
            self.finished.emit()


class CheckerboardPreviewWorker(QtCore.QObject):
    result_ready = QtCore.pyqtSignal(object)

    def __init__(self) -> None:
        super().__init__()
        self._pending_payload: dict[str, object] | None = None
        self._busy = False
        self._process_scheduled = False
        self._cv_modules: tuple[object, object] | None = None
        self._cv_error: str | None = None

    @QtCore.pyqtSlot(object)
    def submit(self, payload: object) -> None:
        self._pending_payload = dict(payload or {}) if isinstance(payload, dict) else {}
        if not self._busy and not self._process_scheduled:
            self._process_scheduled = True
            QtCore.QTimer.singleShot(0, self._process_next)

    @QtCore.pyqtSlot()
    def _process_next(self) -> None:
        self._process_scheduled = False
        if self._busy or self._pending_payload is None:
            return
        payload = self._pending_payload
        self._pending_payload = None
        self._busy = True
        try:
            result = self._detect(payload)
        finally:
            self._busy = False
        self.result_ready.emit(result)
        if self._pending_payload is not None and not self._process_scheduled:
            self._process_scheduled = True
            QtCore.QTimer.singleShot(0, self._process_next)

    def _detect(self, payload: dict[str, object]) -> dict[str, object]:
        image = payload.get("image")
        result = {
            "camera": str(payload.get("camera") or ""),
            "request_id": int(payload.get("request_id") or 0),
            "image": image,
            "found": False,
            "corners": [],
            "status": "checkerboard: not found",
        }
        pattern_size = payload.get("pattern_size")
        if not isinstance(image, QtGui.QImage):
            result["status"] = "checkerboard: missing frame"
            return result
        if not (
            isinstance(pattern_size, tuple)
            and len(pattern_size) == 2
            and all(isinstance(value, int) and value > 0 for value in pattern_size)
        ):
            result["status"] = "checkerboard: invalid pattern"
            return result
        cv2, np, error = self._load_cv_modules()
        if error is not None:
            result["status"] = f"OpenCV unavailable: {error}"
            return result

        rgb_image = image.convertToFormat(QtGui.QImage.Format_RGB888).copy()
        width = rgb_image.width()
        height = rgb_image.height()
        ptr = rgb_image.bits()
        ptr.setsize(rgb_image.byteCount())
        rgb = np.frombuffer(ptr, dtype=np.uint8).reshape((height, rgb_image.bytesPerLine()))[:, : width * 3]
        rgb = rgb.reshape((height, width, 3))
        gray = cv2.cvtColor(rgb, cv2.COLOR_RGB2GRAY)
        flags = int(getattr(cv2, "CALIB_CB_FAST_CHECK", 0))
        found, raw_corners = cv2.findChessboardCorners(gray, pattern_size, flags)
        if not found or raw_corners is None:
            return result
        corners = [(float(point[0][0]), float(point[0][1])) for point in raw_corners]
        result["found"] = True
        result["corners"] = corners
        result["status"] = f"checkerboard: {len(corners)} corners"
        return result

    def _load_cv_modules(self):
        if self._cv_error is not None:
            return None, None, self._cv_error
        if self._cv_modules is not None:
            cv2, np = self._cv_modules
            return cv2, np, None
        try:
            import cv2
            import numpy as np
        except Exception as exc:
            self._cv_error = str(exc)
            return None, None, self._cv_error
        self._cv_modules = (cv2, np)
        return cv2, np, None


class TopicBrowserDialog(QtWidgets.QDialog):
    def __init__(self, topics: list[dict[str, str]], parent=None, *, title: str = "ROS2 Topic Browser") -> None:
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

    def __init__(self, *, auto_start: bool = False, auto_start_pixhawk: bool = False) -> None:
        super().__init__()
        self.setMinimumSize(900, 820)
        self.latest_left: DetectionObservation | None = None
        self.latest_right: DetectionObservation | None = None
        self.latest_calibration_preview_images: dict[str, QtGui.QImage] = {}
        self.calibration_preview_overlay_images: dict[str, QtGui.QImage] = {}
        self.calibration_preview_times = {
            "left": 0.0,
            "right": 0.0,
        }
        self.calibration_preview_request_ids = {
            "left": 0,
            "right": 0,
        }
        self._calibration_preview_next_request_id = 0
        self._calibration_preview_worker: CheckerboardPreviewWorker | None = CheckerboardPreviewWorker()
        self._calibration_preview_thread: QtCore.QThread | None = QtCore.QThread(self)
        self._calibration_preview_worker.moveToThread(self._calibration_preview_thread)
        self.calibration_preview_requested.connect(self._calibration_preview_worker.submit)
        self._calibration_preview_worker.result_ready.connect(self._handle_calibration_preview_result)
        self._calibration_preview_thread.finished.connect(self._calibration_preview_worker.deleteLater)
        self._calibration_preview_thread.start()
        self.destroyed.connect(self._stop_calibration_preview_worker)
        self.detection_buffers = {
            "left": TimedSampleBuffer[DetectionObservation](maxlen=120),
            "right": TimedSampleBuffer[DetectionObservation](maxlen=120),
        }
        self.latest_extrinsics: dict[str, CameraExtrinsicState] = {}
        self.latest_gps: dict[str, GpsPositionState] = {}
        self.custom_watch_specs: list[dict[str, str]] = []
        self.latest_rtk_baseline: RtkBaselineEstimate | None = None
        self.calibration_capture_index = 0
        self.last_calibration_result: StereoIntrinsicsResult | None = None
        self.last_calibration_zoom_level: int | None = None
        self.active_intrinsics: StereoIntrinsicsResult | None = None
        self._active_intrinsics_profile_path = ""
        self.zoom_intrinsics_compare_paths: list[Path] = []
        self.attitude_buffers = {
            "left": TimedSampleBuffer[CameraExtrinsicState](maxlen=1000),
            "right": TimedSampleBuffer[CameraExtrinsicState](maxlen=1000),
        }
        self.pixhawk: DualPixhawkAttitudeSubscriber | None = None
        self.pixhawk_ros2_subscribers: dict[str, DualPixhawkAttitudeSubscriber | None] = {
            "left": None,
            "right": None,
        }
        self.mavlink_pixhawk: DualPixhawkMavlinkSubscriber | None = None
        self._ros2_retry_after_uxrce_agent_setup = False
        self._eth0_probe_buffers: dict[str, str] = {}
        self._eth0_probe_tokens: dict[str, int] = {}
        self._eth0_probe_counter = 0
        self.px4_shell_dialogs: dict[str, Px4ShellDialog] = {}
        self.camera_connected = False
        self.yolo_enabled = False
        self.camera_auto_detect_thread: QtCore.QThread | None = None
        self.camera_auto_detect_worker: CameraAutoDetectWorker | None = None
        self.pixhawk_auto_detect_thread: QtCore.QThread | None = None
        self.pixhawk_auto_detect_worker: PixhawkAutoDetectWorker | None = None
        self.rtsp_check_thread: QtCore.QThread | None = None
        self.rtsp_check_worker: RtspStreamCheckWorker | None = None
        self.rtsp_enable_thread: QtCore.QThread | None = None
        self.rtsp_enable_worker: WirisRtspEnableWorker | None = None
        self.gnss_base_serial_thread: QtCore.QThread | None = None
        self.gnss_base_serial_worker: GnssBaseSerialWorker | None = None
        self.gnss_base_serial_connected = False
        self.gnss_base_detect_thread: QtCore.QThread | None = None
        self.gnss_base_detect_worker: GnssBaseSerialDetectWorker | None = None
        self._gnss_base_candidate_rows: list[dict[str, object]] = []
        self.auto_wiris_connect_pending = False
        self.auto_start = auto_start
        self.auto_start_pixhawk = auto_start_pixhawk
        self.settings = load_settings()
        self.distance_history: deque[DistanceHistoryPoint] = deque(maxlen=600)
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
        self._session_save_thread: QtCore.QThread | None = None
        self._session_save_worker: SessionSaveWorker | None = None
        self._session_save_retry_required = False
        self._session_save_finalized = False
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
        self.latest_vehicle_mag_device_ids: dict[str, int | None] = {"left": None, "right": None}
        self.ros2_sensor_topic_publishers: dict[str, dict[str, bool]] = {"left": {}, "right": {}}
        self.px4_mag_managers = {
            "left": Px4MagManager(MockParameterStore(current_pixhawk_6x_mag_params())),
            "right": Px4MagManager(MockParameterStore(current_pixhawk_6x_mag_params())),
        }
        self.px4_mag_manager = self.px4_mag_managers["left"]
        self.latest_gps_sensors: dict[str, dict[str, object]] = {"left": {}, "right": {}}
        self.latest_gnss_relative: dict[str, object] = {}
        self.last_rtsp_check_status: dict[str, str] = {"left": "unchecked", "right": "unchecked"}
        self.last_distance_quality = "BAD"
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
        save_session_tab = QtWidgets.QWidget()
        save_session_layout = QtWidgets.QVBoxLayout(save_session_tab)
        recording_samples_tab = QtWidgets.QWidget()
        recording_samples_layout = QtWidgets.QVBoxLayout(recording_samples_tab)
        advanced_tab = QtWidgets.QWidget()
        advanced_layout = QtWidgets.QVBoxLayout(advanced_tab)
        wiris_command_panel = QtWidgets.QWidget()
        wiris_command_layout = QtWidgets.QVBoxLayout(wiris_command_panel)
        baseline_field_browser_tab = QtWidgets.QWidget()
        baseline_field_browser_layout = QtWidgets.QVBoxLayout(baseline_field_browser_tab)
        for layout in (
            setup_layout,
            time_layout,
            alignment_layout,
            alignment_overview_layout,
            rtk_layout,
            mag_management_layout,
            alignment_mag_layout,
            alignment_attitude_layout,
            alignment_coordinate_layout,
            alignment_validation_layout,
            alignment_profile_layout,
            signal_alignment_layout,
            rm3100_alignment_layout,
            yaw_alignment_layout,
            calibration_layout,
            checkerboard_calibration_layout,
            zoom_intrinsics_layout,
            camera_params_layout,
            intrinsic_parameters_layout,
            extrinsic_parameters_layout,
            yolo_layout_outer,
            measurement_layout,
            stereo_layout,
            runtime_layout,
            save_layout,
            save_session_layout,
            recording_samples_layout,
            advanced_layout,
            wiris_command_layout,
            baseline_field_browser_layout,
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
        calibration_tabs.addTab(zoom_intrinsics_tab, "줌 내부 파라미터")
        calibration_layout.addWidget(calibration_tabs, 1)
        self.calibration_tabs = calibration_tabs
        self.checkerboard_calibration_tab_index = calibration_tabs.indexOf(checkerboard_calibration_tab)
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

        save_tabs = QtWidgets.QTabWidget()
        save_tabs.setDocumentMode(True)
        save_tabs.addTab(save_session_tab, "세션 저장")
        save_tabs.addTab(recording_samples_tab, "기록 샘플")
        save_layout.addWidget(save_tabs, 1)
        self.save_tabs = save_tabs
        self.save_session_tab_index = save_tabs.indexOf(save_session_tab)
        self.recording_samples_tab_index = save_tabs.indexOf(recording_samples_tab)

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
        self.left_rtsp_apply_button = QtWidgets.QPushButton("RTSP 적용")
        self.left_rtsp_apply_button.clicked.connect(lambda _checked=False: self.apply_rtsp_url("left"))
        self.right_rtsp_apply_button = QtWidgets.QPushButton("RTSP 적용")
        self.right_rtsp_apply_button.clicked.connect(lambda _checked=False: self.apply_rtsp_url("right"))
        self.left_control_apply_button = QtWidgets.QPushButton("Telnet 적용")
        self.left_control_apply_button.clicked.connect(lambda _checked=False: self.apply_control_settings("left"))
        self.right_control_apply_button = QtWidgets.QPushButton("Telnet 적용")
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
        self.yolo_gate_min_bbox_checkbox = self._build_toggle_button(
            "최소 bbox 크기",
            self.settings.yolo_gate_min_bbox_enabled,
        )
        self.yolo_gate_min_width_spin = QtWidgets.QSpinBox()
        self.yolo_gate_min_width_spin.setRange(0, 10000)
        self.yolo_gate_min_width_spin.setValue(self.settings.yolo_gate_min_width_px)
        self.yolo_gate_min_width_spin.setSuffix(" px")
        self.yolo_gate_min_height_spin = QtWidgets.QSpinBox()
        self.yolo_gate_min_height_spin.setRange(0, 10000)
        self.yolo_gate_min_height_spin.setValue(self.settings.yolo_gate_min_height_px)
        self.yolo_gate_min_height_spin.setSuffix(" px")
        self.yolo_gate_min_area_spin = QtWidgets.QSpinBox()
        self.yolo_gate_min_area_spin.setRange(0, 100_000_000)
        self.yolo_gate_min_area_spin.setValue(self.settings.yolo_gate_min_area_px)
        self.yolo_gate_min_area_spin.setSuffix(" px²")
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
        self.decoder_combo.addItem("Software", DECODER_SOFTWARE)
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
        self.detect_control_button = QtWidgets.QPushButton("Telnet port 탐색")
        self.detect_control_button.clicked.connect(lambda _checked=False: self.detect_camera_control("left"))
        self.right_detect_control_button = QtWidgets.QPushButton("Telnet port 탐색")
        self.right_detect_control_button.clicked.connect(lambda _checked=False: self.detect_camera_control("right"))
        self.detect_rtsp_button = QtWidgets.QPushButton("영상 주소 탐색")
        self.detect_rtsp_button.clicked.connect(self.detect_camera_rtsp)
        self.right_detect_rtsp_button = QtWidgets.QPushButton("영상 주소 탐색")
        self.right_detect_rtsp_button.clicked.connect(self.detect_camera_rtsp)
        self.auto_wiris_connect_button = QtWidgets.QPushButton("WIRIS 자동 연결")
        self.auto_wiris_connect_button.clicked.connect(self.auto_connect_wiris)
        self.detect_pc_ip_button = QtWidgets.QPushButton("PC IP 탐색")
        self.detect_pc_ip_button.clicked.connect(self.discover_pc_camera_ip)
        self.apply_pc_ip_button = QtWidgets.QPushButton("PC IP 적용")
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
        self.apply_candidate_nic_button = QtWidgets.QPushButton("선택 NIC")
        self.apply_candidate_nic_button.clicked.connect(self.apply_selected_camera_candidate_nic)
        self.apply_candidate_left_button = QtWidgets.QPushButton("Left에 적용")
        self.apply_candidate_left_button.clicked.connect(lambda _checked=False: self.apply_selected_camera_candidate("left"))
        self.apply_candidate_right_button = QtWidgets.QPushButton("Right에 적용")
        self.apply_candidate_right_button.clicked.connect(lambda _checked=False: self.apply_selected_camera_candidate("right"))
        self.clear_camera_candidate_button = QtWidgets.QPushButton("Clear")
        self.clear_camera_candidate_button.clicked.connect(self.clear_camera_candidate_history)
        self.clear_camera_candidate_button.setEnabled(False)
        self.clear_camera_candidate_button.setToolTip("WIRIS 탐색 후보 내역을 지웁니다.")
        self.camera_candidate_table.itemSelectionChanged.connect(self._refresh_camera_candidate_actions)
        self._camera_candidate_rows: list[dict[str, object]] = []
        self.pixhawk_auto_detect_button = QtWidgets.QPushButton("Pixhawk 자동 탐색")
        self.pixhawk_auto_detect_button.clicked.connect(self.auto_detect_pixhawks)
        self.pixhawk_usb_discover_button = QtWidgets.QPushButton("USB 연결 탐색")
        self.pixhawk_usb_discover_button.clicked.connect(self.discover_usb_pixhawk_connections)
        self.pixhawk_mavlink_url_candidate_button = QtWidgets.QPushButton("MAVLink URL 후보")
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
        self.ros2_pixhawk_candidate_table = build_pixhawk_candidate_table()
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
        self.clear_pixhawk_ros2_candidate_button = QtWidgets.QPushButton("Clear")
        self.clear_pixhawk_ros2_candidate_button.clicked.connect(lambda _checked=False: self.clear_pixhawk_candidate_history("ros2"))
        self.clear_pixhawk_ros2_candidate_button.setEnabled(False)
        self.clear_pixhawk_ros2_candidate_button.setToolTip("Pixhawk ROS2 후보 내역을 지웁니다.")
        self.apply_pixhawk_candidate_host_button = QtWidgets.QPushButton("Host에 적용")
        self.apply_pixhawk_candidate_host_button.setEnabled(False)
        self.apply_pixhawk_candidate_host_button.clicked.connect(
            lambda _checked=False: self.apply_selected_pixhawk_candidate("host")
        )
        self.apply_pixhawk_candidate_left_button = QtWidgets.QPushButton("P1에 적용")
        self.apply_pixhawk_candidate_left_button.setEnabled(False)
        self.apply_pixhawk_candidate_left_button.clicked.connect(
            lambda _checked=False: self.apply_selected_pixhawk_candidate("left")
        )
        self.apply_pixhawk_candidate_right_button = QtWidgets.QPushButton("P2에 적용")
        self.apply_pixhawk_candidate_right_button.setEnabled(False)
        self.apply_pixhawk_candidate_right_button.clicked.connect(
            lambda _checked=False: self.apply_selected_pixhawk_candidate("right")
        )
        self.apply_pixhawk_ros2_candidate_left_button = QtWidgets.QPushButton("P1 적용")
        self.apply_pixhawk_ros2_candidate_left_button.setEnabled(False)
        self.apply_pixhawk_ros2_candidate_left_button.clicked.connect(
            lambda _checked=False: self.apply_selected_pixhawk_candidate("left", source="ros2")
        )
        self.apply_pixhawk_ros2_candidate_right_button = QtWidgets.QPushButton("P2 적용")
        self.apply_pixhawk_ros2_candidate_right_button.setEnabled(False)
        self.apply_pixhawk_ros2_candidate_right_button.clicked.connect(
            lambda _checked=False: self.apply_selected_pixhawk_candidate("right", source="ros2")
        )
        for button in (
            self.pixhawk_auto_detect_button,
            self.pixhawk_usb_discover_button,
            self.pixhawk_mavlink_url_candidate_button,
            self.clear_pixhawk_candidate_button,
            self.clear_pixhawk_pc_candidate_button,
            self.clear_pixhawk_ros2_candidate_button,
            self.apply_pixhawk_candidate_host_button,
            self.apply_pixhawk_candidate_left_button,
            self.apply_pixhawk_candidate_right_button,
            self.apply_pixhawk_ros2_candidate_left_button,
            self.apply_pixhawk_ros2_candidate_right_button,
        ):
            button.setMinimumWidth(0)
            button.setSizePolicy(QtWidgets.QSizePolicy.Ignored, QtWidgets.QSizePolicy.Fixed)
        self._pixhawk_candidate_rows: list[dict[str, object]] = []
        self._pixhawk_candidate_rows_by_source: dict[str, list[dict[str, object]]] = {
            "pc": [],
            "mavlink": [],
            "ros2": [],
        }
        self.swap_camera_button = QtWidgets.QPushButton("좌우 카메라 교체")
        self.swap_camera_button.clicked.connect(self.swap_cameras)
        self.swap_pixhawk_button = QtWidgets.QPushButton("P1/P2 매핑 교체")
        self.swap_pixhawk_button.clicked.connect(self.swap_pixhawks)
        self.check_button = QtWidgets.QPushButton("영상 수신 확인")
        self.check_button.clicked.connect(self.check_cameras)
        self.check_control_button = QtWidgets.QPushButton("제어 확인")
        self.check_control_button.clicked.connect(lambda _checked=False: self.check_wiris_control())
        self.enable_rtsp_button = QtWidgets.QPushButton("영상 송출 활성화")
        self.enable_rtsp_button.clicked.connect(self.enable_wiris_rtsp)
        self.left_stip_button = QtWidgets.QPushButton("STIP 적용")
        self.left_stip_button.clicked.connect(lambda _checked=False: self.apply_wiris_stip("left"))
        self.right_stip_button = QtWidgets.QPushButton("STIP 적용")
        self.right_stip_button.clicked.connect(lambda _checked=False: self.apply_wiris_stip("right"))
        self.camera_connect_button = QtWidgets.QPushButton("카메라 Preview 연결")
        self.camera_connect_button.clicked.connect(self.toggle_camera_connection)
        self.pixhawk_button = QtWidgets.QPushButton("ROS2 연결")
        self.pixhawk_button.clicked.connect(self.toggle_ros2_connection)
        self.left_ros2_button = QtWidgets.QPushButton("P1 ROS2 연결")
        self.left_ros2_button.clicked.connect(lambda _checked=False: self.toggle_ros2_connection("left"))
        self.right_ros2_button = QtWidgets.QPushButton("P2 ROS2 연결")
        self.right_ros2_button.clicked.connect(lambda _checked=False: self.toggle_ros2_connection("right"))
        self.detect_pixhawk_host_ip_button = QtWidgets.QPushButton("PC IP 탐색")
        self.detect_pixhawk_host_ip_button.clicked.connect(self.discover_pc_pixhawk_ip)
        self.left_mavlink_button = QtWidgets.QPushButton("P1 MAVLink 연결")
        self.left_mavlink_button.clicked.connect(lambda _checked=False: self.toggle_mavlink_connection("left"))
        self.right_mavlink_button = QtWidgets.QPushButton("P2 MAVLink 연결")
        self.right_mavlink_button.clicked.connect(lambda _checked=False: self.toggle_mavlink_connection("right"))
        self.left_mavlink_bootstrap_button = QtWidgets.QPushButton("P1 ETH MAV 시작(USB)")
        self.left_mavlink_bootstrap_button.clicked.connect(
            lambda _checked=False: self.bootstrap_ethernet_mavlink_from_usb("left")
        )
        self.right_mavlink_bootstrap_button = QtWidgets.QPushButton("P2 ETH MAV 시작(USB)")
        self.right_mavlink_bootstrap_button.clicked.connect(
            lambda _checked=False: self.bootstrap_ethernet_mavlink_from_usb("right")
        )
        self.left_uxrce_param_button = QtWidgets.QPushButton("P1 uXRCE 설정")
        self.left_uxrce_param_button.setEnabled(False)
        self.left_uxrce_param_button.clicked.connect(lambda _checked=False: self.apply_uxrce_params_from_mavlink("left"))
        self.right_uxrce_param_button = QtWidgets.QPushButton("P2 uXRCE 설정")
        self.right_uxrce_param_button.setEnabled(False)
        self.right_uxrce_param_button.clicked.connect(lambda _checked=False: self.apply_uxrce_params_from_mavlink("right"))
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
        self.domain_spin = QtWidgets.QSpinBox()
        self.domain_spin.setRange(0, 232)
        self.domain_spin.setValue(self.settings.ros_domain_id)
        self.left_namespace_edit = QtWidgets.QLineEdit(self.settings.left_namespace)
        self.right_namespace_edit = QtWidgets.QLineEdit(self.settings.right_namespace)
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
        self.mavlink_configure_uxrce_agent_checkbox = self._build_toggle_button(
            "PX4 uXRCE Agent 설정",
            self.settings.mavlink_configure_uxrce_agent,
        )
        self.mavlink_stream_rate_spin = QtWidgets.QSpinBox()
        self.mavlink_stream_rate_spin.setRange(1_000, 10_000_000)
        self.mavlink_stream_rate_spin.setValue(self.settings.mavlink_stream_rate)
        self.uxrce_port_spin = QtWidgets.QSpinBox()
        self.uxrce_port_spin.setRange(1, 65535)
        self.uxrce_port_spin.setValue(self.settings.uxrce_port)
        self.browse_weights_button = QtWidgets.QPushButton("파일 선택")
        self.browse_weights_button.clicked.connect(self.browse_yolo_weights)
        self.start_button = QtWidgets.QPushButton("YOLO 측정 시작")
        self.start_button.clicked.connect(self.toggle_yolo_detection)
        self.detect_control_button.setText("Telnet port 탐색")
        self.right_detect_control_button.setText("Telnet port 탐색")
        self.detect_rtsp_button.setText("영상 주소 탐색")
        self.right_detect_rtsp_button.setText("영상 주소 탐색")
        self.auto_wiris_connect_button.setText("WIRIS 자동 연결")
        self.detect_pc_ip_button.setText("PC IP 탐색")
        self.apply_pc_ip_button.setText("PC IP 적용")
        self.apply_candidate_nic_button.setText("선택 NIC")
        self.apply_candidate_left_button.setText("Left에 적용")
        self.apply_candidate_right_button.setText("Right에 적용")
        self.clear_camera_candidate_button.setText("Clear")
        self.left_control_apply_button.setText("Telnet 적용")
        self.right_control_apply_button.setText("Telnet 적용")
        self.left_rtsp_apply_button.setText("RTSP 적용")
        self.right_rtsp_apply_button.setText("RTSP 적용")
        self.pixhawk_auto_detect_button.setText("Pixhawk 자동 탐색")
        self.pixhawk_usb_discover_button.setText("USB 연결 탐색")
        self.pixhawk_mavlink_url_candidate_button.setText("MAVLink URL 후보")
        self.detect_pixhawk_host_ip_button.setText("PC IP 탐색")
        self.clear_pixhawk_candidate_button.setText("Clear")
        self.apply_pixhawk_candidate_host_button.setText("Host에 적용")
        self.apply_pixhawk_candidate_left_button.setText("P1에 적용")
        self.apply_pixhawk_candidate_right_button.setText("P2에 적용")
        self.swap_camera_button.setText("좌우 카메라 교체")
        self.swap_pixhawk_button.setText("P1/P2 매핑 교체")
        self.check_button.setText("영상 수신 확인")
        self.check_control_button.setText("제어 확인")
        self.enable_rtsp_button.setText("영상 송출 활성화")
        self.camera_connect_button.setText("카메라 Preview 연결")
        self.pixhawk_button.setText("ROS2 연결")
        self.left_ros2_button.setText("P1 ROS2 연결")
        self.right_ros2_button.setText("P2 ROS2 연결")
        self.left_mavlink_button.setText("P1 MAVLink 연결")
        self.right_mavlink_button.setText("P2 MAVLink 연결")
        self.left_mavlink_bootstrap_button.setText("P1 ETH MAV 시작(USB)")
        self.right_mavlink_bootstrap_button.setText("P2 ETH MAV 시작(USB)")
        self.left_uxrce_param_button.setText("P1 uXRCE 설정")
        self.right_uxrce_param_button.setText("P2 uXRCE 설정")
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
        yolo_gate_layout.addWidget(self.yolo_gate_min_bbox_checkbox, 1, 0)
        yolo_gate_layout.addWidget(QtWidgets.QLabel("W"), 1, 1)
        yolo_gate_layout.addWidget(self.yolo_gate_min_width_spin, 1, 2)
        yolo_gate_layout.addWidget(QtWidgets.QLabel("H"), 1, 3)
        yolo_gate_layout.addWidget(self.yolo_gate_min_height_spin, 1, 4)
        yolo_gate_layout.addWidget(QtWidgets.QLabel("Area"), 1, 5)
        yolo_gate_layout.addWidget(self.yolo_gate_min_area_spin, 1, 6)
        yolo_gate_layout.addWidget(self.yolo_gate_matching_checkbox, 2, 0)
        yolo_gate_layout.addWidget(self.yolo_gate_time_basis_combo, 2, 1)
        yolo_gate_layout.addWidget(self.yolo_gate_time_delta_checkbox, 2, 2)
        yolo_gate_layout.addWidget(self.yolo_gate_time_delta_spin, 2, 3)
        yolo_gate_layout.addWidget(self.yolo_gate_vertical_delta_checkbox, 2, 4)
        yolo_gate_layout.addWidget(self.yolo_gate_vertical_delta_spin, 2, 5)
        self.yolo_gate_status_label = QtWidgets.QLabel("Gate OFF: 기존 YOLO 후보 선택/매칭을 사용합니다.")
        self.yolo_gate_status_label.setObjectName("statusStrip")
        self.yolo_gate_status_label.setWordWrap(True)
        yolo_gate_layout.addWidget(self.yolo_gate_status_label, 3, 0, 1, 7)
        yolo_form.addWidget(yolo_gate_group, 3, 0, 1, 6)
        for checkbox in (
            self.yolo_gate_class_checkbox,
            self.yolo_gate_min_bbox_checkbox,
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

        ros2_candidate_group = QtWidgets.QGroupBox("ROS2 Namespace")
        ros2_candidate_group.setObjectName("pixhawkRos2CandidateGroup")
        ros2_candidate_group.setSizePolicy(QtWidgets.QSizePolicy.Ignored, QtWidgets.QSizePolicy.Preferred)
        ros2_candidate_group.setMinimumHeight(320)
        ros2_candidate_layout = QtWidgets.QVBoxLayout(ros2_candidate_group)
        ros2_candidate_layout.setSpacing(6)
        ros2_candidate_layout.setContentsMargins(8, 18, 8, 8)
        ros2_candidate_layout.addWidget(self.pixhawk_auto_detect_button)
        ros2_candidate_layout.addWidget(self.ros2_pixhawk_candidate_table)
        ros2_candidate_actions = QtWidgets.QHBoxLayout()
        ros2_candidate_actions.setSpacing(6)
        ros2_candidate_actions.addWidget(self.clear_pixhawk_ros2_candidate_button, 1)
        ros2_candidate_actions.addWidget(self.apply_pixhawk_ros2_candidate_left_button, 1)
        ros2_candidate_actions.addWidget(self.apply_pixhawk_ros2_candidate_right_button, 1)
        ros2_candidate_layout.addLayout(ros2_candidate_actions)

        pixhawk_candidate_layout.addWidget(pc_candidate_group)
        pixhawk_candidate_layout.addWidget(mavlink_candidate_group)
        pixhawk_candidate_layout.addWidget(ros2_candidate_group)
        self.pixhawk_pc_candidate_group = pc_candidate_group
        self.pixhawk_mavlink_candidate_group = mavlink_candidate_group
        self.pixhawk_ros2_candidate_group = ros2_candidate_group
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

        def build_pixhawk_ros2_side_group(
            title: str,
            object_name: str,
            namespace_edit: QtWidgets.QLineEdit,
        ) -> QtWidgets.QGroupBox:
            side_group = QtWidgets.QGroupBox(title)
            side_group.setObjectName(object_name)
            side_form = QtWidgets.QGridLayout(side_group)
            side_form.setHorizontalSpacing(8)
            side_form.setVerticalSpacing(10)
            side_form.setContentsMargins(12, 24, 12, 12)
            side_form.setColumnStretch(0, 0)
            side_form.setColumnStretch(1, 1)
            label = QtWidgets.QLabel("ROS2 Namespace")
            label.setAlignment(QtCore.Qt.AlignRight | QtCore.Qt.AlignVCenter)
            side_form.addWidget(label, 0, 0)
            side_form.addWidget(namespace_edit, 0, 1)
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

        pixhawk_ros2_group = QtWidgets.QGroupBox("3 ROS2 Telemetry")
        pixhawk_ros2_group.setObjectName("pixhawkRos2TelemetryGroup")
        pixhawk_ros2_layout = QtWidgets.QVBoxLayout(pixhawk_ros2_group)
        pixhawk_ros2_layout.setSpacing(8)
        pixhawk_ros2_layout.setContentsMargins(12, 24, 12, 12)
        pixhawk_ros2_options = QtWidgets.QGridLayout()
        pixhawk_ros2_options.setHorizontalSpacing(8)
        pixhawk_ros2_options.setVerticalSpacing(8)
        pixhawk_ros2_options.setColumnStretch(1, 1)
        pixhawk_ros2_options.setColumnStretch(3, 1)
        pixhawk_ros2_options.addWidget(QtWidgets.QLabel("ROS_DOMAIN_ID"), 0, 0)
        pixhawk_ros2_options.addWidget(self.domain_spin, 0, 1)
        pixhawk_ros2_options.addWidget(QtWidgets.QLabel("uXRCE 포트"), 0, 2)
        pixhawk_ros2_options.addWidget(self.uxrce_port_spin, 0, 3)
        pixhawk_ros2_options.addWidget(self.mavlink_configure_uxrce_agent_checkbox, 0, 4)
        pixhawk_ros2_pair_layout = QtWidgets.QHBoxLayout()
        pixhawk_ros2_pair_layout.setSpacing(8)
        left_ros2_group = build_pixhawk_ros2_side_group(
            "Pixhawk 1 ROS2",
            "leftPixhawkRos2WorkflowGroup",
            self.left_namespace_edit,
        )
        right_ros2_group = build_pixhawk_ros2_side_group(
            "Pixhawk 2 ROS2",
            "rightPixhawkRos2WorkflowGroup",
            self.right_namespace_edit,
        )
        pixhawk_ros2_pair_layout.addWidget(left_ros2_group, 1)
        pixhawk_ros2_pair_layout.addWidget(right_ros2_group, 1)
        pixhawk_ros2_actions = QtWidgets.QHBoxLayout()
        pixhawk_ros2_actions.setSpacing(6)
        for button in (
            self.left_ros2_button,
            self.right_ros2_button,
            self.left_uxrce_param_button,
            self.right_uxrce_param_button,
        ):
            button.setMinimumWidth(128)
            button.setSizePolicy(QtWidgets.QSizePolicy.Ignored, QtWidgets.QSizePolicy.Fixed)
            pixhawk_ros2_actions.addWidget(button, 1)
        pixhawk_ros2_layout.addLayout(pixhawk_ros2_options)
        pixhawk_ros2_layout.addLayout(pixhawk_ros2_pair_layout)
        pixhawk_ros2_layout.addLayout(pixhawk_ros2_actions)
        self.left_pixhawk_ros2_workflow_group = left_ros2_group
        self.right_pixhawk_ros2_workflow_group = right_ros2_group
        self.pixhawk_ros2_telemetry_group = pixhawk_ros2_group
        self.pixhawk_connection_actions_group = pixhawk_ros2_group
        pixhawk_form.addWidget(pixhawk_ros2_group, 3, 0, 1, 6)

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
        pixhawk_form.addWidget(pixhawk_advanced_group, 4, 0, 1, 6)
        self.pixhawk_connection_label = QtWidgets.QLabel(STARTUP_PIXHAWK_STATUS)
        self.pixhawk_connection_label.setObjectName("statusStrip")
        pixhawk_form.addWidget(self.pixhawk_connection_label, 5, 0, 1, 6)

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
        pixhawk_summary_layout = QtWidgets.QVBoxLayout(pixhawk_summary_tab)
        pixhawk_1_tab = QtWidgets.QWidget()
        pixhawk_1_layout = QtWidgets.QVBoxLayout(pixhawk_1_tab)
        pixhawk_2_tab = QtWidgets.QWidget()
        pixhawk_2_layout = QtWidgets.QVBoxLayout(pixhawk_2_tab)
        pixhawk_attitude_tab = QtWidgets.QWidget()
        pixhawk_common_layout = QtWidgets.QVBoxLayout(pixhawk_attitude_tab)
        custom_watch_tab = QtWidgets.QWidget()
        custom_watch_layout = QtWidgets.QVBoxLayout(custom_watch_tab)
        pixhawk_tabs.addTab(pixhawk_summary_tab, "요약")
        pixhawk_tabs.addTab(pixhawk_1_tab, "픽스호크 1")
        pixhawk_tabs.addTab(pixhawk_2_tab, "픽스호크 2")
        self.pixhawk_tabs = pixhawk_tabs
        advanced_tabs.addTab(wiris_command_panel, "WIRIS SDK")
        advanced_tabs.addTab(custom_watch_tab, "필드 감시")
        advanced_tabs.addTab(baseline_field_browser_tab, "Baseline 필드")
        self._handle_alignment_step_changed(alignment_tabs.currentIndex())
        self.advanced_wiris_command_tab_index = advanced_tabs.indexOf(wiris_command_panel)
        self.advanced_custom_watch_tab_index = advanced_tabs.indexOf(custom_watch_tab)
        self.advanced_baseline_field_tab_index = advanced_tabs.indexOf(baseline_field_browser_tab)
        self.wiris_command_tab_index = self.advanced_wiris_command_tab_index

        custom_watch_group = QtWidgets.QGroupBox("사용자 지정 ROS2/uORB 또는 MAVLink 감시")
        custom_watch_group_layout = QtWidgets.QVBoxLayout(custom_watch_group)
        custom_watch_form = QtWidgets.QGridLayout()
        self.custom_watch_name_edit = QtWidgets.QLineEdit("local_z")
        self.custom_watch_topic_edit = QtWidgets.QLineEdit("/uav_0/fmu/out/vehicle_local_position")
        self.custom_watch_field_edit = QtWidgets.QLineEdit("z")
        self.custom_watch_add_button = QtWidgets.QPushButton("감시 추가")
        self.custom_watch_add_button.clicked.connect(self.add_custom_topic_watch_from_ui)
        self.custom_watch_discover_button = QtWidgets.QPushButton("ROS2 토픽 찾기")
        self.custom_watch_discover_button.clicked.connect(self.discover_custom_topic_watch)
        self.custom_watch_mavlink_discover_button = QtWidgets.QPushButton("MAVLink 필드 찾기")
        self.custom_watch_mavlink_discover_button.clicked.connect(self.discover_custom_mavlink_watch)
        self.custom_watch_remove_combo = QtWidgets.QComboBox()
        self.custom_watch_remove_combo.setMinimumWidth(260)
        self.custom_watch_remove_button = QtWidgets.QPushButton("감시 제거")
        self.custom_watch_remove_button.clicked.connect(self.remove_selected_custom_topic_watch)
        custom_watch_form.addWidget(QtWidgets.QLabel("이름"), 0, 0)
        custom_watch_form.addWidget(self.custom_watch_name_edit, 0, 1)
        custom_watch_form.addWidget(QtWidgets.QLabel("토픽"), 1, 0)
        custom_watch_form.addWidget(self.custom_watch_topic_edit, 1, 1)
        custom_watch_form.addWidget(QtWidgets.QLabel("필드"), 2, 0)
        custom_watch_form.addWidget(self.custom_watch_field_edit, 2, 1)
        custom_watch_buttons = QtWidgets.QHBoxLayout()
        custom_watch_buttons.addWidget(self.custom_watch_discover_button)
        custom_watch_buttons.addWidget(self.custom_watch_mavlink_discover_button)
        custom_watch_buttons.addWidget(self.custom_watch_add_button)
        custom_watch_buttons.addWidget(self.custom_watch_remove_combo)
        custom_watch_buttons.addWidget(self.custom_watch_remove_button)
        custom_watch_buttons.addStretch(1)
        custom_watch_form.addLayout(custom_watch_buttons, 3, 0, 1, 2)
        custom_watch_group_layout.addLayout(custom_watch_form)
        self.custom_watch_plot = TelemetryHistoryPlot("사용자 지정 ROS2/uORB/MAVLink 숫자 필드", "값")
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
        self._build_baseline_calculation_view(alignment_mag_layout, include_source_controls=False, include_alignment_plot=False)
        self._build_baseline_attitude_alignment_tab(alignment_attitude_layout)
        self.refresh_baseline_field_browser()
        self._refresh_baseline_alignment_ui()

        zoom_group = QtWidgets.QGroupBox("WIRIS 줌 명령 제어")
        zoom_group.setObjectName("checkerboardZoomControlGroup")
        zoom_group_layout = QtWidgets.QVBoxLayout(zoom_group)
        self.zoom_level_spin = QtWidgets.QSpinBox()
        self.zoom_level_spin.setRange(MIN_ZOOM_LEVEL, MAX_ZOOM_LEVEL)
        self.zoom_level_spin.setValue(10)
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

        calibration_group = QtWidgets.QGroupBox("체커보드 내부 파라미터 보정")
        calibration_group.setObjectName("checkerboardIntrinsicCalibrationGroup")
        calib_form = QtWidgets.QGridLayout(calibration_group)
        calib_form.setHorizontalSpacing(6)
        calib_form.setVerticalSpacing(6)
        calib_form.setColumnStretch(5, 1)
        calib_form.setColumnStretch(7, 0)
        self.calib_cols_spin = QtWidgets.QSpinBox()
        self.calib_cols_spin.setRange(2, 30)
        self.calib_cols_spin.setValue(13)
        self.calib_cols_spin.valueChanged.connect(self._refresh_all_calibration_previews)
        self.calib_rows_spin = QtWidgets.QSpinBox()
        self.calib_rows_spin.setRange(2, 30)
        self.calib_rows_spin.setValue(8)
        self.calib_rows_spin.valueChanged.connect(self._refresh_all_calibration_previews)
        self.calib_square_spin = QtWidgets.QDoubleSpinBox()
        self.calib_square_spin.setRange(1.0, 500.0)
        self.calib_square_spin.setValue(40.0)
        self.calib_square_spin.setSuffix(" mm")
        self.calib_min_images_spin = QtWidgets.QSpinBox()
        self.calib_min_images_spin.setRange(3, 100)
        self.calib_min_images_spin.setValue(20)
        for spin, width in (
            (self.calib_cols_spin, 72),
            (self.calib_rows_spin, 72),
            (self.calib_square_spin, 96),
            (self.calib_min_images_spin, 72),
        ):
            spin.setProperty("compactFormField", True)
            spin.setMinimumWidth(width)
            spin.setSizePolicy(QtWidgets.QSizePolicy.Fixed, QtWidgets.QSizePolicy.Fixed)
        initial_intrinsics = self._intrinsics_storage_base_path()
        self.calib_dir_edit = QtWidgets.QLineEdit(str(initial_intrinsics.parent))
        self.calib_dir_edit.setProperty("compactFormField", True)
        self.calib_dir_edit.setMinimumWidth(260)
        self.capture_calibration_button = QtWidgets.QPushButton("캡처")
        self.capture_calibration_button.clicked.connect(self.capture_calibration_images)
        self.run_calibration_button = QtWidgets.QPushButton("보정 실행")
        self.run_calibration_button.clicked.connect(self.run_intrinsic_calibration)
        self.save_intrinsics_button = QtWidgets.QPushButton("저장")
        self.save_intrinsics_button.clicked.connect(self.save_intrinsics)
        self.apply_intrinsics_button = QtWidgets.QPushButton("계산에 적용")
        self.apply_intrinsics_button.clicked.connect(self.load_intrinsics)
        self.calibration_status_label = QtWidgets.QLabel("좌/우 중 체커보드가 보이는 카메라 프레임을 캡처하세요.")
        self.calibration_status_label.setObjectName("statusStrip")
        self.calibration_status_label.setWordWrap(True)
        self.capture_calibration_button.setText("캡처")
        self.run_calibration_button.setText("보정 실행")
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
            for text in ("내부 열", "내부 행", "격자 크기", "최소 이미지", "저장 경로")
        ]
        for label in compact_calibration_labels:
            label.setProperty("compactFormLabel", True)
        cols_label, rows_label, square_label, min_images_label, dir_label = compact_calibration_labels

        calib_form.addWidget(cols_label, 0, 0)
        calib_form.addWidget(self.calib_cols_spin, 0, 1)
        calib_form.addWidget(rows_label, 0, 2)
        calib_form.addWidget(self.calib_rows_spin, 0, 3)
        calib_form.addWidget(square_label, 0, 4)
        calib_form.addWidget(self.calib_square_spin, 0, 5)
        calib_form.addWidget(min_images_label, 0, 6)
        calib_form.addWidget(self.calib_min_images_spin, 0, 7)
        calib_form.addWidget(dir_label, 1, 0)
        calib_form.addWidget(self.calib_dir_edit, 1, 1, 1, 5)
        calib_form.addWidget(self.capture_calibration_button, 1, 6)
        calib_form.addWidget(self.run_calibration_button, 1, 7)

        calibration_top_stack = QtWidgets.QVBoxLayout()
        calibration_top_stack.setSpacing(6)
        calibration_top_stack.addWidget(zoom_group)
        calibration_top_stack.addWidget(calibration_group)
        calibration_top_stack.addWidget(self.calibration_status_label)
        checkerboard_calibration_layout.addLayout(calibration_top_stack)
        checkerboard_calibration_layout.addLayout(calibration_preview_row, 2)
        checkerboard_calibration_layout.addWidget(self.calibration_result_text, 1)

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

        self.triangulation_quality_plot = TriangulationQualityPlot()
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
            "현재 매칭 프레임쌍 기록을 동일한 이름의 CSV와 목적별 탭 XLSX로 함께 저장합니다."
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
        save_form.addWidget(self.save_history_button, 0, 0)
        save_form.addWidget(self.save_settings_button, 0, 1)
        save_form.addWidget(self.session_summary_label, 1, 0, 1, 2)
        save_form.addWidget(self.recording_status_label, 2, 0, 1, 2)
        save_session_layout.addWidget(save_group)
        save_session_layout.addStretch(1)

        self.recording_table_group = QtWidgets.QGroupBox("매칭 프레임쌍 실시간 보기")
        recording_table_layout = QtWidgets.QVBoxLayout(self.recording_table_group)
        self.recording_sample_table = QtWidgets.QTableWidget(0, len(RECORDED_SAMPLE_TABLE_COLUMNS))
        self.recording_sample_table.setObjectName("recordingSampleTable")
        self.recording_sample_table.setHorizontalHeaderLabels(
            [label for _field, label in RECORDED_SAMPLE_TABLE_COLUMNS]
        )
        self.recording_sample_table.setEditTriggers(QtWidgets.QAbstractItemView.NoEditTriggers)
        self.recording_sample_table.setSelectionBehavior(QtWidgets.QAbstractItemView.SelectRows)
        self.recording_sample_table.setSelectionMode(QtWidgets.QAbstractItemView.SingleSelection)
        self.recording_sample_table.setAlternatingRowColors(True)
        self.recording_sample_table.verticalHeader().setVisible(False)
        self.recording_sample_table.horizontalHeader().setSectionResizeMode(QtWidgets.QHeaderView.ResizeToContents)
        self.recording_sample_table.horizontalHeader().setSectionResizeMode(
            len(RECORDED_SAMPLE_TABLE_COLUMNS) - 1,
            QtWidgets.QHeaderView.Stretch,
        )
        self.recording_sample_table.itemSelectionChanged.connect(self._refresh_recording_sample_detail)
        recording_table_layout.addWidget(self.recording_sample_table, 2)
        self.recording_sample_detail = QtWidgets.QPlainTextEdit()
        self.recording_sample_detail.setObjectName("recordingSampleDetail")
        self.recording_sample_detail.setReadOnly(True)
        self.recording_sample_detail.setMaximumBlockCount(300)
        self.recording_sample_detail.setPlaceholderText("테이블에서 샘플을 선택하면 전체 CSV 필드가 표시됩니다.")
        recording_table_layout.addWidget(self.recording_sample_detail, 1)
        recording_samples_layout.addWidget(self.recording_table_group, 1)

        self._build_clock_sync_tab(time_layout)
        self._polish_controls()
        self._refresh_session_toggle_buttons()
        self._install_tooltips()
        self._set_focal_override_enabled(self.manual_focal_checkbox.isChecked())
        self._update_distance_settings_visibility()
        self._refresh_camera_parameters_tab()
        self._refresh_operation_header()
        QtCore.QTimer.singleShot(0, self._initial_status)
        QtCore.QTimer.singleShot(0, self._auto_load_intrinsics)

    def _build_toggle_button(self, text: str, checked: bool = False) -> QtWidgets.QPushButton:
        button = QtWidgets.QPushButton(text)
        button.setCheckable(True)
        button.setChecked(bool(checked))
        button.setProperty("role", "toggle")
        button.setAccessibleName(text)
        return button

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
        bbox_enabled = self.yolo_gate_min_bbox_checkbox.isChecked()
        time_basis_enabled = self.yolo_gate_matching_checkbox.isChecked()
        time_delta_enabled = self.yolo_gate_time_delta_checkbox.isChecked()
        vertical_delta_enabled = self.yolo_gate_vertical_delta_checkbox.isChecked()
        self.yolo_gate_class_edit.setEnabled(class_enabled)
        for widget in (
            self.yolo_gate_min_width_spin,
            self.yolo_gate_min_height_spin,
            self.yolo_gate_min_area_spin,
        ):
            widget.setEnabled(bbox_enabled)
        for widget in (
            self.yolo_gate_time_basis_combo,
        ):
            widget.setEnabled(time_basis_enabled)
        self.yolo_gate_time_delta_spin.setEnabled(time_delta_enabled)
        self.yolo_gate_vertical_delta_spin.setEnabled(vertical_delta_enabled)
        enabled = []
        if class_enabled:
            enabled.append("class")
        if bbox_enabled:
            enabled.append("bbox")
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
        if self._yolo_matching_gate_enabled() and hasattr(self, "yolo_gate_time_basis_combo"):
            return str(self.yolo_gate_time_basis_combo.currentData() or "camera_rtc_required")
        return "detection"

    def _yolo_match_timestamp_s(self, observation: DetectionObservation) -> float | None:
        return detection_timestamp_for_basis(observation, self._effective_yolo_time_basis())

    def _effective_yolo_time_basis(self) -> str:
        if self._yolo_time_basis() == "camera_rtcp" and self._runtime_pair_uses_app_fallback:
            return "detection"
        return self._yolo_time_basis()

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
        self.camera_connect_button.setText("카메라 Preview 해제" if self.camera_connected else "카메라 Preview 연결")
        self._set_button_role(self.camera_connect_button, "danger" if self.camera_connected else "primary")

        self.start_button.setText("YOLO 측정 중지" if self.yolo_enabled else "YOLO 측정 시작")
        self._set_button_role(self.start_button, "danger" if self.yolo_enabled else "primary")

        self.record_button.setText("세션 기록 중지" if self._session_recording_active() else "세션 기록 시작")
        self._set_button_role(self.record_button, "danger" if self._session_recording_active() else "success")

        connected_ros2_sides = self._ros2_connected_sides()
        self.pixhawk_button.setText("ROS2 전체 해제" if connected_ros2_sides else "ROS2 전체 연결")
        self._set_button_role(self.pixhawk_button, "danger" if connected_ros2_sides else "primary")
        self.left_ros2_button.setText("P1 ROS2 해제" if "left" in connected_ros2_sides else "P1 ROS2 연결")
        self._set_button_role(self.left_ros2_button, "danger" if "left" in connected_ros2_sides else "primary")
        self.right_ros2_button.setText("P2 ROS2 해제" if "right" in connected_ros2_sides else "P2 ROS2 연결")
        self._set_button_role(self.right_ros2_button, "danger" if "right" in connected_ros2_sides else "primary")

        connected_mavlink_sides = self._mavlink_open_sides()
        self.left_mavlink_button.setText("P1 MAVLink 해제" if "left" in connected_mavlink_sides else "P1 MAVLink 연결")
        self._set_button_role(self.left_mavlink_button, "danger" if "left" in connected_mavlink_sides else "primary")
        self.right_mavlink_button.setText("P2 MAVLink 해제" if "right" in connected_mavlink_sides else "P2 MAVLink 연결")
        self._set_button_role(self.right_mavlink_button, "danger" if "right" in connected_mavlink_sides else "primary")

    def _install_tooltips(self) -> None:
        tooltips = {
            self.interface_edit: "카메라 네트워크 도달성을 확인할 때 사용할 로컬 네트워크 인터페이스입니다.",
            self.host_address_edit: "WIRIS 카메라망에서 PC가 사용할 IP입니다. 카메라 IP가 아니며 Pixhawk host 주소와도 별개입니다.",
            self.pixhawk_host_address_edit: "Pixhawk가 uXRCE-DDS 대상으로 사용할 PC 주소입니다. 예: 10.0.0.10",
            self.left_url_edit: "왼쪽 카메라 스트림의 전체 RTSP URL입니다. 카메라 IP, 포트, 경로를 포함합니다.",
            self.right_url_edit: "오른쪽 카메라 스트림의 전체 RTSP URL입니다. 카메라 IP, 포트, 경로를 포함합니다.",
            self.left_rtsp_ip_edit: "왼쪽 RTSP URL에 적용할 OSD IP Address 값입니다. Telnet 제어 host는 이 버튼으로 바꾸지 않습니다.",
            self.right_rtsp_ip_edit: "오른쪽 RTSP URL에 적용할 OSD IP Address 값입니다. Telnet 제어 host는 이 버튼으로 바꾸지 않습니다.",
            self.left_rtsp_apply_button: "왼쪽 RTSP URL의 host만 OSD IP Address 값으로 바꿉니다. RTSP port는 URL 안의 포트를 유지합니다.",
            self.right_rtsp_apply_button: "오른쪽 RTSP URL의 host만 OSD IP Address 값으로 바꿉니다. RTSP port는 URL 안의 포트를 유지합니다.",
            self.weights_edit: "거리 추정 전 객체 검출에 사용할 YOLO 모델 weight입니다.",
            self.browse_weights_button: ".pt, .onnx, .engine 같은 YOLO weight 파일을 선택합니다.",
            self.device_combo: "YOLO 추론 장치를 선택합니다. CPU는 cpu, CUDA는 cuda:0으로 실행합니다.",
            self.input_size_spin: "YOLO 입력 이미지 크기입니다. 값이 크면 인식 품질은 좋아질 수 있지만 처리 속도는 낮아질 수 있습니다.",
            self.conf_spin: "YOLO 검출을 인정할 최소 confidence 값입니다.",
            self.iou_spin: "YOLO 중복 박스 제거에 사용하는 IoU 기준입니다.",
            self.preview_width_spin: (
                "GUI 미리보기에 표시할 최대 너비입니다. 1920으로 두면 1080p WIRIS 프레임을 추가 축소 없이 "
                "Qt에 넘기고, 검출은 원본 디코딩 프레임 기준으로 수행됩니다."
            ),
            self.rtsp_latency_spin: "GStreamer RTP jitter buffer 지연 시간입니다. 낮추면 지연은 줄지만 네트워크 흔들림에 민감해집니다.",
            self.yolo_event_queue_size_spin: (
                "YOLO worker가 detection, measurement frame, stats, status 이벤트를 보존하기 위해 쌓아둘 큐 크기입니다."
            ),
            self.yolo_frame_queue_size_spin: (
                "실시간 preview frame 큐 크기입니다. 작을수록 지연이 낮고, 포화되면 preview frame만 드롭합니다."
            ),
            self.decoder_combo: "RTSP H.264 디코더입니다. Auto는 NVIDIA를 먼저 시도하고 불가하면 소프트웨어 디코더로 전환합니다.",
            self.stream_backend_combo: (
                "RTSP 수신 백엔드입니다. GStreamer는 기본 저지연 경로, OpenCV/FFmpeg는 단순한 대체 경로, "
                "OpenCV Auto는 OpenCV가 사용 가능한 백엔드를 자동 선택, PyAV는 av 패키지가 있을 때 FFmpeg를 직접 사용합니다."
            ),
            self.rtsp_transport_combo: "RTSP 전송 방식입니다. UDP는 지연 누적이 적고 TCP는 안정적이지만 지연이 쌓일 수 있습니다.",
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
            self.pixhawk_auto_detect_button: "현재 ROS_DOMAIN_ID에서 ROS2 PX4 토픽을 스캔해 Pixhawk namespace를 채웁니다.",
            self.pixhawk_usb_discover_button: "OS에 보이는 USB Pixhawk 포트와 MAVLink heartbeat/권한 상태를 후보 표에 표시합니다. 자동 할당은 하지 않습니다.",
            self.pixhawk_mavlink_url_candidate_button: "Ethernet MAVLink UDP 기본 URL 후보를 표에 추가합니다. PX4가 해당 UDP 포트로 MAVLink를 보내고 있어야 연결됩니다.",
            self.detect_pixhawk_host_ip_button: "P1/P2 Pixhawk ETH IP 대역에 맞는 이 PC의 IPv4를 찾아 Pixhawk Host IP에 채웁니다.",
            self.swap_camera_button: "좌/우 카메라 RTSP URL과 IP Address/Telnet port를 서로 바꿉니다. 실행 중인 스트림은 새 매핑으로 재시작됩니다.",
            self.swap_pixhawk_button: (
                "P1/P2의 MAVLink URL, USB 제어 포트, ETH IP, ROS2 namespace 매핑을 서로 바꿉니다. "
                "실행 중인 구독은 새 매핑으로 재시작됩니다."
            ),
            self.check_button: "좌/우 RTSP 호스트의 RTSP 포트가 열려 있는지 확인합니다.",
            self.camera_connect_button: "좌/우 RTSP Preview를 연결하거나, 연결 중이면 카메라 Preview와 활성 YOLO 검출을 중지합니다.",
            self.pixhawk_button: "P1/P2 ROS2 Pixhawk 세션을 모두 연결하거나, 연결 중이면 ROS2 토픽 구독을 모두 중지합니다.",
            self.left_ros2_button: "Pixhawk 1 ROS2 세션만 연결합니다. /uav_0 namespace의 topic만 왼쪽 Pixhawk 값으로 구독합니다.",
            self.right_ros2_button: "Pixhawk 2 ROS2 세션만 연결합니다. /uav_1 namespace의 topic만 오른쪽 Pixhawk 값으로 구독합니다.",
            self.left_mavlink_button: "Pixhawk 1 MAVLink 세션을 연결하거나, 연결 중이면 P1 MAVLink 링크만 중지합니다.",
            self.right_mavlink_button: "Pixhawk 2 MAVLink 세션을 연결하거나, 연결 중이면 P2 MAVLink 링크만 중지합니다.",
            self.left_uxrce_param_button: "활성 P1 MAVLink 연결로 PX4 UXRCE_DDS_* 파라미터를 Pixhawk Host IP에 맞게 설정하고 확인합니다.",
            self.right_uxrce_param_button: "활성 P2 MAVLink 연결로 PX4 UXRCE_DDS_* 파라미터를 Pixhawk Host IP에 맞게 설정하고 확인합니다.",
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
            self.domain_spin: "PX4/ROS2 uORB 토픽 탐색에 사용할 ROS_DOMAIN_ID입니다.",
            self.left_namespace_edit: "Pixhawk 1의 ROS2 namespace입니다. 왼쪽 카메라와 매핑됩니다.",
            self.right_namespace_edit: "Pixhawk 2의 ROS2 namespace입니다. 오른쪽 카메라와 매핑됩니다.",
            self.left_pixhawk_eth_ip_edit: "왼쪽 Pixhawk에 적용하고 저장할 eth0 고정 IP입니다. 예: 10.0.0.2",
            self.right_pixhawk_eth_ip_edit: "오른쪽 Pixhawk에 적용하고 저장할 eth0 고정 IP입니다. 예: 10.0.0.3",
            self.left_mavlink_url_edit: "Pixhawk 1 MAVLink URL입니다. 예: udpin:0.0.0.0:14550, /dev/ttyACM0, tcp:192.168.0.3:5760",
            self.right_mavlink_url_edit: "Pixhawk 2 MAVLink URL입니다. 예: udpin:0.0.0.0:14551, /dev/ttyACM1, tcp:10.41.10.2:5760",
            self.left_mavlink_baud_spin: "Pixhawk 1 MAVLink URL이 시리얼 장치일 때 사용할 baud rate입니다.",
            self.right_mavlink_baud_spin: "Pixhawk 2 MAVLink URL이 시리얼 장치일 때 사용할 baud rate입니다.",
            self.mavlink_configure_streams_checkbox: "MAVLink 연결 후 MAV_SYS_ID 설정과 Pixhawk Host IP 방향 MAVLink stream 시작 명령을 보냅니다.",
            self.mavlink_stream_rate_spin: "PX4 shell로 MAVLink stream을 설정할 때 사용할 mavlink start -r 값입니다.",
            self.mavlink_configure_uxrce_agent_checkbox: "MAVLink 연결 후 PX4 uXRCE-DDS Client가 접속할 PC Micro XRCE-DDS Agent IP/port 파라미터를 설정하고 readback으로 확인합니다.",
            self.uxrce_port_spin: "호스트 PC의 Micro XRCE-DDS Agent UDP 포트입니다. 일반적으로 8888입니다.",
            self.custom_watch_name_edit: "사용자 지정 ROS2/uORB 필드 감시의 표시 이름입니다.",
            self.custom_watch_topic_edit: "감시할 전체 ROS2 토픽입니다. 예: /uav_0/fmu/out/vehicle_local_position",
            self.custom_watch_field_edit: "읽을 메시지 필드 경로입니다. 예: z, q[0], position[2]. 숫자 필드는 그래프로 표시됩니다.",
            self.custom_watch_add_button: "사용자 지정 감시를 추가합니다. ROS2 연결 중이면 즉시 구독합니다.",
            self.custom_watch_discover_button: "현재 보이는 ROS2 토픽을 탐색하고 그래프 가능한 필드를 골라 자동 감시에 추가합니다.",
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
                "양쪽 전송에서는 같은 STUS UTC 값을 만들고 명령 UTC/TCP 호출 시작 차이를 로그에 남깁니다."
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
            self.calib_dir_edit: "보정 이미지와 intrinsics.json / intrinsics_zoom_##.json을 저장/로드할 디렉터리입니다.",
            self.capture_calibration_button: "현재 좌/우 카메라 프레임 중 체커보드가 검출된 쪽만 내부 파라미터 보정 이미지로 저장합니다.",
            self.run_calibration_button: "캡처된 체커보드 이미지로 OpenCV 내부 파라미터 보정을 실행합니다.",
            self.save_intrinsics_button: "현재 WIRIS 줌 레벨의 내부 파라미터를 intrinsics_zoom_##.json으로 저장하고 latest alias intrinsics.json도 갱신합니다.",
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
                "세션 기록을 시작하거나 중지합니다. 켜면 좌/우 영상 MP4, 거리 샘플 CSV, "
                "시간축 외부 파라미터 CSV를 같은 session id로 묶어 저장합니다."
            ),
            self.clear_history_button: "거리 그래프와 아직 저장하지 않은 메모리 기록을 지웁니다.",
            self.runtime_distance_graph_group: "baseline_center_frd Forward/Right/Down/Range 이력 그래프 영역을 접거나 펼칩니다.",
            self.distance_plot: (
                "3D ray 샘플의 baseline_center_frd Forward/Right/Down/Range 이력 그래프입니다. "
                "점 색상은 동기화 품질을 따릅니다.\n"
                "왼쪽 클릭: 점의 Forward/Right/Down과 midpoint 기준 Range를 표시합니다.\n"
                "오른쪽 클릭: 해당 점을 삭제합니다."
            ),
            self.save_history_button: "현재 매칭 프레임쌍 전체를 wide CSV와 목적별 탭 XLSX로 함께 저장합니다.",
            self.save_settings_button: "현재 내부 파라미터 경로와 전체 런타임 설정을 저장합니다.",
            self.recording_status_label: "메모리 내 세션 기록 상태와 매칭 프레임쌍 수입니다.",
            self.recording_sample_table: "저장될 frame-pair row의 주요 컬럼을 매칭 쌍마다 한 줄씩 표시합니다.",
            self.recording_sample_detail: "선택한 매칭 프레임쌍의 전체 wide CSV 필드를 표시합니다.",
            self.clock_ftp_user_edit: "WIRIS FTP 로그인 사용자 이름입니다. 장비 설정에 맞게 입력하세요.",
            self.clock_ftp_password_edit: "WIRIS FTP 로그인 비밀번호입니다. 비어 있으면 빈 비밀번호로 로그인합니다.",
            self.clock_ftp_port_spin: "WIRIS FTP 포트입니다. 일반 FTP는 21번을 사용합니다.",
            self.clock_ftp_remote_dir_edit: "CPTR로 저장된 JPG/TIFF 파일이 있는 WIRIS FTP 원격 폴더입니다.",
            self.clock_download_dir_edit: "FTP로 내려받은 좌/우 촬영 이미지를 저장할 로컬 폴더입니다.",
            self.clock_download_dir_button: "FTP 다운로드 로컬 저장 폴더를 선택합니다.",
            self.clock_capture_wait_spin: "CPTR 후 ICPT가 촬영 완료 상태가 될 때까지 기다릴 최대 시간입니다.",
            self.clock_cptr_button: "좌/우 WIRIS에 CPTR 촬영 명령을 전송합니다.",
            self.clock_icpt_button: "좌/우 WIRIS에 ICPT 명령을 보내 캡처 진행 상태를 확인합니다.",
            self.clock_download_latest_button: "좌/우 WIRIS FTP 폴더에서 최신 JPG/TIFF 이미지를 다운로드하고 EXIF를 분석합니다.",
            self.clock_workflow_button: "CPTR 촬영, ICPT 완료 대기, FTP 최신 이미지 다운로드, EXIF 분석을 순서대로 실행합니다.",
            self.clock_left_file_edit: "FTP로 내려받은 왼쪽 WIRIS 이미지 파일입니다. JPG/JPEG/TIFF EXIF를 읽습니다.",
            self.clock_right_file_edit: "FTP로 내려받은 오른쪽 WIRIS 이미지 파일입니다. JPG/JPEG/TIFF EXIF를 읽습니다.",
            self.clock_left_folder_edit: "왼쪽 WIRIS에서 FTP로 받은 이미지 폴더입니다. 하위 폴더까지 검색합니다.",
            self.clock_right_folder_edit: "오른쪽 WIRIS에서 FTP로 받은 이미지 폴더입니다. 하위 폴더까지 검색합니다.",
            self.clock_analyze_pair_button: "선택한 좌/우 이미지 한 쌍의 EXIF timestamp 차이를 right-left 기준으로 계산합니다.",
            self.clock_analyze_folder_button: "좌/우 폴더의 공통 파일명 또는 정렬 순서로 이미지 쌍을 만들어 clock offset 분포를 계산합니다.",
            self.clock_result_table: "분석된 이미지 쌍별 촬영 시간, timestamp source, right-left clock offset, 경고입니다.",
            self.clock_summary_text: "clock offset 요약과 EXIF/GPS timestamp 품질 경고입니다.",
            self.wiris_log: "WIRIS, 카메라 Preview 연결, 보정, 시간 확인 메시지 로그입니다.",
            self.camera_yolo_log: "YOLO, 거리 측정, 저장 메시지 로그입니다.",
            self.pixhawk_log: "Pixhawk, PX4, ROS2, MAVLink, uXRCE 메시지 로그입니다.",
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
                background: #d0e2ff;
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
                background: {INFO_BG};
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
            self.pixhawk_button,
            self.left_ros2_button,
            self.right_ros2_button,
            self.left_mavlink_button,
            self.right_mavlink_button,
            self.start_button,
            self.capture_calibration_button,
            self.run_calibration_button,
            self.save_intrinsics_button,
            self.clock_left_file_button,
            self.clock_right_file_button,
            self.clock_left_folder_button,
            self.clock_right_folder_button,
            self.clock_analyze_pair_button,
            self.clock_analyze_folder_button,
            self.clock_download_dir_button,
            self.clock_cptr_button,
            self.clock_icpt_button,
            self.clock_download_latest_button,
            self.clock_workflow_button,
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
            getattr(self, "ros2_pixhawk_candidate_table", None),
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
        mag_count = sum(1 for side in ("left", "right") for slot in ("CAN1", "CAN2") if slot in self.latest_mag_sensors.get(side, {}))
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

        self.signal_gps_sats_plot = TelemetryHistoryPlot("GPS 위성 수 수신 확인 (ROS2/MAVLink)", "sats")
        self.signal_gps_sats_plot.set_series_labels(
            {
                "left_GPS1": "P1 GPS1",
                "left_GPS2": "P1 GPS2",
                "right_GPS1": "P2 GPS1",
                "right_GPS2": "P2 GPS2",
            }
        )
        self.signal_gps_eph_plot = TelemetryHistoryPlot("GPS eph 수신 확인 (ROS2/MAVLink)", "m")
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
        scroll_area = QtWidgets.QScrollArea()
        scroll_area.setObjectName("gnssBaseScrollArea")
        scroll_area.setWidgetResizable(True)
        scroll_area.setFrameShape(QtWidgets.QFrame.NoFrame)
        content = QtWidgets.QWidget()
        content_layout = QtWidgets.QVBoxLayout(content)
        content_layout.setContentsMargins(0, 0, 0, 0)
        content_layout.setSpacing(8)

        settings_group = QtWidgets.QGroupBox("GNSS Base 설정")
        settings_form = QtWidgets.QGridLayout(settings_group)
        self.gnss_base_serial_port_edit = QtWidgets.QLineEdit(self.settings.gnss_base_serial_port)
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
        settings_items = (
            ("Base serial port", self.gnss_base_serial_port_edit),
            ("Baudrate", self.gnss_base_baud_spin),
            ("Survey-in 최소 시간(s)", self.gnss_base_survey_duration_spin),
            ("Survey-in 목표 정확도(m)", self.gnss_base_survey_accuracy_spin),
            ("RTCM 주입 대상", self.gnss_base_rtcm_target_combo),
        )
        for row, (title, widget) in enumerate(settings_items):
            settings_form.addWidget(QtWidgets.QLabel(title), row, 0)
            settings_form.addWidget(widget, row, 1)
        self.gnss_base_connect_button = QtWidgets.QPushButton("Base 연결")
        self.gnss_base_connect_button.clicked.connect(self.connect_gnss_base_serial)
        self.gnss_base_disconnect_button = QtWidgets.QPushButton("연결 해제")
        self.gnss_base_disconnect_button.clicked.connect(self.disconnect_gnss_base_serial)
        self.gnss_base_disconnect_button.setEnabled(False)
        self.gnss_base_survey_start_button = QtWidgets.QPushButton("Survey-in 시작")
        self.gnss_base_survey_start_button.clicked.connect(self.start_gnss_base_survey_in)
        self.gnss_base_survey_start_button.setEnabled(False)
        self.gnss_base_survey_stop_button = QtWidgets.QPushButton("Survey-in 중지")
        self.gnss_base_survey_stop_button.clicked.connect(self.stop_gnss_base_survey_in)
        self.gnss_base_survey_stop_button.setEnabled(False)
        self.gnss_base_discover_button = QtWidgets.QPushButton("Base 포트 탐색")
        self.gnss_base_discover_button.clicked.connect(self.discover_gnss_base_serial_ports)
        self.gnss_base_apply_candidate_button = QtWidgets.QPushButton("선택 포트 적용")
        self.gnss_base_apply_candidate_button.clicked.connect(self.apply_selected_gnss_base_candidate)
        self.gnss_base_apply_candidate_button.setEnabled(False)
        button_row = QtWidgets.QHBoxLayout()
        button_row.addWidget(self.gnss_base_discover_button)
        button_row.addWidget(self.gnss_base_connect_button)
        button_row.addWidget(self.gnss_base_disconnect_button)
        button_row.addWidget(self.gnss_base_survey_start_button)
        button_row.addWidget(self.gnss_base_survey_stop_button)
        button_row.addWidget(self.gnss_base_apply_candidate_button)
        button_row.addStretch(1)
        settings_form.addLayout(button_row, len(settings_items), 0, 1, 2)

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
        settings_form.addWidget(self.gnss_base_candidate_table, len(settings_items) + 1, 0, 1, 2)

        base_group = QtWidgets.QGroupBox("GNSS Base 연결")
        base_group.setObjectName("gnssBaseConnectionGroup")
        base_group.setMinimumHeight(190)
        base_grid = QtWidgets.QGridLayout(base_group)
        base_items = (
            ("connection", "Base 연결", "대기 중", "idle"),
            ("survey_in", "Survey-in", "미연결", "idle"),
            ("base_satellites", "Base 위성 수", "대기 중", "idle"),
            ("rtcm_rx", "Raw 수신", "없음", "idle"),
            ("rtcm_injection", "RTCM 주입", "비활성", "idle"),
        )
        for row, (key, title, text, level) in enumerate(base_items):
            label = QtWidgets.QLabel(text)
            label.setObjectName("statusStrip")
            label.setWordWrap(True)
            base_grid.addWidget(QtWidgets.QLabel(title), row, 0)
            base_grid.addWidget(label, row, 1)
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
        self.gnss_base_survey_progress_bar.setFormat("duration 0%")
        self.gnss_base_survey_progress_bar.setMinimumHeight(24)
        self.gnss_base_survey_progress_bar.setSizePolicy(QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Fixed)
        self.gnss_base_survey_accuracy_label = QtWidgets.QLabel("accuracy 대기 중")
        self.gnss_base_survey_accuracy_label.setObjectName("statusStrip")
        self.gnss_base_survey_accuracy_label.setWordWrap(True)
        survey_summary_row.addWidget(self.gnss_base_survey_progress_bar, 2)
        survey_summary_row.addWidget(self.gnss_base_survey_accuracy_label, 1)
        survey_layout.addLayout(survey_summary_row)

        survey_plot_row = QtWidgets.QHBoxLayout()
        self.gnss_base_survey_accuracy_plot = TelemetryHistoryPlot("Survey-in accuracy", "m")
        self.gnss_base_survey_accuracy_plot.setMinimumHeight(180)
        self.gnss_base_survey_observation_plot = TelemetryHistoryPlot("Survey-in observations", "obs")
        self.gnss_base_survey_observation_plot.setMinimumHeight(180)
        survey_plot_row.addWidget(self.gnss_base_survey_accuracy_plot, 1)
        survey_plot_row.addWidget(self.gnss_base_survey_observation_plot, 1)
        survey_layout.addLayout(survey_plot_row)

        note = QtWidgets.QLabel(
            "Base serial raw 수신, UBX NAV-SVIN 상태 polling, Survey-in 시작/중지 명령을 지원합니다."
        )
        note.setWordWrap(True)
        note.setObjectName("statusStrip")

        content_layout.addWidget(settings_group)
        content_layout.addWidget(base_group)
        content_layout.addWidget(survey_group)
        content_layout.addWidget(rover_group)
        content_layout.addWidget(note)
        content_layout.addStretch(1)
        scroll_area.setWidget(content)
        layout.addWidget(scroll_area, 1)
        self._refresh_gnss_base_tab()

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
        content = QtWidgets.QWidget()
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
        headers = (
            "Sensor",
            "Device ID",
            "Priority",
            "Selected",
            "Health",
            "Calibration",
            "Field Strength",
            "Rate",
            "Timestamp",
        )
        self.alignment_mag_labels: dict[str, dict[str, QtWidgets.QLabel]] = {}
        self.alignment_mag_role_tabs = QtWidgets.QTabWidget()
        self.alignment_mag_role_tabs.setDocumentMode(True)
        for side, side_label in (("left", "Left"), ("right", "Right")):
            side_group = QtWidgets.QGroupBox(f"{side_label} Pixhawk")
            grid = QtWidgets.QGridLayout(side_group)
            grid.setHorizontalSpacing(8)
            grid.setVerticalSpacing(6)
            for column, header in enumerate(headers):
                header_label = QtWidgets.QLabel(header)
                header_label.setWordWrap(True)
                grid.addWidget(header_label, 0, column)
                grid.setColumnMinimumWidth(column, 82 if column >= 1 else 72)
            row = 1
            for slot in ("BOARD", "CAN1", "CAN2", "GPS1_MAG", "GPS2_MAG"):
                key = f"{side}_{slot}"
                self.alignment_mag_labels[key] = {}
                sensor_label = QtWidgets.QLabel(slot)
                sensor_label.setTextInteractionFlags(QtCore.Qt.TextSelectableByMouse)
                grid.addWidget(sensor_label, row, 0)
                for column, field in enumerate(("device", "priority", "selected", "health", "calibration", "field", "rate", "timestamp"), start=2):
                    label = QtWidgets.QLabel("대기 중")
                    label.setObjectName("statusStrip")
                    label.setWordWrap(True)
                    label.setTextInteractionFlags(QtCore.Qt.TextSelectableByMouse)
                    label.setMinimumWidth(82)
                    grid.addWidget(label, row, column - 1)
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
                ("left_can_diff", "Left CAN1 vs CAN2 Difference"),
                ("right_can_diff", "Right CAN1 vs CAN2 Difference"),
                ("left_right_diff", "Left vs Right Difference"),
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
        self.alignment_camera_frame_combo.addItems(["RDF", "ROS Optical Frame"])
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

        detail_group = QtWidgets.QGroupBox("GPS1/GPS2 ROS2 Topic / MAVLink Field 상태")
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
        self.baseline_scan_topics_button = QtWidgets.QPushButton("ROS2 Topic 스캔")
        self.baseline_scan_topics_button.clicked.connect(self.refresh_baseline_field_browser)
        actions.addWidget(self.baseline_scan_topics_button)
        actions.addStretch(1)
        layout.addLayout(actions)

        split = QtWidgets.QHBoxLayout()
        self.baseline_ros2_topic_table = QtWidgets.QTableWidget(0, 3)
        self.baseline_ros2_topic_table.setHorizontalHeaderLabels(["우선", "Topic", "Type"])
        self.baseline_ros2_topic_table.verticalHeader().setVisible(False)
        self.baseline_ros2_topic_table.setEditTriggers(QtWidgets.QAbstractItemView.NoEditTriggers)
        self.baseline_ros2_topic_table.setSelectionBehavior(QtWidgets.QAbstractItemView.SelectRows)
        self.baseline_ros2_topic_table.horizontalHeader().setSectionResizeMode(1, QtWidgets.QHeaderView.Stretch)
        self.baseline_ros2_topic_table.itemSelectionChanged.connect(self._show_selected_baseline_ros2_fields)
        self.baseline_ros2_field_list = QtWidgets.QListWidget()
        ros2_group = QtWidgets.QGroupBox("ROS2 Topic")
        ros2_layout = QtWidgets.QHBoxLayout(ros2_group)
        ros2_layout.addWidget(self.baseline_ros2_topic_table, 2)
        ros2_layout.addWidget(self.baseline_ros2_field_list, 1)
        split.addWidget(ros2_group, 2)

        self.baseline_mavlink_message_table = QtWidgets.QTableWidget(0, 2)
        self.baseline_mavlink_message_table.setHorizontalHeaderLabels(["Message", "Fields"])
        self.baseline_mavlink_message_table.verticalHeader().setVisible(False)
        self.baseline_mavlink_message_table.setEditTriggers(QtWidgets.QAbstractItemView.NoEditTriggers)
        self.baseline_mavlink_message_table.horizontalHeader().setSectionResizeMode(1, QtWidgets.QHeaderView.Stretch)
        mav_group = QtWidgets.QGroupBox("MAVLink Raw")
        mav_layout = QtWidgets.QVBoxLayout(mav_group)
        mav_layout.addWidget(self.baseline_mavlink_message_table)
        split.addWidget(mav_group, 1)
        layout.addLayout(split, 1)

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
        if not hasattr(self, "baseline_ros2_topic_table"):
            return
        priority_topics = (
            "/mavros/global_position/global",
            "/mavros/gpsstatus/gps1/raw",
            "/mavros/gpsstatus/gps2/raw",
            "/mavros/local_position/pose",
            "/mavros/imu/data",
            "/fmu/out/vehicle_gps_position",
            "/fmu/out/sensor_gps",
            "/fmu/out/vehicle_global_position",
            "/fmu/out/vehicle_local_position",
        )
        discovered = discover_ros2_topic_types(self.domain_spin.value(), timeout_s=1.0)
        by_name = {item.get("name", ""): item.get("type", "") for item in discovered}
        rows: list[tuple[str, str, str]] = []
        for topic in priority_topics:
            rows.append(("yes", topic, by_name.pop(topic, "")))
        for topic, topic_type in sorted(by_name.items()):
            if any(pattern in topic for pattern in ("/fmu/out/", "/mavros/")):
                rows.append(("", topic, topic_type))
        self.baseline_ros2_topic_table.setRowCount(len(rows))
        for row, (priority, topic, topic_type) in enumerate(rows):
            for column, value in enumerate((priority, topic, topic_type)):
                self.baseline_ros2_topic_table.setItem(row, column, QtWidgets.QTableWidgetItem(value))

        mavlink_rows = {
            "GPS_RAW_INT": (
                "time_usec, fix_type, lat, lon, alt, eph, epv, vel, cog, satellites_visible, "
                "h_acc, v_acc, vel_acc, hdg_acc, yaw"
            ),
            "GPS2_RAW": (
                "time_usec, fix_type, lat, lon, alt, eph, epv, vel, cog, satellites_visible, "
                "h_acc, v_acc, vel_acc, hdg_acc, yaw"
            ),
            "GLOBAL_POSITION_INT": "time_boot_ms, lat, lon, alt, relative_alt, vx, vy, vz, hdg",
        }
        self.baseline_mavlink_message_table.setRowCount(len(mavlink_rows))
        for row, (message, fields) in enumerate(mavlink_rows.items()):
            self.baseline_mavlink_message_table.setItem(row, 0, QtWidgets.QTableWidgetItem(message))
            self.baseline_mavlink_message_table.setItem(row, 1, QtWidgets.QTableWidgetItem(fields))

    def _show_selected_baseline_ros2_fields(self) -> None:
        if not hasattr(self, "baseline_ros2_topic_table"):
            return
        selected = self.baseline_ros2_topic_table.selectedItems()
        if not selected:
            return
        row = selected[0].row()
        type_item = self.baseline_ros2_topic_table.item(row, 2)
        topic_type = "" if type_item is None else type_item.text().strip()
        self.baseline_ros2_field_list.clear()
        if not topic_type:
            self.baseline_ros2_field_list.addItem("message type 대기 중")
            return
        try:
            fields = describe_ros2_message_fields(topic_type)
        except Exception as exc:
            self.baseline_ros2_field_list.addItem(f"field 조회 실패: {exc}")
            return
        for field in fields:
            self.baseline_ros2_field_list.addItem(field)

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

    def _refresh_baseline_alignment_ui(self) -> None:
        self._refresh_baseline_gps_cards()
        current = self._current_baseline_vector()
        if current is None:
            self.latest_baseline_vector = None
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
                unavailable = state is None and self._ros2_only_unavailable(side, slot_key)
                level = "good" if state is not None else "warn" if unavailable else "idle"
                text = "인식" if state is not None else "ROS2 미게시" if unavailable else "대기"
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
                unavailable = state is None and self._ros2_only_unavailable(side, slot)
                level = "good" if state is not None else "warn" if unavailable else "idle"
                text = "인식" if state is not None else "ROS2 미게시" if unavailable else "대기"
                self._set_health_label(labels[f"{side}_{slot.lower()}"], text, level)
            for slot in ("CAN1", "CAN2"):
                state = self.latest_mag_sensors.get(side, {}).get(slot)
                unavailable = state is None and self._ros2_only_unavailable(side, slot)
                level = "good" if state is not None else "warn" if unavailable else "idle"
                text = "인식" if state is not None else "ROS2 미게시" if unavailable else "대기"
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

    def _refresh_gnss_base_tab(self) -> None:
        if not hasattr(self, "gnss_base_status_labels"):
            return
        labels = self.gnss_base_status_labels
        now_s = time.monotonic()
        for side, key in (("left", "left_rover"), ("right", "right_rover")):
            gps = self.latest_gps.get(side)
            if gps is None:
                self._set_health_label(labels[key], "GPS/RTK 대기 중", "idle")
                continue
            age_ms = max(0.0, now_s - gps.timestamp_s) * 1000.0
            sats = "sats=--" if gps.satellites_used is None else f"sats={gps.satellites_used}"
            eph = "" if gps.eph_m is None else f" | eph={gps.eph_m:.2f}m"
            text = f"{gps.rtk_label} | {sats}{eph} | age={age_ms:.0f}ms"
            self._set_health_label(labels[key], text, self._gps_fix_level(gps.fix_type, age_ms=age_ms))

        baseline = self.latest_rtk_baseline
        if baseline is None:
            self._set_health_label(labels["baseline"], "대기 중", "idle")
            return
        usable, reason = self._gps_baseline_usable_for_translation(baseline)
        level = "good" if usable and baseline.fix_quality == "GOOD" else "warn" if usable else "bad"
        text = (
            f"{baseline.baseline_m:.3f} m | {baseline.fix_quality} | "
            f"N/E/D={baseline.north_m:+.3f}/{baseline.east_m:+.3f}/{baseline.down_m:+.3f} m"
        )
        if not usable:
            text = f"{text} | {reason}"
        self._set_health_label(labels["baseline"], text, level)

    def connect_gnss_base_serial(self) -> None:
        if self.gnss_base_serial_thread is not None:
            return
        port = self.gnss_base_serial_port_edit.text().strip()
        if not port:
            self._set_health_label(self.gnss_base_status_labels["connection"], "serial port 필요", "bad")
            return
        baud = int(self.gnss_base_baud_spin.value())
        self._set_health_label(self.gnss_base_status_labels["connection"], f"connecting {port} @ {baud}", "warn")
        self._set_health_label(self.gnss_base_status_labels["rtcm_rx"], "raw bytes 대기 중", "idle")
        self.gnss_base_connect_button.setEnabled(False)
        self.gnss_base_disconnect_button.setEnabled(True)
        thread = QtCore.QThread(self)
        worker = GnssBaseSerialWorker(port=port, baud=baud)
        worker.moveToThread(thread)
        thread.started.connect(worker.run)
        worker.connected.connect(self._handle_gnss_base_serial_connected)
        worker.stats_changed.connect(self._handle_gnss_base_serial_stats)
        worker.survey_in_changed.connect(self._handle_gnss_base_survey_in)
        worker.satellite_count_changed.connect(self._handle_gnss_base_satellite_count)
        worker.rtcm_frame_received.connect(self._handle_gnss_base_rtcm_frame)
        worker.command_sent.connect(self._handle_gnss_base_command_sent)
        worker.failed.connect(self._handle_gnss_base_serial_failed)
        worker.disconnected.connect(self._handle_gnss_base_serial_disconnected)
        worker.finished.connect(thread.quit)
        worker.finished.connect(worker.deleteLater)
        thread.finished.connect(lambda: self._clear_gnss_base_serial_worker(thread))
        self.gnss_base_serial_thread = thread
        self.gnss_base_serial_worker = worker
        thread.start()

    def disconnect_gnss_base_serial(self) -> None:
        worker = self.gnss_base_serial_worker
        if worker is not None:
            worker.request_stop()
        thread = self.gnss_base_serial_thread
        if thread is not None and not thread.wait(1500):
            thread.quit()
            thread.wait(500)

    def start_gnss_base_survey_in(self) -> None:
        worker = self.gnss_base_serial_worker
        if worker is None:
            self._set_health_label(self.gnss_base_status_labels["survey_in"], "serial 미연결", "bad")
            return
        frame = ubx_cfg_tmode3_survey_in_frame(
            min_duration_s=self.gnss_base_survey_duration_spin.value(),
            accuracy_m=self.gnss_base_survey_accuracy_spin.value(),
        )
        worker.send_bytes("CFG-TMODE3 survey-in start", frame)
        worker.send_bytes("NAV-SVIN poll after survey-in start", ubx_nav_svin_poll_frame())
        self._set_health_label(self.gnss_base_status_labels["survey_in"], "survey-in 시작 명령 전송 대기", "warn")

    def stop_gnss_base_survey_in(self) -> None:
        worker = self.gnss_base_serial_worker
        if worker is None:
            self._set_health_label(self.gnss_base_status_labels["survey_in"], "serial 미연결", "bad")
            return
        worker.send_bytes("CFG-TMODE3 disabled", ubx_cfg_tmode3_disabled_frame())
        worker.send_bytes("NAV-SVIN poll after survey-in stop", ubx_nav_svin_poll_frame())
        self._set_health_label(self.gnss_base_status_labels["survey_in"], "survey-in 중지 명령 전송 대기", "warn")

    def discover_gnss_base_serial_ports(self) -> None:
        if self.gnss_base_detect_thread is not None:
            return
        self.gnss_base_discover_button.setEnabled(False)
        self.gnss_base_apply_candidate_button.setEnabled(False)
        self._set_health_label(self.gnss_base_status_labels["connection"], "base port 탐색 중", "warn")
        thread = QtCore.QThread(self)
        worker = GnssBaseSerialDetectWorker(baud=self.gnss_base_baud_spin.value())
        worker.moveToThread(thread)
        thread.started.connect(worker.run)
        worker.detected.connect(self._handle_gnss_base_candidates_detected)
        worker.failed.connect(self._handle_gnss_base_candidate_detect_failed)
        worker.finished.connect(thread.quit)
        worker.finished.connect(worker.deleteLater)
        thread.finished.connect(lambda: self._clear_gnss_base_detect_worker(thread))
        self.gnss_base_detect_thread = thread
        self.gnss_base_detect_worker = worker
        thread.start()

    def _handle_gnss_base_candidates_detected(self, rows: object) -> None:
        candidate_rows = [dict(row) for row in rows] if isinstance(rows, list) else []
        self._set_gnss_base_candidate_rows(candidate_rows)
        count = len(candidate_rows)
        ubx_count = sum(1 for row in candidate_rows if row.get("ubx"))
        level = "good" if ubx_count else "warn" if count else "bad"
        self._set_health_label(
            self.gnss_base_status_labels["connection"],
            f"탐색 완료: {count} candidates, UBX {ubx_count}",
            level,
        )

    def _handle_gnss_base_candidate_detect_failed(self, message: str) -> None:
        self._set_health_label(self.gnss_base_status_labels["connection"], f"탐색 실패: {message}", "bad")

    def _clear_gnss_base_detect_worker(self, thread: QtCore.QThread) -> None:
        if self.gnss_base_detect_thread is thread:
            self.gnss_base_detect_thread = None
            self.gnss_base_detect_worker = None
            if hasattr(self, "gnss_base_discover_button"):
                self.gnss_base_discover_button.setEnabled(True)
            self._refresh_gnss_base_candidate_actions()
        thread.deleteLater()

    def _set_gnss_base_candidate_rows(self, rows: list[dict[str, object]]) -> None:
        self._gnss_base_candidate_rows = rows
        table = self.gnss_base_candidate_table
        table.setRowCount(len(rows))
        keys = ("port", "target", "baud", "status", "messages")
        for row_index, row in enumerate(rows):
            for column, key in enumerate(keys):
                item = QtWidgets.QTableWidgetItem(str(row.get(key, "")))
                table.setItem(row_index, column, item)
        table.resizeRowsToContents()
        self._refresh_gnss_base_candidate_actions()

    def _refresh_gnss_base_candidate_actions(self) -> None:
        row = self.gnss_base_candidate_table.currentRow() if hasattr(self, "gnss_base_candidate_table") else -1
        self.gnss_base_apply_candidate_button.setEnabled(0 <= row < len(self._gnss_base_candidate_rows))

    def apply_selected_gnss_base_candidate(self) -> None:
        row = self.gnss_base_candidate_table.currentRow()
        if row < 0 or row >= len(self._gnss_base_candidate_rows):
            self._set_health_label(self.gnss_base_status_labels["connection"], "포트 후보를 먼저 선택하세요", "warn")
            return
        entry = self._gnss_base_candidate_rows[row]
        port = str(entry.get("port", "")).strip()
        if not port:
            self._set_health_label(self.gnss_base_status_labels["connection"], "선택 후보 port가 비어 있습니다", "bad")
            return
        self.gnss_base_serial_port_edit.setText(port)
        try:
            baud = int(entry.get("baud", self.gnss_base_baud_spin.value()))
        except (TypeError, ValueError):
            baud = self.gnss_base_baud_spin.value()
        self.gnss_base_baud_spin.setValue(baud)
        self._set_health_label(self.gnss_base_status_labels["connection"], f"선택 포트 적용: {port}", "good")

    def _handle_gnss_base_serial_connected(self, port: str, baud: int) -> None:
        self.gnss_base_serial_connected = True
        self._set_health_label(self.gnss_base_status_labels["connection"], f"connected {port} @ {baud}", "good")
        self._set_health_label(self.gnss_base_status_labels["survey_in"], "UBX NAV-SVIN 대기 중", "idle")
        self._set_health_label(self.gnss_base_status_labels["base_satellites"], "UBX NAV-PVT/NAV-SAT 대기 중", "idle")
        self.gnss_base_survey_start_button.setEnabled(True)
        self.gnss_base_survey_stop_button.setEnabled(True)

    def _handle_gnss_base_serial_stats(self, total_bytes: int, rate_bps: float, age_s: float) -> None:
        age_text = "no data" if age_s < 0.0 else f"age={age_s:.1f}s"
        level = "good" if rate_bps > 0.0 and 0.0 <= age_s < 2.0 else "warn" if rate_bps > 0.0 else "idle"
        self._set_health_label(
            self.gnss_base_status_labels["rtcm_rx"],
            f"raw={total_bytes} bytes | {rate_bps:.1f} B/s | {age_text}",
            level,
        )

    def _handle_gnss_base_serial_failed(self, message: str) -> None:
        self.gnss_base_serial_connected = False
        self._set_health_label(self.gnss_base_status_labels["connection"], f"failed: {message}", "bad")
        self._set_health_label(self.gnss_base_status_labels["survey_in"], "미연결", "idle")
        self._set_health_label(self.gnss_base_status_labels["base_satellites"], "대기 중", "idle")
        self._set_health_label(self.gnss_base_status_labels["rtcm_rx"], "없음", "idle")
        self.gnss_base_survey_start_button.setEnabled(False)
        self.gnss_base_survey_stop_button.setEnabled(False)

    def _handle_gnss_base_serial_disconnected(self, message: str) -> None:
        self.gnss_base_serial_connected = False
        self._set_health_label(self.gnss_base_status_labels["connection"], message, "idle")
        self._set_health_label(self.gnss_base_status_labels["survey_in"], "미연결", "idle")
        self._set_health_label(self.gnss_base_status_labels["base_satellites"], "대기 중", "idle")
        self.gnss_base_survey_start_button.setEnabled(False)
        self.gnss_base_survey_stop_button.setEnabled(False)

    def _handle_gnss_base_satellite_count(self, count: int, source: str) -> None:
        level = "good" if count >= 12 else "warn" if count >= 6 else "bad"
        self._set_health_label(
            self.gnss_base_status_labels["base_satellites"],
            f"{count} satellites | {source}",
            level,
        )

    def _gnss_base_rtcm_target_sides(self) -> tuple[str, ...]:
        target = str(self.gnss_base_rtcm_target_combo.currentData() or "disabled")
        if target == "both":
            return ("left", "right")
        if target in {"left", "right"}:
            return (target,)
        return ()

    def _handle_gnss_base_rtcm_frame(self, frame: object) -> None:
        if not isinstance(frame, (bytes, bytearray)):
            return
        payload = bytes(frame)
        target_sides = self._gnss_base_rtcm_target_sides()
        if not target_sides:
            self._set_health_label(self.gnss_base_status_labels["rtcm_injection"], "비활성", "idle")
            return
        if self.mavlink_pixhawk is None:
            self._set_health_label(
                self.gnss_base_status_labels["rtcm_injection"],
                f"RTCM {len(payload)} bytes 수신 | MAVLink rover 미연결",
                "warn",
            )
            return
        sent_sides: list[str] = []
        failed: list[str] = []
        for side in target_sides:
            try:
                self.mavlink_pixhawk.send_rtcm_data(side, payload)
                sent_sides.append(self._px4_mag_camera_label(side))
            except Exception as exc:
                failed.append(f"{self._px4_mag_camera_label(side)}:{exc}")
        if sent_sides and not failed:
            self._set_health_label(
                self.gnss_base_status_labels["rtcm_injection"],
                f"RTCM {len(payload)} bytes -> {', '.join(sent_sides)}",
                "good",
            )
        elif sent_sides:
            self._set_health_label(
                self.gnss_base_status_labels["rtcm_injection"],
                f"RTCM 일부 주입: {', '.join(sent_sides)} | 실패 {', '.join(failed)}",
                "warn",
            )
        else:
            self._set_health_label(
                self.gnss_base_status_labels["rtcm_injection"],
                f"RTCM 주입 실패: {', '.join(failed)}",
                "bad",
            )

    def _handle_gnss_base_command_sent(self, label: str, byte_count: int) -> None:
        self._set_health_label(
            self.gnss_base_status_labels["connection"],
            f"connected | sent {label} ({byte_count} bytes)",
            "good",
        )
        if label == "CFG-TMODE3 survey-in start":
            self._set_health_label(
                self.gnss_base_status_labels["survey_in"],
                "survey-in 시작 명령 전송 완료 | NAV-SVIN 상태 수신 대기",
                "warn",
            )
        elif label == "CFG-TMODE3 disabled":
            self._set_health_label(
                self.gnss_base_status_labels["survey_in"],
                "survey-in 중지 명령 전송 완료 | NAV-SVIN 상태 수신 대기",
                "warn",
            )

    def _handle_gnss_base_survey_in(self, state: object) -> None:
        if not isinstance(state, dict):
            return
        active = bool(state.get("active"))
        valid = bool(state.get("valid"))
        duration_s = int(state.get("duration_s") or 0)
        accuracy_m = float(state.get("mean_accuracy_m") or 0.0)
        observations = int(state.get("observations") or 0)
        mean_x_m = float(state.get("mean_x_m") or 0.0)
        mean_y_m = float(state.get("mean_y_m") or 0.0)
        mean_z_m = float(state.get("mean_z_m") or 0.0)
        status = "valid" if valid else "active" if active else "not running"
        level = "good" if valid else "warn" if active else "idle"
        text = (
            f"{status} | duration={duration_s}s | acc={accuracy_m:.4f}m | obs={observations} | "
            f"ECEF={mean_x_m:.3f}/{mean_y_m:.3f}/{mean_z_m:.3f}m"
        )
        self._set_health_label(self.gnss_base_status_labels["survey_in"], text, level)
        self._update_gnss_base_survey_visualization(
            active=active,
            valid=valid,
            duration_s=duration_s,
            accuracy_m=accuracy_m,
            observations=observations,
        )

    def _update_gnss_base_survey_visualization(
        self,
        *,
        active: bool,
        valid: bool,
        duration_s: int,
        accuracy_m: float,
        observations: int,
    ) -> None:
        if not hasattr(self, "gnss_base_survey_progress_bar"):
            return
        target_duration_s = max(1, int(self.gnss_base_survey_duration_spin.value()))
        target_accuracy_m = max(0.001, float(self.gnss_base_survey_accuracy_spin.value()))
        progress = min(100, int(round(duration_s * 100.0 / target_duration_s)))
        self.gnss_base_survey_progress_bar.setValue(progress)
        state_text = "valid" if valid else "active" if active else "not running"
        self.gnss_base_survey_progress_bar.setFormat(f"{state_text} {progress}%")
        accuracy_delta_m = accuracy_m - target_accuracy_m
        level = "good" if valid or accuracy_delta_m <= 0.0 else "warn" if active else "idle"
        self._set_health_label(
            self.gnss_base_survey_accuracy_label,
            f"target={target_accuracy_m:.4f}m | current={accuracy_m:.4f}m | Δ={accuracy_delta_m:+.4f}m | obs={observations}",
            level,
        )
        now_s = time.monotonic()
        self.gnss_base_survey_accuracy_plot.add_point(TelemetryHistoryPoint(now_s, "accuracy", accuracy_m))
        self.gnss_base_survey_accuracy_plot.add_point(TelemetryHistoryPoint(now_s, "target", target_accuracy_m))
        self.gnss_base_survey_observation_plot.add_point(TelemetryHistoryPoint(now_s, "observations", float(observations)))

    def _clear_gnss_base_serial_worker(self, thread: QtCore.QThread) -> None:
        if self.gnss_base_serial_thread is thread:
            self.gnss_base_serial_thread = None
            self.gnss_base_serial_worker = None
            self.gnss_base_serial_connected = False
            if hasattr(self, "gnss_base_connect_button"):
                self.gnss_base_connect_button.setEnabled(True)
            if hasattr(self, "gnss_base_disconnect_button"):
                self.gnss_base_disconnect_button.setEnabled(False)
            if hasattr(self, "gnss_base_survey_start_button"):
                self.gnss_base_survey_start_button.setEnabled(False)
            if hasattr(self, "gnss_base_survey_stop_button"):
                self.gnss_base_survey_stop_button.setEnabled(False)
        thread.deleteLater()

    def _refresh_alignment_magnetometer_ui(self) -> None:
        if not hasattr(self, "alignment_mag_labels"):
            return
        for side in ("left", "right"):
            for slot in ("BOARD", "CAN1", "CAN2", "GPS1_MAG", "GPS2_MAG"):
                key = f"{side}_{slot}"
                labels = self.alignment_mag_labels[key]
                state = self.latest_mag_sensors.get(side, {}).get(slot)
                if state is None:
                    unavailable = self._ros2_only_unavailable(side, slot)
                    text = "ROS2 미게시" if unavailable else "대기 중"
                    for label in labels.values():
                        label.setText(text)
                    continue
                device_id, device_source = self._mag_state_display_device_id(side, state)
                if slot == "CAN1":
                    priority_text = "Yaw target 100"
                    selected_text = device_source
                elif slot == "CAN2":
                    priority_text = "Backup 90"
                    selected_text = device_source
                elif slot in {"GPS1_MAG", "GPS2_MAG"}:
                    priority_text = "Disabled 0"
                    selected_text = "raw only"
                elif slot == "BOARD":
                    priority_text = "PX4 selected"
                    selected_text = device_source
                else:
                    priority_text = "PX4 param"
                    selected_text = device_source
                labels["device"].setText("--" if device_id is None else str(device_id))
                labels["priority"].setText(priority_text)
                labels["selected"].setText(selected_text)
                labels["health"].setText("OK" if state.magnitude is not None else "waiting")
                labels["calibration"].setText("PX4 EKF")
                labels["field"].setText("--" if state.magnitude is None else f"{state.magnitude:.3f} G")
                labels["rate"].setText("live")
                labels["timestamp"].setText(f"{state.timestamp_s:.3f}")
        left_diff = self.magnetometer_monitor.can_difference("left")
        right_diff = self.magnetometer_monitor.can_difference("right")
        self.alignment_mag_compare_labels["left_can_diff"].setText("--" if left_diff is None else f"{left_diff:.3f} G")
        self.alignment_mag_compare_labels["right_can_diff"].setText("--" if right_diff is None else f"{right_diff:.3f} G")
        self.alignment_mag_compare_labels["left_right_diff"].setText("heading source 대기")

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
        self.latest_baseline_quality = None
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
        vector, _left, _right = current
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
            str(APP_ROOT / "alignment_profile.yaml"),
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
            str(APP_ROOT),
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
        group = QtWidgets.QGroupBox("PX4 Magnetometer / RM3100 관리")
        group_layout = QtWidgets.QVBoxLayout(group)

        self.px4_mag_tables: dict[str, QtWidgets.QTableWidget] = {}
        self.px4_mag_status_labels: dict[str, QtWidgets.QLabel] = {}
        self.px4_mag_refresh_buttons: dict[str, QtWidgets.QPushButton] = {}
        self.px4_mag_apply_manual_buttons: dict[str, QtWidgets.QPushButton] = {}
        self.px4_mag_save_buttons: dict[str, QtWidgets.QPushButton] = {}
        self.px4_mag_reboot_buttons: dict[str, QtWidgets.QPushButton] = {}
        self.px4_mag_priority_spins: dict[str, dict[str, QtWidgets.QSpinBox]] = {}

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
        manual_layout.setColumnStretch(9, 1)
        default_priorities = {
            "BOARD": 0,
            "CAN1": 100,
            "CAN2": 90,
            "GPS1_MAG": 0,
            "GPS2_MAG": 0,
        }
        self.px4_mag_priority_spins[camera] = {}
        for column, role in enumerate(("BOARD", "CAN1", "CAN2", "GPS1_MAG", "GPS2_MAG")):
            spin = QtWidgets.QSpinBox()
            spin.setRange(0, 100)
            spin.setValue(default_priorities[role])
            spin.setSuffix(" prio")
            spin.setToolTip(f"{role}가 등록된 CAL_MAG 슬롯의 PRIO 값을 설정합니다.")
            manual_layout.addWidget(QtWidgets.QLabel(role), 0, column * 2)
            manual_layout.addWidget(spin, 0, column * 2 + 1)
            self.px4_mag_priority_spins[camera][role] = spin
        manual_note = QtWidgets.QLabel("MAVLink heartbeat + PARAM readback 상태에서만 실제 PX4 CAL_MAG*_PRIO에 적용됩니다.")
        manual_note.setObjectName("statusStrip")
        manual_note.setWordWrap(True)
        manual_layout.addWidget(manual_note, 1, 0, 1, 10)
        layout.addWidget(manual_group)

        detail_group = QtWidgets.QGroupBox("상세: CAL_MAG 슬롯 / priority")
        detail_layout = QtWidgets.QVBoxLayout(detail_group)
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
        table.verticalHeader().setVisible(False)
        table.horizontalHeader().setSectionResizeMode(QtWidgets.QHeaderView.ResizeToContents)
        table.horizontalHeader().setSectionResizeMode(9, QtWidgets.QHeaderView.Stretch)
        detail_layout.addWidget(table)
        self._configure_collapsible_group(detail_group, checked=False)
        layout.addWidget(detail_group)

        self.px4_mag_refresh_buttons[camera] = refresh_button
        self.px4_mag_apply_manual_buttons[camera] = apply_manual_button
        self.px4_mag_save_buttons[camera] = save_button
        self.px4_mag_reboot_buttons[camera] = reboot_button
        self.px4_mag_status_labels[camera] = status_label
        self.px4_mag_tables[camera] = table

        refresh_button.clicked.connect(lambda _checked=False, side=camera: self.refresh_px4_mag_manager(side))
        apply_manual_button.clicked.connect(lambda _checked=False, side=camera: self.apply_manual_mag_priorities_from_panel(side))
        save_button.clicked.connect(lambda _checked=False, side=camera: self.save_px4_mag_params_from_panel(side))
        reboot_button.clicked.connect(lambda _checked=False, side=camera: self.reboot_px4_from_panel(side))

    def _build_single_pixhawk_sensor_tab(self, layout: QtWidgets.QVBoxLayout, camera: str, title: str) -> None:
        layout.setSpacing(8)
        top_row = QtWidgets.QHBoxLayout()
        top_row.setSpacing(8)

        rm_group = QtWidgets.QGroupBox(f"{title} · RM3100 CAN")
        rm_form = QtWidgets.QGridLayout(rm_group)
        rm_form.setContentsMargins(8, 10, 8, 8)
        rm_form.setHorizontalSpacing(8)
        rm_form.setVerticalSpacing(6)
        rm_source_label = QtWidgets.QLabel("소스: ROS2 /fmu/out/sensor_mag, MAVLink SCALED_IMU* fallback")
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
        gnss_source_label = QtWidgets.QLabel("소스: ROS2 /fmu/out/sensor_gps, /fmu/out/sensor_gnss_relative")
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
        layout.setSpacing(8)
        workflow_group = QtWidgets.QGroupBox("CPTR 촬영 / FTP 다운로드")
        workflow_form = QtWidgets.QGridLayout(workflow_group)
        workflow_form.setColumnStretch(1, 1)
        workflow_form.setColumnStretch(4, 1)
        self.clock_ftp_user_edit = QtWidgets.QLineEdit(self.settings.clock_ftp_user or "anonymous")
        self.clock_ftp_password_edit = QtWidgets.QLineEdit(self.settings.clock_ftp_password)
        self.clock_ftp_password_edit.setEchoMode(QtWidgets.QLineEdit.Password)
        self.clock_ftp_port_spin = QtWidgets.QSpinBox()
        self.clock_ftp_port_spin.setRange(1, 65535)
        self.clock_ftp_port_spin.setValue(max(1, min(65535, int(self.settings.clock_ftp_port))))
        self.clock_ftp_remote_dir_edit = QtWidgets.QLineEdit(self.settings.clock_ftp_remote_dir or "/")
        clock_download_dir = resolve_settings_path(
            self.settings.clock_download_dir or DEFAULT_CLOCK_DOWNLOAD_DIR,
            base_dir=APP_ROOT,
        )
        self.clock_download_dir_edit = QtWidgets.QLineEdit(str(clock_download_dir))
        self.clock_download_dir_button = QtWidgets.QPushButton("폴더 선택")
        self.clock_capture_wait_spin = QtWidgets.QDoubleSpinBox()
        self.clock_capture_wait_spin.setRange(0.0, 120.0)
        self.clock_capture_wait_spin.setDecimals(1)
        self.clock_capture_wait_spin.setSingleStep(0.5)
        self.clock_capture_wait_spin.setValue(float(self.settings.clock_capture_wait_s))
        self.clock_workflow_button = QtWidgets.QPushButton("전체 실행")
        self.clock_cptr_button = QtWidgets.QPushButton("CPTR 촬영")
        self.clock_icpt_button = QtWidgets.QPushButton("ICPT 확인")
        self.clock_download_latest_button = QtWidgets.QPushButton("FTP 다운로드")
        self.clock_download_dir_button.clicked.connect(self.browse_clock_download_folder)
        self.clock_cptr_button.clicked.connect(self.capture_clock_photos)
        self.clock_icpt_button.clicked.connect(self.check_clock_capture_status)
        self.clock_download_latest_button.clicked.connect(self.download_latest_clock_photos)
        self.clock_workflow_button.clicked.connect(self.capture_download_analyze_clock_photos)

        workflow_form.addWidget(QtWidgets.QLabel("FTP User"), 0, 0)
        workflow_form.addWidget(self.clock_ftp_user_edit, 0, 1)
        workflow_form.addWidget(QtWidgets.QLabel("Password"), 0, 2)
        workflow_form.addWidget(self.clock_ftp_password_edit, 0, 3)
        workflow_form.addWidget(QtWidgets.QLabel("Port"), 0, 4)
        workflow_form.addWidget(self.clock_ftp_port_spin, 0, 5)
        workflow_form.addWidget(QtWidgets.QLabel("Remote Dir"), 1, 0)
        workflow_form.addWidget(self.clock_ftp_remote_dir_edit, 1, 1, 1, 2)
        workflow_form.addWidget(QtWidgets.QLabel("저장 폴더"), 1, 3)
        workflow_form.addWidget(self.clock_download_dir_edit, 1, 4)
        workflow_form.addWidget(self.clock_download_dir_button, 1, 5)
        workflow_form.addWidget(QtWidgets.QLabel("완료 대기(s)"), 2, 0)
        workflow_form.addWidget(self.clock_capture_wait_spin, 2, 1)
        workflow_form.addWidget(self.clock_workflow_button, 2, 2)
        workflow_form.addWidget(self.clock_cptr_button, 2, 3)
        workflow_form.addWidget(self.clock_icpt_button, 2, 4)
        workflow_form.addWidget(self.clock_download_latest_button, 2, 5)

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

        source_group = QtWidgets.QGroupBox("EXIF 시계 차이")
        source_form = QtWidgets.QGridLayout(source_group)
        source_form.setColumnStretch(1, 1)
        source_form.setColumnStretch(4, 1)
        self.clock_left_file_edit = QtWidgets.QLineEdit()
        self.clock_right_file_edit = QtWidgets.QLineEdit()
        self.clock_left_folder_edit = QtWidgets.QLineEdit()
        self.clock_right_folder_edit = QtWidgets.QLineEdit()
        self.clock_left_file_button = QtWidgets.QPushButton("파일 선택")
        self.clock_right_file_button = QtWidgets.QPushButton("파일 선택")
        self.clock_left_folder_button = QtWidgets.QPushButton("폴더 선택")
        self.clock_right_folder_button = QtWidgets.QPushButton("폴더 선택")
        self.clock_analyze_pair_button = QtWidgets.QPushButton("파일 쌍 비교")
        self.clock_analyze_folder_button = QtWidgets.QPushButton("폴더 비교")
        self.clock_status_label = QtWidgets.QLabel(
            "FTP로 내려받은 좌/우 이미지의 EXIF GPS 시간 또는 촬영 시간을 비교합니다."
        )
        self.clock_status_label.setObjectName("statusStrip")
        self.clock_status_label.setWordWrap(True)
        self.clock_left_file_button.clicked.connect(lambda: self.browse_clock_photo("left"))
        self.clock_right_file_button.clicked.connect(lambda: self.browse_clock_photo("right"))
        self.clock_left_folder_button.clicked.connect(lambda: self.browse_clock_folder("left"))
        self.clock_right_folder_button.clicked.connect(lambda: self.browse_clock_folder("right"))
        self.clock_analyze_pair_button.clicked.connect(self.analyze_clock_photo_pair)
        self.clock_analyze_folder_button.clicked.connect(self.analyze_clock_photo_folders)

        source_form.addWidget(QtWidgets.QLabel("왼쪽 사진"), 0, 0)
        source_form.addWidget(self.clock_left_file_edit, 0, 1)
        source_form.addWidget(self.clock_left_file_button, 0, 2)
        source_form.addWidget(QtWidgets.QLabel("오른쪽 사진"), 0, 3)
        source_form.addWidget(self.clock_right_file_edit, 0, 4)
        source_form.addWidget(self.clock_right_file_button, 0, 5)
        source_form.addWidget(QtWidgets.QLabel("왼쪽 폴더"), 1, 0)
        source_form.addWidget(self.clock_left_folder_edit, 1, 1)
        source_form.addWidget(self.clock_left_folder_button, 1, 2)
        source_form.addWidget(QtWidgets.QLabel("오른쪽 폴더"), 1, 3)
        source_form.addWidget(self.clock_right_folder_edit, 1, 4)
        source_form.addWidget(self.clock_right_folder_button, 1, 5)
        source_form.addWidget(self.clock_analyze_pair_button, 2, 0, 1, 2)
        source_form.addWidget(self.clock_analyze_folder_button, 2, 2, 1, 2)
        source_form.addWidget(self.clock_status_label, 3, 0, 1, 6)

        self.clock_result_table = QtWidgets.QTableWidget(0, 8)
        self.clock_result_table.setHorizontalHeaderLabels(
            ("Pair", "Left file", "Right file", "Left time", "Right time", "Δ right-left", "Source", "Warnings")
        )
        self.clock_result_table.setEditTriggers(QtWidgets.QAbstractItemView.NoEditTriggers)
        self.clock_result_table.setSelectionBehavior(QtWidgets.QAbstractItemView.SelectRows)
        self.clock_result_table.setAlternatingRowColors(True)
        self.clock_result_table.horizontalHeader().setStretchLastSection(True)
        self.clock_result_table.horizontalHeader().setSectionResizeMode(QtWidgets.QHeaderView.ResizeToContents)
        self.clock_result_table.horizontalHeader().setSectionResizeMode(7, QtWidgets.QHeaderView.Stretch)

        self.clock_summary_text = QtWidgets.QPlainTextEdit()
        self.clock_summary_text.setReadOnly(True)
        self.clock_summary_text.setMaximumBlockCount(300)
        self.clock_summary_text.setPlaceholderText("분석 결과가 여기에 표시됩니다.")
        layout.addWidget(rtc_group)
        layout.addWidget(workflow_group)
        layout.addWidget(source_group)
        layout.addWidget(self.clock_result_table, 2)
        layout.addWidget(self.clock_summary_text, 1)

    def browse_clock_photo(self, camera: str) -> None:
        edit = self.clock_left_file_edit if camera == "left" else self.clock_right_file_edit
        start_dir = self._clock_start_dir(edit.text().strip())
        selected, _filter = QtWidgets.QFileDialog.getOpenFileName(
            self,
            f"Select {camera} WIRIS photo",
            str(start_dir),
            "Image files (*.jpg *.jpeg *.tif *.tiff);;All files (*)",
        )
        if selected:
            edit.setText(selected)

    def browse_clock_folder(self, camera: str) -> None:
        edit = self.clock_left_folder_edit if camera == "left" else self.clock_right_folder_edit
        start_dir = self._clock_start_dir(edit.text().strip())
        selected = QtWidgets.QFileDialog.getExistingDirectory(self, f"Select {camera} WIRIS folder", str(start_dir))
        if selected:
            edit.setText(selected)

    def browse_clock_download_folder(self) -> None:
        start_dir = self._clock_start_dir(self.clock_download_dir_edit.text().strip())
        selected = QtWidgets.QFileDialog.getExistingDirectory(self, "Select WIRIS FTP download folder", str(start_dir))
        if selected:
            self.clock_download_dir_edit.setText(selected)

    def capture_clock_photos(self) -> None:
        statuses = self._send_clock_wiris_command_parallel("CPTR")
        status_text = " | ".join(f"{self._camera_name_ko(camera)}: {response or '<no response>'}" for camera, response in statuses)
        self.clock_status_label.setText(f"CPTR 전송: {status_text}")
        self.clock_summary_text.setPlainText(f"CPTR sent\n{status_text}")
        self._log(f"[시계] CPTR 전송: {status_text}")

    def check_clock_capture_status(self) -> None:
        statuses = self._send_clock_wiris_command("ICPT")
        lines = []
        for camera, response in statuses:
            busy = self._wiris_capture_busy(response)
            lines.append(f"{self._camera_name_ko(camera)}: {'capturing' if busy else 'idle'} ({response or '<no response>'})")
        text = "\n".join(lines)
        self.clock_status_label.setText("ICPT 확인 완료")
        self.clock_summary_text.setPlainText(text)
        self._log(f"[시계] ICPT 확인: {' | '.join(lines)}")

    def capture_download_analyze_clock_photos(self) -> None:
        try:
            self.capture_clock_photos()
            completed = self._wait_for_clock_capture_complete(timeout_s=self.clock_capture_wait_spin.value())
            if not completed:
                self.clock_status_label.setText("CPTR 후 ICPT 완료 대기 시간이 초과되었습니다. 최신 파일 다운로드는 계속 시도합니다.")
            paths = self._download_latest_clock_photo_pair()
        except Exception as exc:
            self.clock_status_label.setText(f"촬영/다운로드 실패: {exc}")
            self.clock_summary_text.setPlainText(str(exc))
            self._log(f"[시계] 촬영/다운로드 실패: {exc}")
            return
        self._set_clock_downloaded_paths(paths)
        self.analyze_clock_photo_pair()

    def download_latest_clock_photos(self) -> None:
        try:
            paths = self._download_latest_clock_photo_pair()
        except Exception as exc:
            self.clock_status_label.setText(f"FTP 다운로드 실패: {exc}")
            self.clock_summary_text.setPlainText(str(exc))
            self._log(f"[시계] FTP 다운로드 실패: {exc}")
            return
        self._set_clock_downloaded_paths(paths)
        self.analyze_clock_photo_pair()

    def _send_clock_wiris_command(self, command_text: str) -> list[tuple[str, str]]:
        statuses: list[tuple[str, str]] = []
        for camera, host in self.zoom_hosts("both"):
            command = build_wiris_command(
                host=host,
                port=self._control_port_for_camera(camera),
                command=command_text,
                timeout_s=self.zoom_timeout_spin.value(),
                terminator=LINE_TERMINATOR,
            )
            response = send_tcp_line(command)
            statuses.append((camera, response))
        return statuses

    def _send_clock_wiris_command_parallel(self, command_text: str) -> list[tuple[str, str]]:
        targets = self.zoom_hosts("both")
        if len(targets) <= 1:
            return self._send_clock_wiris_command(command_text)
        barrier = threading.Barrier(len(targets))
        results: dict[str, str] = {}
        send_times: dict[str, float] = {}

        def send_one(camera: str, host: str) -> None:
            try:
                command = build_wiris_command(
                    host=host,
                    port=self._control_port_for_camera(camera),
                    command=command_text,
                    timeout_s=self.zoom_timeout_spin.value(),
                    terminator=LINE_TERMINATOR,
                )
                barrier.wait(timeout=max(1.0, self.zoom_timeout_spin.value()))
                send_times[camera] = time.perf_counter()
                results[camera] = send_tcp_line(command)
            except Exception as exc:
                results[camera] = f"failed: {exc}"

        threads = [
            threading.Thread(target=send_one, args=(camera, host), name=f"clock-{command_text.lower()}-{camera}")
            for camera, host in targets
        ]
        for thread in threads:
            thread.start()
        for thread in threads:
            thread.join(timeout=max(1.0, self.zoom_timeout_spin.value()) + 1.0)
        if len(send_times) >= 2:
            ordered_times = [(camera, send_times[camera]) for camera, _host in targets if camera in send_times]
            if len(ordered_times) >= 2:
                first_camera, first_time = ordered_times[0]
                last_camera, last_time = ordered_times[-1]
                delta_ms = (last_time - first_time) * 1000.0
                self._log(
                    "[시계] CPTR 병렬 전송 시각 차이 "
                    f"{self._camera_name_ko(first_camera)}→{self._camera_name_ko(last_camera)}: {delta_ms:.3f} ms"
                )
        return [(camera, results.get(camera, "<no response>")) for camera, _host in targets]

    def _wait_for_clock_capture_complete(self, *, timeout_s: float) -> bool:
        deadline = time.monotonic() + max(0.0, float(timeout_s))
        while True:
            statuses = self._send_clock_wiris_command("ICPT")
            if not any(self._wiris_capture_busy(response) for _camera, response in statuses):
                return True
            if time.monotonic() >= deadline:
                return False
            QtWidgets.QApplication.processEvents()
            time.sleep(0.25)

    def _wiris_capture_busy(self, response: str) -> bool:
        text = response.strip().upper()
        if not text:
            return False
        if any(token in text for token in ("FALSE", "IDLE", "DONE", "READY", " 0", "=0")) or text == "0":
            return False
        return any(token in text for token in ("TRUE", "BUSY", "CAPT", " 1", "=1")) or text == "1"

    def _clock_sync_side_summary(self, camera: str, metadata: dict[str, object], now_wall_s: float, now_mono_s: float) -> tuple[str, float | None]:
        if metadata.get("rtcp_timing_invalid_reason"):
            return f"{self._camera_name_ko(camera)}: RTCP/NTP 무효(SR-PC offset 초과)", None
        camera_rtc_s = self._camera_rtc_timestamp_s_from_metadata(metadata)
        receive_mono_s = metadata.get("timestamp_s")
        if camera_rtc_s is None:
            return f"{self._camera_name_ko(camera)}: RTCP/NTP 없음", None
        try:
            frame_age_s = max(0.0, now_mono_s - float(receive_mono_s)) if receive_mono_s is not None else 0.0
        except (TypeError, ValueError):
            frame_age_s = 0.0
        frame_receive_delay_s = metadata.get("rtcp_frame_receive_delay_s")
        try:
            if frame_receive_delay_s is not None and isfinite(float(frame_receive_delay_s)):
                return (
                    f"{self._camera_name_ko(camera)}: transit_delay={float(frame_receive_delay_s) * 1000.0:+.1f} ms, "
                    f"frame_age={frame_age_s:.2f} s, basis={infer_runtime_camera_timestamp_reference(metadata)}",
                    float(frame_receive_delay_s) * 1000.0,
                )
        except (TypeError, ValueError):
            pass
        pc_utc_at_frame_s = now_wall_s - frame_age_s
        offset_ms = (float(camera_rtc_s) - pc_utc_at_frame_s) * 1000.0
        reference = infer_runtime_camera_timestamp_reference(metadata)
        return (
            f"{self._camera_name_ko(camera)}: offset={offset_ms:+.1f} ms, "
            f"frame_age={frame_age_s:.2f} s, basis={reference}",
            offset_ms,
        )

    def _clock_sync_pair_metadata(self, now_mono_s: float | None = None) -> tuple[dict[str, object], dict[str, object]]:
        """Select a temporally corresponding recent RTCP frame pair.

        The latest left and latest right frames are not a pair when the
        cameras decode at different rates. Prefer frames received close
        together, then minimize their RTCP-derived timestamp delta.
        """
        now_mono_s = time.monotonic() if now_mono_s is None else float(now_mono_s)
        latest = (
            dict(self.camera_frame_metadata.get("left") or {}),
            dict(self.camera_frame_metadata.get("right") or {}),
        )

        def usable(metadata: dict[str, object]) -> bool:
            return self._camera_rtc_timestamp_s_from_metadata(metadata) is not None

        def recent(history: deque[dict[str, object]]) -> list[dict[str, object]]:
            rows: list[dict[str, object]] = []
            for metadata in history:
                if not usable(metadata):
                    continue
                timestamp_s = metadata.get("timestamp_s")
                if timestamp_s is None:
                    rows.append(dict(metadata))
                    continue
                try:
                    age_s = now_mono_s - float(timestamp_s)
                except (TypeError, ValueError):
                    continue
                if -0.1 <= age_s <= CAMERA_SYNC_HISTORY_MAX_AGE_S:
                    rows.append(dict(metadata))
            return rows

        left_rows = recent(self.camera_frame_metadata_history["left"])
        right_rows = recent(self.camera_frame_metadata_history["right"])
        if not left_rows or not right_rows:
            return latest

        candidates: list[tuple[float, float, dict[str, object], dict[str, object]]] = []
        for left in left_rows:
            left_rtcp_s = self._camera_rtc_timestamp_s_from_metadata(left)
            left_receive_s = left.get("timestamp_s")
            for right in right_rows:
                right_rtcp_s = self._camera_rtc_timestamp_s_from_metadata(right)
                if left_rtcp_s is None or right_rtcp_s is None:
                    continue
                receive_delta_s = float("inf")
                if left_receive_s is not None and right.get("timestamp_s") is not None:
                    try:
                        receive_delta_s = abs(float(right["timestamp_s"]) - float(left_receive_s))
                    except (TypeError, ValueError):
                        pass
                candidates.append(
                    (
                        abs(float(right_rtcp_s) - float(left_rtcp_s)),
                        receive_delta_s,
                        left,
                        right,
                    )
                )
        if not candidates:
            return latest
        close_candidates = [
            candidate
            for candidate in candidates
            if candidate[1] <= CAMERA_SYNC_RECEIVE_PAIR_MAX_DELTA_S
        ]
        search_space = close_candidates or candidates
        _rtcp_delta, _receive_delta, left, right = min(
            search_space,
            key=lambda candidate: (candidate[0], candidate[1]),
        )
        return left, right

    def _camera_time_sync_metrics(
        self,
        left_metadata: dict[str, object] | None = None,
        right_metadata: dict[str, object] | None = None,
        *,
        left_receive_mono_s: float | None = None,
        right_receive_mono_s: float | None = None,
        now_wall_s: float | None = None,
        now_mono_s: float | None = None,
    ) -> dict[str, object]:
        """Summarize RTCP/NTP timing without treating it as verified UTC."""
        now_wall_s = time.time() if now_wall_s is None else float(now_wall_s)
        now_mono_s = time.monotonic() if now_mono_s is None else float(now_mono_s)
        left_metadata = left_metadata or {}
        right_metadata = right_metadata or {}
        left_rtcp_s = self._camera_rtc_timestamp_s_from_metadata(left_metadata)
        right_rtcp_s = self._camera_rtc_timestamp_s_from_metadata(right_metadata)
        invalid_cameras = [
            self._camera_name_ko(camera)
            for camera, metadata in (("left", left_metadata), ("right", right_metadata))
            if metadata.get("rtcp_timing_invalid_reason")
        ]
        offsets_ms: dict[str, float] = {}
        transit_delays_ms: dict[str, float] = {}
        for camera, metadata, explicit_receive_s, rtcp_s in (
            ("left", left_metadata, left_receive_mono_s, left_rtcp_s),
            ("right", right_metadata, right_receive_mono_s, right_rtcp_s),
        ):
            if rtcp_s is None or metadata.get("rtcp_timing_invalid_reason"):
                continue
            frame_receive_delay_s = metadata.get("rtcp_frame_receive_delay_s")
            try:
                if frame_receive_delay_s is not None and isfinite(float(frame_receive_delay_s)):
                    transit_delays_ms[camera] = float(frame_receive_delay_s) * 1000.0
                    continue
            except (TypeError, ValueError):
                pass
            receive_s = explicit_receive_s
            if receive_s is None:
                receive_s = metadata.get("timestamp_s")
            try:
                frame_age_s = max(0.0, now_mono_s - float(receive_s)) if receive_s is not None else 0.0
            except (TypeError, ValueError):
                frame_age_s = 0.0
            pc_utc_at_frame_s = now_wall_s - frame_age_s
            offsets_ms[camera] = (float(rtcp_s) - pc_utc_at_frame_s) * 1000.0

        pair_delta_s = None
        if left_rtcp_s is not None and right_rtcp_s is not None:
            pair_delta_s = float(right_rtcp_s) - float(left_rtcp_s)
        max_offset_ms = max((abs(value) for value in offsets_ms.values()), default=None)
        max_transit_delay_ms = max((abs(value) for value in transit_delays_ms.values()), default=None)
        if pair_delta_s is None:
            if invalid_cameras:
                status = "invalid-rtcp"
                warning = f"{', '.join(invalid_cameras)} Sender Report NTP가 PC 수신 UTC와 500 ms 이상 어긋났습니다."
            else:
                status = "missing-rtcp"
                warning = "좌우 프레임 중 하나에 RTCP/NTP 시각이 없습니다."
        elif abs(pair_delta_s) > CAMERA_RTCP_PAIR_MAX_DELTA_S:
            status = "pair-delta-failed"
            warning = f"좌우 RTCP/NTP 차이가 {CAMERA_RTCP_PAIR_MAX_DELTA_S * 1000.0:.0f} ms를 초과했습니다."
        elif invalid_cameras:
            status = "absolute-rtcp-invalid"
            warning = (
                f"좌우 RTCP/NTP pair는 {abs(pair_delta_s) * 1000.0:.1f} ms 이내지만, "
                f"{', '.join(invalid_cameras)} Sender Report NTP의 PC UTC 기준이 500 ms 이상 어긋났습니다. "
                "절대 UTC 기준으로 사용할 수 없습니다."
            )
        elif max_offset_ms is not None and max_offset_ms > CAMERA_RTCP_MAX_PC_OFFSET_S * 1000.0:
            status = "pc-offset-failed"
            warning = (
                f"RTCP/NTP와 PC UTC 차이가 {CAMERA_RTCP_MAX_PC_OFFSET_S * 1000.0:.0f} ms를 초과했습니다. "
                "카메라 GDTI와 RTCP 매핑을 검증해야 합니다."
            )
        else:
            status = "within-threshold-unverified"
            warning = "좌우/PC 오차는 임계값 이내지만 RTCP/NTP의 UTC 기준은 아직 검증되지 않았습니다."
        return {
            "status": status,
            "basis": "rtcp-sender-report-ntp",
            "pair_delta_s": pair_delta_s,
            "max_pc_offset_ms": max_offset_ms,
            "max_frame_transit_delay_ms": max_transit_delay_ms,
            "left_frame_transit_delay_ms": transit_delays_ms.get("left"),
            "right_frame_transit_delay_ms": transit_delays_ms.get("right"),
            "left_pc_offset_ms": offsets_ms.get("left"),
            "right_pc_offset_ms": offsets_ms.get("right"),
            "warning": warning,
        }

    def _format_clock_sync_status(
        self,
        now_wall_s: float | None = None,
        now_mono_s: float | None = None,
    ) -> tuple[str, str]:
        now_wall_s = time.time() if now_wall_s is None else float(now_wall_s)
        now_mono_s = time.monotonic() if now_mono_s is None else float(now_mono_s)
        left_metadata, right_metadata = self._clock_sync_pair_metadata(now_mono_s)
        left_text, left_offset_ms = self._clock_sync_side_summary("left", left_metadata, now_wall_s, now_mono_s)
        right_text, right_offset_ms = self._clock_sync_side_summary("right", right_metadata, now_wall_s, now_mono_s)
        left_rtc_s = self._camera_rtc_timestamp_s_from_metadata(left_metadata)
        right_rtc_s = self._camera_rtc_timestamp_s_from_metadata(right_metadata)
        invalid_cameras = [
            self._camera_name_ko(camera)
            for camera, metadata in (("left", left_metadata), ("right", right_metadata))
            if metadata.get("rtcp_timing_invalid_reason")
        ]
        if left_rtc_s is not None and right_rtc_s is not None:
            pair_delta_ms = (float(right_rtc_s) - float(left_rtc_s)) * 1000.0
            pair_text = f"좌우 RTCP/NTPΔ={pair_delta_ms:+.1f} ms"
        else:
            pair_text = "좌우 RTCP/NTPΔ=RTCP/NTP 부족"
        quality_parts = []
        if left_offset_ms is not None:
            quality_parts.append(abs(left_offset_ms))
        if right_offset_ms is not None:
            quality_parts.append(abs(right_offset_ms))
        worst_quality_ms = max(quality_parts) if quality_parts else None
        has_transit_delay = any(
            metadata.get("rtcp_frame_receive_delay_s") is not None
            for metadata in (left_metadata, right_metadata)
        )
        quality_label = "RTCP frame transit max" if has_transit_delay else "PC UTC 최대 offset"
        status = (
            f"스트림 RTCP/NTP 상태: {pair_text}"
            if worst_quality_ms is None
            else f"스트림 RTCP/NTP 상태: {pair_text}, {quality_label}={worst_quality_ms:.1f} ms"
        )
        if invalid_cameras:
            status += f", 절대 UTC 검증 실패={','.join(invalid_cameras)}"
        detail = "\n".join(
            (
                "time_basis: RTCP Sender Report NTP-derived stream frame timestamp; WIRIS GDTI/UTC mapping unverified",
                "pc_utc_at_frame: current PC UTC minus app receive frame age",
                left_text,
                right_text,
                pair_text,
            )
        )
        return status, detail

    def refresh_clock_sync_status(self) -> None:
        if not hasattr(self, "clock_sync_status_label"):
            return
        status, detail = self._format_clock_sync_status()
        self.clock_sync_status_label.setText(status)
        self.clock_sync_status_label.setToolTip(detail)
        self.clock_sync_summary_text.setPlainText(detail)
        self._log(f"[시계] {status}")

    def _download_latest_clock_photo_pair(self) -> dict[str, Path]:
        output_root = self._clock_download_root()
        paths = {}
        for camera, host in self.zoom_hosts("both"):
            paths[camera] = self._download_latest_ftp_image(camera, host, output_root)
        return paths

    def _download_latest_ftp_image(self, camera: str, host: str, output_root: Path) -> Path:
        normalized_host = host.strip()
        if not normalized_host:
            raise ValueError(f"{self._camera_name_ko(camera)} WIRIS host is empty")
        user = self.clock_ftp_user_edit.text().strip() or "anonymous"
        password = self.clock_ftp_password_edit.text()
        remote_dir = self.clock_ftp_remote_dir_edit.text().strip() or "/"
        port = int(self.clock_ftp_port_spin.value())
        timeout_s = float(self.zoom_timeout_spin.value())
        camera_dir = output_root / camera
        camera_dir.mkdir(parents=True, exist_ok=True)
        with ftplib.FTP() as ftp:
            ftp.connect(normalized_host, port, timeout=timeout_s)
            ftp.login(user=user, passwd=password)
            if remote_dir and remote_dir != ".":
                ftp.cwd(remote_dir)
            latest = self._latest_ftp_image_name(ftp)
            output = camera_dir / Path(latest).name
            if output.exists():
                timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
                output = camera_dir / f"{timestamp}_{output.name}"
            with output.open("wb") as handle:
                ftp.retrbinary(f"RETR {latest}", handle.write)
        self._log(f"[시계] {self._camera_name_ko(camera)} FTP 최신 이미지 다운로드: {output}")
        return output

    def _latest_ftp_image_name(self, ftp: ftplib.FTP) -> str:
        names = [
            name
            for name in ftp.nlst()
            if Path(name).suffix.lower() in WIRIS_FTP_IMAGE_EXTENSIONS
        ]
        if not names:
            raise FileNotFoundError("FTP remote directory has no JPG/TIFF images")

        def sort_key(name: str) -> tuple[float, str]:
            try:
                response = ftp.voidcmd(f"MDTM {name}")
                timestamp = response.split()[-1]
                parsed = datetime.strptime(timestamp, "%Y%m%d%H%M%S").replace(tzinfo=timezone.utc)
                return parsed.timestamp(), name
            except Exception:
                return 0.0, name

        return max(names, key=sort_key)

    def _set_clock_downloaded_paths(self, paths: dict[str, Path]) -> None:
        left_path = paths.get("left")
        right_path = paths.get("right")
        if left_path is not None:
            self.clock_left_file_edit.setText(str(left_path))
            self.clock_left_folder_edit.setText(str(left_path.parent))
        if right_path is not None:
            self.clock_right_file_edit.setText(str(right_path))
            self.clock_right_folder_edit.setText(str(right_path.parent))

    def analyze_clock_photo_pair(self) -> None:
        left_path = self.clock_left_file_edit.text().strip()
        right_path = self.clock_right_file_edit.text().strip()
        if not left_path or not right_path:
            self.clock_status_label.setText("좌/우 이미지 파일을 모두 선택하세요.")
            return
        try:
            result = compare_photo_clocks(left_path, right_path)
        except PhotoMetadataError as exc:
            self.clock_status_label.setText(f"분석 실패: {exc}")
            self.clock_summary_text.setPlainText(str(exc))
            return
        self._set_clock_table_results([result])
        summary = self._format_clock_result_summary(result)
        self.clock_summary_text.setPlainText(summary)
        self.clock_status_label.setText(f"파일 쌍 clock offset: {format_offset_seconds(result.offset_seconds)}")
        self._log(f"[시계] 파일 쌍 offset: {format_offset_seconds(result.offset_seconds)}")

    def analyze_clock_photo_folders(self) -> None:
        left_dir = self.clock_left_folder_edit.text().strip()
        right_dir = self.clock_right_folder_edit.text().strip()
        if not left_dir or not right_dir:
            self.clock_status_label.setText("좌/우 이미지 폴더를 모두 선택하세요.")
            return
        try:
            result = compare_photo_folders(left_dir, right_dir)
        except PhotoMetadataError as exc:
            self.clock_status_label.setText(f"분석 실패: {exc}")
            self.clock_summary_text.setPlainText(str(exc))
            return
        self._set_clock_table_results(result.pairs)
        summary = self._format_clock_folder_summary(result)
        self.clock_summary_text.setPlainText(summary)
        self.clock_status_label.setText(
            f"폴더 clock offset median: {format_offset_seconds(result.median_offset_seconds)}"
        )
        self._log(
            "[시계] 폴더 offset: "
            f"pairs={len(result.pairs)}, median={format_offset_seconds(result.median_offset_seconds)}"
        )

    def _set_clock_table_results(self, results: list[ClockOffsetResult] | tuple[ClockOffsetResult, ...]) -> None:
        self.clock_result_table.setRowCount(0)
        for index, result in enumerate(results, start=1):
            self._append_clock_result_row(index, result)
        self.clock_result_table.resizeColumnsToContents()
        self.clock_result_table.horizontalHeader().setSectionResizeMode(7, QtWidgets.QHeaderView.Stretch)

    def _append_clock_result_row(self, index: int, result: ClockOffsetResult) -> None:
        row = self.clock_result_table.rowCount()
        self.clock_result_table.insertRow(row)
        source = result.left.capture_time_source
        if result.right.capture_time_source != source:
            source = f"{source} / {result.right.capture_time_source}"
        values = (
            str(index),
            result.left.path.name,
            result.right.path.name,
            format_photo_timestamp(result.left),
            format_photo_timestamp(result.right),
            format_offset_seconds(result.offset_seconds),
            source,
            "; ".join(dict.fromkeys(result.warnings)) or "none",
        )
        for column, value in enumerate(values):
            item = QtWidgets.QTableWidgetItem(value)
            if column == 5:
                item.setTextAlignment(QtCore.Qt.AlignRight | QtCore.Qt.AlignVCenter)
            self.clock_result_table.setItem(row, column, item)

    def _format_clock_result_summary(self, result: ClockOffsetResult) -> str:
        lines = [
            f"Offset (right-left): {format_offset_seconds(result.offset_seconds)}",
            "",
            f"Left: {result.left.path}",
            f"  time: {format_photo_timestamp(result.left)}",
            f"  gps: {self._format_clock_gps(result.left.gps_latitude, result.left.gps_longitude)}",
            f"Right: {result.right.path}",
            f"  time: {format_photo_timestamp(result.right)}",
            f"  gps: {self._format_clock_gps(result.right.gps_latitude, result.right.gps_longitude)}",
        ]
        unique_warnings = list(dict.fromkeys(result.warnings))
        if unique_warnings:
            lines.extend(["", "Warnings:", *[f"- {warning}" for warning in unique_warnings]])
        return "\n".join(lines)

    def _format_clock_folder_summary(self, result: ClockFolderResult) -> str:
        valid_offsets = result.valid_offsets
        lines = [
            f"Pairing mode: {result.pairing_mode}",
            f"Pairs analyzed: {len(result.pairs)}",
            f"Valid offsets: {len(valid_offsets)}",
            f"Median offset: {format_offset_seconds(result.median_offset_seconds)}",
            f"Mean offset: {format_offset_seconds(result.mean_offset_seconds)}",
        ]
        if valid_offsets:
            lines.append(f"Min/Max offset: {min(valid_offsets):+.6f} s / {max(valid_offsets):+.6f} s")
        warnings = list(result.warnings)
        for pair in result.pairs:
            warnings.extend(pair.warnings)
        unique_warnings = list(dict.fromkeys(warnings))
        if unique_warnings:
            lines.extend(["", "Warnings:", *[f"- {warning}" for warning in unique_warnings]])
        return "\n".join(lines)

    def _format_clock_gps(self, latitude: float | None, longitude: float | None) -> str:
        if latitude is None or longitude is None:
            return "missing"
        return f"{latitude:.7f}, {longitude:.7f}"

    def _clock_start_dir(self, text: str) -> Path:
        default_dir = resolve_settings_path(DEFAULT_CLOCK_DOWNLOAD_DIR, base_dir=APP_ROOT)
        if text:
            path = resolve_settings_path(text, base_dir=APP_ROOT)
            if path.is_dir():
                return path
            if path.parent.exists():
                return path.parent
        return default_dir

    def _clock_download_root(self) -> Path:
        text = self.clock_download_dir_edit.text().strip() or DEFAULT_CLOCK_DOWNLOAD_DIR
        path = resolve_settings_path(text, base_dir=APP_ROOT)
        path.mkdir(parents=True, exist_ok=True)
        return path

    def yolo_kwargs(self) -> dict:
        return {
            "weights_path": self.weights_edit.text().strip(),
            "device": self.device_edit.text().strip(),
            "input_size": self.input_size_spin.value(),
            "conf_threshold": self.conf_spin.value(),
            "iou_threshold": self.iou_spin.value(),
            "max_rate_hz": 0.0,
            "display_rate_hz": 0.0,
            "preview_max_width": self.preview_width_spin.value(),
            "rtsp_latency_ms": self.rtsp_latency_spin.value(),
            "event_queue_size": self.yolo_event_queue_size_spin.value(),
            "frame_queue_size": self.yolo_frame_queue_size_spin.value(),
            "decoder": self.decoder_combo.currentData(),
            "stream_backend": self.stream_backend_combo.currentData(),
            "rtsp_transport": self.rtsp_transport_combo.currentData(),
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
        return session_dir(APP_ROOT / "recordings", stamp or self.recording_session_id)

    def _prepare_video_recording_paths(self) -> None:
        root = APP_ROOT / "recordings"
        self.video_recording_paths = raw_video_paths(root, self.recording_session_id)
        output_dir = session_dir(root, self.recording_session_id)
        output_dir.mkdir(parents=True, exist_ok=True)

    def _format_video_recording_paths(self) -> str:
        return format_video_paths(self.video_recording_paths)

    def _session_recording_active(self) -> bool:
        return self.distance_recording_enabled or self.video_recording_enabled

    def browse_yolo_weights(self) -> None:
        current_path = Path(self.weights_edit.text().strip() or DEFAULT_YOLO_WEIGHTS).expanduser()
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

    def auto_connect_wiris(self) -> None:
        self.auto_wiris_connect_pending = True
        self._start_camera_detection("auto_connect")

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

    def _ensure_camera_host_ip(self, context: str) -> bool:
        interface_name = self.interface_edit.text().strip()
        address_text = self.host_address_edit.text().strip()
        if not interface_name or not interface_link_exists(interface_name):
            reason = "비어 있습니다" if not interface_name else f"현재 시스템에 없습니다: {interface_name}"
            self._log(f"[연결] {context}: 카메라망 인터페이스가 {reason}. 유선 어댑터 자동 선택을 시도합니다.")
            candidates = discover_host_interface_candidates(
                host_address=address_text or DEFAULT_HOST_ADDRESS,
                include_unconfigured=True,
            )
            if candidates:
                self._apply_host_interface_candidates(candidates)
                interface_name = self.interface_edit.text().strip()
                address_text = self.host_address_edit.text().strip()
            else:
                self._log(f"[연결] {context}: 사용할 수 있는 유선 카메라망 인터페이스를 찾지 못했습니다.")
                self.camera_connection_label.setText("camera host IP: interface not found")
                return False
        resolved_address = resolve_nm_interface_address(interface_name, self._camera_host_hints())
        if resolved_address is not None:
            resolved_text = str(resolved_address)
            if resolved_text != address_text:
                self.host_address_edit.setText(resolved_text)
                self._log(
                    f"[연결] {context}: NetworkManager 활성 프로필에서 카메라망 IP를 찾았습니다: "
                    f"{interface_name} {resolved_text}"
                )
            address_text = resolved_text
        if not address_text:
            self._log(f"[연결] {context}: 카메라망 인터페이스 또는 IP가 비어 있습니다.")
            self.camera_connection_label.setText("camera host IP: missing")
            return False
        if interface_has_address(interface_name, address_text):
            return True
        self._log(
            f"[연결] {context}: 카메라망 IP가 아직 적용되지 않았습니다. "
            f"자동 적용을 시도합니다: {interface_name} {address_text}"
        )
        ok, message = apply_interface_address(interface_name, address_text)
        if ok:
            self._log(f"[연결] {context}: 카메라망 IP 자동 적용 완료: {message}")
            self._log("[연결] 네트워크 인터페이스 상태:\n" + interface_addresses(interface_name))
            self._save_current_settings()
            return True
        self._log(f"[연결] {context}: 카메라망 IP 자동 적용 실패: {message}")
        self.camera_connection_label.setText("camera host IP: apply failed")
        return False

    def _start_camera_detection(self, mode: str, *, target_side: str | None = None) -> None:
        if self.camera_auto_detect_thread is not None:
            return
        if not self._ensure_camera_host_ip(f"카메라 {mode} 탐색"):
            return
        if mode == "auto_connect":
            self._log(
                "[카메라] WIRIS 자동 연결을 시작합니다: 제어 포트 전체 스캔, HIWS 검증, "
                "RTSP 활성화, 프레임 확인을 순차 실행합니다."
            )
        elif mode == "control":
            self._log("[카메라] 카메라 네트워크에서 WIRIS 제어 포트를 전체 스캔하고 HIWS로 검증합니다.")
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

    def auto_detect_pixhawks(self) -> None:
        if self.pixhawk_auto_detect_thread is not None:
            return
        self._log("[픽스호크] ROS2 PX4 topic과 namespace를 자동 탐색합니다.")
        self.pixhawk_auto_detect_button.setEnabled(False)
        self.pixhawk_auto_detect_thread = QtCore.QThread(self)
        self.pixhawk_auto_detect_worker = PixhawkAutoDetectWorker(domain_id=self.domain_spin.value())
        self.pixhawk_auto_detect_worker.moveToThread(self.pixhawk_auto_detect_thread)
        self.pixhawk_auto_detect_thread.started.connect(self.pixhawk_auto_detect_worker.run)
        self.pixhawk_auto_detect_worker.detected.connect(self._apply_pixhawk_discovery)
        self.pixhawk_auto_detect_worker.failed.connect(self._pixhawk_auto_detect_failed)
        self.pixhawk_auto_detect_worker.finished.connect(self.pixhawk_auto_detect_thread.quit)
        self.pixhawk_auto_detect_worker.finished.connect(self.pixhawk_auto_detect_worker.deleteLater)
        self.pixhawk_auto_detect_thread.finished.connect(self.pixhawk_auto_detect_thread.deleteLater)
        self.pixhawk_auto_detect_thread.finished.connect(self._clear_pixhawk_auto_detect_worker)
        self.pixhawk_auto_detect_thread.start()

    def _apply_camera_detection(self, result: object) -> None:
        if isinstance(result, dict):
            mode = str(result.get("mode") or "rtsp")
            target_side = result.get("target_side")
            host_interfaces = list(result.get("host_interfaces") or [])
            rtsp_candidates = list(result.get("rtsp") or [])
            control_candidates = list(result.get("control") or [])
            verified_control_candidates = list(result.get("verified_control") or [])
            arp_scan_reports = list(result.get("arp_scan") or [])
        else:
            mode = "rtsp"
            target_side = None
            host_interfaces = []
            rtsp_candidates = list(result or [])
            control_candidates = []
            verified_control_candidates = []
            arp_scan_reports = []
        self._log_arp_scan_reports(arp_scan_reports)
        self._set_camera_candidate_rows(host_interfaces, rtsp_candidates, control_candidates, verified_control_candidates)
        if host_interfaces:
            self._apply_host_interface_candidates(host_interfaces)
        if mode == "auto_connect":
            self._apply_auto_wiris_connect_candidates(verified_control_candidates, control_candidates)
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

    def _apply_auto_wiris_connect_candidates(self, verified: list[object], fallback: list[object]) -> None:
        ok_candidates = self._ok_control_candidates(verified)
        self._log_verified_control_candidates(verified)
        if not ok_candidates:
            self.auto_wiris_connect_pending = False
            if fallback:
                self._log("[카메라] 열린 TCP 포트 후보는 있으나 HIWS=OK 응답을 받은 WIRIS를 찾지 못했습니다.")
                self.camera_connection_label.setText("WIRIS auto connect: no HIWS OK response")
            else:
                self._log("[카메라] WIRIS 제어 포트 후보를 찾지 못했습니다. PC IP 대역 또는 카메라 IP를 확인하세요.")
                self.camera_connection_label.setText("WIRIS auto connect: no control candidates")
            self._refresh_operation_header()
            return

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
        self.camera_connection_label.setText("WIRIS auto connect: control verified, enabling RTSP")
        self.enable_wiris_rtsp()

    def _apply_pixhawk_discovery(self, discovery: object | None) -> None:
        if discovery is None:
            self._log("[픽스호크] ROS2 PX4 topic을 자동으로 찾지 못했습니다.")
            return
        self.domain_spin.setValue(discovery.domain_id)
        namespaces = list(discovery.namespaces)
        self._set_pixhawk_candidate_rows(
            [
                {
                    "kind": "ROS2",
                    "value": namespace,
                    "baud": "",
                    "status": f"domain={discovery.domain_id}",
                }
                for namespace in namespaces
            ],
            source="ros2",
        )
        if namespaces:
            self.left_namespace_edit.setText(namespaces[0])
        if len(namespaces) > 1:
            self.right_namespace_edit.setText(namespaces[1])
        self._log(
            f"[픽스호크] 자동 탐색: ROS_DOMAIN_ID={discovery.domain_id}, "
            f"namespace={namespaces or ['<root>']}, topic={len(discovery.topics)}개"
        )

    def _pixhawk_candidate_table_for_source(self, source: str) -> QtWidgets.QTableWidget:
        if source == "pc":
            return self.pc_pixhawk_candidate_table
        if source == "ros2":
            return self.ros2_pixhawk_candidate_table
        return self.mavlink_pixhawk_candidate_table

    def _pixhawk_candidate_source_for_kind(self, kind: str) -> str:
        if kind == "PC_IP":
            return "pc"
        if kind == "ROS2":
            return "ros2"
        return "mavlink"

    def _sync_legacy_pixhawk_candidate_rows(self) -> None:
        self._pixhawk_candidate_rows = [
            *self._pixhawk_candidate_rows_by_source.get("pc", []),
            *self._pixhawk_candidate_rows_by_source.get("mavlink", []),
            *self._pixhawk_candidate_rows_by_source.get("ros2", []),
        ]

    def _set_pixhawk_candidate_rows(self, rows: list[dict[str, object]], *, source: str | None = None) -> None:
        if source is None:
            grouped: dict[str, list[dict[str, object]]] = {"pc": [], "mavlink": [], "ros2": []}
            for entry in rows:
                grouped[self._pixhawk_candidate_source_for_kind(str(entry.get("kind", "")))].append(entry)
            self._pixhawk_candidate_rows_by_source = grouped
            for key, source_rows in grouped.items():
                self._render_pixhawk_candidate_rows(key, source_rows)
        else:
            self._pixhawk_candidate_rows_by_source[source] = rows
            self._render_pixhawk_candidate_rows(source, rows)
        self._sync_legacy_pixhawk_candidate_rows()
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
        ros2_selected = self.ros2_pixhawk_candidate_table.currentRow() >= 0
        self.clear_pixhawk_pc_candidate_button.setEnabled(bool(self._pixhawk_candidate_rows_by_source.get("pc")))
        self.clear_pixhawk_candidate_button.setEnabled(bool(mavlink_rows))
        self.clear_pixhawk_ros2_candidate_button.setEnabled(bool(self._pixhawk_candidate_rows_by_source.get("ros2")))
        self.clear_pixhawk_candidate_button.setToolTip("Pixhawk MAVLink/USB 후보 내역을 지웁니다.")
        self.apply_pixhawk_candidate_host_button.setEnabled(pc_selected)
        self.apply_pixhawk_candidate_host_button.setToolTip(
            "선택한 PC IP 후보를 Pixhawk Host IP/uXRCE Agent IP에 적용합니다."
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
        self.apply_pixhawk_ros2_candidate_left_button.setEnabled(ros2_selected)
        self.apply_pixhawk_ros2_candidate_right_button.setEnabled(ros2_selected)
        for button in (self.apply_pixhawk_ros2_candidate_left_button, self.apply_pixhawk_ros2_candidate_right_button):
            button.setToolTip(
                "선택한 ROS2 namespace 후보를 P1/P2에 적용합니다."
                if ros2_selected
                else "ROS2 namespace 후보를 선택하면 적용할 수 있습니다."
            )

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
        for row in rows:
            self._log(
                f"[픽스호크] MAVLink URL 후보: {row['value']} "
                f"status={row['status']} host={row['interface']}"
            )

    def discover_usb_pixhawk_connections(self) -> None:
        self.pixhawk_usb_discover_button.setEnabled(False)
        self._log("[픽스호크] USB Pixhawk 연결 탐색을 시작합니다. 후보는 자동 할당하지 않습니다.")
        try:
            candidates = usb_mavlink_device_candidates()
            found = probe_usb_mavlink_devices(candidates=candidates, timeout_s=0.8) if candidates else []
        except Exception as exc:
            self.pixhawk_usb_discover_button.setEnabled(True)
            self._log(f"[픽스호크] USB Pixhawk 연결 탐색 실패: {exc}")
            return
        self.pixhawk_usb_discover_button.setEnabled(True)
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
            self._log("[픽스호크] USB Pixhawk 장치가 보이지 않습니다: /dev/ttyACM*, /dev/ttyUSB* 없음")
            return
        self._append_pixhawk_candidate_rows(rows, source="mavlink")
        for row in rows:
            self._log(
                f"[픽스호크] USB 후보: {row['value']} "
                f"baud={row['baud'] or '-'} status={row['status']}"
            )

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

    def apply_selected_pixhawk_candidate(self, side: str, *, source: str | None = None) -> None:
        if source is None:
            source = "pc" if side == "host" else "mavlink"
        entry = self._selected_pixhawk_candidate(source)
        if entry is None:
            return
        kind = str(entry.get("kind", ""))
        value = str(entry.get("value", "")).strip()
        if not value:
            self._log("[픽스호크] 선택 후보에 적용할 값이 없습니다.")
            return
        if kind == "PC_IP":
            if side != "host":
                self._log("[픽스호크] PC IP 후보는 Host에 적용 버튼으로 적용하세요.")
                return
            old_text = self.pixhawk_host_address_edit.text().strip()
            self.pixhawk_host_address_edit.setText(value)
            self._log(
                f"[픽스호크] 선택 PC IP 후보를 Pixhawk Host IP에 적용했습니다: "
                f"{old_text or '<empty>'} -> {value} ({entry.get('interface', '')})"
            )
            self.pixhawk_connection_label.setText(f"Pixhawk Host IP 후보 적용됨: {value}")
            self._save_current_settings()
            self._refresh_operation_header()
            return
        if side not in {"left", "right"}:
            self._log(f"[픽스호크] {kind} 후보는 P1/P2에 적용하세요.")
            return
        mavlink_was_active = side in self._mavlink_open_sides()
        ros2_was_active = side in self._ros2_connected_sides()
        if kind == "ROS2":
            if side == "left":
                self.left_namespace_edit.setText(value)
            else:
                self.right_namespace_edit.setText(value)
        elif kind == "USB":
            try:
                baud = int(entry.get("baud") or 0)
            except (TypeError, ValueError):
                baud = 0
            if side == "left":
                self.left_pixhawk_bootstrap_url_edit.setText(value)
                if baud:
                    self.left_pixhawk_bootstrap_baud_spin.setValue(baud)
            else:
                self.right_pixhawk_bootstrap_url_edit.setText(value)
                if baud:
                    self.right_pixhawk_bootstrap_baud_spin.setValue(baud)
        elif kind == "MAVLink":
            try:
                baud = int(entry.get("baud") or 0)
            except (TypeError, ValueError):
                baud = 0
            if side == "left":
                self.left_mavlink_url_edit.setText(value)
                if baud:
                    self.left_mavlink_baud_spin.setValue(baud)
            else:
                self.right_mavlink_url_edit.setText(value)
                if baud:
                    self.right_mavlink_baud_spin.setValue(baud)
        else:
            self._log(f"[픽스호크] 지원하지 않는 후보 종류입니다: {kind}")
            return
        label = "Pixhawk 1" if side == "left" else "Pixhawk 2"
        self._log(f"[픽스호크] 선택 후보를 {label}에 적용했습니다: {kind} {value}")
        if kind in {"USB", "MAVLink"}:
            self._refresh_mavlink_side_actions()
            if kind == "MAVLink" and mavlink_was_active:
                self._log(f"[픽스호크] {label} MAVLink URL 후보 적용으로 기존 {label} MAVLink 세션을 해제합니다. 다시 연결하세요.")
                self.disconnect_mavlink_pixhawk(side)
            if kind == "USB":
                self.pixhawk_connection_label.setText(f"{label} USB 부트스트랩 후보 적용됨: ETH MAV 시작(USB)을 누르세요.")
            else:
                self.pixhawk_connection_label.setText(f"{label} MAVLink URL 후보 적용됨: {label} MAVLink 연결을 누르세요.")
        elif kind == "ROS2":
            if ros2_was_active:
                self._log(f"[픽스호크] {label} ROS2 후보 적용으로 {label} ROS2 세션을 새 namespace로 재시작합니다.")
                self.disconnect_pixhawk(side)
                self.connect_pixhawk(side)
            else:
                self.pixhawk_connection_label.setText(f"{label} ROS2 후보 적용됨: {label} ROS2 연결을 누르세요.")
        self._refresh_operation_header()

    def _camera_auto_detect_failed(self, error: str) -> None:
        self._log(f"[카메라] 자동 탐색 실패: {error}")
        self.camera_connection_label.setText(f"camera RTSP: auto detect failed ({error})")
        self.last_rtsp_check_status = {"left": "check-failed", "right": "check-failed"}
        self._refresh_operation_header()

    def _pixhawk_auto_detect_failed(self, error: str) -> None:
        self._log(f"[픽스호크] 자동 탐색 실패: {error}")
        self.pixhawk_connection_label.setText(f"pixhawk: auto detect failed ({error})")

    def _clear_camera_auto_detect_worker(self) -> None:
        self.camera_auto_detect_thread = None
        self.camera_auto_detect_worker = None
        self.auto_wiris_connect_button.setEnabled(True)
        self.detect_control_button.setEnabled(True)
        self.right_detect_control_button.setEnabled(True)
        self.detect_rtsp_button.setEnabled(True)
        self.right_detect_rtsp_button.setEnabled(True)

    def _clear_pixhawk_auto_detect_worker(self) -> None:
        self.pixhawk_auto_detect_thread = None
        self.pixhawk_auto_detect_worker = None
        self.pixhawk_auto_detect_button.setEnabled(True)

    def swap_cameras(self) -> None:
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

    def swap_pixhawks(self) -> None:
        ros2_connected_sides = self._ros2_connected_sides()
        mavlink_connected_sides = self._mavlink_open_sides()
        if ros2_connected_sides:
            self.disconnect_pixhawk()
        for side in ("left", "right"):
            if side in mavlink_connected_sides:
                self.disconnect_mavlink_pixhawk(side)
        self.left_namespace_edit.setText(self._swap_line_edit_text(self.left_namespace_edit, self.right_namespace_edit))
        self.left_mavlink_url_edit.setText(self._swap_line_edit_text(self.left_mavlink_url_edit, self.right_mavlink_url_edit))
        self.left_pixhawk_eth_ip_edit.setText(
            self._swap_line_edit_text(self.left_pixhawk_eth_ip_edit, self.right_pixhawk_eth_ip_edit)
        )
        self._log("[픽스호크] Pixhawk 1/2 ROS2 namespace와 MAVLink URL 매핑을 교체했습니다.")
        for side in ("left", "right"):
            if side in ros2_connected_sides:
                self.connect_pixhawk(side)
        for side in ("left", "right"):
            if side in mavlink_connected_sides:
                self.connect_mavlink_pixhawk(side)

    def _swap_line_edit_text(self, left: QtWidgets.QLineEdit, right: QtWidgets.QLineEdit) -> str:
        left_text = left.text()
        right_text = right.text()
        right.setText(left_text)
        return right_text

    def check_cameras(self) -> None:
        if self.rtsp_check_thread is not None:
            self._log("[카메라] RTSP 스트림 확인이 이미 진행 중입니다.")
            return
        self._ensure_camera_host_ip("RTSP 수신 확인")
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
        self._ensure_camera_host_ip("WIRIS 제어 확인")
        statuses = []
        for label, host in self.zoom_hosts("both"):
            normalized_host = host.strip()
            camera_name = self._camera_name_ko(label)
            if not normalized_host:
                statuses.append(f"{label}=missing-host")
                self._log(f"[카메라] {camera_name} WIRIS 제어 host가 비어 있습니다.")
                continue
            self._log_rtsp_control_host_mismatch(label, normalized_host)
            port = self._control_port_for_camera(label)
            port_open = check_tcp_port(
                normalized_host,
                port,
                timeout=self.zoom_timeout_spin.value(),
            )
            if port_open:
                statuses.append(f"{label}=control-open")
                self._log(f"[카메라] {camera_name} WIRIS 제어 포트 열림: {normalized_host}:{port}")
            else:
                statuses.append(f"{label}=control-closed")
                self._log(f"[카메라] {camera_name} WIRIS 제어 포트 닫힘/오류: {normalized_host}:{port}")
        self.camera_connection_label.setText(f"camera control: {', '.join(statuses)}")

    def _ethernet_mode_response_ok(self, response: str) -> bool:
        return response.strip().upper() in {"TRUE", "FALSE"}

    def _set_ethernet_response_ok(self, response: str) -> bool:
        return response.strip().upper() in {"OK", "TRUE"}

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

    def _probe_ethernet_mode_port(self, host: str, port: int) -> tuple[bool, str]:
        command = build_wiris_command(
            host=host,
            port=port,
            command=GET_ETHERNET_MODE_COMMAND,
            timeout_s=self.zoom_timeout_spin.value(),
        )
        response = send_tcp_line(command)
        return self._ethernet_mode_response_ok(response), response

    def _resolve_ethernet_mode_port(self, camera: str, host: str) -> tuple[int | None, str]:
        failures = []
        for port in self._ethernet_mode_candidate_ports(camera):
            try:
                ok, response = self._probe_ethernet_mode_port(host, port)
            except Exception as exc:
                failures.append(f"{port}=failed:{exc}")
                continue
            if ok:
                current_port = self._control_port_for_camera(camera)
                if port != current_port:
                    self._set_control_port_for_camera(camera, port)
                    self.zoom_port_spin.setValue(port)
                    self._save_current_settings()
                    self._log(
                        f"[카메라] {self._camera_name_ko(camera)} Telnet port 자동 보정: "
                        f"{current_port} -> {port} ({GET_ETHERNET_MODE_COMMAND}={response})"
                    )
                return port, response
            failures.append(f"{port}={response or 'empty'}")
        self._log(
            f"[카메라] {self._camera_name_ko(camera)} Ethernet Mode 지원 포트를 찾지 못했습니다: "
            + ", ".join(failures)
        )
        return None, failures[-1] if failures else "no-candidate"

    def enable_wiris_rtsp(self) -> None:
        if self.rtsp_enable_thread is not None:
            self._log("[카메라] RTSP 송출 활성화가 이미 진행 중입니다.")
            return
        self._ensure_camera_host_ip("RTSP 송출 활성화")
        hosts = {label: host.strip() for label, host in self.zoom_hosts("both")}
        candidate_ports = {label: self._ethernet_mode_candidate_ports(label) for label in ("left", "right")}
        urls = {
            "left": self.left_url_edit.text().strip(),
            "right": self.right_url_edit.text().strip(),
        }
        self.enable_rtsp_button.setEnabled(False)
        self.camera_connection_label.setText("camera RTSP enable: enabling...")
        self.rtsp_enable_thread = QtCore.QThread(self)
        self.rtsp_enable_worker = WirisRtspEnableWorker(
            hosts=hosts,
            candidate_ports=candidate_ports,
            urls=urls,
            timeout_s=self.zoom_timeout_spin.value(),
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

    def _apply_wiris_rtsp_enable_result(self, result: object) -> None:
        data = dict(result or {})
        for label, port in dict(data.get("corrected_ports") or {}).items():
            self._set_control_port_for_camera(str(label), int(port))
            self.zoom_port_spin.setValue(int(port))
            self._save_current_settings()
        for message in list(data.get("logs") or []):
            self._log(str(message))
        statuses = [str(status) for status in list(data.get("statuses") or [])]
        self.camera_connection_label.setText(f"camera RTSP enable: {', '.join(statuses)}")
        if data.get("should_check"):
            self.check_cameras()

    def _wiris_rtsp_enable_failed(self, error: str) -> None:
        self._log(f"[카메라] RTSP 송출 활성화 실패: {error}")
        self.camera_connection_label.setText(f"camera RTSP enable: failed ({error})")

    def _clear_rtsp_enable_worker(self) -> None:
        self.rtsp_enable_thread = None
        self.rtsp_enable_worker = None
        self.enable_rtsp_button.setEnabled(True)

    def apply_wiris_stip(self, camera: str) -> None:
        camera_name = self._camera_name_ko(camera)
        host_edit = self.left_zoom_host_edit if camera == "left" else self.right_zoom_host_edit
        new_ip_edit = self.left_stip_ip_edit if camera == "left" else self.right_stip_ip_edit
        url_edit = self.left_url_edit if camera == "left" else self.right_url_edit
        current_host = host_edit.text().strip()
        new_ip = new_ip_edit.text().strip()
        mask = self.stip_mask_edit.text().strip()
        try:
            IPv4Address(current_host)
            IPv4Address(new_ip)
            IPv4Address(mask)
        except ValueError as exc:
            self._log(f"[카메라] {camera_name} STIP 입력 오류: {exc}")
            self.camera_connection_label.setText(f"camera STIP: {camera}=invalid-input")
            return
        if current_host == new_ip:
            self._log(f"[카메라] {camera_name} STIP 건너뜀: 현재 host와 새 IP가 같습니다 ({new_ip}).")
            self.camera_connection_label.setText(f"camera STIP: {camera}=unchanged")
            return
        port = self._control_port_for_camera(camera)
        reboot_text = " and reboot" if self.stip_reboot_checkbox.isChecked() else ""
        answer = QtWidgets.QMessageBox.warning(
            self,
            "Apply WIRIS STIP",
            (
                f"{camera_name} WIRIS at {current_host}:{port} will be changed to {new_ip} "
                f"with subnet mask {mask}{reboot_text}.\n\n"
                "Use this only when the target WIRIS is uniquely identified, preferably with one camera connected."
            ),
            QtWidgets.QMessageBox.Ok | QtWidgets.QMessageBox.Cancel,
            QtWidgets.QMessageBox.Cancel,
        )
        if answer != QtWidgets.QMessageBox.Ok:
            self._log(f"[카메라] {camera_name} STIP 취소됨.")
            return
        self._ensure_camera_host_ip(f"{camera_name} STIP 적용")
        commands = [
            SET_IP_ADDRESS_TEMPLATE.format(address=new_ip),
            SET_SUBNET_MASK_TEMPLATE.format(mask=mask),
        ]
        if self.stip_reboot_checkbox.isChecked():
            commands.append(REBOOT_COMMAND)
        statuses = []
        for command_text in commands:
            try:
                command = build_wiris_command(
                    host=current_host,
                    port=port,
                    command=command_text,
                    timeout_s=self.zoom_timeout_spin.value(),
                )
                response = send_tcp_line(command)
            except Exception as exc:
                statuses.append(f"{command_text}=failed")
                self._log(f"[카메라] {camera_name} STIP 명령 실패 ({command_text}): {exc}")
                self.camera_connection_label.setText(f"camera STIP: {camera}=failed")
                return
            statuses.append(f"{command_text}={response or 'sent'}")
        host_edit.setText(new_ip)
        new_ip_edit.setText(new_ip)
        self._replace_rtsp_url_host(url_edit, new_ip)
        self._rtsp_widgets_for_camera(camera)[1].setText(new_ip)
        self._log(f"[카메라] {camera_name} STIP 적용: {current_host} -> {new_ip}, {'; '.join(statuses)}")
        self.camera_connection_label.setText(f"camera STIP: {camera}={current_host}->{new_ip}")

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
        if self.auto_wiris_connect_pending:
            self.auto_wiris_connect_pending = False
            if all(status == "frame-ok" for status in next_status.values()):
                self._log("[카메라] WIRIS 자동 연결: 좌/우 RTSP 프레임 확인 완료. 카메라 프리뷰를 연결합니다.")
                self.connect_cameras()
            else:
                self._log("[카메라] WIRIS 자동 연결: RTSP 프레임 확인 실패. 제어 주소, Ethernet Mode, 영상 주소를 확인하세요.")

    def _rtsp_stream_check_failed(self, error: str) -> None:
        self._log(f"[카메라] RTSP 스트림 확인 실패: {error}")
        self.camera_connection_label.setText(f"camera RTSP stream: check failed ({error})")
        self.last_rtsp_check_status = {"left": "check-failed", "right": "check-failed"}
        self.auto_wiris_connect_pending = False
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
        direction = (
            QtWidgets.QBoxLayout.LeftToRight
            if current_width >= 1060
            else QtWidgets.QBoxLayout.TopToBottom
        )
        if self.runtime_header_layout.direction() != direction:
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
        tooltip = (
            f"{summary}\n"
            f"{self.left_runtime_stats_label.toolTip()}\n"
            f"{self.right_runtime_stats_label.toolTip()}\n"
            f"{pair_tooltip}"
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
        if hasattr(self, "clock_sync_status_label"):
            status, detail = self._format_clock_sync_status(now_mono_s=now_s)
            self.clock_sync_status_label.setText(status)
            self.clock_sync_status_label.setToolTip(detail)

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
        if "/fmu/out/" in topic:
            return f"ROS2/uORB ({topic})"
        return topic or "unknown"

    def _distance_mode(self) -> str:
        if not hasattr(self, "distance_mode_combo"):
            return "pixhawk"
        mode = str(self.distance_mode_combo.currentData() or "pixhawk")
        return mode if mode in {"pixhawk", "checkerboard"} else "pixhawk"

    def _measurement_display_mode(self) -> str:
        return "detection"

    def _distance_mode_label(self) -> str:
        if self._distance_mode() == "checkerboard":
            return "체커보드 Stereo 외부 파라미터"
        return "Pixhawk 자세 + GPS/RTK baseline"

    def _triangulation_extrinsics_source(self) -> str:
        return self._distance_mode()

    def _measurement_path_label(self) -> str:
        if self.active_intrinsics is None:
            return "3D ray fallback: 내부 파라미터 대기 중"
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
        if self.last_calibration_result is not None and self.last_calibration_zoom_level == zoom_level:
            return (
                self.last_calibration_result,
                f"active calibration result (zoom {zoom_level:02d})",
                f"active calibration result for zoom {zoom_level:02d}",
                None,
            )
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
        self.last_calibration_zoom_level = zoom_level
        self._active_intrinsics_profile_path = str(path)
        self.apply_intrinsics_to_distance()
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

    def _format_extrinsics_text(self, camera: str, extrinsic: CameraExtrinsicState | None) -> str:
        title = "Left Live Extrinsic" if camera == "left" else "Right Live Extrinsic"
        if extrinsic is None:
            return (
                f"{title}\n"
                "status: waiting\n\n"
                f"source: {self._extrinsics_source_label(None)}\n\n"
                "실시간 카메라 자세를 보려면 ROS2 또는 MAVLink로 픽스호크를 연결하세요."
            )
        age_ms = max(0.0, time.monotonic() - extrinsic.timestamp_s) * 1000.0
        px4_timestamp = "n/a" if extrinsic.px4_timestamp_s is None else f"{extrinsic.px4_timestamp_s:.6f}"
        return (
            f"{title}\n"
            f"status: live\n"
            f"source: {self._extrinsics_source_label(extrinsic)}\n"
            f"namespace/source: {extrinsic.namespace}\n"
            f"topic: {extrinsic.topic}\n"
            f"age: {age_ms:.0f} ms\n"
            f"px4_timestamp: {px4_timestamp}\n\n"
            f"roll:  {extrinsic.roll_deg:.6f} deg\n"
            f"pitch: {extrinsic.pitch_deg:.6f} deg\n"
            f"yaw:   {extrinsic.yaw_deg:.6f} deg\n\n"
            "quaternion q(w,x,y,z):\n"
            + ", ".join(f"{value:.9g}" for value in extrinsic.q)
        )

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
            "same_class_required: false\n"
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
        self._update_camera_connection_status()
        if self.yolo_enabled:
            return
        self.latest_calibration_preview_images[camera] = image.copy()
        if not self._is_checkerboard_calibration_visible():
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
        if self.yolo_enabled:
            self._clear_calibration_previews("YOLO depth mode")
            return
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
            if self.yolo_enabled:
                self._clear_calibration_previews("YOLO depth mode")
                return
            self.calibration_preview_times["left"] = 0.0
            self.calibration_preview_times["right"] = 0.0
            self._refresh_all_calibration_previews()

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
            pen = QtGui.QPen(QtGui.QColor(30, 220, 80), 3)
            painter.setPen(pen)
            painter.setBrush(QtGui.QBrush(QtGui.QColor(30, 220, 80)))
            previous = None
            for x, y in corners:
                point = QtCore.QPointF(x, y)
                painter.drawEllipse(point, 5.0, 5.0)
                if previous is not None:
                    painter.drawLine(previous, point)
                previous = point
        else:
            painter.setPen(QtGui.QPen(QtGui.QColor(255, 90, 90), 4))
            painter.drawRect(2, 2, max(0, annotated.width() - 4), max(0, annotated.height() - 4))
        painter.fillRect(0, 0, annotated.width(), 28, QtGui.QColor(0, 0, 0, 170))
        painter.setPen(QtGui.QPen(QtGui.QColor(230, 230, 230), 1))
        painter.drawText(8, 20, status)
        painter.end()
        return annotated

    def _queue_calibration_preview_detection(self, camera: str, image: QtGui.QImage) -> int:
        self._calibration_preview_next_request_id += 1
        request_id = self._calibration_preview_next_request_id
        self.calibration_preview_request_ids[camera] = request_id
        self.calibration_preview_requested.emit(
            {
                "camera": camera,
                "request_id": request_id,
                "image": image.convertToFormat(QtGui.QImage.Format_RGB888).copy(),
                "pattern_size": self._checkerboard_config().pattern_size,
            }
        )
        return request_id

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
        if not self._is_checkerboard_calibration_visible() or self.yolo_enabled:
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

    def toggle_camera_connection(self) -> None:
        if self.camera_connected:
            self.stop_yolo_streams()
        else:
            self.connect_cameras()

    def toggle_yolo_detection(self) -> None:
        if self.yolo_enabled:
            self.stop_yolo_detection()
        else:
            self.start_yolo()

    def toggle_ros2_connection(self, camera: str | None = None) -> None:
        if camera is None:
            if self._ros2_connected_sides():
                self.disconnect_pixhawk()
            else:
                self.connect_pixhawk()
            return
        if camera in self._ros2_connected_sides():
            self.disconnect_pixhawk(camera)
        else:
            self.connect_pixhawk(camera)

    def toggle_mavlink_connection(self, camera: str | None = None) -> None:
        if camera is None:
            if self._mavlink_open_sides():
                self.disconnect_mavlink_pixhawk()
            else:
                self.connect_mavlink_pixhawk()
            return
        if camera in self._mavlink_open_sides():
            self.disconnect_mavlink_pixhawk(camera)
        else:
            self.connect_mavlink_pixhawk(camera)

    def connect_cameras(self) -> None:
        self._start_camera_streams(enable_detection=False, switch_to_depth=False)

    def start_yolo(self) -> None:
        self._start_camera_streams(enable_detection=True, switch_to_depth=True)

    def stop_yolo_detection(self) -> None:
        if not self.camera_connected:
            self._log("[YOLO] 연결된 카메라 스트림이 없어 중지할 YOLO가 없습니다.")
            return
        if not self.yolo_enabled:
            self._log("[YOLO] 이미 중지 상태입니다. 카메라 프리뷰는 유지 중입니다.")
            return
        self._log("[YOLO] 검출을 중지하고 카메라 프리뷰만 유지합니다.")
        self._start_camera_streams(enable_detection=False, switch_to_depth=False)

    def _start_camera_streams(self, *, enable_detection: bool, switch_to_depth: bool) -> None:
        kwargs = self.yolo_kwargs()
        left_url = self.left_url_edit.text().strip()
        right_url = self.right_url_edit.text().strip()
        if not left_url or not right_url:
            self._log("[카메라] Left/Right RTSP URL을 입력해야 합니다.")
            return
        if self._rtsp_urls_are_same_stream():
            message = "카메라: 좌/우 RTSP URL이 같은 스트림입니다. 좌우 카메라 IP를 분리하세요."
            self.camera_connection_label.setText(message)
            self._set_distance_text("거리: 좌/우 RTSP URL이 동일해 측정 차단", quality="BAD")
            self._log(f"[카메라] {message}")
            return
        self._ensure_camera_host_ip("카메라 프리뷰/YOLO 시작")
        self.start_button.setEnabled(False)
        self.camera_connect_button.setEnabled(False)
        if switch_to_depth:
            self.tabs.setCurrentIndex(self.depth_tab_index)
        if enable_detection:
            message = "[YOLO] 좌/우 WIRIS 스트림을 YOLO Depth 측정 모드로 시작합니다."
        else:
            message = "[카메라] 좌/우 WIRIS RTSP 프리뷰 스트림을 시작합니다."
        if self.camera_connected:
            message += " 기존 스트림은 재시작합니다."
        self._log(message)
        self._stop_camera_panes(clear_previews=False)
        self.active_stream_decoders["left"] = None
        self.active_stream_decoders["right"] = None
        self._clear_detection_state()
        if enable_detection:
            self._clear_calibration_previews("YOLO depth mode")

        def start_both() -> None:
            self.left_pane.start_stream(
                left_url,
                enable_detection=enable_detection,
                **kwargs,
                **self._video_recording_kwargs("left"),
            )
            self.right_pane.start_stream(
                right_url,
                enable_detection=enable_detection,
                **kwargs,
                **self._video_recording_kwargs("right"),
            )
            self.camera_connected = True
            self.yolo_enabled = enable_detection
            self.start_button.setEnabled(True)
            self.camera_connect_button.setEnabled(True)
            self._refresh_session_toggle_buttons()
            self._refresh_operation_header()

        start_both()

    def stop_yolo_streams(self) -> None:
        finalize_session = self._session_recording_active()
        self._stop_camera_panes(clear_previews=True)
        if finalize_session:
            self.distance_recording_enabled = False
            self.video_recording_enabled = False
            self.distance_recording_started_monotonic_s = None
            self.record_button.setText("세션 기록 시작")
            self._log(f"[저장] 카메라 Preview 해제로 세션 기록을 중지했습니다: {self._format_video_recording_paths()}")
            self.save_distance_history_csv()
        self._clear_detection_state()
        self.camera_connected = False
        self.yolo_enabled = False
        self.camera_connection_label.setText("카메라: 연결 해제")
        self.start_button.setEnabled(True)
        self.camera_connect_button.setEnabled(True)
        self._refresh_session_toggle_buttons()
        self._refresh_operation_header()

    def _stop_camera_panes(self, *, clear_previews: bool) -> None:
        self.left_pane.stop()
        self.right_pane.stop()
        self.camera_frame_times["left"].clear()
        self.camera_frame_times["right"].clear()
        self.camera_frame_metadata["left"] = {}
        self.camera_frame_metadata["right"] = {}
        self.camera_frame_metadata_history["left"].clear()
        self.camera_frame_metadata_history["right"].clear()
        self.camera_stream_stats["left"] = {}
        self.camera_stream_stats["right"] = {}
        self.active_stream_decoders["left"] = None
        self.active_stream_decoders["right"] = None
        self._update_runtime_camera_stats_labels()
        if not clear_previews:
            return
        self._clear_calibration_previews("left preview", right_text="right preview")

    def _clear_calibration_previews(self, left_text: str, *, right_text: str | None = None) -> None:
        self.latest_calibration_preview_images.clear()
        self.calibration_preview_overlay_images.clear()
        self.calibration_preview_request_ids["left"] = 0
        self.calibration_preview_request_ids["right"] = 0
        if right_text is None:
            right_text = left_text
        for label, text in (
            (self.calibration_left_preview, left_text),
            (self.calibration_right_preview, right_text),
        ):
            label.setPixmap(QtGui.QPixmap())
            label.setText(text)

    def _clear_detection_state(self) -> None:
        self.latest_left = None
        self.latest_right = None
        self.detection_buffers["left"].clear()
        self.detection_buffers["right"].clear()
        self.left_pane.latest_detection = None
        self.right_pane.latest_detection = None
        self._clear_measurement_pair_display()
        self._refresh_distance()

    def stop_all(self) -> None:
        self.stop_yolo_streams()
        self.disconnect_pixhawk()
        self.disconnect_mavlink_pixhawk()
        self._stop_calibration_preview_worker()
        for attr in (
            "camera_auto_detect_thread",
            "pixhawk_auto_detect_thread",
            "rtsp_check_thread",
            "rtsp_enable_thread",
        ):
            thread = getattr(self, attr, None)
            if thread is None:
                continue
            thread.quit()
            thread.wait(1000)
            setattr(self, attr, None)

    def _distance_history_key(self, point: DistanceHistoryPoint) -> tuple[object, ...]:
        return point.sample_key or (point.timestamp_s, point.distance_m, point.method)

    def _refresh_recording_status_label(self) -> None:
        if self._session_save_thread is not None:
            self.recording_status_label.setText(
                f"세션 기록: 저장 중, session={self.recording_session_id or 'none'}"
            )
            return
        state = "진행 중" if self._session_recording_active() else "중지"
        session = f", session={self.recording_session_id}" if self.recording_session_id else ""
        self.recording_status_label.setText(
            "세션 기록: "
            f"{state}, 매칭 프레임쌍={len(self.distance_recording_rows)}{session}"
        )

    def _recording_table_value(self, row: dict[str, object], field: str) -> str:
        value = row.get(field, "")
        if value is None:
            return ""
        return str(value)

    def _recording_preview_rows(self) -> list[dict[str, object]]:
        rows = self.distance_recording_rows
        if isinstance(rows, SpoolRows):
            return rows.preview_rows
        return list(rows[-RECORDING_PREVIEW_LIMIT:])

    def _append_recording_table_row(self, row: dict[str, object]) -> None:
        if not hasattr(self, "recording_sample_table"):
            return
        table = self.recording_sample_table
        if table.rowCount() >= RECORDING_PREVIEW_LIMIT:
            table.removeRow(0)
        row_index = table.rowCount()
        table.insertRow(row_index)
        for column_index, (field, _label) in enumerate(RECORDED_SAMPLE_TABLE_COLUMNS):
            item = QtWidgets.QTableWidgetItem(self._recording_table_value(row, field))
            item.setFlags(QtCore.Qt.ItemIsEnabled | QtCore.Qt.ItemIsSelectable)
            table.setItem(row_index, column_index, item)
        table.scrollToBottom()
        table.selectRow(row_index)

    def _refresh_recording_table(self) -> None:
        if not hasattr(self, "recording_sample_table"):
            return
        table = self.recording_sample_table
        table.setRowCount(0)
        for row in self._recording_preview_rows():
            self._append_recording_table_row(row)
        if not self.distance_recording_rows:
            self.recording_sample_detail.clear()

    def _refresh_recording_sample_detail(self) -> None:
        if not hasattr(self, "recording_sample_table"):
            return
        selected_rows = self.recording_sample_table.selectionModel().selectedRows()
        if not selected_rows:
            self.recording_sample_detail.clear()
            return
        row_index = selected_rows[0].row()
        preview_rows = self._recording_preview_rows()
        if row_index < 0 or row_index >= len(preview_rows):
            self.recording_sample_detail.clear()
            return
        row = preview_rows[row_index]
        lines = [f"{key}: {value}" for key, value in row.items()]
        self.recording_sample_detail.setPlainText("\n".join(lines))

    def toggle_session_recording(self) -> None:
        if self._session_recording_active():
            self._stop_session_recording()
            return
        self._start_session_recording()

    def _start_session_recording(self) -> None:
        if self._session_save_thread is not None:
            self.recording_status_label.setText("세션 기록: 이전 세션 저장 완료 대기 중")
            return
        self.recording_session_id = session_id_now()
        self._session_save_finalized = False
        self._measurement_attempt_generation_id = _new_measurement_attempt_generation_id(
            self.recording_session_id
        )
        monotonic_before_ns = time.monotonic_ns()
        self.recording_session_started_unix_ns = time.time_ns()
        monotonic_after_ns = time.monotonic_ns()
        self.recording_session_started_monotonic_ns = (
            monotonic_before_ns + monotonic_after_ns
        ) // 2
        self.recording_session_anchor_uncertainty_ns = (
            monotonic_after_ns - monotonic_before_ns
        ) // 2
        self.recording_session_started_wall_s = self.recording_session_started_unix_ns * 1e-9
        self.recording_session_started_monotonic_s = self.recording_session_started_monotonic_ns * 1e-9
        self.distance_recording_started_monotonic_s = self.recording_session_started_monotonic_s
        self.distance_recording_rows = SpoolRows(
            preview_limit=RECORDING_PREVIEW_LIMIT,
            supports_updates=True,
        )
        self.distance_recording_keys.clear()
        self.external_parameter_recording_rows.clear()
        spool_root = self._recording_session_dir(self.recording_session_id) / ".spool"
        self.distance_recording_rows.bind(spool_root / "pair_rows.jsonl")
        self.camera_recording_rows = {
            side: SpoolRows() for side in ("left", "right")
        }
        self.px4_recording_rows = {
            side: SpoolRows() for side in ("left", "right")
        }
        for side, rows in self.camera_recording_rows.items():
            rows.bind(spool_root / f"camera_{side}.jsonl")
        for side, rows in self.px4_recording_rows.items():
            rows.bind(spool_root / f"px4_{side}.jsonl")
        self._recording_pair_row_index.clear()
        self._recovered_session_metadata = None
        self.recording_reference_attitudes = {
            camera: AttitudeSnapshot.from_state(state)
            for camera, state in self.latest_extrinsics.items()
            if camera in {"left", "right"}
        }
        self.recording_reference_gps = {
            camera: GpsSnapshot.from_state(state)
            for camera, state in self.latest_gps.items()
            if camera in {"left", "right"}
        }
        self.recording_origin_gps = self.recording_reference_gps.get("left")
        left_reference_gps = self.recording_reference_gps.get("left")
        right_reference_gps = self.recording_reference_gps.get("right")
        self.recording_reference_baseline_ned_m = (
            None
            if left_reference_gps is None or right_reference_gps is None
            else ned_delta_m(left_reference_gps, right_reference_gps)
        )
        self.distance_recording_enabled = True
        self.video_recording_enabled = True
        self._last_distance_history_key = None
        self._measurement_attempt_counter = 0
        self._last_measurement_attempt_key = None
        self._last_measurement_attempt = None
        self._prepare_video_recording_paths()
        write_recovery_metadata(
            self._recording_session_dir(self.recording_session_id),
            self._recording_session_metadata(),
        )
        self._refresh_recording_table()
        self.record_button.setText("세션 기록 중지")
        self._log(
            "[저장] 세션 기록을 시작했습니다: "
            f"session={self.recording_session_id}, {self._format_video_recording_paths()}"
        )
        if self.camera_connected:
            self._start_camera_streams(enable_detection=self.yolo_enabled, switch_to_depth=self.yolo_enabled)
        else:
            self._log("[저장] 카메라 스트림을 시작하면 좌/우 영상 녹화가 함께 시작됩니다.")
        self._refresh_recording_status_label()
        self._refresh_operation_header()

    def _stop_session_recording(self) -> None:
        self.distance_recording_enabled = False
        self.video_recording_enabled = False
        self.distance_recording_started_monotonic_s = None
        self.record_button.setText("세션 기록 시작")
        self._log(
            "[저장] 세션 기록을 중지했습니다: "
            f"session={self.recording_session_id or 'none'}, {self._format_video_recording_paths()}"
        )
        if self.camera_connected:
            self._start_camera_streams(enable_detection=self.yolo_enabled, switch_to_depth=self.yolo_enabled)
        self.save_distance_history_csv()
        self._refresh_recording_status_label()
        self._refresh_operation_header()

    def toggle_distance_recording(self) -> None:
        self.toggle_session_recording()

    def toggle_video_recording(self) -> None:
        self.toggle_session_recording()

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
        if self.distance_recording_enabled and self.recording_session_id:
            spool_root = self._recording_session_dir(self.recording_session_id) / ".spool"
            if isinstance(self.distance_recording_rows, SpoolRows):
                self.distance_recording_rows.bind(spool_root / "pair_rows.jsonl")
            for side, rows in self.camera_recording_rows.items():
                rows.bind(spool_root / f"camera_{side}.jsonl")
            for side, rows in self.px4_recording_rows.items():
                rows.bind(spool_root / f"px4_{side}.jsonl")
        self._recording_pair_row_index.clear()
        if not self.distance_recording_enabled:
            self.distance_recording_started_monotonic_s = None
        self._last_distance_history_key = None
        self.distance_plot.clear()
        self._refresh_recording_table()
        self._refresh_recording_status_label()
        self._log("[Depth] 거리 그래프와 미저장 기록을 초기화했습니다.")
        self._refresh_operation_header()

    def save_current_settings_from_ui(self) -> None:
        self._save_current_settings()
        self._log("[저장] 현재 StereoWIRIS 설정을 저장했습니다.")

    def discover_pc_camera_ip(self) -> None:
        self._log("[연결] PC IP 탐색: 카메라망 인터페이스와 PC IP를 함께 탐색합니다.")
        candidates = discover_host_interface_candidates(
            host_address=self.host_address_edit.text().strip() or DEFAULT_HOST_ADDRESS,
            include_unconfigured=True,
        )
        if not candidates:
            self._log("[연결] PC IP 탐색: 사용할 수 있는 유선 인터페이스를 찾지 못했습니다.")
            self.camera_connection_label.setText("camera host IP: interface not found")
            self._refresh_operation_header()
            return
        self._apply_host_interface_candidates(candidates)
        interface_name = self.interface_edit.text().strip()
        resolved_address = resolve_nm_interface_address(interface_name, self._camera_host_hints())
        if resolved_address is None:
            connection_name = nm_device_connection_name(interface_name)
            if connection_name is None:
                self._log(f"[연결] PC IP 탐색: {interface_name}에 활성 NetworkManager 연결이 없습니다.")
            else:
                addresses = nm_connection_ipv4_addresses(connection_name)
                if addresses:
                    address_text = ", ".join(str(address) for address in addresses)
                    self._log(f"[연결] PC IP 탐색: 활성 프로필 {connection_name} 주소={address_text}")
                else:
                    self._log(f"[연결] PC IP 탐색: 활성 프로필 {connection_name}에서 주소를 찾지 못했습니다.")
            self.camera_connection_label.setText("camera host IP: not found")
            self._refresh_operation_header()
            return
        resolved_text = str(resolved_address)
        self.host_address_edit.setText(resolved_text)
        self._log(
            f"[연결] PC IP 탐색: NetworkManager 활성 프로필에서 카메라망 IP를 찾았습니다: "
            f"{interface_name} {resolved_text}"
        )
        self.camera_connection_label.setText(f"camera host IP: found {resolved_text}")
        self._refresh_operation_header()

    def discover_pc_pixhawk_ip(self) -> None:
        interfaces = local_ipv4_interfaces(include_wireless=True)
        if not interfaces:
            self._log("[픽스호크] PC IP 탐색: 활성 IPv4 인터페이스를 찾지 못했습니다.")
            self.pixhawk_connection_label.setText("pixhawk host IP: not found")
            self._refresh_operation_header()
            return
        self._append_pixhawk_host_ip_candidates(interfaces)
        self.pixhawk_connection_label.setText("pixhawk host IP: 후보 표시됨 - 표에서 PC_IP를 선택해 적용하세요")
        self._refresh_operation_header()

    def _append_pixhawk_host_ip_candidates(self, interfaces: Sequence[object]) -> None:
        hints: list[IPv4Address] = []
        for text in (
            self.left_pixhawk_eth_ip_edit.text().strip(),
            self.right_pixhawk_eth_ip_edit.text().strip(),
            *self._active_mavlink_remote_hosts(),
        ):
            if not text:
                continue
            try:
                hints.append(IPv4Address(str(text).split("/", 1)[0]))
            except ValueError:
                continue
        rows: list[dict[str, object]] = []
        seen: set[tuple[str, str]] = set()
        for candidate in interfaces:
            ip_text = str(candidate.address.ip)
            interface_name = str(getattr(candidate, "name", ""))
            key = (interface_name, ip_text)
            if key in seen:
                continue
            seen.add(key)
            matching_hints = [str(hint) for hint in hints if hint in candidate.address.network]
            status_parts = [str(candidate.address.network)]
            if matching_hints:
                status_parts.append("Pixhawk subnet match: " + ", ".join(matching_hints))
            if interface_name.startswith(("wl", "wlan")):
                status_parts.append("Wi-Fi")
            else:
                status_parts.append("wired/other")
            rows.append(
                {
                    "kind": "PC_IP",
                    "value": ip_text,
                    "baud": "",
                    "status": " | ".join(status_parts),
                    "interface": interface_name,
                }
            )
        if rows:
            self._append_pixhawk_candidate_rows(rows, source="pc")
            for row in rows:
                self._log(
                    f"[픽스호크] PC IP 후보: {row['interface']} {row['value']} "
                    f"({row['status']})"
                )

    def apply_pc_camera_ip(self) -> None:
        interface_name = self.interface_edit.text().strip()
        address_text = self.host_address_edit.text().strip()
        self._log(f"[연결] PC IP 적용 시도: {interface_name} {address_text}")
        ok, message = apply_interface_address(interface_name, address_text)
        if ok:
            self._log(f"[연결] PC IP 적용 완료: {message}")
            self._log("[연결] 네트워크 인터페이스 상태:\n" + interface_addresses(interface_name))
            self._save_current_settings()
            self.camera_connection_label.setText(f"camera host IP: applied {address_text}")
        else:
            self._log(f"[연결] PC IP 적용 실패: {message}")
            self.camera_connection_label.setText("camera host IP: apply failed")
        self._refresh_operation_header()

    def save_distance_history_csv(self) -> None:
        if self.distance_recording_enabled:
            self.recording_status_label.setText("세션 기록: 기록을 중지한 뒤 저장할 수 있습니다")
            self._refresh_operation_header()
            return
        if self._session_save_thread is not None:
            self.recording_status_label.setText("세션 기록: 저장 작업이 이미 진행 중입니다")
            self._refresh_operation_header()
            return
        if self._session_save_finalized:
            self.recording_status_label.setText("세션 기록: 현재 세션 파일은 이미 확정되었습니다")
            return
        if not self.recording_session_id and not self.distance_recording_rows:
            self.recording_status_label.setText("세션 기록: 저장할 샘플 없음")
            self._refresh_operation_header()
            return
        stamp = self.recording_session_id or session_id_now()
        output_dir = self._recording_session_dir(stamp)
        for rows in (self.distance_recording_rows, *self.camera_recording_rows.values(), *self.px4_recording_rows.values()):
            if isinstance(rows, SpoolRows):
                rows.close()
        if self._recovered_session_metadata is None:
            metadata = self._recording_session_metadata()
            write_recovery_metadata(output_dir, metadata)
            self._recovered_session_metadata = json.loads(json.dumps(metadata))
        else:
            metadata = json.loads(json.dumps(self._recovered_session_metadata))
            metadata.update(
                {
                    "matched_frame_pair_count": len(self.distance_recording_rows),
                    "camera_left_gui_frame_count": len(self.camera_recording_rows["left"]),
                    "camera_right_gui_frame_count": len(self.camera_recording_rows["right"]),
                    "px4_left_sample_count": len(self.px4_recording_rows["left"]),
                    "px4_right_sample_count": len(self.px4_recording_rows["right"]),
                    "finalization": {
                        "requested_unix_ns": time.time_ns(),
                        "recovered_retry": True,
                        "provenance_source": ".spool/session_metadata.json",
                    },
                }
            )
        if isinstance(self.distance_recording_rows, SpoolRows):
            pair_rows = self.distance_recording_rows
        else:
            pair_rows = [dict(row) for row in self.distance_recording_rows]
        thread = QtCore.QThread(self)
        worker = SessionSaveWorker(
            output_dir=output_dir,
            metadata=metadata,
            pair_rows=pair_rows,
            camera_rows=dict(self.camera_recording_rows),
            px4_rows=dict(self.px4_recording_rows),
        )
        worker.moveToThread(thread)
        thread.started.connect(worker.run)
        worker.saved.connect(self._handle_session_saved)
        worker.failed.connect(self._handle_session_save_failed)
        worker.finished.connect(thread.quit, QtCore.Qt.DirectConnection)
        worker.finished.connect(worker.deleteLater)
        thread.finished.connect(self._handle_session_save_thread_finished)
        self._session_save_thread = thread
        self._session_save_worker = worker
        self._session_save_retry_required = False
        self.record_button.setEnabled(False)
        self.save_history_button.setEnabled(False)
        self.recording_status_label.setText(
            f"세션 기록: 저장 중, 매칭 프레임쌍={len(pair_rows)}개"
        )
        self._log(f"[저장] 세션 파일을 백그라운드에서 확정 중입니다: {output_dir}")
        thread.start()
        self._refresh_operation_header()

    @QtCore.pyqtSlot(object)
    def _handle_session_saved(self, saved_files: SavedSessionFiles) -> None:
        self._log(f"[저장] 프레임쌍 기준 CSV를 저장했습니다: {saved_files.csv}")
        self._log(
            "[저장] GUI 전달 카메라 프레임 CSV를 저장했습니다: "
            f"left={saved_files.camera_left_csv}, right={saved_files.camera_right_csv}"
        )
        self._log(
            "[저장] PX4 원시 타임라인 CSV를 저장했습니다: "
            f"left={saved_files.px4_left_csv}, right={saved_files.px4_right_csv}"
        )
        self._log(f"[저장] 세션 manifest를 저장했습니다: {saved_files.manifest}")
        if saved_files.finalized:
            self._log(f"[저장] 목적별 탭 XLSX를 저장했습니다: {saved_files.xlsx}")
            output_text = ", ".join(str(output) for output in saved_files.outputs)
            self.recording_status_label.setText(
                "세션 기록 저장 완료: "
                f"매칭 프레임쌍={len(self.distance_recording_rows)}개 - {output_text}"
            )
            self._session_save_retry_required = False
            self._session_save_finalized = True
        else:
            self._session_save_retry_required = True
            self._session_save_finalized = False
            self._log(f"[저장] XLSX 생성 실패; CSV는 확정되었으며 재시도할 수 있습니다: {saved_files.error}")
            self.recording_status_label.setText(
                f"세션 CSV 저장 완료 / XLSX 실패(재시도 가능): {saved_files.error}"
            )
        self._refresh_operation_header()

    @QtCore.pyqtSlot(str)
    def _handle_session_save_failed(self, error: str) -> None:
        self._session_save_retry_required = True
        self._session_save_finalized = False
        self._log(f"[저장] 세션 파일 확정 실패; spool을 보존했습니다: {error}")
        self.recording_status_label.setText(f"세션 저장 실패(재시도 가능): {error}")
        self._refresh_operation_header()

    @QtCore.pyqtSlot()
    def _handle_session_save_thread_finished(self) -> None:
        thread = self._session_save_thread
        self._session_save_thread = None
        self._session_save_worker = None
        if thread is not None:
            thread.deleteLater()
        self.save_history_button.setEnabled(not self._session_save_finalized)
        self.record_button.setEnabled(not self._session_save_retry_required)
        self._refresh_operation_header()

    def _wait_for_session_save(self, timeout_ms: int = 15000) -> bool:
        thread = self._session_save_thread
        if thread is None:
            return True
        completed = thread.wait(timeout_ms)
        if not completed:
            self._log(
                "[저장] 종료 대기 시간이 초과되었습니다. 원본 spool과 manifest를 보존해 재시도할 수 있습니다."
            )
        return completed

    def _record_px4_mag_status_rows(self) -> list[dict[str, object]]:
        if not hasattr(self, "px4_mag_managers"):
            return []
        now_s = time.monotonic()
        rows: list[dict[str, object]] = []
        for camera, label in (("left", "Pixhawk 1"), ("right", "Pixhawk 2")):
            manager = self.px4_mag_managers.get(camera)
            if manager is None:
                continue
            manager.actual_selected_device_id = self.latest_vehicle_mag_device_ids.get(camera)
            status = manager.refresh()
            for slot in status.slots:
                rows.append(
                    {
                        "session_id": self.recording_session_id,
                        "session_started_unix_s": self._fmt_record_float(self.recording_session_started_wall_s, 6),
                        "recording_elapsed_s": self._fmt_record_float(self._recording_elapsed_s(now_s), 6),
                        "recorded_app_timestamp_s": self._fmt_record_float(now_s, 6),
                        "pixhawk_side": camera,
                        "pixhawk_label": label,
                        "cal_mag_slot": f"CAL_MAG{slot.slot}",
                        "device_id": "" if slot.device_id is None else str(slot.device_id),
                        "device_type": slot.device_type,
                        "interface": slot.interface,
                        "is_rm3100": self._fmt_record_bool(slot.is_rm3100),
                        "priority": "" if slot.priority is None else str(slot.priority),
                        "enabled": self._fmt_record_bool(slot.enabled),
                        "expected_selected": self._fmt_record_bool(slot.expected_selected),
                        "actual_selected": self._fmt_record_bool(slot.actual_selected),
                        "expected_selected_device_id": (
                            "" if status.expected_selected_device_id is None else str(status.expected_selected_device_id)
                        ),
                        "actual_selected_device_id": (
                            "" if status.actual_selected_device_id is None else str(status.actual_selected_device_id)
                        ),
                        "is_actual_rm3100": self._fmt_record_bool(status.is_actual_rm3100),
                        "armed": self._fmt_record_bool(status.armed),
                        "status": slot.status,
                        "warning_text": status.warning_text,
                    }
                )
        return rows

    def _recording_session_metadata(self) -> dict[str, object]:
        stamp = self.recording_session_id or time.strftime("%Y%m%d_%H%M%S")
        left_clock_metadata, right_clock_metadata = self._clock_sync_pair_metadata()
        camera_time_metrics = self._camera_time_sync_metrics(
            left_clock_metadata,
            right_clock_metadata,
        )
        triangulation_source = self._triangulation_extrinsics_source()
        _baseline_m, baseline_source, _baseline_warnings = self._active_baseline_m()
        return {
            "session_id": self.recording_session_id,
            "session_dir": str(self._recording_session_dir(stamp)),
            "session_started_unix_s": self.recording_session_started_wall_s,
            "session_started_unix_ns": self.recording_session_started_unix_ns,
            "session_started_monotonic_ns": self.recording_session_started_monotonic_ns,
            "session_time_anchor_uncertainty_ns": self.recording_session_anchor_uncertainty_ns,
            "schema_version": "stereowiris-paired-frame-v2",
            "row_contract": "one row per accepted left/right matched detection frame pair",
            "left_video_path": str(self.video_recording_paths.get("left", "")),
            "right_video_path": str(self.video_recording_paths.get("right", "")),
            "matched_frame_pair_count": len(self.distance_recording_rows),
            "camera_left_gui_frame_count": len(self.camera_recording_rows["left"]),
            "camera_right_gui_frame_count": len(self.camera_recording_rows["right"]),
            "px4_left_sample_count": len(self.px4_recording_rows["left"]),
            "px4_right_sample_count": len(self.px4_recording_rows["right"]),
            "device_timeline_contract": {
                "camera_row": "one row per frame delivered to the GUI frame_changed callback",
                "camera_not_guaranteed": "not a one-to-one index of encoded MP4 frames",
                "px4_row": "one row per PX4 telemetry callback received by the GUI",
                "sample_monotonic": "producer/app receive CLOCK_MONOTONIC timestamp",
                "session_utc": "derived from session host Unix/monotonic anchor",
                "source_utc": "RTCP NTP or GNSS UTC only when explicitly valid",
            },
            "camera_time_sync_status": camera_time_metrics["status"],
            "camera_time_sync_basis": camera_time_metrics["basis"],
            "camera_time_sync_warning": camera_time_metrics["warning"],
            "rtcp_pair_delta_s": camera_time_metrics["pair_delta_s"],
            "rtcp_pc_offset_max_ms": camera_time_metrics["max_pc_offset_ms"],
            "rtcp_frame_transit_max_ms": camera_time_metrics["max_frame_transit_delay_ms"],
            "left_rtcp_frame_transit_ms": camera_time_metrics["left_frame_transit_delay_ms"],
            "right_rtcp_frame_transit_ms": camera_time_metrics["right_frame_transit_delay_ms"],
            "calculation": {
                "triangulation_extrinsics_source": triangulation_source,
                "distance_mode": self._distance_mode_label(),
                "baseline_source": baseline_source,
                "measurement_path": self._measurement_path_label(),
                "method": "two_ray_closest_midpoint",
                "measurement_frame": MEASUREMENT_FRAME,
                "measurement_axis_contract": MEASUREMENT_AXIS_CONTRACT,
                "public_vector_contract": PUBLIC_MEASUREMENT_VECTOR_CONTRACT,
                "legacy_xyz_alias_contract": LEGACY_XYZ_ALIAS_CONTRACT,
                "baseline_frame_scope": "measurement_attempt",
                "baseline_frame_origin_contract": (
                    "midpoint_of_left_optical_origin_and_selected_right_origin_proxy"
                ),
                "baseline_frame_origin_model": (
                    "checkerboard_stereo_rt_optical_centers"
                    if triangulation_source == "checkerboard"
                    else "gps_rtk_antenna_baseline_proxy_no_lever_arm"
                ),
                "baseline_frame_axis_contract": (
                    "right=left_to_right_center; forward=projected_average_optical_forward; "
                    "down=forward_cross_right"
                ),
                "attitude_timing_policy": ATTITUDE_TIMING_POLICY,
                "attitude_timing_blocks_measurement": False,
                "attitude_stale_age_threshold_s": ATTITUDE_STALE_AGE_S,
                "attitude_very_stale_age_threshold_s": ATTITUDE_VERY_STALE_AGE_S,
                "attitude_stale_pair_delta_threshold_s": ATTITUDE_STALE_PAIR_DELTA_S,
                "attitude_very_stale_pair_delta_threshold_s": ATTITUDE_VERY_STALE_PAIR_DELTA_S,
                "mount_assumption_model": (
                    "checkerboard-stereo-extrinsics-v1"
                    if triangulation_source == "checkerboard"
                    else "gps-rtk-baseline-only-v1"
                ),
            },
            "checkerboard_stereo_extrinsics": self._record_current_checkerboard_extrinsics_parameters(),
            "timestamp_contract": {
                "manufacturer_confirmed": False,
                "camera_captured_timestamp_clock": "WIRIS RTC checked by STUS/GDTI",
                "rtp_rtcp_timestamp_clock": "RTP relative media counter; RTCP Sender Report NTP-derived stream time",
                "rtp_timestamp_note": "RTP timestamp is a 90 kHz media clock counter; mapping RTCP NTP to WIRIS GDTI/UTC is unverified.",
                "preferred_frame_time_field": "rtcp_frame_ntp_unix_s",
                "legacy_equivalent_field": "camera_rtc_frame_unix_s",
                "clock_offset_check": "Use the 시간 tab RTCP/NTP check plus direct WIRIS GDTI validation before absolute UTC claims.",
            },
            "intrinsics": self._record_current_intrinsic_parameters(),
            "stream": {
                "left_rtsp_url": self.left_url_edit.text().strip(),
                "right_rtsp_url": self.right_url_edit.text().strip(),
                "backend": self.stream_backend_combo.currentData(),
                "transport": self.rtsp_transport_combo.currentData(),
                "decoder": self.decoder_combo.currentData(),
                "rtsp_latency_ms": self.rtsp_latency_spin.value(),
            },
        }

    def _remove_distance_history_point(self, point: DistanceHistoryPoint) -> None:
        point_key = self._distance_history_key(point)
        filtered_history: deque[DistanceHistoryPoint] = deque(maxlen=self.distance_history.maxlen)
        removed_history = False
        for candidate in self.distance_history:
            if self._distance_history_key(candidate) == point_key:
                removed_history = True
                continue
            filtered_history.append(candidate)
        if not removed_history:
            self._log("[Depth] 거리 그래프 삭제 요청이 현재 기록과 일치하지 않았습니다.")
            return

        self.distance_history = filtered_history
        removed_row = False
        if point_key in self.distance_recording_keys:
            cached_index = self.distance_recording_keys.index(point_key)
            row_index = len(self.distance_recording_rows) - len(self.distance_recording_keys) + cached_index
            self.distance_recording_keys.pop(cached_index)
            if row_index < len(self.distance_recording_rows):
                self.distance_recording_rows.pop(row_index)
                removed_row = True
                base_index = len(self.distance_recording_rows) - len(self.distance_recording_keys)
                self._recording_pair_row_index = {
                    tuple(key): base_index + index
                    for index, key in enumerate(self.distance_recording_keys)
                }
                self._refresh_recording_table()
        self._last_distance_history_key = None
        self._refresh_recording_status_label()
        self._refresh_operation_header()
        message = (
            f"[Depth] 거리 그래프 점을 삭제했습니다: {point.distance_m:.2f} m "
            f"({point.method}, quality={point.quality})"
        )
        if not removed_row:
            message += " [기록 샘플은 찾지 못함]"
        self._log(message)

    def discover_custom_topic_watch(self) -> None:
        self._log("[픽스호크] ROS2 topic 탐색을 시작합니다...")
        topics = discover_ros2_topic_types(self.domain_spin.value(), timeout_s=1.5)
        if not topics:
            self._log("[픽스호크] ROS2 topic을 찾지 못했습니다. ROS2 Connect/uXRCE 상태와 ROS_DOMAIN_ID를 확인하세요.")
            return
        dialog = TopicBrowserDialog(topics, parent=self, title="ROS2 Topic Browser")
        if dialog.exec_() != QtWidgets.QDialog.Accepted:
            return
        selected_topic = dialog.selected_topic()
        if not selected_topic:
            return
        topic_name = selected_topic.get("name", "")
        topic_type = selected_topic.get("type", "")
        if not topic_type:
            self._log(f"[픽스호크] 선택한 topic의 메시지 타입을 알 수 없습니다: {topic_name}")
            return
        try:
            fields = describe_ros2_message_fields(topic_type)
        except Exception as exc:
            self._log(f"[픽스호크] ROS2 field 조회 실패: {topic_type} ({exc})")
            return
        if not fields:
            self._log(f"[픽스호크] 그래프 가능한 numeric field를 찾지 못했습니다: {topic_name}")
            return
        field_dialog = FieldSelectionDialog(topic_name, topic_type, fields, parent=self)
        if field_dialog.exec_() != QtWidgets.QDialog.Accepted:
            return
        selected_fields = field_dialog.selected_fields()
        if not selected_fields:
            self._log("[픽스호크] 선택한 field가 없습니다.")
            return
        for field in selected_fields:
            key = f"ros2:{topic_name}:{field}"
            self._add_custom_topic_watch(key=key, topic=topic_name, field=field, source="ros2")
        self._log(f"[픽스호크] ROS2 Custom watch {len(selected_fields)}개 추가: {topic_name}")

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
            self._add_custom_topic_watch(
                key=key,
                topic=message_name,
                field=field,
                source="mavlink",
                message=message_name,
            )
        self._log(f"[픽스호크] MAVLink Custom watch {len(selected_fields)}개 추가: {message_name}")

    def _add_custom_topic_watch(
        self,
        *,
        key: str,
        topic: str,
        field: str,
        source: str = "ros2",
        message: str | None = None,
    ) -> None:
        spec = {"key": key, "topic": topic, "field": field, "source": source}
        if message:
            spec["message"] = message
        if spec not in self.custom_watch_specs:
            self.custom_watch_specs.append(spec)
        if self.custom_watch_remove_combo.findData(key) < 0:
            self.custom_watch_remove_combo.addItem(key, key)
        if source == "ros2":
            for subscriber in self.pixhawk_ros2_subscribers.values():
                if subscriber is not None:
                    subscriber.add_custom_watch(spec)
        if source == "mavlink" and self.mavlink_pixhawk is not None:
            self.mavlink_pixhawk.add_custom_watch(spec)

    def remove_custom_topic_watch(self, key: str) -> None:
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
        for subscriber in self.pixhawk_ros2_subscribers.values():
            if subscriber is not None:
                subscriber.remove_custom_watch(key)
        if self.mavlink_pixhawk is not None:
            self.mavlink_pixhawk.remove_custom_watch(key)
        self._log(f"[픽스호크] Custom watch 제거: {key}")

    def remove_selected_custom_topic_watch(self) -> None:
        key = self.custom_watch_remove_combo.currentData()
        if key is None:
            self._log("[픽스호크] 제거할 Custom watch가 없습니다.")
            return
        self.remove_custom_topic_watch(str(key))

    def add_custom_topic_watch_from_ui(self) -> None:
        topic = self.custom_watch_topic_edit.text().strip()
        field = self.custom_watch_field_edit.text().strip()
        name = self.custom_watch_name_edit.text().strip() or f"{topic}.{field}"
        if not topic or not field:
            self._log("[픽스호크] Custom watch 추가 실패: topic/field가 비어 있습니다.")
            return
        self._add_custom_topic_watch(key=name, topic=topic, field=field, source="ros2")
        self._log(f"[픽스호크] Custom watch 추가: {name} = {topic}.{field}")

    def _update_custom_watch(self, sample: CustomTopicSample) -> None:
        if sample.numeric_value is not None:
            self.custom_watch_plot.add_point(
                TelemetryHistoryPoint(sample.timestamp_s, sample.key, sample.numeric_value)
            )

    def _ros2_target_sides(self, camera: str | None) -> tuple[str, ...]:
        if camera is None:
            return ("left", "right")
        if camera not in {"left", "right"}:
            raise ValueError(f"Unknown Pixhawk side: {camera}")
        return (camera,)

    def _ros2_connected_sides(self) -> set[str]:
        return {
            side
            for side in ("left", "right")
            if self.pixhawk_ros2_subscribers.get(side) is not None
        }

    def _sync_legacy_pixhawk_ref(self) -> None:
        self.pixhawk = self.pixhawk_ros2_subscribers.get("left") or self.pixhawk_ros2_subscribers.get("right")

    def _create_ros2_pixhawk_subscriber(self, side: str) -> DualPixhawkAttitudeSubscriber:
        kwargs = {
            "domain_id": self.domain_spin.value(),
            "left_namespace": self.left_namespace_edit.text().strip(),
            "right_namespace": self.right_namespace_edit.text().strip(),
            "custom_watches": tuple(self.custom_watch_specs),
            "active_sides": (side,),
        }
        try:
            return DualPixhawkAttitudeSubscriber(**kwargs)
        except TypeError as exc:
            if "active_sides" not in str(exc):
                raise
            kwargs.pop("active_sides")
            return DualPixhawkAttitudeSubscriber(**kwargs)

    def connect_pixhawk(self, camera: str | None = None, *, allow_mavlink_recovery: bool = True) -> bool:
        target_sides = self._ros2_target_sides(camera)
        if all(self.pixhawk_ros2_subscribers.get(side) is not None for side in target_sides):
            self._sync_legacy_pixhawk_ref()
            return True
        ok = True
        failed: list[str] = []
        first_error: Exception | None = None
        self._ensure_micro_xrce_agent()
        for side in target_sides:
            if self.pixhawk_ros2_subscribers.get(side) is not None:
                continue
            label = self._px4_mag_camera_label(side)
            try:
                subscriber = self._create_ros2_pixhawk_subscriber(side)
                subscriber.extrinsic_changed.connect(self._update_extrinsic)
                subscriber.gps_changed.connect(self._update_gps)
                subscriber.mag_sensor_changed.connect(self._update_mag_sensor)
                subscriber.gps_sensor_changed.connect(self._update_gps_sensor)
                subscriber.gnss_relative_changed.connect(self._update_gnss_relative)
                subscriber.custom_watch_changed.connect(self._update_custom_watch)
                subscriber.status_changed.connect(lambda text: self._log(f"[픽스호크] {text}"))
                subscriber.start()
                self.pixhawk_ros2_subscribers[side] = subscriber
                self._log(f"[픽스호크] {label} ROS2 세션을 시작했습니다. 데이터 수신을 기다립니다.")
            except Exception as exc:
                ok = False
                if first_error is None:
                    first_error = exc
                failed.append(label)
                self.pixhawk_ros2_subscribers[side] = None
                self._log(f"[픽스호크] {label} ROS2 연결 실패: {exc}")
                if allow_mavlink_recovery and camera is not None:
                    self._start_mavlink_recovery_after_ros2_failure(exc, camera=side)
        if allow_mavlink_recovery and camera is None and failed and first_error is not None:
            self._start_mavlink_recovery_after_ros2_failure(first_error)
        self._sync_legacy_pixhawk_ref()
        self.pixhawk_button.setEnabled(True)
        self.left_ros2_button.setEnabled(True)
        self.right_ros2_button.setEnabled(True)
        self._refresh_session_toggle_buttons()
        connected = self._ros2_connected_sides()
        if connected:
            labels = ", ".join(self._px4_mag_camera_label(side) for side in sorted(connected))
            self.pixhawk_connection_label.setText(f"픽스호크 ROS2: {labels} 세션 시작, 데이터 대기 중")
            QtCore.QTimer.singleShot(2000, lambda sides=tuple(connected): self._verify_ros2_pixhawk_publishers(sides))
        elif failed and self.mavlink_pixhawk is None:
            self.pixhawk_connection_label.setText(f"pixhawk ROS2: failed ({', '.join(failed)})")
        self.refresh_px4_mag_manager()
        self._refresh_operation_header()
        return ok

    def _ensure_micro_xrce_agent(self) -> None:
        started, message = ensure_micro_xrce_agent(self.uxrce_port_spin.value())
        if started:
            self._log(f"[픽스호크] MicroXRCEAgent를 시작했습니다: {message}")
        else:
            self._log(f"[픽스호크] MicroXRCEAgent가 이미 실행 중입니다: {message}")

    def _verify_ros2_pixhawk_publishers(self, sides: tuple[str, ...] | None = None) -> None:
        connected_sides = self._ros2_connected_sides()
        if not connected_sides:
            return
        target_sides = tuple(side for side in (sides or ("left", "right")) if side in connected_sides)
        if not target_sides:
            return
        domain_id = int(self.domain_spin.value())
        checks = []
        namespaces = {
            "left": self.left_namespace_edit.text().strip().strip("/"),
            "right": self.right_namespace_edit.text().strip().strip("/"),
        }
        for side in target_sides:
            namespace = namespaces[side]
            topic = f"/{namespace}/fmu/out/vehicle_attitude" if namespace else "/fmu/out/vehicle_attitude"
            ok = _ros2_topic_has_publisher(domain_id, topic, timeout_s=0.8)
            checks.append((side, topic, ok))
            sensor_checks = {
                "vehicle_gps_position": (
                    f"/{namespace}/fmu/out/vehicle_gps_position" if namespace else "/fmu/out/vehicle_gps_position",
                ),
                "sensor_gps": sensor_gps_topics(namespace),
                "sensor_mag": sensor_mag_topics(namespace),
                "vehicle_magnetometer": (
                    f"/{namespace}/fmu/out/vehicle_magnetometer" if namespace else "/fmu/out/vehicle_magnetometer",
                ),
            }
            self.ros2_sensor_topic_publishers[side] = {}
            for key, sensor_topics in sensor_checks.items():
                self.ros2_sensor_topic_publishers[side][key] = any(
                    _ros2_topic_has_publisher(domain_id, sensor_topic, timeout_s=0.2)
                    for sensor_topic in sensor_topics
                )
        self._refresh_ros2_only_alignment_availability()
        if any(ok for _side, _topic, ok in checks):
            ok_topics = ", ".join(topic for _side, topic, ok in checks if ok)
            self.pixhawk_connection_label.setText(f"픽스호크 ROS2: publisher 확인됨 ({ok_topics})")
            self._log(f"[픽스호크] ROS2 publisher 확인: {ok_topics}")
            missing = []
            for side, topics in self.ros2_sensor_topic_publishers.items():
                for key in ("sensor_gps", "sensor_mag", "vehicle_magnetometer"):
                    if not topics.get(key, False):
                        missing.append(f"{self._px4_mag_camera_label(side)} {key}")
            if missing:
                self._log("[픽스호크] ROS2 only 센서 미게시: " + ", ".join(missing))
                QtCore.QTimer.singleShot(500, self._start_mavlink_sensor_auxiliary_if_needed)
            return
        topics = ", ".join(topic for _side, topic, _ok in checks)
        self.pixhawk_connection_label.setText("픽스호크 ROS2: topic 이름만 있음, PX4 publisher 없음")
        self._log(
            "[픽스호크] ROS2 publisher 없음: topic 이름은 앱 subscription 때문에 보일 수 있습니다. "
            f"확인 topic={topics}. MicroXRCEAgent, Pixhawk Host IP, PX4 uXRCE client 재부팅/재시작을 확인하세요."
        )

    def _refresh_ros2_only_alignment_availability(self) -> None:
        self._refresh_baseline_gps_cards()
        self._refresh_alignment_magnetometer_ui()
        self._refresh_alignment_overview()
        self._refresh_pixhawk_sensor_health()
        for side in ("left", "right"):
            gps2_label = self.f9p_slot_labels.get(side, {}).get("GPS2") if hasattr(self, "f9p_slot_labels") else None
            if gps2_label is not None and "GPS2" not in self.latest_gps_sensors.get(side, {}) and self._ros2_only_unavailable(side, "GPS2"):
                self._set_health_label(gps2_label, "ROS2 sensor_gps(_0/_1) 미게시", "warn")
            for slot in ("CAN1", "CAN2"):
                mag_label = self.rm3100_slot_labels.get(side, {}).get(slot) if hasattr(self, "rm3100_slot_labels") else None
                if mag_label is not None and slot not in self.latest_mag_sensors.get(side, {}) and self._ros2_only_unavailable(side, slot):
                    self._set_health_label(mag_label, "ROS2 sensor_mag(_0..4)/vehicle_magnetometer 미게시", "warn")

    def _ros2_topic_available(self, side: str, key: str) -> bool:
        return bool(self.ros2_sensor_topic_publishers.get(side, {}).get(key, False))

    def _ros2_only_unavailable(self, side: str, kind: str) -> bool:
        if side not in self._ros2_connected_sides():
            return False
        if self.mavlink_pixhawk is not None and side in self._mavlink_connected_sides():
            return False
        if kind == "GPS2":
            return not self._ros2_topic_available(side, "sensor_gps")
        if kind in {"CAN1", "CAN2", "MAG"}:
            return not (
                self._ros2_topic_available(side, "sensor_mag")
                or self._ros2_topic_available(side, "vehicle_magnetometer")
            )
        return False

    def _mavlink_connected_sides(self) -> set[str]:
        if self.mavlink_pixhawk is None:
            return set()
        connections = getattr(self.mavlink_pixhawk, "connections", {})
        heartbeat_sides = getattr(self.mavlink_pixhawk, "_heartbeat_logged", set())
        try:
            return {side for side in ("left", "right") if side in connections and side in heartbeat_sides}
        except TypeError:
            return set()

    def _mavlink_open_sides(self) -> set[str]:
        if self.mavlink_pixhawk is None:
            return set()
        connections = getattr(self.mavlink_pixhawk, "connections", {})
        try:
            return {side for side in ("left", "right") if side in connections}
        except TypeError:
            return set()

    def _refresh_mavlink_side_actions(self) -> None:
        connected_sides = self._mavlink_connected_sides()
        self.left_uxrce_param_button.setEnabled("left" in connected_sides)
        self.right_uxrce_param_button.setEnabled("right" in connected_sides)
        self.left_px4_shell_button.setEnabled("left" in connected_sides)
        self.right_px4_shell_button.setEnabled("right" in connected_sides)
        self.left_persist_eth_button.setEnabled("left" in connected_sides)
        self.right_persist_eth_button.setEnabled("right" in connected_sides)
        self.left_reboot_pixhawk_button.setEnabled("left" in connected_sides)
        self.right_reboot_pixhawk_button.setEnabled("right" in connected_sides)

    def disconnect_pixhawk(self, camera: str | None = None) -> None:
        target_sides = self._ros2_target_sides(camera)
        for side in target_sides:
            subscriber = self.pixhawk_ros2_subscribers.get(side)
            if subscriber is not None:
                subscriber.stop()
                self.pixhawk_ros2_subscribers[side] = None
            self.pixhawk_attitude_times[side].clear()
            self.ros2_sensor_topic_publishers[side] = {}
            if side not in self._mavlink_connected_sides():
                self._clear_px4_mag_manager_connection_state(side)
        self._sync_legacy_pixhawk_ref()
        self.pixhawk_button.setEnabled(True)
        self.left_ros2_button.setEnabled(True)
        self.right_ros2_button.setEnabled(True)
        self._refresh_session_toggle_buttons()
        if not self._ros2_connected_sides() and self.mavlink_pixhawk is None:
            self.pixhawk_connection_label.setText("픽스호크: 연결 해제")
        label = " / ".join(self._px4_mag_camera_label(side) for side in target_sides)
        self._log(f"[픽스호크] {label} ROS2 연결을 해제했습니다.")
        self._refresh_operation_header()

    def _mavlink_target_sides(self, camera: str | None) -> tuple[str, ...]:
        if camera is None:
            return ("left", "right")
        if camera not in {"left", "right"}:
            raise ValueError(f"Unknown Pixhawk side: {camera}")
        return (camera,)

    def _sync_mavlink_subscriber_settings(self) -> None:
        if self.mavlink_pixhawk is None:
            return
        if hasattr(self.mavlink_pixhawk, "urls"):
            self.mavlink_pixhawk.urls["left"] = self.left_mavlink_url_edit.text().strip()
            self.mavlink_pixhawk.urls["right"] = self.right_mavlink_url_edit.text().strip()
        if hasattr(self.mavlink_pixhawk, "baudrates"):
            self.mavlink_pixhawk.baudrates["left"] = self.left_mavlink_baud_spin.value()
            self.mavlink_pixhawk.baudrates["right"] = self.right_mavlink_baud_spin.value()

    def _ensure_mavlink_subscriber(self) -> DualPixhawkMavlinkSubscriber:
        if self.mavlink_pixhawk is None:
            self.mavlink_pixhawk = DualPixhawkMavlinkSubscriber(
                left_url=self.left_mavlink_url_edit.text().strip(),
                right_url=self.right_mavlink_url_edit.text().strip(),
                left_baud=self.left_mavlink_baud_spin.value(),
                right_baud=self.right_mavlink_baud_spin.value(),
                custom_watches=tuple(self.custom_watch_specs),
            )
            self.mavlink_pixhawk.extrinsic_changed.connect(self._update_extrinsic)
            self.mavlink_pixhawk.gps_changed.connect(self._update_gps)
            self.mavlink_pixhawk.mag_sensor_changed.connect(self._update_mag_sensor)
            self.mavlink_pixhawk.gps_sensor_changed.connect(self._update_gps_sensor)
            self.mavlink_pixhawk.gnss_relative_changed.connect(self._update_gnss_relative)
            self.mavlink_pixhawk.custom_watch_changed.connect(self._update_custom_watch)
            self.mavlink_pixhawk.shell_output.connect(self._append_px4_shell_output)
            self.mavlink_pixhawk.status_changed.connect(self._handle_mavlink_status_changed)
        else:
            self._sync_mavlink_subscriber_settings()
        return self.mavlink_pixhawk

    def _handle_mavlink_status_changed(self, text: str) -> None:
        self._log(f"[픽스호크] {text}")
        self._refresh_session_toggle_buttons()
        self._refresh_mavlink_side_actions()

    def _retry_ros2_after_mavlink_recovery(self, camera: str | None = None) -> None:
        target_sides = self._ros2_target_sides(camera)
        if all(side in self._ros2_connected_sides() for side in target_sides):
            return
        self._log("[픽스호크] MAVLink 복구 설정 이후 ROS2 연결을 다시 시도합니다.")
        if self.connect_pixhawk(camera, allow_mavlink_recovery=False):
            self._log("[픽스호크] MAVLink 복구 설정 이후 ROS2 세션이 시작되었습니다.")
        else:
            self._log("[픽스호크] ROS2 재시도 실패. MAVLink telemetry fallback을 유지합니다.")

    def _start_mavlink_sensor_auxiliary_if_needed(self) -> None:
        if not self._ros2_connected_sides():
            return
        configured_sides = self._configured_mavlink_sides()
        if not configured_sides:
            self._log("[픽스호크] GPS2/RM3100 보조 수신용 MAVLink URL이 없어 ROS2 sensor topic만 사용합니다.")
            return
        missing_sides = [
            side
            for side in configured_sides
            if (
                "GPS2" not in self.latest_gps_sensors.get(side, {})
                or "CAN1" not in self.latest_mag_sensors.get(side, {})
                or "CAN2" not in self.latest_mag_sensors.get(side, {})
            )
        ]
        if not missing_sides:
            return
        missing_sides = self._deduplicate_mavlink_auxiliary_sides(missing_sides)
        if not missing_sides:
            return
        subscriber = self._ensure_mavlink_subscriber()
        started: list[str] = []
        for side in missing_sides:
            if side in self._mavlink_connected_sides():
                continue
            try:
                subscriber.start(side)
            except Exception as exc:
                self._log(f"[픽스호크] {self._px4_mag_camera_label(side)} GPS2/RM3100 MAVLink 보조 수신 시작 실패: {exc}")
                continue
            started.append(side)
        if not started:
            return
        self._refresh_session_toggle_buttons()
        self._refresh_mavlink_side_actions()
        labels = ", ".join(self._px4_mag_camera_label(side) for side in started)
        self._log(
            f"[픽스호크] {labels} GPS2/RM3100 보조 수신을 MAVLink로 시작했습니다. "
            "ROS2 자세/위치 세션은 유지하고 GPS_RAW_INT/GPS2_RAW/SCALED_IMU*만 보조로 수신합니다."
        )
        self.pixhawk_connection_label.setText(f"픽스호크 ROS2 + MAVLink 센서 보조: {labels} 데이터 대기 중")
        self._refresh_operation_header()

    def _start_mavlink_gps2_auxiliary_if_needed(self) -> None:
        self._start_mavlink_sensor_auxiliary_if_needed()

    def _deduplicate_mavlink_auxiliary_sides(self, sides: Sequence[str]) -> list[str]:
        selected: list[str] = []
        by_url: dict[str, list[str]] = {}
        for side in sides:
            url_edit = self.left_mavlink_url_edit if side == "left" else self.right_mavlink_url_edit
            url = url_edit.text().strip()
            if not url:
                continue
            by_url.setdefault(url, []).append(side)
        for url, url_sides in by_url.items():
            if len(url_sides) == 1:
                selected.append(url_sides[0])
                continue
            preferred = max(url_sides, key=self._mavlink_auxiliary_side_score)
            labels = ", ".join(self._px4_mag_camera_label(side) for side in url_sides)
            self._log(
                f"[픽스호크] {labels} MAVLink URL이 동일합니다({url}). "
                f"중복 serial open을 피하려고 {self._px4_mag_camera_label(preferred)} 보조 수신만 시작합니다."
            )
            selected.append(preferred)
        return selected

    def _mavlink_auxiliary_side_score(self, side: str) -> tuple[int, int, int]:
        return (
            1 if side in self._mavlink_connected_sides() else 0,
            1 if self.latest_gps_sensors.get(side) or side in self.latest_gps else 0,
            1 if side in self.latest_extrinsics else 0,
        )

    def _configured_mavlink_sides(self) -> tuple[str, ...]:
        sides = []
        if self.left_mavlink_url_edit.text().strip():
            sides.append("left")
        if self.right_mavlink_url_edit.text().strip():
            sides.append("right")
        return tuple(sides)

    def _mavlink_setup_target_sides(self, target_sides: tuple[str, ...]) -> set[str]:
        configured = set(self._configured_mavlink_sides())
        connected = self._mavlink_connected_sides()
        return {side for side in target_sides if side in configured or side in connected}

    def _apply_mavlink_pixhawk_setup(self, subscriber: DualPixhawkMavlinkSubscriber, target_sides: tuple[str, ...]) -> None:
        setup_sides = self._mavlink_setup_target_sides(target_sides)
        if not setup_sides:
            return
        host_ip = self._validated_pixhawk_host_ip_for_manual_uxrce(sorted(setup_sides)[0])
        if self.mavlink_configure_streams_checkbox.isChecked():
            subscriber.configure_mavlink_streams(
                host_ip=host_ip,
                ports={
                    "left": self._mavlink_stream_port(self.left_mavlink_url_edit.text().strip(), default=14550),
                    "right": self._mavlink_stream_port(self.right_mavlink_url_edit.text().strip(), default=14551),
                },
                sysids={"left": 1, "right": 2},
                rate=self.mavlink_stream_rate_spin.value(),
                cameras=setup_sides,
            )
        if self.mavlink_configure_uxrce_agent_checkbox.isChecked():
            configured_sides = [
                side
                for side in sorted(setup_sides)
                if self._apply_uxrce_params_for_side(subscriber, side, host_ip=host_ip, emit_success=False)
            ]
            if configured_sides and self._ros2_retry_after_uxrce_agent_setup:
                self._ros2_retry_after_uxrce_agent_setup = False
                labels = ", ".join(self._px4_mag_camera_label(side) for side in configured_sides)
                self._log(
                    f"[픽스호크] MAVLink로 {labels} uXRCE 파라미터를 확인했습니다. "
                    "PX4 재부팅/클라이언트 재시작 후 ROS2 연결을 재시도합니다."
                )
                retry_sides = tuple(configured_sides)
                QtCore.QTimer.singleShot(
                    2500,
                    lambda sides=retry_sides: [self._retry_ros2_after_mavlink_recovery(side) for side in sides],
                )

    def _start_mavlink_recovery_after_ros2_failure(self, error: Exception, camera: str | None = None) -> None:
        target_sides = self._mavlink_target_sides(camera)
        configured_sides = tuple(side for side in target_sides if side in self._configured_mavlink_sides())
        if not configured_sides:
            self._log("[픽스호크] ROS2 복구용 MAVLink URL이 없어 MAVLink fallback을 시작하지 못했습니다.")
            return
        self._ros2_retry_after_uxrce_agent_setup = self.mavlink_configure_uxrce_agent_checkbox.isChecked()
        if self._ros2_retry_after_uxrce_agent_setup:
            self._log(
                "[픽스호크] ROS2 실패 후 MAVLink 복구를 시작합니다. "
                "PX4에 MAVLink stream/uXRCE-DDS client 설정을 전송합니다."
            )
        else:
            self._log(
                "[픽스호크] ROS2 실패 후 MAVLink telemetry fallback을 시작합니다. "
                "PX4 uXRCE Agent 설정 옵션이 꺼져 있어 ROS2 자동 재시도는 건너뜁니다."
            )
        try:
            if camera is None:
                self.connect_mavlink_pixhawk(force_setup=True)
            else:
                for side in configured_sides:
                    self.connect_mavlink_pixhawk(side, force_setup=True)
            if not any(side in self._mavlink_connected_sides() for side in configured_sides):
                self._ros2_retry_after_uxrce_agent_setup = False
        except Exception as fallback_exc:
            self._ros2_retry_after_uxrce_agent_setup = False
            self._log(f"[픽스호크] MAVLink 복구/fallback 시작 실패: {fallback_exc} (원래 ROS2 오류: {error})")

    def bootstrap_ethernet_mavlink_from_usb(self, camera: str) -> None:
        if camera not in {"left", "right"}:
            raise ValueError(f"Unknown Pixhawk side: {camera}")
        label = self._px4_mag_camera_label(camera)
        if camera == "left":
            usb_url = self.left_pixhawk_bootstrap_url_edit.text().strip()
            usb_baud = self.left_pixhawk_bootstrap_baud_spin.value()
        else:
            usb_url = self.right_pixhawk_bootstrap_url_edit.text().strip()
            usb_baud = self.right_pixhawk_bootstrap_baud_spin.value()
        if not usb_url:
            self._log(f"[픽스호크] {label} ETH MAVLink USB 부트스트랩 실패: USB 부트스트랩 포트가 비어 있습니다.")
            return
        try:
            command = self._ethernet_mavlink_bootstrap_command(camera, mode="normal")
        except Exception as exc:
            self._log(f"[픽스호크] {label} ETH MAVLink USB 부트스트랩 명령 생성 실패: {exc}")
            return
        baud_candidates = self._usb_bootstrap_baud_candidates(usb_baud)
        last_error = ""
        selected_baud = usb_baud
        for candidate_baud in baud_candidates:
            temp = DualPixhawkMavlinkSubscriber(
                left_url=usb_url if camera == "left" else "",
                right_url=usb_url if camera == "right" else "",
                left_baud=candidate_baud,
                right_baud=candidate_baud,
            )
            temp.status_changed.connect(lambda text: self._log(f"[픽스호크 USB] {text}"))
            try:
                temp.start(camera)
                if not self._temporary_mavlink_heartbeat(temp, camera):
                    last_error = f"{candidate_baud} baud heartbeat timeout"
                    self._log(f"[픽스호크] {label} USB {candidate_baud} baud heartbeat 없음, 다음 후보를 시도합니다.")
                    continue
                selected_baud = candidate_baud
                result = temp.send_shell_command_with_result(camera, command, timeout_s=2.0)
                if "invalid mode" in result.output.lower():
                    fallback_command = self._ethernet_mavlink_bootstrap_command(camera, mode="onboard")
                    self._log(f"[픽스호크] {label} PX4가 MAVLink mode normal을 거부해 onboard로 재시도합니다.")
                    result = temp.send_shell_command_with_result(camera, fallback_command, timeout_s=2.0)
                if result.output.strip():
                    self._log(f"[픽스호크 USB] {label} PX4 shell 응답({result.flag_label}): {result.output.strip()[-800:]}")
                last_error = ""
                break
            except Exception as exc:
                last_error = f"{candidate_baud} baud: {exc}"
                self._log(f"[픽스호크] {label} USB {candidate_baud} baud 부트스트랩 실패: {exc}")
            finally:
                try:
                    temp.stop(camera)
                except Exception:
                    pass
        if last_error:
            self._log(f"[픽스호크] {label} ETH MAVLink USB 부트스트랩 실패: {last_error}")
            return
        self._set_usb_bootstrap_baud(camera, selected_baud)
        self._log(
            f"[픽스호크] {label} USB {selected_baud} baud 부트스트랩으로 Ethernet MAVLink stream 시작 명령을 전송했습니다. "
            "UDP MAVLink 수신을 다시 시작합니다."
        )
        self._save_current_settings()
        self.connect_mavlink_pixhawk(camera)

    def _usb_bootstrap_baud_candidates(self, selected_baud: int) -> tuple[int, ...]:
        ordered = [int(selected_baud)]
        ordered.extend(int(baud) for baud in DEFAULT_USB_MAVLINK_BAUDRATES)
        return tuple(dict.fromkeys(baud for baud in ordered if baud > 0))

    def _set_usb_bootstrap_baud(self, camera: str, baud: int) -> None:
        if camera == "left":
            self.left_pixhawk_bootstrap_baud_spin.setValue(int(baud))
        else:
            self.right_pixhawk_bootstrap_baud_spin.setValue(int(baud))

    def _temporary_mavlink_heartbeat(self, subscriber: DualPixhawkMavlinkSubscriber, camera: str) -> bool:
        connection = getattr(subscriber, "connections", {}).get(camera)
        if connection is None:
            return False
        wait_heartbeat = getattr(connection, "wait_heartbeat", None)
        if wait_heartbeat is None:
            return True
        mav = getattr(connection, "mav", None)
        mavlink = getattr(connection, "mavlink", None)
        if mav is not None and hasattr(mav, "heartbeat_send"):
            try:
                mav.heartbeat_send(
                    getattr(mavlink, "MAV_TYPE_GENERIC", 0),
                    getattr(mavlink, "MAV_AUTOPILOT_INVALID", 8),
                    0,
                    0,
                    0,
                )
            except Exception:
                pass
        try:
            return wait_heartbeat(timeout=1.2) is not None
        except Exception:
            return False

    def _ethernet_mavlink_bootstrap_command(self, camera: str, *, mode: str = "normal") -> str:
        setup = self._persistent_pixhawk_eth_setup(camera)
        mavlink_start_cmd = str(setup["mavlink_start_cmd"])
        if mode == "onboard":
            mavlink_start_cmd = mavlink_start_cmd.replace("-m normal", "-m onboard -n eth0")
        return "\n".join(
            (
                setup["ifup_cmd"],
                setup["ifconfig_cmd"],
                setup["mavlink_stop_cmd"],
                mavlink_start_cmd,
                "mavlink status",
            )
        )

    def connect_mavlink_pixhawk(self, camera: str | None = None, *, force_setup: bool = False) -> None:
        target_sides = self._mavlink_target_sides(camera)
        if all(side in self._mavlink_connected_sides() for side in target_sides):
            if force_setup and self.mavlink_pixhawk is not None:
                self._apply_mavlink_pixhawk_setup(self.mavlink_pixhawk, target_sides)
            return
        try:
            subscriber = self._ensure_mavlink_subscriber()
            subscriber.start(camera)
            if force_setup:
                self._apply_mavlink_pixhawk_setup(subscriber, target_sides)
            elif self.mavlink_configure_streams_checkbox.isChecked() or self.mavlink_configure_uxrce_agent_checkbox.isChecked():
                self._log(
                    "[픽스호크] 일반 MAVLink 연결에서는 UI 응답성을 위해 stream/uXRCE 설정을 자동 실행하지 않습니다. "
                    "필요하면 P1/P2 uXRCE 설정 버튼 또는 ROS2 복구 경로를 사용하세요."
                )
            self._refresh_session_toggle_buttons()
            self._refresh_mavlink_side_actions()
            label = " / ".join(self._px4_mag_camera_label(side) for side in target_sides)
            self.pixhawk_connection_label.setText(f"픽스호크 MAVLink: {label} 세션 시작, 데이터 대기 중")
            self._log(f"[픽스호크] {label} MAVLink 세션을 시작했습니다. 데이터 수신을 기다립니다.")
            self._request_mavlink_eth0_status(target_sides)
            self.refresh_px4_mag_manager()
            self._refresh_operation_header()
        except Exception as exc:
            if self.mavlink_pixhawk is not None:
                try:
                    self.mavlink_pixhawk.stop(camera)
                except Exception:
                    pass
                if not getattr(self.mavlink_pixhawk, "connections", {}):
                    self.mavlink_pixhawk = None
            self._refresh_session_toggle_buttons()
            self._refresh_mavlink_side_actions()
            label = " / ".join(self._px4_mag_camera_label(side) for side in target_sides)
            self.pixhawk_connection_label.setText(f"pixhawk MAVLink {label}: failed ({exc})")
            self._log(f"[픽스호크] {label} MAVLink 연결 실패: {exc}")
            self._refresh_operation_header()

    def disconnect_mavlink_pixhawk(self, camera: str | None = None) -> None:
        target_sides = self._mavlink_target_sides(camera)
        if self.mavlink_pixhawk is not None:
            self.mavlink_pixhawk.stop(camera)
            if not getattr(self.mavlink_pixhawk, "connections", {}):
                self.mavlink_pixhawk = None
        for side in target_sides:
            self.pixhawk_attitude_times[side].clear()
        self._clear_px4_mag_manager_connection_state(camera)
        self._refresh_session_toggle_buttons()
        self._refresh_mavlink_side_actions()
        if not self._ros2_connected_sides():
            if self.mavlink_pixhawk is None:
                self.pixhawk_connection_label.setText("픽스호크: 연결 해제")
            else:
                label = " / ".join(self._px4_mag_camera_label(side) for side in target_sides)
                self.pixhawk_connection_label.setText(f"픽스호크 MAVLink: {label} 연결 해제")
        label = " / ".join(self._px4_mag_camera_label(side) for side in target_sides)
        self._log(f"[픽스호크] {label} MAVLink 연결을 해제했습니다.")
        self._refresh_operation_header()


    def apply_uxrce_params_from_mavlink(self, camera: str) -> None:
        if self.mavlink_pixhawk is None or camera not in self.mavlink_pixhawk.connections:
            self._log(f"[픽스호크] {camera_label_ko(camera)} uXRCE 설정 실패: MAVLink 연결이 없습니다.")
            return
        try:
            host_ip = self._validated_pixhawk_host_ip_for_manual_uxrce(camera)
            ok = self._apply_uxrce_params_for_side(self.mavlink_pixhawk, camera, host_ip=host_ip, emit_success=True)
        except Exception as exc:
            self._log(f"[픽스호크] {camera_label_ko(camera)} uXRCE 설정 실패: {exc}")
            return
        if ok:
            self.pixhawk_connection_label.setText(
                f"{camera_label_ko(camera)} uXRCE 설정 확인 완료: PX4 재부팅/클라이언트 재시작 후 ROS2 연결"
            )

    def _apply_uxrce_params_for_side(
        self,
        subscriber: DualPixhawkMavlinkSubscriber,
        camera: str,
        *,
        host_ip: str,
        emit_success: bool,
    ) -> bool:
        if camera not in subscriber.connections:
            self._log(f"[픽스호크] {camera_label_ko(camera)} uXRCE 설정 건너뜀: MAVLink 연결 없음")
            return False
        dds_key = 1 if camera == "left" else 2
        params = {
            "MAV_2_CONFIG": 0,
            "UXRCE_DDS_CFG": 1000,
            "UXRCE_DDS_KEY": dds_key,
            "UXRCE_DDS_DOM_ID": int(self.domain_spin.value()),
            "UXRCE_DDS_AG_IP": int(IPv4Address(host_ip)),
            "UXRCE_DDS_PRT": int(self.uxrce_port_spin.value()),
            "UXRCE_DDS_PTCFG": 0,
            "UXRCE_DDS_SYNCC": 0,
            "UXRCE_DDS_SYNCT": 1,
        }
        mismatches: list[str] = []
        for name, expected in params.items():
            actual = subscriber.set_param_int(camera, name, expected)
            if actual != expected:
                mismatches.append(f"{name}: expected={expected}, actual={actual}")
        if mismatches:
            self._log(
                f"[픽스호크] {camera_label_ko(camera)} uXRCE 파라미터 readback 불일치: "
                + "; ".join(mismatches)
            )
            return False
        namespace = (self.left_namespace_edit if camera == "left" else self.right_namespace_edit).text().strip()
        message = (
            f"{camera_label_ko(camera)} uXRCE 파라미터 확인: "
            f"agent={host_ip}:{self.uxrce_port_spin.value()}, domain={self.domain_spin.value()}, "
            f"key={dds_key}, namespace={namespace or '<default>'}. "
            "Shell이 아닌 MAVLink PARAM_SET/readback 경로입니다. "
            "Ethernet uXRCE 충돌 방지를 위해 MAV_2_CONFIG=0도 확인합니다."
        )
        self._log(f"[픽스호크] {message}")
        if emit_success:
            self._log(
                f"[픽스호크] {camera_label_ko(camera)} ROS2 topic이 계속 없으면 PX4 재부팅 또는 "
                "uxrce_dds_client 재시작이 필요합니다."
            )
        return True

    def apply_persistent_pixhawk_eth(self, camera: str) -> None:
        if self.mavlink_pixhawk is None or camera not in self.mavlink_pixhawk.connections:
            self._log(f"[픽스호크] {camera_label_ko(camera)} ETH 영구 설정 실패: MAVLink 연결이 없습니다.")
            return
        try:
            command = self._persistent_pixhawk_eth_command(camera)
            self.mavlink_pixhawk.send_shell_command(camera, command)
        except Exception as exc:
            self._log(f"[픽스호크] {camera_label_ko(camera)} ETH shell 즉시 적용 명령 실패: {exc}")
        else:
            self._log(f"[픽스호크] {camera_label_ko(camera)} ETH shell 즉시 적용 명령을 전송했습니다.")
        try:
            extras_text = self._persistent_pixhawk_extras_text(camera)
            self.mavlink_pixhawk.write_remote_file_mavftp(camera, "/fs/microsd/etc/extras.txt", extras_text)
            net_cfg_text = self._persistent_pixhawk_net_cfg_text(camera)
            self.mavlink_pixhawk.write_remote_file_mavftp(camera, "/fs/microsd/net.cfg", net_cfg_text)
        except Exception as exc:
            self._log(
                f"[픽스호크] {camera_label_ko(camera)} ETH MAVLink FTP 영구 저장 실패: {exc}. "
                "PX4 shell과 MAVLink FTP가 모두 막히면 앱만으로 eth0 영구 설정을 저장할 수 없습니다."
            )
            return
        self._log(
            f"[픽스호크] {camera_label_ko(camera)} ETH MAVLink FTP 영구 저장 완료: "
            "/fs/microsd/net.cfg, /fs/microsd/etc/extras.txt. "
            "재부팅 후 eth0, uXRCE-DDS, Ethernet MAVLink stream 적용을 확인하세요."
        )

    def reboot_pixhawk_from_mavlink(self, camera: str) -> None:
        if self.mavlink_pixhawk is None or camera not in self.mavlink_pixhawk.connections:
            self._log(f"[픽스호크] {camera_label_ko(camera)} 재부팅 실패: MAVLink 연결이 없습니다.")
            return
        result = QtWidgets.QMessageBox.question(
            self,
            f"{camera_label_ko(camera)} PX4 재부팅",
            f"{camera_label_ko(camera)} Pixhawk에 MAVLink 재부팅 명령을 보냅니다.\n계속할까요?",
            QtWidgets.QMessageBox.Yes | QtWidgets.QMessageBox.No,
            QtWidgets.QMessageBox.No,
        )
        if result != QtWidgets.QMessageBox.Yes:
            self._log(f"[픽스호크] {camera_label_ko(camera)} MAVLink 재부팅 명령을 취소했습니다.")
            return
        try:
            self.mavlink_pixhawk.reboot_autopilot(camera)
        except Exception as exc:
            self._log(f"[픽스호크] {camera_label_ko(camera)} MAVLink 재부팅 명령 실패: {exc}")
            return
        self._log(
            f"[픽스호크] {camera_label_ko(camera)} MAVLink 재부팅 명령을 전송했습니다. "
            "연결이 끊긴 뒤 부팅 완료 후 다시 MAVLink/ROS2를 연결하세요."
        )

    def _persistent_pixhawk_eth_command(self, camera: str) -> str:
        setup = self._persistent_pixhawk_eth_setup(camera)
        extras = "/fs/microsd/etc/extras.txt"
        extras_lines = setup["extras_lines"]
        return "\n".join(
            (
                "mkdir /fs/microsd/etc",
                setup["ifup_cmd"],
                setup["ifconfig_cmd"],
                *setup["param_lines"],
                "param save",
                setup["uxrce_stop_cmd"],
                setup["uxrce_start_cmd"],
                setup["mavlink_stop_cmd"],
                setup["mavlink_start_cmd"],
                "uxrce_dds_client status",
                "mavlink status",
                f"echo '{extras_lines[0]}' > {extras}",
                *(f"echo '{line}' >> {extras}" for line in extras_lines[1:]),
                f"cat {extras}",
            )
        )

    def _persistent_pixhawk_extras_text(self, camera: str) -> str:
        setup = self._persistent_pixhawk_eth_setup(camera)
        return "\n".join(setup["extras_lines"]) + "\n"

    def _persistent_pixhawk_net_cfg_text(self, camera: str) -> str:
        setup = self._persistent_pixhawk_eth_setup(camera)
        return "\n".join(
            (
                "DEVICE=eth0",
                "BOOTPROTO=static",
                "NETMASK=255.255.255.0",
                f"IPADDR={setup['eth_ip']}",
                f"ROUTER={setup['host_ip']}",
            )
        ) + "\n"

    def _persistent_pixhawk_eth_setup(self, camera: str) -> dict[str, object]:
        if camera == "left":
            eth_ip = self.left_pixhawk_eth_ip_edit.text().strip()
            namespace = self.left_namespace_edit.text().strip().strip("/")
            dds_key = 1
            mavlink_url = self.left_mavlink_url_edit.text().strip()
            mavlink_default_port = 14550
        elif camera == "right":
            eth_ip = self.right_pixhawk_eth_ip_edit.text().strip()
            namespace = self.right_namespace_edit.text().strip().strip("/")
            dds_key = 2
            mavlink_url = self.right_mavlink_url_edit.text().strip()
            mavlink_default_port = 14551
        else:
            raise ValueError(f"Unknown Pixhawk side: {camera}")
        host_ip = self._pixhawk_host_ip_for_uxrce()
        for label, value in (
            ("Pixhawk ETH IP", eth_ip),
            ("Pixhawk Host IP", host_ip),
            ("Pixhawk ROS2 namespace", namespace),
        ):
            if not value or any(ch.isspace() or ch in "'\";`$" for ch in value):
                raise ValueError(f"{label} 값이 비어 있거나 shell 명령에 사용할 수 없는 문자를 포함합니다: {value!r}")
        host_ip_int = int(IPv4Address(host_ip))
        ifup_cmd = "ifup eth0"
        ifconfig_cmd = f"ifconfig eth0 {eth_ip} netmask 255.255.255.0 up"
        uxrce_stop_cmd = "uxrce_dds_client stop"
        uxrce_start_cmd = f"uxrce_dds_client start -t udp -p {self.uxrce_port_spin.value()} -h {host_ip} -n {namespace}"
        mavlink_port = self._mavlink_stream_port(mavlink_url, default=mavlink_default_port)
        mavlink_stop_cmd = f"mavlink stop -u {mavlink_port}"
        mavlink_start_cmd = (
            f"mavlink start -x -u {mavlink_port} -o {mavlink_port} "
            f"-t {host_ip} -r {self.mavlink_stream_rate_spin.value()} -m normal"
        )
        param_lines = [
            "param set MAV_2_CONFIG 0",
            "param set UXRCE_DDS_CFG 1000",
            f"param set UXRCE_DDS_KEY {dds_key}",
            f"param set UXRCE_DDS_DOM_ID {self.domain_spin.value()}",
            f"param set UXRCE_DDS_AG_IP {host_ip_int}",
            f"param set UXRCE_DDS_PRT {self.uxrce_port_spin.value()}",
            "param set UXRCE_DDS_PTCFG 0",
            "param set UXRCE_DDS_SYNCC 0",
            "param set UXRCE_DDS_SYNCT 1",
        ]
        extras_lines = [ifup_cmd, ifconfig_cmd, uxrce_stop_cmd, uxrce_start_cmd, mavlink_stop_cmd, mavlink_start_cmd]
        return {
            "eth_ip": eth_ip,
            "host_ip": host_ip,
            "ifup_cmd": ifup_cmd,
            "ifconfig_cmd": ifconfig_cmd,
            "uxrce_stop_cmd": uxrce_stop_cmd,
            "uxrce_start_cmd": uxrce_start_cmd,
            "mavlink_stop_cmd": mavlink_stop_cmd,
            "mavlink_start_cmd": mavlink_start_cmd,
            "param_lines": param_lines,
            "extras_lines": extras_lines,
        }

    def open_px4_shell(self, camera: str) -> None:
        if self.mavlink_pixhawk is None or camera not in self.mavlink_pixhawk.connections:
            self._log(f"[픽스호크] {camera_label_ko(camera)} PX4 Shell을 열 수 없습니다: MAVLink 연결이 없습니다.")
            return
        dialog = self.px4_shell_dialogs.get(camera)
        if dialog is None:
            dialog = Px4ShellDialog(
                f"{camera_label_ko(camera)} PX4 Shell",
                lambda command, camera=camera: self._send_px4_shell_command(camera, command),
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
        try:
            self.mavlink_pixhawk.send_shell_command(camera, command)
        except Exception as exc:
            self._log(f"[픽스호크] {camera_label_ko(camera)} PX4 Shell 명령 실패: {exc}")
            dialog = self.px4_shell_dialogs.get(camera)
            if dialog is not None:
                dialog.append_output(f"\n[error] {exc}\n")

    def _px4_mag_camera_label(self, camera: str) -> str:
        return "Pixhawk 1" if camera == "left" else "Pixhawk 2"

    def _px4_mag_manager_has_data_source(self, camera: str) -> bool:
        if self.latest_vehicle_mag_device_ids.get(camera) is not None:
            return True
        return camera in self._mavlink_connected_sides()

    def _ros2_raw_mag_slots_seen(self, camera: str) -> list[str]:
        states = self.latest_mag_sensors.get(camera, {})
        return sorted(slot for slot in states if slot != "SELECTED")

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
            if self._ros2_only_unavailable(camera, "MAG"):
                status_label.setText(
                    f"{self._px4_mag_camera_label(camera)} | 픽스호크 MAG 관리: ROS2 MAG topic 미게시\n"
                    "/fmu/out/sensor_mag 또는 /fmu/out/vehicle_magnetometer가 publish되면 실제 MAG device_id가 표시됩니다."
                )
            else:
                status_label.setText(
                    f"{self._px4_mag_camera_label(camera)} | 픽스호크 MAG 관리: 프로토콜 수신 대기 중\n"
                    "물리 연결과 별개로 MAVLink/ROS2에서 실제 MAG device_id를 수신하면 관리 내용이 표시됩니다."
                )
            raw_slots = self._ros2_raw_mag_slots_seen(camera)
            if raw_slots and self.latest_vehicle_mag_device_ids.get(camera) is None:
                status_label.setText(
                    f"{self._px4_mag_camera_label(camera)} | ROS2 raw MAG 수신 중: {', '.join(raw_slots)}\n"
                    "selected MAG /fmu/out/vehicle_magnetometer device_id 수신 대기 중입니다. "
                    "CAL_MAG priority readback은 MAVLink PARAM 연결이 필요합니다."
                )
        self._set_px4_mag_manager_button_state(
            camera,
            refresh_enabled=False,
            actions_enabled=False,
            tooltip="MAVLink/ROS2 MAG 수신 후 사용 가능",
        )

    def _clear_px4_mag_manager_connection_state(self, camera: str | None = None) -> None:
        sides = ("left", "right") if camera is None else (camera,)
        for side in sides:
            self.latest_vehicle_mag_device_ids[side] = None
            self._clear_px4_mag_manager_ui(side)

    def refresh_px4_mag_manager(self, camera: str | None = None) -> None:
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
            status = manager.refresh()
            if using_real_params:
                self._render_px4_mag_status(side, status)
            else:
                self._render_px4_mag_ros2_only_status(side, status)

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
            self._log(f"[PX4 MAG] {label} 수동 Priority 적용 차단: MAVLink/ROS2 MAG 수신 없음")
            self._clear_px4_mag_manager_ui(camera)
            return
        if manager.armed:
            self._log(f"[PX4 MAG] {label} 수동 Priority 적용 차단: Disarm required")
            self.refresh_px4_mag_manager(camera)
            return
        using_mavlink_params = self._sync_px4_mag_manager_store(camera)
        if not using_mavlink_params:
            self._log(
                f"[PX4 MAG] {label} 수동 Priority 적용 차단: "
                "ROS2는 PX4 파라미터 쓰기를 지원하지 않습니다. MAVLink 연결 후 다시 시도하세요."
            )
            self.refresh_px4_mag_manager(camera)
            return
        spins = self.px4_mag_priority_spins.get(camera, {})
        desired = {role: int(spin.value()) for role, spin in spins.items()}
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
            self.refresh_px4_mag_manager(camera)
            return
        self._log(f"[PX4 MAG] {label} 수동 Priority 적용: {', '.join(applied)}")
        if missing:
            self._log(f"[PX4 MAG] {label} 수동 Priority 미적용 role: {', '.join(sorted(missing))}")
        self._render_px4_mag_status(camera, status)

    def save_px4_mag_params_from_panel(self, camera: str = "left") -> None:
        manager = self.px4_mag_managers[camera]
        label = self._px4_mag_camera_label(camera)
        if not self._px4_mag_manager_has_data_source(camera):
            self._log(f"[PX4 MAG] {label} param save 차단: MAVLink/ROS2 MAG 수신 없음")
            self._clear_px4_mag_manager_ui(camera)
            return
        if manager.armed:
            self._log(f"[PX4 MAG] {label} param save 차단: Disarm required")
        else:
            if self.mavlink_pixhawk is not None and camera in self._mavlink_connected_sides():
                try:
                    self.mavlink_pixhawk.send_shell_command(camera, "param save")
                except Exception as exc:
                    self._log(f"[PX4 MAG] {label} param save 명령 실패: {exc}")
                else:
                    self._log(f"[PX4 MAG] {label} MAVLink PX4 shell로 `param save` 명령을 전송했습니다.")
            else:
                self._log(f"[PX4 MAG] {label} QGC 또는 PX4 shell에서 `param save` 후 reboot 필요")
        self.refresh_px4_mag_manager(camera)

    def reboot_px4_from_panel(self, camera: str = "left") -> None:
        manager = self.px4_mag_managers[camera]
        label = self._px4_mag_camera_label(camera)
        if not self._px4_mag_manager_has_data_source(camera):
            self._log(f"[PX4 MAG] {label} reboot 차단: MAVLink/ROS2 MAG 수신 없음")
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
            try:
                self.mavlink_pixhawk.reboot_autopilot(camera)
            except Exception as exc:
                self._log(f"[PX4 MAG] {label} MAVLink 재부팅 명령 실패: {exc}")
            else:
                self._log(f"[PX4 MAG] {label} MAVLink 재부팅 명령을 전송했습니다.")
        else:
            self._log(f"[PX4 MAG] {label} MAVLink 재부팅 명령을 취소했습니다.")
        self.refresh_px4_mag_manager(camera)

    def _render_px4_mag_status(self, camera: str, status) -> None:
        if not hasattr(self, "px4_mag_tables"):
            return
        table = self.px4_mag_tables.get(camera)
        status_label = self.px4_mag_status_labels.get(camera)
        if table is None or status_label is None:
            return
        table.setRowCount(len(status.slots))
        for row, slot in enumerate(status.slots):
            values = [
                f"CAL_MAG{slot.slot}",
                "N/A" if slot.device_id is None else str(slot.device_id),
                slot.device_type,
                slot.interface,
                "true" if slot.is_rm3100 else "false",
                "N/A" if slot.priority is None else str(slot.priority),
                "true" if slot.enabled else "false",
                "true" if slot.expected_selected else "false",
                "true" if slot.actual_selected else "false",
                slot.status,
            ]
            for column, value in enumerate(values):
                item = QtWidgets.QTableWidgetItem(value)
                if slot.is_rm3100:
                    item.setBackground(QtGui.QColor("#123d1f"))
                    item.setForeground(QtGui.QColor("#d8ffd8"))
                if slot.actual_selected:
                    item.setBackground(QtGui.QColor("#1f3d63"))
                    item.setForeground(QtGui.QColor("#e6f2ff"))
                elif slot.expected_selected:
                    item.setBackground(QtGui.QColor("#4b3b00"))
                    item.setForeground(QtGui.QColor("#fff0b3"))
                item.setFlags(QtCore.Qt.ItemIsEnabled | QtCore.Qt.ItemIsSelectable)
                table.setItem(row, column, item)
        table.resizeColumnsToContents()
        table.horizontalHeader().setSectionResizeMode(9, QtWidgets.QHeaderView.Stretch)
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

    def _render_px4_mag_ros2_only_status(self, camera: str, status) -> None:
        if not hasattr(self, "px4_mag_tables"):
            return
        table = self.px4_mag_tables.get(camera)
        status_label = self.px4_mag_status_labels.get(camera)
        if table is not None:
            table.setRowCount(0)
        if status_label is not None:
            actual = status.actual_selected_device_id
            label = self._px4_mag_camera_label(camera)
            raw_slots = self._ros2_raw_mag_slots_seen(camera)
            raw_text = "없음" if not raw_slots else ", ".join(raw_slots)
            status_label.setText(
                f"{label} | Actual selected MAG: {'N/A' if actual is None else actual} (ROS2 vehicle_magnetometer)\n"
                "CAL_MAG slot/priority: N/A - ROS2-only에서는 PX4 PARAM readback이 불가합니다. "
                "실제 CAL_MAG*_PRIO 확인/적용은 MAVLink 연결 또는 QGC/PX4 shell이 필요합니다.\n"
                f"ROS2 raw MAG slots: {raw_text}"
            )
        self._set_px4_mag_manager_button_state(
            camera,
            refresh_enabled=True,
            actions_enabled=False,
            tooltip="CAL_MAG priority readback/apply requires MAVLink PARAM connection",
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
            try:
                self.mavlink_pixhawk.send_shell_command(camera, "ifconfig eth0")
            except Exception as exc:
                self._eth0_probe_tokens.pop(camera, None)
                self._eth0_probe_buffers.pop(camera, None)
                self._log(f"[픽스호크] {label} 실제 eth0 확인 실패: PX4 shell 명령 전송 실패 ({exc})")
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

    def _validated_pixhawk_host_ip_for_manual_uxrce(self, camera: str) -> str:
        text = self.pixhawk_host_address_edit.text().strip()
        if "/" in text:
            text = text.split("/", 1)[0]
        if not text:
            raise ValueError("Pixhawk Host IP가 비어 있습니다. PC IP 탐색 후 PC_IP 후보를 명시적으로 적용하세요.")
        try:
            host_ip = IPv4Address(text)
        except ValueError as exc:
            raise ValueError(f"Pixhawk Host IP 형식이 올바르지 않습니다: {text}") from exc
        interfaces = local_ipv4_interfaces(include_wireless=True)
        matching = [candidate for candidate in interfaces if candidate.address.ip == host_ip]
        if not matching:
            available = ", ".join(f"{candidate.name}={candidate.address}" for candidate in interfaces) or "none"
            raise ValueError(
                f"Pixhawk Host IP가 현재 PC 주소가 아닙니다: {text}. 사용 가능한 IPv4={available}"
            )
        interface_name = str(getattr(matching[0], "name", ""))
        if interface_name.startswith(("wl", "wlan")):
            self._log(
                f"[픽스호크] 경고: Pixhawk Host IP {host_ip}는 Wi-Fi 인터페이스({interface_name})입니다. "
                "Pixhawk가 유선 ETH로 연결되어 있으면 ROS2/uXRCE가 붙지 않을 수 있습니다."
            )
        pixhawk_ip_text = (
            self.left_pixhawk_eth_ip_edit.text().strip()
            if camera == "left"
            else self.right_pixhawk_eth_ip_edit.text().strip()
        )
        if pixhawk_ip_text:
            try:
                pixhawk_ip = IPv4Address(pixhawk_ip_text.split("/", 1)[0])
            except ValueError:
                pixhawk_ip = None
            if pixhawk_ip is not None and not any(pixhawk_ip in candidate.address.network for candidate in matching):
                self._log(
                    f"[픽스호크] 경고: {camera_label_ko(camera)} Pixhawk ETH IP {pixhawk_ip}와 "
                    f"Pixhawk Host IP {host_ip}가 같은 PC 인터페이스 subnet에 없습니다."
                )
        return str(host_ip)

    def _pixhawk_host_ip_for_uxrce(self, *, extra_hints: Sequence[str] = ()) -> str:
        text = self.pixhawk_host_address_edit.text().strip()
        if "/" in text:
            text = text.split("/", 1)[0]
        if not text:
            raise ValueError("Pixhawk Host IP가 비어 있어 uXRCE-DDS client를 시작할 수 없습니다.")
        try:
            current_ip = IPv4Address(text)
        except ValueError as exc:
            raise ValueError(f"Pixhawk Host IP 형식이 올바르지 않습니다: {text}") from exc
        interfaces = local_ipv4_interfaces(include_wireless=True)
        local_ips = {candidate.address.ip for candidate in interfaces}
        if current_ip in local_ips and not extra_hints:
            return text
        selected = self._select_pixhawk_host_interface(interfaces, extra_hints=extra_hints)
        if current_ip in local_ips and selected is not None and selected.address.ip == current_ip:
            return text
        if selected is None:
            available = ", ".join(f"{candidate.name}={candidate.address}" for candidate in interfaces) or "none"
            raise ValueError(
                f"Pixhawk Host IP가 현재 PC 주소가 아닙니다: {text}. 사용 가능한 IPv4={available}"
            )
        host_ip = str(selected.address.ip)
        self.pixhawk_host_address_edit.setText(host_ip)
        self._log(
            f"[픽스호크] Host IP 자동 보정: {text} -> {host_ip} "
            f"({selected.name}, {selected.address.network})"
        )
        return host_ip

    def _select_pixhawk_host_interface(
        self,
        interfaces: Sequence[object],
        *,
        extra_hints: Sequence[str] = (),
    ) -> object | None:
        hints: list[IPv4Address] = []
        for text in extra_hints:
            try:
                hints.append(IPv4Address(str(text).split("/", 1)[0]))
            except ValueError:
                continue
        for edit in (self.left_pixhawk_eth_ip_edit, self.right_pixhawk_eth_ip_edit):
            text = edit.text().strip()
            if not text:
                continue
            try:
                hints.append(IPv4Address(text.split("/", 1)[0]))
            except ValueError:
                continue
        for hint in hints:
            for candidate in interfaces:
                if hint in candidate.address.network:
                    return candidate
        if not interfaces:
            return None
        camera_network = IPv4Address(DEFAULT_HOST_ADDRESS.split("/", 1)[0])
        non_camera = [candidate for candidate in interfaces if camera_network not in candidate.address.network]
        return non_camera[0] if non_camera else interfaces[0]

    def _active_mavlink_remote_hosts(self) -> tuple[str, ...]:
        hosts: list[str] = []
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

    def zoom_command_text(self, action: str) -> str:
        if action == "in":
            return DEFAULT_ZOOM_IN_TEMPLATE
        if action == "out":
            return DEFAULT_ZOOM_OUT_TEMPLATE
        if action == "stop":
            return DEFAULT_ZOOM_STOP_TEMPLATE
        if action == "set":
            return DEFAULT_ZOOM_SET_TEMPLATE
        raise ValueError(f"Unknown zoom action: {action}")

    def selected_zoom_target(self) -> str:
        return str(self.zoom_target_combo.currentData() or "both")

    def zoom_hosts(self, target: str) -> list[tuple[str, str]]:
        if target == "left":
            return [("left", self.left_zoom_host_edit.text())]
        if target == "right":
            return [("right", self.right_zoom_host_edit.text())]
        if target == "both":
            return [("left", self.left_zoom_host_edit.text()), ("right", self.right_zoom_host_edit.text())]
        raise ValueError(f"Unknown zoom target: {target}")

    def _control_port_spin_for_camera(self, camera: str) -> QtWidgets.QSpinBox:
        if camera == "left":
            return self.left_control_port_spin
        if camera == "right":
            return self.right_control_port_spin
        raise ValueError(f"Unknown camera: {camera}")

    def _control_port_for_camera(self, camera: str) -> int:
        return int(self._control_port_spin_for_camera(camera).value())

    def _set_control_port_for_camera(self, camera: str, port: int) -> None:
        if 1 <= int(port) <= 65535:
            self._control_port_spin_for_camera(camera).setValue(int(port))

    def send_zoom(self, target: str, action: str) -> None:
        command_text = self.zoom_command_text(action)
        level = self.zoom_level_spin.value() if action == "set" else None
        statuses = []
        for label, host in self.zoom_hosts(target):
            try:
                command = build_zoom_command(
                    host=host,
                    port=self._control_port_for_camera(label),
                    template=command_text,
                    level=level,
                    timeout_s=self.zoom_timeout_spin.value(),
                )
                response = send_tcp_line(command)
            except Exception as exc:
                statuses.append(f"{self._camera_name_ko(label)}: 실패: {exc}")
                continue
            suffix = f" 응답={response}" if response else " 전송됨"
            statuses.append(f"{self._camera_name_ko(label)}: {command.command}{suffix}")
        status_text = " | ".join(statuses)
        self.zoom_status_label.setText(status_text)
        self._log(f"[카메라] 줌 명령 target={target}, action={action}: {status_text}")

    def _apply_selected_wiris_command(self, *_args) -> None:
        if not hasattr(self, "wiris_command_combo"):
            return
        data = self.wiris_command_combo.currentData()
        if not data:
            return
        command, terminator, description = data
        self.wiris_command_edit.setText(str(command))
        self._set_combo_data(self.wiris_command_terminator_combo, str(terminator))
        self.wiris_command_status_label.setText(str(description))
        self.wiris_command_status_label.setToolTip(str(description))
        self.wiris_command_combo.setToolTip(str(description))
        self.wiris_command_edit.setToolTip(str(description))

    def _append_wiris_command_log(self, text: str) -> None:
        self.wiris_command_log.appendPlainText(text)
        self.wiris_command_log.moveCursor(QtGui.QTextCursor.End)

    def _wiris_command_needs_confirmation(self, command_text: str) -> bool:
        normalized = " ".join(command_text.upper().split())
        return any(normalized.startswith(prefix) for prefix in WIRIS_DANGEROUS_COMMAND_PREFIXES)

    def _stus_command_from_unix_time(self, unix_time_s: float) -> str:
        seconds = int(unix_time_s)
        microseconds = int(round((float(unix_time_s) - seconds) * 1_000_000))
        if microseconds >= 1_000_000:
            seconds += 1
            microseconds -= 1_000_000
        return f"STUS {seconds} {microseconds}"

    def set_selected_wiris_time_from_pc(self, target: str | None = None) -> None:
        if target is None:
            if hasattr(self, "clock_wiris_time_target_combo"):
                target = str(self.clock_wiris_time_target_combo.currentData() or "both")
            else:
                target = str(self.wiris_command_target_combo.currentData() or "both")
        target = str(target or "both")
        target_infos = [
            (label, host, self._control_port_for_camera(label), self.zoom_timeout_spin.value())
            for label, host in self.zoom_hosts(target)
        ]
        if not target_infos:
            status_text = "시간을 설정할 WIRIS 대상이 없습니다."
            self.wiris_command_status_label.setText(status_text)
            if hasattr(self, "clock_sync_status_label"):
                self.clock_sync_status_label.setText(status_text)
            return

        statuses: dict[str, str] = {}
        send_times: dict[str, float] = {}
        tcp_start_times: dict[str, float] = {}
        command_logs: dict[str, str] = {}
        shared_command_time_s: dict[str, float] = {}

        def capture_shared_command_time() -> None:
            shared_command_time_s["unix"] = time.time()

        barrier = (
            threading.Barrier(len(target_infos), action=capture_shared_command_time)
            if len(target_infos) >= 2
            else None
        )

        def send_one(label: str, host: str, port: int, timeout_s: float) -> None:
            camera_name = self._camera_name_ko(label)
            unix_time_s = time.time()
            try:
                if barrier is not None:
                    barrier.wait(timeout=max(1.0, float(timeout_s)))
                    unix_time_s = float(shared_command_time_s.get("unix", time.time()))
                command_text = self._stus_command_from_unix_time(unix_time_s)
                command = build_wiris_command(
                    host=host,
                    port=port,
                    command=command_text,
                    timeout_s=timeout_s,
                    terminator=LINE_TERMINATOR,
                )
                send_times[label] = unix_time_s
                tcp_start_times[label] = time.perf_counter()
                response = send_tcp_line(command)
            except Exception as exc:
                statuses[label] = f"실패: {exc}"
                return
            response_text = response or "<no response>"
            utc_text = datetime.fromtimestamp(unix_time_s, tz=timezone.utc).isoformat(timespec="microseconds")
            statuses[label] = response_text
            command_logs[label] = (
                f"> {camera_name} {command.host}:{command.port} {command.command} "
                f"[pc_utc={utc_text.replace('+00:00', 'Z')}]\n{response_text}\n"
            )

        threads = [
            threading.Thread(
                target=send_one,
                args=(label, host, port, timeout_s),
                name=f"wiris-stus-{label}",
            )
            for label, host, port, timeout_s in target_infos
        ]
        for thread in threads:
            thread.start()
        max_timeout_s = max(max(1.0, float(timeout_s)) for _label, _host, _port, timeout_s in target_infos)
        for thread in threads:
            thread.join(timeout=max_timeout_s + 1.0)

        for label, _host, _port, _timeout_s in target_infos:
            if label not in statuses:
                statuses[label] = "실패: 전송 스레드 timeout"
            if label in command_logs:
                self._append_wiris_command_log(command_logs[label])

        skew_text = ""
        if len(send_times) >= 2:
            ordered_times = [
                (label, send_times[label])
                for label, _host, _port, _timeout_s in target_infos
                if label in send_times
            ]
            first_label, first_time_s = ordered_times[0]
            last_label, last_time_s = ordered_times[-1]
            tcp_skew_text = ""
            ordered_tcp_times = [
                (label, tcp_start_times[label])
                for label, _host, _port, _timeout_s in target_infos
                if label in tcp_start_times
            ]
            if len(ordered_tcp_times) >= 2:
                _first_tcp_label, first_tcp_s = ordered_tcp_times[0]
                _last_tcp_label, last_tcp_s = ordered_tcp_times[-1]
                tcp_skew_text = f", TCP 호출 시작 차이: {(last_tcp_s - first_tcp_s) * 1000.0:.3f} ms"
            skew_text = (
                f" | 명령 UTC 차이 {self._camera_name_ko(first_label)}→"
                f"{self._camera_name_ko(last_label)}: {(last_time_s - first_time_s) * 1000.0:.3f} ms"
                f"{tcp_skew_text}"
            )
        status_text = " | ".join(
            f"{self._camera_name_ko(label)}: {statuses.get(label, '<no response>')}"
            for label, _host, _port, _timeout_s in target_infos
        ) + skew_text
        self.wiris_command_status_label.setText(status_text)
        if hasattr(self, "clock_sync_status_label"):
            self.clock_sync_status_label.setText(f"PC UTC 설정 전송: {status_text}")
            self.clock_sync_summary_text.setPlainText(
                "STUS command sent from PC UTC.\n"
                "Run RTCP/NTP 동기화 확인 after stream frames arrive to verify left/right RTC and PC UTC offsets.\n"
                f"{status_text}"
            )
        self._log(f"[카메라] WIRIS RTC PC UTC 설정 target={target}: {status_text}")

    def send_selected_wiris_command(self) -> None:
        command_text = self.wiris_command_edit.text().strip()
        if not command_text:
            self.wiris_command_status_label.setText("전송할 명령이 비어 있습니다.")
            return
        if "<" in command_text or ">" in command_text or "{" in command_text or "}" in command_text:
            self.wiris_command_status_label.setText("placeholder를 실제 값으로 바꾼 뒤 전송하세요.")
            return
        if self._wiris_command_needs_confirmation(command_text):
            answer = QtWidgets.QMessageBox.warning(
                self,
                "Send WIRIS SDK Command",
                (
                    f"이 명령은 카메라 설정, 네트워크, 녹화, 전원 상태를 바꿀 수 있습니다.\n\n"
                    f"{command_text}\n\n"
                    "대상 카메라가 확실할 때만 전송하세요."
                ),
                QtWidgets.QMessageBox.Ok | QtWidgets.QMessageBox.Cancel,
                QtWidgets.QMessageBox.Cancel,
            )
            if answer != QtWidgets.QMessageBox.Ok:
                self.wiris_command_status_label.setText("명령 전송이 취소되었습니다.")
                return

        target = str(self.wiris_command_target_combo.currentData() or "both")
        terminator = str(self.wiris_command_terminator_combo.currentData() or LINE_TERMINATOR)
        target_infos = [
            (label, host, self._control_port_for_camera(label), self.zoom_timeout_spin.value())
            for label, host in self.zoom_hosts(target)
        ]
        if not target_infos:
            self.wiris_command_status_label.setText("명령을 전송할 WIRIS 대상이 없습니다.")
            return

        statuses: dict[str, str] = {}
        command_logs: dict[str, str] = {}
        send_times: dict[str, float] = {}
        barrier = threading.Barrier(len(target_infos)) if len(target_infos) >= 2 else None

        def send_one(label: str, host: str, port: int, timeout_s: float) -> None:
            camera_name = self._camera_name_ko(label)
            try:
                command = build_wiris_command(
                    host=host,
                    port=port,
                    command=command_text,
                    timeout_s=timeout_s,
                    terminator=terminator,
                )
                if barrier is not None:
                    barrier.wait(timeout=max(1.0, float(timeout_s)))
                send_times[label] = time.perf_counter()
                response = send_tcp_line(command)
            except Exception as exc:
                statuses[label] = f"실패: {exc}"
                return
            response_text = response or "<no response>"
            statuses[label] = response_text
            command_logs[label] = f"> {camera_name} {command.host}:{command.port} {command.command}\n{response_text}\n"

        threads = [
            threading.Thread(
                target=send_one,
                args=(label, host, port, timeout_s),
                name=f"wiris-command-{label}",
            )
            for label, host, port, timeout_s in target_infos
        ]
        for thread in threads:
            thread.start()
        max_timeout_s = max(max(1.0, float(timeout_s)) for _label, _host, _port, timeout_s in target_infos)
        for thread in threads:
            thread.join(timeout=max_timeout_s + 1.0)

        for label, _host, _port, _timeout_s in target_infos:
            if label not in statuses:
                statuses[label] = "실패: 전송 스레드 timeout"
            if label in command_logs:
                self._append_wiris_command_log(command_logs[label])

        skew_text = ""
        if len(send_times) >= 2:
            ordered_times = [
                (label, send_times[label])
                for label, _host, _port, _timeout_s in target_infos
                if label in send_times
            ]
            first_label, first_time_s = ordered_times[0]
            last_label, last_time_s = ordered_times[-1]
            skew_text = (
                f" | 전송 시각 차이 {self._camera_name_ko(first_label)}→"
                f"{self._camera_name_ko(last_label)}: {(last_time_s - first_time_s) * 1000.0:.3f} ms"
            )
        status_text = " | ".join(
            f"{self._camera_name_ko(label)}: {statuses.get(label, '<no response>')}"
            for label, _host, _port, _timeout_s in target_infos
        ) + skew_text
        self.wiris_command_status_label.setText(status_text)
        self._log(f"[카메라] WIRIS SDK 명령 target={target}, command={command_text}: {status_text}")

    def _auto_load_intrinsics(self) -> None:
        if not self.settings.auto_load_intrinsics:
            return
        zoom_level = self.zoom_level_spin.value()
        result = self._load_current_zoom_intrinsics_for_apply()
        if result is None:
            zoom_path = self._intrinsics_storage_path(zoom_level)
            self.calibration_status_label.setText(f"Failed to auto-load intrinsics for zoom {zoom_level:02d}: {zoom_path}")
            self._log(f"[캘리브레이션] intrinsic 자동 로드를 건너뜁니다. 파일 없음: {zoom_path}")
            self._refresh_operation_header()
            self._refresh_zoom_intrinsics_tab()
            return
        try:
            self.last_calibration_result = result
            self.last_calibration_zoom_level = zoom_level
            zoom_path = self._intrinsics_storage_path(zoom_level)
            self._active_intrinsics_profile_path = str(
                zoom_path if zoom_path.exists() else self._intrinsics_storage_base_path()
            )
            self.apply_intrinsics_to_distance()
            self.calibration_status_label.setText(f"Auto-loaded intrinsics for zoom {zoom_level:02d}")
            self._log(f"[캘리브레이션] intrinsic을 자동으로 불러왔습니다 (zoom={zoom_level:02d})")
        except Exception as exc:
            self.calibration_status_label.setText(f"Failed to auto-load intrinsics: {exc}")
            self._log(f"[캘리브레이션] intrinsic 자동 로드 실패: {exc}")
        self._refresh_operation_header()
        self._refresh_zoom_intrinsics_tab()

    def _save_current_settings(self, intrinsics_path: Path | None = None) -> None:
        path = intrinsics_path
        if path is None:
            root, _left_dir, _right_dir = self._calibration_dirs()
            path = root / "intrinsics.json"
        relative = self._app_relative_path_text(path)
        self.settings = AppSettings(
            intrinsics_path=relative,
            yolo_weights_path=self.weights_edit.text().strip() or DEFAULT_YOLO_WEIGHTS,
            interface_name=self.interface_edit.text().strip() or DEFAULT_INTERFACE,
            camera_host_address=self.host_address_edit.text().strip() or DEFAULT_HOST_ADDRESS,
            left_rtsp_url=self.left_url_edit.text().strip() or DEFAULT_CAMERA_URLS[0],
            right_rtsp_url=self.right_url_edit.text().strip() or DEFAULT_CAMERA_URLS[1],
            left_control_host=self.left_zoom_host_edit.text().strip(),
            right_control_host=self.right_zoom_host_edit.text().strip(),
            left_stip_ip=self.left_stip_ip_edit.text().strip(),
            right_stip_ip=self.right_stip_ip_edit.text().strip(),
            stip_subnet_mask=self.stip_mask_edit.text().strip(),
            stip_reboot=self.stip_reboot_checkbox.isChecked(),
            wiris_control_port=self.zoom_port_spin.value(),
            left_control_port=self.left_control_port_spin.value(),
            right_control_port=self.right_control_port_spin.value(),
            wiris_control_timeout_s=self.zoom_timeout_spin.value(),
            yolo_device=self.device_edit.text().strip() or "cpu",
            yolo_input_size=self.input_size_spin.value(),
            yolo_conf_threshold=self.conf_spin.value(),
            yolo_iou_threshold=self.iou_spin.value(),
            yolo_gate_class_enabled=self.yolo_gate_class_checkbox.isChecked(),
            yolo_gate_class_text=self.yolo_gate_class_edit.text().strip(),
            yolo_gate_min_bbox_enabled=self.yolo_gate_min_bbox_checkbox.isChecked(),
            yolo_gate_min_width_px=self.yolo_gate_min_width_spin.value(),
            yolo_gate_min_height_px=self.yolo_gate_min_height_spin.value(),
            yolo_gate_min_area_px=self.yolo_gate_min_area_spin.value(),
            yolo_gate_matching_enabled=self.yolo_gate_matching_checkbox.isChecked(),
            yolo_gate_time_delta_enabled=self.yolo_gate_time_delta_checkbox.isChecked(),
            yolo_gate_vertical_delta_enabled=self.yolo_gate_vertical_delta_checkbox.isChecked(),
            yolo_gate_max_vertical_delta_px=self.yolo_gate_vertical_delta_spin.value(),
            yolo_gate_max_time_delta_s=self.yolo_gate_time_delta_spin.value(),
            yolo_gate_time_basis=str(self.yolo_gate_time_basis_combo.currentData() or "camera_rtc_required"),
            preview_width=self.preview_width_spin.value(),
            rtsp_latency_ms=self.rtsp_latency_spin.value(),
            yolo_event_queue_size=self.yolo_event_queue_size_spin.value(),
            yolo_frame_queue_size=self.yolo_frame_queue_size_spin.value(),
            rtsp_decoder=self.decoder_combo.currentData(),
            stream_backend=self.stream_backend_combo.currentData(),
            rtsp_transport=self.rtsp_transport_combo.currentData(),
            ros_domain_id=self.domain_spin.value(),
            left_namespace=self.left_namespace_edit.text().strip(),
            right_namespace=self.right_namespace_edit.text().strip(),
            pixhawk_host_address=self.pixhawk_host_address_edit.text().strip(),
            left_pixhawk_eth_ip=self.left_pixhawk_eth_ip_edit.text().strip(),
            right_pixhawk_eth_ip=self.right_pixhawk_eth_ip_edit.text().strip(),
            left_mavlink_url=self.left_mavlink_url_edit.text().strip(),
            right_mavlink_url=self.right_mavlink_url_edit.text().strip(),
            left_mavlink_baud=self.left_mavlink_baud_spin.value(),
            right_mavlink_baud=self.right_mavlink_baud_spin.value(),
            left_pixhawk_bootstrap_url=self.left_pixhawk_bootstrap_url_edit.text().strip(),
            right_pixhawk_bootstrap_url=self.right_pixhawk_bootstrap_url_edit.text().strip(),
            left_pixhawk_bootstrap_baud=self.left_pixhawk_bootstrap_baud_spin.value(),
            right_pixhawk_bootstrap_baud=self.right_pixhawk_bootstrap_baud_spin.value(),
            mavlink_configure_streams=self.mavlink_configure_streams_checkbox.isChecked(),
            mavlink_stream_rate=self.mavlink_stream_rate_spin.value(),
            mavlink_configure_uxrce_agent=self.mavlink_configure_uxrce_agent_checkbox.isChecked(),
            uxrce_port=self.uxrce_port_spin.value(),
            auto_load_intrinsics=True,
            manual_focal_override=self.manual_focal_checkbox.isChecked(),
            use_triangulation=True,
            triangulation_extrinsics_source=self._triangulation_extrinsics_source(),
            measurement_display_mode=self._measurement_display_mode(),
            use_rtk_baseline=True,
            require_rtk_fixed=False,
            gnss_base_serial_port=self.gnss_base_serial_port_edit.text().strip() or "/dev/ttyACM0",
            gnss_base_baud=self.gnss_base_baud_spin.value(),
            gnss_base_survey_in_min_duration_s=self.gnss_base_survey_duration_spin.value(),
            gnss_base_survey_in_accuracy_m=self.gnss_base_survey_accuracy_spin.value(),
            gnss_base_rtcm_injection_target=str(self.gnss_base_rtcm_target_combo.currentData() or "disabled"),
            clock_ftp_user=self.clock_ftp_user_edit.text().strip() or "anonymous",
            clock_ftp_password=self.clock_ftp_password_edit.text(),
            clock_ftp_port=self.clock_ftp_port_spin.value(),
            clock_ftp_remote_dir=self.clock_ftp_remote_dir_edit.text().strip() or "/",
            clock_download_dir=self._app_relative_path_text(
                resolve_settings_path(
                    self.clock_download_dir_edit.text().strip() or DEFAULT_CLOCK_DOWNLOAD_DIR,
                    base_dir=APP_ROOT,
                )
            ),
            clock_capture_wait_s=self.clock_capture_wait_spin.value(),
        )
        save_settings(self.settings)

    def _app_relative_path_text(self, path: str | Path) -> str:
        resolved = Path(path).expanduser().resolve()
        try:
            return str(resolved.relative_to(APP_ROOT.resolve()))
        except ValueError:
            return str(resolved)

    def _checkerboard_config(self) -> CheckerboardConfig:
        return CheckerboardConfig(
            inner_cols=self.calib_cols_spin.value(),
            inner_rows=self.calib_rows_spin.value(),
            square_size_m=self.calib_square_spin.value() / 1000.0,
        )

    def _calibration_dirs(self) -> tuple[Path, Path, Path]:
        text = self.calib_dir_edit.text().strip()
        if text:
            root = resolve_settings_path(text, base_dir=APP_ROOT)
        else:
            root = resolve_settings_path(DEFAULT_INTRINSICS_PATH, base_dir=APP_ROOT).parent
        left_dir = root / "left"
        right_dir = root / "right"
        return root, left_dir, right_dir

    def _intrinsics_storage_base_path(self) -> Path:
        if hasattr(self, "calib_dir_edit"):
            root, _left_dir, _right_dir = self._calibration_dirs()
            path = root / "intrinsics.json"
        else:
            path = resolve_settings_path(self.settings.intrinsics_path, base_dir=APP_ROOT)
        stem = path.stem
        if "_zoom_" in stem:
            base_stem, zoom_suffix = stem.rsplit("_zoom_", 1)
            if zoom_suffix.isdigit():
                return path.with_name(f"{base_stem}{path.suffix}")
        return path

    def _intrinsics_storage_path(self, zoom_level: int | None = None) -> Path:
        base_path = self._intrinsics_storage_base_path()
        level = self.zoom_level_spin.value() if zoom_level is None else int(zoom_level)
        return base_path.with_name(f"{base_path.stem}_zoom_{level:02d}{base_path.suffix}")

    def _calibration_image_paths(self, side: str) -> list[Path]:
        _root, left_dir, right_dir = self._calibration_dirs()
        directory = left_dir if side == "left" else right_dir
        return sorted(directory.glob(f"{side}_*.png"))

    def _next_calibration_capture_stem(self) -> str:
        max_index = self.calibration_capture_index
        for side in ("left", "right"):
            for path in self._calibration_image_paths(side):
                suffix = path.stem.replace(f"{side}_", "", 1)
                if suffix.isdigit():
                    max_index = max(max_index, int(suffix))
        self.calibration_capture_index = max_index + 1
        return f"{self.calibration_capture_index:03d}"

    def _checkerboard_capture_pair_timing(self) -> tuple[bool, str, float | None, str]:
        left_metadata = self.camera_frame_metadata.get("left") or {}
        right_metadata = self.camera_frame_metadata.get("right") or {}
        left_timestamp = self._camera_rtc_timestamp_s_from_metadata(left_metadata)
        right_timestamp = self._camera_rtc_timestamp_s_from_metadata(right_metadata)
        if left_timestamp is None or right_timestamp is None:
            return (
                False,
                "rtcp-ntp",
                None,
                "missing RTCP/NTP camera timestamp for left or right frame",
            )
        delta_s = float(right_timestamp) - float(left_timestamp)
        if abs(delta_s) > CHECKERBOARD_STEREO_PAIR_MAX_DELTA_S:
            return (
                False,
                "rtcp-ntp",
                delta_s,
                f"RTCP/NTP pair delta {delta_s * 1000.0:.1f} ms exceeds "
                f"{CHECKERBOARD_STEREO_PAIR_MAX_DELTA_S * 1000.0:.1f} ms",
            )
        return True, "rtcp-ntp", delta_s, "ok"

    def capture_calibration_images(self) -> None:
        frames = {
            "left": self.left_pane.current_image(),
            "right": self.right_pane.current_image(),
        }
        if frames["left"] is None and frames["right"] is None:
            self.calibration_status_label.setText("Start at least one camera stream before capturing calibration images.")
            self._log("[캘리브레이션] 이미지 캡처 실패: 좌/우 카메라 프레임 중 하나 이상이 필요합니다.")
            return
        root, left_dir, right_dir = self._calibration_dirs()
        left_dir.mkdir(parents=True, exist_ok=True)
        right_dir.mkdir(parents=True, exist_ok=True)
        stem = self._next_calibration_capture_stem()
        available, error = opencv_available()
        config = self._checkerboard_config() if available else None
        directories = {"left": left_dir, "right": right_dir}
        saved_sides: list[str] = []
        status_parts: list[str] = []
        pair_candidate = frames["left"] is not None and frames["right"] is not None
        pair_timing_ok = False
        pair_time_basis = "rtcp-ntp"
        pair_delta_s: float | None = None
        pair_timing_detail = "single-camera capture"
        if pair_candidate:
            pair_timing_ok, pair_time_basis, pair_delta_s, pair_timing_detail = self._checkerboard_capture_pair_timing()
            if not pair_timing_ok:
                status_parts.append(f"pair=rejected ({pair_timing_detail})")
        for side in ("left", "right"):
            image = frames[side]
            if image is None:
                status_parts.append(f"{side}=no frame")
                continue
            if pair_candidate and not pair_timing_ok:
                other_side = "right" if side == "left" else "left"
                other_image = frames[other_side]
                if other_image is not None:
                    status_parts.append(f"{side}=not saved as stereo pair ({pair_timing_detail})")
                    continue
            path = directories[side] / f"{side}_{stem}.png"
            if not image.save(str(path), "PNG"):
                status_parts.append(f"{side}=save FAIL")
                continue
            if not available:
                saved_sides.append(side)
                opencv_detail = f": {error}" if error else ""
                status_parts.append(f"{side}=saved (OpenCV unavailable{opencv_detail})")
                continue
            assert config is not None
            detection = detect_checkerboard(path, config)
            if detection.found:
                saved_sides.append(side)
                status_parts.append(f"{side}=OK saved ({detection.corner_count})")
            else:
                path.unlink(missing_ok=True)
                detail = f"error: {detection.error}" if detection.error else f"{detection.corner_count}"
                status_parts.append(f"{side}=not found ({detail})")

        if not saved_sides:
            status = f"Capture {stem}: no calibration images saved; " + ", ".join(status_parts)
            self.calibration_status_label.setText(status)
            self._log(f"[캘리브레이션] 이미지 캡처 실패 {stem}: {', '.join(status_parts)}")
            return

        metadata_dir = root / "capture_metadata"
        metadata_dir.mkdir(parents=True, exist_ok=True)
        metadata_payload = {
            "capture_id": stem,
            "saved_sides": saved_sides,
            "stereo_pair_saved": set(saved_sides) == {"left", "right"},
            "pair_time_basis": pair_time_basis,
            "pair_delta_s": pair_delta_s,
            "pair_max_delta_s": CHECKERBOARD_STEREO_PAIR_MAX_DELTA_S,
            "pair_timing_status": pair_timing_detail,
            "left_rtcp_frame_ntp_unix_s": (self.camera_frame_metadata.get("left") or {}).get("rtcp_frame_ntp_unix_s"),
            "right_rtcp_frame_ntp_unix_s": (self.camera_frame_metadata.get("right") or {}).get("rtcp_frame_ntp_unix_s"),
            "frame_contract": "OpenCV image pixels; stereo timing uses RTCP/NTP camera timestamp before saving a left/right pair",
        }
        (metadata_dir / f"capture_{stem}.json").write_text(
            json.dumps(metadata_payload, indent=2, ensure_ascii=False),
            encoding="utf-8",
        )
        status = f"Captured {stem}: " + ", ".join(status_parts)
        if pair_timing_ok and pair_delta_s is not None and set(saved_sides) == {"left", "right"}:
            status += f"; pair_time_basis={pair_time_basis}, pair_delta_s={pair_delta_s:.6f}"
        self.calibration_status_label.setText(status)
        self._log(f"[캘리브레이션] 이미지 캡처 {stem}: 저장={','.join(saved_sides)}; {', '.join(status_parts)}")

    def run_intrinsic_calibration(self) -> None:
        available, error = opencv_available()
        if not available:
            self.calibration_status_label.setText(f"OpenCV unavailable: {error}")
            self.calibration_result_text.setPlainText(f"OpenCV unavailable:\n{error}")
            return
        config = self._checkerboard_config()
        left_paths = self._calibration_image_paths("left")
        right_paths = self._calibration_image_paths("right")
        min_images = self.calib_min_images_spin.value()
        try:
            left = calibrate_camera_from_images(left_paths, config)
            right = calibrate_camera_from_images(right_paths, config)
        except Exception as exc:
            self.calibration_status_label.setText(f"Calibration failed: {exc}")
            self.calibration_result_text.setPlainText(str(exc))
            self._log(f"[캘리브레이션] intrinsic 계산 실패: {exc}")
            return
        extrinsics = None
        extrinsics_warning = ""
        try:
            extrinsics = calibrate_stereo_extrinsics_from_image_pairs(left_paths, right_paths, config, left, right)
        except Exception as exc:
            extrinsics_warning = f"\n\nStereo extrinsics not computed: {exc}"
            self._log(f"[캘리브레이션] 체커보드 stereo 외부 파라미터 계산 건너뜀: {exc}")
        self.last_calibration_result = StereoIntrinsicsResult(left=left, right=right, board=config, extrinsics=extrinsics)
        self.last_calibration_zoom_level = self.zoom_level_spin.value()
        self._active_intrinsics_profile_path = (
            f"memory:checkerboard-calibration:zoom-{self.last_calibration_zoom_level:02d}"
        )
        warning = ""
        if left.image_count < min_images or right.image_count < min_images:
            warning = f"\nWARNING: valid images below requested minimum {min_images}."
        extrinsics_text = ""
        if extrinsics is not None:
            baseline_m = math.sqrt(sum(value * value for value in extrinsics.translation_left_to_right_m))
            extrinsics_text = (
                "\n\nCheckerboard stereo extrinsics "
                "(OpenCV optical RDF; X_right = R * X_left + T):\n"
                f"pairs={extrinsics.pair_count}, error={extrinsics.reprojection_error_px:.3f}px, "
                f"|T|={baseline_m:.6f} m\n"
                f"T_left_to_right_m={extrinsics.translation_left_to_right_m}\n"
                f"R_left_to_right={extrinsics.rotation_left_to_right}"
            )
        text = (
            f"Left: images={left.image_count}, error={left.reprojection_error_px:.3f}px, "
            f"fx={left.camera_matrix[0][0]:.2f}, fy={left.camera_matrix[1][1]:.2f}, "
            f"cx={left.camera_matrix[0][2]:.2f}, cy={left.camera_matrix[1][2]:.2f}\n"
            f"D_left={left.distortion_coefficients}\n\n"
            f"Right: images={right.image_count}, error={right.reprojection_error_px:.3f}px, "
            f"fx={right.camera_matrix[0][0]:.2f}, fy={right.camera_matrix[1][1]:.2f}, "
            f"cx={right.camera_matrix[0][2]:.2f}, cy={right.camera_matrix[1][2]:.2f}\n"
            f"D_right={right.distortion_coefficients}{warning}{extrinsics_text}{extrinsics_warning}"
        )
        self.calibration_result_text.setPlainText(text)
        status = "Intrinsic + checkerboard stereo extrinsics calibration complete." if extrinsics else "Intrinsic calibration complete."
        self.calibration_status_label.setText(status)
        self._refresh_camera_parameters_tab()
        self._refresh_zoom_intrinsics_tab()
        if extrinsics is not None:
            self._log("[캘리브레이션] intrinsic 및 체커보드 stereo 외부 파라미터 계산을 완료했습니다.")
        else:
            self._log("[캘리브레이션] intrinsic 계산을 완료했습니다.")
        self._refresh_operation_header()

    def save_intrinsics(self) -> None:
        if self.last_calibration_result is None:
            self.calibration_status_label.setText("Run calibration before saving intrinsics.")
            return
        root, _left_dir, _right_dir = self._calibration_dirs()
        root.mkdir(parents=True, exist_ok=True)
        zoom_level = self.zoom_level_spin.value()
        zoom_output = self._intrinsics_storage_path(zoom_level)
        latest_output = self._intrinsics_storage_base_path()
        metadata = {
            "note": "Generated by StereoWIRIS Camera Calibration tab",
            "timestamp_s": time.time(),
            "zoom_level": zoom_level,
            "zoom_target": str(self.zoom_target_combo.currentData() or "both"),
        }
        save_intrinsics_json(
            zoom_output,
            self.last_calibration_result,
            metadata=metadata,
        )
        if zoom_output != latest_output:
            save_intrinsics_json(latest_output, self.last_calibration_result, metadata=metadata)
        self.last_calibration_zoom_level = zoom_level
        self.calibration_status_label.setText(f"Saved intrinsics for zoom {zoom_level:02d}: {zoom_output}")
        self._add_zoom_intrinsics_comparison_path(zoom_output)
        self._save_current_settings()
        self._log(f"[캘리브레이션] intrinsic 파일을 저장했습니다 (zoom={zoom_level:02d}): {zoom_output}")
        self._refresh_operation_header()
        self._refresh_zoom_intrinsics_tab()

    def load_intrinsics(self) -> None:
        zoom_level = self.zoom_level_spin.value()
        result = self._load_current_zoom_intrinsics_for_apply()
        if result is None:
            path = self._intrinsics_storage_path(zoom_level)
            self.calibration_status_label.setText(f"Failed to load intrinsics for zoom {zoom_level:02d}: {path}")
            self._refresh_operation_header()
            self._refresh_zoom_intrinsics_tab()
            return
        self.last_calibration_result = result
        self.last_calibration_zoom_level = zoom_level
        zoom_path = self._intrinsics_storage_path(zoom_level)
        self._active_intrinsics_profile_path = str(
            zoom_path if zoom_path.exists() else self._intrinsics_storage_base_path()
        )
        self.apply_intrinsics_to_distance()
        self._save_current_settings()
        self.calibration_status_label.setText(f"Applied intrinsics for zoom {zoom_level:02d}")
        self._log(f"[캘리브레이션] intrinsic 파일을 적용했습니다 (zoom={zoom_level:02d})")
        self._refresh_zoom_intrinsics_tab()
        self._refresh_operation_header()

    def apply_intrinsics_to_distance(self) -> None:
        if self.last_calibration_result is None:
            self.calibration_status_label.setText("Run or load calibration before applying intrinsics.")
            return
        self.active_intrinsics = self.last_calibration_result
        left_fx = self.active_intrinsics.left.camera_matrix[0][0]
        right_fx = self.active_intrinsics.right.camera_matrix[0][0]
        self.focal_spin.setValue((left_fx + right_fx) / 2.0)
        self.manual_focal_checkbox.setChecked(False)
        self.calibration_status_label.setText("Applied intrinsics and average fx to StereoWIRIS.")
        self._refresh_camera_parameters_tab()
        self._refresh_distance()
        self._refresh_operation_header()

    def closeEvent(self, event) -> None:  # noqa: N802 - Qt override.
        self._save_current_settings()
        self._stop_calibration_preview_worker()
        if self.camera_auto_detect_thread is not None:
            self.camera_auto_detect_thread.quit()
            self.camera_auto_detect_thread.wait(1000)
        if self.pixhawk_auto_detect_thread is not None:
            self.pixhawk_auto_detect_thread.quit()
            self.pixhawk_auto_detect_thread.wait(1000)
        if self.rtsp_check_thread is not None:
            self.rtsp_check_thread.quit()
            self.rtsp_check_thread.wait(1000)
        if self.rtsp_enable_thread is not None:
            self.rtsp_enable_thread.quit()
            self.rtsp_enable_thread.wait(1000)
        if self.gnss_base_detect_thread is not None:
            self.gnss_base_detect_thread.quit()
            self.gnss_base_detect_thread.wait(1000)
        self.disconnect_gnss_base_serial()
        self.stop_all()
        if not self._wait_for_session_save():
            event.ignore()
            QtCore.QTimer.singleShot(500, self.close)
            return
        super().closeEvent(event)

    def _stop_calibration_preview_worker(self, *_args) -> None:
        thread = getattr(self, "_calibration_preview_thread", None)
        if thread is None:
            return
        thread.quit()
        thread.wait(1000)
        self._calibration_preview_thread = None
        self._calibration_preview_worker = None

    def _update_detection(self, camera: str, detection: DetectionObservation | None) -> None:
        if camera == "left":
            self.latest_left = detection
        elif camera == "right":
            self.latest_right = detection
        if detection is not None:
            self.detection_buffers[camera].add(detection)
        self._refresh_distance()

    def _checkerboard_translation_right_in_left_m(self) -> tuple[float, float, float] | None:
        calibration = self.active_intrinsics
        extrinsics = None if calibration is None else calibration.extrinsics
        if extrinsics is None:
            return None
        _rotation_right_to_left, translation_right_in_left_m = invert_left_to_right_camera_transform(
            extrinsics.rotation_left_to_right,
            extrinsics.translation_left_to_right_m,
        )
        return translation_right_in_left_m

    def _active_baseline_m(self) -> tuple[float, str, tuple[str, ...]]:
        if self._distance_mode() == "checkerboard":
            translation = self._checkerboard_translation_right_in_left_m()
            if translation is None:
                return 0.0, "checkerboard-waiting", ("waiting for checkerboard stereo extrinsics",)
            baseline_m = math.sqrt(sum(value * value for value in translation))
            if baseline_m <= 0.0:
                return 0.0, "checkerboard-waiting", ("checkerboard baseline length <= 0",)
            return baseline_m, "checkerboard", ()
        baseline = self.latest_rtk_baseline
        if baseline is None:
            return 0.0, "gps-rtk-waiting", ("waiting for live GPS baseline",)
        usable, reason = self._gps_baseline_usable_for_translation(baseline)
        if not usable:
            return baseline.baseline_m, "gps-rtk-blocked", (*baseline.warnings, reason)
        return baseline.baseline_m, "gps-rtk", baseline.warnings

    def _gps_fix_ready_for_baseline(self, state: GpsPositionState | None) -> bool:
        return state is not None and state.has_position and state.fix_type is not None and state.fix_type >= 3

    def _gps_baseline_usable_for_translation(self, baseline: RtkBaselineEstimate | None) -> tuple[bool, str]:
        if baseline is None:
            return False, "live GPS baseline waiting"
        left, right = self._selected_baseline_gps_states()
        if left is None or right is None:
            if baseline.baseline_m <= 0.0:
                return False, "GPS baseline length <= 0"
            return True, "usable"
        if not self._gps_fix_ready_for_baseline(left) or not self._gps_fix_ready_for_baseline(right):
            return False, "GPS fix < 3D"
        if baseline.baseline_m <= 0.0:
            return False, "GPS baseline length <= 0"
        return True, "usable"

    def _rtk_fixed_status_text(self, baseline: RtkBaselineEstimate | None) -> str:
        left = self.latest_gps.get("left")
        right = self.latest_gps.get("right")

        def side_text(label: str, state: GpsPositionState | None) -> str:
            if state is None:
                return f"{label}=대기"
            sats = "--" if state.satellites_used is None else str(state.satellites_used)
            eph = "--" if state.eph_m is None else f"{state.eph_m:.2f}m"
            return f"{label}={state.rtk_label}(fix={state.fix_type}, sats={sats}, eph={eph})"

        fixed = left is not None and right is not None and left.fix_type == 6 and right.fix_type == 6
        usable, reason = self._gps_baseline_usable_for_translation(baseline)
        fixed_text = "YES" if fixed else "NO"
        usability = "사용 가능" if usable else f"대기/차단: {reason}"
        return f"RTK Fixed: {fixed_text} | GPS baseline 필수, {usability} | {side_text('P1', left)} | {side_text('P2', right)}"

    def _refresh_rtk_baseline(self) -> RtkBaselineEstimate | None:
        left_gps, right_gps = self._selected_baseline_gps_states()
        baseline = estimate_rtk_baseline(
            left_gps,
            right_gps,
            now_s=time.monotonic(),
            require_fixed=False,
        )
        self.latest_rtk_baseline = baseline
        reference_text = " | 거리 계산 필수 입력"
        if hasattr(self, "rtk_fixed_status_label"):
            self.rtk_fixed_status_label.setText(self._rtk_fixed_status_text(baseline))
        if baseline is None:
            self._set_rtk_summary_chip("baseline", "Baseline: 대기 중", "idle")
            self._set_rtk_summary_chip("camera", "Camera: 자세 대기 중", "idle")
            self._set_rtk_summary_chip("status", "Status: GPS 대기 중", "idle")
            self.rtk_baseline_label.setText(f"GPS baseline: 좌/우 GPS 대기 중{reference_text}")
            self._refresh_rtk_alignment_view()
            self._refresh_operation_header()
            self._refresh_alignment_workflow_status()
            return None
        warning_text = f" | {'; '.join(baseline.warnings)}" if baseline.warnings else ""
        usable, reason = self._gps_baseline_usable_for_translation(baseline)
        baseline_level = "good" if baseline.fix_quality == "GOOD" else "warn" if baseline.fix_quality == "DEGRADED" else "bad"
        status_level = "good" if usable else "bad"
        self._set_rtk_summary_chip("baseline", f"Baseline {baseline.baseline_m:.3f} m", baseline_level)
        self._set_rtk_summary_chip(
            "status",
            "Status: 사용 가능" if usable else f"Status: {reason}",
            status_level,
        )
        self.rtk_baseline_label.setText(
            f"GPS baseline: {baseline.baseline_m:.3f} m "
            f"N/E/D={baseline.north_m:.3f}/{baseline.east_m:.3f}/{baseline.down_m:.3f} m "
            f"age={baseline.left_age_s * 1000:.0f}/{baseline.right_age_s * 1000:.0f}ms "
            f"quality={baseline.fix_quality}{reference_text}{warning_text}"
        )
        self._refresh_rtk_alignment_view()
        self._refresh_operation_header()
        self._refresh_alignment_workflow_status()
        return baseline

    def _set_rtk_summary_chip(self, key: str, text: str, level: str) -> None:
        labels = getattr(self, "rtk_summary_labels", {})
        label = labels.get(key)
        if label is not None:
            self._set_health_label(label, text, level)

    def _baseline_forward_geometry_from_camera_translation(
        self, camera_translation: tuple[float, float, float] | None
    ) -> tuple[float, float, float, float] | None:
        if camera_translation is None:
            return None
        right_m, down_m, forward_m = camera_translation
        length_m = math.sqrt(right_m * right_m + down_m * down_m + forward_m * forward_m)
        if length_m <= 1e-9:
            return None
        ratio = max(-1.0, min(1.0, forward_m / length_m))
        angle_deg = math.degrees(math.acos(ratio))
        effective_m = math.sqrt(right_m * right_m + down_m * down_m)
        return angle_deg, effective_m, length_m, forward_m

    def _baseline_camera_translation_in_side_rdf_m(
        self,
        baseline: RtkBaselineEstimate,
        side: str,
    ) -> tuple[float, float, float] | None:
        extrinsic = self.latest_extrinsics.get(side)
        if extrinsic is None:
            return None
        world_vector = (baseline.north_m, baseline.east_m, baseline.down_m)
        body_vector = world_ned_vector_to_body_frd(world_vector, extrinsic.q)
        camera_vector = body_frd_vector_to_camera(body_vector)
        scale_to_m = self.baseline_spin.value() if hasattr(self, "baseline_spin") else None
        if scale_to_m is None:
            return camera_vector
        length_m = math.sqrt(sum(component * component for component in camera_vector))
        if length_m <= 1e-9:
            return None
        scale = float(scale_to_m) / length_m
        return (camera_vector[0] * scale, camera_vector[1] * scale, camera_vector[2] * scale)

    def _set_baseline_forward_metric_labels(
        self,
        *,
        prefix: str,
        geometry: tuple[float, float, float, float] | None,
        missing_text: str = "자세 대기 중",
    ) -> None:
        if not hasattr(self, "baseline_attitude_labels"):
            return
        angle_key = "rdf_forward_angle" if prefix == "left" else "right_rdf_forward_angle"
        effective_key = "effective_baseline" if prefix == "left" else "right_effective_baseline"
        forward_key = "forward_component" if prefix == "left" else "right_forward_component"
        if geometry is None:
            self._set_health_label(self.baseline_attitude_labels[angle_key], missing_text, "warn")
            self._set_health_label(self.baseline_attitude_labels[effective_key], missing_text, "warn")
            self._set_health_label(self.baseline_attitude_labels[forward_key], missing_text, "warn")
            return
        angle_deg, effective_m, length_m, forward_m = geometry
        self._set_health_label(
            self.baseline_attitude_labels[angle_key],
            f"{angle_deg:.2f} deg ({'P1' if prefix == 'left' else 'P2'} RDF +Z vs P1->P2 baseline)",
            self._baseline_forward_angle_level(angle_deg),
        )
        self._set_label_text_if_changed(
            self.baseline_attitude_labels[effective_key],
            f"{effective_m:.4f} m / {length_m:.4f} m",
        )
        self._set_label_text_if_changed(
            self.baseline_attitude_labels[forward_key],
            f"{forward_m:+.4f} m",
        )

    def _baseline_forward_angle_level(self, angle_deg: float) -> str:
        if 80.0 <= angle_deg <= 100.0:
            return "good"
        if 70.0 <= angle_deg <= 110.0:
            return "warn"
        return "bad"

    def _baseline_forward_geometry_text(self, camera_translation: tuple[float, float, float] | None) -> str:
        geometry = self._baseline_forward_geometry_from_camera_translation(camera_translation)
        if geometry is None:
            return "P1 RDF Forward 각도: 자세 대기 중"
        angle_deg, effective_m, length_m, forward_m = geometry
        return (
            f"P1 RDF Forward vs baseline angle={angle_deg:.2f}°, "
            f"effective={effective_m:.3f}/{length_m:.3f} m, forward={forward_m:+.3f} m"
        )

    def _refresh_rtk_alignment_view(self) -> None:
        if not hasattr(self, "rtk_alignment_plot"):
            return
        baseline = self.latest_rtk_baseline
        if baseline is None:
            self.rtk_alignment_plot.set_alignment(
                None,
                left_attitude=self.latest_extrinsics.get("left"),
                right_attitude=self.latest_extrinsics.get("right"),
            )
            self.rtk_alignment_detail_label.setText(
                "정렬 성분: 좌/우 GPS 위치 대기 중. 안테나 baseline은 카메라 광축 수직 baseline과 다를 수 있습니다."
            )
            return
        camera_translation = self._alignment_view_camera_translation_right_in_left_m(baseline)
        left_ext = self.latest_extrinsics.get("left")
        right_ext = self.latest_extrinsics.get("right")
        self.rtk_alignment_plot.set_alignment(baseline, camera_translation, left_ext, right_ext)
        antenna_text = (
            f"안테나 |T|={baseline.baseline_m:.3f} m, "
            f"N/E/D={baseline.north_m:.3f}/{baseline.east_m:.3f}/{baseline.down_m:.3f} m"
        )
        if camera_translation is None:
            self._set_rtk_summary_chip("camera", "Camera: 자세 대기 중", "warn")
            self.rtk_alignment_detail_label.setText(
                f"정렬 성분: {antenna_text}. GPS-only 모드: N/E/D 기준선을 표시합니다. "
                "카메라 기준 Right/Down/Forward 변환은 픽스호크 자세 수신 후 추가됩니다."
            )
            return
        attitude_text = ""
        if left_ext is not None and right_ext is not None:
            relative = relative_attitude(left_ext.q, right_ext.q)
            attitude_text = (
                f" 상대 자세 ΔR/P/Y={relative.roll_deg:+.2f}/"
                f"{relative.pitch_deg:+.2f}/{relative.yaw_deg:+.2f}°."
            )
        x_m, y_m, z_m = camera_translation
        right_camera_translation = self._baseline_camera_translation_in_side_rdf_m(baseline, "right")
        left_geometry = self._baseline_forward_geometry_from_camera_translation(camera_translation)
        right_geometry = self._baseline_forward_geometry_from_camera_translation(right_camera_translation)
        self._set_baseline_forward_metric_labels(prefix="left", geometry=left_geometry)
        self._set_baseline_forward_metric_labels(prefix="right", geometry=right_geometry)
        right_geometry_text = self._baseline_forward_geometry_text(right_camera_translation).replace(
            "P1 RDF Forward", "P2 RDF Forward"
        )
        forward_warn = abs(z_m) > max(0.2, abs(x_m) * 0.5)
        down_warn = abs(y_m) > max(0.1, abs(x_m) * 0.25)
        self._set_rtk_summary_chip(
            "camera",
            f"P1 RDF R/D/F {x_m:+.3f}/{y_m:+.3f}/{z_m:+.3f} m",
            "warn" if forward_warn or down_warn else "good",
        )
        self.rtk_alignment_detail_label.setText(
            f"정렬 성분: {antenna_text}. "
            f"P1 RDF Right={x_m:.3f} m, Down={y_m:.3f} m, Forward={z_m:.3f} m. "
            f"P1 RDF 좌우 성분 |Right|={abs(x_m):.3f} m입니다. "
            f"{self._baseline_forward_geometry_text(camera_translation)} | {right_geometry_text}.{attitude_text}"
        )

    def _build_measurement_attempt(self, pair) -> MeasurementAttempt:
        """Freeze every input selected for one detection pair before geometry runs."""

        triangulation_source = self._triangulation_extrinsics_source()
        timestamp_basis = self._effective_yolo_time_basis()
        left_pairing_timestamp_s = self._yolo_match_timestamp_s(pair.left)
        right_pairing_timestamp_s = self._yolo_match_timestamp_s(pair.right)
        if left_pairing_timestamp_s is None or right_pairing_timestamp_s is None:
            raise ValueError("matched detection pair has no timestamp on its selected basis")
        key = detection_pair_key(
            triangulation_source=triangulation_source,
            timestamp_basis=timestamp_basis,
            left=pair.left,
            right=pair.right,
            left_pairing_timestamp_s=left_pairing_timestamp_s,
            right_pairing_timestamp_s=right_pairing_timestamp_s,
        )
        cached_attempt = (
            self._last_measurement_attempt
            if key == self._last_measurement_attempt_key
            else None
        )
        if cached_attempt is not None and cached_attempt.is_geometry_ready:
            return cached_attempt

        created_monotonic_s = time.monotonic()
        detection_midpoint_s = (float(pair.left.timestamp_s) + float(pair.right.timestamp_s)) / 2.0
        left_detection = DetectionSnapshot.from_observation(
            pair.left,
            pairing_timestamp_basis=timestamp_basis,
            pairing_timestamp_s=left_pairing_timestamp_s,
        )
        right_detection = DetectionSnapshot.from_observation(
            pair.right,
            pairing_timestamp_basis=timestamp_basis,
            pairing_timestamp_s=right_pairing_timestamp_s,
        )
        intrinsics = None
        if self.active_intrinsics is not None:
            profile_path = self._active_intrinsics_profile_path or (
                f"memory:active-calibration:zoom-{self.zoom_level_spin.value():02d}"
            )
            intrinsics = StereoIntrinsicsSnapshot.from_result(
                self.active_intrinsics,
                active_zoom_level=self.zoom_level_spin.value(),
                selected_profile_path=profile_path,
            )
        max_time_delta_s = self._yolo_pair_max_delta_s()
        max_vertical_delta_px = self._yolo_vertical_max_delta_px()
        geometry_config = GeometryConfigSnapshot(
            focal_length_px=self.focal_spin.value(),
            max_time_delta_s=max_time_delta_s if math.isfinite(max_time_delta_s) else None,
            max_vertical_delta_px=(
                max_vertical_delta_px if math.isfinite(max_vertical_delta_px) else None
            ),
            require_same_class=True,
            timestamp_basis=timestamp_basis,
            manual_focal_override=self.manual_focal_checkbox.isChecked(),
            require_rtk_fixed_setting=bool(self.settings.require_rtk_fixed),
        )

        left_attitude_snapshot = None
        right_attitude_snapshot = None
        left_gps_snapshot = None
        right_gps_snapshot = None
        left_attitude_age_s = None
        right_attitude_age_s = None
        attitude_pair_delta_s = None
        relative = None
        warnings: tuple[str, ...] = ()
        left_nearest = self.attitude_buffers["left"].nearest(float(pair.left.timestamp_s))
        right_nearest = self.attitude_buffers["right"].nearest(float(pair.right.timestamp_s))
        if left_nearest is not None:
            left_attitude_snapshot = AttitudeSnapshot.from_state(left_nearest.sample)
            left_attitude_age_s = left_nearest.age_s
        if right_nearest is not None:
            right_attitude_snapshot = AttitudeSnapshot.from_state(right_nearest.sample)
            right_attitude_age_s = right_nearest.age_s
        if left_attitude_snapshot is not None and right_attitude_snapshot is not None:
            attitude_pair_delta_s = abs(
                left_attitude_snapshot.timestamp_s - right_attitude_snapshot.timestamp_s
            )
            relative = relative_attitude(
                left_attitude_snapshot.quaternion_wxyz,
                right_attitude_snapshot.quaternion_wxyz,
            )

        # Preserve the selected GPS antenna samples in both checkerboard and RTK modes.
        selected_left_gps, selected_right_gps = self._selected_baseline_gps_states()
        if selected_left_gps is not None:
            left_gps_snapshot = GpsSnapshot.from_state(selected_left_gps)
        if selected_right_gps is not None:
            right_gps_snapshot = GpsSnapshot.from_state(selected_right_gps)

        if triangulation_source == "checkerboard":
            baseline = build_checkerboard_baseline_snapshot(intrinsics)
            if baseline is None:
                warnings = ("waiting for checkerboard stereo extrinsics",)
            elif baseline.warnings:
                warnings = baseline.warnings
        else:
            baseline = None
            if left_gps_snapshot is not None and right_gps_snapshot is not None:
                baseline = build_pixhawk_baseline_snapshot(
                    left_gps=left_gps_snapshot,
                    right_gps=right_gps_snapshot,
                    selected_left_quaternion_wxyz=(
                        None
                        if left_attitude_snapshot is None
                        else left_attitude_snapshot.quaternion_wxyz
                    ),
                    selected_right_quaternion_wxyz=(
                        None
                        if right_attitude_snapshot is None
                        else right_attitude_snapshot.quaternion_wxyz
                    ),
                    selected_left_attitude_monotonic_s=(
                        None
                        if left_attitude_snapshot is None
                        else left_attitude_snapshot.timestamp_s
                    ),
                    created_monotonic_s=created_monotonic_s,
                    detection_midpoint_monotonic_s=detection_midpoint_s,
                )
                warnings = baseline.warnings
                if baseline.unusable_reason:
                    warnings = tuple(dict.fromkeys((*warnings, baseline.unusable_reason)))
            else:
                warnings = ("waiting for live GPS baseline",)

        candidate_attempt_id = self._measurement_attempt_counter + 1
        attempt = MeasurementAttempt(
            attempt_id=candidate_attempt_id,
            attempt_session_id=self._measurement_attempt_generation_id,
            detection_pair_key=key,
            created_monotonic_s=created_monotonic_s,
            created_wall_s=time.time(),
            detection_midpoint_monotonic_s=detection_midpoint_s,
            timestamp_basis=timestamp_basis,
            triangulation_source=triangulation_source,
            left_detection=left_detection,
            right_detection=right_detection,
            left_attitude=left_attitude_snapshot,
            right_attitude=right_attitude_snapshot,
            left_gps=left_gps_snapshot,
            right_gps=right_gps_snapshot,
            intrinsics=intrinsics,
            baseline=baseline,
            geometry_config=geometry_config,
            left_attitude_age_s=left_attitude_age_s,
            right_attitude_age_s=right_attitude_age_s,
            attitude_pair_delta_s=attitude_pair_delta_s,
            relative_roll_deg=None if relative is None else relative.roll_deg,
            relative_pitch_deg=None if relative is None else relative.pitch_deg,
            relative_yaw_deg=None if relative is None else relative.yaw_deg,
            source_quality=(
                "GOOD"
                if baseline is None or baseline.fix_quality == "NOT_USED"
                else baseline.fix_quality
            ),
            warnings=warnings,
        )
        if cached_attempt is not None:
            previous_missing = set(cached_attempt.missing_geometry_inputs)
            candidate_missing = set(attempt.missing_geometry_inputs)
            if not candidate_missing < previous_missing:
                return cached_attempt
        self._measurement_attempt_counter = candidate_attempt_id
        self._last_measurement_attempt_key = key
        self._last_measurement_attempt = attempt
        return attempt

    def _translation_right_in_left_m(
        self, attempt: MeasurementAttempt
    ) -> tuple[float, float, float] | None:
        """Return the immutable attempt translation without consulting latest state."""

        return attempt.translation_right_in_left_m

    def _latest_translation_right_in_left_m_for_diagnostics(self) -> tuple[float, float, float] | None:
        if self._distance_mode() == "checkerboard":
            return self._checkerboard_translation_right_in_left_m()
        left_ext = self.latest_extrinsics.get("left")
        if left_ext is None:
            return None
        baseline = self.latest_rtk_baseline
        if baseline is None:
            return None
        usable, _reason = self._gps_baseline_usable_for_translation(baseline)
        if not usable:
            return None
        right_ext = self.latest_extrinsics.get("right")
        world_vector = (baseline.north_m, baseline.east_m, baseline.down_m)
        return camera_translation_right_in_left_camera(
            baseline_right_in_left_world_m=world_vector,
            left_body_to_world_q=left_ext.q,
            right_body_to_world_q=right_ext.q if right_ext is not None else left_ext.q,
            scale_to_m=None,
        )

    def _alignment_view_camera_translation_right_in_left_m(
        self, baseline: RtkBaselineEstimate
    ) -> tuple[float, float, float] | None:
        left_ext = self.latest_extrinsics.get("left")
        if left_ext is None:
            return None
        right_ext = self.latest_extrinsics.get("right")
        world_vector = (baseline.north_m, baseline.east_m, baseline.down_m)
        scale_to_m = self.baseline_spin.value() if hasattr(self, "baseline_spin") else None
        return camera_translation_right_in_left_camera(
            baseline_right_in_left_world_m=world_vector,
            left_body_to_world_q=left_ext.q,
            right_body_to_world_q=right_ext.q if right_ext is not None else left_ext.q,
            scale_to_m=scale_to_m,
        )

    def _refresh_distance(self) -> None:
        if self._rtsp_urls_are_same_stream():
            self._clear_measurement_pair_display()
            self._clear_triangulation_quality("좌/우 RTSP URL이 동일해 3D ray 품질을 표시할 수 없습니다.")
            self._set_distance_text(
                "거리: 좌/우 RTSP URL이 동일해 측정 차단",
                quality="BAD",
                detail="그래프 미표시: 좌/우가 같은 스트림이라 유효한 매칭 쌍이 없습니다.",
            )
            return
        pair = self._matched_detection_pair()
        if pair is None:
            self._clear_measurement_pair_display()
            wait_status = self._detection_wait_status()
            self._clear_triangulation_quality(wait_status.removeprefix("거리: "))
            self._set_distance_text(
                wait_status,
                quality="BAD",
                detail=self._detection_wait_detail(),
            )
            return
        if self._measurement_display_mode() == "detection":
            self._show_measurement_pair(pair.left, pair.right)
        else:
            self._clear_measurement_pair_display()

        try:
            attempt = self._build_measurement_attempt(pair)
        except (TypeError, ValueError) as exc:
            self._clear_triangulation_quality("측정 입력 snapshot 생성 실패")
            self._set_distance_text(
                f"거리: 측정 입력 snapshot 생성 실패 | {exc}",
                quality="BAD",
                detail="그래프 미표시: 동일 입력 provenance를 고정할 수 없습니다.",
            )
            return

        baseline_m = attempt.baseline_m
        baseline_source = attempt.baseline_source
        if baseline_source in {"rtk-blocked", "gps-rtk-blocked"}:
            self._record_unresolved_pair(
                attempt,
                status="matched_baseline_blocked",
                warnings=attempt.warnings,
            )
            self._clear_triangulation_quality("GPS baseline 조건 미충족")
            self._set_distance_text(
                "거리: GPS baseline 조건 미충족 | " + "; ".join(attempt.warnings),
                quality="BAD",
                detail="그래프 미표시: GPS baseline 조건을 만족하지 못했습니다.",
            )
            return
        triangulation_source = attempt.triangulation_source
        geometry_result = calculate_measurement_geometry(attempt)
        match = geometry_result.match
        triangulated = geometry_result.triangulated
        estimate = geometry_result.estimate
        combined_warnings = geometry_result.warnings
        if match is None:
            self._record_unresolved_pair(
                attempt,
                status="matched_stereo_geometry_invalid",
                warnings=combined_warnings,
            )
            self._clear_triangulation_quality("유효한 스테레오 기하 대기 중")
            self._set_distance_text(
                "거리: 유효한 스테레오 기하 대기 중",
                quality="BAD",
                detail=self._stereo_geometry_wait_detail(pair),
            )
            return
        if triangulated is None:
            self._record_unresolved_pair(
                attempt,
                status="matched_triangulation_pending",
                warnings=combined_warnings,
            )
            warning_text = f" | {'; '.join(combined_warnings)}" if combined_warnings else ""
            self._clear_triangulation_quality("3D ray 기하 대기 중")
            self._set_distance_text(
                f"거리: 3D ray 기하 대기 중 ({self._triangulation_extrinsics_source_label()}){warning_text}",
                quality="BAD",
                detail=self._triangulation_wait_detail(combined_warnings),
            )
            return
        if estimate is None:
            self._record_unresolved_pair(
                attempt,
                status="matched_distance_invalid",
                warnings=combined_warnings,
            )
            self._clear_triangulation_quality("유효한 거리 계산 대기 중")
            self._set_distance_text(
                "거리: 유효한 거리 계산 대기 중",
                quality="BAD",
                detail="그래프 미표시: 현재 입력으로 거리를 만들 수 없습니다.",
            )
            return

        public_point_m = triangulated.point_m
        source_point_left_rdf_m = getattr(triangulated, "source_point_left_rdf_m", None)
        display_point_m = public_point_m
        display_distance_m = triangulated.range_m
        method_text = f"{triangulated.method}-{triangulation_source}"
        gap_text = f" gap={triangulated.closest_gap_m:.3f}m"
        display_baseline_source = baseline_source
        display_baseline_m = baseline_m
        attitude_timing_status = classify_attitude_timing(
            uses_pixhawk_attitude=triangulation_source != "checkerboard",
            left_age_s=estimate.left_attitude_age_s,
            right_age_s=estimate.right_attitude_age_s,
            pair_delta_s=estimate.attitude_pair_delta_s,
        )
        self._update_triangulation_quality(
            triangulated=triangulated,
            method_text=method_text,
            display_distance_m=display_distance_m,
            display_baseline_source=display_baseline_source,
            display_baseline_m=display_baseline_m,
            public_point_m=public_point_m,
            source_point_left_rdf_m=source_point_left_rdf_m,
            attitude_timing_status=attitude_timing_status,
            left_attitude_age_s=estimate.left_attitude_age_s,
            right_attitude_age_s=estimate.right_attitude_age_s,
            attitude_pair_delta_s=estimate.attitude_pair_delta_s,
        )

        attitude_badge = "N/A" if attitude_timing_status == "NOT_USED" else attitude_timing_status
        display_quality = estimate.sync_quality
        if attitude_timing_status in {"STALE", "VERY_STALE"} and display_quality == "GOOD":
            display_quality = "DEGRADED"
        relative_text = "relRPY=대기"
        if estimate.relative_roll_deg is not None:
            relative_text = (
                f"relRPY={estimate.relative_roll_deg:.2f}/"
                f"{estimate.relative_pitch_deg:.2f}/{estimate.relative_yaw_deg:.2f}°"
            )
        warning_text = f" | {'; '.join(estimate.warnings)}" if estimate.warnings else ""
        self._set_distance_text(
            f"거리: {display_distance_m:.2f} m ({method_text}{gap_text}) | bboxΔx={estimate.disparity_px:.1f}px "
            f"camΔt={estimate.time_delta_s * 1000:.0f}ms Δy={estimate.vertical_delta_px:.1f}px "
            f"baseline={display_baseline_source}:{display_baseline_m:.3f}m "
            f"ATT:{attitude_badge} {relative_text} quality={display_quality}{warning_text}",
            quality=display_quality,
        )
        appended = self._append_distance_history(
            estimate=estimate,
            distance_m=display_distance_m,
            method=method_text,
            baseline_source=display_baseline_source,
            baseline_m=display_baseline_m,
            point_m=display_point_m,
            raw_point_m=public_point_m,
            raw_point_left_m=source_point_left_rdf_m,
            measurement_attempt=attempt,
        )
        if not appended:
            self._set_distance_text(
                self.last_distance_status,
                quality=display_quality,
                detail="그래프 미표시: 직전과 동일한 샘플이라 중복 추가를 건너뛰었습니다.",
            )

    def _show_measurement_pair(self, left: DetectionObservation, right: DetectionObservation) -> None:
        left_shown = self.left_pane.show_measurement_frame_for_detection(left)
        right_shown = self.right_pane.show_measurement_frame_for_detection(right)
        if not (left_shown and right_shown):
            self._update_runtime_camera_stats_labels()
            return
        self.selected_runtime_frame_metadata["left"] = dict(left.timestamp_metadata)
        self.selected_runtime_frame_metadata["right"] = dict(right.timestamp_metadata)
        left_timestamp_s = self._yolo_match_timestamp_s(left)
        right_timestamp_s = self._yolo_match_timestamp_s(right)
        self.selected_runtime_frame_pair_delta_s = (
            None if left_timestamp_s is None or right_timestamp_s is None else abs(left_timestamp_s - right_timestamp_s)
        )
        (
            self.selected_runtime_frame_pair_basis,
            self.selected_runtime_frame_pair_camera_rtc_delta_s,
        ) = self._selected_pair_sync_summary(left, right)
        self._update_runtime_camera_stats_labels()

    def _clear_measurement_pair_display(self) -> None:
        self.left_pane.clear_measurement_pair_display()
        self.right_pane.clear_measurement_pair_display()
        self._clear_selected_runtime_measurement_pair(update_labels=True)

    def _clear_selected_runtime_measurement_pair(self, *, update_labels: bool = True) -> None:
        self.selected_runtime_frame_metadata["left"] = {}
        self.selected_runtime_frame_metadata["right"] = {}
        self.selected_runtime_frame_pair_delta_s = None
        self.selected_runtime_frame_pair_basis = ""
        self.selected_runtime_frame_pair_camera_rtc_delta_s = None
        if update_labels:
            self._update_runtime_camera_stats_labels()

    def _detection_wait_detail(self) -> str:
        left_count = len(self.detection_buffers["left"])
        right_count = len(self.detection_buffers["right"])
        left_stats = self.camera_stream_stats.get("left") or {}
        right_stats = self.camera_stream_stats.get("right") or {}
        left_yolo_hz = float(left_stats.get("inferred_hz", 0.0))
        right_yolo_hz = float(right_stats.get("inferred_hz", 0.0))
        if self._last_correspondence_rejection_reason and not self._last_correspondence_rejection_reason.startswith(
            "missing_"
        ):
            return (
                "그래프 미표시: correspondence 검증 실패 "
                f"({self._last_correspondence_rejection_reason})."
            )
        if left_count == 0 or right_count == 0:
            if left_yolo_hz <= 0.0 or right_yolo_hz <= 0.0:
                return (
                    "그래프 미표시: YOLO 추론 대기 중 "
                    f"(left={left_yolo_hz:.1f} Hz, right={right_yolo_hz:.1f} Hz)."
                )
            return f"그래프 미표시: bbox 검출 대기 중 (left={left_count}, right={right_count})."
        if self._yolo_time_basis() == "camera_rtc_required":
            left_has_rtc = any(
                self._camera_rtc_timestamp_s_from_metadata(detection.timestamp_metadata) is not None
                for detection in self.detection_buffers["left"]
            )
            right_has_rtc = any(
                self._camera_rtc_timestamp_s_from_metadata(detection.timestamp_metadata) is not None
                for detection in self.detection_buffers["right"]
            )
            if not left_has_rtc or not right_has_rtc:
                return (
                    "그래프 미표시: RTCP/NTP 필수 모드에서 RTCP/NTP timestamp가 없는 검출이 있습니다 "
                    f"(left_rtc={'OK' if left_has_rtc else 'missing'}, "
                    f"right_rtc={'OK' if right_has_rtc else 'missing'})."
                )
        return (
            "그래프 미표시: 좌/우 bbox timestamp가 `최대 Δt` 안에서 아직 맞지 않습니다 "
            f"(left={left_count}, right={right_count})."
            if self._yolo_time_delta_gate_enabled()
            else f"그래프 미표시: 좌/우 bbox 매칭 후보 대기 중 (left={left_count}, right={right_count})."
        )

    def _detection_wait_status(self) -> str:
        if self._last_correspondence_rejection_reason and not self._last_correspondence_rejection_reason.startswith(
            "missing_"
        ):
            return f"거리: 검출 대응 거부 ({self._last_correspondence_rejection_reason})"
        if self._yolo_time_delta_gate_enabled() or self._yolo_time_basis() != "detection":
            return "거리: timestamp가 맞는 좌/우 검출 대기 중"
        return "거리: 좌/우 검출 쌍 대기 중"

    def _stereo_geometry_wait_detail(self, pair) -> str:
        left = pair.left
        right = pair.right
        left_center = getattr(left, "center", (0.0, 0.0))
        right_center = getattr(right, "center", (0.0, 0.0))
        disparity_px = abs(float(left_center[0]) - float(right_center[0]))
        left_timestamp_s = self._yolo_match_timestamp_s(left)
        right_timestamp_s = self._yolo_match_timestamp_s(right)
        time_delta_ms = (
            float("inf")
            if left_timestamp_s is None or right_timestamp_s is None
            else abs(left_timestamp_s - right_timestamp_s) * 1000.0
        )
        time_limit_ms = self._yolo_pair_max_delta_s() * 1000.0
        if not isfinite(disparity_px) or disparity_px <= 1e-6:
            return (
                "그래프 미표시: 좌/우 bbox 중심의 x 차이가 0에 가까워 disparity를 만들 수 없습니다. "
                f"left_x={float(left_center[0]):.1f}px, right_x={float(right_center[0]):.1f}px."
            )
        if time_delta_ms > time_limit_ms:
            return (
                "그래프 미표시: 좌/우 bbox timestamp 차이가 `최대 Δt`보다 큽니다. "
                f"camΔt={time_delta_ms:.0f}ms, limit={time_limit_ms:.0f}ms."
            )
        return (
            "그래프 미표시: 좌/우 bbox는 선택됐지만 거리 계산용 stereo match가 생성되지 않았습니다. "
            f"disparity={disparity_px:.1f}px, camΔt={time_delta_ms:.0f}ms."
        )

    def _triangulation_wait_detail(self, warnings: tuple[str, ...]) -> str:
        warning_text = "; ".join(warnings)
        if any("waiting for intrinsics" in warning for warning in warnings):
            return (
                "그래프 미표시: bbox 매칭은 됐지만 내부 파라미터가 아직 적용되지 않아 3D ray triangulation을 "
                "계산할 수 없습니다. `줌 내부 파라미터`에서 intrinsics JSON을 불러온 뒤 `계산에 적용`을 누르세요. "
                f"{warning_text}"
            )
        if any("waiting for Pixhawk R" in warning for warning in warnings):
            return (
                "그래프 미표시: bbox 매칭은 됐지만 Pixhawk 자세 샘플이 아직 없어 3D ray 방향을 만들 수 없습니다. "
                f"{warning_text}"
            )
        if any("waiting for checkerboard stereo extrinsics" in warning for warning in warnings):
            return (
                "그래프 미표시: bbox 매칭은 됐지만 적용된 체커보드 calibration에 stereo 외부 파라미터가 없습니다. "
                "좌/우 체커보드 쌍을 최소 3개 이상 보정한 뒤 결과를 계산에 적용하세요. "
                f"{warning_text}"
            )
        if any("waiting for RTK T" in warning for warning in warnings):
            return (
                "그래프 미표시: bbox 매칭은 됐지만 좌/우 카메라 translation을 만들 RTK/GPS baseline이 아직 없습니다. "
                f"{warning_text}"
            )
        if any("triangulation error" in warning for warning in warnings):
            return f"그래프 미표시: triangulation 계산 중 오류가 발생했습니다. {warning_text}"
        if warning_text:
            return f"그래프 미표시: 3D ray triangulation 결과가 아직 없습니다. {warning_text}"
        return "그래프 미표시: bbox 매칭은 됐지만 3D ray triangulation 결과가 아직 없습니다."

    def _pair_recording_base_row(self, measurement_attempt: MeasurementAttempt) -> dict[str, object]:
        pair_mid_monotonic_s = measurement_attempt.detection_midpoint_monotonic_s
        pair_mid_monotonic_ns = int(round(pair_mid_monotonic_s * 1_000_000_000.0))
        session_elapsed_ns = (
            None
            if self.recording_session_started_monotonic_ns is None
            else max(0, pair_mid_monotonic_ns - self.recording_session_started_monotonic_ns)
        )
        row: dict[str, object] = {
            "session_id": self.recording_session_id,
            "measurement_attempt_id": str(measurement_attempt.attempt_id),
            "measurement_attempt_session_id": measurement_attempt.attempt_session_id,
            "measurement_attempt_sha256": measurement_attempt.canonical_sha256(),
            "measurement_detection_pair_key": json.dumps(
                measurement_attempt.detection_pair_key, separators=(",", ":")
            ),
            "schema_version": "stereowiris-paired-frame-v2",
            "measurement_attempt_content_sha256": measurement_attempt.content_sha256(),
            "measurement_attempt_instance_sha256": measurement_attempt.instance_sha256(),
            "measurement_attempt_timestamp_basis": measurement_attempt.timestamp_basis,
            "measurement_attempt_source_quality": measurement_attempt.source_quality,
            "measurement_attempt_warnings": "; ".join(measurement_attempt.warnings),
            "measurement_frame": MEASUREMENT_FRAME,
            "measurement_axis_contract": MEASUREMENT_AXIS_CONTRACT,
            "public_vector_contract": PUBLIC_MEASUREMENT_VECTOR_CONTRACT,
            "legacy_xyz_alias_contract": LEGACY_XYZ_ALIAS_CONTRACT,
            "baseline_frame_scope": "measurement_attempt",
            "baseline_frame_instance_id": measurement_attempt.instance_sha256(),
            "baseline_frame_origin_contract": (
                "midpoint_of_left_optical_origin_and_selected_right_origin_proxy"
            ),
            "baseline_frame_origin_model": (
                "checkerboard_stereo_rt_optical_centers"
                if measurement_attempt.triangulation_source == "checkerboard"
                else "gps_rtk_antenna_baseline_proxy_no_lever_arm"
            ),
            "baseline_frame_axis_construction": (
                "right=left_to_right_center;forward=projected_average_optical_forward;"
                "down=forward_cross_right"
            ),
            "measurement_valid": "false",
            "pair_status": "matched_geometry_pending",
            "session_started_unix_ns": self.recording_session_started_unix_ns,
            "session_started_monotonic_ns": self.recording_session_started_monotonic_ns,
            "recorded_unix_ns": int(round(measurement_attempt.created_wall_s * 1_000_000_000.0)),
            "pair_app_mid_monotonic_ns": pair_mid_monotonic_ns,
            "session_elapsed_ns": session_elapsed_ns,
            "session_elapsed_s": (
                "" if session_elapsed_ns is None else session_elapsed_ns * 1e-9
            ),
            "left_video_path": str(self.video_recording_paths.get("left", "")),
            "right_video_path": str(self.video_recording_paths.get("right", "")),
            "selected_left_pairing_timestamp_s": self._fmt_record_float(
                measurement_attempt.left_detection.pairing_timestamp_s, 9
            ),
            "selected_right_pairing_timestamp_s": self._fmt_record_float(
                measurement_attempt.right_detection.pairing_timestamp_s, 9
            ),
        }
        row.update(self._record_bbox_parameters(measurement_attempt))
        row.update(self._record_measurement_sync_timestamp_parameters(measurement_attempt))
        row.update(self._record_intrinsic_parameters(measurement_attempt))
        row.update(self._record_frame_timestamp_parameters(measurement_attempt))
        row.update(self._record_live_telemetry_snapshot_parameters(measurement_attempt))
        row.update(self._record_external_parameters(estimate=None, measurement_attempt=measurement_attempt))
        row.update(self._record_reference_delta_parameters(measurement_attempt))
        return row

    def _upsert_pair_recording_row(
        self,
        measurement_attempt: MeasurementAttempt,
        values: dict[str, object],
    ) -> bool:
        if not self.distance_recording_enabled:
            return False
        key = tuple(measurement_attempt.detection_pair_key)
        row_index = self._recording_pair_row_index.get(key)
        if row_index is None:
            row_index = len(self.distance_recording_rows)
            values["sample_index"] = row_index
            values["pair_sequence"] = row_index
            content_sha256 = values.pop("measurement_attempt_content_sha256", None)
            instance_sha256 = values.pop("measurement_attempt_instance_sha256", None)
            values["measurement_attempt_content_sha256"] = content_sha256
            values["measurement_attempt_instance_sha256"] = instance_sha256
            self._recording_pair_row_index[key] = row_index
            self.distance_recording_rows.append(values)
            self.distance_recording_keys.append(key)
            if len(self.distance_recording_keys) > RECORDING_KEY_CACHE_LIMIT:
                self.distance_recording_keys.pop(0)
            minimum_cached_index = max(
                0, len(self.distance_recording_rows) - RECORDING_KEY_CACHE_LIMIT
            )
            self._recording_pair_row_index = {
                cached_key: cached_index
                for cached_key, cached_index in self._recording_pair_row_index.items()
                if cached_index >= minimum_cached_index
            }
            self._append_recording_table_row(values)
            inserted = True
        else:
            current = self.distance_recording_rows[row_index]
            sample_index = current.get("sample_index", row_index)
            pair_sequence = current.get("pair_sequence", row_index)
            current.update(values)
            current["sample_index"] = sample_index
            current["pair_sequence"] = pair_sequence
            if isinstance(self.distance_recording_rows, SpoolRows):
                self.distance_recording_rows.replace(row_index, current)
            self._refresh_recording_table()
            inserted = False
        self._refresh_recording_status_label()
        self._refresh_operation_header()
        return inserted

    def _record_unresolved_pair(
        self,
        measurement_attempt: MeasurementAttempt,
        *,
        status: str,
        warnings: tuple[str, ...] = (),
    ) -> None:
        if not self.distance_recording_enabled:
            return
        row = self._pair_recording_base_row(measurement_attempt)
        row.update(
            {
                "pair_status": status,
                "measurement_valid": "false",
                "warnings": "; ".join(warnings or measurement_attempt.warnings),
            }
        )
        self._upsert_pair_recording_row(measurement_attempt, row)

    def _append_distance_history(
        self,
        *,
        estimate,
        distance_m: float,
        method: str,
        baseline_source: str,
        baseline_m: float,
        point_m: tuple[float, float, float] | None = None,
        raw_point_m: tuple[float, float, float] | None = None,
        raw_point_left_m: tuple[float, float, float] | None = None,
        point_left_m: tuple[float, float, float] | None = None,
        measurement_attempt: MeasurementAttempt,
    ) -> bool:
        baseline_source = measurement_attempt.baseline_source
        baseline_m = measurement_attempt.baseline_m
        key = tuple(measurement_attempt.detection_pair_key)
        if key == self._last_distance_history_key:
            return False
        self._last_distance_history_key = key
        if point_m is None:
            point_m = point_left_m
        if raw_point_m is None:
            raw_point_m = raw_point_left_m
        timestamp_s = max(estimate.left.timestamp_s, estimate.right.timestamp_s)
        class_name = self._distance_history_class_name(estimate)
        recording_elapsed_s = None
        if self.distance_recording_enabled:
            if self.distance_recording_started_monotonic_s is None:
                self.distance_recording_started_monotonic_s = time.monotonic()
            recording_elapsed_s = max(0.0, time.monotonic() - self.distance_recording_started_monotonic_s)
        point = DistanceHistoryPoint(
            timestamp_s=timestamp_s,
            distance_m=distance_m,
            quality=estimate.sync_quality,
            method=method,
            forward_m=None if point_m is None else float(point_m[0]),
            right_m=None if point_m is None else float(point_m[1]),
            down_m=None if point_m is None else float(point_m[2]),
            class_name=class_name,
            sample_key=tuple(measurement_attempt.detection_pair_key),
            recording_elapsed_s=recording_elapsed_s,
        )
        self.distance_history.append(point)
        self.distance_plot.add_point(point)
        if not self.distance_recording_enabled:
            return True
        raw_distance_m = None
        if raw_point_m is not None:
            raw_distance_m = math.sqrt(
                raw_point_m[0] * raw_point_m[0]
                + raw_point_m[1] * raw_point_m[1]
                + raw_point_m[2] * raw_point_m[2]
            )
        measurement_attempt_content_sha256 = measurement_attempt.content_sha256()
        measurement_attempt_instance_sha256 = measurement_attempt.instance_sha256()
        row = self._pair_recording_base_row(measurement_attempt)
        row.update({
            "session_id": self.recording_session_id,
            "measurement_attempt_id": str(measurement_attempt.attempt_id),
            "measurement_attempt_session_id": measurement_attempt.attempt_session_id,
            "measurement_attempt_sha256": measurement_attempt.canonical_sha256(),
            "measurement_detection_pair_key": json.dumps(
                measurement_attempt.detection_pair_key, separators=(",", ":")
            ),
            "measurement_attempt_timestamp_basis": measurement_attempt.timestamp_basis,
            "selected_left_pairing_timestamp_s": self._fmt_record_float(
                measurement_attempt.left_detection.pairing_timestamp_s, 6
            ),
            "selected_right_pairing_timestamp_s": self._fmt_record_float(
                measurement_attempt.right_detection.pairing_timestamp_s, 6
            ),
            "measurement_attempt_source_quality": measurement_attempt.source_quality,
            "measurement_attempt_warnings": "; ".join(measurement_attempt.warnings),
            "session_started_unix_s": self._fmt_record_float(self.recording_session_started_wall_s, 6),
            "left_video_path": str(self.video_recording_paths.get("left", "")),
            "right_video_path": str(self.video_recording_paths.get("right", "")),
            "timestamp_s": f"{timestamp_s:.6f}",
            "distance_m": f"{distance_m:.6f}",
            "method": method,
            "quality": estimate.sync_quality,
            "disparity_px": f"{estimate.disparity_px:.6f}",
            "raw_disparity_px": f"{estimate.disparity_px:.6f}",
            "camera_delta_s": f"{estimate.time_delta_s:.6f}",
            "vertical_delta_px": f"{estimate.vertical_delta_px:.6f}",
            "baseline_source": baseline_source,
            "baseline_m": f"{baseline_m:.6f}",
            "measurement_frame": MEASUREMENT_FRAME,
            "measurement_axis_contract": MEASUREMENT_AXIS_CONTRACT,
            "legacy_xyz_alias_contract": LEGACY_XYZ_ALIAS_CONTRACT,
            "point_frame": MEASUREMENT_FRAME,
            "output_frame": MEASUREMENT_FRAME,
            "public_vector_contract": PUBLIC_MEASUREMENT_VECTOR_CONTRACT,
            "baseline_frame_scope": "measurement_attempt",
            "baseline_frame_instance_id": measurement_attempt.instance_sha256(),
            "baseline_frame_origin_contract": (
                "midpoint_of_left_optical_origin_and_selected_right_origin_proxy"
            ),
            "baseline_frame_origin_model": (
                "checkerboard_stereo_rt_optical_centers"
                if measurement_attempt.triangulation_source == "checkerboard"
                else "gps_rtk_antenna_baseline_proxy_no_lever_arm"
            ),
            "measurement_method": "two_ray_closest_midpoint",
            "range_m": self._fmt_record_float(distance_m, 6),
            "baseline_center_forward_m": self._fmt_record_float(point.forward_m, 6),
            "baseline_center_right_m": self._fmt_record_float(point.right_m, 6),
            "baseline_center_down_m": self._fmt_record_float(point.down_m, 6),
            "baseline_center_range_m": self._fmt_record_float(distance_m, 6),
            "raw_baseline_center_forward_m": self._fmt_record_float(
                None if raw_point_m is None else raw_point_m[0], 6
            ),
            "raw_baseline_center_right_m": self._fmt_record_float(
                None if raw_point_m is None else raw_point_m[1], 6
            ),
            "raw_baseline_center_down_m": self._fmt_record_float(
                None if raw_point_m is None else raw_point_m[2], 6
            ),
            "raw_baseline_center_range_m": self._fmt_record_float(raw_distance_m, 6),
            "source_left_camera_rdf_right_m": self._fmt_record_float(
                None if raw_point_left_m is None else raw_point_left_m[0], 6
            ),
            "source_left_camera_rdf_down_m": self._fmt_record_float(
                None if raw_point_left_m is None else raw_point_left_m[1], 6
            ),
            "source_left_camera_rdf_forward_m": self._fmt_record_float(
                None if raw_point_left_m is None else raw_point_left_m[2], 6
            ),
            "recording_elapsed_s": self._fmt_record_float(recording_elapsed_s, 6),
            "warnings": "; ".join(estimate.warnings),
            "pair_status": "measured",
            "measurement_valid": "true",
        })
        row.update(self._record_bbox_parameters(estimate))
        row.update(self._record_measurement_sync_timestamp_parameters(estimate))
        row.update(self._record_intrinsic_parameters(measurement_attempt))
        row.update(self._record_frame_timestamp_parameters(measurement_attempt))
        row.update(self._record_live_telemetry_snapshot_parameters(measurement_attempt))
        row.update(self._record_external_parameters(estimate=estimate, measurement_attempt=measurement_attempt))
        row.pop("measurement_attempt_content_sha256", None)
        row.pop("measurement_attempt_instance_sha256", None)
        row["measurement_attempt_content_sha256"] = measurement_attempt_content_sha256
        row["measurement_attempt_instance_sha256"] = measurement_attempt_instance_sha256
        self._upsert_pair_recording_row(measurement_attempt, row)
        return True

    def _distance_history_class_name(self, estimate) -> str | None:
        left_class_name = str(getattr(estimate.left, "class_name", "") or "").strip()
        right_class_name = str(getattr(estimate.right, "class_name", "") or "").strip()
        if left_class_name and right_class_name:
            if left_class_name == right_class_name:
                return left_class_name
            return f"{left_class_name}/{right_class_name}"
        return left_class_name or right_class_name or None

    def _fmt_record_float(self, value: float | None, precision: int = 6) -> str:
        return "" if value is None else f"{float(value):.{precision}f}"

    def _fmt_record_bool(self, value: bool | None) -> str:
        return "" if value is None else ("true" if value else "false")

    def _recording_elapsed_s(self, timestamp_s: float | None = None) -> float | None:
        started_s = self.distance_recording_started_monotonic_s or self.recording_session_started_monotonic_s
        if started_s is None:
            return None
        if timestamp_s is None:
            timestamp_s = time.monotonic()
        return max(0.0, float(timestamp_s) - started_s)

    def _session_utc_ns_for_monotonic_ns(self, monotonic_ns: int) -> int | None:
        if (
            self.recording_session_started_unix_ns is None
            or self.recording_session_started_monotonic_ns is None
        ):
            return None
        return (
            self.recording_session_started_unix_ns
            + monotonic_ns
            - self.recording_session_started_monotonic_ns
        )

    def _recording_side(self, camera: str) -> str | None:
        normalized = str(camera).lower()
        if normalized.startswith("left"):
            return "left"
        if normalized.startswith("right"):
            return "right"
        return None

    def _raw_timeline_row(
        self,
        *,
        side: str,
        sample_type: str,
        row_contract: str,
        sample_monotonic_s: float,
        host_receive_monotonic_ns: int,
        host_receive_utc_ns: int,
        source_utc_ns: int | None,
        source_utc_method: str,
        sequence: int,
    ) -> dict[str, object] | None:
        sample_monotonic_ns = int(round(float(sample_monotonic_s) * 1_000_000_000.0))
        started_monotonic_ns = self.recording_session_started_monotonic_ns
        if started_monotonic_ns is not None and sample_monotonic_ns < started_monotonic_ns:
            return None
        session_elapsed_s = (
            None
            if started_monotonic_ns is None
            else (sample_monotonic_ns - started_monotonic_ns) * 1e-9
        )
        session_utc_ns = self._session_utc_ns_for_monotonic_ns(sample_monotonic_ns)
        return {
            "sequence": sequence,
            "session_id": self.recording_session_id,
            "side": side,
            "sample_type": sample_type,
            "row_contract": row_contract,
            "sample_monotonic_ns": sample_monotonic_ns,
            "sample_monotonic_s": self._fmt_record_float(sample_monotonic_s, 9),
            "session_elapsed_s": self._fmt_record_float(session_elapsed_s, 9),
            "session_utc_ns": "" if session_utc_ns is None else session_utc_ns,
            "session_utc_s": self._fmt_record_float(
                None if session_utc_ns is None else session_utc_ns * 1e-9,
                9,
            ),
            "session_utc_method": (
                "unavailable" if session_utc_ns is None else "host_unix_monotonic_session_anchor"
            ),
            "session_utc_anchor_uncertainty_ns": (
                ""
                if self.recording_session_anchor_uncertainty_ns is None
                else self.recording_session_anchor_uncertainty_ns
            ),
            "source_utc_ns": "" if source_utc_ns is None else source_utc_ns,
            "source_utc_s": self._fmt_record_float(
                None if source_utc_ns is None else source_utc_ns * 1e-9,
                9,
            ),
            "source_utc_valid": self._fmt_record_bool(source_utc_ns is not None),
            "source_utc_method": source_utc_method if source_utc_ns is not None else "unavailable",
            "host_receive_monotonic_ns": host_receive_monotonic_ns,
            "host_receive_utc_ns": host_receive_utc_ns,
        }

    def _record_gui_camera_frame(
        self,
        camera: str,
        image: QtGui.QImage,
        metadata: dict[str, object],
    ) -> None:
        if not self.distance_recording_enabled or camera not in self.camera_recording_rows:
            return
        host_receive_monotonic_ns = time.monotonic_ns()
        host_receive_utc_ns = time.time_ns()
        sample_monotonic_s = float(
            metadata.get("timestamp_s") or host_receive_monotonic_ns * 1e-9
        )
        rtcp_ntp_s = metadata.get("rtcp_frame_ntp_unix_s")
        source_utc_ns = (
            None
            if rtcp_ntp_s is None
            else self._unix_seconds_to_ns(float(rtcp_ntp_s))
        )
        rows = self.camera_recording_rows[camera]
        row = self._raw_timeline_row(
            side=camera,
            sample_type="gui_camera_frame",
            row_contract="one row per frame delivered to the GUI frame_changed callback",
            sample_monotonic_s=sample_monotonic_s,
            host_receive_monotonic_ns=host_receive_monotonic_ns,
            host_receive_utc_ns=host_receive_utc_ns,
            source_utc_ns=source_utc_ns,
            source_utc_method="rtcp_ntp",
            sequence=len(rows),
        )
        if row is None:
            return
        row.update(
            {
                "frame_stage": "gui_delivered_preview",
                "mp4_frame_index_contract": "not_available",
                "image_width_px": image.width(),
                "image_height_px": image.height(),
                "image_axis_contract": "opencv_pixels_origin_top_left_u_right_v_down",
                "timestamp_source": str(metadata.get("timestamp_source") or ""),
                "timestamp_reference": str(metadata.get("timestamp_reference") or ""),
                "source_timestamp_s": self._fmt_record_float(
                    None
                    if metadata.get("source_timestamp_s") is None
                    else float(metadata["source_timestamp_s"]),
                    9,
                ),
                "rtcp_frame_ntp_unix_s": self._fmt_record_float(
                    None if rtcp_ntp_s is None else float(rtcp_ntp_s),
                    9,
                ),
                "rtcp_frame_received_wall_s": self._fmt_record_float(
                    None
                    if metadata.get("rtcp_frame_received_wall_s") is None
                    else float(metadata["rtcp_frame_received_wall_s"]),
                    9,
                ),
                "rtcp_frame_receive_delay_s": self._fmt_record_float(
                    None
                    if metadata.get("rtcp_frame_receive_delay_s") is None
                    else float(metadata["rtcp_frame_receive_delay_s"]),
                    9,
                ),
                "rtcp_timing_invalid_reason": str(
                    metadata.get("rtcp_timing_invalid_reason") or ""
                ),
            }
        )
        rows.append(row)

    def _record_external_timeline_row(self, camera: str, sample_type: str, values: dict[str, object]) -> None:
        if not self.distance_recording_enabled:
            return
        side = self._recording_side(camera)
        if side is None:
            return
        host_receive_monotonic_ns = time.monotonic_ns()
        host_receive_utc_ns = time.time_ns()
        raw_sample_monotonic_s = values.get("sample_app_timestamp_s")
        sample_monotonic_s = (
            host_receive_monotonic_ns * 1e-9
            if raw_sample_monotonic_s in (None, "")
            else float(raw_sample_monotonic_s)
        )
        raw_gps_utc_us = values.get("gps_utc_us")
        source_utc_ns = (
            None
            if raw_gps_utc_us in (None, "", 0, "0")
            else int(raw_gps_utc_us) * 1000
        )
        rows = self.px4_recording_rows[side]
        row = self._raw_timeline_row(
            side=side,
            sample_type=sample_type,
            row_contract="one row per PX4 telemetry callback received by the GUI",
            sample_monotonic_s=sample_monotonic_s,
            host_receive_monotonic_ns=host_receive_monotonic_ns,
            host_receive_utc_ns=host_receive_utc_ns,
            source_utc_ns=source_utc_ns,
            source_utc_method=str(values.get("source_utc_method") or "gps_utc"),
            sequence=len(rows),
        )
        if row is None:
            return
        row.update(
            {
                "camera": camera,
                "triangulation_extrinsics_source": self._triangulation_extrinsics_source(),
                "distance_mode": self._distance_mode_label(),
            }
        )
        row.update(values)
        rows.append(row)
        self.external_parameter_recording_rows.append(row)
        self._refresh_recording_status_label()

    def _record_bbox_parameters(self, estimate) -> dict[str, str]:
        row: dict[str, str] = {}
        for camera in ("left", "right"):
            detection = getattr(estimate, camera, None)
            center = getattr(detection, "center", None)
            if center is None:
                center_x = None
                center_y = None
            else:
                center_x = float(center[0])
                center_y = float(center[1])
            x1 = getattr(detection, "x1", None)
            y1 = getattr(detection, "y1", None)
            x2 = getattr(detection, "x2", None)
            y2 = getattr(detection, "y2", None)
            bbox_width = None
            bbox_height = None
            if x1 is not None and x2 is not None:
                bbox_width = float(x2) - float(x1)
            if y1 is not None and y2 is not None:
                bbox_height = float(y2) - float(y1)
            row[f"{camera}_bbox_x1_px"] = self._fmt_record_float(getattr(detection, "x1", None), 3)
            row[f"{camera}_bbox_y1_px"] = self._fmt_record_float(getattr(detection, "y1", None), 3)
            row[f"{camera}_bbox_x2_px"] = self._fmt_record_float(getattr(detection, "x2", None), 3)
            row[f"{camera}_bbox_y2_px"] = self._fmt_record_float(getattr(detection, "y2", None), 3)
            row[f"{camera}_bbox_center_x_px"] = self._fmt_record_float(center_x, 3)
            row[f"{camera}_bbox_center_y_px"] = self._fmt_record_float(center_y, 3)
            row[f"{camera}_bbox_confidence"] = self._fmt_record_float(getattr(detection, "confidence", None), 6)
            row[f"{camera}_bbox_class_id"] = "" if getattr(detection, "class_id", None) is None else str(detection.class_id)
            row[f"{camera}_bbox_class_name"] = str(getattr(detection, "class_name", "") or "")
            row[f"{camera}_x1"] = self._fmt_record_float(x1, 3)
            row[f"{camera}_y1"] = self._fmt_record_float(y1, 3)
            row[f"{camera}_x2"] = self._fmt_record_float(x2, 3)
            row[f"{camera}_y2"] = self._fmt_record_float(y2, 3)
            row[f"{camera}_cx"] = self._fmt_record_float(center_x, 3)
            row[f"{camera}_cy"] = self._fmt_record_float(center_y, 3)
            row[f"{camera}_width_px"] = self._fmt_record_float(bbox_width, 3)
            row[f"{camera}_height_px"] = self._fmt_record_float(bbox_height, 3)
            row[f"{camera}_confidence"] = self._fmt_record_float(getattr(detection, "confidence", None), 6)
        return row

    def _detection_utc_unix_s(self, detection) -> float | None:
        metadata = dict(getattr(detection, "timestamp_metadata", None) or {})
        value = metadata.get("rtcp_frame_ntp_unix_s")
        if value is None:
            return None
        try:
            parsed = float(value)
        except (TypeError, ValueError):
            return None
        return parsed if isfinite(parsed) else None

    def _unix_seconds_to_ns(self, value: float | None) -> int | None:
        if value is None:
            return None
        try:
            return int(
                (Decimal(str(value)) * Decimal(1_000_000_000)).to_integral_value(
                    rounding=ROUND_HALF_UP
                )
            )
        except (InvalidOperation, ValueError):
            return None

    def _record_measurement_sync_timestamp_parameters(self, estimate) -> dict[str, object]:
        left_timestamp_s = float(getattr(estimate.left, "timestamp_s", 0.0))
        right_timestamp_s = float(getattr(estimate.right, "timestamp_s", 0.0))
        bbox_mid_timestamp_s = (left_timestamp_s + right_timestamp_s) / 2.0
        left_utc_unix_s = self._detection_utc_unix_s(estimate.left)
        right_utc_unix_s = self._detection_utc_unix_s(estimate.right)
        bbox_mid_utc_unix_s = (
            (left_utc_unix_s + right_utc_unix_s) / 2.0
            if left_utc_unix_s is not None and right_utc_unix_s is not None
            else None
        )
        left_utc_ns = self._unix_seconds_to_ns(left_utc_unix_s)
        right_utc_ns = self._unix_seconds_to_ns(right_utc_unix_s)
        pair_mid_utc_ns = (
            None if left_utc_ns is None or right_utc_ns is None else (left_utc_ns + right_utc_ns) // 2
        )
        rtcp_pair_delta_ns = (
            None if left_utc_ns is None or right_utc_ns is None else right_utc_ns - left_utc_ns
        )
        pair_limit_s = self._yolo_pair_max_delta_s()
        pair_limit_ns = (
            None
            if not isfinite(pair_limit_s)
            else int(round(pair_limit_s * 1_000_000_000.0))
        )
        camera_time_metrics = self._camera_time_sync_metrics(
            dict(getattr(estimate.left, "timestamp_metadata", None) or {}),
            dict(getattr(estimate.right, "timestamp_metadata", None) or {}),
            left_receive_mono_s=left_timestamp_s,
            right_receive_mono_s=right_timestamp_s,
        )
        row = {
            "left_bbox_timestamp_s": self._fmt_record_float(left_timestamp_s, 6),
            "right_bbox_timestamp_s": self._fmt_record_float(right_timestamp_s, 6),
            "bbox_mid_timestamp_s": self._fmt_record_float(bbox_mid_timestamp_s, 6),
            "left_bbox_utc_unix_s": self._fmt_record_float(left_utc_unix_s, 6),
            "right_bbox_utc_unix_s": self._fmt_record_float(right_utc_unix_s, 6),
            "bbox_mid_utc_unix_s": self._fmt_record_float(bbox_mid_utc_unix_s, 6),
            "left_frame_utc_ns": "" if left_utc_ns is None else left_utc_ns,
            "right_frame_utc_ns": "" if right_utc_ns is None else right_utc_ns,
            "pair_mid_utc_ns": "" if pair_mid_utc_ns is None else pair_mid_utc_ns,
            "rtcp_pair_delta_ns": "" if rtcp_pair_delta_ns is None else rtcp_pair_delta_ns,
            "pair_timing_limit_ns": "" if pair_limit_ns is None else pair_limit_ns,
            "pair_timing_valid": self._fmt_record_bool(
                rtcp_pair_delta_ns is not None
                and (pair_limit_ns is None or abs(rtcp_pair_delta_ns) <= pair_limit_ns)
            ),
            "left_bbox_timestamp_reference": str(
                (getattr(estimate.left, "timestamp_metadata", None) or {}).get("timestamp_reference") or "app-receive"
            ),
            "right_bbox_timestamp_reference": str(
                (getattr(estimate.right, "timestamp_metadata", None) or {}).get("timestamp_reference") or "app-receive"
            ),
            "camera_time_sync_status": str(camera_time_metrics["status"]),
            "camera_time_sync_basis": str(camera_time_metrics["basis"]),
            "camera_time_sync_warning": str(camera_time_metrics["warning"]),
            "rtcp_pair_delta_s": self._fmt_record_float(camera_time_metrics["pair_delta_s"], 6),
            "rtcp_pc_offset_max_ms": self._fmt_record_float(camera_time_metrics["max_pc_offset_ms"], 3),
            "rtcp_frame_transit_max_ms": self._fmt_record_float(camera_time_metrics["max_frame_transit_delay_ms"], 3),
            "left_rtcp_frame_transit_ms": self._fmt_record_float(camera_time_metrics["left_frame_transit_delay_ms"], 3),
            "right_rtcp_frame_transit_ms": self._fmt_record_float(camera_time_metrics["right_frame_transit_delay_ms"], 3),
            "left_rtcp_pc_offset_ms": self._fmt_record_float(camera_time_metrics["left_pc_offset_ms"], 3),
            "right_rtcp_pc_offset_ms": self._fmt_record_float(camera_time_metrics["right_pc_offset_ms"], 3),
        }
        return row

    def _record_frame_timestamp_parameters(
        self, measurement_attempt: MeasurementAttempt
    ) -> dict[str, object]:
        row: dict[str, object] = {}
        detections = {
            "left": measurement_attempt.left_detection,
            "right": measurement_attempt.right_detection,
        }
        for camera, detection in detections.items():
            metadata = detection.timestamp_metadata
            timestamp_s = detection.app_receive_monotonic_s
            source_timestamp = metadata.get("source_timestamp_s")
            timestamp_reference = infer_runtime_camera_timestamp_reference(metadata)
            row[f"{camera}_display_frame_timestamp_s"] = self._fmt_record_float(timestamp_s, 6)
            row[f"{camera}_source_frame_timestamp_s"] = self._fmt_record_float(
                None if source_timestamp is None else float(source_timestamp),
                6,
            )
            row[f"{camera}_frame_timestamp_source"] = str(metadata.get("timestamp_source") or "")
            row[f"{camera}_frame_timestamp_reference"] = timestamp_reference
            row[f"{camera}_rtp_timestamp"] = "" if metadata.get("rtp_timestamp") is None else str(int(metadata["rtp_timestamp"]))
            row[f"{camera}_rtp_ssrc"] = "" if metadata.get("rtp_ssrc") is None else str(int(metadata["rtp_ssrc"]))
            row[f"{camera}_rtcp_frame_ntp_unix_s"] = self._fmt_record_float(
                None if metadata.get("rtcp_frame_ntp_unix_s") is None else float(metadata["rtcp_frame_ntp_unix_s"]),
                6,
            )
            rtcp_frame_ntp_unix_s = metadata.get("rtcp_frame_ntp_unix_s")
            row[f"{camera}_rtcp_frame_ntp_unix_ns"] = (
                ""
                if rtcp_frame_ntp_unix_s is None
                else self._unix_seconds_to_ns(float(rtcp_frame_ntp_unix_s))
            )
            row[f"{camera}_app_receive_monotonic_ns"] = int(
                round(timestamp_s * 1_000_000_000.0)
            )
            row[f"{camera}_camera_rtc_frame_unix_s"] = row[f"{camera}_rtcp_frame_ntp_unix_s"]
            row[f"{camera}_rtcp_sr_ntp_unix_s"] = self._fmt_record_float(
                None if metadata.get("rtcp_sr_ntp_unix_s") is None else float(metadata["rtcp_sr_ntp_unix_s"]),
                6,
            )
            row[f"{camera}_camera_rtc_sr_unix_s"] = row[f"{camera}_rtcp_sr_ntp_unix_s"]
            row[f"{camera}_rtcp_sr_rtp_timestamp"] = (
                "" if metadata.get("rtcp_sr_rtp_timestamp") is None else str(int(metadata["rtcp_sr_rtp_timestamp"]))
            )
            row[f"{camera}_camera_rtc_basis"] = (
                "rtcp-sender-report"
                if metadata.get("rtcp_frame_ntp_unix_s") is not None
                else timestamp_reference
            )
        display_delta_s = (
            measurement_attempt.right_detection.app_receive_monotonic_s
            - measurement_attempt.left_detection.app_receive_monotonic_s
        )
        left_metadata = measurement_attempt.left_detection.timestamp_metadata
        right_metadata = measurement_attempt.right_detection.timestamp_metadata
        left_source = left_metadata.get("source_timestamp_s")
        right_source = right_metadata.get("source_timestamp_s")
        source_delta_s = (
            float(right_source) - float(left_source)
            if left_source is not None and right_source is not None
            else None
        )
        left_rtcp_ntp = left_metadata.get("rtcp_frame_ntp_unix_s")
        right_rtcp_ntp = right_metadata.get("rtcp_frame_ntp_unix_s")
        rtcp_ntp_delta_s = (
            float(right_rtcp_ntp) - float(left_rtcp_ntp)
            if left_rtcp_ntp is not None and right_rtcp_ntp is not None
            else None
        )
        row["display_frame_delta_s"] = self._fmt_record_float(display_delta_s, 6)
        row["source_frame_delta_s"] = self._fmt_record_float(source_delta_s, 6)
        row["rtcp_frame_ntp_delta_s"] = self._fmt_record_float(rtcp_ntp_delta_s, 6)
        row["camera_rtc_frame_delta_s"] = row["rtcp_frame_ntp_delta_s"]
        return row

    def _record_live_telemetry_snapshot_parameters(
        self, measurement_attempt: MeasurementAttempt
    ) -> dict[str, object]:
        row: dict[str, object] = {
            "attitude_frame": BODY_FRAME,
            "attitude_units": "deg",
            "gps_quality_frame": "gps_telemetry_metadata",
            "gps_quality_units": "satellites_count,eph_m,epv_m",
        }
        for camera in ("left", "right"):
            attitude = (
                measurement_attempt.left_attitude
                if camera == "left"
                else measurement_attempt.right_attitude
            )
            attitude_age_s = (
                measurement_attempt.left_attitude_age_s
                if camera == "left"
                else measurement_attempt.right_attitude_age_s
            )
            row[f"{camera}_roll_deg"] = self._fmt_record_float(None if attitude is None else attitude.roll_deg, 6)
            row[f"{camera}_pitch_deg"] = self._fmt_record_float(None if attitude is None else attitude.pitch_deg, 6)
            row[f"{camera}_yaw_deg"] = self._fmt_record_float(None if attitude is None else attitude.yaw_deg, 6)
            row[f"{camera}_attitude_sample_age_s"] = self._fmt_record_float(
                attitude_age_s,
                6,
            )
            row[f"{camera}_attitude_alignment"] = (
                "missing" if attitude is None else "nearest_app_receive_monotonic"
            )
            row[f"{camera}_attitude_topic"] = "" if attitude is None else str(attitude.topic)
            gps = measurement_attempt.left_gps if camera == "left" else measurement_attempt.right_gps
            gps_age_s = (
                None
                if gps is None
                else abs(
                    (
                        measurement_attempt.left_detection.app_receive_monotonic_s
                        if camera == "left"
                        else measurement_attempt.right_detection.app_receive_monotonic_s
                    )
                    - gps.app_receive_monotonic_s
                )
            )
            row[f"{camera}_gps_satellites_used"] = (
                "" if gps is None or getattr(gps, "satellites_used", None) is None else str(gps.satellites_used)
            )
            row[f"{camera}_gps_eph_m"] = self._fmt_record_float(
                None if gps is None else getattr(gps, "eph_m", None),
                6,
            )
            row[f"{camera}_gps_epv_m"] = self._fmt_record_float(
                None if gps is None else getattr(gps, "epv_m", None),
                6,
            )
            row[f"{camera}_gps_fix_type"] = "" if gps is None or getattr(gps, "fix_type", None) is None else str(gps.fix_type)
            row[f"{camera}_gps_sample_age_s"] = self._fmt_record_float(
                gps_age_s
                if measurement_attempt.baseline is None
                else (
                    measurement_attempt.baseline.left_age_at_capture_s
                    if camera == "left"
                    else measurement_attempt.baseline.right_age_at_capture_s
                ),
                6,
            )
            row[f"{camera}_gps_alignment"] = (
                "missing" if gps is None else "latest_selected_app_receive_monotonic"
            )
            row[f"{camera}_gps_topic"] = "" if gps is None else str(gps.topic)
        return row

    def _record_reference_delta_parameters(
        self, measurement_attempt: MeasurementAttempt
    ) -> dict[str, object]:
        row: dict[str, object] = {
            "gps_position_frame": "session_left_start_gps_antenna_ned",
            "gps_position_axis_contract": "north,east,down; metres",
            "attitude_orientation_contract": "px4_frd_to_ned; quaternion_wxyz",
        }
        attitudes = {
            "left": measurement_attempt.left_attitude,
            "right": measurement_attempt.right_attitude,
        }
        gps_states = {
            "left": measurement_attempt.left_gps,
            "right": measurement_attempt.right_gps,
        }
        for camera, attitude in attitudes.items():
            if attitude is None:
                continue
            reference = self.recording_reference_attitudes.setdefault(camera, attitude)
            delta = relative_attitude(reference.quaternion_wxyz, attitude.quaternion_wxyz)
            prefix = f"{camera}_pixhawk"
            row.update(
                {
                    f"{prefix}_reference_q_wxyz": json.dumps(
                        list(reference.quaternion_wxyz), separators=(",", ":")
                    ),
                    f"{prefix}_delta_roll_deg": self._fmt_record_float(delta.roll_deg, 9),
                    f"{prefix}_delta_pitch_deg": self._fmt_record_float(delta.pitch_deg, 9),
                    f"{prefix}_delta_yaw_deg": self._fmt_record_float(delta.yaw_deg, 9),
                    f"{prefix}_delta_rotation_angle_deg": self._fmt_record_float(
                        relative_rotation_angle_deg(reference.quaternion_wxyz, attitude.quaternion_wxyz), 9
                    ),
                }
            )

        if (
            attitudes["left"] is not None
            and attitudes["right"] is not None
            and "left" in self.recording_reference_attitudes
            and "right" in self.recording_reference_attitudes
        ):
            initial_relative_q = quaternion_multiply(
                self.recording_reference_attitudes["right"].quaternion_wxyz,
                quaternion_conjugate(self.recording_reference_attitudes["left"].quaternion_wxyz),
            )
            current_relative_q = quaternion_multiply(
                attitudes["right"].quaternion_wxyz,
                quaternion_conjugate(attitudes["left"].quaternion_wxyz),
            )
            row["left_right_relative_reference_q_wxyz"] = json.dumps(
                list(initial_relative_q), separators=(",", ":")
            )
            row["left_right_relative_q_wxyz"] = json.dumps(
                list(current_relative_q), separators=(",", ":")
            )
            row["left_right_relative_delta_rotation_angle_deg"] = self._fmt_record_float(
                relative_rotation_angle_deg(initial_relative_q, current_relative_q), 9
            )

        if self.recording_origin_gps is None and gps_states["left"] is not None:
            self.recording_origin_gps = gps_states["left"]
        for camera, gps in gps_states.items():
            if gps is None:
                continue
            reference = self.recording_reference_gps.setdefault(camera, gps)
            origin_delta = (
                None if self.recording_origin_gps is None else ned_delta_m(self.recording_origin_gps, gps)
            )
            reference_delta = ned_delta_m(reference, gps)
            prefix = f"{camera}_gps_antenna"
            if origin_delta is not None:
                row[f"{prefix}_north_m"] = self._fmt_record_float(origin_delta[0], 9)
                row[f"{prefix}_east_m"] = self._fmt_record_float(origin_delta[1], 9)
                row[f"{prefix}_down_m"] = self._fmt_record_float(origin_delta[2], 9)
            if reference_delta is not None:
                row[f"{prefix}_delta_north_m"] = self._fmt_record_float(reference_delta[0], 9)
                row[f"{prefix}_delta_east_m"] = self._fmt_record_float(reference_delta[1], 9)
                row[f"{prefix}_delta_down_m"] = self._fmt_record_float(reference_delta[2], 9)
                row[f"{prefix}_delta_position_m"] = self._fmt_record_float(
                    math.sqrt(sum(value * value for value in reference_delta)), 9
                )

        if gps_states["left"] is not None and gps_states["right"] is not None:
            baseline_ned = ned_delta_m(gps_states["left"], gps_states["right"])
            if baseline_ned is not None:
                if self.recording_reference_baseline_ned_m is None:
                    self.recording_reference_baseline_ned_m = baseline_ned
                baseline_delta = tuple(
                    value - reference
                    for value, reference in zip(baseline_ned, self.recording_reference_baseline_ned_m)
                )
                baseline_length_m = math.sqrt(sum(value * value for value in baseline_ned))
                reference_baseline_length_m = math.sqrt(
                    sum(value * value for value in self.recording_reference_baseline_ned_m)
                )
                row.update(
                    {
                        "baseline_north_m": self._fmt_record_float(baseline_ned[0], 9),
                        "baseline_east_m": self._fmt_record_float(baseline_ned[1], 9),
                        "baseline_down_m": self._fmt_record_float(baseline_ned[2], 9),
                        "baseline_length_m": self._fmt_record_float(baseline_length_m, 9),
                        "baseline_reference_length_m": self._fmt_record_float(
                            reference_baseline_length_m, 9
                        ),
                        "baseline_length_change_m": self._fmt_record_float(
                            baseline_length_m - reference_baseline_length_m, 9
                        ),
                        "baseline_delta_north_m": self._fmt_record_float(baseline_delta[0], 9),
                        "baseline_delta_east_m": self._fmt_record_float(baseline_delta[1], 9),
                        "baseline_delta_down_m": self._fmt_record_float(baseline_delta[2], 9),
                        "baseline_delta_length_m": self._fmt_record_float(
                            math.sqrt(sum(value * value for value in baseline_delta)), 9
                        ),
                    }
                )
        return row

    def _record_current_intrinsic_parameters(self) -> dict[str, str]:
        row: dict[str, str] = {
            "intrinsics_active": "true" if self.active_intrinsics is not None else "false",
            "intrinsics_path": self.settings.intrinsics_path,
            "intrinsics_zoom_level": str(self.zoom_level_spin.value()),
            "intrinsics_zoom_path": str(self._intrinsics_storage_path()),
            "intrinsics_latest_path": str(self._intrinsics_storage_base_path()),
            "manual_focal_override": self._fmt_record_bool(self.manual_focal_checkbox.isChecked()),
            "runtime_focal_px": self._fmt_record_float(self.focal_spin.value(), 3),
        }
        if self.active_intrinsics is None:
            return row
        for prefix, intrinsics in (("left", self.active_intrinsics.left), ("right", self.active_intrinsics.right)):
            matrix = intrinsics.camera_matrix
            row.update(
                {
                    f"{prefix}_intr_width": str(intrinsics.width),
                    f"{prefix}_intr_height": str(intrinsics.height),
                    f"{prefix}_intr_fx": self._fmt_record_float(matrix[0][0], 6),
                    f"{prefix}_intr_fy": self._fmt_record_float(matrix[1][1], 6),
                    f"{prefix}_intr_cx": self._fmt_record_float(matrix[0][2], 6),
                    f"{prefix}_intr_cy": self._fmt_record_float(matrix[1][2], 6),
                    f"{prefix}_intr_distortion": json.dumps(intrinsics.distortion_coefficients, separators=(",", ":")),
                    f"{prefix}_intr_reprojection_error_px": self._fmt_record_float(
                        intrinsics.reprojection_error_px, 6
                    ),
                    f"{prefix}_intr_image_count": str(intrinsics.image_count),
                }
            )
        return row

    def _record_intrinsic_parameters(
        self, measurement_attempt: MeasurementAttempt
    ) -> dict[str, str]:
        intrinsics_snapshot = measurement_attempt.intrinsics
        row: dict[str, str] = {
            "intrinsics_active": self._fmt_record_bool(intrinsics_snapshot is not None),
            "intrinsics_path": "" if intrinsics_snapshot is None else intrinsics_snapshot.selected_profile_path,
            "intrinsics_zoom_level": "" if intrinsics_snapshot is None else str(intrinsics_snapshot.active_zoom_level),
            "intrinsics_zoom_path": "" if intrinsics_snapshot is None else intrinsics_snapshot.selected_profile_path,
            "intrinsics_latest_path": "" if intrinsics_snapshot is None else intrinsics_snapshot.selected_profile_path,
            "intrinsics_content_sha256": "" if intrinsics_snapshot is None else intrinsics_snapshot.content_sha256,
            "manual_focal_override": self._fmt_record_bool(
                measurement_attempt.geometry_config.manual_focal_override
            ),
            "runtime_focal_px": self._fmt_record_float(
                measurement_attempt.geometry_config.focal_length_px, 3
            ),
        }
        if intrinsics_snapshot is None:
            return row
        for prefix, intrinsics in (
            ("left", intrinsics_snapshot.left),
            ("right", intrinsics_snapshot.right),
        ):
            matrix = intrinsics.camera_matrix
            row.update(
                {
                    f"{prefix}_intr_width": str(intrinsics.width),
                    f"{prefix}_intr_height": str(intrinsics.height),
                    f"{prefix}_intr_fx": self._fmt_record_float(matrix[0][0], 6),
                    f"{prefix}_intr_fy": self._fmt_record_float(matrix[1][1], 6),
                    f"{prefix}_intr_cx": self._fmt_record_float(matrix[0][2], 6),
                    f"{prefix}_intr_cy": self._fmt_record_float(matrix[1][2], 6),
                    f"{prefix}_intr_distortion": json.dumps(
                        intrinsics.distortion_coefficients, separators=(",", ":")
                    ),
                    f"{prefix}_intr_reprojection_error_px": self._fmt_record_float(
                        intrinsics.reprojection_error_px, 6
                    ),
                    f"{prefix}_intr_image_count": str(intrinsics.image_count),
                }
            )
        return row

    def _record_current_checkerboard_extrinsics_parameters(self) -> dict[str, object]:
        calibration = self.active_intrinsics
        extrinsics = None if calibration is None else calibration.extrinsics
        row: dict[str, object] = {
            "active": extrinsics is not None,
            "source": "applied_intrinsics",
            "frame_contract": "opencv_optical:RDF(right,down,forward); X_right = R_left_to_right * X_left + T_left_to_right_m",
        }
        if extrinsics is None:
            return row
        right_origin = self._checkerboard_translation_right_in_left_m()
        row.update(
            {
                "pair_count": int(extrinsics.pair_count),
                "reprojection_error_px": float(extrinsics.reprojection_error_px),
                "rotation_left_to_right": [list(map(float, values)) for values in extrinsics.rotation_left_to_right],
                "translation_left_to_right_m": [float(value) for value in extrinsics.translation_left_to_right_m],
                "right_camera_origin_in_left_rdf_m": (
                    None if right_origin is None else [float(value) for value in right_origin]
                ),
            }
        )
        return row

    def _record_checkerboard_extrinsics_parameters(
        self, measurement_attempt: MeasurementAttempt
    ) -> dict[str, object]:
        calibration = measurement_attempt.intrinsics
        extrinsics = None if calibration is None else calibration.checkerboard_extrinsics
        row: dict[str, object] = {
            "active": extrinsics is not None,
            "source": "measurement_attempt",
            "frame_contract": "opencv_optical:RDF(right,down,forward); X_right = R_left_to_right * X_left + T_left_to_right_m",
        }
        if extrinsics is None:
            return row
        right_origin = measurement_attempt.translation_right_in_left_m
        row.update(
            {
                "pair_count": extrinsics.pair_count,
                "reprojection_error_px": extrinsics.reprojection_error_px,
                "rotation_left_to_right": [list(values) for values in extrinsics.rotation_left_to_right],
                "translation_left_to_right_m": list(extrinsics.translation_left_to_right_m),
                "right_camera_origin_in_left_rdf_m": (
                    None if right_origin is None else list(right_origin)
                ),
            }
        )
        return row

    def _record_external_parameters(
        self, *, estimate=None, measurement_attempt: MeasurementAttempt
    ) -> dict[str, str]:
        triangulation_source = measurement_attempt.triangulation_source
        checkerboard_mode = triangulation_source == "checkerboard"
        attitude_timing_status = classify_attitude_timing(
            uses_pixhawk_attitude=not checkerboard_mode,
            left_age_s=(
                measurement_attempt.left_attitude_age_s
                if estimate is None
                else estimate.left_attitude_age_s
            ),
            right_age_s=(
                measurement_attempt.right_attitude_age_s
                if estimate is None
                else estimate.right_attitude_age_s
            ),
            pair_delta_s=(
                measurement_attempt.attitude_pair_delta_s
                if estimate is None
                else estimate.attitude_pair_delta_s
            ),
        )
        checkerboard = self._record_checkerboard_extrinsics_parameters(measurement_attempt)
        row: dict[str, str] = {
            "triangulation_extrinsics_source": triangulation_source,
            "use_rtk_baseline": self._fmt_record_bool(not checkerboard_mode),
            "rtk_baseline_applied": self._fmt_record_bool(not checkerboard_mode),
            "require_rtk_fixed": self._fmt_record_bool(
                False if checkerboard_mode else measurement_attempt.geometry_config.require_rtk_fixed_setting
            ),
            "use_triangulation": self._fmt_record_bool(True),
            "distance_mode": (
                "체커보드 Stereo 외부 파라미터"
                if checkerboard_mode
                else "Pixhawk 자세 + GPS/RTK baseline"
            ),
            "world_frame": WORLD_FRAME,
            "body_frame": BODY_FRAME,
            "camera_frame": CAMERA_FRAME,
            "camera_mount_profile": CAMERA_MOUNT_PROFILE,
            "mount_assumption_model": (
                "checkerboard-stereo-extrinsics-v1"
                if checkerboard_mode
                else "gps-rtk-baseline-only-v1"
            ),
            "mount_assumption_status": (
                "checkerboard stereo extrinsics" if checkerboard_mode else "GPS/RTK baseline only"
            ),
            "mount_assumption_detail": (
                "applied_checkerboard_stereo_extrinsics_used_as_fixed_optical_transform"
                if checkerboard_mode
                else "live_gps_rtk_baseline_used_as_stereo_translation"
            ),
            "checkerboard_extrinsics_active": self._fmt_record_bool(bool(checkerboard["active"])),
            "checkerboard_extrinsics_pair_count": str(checkerboard.get("pair_count", "")),
            "checkerboard_extrinsics_reprojection_error_px": self._fmt_record_float(
                checkerboard.get("reprojection_error_px"), 6
            ),
            "checkerboard_extrinsics_frame_contract": str(checkerboard["frame_contract"]),
            "require_attitude": self._fmt_record_bool(False),
            "attitude_used_for_triangulation": self._fmt_record_bool(not checkerboard_mode),
            "attitude_timing_policy": ATTITUDE_TIMING_POLICY,
            "attitude_timing_status": attitude_timing_status,
            "attitude_timing_blocks_measurement": self._fmt_record_bool(False),
            "attitude_stale_age_threshold_s": self._fmt_record_float(ATTITUDE_STALE_AGE_S, 3),
            "attitude_very_stale_age_threshold_s": self._fmt_record_float(
                ATTITUDE_VERY_STALE_AGE_S, 3
            ),
            "attitude_stale_pair_delta_threshold_s": self._fmt_record_float(
                ATTITUDE_STALE_PAIR_DELTA_S, 3
            ),
            "attitude_very_stale_pair_delta_threshold_s": self._fmt_record_float(
                ATTITUDE_VERY_STALE_PAIR_DELTA_S, 3
            ),
            "left_attitude_age_s": self._fmt_record_float(
                measurement_attempt.left_attitude_age_s if estimate is None else estimate.left_attitude_age_s, 6
            ),
            "right_attitude_age_s": self._fmt_record_float(
                measurement_attempt.right_attitude_age_s if estimate is None else estimate.right_attitude_age_s, 6
            ),
            "attitude_pair_delta_s": self._fmt_record_float(
                measurement_attempt.attitude_pair_delta_s if estimate is None else estimate.attitude_pair_delta_s, 6
            ),
            "relative_roll_deg": self._fmt_record_float(
                measurement_attempt.relative_roll_deg if estimate is None else estimate.relative_roll_deg, 6
            ),
            "relative_pitch_deg": self._fmt_record_float(
                measurement_attempt.relative_pitch_deg if estimate is None else estimate.relative_pitch_deg, 6
            ),
            "relative_yaw_deg": self._fmt_record_float(
                measurement_attempt.relative_yaw_deg if estimate is None else estimate.relative_yaw_deg, 6
            ),
        }
        baseline = measurement_attempt.baseline
        if baseline is not None:
            camera_translation = baseline.translation_right_in_left_rdf_m
            baseline_forward_geometry = self._baseline_forward_geometry_from_camera_translation(camera_translation)
            ned = baseline.ned_right_from_left_m
            row.update(
                {
                    "rtk_baseline_north_m": self._fmt_record_float(None if ned is None else ned[0], 6),
                    "rtk_baseline_east_m": self._fmt_record_float(None if ned is None else ned[1], 6),
                    "rtk_baseline_down_m": self._fmt_record_float(None if ned is None else ned[2], 6),
                    "rtk_left_age_s": self._fmt_record_float(baseline.left_age_at_capture_s, 6),
                    "rtk_right_age_s": self._fmt_record_float(baseline.right_age_at_capture_s, 6),
                    "rtk_fix_quality": baseline.fix_quality,
                    "rtk_warnings": "; ".join(baseline.warnings),
                    "selected_gps_pair_delta_s": self._fmt_record_float(baseline.gps_pair_delta_s, 6),
                    "selected_left_gps_detection_age_s": self._fmt_record_float(
                        baseline.left_age_from_detection_midpoint_s, 6
                    ),
                    "selected_right_gps_detection_age_s": self._fmt_record_float(
                        baseline.right_age_from_detection_midpoint_s, 6
                    ),
                }
            )
            if camera_translation is not None:
                row.update(
                    {
                        "baseline_left_camera_rdf_right_m": self._fmt_record_float(camera_translation[0], 6),
                        "baseline_left_camera_rdf_down_m": self._fmt_record_float(camera_translation[1], 6),
                        "baseline_left_camera_rdf_forward_m": self._fmt_record_float(camera_translation[2], 6),
                    }
                )
            if baseline_forward_geometry is not None:
                angle_deg, effective_m, length_m, forward_m = baseline_forward_geometry
                row.update(
                    {
                        "baseline_rdf_forward_angle_deg": self._fmt_record_float(angle_deg, 6),
                        "baseline_effective_perpendicular_m": self._fmt_record_float(effective_m, 6),
                        "baseline_camera_frame_length_m": self._fmt_record_float(length_m, 6),
                        "baseline_forward_component_m": self._fmt_record_float(forward_m, 6),
                    }
                )
        for camera in ("left", "right"):
            attitude = (
                measurement_attempt.left_attitude
                if camera == "left"
                else measurement_attempt.right_attitude
            )
            attitude_age_s = (
                measurement_attempt.left_attitude_age_s
                if camera == "left"
                else measurement_attempt.right_attitude_age_s
            )
            gps = measurement_attempt.left_gps if camera == "left" else measurement_attempt.right_gps
            gps_age_s = None
            if measurement_attempt.baseline is not None:
                gps_age_s = (
                    measurement_attempt.baseline.left_age_at_capture_s
                    if camera == "left"
                    else measurement_attempt.baseline.right_age_at_capture_s
                )
            if gps_age_s is None and gps is not None:
                frame_timestamp_s = (
                    measurement_attempt.left_detection.app_receive_monotonic_s
                    if camera == "left"
                    else measurement_attempt.right_detection.app_receive_monotonic_s
                )
                gps_age_s = abs(frame_timestamp_s - gps.app_receive_monotonic_s)
            row.update(self._record_extrinsic_state(camera, attitude, attitude_age_s))
            row.update(self._record_gps_state(camera, gps, gps_age_s))
            row.update(self._record_gnss_relative_state(camera))
        return row

    def _record_extrinsic_state(
        self,
        camera: str,
        state: AttitudeSnapshot | None,
        age_s: float | None,
    ) -> dict[str, str]:
        prefix = f"{camera}_pixhawk"
        if state is None:
            return {}
        return {
            f"{prefix}_namespace": state.namespace,
            f"{prefix}_topic": state.topic,
            f"{prefix}_roll_deg": self._fmt_record_float(state.roll_deg, 6),
            f"{prefix}_pitch_deg": self._fmt_record_float(state.pitch_deg, 6),
            f"{prefix}_yaw_deg": self._fmt_record_float(state.yaw_deg, 6),
            f"{prefix}_q": json.dumps(list(state.quaternion_wxyz), separators=(",", ":")),
            f"{prefix}_q_w": self._fmt_record_float(state.quaternion_wxyz[0], 9),
            f"{prefix}_q_x": self._fmt_record_float(state.quaternion_wxyz[1], 9),
            f"{prefix}_q_y": self._fmt_record_float(state.quaternion_wxyz[2], 9),
            f"{prefix}_q_z": self._fmt_record_float(state.quaternion_wxyz[3], 9),
            f"{prefix}_age_s": self._fmt_record_float(age_s, 6),
            f"{prefix}_app_receive_timestamp_s": self._fmt_record_float(state.timestamp_s, 6),
            f"{prefix}_app_receive_monotonic_ns": int(round(state.timestamp_s * 1_000_000_000.0)),
            f"{prefix}_px4_timestamp_s": self._fmt_record_float(state.px4_timestamp_s, 6),
            f"{prefix}_px4_timestamp_us": "" if state.px4_timestamp_us is None else state.px4_timestamp_us,
            f"{prefix}_px4_timestamp_sample_us": (
                "" if state.px4_timestamp_sample_us is None else state.px4_timestamp_sample_us
            ),
            f"{prefix}_delta_q_reset_wxyz": (
                "" if state.delta_q_reset is None else json.dumps(list(state.delta_q_reset), separators=(",", ":"))
            ),
            f"{prefix}_quat_reset_counter": (
                "" if state.quat_reset_counter is None else state.quat_reset_counter
            ),
        }

    def _record_gps_state(
        self,
        camera: str,
        state: GpsSnapshot | None,
        age_s: float | None,
    ) -> dict[str, str]:
        prefix = f"{camera}_gps"
        if state is None:
            return {}
        return {
            f"{prefix}_namespace": state.namespace,
            f"{prefix}_topic": state.topic,
            f"{prefix}_lat_deg": self._fmt_record_float(state.latitude_deg, 9),
            f"{prefix}_lon_deg": self._fmt_record_float(state.longitude_deg, 9),
            f"{prefix}_alt_m": self._fmt_record_float(state.altitude_m, 6),
            f"{prefix}_altitude_ellipsoid_m": self._fmt_record_float(state.altitude_ellipsoid_m, 6),
            f"{prefix}_fix_type": "" if state.fix_type is None else str(state.fix_type),
            f"{prefix}_satellites": "" if state.satellites_used is None else str(state.satellites_used),
            f"{prefix}_eph_m": self._fmt_record_float(state.eph_m, 6),
            f"{prefix}_epv_m": self._fmt_record_float(state.epv_m, 6),
            f"{prefix}_heading_deg": self._fmt_record_float(state.heading_deg, 6),
            f"{prefix}_heading_accuracy_deg": self._fmt_record_float(state.heading_accuracy_deg, 6),
            f"{prefix}_rtk_label": state.rtk_label,
            f"{prefix}_age_s": self._fmt_record_float(age_s, 6),
            f"{prefix}_app_receive_timestamp_s": self._fmt_record_float(state.timestamp_s, 6),
            f"{prefix}_app_receive_monotonic_ns": int(round(state.timestamp_s * 1_000_000_000.0)),
            f"{prefix}_px4_timestamp_s": self._fmt_record_float(state.px4_timestamp_s, 6),
            f"{prefix}_px4_timestamp_us": "" if state.px4_timestamp_us is None else state.px4_timestamp_us,
            f"{prefix}_px4_timestamp_sample_us": (
                "" if state.px4_timestamp_sample_us is None else state.px4_timestamp_sample_us
            ),
            f"{prefix}_gps_utc_us": "" if state.gps_utc_us is None else state.gps_utc_us,
            f"{prefix}_gps_utc_valid": self._fmt_record_bool(state.gps_utc_us is not None),
            f"{prefix}_timestamp_time_relative_us": (
                ""
                if state.gps_timestamp_time_relative_us is None
                else state.gps_timestamp_time_relative_us
            ),
            f"{prefix}_velocity_n_m_s": self._fmt_record_float(state.velocity_n_m_s, 6),
            f"{prefix}_velocity_e_m_s": self._fmt_record_float(state.velocity_e_m_s, 6),
            f"{prefix}_velocity_d_m_s": self._fmt_record_float(state.velocity_d_m_s, 6),
            f"{prefix}_velocity_ned_valid": self._fmt_record_bool(state.velocity_ned_valid),
            f"{prefix}_hdop": self._fmt_record_float(state.hdop, 6),
            f"{prefix}_vdop": self._fmt_record_float(state.vdop, 6),
            f"{prefix}_device_id": "" if state.device_id is None else state.device_id,
            f"{prefix}_time_source": state.time_source,
        }

    def _record_gnss_relative_state(self, camera: str) -> dict[str, str]:
        state = self.latest_gnss_relative.get(camera)
        prefix = f"{camera}_gnss_relative"
        if state is None:
            return {}
        age_s = max(0.0, time.monotonic() - state.timestamp_s)
        heading_deg = None if state.heading_rad is None else math.degrees(state.heading_rad)
        heading_accuracy_deg = None if state.heading_accuracy_rad is None else math.degrees(state.heading_accuracy_rad)
        return {
            f"{prefix}_namespace": state.namespace,
            f"{prefix}_topic": state.topic,
            f"{prefix}_device_id": "" if state.device_id is None else str(state.device_id),
            f"{prefix}_heading_deg": self._fmt_record_float(heading_deg, 6),
            f"{prefix}_heading_accuracy_deg": self._fmt_record_float(heading_accuracy_deg, 6),
            f"{prefix}_position_length_m": self._fmt_record_float(state.position_length_m, 6),
            f"{prefix}_accuracy_length_m": self._fmt_record_float(state.accuracy_length_m, 6),
            f"{prefix}_gnss_fix_ok": self._fmt_record_bool(state.gnss_fix_ok),
            f"{prefix}_differential_solution": self._fmt_record_bool(state.differential_solution),
            f"{prefix}_relative_position_valid": self._fmt_record_bool(state.relative_position_valid),
            f"{prefix}_carrier_solution_floating": self._fmt_record_bool(state.carrier_solution_floating),
            f"{prefix}_carrier_solution_fixed": self._fmt_record_bool(state.carrier_solution_fixed),
            f"{prefix}_moving_base_mode": self._fmt_record_bool(state.moving_base_mode),
            f"{prefix}_heading_valid": self._fmt_record_bool(state.heading_valid),
            f"{prefix}_age_s": self._fmt_record_float(age_s, 6),
            f"{prefix}_px4_timestamp_s": self._fmt_record_float(state.px4_timestamp_s, 6),
            f"{prefix}_px4_timestamp_us": "" if state.px4_timestamp_us is None else state.px4_timestamp_us,
            f"{prefix}_px4_timestamp_sample_us": (
                "" if state.px4_timestamp_sample_us is None else state.px4_timestamp_sample_us
            ),
            f"{prefix}_gps_utc_us": "" if state.gps_utc_us is None else state.gps_utc_us,
            f"{prefix}_gps_utc_valid": self._fmt_record_bool(state.gps_utc_us is not None),
        }

    def _format_distance_label_text(self, text: str, detail: str | None = None) -> str:
        if not detail:
            return text
        return (
            "<div style='line-height: 1.1;'>"
            f"{html_lib.escape(text)}"
            "<br>"
            f"<span style='font-size: 9px; font-weight: 400;'>{html_lib.escape(detail)}</span>"
            "</div>"
        )

    def _format_quality_point(self, point: tuple[float, float, float] | None) -> str:
        if point is None:
            return "--"
        return f"Right={point[0]:+.3f} m, Down={point[1]:+.3f} m, Forward={point[2]:+.3f} m"

    def _format_attitude_timing_age(self, value_s: float | None) -> str:
        return "--" if value_s is None else f"{value_s:.3f} s"

    def _attitude_timing_health_level(self, status: str) -> str:
        return {
            "FRESH": "good",
            "STALE": "warn",
            "VERY_STALE": "bad",
            "WAITING": "warn",
            "NOT_USED": "idle",
        }.get(status, "idle")

    def _format_attitude_timing_text(
        self,
        *,
        status: str,
        left_age_s: float | None,
        right_age_s: float | None,
        pair_delta_s: float | None,
        baseline_source: str,
        baseline_m: float,
    ) -> str:
        baseline_text = f"Baseline {baseline_source}:{baseline_m:.3f} m"
        if status == "NOT_USED":
            return f"Attitude timing [N/A] | {baseline_text} | PIXHAWK ATTITUDE NOT USED (checkerboard)"
        return (
            f"Attitude timing [{status}] | {baseline_text} | "
            f"Left age {self._format_attitude_timing_age(left_age_s)} | "
            f"Right age {self._format_attitude_timing_age(right_age_s)} | "
            f"Pair Δt {self._format_attitude_timing_age(pair_delta_s)} | "
            "display-only; measurement continues"
        )

    def _clear_triangulation_quality(self, reason: str) -> None:
        if not hasattr(self, "triangulation_quality_status_label"):
            return
        self.triangulation_quality_status_label.setText(reason)
        self.triangulation_quality_gap_label.setText("gap: --")
        self.triangulation_quality_distance_label.setText("distance: --")
        self.triangulation_quality_mode_label.setText("mode: --")
        self._set_health_label(self.triangulation_quality_attitude_label, "Attitude timing: --", "idle")
        self.triangulation_quality_raw_point_label.setText("Raw baseline_center_frd point: --")
        self.triangulation_quality_plot.set_quality(None)

    def _update_triangulation_quality(
        self,
        *,
        triangulated: RayTriangulationResult,
        method_text: str,
        display_distance_m: float,
        display_baseline_source: str,
        display_baseline_m: float,
        public_point_m: tuple[float, float, float] | None,
        source_point_left_rdf_m: tuple[float, float, float] | None,
        attitude_timing_status: str,
        left_attitude_age_s: float | None,
        right_attitude_age_s: float | None,
        attitude_pair_delta_s: float | None,
    ) -> None:
        if not hasattr(self, "triangulation_quality_status_label"):
            return
        self.triangulation_quality_status_label.setText(
            "표시 전용: 현재 좌/우 bbox ray의 최단거리 gap과 3D midpoint를 보여줍니다."
        )
        self.triangulation_quality_gap_label.setText(f"gap: {triangulated.closest_gap_m:.3f} m")
        self.triangulation_quality_distance_label.setText(f"distance: {display_distance_m:.3f} m")
        self.triangulation_quality_mode_label.setText(f"mode: {method_text}")
        self._set_health_label(
            self.triangulation_quality_attitude_label,
            self._format_attitude_timing_text(
                status=attitude_timing_status,
                left_age_s=left_attitude_age_s,
                right_age_s=right_attitude_age_s,
                pair_delta_s=attitude_pair_delta_s,
                baseline_source=display_baseline_source,
                baseline_m=display_baseline_m,
            ),
            self._attitude_timing_health_level(attitude_timing_status),
        )
        self.triangulation_quality_raw_point_label.setText(
            "Raw baseline_center_frd point [F/R/D]: "
            f"{self._format_quality_point(public_point_m)} | origin=baseline midpoint "
            "| source left_camera_rdf [R/D/F]: "
            f"{self._format_quality_point(source_point_left_rdf_m)}"
        )
        self.triangulation_quality_plot.set_quality(
            {
                "closest_gap_m": triangulated.closest_gap_m,
                "distance_m": display_distance_m,
                "method": method_text,
            }
        )

    def _set_distance_text(self, text: str, *, quality: str = "unknown", detail: str | None = None) -> None:
        self.last_distance_status = text
        self.last_distance_quality = quality
        if hasattr(self, "distance_plot"):
            self.distance_plot.set_empty_message(detail or text)
        if detail:
            self.distance_label.setTextFormat(QtCore.Qt.RichText)
        else:
            self.distance_label.setTextFormat(QtCore.Qt.PlainText)
        self.distance_label.setText(self._format_distance_label_text(text, detail))
        style = self._distance_style(quality)
        self.distance_label.setStyleSheet(
            "font-size: 14px; font-weight: 700; padding: 8px 10px; "
            f"border: 1px solid {PANEL_BORDER}; border-radius: 0px; " + style
        )
        self._refresh_operation_header()

    def _distance_style(self, quality: str) -> str:
        if quality == "GOOD":
            return f"background: {SUCCESS_BG}; color: {SUCCESS_TEXT}; border-color: {SUCCESS};"
        if quality == "DEGRADED":
            return f"background: {WARNING_BG}; color: {WARNING_TEXT}; border-color: {WARNING};"
        if quality == "BAD":
            return f"background: {ERROR_BG}; color: {ERROR_TEXT}; border-color: {ERROR};"
        return f"background: {PANEL_ALT_BG}; color: {TEXT_MUTED}; border-color: {PANEL_BORDER};"

    def _update_extrinsic(self, camera: str, extrinsic: CameraExtrinsicState) -> None:
        age_ms = max(0.0, time.monotonic() - extrinsic.timestamp_s) * 1000.0
        px4_text = f", px4_t={extrinsic.px4_timestamp_s:.3f}" if extrinsic.px4_timestamp_s is not None else ""
        text = (
            f"{camera}/namespace={extrinsic.namespace}: "
            f"roll={extrinsic.roll_deg:.4f}°, pitch={extrinsic.pitch_deg:.4f}°, "
            f"yaw={extrinsic.yaw_deg:.4f}°, age={age_ms:.0f}ms{px4_text}"
        )
        self.latest_extrinsics[camera] = extrinsic
        self.attitude_monitor.update(camera, extrinsic)
        self._record_external_timeline_row(
            camera,
            "pixhawk_attitude",
            {
                "namespace": extrinsic.namespace,
                "topic": extrinsic.topic,
                "sample_app_timestamp_s": extrinsic.timestamp_s,
                "sample_px4_timestamp_s": self._fmt_record_float(extrinsic.px4_timestamp_s, 6),
                "px4_timestamp_us": (
                    "" if extrinsic.px4_timestamp_us is None else extrinsic.px4_timestamp_us
                ),
                "px4_timestamp_sample_us": (
                    ""
                    if extrinsic.px4_timestamp_sample_us is None
                    else extrinsic.px4_timestamp_sample_us
                ),
                "roll_deg": self._fmt_record_float(extrinsic.roll_deg, 6),
                "pitch_deg": self._fmt_record_float(extrinsic.pitch_deg, 6),
                "yaw_deg": self._fmt_record_float(extrinsic.yaw_deg, 6),
                "q": json.dumps(list(extrinsic.q), separators=(",", ":")),
                "q_w": self._fmt_record_float(extrinsic.q[0], 9),
                "q_x": self._fmt_record_float(extrinsic.q[1], 9),
                "q_y": self._fmt_record_float(extrinsic.q[2], 9),
                "q_z": self._fmt_record_float(extrinsic.q[3], 9),
                "orientation_contract": "PX4 vehicle attitude quaternion rotates body FRD to world NED",
                "delta_q_reset_wxyz": (
                    ""
                    if extrinsic.delta_q_reset is None
                    else json.dumps(list(extrinsic.delta_q_reset), separators=(",", ":"))
                ),
                "quat_reset_counter": (
                    "" if extrinsic.quat_reset_counter is None else extrinsic.quat_reset_counter
                ),
                "age_s": self._fmt_record_float(max(0.0, time.monotonic() - extrinsic.timestamp_s), 6),
            },
        )
        self.attitude_buffers[camera].add(extrinsic)
        self.pixhawk_attitude_times[camera].append(time.monotonic())
        if hasattr(self, "signal_attitude_labels"):
            source = self._telemetry_source_label(extrinsic.topic)
            self.signal_attitude_labels[camera].setText(
                f"Attitude: {source} | roll={extrinsic.roll_deg:.2f} pitch={extrinsic.pitch_deg:.2f} "
                f"yaw={extrinsic.yaw_deg:.2f} | age={age_ms:.0f}ms{px4_text}"
            )
        if hasattr(self, "signal_attitude_age_plot"):
            self.signal_attitude_age_plot.add_point(TelemetryHistoryPoint(time.monotonic(), camera, age_ms))
        self._update_pixhawk_connection_status()
        self._update_relative_attitude_status()
        self._refresh_rtk_alignment_view()
        self._refresh_alignment_attitude_ui()
        self._refresh_alignment_overview()
        self._refresh_gnss_base_tab()
        self._refresh_alignment_validation()
        self._refresh_camera_parameters_tab()
        self._refresh_distance()


    def _update_gps(self, camera: str, gps: GpsPositionState) -> None:
        self.latest_gps[camera] = gps
        if camera in {"left", "right"}:
            self.gnss_monitor.update_position(camera, gps)
        if hasattr(self, "baseline_gps_rate_history") and camera in self.baseline_gps_rate_history:
            self.baseline_gps_rate_history[camera].append(time.monotonic())
        self._record_external_timeline_row(
            camera,
            "gps_rtk",
            {
                "namespace": gps.namespace,
                "topic": gps.topic,
                "sample_app_timestamp_s": gps.timestamp_s,
                "sample_px4_timestamp_s": self._fmt_record_float(gps.px4_timestamp_s, 6),
                "px4_timestamp_us": "" if gps.px4_timestamp_us is None else gps.px4_timestamp_us,
                "px4_timestamp_sample_us": (
                    "" if gps.px4_timestamp_sample_us is None else gps.px4_timestamp_sample_us
                ),
                "gps_utc_us": "" if gps.gps_utc_us is None else gps.gps_utc_us,
                "gps_timestamp_time_relative_us": (
                    ""
                    if gps.gps_timestamp_time_relative_us is None
                    else gps.gps_timestamp_time_relative_us
                ),
                "time_source": gps.time_source,
                "source_utc_method": (
                    gps.time_source
                    if gps.time_source.startswith("mavlink_")
                    else "px4_vehicle_gps_position_time_utc_usec"
                ),
                "lat_deg": self._fmt_record_float(gps.latitude_deg, 9),
                "lon_deg": self._fmt_record_float(gps.longitude_deg, 9),
                "alt_m": self._fmt_record_float(gps.altitude_m, 6),
                "altitude_ellipsoid_m": self._fmt_record_float(gps.altitude_ellipsoid_m, 6),
                "fix_type": "" if gps.fix_type is None else str(gps.fix_type),
                "satellites": "" if gps.satellites_used is None else str(gps.satellites_used),
                "eph_m": self._fmt_record_float(gps.eph_m, 6),
                "epv_m": self._fmt_record_float(gps.epv_m, 6),
                "heading_deg": self._fmt_record_float(gps.heading_deg, 6),
                "heading_accuracy_deg": self._fmt_record_float(gps.heading_accuracy_deg, 6),
                "velocity_n_m_s": self._fmt_record_float(gps.velocity_n_m_s, 6),
                "velocity_e_m_s": self._fmt_record_float(gps.velocity_e_m_s, 6),
                "velocity_d_m_s": self._fmt_record_float(gps.velocity_d_m_s, 6),
                "velocity_ned_valid": self._fmt_record_bool(gps.velocity_ned_valid),
                "position_contract": "WGS84 latitude/longitude; MSL and ellipsoid altitude fields explicit",
                "velocity_contract": "world NED north/east/down metres_per_second",
                "rtk_label": gps.rtk_label,
                "age_s": self._fmt_record_float(max(0.0, time.monotonic() - gps.timestamp_s), 6),
            },
        )
        age_ms = max(0.0, time.monotonic() - gps.timestamp_s) * 1000.0
        accuracy_text = []
        if gps.satellites_used is not None:
            accuracy_text.append(f"sats={gps.satellites_used}")
        if gps.eph_m is not None:
            accuracy_text.append(f"eph={gps.eph_m:.2f}m")
        if gps.heading_deg is not None:
            accuracy_text.append(f"gpsYaw={gps.heading_deg:.2f}°")
        extra = " ".join(accuracy_text)
        source = self._telemetry_source_label(gps.topic)
        text = f"{camera} GPS/RTK: {source} | {gps.rtk_label}, age={age_ms:.0f}ms, {extra}"
        if camera == "left":
            self._set_health_label(self.left_gps_label, text, self._gps_fix_level(gps.fix_type, age_ms=age_ms))
        elif camera == "right":
            self._set_health_label(self.right_gps_label, text, self._gps_fix_level(gps.fix_type, age_ms=age_ms))
        if hasattr(self, "signal_gps_position_labels") and camera in self.signal_gps_position_labels:
            self._set_health_label(
                self.signal_gps_position_labels[camera],
                f"GPS position: {self._telemetry_source_label(gps.topic)} | {gps.rtk_label} | "
                f"{self._gps_position_summary_text(gps, age_ms=age_ms)}",
                self._gps_fix_level(gps.fix_type, age_ms=age_ms),
            )
        self._refresh_rtk_baseline()
        self._refresh_baseline_alignment_ui()
        self._refresh_gnss_base_tab()
        self._append_baseline_sample_if_active()
        self._refresh_camera_parameters_tab()
        self._refresh_distance()

    def _update_mag_sensor(self, camera: str, state) -> None:
        topic = str(getattr(state, "topic", ""))
        if "vehicle_magnetometer" in topic and getattr(state, "device_id", None) is not None:
            self.latest_vehicle_mag_device_ids[camera] = int(state.device_id)
            self.refresh_px4_mag_manager(camera)
        self.latest_mag_sensors.setdefault(camera, {})[state.sensor_slot] = state
        if topic.startswith("mavlink/SCALED_IMU"):
            table = self.px4_mag_tables.get(camera) if hasattr(self, "px4_mag_tables") else None
            if table is not None and table.rowCount() == 0 and self._mag_state_display_device_id(camera, state)[0] is not None:
                self.refresh_px4_mag_manager(camera)
        elif "sensor_mag" in topic and self.latest_vehicle_mag_device_ids.get(camera) is None:
            status_label = self.px4_mag_status_labels.get(camera) if hasattr(self, "px4_mag_status_labels") else None
            if status_label is not None and "ROS2 raw MAG 수신 중" not in status_label.text():
                self.refresh_px4_mag_manager(camera)
        self.magnetometer_monitor.update(camera, state)
        label = self.mag_sensor_labels.get(camera, {}).get(state.sensor_slot)
        age_ms = max(0.0, time.monotonic() - state.timestamp_s) * 1000.0
        components = "x/y/z=waiting"
        if state.x is not None and state.y is not None and state.z is not None:
            components = f"x/y/z={state.x:.3f}/{state.y:.3f}/{state.z:.3f}"
        magnitude = f" mag={state.magnitude:.3f}" if state.magnitude is not None else ""
        device_id, device_source = self._mag_state_display_device_id(camera, state)
        device = "" if device_id is None else f" device={device_id} ({device_source})"
        text = f"{components}{magnitude}, age={age_ms:.0f}ms{device}"
        if label is not None:
            label.setText(text)
        if state.magnitude is not None:
            legacy_plot = self.mag_sensor_plots.get(camera)
            if legacy_plot is not None:
                legacy_plot.add_point(TelemetryHistoryPoint(state.timestamp_s, state.sensor_slot, state.magnitude))
            if hasattr(self, "rm3100_norm_plots"):
                plot = self.rm3100_norm_plots.get(camera)
                if plot is not None:
                    plot.add_point(TelemetryHistoryPoint(state.timestamp_s, state.sensor_slot, state.magnitude))
        self._refresh_pixhawk_sensor_health()
        self._refresh_alignment_workflow_status()
        self._refresh_alignment_magnetometer_ui()
        self._refresh_alignment_overview()

    def _update_gps_sensor(self, camera: str, state) -> None:
        self.latest_gps_sensors.setdefault(camera, {})[state.sensor_slot] = state
        self.gnss_monitor.update_sensor(camera, state.sensor_slot, state)
        self._record_external_timeline_row(
            camera,
            "sensor_gps",
            {
                "namespace": state.namespace,
                "topic": state.topic,
                "sensor_slot": state.sensor_slot,
                "sample_app_timestamp_s": state.timestamp_s,
                "sample_px4_timestamp_s": self._fmt_record_float(state.px4_timestamp_s, 6),
                "px4_timestamp_us": "" if state.px4_timestamp_us is None else state.px4_timestamp_us,
                "px4_timestamp_sample_us": (
                    "" if state.px4_timestamp_sample_us is None else state.px4_timestamp_sample_us
                ),
                "gps_utc_us": "" if state.gps_utc_us is None else state.gps_utc_us,
                "gps_timestamp_time_relative_us": (
                    ""
                    if state.gps_timestamp_time_relative_us is None
                    else state.gps_timestamp_time_relative_us
                ),
                "time_source": "px4_sensor_gps",
                "source_utc_method": "px4_sensor_gps_time_utc_usec",
                "device_id": "" if state.device_id is None else state.device_id,
                "fix_type": "" if state.fix_type is None else state.fix_type,
                "satellites": "" if state.satellites_used is None else state.satellites_used,
                "eph_m": self._fmt_record_float(state.eph_m, 6),
                "epv_m": self._fmt_record_float(state.epv_m, 6),
            },
        )
        label = self.gps_sensor_labels.get(camera, {}).get(state.sensor_slot)
        text = self._gps_sensor_status_text(state, include_source=True, include_detail=True)
        if label is not None:
            label.setText(text)
        if hasattr(self, "signal_gps_sensor_labels") and camera in self.signal_gps_sensor_labels:
            age_ms = max(0.0, time.monotonic() - state.timestamp_s) * 1000.0
            self._set_health_label(
                self.signal_gps_sensor_labels[camera],
                f"GPS sensor: {state.sensor_slot} | "
                f"{self._gps_sensor_status_text(state, include_source=True, include_detail=False)}",
                self._gps_fix_level(state.fix_type, age_ms=age_ms),
            )
        signal_series = f"{camera}_{state.sensor_slot}"
        if state.satellites_used is not None and hasattr(self, "signal_gps_sats_plot"):
            self.signal_gps_sats_plot.add_point(
                TelemetryHistoryPoint(state.timestamp_s, signal_series, float(state.satellites_used))
            )
        if state.eph_m is not None and hasattr(self, "signal_gps_eph_plot"):
            self.signal_gps_eph_plot.add_point(TelemetryHistoryPoint(state.timestamp_s, signal_series, state.eph_m))
        if state.satellites_used is not None:
            legacy_plot = self.gps_sensor_plots.get(camera)
            if legacy_plot is not None:
                legacy_plot.add_point(
                    TelemetryHistoryPoint(state.timestamp_s, state.sensor_slot, float(state.satellites_used))
                )
            if hasattr(self, "f9p_sats_plots"):
                plot = self.f9p_sats_plots.get(camera)
                if plot is not None:
                    plot.add_point(TelemetryHistoryPoint(state.timestamp_s, state.sensor_slot, float(state.satellites_used)))
        if state.eph_m is not None and hasattr(self, "f9p_quality_plots"):
            plot = self.f9p_quality_plots.get(camera)
            if plot is not None:
                plot.add_point(TelemetryHistoryPoint(state.timestamp_s, state.sensor_slot, state.eph_m))
        self._refresh_pixhawk_sensor_health()
        self._refresh_alignment_workflow_status()
        self._refresh_baseline_alignment_ui()
        self._refresh_gnss_base_tab()
        self._refresh_f9p_compare_label(camera)

    def _refresh_f9p_compare_label(self, camera: str) -> None:
        label = self.f9p_compare_labels.get(camera)
        if label is None:
            return
        now_s = time.monotonic()
        gps_slots = self.latest_gps_sensors.get(camera, {})
        gps_detected = sum(1 for slot in ("GPS1", "GPS2") if slot in gps_slots)
        gps1 = gps_slots.get("GPS1")
        gps2 = gps_slots.get("GPS2")
        if gps1 is None or gps2 is None:
            self._set_health_label(label, f"{gps_detected}/2 detected", "bad" if gps_detected else "idle")
            return
        details = []
        if gps1.fix_type is not None and gps2.fix_type is not None:
            details.append(f"fix={self._fix_type_label(gps1.fix_type)}/{self._fix_type_label(gps2.fix_type)}")
        if gps1.satellites_used is not None and gps2.satellites_used is not None:
            details.append(f"Δsats={abs(gps1.satellites_used - gps2.satellites_used)}")
        if gps1.eph_m is not None and gps2.eph_m is not None:
            details.append(f"Δeph={abs(gps1.eph_m - gps2.eph_m):.2f}m")
        details.append(f"skew={abs(gps1.timestamp_s - gps2.timestamp_s) * 1000.0:.0f}ms")
        details.append(f"src={self._telemetry_source_label(gps1.topic)} / {self._telemetry_source_label(gps2.topic)}")
        poor_fix = (gps1.fix_type is not None and gps1.fix_type < 3) or (
            gps2.fix_type is not None and gps2.fix_type < 3
        )
        not_rtk = (gps1.fix_type or 0) < 5 or (gps2.fix_type or 0) < 5
        stale = now_s - gps1.timestamp_s > 1.0 or now_s - gps2.timestamp_s > 1.0
        level = "bad" if poor_fix else "warn" if stale or not_rtk else "good"
        self._set_health_label(label, f"2/2 detected · {' · '.join(details)}", level)

    def _update_gnss_relative(self, camera: str, state) -> None:
        self.latest_gnss_relative[camera] = state
        heading_deg = None if state.heading_rad is None else math.degrees(state.heading_rad)
        heading_accuracy_deg = None if state.heading_accuracy_rad is None else math.degrees(state.heading_accuracy_rad)
        self._record_external_timeline_row(
            camera,
            "gnss_relative",
            {
                "namespace": state.namespace,
                "topic": state.topic,
                "sample_app_timestamp_s": state.timestamp_s,
                "sample_px4_timestamp_s": self._fmt_record_float(state.px4_timestamp_s, 6),
                "px4_timestamp_us": "" if state.px4_timestamp_us is None else state.px4_timestamp_us,
                "px4_timestamp_sample_us": (
                    "" if state.px4_timestamp_sample_us is None else state.px4_timestamp_sample_us
                ),
                "gps_utc_us": "" if state.gps_utc_us is None else state.gps_utc_us,
                "time_source": "px4_sensor_gnss_relative",
                "source_utc_method": "px4_sensor_gnss_relative_time_utc_usec",
                "device_id": "" if state.device_id is None else str(state.device_id),
                "heading_deg": self._fmt_record_float(heading_deg, 6),
                "heading_accuracy_deg": self._fmt_record_float(heading_accuracy_deg, 6),
                "position_length_m": self._fmt_record_float(state.position_length_m, 6),
                "accuracy_length_m": self._fmt_record_float(state.accuracy_length_m, 6),
                "gnss_fix_ok": self._fmt_record_bool(state.gnss_fix_ok),
                "differential_solution": self._fmt_record_bool(state.differential_solution),
                "relative_position_valid": self._fmt_record_bool(state.relative_position_valid),
                "carrier_solution_floating": self._fmt_record_bool(state.carrier_solution_floating),
                "carrier_solution_fixed": self._fmt_record_bool(state.carrier_solution_fixed),
                "moving_base_mode": self._fmt_record_bool(state.moving_base_mode),
                "heading_valid": self._fmt_record_bool(state.heading_valid),
                "age_s": self._fmt_record_float(max(0.0, time.monotonic() - state.timestamp_s), 6),
            },
        )
        self._refresh_pixhawk_sensor_health()

    def _freshest_sensor_slots(self, sensor_map: dict[str, dict[str, object]]) -> dict[str, object]:
        merged: dict[str, object] = {}
        for states in sensor_map.values():
            for slot, state in states.items():
                previous = merged.get(slot)
                if previous is None or getattr(state, "timestamp_s", 0.0) > getattr(previous, "timestamp_s", 0.0):
                    merged[slot] = state
        return merged

    def _set_health_label(self, label: QtWidgets.QLabel, text: str, level: str) -> None:
        colors = {
            "good": (SUCCESS_BG, SUCCESS, SUCCESS_TEXT),
            "warn": (WARNING_BG, WARNING, WARNING_TEXT),
            "bad": (ERROR_BG, ERROR, ERROR_TEXT),
            "idle": (PANEL_ALT_BG, PANEL_BORDER, TEXT_MUTED),
        }
        background, border, color = colors.get(level, colors["idle"])
        style = (
            f"background: {background}; border: 1px solid {border}; "
            f"border-radius: 0px; color: {color}; padding: 6px 8px; font-weight: 600;"
        )
        if label.text() == text and label.property("healthLevel") == level and label.styleSheet() == style:
            return
        label.setProperty("healthLevel", level)
        label.setText(text)
        label.setStyleSheet(style)

    def _set_label_text_if_changed(self, label: QtWidgets.QLabel, text: str) -> None:
        if label.text() != text:
            label.setText(text)

    def _telemetry_source_label(self, topic: object) -> str:
        text = str(topic or "").strip()
        if not text:
            return "source=unknown"
        if text.startswith("mavlink/"):
            return f"MAVLink {text.removeprefix('mavlink/')}"
        if "/fmu/out/" in text:
            return f"ROS2 {text}"
        return text

    def _gps_position_summary_text(self, gps: GpsPositionState, *, age_ms: float) -> str:
        sats = f"sats={gps.satellites_used}" if gps.satellites_used is not None else "sats=waiting"
        eph = f" | eph={gps.eph_m:.2f}m" if gps.eph_m is not None else ""
        return f"{sats}{eph} | age={age_ms:.0f}ms"

    def _gps_fix_level(self, fix_type: int | None, *, age_ms: float | None = None) -> str:
        if fix_type is None or fix_type < 3:
            return "bad"
        if age_ms is not None and age_ms > 3000:
            return "bad"
        if age_ms is not None and age_ms > 1000:
            return "warn"
        if fix_type >= 6:
            return "good"
        return "warn"

    def _gps_sensor_status_text(
        self,
        state: object,
        *,
        now_s: float | None = None,
        include_source: bool,
        include_detail: bool = True,
    ) -> str:
        if now_s is None:
            now_s = time.monotonic()
        age_ms = max(0.0, now_s - float(getattr(state, "timestamp_s", now_s))) * 1000.0
        fix_value = getattr(state, "fix_type", None)
        fix = self._fix_type_label(fix_value)
        sats = (
            f"sats={getattr(state, 'satellites_used')}"
            if getattr(state, "satellites_used", None) is not None
            else "sats=waiting"
        )
        eph = f" | eph={getattr(state, 'eph_m'):.2f}m" if getattr(state, "eph_m", None) is not None else ""
        epv = ""
        device = ""
        px4_time = ""
        if include_detail:
            epv = f" | epv={getattr(state, 'epv_m'):.2f}m" if getattr(state, "epv_m", None) is not None else ""
            device = f" | device={getattr(state, 'device_id')}" if getattr(state, "device_id", None) is not None else ""
            px4_time = (
                f" | px4_t={getattr(state, 'px4_timestamp_s'):.3f}s"
                if getattr(state, "px4_timestamp_s", None) is not None
                else ""
            )
        source = f" | {self._telemetry_source_label(getattr(state, 'topic', ''))}" if include_source else ""
        return f"fix={fix} | {sats}{eph}{epv} | age={age_ms:.0f}ms{device}{px4_time}{source}"

    def _vector_angle_deg(self, first, second) -> float | None:
        if (
            first.x is None
            or first.y is None
            or first.z is None
            or second.x is None
            or second.y is None
            or second.z is None
        ):
            return None
        first_norm = math.sqrt(first.x * first.x + first.y * first.y + first.z * first.z)
        second_norm = math.sqrt(second.x * second.x + second.y * second.y + second.z * second.z)
        if first_norm < 1e-9 or second_norm < 1e-9:
            return None
        dot = first.x * second.x + first.y * second.y + first.z * second.z
        ratio = max(-1.0, min(1.0, dot / (first_norm * second_norm)))
        return math.degrees(math.acos(ratio))

    def _fix_type_label(self, fix_type: int | None) -> str:
        return {
            0: "no fix",
            1: "dead reckoning",
            2: "2D",
            3: "3D",
            4: "DGPS",
            5: "RTK Float",
            6: "RTK Fixed",
            8: "Static",
        }.get(fix_type, "waiting" if fix_type is None else str(fix_type))

    def _refresh_pixhawk_sensor_health(self) -> None:
        if not hasattr(self, "pixhawk_sensor_summary_labels"):
            return
        if not self.pixhawk_sensor_summary_labels.get("left") and not self.pixhawk_sensor_summary_labels.get("right"):
            return
        now_s = time.monotonic()
        for camera, title in (("left", "Pixhawk 1"), ("right", "Pixhawk 2")):
            self._refresh_one_pixhawk_sensor_health(camera, title, now_s)

    def _refresh_one_pixhawk_sensor_health(self, camera: str, title: str, now_s: float) -> None:
        mag_slots = self.latest_mag_sensors.get(camera, {})
        gps_slots = self.latest_gps_sensors.get(camera, {})
        gnss_relative = self.latest_gnss_relative.get(camera)
        rm_detected = sum(1 for slot in ("CAN1", "CAN2") if slot in mag_slots)
        for slot, label in self.rm3100_slot_labels[camera].items():
            state = mag_slots.get(slot)
            if state is None:
                unavailable = self._ros2_only_unavailable(camera, slot)
                self._set_health_label(
                    label,
                    "ROS2 sensor_mag(_0..4)/vehicle_magnetometer 미게시" if unavailable else "waiting",
                    "warn" if unavailable else "idle",
                )
                continue
            age_ms = max(0.0, now_s - state.timestamp_s) * 1000.0
            components = "x/y/z=waiting"
            if state.x is not None and state.y is not None and state.z is not None:
                components = f"x/y/z={state.x:.3f}/{state.y:.3f}/{state.z:.3f}G"
            magnitude = f" | norm={state.magnitude:.3f}G" if state.magnitude is not None else ""
            temp = f" | temp={state.temperature_c:.1f}C" if state.temperature_c is not None else ""
            errors = f" | errors={state.error_count}" if getattr(state, "error_count", None) is not None else ""
            device = f" | device={state.device_id}" if state.device_id is not None else ""
            level = "bad" if age_ms > 3000 else "warn" if age_ms > 1000 else "good"
            self._set_health_label(label, f"{components}{magnitude}{temp}{errors} | age={age_ms:.0f}ms{device}", level)

        can1 = mag_slots.get("CAN1")
        can2 = mag_slots.get("CAN2")
        rm_level = "bad"
        rm_unavailable = rm_detected == 0 and self._ros2_only_unavailable(camera, "MAG")
        rm_summary = (
            "Status: ROS2 MAG topic 미게시 | 0/2 detected"
            if rm_unavailable
            else f"Status: 대기 중 | {rm_detected}/2 detected"
        )
        if can1 is not None and can2 is not None:
            can1_age = now_s - can1.timestamp_s
            can2_age = now_s - can2.timestamp_s
            delta_mag = None
            if can1.magnitude is not None and can2.magnitude is not None:
                delta_mag = abs(can1.magnitude - can2.magnitude)
            angle = self._vector_angle_deg(can1, can2)
            details = []
            if delta_mag is not None:
                details.append(f"Δnorm={delta_mag:.3f}G")
                self.rm3100_delta_plots[camera].add_point(TelemetryHistoryPoint(now_s, "Δnorm", delta_mag))
            if angle is not None:
                details.append(f"angle={angle:.1f}deg")
                self.rm3100_delta_plots[camera].add_point(TelemetryHistoryPoint(now_s, "angle", angle))
            details.append(f"skew={abs(can1_age - can2_age) * 1000.0:.0f}ms")
            stale = can1_age > 1.0 or can2_age > 1.0
            abnormal_delta = delta_mag is not None and delta_mag > 0.20
            abnormal_angle = angle is not None and angle > 20.0
            rm_level = "warn" if stale or abnormal_delta or abnormal_angle else "good"
            status = "WARN" if rm_level == "warn" else "OK"
            can1_text = "--" if can1.magnitude is None else f"{can1.magnitude:.3f}G"
            can2_text = "--" if can2.magnitude is None else f"{can2.magnitude:.3f}G"
            delta_text = "--" if delta_mag is None else f"{delta_mag:.3f}G"
            angle_text = "--" if angle is None else f"{angle:.1f}deg"
            rm_summary = f"Status: {status} | CAN1 {can1_text} / CAN2 {can2_text} | Δ {delta_text}, {angle_text}"
            self._set_health_label(self.rm3100_compare_labels[camera], rm_summary, rm_level)
        else:
            self._set_health_label(
                self.rm3100_compare_labels[camera],
                rm_summary,
                "bad" if rm_detected else "warn" if rm_unavailable else "idle",
            )
        self._set_health_label(
            self.pixhawk_sensor_summary_labels[camera]["rm3100"],
            f"{title}: {rm_summary}",
            rm_level if rm_detected else "warn" if rm_unavailable else "idle",
        )

        gps_detected = sum(1 for slot in ("GPS1", "GPS2") if slot in gps_slots)
        for slot, label in self.f9p_slot_labels[camera].items():
            state = gps_slots.get(slot)
            if state is None:
                unavailable = self._ros2_only_unavailable(camera, slot)
                self._set_health_label(
                    label,
                    "ROS2 sensor_gps(_0/_1) 미게시" if unavailable else "waiting",
                    "warn" if unavailable else "idle",
                )
                continue
            age_ms = max(0.0, now_s - state.timestamp_s) * 1000.0
            poor_fix = state.fix_type is not None and state.fix_type < 3
            level = "bad" if age_ms > 3000 or poor_fix else "warn" if age_ms > 1000 or (state.fix_type or 0) < 5 else "good"
            self._set_health_label(label, self._gps_sensor_status_text(state, now_s=now_s, include_source=True), level)

        gps1 = gps_slots.get("GPS1")
        gps2 = gps_slots.get("GPS2")
        f9p_level = "bad"
        gps2_unavailable = gps2 is None and self._ros2_only_unavailable(camera, "GPS2")
        f9p_summary = (
            f"{gps_detected}/2 detected · GPS2 ROS2 sensor_gps(_0/_1) 미게시"
            if gps2_unavailable
            else f"{gps_detected}/2 detected"
        )
        if gps1 is not None and gps2 is not None:
            details = []
            if gps1.fix_type is not None and gps2.fix_type is not None:
                details.append(f"fix={self._fix_type_label(gps1.fix_type)}/{self._fix_type_label(gps2.fix_type)}")
            if gps1.satellites_used is not None and gps2.satellites_used is not None:
                details.append(f"Δsats={abs(gps1.satellites_used - gps2.satellites_used)}")
            if gps1.eph_m is not None and gps2.eph_m is not None:
                details.append(f"Δeph={abs(gps1.eph_m - gps2.eph_m):.2f}m")
            details.append(f"skew={abs(gps1.timestamp_s - gps2.timestamp_s) * 1000.0:.0f}ms")
            details.append(f"src={self._telemetry_source_label(gps1.topic)} / {self._telemetry_source_label(gps2.topic)}")
            poor_fix = (gps1.fix_type is not None and gps1.fix_type < 3) or (gps2.fix_type is not None and gps2.fix_type < 3)
            not_rtk = (gps1.fix_type or 0) < 5 or (gps2.fix_type or 0) < 5
            stale = now_s - gps1.timestamp_s > 1.0 or now_s - gps2.timestamp_s > 1.0
            f9p_level = "bad" if poor_fix else "warn" if stale or not_rtk else "good"
            f9p_summary = f"2/2 detected · {' · '.join(details)}"
            self._set_health_label(self.f9p_compare_labels[camera], f9p_summary, f9p_level)
        else:
            self._set_health_label(
                self.f9p_compare_labels[camera],
                f9p_summary,
                "bad" if gps_detected else "warn" if gps2_unavailable else "idle",
            )
        self._set_health_label(
            self.pixhawk_sensor_summary_labels[camera]["f9p"],
            f"{title}: {f9p_summary}",
            f9p_level if gps_detected else "warn" if gps2_unavailable else "idle",
        )
        heading_text = "GNSS Heading: sensor_gnss_relative waiting"
        heading_level = "idle"
        if gnss_relative is not None:
            age_ms = max(0.0, now_s - gnss_relative.timestamp_s) * 1000.0
            heading_deg = None
            if gnss_relative.heading_rad is not None:
                heading_deg = math.degrees(gnss_relative.heading_rad)
            accuracy_deg = None
            if gnss_relative.heading_accuracy_rad is not None:
                accuracy_deg = math.degrees(gnss_relative.heading_accuracy_rad)
            heading_parts = [
                f"heading={heading_deg:.2f}deg" if heading_deg is not None else "heading=waiting",
                f"acc={accuracy_deg:.2f}deg" if accuracy_deg is not None else "acc=waiting",
                f"fixed={gnss_relative.carrier_solution_fixed}",
                f"moving_base={gnss_relative.moving_base_mode}",
                f"relative_valid={gnss_relative.relative_position_valid}",
                f"age={age_ms:.0f}ms",
            ]
            if gnss_relative.position_length_m is not None:
                heading_parts.append(f"baseline={gnss_relative.position_length_m:.2f}m")
            heading_level = (
                "good"
                if gnss_relative.heading_valid and gnss_relative.relative_position_valid and gnss_relative.carrier_solution_fixed
                else "warn"
                if gnss_relative.heading_valid or gnss_relative.relative_position_valid
                else "bad"
            )
            if age_ms > 3000:
                heading_level = "bad"
            elif age_ms > 1000 and heading_level == "good":
                heading_level = "warn"
            heading_text = f"GNSS Heading: {' · '.join(heading_parts)}"
        self._set_health_label(self.gnss_heading_labels[camera], heading_text, heading_level)
        self._set_health_label(self.pixhawk_sensor_summary_labels[camera]["heading"], heading_text, heading_level)

    def _update_relative_attitude_status(self) -> None:
        left = self.latest_extrinsics.get("left")
        right = self.latest_extrinsics.get("right")
        axis_values = (
            ("roll", None if left is None else left.roll_deg, None if right is None else right.roll_deg),
            ("pitch", None if left is None else left.pitch_deg, None if right is None else right.pitch_deg),
            ("yaw", None if left is None else left.yaw_deg, None if right is None else right.yaw_deg),
        )
        labels_by_axis = getattr(self, "attitude_compare_labels", {})
        for axis, left_value, right_value in axis_values:
            labels = labels_by_axis.get(axis)
            if labels is None:
                continue
            left_label, right_label, delta_label = labels
            left_label.setText("대기 중" if left_value is None else f"{left_value:.4f} deg")
            right_label.setText("대기 중" if right_value is None else f"{right_value:.4f} deg")
            if left_value is None or right_value is None:
                self._set_health_label(delta_label, "대기 중", "idle")
            else:
                delta = angle_diff_deg(right_value, left_value) if axis == "yaw" else right_value - left_value
                self._set_health_label(
                    delta_label,
                    f"Δ{axis.capitalize()} {delta:+.2f}°",
                    self._attitude_delta_level(axis, delta),
                )
        if left is None and right is None:
            self._refresh_alignment_workflow_status()
            return
        now_s = time.monotonic()
        plots_by_axis = getattr(self, "attitude_plots", {})
        for axis, left_value, right_value in axis_values:
            point = AttitudeHistoryPoint(
                timestamp_s=now_s,
                left_deg=left_value,
                right_deg=right_value,
            )
            self.attitude_history[axis].append(point)
            plot = plots_by_axis.get(axis)
            if plot is not None:
                plot.add_point(point)
        self._refresh_alignment_workflow_status()

    def _attitude_delta_level(self, axis: str, delta_deg: float) -> str:
        abs_delta = abs(delta_deg)
        good_limit = 3.0 if axis == "yaw" else 1.0
        bad_limit = 8.0 if axis == "yaw" else 3.0
        if abs_delta <= good_limit:
            return "good"
        if abs_delta <= bad_limit:
            return "warn"
        return "bad"

    def _initial_status(self) -> None:
        self._log("[연결] 네트워크 인터페이스 상태:\n" + interface_addresses(self.interface_edit.text().strip()))
        self.camera_connection_label.setText(STARTUP_CAMERA_STATUS)
        self.pixhawk_connection_label.setText(STARTUP_PIXHAWK_STATUS)
        self._log(STARTUP_READY_MESSAGE)
        self._recover_retryable_recording_session()
        if self.auto_start_pixhawk:
            QtCore.QTimer.singleShot(500, self._auto_start_pixhawk)
        if self.auto_start:
            self.start_yolo()

    def _recover_retryable_recording_session(self) -> bool:
        candidates = discover_retryable_sessions(APP_ROOT / "recordings")
        if not candidates:
            return False
        try:
            recovered = load_retryable_session(candidates[0])
        except Exception as exc:
            self._log(f"[저장] 미완료 세션 복구 정보를 읽지 못했습니다: {exc}")
            return False
        metadata = recovered.metadata
        def optional_int(value: object) -> int | None:
            return None if value in (None, "") else int(value)

        self.recording_session_id = str(metadata.get("session_id") or recovered.output_dir.name)
        self.recording_session_started_unix_ns = optional_int(
            metadata.get("session_started_unix_ns")
        )
        self.recording_session_started_monotonic_ns = optional_int(
            metadata.get("session_started_monotonic_ns")
        )
        self.recording_session_anchor_uncertainty_ns = optional_int(
            metadata.get("session_time_anchor_uncertainty_ns")
        )
        self.recording_session_started_wall_s = (
            None
            if self.recording_session_started_unix_ns is None
            else self.recording_session_started_unix_ns * 1e-9
        )
        self.recording_session_started_monotonic_s = (
            None
            if self.recording_session_started_monotonic_ns is None
            else self.recording_session_started_monotonic_ns * 1e-9
        )
        self.distance_recording_rows = recovered.pair_rows
        self.camera_recording_rows = recovered.camera_rows
        self.px4_recording_rows = recovered.px4_rows
        self._recovered_session_metadata = metadata
        self._session_save_retry_required = True
        self._session_save_finalized = False
        self.distance_recording_enabled = False
        self.video_recording_enabled = False
        self.record_button.setEnabled(False)
        self.save_history_button.setEnabled(True)
        self._refresh_recording_table()
        self.recording_status_label.setText(
            f"미완료 세션 복구됨: session={self.recording_session_id}, "
            f"프레임쌍={len(self.distance_recording_rows)}개 — 세션 파일 저장으로 재시도"
        )
        self._log(
            f"[저장] 미완료 세션 spool을 복구했습니다: {recovered.output_dir} "
            f"(pair={len(self.distance_recording_rows)})"
        )
        return True

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
        targets: list[QtWidgets.QPlainTextEdit] = []
        is_wiris_message = (
            text.startswith("[카메라]")
            or text.startswith("[캘리브레이션]")
            or text.startswith("[시계]")
            or text.startswith("[연결]")
            or "WIRIS" in text
        )
        is_pixhawk_message = text.startswith("[픽스호크]") or (
            not is_wiris_message and ("Pixhawk" in text or "PX4" in text or "MAVLink" in text)
        )
        if is_pixhawk_message:
            targets.append(self.pixhawk_log)
        if is_wiris_message:
            targets.append(self.wiris_log)
        if text.startswith("[YOLO]") or text.startswith("[Depth]") or text.startswith("[저장]"):
            targets.append(self.camera_yolo_log)
        if not targets:
            targets.append(self.wiris_log)
        for target in dict.fromkeys(targets):
            target.appendPlainText(text)



class WirisStereoDistanceGui(QtWidgets.QMainWindow):
    def __init__(self, *, auto_start: bool = False, auto_start_pixhawk: bool = False) -> None:
        super().__init__()
        self.setWindowTitle(APP_WINDOW_TITLE)
        self.setWindowFlags(
            self.windowFlags()
            | QtCore.Qt.WindowMinimizeButtonHint
            | QtCore.Qt.WindowMaximizeButtonHint
            | QtCore.Qt.WindowCloseButtonHint
        )
        self.resize(1280, 820)
        self.panel = WirisStereoDistancePanel(auto_start=auto_start, auto_start_pixhawk=auto_start_pixhawk)
        self.setCentralWidget(self.panel)

    def closeEvent(self, event) -> None:  # noqa: N802 - Qt override.
        self.panel.stop_all()
        if not self.panel._wait_for_session_save():
            event.ignore()
            QtCore.QTimer.singleShot(500, self.close)
            return
        super().closeEvent(event)


class IntegratedWirisPixhawkConsole(QtWidgets.QMainWindow):
    def __init__(self, *, auto_start_wiris: bool = False, auto_start_pixhawk: bool = False) -> None:
        super().__init__()
        self.setWindowTitle(APP_WINDOW_TITLE)
        self.setWindowFlags(
            self.windowFlags()
            | QtCore.Qt.WindowMinimizeButtonHint
            | QtCore.Qt.WindowMaximizeButtonHint
            | QtCore.Qt.WindowCloseButtonHint
        )
        self.resize(1500, 920)
        self.panel = WirisStereoDistancePanel(auto_start=auto_start_wiris, auto_start_pixhawk=auto_start_pixhawk)
        self.setCentralWidget(self.panel)

    def closeEvent(self, event) -> None:  # noqa: N802 - Qt override.
        self.panel.stop_all()
        if not self.panel._wait_for_session_save():
            event.ignore()
            QtCore.QTimer.singleShot(500, self.close)
            return
        super().closeEvent(event)

def main() -> int:
    os.environ.setdefault("QT_AUTO_SCREEN_SCALE_FACTOR", "1")
    try:
        faulthandler.register(signal.SIGUSR1)
    except Exception:
        pass
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
    wiris_only = "--wiris-only" in sys.argv
    if wiris_only:
        sys.argv.remove("--wiris-only")
    app = QtWidgets.QApplication(sys.argv)
    if wiris_only:
        window = WirisStereoDistanceGui(auto_start=auto_start, auto_start_pixhawk=auto_start_pixhawk)
    else:
        window = IntegratedWirisPixhawkConsole(auto_start_wiris=auto_start, auto_start_pixhawk=auto_start_pixhawk)
    if maximized:
        window.showMaximized()
    else:
        window.show()
    return app.exec_()
