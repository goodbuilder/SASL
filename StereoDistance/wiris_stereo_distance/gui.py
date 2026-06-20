from __future__ import annotations

import csv
import ftplib
import html as html_lib
import faulthandler
import json
import math
import os
import re
import signal
import sys
import threading
import time
from collections.abc import Sequence
from collections import deque
from dataclasses import dataclass
from datetime import datetime, timezone
from ipaddress import IPv4Address
from math import isfinite
from pathlib import Path
from urllib.parse import urlparse

from PyQt5 import QtCore, QtGui, QtWidgets

from sasl_common.truth_coordinates import local_ned_from_lla
from wiris_stereo_distance.calibration import (
    CheckerboardConfig,
    StereoIntrinsicsResult,
    calibrate_camera_from_images,
    detect_checkerboard,
    opencv_available,
    load_intrinsics_json,
    save_intrinsics_json,
)

from wiris_stereo_distance.attitude_bias import relative_attitude
from wiris_stereo_distance.camera_pane import CameraPane
from wiris_stereo_distance.coordinate_contract import (
    BODY_FRAME,
    CAMERA_FRAME,
    CAMERA_MOUNT_PROFILE,
    MEASUREMENT_AXIS_CONTRACT,
    MEASUREMENT_FRAME,
    TRUTH_AXIS_CONTRACT,
    TRUTH_FRAME,
    WORLD_FRAME,
)
from wiris_stereo_distance.measurement_widgets import (
    DistanceHistoryPlot,
    DistanceHistoryPoint,
    TriangulationQualityPlot,
)
from wiris_stereo_distance.measurement_controller import (
    MeasurementGeometryInput,
    calculate_measurement_geometry,
)
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
    baseline_profile_payload,
    dump_baseline_profile,
    evaluate_baseline_quality,
    load_baseline_profile,
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
    MockParameterStore,
    Px4MagManager,
    RM3100_BACKUP_ID,
    RM3100_PRIMARY_ID,
    current_pixhawk_6x_mag_params,
)
from wiris_stereo_distance.rtk_geometry import GpsPositionState, RtkBaselineEstimate, estimate_rtk_baseline
from wiris_stereo_distance.recording_session import (
    format_video_paths,
    raw_video_paths,
    save_session_files,
    session_dir,
    session_id_now,
)
from wiris_stereo_distance.reprojection_profile import (
    OriginBBoxReprojectionProfile,
    ReprojectionLookupResult,
    load_reprojection_profile,
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
    StereoCalibration,
)
from wiris_stereo_distance.tracking_filter import (
    ConstantVelocityKalmanFilter,
    TrackingUpdateResult,
    measurement_variance_from_stereo,
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
from wiris_stereo_distance.sync import TimedSampleBuffer, match_nearest_pair
from wiris_stereo_distance.triangulation import (
    RayTriangulationResult,
    camera_translation_right_in_left_camera,
    stereo_center_point_from_left_camera,
    triangulate_pixels_with_pixhawk_mount,
)
from wiris_stereo_distance.uam_telemetry import (
    TRUTH_SOURCE_AUTO,
    TRUTH_SOURCE_GLOBAL_POSITION,
    TRUTH_SOURCE_GPS_POSITION,
    TRUTH_SOURCE_LOCAL_NED,
    UamAlignmentError,
    UamAlignmentProfile,
    ULogTelemetryError,
    UamTelemetrySample,
    compare_measurement_rows,
    comparison_summary,
    estimate_alignment_profile_from_rows,
    estimate_time_offset_s,
    load_ulog_local_position_samples,
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

FORM_LABEL_MIN_WIDTH = 138
FORM_BUTTON_MIN_WIDTH = 112
NETWORK_FIELD_MIN_WIDTHS = {
    "interface": 120,
    "host": 140,
    "port": 88,
    "timeout": 104,
    "mask": 140,
}
APP_WINDOW_TITLE = "영상감시시스템 개발"
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
VISIBLE_STREAM_GOP_GET_COMMAND = "GEGS VISIBLE"
WIRIS_FTP_IMAGE_EXTENSIONS = (".jpg", ".jpeg", ".tif", ".tiff")
RECORDED_SAMPLE_TABLE_COLUMNS = (
    ("timestamp_s", "시간(s)"),
    ("distance_m", "YOLO Range(m)"),
    ("method", "방식"),
    ("quality", "품질"),
    ("disparity_px", "Disparity(px)"),
    ("camera_delta_s", "좌우Δt(s)"),
    ("vertical_delta_px", "세로Δ(px)"),
    ("baseline_source", "Baseline 출처"),
    ("baseline_m", "Baseline(m)"),
    ("relative_roll_deg", "상대 Roll(deg)"),
    ("relative_pitch_deg", "상대 Pitch(deg)"),
    ("relative_yaw_deg", "상대 Yaw(deg)"),
    ("rtk_fix_quality", "RTK 품질"),
    ("intrinsics_zoom_level", "줌"),
    ("intrinsics_path", "Intrinsics"),
    ("x_m", "Cam Right(m)"),
    ("y_m", "Cam Down(m)"),
    ("z_m", "Cam Forward(m)"),
    ("raw_distance_m", "Raw 거리(m)"),
    ("telemetry_range_m", "Telemetry Range(m)"),
    ("telemetry_range_error_m", "YOLO-Telemetry Err(m)"),
    ("raw_x_m", "Raw Right(m)"),
    ("raw_y_m", "Raw Down(m)"),
    ("raw_z_m", "Raw Forward(m)"),
    ("drone_north_m", "Truth N(m)"),
    ("drone_east_m", "Truth E(m)"),
    ("drone_down_m", "Truth D(m)"),
    ("error_x_m", "Mixed Err Right-North(m)"),
    ("error_y_m", "Mixed Err Down-East(m)"),
    ("error_z_m", "Mixed Err Forward-Down(m)"),
    ("bias_x_m", "Bias Right(m)"),
    ("bias_y_m", "Bias Down(m)"),
    ("bias_z_m", "Bias Forward(m)"),
    ("bias_profile_id", "Bias Profile"),
    ("profile_id", "BBox Profile"),
    ("profile_fallback_level", "Fallback"),
    ("left_cx", "L cx(px)"),
    ("right_cx", "R cx(px)"),
    ("left_corrected_cx", "L corr cx(px)"),
    ("right_corrected_cx", "R corr cx(px)"),
    ("bbox_mid_timestamp_s", "BBox mid t(s)"),
    ("bbox_mid_utc_unix_s", "BBox UTC(s)"),
    ("drone_timestamp_s", "Drone t(s)"),
    ("drone_utc_unix_s", "Drone UTC(s)"),
    ("drone_age_s", "Drone age(s)"),
)
UAM_ANALYSIS_TABLE_COLUMNS = (
    ("row_index", "#"),
    ("measurement_timestamp_s", "YOLO t(s)"),
    ("truth_timestamp_s", "Truth t(s)"),
    ("truth_dt_s", "Δt(s)"),
    ("truth_time_basis", "Basis"),
    ("truth_source", "Truth source"),
    ("truth_source_type", "Truth type"),
    ("comparison_frame", "Frame"),
    ("measurement_right_m", "YOLO Right"),
    ("measurement_down_m", "YOLO Down"),
    ("measurement_forward_m", "YOLO Forward"),
    ("estimated_range_m", "YOLO Range"),
    ("measurement_frd_forward_m", "YOLO Forward"),
    ("measurement_frd_right_m", "YOLO Right"),
    ("measurement_frd_down_m", "YOLO Down"),
    ("truth_frd_forward_m", "Truth Forward"),
    ("truth_frd_right_m", "Truth Right"),
    ("truth_frd_down_m", "Truth Down"),
    ("truth_frd_range_m", "Truth FRD Range"),
    ("error_frd_forward_m", "Err Forward"),
    ("error_frd_right_m", "Err Right"),
    ("error_frd_down_m", "Err Down"),
    ("estimated_north_m", "Est N"),
    ("estimated_east_m", "Est E"),
    ("estimated_down_m", "Est D"),
    ("truth_north_m", "Truth N"),
    ("truth_east_m", "Truth E"),
    ("truth_down_m", "Truth D"),
    ("truth_range_m", "Truth Range"),
    ("error_north_m", "Err N"),
    ("error_east_m", "Err E"),
    ("error_down_m", "Err D"),
    ("range_error_m", "Range Err"),
    ("error_distance_m", "Err Dist"),
    ("alignment_profile_id", "Align Profile"),
)


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
    displayed_hz = float(stats.get("displayed_hz", 0.0))
    inferred_hz = float(stats.get("inferred_hz", 0.0))
    resolution = camera_resolution_text(stats)
    resolution_text = resolution.removeprefix(" @ ")
    if stats.get("enable_detection"):
        receive_rate_text = f"{resolution_text} 수신 {decoded_hz:.1f} Hz".strip()
        return f"{camera_name}={state_text} {receive_rate_text}, YOLO {inferred_hz:.1f} Hz"
    receive_rate_text = f"{resolution_text} 수신 {decoded_hz:.1f} Hz".strip()
    return f"{camera_name}={state_text} {receive_rate_text}, 표시 {displayed_hz:.1f} Hz"


def format_runtime_camera_stats(camera: str, stats: dict, fallback_hz: float, *, stale: bool) -> str:
    label = camera_display_name(camera)
    resolution = camera_resolution_text(stats) or " @ --"
    has_stats = bool(stats)
    decoded_hz = float(stats.get("decoded_hz", 0.0)) if has_stats else 0.0
    stream_hz = decoded_hz if decoded_hz > 0.0 else fallback_hz
    if has_stats:
        yolo_hz_text = f"{float(stats.get('inferred_hz', 0.0)):.1f} Hz"
    else:
        yolo_hz_text = "--"
    state_text = " | 지연" if stale else ""
    stream_rate_text = f"{resolution.removeprefix(' @ ')} 수신 {stream_hz:.1f} Hz".strip()
    return (
        f"{label}: {stream_rate_text} | "
        f"YOLO {yolo_hz_text}{state_text}"
    )


def format_runtime_camera_status(camera: str, stats: dict, metadata: dict, fallback_hz: float, *, stale: bool) -> str:
    return f"{format_runtime_camera_stats(camera, stats, fallback_hz, stale=stale)} | 시각: {format_runtime_camera_timestamp(metadata)}"


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
    if metadata.get("rtcp_frame_ntp_unix_s") is not None:
        return "rtcp-ntp"
    if metadata.get("source_timestamp_s") is not None:
        return "source"
    return "app-receive"


def format_runtime_camera_timestamp(metadata: dict) -> str:
    frame_ntp_unix_s = metadata.get("rtcp_frame_ntp_unix_s")
    timestamp_reference = infer_runtime_camera_timestamp_reference(metadata)
    if frame_ntp_unix_s is not None:
        try:
            utc_text = datetime.fromtimestamp(float(frame_ntp_unix_s), tz=timezone.utc).isoformat(timespec="microseconds")
        except (TypeError, ValueError, OverflowError, OSError):
            utc_text = ""
        if utc_text:
            if timestamp_reference == "gnss-utc":
                return f"UTC {utc_text.replace('+00:00', 'Z')}"
            return f"Camera RTC {utc_text.replace('+00:00', 'Z')}"
    source_timestamp_s = metadata.get("source_timestamp_s")
    if source_timestamp_s is not None:
        source_source = str(metadata.get("timestamp_source") or "source")
        return f"src {float(source_timestamp_s):.6f} s [{source_source}]"
    return "missing"


@dataclass(frozen=True)
class BBoxBiasBin:
    min_m: float
    max_m: float
    bias_x_m: float
    bias_y_m: float
    bias_z_m: float
    sample_count: int

    def contains(self, distance_m: float) -> bool:
        return self.min_m <= distance_m < self.max_m


@dataclass(frozen=True)
class BBoxBiasProfile:
    profile_id: str
    bins: tuple[BBoxBiasBin, ...]
    created_unix_s: float

    def correction(self, distance_m: float) -> tuple[float, float, float] | None:
        if not self.bins:
            return None
        for bias_bin in self.bins:
            if bias_bin.contains(distance_m):
                return bias_bin.bias_x_m, bias_bin.bias_y_m, bias_bin.bias_z_m
        if distance_m < self.bins[0].min_m:
            first = self.bins[0]
            return first.bias_x_m, first.bias_y_m, first.bias_z_m
        last = self.bins[-1]
        return last.bias_x_m, last.bias_y_m, last.bias_z_m

    def to_dict(self) -> dict[str, object]:
        return {
            "profile_id": self.profile_id,
            "model": "distance_bins_mean_error",
            "created_unix_s": self.created_unix_s,
            "bins": [bias_bin.__dict__ for bias_bin in self.bins],
        }

    @classmethod
    def from_dict(cls, payload: dict[str, object]) -> "BBoxBiasProfile":
        bins = []
        for item in payload.get("bins", []):
            if not isinstance(item, dict):
                continue
            bins.append(
                BBoxBiasBin(
                    min_m=float(item["min_m"]),
                    max_m=float(item["max_m"]),
                    bias_x_m=float(item["bias_x_m"]),
                    bias_y_m=float(item.get("bias_y_m", 0.0)),
                    bias_z_m=float(item["bias_z_m"]),
                    sample_count=int(item.get("sample_count", 0)),
                )
            )
        return cls(
            profile_id=str(payload.get("profile_id") or f"bbox_bias_{int(time.time())}"),
            bins=tuple(sorted(bins, key=lambda item: item.min_m)),
            created_unix_s=float(payload.get("created_unix_s") or time.time()),
        )


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
            "truth_north_m": "N",
            "truth_east_m": "E",
            "truth_down_m": "D",
            "error_north_m": "Err N",
            "error_east_m": "Err E",
            "error_down_m": "Err D",
            "error_rdf_right_m": "Err Right",
            "error_rdf_down_m": "Err Down",
            "error_rdf_forward_m": "Err Forward",
            "error_frd_forward_m": "Err Forward",
            "error_frd_right_m": "Err Right",
            "error_frd_down_m": "Err Down",
            "error_x_m": "Err X",
            "error_y_m": "Err Y",
            "error_z_m": "Err Z",
            "error_distance_m": "Err Dist",
            "position_error_m": "Pos Err",
            "Δnorm": "Delta Norm",
            "angle": "Angle",
        }
        self.series_order: dict[str, int] = {
            "truth_north_m": 10,
            "truth_east_m": 11,
            "truth_down_m": 12,
            "error_north_m": 20,
            "error_east_m": 21,
            "error_down_m": 22,
            "error_frd_forward_m": 23,
            "error_frd_right_m": 24,
            "error_frd_down_m": 25,
            "error_rdf_right_m": 23,
            "error_rdf_down_m": 24,
            "error_rdf_forward_m": 25,
            "error_x_m": 30,
            "error_y_m": 31,
            "error_z_m": 32,
            "error_distance_m": 40,
            "position_error_m": 41,
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

        gap = 12
        panel_width = (content.width() - gap) // 2
        map_rect = QtCore.QRect(content.left(), content.top(), panel_width, content.height())
        camera_rect = QtCore.QRect(
            map_rect.right() + gap,
            content.top(),
            content.right() - map_rect.right() - gap,
            content.height(),
        )

        self._draw_vector_panel(
            painter,
            map_rect,
            title="안테나 평면 N/E",
            x_label="E",
            y_label="N",
            x_m=self.baseline.east_m,
            y_m=self.baseline.north_m,
            footer=f"|T_ant|={self.baseline.baseline_m:.3f} m, D={self.baseline.down_m:.3f} m",
            color=SERIES_DISTANCE,
        )

        if self.camera_translation_m is None:
            self._draw_gps_only_panel(painter, camera_rect)
        else:
            x_m, y_m, z_m = self.camera_translation_m
            self._draw_vector_panel(
                painter,
                camera_rect,
                title="카메라 기준 Right/Forward",
                x_label="Right",
                y_label="Forward",
                x_m=x_m,
                y_m=z_m,
                footer=f"Right={x_m:.3f} m, Down={y_m:.3f} m, Forward={z_m:.3f} m",
                color=SERIES_RIGHT,
            )
        self._draw_camera_attitude_arrows(painter, camera_rect)
        painter.end()

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
        self.zoom_intrinsics_compare_paths: list[Path] = []
        self.attitude_buffers = {
            "left": TimedSampleBuffer[CameraExtrinsicState](maxlen=1000),
            "right": TimedSampleBuffer[CameraExtrinsicState](maxlen=1000),
        }
        self.pixhawk: DualPixhawkAttitudeSubscriber | None = None
        self.mavlink_pixhawk: DualPixhawkMavlinkSubscriber | None = None
        self.uam_mavlink_pixhawk: DualPixhawkMavlinkSubscriber | None = None
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
        self.auto_wiris_connect_pending = False
        self.auto_start = auto_start
        self.auto_start_pixhawk = auto_start_pixhawk
        self.settings = load_settings()
        self.distance_history: deque[DistanceHistoryPoint] = deque(maxlen=600)
        self.tracking_filter = ConstantVelocityKalmanFilter()
        self.distance_recording_rows: list[dict[str, object]] = []
        self.distance_recording_keys: list[tuple[float, float, str]] = []
        self.external_parameter_recording_rows: list[dict[str, object]] = []
        self.uam_live_telemetry_samples: deque[UamTelemetrySample] = deque(maxlen=5000)
        self.uam_ulog_samples: list[UamTelemetrySample] = []
        self.uam_ulog_path: Path | None = None
        self.uam_ulog_truth_source_type = TRUTH_SOURCE_AUTO
        self.uam_analysis_rows: list[dict[str, object]] = []
        self.uam_alignment_profile: UamAlignmentProfile | None = None
        self.uam_alignment_profile_path: Path | None = None
        self._last_uam_live_sample_signature: tuple[float, float, float, float] | None = None
        self.distance_recording_enabled = False
        self.distance_recording_started_monotonic_s: float | None = None
        self.recording_session_id = ""
        self.recording_session_started_wall_s: float | None = None
        self.recording_session_started_monotonic_s: float | None = None
        self.video_recording_enabled = False
        self.video_recording_paths: dict[str, Path] = {}
        self._last_distance_history_key: tuple[float, float, str] | None = None
        self.camera_frame_times = {
            "left": deque(maxlen=120),
            "right": deque(maxlen=120),
        }
        self.camera_frame_metadata = {
            "left": {},
            "right": {},
        }
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
        self.latest_drone_position_m: tuple[float, float, float] | None = None
        self.latest_drone_position_components_m: dict[str, float] = {}
        self.latest_drone_position_component_timestamps_s: dict[str, float] = {}
        self.latest_drone_position_timestamp_s: float | None = None
        self.latest_drone_position_timestamp_reference = "app-receive"
        self.latest_drone_position_utc_unix_s: float | None = None
        self.latest_drone_truth_sample: UamTelemetrySample | None = None
        self.uam_gps_reference_lla: tuple[float, float, float] | None = None
        self.uam_gps_reference_kind = ""
        self.active_bias_profile: BBoxBiasProfile | None = None
        self.bias_profile_applied = False
        self.bias_candidate_rows: list[dict[str, str]] = []
        self.bbox_center_adjustment_path: Path | None = None
        self.bbox_center_adjustments: dict[str, tuple[tuple[float, float, float, float], tuple[float, float, float, float]]] = {}
        self.bbox_reprojection_profile_path: Path | None = None
        self.bbox_reprojection_profile: OriginBBoxReprojectionProfile | None = None

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
        center_bias_tab = QtWidgets.QWidget()
        center_bias_layout = QtWidgets.QVBoxLayout(center_bias_tab)
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
        uam_analysis_tab = QtWidgets.QWidget()
        uam_analysis_tab_layout = QtWidgets.QVBoxLayout(uam_analysis_tab)
        recording_samples_tab = QtWidgets.QWidget()
        recording_samples_layout = QtWidgets.QVBoxLayout(recording_samples_tab)
        advanced_tab = QtWidgets.QWidget()
        advanced_layout = QtWidgets.QVBoxLayout(advanced_tab)
        wiris_command_panel = QtWidgets.QWidget()
        wiris_command_layout = QtWidgets.QVBoxLayout(wiris_command_panel)
        for layout in (
            setup_layout,
            time_layout,
            alignment_layout,
            alignment_overview_layout,
            rtk_layout,
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
            center_bias_layout,
            yolo_layout_outer,
            measurement_layout,
            stereo_layout,
            runtime_layout,
            save_layout,
            save_session_layout,
            uam_analysis_tab_layout,
            recording_samples_layout,
            advanced_layout,
            wiris_command_layout,
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
        camera_params_layout.addWidget(parameter_tabs, 1)
        self.parameter_tabs = parameter_tabs
        self.intrinsic_parameters_tab_index = parameter_tabs.indexOf(intrinsic_parameters_tab)
        self.extrinsic_parameters_tab_index = parameter_tabs.indexOf(extrinsic_parameters_tab)

        alignment_tabs = QtWidgets.QTabWidget()
        alignment_tabs.setDocumentMode(True)
        alignment_tabs.addTab(alignment_overview_tab, "개요")
        alignment_tabs.addTab(rtk_tab, "GNSS / Baseline")
        alignment_tabs.addTab(alignment_mag_tab, "Magnetometer / Yaw")
        alignment_tabs.addTab(alignment_attitude_tab, "Attitude")
        alignment_tabs.addTab(alignment_coordinate_tab, "Coordinate System")
        alignment_tabs.addTab(alignment_validation_tab, "Validation")
        alignment_tabs.addTab(alignment_profile_tab, "Alignment Profile")
        alignment_layout.addWidget(alignment_tabs, 1)
        self.alignment_tabs = alignment_tabs
        self.alignment_overview_tab_index = alignment_tabs.indexOf(alignment_overview_tab)
        self.baseline_alignment_tab_index = alignment_tabs.indexOf(rtk_tab)
        self.signal_alignment_tab_index = self.baseline_alignment_tab_index
        self.rm3100_alignment_tab_index = alignment_tabs.indexOf(alignment_mag_tab)
        self.yaw_alignment_tab_index = self.rm3100_alignment_tab_index
        self.attitude_alignment_tab_index = alignment_tabs.indexOf(alignment_attitude_tab)
        self.coordinate_alignment_tab_index = alignment_tabs.indexOf(alignment_coordinate_tab)
        self.validation_alignment_tab_index = alignment_tabs.indexOf(alignment_validation_tab)
        self.alignment_profile_tab_index = alignment_tabs.indexOf(alignment_profile_tab)

        advanced_tabs = QtWidgets.QTabWidget()
        advanced_tabs.setDocumentMode(True)
        advanced_tabs.currentChanged.connect(lambda _index: self._handle_tab_changed(self.tabs.currentIndex()))
        advanced_layout.addWidget(advanced_tabs, 1)
        self.advanced_tabs = advanced_tabs

        measurement_tabs = QtWidgets.QTabWidget()
        measurement_tabs.setDocumentMode(True)
        measurement_tabs.addTab(runtime_tab, "실시간")
        measurement_tabs.addTab(stereo_tab, "계산 설정")
        measurement_tabs.addTab(triangulation_quality_tab, "삼각측량 품질")
        measurement_layout.addWidget(measurement_tabs, 1)
        self.measurement_tabs = measurement_tabs
        self.live_measurement_tab_index = measurement_tabs.indexOf(runtime_tab)
        self.stereo_settings_tab_index = measurement_tabs.indexOf(stereo_tab)
        self.triangulation_quality_tab_index = measurement_tabs.indexOf(triangulation_quality_tab)

        save_tabs = QtWidgets.QTabWidget()
        save_tabs.setDocumentMode(True)
        save_tabs.addTab(save_session_tab, "세션 저장")
        save_tabs.addTab(uam_analysis_tab, "UAM 비교")
        save_tabs.addTab(recording_samples_tab, "기록 샘플")
        save_layout.addWidget(save_tabs, 1)
        self.save_tabs = save_tabs
        self.save_session_tab_index = save_tabs.indexOf(save_session_tab)
        self.uam_analysis_tab_index = save_tabs.indexOf(uam_analysis_tab)
        self.recording_samples_tab_index = save_tabs.indexOf(recording_samples_tab)

        connection_tabs = QtWidgets.QTabWidget()
        connection_tabs.setDocumentMode(True)
        camera_tab = QtWidgets.QWidget()
        camera_layout_outer = QtWidgets.QVBoxLayout(camera_tab)
        pixhawk_connection_tab = QtWidgets.QWidget()
        pixhawk_connection_layout = QtWidgets.QVBoxLayout(pixhawk_connection_tab)
        uam_connection_tab = QtWidgets.QWidget()
        uam_connection_layout = QtWidgets.QVBoxLayout(uam_connection_tab)
        for layout in (camera_layout_outer, yolo_layout_outer, pixhawk_connection_layout, uam_connection_layout):
            layout.setContentsMargins(0, 0, 0, 0)
            layout.setSpacing(6)
        connection_tabs.addTab(camera_tab, "WIRIS")
        connection_tabs.addTab(pixhawk_connection_tab, "Pixhawk")
        connection_tabs.addTab(uam_connection_tab, "UAM")
        self.setup_tabs = connection_tabs
        self.pixhawk_connection_tab_index = connection_tabs.indexOf(pixhawk_connection_tab)
        self.uam_connection_tab_index = connection_tabs.indexOf(uam_connection_tab)
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
        self.preview_width_spin = QtWidgets.QSpinBox()
        self.preview_width_spin.setRange(320, 3840)
        self.preview_width_spin.setSingleStep(160)
        self.preview_width_spin.setValue(self.settings.preview_width)
        self.preview_rate_spin = QtWidgets.QDoubleSpinBox()
        self.preview_rate_spin.setRange(1.0, 120.0)
        self.preview_rate_spin.setSingleStep(5.0)
        self.preview_rate_spin.setValue(self.settings.preview_rate_hz)
        self.preview_rate_spin.setSuffix(" Hz")
        self.rtsp_latency_spin = QtWidgets.QSpinBox()
        self.rtsp_latency_spin.setRange(0, 5000)
        self.rtsp_latency_spin.setSingleStep(25)
        self.rtsp_latency_spin.setValue(self.settings.rtsp_latency_ms)
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
        self._camera_candidate_rows: list[dict[str, object]] = []
        self.pixhawk_auto_detect_button = QtWidgets.QPushButton("Pixhawk 자동 탐색")
        self.pixhawk_auto_detect_button.clicked.connect(self.auto_detect_pixhawks)
        self.pixhawk_usb_discover_button = QtWidgets.QPushButton("USB 연결 탐색")
        self.pixhawk_usb_discover_button.clicked.connect(self.discover_usb_pixhawk_connections)
        self.pixhawk_mavlink_url_candidate_button = QtWidgets.QPushButton("MAVLink URL 후보")
        self.pixhawk_mavlink_url_candidate_button.clicked.connect(self.discover_mavlink_url_candidates)
        self.pixhawk_candidate_table = QtWidgets.QTableWidget(0, 5)
        self.pixhawk_candidate_table.setHorizontalHeaderLabels(["종류", "값", "Baud", "상태", "인터페이스"])
        self.pixhawk_candidate_table.verticalHeader().setVisible(False)
        self.pixhawk_candidate_table.setSelectionBehavior(QtWidgets.QAbstractItemView.SelectRows)
        self.pixhawk_candidate_table.setSelectionMode(QtWidgets.QAbstractItemView.SingleSelection)
        self.pixhawk_candidate_table.setEditTriggers(QtWidgets.QAbstractItemView.NoEditTriggers)
        self.pixhawk_candidate_table.setMinimumHeight(110)
        self.pixhawk_candidate_table.itemSelectionChanged.connect(self._refresh_pixhawk_candidate_actions)
        pixhawk_candidate_header = self.pixhawk_candidate_table.horizontalHeader()
        pixhawk_candidate_header.setSectionResizeMode(QtWidgets.QHeaderView.Stretch)
        pixhawk_candidate_header.setStretchLastSection(False)
        self.clear_pixhawk_candidate_button = QtWidgets.QPushButton("Clear")
        self.clear_pixhawk_candidate_button.clicked.connect(self.clear_pixhawk_candidate_history)
        self.clear_pixhawk_candidate_button.setEnabled(False)
        self.clear_pixhawk_candidate_button.setToolTip("Pixhawk 탐색 후보 내역을 지웁니다.")
        self.apply_pixhawk_candidate_host_button = QtWidgets.QPushButton("Host에 적용")
        self.apply_pixhawk_candidate_host_button.clicked.connect(
            lambda _checked=False: self.apply_selected_pixhawk_candidate("host")
        )
        self.apply_pixhawk_candidate_left_button = QtWidgets.QPushButton("P1에 적용")
        self.apply_pixhawk_candidate_left_button.clicked.connect(
            lambda _checked=False: self.apply_selected_pixhawk_candidate("left")
        )
        self.apply_pixhawk_candidate_right_button = QtWidgets.QPushButton("P2에 적용")
        self.apply_pixhawk_candidate_right_button.clicked.connect(
            lambda _checked=False: self.apply_selected_pixhawk_candidate("right")
        )
        self._pixhawk_candidate_rows: list[dict[str, object]] = []
        self.swap_camera_button = QtWidgets.QPushButton("좌우 카메라 교체")
        self.swap_camera_button.clicked.connect(self.swap_cameras)
        self.swap_pixhawk_button = QtWidgets.QPushButton("Pixhawk 교체")
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
        self.detect_pixhawk_host_ip_button = QtWidgets.QPushButton("PC IP 탐색")
        self.detect_pixhawk_host_ip_button.clicked.connect(self.discover_pc_pixhawk_ip)
        self.left_mavlink_button = QtWidgets.QPushButton("P1 MAVLink 연결")
        self.left_mavlink_button.clicked.connect(lambda _checked=False: self.toggle_mavlink_connection("left"))
        self.right_mavlink_button = QtWidgets.QPushButton("P2 MAVLink 연결")
        self.right_mavlink_button.clicked.connect(lambda _checked=False: self.toggle_mavlink_connection("right"))
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
        self.swap_pixhawk_button.setText("Pixhawk 교체")
        self.check_button.setText("영상 수신 확인")
        self.check_control_button.setText("제어 확인")
        self.enable_rtsp_button.setText("영상 송출 활성화")
        self.camera_connect_button.setText("카메라 Preview 연결")
        self.pixhawk_button.setText("ROS2 연결")
        self.left_mavlink_button.setText("P1 MAVLink 연결")
        self.right_mavlink_button.setText("P2 MAVLink 연결")
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
        for button in (self.apply_candidate_nic_button, self.apply_candidate_left_button, self.apply_candidate_right_button):
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
                ("미리보기 Hz", self.preview_rate_spin),
                ("RTP 지연(ms)", self.rtsp_latency_spin),
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

        pixhawk_common_group = QtWidgets.QGroupBox("0 공통 준비")
        pixhawk_common_group.setObjectName("pixhawkCommonSetupGroup")
        pixhawk_common_form = QtWidgets.QGridLayout(pixhawk_common_group)
        pixhawk_common_form.setHorizontalSpacing(8)
        pixhawk_common_form.setVerticalSpacing(8)
        pixhawk_common_form.setContentsMargins(12, 24, 12, 12)
        pixhawk_common_form.setColumnStretch(1, 1)
        pixhawk_common_form.setColumnStretch(3, 1)
        pixhawk_common_form.setColumnStretch(6, 1)
        pixhawk_common_form.addWidget(QtWidgets.QLabel("픽스호크 Host IP"), 0, 0)
        pixhawk_common_form.addWidget(self.pixhawk_host_address_edit, 0, 1)
        pixhawk_common_form.addWidget(self.detect_pixhawk_host_ip_button, 0, 2)
        pixhawk_common_form.addWidget(QtWidgets.QLabel("ROS_DOMAIN_ID"), 0, 3)
        pixhawk_common_form.addWidget(self.domain_spin, 0, 4)
        pixhawk_common_form.addWidget(QtWidgets.QLabel("uXRCE 포트"), 0, 5)
        pixhawk_common_form.addWidget(self.uxrce_port_spin, 0, 6)
        pixhawk_common_form.addWidget(QtWidgets.QLabel("MAV 속도"), 1, 0)
        pixhawk_common_form.addWidget(self.mavlink_stream_rate_spin, 1, 1)
        pixhawk_common_form.addWidget(self.mavlink_configure_streams_checkbox, 1, 2, 1, 2)
        pixhawk_common_form.addWidget(self.mavlink_configure_uxrce_agent_checkbox, 1, 4, 1, 3)
        self.pixhawk_common_setup_group = pixhawk_common_group
        pixhawk_form.addWidget(pixhawk_common_group, 0, 0, 1, 6)

        pixhawk_candidate_group = QtWidgets.QGroupBox("1 ROS2 탐색 후보")
        pixhawk_candidate_group.setObjectName("pixhawkCandidateGroup")
        pixhawk_candidate_layout = QtWidgets.QVBoxLayout(pixhawk_candidate_group)
        pixhawk_candidate_layout.setSpacing(10)
        pixhawk_candidate_layout.setContentsMargins(12, 24, 12, 12)
        pixhawk_candidate_discovery_actions = QtWidgets.QHBoxLayout()
        pixhawk_candidate_discovery_actions.setSpacing(6)
        pixhawk_candidate_discovery_actions.addWidget(self.pixhawk_auto_detect_button, 1)
        pixhawk_candidate_discovery_actions.addWidget(self.pixhawk_usb_discover_button, 1)
        pixhawk_candidate_discovery_actions.addWidget(self.pixhawk_mavlink_url_candidate_button, 1)
        pixhawk_candidate_layout.addLayout(pixhawk_candidate_discovery_actions)
        pixhawk_candidate_layout.addWidget(self.pixhawk_candidate_table)
        pixhawk_candidate_actions = QtWidgets.QHBoxLayout()
        pixhawk_candidate_actions.setSpacing(6)
        pixhawk_candidate_actions.addWidget(self.clear_pixhawk_candidate_button, 1)
        pixhawk_candidate_actions.addWidget(self.apply_pixhawk_candidate_host_button, 1)
        pixhawk_candidate_actions.addWidget(self.apply_pixhawk_candidate_left_button, 1)
        pixhawk_candidate_actions.addWidget(self.apply_pixhawk_candidate_right_button, 1)
        pixhawk_candidate_layout.addLayout(pixhawk_candidate_actions)
        self.pixhawk_candidate_group = pixhawk_candidate_group
        pixhawk_form.addWidget(pixhawk_candidate_group, 1, 0, 1, 6)

        def build_pixhawk_side_group(
            title: str,
            object_name: str,
            namespace_edit: QtWidgets.QLineEdit,
            eth_ip_edit: QtWidgets.QLineEdit,
            mavlink_url_edit: QtWidgets.QLineEdit,
            baud_spin: QtWidgets.QSpinBox,
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
                (0, "MAVLink", mavlink_url_edit),
                (1, "Baudrate", baud_spin),
                (2, "ETH IP", eth_ip_edit),
                (3, "Namespace", namespace_edit),
            ):
                label = QtWidgets.QLabel(label_text)
                label.setAlignment(QtCore.Qt.AlignRight | QtCore.Qt.AlignVCenter)
                side_form.addWidget(label, row, 0)
                side_form.addWidget(field_widget, row, 1)
            return side_group

        pixhawk_pair_group = QtWidgets.QGroupBox("2 Pixhawk 1/2 사전 설정")
        pixhawk_pair_group.setObjectName("pixhawkPairSettingsGroup")
        pixhawk_pair_layout = QtWidgets.QHBoxLayout(pixhawk_pair_group)
        pixhawk_pair_layout.setSpacing(8)
        pixhawk_pair_layout.setContentsMargins(12, 24, 12, 12)
        left_pixhawk_group = build_pixhawk_side_group(
            "Pixhawk 1",
            "leftPixhawkWorkflowGroup",
            self.left_namespace_edit,
            self.left_pixhawk_eth_ip_edit,
            self.left_mavlink_url_edit,
            self.left_mavlink_baud_spin,
        )
        right_pixhawk_group = build_pixhawk_side_group(
            "Pixhawk 2",
            "rightPixhawkWorkflowGroup",
            self.right_namespace_edit,
            self.right_pixhawk_eth_ip_edit,
            self.right_mavlink_url_edit,
            self.right_mavlink_baud_spin,
        )
        pixhawk_pair_layout.addWidget(left_pixhawk_group, 1)
        pixhawk_pair_layout.addWidget(right_pixhawk_group, 1)
        self.left_pixhawk_workflow_group = left_pixhawk_group
        self.right_pixhawk_workflow_group = right_pixhawk_group
        self.pixhawk_pair_settings_group = pixhawk_pair_group
        pixhawk_form.addWidget(pixhawk_pair_group, 2, 0, 1, 6)

        pixhawk_connection_group = QtWidgets.QGroupBox("3 연결 실행")
        pixhawk_connection_group.setObjectName("pixhawkConnectionActionsGroup")
        pixhawk_connection_actions = QtWidgets.QHBoxLayout(pixhawk_connection_group)
        pixhawk_connection_actions.setSpacing(6)
        pixhawk_connection_actions.setContentsMargins(12, 24, 12, 12)
        for button in (
            self.swap_pixhawk_button,
            self.left_mavlink_button,
            self.right_mavlink_button,
            self.left_uxrce_param_button,
            self.right_uxrce_param_button,
            self.pixhawk_button,
        ):
            pixhawk_connection_actions.addWidget(button, 1)
        self.pixhawk_connection_actions_group = pixhawk_connection_group
        pixhawk_form.addWidget(pixhawk_connection_group, 3, 0, 1, 6)

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
        pixhawk_splitter.setSizes([460, 220])
        self.pixhawk_log_splitter = pixhawk_splitter
        pixhawk_connection_layout.addWidget(pixhawk_splitter, 1)
        self._build_uam_connection_tab(uam_connection_layout)

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
        self.max_dt_spin = QtWidgets.QDoubleSpinBox()
        self.max_dt_spin.setRange(0.01, 10.0)
        self.max_dt_spin.setValue(0.25)
        self.max_dt_spin.setSuffix(" s")
        self.max_dt_spin.valueChanged.connect(lambda _value: self._refresh_camera_parameters_tab())
        self.same_class_checkbox = self._build_toggle_button("같은 클래스만 매칭", True)
        self.same_class_checkbox.toggled.connect(lambda _checked: self._refresh_camera_parameters_tab())
        self.use_rtk_baseline_checkbox = self._build_toggle_button(
            "실시간 GPS baseline 사용",
            self.settings.use_rtk_baseline,
        )
        self.use_rtk_baseline_checkbox.toggled.connect(self._handle_gps_baseline_toggle_changed)
        self.require_rtk_fixed_checkbox = self._build_toggle_button(
            "RTK Fixed 품질 확인",
            self.settings.require_rtk_fixed,
        )
        self.require_rtk_fixed_checkbox.toggled.connect(self._handle_gps_baseline_toggle_changed)
        self.distance_label = QtWidgets.QLabel("거리: 좌/우 검출 매칭 대기 중")
        self.distance_label.setObjectName("distanceStatus")
        self.distance_label.setWordWrap(True)
        self.distance_mode_combo = QtWidgets.QComboBox()
        self.distance_mode_combo.setObjectName("distanceModeCombo")
        self.distance_mode_combo.setAccessibleName("외부 파라미터 계산 방식")
        self.distance_mode_combo.setToolTip(
            "3D ray 거리 계산에 사용할 외부 파라미터 출처를 고릅니다.\n"
            "Pixhawk 자세와 GPS/RTK baseline translation을 사용합니다."
        )
        self.distance_mode_combo.addItem("Pixhawk 자세 + GPS/RTK baseline", "pixhawk")
        initial_distance_mode = "pixhawk"
        self.distance_mode_combo.setCurrentIndex(self.distance_mode_combo.findData(initial_distance_mode))
        self._distance_mode_last_source = initial_distance_mode
        self.distance_mode_combo.currentIndexChanged.connect(self._handle_distance_mode_changed)
        self.measurement_display_combo = QtWidgets.QComboBox()
        self.measurement_display_combo.setObjectName("measurementDisplayCombo")
        self.measurement_display_combo.setAccessibleName("측정탭 영상 표시 방식")
        self.measurement_display_combo.setToolTip(
            "측정탭 좌/우 영상에 실시간 preview를 계속 보여줄지, 거리 계산에 선택된 검출 프레임 쌍을 보여줄지 고릅니다."
        )
        self.measurement_display_combo.addItem("계산 선택 프레임", "detection")
        self.measurement_display_combo.addItem("실시간 Preview", "preview")
        initial_display_mode = self.settings.measurement_display_mode
        if initial_display_mode not in {"detection", "preview"}:
            initial_display_mode = "detection"
        self.measurement_display_combo.setCurrentIndex(self.measurement_display_combo.findData(initial_display_mode))
        self.measurement_display_combo.currentIndexChanged.connect(self._handle_measurement_display_mode_changed)
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

        match_group = QtWidgets.QGroupBox("매칭 조건")
        match_form = QtWidgets.QGridLayout(match_group)
        match_form.setHorizontalSpacing(8)
        match_form.setColumnStretch(1, 1)
        match_form.addWidget(QtWidgets.QLabel("최대 Δt"), 0, 0)
        match_form.addWidget(self.max_dt_spin, 0, 1)
        match_form.addWidget(QtWidgets.QLabel("영상 표시"), 1, 0)
        match_form.addWidget(self.measurement_display_combo, 1, 1)
        match_form.addWidget(self.same_class_checkbox, 2, 0, 1, 2)

        stereo_form.addWidget(self.current_parameters_group, 0, 0, 1, 6)
        stereo_form.addWidget(stereo_parameter_splitter, 1, 0, 1, 6)
        stereo_form.addWidget(match_group, 2, 0, 1, 6)
        stereo_form.addWidget(self.distance_label, 3, 0, 1, 6)
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
        advanced_tabs.addTab(center_bias_tab, "Bias 보정")
        self._handle_alignment_step_changed(alignment_tabs.currentIndex())
        self.advanced_wiris_command_tab_index = advanced_tabs.indexOf(wiris_command_panel)
        self.advanced_custom_watch_tab_index = advanced_tabs.indexOf(custom_watch_tab)
        self.advanced_center_bias_tab_index = advanced_tabs.indexOf(center_bias_tab)
        self.wiris_command_tab_index = self.advanced_wiris_command_tab_index
        self.center_bias_tab_index = self.advanced_center_bias_tab_index

        attitude_compare_group = QtWidgets.QGroupBox("자세 축 비교")
        attitude_compare_form = QtWidgets.QGridLayout(attitude_compare_group)
        attitude_compare_form.addWidget(QtWidgets.QLabel("축"), 0, 0)
        attitude_compare_form.addWidget(QtWidgets.QLabel("픽스호크 1"), 0, 1)
        attitude_compare_form.addWidget(QtWidgets.QLabel("픽스호크 2"), 0, 2)
        attitude_compare_form.addWidget(QtWidgets.QLabel("차이"), 0, 3)
        self.attitude_compare_labels: dict[str, tuple[QtWidgets.QLabel, QtWidgets.QLabel, QtWidgets.QLabel]] = {}
        for row, axis in enumerate(("Roll", "Pitch", "Yaw"), start=1):
            left_label = QtWidgets.QLabel("대기 중")
            right_label = QtWidgets.QLabel("대기 중")
            delta_label = QtWidgets.QLabel("대기 중")
            for label in (left_label, right_label, delta_label):
                label.setObjectName("statusStrip")
            attitude_compare_form.addWidget(QtWidgets.QLabel(axis), row, 0)
            attitude_compare_form.addWidget(left_label, row, 1)
            attitude_compare_form.addWidget(right_label, row, 2)
            attitude_compare_form.addWidget(delta_label, row, 3)
            self.attitude_compare_labels[axis.lower()] = (left_label, right_label, delta_label)
        self.attitude_plots = {
            "roll": AttitudeHistoryPlot("Roll"),
            "pitch": AttitudeHistoryPlot("Pitch"),
            "yaw": AttitudeHistoryPlot("Yaw"),
        }
        pixhawk_common_layout.addWidget(attitude_compare_group)
        for plot in self.attitude_plots.values():
            pixhawk_common_layout.addWidget(plot, 1)

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
        self.rm3100_yaw_source_combos: dict[str, QtWidgets.QComboBox] = {}
        self.rm3100_yaw_source_status_labels: dict[str, QtWidgets.QLabel] = {}
        self.f9p_slot_labels: dict[str, dict[str, QtWidgets.QLabel]] = {"left": {}, "right": {}}
        self.rm3100_compare_labels: dict[str, QtWidgets.QLabel] = {}
        self.f9p_compare_labels: dict[str, QtWidgets.QLabel] = {}
        self.gnss_heading_labels: dict[str, QtWidgets.QLabel] = {}
        self.rm3100_norm_plots: dict[str, TelemetryHistoryPlot] = {}
        self.rm3100_delta_plots: dict[str, TelemetryHistoryPlot] = {}
        self.f9p_sats_plots: dict[str, TelemetryHistoryPlot] = {}
        self.f9p_quality_plots: dict[str, TelemetryHistoryPlot] = {}
        self._build_alignment_overview_tab(alignment_overview_layout)
        self._build_baseline_alignment_tab(rtk_layout)
        self._build_alignment_magnetometer_tab(alignment_mag_layout)
        self._build_alignment_attitude_tab(alignment_attitude_layout)
        self._build_alignment_coordinate_tab(alignment_coordinate_layout)
        self._build_alignment_validation_tab(alignment_validation_layout)
        self._build_alignment_profile_tab(alignment_profile_layout)

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
        self.wiris_set_pc_time_button = QtWidgets.QPushButton("PC UTC로 시간 설정")
        self.wiris_set_pc_time_button.clicked.connect(self.set_selected_wiris_time_from_pc)
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
        command_form.addWidget(self.wiris_set_pc_time_button, 3, 3)
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
        self.runtime_camera_settings_group = QtWidgets.QGroupBox("카메라 설정 / 상태")
        self.runtime_camera_settings_group.setObjectName("runtimeCameraSettingsGroup")
        runtime_camera_settings_layout = QtWidgets.QVBoxLayout(self.runtime_camera_settings_group)
        runtime_camera_settings_layout.setSpacing(4)
        self.runtime_header_layout = QtWidgets.QBoxLayout(QtWidgets.QBoxLayout.LeftToRight)
        self.runtime_header_layout.setContentsMargins(0, 0, 0, 0)
        self.runtime_header_layout.setSpacing(8)
        self.runtime_header_layout.addLayout(runtime_controls, 2)
        self.runtime_stream_settings_label = QtWidgets.QLabel(self._format_runtime_stream_settings())
        self.runtime_stream_settings_label.setObjectName("statChip")
        self.runtime_stream_settings_label.setAlignment(QtCore.Qt.AlignCenter)
        self.runtime_stream_settings_label.setTextInteractionFlags(QtCore.Qt.TextSelectableByMouse)
        self.runtime_stream_settings_label.setWordWrap(True)
        for widget in (self.rtsp_transport_combo, self.stream_backend_combo, self.decoder_combo):
            widget.currentIndexChanged.connect(self._update_runtime_stream_settings_label)
        self.preview_rate_spin.valueChanged.connect(self._update_runtime_stream_settings_label)
        self.rtsp_latency_spin.valueChanged.connect(self._update_runtime_stream_settings_label)
        self.runtime_header_layout.addWidget(self.runtime_stream_settings_label, 3)
        self.runtime_camera_status_group = QtWidgets.QGroupBox("카메라 상태")
        self.runtime_camera_status_group.setObjectName("runtimeCameraStatusGroup")
        self.left_runtime_stats_label = QtWidgets.QLabel("왼쪽: -- 0.0 Hz | YOLO -- | 대기 중 | 시각: missing")
        self.right_runtime_stats_label = QtWidgets.QLabel("오른쪽: -- 0.0 Hz | YOLO -- | 대기 중 | 시각: missing")
        for column, stats_label in enumerate((self.left_runtime_stats_label, self.right_runtime_stats_label)):
            stats_label.setMinimumWidth(320)
            stats_label.setSizePolicy(QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Fixed)
            stats_label.setAlignment(QtCore.Qt.AlignCenter)
            stats_label.setObjectName("statChip")
            stats_label.setTextInteractionFlags(QtCore.Qt.TextSelectableByMouse)
            stats_label.setWordWrap(False)
        self.runtime_camera_status_layout = QtWidgets.QGridLayout(self.runtime_camera_status_group)
        self.runtime_camera_status_layout.setContentsMargins(8, 8, 8, 8)
        self.runtime_camera_status_layout.setHorizontalSpacing(6)
        self.runtime_camera_status_layout.setVerticalSpacing(6)
        for column, stats_label in enumerate((self.left_runtime_stats_label, self.right_runtime_stats_label)):
            self.runtime_camera_status_layout.addWidget(stats_label, 0, column)
        self.runtime_camera_status_layout.setColumnStretch(0, 1)
        self.runtime_camera_status_layout.setColumnStretch(1, 1)
        self.runtime_camera_status_group.setSizePolicy(QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Fixed)
        runtime_camera_settings_layout.addLayout(self.runtime_header_layout)
        runtime_camera_settings_layout.addWidget(self.runtime_camera_status_group)
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

        quality_summary_group = QtWidgets.QGroupBox("현재 triangulation 상태")
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
        self.triangulation_quality_baseline_label = QtWidgets.QLabel("baseline: --")
        self.triangulation_quality_mode_label = QtWidgets.QLabel("mode: --")
        for label in (
            self.triangulation_quality_gap_label,
            self.triangulation_quality_distance_label,
            self.triangulation_quality_baseline_label,
            self.triangulation_quality_mode_label,
        ):
            label.setObjectName("statChip")
            label.setAlignment(QtCore.Qt.AlignCenter)
            label.setTextInteractionFlags(QtCore.Qt.TextSelectableByMouse)
        quality_summary_form.addWidget(self.triangulation_quality_status_label, 0, 0, 1, 4)
        quality_summary_form.addWidget(self.triangulation_quality_gap_label, 1, 0)
        quality_summary_form.addWidget(self.triangulation_quality_distance_label, 1, 1)
        quality_summary_form.addWidget(self.triangulation_quality_baseline_label, 1, 2)
        quality_summary_form.addWidget(self.triangulation_quality_mode_label, 1, 3)
        triangulation_quality_layout.addWidget(quality_summary_group)

        self.triangulation_quality_plot = TriangulationQualityPlot()
        triangulation_quality_layout.addWidget(self.triangulation_quality_plot, 1)
        point_group = QtWidgets.QGroupBox("3D point 비교")
        point_layout = QtWidgets.QGridLayout(point_group)
        point_layout.setColumnStretch(1, 1)
        self.triangulation_quality_raw_point_label = QtWidgets.QLabel("Raw 3D point: --")
        self.triangulation_quality_corrected_point_label = QtWidgets.QLabel("Corrected 3D point: --")
        self.triangulation_quality_correction_label = QtWidgets.QLabel("Correction vector: --")
        for row, label in enumerate(
            (
                self.triangulation_quality_raw_point_label,
                self.triangulation_quality_corrected_point_label,
                self.triangulation_quality_correction_label,
            )
        ):
            label.setObjectName("statusStrip")
            label.setTextInteractionFlags(QtCore.Qt.TextSelectableByMouse)
            point_layout.addWidget(label, row, 0, 1, 2)
        triangulation_quality_layout.addWidget(point_group)
        self._clear_triangulation_quality("3D ray triangulation 대기 중")

        save_group = QtWidgets.QGroupBox("세션 파일 / 설정")
        save_form = QtWidgets.QGridLayout(save_group)
        self.save_history_button = QtWidgets.QPushButton("세션 파일 저장")
        self.save_history_button.clicked.connect(self.save_distance_history_csv)
        self.save_settings_button = QtWidgets.QPushButton("전체 설정 저장")
        self.save_settings_button.clicked.connect(self.save_current_settings_from_ui)
        self.save_history_button.setText("세션 파일 저장")
        self.save_settings_button.setText("전체 설정 저장")
        self.session_summary_label = QtWidgets.QLabel("아직 기록된 측정 샘플이 없습니다.")
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

        uam_analysis_group = QtWidgets.QGroupBox("UAM Ground Truth 비교")
        uam_analysis_layout = QtWidgets.QVBoxLayout(uam_analysis_group)
        uam_analysis_form = QtWidgets.QGridLayout()
        uam_analysis_form.setHorizontalSpacing(8)
        uam_analysis_form.setVerticalSpacing(8)
        uam_analysis_form.setColumnStretch(1, 1)
        self.uam_ulog_path_edit = QtWidgets.QLineEdit()
        self.uam_ulog_path_edit.setPlaceholderText("비행 후 분석용 .ulg 파일")
        self.uam_ulog_browse_button = QtWidgets.QPushButton("ULog 선택/로드")
        self.uam_ulog_browse_button.clicked.connect(self.browse_uam_ulog_file)
        self.uam_load_ulog_button = QtWidgets.QPushButton("현재 경로 다시 로드")
        self.uam_load_ulog_button.clicked.connect(self.load_uam_ulog_file)
        self.uam_truth_source_combo = QtWidgets.QComboBox()
        self.uam_truth_source_combo.addItem("Auto", TRUTH_SOURCE_AUTO)
        self.uam_truth_source_combo.addItem("Local NED", TRUTH_SOURCE_LOCAL_NED)
        self.uam_truth_source_combo.addItem("Global Position", TRUTH_SOURCE_GLOBAL_POSITION)
        self.uam_truth_source_combo.addItem("GPS Position", TRUTH_SOURCE_GPS_POSITION)
        self.uam_truth_source_combo.currentIndexChanged.connect(self._on_uam_truth_source_changed)
        self.uam_compare_max_dt_spin = QtWidgets.QDoubleSpinBox()
        self.uam_compare_max_dt_spin.setRange(0.001, 60.0)
        self.uam_compare_max_dt_spin.setDecimals(3)
        self.uam_compare_max_dt_spin.setSingleStep(0.05)
        self.uam_compare_max_dt_spin.setValue(self.settings.uam_compare_max_dt_s)
        self.uam_compare_max_dt_spin.setSuffix(" s")
        self.uam_time_offset_spin = QtWidgets.QDoubleSpinBox()
        self.uam_time_offset_spin.setRange(-3600.0, 3600.0)
        self.uam_time_offset_spin.setDecimals(6)
        self.uam_time_offset_spin.setSingleStep(0.001)
        self.uam_time_offset_spin.setValue(self.settings.uam_compare_time_offset_s)
        self.uam_time_offset_spin.setSuffix(" s")
        self.uam_run_analysis_button = QtWidgets.QPushButton("UAM 비교 실행")
        self.uam_run_analysis_button.clicked.connect(self.run_uam_analysis)
        self.uam_estimate_offset_button = QtWidgets.QPushButton("시간 offset 추정")
        self.uam_estimate_offset_button.clicked.connect(self.estimate_uam_time_offset_from_ui)
        self.uam_estimate_alignment_button = QtWidgets.QPushButton("좌표 보정 자동 추정")
        self.uam_estimate_alignment_button.clicked.connect(self.estimate_uam_alignment_from_ui)
        self.uam_save_alignment_button = QtWidgets.QPushButton("보정 저장")
        self.uam_save_alignment_button.clicked.connect(self.save_uam_alignment_profile)
        self.uam_load_alignment_button = QtWidgets.QPushButton("보정 불러오기")
        self.uam_load_alignment_button.clicked.connect(self.load_uam_alignment_profile)
        self.uam_clear_alignment_button = QtWidgets.QPushButton("보정 해제")
        self.uam_clear_alignment_button.clicked.connect(self.clear_uam_alignment_profile)
        self.uam_alignment_status_label = QtWidgets.QLabel("좌표 보정: 없음")
        self.uam_alignment_status_label.setObjectName("statusStrip")
        self.uam_alignment_status_label.setWordWrap(True)
        self.uam_analysis_status_label = QtWidgets.QLabel(
            "UAM 비교: ULog 최종 평가 또는 live/recorded-row 백업 비교 대기"
        )
        self.uam_analysis_status_label.setObjectName("statusStrip")
        self.uam_analysis_status_label.setWordWrap(True)
        uam_path_actions = QtWidgets.QHBoxLayout()
        uam_path_actions.addWidget(self.uam_ulog_path_edit, 1)
        uam_path_actions.addWidget(self.uam_ulog_browse_button)
        uam_path_actions.addWidget(self.uam_load_ulog_button)
        uam_alignment_actions = QtWidgets.QHBoxLayout()
        uam_alignment_actions.addWidget(self.uam_estimate_alignment_button)
        uam_alignment_actions.addWidget(self.uam_save_alignment_button)
        uam_alignment_actions.addWidget(self.uam_load_alignment_button)
        uam_alignment_actions.addWidget(self.uam_clear_alignment_button)
        uam_alignment_actions.addStretch(1)
        uam_analysis_form.addWidget(QtWidgets.QLabel("ULog"), 0, 0)
        uam_analysis_form.addLayout(uam_path_actions, 0, 1, 1, 6)
        uam_analysis_form.addWidget(QtWidgets.QLabel("truth source"), 1, 0)
        uam_analysis_form.addWidget(self.uam_truth_source_combo, 1, 1)
        uam_analysis_form.addWidget(QtWidgets.QLabel("최대 Δt"), 1, 2)
        uam_analysis_form.addWidget(self.uam_compare_max_dt_spin, 1, 3)
        uam_analysis_form.addWidget(QtWidgets.QLabel("time offset"), 1, 4)
        uam_analysis_form.addWidget(self.uam_time_offset_spin, 1, 5)
        uam_analysis_form.addWidget(self.uam_run_analysis_button, 1, 6)
        uam_analysis_form.addWidget(self.uam_estimate_offset_button, 2, 1)
        uam_analysis_form.addWidget(QtWidgets.QLabel("좌표 보정"), 3, 0)
        uam_analysis_form.addLayout(uam_alignment_actions, 3, 1, 1, 6)
        uam_analysis_form.addWidget(self.uam_alignment_status_label, 4, 0, 1, 7)
        uam_analysis_form.addWidget(self.uam_analysis_status_label, 5, 0, 1, 7)
        uam_analysis_layout.addLayout(uam_analysis_form)
        uam_analysis_splitter = QtWidgets.QSplitter(QtCore.Qt.Horizontal)
        uam_analysis_left = QtWidgets.QWidget()
        uam_analysis_left_layout = QtWidgets.QVBoxLayout(uam_analysis_left)
        uam_analysis_left_layout.setContentsMargins(0, 0, 0, 0)
        self.uam_analysis_table = QtWidgets.QTableWidget(0, len(UAM_ANALYSIS_TABLE_COLUMNS))
        self.uam_analysis_table.setObjectName("uamAnalysisTable")
        self.uam_analysis_table.setHorizontalHeaderLabels([label for _field, label in UAM_ANALYSIS_TABLE_COLUMNS])
        self.uam_analysis_table.setEditTriggers(QtWidgets.QAbstractItemView.NoEditTriggers)
        self.uam_analysis_table.setSelectionBehavior(QtWidgets.QAbstractItemView.SelectRows)
        self.uam_analysis_table.setSelectionMode(QtWidgets.QAbstractItemView.SingleSelection)
        self.uam_analysis_table.setAlternatingRowColors(True)
        self.uam_analysis_table.verticalHeader().setVisible(False)
        self.uam_analysis_table.horizontalHeader().setSectionResizeMode(QtWidgets.QHeaderView.ResizeToContents)
        self.uam_analysis_table.itemSelectionChanged.connect(self._refresh_uam_analysis_detail)
        self.uam_analysis_detail = QtWidgets.QPlainTextEdit()
        self.uam_analysis_detail.setReadOnly(True)
        self.uam_analysis_detail.setMaximumBlockCount(200)
        self.uam_analysis_detail.setPlaceholderText("비교 행을 선택하면 YOLO와 Ground Truth의 전체 오차 필드가 표시됩니다.")
        uam_analysis_left_layout.addWidget(self.uam_analysis_table, 2)
        uam_analysis_left_layout.addWidget(self.uam_analysis_detail, 1)
        uam_plot_panel = QtWidgets.QWidget()
        uam_plot_panel_layout = QtWidgets.QVBoxLayout(uam_plot_panel)
        uam_plot_panel_layout.setContentsMargins(0, 0, 0, 0)
        uam_plot_button_row = QtWidgets.QHBoxLayout()
        self.uam_show_error_button = QtWidgets.QPushButton("오차 그래프")
        self.uam_show_error_button.clicked.connect(self.show_uam_error_plot)
        self.uam_show_raw_button = QtWidgets.QPushButton("Raw NED")
        self.uam_show_raw_button.clicked.connect(self.show_uam_raw_plot)
        uam_plot_button_row.addWidget(self.uam_show_error_button)
        uam_plot_button_row.addWidget(self.uam_show_raw_button)
        uam_plot_button_row.addStretch(1)
        self.uam_error_plot = TelemetryHistoryPlot("UAM 비교 오차", "m")
        self.uam_error_plot.setMinimumHeight(240)
        self.uam_raw_plot = TelemetryHistoryPlot("UAM Ground Truth Raw N/E/D", "m")
        self.uam_raw_plot.setMinimumHeight(240)
        self.uam_plot_stack = QtWidgets.QStackedWidget()
        self.uam_plot_stack.addWidget(self.uam_error_plot)
        self.uam_plot_stack.addWidget(self.uam_raw_plot)
        uam_plot_panel_layout.addLayout(uam_plot_button_row)
        uam_plot_panel_layout.addWidget(self.uam_plot_stack, 1)
        uam_analysis_splitter.addWidget(uam_analysis_left)
        uam_analysis_splitter.addWidget(uam_plot_panel)
        uam_analysis_splitter.setStretchFactor(0, 3)
        uam_analysis_splitter.setStretchFactor(1, 2)
        uam_analysis_layout.addWidget(uam_analysis_splitter, 1)
        uam_analysis_tab_layout.addWidget(uam_analysis_group, 1)

        self.recording_table_group = QtWidgets.QGroupBox("거리 샘플 실시간 보기")
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

        bias_status_group = QtWidgets.QGroupBox("운용 Bias Profile 상태 (최종 UAM 평가 제외)")
        bias_status_layout = QtWidgets.QGridLayout(bias_status_group)
        self.bias_profile_status_label = QtWidgets.QLabel("Profile: 없음")
        self.bias_profile_status_label.setObjectName("statusStrip")
        self.bias_profile_status_label.setWordWrap(True)
        self.bias_apply_checkbox = self._build_toggle_button("실시간 보정 적용(최종 평가 제외)", False)
        self.bias_apply_checkbox.toggled.connect(self.set_bias_profile_applied)
        self.bias_bin_size_spin = QtWidgets.QDoubleSpinBox()
        self.bias_bin_size_spin.setRange(10.0, 5000.0)
        self.bias_bin_size_spin.setDecimals(1)
        self.bias_bin_size_spin.setValue(200.0)
        self.bias_bin_size_spin.setSuffix(" m")
        bias_status_layout.addWidget(self.bias_profile_status_label, 0, 0, 1, 4)
        bias_status_layout.addWidget(self.bias_apply_checkbox, 1, 0)
        bias_status_layout.addWidget(QtWidgets.QLabel("거리 bin"), 1, 1)
        bias_status_layout.addWidget(self.bias_bin_size_spin, 1, 2)
        bias_status_layout.setColumnStretch(3, 1)

        bbox_adjust_group = QtWidgets.QGroupBox("2D bbox 중심 보정 / Reprojection Profile")
        bbox_adjust_layout = QtWidgets.QGridLayout(bbox_adjust_group)
        self.bbox_adjust_status_label = QtWidgets.QLabel("2D bbox 보정: 없음")
        self.bbox_adjust_status_label.setObjectName("statusStrip")
        self.bbox_adjust_status_label.setWordWrap(True)
        self.bbox_adjust_load_button = QtWidgets.QPushButton("CSV/JSON profile 불러오기")
        self.bbox_adjust_load_button.clicked.connect(self.load_bbox_center_adjustment_csv)
        self.bbox_adjust_clear_button = QtWidgets.QPushButton("2D 보정 해제")
        self.bbox_adjust_clear_button.clicked.connect(self.clear_bbox_center_adjustment)
        bbox_adjust_layout.addWidget(self.bbox_adjust_status_label, 0, 0, 1, 3)
        bbox_adjust_layout.addWidget(self.bbox_adjust_load_button, 1, 0)
        bbox_adjust_layout.addWidget(self.bbox_adjust_clear_button, 1, 1)
        bbox_adjust_layout.setColumnStretch(2, 1)

        bias_drone_group = QtWidgets.QGroupBox("UAM 중심 위치 상태")
        bias_drone_layout = QtWidgets.QVBoxLayout(bias_drone_group)
        self.bias_drone_status_label = QtWidgets.QLabel("UAM 중심 위치: 연결 > UAM 수신 대기")
        self.bias_drone_status_label.setObjectName("statusStrip")
        self.bias_drone_status_label.setWordWrap(True)
        bias_drone_layout.addWidget(self.bias_drone_status_label)

        bias_actions = QtWidgets.QHBoxLayout()
        self.bias_build_session_button = QtWidgets.QPushButton("현재 세션에서 Profile 생성")
        self.bias_build_session_button.clicked.connect(self.build_bias_profile_from_current_session)
        self.bias_load_csv_button = QtWidgets.QPushButton("CSV 불러와 후보정")
        self.bias_load_csv_button.clicked.connect(self.load_bias_candidates_csv)
        self.bias_save_profile_button = QtWidgets.QPushButton("Profile 저장")
        self.bias_save_profile_button.clicked.connect(self.save_bias_profile)
        self.bias_load_profile_button = QtWidgets.QPushButton("Profile 불러오기")
        self.bias_load_profile_button.clicked.connect(self.load_bias_profile)
        for button in (
            self.bias_build_session_button,
            self.bias_load_csv_button,
            self.bias_save_profile_button,
            self.bias_load_profile_button,
        ):
            bias_actions.addWidget(button)
        bias_actions.addStretch(1)

        self.bias_profile_table = QtWidgets.QTableWidget(0, 6)
        self.bias_profile_table.setHorizontalHeaderLabels(
            ["min(m)", "max(m)", "bias Right", "bias Down", "bias Forward", "samples"]
        )
        self.bias_profile_table.setEditTriggers(QtWidgets.QAbstractItemView.NoEditTriggers)
        self.bias_profile_table.horizontalHeader().setSectionResizeMode(QtWidgets.QHeaderView.ResizeToContents)
        self.bias_profile_table.horizontalHeader().setSectionResizeMode(5, QtWidgets.QHeaderView.Stretch)

        self.bias_profile_detail = QtWidgets.QPlainTextEdit()
        self.bias_profile_detail.setReadOnly(True)
        self.bias_profile_detail.setPlaceholderText(
            "CSV 또는 현재 세션 기록에서 UAM 중심 N/E/D와 raw Right/Down/Forward 차이를 거리 bin별 평균으로 profile화합니다. "
            "주 텔레메트리 연결은 연결 > UAM을 사용하세요."
        )
        center_bias_layout.addWidget(bias_status_group)
        center_bias_layout.addWidget(bbox_adjust_group)
        center_bias_layout.addWidget(bias_drone_group)
        center_bias_layout.addLayout(bias_actions)
        center_bias_layout.addWidget(self.bias_profile_table, 2)
        center_bias_layout.addWidget(self.bias_profile_detail, 1)
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

    def _build_uam_connection_tab(self, layout: QtWidgets.QVBoxLayout) -> None:
        group = QtWidgets.QGroupBox("UAM / 드론 Ground Truth 텔레메트리")
        group_layout = QtWidgets.QVBoxLayout(group)
        form = QtWidgets.QGridLayout()
        form.setHorizontalSpacing(8)
        form.setVerticalSpacing(8)
        form.setColumnStretch(1, 1)
        form.setColumnStretch(3, 1)

        self.uam_source_combo = QtWidgets.QComboBox()
        self.uam_source_combo.addItem("ROS2 topic", "ros2")
        self.uam_source_combo.addItem("USB MAVLink LOCAL_POSITION_NED", "mavlink")
        self.uam_source_combo.addItem("USB MAVLink GPS Position", "mavlink_gps")
        self.uam_source_combo.addItem("USB MAVLink Global Position", "mavlink_global")
        self.uam_source_combo.addItem("사용 안 함", "off")
        self._set_combo_data(self.uam_source_combo, self.settings.uam_telemetry_source)

        self.uam_ros2_topic_edit = QtWidgets.QLineEdit(self.settings.uam_ros2_topic)
        self.uam_ros2_x_field_edit = QtWidgets.QLineEdit(self.settings.uam_ros2_x_field)
        self.uam_ros2_y_field_edit = QtWidgets.QLineEdit(self.settings.uam_ros2_y_field)
        self.uam_ros2_z_field_edit = QtWidgets.QLineEdit(self.settings.uam_ros2_z_field)

        self.uam_mavlink_url_edit = QtWidgets.QLineEdit(self.settings.uam_mavlink_url)
        self.uam_mavlink_baud_spin = self._build_mavlink_baud_spin()
        self.uam_mavlink_baud_spin.setValue(self.settings.uam_mavlink_baud)
        self.uam_mavlink_side_combo = QtWidgets.QComboBox()
        self.uam_mavlink_side_combo.addItem("UAM USB", "left")
        self._set_combo_data(self.uam_mavlink_side_combo, self.settings.uam_mavlink_side)
        self.uam_mavlink_message_edit = QtWidgets.QLineEdit(self.settings.uam_mavlink_message)
        self.uam_mavlink_x_field_edit = QtWidgets.QLineEdit(self.settings.uam_mavlink_x_field)
        self.uam_mavlink_y_field_edit = QtWidgets.QLineEdit(self.settings.uam_mavlink_y_field)
        self.uam_mavlink_z_field_edit = QtWidgets.QLineEdit(self.settings.uam_mavlink_z_field)

        self.uam_connect_button = QtWidgets.QPushButton("UAM 텔레메트리 연결")
        self.uam_connect_button.clicked.connect(self.connect_uam_telemetry_from_ui)
        self.uam_disconnect_button = QtWidgets.QPushButton("UAM 텔레메트리 해제")
        self.uam_disconnect_button.clicked.connect(self.disconnect_uam_telemetry_from_ui)
        self.uam_clear_live_button = QtWidgets.QPushButton("실시간 버퍼 초기화")
        self.uam_clear_live_button.clicked.connect(self.clear_uam_live_telemetry)
        self.uam_status_label = QtWidgets.QLabel("UAM telemetry: 대기")
        self.uam_status_label.setObjectName("statusStrip")
        self.uam_status_label.setWordWrap(True)
        self.uam_mount_reference_label = QtWidgets.QLabel(
            "기준 위치: GPS 안테나와 Pixhawk를 UAM/드론 중앙에 둔 body origin N/E/D"
        )
        self.uam_mount_reference_label.setObjectName("statusStrip")
        self.uam_mount_reference_label.setWordWrap(True)
        self.uam_latest_label = QtWidgets.QLabel("latest: --")
        self.uam_latest_label.setObjectName("statusStrip")
        self.uam_latest_label.setWordWrap(True)

        form.addWidget(QtWidgets.QLabel("소스"), 0, 0)
        form.addWidget(self.uam_source_combo, 0, 1)
        form.addWidget(self.uam_connect_button, 0, 2)
        form.addWidget(self.uam_disconnect_button, 0, 3)
        form.addWidget(self.uam_clear_live_button, 0, 4)
        form.addWidget(QtWidgets.QLabel("ROS2 topic"), 1, 0)
        form.addWidget(self.uam_ros2_topic_edit, 1, 1, 1, 4)
        form.addWidget(QtWidgets.QLabel("ROS2 N/E/D field"), 2, 0)
        form.addWidget(self.uam_ros2_x_field_edit, 2, 1)
        form.addWidget(self.uam_ros2_y_field_edit, 2, 2)
        form.addWidget(self.uam_ros2_z_field_edit, 2, 3)
        form.addWidget(QtWidgets.QLabel("UAM USB port"), 3, 0)
        form.addWidget(self.uam_mavlink_url_edit, 3, 1)
        form.addWidget(QtWidgets.QLabel("Baud"), 3, 2)
        form.addWidget(self.uam_mavlink_baud_spin, 3, 3)
        form.addWidget(QtWidgets.QLabel("Message"), 4, 0)
        form.addWidget(self.uam_mavlink_message_edit, 4, 1)
        form.addWidget(QtWidgets.QLabel("MAVLink N/E/D field"), 5, 0)
        form.addWidget(self.uam_mavlink_x_field_edit, 5, 1)
        form.addWidget(self.uam_mavlink_y_field_edit, 5, 2)
        form.addWidget(self.uam_mavlink_z_field_edit, 5, 3)
        form.addWidget(self.uam_status_label, 6, 0, 1, 5)
        form.addWidget(self.uam_mount_reference_label, 7, 0, 1, 5)
        form.addWidget(self.uam_latest_label, 8, 0, 1, 5)

        self.uam_live_plot = TelemetryHistoryPlot("UAM Ground Truth N/E/D", "m")
        self.uam_live_plot.setMinimumHeight(260)
        group_layout.addLayout(form)
        group_layout.addWidget(self.uam_live_plot, 1)
        layout.addWidget(group, 1)

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

        self.pixhawk_button.setText("ROS2 해제" if self.pixhawk is not None else "ROS2 연결")
        self._set_button_role(self.pixhawk_button, "danger" if self.pixhawk is not None else "primary")

        connected_mavlink_sides = self._mavlink_connected_sides()
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
            self.preview_rate_spin: "Qt 미리보기 표시 최대 주기입니다. 카메라 원본 FPS를 넘지는 않지만 GUI 표시 속도를 제한합니다. 15 Hz가 기본 튜닝값입니다.",
            self.rtsp_latency_spin: "GStreamer RTP jitter buffer 지연 시간입니다. 낮추면 지연은 줄지만 네트워크 흔들림에 민감해집니다.",
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
            self.swap_pixhawk_button: "Pixhawk 1/2 ROS namespace와 MAVLink URL을 서로 바꿉니다. 구독은 새 매핑으로 재시작됩니다.",
            self.check_button: "좌/우 RTSP 호스트의 RTSP 포트가 열려 있는지 확인합니다.",
            self.camera_connect_button: "좌/우 RTSP Preview를 연결하거나, 연결 중이면 카메라 Preview와 활성 YOLO 검출을 중지합니다.",
            self.pixhawk_button: "ROS2 Pixhawk 세션을 연결하거나, 연결 중이면 ROS2 토픽 구독을 중지합니다.",
            self.left_mavlink_button: "Pixhawk 1 MAVLink 세션을 연결하거나, 연결 중이면 P1 MAVLink 링크만 중지합니다.",
            self.right_mavlink_button: "Pixhawk 2 MAVLink 세션을 연결하거나, 연결 중이면 P2 MAVLink 링크만 중지합니다.",
            self.left_uxrce_param_button: "활성 P1 MAVLink 연결로 PX4 UXRCE_DDS_* 파라미터를 Pixhawk Host IP에 맞게 설정하고 확인합니다.",
            self.right_uxrce_param_button: "활성 P2 MAVLink 연결로 PX4 UXRCE_DDS_* 파라미터를 Pixhawk Host IP에 맞게 설정하고 확인합니다.",
            self.uam_source_combo: "UAM/드론 ground truth 위치를 ROS2 topic으로 받을지, MAVLink fallback으로 받을지 선택합니다.",
            self.uam_ros2_topic_edit: "ROS2 ground truth 위치 topic입니다. 기본값은 PX4 vehicle_local_position입니다.",
            self.uam_ros2_x_field_edit: "ROS2 ground truth North field입니다. PX4 local position에서는 보통 x입니다.",
            self.uam_ros2_y_field_edit: "ROS2 ground truth East field입니다. PX4 local position에서는 보통 y입니다.",
            self.uam_ros2_z_field_edit: "ROS2 ground truth Down field입니다. PX4 local position에서는 보통 z입니다.",
            self.uam_mavlink_url_edit: "UAM/드론 PX4 텔레메트리를 받을 별도 USB MAVLink 포트입니다. 예: /dev/ttyACM0, /dev/serial/by-id/...",
            self.uam_mavlink_baud_spin: "UAM USB MAVLink 포트 baud rate입니다.",
            self.uam_mavlink_side_combo: "내부 UAM USB 링크 슬롯입니다. 좌/우 Pixhawk와 분리되어 있습니다.",
            self.uam_mavlink_message_edit: "UAM USB MAVLink 위치 메시지입니다. LOCAL_POSITION_NED는 x=north, y=east, z=down입니다.",
            self.uam_mavlink_x_field_edit: "MAVLink ground truth North field입니다. LOCAL_POSITION_NED에서는 x입니다.",
            self.uam_mavlink_y_field_edit: "MAVLink ground truth East field입니다. LOCAL_POSITION_NED에서는 y입니다.",
            self.uam_mavlink_z_field_edit: "MAVLink ground truth Down field입니다. LOCAL_POSITION_NED에서는 z입니다.",
            self.uam_connect_button: "선택한 ROS2 또는 MAVLink 소스에 uam_x/y/z 감시를 등록하고 실시간 fallback 버퍼를 채웁니다.",
            self.uam_disconnect_button: "UAM ground truth uam_x/y/z 감시만 해제합니다. Pixhawk ROS2/MAVLink 연결과 기존 live buffer는 유지합니다.",
            self.uam_clear_live_button: "실시간 UAM ground truth 버퍼와 N/E/D 그래프를 초기화합니다.",
            self.uam_status_label: "UAM ground truth 텔레메트리 연결 및 버퍼 상태입니다.",
            self.uam_mount_reference_label: "UAM/드론의 GPS 안테나와 Pixhawk가 기체 중앙에 배치된다는 실험 기준입니다.",
            self.uam_latest_label: "최근 수신한 UAM/드론 중심 좌표입니다.",
            self.uam_live_plot: "ULog 또는 실시간 텔레메트리의 N/E/D ground truth 이력입니다.",
            self.uam_ulog_path_edit: "비행 후 비교 분석에 사용할 PX4 ULog(.ulg) 파일 경로입니다.",
            self.uam_ulog_browse_button: "PX4 ULog(.ulg) 파일을 선택하고 즉시 로드합니다.",
            self.uam_load_ulog_button: "입력칸의 현재 .ulg 경로를 다시 로드합니다. 직접 경로를 수정한 뒤 사용합니다.",
            self.uam_truth_source_combo: "Profile 생성/비교에 사용할 truth source를 선택합니다. Auto는 local NED, global position, GPS position 순서로 사용합니다.",
            self.uam_compare_max_dt_spin: "YOLO 측정 timestamp와 ground truth timestamp를 매칭할 때 허용할 최대 시간 차이입니다.",
            self.uam_time_offset_spin: "YOLO 측정 timestamp에 더할 post-flight 시간 보정값입니다.",
            self.uam_run_analysis_button: "ULog, 실시간 UAM buffer, 기록 row fallback 순서로 YOLO 3D 측정값과 ground truth를 비교합니다. ULog/live와 기록 row fallback은 한 결과 안에서 섞지 않습니다. 좌표 보정 profile이 있으면 ground truth를 camera FRD로 역변환해 FRD 축별 오차도 계산합니다.",
            self.uam_estimate_offset_button: "현재 기록 row와 ULog/live buffer의 거리 차이가 가장 작아지는 시간 offset 후보를 추정하고 적용합니다.",
            self.uam_estimate_alignment_button: "현재 매칭된 YOLO Right/Down/Forward와 UAM N/E/D로 축 일치화와 origin offset profile을 자동 추정합니다.",
            self.uam_save_alignment_button: "현재 좌표 보정 profile을 JSON으로 저장합니다. 본 실험 재사용과 결과 재현성 확인에 필요합니다.",
            self.uam_load_alignment_button: "저장된 좌표 보정 profile JSON을 불러와 UAM 비교에 적용합니다.",
            self.uam_clear_alignment_button: "좌표 보정 profile 적용을 해제하고 mixed-frame 비교로 되돌립니다.",
            self.uam_alignment_status_label: "적용 중인 좌표 보정 profile의 샘플 수, RMSE, origin N/E/D offset입니다.",
            self.uam_show_error_button: "UAM 비교 오차 그래프를 표시합니다.",
            self.uam_show_raw_button: "ULog 또는 실시간 UAM ground truth Raw N/E/D 그래프를 표시합니다.",
            self.uam_error_plot: "좌표 보정 profile이 있으면 YOLO Forward/Right/Down과 ground truth를 역변환한 camera FRD의 축별 오차 이력입니다. profile이 없으면 mixed-frame 또는 N/E/D 보조 오차를 표시합니다.",
            self.uam_raw_plot: "ULog 또는 실시간 UAM ground truth Raw N/E/D 이력입니다.",
            self.left_px4_shell_button: "활성 MAVLink 연결을 통해 Pixhawk 1 PX4 shell 창을 엽니다.",
            self.right_px4_shell_button: "활성 MAVLink 연결을 통해 Pixhawk 2 PX4 shell 창을 엽니다.",
            self.left_persist_eth_button: "왼쪽 Pixhawk eth0/MAVLink UDP 설정을 즉시 적용하고 extras.txt에 저장합니다.",
            self.right_persist_eth_button: "오른쪽 Pixhawk eth0/MAVLink UDP 설정을 즉시 적용하고 extras.txt에 저장합니다.",
            self.left_reboot_pixhawk_button: "활성 MAVLink 연결로 Pixhawk 1에 MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN을 보냅니다.",
            self.right_reboot_pixhawk_button: "활성 MAVLink 연결로 Pixhawk 2에 MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN을 보냅니다.",
            self.camera_connection_label: "좌/우 카메라 Preview 연결 상태와 측정된 프레임 속도를 표시합니다.",
            self.runtime_camera_settings_group: "실시간 카메라 화면 위의 실행 버튼, 스트림 설정, 프레임 상태 표시를 접거나 펼칩니다.",
            self.runtime_stream_settings_label: "현재 RTSP 전송 방식(TCP/UDP), 수신 백엔드, 디코더, 지연, 미리보기 속도를 표시합니다.",
            self.runtime_camera_status_group: "해상도/Hz, YOLO Hz, 시각을 한 줄씩 보여주는 상태 그룹입니다.",
            self.left_runtime_stats_label: (
                "측정 탭 왼쪽 카메라의 최신 해상도/Hz, YOLO Hz, 시각입니다.\n"
                "수신 Hz: 스트림 reader가 디코딩한 프레임 속도입니다. 측정 모드에서도 YOLO 추론 완료 속도와 분리됩니다.\n"
                "YOLO Hz: 실제 YOLO 추론이 완료된 속도입니다. -- 는 아직 통계가 없고, 0.0 Hz 는 통계는 있으나 최근 추론 완료가 없다는 뜻입니다.\n"
                "실시간/지연: 최근 스트림 통계나 프레임이 2초 안에 갱신됐는지 표시합니다.\n"
                "시각: GNSS로 UTC에 동기된 경우 UTC를 표시하고, 그 외에는 RTCP Sender Report로 환산한 camera RTC 시각을 표시합니다. 없으면 source timestamp를, 그것도 없으면 missing을 표시합니다."
            ),
            self.right_runtime_stats_label: (
                "측정 탭 오른쪽 카메라의 최신 해상도/Hz, YOLO Hz, 시각입니다.\n"
                "수신 Hz: 스트림 reader가 디코딩한 프레임 속도입니다. 측정 모드에서도 YOLO 추론 완료 속도와 분리됩니다.\n"
                "YOLO Hz: 실제 YOLO 추론이 완료된 속도입니다. -- 는 아직 통계가 없고, 0.0 Hz 는 통계는 있으나 최근 추론 완료가 없다는 뜻입니다.\n"
                "실시간/지연: 최근 스트림 통계나 프레임이 2초 안에 갱신됐는지 표시합니다.\n"
                "시각: GNSS로 UTC에 동기된 경우 UTC를 표시하고, 그 외에는 RTCP Sender Report로 환산한 camera RTC 시각을 표시합니다. 없으면 source timestamp를, 그것도 없으면 missing을 표시합니다."
            ),
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
            self.bias_drone_status_label: "연결 > UAM 또는 legacy row에서 합쳐진 최신 UAM/드론 중심 위치 상태입니다.",
            self.bias_apply_checkbox: "로드된 중심점 bias profile을 실시간 표시/기록값에만 적용합니다. 저장·분석 > UAM 비교는 최종 평가 충돌을 피하기 위해 raw stereo-center 값을 우선 사용합니다.",
            self.bias_bin_size_spin: "후보정 profile을 만들 때 raw 거리 기준으로 묶을 bin 크기입니다.",
            self.bias_build_session_button: "현재 기록된 raw Right/Down/Forward 점과 UAM 중심 N/E/D 차이로 legacy profile을 생성합니다.",
            self.bias_load_csv_button: "저장된 거리 CSV를 불러와 후보정 profile을 생성합니다.",
            self.bias_save_profile_button: "현재 bias profile을 JSON 파일로 저장합니다.",
            self.bias_load_profile_button: "저장된 bias profile JSON을 불러옵니다.",
            self.attitude_plots["roll"]: "Roll 각도 이력입니다. Pixhawk 1과 Pixhawk 2가 같은 그래프에 별도 선으로 표시됩니다.",
            self.attitude_plots["pitch"]: "Pitch 각도 이력입니다. Pixhawk 1과 Pixhawk 2가 같은 그래프에 별도 선으로 표시됩니다.",
            self.attitude_plots["yaw"]: "Yaw 각도 이력입니다. Pixhawk 1과 Pixhawk 2가 같은 그래프에 별도 선으로 표시됩니다.",
            self.use_rtk_baseline_checkbox: "실시간 Pixhawk GPS baseline을 정렬 참고값으로 표시합니다. 거리 계산은 Baseline 입력값을 사용합니다.",
            self.require_rtk_fixed_checkbox: "실시간 GPS baseline 품질 판단에서 두 GPS 위치가 RTK Fixed인지 확인합니다. 꺼져 있으면 일반 3D GPS도 시각화합니다.",
            self.rtk_baseline_label: "실시간 baseline 길이, N/E/D 벡터, 샘플 age, GPS/RTK 품질 경고를 표시합니다.",
            self.rtk_alignment_detail_label: "안테나 baseline과 카메라 기준 Right/Down/Forward 성분을 분리해 표시합니다.",
            self.rtk_alignment_plot: "GPS 안테나 baseline과 카메라 기준 baseline 성분을 시각화합니다.",
            self.left_gps_label: "Pixhawk 1의 최신 GPS/RTK 상태입니다.",
            self.right_gps_label: "Pixhawk 2의 최신 GPS/RTK 상태입니다.",
            self.left_zoom_host_edit: "왼쪽 WIRIS의 OSD IP Address입니다. Telnet/SDK 명령과 RTSP URL host의 기준값입니다.",
            self.right_zoom_host_edit: "오른쪽 WIRIS의 OSD IP Address입니다. Telnet/SDK 명령과 RTSP URL host의 기준값입니다.",
            self.zoom_port_spin: "Telnet port 탐색을 시작할 때 먼저 확인하는 스캔 기본 포트입니다. 좌/우 실제 명령은 각 WIRIS의 Telnet port를 사용합니다.",
            self.zoom_timeout_spin: "WIRIS 줌 명령 TCP 요청 timeout입니다.",
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
            self.wiris_set_pc_time_button: (
                "선택한 WIRIS 대상의 RTC를 PC 현재 UTC로 설정합니다. "
                "양쪽 전송에서는 같은 STUS UTC 값을 만들고 명령 UTC/TCP 호출 시작 차이를 로그에 남깁니다."
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
            self.distance_mode_combo: "3D ray 거리 계산은 Pixhawk 자세와 GPS/RTK baseline translation을 사용합니다.",
            self.measurement_display_combo: "측정탭 좌/우 영상에 실시간 preview를 보여줄지, 거리 계산에 실제로 선택된 검출 프레임 쌍을 보여줄지 선택합니다.",
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
            self.max_dt_spin: (
                "좌/우 검출을 같은 순간의 샘플로 볼 최대 timestamp 차이입니다.\n"
                "거리 그래프는 허용 Δt 안의 최근 후보 중 timestamp 차이가 작은 새 검출 쌍만 기록합니다."
            ),
            self.same_class_checkbox: "좌/우 YOLO 검출 class가 같을 때만 매칭합니다.",
            self.distance_label: "현재 거리 추정값, 계산 방식, 동기화 품질, 경고를 자세히 표시합니다.",
            self.start_button: "YOLO 검출과 3D 거리 측정을 시작하거나, 측정 중이면 Pixhawk 연결은 유지한 채 카메라 Preview로 돌아갑니다.",
            self.record_button: (
                "세션 기록을 시작하거나 중지합니다. 켜면 좌/우 영상 MP4, 거리 샘플 CSV, "
                "시간축 외부 파라미터 CSV를 같은 session id로 묶어 저장합니다."
            ),
            self.clear_history_button: "거리 그래프와 아직 저장하지 않은 메모리 기록을 지웁니다.",
            self.runtime_distance_graph_group: "Right/Down/Forward/Distance 이력 그래프 영역을 접거나 펼칩니다.",
            self.distance_plot: (
                "3D ray 샘플의 Right/Down/Forward/Distance 이력 그래프입니다. 점 색상은 동기화 품질을 따릅니다.\n"
                "왼쪽 클릭: 점의 Right/Down/Forward와 거리값을 표시합니다.\n"
                "오른쪽 클릭: 해당 점을 삭제합니다."
            ),
            self.save_history_button: "현재 세션의 metadata, 거리 샘플, 외부 파라미터, MAG 상태 파일을 디스크에 저장합니다.",
            self.save_settings_button: "현재 내부 파라미터 경로와 전체 런타임 설정을 저장합니다.",
            self.uam_ulog_path_edit: "비행 후 기준 위치를 읽을 PX4 .ulg 파일입니다. ULog가 로드되면 실시간 버퍼보다 우선합니다.",
            self.uam_ulog_browse_button: "PX4 .ulg 파일을 선택하고 즉시 로드합니다.",
            self.uam_load_ulog_button: "입력칸의 현재 .ulg 경로를 다시 로드합니다. 직접 경로를 수정한 뒤 사용합니다.",
            self.uam_truth_source_combo: "ULog/profile 분석에 사용할 source입니다. local/global/GPS는 한 profile 안에서 섞지 않습니다.",
            self.uam_compare_max_dt_spin: "YOLO 샘플과 ground truth 샘플을 같은 시각으로 볼 최대 시간 차이입니다.",
            self.uam_time_offset_spin: "YOLO timestamp에 더할 보정 offset입니다. 비행 후 시간축 보정값을 여기에 입력합니다.",
            self.uam_run_analysis_button: "거리 기록을 ULog, 실시간 버퍼, 기록 행 fallback 순서로 ground truth와 비교합니다. ULog/live 결과에는 기록 행 fallback을 섞지 않습니다. 좌표 보정 profile이 있으면 estimated N/E/D와 truth camera FRD, FRD 축별 오차를 함께 계산합니다.",
            self.uam_analysis_status_label: "UAM 비교 결과 요약입니다.",
            self.uam_analysis_table: "YOLO stereo-center Forward/Right/Down과 ground truth North/East/Down을 시간 기준으로 1:1 매칭한 행별 비교표입니다. profile이 있으면 ground truth를 camera FRD로 역변환한 축별 비교도 표시합니다.",
            self.uam_analysis_detail: "선택한 UAM 비교 행의 전체 필드입니다.",
            self.uam_error_plot: "좌표 보정 profile이 있으면 camera FRD 축별 오차를 표시하고, 없으면 mixed-frame 임시 축별 차이 및 거리 오차를 표시합니다.",
            self.recording_status_label: "메모리 내 거리 기록 상태와 샘플 수입니다.",
            self.recording_sample_table: "저장될 거리 기록 CSV의 주요 컬럼을 새 샘플이 생길 때마다 한 줄씩 표시합니다.",
            self.recording_sample_detail: "선택한 거리 샘플의 전체 CSV 필드를 표시합니다.",
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
        if hasattr(self, "rm3100_yaw_source_combos"):
            for combo in self.rm3100_yaw_source_combos.values():
                combo.setToolTip(
                    "Pixhawk EKF가 Yaw에 사용할 RM3100 후보입니다. CAN1/CAN2를 적용하려면 해당 샘플에 device_id가 있어야 합니다."
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
        rows = self.distance_recording_rows
        if not rows:
            latest = self.distance_history[-1].distance_m if self.distance_history else None
            latest_text = "" if latest is None else f"\n최근 표시 거리: {latest:.2f} m"
            self.session_summary_label.setText(f"아직 기록된 측정 샘플이 없습니다.{latest_text}")
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
        latest = distances[-1] if distances else 0.0
        average = sum(distances) / len(distances) if distances else 0.0
        quality_text = ", ".join(f"{name} {count}" for name, count in sorted(qualities.items()))
        uam_text = ""
        if self.uam_analysis_rows:
            uam_text = f"\n{self._format_uam_analysis_summary()}"
        self.session_summary_label.setText(
            f"기록 샘플: {len(rows)}개\n"
            f"품질 분포: {quality_text or '없음'}\n"
            f"최신 거리: {latest:.2f} m | 평균 거리: {average:.2f} m"
            f"{uam_text}"
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
            self.uam_connect_button,
            self.uam_ulog_browse_button,
            self.uam_load_ulog_button,
            self.uam_run_analysis_button,
            self.uam_estimate_offset_button,
            self.uam_estimate_alignment_button,
            self.uam_save_alignment_button,
            self.uam_load_alignment_button,
            self.uam_clear_alignment_button,
            self.uam_show_error_button,
            self.uam_show_raw_button,
        )
        danger_buttons = (
            self.uam_disconnect_button,
        )
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
            ("gps", "1 GPS", "신호 안정 확인"),
            ("baseline", "2 Baseline", "카메라 간격 조정"),
            ("mag", "3 MAG", "CAN1/CAN2 비교"),
            ("yaw", "4 Yaw", "기준 MAG 선택"),
            ("attitude", "5 자세", "R/P/Y 맞춤"),
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
            "gps": "signal_alignment_tab_index",
            "baseline": "baseline_alignment_tab_index",
            "mag": "rm3100_alignment_tab_index",
            "yaw": "yaw_alignment_tab_index",
            "attitude": "attitude_alignment_tab_index",
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
        if gps_count == 2:
            self._set_alignment_step_status("gps", "P1/P2 GPS OK", "good")
        elif gps_count:
            self._set_alignment_step_status("gps", f"GPS {gps_count}/2", "warn")
        else:
            self._set_alignment_step_status("gps", "GPS 대기", "idle")

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

        mag_count = sum(1 for side in ("left", "right") for slot in ("CAN1", "CAN2") if slot in self.latest_mag_sensors.get(side, {}))
        if mag_count >= 4:
            self._set_alignment_step_status("mag", "CAN1/CAN2 OK", "good")
        elif mag_count:
            self._set_alignment_step_status("mag", f"MAG {mag_count}/4", "warn")
        else:
            self._set_alignment_step_status("mag", "MAG 대기", "idle")

        selected_mag_count = sum(1 for value in self.latest_vehicle_mag_device_ids.values() if value is not None)
        if selected_mag_count >= 2:
            self._set_alignment_step_status("yaw", "Yaw 기준 확인", "good")
        elif mag_count:
            self._set_alignment_step_status("yaw", "Yaw 선택 필요", "warn")
        else:
            self._set_alignment_step_status("yaw", "Yaw 대기", "idle")

        left = self.latest_extrinsics.get("left")
        right = self.latest_extrinsics.get("right")
        if left is not None and right is not None:
            deltas = {
                "roll": right.roll_deg - left.roll_deg,
                "pitch": right.pitch_deg - left.pitch_deg,
                "yaw": right.yaw_deg - left.yaw_deg,
            }
            levels = [self._attitude_delta_level(axis, delta) for axis, delta in deltas.items()]
            level = "bad" if "bad" in levels else "warn" if "warn" in levels else "good"
            self._set_alignment_step_status(
                "attitude",
                f"ΔR/P/Y {deltas['roll']:+.1f}/{deltas['pitch']:+.1f}/{deltas['yaw']:+.1f}°",
                level,
            )
        elif left is not None or right is not None:
            self._set_alignment_step_status("attitude", "자세 1/2", "warn")
        else:
            self._set_alignment_step_status("attitude", "자세 대기", "idle")

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
        self.rm3100_yaw_source_combos: dict[str, QtWidgets.QComboBox] = {}
        self.rm3100_yaw_source_status_labels: dict[str, QtWidgets.QLabel] = {}
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

    def _build_baseline_alignment_tab(self, layout: QtWidgets.QVBoxLayout) -> None:
        layout.setSpacing(8)
        self.baseline_sampling_active = False
        self.baseline_sampling_started_s: float | None = None
        self.baseline_samples: list[tuple[BaselineVector, GpsPositionState, GpsPositionState]] = []
        self.latest_baseline_quality: BaselineQuality | None = None
        self.latest_baseline_vector: BaselineVector | None = None
        self.loaded_baseline_profile_vector: tuple[float, float, float] | None = None
        self.baseline_gps_rate_history: dict[str, deque[float]] = {"left": deque(maxlen=120), "right": deque(maxlen=120)}

        self.baseline_tabs = QtWidgets.QTabWidget()
        self.baseline_tabs.setDocumentMode(True)
        gps_tab = QtWidgets.QWidget()
        fields_tab = QtWidgets.QWidget()
        calc_tab = QtWidgets.QWidget()
        quality_tab = QtWidgets.QWidget()
        profile_tab = QtWidgets.QWidget()
        gps_layout = QtWidgets.QVBoxLayout(gps_tab)
        fields_layout = QtWidgets.QVBoxLayout(fields_tab)
        calc_layout = QtWidgets.QVBoxLayout(calc_tab)
        quality_layout = QtWidgets.QVBoxLayout(quality_tab)
        profile_layout = QtWidgets.QVBoxLayout(profile_tab)
        for child_layout in (gps_layout, fields_layout, calc_layout, quality_layout, profile_layout):
            child_layout.setContentsMargins(6, 6, 6, 6)
            child_layout.setSpacing(8)
        self._build_baseline_gps_recognition_view(gps_layout)
        self._build_baseline_field_browser_view(fields_layout)
        self._build_baseline_calculation_view(calc_layout)
        self._build_baseline_quality_view(quality_layout)
        self._build_baseline_profile_view(profile_layout)
        self.baseline_tabs.addTab(gps_tab, "GPS 인식")
        self.baseline_tabs.addTab(fields_tab, "ROS2 Topic / MAVLink Field")
        self.baseline_tabs.addTab(calc_tab, "Baseline 계산")
        self.baseline_tabs.addTab(quality_tab, "신뢰성 평가")
        self.baseline_tabs.addTab(profile_tab, "Baseline Profile")
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
            ("left_rpy", "Left Roll/Pitch/Yaw"),
            ("right_rpy", "Right Roll/Pitch/Yaw"),
            ("yaw_diff", "Left-Right Yaw Difference"),
            ("pitch_diff", "Left-Right Pitch Difference"),
            ("roll_diff", "Left-Right Roll Difference"),
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

    def _build_alignment_magnetometer_tab(self, layout: QtWidgets.QVBoxLayout) -> None:
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

        status_group = QtWidgets.QGroupBox("CAN1/CAN2 Magnetometer 상태")
        grid = QtWidgets.QGridLayout(status_group)
        headers = ("Side", "Sensor", "Device ID", "Priority", "Selected", "Health", "Calibration", "Field Strength", "Rate", "Timestamp")
        for column, header in enumerate(headers):
            grid.addWidget(QtWidgets.QLabel(header), 0, column)
        self.alignment_mag_labels: dict[str, dict[str, QtWidgets.QLabel]] = {}
        row = 1
        for side, side_label in (("left", "Left"), ("right", "Right")):
            for slot in ("CAN1", "CAN2"):
                key = f"{side}_{slot}"
                self.alignment_mag_labels[key] = {}
                grid.addWidget(QtWidgets.QLabel(side_label), row, 0)
                grid.addWidget(QtWidgets.QLabel(slot), row, 1)
                for column, field in enumerate(("device", "priority", "selected", "health", "calibration", "field", "rate", "timestamp"), start=2):
                    label = QtWidgets.QLabel("대기 중")
                    label.setObjectName("statusStrip")
                    label.setWordWrap(True)
                    grid.addWidget(label, row, column)
                    self.alignment_mag_labels[key][field] = label
                row += 1
        layout.addWidget(status_group)
        compare_group = QtWidgets.QGroupBox("Heading / Yaw 비교")
        compare_form = QtWidgets.QGridLayout(compare_group)
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
            compare_form.addWidget(QtWidgets.QLabel(title), row, 0)
            compare_form.addWidget(label, row, 1)
            self.alignment_mag_compare_labels[key] = label
        layout.addWidget(compare_group)

        yaw_group = QtWidgets.QGroupBox("Yaw 기준 Magnetometer 선택")
        yaw_form = QtWidgets.QGridLayout(yaw_group)
        for row, (side, title) in enumerate((("left", "Left Pixhawk"), ("right", "Right Pixhawk"))):
            combo = QtWidgets.QComboBox()
            combo.addItem("PX4 현재 설정", "current")
            combo.addItem("CAN1", "CAN1")
            combo.addItem("CAN2", "CAN2")
            button = QtWidgets.QPushButton("선택 적용")
            status = QtWidgets.QLabel("Yaw 기준: PX4 현재 설정")
            status.setObjectName("statusStrip")
            status.setWordWrap(True)
            button.clicked.connect(lambda _checked=False, camera=side: self.apply_rm3100_yaw_source(camera))
            yaw_form.addWidget(QtWidgets.QLabel(title), row, 0)
            yaw_form.addWidget(combo, row, 1)
            yaw_form.addWidget(button, row, 2)
            yaw_form.addWidget(status, row, 3)
            self.rm3100_yaw_source_combos[side] = combo
            self.rm3100_yaw_source_status_labels[side] = status
        layout.addWidget(yaw_group)

        self.alignment_mag_heading_plot = RtkAlignmentPlot()
        layout.addWidget(self.alignment_mag_heading_plot, 1)
        self._build_px4_mag_manager_panel(layout)
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
        layout.addWidget(group)

        compare_group = QtWidgets.QGroupBox("자세 축 비교")
        compare_form = QtWidgets.QGridLayout(compare_group)
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
        layout.addWidget(compare_group)

        self.alignment_optical_axis_plot = RtkAlignmentPlot()
        layout.addWidget(self.alignment_optical_axis_plot, 1)
        self.attitude_plots = {
            "roll": AttitudeHistoryPlot("Roll"),
            "pitch": AttitudeHistoryPlot("Pitch"),
            "yaw": AttitudeHistoryPlot("Yaw"),
        }
        for plot in self.attitude_plots.values():
            layout.addWidget(plot, 1)

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
        hint = QtWidgets.QLabel("GPS1/GPS2가 모두 있으면 Baseline 계산 화면에서 사용할 수신기를 선택합니다. 위치 계산은 raw GNSS/RTK 필드를 우선 사용합니다.")
        hint.setObjectName("statusStrip")
        hint.setWordWrap(True)
        layout.addWidget(hint)

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

    def _build_baseline_calculation_view(self, layout: QtWidgets.QVBoxLayout) -> None:
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
        form.addWidget(self.use_rtk_baseline_checkbox, 1, 0, 1, 2)
        form.addWidget(self.require_rtk_fixed_checkbox, 1, 2, 1, 2)
        layout.addWidget(control_group)

        summary_group = QtWidgets.QGroupBox("Baseline 계산 결과")
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
                ("profile_delta", "profile delta"),
            )
        ):
            label = QtWidgets.QLabel("대기 중")
            label.setObjectName("statusStrip")
            label.setWordWrap(True)
            summary_form.addWidget(QtWidgets.QLabel(label_text), row, 0)
            summary_form.addWidget(label, row, 1)
            self.baseline_calc_labels[key] = label
        layout.addWidget(summary_group)

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
        self._refresh_baseline_alignment_ui()

    def _build_baseline_profile_view(self, layout: QtWidgets.QVBoxLayout) -> None:
        actions = QtWidgets.QHBoxLayout()
        self.baseline_save_profile_button = QtWidgets.QPushButton("Save Baseline Profile")
        self.baseline_load_profile_button = QtWidgets.QPushButton("Load Profile")
        self.baseline_save_profile_button.clicked.connect(self.save_baseline_profile)
        self.baseline_load_profile_button.clicked.connect(self.load_baseline_profile)
        actions.addWidget(self.baseline_save_profile_button)
        actions.addWidget(self.baseline_load_profile_button)
        actions.addStretch(1)
        layout.addLayout(actions)
        self.baseline_profile_status_label = QtWidgets.QLabel("Baseline profile: 대기 중")
        self.baseline_profile_status_label.setObjectName("statusStrip")
        self.baseline_profile_status_label.setWordWrap(True)
        layout.addWidget(self.baseline_profile_status_label)
        self.baseline_profile_preview = QtWidgets.QPlainTextEdit()
        self.baseline_profile_preview.setReadOnly(True)
        layout.addWidget(self.baseline_profile_preview, 1)

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
        combo = self.baseline_left_gps_combo if side == "left" else self.baseline_right_gps_combo
        slot = str(combo.currentData() or "GPS1") if hasattr(self, "baseline_left_gps_combo") else "GPS1"
        if slot == "GPS1":
            return self.latest_gps.get(side)
        # GPS2 position is only usable when the producer emits raw GPS2 lat/lon into latest_gps.
        state = self.latest_gps.get(f"{side}_GPS2")
        return state if isinstance(state, GpsPositionState) else None

    def _current_baseline_vector(self) -> tuple[BaselineVector, GpsPositionState, GpsPositionState] | None:
        left = self._selected_baseline_gps_state("left")
        right = self._selected_baseline_gps_state("right")
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
            if self.loaded_baseline_profile_vector is None:
                self._set_health_label(self.baseline_calc_labels["profile_delta"], "profile 없음", "idle")
            else:
                delta = sqrt(
                    (vector.east_m - self.loaded_baseline_profile_vector[0]) ** 2
                    + (vector.north_m - self.loaded_baseline_profile_vector[1]) ** 2
                    + (vector.up_m - self.loaded_baseline_profile_vector[2]) ** 2
                )
                self._set_health_label(
                    self.baseline_calc_labels["profile_delta"],
                    f"{delta:.3f} m",
                    "warn" if delta >= 0.05 else "good",
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
            self.rtk_alignment_plot.set_alignment(estimate)
            self._set_label_text_if_changed(
                self.rtk_alignment_detail_label,
                f"ENU E/N/U={vector.east_m:+.3f}/{vector.north_m:+.3f}/{vector.up_m:+.3f} m | "
                f"NED N/E/D={vector.north_m:+.3f}/{vector.east_m:+.3f}/{-vector.up_m:+.3f} m",
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
        left = self.latest_extrinsics.get("left")
        right = self.latest_extrinsics.get("right")
        labels["left_rpy"].setText("대기 중" if left is None else f"{left.roll_deg:.2f}/{left.pitch_deg:.2f}/{left.yaw_deg:.2f} deg")
        labels["right_rpy"].setText("대기 중" if right is None else f"{right.roll_deg:.2f}/{right.pitch_deg:.2f}/{right.yaw_deg:.2f} deg")
        if left is not None and right is not None:
            labels["yaw_diff"].setText(f"{right.yaw_deg - left.yaw_deg:+.2f} deg")
            labels["pitch_diff"].setText(f"{right.pitch_deg - left.pitch_deg:+.2f} deg")
            labels["roll_diff"].setText(f"{right.roll_deg - left.roll_deg:+.2f} deg")
        grade = "GOOD" if quality is not None and quality.grade == "GOOD" else "WARNING" if vector is not None else "BAD"
        self._set_health_label(labels["alignment_quality"], grade, "good" if grade == "GOOD" else "warn" if grade == "WARNING" else "bad")

    def _refresh_alignment_magnetometer_ui(self) -> None:
        if not hasattr(self, "alignment_mag_labels"):
            return
        for side in ("left", "right"):
            for slot in ("CAN1", "CAN2"):
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
                labels["device"].setText("--" if device_id is None else str(device_id))
                labels["priority"].setText("PX4 param")
                labels["selected"].setText(device_source)
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
            if candidate in {RM3100_PRIMARY_ID, RM3100_BACKUP_ID} and candidate not in rm_ids:
                rm_ids.append(candidate)
        if sensor_slot == "CAN1":
            return rm_ids[0] if rm_ids else RM3100_PRIMARY_ID
        if sensor_slot == "CAN2":
            if len(rm_ids) >= 2:
                return rm_ids[1]
            return RM3100_BACKUP_ID
        return None

    def _refresh_alignment_attitude_ui(self) -> None:
        if not hasattr(self, "alignment_attitude_labels"):
            return
        for side, prefix in (("left", "left"), ("right", "right")):
            state = self.latest_extrinsics.get(side)
            if state is None:
                self.alignment_attitude_labels[f"{prefix}_rpy"].setText("대기 중")
                self.alignment_attitude_labels[f"{prefix}_q"].setText("대기 중")
            else:
                self.alignment_attitude_labels[f"{prefix}_rpy"].setText(
                    f"{state.roll_deg:.3f} / {state.pitch_deg:.3f} / {state.yaw_deg:.3f} deg"
                )
                self.alignment_attitude_labels[f"{prefix}_q"].setText(", ".join(f"{value:.6f}" for value in state.q))
        left = self.latest_extrinsics.get("left")
        right = self.latest_extrinsics.get("right")
        if left is not None and right is not None:
            self.alignment_attitude_labels["yaw_diff"].setText(f"{right.yaw_deg - left.yaw_deg:+.3f} deg")
            self.alignment_attitude_labels["pitch_diff"].setText(f"{right.pitch_deg - left.pitch_deg:+.3f} deg")
            self.alignment_attitude_labels["roll_diff"].setText(f"{right.roll_deg - left.roll_deg:+.3f} deg")

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

    def _baseline_profile_topics(self) -> tuple[str, str, str, str]:
        return (
            self._gps_sensor_topic("left", "GPS1"),
            self._gps_sensor_topic("left", "GPS2"),
            self._gps_sensor_topic("right", "GPS1"),
            self._gps_sensor_topic("right", "GPS2"),
        )

    def _gps_sensor_topic(self, side: str, slot: str) -> str:
        state = self.latest_gps_sensors.get(side, {}).get(slot)
        return "" if state is None else str(getattr(state, "topic", ""))

    def save_baseline_profile(self) -> None:
        current = self._current_baseline_vector()
        if current is None:
            QtWidgets.QMessageBox.warning(self, "Baseline Profile", "저장할 유효한 baseline이 없습니다.")
            return
        vector, left, right = current
        if left.fix_type != 6 or right.fix_type != 6:
            QtWidgets.QMessageBox.warning(self, "Baseline Profile", "RTK Fixed가 아닌 상태입니다. 저장 전 품질을 확인하세요.")
        selected, _filter = QtWidgets.QFileDialog.getSaveFileName(
            self,
            "Save Baseline Profile",
            str(APP_ROOT / "baseline_profile.yaml"),
            "YAML (*.yaml *.yml)",
        )
        if not selected:
            return
        payload = baseline_profile_payload(
            left_gps1_topic=self._baseline_profile_topics()[0],
            left_gps2_topic=self._baseline_profile_topics()[1],
            right_gps1_topic=self._baseline_profile_topics()[2],
            right_gps2_topic=self._baseline_profile_topics()[3],
            baseline=vector,
            quality=self.latest_baseline_quality,
        )
        path = Path(selected)
        path.write_text(dump_baseline_profile(payload), encoding="utf-8")
        self.baseline_profile_preview.setPlainText(dump_baseline_profile(payload))
        self.baseline_profile_status_label.setText(f"Baseline profile 저장: {path}")

    def load_baseline_profile(self) -> None:
        selected, _filter = QtWidgets.QFileDialog.getOpenFileName(
            self,
            "Load Baseline Profile",
            str(APP_ROOT),
            "YAML (*.yaml *.yml)",
        )
        if not selected:
            return
        path = Path(selected)
        try:
            payload = load_baseline_profile(path.read_text(encoding="utf-8"))
        except Exception as exc:
            QtWidgets.QMessageBox.warning(self, "Baseline Profile", str(exc))
            return
        profile = payload["baseline_profile"]
        vector = profile.get("baseline", {}).get("vector_m", [0.0, 0.0, 0.0])
        if isinstance(vector, list) and len(vector) == 3:
            self.loaded_baseline_profile_vector = (float(vector[0]), float(vector[1]), float(vector[2]))
        self.baseline_profile_preview.setPlainText(path.read_text(encoding="utf-8"))
        self.baseline_profile_status_label.setText(f"Baseline profile 불러옴: {path}")
        self._refresh_baseline_alignment_ui()

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

    def _build_px4_mag_manager_panel(self, layout: QtWidgets.QVBoxLayout) -> None:
        group = QtWidgets.QGroupBox("PX4 Magnetometer / RM3100 관리")
        group_layout = QtWidgets.QVBoxLayout(group)

        self.px4_mag_tables: dict[str, QtWidgets.QTableWidget] = {}
        self.px4_mag_status_labels: dict[str, QtWidgets.QLabel] = {}
        self.px4_mag_refresh_buttons: dict[str, QtWidgets.QPushButton] = {}
        self.px4_mag_use_rm3100_buttons: dict[str, QtWidgets.QPushButton] = {}
        self.px4_mag_save_buttons: dict[str, QtWidgets.QPushButton] = {}
        self.px4_mag_reboot_buttons: dict[str, QtWidgets.QPushButton] = {}

        self.px4_mag_tab_widget = QtWidgets.QTabWidget()
        for camera, title in (("left", "Pixhawk 1"), ("right", "Pixhawk 2")):
            tab = QtWidgets.QWidget()
            tab_layout = QtWidgets.QVBoxLayout(tab)
            self._build_single_px4_mag_manager_tab(tab_layout, camera)
            self.px4_mag_tab_widget.addTab(tab, title)
        group_layout.addWidget(self.px4_mag_tab_widget)
        layout.addWidget(group)
        self.refresh_px4_mag_manager()

    def _build_single_px4_mag_manager_tab(self, layout: QtWidgets.QVBoxLayout, camera: str) -> None:
        control_row = QtWidgets.QHBoxLayout()
        refresh_button = QtWidgets.QPushButton("새로고침")
        use_rm3100_button = QtWidgets.QPushButton("RM3100을 Yaw 기준으로")
        save_button = QtWidgets.QPushButton("파라미터 저장")
        reboot_button = QtWidgets.QPushButton("PX4 재부팅")
        for button in (
            refresh_button,
            use_rm3100_button,
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
        self.px4_mag_use_rm3100_buttons[camera] = use_rm3100_button
        self.px4_mag_save_buttons[camera] = save_button
        self.px4_mag_reboot_buttons[camera] = reboot_button
        self.px4_mag_status_labels[camera] = status_label
        self.px4_mag_tables[camera] = table

        refresh_button.clicked.connect(lambda _checked=False, side=camera: self.refresh_px4_mag_manager(side))
        use_rm3100_button.clicked.connect(lambda _checked=False, side=camera: self.use_rm3100_for_yaw_from_panel(side))
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
        yaw_source_combo = QtWidgets.QComboBox()
        yaw_source_combo.addItem("PX4 현재 설정", "current")
        yaw_source_combo.addItem("CAN1", "CAN1")
        yaw_source_combo.addItem("CAN2", "CAN2")
        apply_yaw_source_button = QtWidgets.QPushButton("선택 적용")
        apply_yaw_source_button.clicked.connect(
            lambda _checked=False, camera=camera: self.apply_rm3100_yaw_source(camera)
        )
        yaw_source_status = QtWidgets.QLabel("Yaw 기준: PX4 현재 설정")
        yaw_source_status.setObjectName("statusStrip")
        yaw_source_row = QtWidgets.QHBoxLayout()
        yaw_source_row.setContentsMargins(0, 0, 0, 0)
        yaw_source_row.addWidget(yaw_source_combo, 1)
        yaw_source_row.addWidget(apply_yaw_source_button)
        rm_form.addWidget(QtWidgets.QLabel("PX4 Yaw 기준(CAL_MAG_PRIME)"), 5, 0)
        rm_form.addLayout(yaw_source_row, 5, 1)
        rm_form.addWidget(yaw_source_status, 6, 1)
        self.rm3100_yaw_source_combos[camera] = yaw_source_combo
        self.rm3100_yaw_source_status_labels[camera] = yaw_source_status
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
            "display_rate_hz": self.preview_rate_spin.value(),
            "preview_max_width": self.preview_width_spin.value(),
            "rtsp_latency_ms": self.rtsp_latency_spin.value(),
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
        root, _left_dir, _right_dir = self._calibration_dirs()
        return session_dir(root, stamp or self.recording_session_id)

    def _prepare_video_recording_paths(self) -> None:
        root, _left_dir, _right_dir = self._calibration_dirs()
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
            ]
            + self._mavlink_url_candidate_rows()
        )
        if namespaces:
            self.left_namespace_edit.setText(namespaces[0])
        if len(namespaces) > 1:
            self.right_namespace_edit.setText(namespaces[1])
        self._log(
            f"[픽스호크] 자동 탐색: ROS_DOMAIN_ID={discovery.domain_id}, "
            f"namespace={namespaces or ['<root>']}, topic={len(discovery.topics)}개"
        )

    def _set_pixhawk_candidate_rows(self, rows: list[dict[str, object]]) -> None:
        self._pixhawk_candidate_rows = rows
        self.pixhawk_candidate_table.setRowCount(len(rows))
        for row, entry in enumerate(rows):
            for column, key in enumerate(("kind", "value", "baud", "status", "interface")):
                item = QtWidgets.QTableWidgetItem(str(entry.get(key, "")))
                self.pixhawk_candidate_table.setItem(row, column, item)
        self.pixhawk_candidate_table.resizeRowsToContents()
        self._refresh_pixhawk_candidate_actions()

    def _append_pixhawk_candidate_rows(self, rows: list[dict[str, object]]) -> None:
        self._set_pixhawk_candidate_rows([*self._pixhawk_candidate_rows, *rows])

    def clear_pixhawk_candidate_history(self) -> None:
        if not self._pixhawk_candidate_rows:
            return
        self._set_pixhawk_candidate_rows([])
        self._log("[픽스호크] 탐색 후보 내역을 지웠습니다.")

    def _refresh_pixhawk_candidate_actions(self) -> None:
        row = self.pixhawk_candidate_table.currentRow()
        kind = ""
        if 0 <= row < len(self._pixhawk_candidate_rows):
            kind = str(self._pixhawk_candidate_rows[row].get("kind", ""))
        self.clear_pixhawk_candidate_button.setEnabled(bool(self._pixhawk_candidate_rows))
        self.clear_pixhawk_candidate_button.setToolTip("Pixhawk 탐색 후보 내역을 지웁니다.")
        self.apply_pixhawk_candidate_host_button.setEnabled(kind == "PC_IP")
        self.apply_pixhawk_candidate_host_button.setToolTip(
            "선택한 PC IP 후보를 Pixhawk Host IP/uXRCE Agent IP에 적용합니다."
            if kind == "PC_IP"
            else "PC_IP 후보를 선택하면 Host에 적용할 수 있습니다."
        )
        side_enabled = kind in {"ROS2", "USB", "MAVLink"}
        for button in (self.apply_pixhawk_candidate_left_button, self.apply_pixhawk_candidate_right_button):
            button.setEnabled(side_enabled)
            if kind == "ROS2":
                button.setToolTip("선택한 ROS2 namespace 후보를 Pixhawk에 적용합니다.")
            elif kind == "USB":
                button.setToolTip("선택한 USB 후보를 P1/P2 MAVLink URL에 적용합니다. 연결은 적용 후 직접 누르세요.")
            elif kind == "MAVLink":
                button.setToolTip("선택한 MAVLink URL 후보를 P1/P2 MAVLink URL에 적용합니다. 연결은 적용 후 직접 누르세요.")
            else:
                button.setToolTip("ROS2, USB, MAVLink 후보를 선택하면 P1/P2에 적용할 수 있습니다.")

    def _mavlink_url_candidate_rows(self) -> list[dict[str, object]]:
        host_ip = self.pixhawk_host_address_edit.text().strip() or "0.0.0.0"
        return [
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

    def discover_mavlink_url_candidates(self) -> None:
        rows = self._mavlink_url_candidate_rows()
        self._append_pixhawk_candidate_rows(rows)
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
        if not rows:
            self._log("[픽스호크] USB Pixhawk 장치가 보이지 않습니다: /dev/ttyACM*, /dev/ttyUSB* 없음")
            return
        self._append_pixhawk_candidate_rows(rows)
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

    def _selected_pixhawk_candidate(self) -> dict[str, object] | None:
        row = self.pixhawk_candidate_table.currentRow()
        if row < 0 or row >= len(self._pixhawk_candidate_rows):
            self._log("[픽스호크] 탐색 후보를 먼저 선택하세요.")
            return None
        return self._pixhawk_candidate_rows[row]

    def apply_selected_pixhawk_candidate(self, side: str) -> None:
        entry = self._selected_pixhawk_candidate()
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
        mavlink_was_active = side in self._mavlink_connected_sides()
        ros2_was_active = self.pixhawk is not None
        if kind == "ROS2":
            if side == "left":
                self.left_namespace_edit.setText(value)
            else:
                self.right_namespace_edit.setText(value)
        elif kind in {"USB", "MAVLink"}:
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
            if mavlink_was_active:
                self._log(f"[픽스호크] {label} MAVLink URL 후보 적용으로 기존 {label} MAVLink 세션을 해제합니다. 다시 연결하세요.")
                self.disconnect_mavlink_pixhawk(side)
            self.pixhawk_connection_label.setText(f"{label} MAVLink URL 후보 적용됨: {label} MAVLink 연결을 누르세요.")
        elif kind == "ROS2":
            if ros2_was_active:
                self._log("[픽스호크] ROS2 후보 적용으로 ROS2 세션을 새 namespace로 재시작합니다.")
                self.disconnect_pixhawk()
                self.connect_pixhawk()
            else:
                self.pixhawk_connection_label.setText(f"{label} ROS2 후보 적용됨: ROS2 연결을 누르세요.")
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
        was_connected = self.pixhawk is not None
        mavlink_connected_sides = self._mavlink_connected_sides()
        if was_connected:
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
        if was_connected:
            self.connect_pixhawk()
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

    def _format_runtime_stream_settings(self) -> str:
        transport = self.rtsp_transport_combo.currentText().strip().upper() or "--"
        backend = self.stream_backend_combo.currentText().strip() or "--"
        decoder = self._runtime_decoder_text()
        return (
            f"스트림: RTSP 전송={transport} | 백엔드={backend} | 디코더={decoder} | "
            f"RTP 지연={self.rtsp_latency_spin.value()} ms | 미리보기={self.preview_rate_spin.value():.1f} Hz"
        )

    def _update_runtime_stream_settings_label(self, *_args) -> None:
        if hasattr(self, "runtime_stream_settings_label"):
            self.runtime_stream_settings_label.setText(self._format_runtime_stream_settings())

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
            metadata = self.camera_frame_metadata.get(camera) or {}
            label.setText(
                format_runtime_camera_status(
                    camera,
                    stats,
                    metadata,
                    self._sample_hz(timestamps),
                    stale=not (has_recent_stats or has_recent_frame),
                )
            )

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
        if mode == "pixhawk":
            self._distance_mode_last_source = mode
        self._update_distance_settings_visibility()
        self._refresh_camera_parameters_tab()

    def _handle_gps_baseline_toggle_changed(self, _checked: bool) -> None:
        self._refresh_rtk_baseline()
        self._refresh_camera_parameters_tab()
        self._refresh_distance()

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
        if baseline_source == "gps-rtk-disabled":
            return "GPS/RTK baseline disabled"
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
        return "pixhawk"

    def _measurement_display_mode(self) -> str:
        if not hasattr(self, "measurement_display_combo"):
            mode = str(getattr(self.settings, "measurement_display_mode", "detection"))
        else:
            mode = str(self.measurement_display_combo.currentData() or "detection")
        return "preview" if mode == "preview" else "detection"

    def _distance_mode_label(self) -> str:
        return "Pixhawk 자세 + GPS/RTK baseline"

    def _triangulation_extrinsics_source(self) -> str:
        return "pixhawk"

    def _measurement_path_label(self) -> str:
        if self.active_intrinsics is None:
            return "3D ray fallback: 내부 파라미터 대기 중"
        left_ext = self.latest_extrinsics.get("left")
        right_ext = self.latest_extrinsics.get("right")
        if left_ext is None or right_ext is None:
            return "3D ray fallback: Pixhawk 자세 대기 중"
        if self._translation_right_in_left_m() is None:
            return "3D ray fallback: GPS/RTK baseline 대기 중"
        return "3D ray 거리: Pixhawk 자세 + GPS/RTK baseline 사용"

    def _triangulation_extrinsics_source_label(self) -> str:
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
        baseline_source = "gps-rtk-waiting"
        baseline_m = 0.0
        warnings: tuple[str, ...] = ()
        if hasattr(self, "use_rtk_baseline_checkbox"):
            baseline_m, baseline_source, warnings = self._active_baseline_m()
        translation = self._translation_right_in_left_m()
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
            f"{self._format_mount_assumption_summary()}\n\n"
            f"{self._bbox_center_adjustment_summary()}\n\n"
            f"{self._format_uam_telemetry_summary()}\n\n"
            f"max_detection_dt: {self.max_dt_spin.value():.6f} s\n"
            "max_vertical_delta: disabled\n"
            f"same_class_required: {self.same_class_checkbox.isChecked()}\n"
            "attitude_gate: false"
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
        frame_timestamp_s = float(frame_metadata.get("timestamp_s", now_s))
        frame_metadata.setdefault("timestamp_s", frame_timestamp_s)
        frame_metadata.setdefault("source_timestamp_s", None)
        frame_metadata.setdefault("timestamp_source", "app-receive")
        frame_metadata.setdefault("timestamp_reference", infer_runtime_camera_timestamp_reference(frame_metadata))
        self.camera_frame_metadata[camera] = frame_metadata
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

    def toggle_ros2_connection(self) -> None:
        if self.pixhawk is not None:
            self.disconnect_pixhawk()
        else:
            self.connect_pixhawk()

    def toggle_mavlink_connection(self, camera: str | None = None) -> None:
        if camera is None:
            if self._mavlink_connected_sides():
                self.disconnect_mavlink_pixhawk()
            else:
                self.connect_mavlink_pixhawk()
            return
        if camera in self._mavlink_connected_sides():
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
        self._update_runtime_stream_settings_label()
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
        self._stop_camera_panes(clear_previews=True)
        if self._session_recording_active():
            self.distance_recording_enabled = False
            self.video_recording_enabled = False
            self.distance_recording_started_monotonic_s = None
            self.record_button.setText("세션 기록 시작")
            self._log(f"[저장] 카메라 Preview 해제로 세션 기록을 중지했습니다: {self._format_video_recording_paths()}")
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
        self.camera_stream_stats["left"] = {}
        self.camera_stream_stats["right"] = {}
        self.active_stream_decoders["left"] = None
        self.active_stream_decoders["right"] = None
        self._update_runtime_camera_stats_labels()
        self._update_runtime_stream_settings_label()
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

    def _distance_history_key(self, point: DistanceHistoryPoint) -> tuple[float, float, str]:
        return point.sample_key or (point.timestamp_s, point.distance_m, point.method)

    def _refresh_recording_status_label(self) -> None:
        state = "진행 중" if self._session_recording_active() else "중지"
        session = f", session={self.recording_session_id}" if self.recording_session_id else ""
        self.recording_status_label.setText(
            "세션 기록: "
            f"{state}, 거리 샘플={len(self.distance_recording_rows)}, "
            f"외부 파라미터={len(self.external_parameter_recording_rows)}{session}"
        )

    def _recording_table_value(self, row: dict[str, object], field: str) -> str:
        fallback_fields = {
            "telemetry_range_m": "telemetry_baseline_center_distance_m",
            "telemetry_range_error_m": "telemetry_baseline_range_error_m",
        }
        if field not in row and field in fallback_fields:
            field = fallback_fields[field]
        value = row.get(field, "")
        if value is None:
            return ""
        return str(value)

    def _append_recording_table_row(self, row: dict[str, object]) -> None:
        if not hasattr(self, "recording_sample_table"):
            return
        table = self.recording_sample_table
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
        for row in self.distance_recording_rows:
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
        if row_index < 0 or row_index >= len(self.distance_recording_rows):
            self.recording_sample_detail.clear()
            return
        row = self.distance_recording_rows[row_index]
        lines = [f"{key}: {value}" for key, value in row.items()]
        self.recording_sample_detail.setPlainText("\n".join(lines))

    def browse_uam_ulog_file(self) -> None:
        start_path = self._calibration_dirs()[0]
        selected, _filter = QtWidgets.QFileDialog.getOpenFileName(
            self,
            "Select PX4 ULog",
            str(start_path),
            "PX4 ULog (*.ulg);;All files (*)",
        )
        if not selected:
            return
        self.uam_ulog_path_edit.setText(selected)
        self.load_uam_ulog_file()

    def _selected_uam_truth_source_type(self) -> str:
        if not hasattr(self, "uam_truth_source_combo"):
            return TRUTH_SOURCE_AUTO
        return str(self.uam_truth_source_combo.currentData() or TRUTH_SOURCE_AUTO)

    def _on_uam_truth_source_changed(self) -> None:
        self.uam_ulog_samples = []
        self.uam_ulog_path = None
        self.uam_ulog_truth_source_type = TRUTH_SOURCE_AUTO
        self.uam_analysis_rows = []
        self._refresh_uam_analysis_table()
        self._refresh_uam_error_plot()
        source = self._selected_uam_truth_source_type()
        self.uam_analysis_status_label.setText(
            f"UAM 비교: truth source={source}. ULog를 다시 로드하거나 비교를 실행하세요."
        )

    def load_uam_ulog_file(self) -> bool:
        path_text = self.uam_ulog_path_edit.text().strip()
        if not path_text:
            self.uam_analysis_status_label.setText("UAM 비교: ULog 경로가 비어 있습니다.")
            return False
        try:
            truth_source_type = self._selected_uam_truth_source_type()
            samples, topic_name = load_ulog_local_position_samples(
                path_text,
                truth_source_type=truth_source_type,
            )
        except ULogTelemetryError as exc:
            self.uam_ulog_samples = []
            self.uam_ulog_path = None
            self.uam_ulog_truth_source_type = TRUTH_SOURCE_AUTO
            self.uam_analysis_status_label.setText(f"UAM 비교: ULog 로드 실패 - {exc}")
            self._log(f"[UAM] ULog ground truth 로드 실패: {exc}")
            return False
        self.uam_ulog_samples = samples
        self.uam_ulog_path = Path(path_text).expanduser()
        self.uam_ulog_truth_source_type = truth_source_type
        self._refresh_uam_truth_plot(samples)
        first_s = samples[0].timestamp_s
        last_s = samples[-1].timestamp_s
        utc_count = sum(1 for sample in samples if sample.utc_unix_s is not None)
        utc_text = f", utc={utc_count}" if utc_count else ", utc=none"
        self.uam_analysis_status_label.setText(
            f"UAM 비교: ULog 로드 완료 topic={topic_name}, samples={len(samples)}, "
            f"truth_source={truth_source_type}, t={first_s:.3f}..{last_s:.3f}s{utc_text}"
        )
        self._log(
            f"[UAM] ULog ground truth 로드 완료: {self.uam_ulog_path} "
            f"topic={topic_name} truth_source={truth_source_type} samples={len(samples)} utc={utc_count}"
        )
        self.show_uam_raw_plot()
        self._refresh_camera_parameters_tab()
        return True

    def _uam_analysis_truth_samples(self) -> tuple[list[UamTelemetrySample], str]:
        if self.uam_ulog_samples:
            return list(self.uam_ulog_samples), "ulog"
        if self.uam_live_telemetry_samples:
            return list(self.uam_live_telemetry_samples), "live-buffer"
        return [], "embedded-recorded-row"

    def _ensure_uam_ulog_loaded_from_path(self) -> None:
        path_text = self.uam_ulog_path_edit.text().strip()
        if not path_text:
            return
        path = Path(path_text).expanduser()
        truth_source_type = self._selected_uam_truth_source_type()
        if (
            self.uam_ulog_path != path
            or self.uam_ulog_truth_source_type != truth_source_type
            or not self.uam_ulog_samples
        ):
            self.load_uam_ulog_file()

    def _compare_uam_rows(self, *, alignment_profile: UamAlignmentProfile | None) -> list[dict[str, object]]:
        truth_samples, truth_source = self._uam_analysis_truth_samples()
        return compare_measurement_rows(
            self.distance_recording_rows,
            truth_samples,
            truth_source=truth_source,
            max_dt_s=self.uam_compare_max_dt_spin.value(),
            time_offset_s=self.uam_time_offset_spin.value(),
            allow_embedded_truth=truth_source == "embedded-recorded-row",
            alignment_profile=alignment_profile,
        )

    def run_uam_analysis(self) -> None:
        if not self.distance_recording_rows:
            self.uam_analysis_rows = []
            self._refresh_uam_analysis_table()
            self._refresh_uam_error_plot()
            self.uam_analysis_status_label.setText("UAM 비교: 거리 기록 샘플이 없습니다.")
            self._refresh_operation_header()
            return
        self._ensure_uam_ulog_loaded_from_path()
        self.uam_analysis_rows = self._compare_uam_rows(alignment_profile=self.uam_alignment_profile)
        self._refresh_uam_analysis_table()
        self._refresh_uam_error_plot()
        self.show_uam_error_plot()
        summary = self._format_uam_analysis_summary()
        if self.uam_analysis_rows:
            self.uam_analysis_status_label.setText(summary)
            self._log(f"[UAM] YOLO 측정값과 ground truth 비교 완료: {summary}")
        else:
            total_rows = len(self.distance_recording_rows)
            self.uam_analysis_status_label.setText(
                f"UAM 비교: matched=0/{total_rows}. ULog/실시간 버퍼 시간축, 최대 Δt, "
                "Pixhawk 부팅 전 기록 구간을 확인하세요."
            )
            self._log("[UAM] YOLO 측정값과 ground truth 비교 결과가 비어 있습니다.")
        self._refresh_operation_header()
        self._refresh_camera_parameters_tab()

    def estimate_uam_alignment_from_ui(self) -> None:
        if not self.distance_recording_rows:
            self.uam_alignment_status_label.setText("좌표 보정: 거리 기록 샘플이 없습니다.")
            return
        self._ensure_uam_ulog_loaded_from_path()
        raw_rows = self._compare_uam_rows(alignment_profile=None)
        if not raw_rows:
            self.uam_alignment_status_label.setText(
                "좌표 보정: 매칭된 샘플이 없습니다. ULog/실시간 버퍼 시간축과 최대 Δt를 확인하세요."
            )
            return
        try:
            profile = estimate_alignment_profile_from_rows(raw_rows)
        except UamAlignmentError as exc:
            self.uam_alignment_status_label.setText(f"좌표 보정 추정 실패: {exc}")
            self._log(f"[UAM] 좌표 보정 추정 실패: {exc}")
            return
        self.uam_alignment_profile = profile
        self.uam_alignment_profile_path = None
        self._refresh_uam_alignment_status()
        self._log(
            "[UAM] 좌표 보정 profile 자동 추정: "
            f"{profile.profile_id}, samples={profile.sample_count}, rmse={profile.rmse_m:.4f}m"
        )
        self.run_uam_analysis()

    def save_uam_alignment_profile(self) -> None:
        if self.uam_alignment_profile is None:
            self.uam_alignment_status_label.setText("좌표 보정: 저장할 profile이 없습니다.")
            return
        default_path = self._calibration_dirs()[0] / f"{self.uam_alignment_profile.profile_id}.json"
        selected, _filter = QtWidgets.QFileDialog.getSaveFileName(
            self,
            "Save UAM alignment profile",
            str(default_path),
            "UAM alignment JSON (*.json);;All files (*)",
        )
        if not selected:
            return
        path = Path(selected)
        try:
            path.write_text(
                json.dumps(self.uam_alignment_profile.to_dict(), indent=2, ensure_ascii=False),
                encoding="utf-8",
            )
        except OSError as exc:
            self.uam_alignment_status_label.setText(f"좌표 보정 저장 실패: {exc}")
            return
        self.uam_alignment_profile_path = path
        self._refresh_uam_alignment_status()
        self._log(f"[UAM] 좌표 보정 profile 저장: {path}")

    def load_uam_alignment_profile(self) -> None:
        selected, _filter = QtWidgets.QFileDialog.getOpenFileName(
            self,
            "Load UAM alignment profile",
            str(self._calibration_dirs()[0]),
            "UAM alignment JSON (*.json);;All files (*)",
        )
        if not selected:
            return
        path = Path(selected)
        try:
            payload = json.loads(path.read_text(encoding="utf-8"))
            if not isinstance(payload, dict):
                raise ValueError("profile JSON root must be an object")
            profile = UamAlignmentProfile.from_dict(payload)
        except (OSError, ValueError, KeyError, TypeError, json.JSONDecodeError) as exc:
            self.uam_alignment_status_label.setText(f"좌표 보정 로드 실패: {exc}")
            return
        self.uam_alignment_profile = profile
        self.uam_alignment_profile_path = path
        self._refresh_uam_alignment_status()
        self._log(f"[UAM] 좌표 보정 profile 로드: {path}")
        if self.distance_recording_rows:
            self.run_uam_analysis()

    def clear_uam_alignment_profile(self) -> None:
        self.uam_alignment_profile = None
        self.uam_alignment_profile_path = None
        self._refresh_uam_alignment_status()
        self._log("[UAM] 좌표 보정 profile을 해제했습니다.")
        if self.distance_recording_rows:
            self.run_uam_analysis()

    def _refresh_uam_alignment_status(self) -> None:
        if not hasattr(self, "uam_alignment_status_label"):
            return
        profile = self.uam_alignment_profile
        if profile is None:
            self.uam_alignment_status_label.setText("좌표 보정: 없음")
            return
        path_text = f" | file={self.uam_alignment_profile_path.name}" if self.uam_alignment_profile_path else ""
        offset = profile.origin_offset_m
        source_text = f" | truth={profile.truth_source_type}" if profile.truth_source_type else ""
        self.uam_alignment_status_label.setText(
            f"좌표 보정: {profile.profile_id} | samples={profile.sample_count} | "
            f"rmse={profile.rmse_m:.3f}m | max={profile.max_error_m:.3f}m | "
            f"origin N/E/D={offset[0]:+.3f}/{offset[1]:+.3f}/{offset[2]:+.3f}m{source_text}{path_text}"
        )

    def estimate_uam_time_offset_from_ui(self) -> None:
        if not self.distance_recording_rows:
            self.uam_analysis_status_label.setText("UAM offset 추정: 거리 기록 샘플이 없습니다.")
            return
        path_text = self.uam_ulog_path_edit.text().strip()
        if path_text:
            path = Path(path_text).expanduser()
            truth_source_type = self._selected_uam_truth_source_type()
            if (
                self.uam_ulog_path != path
                or self.uam_ulog_truth_source_type != truth_source_type
                or not self.uam_ulog_samples
            ):
                self.load_uam_ulog_file()
        truth_samples, truth_source = self._uam_analysis_truth_samples()
        if not truth_samples:
            self.uam_analysis_status_label.setText("UAM offset 추정: ULog 또는 실시간 ground truth 샘플이 없습니다.")
            return
        estimate = estimate_time_offset_s(
            self.distance_recording_rows,
            truth_samples,
            max_dt_s=self.uam_compare_max_dt_spin.value(),
            initial_offset_s=self.uam_time_offset_spin.value(),
        )
        if estimate is None:
            self.uam_analysis_status_label.setText("UAM offset 추정: 매칭 가능한 후보를 찾지 못했습니다.")
            return
        offset_s = float(estimate["time_offset_s"])
        self.uam_time_offset_spin.setValue(offset_s)
        self.uam_analysis_status_label.setText(
            "UAM offset 추정: "
            f"offset={offset_s:+.6f}s, source={truth_source}, "
            f"matches={int(estimate['matched_count'])}, "
            f"rmse={estimate['rmse_m']:.3f}m, mean|dt|={estimate['mean_abs_dt_s']:.6f}s"
        )
        self._log(f"[UAM] 시간 offset 추정 완료: {self.uam_analysis_status_label.text()}")
        self.run_uam_analysis()

    def show_uam_error_plot(self) -> None:
        if hasattr(self, "uam_plot_stack"):
            self.uam_plot_stack.setCurrentWidget(self.uam_error_plot)

    def show_uam_raw_plot(self) -> None:
        if hasattr(self, "uam_plot_stack"):
            self.uam_plot_stack.setCurrentWidget(self.uam_raw_plot)

    def _uam_analysis_table_value(self, row: dict[str, object], field: str) -> str:
        fallback_fields = {
            "estimated_range_m": "estimated_baseline_center_distance_m",
            "truth_range_m": "truth_baseline_center_distance_m",
        }
        if field not in row and field in fallback_fields:
            field = fallback_fields[field]
        value = row.get(field, "")
        if value is None:
            return ""
        return str(value)

    def _refresh_uam_analysis_table(self) -> None:
        if not hasattr(self, "uam_analysis_table"):
            return
        table = self.uam_analysis_table
        table.setRowCount(0)
        for row in self.uam_analysis_rows:
            row_index = table.rowCount()
            table.insertRow(row_index)
            for column_index, (field, _label) in enumerate(UAM_ANALYSIS_TABLE_COLUMNS):
                item = QtWidgets.QTableWidgetItem(self._uam_analysis_table_value(row, field))
                item.setFlags(QtCore.Qt.ItemIsEnabled | QtCore.Qt.ItemIsSelectable)
                table.setItem(row_index, column_index, item)
        if self.uam_analysis_rows:
            table.selectRow(table.rowCount() - 1)
            table.scrollToBottom()
        elif hasattr(self, "uam_analysis_detail"):
            self.uam_analysis_detail.clear()

    def _refresh_uam_analysis_detail(self) -> None:
        if not hasattr(self, "uam_analysis_table"):
            return
        selected_rows = self.uam_analysis_table.selectionModel().selectedRows()
        if not selected_rows:
            self.uam_analysis_detail.clear()
            return
        row_index = selected_rows[0].row()
        if row_index < 0 or row_index >= len(self.uam_analysis_rows):
            self.uam_analysis_detail.clear()
            return
        row = self.uam_analysis_rows[row_index]
        self.uam_analysis_detail.setPlainText("\n".join(f"{key}: {value}" for key, value in row.items()))

    def _analysis_float(self, row: dict[str, object], field: str) -> float | None:
        try:
            value = float(row.get(field, ""))
        except (TypeError, ValueError):
            return None
        return value if math.isfinite(value) else None

    def _refresh_uam_error_plot(self) -> None:
        if not hasattr(self, "uam_error_plot"):
            return
        self.uam_error_plot.points.clear()
        has_frd_errors = any(
            self._analysis_float(row, "error_frd_forward_m") is not None for row in self.uam_analysis_rows
        )
        has_rdf_errors = any(
            self._analysis_float(row, "error_rdf_right_m") is not None for row in self.uam_analysis_rows
        )
        has_aligned_errors = any(
            self._analysis_float(row, "error_north_m") is not None for row in self.uam_analysis_rows
        )
        axis_fields = (
            (
                ("error_frd_forward_m", "error_frd_forward_m"),
                ("error_frd_right_m", "error_frd_right_m"),
                ("error_frd_down_m", "error_frd_down_m"),
            )
            if has_frd_errors
            else
            (
                ("error_rdf_right_m", "error_rdf_right_m"),
                ("error_rdf_down_m", "error_rdf_down_m"),
                ("error_rdf_forward_m", "error_rdf_forward_m"),
            )
            if has_rdf_errors
            else
            (
                ("error_north_m", "error_north_m"),
                ("error_east_m", "error_east_m"),
                ("error_down_m", "error_down_m"),
            )
            if has_aligned_errors
            else (
                ("error_x_m", "error_x_m"),
                ("error_y_m", "error_y_m"),
                ("error_z_m", "error_z_m"),
            )
        )
        for row in self.uam_analysis_rows:
            timestamp_s = self._analysis_float(row, "measurement_timestamp_s")
            if timestamp_s is None:
                continue
            for series, field in (*axis_fields, ("error_distance_m", "error_distance_m")):
                value = self._analysis_float(row, field)
                if value is not None:
                    self.uam_error_plot.add_point(TelemetryHistoryPoint(timestamp_s, series, value))
        self.uam_error_plot.update()

    def _format_uam_analysis_summary(self) -> str:
        summary = comparison_summary(self.uam_analysis_rows)
        total_rows = len(self.distance_recording_rows)
        matched_rows = len(self.uam_analysis_rows)
        unmatched_rows = max(0, total_rows - matched_rows)
        source_samples, truth_source = self._uam_analysis_truth_samples()
        source_text = truth_source
        if self.uam_ulog_path is not None and self.uam_ulog_samples:
            source_text = f"ulog:{self.uam_ulog_path.name}"
        elif source_samples:
            source_text = f"{truth_source}:{len(source_samples)} samples"
        else:
            source_text = "recorded-row fallback"
        coverage_text = f"matched={matched_rows}/{total_rows}"
        if unmatched_rows:
            coverage_text += f", unmatched={unmatched_rows}"
        if not summary:
            return f"UAM 비교: rows=0, {coverage_text}, source={source_text}"
        alignment_text = (
            f", alignment={self.uam_alignment_profile.profile_id}"
            if self.uam_alignment_profile is not None
            else ", alignment=none"
        )
        return (
            f"UAM 비교: rows={matched_rows}, {coverage_text}, source={source_text}, "
            f"median|err_dist|={summary['median_abs_m']:.3f} m, "
            f"p95|err_dist|={summary['p95_abs_m']:.3f} m, "
            f"rmse={summary['rmse_m']:.3f} m{alignment_text}"
        )

    def _format_uam_telemetry_summary(self) -> str:
        source = str(self.uam_source_combo.currentData() or "ros2") if hasattr(self, "uam_source_combo") else "ros2"
        ulog_text = str(self.uam_ulog_path) if self.uam_ulog_path is not None else "none"
        latest = "none"
        if self.latest_drone_position_m is not None:
            latest = (
                f"N={self.latest_drone_position_m[0]:+.3f}, "
                f"E={self.latest_drone_position_m[1]:+.3f}, "
                f"D={self.latest_drone_position_m[2]:+.3f}, "
                f"t={self.latest_drone_position_timestamp_s}"
            )
        alignment = "none"
        if self.uam_alignment_profile is not None:
            alignment = (
                f"{self.uam_alignment_profile.profile_id}, "
                f"rmse={self.uam_alignment_profile.rmse_m:.3f}m, "
                f"samples={self.uam_alignment_profile.sample_count}, "
                f"truth_source={self.uam_alignment_profile.truth_source_type or 'unknown'}"
            )
        return (
            "uam_ground_truth:\n"
            f"  preferred_analysis_source: ULog if loaded, live telemetry buffer otherwise, recorded row fallback last\n"
            f"  selected_truth_source: {self._selected_uam_truth_source_type()}\n"
            f"  source_mixing_policy: do not mix ULog/live with recorded-row fallback inside one comparison run\n"
            f"  metric_policy: summarize every matched row; separate ULog/live/recorded-row results by truth_source in reports\n"
            f"  time_match_order: bbox UTC -> timestamp + offset -> recorded row fallback\n"
            f"  live_source: {source}\n"
            f"  live_samples: {len(self.uam_live_telemetry_samples)}\n"
            f"  ulog: {ulog_text}\n"
            f"  ulog_samples: {len(self.uam_ulog_samples)}\n"
            f"  alignment_profile: {alignment}\n"
            f"  analysis_rows: {len(self.uam_analysis_rows)}\n"
            f"  latest_live: {latest}"
        )

    def toggle_session_recording(self) -> None:
        if self._session_recording_active():
            self._stop_session_recording()
            return
        self._start_session_recording()

    def _start_session_recording(self) -> None:
        self.recording_session_id = session_id_now()
        self.recording_session_started_wall_s = time.time()
        self.recording_session_started_monotonic_s = time.monotonic()
        self.distance_recording_started_monotonic_s = self.recording_session_started_monotonic_s
        self.distance_recording_enabled = True
        self.video_recording_enabled = True
        self.tracking_filter.reset()
        self._last_distance_history_key = None
        self._prepare_video_recording_paths()
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
        self._refresh_recording_status_label()
        self._refresh_operation_header()

    def toggle_distance_recording(self) -> None:
        self.toggle_session_recording()

    def toggle_video_recording(self) -> None:
        self.toggle_session_recording()

    def clear_distance_history(self) -> None:
        self.distance_history.clear()
        self.distance_recording_rows.clear()
        self.distance_recording_keys.clear()
        self.external_parameter_recording_rows.clear()
        self.uam_analysis_rows.clear()
        if not self.distance_recording_enabled:
            self.distance_recording_started_monotonic_s = None
        self._last_distance_history_key = None
        self.tracking_filter.reset()
        self.distance_plot.clear()
        self._refresh_recording_table()
        self._refresh_uam_analysis_table()
        self._refresh_uam_error_plot()
        if hasattr(self, "uam_analysis_status_label"):
            self.uam_analysis_status_label.setText("UAM 비교: 거리 기록 초기화로 분석 결과를 비웠습니다.")
        self._refresh_recording_status_label()
        self._log("[Depth] 거리 그래프와 미저장 기록을 초기화했습니다.")
        self._refresh_operation_header()

    def save_current_settings_from_ui(self) -> None:
        self._save_current_settings()
        self._log("[저장] 현재 StereoDistance 설정을 저장했습니다.")

    def discover_pc_camera_ip(self) -> None:
        interface_name = self.interface_edit.text().strip()
        if not interface_name or not interface_link_exists(interface_name):
            reason = "비어 있습니다" if not interface_name else f"현재 시스템에 없습니다: {interface_name}"
            self._log(f"[연결] PC IP 탐색: 카메라망 인터페이스가 {reason}. 유선 어댑터 자동 선택을 시도합니다.")
            candidates = discover_host_interface_candidates(
                host_address=self.host_address_edit.text().strip() or DEFAULT_HOST_ADDRESS,
                include_unconfigured=True,
            )
            if not candidates:
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
            self._append_pixhawk_candidate_rows(rows)
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
        if not self.recording_session_id and not self.distance_recording_rows and not self.external_parameter_recording_rows:
            self.recording_status_label.setText("세션 기록: 저장할 샘플 없음")
            self._refresh_operation_header()
            return
        stamp = self.recording_session_id or session_id_now()
        output_dir = self._recording_session_dir(stamp)
        mag_rows = self._record_px4_mag_status_rows()
        saved_files = save_session_files(
            output_dir,
            metadata=self._recording_session_metadata(mag_row_count=len(mag_rows)),
            distance_rows=self.distance_recording_rows,
            external_parameter_rows=self.external_parameter_recording_rows,
            pixhawk_mag_rows=mag_rows,
            uam_analysis_rows=self.uam_analysis_rows,
        )
        self._log(f"[저장] 세션 메타데이터를 저장했습니다: {saved_files.metadata}")
        if saved_files.distance_history is not None:
            self._log(f"[저장] 거리 기록 CSV를 저장했습니다: {saved_files.distance_history}")
        if saved_files.external_parameters is not None:
            self._log(f"[저장] 시간축 외부 파라미터 CSV를 저장했습니다: {saved_files.external_parameters}")
        if saved_files.pixhawk_mag_status is not None:
            self._log(f"[저장] Pixhawk MAG 상태 CSV를 저장했습니다: {saved_files.pixhawk_mag_status}")
        if saved_files.uam_analysis is not None:
            self._log(f"[저장] UAM 비교 분석 CSV를 저장했습니다: {saved_files.uam_analysis}")
        output_text = ", ".join(str(output) for output in saved_files.outputs)
        self.recording_status_label.setText(
            "세션 기록 저장 완료: "
            f"거리={len(self.distance_recording_rows)}개, "
            f"외부={len(self.external_parameter_recording_rows)}개, "
            f"MAG={len(mag_rows)}개, "
            f"UAM분석={len(self.uam_analysis_rows)}개 - {output_text}"
        )
        self._refresh_operation_header()

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

    def _recording_session_metadata(self, *, mag_row_count: int | None = None) -> dict[str, object]:
        stamp = self.recording_session_id or time.strftime("%Y%m%d_%H%M%S")
        if mag_row_count is None:
            mag_row_count = len(self._record_px4_mag_status_rows())
        return {
            "session_id": self.recording_session_id,
            "session_dir": str(self._recording_session_dir(stamp)),
            "session_started_unix_s": self.recording_session_started_wall_s,
            "left_video_path": str(self.video_recording_paths.get("left", "")),
            "right_video_path": str(self.video_recording_paths.get("right", "")),
            "distance_sample_count": len(self.distance_recording_rows),
            "external_parameter_sample_count": len(self.external_parameter_recording_rows),
            "uam_analysis_sample_count": len(self.uam_analysis_rows),
            "uam_analysis_matched_distance_sample_count": len(self.uam_analysis_rows),
            "uam_analysis_unmatched_distance_sample_count": max(
                0, len(self.distance_recording_rows) - len(self.uam_analysis_rows)
            ),
            "uam_live_telemetry_sample_count": len(self.uam_live_telemetry_samples),
            "uam_ulog_sample_count": len(self.uam_ulog_samples),
            "uam_ulog_path": str(self.uam_ulog_path or ""),
            "uam_selected_truth_source_type": self._selected_uam_truth_source_type(),
            "uam_loaded_ulog_truth_source_type": self.uam_ulog_truth_source_type,
            "uam_alignment_profile": (
                None if self.uam_alignment_profile is None else self.uam_alignment_profile.to_dict()
            ),
            "uam_alignment_profile_path": str(self.uam_alignment_profile_path or ""),
            "bbox_reprojection_profile": (
                None if self.bbox_reprojection_profile is None else self.bbox_reprojection_profile.to_dict()
            ),
            "bbox_reprojection_profile_path": str(self.bbox_reprojection_profile_path or ""),
            "bbox_center_adjustment_path": str(self.bbox_center_adjustment_path or ""),
            "uam_compare_max_dt_s": (
                self.uam_compare_max_dt_spin.value() if hasattr(self, "uam_compare_max_dt_spin") else None
            ),
            "uam_compare_time_offset_s": (
                self.uam_time_offset_spin.value() if hasattr(self, "uam_time_offset_spin") else None
            ),
            "pixhawk_mag_status_sample_count": mag_row_count,
            "pixhawk_mag_actual_selected_device_ids": {
                "left": self.latest_vehicle_mag_device_ids.get("left"),
                "right": self.latest_vehicle_mag_device_ids.get("right"),
            },
            "timestamp_contract": {
                "manufacturer_confirmed": True,
                "camera_captured_timestamp_clock": "WIRIS camera RTC",
                "rtp_rtcp_timestamp_clock": "same WIRIS camera RTC as camera_captured timestamp",
                "rtp_timestamp_note": "RTP timestamp is a 90 kHz media clock counter; absolute camera RTC time is derived through RTCP Sender Report RTP-to-NTP mapping.",
                "preferred_frame_time_field": "camera_rtc_frame_unix_s",
                "legacy_equivalent_field": "rtcp_frame_ntp_unix_s",
                "clock_offset_check": "Use the 시간 tab EXIF comparison to estimate left/right WIRIS camera RTC offset.",
            },
            "intrinsics": self._record_intrinsic_parameters(),
            "stream": {
                "left_rtsp_url": self.left_url_edit.text().strip(),
                "right_rtsp_url": self.right_url_edit.text().strip(),
                "backend": self.stream_backend_combo.currentData(),
                "transport": self.rtsp_transport_combo.currentData(),
                "decoder": self.decoder_combo.currentData(),
                "rtsp_latency_ms": self.rtsp_latency_spin.value(),
                "preview_rate_hz": self.preview_rate_spin.value(),
            },
        }

    def set_bias_profile_applied(self, checked: bool) -> None:
        self.bias_profile_applied = bool(checked)
        self._refresh_bias_profile_view()
        self._log(f"[Bias] 실시간 보정 적용: {'ON' if self.bias_profile_applied else 'OFF'}")

    def load_bbox_center_adjustment_csv(self) -> None:
        start_path = self._calibration_dirs()[0]
        selected, _filter = QtWidgets.QFileDialog.getOpenFileName(
            self,
            "Load bbox center adjustment matrix",
            str(start_path),
            "Profile (*.json *.csv);;JSON (*.json);;CSV (*.csv);;All files (*)",
        )
        if not selected:
            return
        try:
            self._load_bbox_center_adjustment_path(Path(selected))
        except Exception as exc:
            self.bbox_adjust_status_label.setText(f"2D bbox 보정 로드 실패: {exc}")
            self._log(f"[Bias] 2D bbox 보정 행렬 로드 실패: {exc}")
            return
        self._log(f"[Bias] 2D bbox 보정 행렬을 로드했습니다: {selected}")

    def _load_bbox_center_adjustment_path(self, path: Path) -> None:
        if path.suffix.lower() == ".json":
            profile = load_reprojection_profile(path)
            self.bbox_reprojection_profile = profile
            self.bbox_reprojection_profile_path = path
            self.bbox_center_adjustment_path = None
            self.bbox_center_adjustments.clear()
            self._apply_bbox_center_adjustments_to_panes()
            self._refresh_bbox_center_adjustment_view()
            self._refresh_camera_parameters_tab()
            return
        rows: list[list[float]] = []
        with path.open(newline="", encoding="utf-8") as handle:
            reader = csv.reader(handle)
            for raw_row in reader:
                if not raw_row:
                    continue
                rows.append([float(value) for value in raw_row[:4]])
        if len(rows) < 6 or any(len(row) < 4 for row in rows[:6]):
            raise ValueError("6x4 CSV 행렬이 필요합니다.")
        self.bbox_center_adjustment_path = path
        self.bbox_reprojection_profile = None
        self.bbox_reprojection_profile_path = None
        self.bbox_center_adjustments = {
            "left": (tuple(rows[0][:4]), tuple(rows[1][:4])),
            "right": (tuple(rows[3][:4]), tuple(rows[4][:4])),
        }
        self._apply_bbox_center_adjustments_to_panes()
        self._refresh_bbox_center_adjustment_view()
        self._refresh_camera_parameters_tab()

    def clear_bbox_center_adjustment(self) -> None:
        self.bbox_center_adjustment_path = None
        self.bbox_reprojection_profile_path = None
        self.bbox_reprojection_profile = None
        self.bbox_center_adjustments.clear()
        self._apply_bbox_center_adjustments_to_panes()
        self._refresh_bbox_center_adjustment_view()
        self._refresh_camera_parameters_tab()
        self._log("[Bias] 2D bbox 보정/profile을 해제했습니다.")

    def _apply_bbox_center_adjustments_to_panes(self) -> None:
        if hasattr(self, "left_pane"):
            self.left_pane.set_bbox_center_adjustment(self.bbox_center_adjustments.get("left"))
        if hasattr(self, "right_pane"):
            self.right_pane.set_bbox_center_adjustment(self.bbox_center_adjustments.get("right"))

    def _bbox_center_adjustment_equation(self, camera: str) -> str:
        adjustment = self.bbox_center_adjustments.get(camera)
        if adjustment is None:
            return f"{camera}: inactive"
        row_u, row_v = adjustment
        return (
            f"{camera}: u'={row_u[0]:.6f}u + {row_u[1]:.6f}v + {row_u[2] + row_u[3]:.6f}, "
            f"v'={row_v[0]:.6f}u + {row_v[1]:.6f}v + {row_v[2] + row_v[3]:.6f}"
        )

    def _bbox_center_adjustment_summary(self) -> str:
        if self.bbox_reprojection_profile is not None:
            profile = self.bbox_reprojection_profile
            source = str(self.bbox_reprojection_profile_path or "")
            return (
                "bbox_reprojection_profile: active\n"
                f"profile_id: {profile.profile_id}\n"
                f"profile_method: {profile.method}\n"
                f"bin_axis: {profile.bin_axis}, lookup_policy: {profile.lookup_policy}, "
                f"output_policy: {profile.output_policy}\n"
                f"cells: {len(profile.cells)}, source: {source}"
            )
        if not self.bbox_center_adjustments:
            return "bbox_center_adjustment: inactive"
        source = str(self.bbox_center_adjustment_path or "")
        return (
            "bbox_center_adjustment: active\n"
            f"bbox_center_adjustment_source: {source}\n"
            f"{self._bbox_center_adjustment_equation('left')}\n"
            f"{self._bbox_center_adjustment_equation('right')}"
        )

    def _refresh_bbox_center_adjustment_view(self) -> None:
        if not hasattr(self, "bbox_adjust_status_label"):
            return
        if not self.bbox_center_adjustments:
            self.bbox_adjust_status_label.setText("2D bbox 보정: 없음")
            return
        self.bbox_adjust_status_label.setText(self._bbox_center_adjustment_summary())

    def _parse_float_field(self, row: dict[str, object], name: str) -> float | None:
        value = row.get(name)
        if value is None or value == "":
            return None
        try:
            parsed = float(value)
        except (TypeError, ValueError):
            return None
        return parsed if isfinite(parsed) else None

    def _first_float_field(self, row: dict[str, object], names: tuple[str, ...]) -> float | None:
        for name in names:
            value = self._parse_float_field(row, name)
            if value is not None:
                return value
        return None

    def _bias_profile_from_rows(self, rows: list[dict[str, object]], *, bin_size_m: float) -> BBoxBiasProfile | None:
        if bin_size_m <= 0.0:
            return None
        buckets: dict[int, list[tuple[float, float, float]]] = {}
        for row in rows:
            distance = self._first_float_field(row, ("raw_distance_m", "distance_m"))
            raw_x = self._first_float_field(row, ("raw_x_m", "x_m"))
            raw_y = self._first_float_field(row, ("raw_y_m", "y_m"))
            raw_z = self._first_float_field(row, ("raw_z_m", "z_m"))
            drone_x = self._first_float_field(row, ("drone_north_m", "drone_x_m"))
            drone_y = self._first_float_field(row, ("drone_east_m", "drone_y_m"))
            drone_z = self._first_float_field(row, ("drone_down_m", "drone_z_m"))
            if None in (distance, raw_x, raw_y, raw_z, drone_x, drone_y, drone_z):
                continue
            bucket = int(max(0.0, distance) // bin_size_m)
            buckets.setdefault(bucket, []).append((raw_x - drone_x, raw_y - drone_y, raw_z - drone_z))
        bins = []
        for bucket, errors in sorted(buckets.items()):
            if not errors:
                continue
            count = len(errors)
            bins.append(
                BBoxBiasBin(
                    min_m=bucket * bin_size_m,
                    max_m=(bucket + 1) * bin_size_m,
                    bias_x_m=sum(item[0] for item in errors) / count,
                    bias_y_m=sum(item[1] for item in errors) / count,
                    bias_z_m=sum(item[2] for item in errors) / count,
                    sample_count=count,
                )
            )
        if not bins:
            return None
        return BBoxBiasProfile(
            profile_id=f"bbox_bias_{time.strftime('%Y%m%d_%H%M%S')}",
            bins=tuple(bins),
            created_unix_s=time.time(),
        )

    def build_bias_profile_from_current_session(self) -> None:
        profile = self._bias_profile_from_rows(
            [dict(row) for row in self.distance_recording_rows],
            bin_size_m=self.bias_bin_size_spin.value(),
        )
        if profile is None:
            self.bias_profile_detail.setPlainText(
                "현재 세션 기록에서 profile을 만들 수 없습니다.\n"
                "drone_north/east/down 또는 drone_x/y/z와 raw_x/y/z가 있는 샘플이 필요합니다."
            )
            return
        self.active_bias_profile = profile
        self.bias_candidate_rows = [dict(row) for row in self.distance_recording_rows]
        self._refresh_bias_profile_view()
        self._log(f"[Bias] 현재 세션에서 profile 생성: {profile.profile_id}")

    def load_bias_candidates_csv(self) -> None:
        selected, _filter = QtWidgets.QFileDialog.getOpenFileName(
            self,
            "Load bias candidate CSV",
            str(self._calibration_dirs()[0]),
            "CSV files (*.csv);;All files (*)",
        )
        if not selected:
            return
        path = Path(selected)
        try:
            with path.open("r", newline="", encoding="utf-8") as handle:
                rows = list(csv.DictReader(handle))
        except OSError as exc:
            self.bias_profile_detail.setPlainText(f"CSV 로드 실패: {exc}")
            return
        profile = self._bias_profile_from_rows(rows, bin_size_m=self.bias_bin_size_spin.value())
        self.bias_candidate_rows = rows
        if profile is None:
            self.bias_profile_detail.setPlainText(
                f"CSV에서 profile을 만들 수 없습니다: {path}\n"
                "drone_x/y/z와 raw_x/y/z 필드를 확인하세요."
            )
            return
        self.active_bias_profile = profile
        self._refresh_bias_profile_view()
        self._log(f"[Bias] CSV 후보정 profile 생성: {path}")

    def save_bias_profile(self) -> None:
        if self.active_bias_profile is None:
            self.bias_profile_detail.setPlainText("저장할 bias profile이 없습니다.")
            return
        default_path = self._calibration_dirs()[0] / f"{self.active_bias_profile.profile_id}.json"
        selected, _filter = QtWidgets.QFileDialog.getSaveFileName(
            self,
            "Save bias profile",
            str(default_path),
            "Bias profile JSON (*.json);;All files (*)",
        )
        if not selected:
            return
        path = Path(selected)
        path.write_text(json.dumps(self.active_bias_profile.to_dict(), indent=2, ensure_ascii=False), encoding="utf-8")
        self._log(f"[Bias] profile 저장: {path}")

    def load_bias_profile(self) -> None:
        selected, _filter = QtWidgets.QFileDialog.getOpenFileName(
            self,
            "Load bias profile",
            str(self._calibration_dirs()[0]),
            "Bias profile JSON (*.json);;All files (*)",
        )
        if not selected:
            return
        path = Path(selected)
        try:
            payload = json.loads(path.read_text(encoding="utf-8"))
            if not isinstance(payload, dict):
                raise ValueError("profile JSON root must be an object")
            self.active_bias_profile = BBoxBiasProfile.from_dict(payload)
        except (OSError, ValueError, KeyError, TypeError, json.JSONDecodeError) as exc:
            self.bias_profile_detail.setPlainText(f"Profile 로드 실패: {exc}")
            return
        self._refresh_bias_profile_view()
        self._log(f"[Bias] profile 로드: {path}")

    def _refresh_bias_profile_view(self) -> None:
        if not hasattr(self, "bias_profile_status_label"):
            return
        profile = self.active_bias_profile
        applied = self.bias_profile_applied and profile is not None
        status = "적용 중" if applied else "대기"
        if profile is None:
            self.bias_profile_status_label.setText("Profile: 없음")
            self.bias_profile_table.setRowCount(0)
            return
        total_samples = sum(bias_bin.sample_count for bias_bin in profile.bins)
        self.bias_profile_status_label.setText(
            f"Profile: {profile.profile_id} | bins={len(profile.bins)} | samples={total_samples} | {status}"
        )
        self.bias_profile_table.setRowCount(0)
        for bias_bin in profile.bins:
            row_index = self.bias_profile_table.rowCount()
            self.bias_profile_table.insertRow(row_index)
            values = (
                f"{bias_bin.min_m:.1f}",
                f"{bias_bin.max_m:.1f}",
                f"{bias_bin.bias_x_m:+.4f}",
                f"{bias_bin.bias_y_m:+.4f}",
                f"{bias_bin.bias_z_m:+.4f}",
                str(bias_bin.sample_count),
            )
            for column, value in enumerate(values):
                item = QtWidgets.QTableWidgetItem(value)
                item.setFlags(QtCore.Qt.ItemIsEnabled | QtCore.Qt.ItemIsSelectable)
                self.bias_profile_table.setItem(row_index, column, item)
        detail = json.dumps(profile.to_dict(), indent=2, ensure_ascii=False)
        self.bias_profile_detail.setPlainText(detail)

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
            row_index = self.distance_recording_keys.index(point_key)
            self.distance_recording_keys.pop(row_index)
            if row_index < len(self.distance_recording_rows):
                self.distance_recording_rows.pop(row_index)
                removed_row = True
                self._refresh_recording_table()
        self._last_distance_history_key = self.distance_history[-1].sample_key if self.distance_history else None
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
        if source == "ros2" and self.pixhawk is not None:
            self.pixhawk.add_custom_watch(spec)
        if source == "mavlink" and self.mavlink_pixhawk is not None:
            self.mavlink_pixhawk.add_custom_watch(spec)
        if source == "uam_mavlink" and self.uam_mavlink_pixhawk is not None:
            self.uam_mavlink_pixhawk.add_custom_watch(spec)

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
        if self.pixhawk is not None:
            self.pixhawk.remove_custom_watch(key)
        if self.mavlink_pixhawk is not None:
            self.mavlink_pixhawk.remove_custom_watch(key)
        if self.uam_mavlink_pixhawk is not None:
            self.uam_mavlink_pixhawk.remove_custom_watch(key)
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
            self._update_uam_live_telemetry_from_custom_watch(sample)

    def _custom_watch_uam_axis(self, key: str) -> str | None:
        tail = key.rsplit(":", 1)[-1]
        if tail in {"drone_x", "drone_y", "drone_z", "uam_x", "uam_y", "uam_z"}:
            return tail[-1]
        return None

    def _custom_watch_uam_source(self, sample: CustomTopicSample) -> str:
        if sample.key.startswith(("left:", "right:")) or sample.topic.startswith("mavlink/"):
            side = sample.key.split(":", 1)[0] if ":" in sample.key else "mavlink"
            return f"live-mavlink:{side}"
        return "live-ros2"

    def _update_uam_live_telemetry_from_custom_watch(self, sample: CustomTopicSample) -> None:
        if sample.numeric_value is None:
            return
        axis = self._custom_watch_uam_axis(sample.key)
        if axis is None:
            return
        self.latest_drone_position_components_m[axis] = float(sample.numeric_value)
        self.latest_drone_position_component_timestamps_s[axis] = float(sample.timestamp_s)
        if not all(key in self.latest_drone_position_components_m for key in ("x", "y", "z")):
            return
        self.latest_drone_position_m = (
            self.latest_drone_position_components_m["x"],
            self.latest_drone_position_components_m["y"],
            self.latest_drone_position_components_m["z"],
        )
        self.latest_drone_position_timestamp_s = max(
            self.latest_drone_position_component_timestamps_s.get(key, sample.timestamp_s)
            for key in ("x", "y", "z")
        )
        self.latest_drone_position_timestamp_reference = self._custom_watch_uam_source(sample)
        self.latest_drone_position_utc_unix_s = None
        signature = (
            round(float(self.latest_drone_position_timestamp_s), 6),
            round(float(self.latest_drone_position_m[0]), 6),
            round(float(self.latest_drone_position_m[1]), 6),
            round(float(self.latest_drone_position_m[2]), 6),
        )
        if signature != self._last_uam_live_sample_signature:
            self._last_uam_live_sample_signature = signature
            uam_sample = UamTelemetrySample(
                timestamp_s=float(self.latest_drone_position_timestamp_s),
                x_m=float(self.latest_drone_position_m[0]),
                y_m=float(self.latest_drone_position_m[1]),
                z_m=float(self.latest_drone_position_m[2]),
                source=self.latest_drone_position_timestamp_reference,
                timestamp_reference="app-receive",
            )
            self.latest_drone_truth_sample = uam_sample
            self.uam_live_telemetry_samples.append(uam_sample)
            self._append_uam_truth_plot_sample(uam_sample)
        latest_text = (
            f"N={self.latest_drone_position_m[0]:+.3f} "
            f"E={self.latest_drone_position_m[1]:+.3f} "
            f"D={self.latest_drone_position_m[2]:+.3f} m "
            f"t={self.latest_drone_position_timestamp_s:.3f}s "
            f"source={self.latest_drone_position_timestamp_reference}"
        )
        if hasattr(self, "bias_drone_status_label"):
            self.bias_drone_status_label.setText(f"UAM 중심 위치: {latest_text}")
        if hasattr(self, "uam_latest_label"):
            self.uam_latest_label.setText(f"latest: {latest_text}")
        if hasattr(self, "uam_status_label"):
            self.uam_status_label.setText(f"UAM telemetry: 실시간 수신 중, buffer={len(self.uam_live_telemetry_samples)}")
        self._refresh_camera_parameters_tab()

    def _remove_uam_axis_watches(self) -> None:
        for key in ("uam_x", "uam_y", "uam_z"):
            if any(str(spec.get("key", "")) == key for spec in self.custom_watch_specs):
                self.remove_custom_topic_watch(key)

    def disconnect_uam_telemetry_from_ui(self) -> None:
        self._remove_uam_axis_watches()
        if self.uam_mavlink_pixhawk is not None:
            self.uam_mavlink_pixhawk.stop()
            self.uam_mavlink_pixhawk = None
        self.uam_status_label.setText(
            f"UAM telemetry: 해제됨, 실시간 buffer={len(self.uam_live_telemetry_samples)}개 유지"
        )
        self.uam_latest_label.setText("latest: --")
        self.latest_drone_position_m = None
        self.latest_drone_position_components_m.clear()
        self.latest_drone_position_component_timestamps_s.clear()
        self.latest_drone_position_timestamp_s = None
        self.latest_drone_position_timestamp_reference = "app-receive"
        self.latest_drone_position_utc_unix_s = None
        self.latest_drone_truth_sample = None
        self.uam_gps_reference_lla = None
        self.uam_gps_reference_kind = ""
        self._last_uam_live_sample_signature = None
        self._log("[UAM] ground truth 텔레메트리 watch를 해제했습니다. Pixhawk ROS2/MAVLink 링크는 유지합니다.")
        self._refresh_camera_parameters_tab()

    def _ensure_uam_mavlink_subscriber(self) -> DualPixhawkMavlinkSubscriber:
        url = self.uam_mavlink_url_edit.text().strip()
        baud = self.uam_mavlink_baud_spin.value()
        if not url:
            raise RuntimeError("UAM USB MAVLink 포트가 비어 있습니다.")
        if self.uam_mavlink_pixhawk is None:
            self.uam_mavlink_pixhawk = DualPixhawkMavlinkSubscriber(
                left_url=url,
                right_url="",
                left_baud=baud,
                right_baud=baud,
                custom_watches=tuple(self.custom_watch_specs),
            )
            self.uam_mavlink_pixhawk.gps_changed.connect(self._update_uam_live_telemetry_from_gps)
            self.uam_mavlink_pixhawk.custom_watch_changed.connect(self._update_custom_watch)
            self.uam_mavlink_pixhawk.status_changed.connect(
                lambda text: self._log(f"[UAM] USB MAVLink {text}")
            )
        else:
            if hasattr(self.uam_mavlink_pixhawk, "urls"):
                self.uam_mavlink_pixhawk.urls["left"] = url
                self.uam_mavlink_pixhawk.urls["right"] = ""
            if hasattr(self.uam_mavlink_pixhawk, "baudrates"):
                self.uam_mavlink_pixhawk.baudrates["left"] = baud
                self.uam_mavlink_pixhawk.baudrates["right"] = baud
        return self.uam_mavlink_pixhawk

    def connect_uam_mavlink_usb(self) -> bool:
        try:
            subscriber = self._ensure_uam_mavlink_subscriber()
            subscriber.start("left")
        except Exception as exc:
            if self.uam_mavlink_pixhawk is not None:
                try:
                    self.uam_mavlink_pixhawk.stop()
                except Exception:
                    pass
                self.uam_mavlink_pixhawk = None
            self._log(f"[UAM] USB MAVLink 연결 실패: {exc}")
            return False
        return bool(getattr(subscriber, "connections", {}).get("left"))

    def connect_uam_telemetry_from_ui(self) -> None:
        source = str(self.uam_source_combo.currentData() or "ros2")
        self._remove_uam_axis_watches()
        if source == "off":
            self.disconnect_uam_telemetry_from_ui()
            self._save_current_settings()
            return
        if source == "ros2":
            topic = self.uam_ros2_topic_edit.text().strip()
            fields = {
                "x": self.uam_ros2_x_field_edit.text().strip(),
                "y": self.uam_ros2_y_field_edit.text().strip(),
                "z": self.uam_ros2_z_field_edit.text().strip(),
            }
            if not topic or any(not field for field in fields.values()):
                self.uam_status_label.setText("UAM telemetry: ROS2 topic 또는 N/E/D field가 비어 있습니다.")
                return
            for axis, field in fields.items():
                self._add_custom_topic_watch(key=f"uam_{axis}", topic=topic, field=field, source="ros2")
            connected = self.connect_pixhawk()
            state = "ROS2 연결됨" if connected else "ROS2 연결 실패, MAVLink fallback 상태 확인 필요"
            self.uam_status_label.setText(
                f"UAM telemetry: ROS2 watch 등록 {topic} "
                f"({fields['x']}, {fields['y']}, {fields['z']}) - {state}"
            )
            self._log(f"[UAM] ROS2 ground truth 텔레메트리 watch 등록: {topic} {fields} ({state})")
            self._save_current_settings()
            return
        if source in {"mavlink_gps", "mavlink_global"}:
            connected = self.connect_uam_mavlink_usb()
            label = "GPS Position" if source == "mavlink_gps" else "Global Position"
            state = "MAVLink 연결됨" if connected else "MAVLink 연결 대기/실패"
            self.uam_status_label.setText(
                f"UAM telemetry: USB MAVLink {label} 수신 후 lat/lon/alt -> local NED 변환 - {state}"
            )
            self._log(f"[UAM] USB MAVLink {label} ground truth 변환 대기 ({state})")
            self._save_current_settings()
            return
        message = self.uam_mavlink_message_edit.text().strip().upper()
        fields = {
            "x": self.uam_mavlink_x_field_edit.text().strip(),
            "y": self.uam_mavlink_y_field_edit.text().strip(),
            "z": self.uam_mavlink_z_field_edit.text().strip(),
        }
        if not message or any(not field for field in fields.values()):
            self.uam_status_label.setText("UAM telemetry: MAVLink message 또는 N/E/D field가 비어 있습니다.")
            return
        for axis, field in fields.items():
            self._add_custom_topic_watch(
                key=f"uam_{axis}",
                topic=message,
                field=field,
                source="uam_mavlink",
                message=message,
            )
        connected = self.connect_uam_mavlink_usb()
        state = "MAVLink 연결됨" if connected else "MAVLink 연결 대기/실패"
        self.uam_status_label.setText(
            f"UAM telemetry: USB MAVLink watch 등록 {message} "
            f"({fields['x']}, {fields['y']}, {fields['z']}) - {state}"
        )
        self._log(f"[UAM] USB MAVLink ground truth 텔레메트리 watch 등록: {message} {fields} ({state})")
        self._save_current_settings()

    def clear_uam_live_telemetry(self) -> None:
        self.uam_live_telemetry_samples.clear()
        self._last_uam_live_sample_signature = None
        if hasattr(self, "uam_live_plot"):
            self.uam_live_plot.points.clear()
            self.uam_live_plot.update()
        if hasattr(self, "uam_raw_plot") and not self.uam_ulog_samples:
            self.uam_raw_plot.points.clear()
            self.uam_raw_plot.update()
        if hasattr(self, "uam_status_label"):
            self.uam_status_label.setText("UAM telemetry: 실시간 버퍼 초기화됨")
        if hasattr(self, "uam_latest_label"):
            self.uam_latest_label.setText("latest: --")
        self._log("[UAM] 실시간 ground truth 텔레메트리 버퍼를 초기화했습니다.")

    def _append_uam_truth_plot_sample(self, sample: UamTelemetrySample) -> None:
        plots = []
        if hasattr(self, "uam_live_plot"):
            plots.append(self.uam_live_plot)
        if hasattr(self, "uam_raw_plot"):
            plots.append(self.uam_raw_plot)
        for plot in plots:
            plot.add_point(TelemetryHistoryPoint(sample.timestamp_s, "truth_north_m", sample.x_m))
            plot.add_point(TelemetryHistoryPoint(sample.timestamp_s, "truth_east_m", sample.y_m))
            plot.add_point(TelemetryHistoryPoint(sample.timestamp_s, "truth_down_m", sample.z_m))

    def _refresh_uam_truth_plot(self, samples: Sequence[UamTelemetrySample]) -> None:
        plots = []
        if hasattr(self, "uam_live_plot"):
            plots.append(self.uam_live_plot)
        if hasattr(self, "uam_raw_plot"):
            plots.append(self.uam_raw_plot)
        if not plots:
            return
        for plot in plots:
            plot.points.clear()
        if samples:
            step = max(1, math.ceil(len(samples) / 200))
            for sample in samples[::step]:
                self._append_uam_truth_plot_sample(sample)
        for plot in plots:
            plot.update()

    def connect_pixhawk(self, *, allow_mavlink_recovery: bool = True) -> bool:
        if self.pixhawk is not None:
            return True
        try:
            self._ensure_micro_xrce_agent()
            self.pixhawk = DualPixhawkAttitudeSubscriber(
                domain_id=self.domain_spin.value(),
                left_namespace=self.left_namespace_edit.text().strip(),
                right_namespace=self.right_namespace_edit.text().strip(),
                custom_watches=tuple(self.custom_watch_specs),
            )
            self.pixhawk.extrinsic_changed.connect(self._update_extrinsic)
            self.pixhawk.gps_changed.connect(self._update_gps)
            self.pixhawk.mag_sensor_changed.connect(self._update_mag_sensor)
            self.pixhawk.gps_sensor_changed.connect(self._update_gps_sensor)
            self.pixhawk.gnss_relative_changed.connect(self._update_gnss_relative)
            self.pixhawk.custom_watch_changed.connect(self._update_custom_watch)
            self.pixhawk.status_changed.connect(lambda text: self._log(f"[픽스호크] {text}"))
            self.pixhawk.start()
            self.pixhawk_button.setEnabled(True)
            self._refresh_session_toggle_buttons()
            self.pixhawk_connection_label.setText("픽스호크 ROS2: 세션 시작, 데이터 대기 중")
            self._log("[픽스호크] ROS2 세션을 시작했습니다. 데이터 수신을 기다립니다.")
            QtCore.QTimer.singleShot(2000, self._verify_ros2_pixhawk_publishers)
            self.refresh_px4_mag_manager()
            self._refresh_operation_header()
            return True
        except Exception as exc:
            self.pixhawk = None
            self.pixhawk_button.setEnabled(True)
            self._refresh_session_toggle_buttons()
            self.pixhawk_connection_label.setText(f"pixhawk ROS2: failed ({exc})")
            self._log(f"[픽스호크] ROS2 연결 실패: {exc}")
            self._refresh_operation_header()
            if allow_mavlink_recovery:
                self._start_mavlink_recovery_after_ros2_failure(exc)
            return False

    def _ensure_micro_xrce_agent(self) -> None:
        started, message = ensure_micro_xrce_agent(self.uxrce_port_spin.value())
        if started:
            self._log(f"[픽스호크] MicroXRCEAgent를 시작했습니다: {message}")
        else:
            self._log(f"[픽스호크] MicroXRCEAgent가 이미 실행 중입니다: {message}")

    def _verify_ros2_pixhawk_publishers(self) -> None:
        if self.pixhawk is None:
            return
        domain_id = int(self.domain_spin.value())
        checks = []
        for side, namespace in (
            ("left", self.left_namespace_edit.text().strip().strip("/")),
            ("right", self.right_namespace_edit.text().strip().strip("/")),
        ):
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
        if self.pixhawk is None:
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

    def disconnect_pixhawk(self) -> None:
        if self.pixhawk is not None:
            self.pixhawk.stop()
            self.pixhawk = None
        self.pixhawk_attitude_times["left"].clear()
        self.pixhawk_attitude_times["right"].clear()
        self._clear_px4_mag_manager_connection_state()
        self.pixhawk_button.setEnabled(True)
        self._refresh_session_toggle_buttons()
        if self.mavlink_pixhawk is None:
            self.pixhawk_connection_label.setText("픽스호크: 연결 해제")
        self._log("[픽스호크] ROS2 연결을 해제했습니다.")
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

    def _retry_ros2_after_mavlink_recovery(self) -> None:
        if self.pixhawk is not None:
            return
        self._log("[픽스호크] MAVLink 복구 설정 이후 ROS2 연결을 다시 시도합니다.")
        if self.connect_pixhawk(allow_mavlink_recovery=False):
            self._log("[픽스호크] MAVLink 복구 설정 이후 ROS2 세션이 시작되었습니다.")
        else:
            self._log("[픽스호크] ROS2 재시도 실패. MAVLink telemetry fallback을 유지합니다.")

    def _start_mavlink_sensor_auxiliary_if_needed(self) -> None:
        if self.pixhawk is None:
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
        if self.mavlink_configure_streams_checkbox.isChecked() and not self.mavlink_configure_uxrce_agent_checkbox.isChecked():
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
        elif self.mavlink_configure_streams_checkbox.isChecked():
            self._log(
                "[픽스호크] Ethernet uXRCE 설정 중이므로 PX4 Ethernet MAVLink stream 시작은 건너뜁니다. "
                "USB MAVLink telemetry와 MAV_CMD_SET_MESSAGE_INTERVAL fallback을 사용합니다."
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
                QtCore.QTimer.singleShot(2500, self._retry_ros2_after_mavlink_recovery)

    def _start_mavlink_recovery_after_ros2_failure(self, error: Exception) -> None:
        if not self._configured_mavlink_sides():
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
            self.connect_mavlink_pixhawk(force_setup=True)
            if not self._mavlink_connected_sides():
                self._ros2_retry_after_uxrce_agent_setup = False
        except Exception as fallback_exc:
            self._ros2_retry_after_uxrce_agent_setup = False
            self._log(f"[픽스호크] MAVLink 복구/fallback 시작 실패: {fallback_exc} (원래 ROS2 오류: {error})")

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
        if self.pixhawk is None:
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
            "/fs/microsd/net.cfg, /fs/microsd/etc/extras.txt. PX4 재부팅 후 ping으로 eth0 적용을 확인하세요."
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
                "uxrce_dds_client status",
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
        elif camera == "right":
            eth_ip = self.right_pixhawk_eth_ip_edit.text().strip()
            namespace = self.right_namespace_edit.text().strip().strip("/")
            dds_key = 2
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
        extras_lines = [ifup_cmd, ifconfig_cmd, uxrce_stop_cmd, uxrce_start_cmd]
        return {
            "eth_ip": eth_ip,
            "host_ip": host_ip,
            "ifup_cmd": ifup_cmd,
            "ifconfig_cmd": ifconfig_cmd,
            "uxrce_stop_cmd": uxrce_stop_cmd,
            "uxrce_start_cmd": uxrce_start_cmd,
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

    def apply_rm3100_yaw_source(self, camera: str) -> None:
        combo = self.rm3100_yaw_source_combos.get(camera)
        status_label = self.rm3100_yaw_source_status_labels.get(camera)
        selected = "current" if combo is None else str(combo.currentData() or "current")
        if selected == "current":
            command = "param show CAL_MAG_PRIME"
            status_text = "Yaw 기준: PX4 현재 CAL_MAG_PRIME 조회"
            self._send_px4_shell_command(camera, command)
            if status_label is not None:
                status_label.setText(status_text)
            return
        state = self.latest_mag_sensors.get(camera, {}).get(selected)
        if state is None:
            status_text = f"Yaw 기준 적용 실패: {selected} 샘플 대기 중"
            if status_label is not None:
                status_label.setText(status_text)
            self._log(f"[픽스호크] {camera_label_ko(camera)} {status_text}")
            return
        device_id, device_source = self._mag_state_display_device_id(camera, state)
        if device_id is None:
            status_text = f"Yaw 기준 적용 실패: {selected} device_id 없음"
            if status_label is not None:
                status_label.setText(status_text)
            self._log(
                f"[픽스호크] {camera_label_ko(camera)} {status_text}. "
                "MAVLink SCALED_IMU fallback 샘플에서 PX4 파라미터 기반 device_id도 찾지 못했습니다."
            )
            return
        command = f"param set CAL_MAG_PRIME {int(device_id)}\nparam save\nparam show CAL_MAG_PRIME"
        self._send_px4_shell_command(camera, command)
        status_text = f"Yaw 기준: {selected} device={int(device_id)} ({device_source}) 적용 명령 전송"
        if status_label is not None:
            status_label.setText(status_text)
        self._log(f"[픽스호크] {camera_label_ko(camera)} {status_text}")

    def _px4_mag_camera_label(self, camera: str) -> str:
        return "Pixhawk 1" if camera == "left" else "Pixhawk 2"

    def _px4_mag_manager_has_data_source(self, camera: str) -> bool:
        if self.latest_vehicle_mag_device_ids.get(camera) is not None:
            return True
        mavlink = getattr(self, "mavlink_pixhawk", None)
        return mavlink is not None and camera in getattr(mavlink, "connections", {})

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
            self.px4_mag_use_rm3100_buttons.get(camera),
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
            manager.actual_selected_device_id = self.latest_vehicle_mag_device_ids.get(side)
            status = manager.refresh()
            self._render_px4_mag_status(side, status)

    def use_rm3100_for_yaw_from_panel(self, camera: str = "left") -> None:
        manager = self.px4_mag_managers[camera]
        label = self._px4_mag_camera_label(camera)
        if not self._px4_mag_manager_has_data_source(camera):
            self._log(f"[PX4 MAG] {label} 설정 변경 차단: MAVLink/ROS2 MAG 수신 없음")
            self._clear_px4_mag_manager_ui(camera)
            return
        if manager.armed:
            self._log(f"[PX4 MAG] {label} 설정 변경 차단: Disarm required")
            self.refresh_px4_mag_manager(camera)
            return
        try:
            status = manager.use_rm3100_for_yaw()
        except RuntimeError as exc:
            self._log(f"[PX4 MAG] {label} RM3100을 Yaw 기준으로 설정 실패: {exc}")
            self.refresh_px4_mag_manager(camera)
            return
        for line in manager.last_change_log:
            self._log(f"[PX4 MAG] {label} {line}")
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
        result = QtWidgets.QMessageBox.question(
            self,
            f"Reboot {label}",
            f"{label} reboot command is not wired in this GUI path.\nQGC 또는 PX4 shell에서 재부팅하세요.",
            QtWidgets.QMessageBox.Ok,
        )
        if result == QtWidgets.QMessageBox.Ok:
            self._log(f"[PX4 MAG] {label} PX4 재부팅: QGC 또는 PX4 shell에서 수행 필요")
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
        return "GPS/RTK baseline only"

    def _format_mount_assumption_summary(self) -> str:
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

    def set_selected_wiris_time_from_pc(self) -> None:
        target = str(self.wiris_command_target_combo.currentData() or "both")
        target_infos = [
            (label, host, self._control_port_for_camera(label), self.zoom_timeout_spin.value())
            for label, host in self.zoom_hosts(target)
        ]
        if not target_infos:
            self.wiris_command_status_label.setText("시간을 설정할 WIRIS 대상이 없습니다.")
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
            preview_width=self.preview_width_spin.value(),
            preview_rate_hz=self.preview_rate_spin.value(),
            rtsp_latency_ms=self.rtsp_latency_spin.value(),
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
            mavlink_configure_streams=self.mavlink_configure_streams_checkbox.isChecked(),
            mavlink_stream_rate=self.mavlink_stream_rate_spin.value(),
            mavlink_configure_uxrce_agent=self.mavlink_configure_uxrce_agent_checkbox.isChecked(),
            uxrce_port=self.uxrce_port_spin.value(),
            auto_load_intrinsics=True,
            manual_focal_override=self.manual_focal_checkbox.isChecked(),
            use_triangulation=True,
            triangulation_extrinsics_source=self._triangulation_extrinsics_source(),
            measurement_display_mode=self._measurement_display_mode(),
            use_rtk_baseline=self.use_rtk_baseline_checkbox.isChecked(),
            require_rtk_fixed=self.require_rtk_fixed_checkbox.isChecked(),
            uam_telemetry_source=str(self.uam_source_combo.currentData() or "ros2"),
            uam_ros2_topic=self.uam_ros2_topic_edit.text().strip(),
            uam_ros2_x_field=self.uam_ros2_x_field_edit.text().strip(),
            uam_ros2_y_field=self.uam_ros2_y_field_edit.text().strip(),
            uam_ros2_z_field=self.uam_ros2_z_field_edit.text().strip(),
            uam_mavlink_side=str(self.uam_mavlink_side_combo.currentData() or "left"),
            uam_mavlink_url=self.uam_mavlink_url_edit.text().strip(),
            uam_mavlink_baud=self.uam_mavlink_baud_spin.value(),
            uam_mavlink_message=self.uam_mavlink_message_edit.text().strip().upper(),
            uam_mavlink_x_field=self.uam_mavlink_x_field_edit.text().strip(),
            uam_mavlink_y_field=self.uam_mavlink_y_field_edit.text().strip(),
            uam_mavlink_z_field=self.uam_mavlink_z_field_edit.text().strip(),
            uam_compare_max_dt_s=self.uam_compare_max_dt_spin.value(),
            uam_compare_time_offset_s=self.uam_time_offset_spin.value(),
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

    def capture_calibration_images(self) -> None:
        frames = {
            "left": self.left_pane.current_image(),
            "right": self.right_pane.current_image(),
        }
        if frames["left"] is None and frames["right"] is None:
            self.calibration_status_label.setText("Start at least one camera stream before capturing calibration images.")
            self._log("[캘리브레이션] 이미지 캡처 실패: 좌/우 카메라 프레임 중 하나 이상이 필요합니다.")
            return
        _root, left_dir, right_dir = self._calibration_dirs()
        left_dir.mkdir(parents=True, exist_ok=True)
        right_dir.mkdir(parents=True, exist_ok=True)
        stem = self._next_calibration_capture_stem()
        available, error = opencv_available()
        config = self._checkerboard_config() if available else None
        directories = {"left": left_dir, "right": right_dir}
        saved_sides: list[str] = []
        status_parts: list[str] = []
        for side in ("left", "right"):
            image = frames[side]
            if image is None:
                status_parts.append(f"{side}=no frame")
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

        status = f"Captured {stem}: " + ", ".join(status_parts)
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
        self.last_calibration_result = StereoIntrinsicsResult(left=left, right=right, board=config)
        self.last_calibration_zoom_level = self.zoom_level_spin.value()
        warning = ""
        if left.image_count < min_images or right.image_count < min_images:
            warning = f"\nWARNING: valid images below requested minimum {min_images}."
        text = (
            f"Left: images={left.image_count}, error={left.reprojection_error_px:.3f}px, "
            f"fx={left.camera_matrix[0][0]:.2f}, fy={left.camera_matrix[1][1]:.2f}, "
            f"cx={left.camera_matrix[0][2]:.2f}, cy={left.camera_matrix[1][2]:.2f}\n"
            f"D_left={left.distortion_coefficients}\n\n"
            f"Right: images={right.image_count}, error={right.reprojection_error_px:.3f}px, "
            f"fx={right.camera_matrix[0][0]:.2f}, fy={right.camera_matrix[1][1]:.2f}, "
            f"cx={right.camera_matrix[0][2]:.2f}, cy={right.camera_matrix[1][2]:.2f}\n"
            f"D_right={right.distortion_coefficients}{warning}"
        )
        self.calibration_result_text.setPlainText(text)
        self.calibration_status_label.setText("Intrinsic calibration complete.")
        self._refresh_camera_parameters_tab()
        self._refresh_zoom_intrinsics_tab()
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
            "note": "Generated by StereoDistance Camera Calibration tab",
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
        self.calibration_status_label.setText("Applied intrinsics and average fx to Stereo Distance.")
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
        self.stop_all()
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

    def _active_baseline_m(self) -> tuple[float, str, tuple[str, ...]]:
        if not self._live_gps_baseline_enabled():
            return 0.0, "gps-rtk-disabled", ("GPS/RTK baseline disabled",)
        baseline = self.latest_rtk_baseline
        if baseline is None:
            return 0.0, "gps-rtk-waiting", ("waiting for live GPS baseline",)
        usable, reason = self._gps_baseline_usable_for_translation(baseline)
        if not usable:
            return baseline.baseline_m, "gps-rtk-blocked", (*baseline.warnings, reason)
        return baseline.baseline_m, "gps-rtk", baseline.warnings

    def _live_gps_baseline_enabled(self) -> bool:
        return not hasattr(self, "use_rtk_baseline_checkbox") or self.use_rtk_baseline_checkbox.isChecked()

    def _gps_fix_ready_for_baseline(self, state: GpsPositionState | None) -> bool:
        return state is not None and state.has_position and state.fix_type is not None and state.fix_type >= 3

    def _gps_baseline_usable_for_translation(self, baseline: RtkBaselineEstimate | None) -> tuple[bool, str]:
        if baseline is None:
            return False, "live GPS baseline waiting"
        left = self.latest_gps.get("left")
        right = self.latest_gps.get("right")
        if left is None or right is None:
            if self.require_rtk_fixed_checkbox.isChecked() and baseline.fix_quality != "GOOD":
                return False, "RTK Fixed required"
            if baseline.baseline_m <= 0.0:
                return False, "GPS baseline length <= 0"
            return True, "usable"
        if not self._gps_fix_ready_for_baseline(left) or not self._gps_fix_ready_for_baseline(right):
            return False, "GPS fix < 3D"
        if self.require_rtk_fixed_checkbox.isChecked() and not (
            left is not None and right is not None and left.fix_type == 6 and right.fix_type == 6
        ):
            return False, "RTK Fixed required"
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
        mode = "계산 ON" if self._live_gps_baseline_enabled() else "계산 OFF"
        fixed_text = "YES" if fixed else "NO"
        usability = "사용 가능" if usable else f"대기/차단: {reason}"
        return f"RTK Fixed: {fixed_text} | GPS baseline {mode}, {usability} | {side_text('P1', left)} | {side_text('P2', right)}"

    def _refresh_rtk_baseline(self) -> RtkBaselineEstimate | None:
        gps_reference_enabled = (
            not hasattr(self, "use_rtk_baseline_checkbox") or self.use_rtk_baseline_checkbox.isChecked()
        )
        baseline = estimate_rtk_baseline(
            self.latest_gps.get("left"),
            self.latest_gps.get("right"),
            now_s=time.monotonic(),
            require_fixed=self.require_rtk_fixed_checkbox.isChecked(),
        )
        self.latest_rtk_baseline = baseline
        reference_text = " | 계산 ON" if gps_reference_enabled else " | 계산 OFF, 표시만"
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
        forward_warn = abs(z_m) > max(0.2, abs(x_m) * 0.5)
        down_warn = abs(y_m) > max(0.1, abs(x_m) * 0.25)
        self._set_rtk_summary_chip(
            "camera",
            f"Camera R/D/F {x_m:+.3f}/{y_m:+.3f}/{z_m:+.3f} m",
            "warn" if forward_warn or down_warn else "good",
        )
        self.rtk_alignment_detail_label.setText(
            f"정렬 성분: {antenna_text}. "
            f"카메라 기준 Right={x_m:.3f} m, Down={y_m:.3f} m, Forward={z_m:.3f} m. "
            f"카메라 좌우 성분 |Right|={abs(x_m):.3f} m입니다.{attitude_text}"
        )

    def _translation_right_in_left_m(self) -> tuple[float, float, float] | None:
        left_ext = self.latest_extrinsics.get("left")
        if left_ext is None:
            return None
        if not self._live_gps_baseline_enabled():
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
        return camera_translation_right_in_left_camera(
            baseline_right_in_left_world_m=world_vector,
            left_body_to_world_q=left_ext.q,
            right_body_to_world_q=right_ext.q if right_ext is not None else left_ext.q,
            scale_to_m=None,
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
        pair = match_nearest_pair(
            self.detection_buffers["left"],
            self.detection_buffers["right"],
            max_delta_s=self.max_dt_spin.value(),
        )
        if pair is None:
            self._clear_measurement_pair_display()
            self._clear_triangulation_quality("timestamp가 맞는 좌/우 검출 쌍 대기 중")
            self._set_distance_text(
                "거리: timestamp가 맞는 좌/우 검출 대기 중",
                quality="BAD",
                detail=self._detection_wait_detail(),
            )
            return
        if self._measurement_display_mode() == "detection":
            self._show_measurement_pair(pair.left, pair.right)
        else:
            self._clear_measurement_pair_display()

        left_attitude = self.attitude_buffers["left"].nearest(pair.midpoint_s)
        right_attitude = self.attitude_buffers["right"].nearest(pair.midpoint_s)
        left_ext = left_attitude.sample if left_attitude is not None else None
        right_ext = right_attitude.sample if right_attitude is not None else None
        attitude_pair_delta_s = None
        relative = None
        if left_ext is not None and right_ext is not None:
            attitude_pair_delta_s = abs(left_ext.timestamp_s - right_ext.timestamp_s)
            relative = relative_attitude(left_ext.q, right_ext.q)

        quality = "GOOD"
        warnings: tuple[str, ...] = ()

        baseline_m, baseline_source, baseline_warnings = self._active_baseline_m()
        if baseline_source in {"rtk-blocked", "gps-rtk-blocked"}:
            self._clear_triangulation_quality("GPS baseline 조건 미충족")
            self._set_distance_text(
                "거리: GPS baseline 조건 미충족 | " + "; ".join(baseline_warnings),
                quality="BAD",
                detail="그래프 미표시: GPS baseline 조건을 만족하지 못했습니다.",
            )
            return
        combined_warnings = tuple(dict.fromkeys((*warnings, *baseline_warnings)))
        if baseline_source.startswith("RTK") and quality == "GOOD" and self.latest_rtk_baseline is not None:
            quality = self.latest_rtk_baseline.fix_quality

        calibration = StereoCalibration(
            focal_length_px=self.focal_spin.value(),
            baseline_m=baseline_m,
            max_time_delta_s=self.max_dt_spin.value(),
            max_vertical_delta_px=float("inf"),
            require_same_class=self.same_class_checkbox.isChecked(),
        )
        triangulation_source = self._triangulation_extrinsics_source()
        translation_right_in_left_m = self._translation_right_in_left_m()
        geometry_result = calculate_measurement_geometry(
            MeasurementGeometryInput(
                left=pair.left,
                right=pair.right,
                calibration=calibration,
                intrinsics=self.active_intrinsics,
                triangulation_source=triangulation_source,
                left_extrinsic=left_ext,
                right_extrinsic=right_ext,
                left_attitude_age_s=left_attitude.age_s if left_attitude is not None else None,
                right_attitude_age_s=right_attitude.age_s if right_attitude is not None else None,
                attitude_pair_delta_s=attitude_pair_delta_s,
                relative_roll_deg=relative.roll_deg if relative is not None else None,
                relative_pitch_deg=relative.pitch_deg if relative is not None else None,
                relative_yaw_deg=relative.yaw_deg if relative is not None else None,
                sync_quality=quality,
                warnings=combined_warnings,
                translation_right_in_left_m=translation_right_in_left_m,
                reprojection_profile=self.bbox_reprojection_profile,
            )
        )
        match = geometry_result.match
        triangulated = geometry_result.triangulated
        estimate = geometry_result.estimate
        combined_warnings = geometry_result.warnings
        if match is None:
            self._clear_triangulation_quality("유효한 스테레오 기하 대기 중")
            self._set_distance_text(
                "거리: 유효한 스테레오 기하 대기 중",
                quality="BAD",
                detail=self._stereo_geometry_wait_detail(pair),
            )
            return
        if triangulated is None:
            warning_text = f" | {'; '.join(combined_warnings)}" if combined_warnings else ""
            self._clear_triangulation_quality("3D ray 기하 대기 중")
            self._set_distance_text(
                f"거리: 3D ray 기하 대기 중 ({self._triangulation_extrinsics_source_label()}){warning_text}",
                quality="BAD",
                detail=self._triangulation_wait_detail(combined_warnings),
            )
            return
        if estimate is None:
            self._clear_triangulation_quality("유효한 거리 계산 대기 중")
            self._set_distance_text(
                "거리: 유효한 거리 계산 대기 중",
                quality="BAD",
                detail="그래프 미표시: 현재 입력으로 거리를 만들 수 없습니다.",
            )
            return

        raw_point_left_m = triangulated.point_left_m
        raw_point_center_m = (
            stereo_center_point_from_left_camera(raw_point_left_m, translation_right_in_left_m)
            if translation_right_in_left_m is not None
            else raw_point_left_m
        )
        reprojection_result = geometry_result.reprojection
        bbox_correction: dict[str, object] | None = None
        profile_status = self._inactive_profile_status()
        corrected_point_center_m = raw_point_center_m
        corrected_point_left_m = raw_point_left_m
        raw_center_distance_m = math.sqrt(
            raw_point_center_m[0] * raw_point_center_m[0]
            + raw_point_center_m[1] * raw_point_center_m[1]
            + raw_point_center_m[2] * raw_point_center_m[2]
        )
        bias_correction = None
        if reprojection_result is not None:
            bbox_correction = self._bbox_correction_from_reprojection_lookup(reprojection_result.lookup)
            profile_status = self._profile_status_from_reprojection_lookup(reprojection_result.lookup)
            if reprojection_result.triangulated is not None:
                corrected_point_left_m = reprojection_result.triangulated.point_left_m
                corrected_point_center_m = (
                    stereo_center_point_from_left_camera(corrected_point_left_m, translation_right_in_left_m)
                    if translation_right_in_left_m is not None
                    else corrected_point_left_m
                )
                corrected_point_center_m = self._apply_reprojection_output_policy(
                    raw_point_center_m=raw_point_center_m,
                    corrected_point_center_m=corrected_point_center_m,
                )
        elif self.bbox_center_adjustments:
            affine = self._affine_bbox_correction(pair.left, pair.right)
            if affine is not None:
                affine_triangulated = self._triangulate_bbox_centers(
                    affine["left_corrected_cx"],
                    affine["left_corrected_cy"],
                    affine["right_corrected_cx"],
                    affine["right_corrected_cy"],
                    left_ext=left_ext,
                    right_ext=right_ext,
                    translation_right_in_left_m=translation_right_in_left_m,
                )
                if affine_triangulated is not None:
                    bbox_correction = affine
                    profile_status = {
                        "profile_id": "global_affine_csv",
                        "profile_mode": "global_affine_csv",
                        "profile_fallback_level": "0",
                        "profile_fallback_reason": "csv_affine",
                    }
                    corrected_point_left_m = affine_triangulated.point_left_m
                    corrected_point_center_m = (
                        stereo_center_point_from_left_camera(corrected_point_left_m, translation_right_in_left_m)
                        if translation_right_in_left_m is not None
                        else corrected_point_left_m
                    )
        if bbox_correction is None and self.bias_profile_applied and self.active_bias_profile is not None:
            bias_correction = self.active_bias_profile.correction(raw_center_distance_m)
            if bias_correction is not None:
                corrected_point_center_m = (
                    raw_point_center_m[0] - bias_correction[0],
                    raw_point_center_m[1] - bias_correction[1],
                    raw_point_center_m[2] - bias_correction[2],
                )
        tracking_result = self._apply_tracking_filter(
            timestamp_s=max(pair.left.timestamp_s, pair.right.timestamp_s),
            point_m=corrected_point_center_m,
            estimate=estimate,
            baseline_m=baseline_m,
        )
        filtered_point_center_m = tracking_result.point_m if tracking_result is not None else corrected_point_center_m
        display_distance_m = math.sqrt(
            filtered_point_center_m[0] * filtered_point_center_m[0]
            + filtered_point_center_m[1] * filtered_point_center_m[1]
            + filtered_point_center_m[2] * filtered_point_center_m[2]
        )
        method_text = f"triangulated-{triangulation_source}"
        if profile_status.get("profile_id"):
            profile_suffix = profile_status.get("profile_mode") or "bbox-profile"
            if profile_status.get("profile_output_policy") == "range_only_forward_fusion":
                profile_suffix = f"{profile_suffix}:range-only-forward"
            method_text = f"{method_text}+{profile_suffix}"
        elif bias_correction is not None:
            method_text = f"{method_text}+legacy-3d-bias"
        if tracking_result is not None:
            method_text = f"{method_text}+kf"
        gap_text = f" gap={triangulated.closest_gap_m:.3f}m"
        bias_text = ""
        if bias_correction is not None:
            bias_text = f" bias={bias_correction[0]:+.2f}/{bias_correction[1]:+.2f}/{bias_correction[2]:+.2f}m"
        tracking_text = ""
        if tracking_result is not None:
            tracking_state = "reset" if tracking_result.reset else "update"
            if not tracking_result.accepted:
                tracking_state = "predict"
            tracking_text = f" kf={tracking_state} ni={tracking_result.normalized_innovation:.1f}"
        display_baseline_source = baseline_source
        display_baseline_m = baseline_m
        self._update_triangulation_quality(
            triangulated=triangulated,
            method_text=method_text,
            display_distance_m=display_distance_m,
            display_baseline_source=display_baseline_source,
            display_baseline_m=display_baseline_m,
            raw_point_left_m=raw_point_left_m,
            raw_point_center_m=raw_point_center_m,
            corrected_point_center_m=filtered_point_center_m,
            bias_correction=bias_correction,
        )

        attitude_text = "att=대기"
        if estimate.left_attitude_age_s is not None and estimate.right_attitude_age_s is not None:
            attitude_text = (
                f"attAge={estimate.left_attitude_age_s * 1000:.0f}/{estimate.right_attitude_age_s * 1000:.0f}ms"
            )
        relative_text = "relRPY=대기"
        if estimate.relative_roll_deg is not None:
            relative_text = (
                f"relRPY={estimate.relative_roll_deg:.2f}/"
                f"{estimate.relative_pitch_deg:.2f}/{estimate.relative_yaw_deg:.2f}°"
            )
        telemetry_range_m, origin_profile_id = self._live_telemetry_range()
        telemetry_text = ""
        if telemetry_range_m is not None:
            profile_text = f", profile={origin_profile_id}" if origin_profile_id else ""
            telemetry_text = f" telemetryRange={telemetry_range_m:.2f}m{profile_text}"
        warning_text = f" | {'; '.join(estimate.warnings)}" if estimate.warnings else ""
        self._set_distance_text(
            f"거리: {display_distance_m:.2f} m ({method_text}{gap_text}{bias_text}{tracking_text}) | bboxΔx={estimate.disparity_px:.1f}px "
            f"camΔt={estimate.time_delta_s * 1000:.0f}ms Δy={estimate.vertical_delta_px:.1f}px "
            f"baseline={display_baseline_source}:{display_baseline_m:.3f}m "
            f"{attitude_text} {relative_text}{telemetry_text} quality={estimate.sync_quality}{warning_text}",
            quality=estimate.sync_quality,
        )
        appended = self._append_distance_history(
            estimate=estimate,
            distance_m=display_distance_m,
            method=method_text,
            baseline_source=display_baseline_source,
            baseline_m=display_baseline_m,
            point_m=filtered_point_center_m,
            raw_point_m=raw_point_center_m,
            raw_point_left_m=raw_point_left_m,
            translation_right_in_left_m=translation_right_in_left_m,
            bias_correction_m=bias_correction,
            corrected_point_m=corrected_point_center_m,
            corrected_point_left_m=corrected_point_left_m,
            bbox_correction=bbox_correction,
            profile_status=profile_status,
            tracking_result=tracking_result,
        )
        if not appended:
            self._set_distance_text(
                self.last_distance_status,
                quality=estimate.sync_quality,
                detail="그래프 미표시: 직전과 동일한 샘플이라 중복 추가를 건너뛰었습니다.",
            )

    def _show_measurement_pair(self, left: DetectionObservation, right: DetectionObservation) -> None:
        self.left_pane.show_measurement_frame_for_detection(left)
        self.right_pane.show_measurement_frame_for_detection(right)

    def _clear_measurement_pair_display(self) -> None:
        self.left_pane.clear_measurement_pair_display()
        self.right_pane.clear_measurement_pair_display()

    def _detection_wait_detail(self) -> str:
        left_count = len(self.detection_buffers["left"])
        right_count = len(self.detection_buffers["right"])
        left_stats = self.camera_stream_stats.get("left") or {}
        right_stats = self.camera_stream_stats.get("right") or {}
        left_yolo_hz = float(left_stats.get("inferred_hz", 0.0))
        right_yolo_hz = float(right_stats.get("inferred_hz", 0.0))
        if left_yolo_hz <= 0.0 or right_yolo_hz <= 0.0:
            return (
                "그래프 미표시: YOLO 추론 대기 중 "
                f"(left={left_yolo_hz:.1f} Hz, right={right_yolo_hz:.1f} Hz)."
            )
        if left_count == 0 or right_count == 0:
            return f"그래프 미표시: bbox 검출 대기 중 (left={left_count}, right={right_count})."
        return (
            "그래프 미표시: 좌/우 bbox timestamp가 `최대 Δt` 안에서 아직 맞지 않습니다 "
            f"(left={left_count}, right={right_count})."
        )

    def _stereo_geometry_wait_detail(self, pair) -> str:
        left = pair.left
        right = pair.right
        left_class = getattr(left, "class_name", None) or f"class {getattr(left, 'class_id', '?')}"
        right_class = getattr(right, "class_name", None) or f"class {getattr(right, 'class_id', '?')}"
        left_center = getattr(left, "center", (0.0, 0.0))
        right_center = getattr(right, "center", (0.0, 0.0))
        disparity_px = abs(float(left_center[0]) - float(right_center[0]))
        time_delta_ms = abs(float(left.timestamp_s) - float(right.timestamp_s)) * 1000.0
        if self.same_class_checkbox.isChecked() and left.class_id != right.class_id:
            return (
                "그래프 미표시: `같은 클래스만 매칭`이 켜져 있지만 좌/우 class가 다릅니다. "
                f"left={left_class}, right={right_class}."
            )
        if not isfinite(disparity_px) or disparity_px <= 1e-6:
            return (
                "그래프 미표시: 좌/우 bbox 중심의 x 차이가 0에 가까워 disparity를 만들 수 없습니다. "
                f"left_x={float(left_center[0]):.1f}px, right_x={float(right_center[0]):.1f}px."
            )
        if time_delta_ms > self.max_dt_spin.value() * 1000.0:
            return (
                "그래프 미표시: 좌/우 bbox timestamp 차이가 `최대 Δt`보다 큽니다. "
                f"camΔt={time_delta_ms:.0f}ms, limit={self.max_dt_spin.value() * 1000.0:.0f}ms."
            )
        return (
            "그래프 미표시: 좌/우 bbox는 선택됐지만 거리 계산용 stereo match가 생성되지 않았습니다. "
            f"disparity={disparity_px:.1f}px, camΔt={time_delta_ms:.0f}ms."
        )

    def _triangulation_wait_detail(self, warnings: tuple[str, ...]) -> str:
        warning_text = "; ".join(warnings)
        if any("waiting for Pixhawk R" in warning for warning in warnings):
            return (
                "그래프 미표시: bbox 매칭은 됐지만 Pixhawk 자세 샘플이 아직 없어 3D ray 방향을 만들 수 없습니다. "
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

    def _inactive_profile_status(self) -> dict[str, str]:
        return {
            "profile_id": "",
            "profile_mode": "",
            "profile_output_policy": "",
            "profile_fallback_level": "",
            "profile_fallback_reason": "no_profile",
        }

    def _profile_status_from_reprojection_lookup(self, lookup: ReprojectionLookupResult) -> dict[str, str]:
        mode = self.bbox_reprojection_profile.mode if self.bbox_reprojection_profile is not None else ""
        output_policy = (
            self.bbox_reprojection_profile.output_policy if self.bbox_reprojection_profile is not None else ""
        )
        return {
            "profile_id": lookup.profile_id,
            "profile_mode": mode,
            "profile_output_policy": output_policy,
            "profile_fallback_level": str(lookup.fallback_level),
            "profile_fallback_reason": lookup.fallback_reason,
        }

    def _apply_reprojection_output_policy(
        self,
        *,
        raw_point_center_m: tuple[float, float, float],
        corrected_point_center_m: tuple[float, float, float],
    ) -> tuple[float, float, float]:
        profile = self.bbox_reprojection_profile
        if profile is None or profile.output_policy != "range_only_forward_fusion":
            return corrected_point_center_m
        return (raw_point_center_m[0], raw_point_center_m[1], corrected_point_center_m[2])

    def _bbox_correction_from_reprojection_lookup(self, lookup: ReprojectionLookupResult) -> dict[str, object]:
        return {
            "left_corrected_cx": lookup.corrected_left_u,
            "left_corrected_cy": lookup.corrected_left_v,
            "right_corrected_cx": lookup.corrected_right_u,
            "right_corrected_cy": lookup.corrected_right_v,
            "left_bbox_du_px": lookup.du_left,
            "left_bbox_dv_px": lookup.dv_left,
            "right_bbox_du_px": lookup.du_right,
            "right_bbox_dv_px": lookup.dv_right,
            "profile_id": lookup.profile_id,
            "profile_id_hash": (
                self.bbox_reprojection_profile.profile_id_hash
                if self.bbox_reprojection_profile is not None
                else 0
            ),
            "fallback_level": lookup.fallback_level,
            "fallback_reason": lookup.fallback_reason,
            "profile_cell_key": lookup.cell_key,
            "profile_cell_sample_count": lookup.sample_count,
        }

    def _affine_bbox_correction(
        self,
        left: DetectionObservation,
        right: DetectionObservation,
    ) -> dict[str, object] | None:
        left_adjustment = self.bbox_center_adjustments.get("left")
        right_adjustment = self.bbox_center_adjustments.get("right")
        if left_adjustment is None or right_adjustment is None:
            return None
        left_u, left_v = left.center
        right_u, right_v = right.center
        left_corrected = self._apply_bbox_affine(left_adjustment, left_u, left_v)
        right_corrected = self._apply_bbox_affine(right_adjustment, right_u, right_v)
        return {
            "left_corrected_cx": left_corrected[0],
            "left_corrected_cy": left_corrected[1],
            "right_corrected_cx": right_corrected[0],
            "right_corrected_cy": right_corrected[1],
            "left_bbox_du_px": left_corrected[0] - left_u,
            "left_bbox_dv_px": left_corrected[1] - left_v,
            "right_bbox_du_px": right_corrected[0] - right_u,
            "right_bbox_dv_px": right_corrected[1] - right_v,
            "profile_id": "global_affine_csv",
            "profile_id_hash": 0,
            "fallback_level": 0,
            "fallback_reason": "csv_affine",
            "profile_cell_key": "",
            "profile_cell_sample_count": 0,
        }

    @staticmethod
    def _apply_bbox_affine(
        adjustment: tuple[tuple[float, float, float, float], tuple[float, float, float, float]],
        u: float,
        v: float,
    ) -> tuple[float, float]:
        row_u, row_v = adjustment
        return (
            row_u[0] * u + row_u[1] * v + row_u[2] + row_u[3],
            row_v[0] * u + row_v[1] * v + row_v[2] + row_v[3],
        )

    def _triangulate_bbox_centers(
        self,
        left_u: float,
        left_v: float,
        right_u: float,
        right_v: float,
        *,
        left_ext: object | None,
        right_ext: object | None,
        translation_right_in_left_m: tuple[float, float, float] | None,
    ) -> RayTriangulationResult | None:
        if self.active_intrinsics is None or left_ext is None or right_ext is None or translation_right_in_left_m is None:
            return None
        try:
            return triangulate_pixels_with_pixhawk_mount(
                left_pixel=(float(left_u), float(left_v)),
                right_pixel=(float(right_u), float(right_v)),
                left_intrinsics=self.active_intrinsics.left,
                right_intrinsics=self.active_intrinsics.right,
                left_body_to_world_q=left_ext.q,
                right_body_to_world_q=right_ext.q,
                translation_right_in_left_m=translation_right_in_left_m,
            )
        except Exception:
            return None

    def _live_telemetry_range(self) -> tuple[float | None, str | None]:
        drone_position_m = self.latest_drone_position_m
        if self.uam_alignment_profile is None or drone_position_m is None:
            return None, None
        baseline_center_ned = self.uam_alignment_profile.origin_offset_m
        range_m = math.sqrt(
            (drone_position_m[0] - baseline_center_ned[0]) ** 2
            + (drone_position_m[1] - baseline_center_ned[1]) ** 2
            + (drone_position_m[2] - baseline_center_ned[2]) ** 2
        )
        return range_m, self.uam_alignment_profile.profile_id

    def _tracking_focal_px(self) -> float:
        if self.active_intrinsics is not None:
            try:
                matrix = self.active_intrinsics.left.camera_matrix
                fx = float(matrix[0][0])
                if isfinite(fx) and fx > 0.0:
                    return fx
            except Exception:
                pass
        return float(self.focal_spin.value())

    def _apply_tracking_filter(
        self,
        *,
        timestamp_s: float,
        point_m: tuple[float, float, float],
        estimate,
        baseline_m: float,
    ) -> TrackingUpdateResult | None:
        try:
            variance_m2 = measurement_variance_from_stereo(
                point_m=point_m,
                focal_length_px=self._tracking_focal_px(),
                baseline_m=baseline_m,
                disparity_px=float(estimate.disparity_px),
                left_confidence=getattr(estimate.left, "confidence", None),
                right_confidence=getattr(estimate.right, "confidence", None),
                config=self.tracking_filter.config,
            )
            return self.tracking_filter.update(
                timestamp_s=float(timestamp_s),
                point_m=point_m,
                measurement_variance_m2=variance_m2,
            )
        except Exception as exc:
            self._log(f"[Tracking] Kalman update skipped: {exc}")
            return None

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
        translation_right_in_left_m: tuple[float, float, float] | None = None,
        bias_correction_m: tuple[float, float, float] | None = None,
        corrected_point_m: tuple[float, float, float] | None = None,
        corrected_point_left_m: tuple[float, float, float] | None = None,
        bbox_correction: dict[str, object] | None = None,
        profile_status: dict[str, str] | None = None,
        tracking_result: TrackingUpdateResult | None = None,
        point_left_m: tuple[float, float, float] | None = None,
    ) -> bool:
        key = (estimate.left.timestamp_s, estimate.right.timestamp_s, method)
        if key == self._last_distance_history_key:
            return False
        self._last_distance_history_key = key
        if point_m is None:
            point_m = point_left_m
        if raw_point_m is None:
            raw_point_m = raw_point_left_m
        if corrected_point_m is None:
            corrected_point_m = point_m
        if corrected_point_left_m is None:
            corrected_point_left_m = point_left_m
        if profile_status is None:
            profile_status = self._inactive_profile_status()
        timestamp_s = max(estimate.left.timestamp_s, estimate.right.timestamp_s)
        class_name = self._distance_history_class_name(estimate)
        recording_elapsed_s = None
        if self.distance_recording_enabled:
            if self.distance_recording_started_monotonic_s is None:
                self.distance_recording_started_monotonic_s = time.monotonic()
            recording_elapsed_s = max(0.0, time.monotonic() - self.distance_recording_started_monotonic_s)
        drone_position_m = self.latest_drone_position_m
        live_truth_range_m = None
        live_range_error_m = None
        live_alignment_profile_id = None
        live_truth_range_m, live_alignment_profile_id = self._live_telemetry_range()
        if live_truth_range_m is not None:
            live_range_error_m = distance_m - live_truth_range_m
        point = DistanceHistoryPoint(
            timestamp_s=timestamp_s,
            distance_m=distance_m,
            quality=estimate.sync_quality,
            method=method,
            x_m=None if point_m is None else float(point_m[0]),
            y_m=None if point_m is None else float(point_m[1]),
            z_m=None if point_m is None else float(point_m[2]),
            live_truth_range_m=live_truth_range_m,
            live_range_error_m=live_range_error_m,
            live_truth_timestamp_s=self.latest_drone_position_timestamp_s,
            alignment_profile_id=live_alignment_profile_id,
            class_name=class_name,
            sample_key=key,
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
        corrected_distance_m = None
        if corrected_point_m is not None:
            corrected_distance_m = math.sqrt(
                corrected_point_m[0] * corrected_point_m[0]
                + corrected_point_m[1] * corrected_point_m[1]
                + corrected_point_m[2] * corrected_point_m[2]
            )
        error_m = None
        if point_m is not None and drone_position_m is not None:
            error_m = (
                point_m[0] - drone_position_m[0],
                point_m[1] - drone_position_m[1],
                point_m[2] - drone_position_m[2],
            )
        error_basis_warning = (
            "measurement stereo_center_opencv_optical vs truth px4_local_ned; "
            "alignment profile required for final axis error"
        )
        row = {
            "session_id": self.recording_session_id,
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
            "truth_frame": TRUTH_FRAME,
            "truth_axis_contract": TRUTH_AXIS_CONTRACT,
            "comparison_frame": "mixed-frame",
            "x_m": self._fmt_record_float(point.x_m, 6),
            "y_m": self._fmt_record_float(point.y_m, 6),
            "z_m": self._fmt_record_float(point.z_m, 6),
            "stereo_center_right_m": self._fmt_record_float(point.x_m, 6),
            "stereo_center_down_m": self._fmt_record_float(point.y_m, 6),
            "stereo_center_forward_m": self._fmt_record_float(point.z_m, 6),
            "raw_distance_m": self._fmt_record_float(raw_distance_m, 6),
            "raw_x_m": self._fmt_record_float(None if raw_point_m is None else raw_point_m[0], 6),
            "raw_y_m": self._fmt_record_float(None if raw_point_m is None else raw_point_m[1], 6),
            "raw_z_m": self._fmt_record_float(None if raw_point_m is None else raw_point_m[2], 6),
            "raw_stereo_center_right_m": self._fmt_record_float(None if raw_point_m is None else raw_point_m[0], 6),
            "raw_stereo_center_down_m": self._fmt_record_float(None if raw_point_m is None else raw_point_m[1], 6),
            "raw_stereo_center_forward_m": self._fmt_record_float(None if raw_point_m is None else raw_point_m[2], 6),
            "raw_right_m": self._fmt_record_float(None if raw_point_m is None else raw_point_m[0], 6),
            "raw_down_m": self._fmt_record_float(None if raw_point_m is None else raw_point_m[1], 6),
            "raw_forward_m": self._fmt_record_float(None if raw_point_m is None else raw_point_m[2], 6),
            "raw_range_m": self._fmt_record_float(raw_distance_m, 6),
            "estimated_right_m": self._fmt_record_float(None if raw_point_m is None else raw_point_m[0], 6),
            "estimated_down_m": self._fmt_record_float(None if raw_point_m is None else raw_point_m[1], 6),
            "estimated_forward_m": self._fmt_record_float(None if raw_point_m is None else raw_point_m[2], 6),
            "estimated_range_m": self._fmt_record_float(raw_distance_m, 6),
            "raw_left_x_m": self._fmt_record_float(None if raw_point_left_m is None else raw_point_left_m[0], 6),
            "raw_left_y_m": self._fmt_record_float(None if raw_point_left_m is None else raw_point_left_m[1], 6),
            "raw_left_z_m": self._fmt_record_float(None if raw_point_left_m is None else raw_point_left_m[2], 6),
            "translation_right_in_left_x_m": self._fmt_record_float(
                None if translation_right_in_left_m is None else translation_right_in_left_m[0], 6
            ),
            "translation_right_in_left_y_m": self._fmt_record_float(
                None if translation_right_in_left_m is None else translation_right_in_left_m[1], 6
            ),
            "translation_right_in_left_z_m": self._fmt_record_float(
                None if translation_right_in_left_m is None else translation_right_in_left_m[2], 6
            ),
            "live_truth_range_m": self._fmt_record_float(live_truth_range_m, 6),
            "live_range_error_m": self._fmt_record_float(live_range_error_m, 6),
            "live_range_error_basis": "baseline_center" if live_truth_range_m is not None else "",
            "live_range_alignment_profile_id": live_alignment_profile_id or "",
            "telemetry_range_m": self._fmt_record_float(live_truth_range_m, 6),
            "telemetry_range_error_m": self._fmt_record_float(live_range_error_m, 6),
            "telemetry_range_basis": "baseline_center" if live_truth_range_m is not None else "",
            "telemetry_range_profile_id": live_alignment_profile_id or "",
            "live_truth_timestamp_s": self._fmt_record_float(self.latest_drone_position_timestamp_s, 6),
            "corrected_distance_m": self._fmt_record_float(corrected_distance_m, 6),
            "corrected_range_m": self._fmt_record_float(corrected_distance_m, 6),
            "corrected_x_m": self._fmt_record_float(None if corrected_point_m is None else corrected_point_m[0], 6),
            "corrected_y_m": self._fmt_record_float(None if corrected_point_m is None else corrected_point_m[1], 6),
            "corrected_z_m": self._fmt_record_float(None if corrected_point_m is None else corrected_point_m[2], 6),
            "corrected_right_m": self._fmt_record_float(None if corrected_point_m is None else corrected_point_m[0], 6),
            "corrected_down_m": self._fmt_record_float(None if corrected_point_m is None else corrected_point_m[1], 6),
            "corrected_forward_m": self._fmt_record_float(None if corrected_point_m is None else corrected_point_m[2], 6),
            "filtered_distance_m": f"{distance_m:.6f}",
            "filtered_range_m": f"{distance_m:.6f}",
            "filtered_x_m": self._fmt_record_float(point.x_m, 6),
            "filtered_y_m": self._fmt_record_float(point.y_m, 6),
            "filtered_z_m": self._fmt_record_float(point.z_m, 6),
            "filtered_right_m": self._fmt_record_float(point.x_m, 6),
            "filtered_down_m": self._fmt_record_float(point.y_m, 6),
            "filtered_forward_m": self._fmt_record_float(point.z_m, 6),
            "tracking_filter_id": "" if tracking_result is None else tracking_result.filter_id,
            "tracking_initialized": self._fmt_record_bool(None if tracking_result is None else tracking_result.initialized),
            "tracking_accepted": self._fmt_record_bool(None if tracking_result is None else tracking_result.accepted),
            "tracking_reset": self._fmt_record_bool(None if tracking_result is None else tracking_result.reset),
            "tracking_normalized_innovation": self._fmt_record_float(
                None if tracking_result is None else tracking_result.normalized_innovation, 6
            ),
            "tracking_measurement_std_right_m": self._fmt_record_float(
                None if tracking_result is None else tracking_result.measurement_std_m[0], 6
            ),
            "tracking_measurement_std_down_m": self._fmt_record_float(
                None if tracking_result is None else tracking_result.measurement_std_m[1], 6
            ),
            "tracking_measurement_std_forward_m": self._fmt_record_float(
                None if tracking_result is None else tracking_result.measurement_std_m[2], 6
            ),
            "tracking_cov_right_m2": self._fmt_record_float(
                None if tracking_result is None else tracking_result.covariance_diag_m2[0], 6
            ),
            "tracking_cov_down_m2": self._fmt_record_float(
                None if tracking_result is None else tracking_result.covariance_diag_m2[1], 6
            ),
            "tracking_cov_forward_m2": self._fmt_record_float(
                None if tracking_result is None else tracking_result.covariance_diag_m2[2], 6
            ),
            "tracking_velocity_right_mps": self._fmt_record_float(
                None if tracking_result is None else tracking_result.velocity_mps[0], 6
            ),
            "tracking_velocity_down_mps": self._fmt_record_float(
                None if tracking_result is None else tracking_result.velocity_mps[1], 6
            ),
            "tracking_velocity_forward_mps": self._fmt_record_float(
                None if tracking_result is None else tracking_result.velocity_mps[2], 6
            ),
            "corrected_left_x_m": self._fmt_record_float(
                None if corrected_point_left_m is None else corrected_point_left_m[0], 6
            ),
            "corrected_left_y_m": self._fmt_record_float(
                None if corrected_point_left_m is None else corrected_point_left_m[1], 6
            ),
            "corrected_left_z_m": self._fmt_record_float(
                None if corrected_point_left_m is None else corrected_point_left_m[2], 6
            ),
            "drone_x_m": self._fmt_record_float(None if drone_position_m is None else drone_position_m[0], 6),
            "drone_y_m": self._fmt_record_float(None if drone_position_m is None else drone_position_m[1], 6),
            "drone_z_m": self._fmt_record_float(None if drone_position_m is None else drone_position_m[2], 6),
            "drone_north_m": self._fmt_record_float(None if drone_position_m is None else drone_position_m[0], 6),
            "drone_east_m": self._fmt_record_float(None if drone_position_m is None else drone_position_m[1], 6),
            "drone_down_m": self._fmt_record_float(None if drone_position_m is None else drone_position_m[2], 6),
            "error_x_m": self._fmt_record_float(None if error_m is None else error_m[0], 6),
            "error_y_m": self._fmt_record_float(None if error_m is None else error_m[1], 6),
            "error_z_m": self._fmt_record_float(None if error_m is None else error_m[2], 6),
            "error_frame": "mixed-stereo-center-vs-px4-local-ned",
            "error_basis_warning": error_basis_warning if error_m is not None else "",
            "bias_x_m": self._fmt_record_float(None if bias_correction_m is None else bias_correction_m[0], 6),
            "bias_y_m": self._fmt_record_float(None if bias_correction_m is None else bias_correction_m[1], 6),
            "bias_z_m": self._fmt_record_float(None if bias_correction_m is None else bias_correction_m[2], 6),
            "bias_profile_id": ""
            if bias_correction_m is None or self.active_bias_profile is None
            else self.active_bias_profile.profile_id,
            "profile_id": profile_status.get("profile_id", ""),
            "profile_mode": profile_status.get("profile_mode", ""),
            "profile_output_policy": profile_status.get("profile_output_policy", ""),
            "profile_fallback_level": profile_status.get("profile_fallback_level", ""),
            "profile_fallback_reason": profile_status.get("profile_fallback_reason", ""),
            "recording_elapsed_s": self._fmt_record_float(recording_elapsed_s, 6),
            "warnings": "; ".join(estimate.warnings),
        }
        row.update(self._record_bbox_correction_parameters(bbox_correction, estimate, raw_distance_m))
        row.update(self._record_bbox_parameters(estimate))
        row.update(self._record_bias_sync_timestamp_parameters(estimate))
        row.update(self._record_intrinsic_parameters())
        row.update(self._record_frame_timestamp_parameters())
        row.update(self._record_external_parameters(estimate=estimate))
        self.distance_recording_rows.append(row)
        self.distance_recording_keys.append(key)
        self._append_recording_table_row(row)
        self._refresh_recording_status_label()
        self._refresh_operation_header()
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

    def _record_external_timeline_row(self, camera: str, sample_type: str, values: dict[str, object]) -> None:
        if not self.distance_recording_enabled:
            return
        now_s = time.monotonic()
        row: dict[str, object] = {
            "session_id": self.recording_session_id,
            "session_started_unix_s": self._fmt_record_float(self.recording_session_started_wall_s, 6),
            "recording_elapsed_s": self._fmt_record_float(self._recording_elapsed_s(now_s), 6),
            "recorded_app_timestamp_s": self._fmt_record_float(now_s, 6),
            "camera": camera,
            "sample_type": sample_type,
        }
        row.update(values)
        self.external_parameter_recording_rows.append(row)
        self._refresh_recording_status_label()

    def _record_bbox_correction_parameters(
        self,
        bbox_correction: dict[str, object] | None,
        estimate,
        raw_range_m: float | None,
    ) -> dict[str, str]:
        row: dict[str, str] = {
            "left_corrected_cx": "",
            "left_corrected_cy": "",
            "right_corrected_cx": "",
            "right_corrected_cy": "",
            "left_bbox_du_px": "",
            "left_bbox_dv_px": "",
            "right_bbox_du_px": "",
            "right_bbox_dv_px": "",
            "profile_id_hash": "",
            "fallback_level": "",
            "fallback_reason": "",
            "profile_cell_key": "",
            "profile_cell_sample_count": "",
            "bbox_adjustment_raw_disparity_px": self._fmt_record_float(getattr(estimate, "disparity_px", None), 6),
            "bbox_adjustment_raw_range_m": self._fmt_record_float(raw_range_m, 6),
        }
        if bbox_correction is None:
            return row
        float_fields = (
            "left_corrected_cx",
            "left_corrected_cy",
            "right_corrected_cx",
            "right_corrected_cy",
            "left_bbox_du_px",
            "left_bbox_dv_px",
            "right_bbox_du_px",
            "right_bbox_dv_px",
        )
        for field in float_fields:
            row[field] = self._fmt_record_float(
                None if bbox_correction.get(field) is None else float(bbox_correction[field]), 6
            )
        row["profile_id_hash"] = str(bbox_correction.get("profile_id_hash", ""))
        row["fallback_level"] = str(bbox_correction.get("fallback_level", ""))
        row["fallback_reason"] = str(bbox_correction.get("fallback_reason", ""))
        row["profile_cell_key"] = str(bbox_correction.get("profile_cell_key", ""))
        row["profile_cell_sample_count"] = str(bbox_correction.get("profile_cell_sample_count", ""))
        return row

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

    def _record_bias_sync_timestamp_parameters(self, estimate) -> dict[str, str]:
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
        drone_timestamp_s = self.latest_drone_position_timestamp_s
        drone_age_s = (
            bbox_mid_timestamp_s - drone_timestamp_s
            if drone_timestamp_s is not None
            else None
        )
        truth_sample = self.latest_drone_truth_sample
        row = {
            "left_bbox_timestamp_s": self._fmt_record_float(left_timestamp_s, 6),
            "right_bbox_timestamp_s": self._fmt_record_float(right_timestamp_s, 6),
            "bbox_mid_timestamp_s": self._fmt_record_float(bbox_mid_timestamp_s, 6),
            "left_bbox_utc_unix_s": self._fmt_record_float(left_utc_unix_s, 6),
            "right_bbox_utc_unix_s": self._fmt_record_float(right_utc_unix_s, 6),
            "bbox_mid_utc_unix_s": self._fmt_record_float(bbox_mid_utc_unix_s, 6),
            "left_bbox_timestamp_reference": str(
                (getattr(estimate.left, "timestamp_metadata", None) or {}).get("timestamp_reference") or "app-receive"
            ),
            "right_bbox_timestamp_reference": str(
                (getattr(estimate.right, "timestamp_metadata", None) or {}).get("timestamp_reference") or "app-receive"
            ),
            "drone_timestamp_s": self._fmt_record_float(drone_timestamp_s, 6),
            "drone_timestamp_reference": self.latest_drone_position_timestamp_reference,
            "drone_utc_unix_s": self._fmt_record_float(self.latest_drone_position_utc_unix_s, 6),
            "drone_age_s": self._fmt_record_float(drone_age_s, 6),
            "drone_source_frame": "" if truth_sample is None else truth_sample.source_frame,
            "drone_source_axis_contract": "" if truth_sample is None else truth_sample.source_axis_contract,
            "drone_conversion_status": "" if truth_sample is None else truth_sample.conversion_status,
            "drone_raw_lat_deg": self._fmt_record_float(None if truth_sample is None else truth_sample.raw_lat_deg, 9),
            "drone_raw_lon_deg": self._fmt_record_float(None if truth_sample is None else truth_sample.raw_lon_deg, 9),
            "drone_raw_alt_m": self._fmt_record_float(None if truth_sample is None else truth_sample.raw_alt_m, 6),
            "drone_gps_reference_lat_deg": self._fmt_record_float(
                None if truth_sample is None else truth_sample.gps_reference_lat_deg, 9
            ),
            "drone_gps_reference_lon_deg": self._fmt_record_float(
                None if truth_sample is None else truth_sample.gps_reference_lon_deg, 9
            ),
            "drone_gps_reference_alt_m": self._fmt_record_float(
                None if truth_sample is None else truth_sample.gps_reference_alt_m, 6
            ),
            "drone_gps_reference_kind": "" if truth_sample is None else truth_sample.gps_reference_kind,
        }
        return row

    def _record_frame_timestamp_parameters(self) -> dict[str, str]:
        row: dict[str, str] = {}
        for camera in ("left", "right"):
            timestamps = self.camera_frame_times[camera]
            metadata = self.camera_frame_metadata.get(camera) or {}
            timestamp_s = float(timestamps[-1]) if timestamps else None
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
        if self.camera_frame_times["left"] and self.camera_frame_times["right"]:
            display_delta_s = float(self.camera_frame_times["right"][-1]) - float(self.camera_frame_times["left"][-1])
        else:
            display_delta_s = None
        left_source = (self.camera_frame_metadata.get("left") or {}).get("source_timestamp_s")
        right_source = (self.camera_frame_metadata.get("right") or {}).get("source_timestamp_s")
        source_delta_s = (
            float(right_source) - float(left_source)
            if left_source is not None and right_source is not None
            else None
        )
        left_rtcp_ntp = (self.camera_frame_metadata.get("left") or {}).get("rtcp_frame_ntp_unix_s")
        right_rtcp_ntp = (self.camera_frame_metadata.get("right") or {}).get("rtcp_frame_ntp_unix_s")
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

    def _record_intrinsic_parameters(self) -> dict[str, str]:
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

    def _record_external_parameters(self, *, estimate) -> dict[str, str]:
        row: dict[str, str] = {
            "use_rtk_baseline": self._fmt_record_bool(self.use_rtk_baseline_checkbox.isChecked()),
            "require_rtk_fixed": self._fmt_record_bool(self.require_rtk_fixed_checkbox.isChecked()),
            "use_triangulation": self._fmt_record_bool(True),
            "distance_mode": self._distance_mode_label(),
            "world_frame": WORLD_FRAME,
            "body_frame": BODY_FRAME,
            "camera_frame": CAMERA_FRAME,
            "camera_mount_profile": CAMERA_MOUNT_PROFILE,
            "mount_assumption_model": "gps-rtk-baseline-only-v1",
            "mount_assumption_status": self._mount_assumption_status_label(),
            "mount_assumption_detail": "live_gps_rtk_baseline_used_as_stereo_translation",
            "require_attitude": self._fmt_record_bool(False),
            "left_attitude_age_s": self._fmt_record_float(estimate.left_attitude_age_s, 6),
            "right_attitude_age_s": self._fmt_record_float(estimate.right_attitude_age_s, 6),
            "attitude_pair_delta_s": self._fmt_record_float(estimate.attitude_pair_delta_s, 6),
            "relative_roll_deg": self._fmt_record_float(estimate.relative_roll_deg, 6),
            "relative_pitch_deg": self._fmt_record_float(estimate.relative_pitch_deg, 6),
            "relative_yaw_deg": self._fmt_record_float(estimate.relative_yaw_deg, 6),
        }
        baseline = self.latest_rtk_baseline
        if baseline is not None:
            row.update(
                {
                    "rtk_baseline_north_m": self._fmt_record_float(baseline.north_m, 6),
                    "rtk_baseline_east_m": self._fmt_record_float(baseline.east_m, 6),
                    "rtk_baseline_down_m": self._fmt_record_float(baseline.down_m, 6),
                    "rtk_left_age_s": self._fmt_record_float(baseline.left_age_s, 6),
                    "rtk_right_age_s": self._fmt_record_float(baseline.right_age_s, 6),
                    "rtk_fix_quality": baseline.fix_quality,
                    "rtk_warnings": "; ".join(baseline.warnings),
                }
            )
        for camera in ("left", "right"):
            row.update(self._record_extrinsic_state(camera))
            row.update(self._record_gps_state(camera))
            row.update(self._record_gnss_relative_state(camera))
        return row

    def _record_extrinsic_state(self, camera: str) -> dict[str, str]:
        state = self.latest_extrinsics.get(camera)
        prefix = f"{camera}_pixhawk"
        if state is None:
            return {}
        age_s = max(0.0, time.monotonic() - state.timestamp_s)
        return {
            f"{prefix}_namespace": state.namespace,
            f"{prefix}_topic": state.topic,
            f"{prefix}_roll_deg": self._fmt_record_float(state.roll_deg, 6),
            f"{prefix}_pitch_deg": self._fmt_record_float(state.pitch_deg, 6),
            f"{prefix}_yaw_deg": self._fmt_record_float(state.yaw_deg, 6),
            f"{prefix}_q": json.dumps(list(state.q), separators=(",", ":")),
            f"{prefix}_age_s": self._fmt_record_float(age_s, 6),
            f"{prefix}_px4_timestamp_s": self._fmt_record_float(state.px4_timestamp_s, 6),
        }

    def _record_gps_state(self, camera: str) -> dict[str, str]:
        state = self.latest_gps.get(camera)
        prefix = f"{camera}_gps"
        if state is None:
            return {}
        age_s = max(0.0, time.monotonic() - state.timestamp_s)
        return {
            f"{prefix}_namespace": state.namespace,
            f"{prefix}_topic": state.topic,
            f"{prefix}_lat_deg": self._fmt_record_float(state.latitude_deg, 9),
            f"{prefix}_lon_deg": self._fmt_record_float(state.longitude_deg, 9),
            f"{prefix}_alt_m": self._fmt_record_float(state.altitude_m, 6),
            f"{prefix}_fix_type": "" if state.fix_type is None else str(state.fix_type),
            f"{prefix}_satellites": "" if state.satellites_used is None else str(state.satellites_used),
            f"{prefix}_eph_m": self._fmt_record_float(state.eph_m, 6),
            f"{prefix}_epv_m": self._fmt_record_float(state.epv_m, 6),
            f"{prefix}_heading_deg": self._fmt_record_float(state.heading_deg, 6),
            f"{prefix}_heading_accuracy_deg": self._fmt_record_float(state.heading_accuracy_deg, 6),
            f"{prefix}_rtk_label": state.rtk_label,
            f"{prefix}_age_s": self._fmt_record_float(age_s, 6),
            f"{prefix}_px4_timestamp_s": self._fmt_record_float(state.px4_timestamp_s, 6),
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

    def _clear_triangulation_quality(self, reason: str) -> None:
        if not hasattr(self, "triangulation_quality_status_label"):
            return
        self.triangulation_quality_status_label.setText(reason)
        self.triangulation_quality_gap_label.setText("gap: --")
        self.triangulation_quality_distance_label.setText("distance: --")
        self.triangulation_quality_baseline_label.setText("baseline: --")
        self.triangulation_quality_mode_label.setText("mode: --")
        self.triangulation_quality_raw_point_label.setText("Raw center 3D point: --")
        self.triangulation_quality_corrected_point_label.setText("Corrected center 3D point: --")
        self.triangulation_quality_correction_label.setText("Correction vector: --")
        self.triangulation_quality_plot.set_quality(None)

    def _update_triangulation_quality(
        self,
        *,
        triangulated: RayTriangulationResult,
        method_text: str,
        display_distance_m: float,
        display_baseline_source: str,
        display_baseline_m: float,
        raw_point_left_m: tuple[float, float, float] | None,
        raw_point_center_m: tuple[float, float, float] | None,
        corrected_point_center_m: tuple[float, float, float] | None,
        bias_correction: tuple[float, float, float] | None,
    ) -> None:
        if not hasattr(self, "triangulation_quality_status_label"):
            return
        self.triangulation_quality_status_label.setText(
            "표시 전용: 현재 좌/우 bbox ray의 최단거리 gap과 3D midpoint를 보여줍니다."
        )
        self.triangulation_quality_gap_label.setText(f"gap: {triangulated.closest_gap_m:.3f} m")
        self.triangulation_quality_distance_label.setText(f"distance: {display_distance_m:.3f} m")
        self.triangulation_quality_baseline_label.setText(
            f"baseline: {display_baseline_source}:{display_baseline_m:.3f} m"
        )
        self.triangulation_quality_mode_label.setText(f"mode: {method_text}")
        self.triangulation_quality_raw_point_label.setText(
            f"Raw center 3D point: {self._format_quality_point(raw_point_center_m)} "
            f"| left raw: {self._format_quality_point(raw_point_left_m)}"
        )
        self.triangulation_quality_corrected_point_label.setText(
            f"Corrected center 3D point: {self._format_quality_point(corrected_point_center_m)}"
        )
        self.triangulation_quality_correction_label.setText(
            f"Correction vector: {self._format_quality_point(bias_correction)}"
        )
        self.triangulation_quality_plot.set_quality(
            {
                "closest_gap_m": triangulated.closest_gap_m,
                "distance_m": display_distance_m,
                "baseline_m": triangulated.baseline_m,
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
                "sample_app_timestamp_s": self._fmt_record_float(extrinsic.timestamp_s, 6),
                "sample_px4_timestamp_s": self._fmt_record_float(extrinsic.px4_timestamp_s, 6),
                "roll_deg": self._fmt_record_float(extrinsic.roll_deg, 6),
                "pitch_deg": self._fmt_record_float(extrinsic.pitch_deg, 6),
                "yaw_deg": self._fmt_record_float(extrinsic.yaw_deg, 6),
                "q": json.dumps(list(extrinsic.q), separators=(",", ":")),
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
        self._refresh_alignment_validation()
        self._refresh_camera_parameters_tab()
        self._refresh_distance()


    def _update_gps(self, camera: str, gps: GpsPositionState) -> None:
        self.latest_gps[camera] = gps
        if camera in {"left", "right"}:
            self.gnss_monitor.update_position(camera, gps)
        if hasattr(self, "baseline_gps_rate_history") and camera in self.baseline_gps_rate_history:
            self.baseline_gps_rate_history[camera].append(time.monotonic())
        uam_source = str(self.uam_source_combo.currentData() or "") if hasattr(self, "uam_source_combo") else ""
        if uam_source not in {"mavlink_gps", "mavlink_global"}:
            self._update_uam_live_telemetry_from_gps(camera, gps)
        self._record_external_timeline_row(
            camera,
            "gps_rtk",
            {
                "namespace": gps.namespace,
                "topic": gps.topic,
                "sample_app_timestamp_s": self._fmt_record_float(gps.timestamp_s, 6),
                "sample_px4_timestamp_s": self._fmt_record_float(gps.px4_timestamp_s, 6),
                "lat_deg": self._fmt_record_float(gps.latitude_deg, 9),
                "lon_deg": self._fmt_record_float(gps.longitude_deg, 9),
                "alt_m": self._fmt_record_float(gps.altitude_m, 6),
                "fix_type": "" if gps.fix_type is None else str(gps.fix_type),
                "satellites": "" if gps.satellites_used is None else str(gps.satellites_used),
                "eph_m": self._fmt_record_float(gps.eph_m, 6),
                "epv_m": self._fmt_record_float(gps.epv_m, 6),
                "heading_deg": self._fmt_record_float(gps.heading_deg, 6),
                "heading_accuracy_deg": self._fmt_record_float(gps.heading_accuracy_deg, 6),
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
        self._append_baseline_sample_if_active()
        self._refresh_camera_parameters_tab()
        self._refresh_distance()

    def _update_uam_live_telemetry_from_gps(self, camera: str, gps: GpsPositionState) -> None:
        if not hasattr(self, "uam_source_combo"):
            return
        source = str(self.uam_source_combo.currentData() or "")
        if source not in {"mavlink_gps", "mavlink_global"}:
            return
        if camera not in {"left", "uam"} or not gps.has_position:
            return
        topic = str(gps.topic or "")
        if source == "mavlink_global" and "GLOBAL_POSITION_INT" not in topic:
            return
        if source == "mavlink_gps" and not any(
            name in topic for name in ("GPS_RAW_INT", "GPS2_RAW", "VehicleGpsPosition", "SensorGps", "sensor_gps")
        ):
            return
        if self.uam_gps_reference_lla is None:
            self.uam_gps_reference_lla = (float(gps.latitude_deg), float(gps.longitude_deg), float(gps.altitude_m))
            self.uam_gps_reference_kind = "first_valid_global_sample" if source == "mavlink_global" else "first_valid_gps_sample"
        ref_lat, ref_lon, ref_alt = self.uam_gps_reference_lla
        converted = local_ned_from_lla(
            lat=gps.latitude_deg,
            lon=gps.longitude_deg,
            alt=gps.altitude_m,
            reference_lat_deg=ref_lat,
            reference_lon_deg=ref_lon,
            reference_alt_m=ref_alt,
            source=f"live-{source}:{camera}:{topic}",
            origin_kind=self.uam_gps_reference_kind,
            altitude_reference="unknown",
        )
        if converted is None:
            return
        self.latest_drone_position_m = (converted.north_m, converted.east_m, converted.down_m)
        self.latest_drone_position_timestamp_s = float(gps.timestamp_s)
        self.latest_drone_position_timestamp_reference = converted.source
        self.latest_drone_position_utc_unix_s = None
        signature = (
            round(float(self.latest_drone_position_timestamp_s), 6),
            round(float(converted.north_m), 6),
            round(float(converted.east_m), 6),
            round(float(converted.down_m), 6),
        )
        if signature != self._last_uam_live_sample_signature:
            self._last_uam_live_sample_signature = signature
            uam_sample = UamTelemetrySample(
                timestamp_s=float(gps.timestamp_s),
                x_m=converted.north_m,
                y_m=converted.east_m,
                z_m=converted.down_m,
                source=converted.source,
                timestamp_reference="app-receive",
                gps_reference_lat_deg=converted.origin_lat_deg,
                gps_reference_lon_deg=converted.origin_lon_deg,
                gps_reference_alt_m=converted.origin_alt_m,
                gps_reference_kind=converted.origin_kind,
                gps_altitude_reference=converted.altitude_reference,
                source_frame=converted.source_frame,
                source_axis_contract=converted.source_axis_contract,
                raw_lat_deg=converted.raw_lat_deg,
                raw_lon_deg=converted.raw_lon_deg,
                raw_alt_m=converted.raw_alt_m,
                conversion_status=converted.conversion_status,
            )
            self.latest_drone_truth_sample = uam_sample
            self.uam_live_telemetry_samples.append(uam_sample)
            self._append_uam_truth_plot_sample(uam_sample)
        latest_text = (
            f"N={converted.north_m:+.3f} E={converted.east_m:+.3f} D={converted.down_m:+.3f} m "
            f"origin={converted.origin_kind} source={converted.source}"
        )
        if hasattr(self, "uam_latest_label"):
            self.uam_latest_label.setText(f"latest: {latest_text}")
        if hasattr(self, "uam_status_label"):
            self.uam_status_label.setText(
                f"UAM telemetry: {source} 변환 수신 중, buffer={len(self.uam_live_telemetry_samples)}"
            )

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
                "sample_app_timestamp_s": self._fmt_record_float(state.timestamp_s, 6),
                "sample_px4_timestamp_s": self._fmt_record_float(state.px4_timestamp_s, 6),
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
        if not hasattr(self, "attitude_compare_labels"):
            return
        left = self.latest_extrinsics.get("left")
        right = self.latest_extrinsics.get("right")
        axis_values = (
            ("roll", None if left is None else left.roll_deg, None if right is None else right.roll_deg),
            ("pitch", None if left is None else left.pitch_deg, None if right is None else right.pitch_deg),
            ("yaw", None if left is None else left.yaw_deg, None if right is None else right.yaw_deg),
        )
        for axis, left_value, right_value in axis_values:
            left_label, right_label, delta_label = self.attitude_compare_labels[axis]
            left_label.setText("대기 중" if left_value is None else f"{left_value:.4f} deg")
            right_label.setText("대기 중" if right_value is None else f"{right_value:.4f} deg")
            if left_value is None or right_value is None:
                self._set_health_label(delta_label, "대기 중", "idle")
            else:
                delta = right_value - left_value
                self._set_health_label(
                    delta_label,
                    f"Δ{axis.capitalize()} {delta:+.2f}°",
                    self._attitude_delta_level(axis, delta),
                )
        if left is None and right is None:
            self._refresh_alignment_workflow_status()
            return
        now_s = time.monotonic()
        for axis, left_value, right_value in axis_values:
            point = AttitudeHistoryPoint(
                timestamp_s=now_s,
                left_deg=left_value,
                right_deg=right_value,
            )
            self.attitude_history[axis].append(point)
            self.attitude_plots[axis].add_point(point)
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
        if self.auto_start_pixhawk:
            QtCore.QTimer.singleShot(500, self._auto_start_pixhawk)
        if self.auto_start:
            self.start_yolo()

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
        self._update_runtime_stream_settings_label()

    def _runtime_decoder_text(self) -> str:
        active = {
            camera: decoder
            for camera, decoder in self.active_stream_decoders.items()
            if decoder
        }
        if not active:
            return self.decoder_combo.currentText().strip() or "--"
        unique_decoders = list(dict.fromkeys(active.values()))
        if len(unique_decoders) == 1:
            return unique_decoders[0]
        return " / ".join(f"{self._camera_name_ko(camera)}={decoder}" for camera, decoder in active.items())

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
