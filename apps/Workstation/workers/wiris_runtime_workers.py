"""Explicit compatibility exports for Workstation WIRIS worker owners.

Worker state and I/O live in wiris_workers modules. New worker code must not
import this facade. Legacy class names identify exactly the same QObject types.
"""
from __future__ import annotations

import ftplib
import hashlib
import math
import os
import posixpath
import re
import tempfile
import threading
import time
from collections.abc import Mapping, Sequence
from pathlib import Path, PurePosixPath
from statistics import median
from PyQt5 import QtCore, QtGui
from Workstation.workstation_core.network import DEFAULT_HOST_ADDRESS, apply_interface_address, check_tcp_port, discover_host_interface_candidates, interface_has_address, interface_link_exists, resolve_nm_interface_address, rtsp_host_port
from Workstation.workstation_core.runtime_camera_status import camera_display_name
from Workstation.workstation_core.wiris_zoom import GET_ETHERNET_MODE_COMMAND, LINE_TERMINATOR, REBOOT_COMMAND, SET_ETHERNET_MODE_COMMAND, SET_IP_ADDRESS_TEMPLATE, SET_SUBNET_MASK_TEMPLATE, build_wiris_command, build_zoom_command, send_tcp_line, WirisControlBusyError
from Workstation.workstation_core.yolo_stream import DECODER_AUTO, STREAM_BACKEND_OPENCV, STREAM_BACKEND_OPENCV_AUTO, STREAM_BACKEND_PYAV, build_gst_pipeline_text, decoder_candidates, normalize_decoder_mode, normalize_rtsp_transport, normalize_stream_backend

from Workstation.wiris_workers.policy import (
    RTSP_ENABLE_SETTLE_DELAY_MS,
    RTSP_ENABLE_PORT_WAIT_INTERVAL_MS,
    RTSP_ENABLE_PORT_WAIT_MAX_ATTEMPTS,
    ENCODER_PROFILE_COMMAND_SETTLE_MS,
    ENCODER_PROFILE_READBACK_RETRY_INTERVAL_MS,
    ENCODER_PROFILE_READBACK_MAX_ATTEMPTS,
    WIRIS_STUS_RTT_PROBE_COMMAND,
    WIRIS_STUS_RTT_PROBE_COUNT,
    WIRIS_STUS_MIN_VALID_RTT_PROBES,
    WIRIS_STUS_HIGH_RTT_WARNING_S,
    WIRIS_STUS_MAX_VALID_RTT_S,
    WIRIS_STUS_PARSE_MARGIN_S,
    WIRIS_STUS_MAX_ABSOLUTE_COMPENSATION_S,
    WIRIS_STUS_MAX_ATTEMPTS,
    WIRIS_STUS_MAX_OPERATION_S,
    WIRIS_STUS_RESPONSE_TIMEOUT_S,
    WIRIS_LRF_COMMAND,
    _LRF_NUMBER_RE,
    _wait_for_peer_or_continue,
    _abort_barrier,
    parse_lrf_distance,
)
from Workstation.wiris_workers.clock import WirisClockCommandWorker
from Workstation.wiris_workers.network import WirisStipWorker, WirisRtspEnableWorker
from Workstation.wiris_workers.capture import WirisPnPPhotoCaptureWorker, WirisOZPhotoCaptureWorker
from Workstation.wiris_workers.ftp import WirisOzFtpSyncWorker, WirisFtpBatchDeleteWorker
from Workstation.wiris_workers.commands import WirisCommandWorker, WirisImageFormatWorker, WirisLrfPollWorker, WirisZoomWorker
from Workstation.wiris_workers.probe import RtspStreamCheckWorker
