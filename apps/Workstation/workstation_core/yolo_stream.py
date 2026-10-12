"""Explicit compatibility exports for the decomposed media stream owners.

New implementation code imports media_stream owners directly. This module
holds no second worker, recorder, lock, queue or timing state.
"""

from __future__ import annotations
import ctypes
import os
import multiprocessing
import math
import queue
import signal
import subprocess
import sys
import threading
import time
from collections import deque
from collections.abc import Mapping
from dataclasses import dataclass, field
from pathlib import Path
from statistics import median
from typing import Any, Callable
from PyQt5 import QtCore, QtGui
from Workstation.runtime_paths import WORKSTATION_MODELS_ROOT
from Workstation.workstation_core.error_throttle import should_emit_throttled_error
from Workstation.workstation_core.mp4_integrity import H264_MP4_INTEGRITY_SCHEMA, validate_h264_copy_mp4_integrity
from Workstation.workstation_core.native_frame_ring import (
    DEFAULT_NATIVE_MAX_HEIGHT,
    DEFAULT_NATIVE_MAX_WIDTH,
    DEFAULT_NATIVE_RING_CAPACITY,
    SharedNativeFrameRing,
)
from Workstation.workstation_core.runtime_camera_status import is_sr_pc_diagnostic_reason
from Workstation.workstation_core.settings import (
    RTSP_ENCODED_QUEUE_MODE_LATEST,
    RTSP_ENCODED_QUEUE_MODE_REFERENCE_SAFE,
    normalize_rtsp_encoded_queue_mode,
)
from .media_stream.child_runtime import _arm_linux_parent_death_signal, _run_yolo_process
from .media_stream.configuration import (
    _record_video_path,
    _record_video_fps,
    _record_video_queue_size,
    normalize_stream_backend,
    recording_integrity_contract,
    normalize_rtsp_transport,
    normalize_rtsp_reconnect_policy,
    _device_uses_cpu,
    default_yolo_max_rate_hz,
    effective_yolo_max_rate_hz,
    normalize_yolo_input_size,
    _resolve_frame_timeout_s,
)
from .media_stream.decoder_policy import (
    normalize_decoder_mode,
    decoder_candidates,
    classify_gstreamer_error,
    gstreamer_error_fallback_reason,
    _gstreamer_message_source_name,
    _requested_decoder_mode,
    runtime_decoder_fallback_reason,
    nvidia_decoder_health_failure_reason,
    should_suppress_decoded_output,
    decoder_element_text,
)
from .media_stream.detections import (
    DetectionBox,
    _resolve_class_name,
    highest_confidence_detection,
    filter_detections_by_class_gate,
    detections_from_ultralytics_result,
)
from .media_stream.events import _emit_status, _emit_terminal_exit, _emit_decoder_fallback, _emit_recording_finalized
from .media_stream.frame_output import (
    app_receive_monotonic_sample,
    _should_emit_preview_frame,
    _publish_native_frame_to_ring,
    _emit_preview_frame,
    _emit_measurement_frame,
    _emit_stream_status,
)
from .media_stream.gstreamer_backend import (
    _gst_location,
    build_gst_pipeline_text,
    _launch_gst_pipeline,
    _run_gstreamer_stream_once,
    _run_gstreamer_stream,
)
from .media_stream.inference import _load_yolo_model, _process_frame_for_output, _yolo_device_stats
from .media_stream.ipc import (
    CameraIpcCounters,
    _BoundedProcessOutputQueue,
    _queue_has_pending_messages,
    _get_queue_message,
    _get_queue_message_nowait,
)
from .media_stream.opencv_backend import _run_opencv_stream
from .media_stream.policy import (
    SASL_ROOT,
    DEFAULT_YOLO_PYTHON,
    DEFAULT_YOLO_WEIGHTS,
    DEFAULT_RTSP_LATENCY_MS,
    DEFAULT_PREVIEW_DISPLAY_RATE_HZ,
    DEFAULT_CPU_YOLO_MAX_RATE_HZ,
    DEFAULT_CUDA_YOLO_MAX_RATE_HZ,
    DEFAULT_CPU_YOLO_INITIAL_RATE_HZ,
    DEFAULT_CPU_YOLO_ADAPTIVE_MAX_RATE_HZ,
    DEFAULT_CPU_YOLO_MIN_RATE_HZ,
    DEFAULT_DECODE_RATE_WINDOW_S,
    CPU_DECODE_HZ_FLOOR,
    CPU_DECODE_HZ_CRITICAL_GUARD,
    CPU_DECODE_HZ_SOFT_FLOOR,
    CPU_DECODE_HZ_GUARD,
    CPU_DECODE_HZ_RECOVERY,
    CPU_DECODE_HZ_STRONG_RECOVERY,
    DEFAULT_CPU_WORKER_THREADS,
    DEFAULT_CUDA_WORKER_THREADS,
    DEFAULT_EVENT_QUEUE_SIZE,
    DEFAULT_FRAME_QUEUE_SIZE,
    DEFAULT_METADATA_QUEUE_SIZE,
    DEFAULT_RTSP_FRAME_TIMEOUT_S,
    YOLO_INPUT_SIZE_STRIDE,
    DECODER_AUTO,
    DECODER_NVIDIA,
    DECODER_SOFTWARE,
    STREAM_BACKEND_GSTREAMER,
    STREAM_BACKEND_OPENCV,
    STREAM_BACKEND_OPENCV_AUTO,
    STREAM_BACKEND_PYAV,
    RECORDING_ENCODING_H264_COPY_GATED,
    RECORDING_ENCODING_DECODED_REENCODE_UNVERIFIED,
    RECORDING_ENCODING_DECODED_NVENC_LOSSLESS,
    RTSP_TRANSPORT_UDP,
    RTSP_TRANSPORT_TCP,
    RTSP_RECONNECT_MANUAL,
    RTSP_RECONNECT_AUTOMATIC,
    NVIDIA_H264_DECODER,
    SOFTWARE_H264_DECODER,
    RTP_CLOCK_RATE_HZ,
    NTP_UNIX_EPOCH_OFFSET_S,
    UINT32_MODULUS,
    UINT32_HALF_RANGE,
    MAX_RTP_TIMING_CACHE,
    MAX_RTP_PTS_MATCH_DELTA_NS,
    MAX_RTP_DISCONTINUITY_CACHE,
    H264_NAL_TYPE_IDR,
    H264_NAL_TYPE_NON_IDR_SLICE,
    H264_NAL_TYPE_SPS,
    H264_NAL_TYPE_PPS,
    H264_RECOVERY_BASIS_IDR,
    H264_RECOVERY_BASIS_NON_IDR_INTRA_EXPERIMENTAL,
    H264_ACCESS_UNIT_KIND_NAL1_NON_IDR_INTRA,
    MAX_RTCP_SR_ARRIVAL_CACHE,
    MAX_RTCP_RATE_SAMPLES,
    DEFAULT_RECORD_VIDEO_QUEUE_SIZE,
    GST_RTSP_RECONNECT_DELAYS_S,
    GST_ENCODED_QUEUE_MAX_TIME_NS,
    GST_DECODED_QUEUE_MAX_BUFFERS,
    GST_RECORDING_QUEUE_MAX_TIME_NS,
    GST_RECORDING_EOS_TIMEOUT_S,
    INTEGRITY_RECOVERY_TIMEOUT_S,
    YOLO_COOPERATIVE_STOP_TIMEOUT_S,
    YOLO_KILL_AFTER_STOP_S,
    YOLO_KILL_JOIN_TIMEOUT_S,
    YOLO_PROCESS_CRASH_RESTART_DELAYS_S,
    YOLO_PROCESS_STABLE_RUN_S,
    _LINUX_PR_SET_PDEATHSIG,
    DECODER_FALLBACK_FIRST_FRAME_TIMEOUT_S,
    DECODER_FALLBACK_STALL_TIMEOUT_S,
    DECODER_FALLBACK_STATUS_PREFIX,
    DECODER_FALLBACK_ACTIVE_STATUS_PREFIX,
    GST_ERROR_RTSP_SOURCE,
    GST_ERROR_RECORDING_OUTPUT,
    GST_ERROR_DECODER_HEALTH,
    GST_ERROR_PIPELINE,
)
from .media_stream.pyav_backend import _run_pyav_stream
from .media_stream.rates import (
    RollingFrameRateEstimator,
    RollingCounterRateEstimator,
    AdaptiveYoloRateLimiter,
    _activate_cpu_fallback_rate_limiter,
)
from .media_stream.recorders import (
    AsyncVideoWriter,
    ffmpeg_overwrite_flag,
    DecodedNv12Recorder,
    _open_record_video_writer,
    _write_record_video_frame,
    _release_record_video_writer,
)
from .media_stream.rtp_timing import (
    format_rtp_sequence_discontinuity,
    h264_byte_stream_nal_unit_types,
    h264_access_unit_has_idr,
    h264_nal1_non_idr_intra_metadata,
    h264_access_unit_recovery_basis,
    ntp64_to_unix_seconds,
    rtp_timestamp_delta,
    estimate_rtcp_ntp_unix_seconds,
    _gst_structure_value,
    RtpFrameTiming,
    RtpSequenceDiscontinuity,
    RtpSequenceRecovery,
    H264AccessUnitDiscontinuity,
    RtpDiscontinuityReporter,
    RtcpSenderReportTiming,
    rtcp_sender_report_pc_offset_s,
    sender_report_clock_rates,
    robust_sender_report_clock_rates,
    GstRtpTimingTracker,
)
from .media_stream.supervisor import YoloStreamWorker
from .media_stream.worker_threads import (
    _worker_cpu_threads,
    _default_worker_cpu_threads,
    _configure_worker_thread_environment,
    _configure_cv2_threads,
    _configure_torch_threads,
)
