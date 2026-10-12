"""child runtime owner extracted from the Workstation media runtime.

Existing queues, locks, clocks, source-frame identity and policies are preserved.
Native pixels use image u/v and RGB/BGR/NV12; no world/body transform is introduced.
"""

from __future__ import annotations

import ctypes
import os
import signal
import sys
from typing import Any
from .configuration import normalize_stream_backend
from .events import _emit_status
from .gstreamer_backend import _run_gstreamer_stream
from .inference import _load_yolo_model
from .opencv_backend import _run_opencv_stream
from .policy import (
    STREAM_BACKEND_GSTREAMER,
    STREAM_BACKEND_OPENCV,
    STREAM_BACKEND_OPENCV_AUTO,
    STREAM_BACKEND_PYAV,
    _LINUX_PR_SET_PDEATHSIG,
)
from .pyav_backend import _run_pyav_stream
from .rates import AdaptiveYoloRateLimiter
from .worker_threads import _configure_cv2_threads, _configure_worker_thread_environment, _worker_cpu_threads


def _arm_linux_parent_death_signal() -> bool:
    """Terminate a camera child if its Workstation parent disappears.

    Normal shutdown still uses the cooperative stop event. This Linux-only
    guard covers Force Quit, SIGKILL, and crashes where Python cannot run the
    QThread/multiprocessing cleanup path. The second getppid check closes the
    race where the parent exits immediately before ``prctl`` is installed.
    """

    if not sys.platform.startswith("linux"):
        return False
    parent_pid = os.getppid()
    try:
        libc = ctypes.CDLL(None, use_errno=True)
        prctl = libc.prctl
        prctl.argtypes = (
            ctypes.c_int,
            ctypes.c_ulong,
            ctypes.c_ulong,
            ctypes.c_ulong,
            ctypes.c_ulong,
        )
        prctl.restype = ctypes.c_int
        if prctl(_LINUX_PR_SET_PDEATHSIG, int(signal.SIGTERM), 0, 0, 0) != 0:
            return False
    except (AttributeError, OSError):
        return False
    if os.getppid() != parent_pid:
        os._exit(128 + int(signal.SIGTERM))
    return True


def _run_yolo_process(config: dict[str, Any], output_queue, stop_event) -> None:
    _arm_linux_parent_death_signal()
    enable_detection = bool(config.get("enable_detection", True))
    thread_count = _worker_cpu_threads(config)
    _configure_worker_thread_environment(thread_count)
    try:
        import cv2
    except Exception as exc:
        _emit_status(output_queue, f"OpenCV 사용 불가: {exc}")
        return
    _configure_cv2_threads(cv2, thread_count)
    if enable_detection:
        rate_limiter = AdaptiveYoloRateLimiter.from_config(config, enable_detection=enable_detection)
        rate_text = (
            f"adaptive={rate_limiter.target_rate_hz:.1f}->{rate_limiter.max_rate_hz:.1f} Hz"
            if rate_limiter.enabled
            else f"max_rate={float(config.get('max_rate_hz', 0.0)):.1f} Hz"
        )
        _emit_status(
            output_queue,
            f"YOLO worker 제한: cpu_threads={thread_count}, {rate_text}",
        )

    model = _load_yolo_model(config, output_queue, enable_detection)
    if enable_detection and model is None:
        _emit_status(output_queue, "YOLO 없이 카메라 preview만 계속 표시합니다.")
        enable_detection = False
        config["enable_detection"] = False
        config["emit_measurement_frames"] = False
        config["adaptive_rate"] = False

    backend = normalize_stream_backend(str(config.get("stream_backend", STREAM_BACKEND_GSTREAMER)))
    if backend in {STREAM_BACKEND_OPENCV, STREAM_BACKEND_OPENCV_AUTO}:
        _run_opencv_stream(config, output_queue, stop_event, cv2, model, enable_detection)
    elif backend == STREAM_BACKEND_PYAV:
        _run_pyav_stream(config, output_queue, stop_event, cv2, model, enable_detection)
    else:
        _run_gstreamer_stream(config, output_queue, stop_event, cv2, model, enable_detection)
