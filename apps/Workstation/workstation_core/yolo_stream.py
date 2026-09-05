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
from Workstation.workstation_core.mp4_integrity import (
    H264_MP4_INTEGRITY_SCHEMA,
)
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


SASL_ROOT = Path(__file__).resolve().parents[3]
DEFAULT_YOLO_PYTHON = sys.executable
DEFAULT_YOLO_WEIGHTS = os.environ.get("YOLO_MODEL_PATH") or str(
    WORKSTATION_MODELS_ROOT / "yolov8m.pt"
)
DEFAULT_RTSP_LATENCY_MS = 50
DEFAULT_PREVIEW_DISPLAY_RATE_HZ = 15.0
DEFAULT_CPU_YOLO_MAX_RATE_HZ = 20.0
DEFAULT_CUDA_YOLO_MAX_RATE_HZ = 15.0
DEFAULT_CPU_YOLO_INITIAL_RATE_HZ = 15.0
DEFAULT_CPU_YOLO_ADAPTIVE_MAX_RATE_HZ = 20.0
DEFAULT_CPU_YOLO_MIN_RATE_HZ = 2.0
DEFAULT_DECODE_RATE_WINDOW_S = 5.0
CPU_DECODE_HZ_FLOOR = 10.0
CPU_DECODE_HZ_CRITICAL_GUARD = 11.5
CPU_DECODE_HZ_SOFT_FLOOR = 12.5
CPU_DECODE_HZ_GUARD = 14.0
CPU_DECODE_HZ_RECOVERY = 16.0
CPU_DECODE_HZ_STRONG_RECOVERY = 18.0
DEFAULT_CPU_WORKER_THREADS = 4
DEFAULT_CUDA_WORKER_THREADS = 1
DEFAULT_EVENT_QUEUE_SIZE = 128
DEFAULT_FRAME_QUEUE_SIZE = 2
DEFAULT_METADATA_QUEUE_SIZE = 64
DEFAULT_RTSP_FRAME_TIMEOUT_S = 30.0
YOLO_INPUT_SIZE_STRIDE = 32
DECODER_AUTO = "auto"
DECODER_NVIDIA = "nvidia"
DECODER_SOFTWARE = "software"
STREAM_BACKEND_GSTREAMER = "gstreamer"
STREAM_BACKEND_OPENCV = "opencv"
STREAM_BACKEND_OPENCV_AUTO = "opencv_auto"
STREAM_BACKEND_PYAV = "pyav"
RECORDING_ENCODING_H264_COPY_GATED = "h264_copy_integrity_gated"
RECORDING_ENCODING_DECODED_REENCODE_UNVERIFIED = "decoded_reencode_unverified"
RECORDING_ENCODING_DECODED_NVENC_LOSSLESS = "decoded_nvenc_lossless_reencode"
RTSP_TRANSPORT_UDP = "udp"
RTSP_TRANSPORT_TCP = "tcp"
RTSP_RECONNECT_MANUAL = "manual"
RTSP_RECONNECT_AUTOMATIC = "automatic"
NVIDIA_H264_DECODER = "nvh264dec"
SOFTWARE_H264_DECODER = "avdec_h264"
RED_BGR = (0, 0, 255)
NEUTRAL_BGR = (128, 128, 128)
WHITE_BGR = (255, 255, 255)
RTP_CLOCK_RATE_HZ = 90_000
NTP_UNIX_EPOCH_OFFSET_S = 2_208_988_800
UINT32_MODULUS = 2**32
UINT32_HALF_RANGE = 2**31
MAX_RTP_TIMING_CACHE = 240
MAX_RTP_PTS_MATCH_DELTA_NS = 5_000_000
MAX_RTP_DISCONTINUITY_CACHE = 64
H264_NAL_TYPE_IDR = 5
H264_NAL_TYPE_NON_IDR_SLICE = 1
H264_NAL_TYPE_SPS = 7
H264_NAL_TYPE_PPS = 8
H264_RECOVERY_BASIS_IDR = "h264_idr"
H264_RECOVERY_BASIS_NON_IDR_INTRA_EXPERIMENTAL = (
    "wiris_non_idr_intra_experimental"
)
H264_ACCESS_UNIT_KIND_NAL1_NON_IDR_INTRA = "nal1_non_idr_intra"
MAX_RTCP_SR_ARRIVAL_CACHE = 128
MAX_RTCP_RATE_SAMPLES = 12
DEFAULT_RECORD_VIDEO_QUEUE_SIZE = 300
GST_RTSP_RECONNECT_DELAYS_S = (1.0, 2.0, 4.0, 8.0)
GST_ENCODED_QUEUE_MAX_TIME_NS = 250_000_000
GST_DECODED_QUEUE_MAX_BUFFERS = 8
GST_RECORDING_QUEUE_MAX_TIME_NS = 5_000_000_000
GST_RECORDING_EOS_TIMEOUT_S = 9.0
INTEGRITY_RECOVERY_TIMEOUT_S = 30.0
YOLO_COOPERATIVE_STOP_TIMEOUT_S = 30.0
YOLO_KILL_AFTER_STOP_S = 40.0
YOLO_KILL_JOIN_TIMEOUT_S = 1.0
YOLO_PROCESS_CRASH_RESTART_DELAYS_S = (0.25, 0.5, 1.0)
YOLO_PROCESS_STABLE_RUN_S = 30.0
_LINUX_PR_SET_PDEATHSIG = 1
DECODER_FALLBACK_FIRST_FRAME_TIMEOUT_S = 30.0
DECODER_FALLBACK_STALL_TIMEOUT_S = 30.0
DECODER_FALLBACK_STATUS_PREFIX = "DECODER_FALLBACK_REQUIRED:"
DECODER_FALLBACK_ACTIVE_STATUS_PREFIX = "DECODER_FALLBACK_ACTIVE:"
GST_ERROR_RTSP_SOURCE = "rtsp-source"
GST_ERROR_RECORDING_OUTPUT = "recording-output"
GST_ERROR_DECODER_HEALTH = "decoder-health"
GST_ERROR_PIPELINE = "pipeline"


def app_receive_monotonic_sample() -> tuple[int, float]:
    """Capture one exact host-monotonic decoded-frame receipt timestamp."""

    monotonic_ns = time.monotonic_ns()
    return monotonic_ns, monotonic_ns * 1.0e-9


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


@dataclass(frozen=True)
class DetectionBox:
    x1: float
    y1: float
    x2: float
    y2: float
    confidence: float
    class_id: int
    class_name: str | None = None

    @property
    def center(self) -> tuple[float, float]:
        return ((self.x1 + self.x2) / 2.0, (self.y1 + self.y2) / 2.0)

    def as_dict(self) -> dict[str, float | int | str]:
        center_x, center_y = self.center
        payload: dict[str, float | int | str] = {
            "x1": self.x1,
            "y1": self.y1,
            "x2": self.x2,
            "y2": self.y2,
            "center_x": center_x,
            "center_y": center_y,
            "confidence": self.confidence,
            "class_id": self.class_id,
        }
        if self.class_name:
            payload["class_name"] = self.class_name
        return payload


def _resolve_class_name(names: object | None, class_id: int) -> str:
    if isinstance(names, dict):
        value = names.get(class_id)
        if value is not None:
            return str(value)
    elif isinstance(names, (list, tuple)):
        if 0 <= class_id < len(names):
            value = names[class_id]
            if value is not None:
                return str(value)
    return f"#{class_id}"


def highest_confidence_detection(detections: list[DetectionBox]) -> DetectionBox | None:
    if not detections:
        return None
    return max(detections, key=lambda detection: detection.confidence)


def filter_detections_by_class_gate(
    detections: list[DetectionBox],
    *,
    enabled: bool,
    target: object = "",
) -> list[DetectionBox]:
    """Keep only the operator-selected target class before choosing ``best``.

    The gate is a target-class filter, not the stereo correspondence rule.
    Left/right observations still have to carry the same class downstream;
    this function only prevents a higher-confidence distractor from winning
    before the target-class setting can be applied.
    """

    if not enabled:
        return list(detections)
    text = str(target or "").strip()
    if not text:
        return []
    normalized = text.casefold()
    target_id: int | None = None
    candidate = normalized[1:] if normalized.startswith("#") else normalized
    try:
        target_id = int(candidate)
    except ValueError:
        pass
    if target_id is not None:
        return [detection for detection in detections if detection.class_id == target_id]
    return [
        detection
        for detection in detections
        if detection.class_name is not None
        and detection.class_name.strip().casefold() == normalized
    ]


def detections_from_ultralytics_result(result, *, names: object | None = None) -> list[DetectionBox]:
    detections: list[DetectionBox] = []
    boxes = getattr(result, "boxes", None)
    if boxes is None:
        return detections
    for box in boxes:
        class_id = int(box.cls.item())
        x1, y1, x2, y2 = (float(value) for value in box.xyxy[0].tolist())
        detections.append(
            DetectionBox(
                x1=x1,
                y1=y1,
                x2=x2,
                y2=y2,
                confidence=float(box.conf.item()),
                class_id=class_id,
                class_name=_resolve_class_name(names, class_id),
            )
        )
    return detections


def _draw_label(cv2, image, text: str, origin: tuple[int, int], color: tuple[int, int, int]) -> None:
    font = cv2.FONT_HERSHEY_SIMPLEX
    scale = 0.55
    thickness = 1
    (text_width, text_height), baseline = cv2.getTextSize(text, font, scale, thickness)
    x, y = origin
    y = max(text_height + baseline + 2, y)
    cv2.rectangle(
        image,
        (x, y - text_height - baseline - 4),
        (x + text_width + 6, y + baseline + 2),
        color,
        -1,
    )
    cv2.putText(image, text, (x + 3, y - 3), font, scale, WHITE_BGR, thickness, cv2.LINE_AA)


def _draw_detections(cv2, frame, detections: list[DetectionBox], best_detection: DetectionBox | None):
    annotated = frame.copy()
    for detection in detections:
        is_best = detection is best_detection
        color = RED_BGR if is_best else NEUTRAL_BGR
        thickness = 3 if is_best else 2
        x1 = int(round(detection.x1))
        y1 = int(round(detection.y1))
        x2 = int(round(detection.x2))
        y2 = int(round(detection.y2))
        center_x, center_y = detection.center
        center = (int(round(center_x)), int(round(center_y)))

        cv2.rectangle(annotated, (x1, y1), (x2, y2), color, thickness)
        class_label = detection.class_name or f"class {detection.class_id}"
        label = f"{class_label} {detection.confidence:.2f}"
        if is_best:
            label = f"BEST {label} c=({center[0]},{center[1]})"
            cv2.drawMarker(
                annotated,
                center,
                RED_BGR,
                markerType=cv2.MARKER_CROSS,
                markerSize=22,
                thickness=2,
            )
        _draw_label(cv2, annotated, label, (x1, max(0, y1 - 8)), color)
    return annotated


def _emit_status(output_queue, text: str) -> None:
    output_queue.put(("status", text))


def _emit_terminal_exit(output_queue, reason: str) -> None:
    output_queue.put(("terminal_exit", {"reason": str(reason)}))


def _emit_decoder_fallback(
    output_queue,
    *,
    reason: str,
    recording_restart_required: bool,
) -> None:
    output_queue.put(
        (
            "decoder_fallback",
            {
                "reason": str(reason),
                "recording_restart_required": bool(recording_restart_required),
            },
        )
    )


def _emit_recording_finalized(
    output_queue,
    *,
    path: str,
    backend: str,
    success: bool,
    detail: str,
    contract: dict[str, str | bool] | None = None,
    metrics: dict[str, int | float | str | bool] | None = None,
) -> None:
    payload: dict[str, Any] = {
        "path": str(Path(path).expanduser()),
        "stream_backend": normalize_stream_backend(backend),
        "success": bool(success),
        "detail": str(detail),
        **(contract or recording_integrity_contract(backend)),
    }
    if metrics:
        payload.update(metrics)
    output_queue.put(("recording_finalized", payload))


def _record_video_path(config: dict[str, Any]) -> str:
    return str(config.get("record_video_path") or "").strip()


def _record_video_fps(config: dict[str, Any]) -> float:
    try:
        fps = float(config.get("record_video_fps") or 30.0)
    except (TypeError, ValueError):
        fps = 30.0
    return max(1.0, fps)


def _record_video_queue_size(config: dict[str, Any]) -> int:
    try:
        size = int(config.get("record_video_queue_size") or DEFAULT_RECORD_VIDEO_QUEUE_SIZE)
    except (TypeError, ValueError):
        size = DEFAULT_RECORD_VIDEO_QUEUE_SIZE
    return max(1, size)


def _gst_location(value: str) -> str:
    # Gst.parse_launch() does not apply POSIX shell quoting.  Quote the value
    # as a GStreamer string so spaces and apostrophes remain part of the path.
    path = str(Path(value).expanduser())
    escaped = path.replace("\\", "\\\\").replace('"', '\\"')
    return f'"{escaped}"'


class AsyncVideoWriter:
    def __init__(
        self,
        cv2,
        *,
        path: str,
        fps: float,
        width: int,
        height: int,
        max_queue_size: int,
        output_queue,
        backend: str,
    ) -> None:
        self.cv2 = cv2
        self.path = str(path)
        self.fps = float(fps)
        self.width = int(width)
        self.height = int(height)
        self.output_queue = output_queue
        self.backend = normalize_stream_backend(backend)
        self.frames: queue.Queue[Any] = queue.Queue(maxsize=max_queue_size)
        self.stop_event = threading.Event()
        self.dropped_frames = 0
        self.written_frames = 0
        self._last_drop_status_s = 0.0
        self.thread = threading.Thread(target=self._run, name="stereo-video-writer", daemon=True)
        self.thread.start()

    def enqueue(self, frame) -> None:
        if self.stop_event.is_set():
            return
        try:
            self.frames.put_nowait(frame)
            return
        except queue.Full:
            pass
        try:
            self.frames.get_nowait()
            self.frames.task_done()
        except queue.Empty:
            pass
        self.dropped_frames += 1
        now_s = time.monotonic()
        if now_s - self._last_drop_status_s >= 1.0:
            self._last_drop_status_s = now_s
            _emit_status(
                self.output_queue,
                f"영상 녹화 queue 포화: 오래된 프레임 드롭 중 ({self.dropped_frames} dropped)",
            )
        try:
            self.frames.put_nowait(frame)
        except queue.Full:
            self.dropped_frames += 1

    def close(self) -> None:
        self.stop_event.set()
        self.thread.join(timeout=GST_RECORDING_EOS_TIMEOUT_S)
        if self.thread.is_alive():
            _emit_status(self.output_queue, f"영상 녹화 writer 종료 지연: {self.path}")

    def _run(self) -> None:
        writer = None
        success = False
        detail = "writer did not open"
        try:
            output_path = Path(self.path).expanduser()
            output_path.parent.mkdir(parents=True, exist_ok=True)
            fourcc = self.cv2.VideoWriter_fourcc(*"mp4v")
            writer = self.cv2.VideoWriter(str(output_path), fourcc, self.fps, (self.width, self.height))
            if not writer.isOpened():
                writer.release()
                writer = None
                detail = "VideoWriter open failed"
                _emit_status(self.output_queue, f"영상 녹화 파일 열기 실패: {output_path}")
                return
            _emit_status(self.output_queue, f"영상 녹화 시작: {output_path}")
            while not self.stop_event.is_set() or not self.frames.empty():
                try:
                    frame = self.frames.get(timeout=0.1)
                except queue.Empty:
                    continue
                try:
                    if frame.shape[1] != self.width or frame.shape[0] != self.height:
                        frame = self.cv2.resize(frame, (self.width, self.height), interpolation=self.cv2.INTER_AREA)
                    writer.write(frame)
                    self.written_frames += 1
                finally:
                    self.frames.task_done()
            success = True
            detail = "VideoWriter released"
        except Exception as exc:
            detail = f"{type(exc).__name__}: {exc}"
            _emit_status(self.output_queue, f"영상 녹화 writer 오류: {exc}")
        finally:
            if writer is not None:
                writer.release()
            _emit_recording_finalized(
                self.output_queue,
                path=self.path,
                backend=self.backend,
                success=success,
                detail=detail,
                metrics={
                    "written_frames": self.written_frames,
                    "dropped_frames": self.dropped_frames,
                },
            )
            status_text = "저장 완료" if success else "저장 실패"
            _emit_status(
                self.output_queue,
                f"영상 녹화 {status_text}: {self.path} "
                f"(written={self.written_frames}, dropped={self.dropped_frames}, detail={detail})",
            )


class DecodedNv12Recorder:
    """Own one isolated bounded raw-NV12 -> FFmpeg NVENC MP4 writer."""

    def __init__(
        self,
        Gst,
        *,
        path: str,
        width: int,
        height: int,
        fallback_fps: float,
        output_queue,
    ) -> None:
        self.Gst = Gst
        self.path = str(Path(path).expanduser())
        self.width = int(width)
        self.height = int(height)
        self.expected_frame_bytes = self.width * self.height * 3 // 2
        self.frame_period_ns = max(
            1,
            int(round(1_000_000_000.0 / max(1.0, float(fallback_fps)))),
        )
        self.output_queue = output_queue
        self.submitted_frames = 0
        self.written_frames = 0
        self.dropped_frames = 0
        self.duplicated_frames = 0
        self._first_source_pts_ns: int | None = None
        self._last_source_pts_ns: int | None = None
        self._failed_detail = ""
        self._closed = False
        self._accepting = True
        self.frames: queue.Queue[tuple[bytes, int] | None] = queue.Queue(
            maxsize=2
        )

        output_path = Path(self.path)
        output_path.parent.mkdir(parents=True, exist_ok=True)
        command = [
            "ffmpeg",
            "-nostdin",
            "-hide_banner",
            "-loglevel",
            "error",
            "-n",
            "-f",
            "rawvideo",
            "-pixel_format",
            "nv12",
            "-video_size",
            f"{self.width}x{self.height}",
            "-framerate",
            f"{max(1.0, float(fallback_fps)):.6f}",
            "-i",
            "pipe:0",
            "-an",
            "-c:v",
            "h264_nvenc",
            "-preset",
            "losslesshp",
            "-tune",
            "lossless",
            "-rc",
            "constqp",
            "-qp",
            "0",
            "-bf",
            "0",
            "-g",
            "30",
            "-vsync",
            "0",
            self.path,
        ]
        self.process = subprocess.Popen(
            command,
            stdin=subprocess.PIPE,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.PIPE,
            bufsize=0,
        )
        if self.process.stdin is None or self.process.stderr is None:
            self.process.terminate()
            raise RuntimeError("FFmpeg NVENC recorder pipes are unavailable")
        self.thread = threading.Thread(
            target=self._run,
            name="stereo-nvenc-video-writer",
            daemon=True,
        )
        self.thread.start()
        _emit_status(
            self.output_queue,
            f"Decoded NV12 lossless FFmpeg/NVENC recording started: {self.path}",
        )

    def submit(self, buffer, *, observed_monotonic_ns: int) -> bool:
        if self._closed or not self._accepting or self._failed_detail:
            return False
        if self.process.poll() is not None:
            self._failed_detail = f"FFmpeg exited early: {self.process.returncode}"
            return False
        success, map_info = buffer.map(self.Gst.MapFlags.READ)
        if not success:
            return False
        try:
            data = bytes(map_info.data)
        finally:
            buffer.unmap(map_info)
        if len(data) != self.expected_frame_bytes:
            self._failed_detail = (
                "NV12 buffer size mismatch: "
                f"expected={self.expected_frame_bytes}, actual={len(data)}"
            )
            return False
        source_pts_ns = int(
            observed_monotonic_ns
            if buffer.pts == self.Gst.CLOCK_TIME_NONE
            else buffer.pts
        )
        if self._first_source_pts_ns is None:
            self._first_source_pts_ns = source_pts_ns
        self._last_source_pts_ns = source_pts_ns
        self.submitted_frames += 1
        item = (data, source_pts_ns)
        try:
            self.frames.put_nowait(item)
            return True
        except queue.Full:
            pass
        try:
            self.frames.get_nowait()
            self.frames.task_done()
            self.dropped_frames += 1
        except queue.Empty:
            pass
        try:
            self.frames.put_nowait(item)
            return True
        except queue.Full:
            self.dropped_frames += 1
            return False

    def _run(self) -> None:
        assert self.process.stdin is not None
        previous_data: bytes | None = None
        previous_source_pts_ns: int | None = None
        try:
            while True:
                item = self.frames.get()
                try:
                    if item is None:
                        return
                    data, source_pts_ns = item
                    if previous_data is not None and previous_source_pts_ns is not None:
                        elapsed_ns = max(0, source_pts_ns - previous_source_pts_ns)
                        missing_intervals = max(
                            0,
                            int(round(elapsed_ns / self.frame_period_ns)) - 1,
                        )
                        for _ in range(min(30, missing_intervals)):
                            self.process.stdin.write(previous_data)
                            self.written_frames += 1
                            self.duplicated_frames += 1
                    self.process.stdin.write(data)
                    self.written_frames += 1
                    previous_data = data
                    previous_source_pts_ns = source_pts_ns
                finally:
                    self.frames.task_done()
        except (BrokenPipeError, OSError) as exc:
            self._failed_detail = f"{type(exc).__name__}: {exc}"
        finally:
            try:
                self.process.stdin.close()
            except OSError:
                pass

    def _enqueue_stop(self) -> None:
        try:
            self.frames.put(None, timeout=GST_RECORDING_EOS_TIMEOUT_S)
        except queue.Full:
            self._failed_detail = "recorder queue did not drain for shutdown"

    def _stderr_text(self) -> str:
        if self.process.stderr is None:
            return ""
        try:
            data = self.process.stderr.read(16_384)
        except OSError:
            return ""
        return data.decode("utf-8", errors="replace").strip()

    def close(self) -> bool:
        if self._closed:
            return not bool(self._failed_detail)
        self._closed = True
        self._accepting = False
        self._enqueue_stop()
        self.thread.join(timeout=1.0)
        if self.thread.is_alive():
            self._failed_detail = "recorder writer timeout"
            self.process.terminate()
            self.thread.join(timeout=1.0)
        try:
            returncode = self.process.wait(timeout=4.0)
        except subprocess.TimeoutExpired:
            self.process.kill()
            returncode = self.process.wait(timeout=1.0)
            self._failed_detail = self._failed_detail or "FFmpeg exit timeout"
        stderr_text = self._stderr_text()
        if returncode != 0 and not self._failed_detail:
            self._failed_detail = (
                f"FFmpeg exitcode={returncode}"
                + (f": {stderr_text}" if stderr_text else "")
            )
        elif returncode != 0 and stderr_text:
            self._failed_detail = f"{self._failed_detail}: {stderr_text}"

        output_path = Path(self.path)
        try:
            present = output_path.is_file() and output_path.stat().st_size > 0
        except OSError:
            present = False
        success = bool(returncode == 0 and present and not self._failed_detail)
        detail = (
            "FFmpeg closed; lossless NVENC MP4 finalized; output validation pending"
            if success
            else self._failed_detail or "lossless NVENC MP4 unavailable"
        )
        _emit_recording_finalized(
            self.output_queue,
            path=self.path,
            backend=STREAM_BACKEND_GSTREAMER,
            success=success,
            detail=detail,
            contract=recording_integrity_contract(STREAM_BACKEND_GSTREAMER),
            metrics={
                "container_finalization_success": success,
                "source_media_integrity_verified": False,
                "recording_file_integrity_verified": False,
                "integrity_validation_status": "PENDING" if success else "FAIL",
                "integrity_failure_reasons": (
                    ["finalized_mp4_output_validation_pending"]
                    if success
                    else ["decoded_nvenc_recording_failed"]
                ),
                "recording_usable": success,
                "submitted_frames": self.submitted_frames,
                "encoded_frames": self.written_frames,
                "dropped_frames": self.dropped_frames,
                "duplicated_frames": self.duplicated_frames,
                "recording_timing_basis": "source_pts_fixed_rate_gap_duplication",
                "source_duration_ns": (
                    0
                    if self._first_source_pts_ns is None
                    or self._last_source_pts_ns is None
                    else max(
                        0,
                        self._last_source_pts_ns - self._first_source_pts_ns,
                    )
                ),
            },
        )
        _emit_status(
            self.output_queue,
            f"Decoded NVENC recording {'finalized' if success else 'failed'}: "
            f"{self.path} (submitted={self.submitted_frames}, "
            f"encoded={self.written_frames}, dropped={self.dropped_frames}, "
            f"detail={detail})",
        )
        return success


def _open_record_video_writer(cv2, config: dict[str, Any], output_queue, width: int, height: int):
    path = _record_video_path(config)
    if not path or width <= 0 or height <= 0:
        return None
    try:
        return AsyncVideoWriter(
            cv2,
            path=path,
            fps=_record_video_fps(config),
            width=width,
            height=height,
            max_queue_size=_record_video_queue_size(config),
            output_queue=output_queue,
            backend=str(config.get("stream_backend", STREAM_BACKEND_OPENCV)),
        )
    except Exception as exc:
        _emit_status(output_queue, f"영상 녹화 시작 실패: {exc}")
        return None


def _write_record_video_frame(cv2, writer_state, frame) -> None:
    if writer_state is None:
        return
    writer_state.enqueue(frame)


def _release_record_video_writer(output_queue, writer_state) -> None:
    if writer_state is None:
        return
    writer_state.close()


def normalize_decoder_mode(decoder: str | None) -> str:
    value = (decoder or DECODER_AUTO).strip().lower()
    if value in {DECODER_AUTO, DECODER_NVIDIA, DECODER_SOFTWARE}:
        return value
    if value in {NVIDIA_H264_DECODER, "nvdec", "cuda"}:
        return DECODER_NVIDIA
    if value in {SOFTWARE_H264_DECODER, "ffmpeg", "cpu"}:
        return DECODER_SOFTWARE
    return DECODER_AUTO


def decoder_candidates(decoder: str | None) -> tuple[str, ...]:
    mode = normalize_decoder_mode(decoder)
    if mode == DECODER_NVIDIA:
        return (NVIDIA_H264_DECODER,)
    if mode == DECODER_SOFTWARE:
        return (SOFTWARE_H264_DECODER,)
    return (NVIDIA_H264_DECODER, SOFTWARE_H264_DECODER)


def classify_gstreamer_error(
    *,
    source_name: object,
    detail: object,
    active_decoder: object,
) -> str:
    """Classify a GStreamer failure by the element that owns the boundary.

    Generic pipeline errors are deliberately *not* treated as decoder health.
    Falling back to CPU for an RTSP, parser, mux, or filesink failure merely
    hides the real fault and restarts an otherwise healthy decoder.
    """

    source = str(source_name or "").strip().lower().replace("-", "_")
    message = str(detail or "").strip().lower()
    decoder = str(active_decoder or "").strip().lower()

    recording_sources = ("filesink", "record_mux", "mp4mux", "qtmux")
    if any(token in source for token in recording_sources):
        return GST_ERROR_RECORDING_OUTPUT

    rtsp_sources = (
        "rtspsrc",
        "rtph264depay",
        "h264parse",
        "typefind",
        "tcpclientsrc",
        "udpsrc",
    )
    if source == "src" or any(token in source for token in rtsp_sources):
        return GST_ERROR_RTSP_SOURCE

    decoder_sources = tuple(
        token for token in (decoder, NVIDIA_H264_DECODER, SOFTWARE_H264_DECODER) if token
    )
    if any(token in source for token in decoder_sources):
        return GST_ERROR_DECODER_HEALTH

    # parse_launch has no GstMessage source. In that one boundary, an explicit
    # missing/failing decoder element name is still direct decoder evidence.
    if not source and any(token in message for token in decoder_sources):
        return GST_ERROR_DECODER_HEALTH
    if not source and any(token in message for token in recording_sources):
        return GST_ERROR_RECORDING_OUTPUT
    if not source and any(token in message for token in rtsp_sources):
        return GST_ERROR_RTSP_SOURCE
    return GST_ERROR_PIPELINE


def gstreamer_error_fallback_reason(
    *,
    requested_decoder: str,
    active_decoder: str,
    already_fell_back: bool,
    error_class: str,
    detail: object,
) -> str | None:
    """Return an Auto CPU fallback reason only for direct NVDEC health evidence."""

    if error_class != GST_ERROR_DECODER_HEALTH:
        return None
    if normalize_decoder_mode(requested_decoder) != DECODER_AUTO:
        return None
    if active_decoder != NVIDIA_H264_DECODER or already_fell_back:
        return None
    normalized_detail = " ".join(str(detail or "unknown decoder error").split())
    return f"nvh264dec decoder-health error: {normalized_detail}"


def _gstreamer_message_source_name(message: object) -> str:
    source = getattr(message, "src", None)
    getter = getattr(source, "get_name", None)
    if callable(getter):
        try:
            return str(getter() or "")
        except Exception:
            return ""
    return str(source or "")


def _requested_decoder_mode(config: dict[str, Any]) -> str:
    return normalize_decoder_mode(
        str(config.get("_requested_decoder_mode", config.get("decoder", DECODER_AUTO)))
    )


def runtime_decoder_fallback_reason(
    *,
    requested_decoder: str,
    active_decoder: str,
    started_s: float,
    last_frame_s: float | None,
    now_s: float,
    already_fell_back: bool,
    rtp_recovery_pending: bool = False,
    last_recovery_s: float | None = None,
    encoded_input_active: bool = True,
) -> str | None:
    if normalize_decoder_mode(requested_decoder) != DECODER_AUTO:
        return None
    if rtp_recovery_pending:
        return None
    return nvidia_decoder_health_failure_reason(
        active_decoder=active_decoder,
        started_s=started_s,
        last_frame_s=last_frame_s,
        now_s=now_s,
        already_fell_back=already_fell_back,
        rtp_recovery_pending=rtp_recovery_pending,
        last_recovery_s=last_recovery_s,
        encoded_input_active=encoded_input_active,
    )


def nvidia_decoder_health_failure_reason(
    *,
    active_decoder: str,
    started_s: float,
    last_frame_s: float | None,
    now_s: float,
    already_fell_back: bool,
    rtp_recovery_pending: bool = False,
    last_recovery_s: float | None = None,
    encoded_input_active: bool = True,
) -> str | None:
    if rtp_recovery_pending:
        return None
    if (
        already_fell_back
        or active_decoder != NVIDIA_H264_DECODER
        or not encoded_input_active
    ):
        return None
    if last_recovery_s is not None and (
        last_frame_s is None or float(last_recovery_s) > float(last_frame_s)
    ):
        last_frame_s = float(last_recovery_s)
    if last_frame_s is None:
        if now_s - started_s >= DECODER_FALLBACK_FIRST_FRAME_TIMEOUT_S:
            return "nvh264dec first-frame timeout"
        return None
    if now_s - last_frame_s >= DECODER_FALLBACK_STALL_TIMEOUT_S:
        return "nvh264dec decoded-frame stall"
    return None


def format_rtp_sequence_discontinuity(discontinuity: "RtpSequenceDiscontinuity") -> str:
    return (
        "RTP sequence discontinuity: "
        f"ssrc={discontinuity.ssrc}, "
        f"seq={discontinuity.previous_sequence}->{discontinuity.sequence}, "
        f"missing={discontinuity.missing_packets}"
    )


def h264_byte_stream_nal_unit_types(
    access_unit: bytes | bytearray | memoryview,
) -> tuple[int, ...]:
    """Return NAL types from a normalized Annex-B H.264 access unit.

    The live pipeline forces ``h264parse`` to emit byte-stream/alignment=au,
    so start codes are an independent boundary for inspecting NAL headers.
    Payload bytes are never searched as candidate headers: H.264 byte-stream
    emulation prevention guarantees that a real ``00 00 01`` begins a NAL.
    Malformed or non-Annex-B input therefore returns no types and fails closed.
    """

    try:
        data = (
            access_unit
            if isinstance(access_unit, (bytes, bytearray))
            else memoryview(access_unit).cast("B").tobytes()
        )
    except (TypeError, ValueError):
        return ()
    nal_types: list[int] = []
    cursor = 0
    while True:
        # Searching for the three-byte suffix also finds a four-byte start
        # code at its second zero. bytes.find() keeps this inspection in C;
        # the Python camera thread does not walk multi-megabyte slice payloads.
        start_index = data.find(b"\x00\x00\x01", cursor)
        if start_index < 0:
            break
        header_index = start_index + 3
        if header_index >= len(data):
            break
        header = int(data[header_index])
        nal_type = header & 0x1F
        if not (header & 0x80) and not (
            nal_type == H264_NAL_TYPE_IDR and not (header & 0x60)
        ):
            nal_types.append(nal_type)
        cursor = header_index + 1
    return tuple(nal_types)


def h264_access_unit_has_idr(access_unit: bytes | bytearray | memoryview) -> bool:
    """Prove that an AU contains an H.264 IDR slice (NAL unit type 5)."""

    return H264_NAL_TYPE_IDR in h264_byte_stream_nal_unit_types(access_unit)


def h264_nal1_non_idr_intra_metadata(
    access_unit: bytes | bytearray | memoryview,
    *,
    pts_ns: int | None,
    is_delta: bool,
    is_corrupted: bool = False,
) -> dict[str, object] | None:
    """Return positive Auto-capture provenance for one exact NAL1 AU.

    NAL type 1 alone also covers inter-predicted non-IDR pictures. The
    parser-owned ``DELTA_UNIT`` flag is therefore part of the operational
    capture contract. This evidence deliberately does not claim IDR or a
    random-access integrity boundary.
    """

    if pts_ns is None or bool(is_delta) or bool(is_corrupted):
        return None
    nal_types = h264_byte_stream_nal_unit_types(access_unit)
    if (
        H264_NAL_TYPE_NON_IDR_SLICE not in nal_types
        or H264_NAL_TYPE_IDR in nal_types
    ):
        return None
    return {
        "h264_access_unit_kind": H264_ACCESS_UNIT_KIND_NAL1_NON_IDR_INTRA,
        "h264_access_unit_nal_types": list(nal_types),
        "h264_access_unit_is_delta": False,
        "h264_access_unit_pts_ns": int(pts_ns),
        "h264_nal1_non_idr_intra": True,
        "h264_random_access_guaranteed": False,
    }


def h264_access_unit_recovery_basis(
    access_unit: bytes | bytearray | memoryview,
    *,
    is_delta: bool,
    allow_non_idr_intra_experimental: bool = False,
) -> str | None:
    """Classify one parser-normalized AU without relabelling NAL1 as IDR.

    ``h264parse`` owns the AU/non-delta classification.  The experimental
    WIRIS branch additionally requires in-band SPS and PPS in that exact AU;
    a bare NAL1 header or a parser flag alone is never a recovery candidate.
    """

    nal_types = h264_byte_stream_nal_unit_types(access_unit)
    if H264_NAL_TYPE_IDR in nal_types:
        return H264_RECOVERY_BASIS_IDR
    if not allow_non_idr_intra_experimental or bool(is_delta):
        return None
    required_types = {
        H264_NAL_TYPE_NON_IDR_SLICE,
        H264_NAL_TYPE_SPS,
        H264_NAL_TYPE_PPS,
    }
    if required_types.issubset(nal_types):
        return H264_RECOVERY_BASIS_NON_IDR_INTRA_EXPERIMENTAL
    return None


def should_suppress_decoded_output(
    *,
    active_decoder: str,
    rtp_discontinuity_latched: bool,
    tolerant_input: bool = False,
) -> bool:
    """Keep decoder concealment frames out unless the YOLO trial opts in."""

    _ = active_decoder
    return bool(rtp_discontinuity_latched) and not bool(tolerant_input)


def normalize_stream_backend(backend: str | None) -> str:
    value = (backend or STREAM_BACKEND_GSTREAMER).strip().lower()
    if value in {STREAM_BACKEND_GSTREAMER, "gst"}:
        return STREAM_BACKEND_GSTREAMER
    if value in {STREAM_BACKEND_OPENCV, "ffmpeg", "opencv_ffmpeg", "cv2"}:
        return STREAM_BACKEND_OPENCV
    if value in {STREAM_BACKEND_OPENCV_AUTO, "opencv-auto", "opencv_any", "opencv-any", "cap_any"}:
        return STREAM_BACKEND_OPENCV_AUTO
    if value in {STREAM_BACKEND_PYAV, "av"}:
        return STREAM_BACKEND_PYAV
    return STREAM_BACKEND_GSTREAMER


def recording_integrity_contract(backend: str | None) -> dict[str, str | bool]:
    normalized_backend = normalize_stream_backend(backend)
    if normalized_backend == STREAM_BACKEND_GSTREAMER:
        return {
            "recording_encoding_mode": RECORDING_ENCODING_DECODED_NVENC_LOSSLESS,
            "recording_integrity_verified": False,
            "recording_integrity_scope": (
                "decoded-nv12:nvenc-lossless+finalized-mp4;"
                "source-media-integrity=tolerant-unverified"
            ),
        }
    return {
        "recording_encoding_mode": RECORDING_ENCODING_DECODED_REENCODE_UNVERIFIED,
        "recording_integrity_verified": False,
        "recording_integrity_scope": "decoded-frames-only",
    }


def h264_copy_recording_integrity_contract() -> dict[str, str | bool]:
    """Historical direct-copy contract retained for old finalization evidence."""

    return {
        "recording_encoding_mode": RECORDING_ENCODING_H264_COPY_GATED,
        "recording_integrity_verified": False,
        "recording_integrity_scope": (
            "finalized-mp4:h264-idr+frame-num+decode-count+sample-timing"
        ),
    }


def normalize_rtsp_transport(transport: str | None) -> str:
    value = (transport or RTSP_TRANSPORT_UDP).strip().lower()
    if value in {RTSP_TRANSPORT_UDP, "udp-mcast", "udp_multicast"}:
        return RTSP_TRANSPORT_UDP
    if value in {RTSP_TRANSPORT_TCP, "interleaved"}:
        return RTSP_TRANSPORT_TCP
    return RTSP_TRANSPORT_UDP


def normalize_rtsp_reconnect_policy(policy: str | None) -> str:
    value = (policy or RTSP_RECONNECT_MANUAL).strip().lower()
    if value in {RTSP_RECONNECT_AUTOMATIC, "auto", "retry"}:
        return RTSP_RECONNECT_AUTOMATIC
    return RTSP_RECONNECT_MANUAL


def _device_uses_cpu(device: str | None) -> bool:
    return not (device or "").strip() or (device or "").strip().lower() == "cpu"


def default_yolo_max_rate_hz(device: str | None) -> float:
    return (
        DEFAULT_CPU_YOLO_MAX_RATE_HZ
        if _device_uses_cpu(device)
        else DEFAULT_CUDA_YOLO_MAX_RATE_HZ
    )


def effective_yolo_max_rate_hz(max_rate_hz: float | int | str | None, *, device: str | None, enable_detection: bool) -> float:
    try:
        rate_hz = float(max_rate_hz or 0.0)
    except (TypeError, ValueError):
        rate_hz = 0.0
    if enable_detection and rate_hz <= 0.0:
        return default_yolo_max_rate_hz(device)
    return max(0.0, rate_hz)


def _should_emit_preview_frame(config: dict[str, Any], timestamp_s: float) -> bool:
    """Rate-limit UI preview frames while keeping capture/detection independent."""

    try:
        display_rate_hz = float(config.get("display_rate_hz", 0.0) or 0.0)
    except (TypeError, ValueError):
        display_rate_hz = 0.0
    if display_rate_hz <= 0.0:
        return True

    current_s = float(timestamp_s)
    last_s = config.get("_last_preview_emit_s")
    try:
        elapsed_s = current_s - float(last_s) if last_s is not None else float("inf")
    except (TypeError, ValueError):
        elapsed_s = float("inf")
    if elapsed_s >= 0.0 and elapsed_s < 1.0 / display_rate_hz:
        return False
    config["_last_preview_emit_s"] = current_s
    return True


@dataclass
class RollingFrameRateEstimator:
    """Estimate a decoded frame rate without one-second boundary jitter."""

    window_s: float = DEFAULT_DECODE_RATE_WINDOW_S
    _timestamps: deque[float] = field(default_factory=deque)

    def __post_init__(self) -> None:
        try:
            window_s = float(self.window_s)
        except (TypeError, ValueError):
            window_s = DEFAULT_DECODE_RATE_WINDOW_S
        self.window_s = max(1.0, window_s) if math.isfinite(window_s) else DEFAULT_DECODE_RATE_WINDOW_S

    def _trim(self, now_s: float) -> None:
        cutoff_s = now_s - self.window_s
        while self._timestamps and self._timestamps[0] < cutoff_s:
            self._timestamps.popleft()

    def add(self, timestamp_s: float) -> None:
        try:
            timestamp_s = float(timestamp_s)
        except (TypeError, ValueError):
            return
        if not math.isfinite(timestamp_s):
            return
        if self._timestamps and timestamp_s < self._timestamps[-1]:
            return
        self._timestamps.append(timestamp_s)
        self._trim(timestamp_s)

    def rate(self, now_s: float) -> float:
        try:
            now_s = float(now_s)
        except (TypeError, ValueError):
            return 0.0
        if not math.isfinite(now_s):
            return 0.0
        self._trim(now_s)
        if len(self._timestamps) < 2:
            return 0.0
        elapsed_s = min(self.window_s, max(0.0, now_s - self._timestamps[0]))
        if elapsed_s <= 0.0:
            return 0.0
        return (len(self._timestamps) - 1) / elapsed_s


@dataclass
class RollingCounterRateEstimator:
    """Estimate a rate from a monotonic cumulative counter."""

    window_s: float = DEFAULT_DECODE_RATE_WINDOW_S
    _samples: deque[tuple[float, int]] = field(default_factory=deque)

    def __post_init__(self) -> None:
        try:
            window_s = float(self.window_s)
        except (TypeError, ValueError):
            window_s = DEFAULT_DECODE_RATE_WINDOW_S
        self.window_s = max(1.0, window_s) if math.isfinite(window_s) else DEFAULT_DECODE_RATE_WINDOW_S

    def _trim(self, now_s: float) -> None:
        cutoff_s = now_s - self.window_s
        while self._samples and self._samples[0][0] < cutoff_s:
            self._samples.popleft()

    def add(self, timestamp_s: float, total_count: int) -> None:
        try:
            timestamp_s = float(timestamp_s)
            total_count = int(total_count)
        except (TypeError, ValueError):
            return
        if not math.isfinite(timestamp_s) or total_count < 0:
            return
        if self._samples and timestamp_s < self._samples[-1][0]:
            return
        if self._samples and total_count < self._samples[-1][1]:
            # A worker reconnect or counter reset starts a fresh rate window.
            self._samples.clear()
        self._samples.append((timestamp_s, total_count))
        self._trim(timestamp_s)

    def rate(self, now_s: float) -> float:
        try:
            now_s = float(now_s)
        except (TypeError, ValueError):
            return 0.0
        if not math.isfinite(now_s):
            return 0.0
        self._trim(now_s)
        if len(self._samples) < 2:
            return 0.0
        first_s, first_count = self._samples[0]
        _last_s, last_count = self._samples[-1]
        elapsed_s = min(self.window_s, max(0.0, now_s - first_s))
        if elapsed_s <= 0.0:
            return 0.0
        return max(0, last_count - first_count) / elapsed_s


@dataclass
class AdaptiveYoloRateLimiter:
    enabled: bool
    target_rate_hz: float
    max_rate_hz: float
    min_rate_hz: float = DEFAULT_CPU_YOLO_MIN_RATE_HZ

    @classmethod
    def from_config(cls, config: dict[str, Any], *, enable_detection: bool) -> "AdaptiveYoloRateLimiter":
        max_rate_hz = max(0.0, float(config.get("max_rate_hz", 0.0)))
        enabled = bool(config.get("adaptive_rate", False)) and enable_detection and max_rate_hz > 0.0
        if not enabled:
            return cls(False, max_rate_hz, max_rate_hz, 0.0)
        try:
            adaptive_max_rate_hz = float(config.get("adaptive_max_rate_hz", DEFAULT_CPU_YOLO_ADAPTIVE_MAX_RATE_HZ))
        except (TypeError, ValueError):
            adaptive_max_rate_hz = DEFAULT_CPU_YOLO_ADAPTIVE_MAX_RATE_HZ
        max_rate_hz = min(max_rate_hz, max(DEFAULT_CPU_YOLO_MIN_RATE_HZ, adaptive_max_rate_hz))
        min_rate_hz = min(DEFAULT_CPU_YOLO_MIN_RATE_HZ, max_rate_hz)
        initial_rate_hz = min(max(DEFAULT_CPU_YOLO_INITIAL_RATE_HZ, min_rate_hz), max_rate_hz)
        return cls(True, initial_rate_hz, max_rate_hz, min_rate_hz)

    @property
    def interval_s(self) -> float:
        if self.target_rate_hz <= 0.0:
            return 0.0
        return 1.0 / self.target_rate_hz

    def update(self, *, decoded_hz: float, inferred_hz: float) -> bool:
        if not self.enabled:
            return False
        if inferred_hz <= 0.0:
            return False
        previous_rate_hz = self.target_rate_hz
        if decoded_hz <= CPU_DECODE_HZ_FLOOR:
            self.target_rate_hz = self.min_rate_hz
        elif decoded_hz < CPU_DECODE_HZ_CRITICAL_GUARD:
            self.target_rate_hz = self.min_rate_hz
        elif decoded_hz < CPU_DECODE_HZ_SOFT_FLOOR:
            self.target_rate_hz = max(self.min_rate_hz, self.target_rate_hz - 3.0)
        elif decoded_hz < CPU_DECODE_HZ_GUARD:
            self.target_rate_hz = max(self.min_rate_hz, self.target_rate_hz - 1.5)
        elif decoded_hz >= CPU_DECODE_HZ_STRONG_RECOVERY:
            self.target_rate_hz = min(self.max_rate_hz, self.target_rate_hz + 0.3)
        elif decoded_hz >= CPU_DECODE_HZ_RECOVERY:
            self.target_rate_hz = min(self.max_rate_hz, self.target_rate_hz + 0.15)
        return abs(self.target_rate_hz - previous_rate_hz) >= 0.05


def _activate_cpu_fallback_rate_limiter(
    rate_limiter: AdaptiveYoloRateLimiter,
    config: dict[str, Any],
) -> None:
    """Enable the CPU limiter when a requested CUDA device falls back at runtime."""

    if rate_limiter.enabled:
        return
    try:
        configured_max_rate_hz = float(config.get("max_rate_hz", 0.0) or 0.0)
    except (TypeError, ValueError):
        configured_max_rate_hz = 0.0
    max_rate_hz = min(
        configured_max_rate_hz if configured_max_rate_hz > 0.0 else DEFAULT_CPU_YOLO_MAX_RATE_HZ,
        DEFAULT_CPU_YOLO_ADAPTIVE_MAX_RATE_HZ,
    )
    min_rate_hz = min(DEFAULT_CPU_YOLO_MIN_RATE_HZ, max_rate_hz)
    rate_limiter.enabled = True
    rate_limiter.max_rate_hz = max_rate_hz
    rate_limiter.min_rate_hz = min_rate_hz
    rate_limiter.target_rate_hz = min(max(DEFAULT_CPU_YOLO_INITIAL_RATE_HZ, min_rate_hz), max_rate_hz)
    config["device"] = "cpu"
    config["adaptive_rate"] = True
    config["adaptive_max_rate_hz"] = max_rate_hz
    config["max_rate_hz"] = max_rate_hz


def _worker_cpu_threads(config: dict[str, Any]) -> int:
    env_value = os.environ.get("STEREO_YOLO_WORKER_THREADS")
    raw_value = env_value if env_value not in (None, "") else config.get("worker_cpu_threads")
    try:
        value = int(raw_value)
    except (TypeError, ValueError):
        value = DEFAULT_CPU_WORKER_THREADS
    return max(1, min(8, value))


def _default_worker_cpu_threads(device: str | None) -> int:
    return DEFAULT_CPU_WORKER_THREADS if _device_uses_cpu(device) else DEFAULT_CUDA_WORKER_THREADS


def normalize_yolo_input_size(input_size: int | str | None) -> int:
    try:
        value = int(input_size or 0)
    except (TypeError, ValueError):
        value = 640
    value = max(YOLO_INPUT_SIZE_STRIDE, value)
    return ((value + YOLO_INPUT_SIZE_STRIDE - 1) // YOLO_INPUT_SIZE_STRIDE) * YOLO_INPUT_SIZE_STRIDE


def _configure_worker_thread_environment(thread_count: int) -> None:
    value = str(max(1, int(thread_count)))
    for name in (
        "OMP_NUM_THREADS",
        "OPENBLAS_NUM_THREADS",
        "MKL_NUM_THREADS",
        "VECLIB_MAXIMUM_THREADS",
        "NUMEXPR_NUM_THREADS",
    ):
        os.environ[name] = value


def _configure_cv2_threads(cv2, thread_count: int) -> None:
    try:
        cv2.setNumThreads(max(1, int(thread_count)))
    except Exception:
        pass
    try:
        cv2.ocl.setUseOpenCL(False)
    except Exception:
        pass


def _configure_torch_threads(thread_count: int) -> None:
    try:
        import torch
    except Exception:
        return
    thread_count = max(1, int(thread_count))
    try:
        torch.set_num_threads(thread_count)
    except Exception:
        pass
    try:
        torch.set_num_interop_threads(1)
    except Exception:
        pass


def decoder_element_text(decoder: str) -> str:
    decoder_element = decoder_candidates(decoder)[0]
    if decoder_element == SOFTWARE_H264_DECODER:
        # Do not skip B-frames: decoded_hz must represent the complete raw
        # stream. Corrupt frames are still rejected at the decoder boundary.
        return f"{SOFTWARE_H264_DECODER} output-corrupt=false"
    return decoder_element


def ntp64_to_unix_seconds(ntp_timestamp: int) -> float | None:
    if not ntp_timestamp:
        return None
    seconds = int(ntp_timestamp) >> 32
    fraction = int(ntp_timestamp) & 0xFFFFFFFF
    return seconds - NTP_UNIX_EPOCH_OFFSET_S + fraction / float(UINT32_MODULUS)


def rtp_timestamp_delta(timestamp: int, reference: int) -> int:
    return (int(timestamp) - int(reference) + UINT32_HALF_RANGE) % UINT32_MODULUS - UINT32_HALF_RANGE


def estimate_rtcp_ntp_unix_seconds(
    *,
    rtp_timestamp: int,
    sr_rtp_timestamp: int,
    sr_ntp_unix_s: float,
    clock_rate_hz: int,
) -> float | None:
    if clock_rate_hz <= 0:
        return None
    delta_ticks = rtp_timestamp_delta(rtp_timestamp, sr_rtp_timestamp)
    return float(sr_ntp_unix_s) + delta_ticks / float(clock_rate_hz)


def _gst_structure_value(structure, key: str, default=None):
    try:
        return structure.get_value(key)
    except Exception:
        return default


@dataclass(frozen=True)
class RtpFrameTiming:
    rtp_timestamp: int
    rtp_ssrc: int
    rtp_marker_seq: int
    rtp_pts_s: float | None


@dataclass(frozen=True)
class RtpSequenceDiscontinuity:
    ssrc: int
    previous_sequence: int
    sequence: int
    missing_packets: int
    observed_monotonic_s: float
    rtp_pts_ns: int | None = None


@dataclass(frozen=True)
class RtpSequenceRecovery:
    ssrc: int
    resume_pts_ns: int
    observed_monotonic_s: float
    blocked_duration_s: float
    recovery_basis: str = H264_RECOVERY_BASIS_IDR


@dataclass(frozen=True)
class H264AccessUnitDiscontinuity:
    reason: str
    pts_ns: int | None
    observed_monotonic_s: float


@dataclass
class RtpDiscontinuityReporter:
    interval_s: float = 5.0
    total_events: int = 0
    total_missing_packets: int = 0
    _reported_events: int = 0
    _reported_missing_packets: int = 0
    _last_report_s: float | None = None

    def observe(
        self,
        discontinuity: RtpSequenceDiscontinuity,
        *,
        now_s: float | None = None,
    ) -> str | None:
        observed_s = discontinuity.observed_monotonic_s if now_s is None else float(now_s)
        self.total_events += 1
        self.total_missing_packets += int(discontinuity.missing_packets)
        if self._last_report_s is None:
            self._last_report_s = observed_s
            return None
        if observed_s - self._last_report_s < max(0.1, float(self.interval_s)):
            return None
        return self._render(observed_s)

    def flush(self, *, now_s: float | None = None) -> str | None:
        observed_s = time.monotonic() if now_s is None else float(now_s)
        return self._render(observed_s)

    def _render(self, observed_s: float) -> str | None:
        event_delta = self.total_events - self._reported_events
        missing_delta = self.total_missing_packets - self._reported_missing_packets
        if event_delta <= 0:
            return None
        self._reported_events = self.total_events
        self._reported_missing_packets = self.total_missing_packets
        self._last_report_s = observed_s
        return (
            "RTP discontinuity summary: "
            f"events={self.total_events} (+{event_delta}), "
            f"missing={self.total_missing_packets} (+{missing_delta})"
        )


@dataclass(frozen=True)
class RtcpSenderReportTiming:
    ssrc: int
    ntp_raw: int
    ntp_unix_s: float
    rtp_timestamp: int
    clock_rate_hz: int
    received_monotonic_s: float
    received_wall_s: float | None = None
    arrival_basis: str = "poll-detected"


def rtcp_sender_report_pc_offset_s(
    *,
    sr_ntp_unix_s: float,
    received_wall_s: float,
) -> float | None:
    offset_s = float(sr_ntp_unix_s) - float(received_wall_s)
    return offset_s if abs(offset_s) <= 60.0 else None


def sender_report_clock_rates(
    previous: RtcpSenderReportTiming,
    current: RtcpSenderReportTiming,
) -> dict[str, float] | None:
    """Compare the camera's NTP and media clocks against the PC clock.

    Both rates are "camera seconds advanced per PC second" between two Sender
    Reports, so 1.0 means agreement. ``ntp_rate`` below 1.0 means the camera's
    wall clock is losing time (the drift that inflates transit delay);
    ``media_rate`` below 1.0 means its RTP clock ticks slower than the declared
    rate, which inflates transport delay because the media time between two
    reports is then undercounted.
    """

    pc_elapsed_s = current.received_monotonic_s - previous.received_monotonic_s
    if pc_elapsed_s <= 0.0 or current.clock_rate_hz <= 0:
        return None
    ntp_elapsed_s = current.ntp_unix_s - previous.ntp_unix_s
    media_ticks = rtp_timestamp_delta(current.rtp_timestamp, previous.rtp_timestamp)
    media_elapsed_s = media_ticks / float(current.clock_rate_hz)
    return {
        "pc_elapsed_s": pc_elapsed_s,
        "ntp_elapsed_s": ntp_elapsed_s,
        "media_elapsed_s": media_elapsed_s,
        "ntp_rate": ntp_elapsed_s / pc_elapsed_s,
        "media_rate": media_elapsed_s / pc_elapsed_s,
    }


def robust_sender_report_clock_rates(
    samples: deque[dict[str, float]] | list[dict[str, float]],
) -> dict[str, float] | None:
    """Return median clock rates over recent Sender Report intervals."""

    valid = [
        sample
        for sample in samples
        if all(
            key in sample
            and isinstance(sample[key], (int, float))
            and math.isfinite(float(sample[key]))
            and float(sample[key]) > 0.0
            for key in ("ntp_rate", "media_rate")
        )
    ]
    if not valid:
        return None
    return {
        "ntp_rate": float(median(float(sample["ntp_rate"]) for sample in valid)),
        "media_rate": float(median(float(sample["media_rate"]) for sample in valid)),
    }


class GstRtpTimingTracker:
    def __init__(
        self,
        Gst,
        GstRtp,
        *,
        status_callback: Callable[[str], None] | None = None,
        allow_non_idr_intra_experimental: bool = False,
        allow_unverified_encoded_output: bool = False,
    ) -> None:
        self.Gst = Gst
        self.GstRtp = GstRtp
        self._status_callback = status_callback
        self._allow_non_idr_intra_experimental = bool(
            allow_non_idr_intra_experimental
        )
        self._allow_unverified_encoded_output = bool(
            allow_unverified_encoded_output
        )
        self._lock = threading.Lock()
        self._pending_rtp_frames: deque[RtpFrameTiming] = deque(maxlen=MAX_RTP_TIMING_CACHE)
        self._timing_by_pts_ns: dict[int, RtpFrameTiming] = {}
        self._pts_order: deque[int] = deque(maxlen=MAX_RTP_TIMING_CACHE)
        self._nal1_metadata_by_pts_ns: dict[int, dict[str, object]] = {}
        self._nal1_metadata_pts_order: deque[int] = deque()
        self._active_rtp_ssrc: int | None = None
        self._last_rtp_sequence: int | None = None
        self._rtp_packet_count = 0
        self._rtp_gap_events_total = 0
        self._rtp_missing_packets_total = 0
        self._h264_access_units_total = 0
        self._last_rtp_packet_monotonic_s: float | None = None
        self._rtp_discontinuities: deque[RtpSequenceDiscontinuity] = deque(
            maxlen=MAX_RTP_DISCONTINUITY_CACHE
        )
        self._rtp_recoveries: deque[RtpSequenceRecovery] = deque(
            maxlen=MAX_RTP_DISCONTINUITY_CACHE
        )
        self._h264_access_unit_discontinuities: deque[H264AccessUnitDiscontinuity] = deque(
            maxlen=MAX_RTP_DISCONTINUITY_CACHE
        )
        # A newly attached decoder can start in the middle of a dependency
        # chain. Treat startup exactly like post-loss recovery: no compressed
        # or decoded output is trusted until a parser-normalized IDR AU enters
        # the decoder and that exact PTS returns from it.
        self._decoded_output_blocked = True
        self._encoded_output_blocked = True
        self._blocked_after_pts_ns: int | None = None
        self._blocked_since_monotonic_s: float | None = time.monotonic()
        self._recovery_candidate_pts_ns: int | None = None
        self._recovery_candidate_observed_s: float | None = None
        self._recovery_candidate_basis: str | None = None
        self._output_recovery_basis: str | None = None
        self._encoded_recovery_marker_pts_ns: int | None = None
        self._accepted_access_unit_seen = False
        self._access_unit_discontinuity_reported = False
        self._sr_by_ssrc: dict[int, RtcpSenderReportTiming] = {}
        self._sr_count_by_ssrc: dict[int, int] = {}
        self._sr_history_by_ssrc: dict[int, deque[RtcpSenderReportTiming]] = {}
        self._clock_rate_samples_by_ssrc: dict[int, deque[dict[str, float]]] = {}
        self._clock_rates_by_ssrc: dict[int, dict[str, float]] = {}
        self._sr_arrival_by_key: dict[tuple[int, int], tuple[float, float]] = {}
        self._sr_arrival_order: deque[tuple[int, int]] = deque()
        self._rtcp_probe_pads: set[str] = set()
        self._manager = None
        self._last_rtcp_poll_s = 0.0
        self._last_clock_rates: dict[str, float] | None = None

    def _emit_status(self, text: str) -> None:
        if self._status_callback is not None:
            self._status_callback(text)

    def _remember_nal1_metadata(self, metadata: dict[str, object]) -> None:
        pts_ns = int(metadata["h264_access_unit_pts_ns"])
        with self._lock:
            if pts_ns not in self._nal1_metadata_by_pts_ns:
                self._nal1_metadata_pts_order.append(pts_ns)
            self._nal1_metadata_by_pts_ns[pts_ns] = dict(metadata)
            while len(self._nal1_metadata_pts_order) > MAX_RTP_TIMING_CACHE:
                stale_pts_ns = self._nal1_metadata_pts_order.popleft()
                self._nal1_metadata_by_pts_ns.pop(stale_pts_ns, None)

    def _pop_nal1_metadata(self, pts_ns: int | None) -> dict[str, object] | None:
        if pts_ns is None:
            return None
        with self._lock:
            normalized_pts_ns = int(pts_ns)
            metadata = self._nal1_metadata_by_pts_ns.pop(normalized_pts_ns, None)
            if metadata is not None:
                try:
                    self._nal1_metadata_pts_order.remove(normalized_pts_ns)
                except ValueError:
                    pass
        return None if metadata is None else dict(metadata)

    def register_rtp_sequence(
        self,
        *,
        ssrc: int,
        sequence: int,
        observed_monotonic_s: float | None = None,
        rtp_pts_ns: int | None = None,
    ) -> RtpSequenceDiscontinuity | None:
        """Record forward gaps in the active SSRC's 16-bit sequence space."""

        normalized_ssrc = int(ssrc)
        normalized_sequence = int(sequence) & 0xFFFF
        observed_s = time.monotonic() if observed_monotonic_s is None else float(observed_monotonic_s)
        normalized_pts_ns = None if rtp_pts_ns is None else int(rtp_pts_ns)
        with self._lock:
            self._rtp_packet_count += 1
            self._last_rtp_packet_monotonic_s = observed_s
            if self._active_rtp_ssrc != normalized_ssrc:
                previous_ssrc = self._active_rtp_ssrc
                self._active_rtp_ssrc = normalized_ssrc
                self._last_rtp_sequence = normalized_sequence
                if previous_ssrc is not None:
                    self._h264_access_unit_discontinuities.append(
                        H264AccessUnitDiscontinuity(
                            reason="ssrc-change",
                            pts_ns=normalized_pts_ns,
                            observed_monotonic_s=observed_s,
                        )
                    )
                    self._access_unit_discontinuity_reported = True
                    # A new SSRC owns a new RTP timestamp domain. Do not carry
                    # the prior stream's high PTS threshold into its recovery.
                    self._blocked_after_pts_ns = None
                    self._latch_compressed_discontinuity_locked(
                        observed_s=observed_s,
                        pts_ns=normalized_pts_ns,
                    )
                self._clear_frame_timing_locked()
                return None
            previous = self._last_rtp_sequence
            if previous is None:
                self._last_rtp_sequence = normalized_sequence
                return None
            delta = (normalized_sequence - previous) & 0xFFFF
            if delta == 0 or delta >= 0x8000:
                # Duplicate or older/reordered packets do not move the forward
                # baseline and therefore do not create a false discontinuity.
                return None
            self._last_rtp_sequence = normalized_sequence
            if delta == 1:
                return None
            discontinuity = RtpSequenceDiscontinuity(
                ssrc=normalized_ssrc,
                previous_sequence=previous,
                sequence=normalized_sequence,
                missing_packets=delta - 1,
                observed_monotonic_s=observed_s,
                rtp_pts_ns=normalized_pts_ns,
            )
            self._rtp_discontinuities.append(discontinuity)
            self._rtp_gap_events_total += 1
            self._rtp_missing_packets_total += int(discontinuity.missing_packets)
            # Popping the status event must not release this latch. Only a
            # parser-proven IDR followed by that exact decoded PTS can do so.
            self._latch_compressed_discontinuity_locked(
                observed_s=observed_s,
                pts_ns=normalized_pts_ns,
            )
            self._clear_frame_timing_locked()
            return discontinuity

    def has_recent_rtp_input(
        self,
        *,
        now_s: float | None = None,
        max_age_s: float = 2.0,
    ) -> bool:
        """True only when the compressed-input boundary is still receiving RTP."""

        observed_s = time.monotonic() if now_s is None else float(now_s)
        with self._lock:
            last_s = self._last_rtp_packet_monotonic_s
            return bool(
                self._rtp_packet_count > 0
                and last_s is not None
                and observed_s - last_s <= max(0.0, float(max_age_s))
            )

    def rtp_metrics_snapshot(self) -> dict[str, int | None]:
        """Return transport and parser counters for the UI rate display."""

        with self._lock:
            return {
                "rtp_packets_received_total": int(self._rtp_packet_count),
                "rtp_gap_events_total": int(self._rtp_gap_events_total),
                "rtp_missing_packets_total": int(self._rtp_missing_packets_total),
                "h264_access_units_total": int(self._h264_access_units_total),
                "rtp_active_ssrc": self._active_rtp_ssrc,
            }

    def pop_rtp_discontinuity(self) -> RtpSequenceDiscontinuity | None:
        with self._lock:
            if not self._rtp_discontinuities:
                return None
            return self._rtp_discontinuities.popleft()

    def decoded_output_blocked(self) -> bool:
        with self._lock:
            return self._decoded_output_blocked

    def encoded_output_blocked(self) -> bool:
        with self._lock:
            return self._encoded_output_blocked

    def blocked_duration_s(self, *, now_s: float | None = None) -> float:
        observed_s = time.monotonic() if now_s is None else float(now_s)
        with self._lock:
            if not self._decoded_output_blocked or self._blocked_since_monotonic_s is None:
                return 0.0
            return max(0.0, observed_s - self._blocked_since_monotonic_s)

    def _latch_compressed_discontinuity_locked(
        self,
        *,
        observed_s: float,
        pts_ns: int | None,
    ) -> None:
        self._decoded_output_blocked = True
        self._encoded_output_blocked = True
        if self._blocked_since_monotonic_s is None:
            self._blocked_since_monotonic_s = observed_s
        if pts_ns is None:
            self._blocked_after_pts_ns = None
        elif self._blocked_after_pts_ns is None:
            self._blocked_after_pts_ns = int(pts_ns)
        else:
            self._blocked_after_pts_ns = max(self._blocked_after_pts_ns, int(pts_ns))
        self._recovery_candidate_pts_ns = None
        self._recovery_candidate_observed_s = None
        self._recovery_candidate_basis = None
        self._encoded_recovery_marker_pts_ns = None

    def _register_access_unit_locked(
        self,
        *,
        pts_ns: int | None,
        is_delta: bool,
        is_corrupted: bool,
        is_random_access: bool,
        recovery_basis: str | None,
        observed_s: float,
    ) -> bool:
        _ = is_delta
        normalized_basis = str(recovery_basis or "").strip()
        if not normalized_basis and is_random_access:
            normalized_basis = H264_RECOVERY_BASIS_IDR
        allowed_basis = normalized_basis == H264_RECOVERY_BASIS_IDR or (
            self._allow_non_idr_intra_experimental
            and normalized_basis
            == H264_RECOVERY_BASIS_NON_IDR_INTRA_EXPERIMENTAL
        )
        if (
            not self._decoded_output_blocked
            or not allowed_basis
            or is_corrupted
            or pts_ns is None
        ):
            return False
        # The AU containing the missing RTP packet is itself suspect even when
        # it contains an IDR. Recovery must be a later, complete IDR AU.
        if self._blocked_after_pts_ns is not None and pts_ns <= self._blocked_after_pts_ns:
            return False
        self._recovery_candidate_pts_ns = int(pts_ns)
        self._recovery_candidate_observed_s = observed_s
        self._recovery_candidate_basis = normalized_basis
        self._encoded_recovery_marker_pts_ns = int(pts_ns)
        self._encoded_output_blocked = False
        return True

    def register_access_unit(
        self,
        *,
        pts_ns: int | None,
        is_delta: bool,
        is_corrupted: bool,
        is_random_access: bool = False,
        recovery_basis: str | None = None,
        observed_monotonic_s: float | None = None,
    ) -> bool:
        observed_s = time.monotonic() if observed_monotonic_s is None else float(observed_monotonic_s)
        normalized_pts_ns = None if pts_ns is None else int(pts_ns)
        with self._lock:
            return self._register_access_unit_locked(
                pts_ns=normalized_pts_ns,
                is_delta=bool(is_delta),
                is_corrupted=bool(is_corrupted),
                is_random_access=bool(is_random_access),
                recovery_basis=recovery_basis,
                observed_s=observed_s,
            )

    def should_drop_encoded_access_unit(
        self,
        *,
        pts_ns: int | None,
        is_delta: bool,
        is_corrupted: bool,
        is_discontinuous: bool,
        is_random_access: bool = False,
        recovery_basis: str | None = None,
        observed_monotonic_s: float | None = None,
    ) -> bool:
        """Keep an invalid AU and every dependent AU out of all consumers."""

        observed_s = time.monotonic() if observed_monotonic_s is None else float(observed_monotonic_s)
        normalized_pts_ns = None if pts_ns is None else int(pts_ns)
        normalized_basis = str(recovery_basis or "").strip()
        random_access_candidate = bool(is_random_access or normalized_basis)
        with self._lock:
            # GstBaseParse marks the first valid IDR DISCONT at a normal stream
            # or segment start. An explicitly parsed IDR is a safe reset for
            # that marker; DISCONT on every other accepted chain remains strict.
            reason = (
                "missing-pts"
                if normalized_pts_ns is None
                else "corrupted"
                if is_corrupted
                else "discont"
                if (
                    is_discontinuous
                    and self._accepted_access_unit_seen
                    and not random_access_candidate
                )
                else None
            )
            if reason is not None:
                is_new_event = (
                    not self._access_unit_discontinuity_reported
                    or not self._encoded_output_blocked
                    or self._recovery_candidate_pts_ns is not None
                )
                if is_new_event:
                    self._h264_access_unit_discontinuities.append(
                        H264AccessUnitDiscontinuity(
                            reason=reason,
                            pts_ns=normalized_pts_ns,
                            observed_monotonic_s=observed_s,
                        )
                    )
                    self._access_unit_discontinuity_reported = True
                self._latch_compressed_discontinuity_locked(
                    observed_s=observed_s,
                    pts_ns=normalized_pts_ns,
                )
                return True
            if not self._encoded_output_blocked:
                self._accepted_access_unit_seen = True
                return False
            if self._register_access_unit_locked(
                pts_ns=normalized_pts_ns,
                is_delta=bool(is_delta),
                is_corrupted=bool(is_corrupted),
                is_random_access=random_access_candidate,
                recovery_basis=normalized_basis,
                observed_s=observed_s,
            ):
                self._accepted_access_unit_seen = True
                return False
            return True

    def pop_h264_access_unit_discontinuity(self) -> H264AccessUnitDiscontinuity | None:
        with self._lock:
            if not self._h264_access_unit_discontinuities:
                return None
            return self._h264_access_unit_discontinuities.popleft()

    def consume_encoded_recovery_marker(self, pts_ns: int | None) -> bool:
        normalized_pts_ns = None if pts_ns is None else int(pts_ns)
        with self._lock:
            if (
                normalized_pts_ns is None
                or self._encoded_recovery_marker_pts_ns != normalized_pts_ns
            ):
                return False
            self._encoded_recovery_marker_pts_ns = None
            return True

    def should_suppress_decoded_buffer(self, pts_ns: int | None) -> bool:
        normalized_pts_ns = None if pts_ns is None else int(pts_ns)
        with self._lock:
            if not self._decoded_output_blocked:
                return False
            resume_pts_ns = self._recovery_candidate_pts_ns
            if (
                resume_pts_ns is None
                or normalized_pts_ns is None
                or normalized_pts_ns != resume_pts_ns
            ):
                return True
            observed_s = self._recovery_candidate_observed_s or time.monotonic()
            blocked_since_s = self._blocked_since_monotonic_s or observed_s
            recovery_basis = (
                self._recovery_candidate_basis or H264_RECOVERY_BASIS_IDR
            )
            self._rtp_recoveries.append(
                RtpSequenceRecovery(
                    ssrc=int(self._active_rtp_ssrc or 0),
                    resume_pts_ns=resume_pts_ns,
                    observed_monotonic_s=observed_s,
                    blocked_duration_s=max(0.0, observed_s - blocked_since_s),
                    recovery_basis=recovery_basis,
                )
            )
            self._decoded_output_blocked = False
            self._encoded_output_blocked = False
            self._blocked_after_pts_ns = None
            self._blocked_since_monotonic_s = None
            self._recovery_candidate_pts_ns = None
            self._recovery_candidate_observed_s = None
            self._recovery_candidate_basis = None
            self._encoded_recovery_marker_pts_ns = None
            self._access_unit_discontinuity_reported = False
            self._output_recovery_basis = recovery_basis
            return False

    def current_output_recovery_basis(self) -> str | None:
        with self._lock:
            return self._output_recovery_basis

    def pop_rtp_recovery(self) -> RtpSequenceRecovery | None:
        with self._lock:
            if not self._rtp_recoveries:
                return None
            return self._rtp_recoveries.popleft()

    def _clear_frame_timing_locked(self) -> None:
        self._pending_rtp_frames.clear()
        self._timing_by_pts_ns.clear()
        self._pts_order.clear()
        self._nal1_metadata_by_pts_ns.clear()
        self._nal1_metadata_pts_order.clear()

    def _register_sender_report(self, timing: RtcpSenderReportTiming) -> str | None:
        with self._lock:
            previous = self._sr_by_ssrc.get(timing.ssrc)
            if previous is not None and previous.ntp_raw == timing.ntp_raw:
                # The poll re-reads the same report from the GStreamer session
                # stats every 250 ms. Keep the timestamps from when this report
                # first appeared, so rtcp_sr_age_s measures how stale the anchor
                # is instead of how recently we polled.
                return None
            self._sr_count_by_ssrc[timing.ssrc] = self._sr_count_by_ssrc.get(timing.ssrc, 0) + 1
            self._sr_by_ssrc[timing.ssrc] = timing
            self._sr_history_by_ssrc.setdefault(timing.ssrc, deque(maxlen=8)).append(timing)
        if previous is None:
            return None
        interval_s = timing.received_monotonic_s - previous.received_monotonic_s
        if interval_s <= 0.0:
            return None
        rates = sender_report_clock_rates(previous, timing)
        if rates is not None:
            with self._lock:
                samples = self._clock_rate_samples_by_ssrc.setdefault(
                    timing.ssrc,
                    deque(maxlen=MAX_RTCP_RATE_SAMPLES),
                )
                samples.append(rates)
                robust_rates = robust_sender_report_clock_rates(samples)
                if robust_rates is not None:
                    self._clock_rates_by_ssrc[timing.ssrc] = robust_rates
                    self._last_clock_rates = robust_rates
        rate_text = ""
        robust_rates = self._clock_rates_by_ssrc.get(timing.ssrc)
        if robust_rates is not None:
            rate_text = (
                f", ntp_rate={robust_rates['ntp_rate']:.6f}"
                f", media_rate={robust_rates['media_rate']:.6f}"
            )
        return (
            "[RTCP] Sender Report 갱신: "
            f"ssrc={timing.ssrc}, interval={interval_s:.2f}s, "
            f"ntp={timing.ntp_unix_s:.6f}{rate_text}"
        )

    def poll_sender_reports(self, *, now_s: float | None = None) -> None:
        """Refresh RTCP statistics without requiring a decoded frame."""

        self._poll_rtcp_sender_reports(
            time.monotonic() if now_s is None else float(now_s)
        )

    def sender_report_observations(
        self,
        *,
        now_s: float | None = None,
    ) -> list[dict[str, int | float | str | None]]:
        """Return bounded pad-arrival SR evidence independent of decode output."""

        observed_s = time.monotonic() if now_s is None else float(now_s)
        with self._lock:
            histories = {
                ssrc: tuple(rows)
                for ssrc, rows in self._sr_history_by_ssrc.items()
            }
            counts = dict(self._sr_count_by_ssrc)
            rates = {
                ssrc: dict(values)
                for ssrc, values in self._clock_rates_by_ssrc.items()
            }
        observations: list[dict[str, int | float | str | None]] = []
        for ssrc in sorted(histories):
            clock_rates = rates.get(ssrc, {})
            previous_timing: RtcpSenderReportTiming | None = None
            for timing in histories[ssrc]:
                offset_s = (
                    None
                    if timing.received_wall_s is None
                    else rtcp_sender_report_pc_offset_s(
                        sr_ntp_unix_s=timing.ntp_unix_s,
                        received_wall_s=timing.received_wall_s,
                    )
                )
                raw_offset_s = (
                    None
                    if timing.received_wall_s is None
                    else float(timing.ntp_unix_s) - float(timing.received_wall_s)
                )
                host_clock_step_s = None
                if (
                    previous_timing is not None
                    and previous_timing.received_wall_s is not None
                    and timing.received_wall_s is not None
                ):
                    host_clock_step_s = (
                        float(timing.received_wall_s)
                        - float(previous_timing.received_wall_s)
                        - (
                            float(timing.received_monotonic_s)
                            - float(previous_timing.received_monotonic_s)
                        )
                    )
                observations.append(
                    {
                        "ssrc": int(timing.ssrc),
                        "ntp_raw": int(timing.ntp_raw),
                        "ntp_unix_s": float(timing.ntp_unix_s),
                        "rtp_timestamp": int(timing.rtp_timestamp),
                        "clock_rate_hz": int(timing.clock_rate_hz),
                        "received_monotonic_s": float(timing.received_monotonic_s),
                        "received_wall_s": timing.received_wall_s,
                        "received_count": int(counts.get(ssrc, 0)),
                        "age_s": max(0.0, observed_s - timing.received_monotonic_s),
                        "arrival_basis": str(timing.arrival_basis),
                        "sr_pc_offset_s": offset_s,
                        "sr_pc_offset_raw_s": raw_offset_s,
                        "ntp_rate": clock_rates.get("ntp_rate"),
                        "media_rate": clock_rates.get("media_rate"),
                        "host_clock_step_s": host_clock_step_s,
                    }
                )
                previous_timing = timing
        observations.sort(
            key=lambda row: (
                float(row["received_monotonic_s"]),
                int(row["ssrc"]),
                int(row["ntp_raw"]),
            )
        )
        return observations

    def attach(self, pipeline) -> None:
        source = pipeline.get_by_name("src")
        depay = pipeline.get_by_name("depay")
        parser = pipeline.get_by_name("parse")
        if source is not None:
            source.connect("new-manager", self._handle_new_manager)
        if depay is None:
            return
        sink_pad = depay.get_static_pad("sink")
        src_pad = depay.get_static_pad("src")
        if sink_pad is not None:
            sink_pad.add_probe(self.Gst.PadProbeType.BUFFER, self._capture_rtp_packet)
        if src_pad is not None:
            src_pad.add_probe(self.Gst.PadProbeType.BUFFER, self._map_depay_buffer_pts)
        if parser is not None:
            parser_src_pad = parser.get_static_pad("src")
            if parser_src_pad is not None:
                parser_src_pad.add_probe(self.Gst.PadProbeType.BUFFER, self._observe_access_unit)

    def frame_metadata(
        self,
        buffer,
        now_s: float,
        *,
        received_wall_s: float | None = None,
    ) -> dict[str, object] | None:
        received_wall_s = time.time() if received_wall_s is None else float(received_wall_s)
        self._poll_rtcp_sender_reports(now_s)
        pts_ns = None if buffer.pts == self.Gst.CLOCK_TIME_NONE else int(buffer.pts)
        metadata = self._pop_nal1_metadata(pts_ns) or {}
        if metadata:
            metadata["decoded_buffer_pts_ns"] = pts_ns
        frame_timing = self._pop_frame_timing(pts_ns)
        if frame_timing is None:
            return metadata or None
        metadata.update(
            {
                "rtp_timestamp": frame_timing.rtp_timestamp,
                "rtp_ssrc": frame_timing.rtp_ssrc,
                "rtp_marker_seq": frame_timing.rtp_marker_seq,
            }
        )
        recovery_basis = self.current_output_recovery_basis()
        if recovery_basis:
            metadata.update(
                {
                    "media_recovery_basis": recovery_basis,
                    "media_integrity_mode": (
                        "experimental_non_idr_intra"
                        if self._allow_non_idr_intra_experimental
                        else "strict_idr"
                    ),
                    "media_random_access_guaranteed": (
                        recovery_basis == H264_RECOVERY_BASIS_IDR
                    ),
                }
            )
        if frame_timing.rtp_pts_s is not None:
            metadata["rtp_pts_s"] = frame_timing.rtp_pts_s
        sr_timing = self._sr_by_ssrc.get(frame_timing.rtp_ssrc)
        if sr_timing is None:
            return metadata
        sr_received_count = self._sr_count_by_ssrc.get(frame_timing.rtp_ssrc, 0)
        frame_ntp_s = estimate_rtcp_ntp_unix_seconds(
            rtp_timestamp=frame_timing.rtp_timestamp,
            sr_rtp_timestamp=sr_timing.rtp_timestamp,
            sr_ntp_unix_s=sr_timing.ntp_unix_s,
            clock_rate_hz=sr_timing.clock_rate_hz,
        )
        metadata.update(
            {
                "rtcp_sr_ntp_raw": sr_timing.ntp_raw,
                "rtcp_sr_ntp_unix_s": sr_timing.ntp_unix_s,
                "rtcp_sr_rtp_timestamp": sr_timing.rtp_timestamp,
                "rtcp_clock_rate_hz": sr_timing.clock_rate_hz,
                "rtcp_sr_age_s": max(0.0, now_s - sr_timing.received_monotonic_s),
                "rtcp_sr_received_wall_s": sr_timing.received_wall_s,
                "rtcp_sr_received_count": sr_received_count,
            }
        )
        clock_rates = self._clock_rates_by_ssrc.get(frame_timing.rtp_ssrc) or self._last_clock_rates
        if clock_rates is not None:
            metadata["rtcp_camera_ntp_rate"] = clock_rates["ntp_rate"]
            metadata["rtcp_camera_media_rate"] = clock_rates["media_rate"]
        metadata["rtcp_sr_arrival_basis"] = sr_timing.arrival_basis
        sr_pc_offset_s = None
        if sr_timing.received_wall_s is not None:
            sr_pc_offset_s = rtcp_sender_report_pc_offset_s(
                sr_ntp_unix_s=sr_timing.ntp_unix_s,
                received_wall_s=sr_timing.received_wall_s,
            )
            if sr_pc_offset_s is not None:
                metadata["rtcp_sr_pc_offset_s"] = sr_pc_offset_s
        if frame_ntp_s is not None:
            # An RTCP Sender Report can describe an older encoded frame. Its
            # arrival delay is a transport/encoder diagnostic, not proof that
            # the camera clock is wrong, so keep the RTP->NTP mapping intact.
            metadata["rtcp_frame_ntp_unix_s"] = frame_ntp_s
            metadata["rtcp_frame_received_wall_s"] = received_wall_s
            receive_delay_s = received_wall_s - frame_ntp_s
            metadata["rtcp_frame_receive_delay_s"] = receive_delay_s
            if sr_pc_offset_s is not None:
                # receive_delay carries the camera's absolute clock error, so it
                # grows without bound as that clock drifts. Adding the report's
                # own PC offset cancels the sr_ntp term, leaving
                #   (frame arrival - SR arrival) - (media time between them),
                # a transport/encoder figure that stays flat under clock drift.
                metadata["rtcp_frame_transport_delay_s"] = receive_delay_s + sr_pc_offset_s
        return metadata

    def _handle_new_manager(self, _source, manager) -> None:
        with self._lock:
            self._manager = manager
        try:
            manager.connect("pad-added", self._handle_manager_pad_added)
            iterator = manager.iterate_pads()
            while True:
                result, pad = iterator.next()
                if result != self.Gst.IteratorResult.OK:
                    break
                self._handle_manager_pad_added(manager, pad)
        except Exception as exc:
            self._emit_status(f"[RTCP] 실제 도착 probe 연결 실패, 통계 폴링 fallback: {exc}")

    def _handle_manager_pad_added(self, _manager, pad) -> None:
        try:
            pad_name = str(pad.get_name())
        except Exception:
            return
        if not pad_name.startswith("recv_rtcp_sink_") or pad_name in self._rtcp_probe_pads:
            return
        try:
            pad.add_probe(self.Gst.PadProbeType.BUFFER, self._capture_rtcp_packet)
        except Exception as exc:
            self._emit_status(f"[RTCP] {pad_name} probe 연결 실패, 통계 폴링 fallback: {exc}")
            return
        self._rtcp_probe_pads.add(pad_name)
        self._emit_status(f"[RTCP] 실제 Sender Report 도착시각 probe 연결: {pad_name}")

    def _remember_sender_report_arrival(
        self,
        *,
        ssrc: int,
        ntp_raw: int,
        received_monotonic_s: float,
        received_wall_s: float,
    ) -> None:
        key = (int(ssrc), int(ntp_raw))
        with self._lock:
            if key not in self._sr_arrival_by_key:
                self._sr_arrival_order.append(key)
            self._sr_arrival_by_key[key] = (float(received_monotonic_s), float(received_wall_s))
            while len(self._sr_arrival_by_key) > MAX_RTCP_SR_ARRIVAL_CACHE and self._sr_arrival_order:
                old_key = self._sr_arrival_order.popleft()
                self._sr_arrival_by_key.pop(old_key, None)

    def _sender_report_arrival(self, *, ssrc: int, ntp_raw: int) -> tuple[float, float] | None:
        with self._lock:
            return self._sr_arrival_by_key.get((int(ssrc), int(ntp_raw)))

    def _capture_rtcp_packet(self, _pad, info):
        buffer = info.get_buffer()
        if buffer is None:
            return self.Gst.PadProbeReturn.OK
        received_monotonic_s = time.monotonic()
        received_wall_s = time.time()
        rtcp_buffer = self.GstRtp.RTCPBuffer()
        try:
            success = self.GstRtp.RTCPBuffer.map(
                buffer,
                self.Gst.MapFlags.READ,
                rtcp_buffer,
            )
        except Exception:
            return self.Gst.PadProbeReturn.OK
        if not success:
            return self.Gst.PadProbeReturn.OK
        try:
            packet = self.GstRtp.RTCPPacket()
            if not rtcp_buffer.get_first_packet(packet):
                return self.Gst.PadProbeReturn.OK
            while True:
                if packet.get_type() == self.GstRtp.RTCPType.SR:
                    ssrc, ntp_raw, _rtp_timestamp, _packet_count, _octet_count = packet.sr_get_sender_info()
                    self._remember_sender_report_arrival(
                        ssrc=int(ssrc),
                        ntp_raw=int(ntp_raw),
                        received_monotonic_s=received_monotonic_s,
                        received_wall_s=received_wall_s,
                    )
                if not packet.move_to_next():
                    break
        except Exception:
            # Keep the stats-based path alive if a platform's GI bindings do
            # not expose one of the RTCP packet accessors.
            return self.Gst.PadProbeReturn.OK
        finally:
            rtcp_buffer.unmap()
        return self.Gst.PadProbeReturn.OK

    def _capture_rtp_packet(self, _pad, info):
        buffer = info.get_buffer()
        if buffer is None:
            return self.Gst.PadProbeReturn.OK
        success, rtp_buffer = self.GstRtp.RTPBuffer.map(buffer, self.Gst.MapFlags.READ)
        if not success:
            return self.Gst.PadProbeReturn.OK
        try:
            ssrc = int(rtp_buffer.get_ssrc())
            sequence = int(rtp_buffer.get_seq())
            rtp_pts_ns = None if buffer.pts == self.Gst.CLOCK_TIME_NONE else int(buffer.pts)
            self.register_rtp_sequence(
                ssrc=ssrc,
                sequence=sequence,
                rtp_pts_ns=rtp_pts_ns,
            )
            if rtp_buffer.get_marker():
                timing = RtpFrameTiming(
                    rtp_timestamp=int(rtp_buffer.get_timestamp()),
                    rtp_ssrc=ssrc,
                    rtp_marker_seq=sequence,
                    rtp_pts_s=(
                        None
                        if buffer.pts == self.Gst.CLOCK_TIME_NONE
                        else int(buffer.pts) / 1e9
                    ),
                )
                with self._lock:
                    self._pending_rtp_frames.append(timing)
        finally:
            rtp_buffer.unmap()
        return self.Gst.PadProbeReturn.OK

    def _observe_access_unit(self, _pad, info):
        buffer = info.get_buffer()
        if buffer is None:
            return self.Gst.PadProbeReturn.OK
        with self._lock:
            self._h264_access_units_total += 1
        flags = buffer.get_flags()
        pts_ns = None if buffer.pts == self.Gst.CLOCK_TIME_NONE else int(buffer.pts)
        is_delta = bool(flags & self.Gst.BufferFlags.DELTA_UNIT)
        is_corrupted = bool(flags & self.Gst.BufferFlags.CORRUPTED)
        is_discontinuous = bool(flags & self.Gst.BufferFlags.DISCONT)
        recovery_basis = None
        nal1_metadata = None
        map_info = None
        if self.encoded_output_blocked() or is_discontinuous or not is_delta:
            try:
                mapped, map_info = buffer.map(self.Gst.MapFlags.READ)
                if mapped:
                    nal1_metadata = h264_nal1_non_idr_intra_metadata(
                        map_info.data,
                        pts_ns=pts_ns,
                        is_delta=is_delta,
                        is_corrupted=is_corrupted,
                    )
                    recovery_basis = h264_access_unit_recovery_basis(
                        map_info.data,
                        is_delta=is_delta,
                        allow_non_idr_intra_experimental=(
                            self._allow_non_idr_intra_experimental
                        ),
                    )
            except Exception:
                # A boundary that cannot be inspected is not safe to recover on.
                recovery_basis = None
            finally:
                if map_info is not None:
                    try:
                        buffer.unmap(map_info)
                    except Exception:
                        pass
        if nal1_metadata is not None:
            self._remember_nal1_metadata(nal1_metadata)
        should_drop = self.should_drop_encoded_access_unit(
            pts_ns=pts_ns,
            is_delta=is_delta,
            is_corrupted=is_corrupted,
            is_discontinuous=is_discontinuous,
            is_random_access=bool(recovery_basis),
            recovery_basis=recovery_basis,
        )
        if should_drop:
            if self._allow_unverified_encoded_output:
                # Preview-only tolerant mode deliberately lets the decoder see
                # the same incomplete AUs that the initial StereoWIRIS path
                # exposed to its latest-frame consumer. The reader marks the
                # resulting BGR sample UNVERIFIED; recording remains strict.
                return self.Gst.PadProbeReturn.OK
            return self.Gst.PadProbeReturn.DROP
        if self.consume_encoded_recovery_marker(pts_ns):
            buffer.set_flags(self.Gst.BufferFlags.DISCONT)
        return self.Gst.PadProbeReturn.OK

    def _map_depay_buffer_pts(self, _pad, info):
        buffer = info.get_buffer()
        if buffer is None or buffer.pts == self.Gst.CLOCK_TIME_NONE:
            return self.Gst.PadProbeReturn.OK
        pts_ns = int(buffer.pts)
        pts_s = pts_ns / 1e9
        with self._lock:
            timing = self._pop_pending_rtp_timing_locked(pts_ns)
            if timing is None:
                return self.Gst.PadProbeReturn.OK
            timing = RtpFrameTiming(
                rtp_timestamp=timing.rtp_timestamp,
                rtp_ssrc=timing.rtp_ssrc,
                rtp_marker_seq=timing.rtp_marker_seq,
                rtp_pts_s=pts_s,
            )
            self._timing_by_pts_ns[pts_ns] = timing
            self._pts_order.append(pts_ns)
            self._trim_timing_cache_locked()
        return self.Gst.PadProbeReturn.OK

    def _pop_pending_rtp_timing_locked(self, pts_ns: int) -> RtpFrameTiming | None:
        """Match a depayloaded access unit to its RTP marker by PTS.

        A camera can emit marker packets that ``rtph264depay`` later merges or
        drops.  Consuming marker timings as a strict FIFO therefore leaves one
        extra entry behind whenever that happens; each extra entry makes every
        following frame appear one frame older and produces a steadily growing
        RTCP transit delay.  Packet and depay-buffer PTS are preserved by
        ``rtspsrc``, so discard unmatched older markers and take the closest
        matching PTS instead.
        """

        if not self._pending_rtp_frames:
            return None
        known_pts = [
            (index, int(round(float(timing.rtp_pts_s) * 1e9)))
            for index, timing in enumerate(self._pending_rtp_frames)
            if timing.rtp_pts_s is not None
        ]
        if not known_pts:
            # Keep compatibility with GStreamer builds that do not put a PTS
            # on RTP packet buffers.
            return self._pending_rtp_frames.popleft()

        match_index, match_pts_ns = min(
            known_pts,
            key=lambda item: abs(item[1] - int(pts_ns)),
        )
        if abs(match_pts_ns - int(pts_ns)) <= MAX_RTP_PTS_MATCH_DELTA_NS:
            for _ in range(match_index):
                self._pending_rtp_frames.popleft()
            return self._pending_rtp_frames.popleft()

        stale_before_ns = int(pts_ns) - MAX_RTP_PTS_MATCH_DELTA_NS
        while self._pending_rtp_frames:
            timing = self._pending_rtp_frames[0]
            if timing.rtp_pts_s is None:
                break
            timing_pts_ns = int(round(float(timing.rtp_pts_s) * 1e9))
            if timing_pts_ns >= stale_before_ns:
                break
            self._pending_rtp_frames.popleft()
        return None

    def _pop_frame_timing(self, pts_ns: int | None) -> RtpFrameTiming | None:
        with self._lock:
            if pts_ns is not None:
                timing = self._timing_by_pts_ns.pop(pts_ns, None)
                if timing is not None:
                    return timing
                timing = self._pop_nearest_timing_locked(pts_ns)
                if timing is not None:
                    return timing
                return None
            if self._timing_by_pts_ns:
                oldest_pts = min(self._timing_by_pts_ns)
                return self._timing_by_pts_ns.pop(oldest_pts, None)
            return None

    def _pop_nearest_timing_locked(self, pts_ns: int) -> RtpFrameTiming | None:
        if not self._timing_by_pts_ns:
            return None
        nearest_pts = min(self._timing_by_pts_ns, key=lambda candidate: abs(candidate - pts_ns))
        if abs(nearest_pts - pts_ns) > MAX_RTP_PTS_MATCH_DELTA_NS:
            return None
        return self._timing_by_pts_ns.pop(nearest_pts, None)

    def _trim_timing_cache_locked(self) -> None:
        while len(self._timing_by_pts_ns) > MAX_RTP_TIMING_CACHE and self._pts_order:
            old_pts = self._pts_order.popleft()
            self._timing_by_pts_ns.pop(old_pts, None)

    def _poll_rtcp_sender_reports(self, now_s: float) -> None:
        if now_s - self._last_rtcp_poll_s < 0.25:
            return
        self._last_rtcp_poll_s = now_s
        with self._lock:
            manager = self._manager
        if manager is None:
            return
        try:
            session = manager.emit("get-internal-session", 0)
            stats = session.get_property("stats")
            source_stats = stats.get_value("source-stats")
        except Exception:
            return
        if not isinstance(source_stats, list):
            return
        emitted_messages: list[str] = []
        for source in source_stats:
            if not bool(_gst_structure_value(source, "is-sender", False)):
                continue
            if not bool(_gst_structure_value(source, "have-sr", False)):
                continue
            ntp_raw = int(_gst_structure_value(source, "sr-ntptime", 0) or 0)
            sr_ntp_unix_s = ntp64_to_unix_seconds(ntp_raw)
            if sr_ntp_unix_s is None:
                continue
            ssrc = int(_gst_structure_value(source, "ssrc", 0) or 0)
            sr_rtp_timestamp = int(_gst_structure_value(source, "sr-rtptime", 0) or 0)
            clock_rate_hz = int(_gst_structure_value(source, "clock-rate", RTP_CLOCK_RATE_HZ) or RTP_CLOCK_RATE_HZ)
            timing = RtcpSenderReportTiming(
                ssrc=ssrc,
                ntp_raw=ntp_raw,
                ntp_unix_s=sr_ntp_unix_s,
                rtp_timestamp=sr_rtp_timestamp,
                clock_rate_hz=clock_rate_hz,
                received_monotonic_s=now_s,
                received_wall_s=time.time(),
            )
            arrival = self._sender_report_arrival(ssrc=ssrc, ntp_raw=ntp_raw)
            if arrival is not None:
                timing = RtcpSenderReportTiming(
                    ssrc=timing.ssrc,
                    ntp_raw=timing.ntp_raw,
                    ntp_unix_s=timing.ntp_unix_s,
                    rtp_timestamp=timing.rtp_timestamp,
                    clock_rate_hz=timing.clock_rate_hz,
                    received_monotonic_s=arrival[0],
                    received_wall_s=arrival[1],
                    arrival_basis="rtcp-pad-probe",
                )
            message = self._register_sender_report(timing)
            if message is not None:
                emitted_messages.append(message)
        for message in emitted_messages:
            self._emit_status(message)


def build_gst_pipeline_text(
    url: str,
    *,
    latency_ms: int = DEFAULT_RTSP_LATENCY_MS,
    decoder: str = NVIDIA_H264_DECODER,
    transport: str = RTSP_TRANSPORT_UDP,
    keyframe_only: bool = False,
    record_video_path: str = "",
    encoded_queue_mode: str = RTSP_ENCODED_QUEUE_MODE_LATEST,
) -> str:
    latency_ms = max(0, int(latency_ms))
    rtsp_transport = normalize_rtsp_transport(transport)
    decoder_element = decoder_element_text(decoder)
    record_path = str(record_video_path or "").strip()
    # The permanent camera contract is latest-only decoded input. RTP/IDR
    # integrity remains observable, but it never backpressures Preview, YOLO,
    # native evidence, or Measurement.
    h264parse_element = (
        "h264parse config-interval=-1 disable-passthrough=true name=parse "
        "! video/x-h264,stream-format=byte-stream,alignment=au"
    )
    if normalize_rtsp_encoded_queue_mode(encoded_queue_mode) == (
        RTSP_ENCODED_QUEUE_MODE_REFERENCE_SAFE
    ):
        encoded_queue = (
            "queue max-size-buffers=8 "
            "max-size-bytes=0 "
            "max-size-time=0"
        )
    else:
        encoded_queue = (
            "queue leaky=downstream "
            "max-size-buffers=1 "
            "max-size-bytes=0 "
            "max-size-time=0"
        )
    decoded_queue = (
        "queue leaky=downstream "
        f"max-size-buffers={GST_DECODED_QUEUE_MAX_BUFFERS} "
        "max-size-bytes=0 "
        "max-size-time=0"
    )
    keyframe_filter = ""
    if keyframe_only:
        keyframe_filter = (
            "! identity drop-buffer-flags=delta-unit silent=true "
            "! queue leaky=downstream max-size-buffers=1 max-size-bytes=0 max-size-time=0 "
        )
    pipeline = (
        f"rtspsrc name=src location={url} latency={latency_ms} protocols={rtsp_transport} "
        "drop-on-latency=true do-rtcp=true "
        "! rtph264depay name=depay source-info=true "
        f"! {encoded_queue} "
        f"! {h264parse_element} "
        f"{keyframe_filter}"
        f"! {encoded_queue} "
        f"! {decoder_element} "
        "! tee name=decoded "
        f"decoded. ! {decoded_queue} "
        "! videoconvert "
        "! video/x-raw,format=BGR "
        "! appsink name=sink emit-signals=false sync=false max-buffers=1 drop=true"
    )
    if record_path:
        pipeline += (
            " decoded. ! queue name=record_capture_queue leaky=downstream "
            "max-size-buffers=2 max-size-bytes=0 max-size-time=0 "
            "! videoconvert ! video/x-raw,format=NV12 "
            "! appsink name=record_sink emit-signals=false sync=false max-buffers=2 drop=true"
        )
    return pipeline


def _launch_gst_pipeline(
    Gst,
    url: str,
    *,
    latency_ms: int,
    decoder_mode: str,
    transport: str,
    keyframe_only: bool,
    record_video_path: str,
    output_queue,
    rtp_timing_tracker: GstRtpTimingTracker | None = None,
    encoded_queue_mode: str = RTSP_ENCODED_QUEUE_MODE_LATEST,
):
    last_error: str | None = None
    for decoder in decoder_candidates(decoder_mode):
        pipeline_text = build_gst_pipeline_text(
            url,
            latency_ms=latency_ms,
            decoder=decoder,
            transport=transport,
            keyframe_only=keyframe_only,
            record_video_path=record_video_path,
            encoded_queue_mode=encoded_queue_mode,
        )
        pipeline = None
        try:
            pipeline = Gst.parse_launch(pipeline_text)
            sink = pipeline.get_by_name("sink")
            bus = pipeline.get_bus()
            if sink is None or bus is None:
                raise RuntimeError("pipeline missing appsink or bus")
            if rtp_timing_tracker is not None:
                rtp_timing_tracker.attach(pipeline)
            state_result = pipeline.set_state(Gst.State.PLAYING)
            if state_result == Gst.StateChangeReturn.FAILURE:
                # PLAYING failure alone does not identify the failing element.
                # Do not label it as a decoder fault and hide an RTSP/filesink
                # failure behind an unnecessary CPU retry.
                raise RuntimeError("pipeline failed to enter PLAYING state")
            _emit_status(
                output_queue,
                f"GStreamer decoder candidate: {decoder} "
                "(PLAYING requested; current-generation frame pending)",
            )
            if record_video_path:
                _emit_status(
                    output_queue,
                    f"GStreamer decoded NV12 recording capture ready: {record_video_path}",
                )
            return pipeline, sink, bus, decoder
        except Exception as exc:
            last_error = f"{decoder}: {exc}"
            if pipeline is not None:
                pipeline.set_state(Gst.State.NULL)
            error_class = classify_gstreamer_error(
                source_name="",
                detail=exc,
                active_decoder=decoder,
            )
            fallback_reason = gstreamer_error_fallback_reason(
                requested_decoder=decoder_mode,
                active_decoder=decoder,
                already_fell_back=False,
                error_class=error_class,
                detail=exc,
            )
            if fallback_reason is not None:
                _emit_decoder_fallback(
                    output_queue,
                    reason=(
                        f"{fallback_reason}; software candidate pending "
                        "current-generation frame"
                    ),
                    recording_restart_required=False,
                )
                continue
            break
    raise RuntimeError(last_error or "no usable decoder")


def _finalize_gst_recording(
    Gst,
    pipeline,
    bus,
    output_queue,
    record_video_path: str,
    *,
    config: dict[str, Any] | None = None,
) -> bool:
    if not record_video_path:
        return True
    finalization_config = config or {}
    live_integrity_terminal_failure = bool(
        finalization_config.get("_integrity_manual_reconnect_required", False)
    )

    def finish(
        success: bool,
        detail: str,
        *,
        container_success: bool = False,
        validation: dict[str, Any] | None = None,
    ) -> bool:
        integrity = dict(validation or {})
        if not integrity:
            integrity = {
                "integrity_validation_status": "NOT_RUN",
                "integrity_failure_reasons": ["container_finalization_failed"],
                "recording_usable": False,
                "recording_integrity_verified": False,
            }
        _emit_recording_finalized(
            output_queue,
            path=record_video_path,
            backend=STREAM_BACKEND_GSTREAMER,
            success=success,
            detail=detail,
            contract=h264_copy_recording_integrity_contract(),
            metrics={
                "container_finalization_success": bool(container_success),
                "integrity_terminal_failure": (
                    "H264_IDR_RANDOM_ACCESS_UNAVAILABLE"
                    if live_integrity_terminal_failure
                    else ""
                ),
                "integrity_gap_events": int(
                    finalization_config.get("_integrity_gap_events", 0) or 0
                ),
                "integrity_missing_packets": int(
                    finalization_config.get("_integrity_missing_packets", 0) or 0
                ),
                "integrity_suppressed_decoded_frames": int(
                    finalization_config.get("_integrity_suppressed_decoded_frames", 0)
                    or 0
                ),
                **integrity,
            },
        )
        return success

    try:
        if not pipeline.send_event(Gst.Event.new_eos()):
            _emit_status(output_queue, f"GStreamer 녹화 EOS 전송 실패: {record_video_path}")
            return finish(False, "EOS send failed")
        deadline_s = time.monotonic() + GST_RECORDING_EOS_TIMEOUT_S
        while time.monotonic() < deadline_s:
            message = bus.timed_pop_filtered(
                200_000_000,
                Gst.MessageType.EOS | Gst.MessageType.ERROR,
            )
            if message is None:
                continue
            if message.type == Gst.MessageType.ERROR:
                error, debug = message.parse_error()
                detail = error.message if debug is None else f"{error.message} ({debug})"
                _emit_status(output_queue, f"GStreamer 녹화 종료 오류: {detail}")
                return finish(False, f"EOS bus error: {detail}")
            if message.type == Gst.MessageType.EOS:
                _emit_status(
                    output_queue,
                    f"GStreamer H264 원본 녹화 mux 저장 완료; "
                    f"최종 MP4 무결성 검증 중: {record_video_path}",
                )
                validator = finalization_config.get("_recording_integrity_validator")
                if callable(validator):
                    try:
                        validation = dict(validator(record_video_path))
                    except Exception as exc:
                        validation = {
                            "integrity_validation_status": "FAIL",
                            "integrity_failure_reasons": [
                                f"integrity_validator_exception:{type(exc).__name__}"
                            ],
                            "recording_usable": False,
                            "recording_integrity_verified": False,
                        }
                else:
                    # Full decode/header/timing validation is intentionally not
                    # run inside the camera child. EOS already consumes up to
                    # nine seconds of its twelve-second cooperative stop budget;
                    # the package worker validates the immutable bytes later.
                    validation = {
                        "integrity_schema": H264_MP4_INTEGRITY_SCHEMA,
                        "integrity_validation_status": "PENDING",
                        "integrity_failure_reasons": [
                            "finalized_mp4_package_validation_pending"
                        ],
                        "recording_usable": False,
                        "recording_integrity_verified": False,
                    }

                live_failures: list[str] = []
                if live_integrity_terminal_failure:
                    live_failures.append("live_h264_idr_random_access_unavailable")
                if int(finalization_config.get("_integrity_gap_events", 0) or 0) > 0:
                    live_failures.append("live_rtp_sequence_gap_detected")
                if int(finalization_config.get("_integrity_missing_packets", 0) or 0) > 0:
                    live_failures.append("live_rtp_missing_packet_detected")
                if int(
                    finalization_config.get("_integrity_suppressed_decoded_frames", 0)
                    or 0
                ) > 0:
                    live_failures.append("live_decoder_frame_suppression_detected")
                if live_failures:
                    validation["integrity_failure_reasons"] = list(
                        dict.fromkeys(
                            [
                                *tuple(validation.get("integrity_failure_reasons", ()) or ()),
                                *live_failures,
                            ]
                        )
                    )
                    validation.update(
                        integrity_validation_status="FAIL",
                        recording_usable=False,
                        recording_integrity_verified=False,
                    )

                usable = bool(
                    validation.get("integrity_schema") == H264_MP4_INTEGRITY_SCHEMA
                    and validation.get("integrity_validation_status") == "PASS"
                    and validation.get("recording_usable") is True
                    and validation.get("recording_integrity_verified") is True
                )
                if not usable:
                    validation_failures = list(
                        validation.get("integrity_failure_reasons", ()) or ()
                    )
                    if validation.get("integrity_schema") != H264_MP4_INTEGRITY_SCHEMA:
                        validation_failures.append("integrity_validation_schema_invalid")
                    if not validation_failures:
                        validation_failures.append("integrity_validation_incomplete")
                    validation.update(
                        integrity_validation_status="FAIL",
                        integrity_failure_reasons=list(
                            dict.fromkeys(validation_failures)
                        ),
                        recording_usable=False,
                        recording_integrity_verified=False,
                    )
                if usable:
                    detail = "EOS received; mp4mux finalized; MP4 integrity PASS"
                    _emit_status(
                        output_queue,
                        f"GStreamer H264 원본 녹화 무결성 PASS: {record_video_path}",
                    )
                else:
                    reasons = ",".join(
                        str(item)
                        for item in tuple(
                            validation.get("integrity_failure_reasons", ()) or ()
                        )
                    ) or "integrity_validation_incomplete"
                    detail = (
                        "EOS received; mp4mux finalized; MP4 integrity FAIL: "
                        f"{reasons}"
                    )
                    _emit_status(
                        output_queue,
                        f"GStreamer H264 원본 녹화 무결성 FAIL: {reasons}",
                    )
                return finish(
                    usable,
                    detail,
                    container_success=True,
                    validation=validation,
                )
        _emit_status(output_queue, f"GStreamer 녹화 EOS 대기 시간 초과: {record_video_path}")
        return finish(False, "EOS timeout")
    except Exception as exc:
        _emit_status(output_queue, f"GStreamer 녹화 종료 실패: {exc}")
        return finish(False, f"{type(exc).__name__}: {exc}")


def _load_yolo_model(config: dict[str, Any], output_queue, enable_detection: bool):
    if not enable_detection:
        return None
    if enable_detection:
        _configure_torch_threads(_worker_cpu_threads(config))
        try:
            from ultralytics import YOLO
        except Exception as exc:
            _emit_status(output_queue, f"YOLO 사용 불가: {exc}")
            return None
        _emit_status(output_queue, "YOLO 모델 로드 중")
        try:
            return YOLO(config["weights_path"])
        except Exception as exc:
            _emit_status(output_queue, f"모델 로드 오류: {exc}")
            return None


def _process_frame_for_output(
    cv2,
    frame,
    *,
    model,
    config: dict[str, Any],
    output_queue,
    now_s: float,
    enable_detection: bool,
    draw_preview_overlay: bool = False,
    timestamp_metadata: dict[str, Any] | None = None,
):
    height, width = frame.shape[:2]
    annotated = frame
    best_detection = None
    inferred = False
    if enable_detection:
        class_names = getattr(model, "names", None)
        configured_device = str(config.get("device") or "")
        predict_devices = [configured_device]
        if configured_device and configured_device.lower() != "cpu" and not config.get("_cpu_fallback_attempted"):
            predict_devices.append("cpu")
        for attempt_index, predict_device in enumerate(predict_devices):
            try:
                result = model.predict(
                    frame,
                    imgsz=config["input_size"],
                    conf=config["conf_threshold"],
                    iou=config["iou_threshold"],
                    device=predict_device or None,
                    verbose=False,
                )[0]
                active_device = (predict_device or "auto").strip() or "auto"
                requested_device = str(config.get("_requested_yolo_device") or configured_device or "").strip().lower()
                config["_active_yolo_device"] = active_device
                config["_yolo_cpu_fallback_active"] = active_device.lower() == "cpu" and requested_device not in {"", "cpu"}
                if attempt_index > 0:
                    config["device"] = "cpu"
                    config["_cpu_fallback_attempted"] = True
                    _emit_status(output_queue, "CUDA 추론 실패 후 CPU로 전환했습니다.")
                break
            except Exception as exc:
                if attempt_index + 1 < len(predict_devices):
                    config["_cpu_fallback_attempted"] = True
                    config["_yolo_cpu_fallback_active"] = False
                    _emit_status(output_queue, f"추론 오류({exc}); CPU로 재시도합니다.")
                    continue
                throttle_state = config.setdefault("_inference_error_throttle_state", {})
                if should_emit_throttled_error(throttle_state, "inference", str(exc), now_s):
                    _emit_status(output_queue, f"추론 오류: {exc}")
                output_queue.put(
                    (
                        "detection",
                        now_s,
                        width,
                        height,
                        None,
                        [],
                        dict(timestamp_metadata or {}),
                    )
                )
                return frame, None, False
        try:
            detections = filter_detections_by_class_gate(
                detections_from_ultralytics_result(result, names=class_names),
                enabled=bool(config.get("class_gate_enabled", False)),
                target=config.get("class_gate", ""),
            )
            best_detection = highest_confidence_detection(detections)
            if draw_preview_overlay and best_detection is not None:
                annotated = _draw_detections(cv2, frame, [best_detection], best_detection)
            inferred = True
            output_queue.put(
                (
                    "detection",
                    now_s,
                    width,
                    height,
                    best_detection.as_dict() if best_detection is not None else None,
                    [detection.as_dict() for detection in detections],
                    dict(timestamp_metadata or {}),
                )
            )
        except Exception as exc:
            _emit_status(output_queue, f"검출 결과 처리 오류: {exc}")
            return frame, None, False
    return annotated, best_detection, inferred


def _publish_native_frame_to_ring(
    frame,
    config: dict[str, Any],
    output_queue,
    *,
    timestamp_s: float,
    source_timestamp_s: float | None,
) -> dict[str, Any]:
    ring = config.get("_native_frame_ring")
    if not isinstance(ring, SharedNativeFrameRing):
        return {}
    try:
        descriptor = ring.publish_frame(
            frame,
            timestamp_s=timestamp_s,
            source_timestamp_s=source_timestamp_s,
        )
    except Exception as exc:
        throttle = config.setdefault("_native_ring_error_throttle", {})
        if should_emit_throttled_error(
            throttle,
            "native-ring-publish",
            str(exc),
            float(timestamp_s),
        ):
            _emit_status(output_queue, f"NATIVE_RING publish 실패: {exc}")
        return {
            "native_ring_available": False,
            "native_ring_unavailable_reason": f"publish-error:{type(exc).__name__}",
        }
    metadata = descriptor.to_mapping()
    metadata.update(
        {
            "source_native_frame_id": descriptor.key.text,
            "native_lossless_source": True,
            "overlay_applied": False,
        }
    )
    if not descriptor.available:
        throttle = config.setdefault("_native_ring_full_throttle", {})
        if should_emit_throttled_error(
            throttle,
            "native-ring-unavailable",
            descriptor.unavailable_reason,
            float(timestamp_s),
        ):
            _emit_status(
                output_queue,
                "BACKPRESSURE/DEGRADED: native ring frame unavailable "
                f"frame={descriptor.key.text}, reason={descriptor.unavailable_reason}",
            )
    return metadata


def _emit_preview_frame(
    cv2,
    frame,
    config: dict[str, Any],
    output_queue,
    *,
    timestamp_s: float,
    source_timestamp_s: float | None = None,
    timestamp_source: str = "app-receive",
    extra_metadata: dict[str, Any] | None = None,
    force_emit: bool = False,
) -> bool:
    native_height, native_width = frame.shape[:2]
    if not force_emit and not _should_emit_preview_frame(config, timestamp_s):
        return False
    display_frame = frame
    max_preview_width = int(config["preview_max_width"])
    if max_preview_width > 0 and display_frame.shape[1] > max_preview_width:
        scale = max_preview_width / float(display_frame.shape[1])
        display_frame = cv2.resize(
            display_frame,
            (max_preview_width, max(1, int(round(display_frame.shape[0] * scale)))),
            interpolation=cv2.INTER_AREA,
        )
    rgb = cv2.cvtColor(display_frame, cv2.COLOR_BGR2RGB)
    metadata = {
        "timestamp_s": float(timestamp_s),
        "source_timestamp_s": None if source_timestamp_s is None else float(source_timestamp_s),
        "timestamp_source": timestamp_source,
        "native_width": int(native_width),
        "native_height": int(native_height),
        "preview_width": int(rgb.shape[1]),
        "preview_height": int(rgb.shape[0]),
    }
    if extra_metadata:
        metadata.update(extra_metadata)
    metadata.setdefault(
        "timestamp_reference",
        "rtcp-invalid"
        if metadata.get("rtcp_timing_invalid_reason")
        and not is_sr_pc_diagnostic_reason(metadata.get("rtcp_timing_invalid_reason"))
        else "rtcp-ntp"
        if metadata.get("rtcp_frame_ntp_unix_s") is not None
        else "source"
        if metadata.get("source_timestamp_s") is not None
        else "app-receive",
    )
    output_queue.put(("frame", metadata, rgb.shape[1], rgb.shape[0], rgb.tobytes()))
    return True


def _emit_measurement_frame(
    cv2,
    frame,
    config: dict[str, Any],
    output_queue,
    *,
    timestamp_s: float,
    timestamp_metadata: dict[str, Any] | None = None,
) -> None:
    rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
    metadata = dict(timestamp_metadata or {})
    metadata.update(
        {
            "timestamp_s": float(timestamp_s),
            "native_width": int(rgb.shape[1]),
            "native_height": int(rgb.shape[0]),
            "native_lossless_source": True,
            "overlay_applied": False,
            "source_native_frame_id": (
                "decoded-"
                f"{int(round(float(timestamp_s) * 1_000_000_000.0)):019d}"
            ),
        }
    )
    output_queue.put(
        (
            "measurement_frame",
            metadata,
            rgb.shape[1],
            rgb.shape[0],
            rgb.tobytes(),
        )
    )


def _emit_stream_status(output_queue, enable_detection: bool, best_detection, now_s: float, last_status_s: float) -> float:
    if now_s - last_status_s < 1.0:
        return last_status_s
    if not enable_detection:
        _emit_status(output_queue, "카메라 스트림 실행 중")
    elif best_detection is None:
        _emit_status(output_queue, "YOLO 실행 중: 검출 없음")
    else:
        center_x, center_y = best_detection.center
        class_label = best_detection.class_name or f"class {best_detection.class_id}"
        _emit_status(
            output_queue,
            "YOLO 실행 중: "
            f"최고 class={class_label} "
            f"신뢰도={best_detection.confidence:.2f} "
            f"중심=({center_x:.1f}, {center_y:.1f}) px",
        )
    return now_s


def _yolo_device_stats(config: dict[str, Any]) -> tuple[str, str, bool]:
    requested_device = str(config.get("_requested_yolo_device") or config.get("device") or "").strip()
    active_device = str(config.get("_active_yolo_device") or "").strip()
    cpu_fallback = bool(config.get("_yolo_cpu_fallback_active", False))
    return requested_device, active_device, cpu_fallback


def _run_gstreamer_stream_once(
    config: dict[str, Any],
    output_queue,
    stop_event,
    cv2,
    model,
    enable_detection: bool,
) -> bool:
    try:
        import gi
        import numpy as np

        gi.require_version("Gst", "1.0")
        gi.require_version("GstRtp", "1.0")
        from gi.repository import Gst
        from gi.repository import GstRtp
    except Exception as exc:
        _emit_status(output_queue, f"GStreamer 사용 불가: {exc}")
        return False

    Gst.init(None)
    keyframe_only = False
    encoded_queue_mode = normalize_rtsp_encoded_queue_mode(
        config.get("rtsp_encoded_queue_mode", RTSP_ENCODED_QUEUE_MODE_LATEST)
    )
    tolerant_input = True
    record_video_path = _record_video_path(config)
    config["_media_input_policy"] = "tolerant_decoded"
    config["_integrity_recovery_mode"] = "tolerant_decoded"
    rtp_timing_tracker = GstRtpTimingTracker(
        Gst,
        GstRtp,
        status_callback=lambda text: _emit_status(output_queue, text),
        allow_non_idr_intra_experimental=False,
        allow_unverified_encoded_output=True,
    )
    config.update(recording_integrity_contract(STREAM_BACKEND_GSTREAMER))
    config["_integrity_gap_events"] = 0
    config["_integrity_missing_packets"] = 0
    config["_integrity_suppressed_decoded_frames"] = 0
    config.pop("_integrity_manual_reconnect_required", None)
    if record_video_path:
        try:
            Path(record_video_path).expanduser().parent.mkdir(parents=True, exist_ok=True)
        except OSError as exc:
            _emit_status(output_queue, f"GStreamer MP4 저장 경로 준비 실패: {exc}")
            return False
    stream_failed = False
    launch_decoder_mode = str(config.get("decoder", DECODER_AUTO))
    config.setdefault(
        "_requested_decoder_mode",
        normalize_decoder_mode(launch_decoder_mode),
    )
    try:
        pipeline, sink, bus, active_decoder = _launch_gst_pipeline(
            Gst,
            config["url"],
            latency_ms=int(config.get("rtsp_latency_ms", DEFAULT_RTSP_LATENCY_MS)),
            decoder_mode=launch_decoder_mode,
            transport=str(config.get("rtsp_transport", RTSP_TRANSPORT_UDP)),
            keyframe_only=keyframe_only,
            record_video_path=record_video_path,
                output_queue=output_queue,
                rtp_timing_tracker=rtp_timing_tracker,
                encoded_queue_mode=encoded_queue_mode,
            )
    except Exception as exc:
        _emit_status(output_queue, f"GStreamer 오류: {exc}")
        return False
    if (
        normalize_decoder_mode(launch_decoder_mode) == DECODER_AUTO
        and active_decoder == SOFTWARE_H264_DECODER
    ):
        # _launch_gst_pipeline authorizes this only for a classified NVDEC
        # launch failure. Keep later reconnects on the proven per-process
        # fallback instead of retrying NVDEC forever.
        config["_runtime_decoder_fallback_active"] = True
        config["decoder"] = DECODER_SOFTWARE

    rate_limiter = AdaptiveYoloRateLimiter.from_config(config, enable_detection=enable_detection)
    next_frame_s = 0.0
    last_status_s = 0.0
    last_stats_s = time.monotonic()
    displayed_frames = 0
    inferred_frames = 0
    latest_lock = threading.Lock()
    latest_frame: dict[str, Any] | None = None
    latest_sequence = 0
    decoded_rate = RollingFrameRateEstimator()
    latest_width = 0
    latest_height = 0
    stream_started_s = time.monotonic()
    rtp_packet_rate = RollingCounterRateEstimator()
    rtp_missing_rate = RollingCounterRateEstimator()
    rtp_gap_event_rate = RollingCounterRateEstimator()
    h264_access_unit_rate = RollingCounterRateEstimator()
    for estimator in (
        rtp_packet_rate,
        rtp_missing_rate,
        rtp_gap_event_rate,
        h264_access_unit_rate,
    ):
        estimator.add(stream_started_s, 0)
    last_frame_received_s: float | None = None
    try:
        frame_timeout_s = float(
            config.get("rtsp_frame_timeout_s", DEFAULT_RTSP_FRAME_TIMEOUT_S)
        )
    except (TypeError, ValueError):
        frame_timeout_s = DEFAULT_RTSP_FRAME_TIMEOUT_S
    if not math.isfinite(frame_timeout_s) or frame_timeout_s <= 0.0:
        frame_timeout_s = DEFAULT_RTSP_FRAME_TIMEOUT_S
    try:
        integrity_recovery_timeout_s = float(
            config.get("integrity_recovery_timeout_s", INTEGRITY_RECOVERY_TIMEOUT_S)
        )
    except (TypeError, ValueError):
        integrity_recovery_timeout_s = INTEGRITY_RECOVERY_TIMEOUT_S
    if not math.isfinite(integrity_recovery_timeout_s) or integrity_recovery_timeout_s <= 0.0:
        integrity_recovery_timeout_s = INTEGRITY_RECOVERY_TIMEOUT_S
    config["_gst_last_run_received_frame"] = False
    reader_stop = threading.Event()
    suppressed_decoded_frames = 0
    reported_suppressed_frames = 0
    decoder_frame_announced = False
    rtp_gap_reporter = RtpDiscontinuityReporter()
    record_sink = pipeline.get_by_name("record_sink") if record_video_path else None
    record_reader_stop = threading.Event()
    decoded_recorder: DecodedNv12Recorder | None = None
    recorder_start_error = ""

    def recording_loop() -> None:
        nonlocal decoded_recorder, recorder_start_error
        if record_sink is None:
            recorder_start_error = "record appsink is unavailable"
            return
        while not stop_event.is_set() and not record_reader_stop.is_set():
            sample = record_sink.emit("try-pull-sample", 200_000_000)
            if sample is None:
                continue
            buffer = sample.get_buffer()
            if decoded_recorder is None and not recorder_start_error:
                try:
                    structure = sample.get_caps().get_structure(0)
                    sample_duration_ns = (
                        None
                        if buffer.duration == Gst.CLOCK_TIME_NONE
                        else int(buffer.duration)
                    )
                    recording_fps = (
                        _record_video_fps(config)
                        if sample_duration_ns is None or sample_duration_ns <= 0
                        else 1_000_000_000.0 / sample_duration_ns
                    )
                    decoded_recorder = DecodedNv12Recorder(
                        Gst,
                        path=record_video_path,
                        width=int(structure.get_value("width")),
                        height=int(structure.get_value("height")),
                        fallback_fps=recording_fps,
                        output_queue=output_queue,
                    )
                except Exception as exc:
                    recorder_start_error = f"{type(exc).__name__}: {exc}"
                    _emit_status(
                        output_queue,
                        f"Decoded NVENC recording unavailable; camera/YOLO continue: "
                        f"{recorder_start_error}",
                    )
                    continue
            if decoded_recorder is not None:
                decoded_recorder.submit(
                    buffer,
                    observed_monotonic_ns=time.monotonic_ns(),
                )

    record_reader = (
        threading.Thread(
            target=recording_loop,
            name="stereo-gst-nv12-recorder",
            daemon=True,
        )
        if record_video_path
        else None
    )
    if record_reader is not None:
        record_reader.start()

    def reader_loop() -> None:
        nonlocal displayed_frames, last_frame_received_s
        nonlocal latest_frame, latest_height, latest_sequence, latest_width
        nonlocal decoder_frame_announced, suppressed_decoded_frames
        while not stop_event.is_set() and not reader_stop.is_set():
            sample = sink.emit("try-pull-sample", 200_000_000)
            if sample is None:
                continue
            receive_monotonic_ns, receive_s = app_receive_monotonic_sample()
            buffer = sample.get_buffer()
            decoded_pts_ns = None if buffer.pts == Gst.CLOCK_TIME_NONE else int(buffer.pts)
            initially_integrity_latched = rtp_timing_tracker.should_suppress_decoded_buffer(
                decoded_pts_ns
            )
            if should_suppress_decoded_output(
                active_decoder=active_decoder,
                rtp_discontinuity_latched=initially_integrity_latched,
                tolerant_input=tolerant_input,
            ):
                with latest_lock:
                    latest_frame = None
                suppressed_decoded_frames += 1
                continue
            with latest_lock:
                decoded_rate.add(receive_s)
            receive_wall_s = time.time()
            caps = sample.get_caps().get_structure(0)
            width = int(caps.get_value("width"))
            height = int(caps.get_value("height"))
            source_timestamp_s = None if buffer.pts == Gst.CLOCK_TIME_NONE else buffer.pts / 1e9
            rtp_timing_metadata = rtp_timing_tracker.frame_metadata(
                buffer,
                receive_s,
                received_wall_s=receive_wall_s,
            ) or {}
            rtp_timing_metadata.update(
                {
                    "app_receive_monotonic_ns": receive_monotonic_ns,
                    "app_receive_timestamp_role": "authoritative_distance",
                    "decoded_buffer_pts_ns": decoded_pts_ns,
                    "active_decoder": active_decoder,
                    "requested_decoder": _requested_decoder_mode(config),
                }
            )
            success, map_info = buffer.map(Gst.MapFlags.READ)
            if not success:
                continue
            try:
                frame = np.frombuffer(map_info.data, dtype=np.uint8).reshape((height, width, 3)).copy()
            finally:
                buffer.unmap(map_info)
            # A gap can be observed by the RTP probe while this buffer is being
            # copied.  Check the latch again before publishing to any consumer.
            after_copy_integrity_latched = rtp_timing_tracker.should_suppress_decoded_buffer(
                decoded_pts_ns
            )
            if should_suppress_decoded_output(
                active_decoder=active_decoder,
                rtp_discontinuity_latched=after_copy_integrity_latched,
                tolerant_input=tolerant_input,
            ):
                with latest_lock:
                    latest_frame = None
                suppressed_decoded_frames += 1
                continue
            unverified_tolerant_frame = (
                initially_integrity_latched or after_copy_integrity_latched
            )
            rtp_timing_metadata.update(
                {
                    "media_integrity_mode": "tolerant_decoded",
                    "media_integrity_unverified": unverified_tolerant_frame,
                    "source_media_integrity_verified": (
                        not unverified_tolerant_frame
                    ),
                    "rtp_integrity_latched": unverified_tolerant_frame,
                    "media_random_access_guaranteed": (
                        not unverified_tolerant_frame
                        and rtp_timing_tracker.current_output_recovery_basis()
                        == H264_RECOVERY_BASIS_IDR
                    ),
                    "measurement_input_admitted": True,
                    "overlay_applied": False,
                }
            )
            rtp_timing_metadata.update(
                _publish_native_frame_to_ring(
                    frame,
                    config,
                    output_queue,
                    timestamp_s=receive_s,
                    source_timestamp_s=source_timestamp_s,
                )
            )
            preview_emitted = _emit_preview_frame(
                cv2,
                frame,
                config,
                output_queue,
                timestamp_s=receive_s,
                source_timestamp_s=source_timestamp_s,
                timestamp_source="gstreamer-pts",
                extra_metadata=rtp_timing_metadata,
                force_emit=bool(
                    rtp_timing_metadata.get("h264_nal1_non_idr_intra") is True
                ),
            )
            if preview_emitted and not decoder_frame_announced:
                decoder_frame_announced = True
                _emit_status(
                    output_queue,
                    (
                        f"GStreamer decoded frame admitted (INPUT UNVERIFIED): {active_decoder}"
                        if unverified_tolerant_frame
                        else f"GStreamer decoder frame verified: {active_decoder}"
                    ),
                )
            with latest_lock:
                latest_sequence += 1
                displayed_frames += int(preview_emitted)
                latest_width = width
                latest_height = height
                last_frame_received_s = receive_s
                config["_gst_last_run_received_frame"] = True
                latest_frame = {
                    "sequence": latest_sequence,
                    "frame": frame,
                    "timestamp_s": receive_s,
                    "source_timestamp_s": source_timestamp_s,
                    "rtp_timing_metadata": rtp_timing_metadata,
                    "width": width,
                    "height": height,
                }

    reader = threading.Thread(target=reader_loop, name="stereo-gst-reader", daemon=True)
    reader.start()
    last_rtp_recovery_s: float | None = None
    if keyframe_only:
        _emit_status(output_queue, "GStreamer I-frame only 진단 모드")
    recording_text = (
        ", decoded NV12->lossless NVENC MP4"
        if record_video_path
        else ""
    )
    encoded_recovery_boundary_text = "RTP/H264 provenance retained; "
    pending_recovery_text = (
        "TOLERANT INPUT: RTP/H264 integrity unverified; decoded "
        "Preview/YOLO/Measurement/recording continue "
    )
    _emit_status(
        output_queue,
        "Permanent tolerant decoded input enabled: Preview, YOLO, native "
        "evidence, Measurement, and decoded recording continue while source "
        "media provenance remains INPUT UNVERIFIED"
        f"{recording_text}",
    )
    last_processed_sequence = 0
    try:
        while not stop_event.is_set():
            while True:
                h264_discontinuity = rtp_timing_tracker.pop_h264_access_unit_discontinuity()
                if h264_discontinuity is None:
                    break
                _emit_status(
                    output_queue,
                    "RECOVERING_INTEGRITY: H264 access-unit discontinuity "
                    f"reason={h264_discontinuity.reason}, pts_ns={h264_discontinuity.pts_ns}; "
                    + encoded_recovery_boundary_text
                    + "decoded Preview/YOLO/Measurement/recording continue as INPUT UNVERIFIED",
                )
            while True:
                rtp_recovery = rtp_timing_tracker.pop_rtp_recovery()
                if rtp_recovery is None:
                    break
                last_rtp_recovery_s = time.monotonic()
                newly_suppressed_frames = max(
                    0,
                    suppressed_decoded_frames - reported_suppressed_frames,
                )
                reported_suppressed_frames = suppressed_decoded_frames
                config["_integrity_suppressed_decoded_frames"] = suppressed_decoded_frames
                if (
                    rtp_recovery.recovery_basis
                    == H264_RECOVERY_BASIS_NON_IDR_INTRA_EXPERIMENTAL
                ):
                    prefix = (
                        "EXPERIMENTAL NON-IDR INTRA: exact decoded PTS에서 출력 복구; "
                        "IDR/keyframe 보장 없음; direct-copy MP4 integrity 미검증 "
                    )
                else:
                    prefix = "HEALTHY: H264 IDR random-access + exact decoded PTS에서 출력 복구 "
                _emit_status(
                    output_queue,
                    prefix
                    + f"ssrc={rtp_recovery.ssrc}, pts_ns={rtp_recovery.resume_pts_ns}, "
                    f"basis={rtp_recovery.recovery_basis}, "
                    f"blocked={rtp_recovery.blocked_duration_s:.3f}s, "
                    f"suppressed={newly_suppressed_frames}",
                )
            while True:
                rtp_discontinuity = rtp_timing_tracker.pop_rtp_discontinuity()
                if rtp_discontinuity is None:
                    break
                now_s = time.monotonic()
                if rtp_gap_reporter.total_events == 0:
                    _emit_status(
                        output_queue,
                        f"RECOVERING_INTEGRITY: {format_rtp_sequence_discontinuity(rtp_discontinuity)}; "
                        + encoded_recovery_boundary_text
                        + "decoded Preview/YOLO/Measurement/recording continue as INPUT UNVERIFIED",
                    )
                summary = rtp_gap_reporter.observe(rtp_discontinuity, now_s=now_s)
                config["_integrity_gap_events"] = rtp_gap_reporter.total_events
                config["_integrity_missing_packets"] = rtp_gap_reporter.total_missing_packets
                if summary is not None:
                    _emit_status(output_queue, f"RECOVERING_INTEGRITY: {summary}")
            message = bus.pop_filtered(Gst.MessageType.ERROR | Gst.MessageType.EOS)
            if message is not None:
                if message.type == Gst.MessageType.ERROR:
                    error, debug = message.parse_error()
                    detail = error.message if debug is None else f"{error.message} ({debug})"
                    stream_failed = True
                    source_name = _gstreamer_message_source_name(message)
                    error_class = classify_gstreamer_error(
                        source_name=source_name,
                        detail=detail,
                        active_decoder=active_decoder,
                    )
                    fallback_reason = gstreamer_error_fallback_reason(
                        requested_decoder=_requested_decoder_mode(config),
                        active_decoder=active_decoder,
                        already_fell_back=bool(config.get("_runtime_decoder_fallback_active")),
                        error_class=error_class,
                        detail=detail,
                    )
                    if fallback_reason is not None:
                        config["_runtime_decoder_fallback_reason"] = fallback_reason
                        _emit_status(
                            output_queue,
                            "NVIDIA decoder-health failure: "
                            f"source={source_name or '--'}; {detail}",
                        )
                    elif error_class == GST_ERROR_DECODER_HEALTH:
                        config["_decoder_manual_restart_required"] = True
                        if (
                            _requested_decoder_mode(config) == DECODER_NVIDIA
                            and active_decoder == NVIDIA_H264_DECODER
                        ):
                            _emit_status(
                                output_queue,
                                "NVIDIA 디코더 오류: decoder-health failure; "
                                "software fallback 금지, Auto로 변경하세요. "
                                f"source={source_name or '--'}; {detail}",
                            )
                        else:
                            _emit_status(
                                output_queue,
                                "디코더 오류: decoder-health failure; "
                                f"source={source_name or '--'}; {detail}",
                            )
                    elif error_class == GST_ERROR_RTSP_SOURCE:
                        _emit_status(
                            output_queue,
                            f"RTSP/source 오류: source={source_name or '--'}; {detail}",
                        )
                    elif error_class == GST_ERROR_RECORDING_OUTPUT:
                        config["_recording_output_failure"] = detail
                        _emit_status(
                            output_queue,
                            "MP4 mux/filesink 오류: "
                            f"source={source_name or '--'}; {detail}",
                        )
                    else:
                        _emit_status(
                            output_queue,
                            f"GStreamer pipeline 오류: source={source_name or '--'}; {detail}",
                        )
                else:
                    stream_failed = True
                    _emit_status(output_queue, "스트림 종료")
                break

            now_s = time.monotonic()
            integrity_recovery_pending = rtp_timing_tracker.decoded_output_blocked()
            integrity_blocked_s = rtp_timing_tracker.blocked_duration_s(now_s=now_s)
            if integrity_recovery_pending and integrity_blocked_s >= integrity_recovery_timeout_s:
                config["_integrity_suppressed_decoded_frames"] = suppressed_decoded_frames
                summary = rtp_gap_reporter.flush(now_s=now_s)
                if summary is not None:
                    _emit_status(output_queue, f"RECOVERING_INTEGRITY: {summary}")
                if not config.get("_tolerant_integrity_timeout_reported"):
                    _emit_status(
                        output_queue,
                        "TOLERANT INPUT: integrity remained unverified for "
                        f"{integrity_blocked_s:.1f}s; all decoded consumers continue",
                    )
                    config["_tolerant_integrity_timeout_reported"] = True
            with latest_lock:
                last_receive_s = last_frame_received_s
            frame_age_s = now_s - (
                stream_started_s if last_receive_s is None else last_receive_s
            )
            recent_input = getattr(rtp_timing_tracker, "has_recent_rtp_input", None)
            try:
                encoded_input_active = bool(
                    recent_input(now_s=now_s, max_age_s=2.0)
                ) if callable(recent_input) else False
            except (TypeError, ValueError):
                encoded_input_active = False
            fallback_reason = runtime_decoder_fallback_reason(
                requested_decoder=_requested_decoder_mode(config),
                active_decoder=active_decoder,
                started_s=stream_started_s,
                last_frame_s=last_receive_s,
                now_s=now_s,
                already_fell_back=bool(config.get("_runtime_decoder_fallback_active")),
                rtp_recovery_pending=integrity_recovery_pending,
                last_recovery_s=last_rtp_recovery_s,
                encoded_input_active=encoded_input_active,
            )
            if fallback_reason is not None:
                stream_failed = True
                config["_runtime_decoder_fallback_reason"] = fallback_reason
                break
            if _requested_decoder_mode(config) == DECODER_NVIDIA:
                strict_reason = nvidia_decoder_health_failure_reason(
                    active_decoder=active_decoder,
                    started_s=stream_started_s,
                    last_frame_s=last_receive_s,
                    now_s=now_s,
                    already_fell_back=False,
                    rtp_recovery_pending=integrity_recovery_pending,
                    last_recovery_s=last_rtp_recovery_s,
                    encoded_input_active=encoded_input_active,
                )
                if strict_reason is not None:
                    stream_failed = True
                    config["_decoder_manual_restart_required"] = True
                    _emit_status(
                        output_queue,
                        f"NVIDIA 디코더 오류: {strict_reason}; software fallback 금지, Auto로 변경하세요.",
                    )
                    break
            if frame_age_s >= frame_timeout_s:
                stream_failed = True
                phase = "첫 디코드 프레임" if last_receive_s is None else "새 디코드 프레임"
                _emit_status(
                    output_queue,
                    f"RTSP 프레임 타임아웃: {phase}이 {frame_age_s:.1f}s 동안 없습니다.",
                )
                break
            if now_s - last_stats_s >= 1.0:
                elapsed_s = now_s - last_stats_s
                with latest_lock:
                    interval_displayed_frames = displayed_frames
                    displayed_frames = 0
                    stats_width = latest_width
                    stats_height = latest_height
                    decoded_hz = decoded_rate.rate(now_s)
                displayed_hz = interval_displayed_frames / elapsed_s
                inferred_hz = inferred_frames / elapsed_s
                metrics_snapshot_reader = getattr(
                    rtp_timing_tracker,
                    "rtp_metrics_snapshot",
                    None,
                )
                try:
                    transport_snapshot = (
                        dict(metrics_snapshot_reader())
                        if callable(metrics_snapshot_reader)
                        else {}
                    )
                except (AttributeError, TypeError, ValueError):
                    transport_snapshot = {}

                def counter_value(key: str) -> int:
                    try:
                        return max(0, int(transport_snapshot.get(key, 0) or 0))
                    except (TypeError, ValueError):
                        return 0

                rtp_packets_total = counter_value("rtp_packets_received_total")
                rtp_missing_total = counter_value("rtp_missing_packets_total")
                rtp_gap_events_total = counter_value("rtp_gap_events_total")
                h264_access_units_total = counter_value("h264_access_units_total")
                rtp_packet_rate.add(now_s, rtp_packets_total)
                rtp_missing_rate.add(now_s, rtp_missing_total)
                rtp_gap_event_rate.add(now_s, rtp_gap_events_total)
                h264_access_unit_rate.add(now_s, h264_access_units_total)
                rtp_packets_hz = rtp_packet_rate.rate(now_s)
                rtp_missing_hz = rtp_missing_rate.rate(now_s)
                rtp_loss_denominator = rtp_packets_hz + rtp_missing_hz
                rtp_loss_pct = (
                    100.0 * rtp_missing_hz / rtp_loss_denominator
                    if rtp_loss_denominator > 0.0
                    else None
                )
                stream_metrics = {
                    "rtp_packets_hz": rtp_packets_hz,
                    "rtp_missing_hz": rtp_missing_hz,
                    "rtp_loss_pct": rtp_loss_pct,
                    "rtp_gap_events_hz": rtp_gap_event_rate.rate(now_s),
                    "h264_au_hz": h264_access_unit_rate.rate(now_s),
                    "rtp_packets_received_total": rtp_packets_total,
                    "rtp_missing_packets_total": rtp_missing_total,
                    "rtp_gap_events_total": rtp_gap_events_total,
                    "h264_access_units_total": h264_access_units_total,
                    "rtp_active_ssrc": transport_snapshot.get("rtp_active_ssrc"),
                    "rtp_metrics_window_s": DEFAULT_DECODE_RATE_WINDOW_S,
                    "rtp_metrics_available": callable(metrics_snapshot_reader),
                }
                output_queue.put(
                    (
                        "stats",
                        decoded_hz,
                        displayed_hz,
                        inferred_hz,
                        stats_width,
                        stats_height,
                        rate_limiter.target_rate_hz,
                        *_yolo_device_stats(config),
                        stream_metrics,
                    )
                )
                rate_limiter.update(decoded_hz=decoded_hz, inferred_hz=inferred_hz)
                inferred_frames = 0
                last_stats_s = now_s
            with latest_lock:
                frame_record = None if latest_frame is None else dict(latest_frame)
            if frame_record is None:
                if now_s - last_status_s > 1.0:
                    if integrity_recovery_pending:
                        _emit_status(
                            output_queue,
                            pending_recovery_text
                            + f"({integrity_blocked_s:.1f}/{integrity_recovery_timeout_s:.0f}s)",
                        )
                    else:
                        _emit_status(output_queue, "프레임 수신 대기 중")
                    last_status_s = now_s
                time.sleep(0.02)
                continue
            sequence = int(frame_record["sequence"])
            if sequence == last_processed_sequence:
                time.sleep(0.005)
                continue
            if now_s < next_frame_s:
                last_processed_sequence = sequence
                time.sleep(min(0.02, max(0.0, next_frame_s - now_s)))
                continue
            last_processed_sequence = sequence
            next_frame_s = now_s + rate_limiter.interval_s

            frame = frame_record["frame"]
            source_timestamp_s = frame_record["source_timestamp_s"]
            rtp_timing_metadata = dict(frame_record["rtp_timing_metadata"])
            detection_timestamp_metadata = {
                "timestamp_s": float(frame_record["timestamp_s"]),
                "source_timestamp_s": None if source_timestamp_s is None else float(source_timestamp_s),
                "timestamp_source": "gstreamer-pts",
            }
            detection_timestamp_metadata.update(rtp_timing_metadata)
            detection_timestamp_metadata.setdefault(
                "timestamp_reference",
                "rtcp-invalid"
                if detection_timestamp_metadata.get("rtcp_timing_invalid_reason")
                and not is_sr_pc_diagnostic_reason(
                    detection_timestamp_metadata.get("rtcp_timing_invalid_reason")
                )
                else "rtcp-ntp"
                if detection_timestamp_metadata.get("rtcp_frame_ntp_unix_s") is not None
                else "source"
                if detection_timestamp_metadata.get("source_timestamp_s") is not None
                else "app-receive",
            )
            annotated, best_detection, inferred = _process_frame_for_output(
                cv2,
                frame,
                model=model,
                config=config,
                output_queue=output_queue,
                now_s=float(frame_record["timestamp_s"]),
                enable_detection=enable_detection,
                draw_preview_overlay=True,
                timestamp_metadata=detection_timestamp_metadata,
            )
            if annotated is None:
                time.sleep(0.2)
                continue
            if config.get("_yolo_cpu_fallback_active"):
                _activate_cpu_fallback_rate_limiter(rate_limiter, config)
            if inferred:
                inferred_frames += 1
            last_status_s = _emit_stream_status(output_queue, enable_detection, best_detection, now_s, last_status_s)

    finally:
        reader_stop.set()
        reader.join(timeout=1.0)
        record_reader_stop.set()
        if record_reader is not None:
            record_reader.join(timeout=1.0)
        config["_integrity_suppressed_decoded_frames"] = suppressed_decoded_frames
        if decoded_recorder is not None:
            _emit_recording_finalized(
                output_queue,
                path=record_video_path,
                backend=STREAM_BACKEND_GSTREAMER,
                success=False,
                detail="decoded NVENC recorder finalization started",
                contract=recording_integrity_contract(STREAM_BACKEND_GSTREAMER),
                metrics={
                    "container_finalization_success": False,
                    "source_media_integrity_verified": False,
                    "recording_file_integrity_verified": False,
                    "integrity_validation_status": "FINALIZING",
                    "integrity_failure_reasons": [],
                    "recording_usable": False,
                },
            )
            decoded_recorder.close()
        elif record_video_path:
            _emit_recording_finalized(
                output_queue,
                path=record_video_path,
                backend=STREAM_BACKEND_GSTREAMER,
                success=False,
                detail=recorder_start_error or "no decoded NV12 recording frame",
                contract=recording_integrity_contract(STREAM_BACKEND_GSTREAMER),
                metrics={
                    "container_finalization_success": False,
                    "source_media_integrity_verified": False,
                    "recording_file_integrity_verified": False,
                    "integrity_validation_status": "FAIL",
                    "integrity_failure_reasons": [
                        "decoded_nvenc_recorder_unavailable"
                    ],
                    "recording_usable": False,
                },
            )
        pipeline.set_state(Gst.State.NULL)
        if not stream_failed or stop_event.is_set():
            _emit_status(output_queue, "중지됨")
    return not stream_failed


def _run_gstreamer_stream(config: dict[str, Any], output_queue, stop_event, cv2, model, enable_detection: bool) -> None:
    """Run GStreamer with one explicit owner for any reconnect policy."""

    config.setdefault(
        "_requested_decoder_mode",
        normalize_decoder_mode(str(config.get("decoder", DECODER_AUTO))),
    )
    reconnect_policy = normalize_rtsp_reconnect_policy(
        str(config.get("rtsp_reconnect_policy", RTSP_RECONNECT_AUTOMATIC))
    )
    if reconnect_policy == RTSP_RECONNECT_MANUAL:
        recovered = _run_gstreamer_stream_once(
            config,
            output_queue,
            stop_event,
            cv2,
            model,
            enable_detection,
        )
        integrity_manual_reconnect = bool(
            config.pop("_integrity_manual_reconnect_required", False)
        )
        if integrity_manual_reconnect and not stop_event.is_set():
            _emit_terminal_exit(
                output_queue,
                "H264 IDR random-access recovery unavailable; manual reconnect required",
            )
            return
        recording_failure = config.pop("_recording_output_failure", None)
        if recording_failure and not stop_event.is_set():
            _emit_status(
                output_queue,
                "MP4 mux/filesink failure: recording side stopped; "
                "decoder fallback/reconnect disabled",
            )
            _emit_terminal_exit(output_queue, "MP4 mux/filesink failure")
            return
        if bool(config.pop("_decoder_manual_restart_required", False)):
            _emit_terminal_exit(output_queue, "decoder-health failure requires manual restart")
            return
        fallback_reason = config.pop("_runtime_decoder_fallback_reason", None)
        if fallback_reason and not stop_event.is_set():
            config["_runtime_decoder_fallback_active"] = True
            config["decoder"] = DECODER_SOFTWARE
            if _record_video_path(config):
                _emit_decoder_fallback(
                    output_queue,
                    reason=(
                        f"{fallback_reason}; this side MP4 finalized; "
                        "software override applies on next side start"
                    ),
                    recording_restart_required=True,
                )
                return
            _emit_decoder_fallback(
                output_queue,
                reason=(
                    f"{fallback_reason}; preview-only avdec_h264 restart; "
                    "current-generation frame pending"
                ),
                recording_restart_required=False,
            )
            _run_gstreamer_stream_once(config, output_queue, stop_event, cv2, model, enable_detection)
            return
        if not recovered and not stop_event.is_set():
            _emit_status(
                output_queue,
                "GStreamer RTSP 세션 종료: 자동 재연결 안 함",
            )
        return

    delays = tuple(max(0.0, float(delay)) for delay in GST_RTSP_RECONNECT_DELAYS_S) or (1.0,)
    consecutive_failures = 0
    reconnect_attempt = 0
    while not stop_event.is_set():
        if _run_gstreamer_stream_once(config, output_queue, stop_event, cv2, model, enable_detection):
            return
        if stop_event.is_set():
            return
        if bool(config.pop("_integrity_manual_reconnect_required", False)):
            _emit_status(
                output_queue,
                "H264 IDR random-access 복구 실패: 이 side unavailable; "
                "자동 재연결 안 함; 수동 재연결이 필요합니다.",
            )
            _emit_terminal_exit(
                output_queue,
                "H264 IDR random-access recovery unavailable; manual reconnect required",
            )
            return
        recording_failure = config.pop("_recording_output_failure", None)
        if recording_failure:
            _emit_status(
                output_queue,
                "MP4 mux/filesink failure: recording side stopped; "
                "decoder fallback/reconnect disabled",
            )
            _emit_terminal_exit(output_queue, "MP4 mux/filesink failure")
            return
        if bool(config.pop("_decoder_manual_restart_required", False)):
            _emit_terminal_exit(output_queue, "decoder-health failure requires manual restart")
            return
        fallback_reason = config.pop("_runtime_decoder_fallback_reason", None)
        if fallback_reason:
            config["_runtime_decoder_fallback_active"] = True
            config["decoder"] = DECODER_SOFTWARE
            if _record_video_path(config):
                _emit_decoder_fallback(
                    output_queue,
                    reason=(
                        f"{fallback_reason}; this side MP4 finalized; "
                        "software override applies on next side start"
                    ),
                    recording_restart_required=True,
                )
                return
            _emit_decoder_fallback(
                output_queue,
                reason=(
                    f"{fallback_reason}; preview-only avdec_h264 restart; "
                    "current-generation frame pending"
                ),
                recording_restart_required=False,
            )
            continue
        if bool(config.pop("_gst_last_run_received_frame", False)):
            consecutive_failures = 0
        delay_s = delays[min(consecutive_failures, len(delays) - 1)]
        consecutive_failures += 1
        reconnect_attempt += 1
        _emit_status(
            output_queue,
            f"GStreamer RTSP 재연결 대기 (시도 {reconnect_attempt}): "
            f"{delay_s:.1f}s 후 같은 endpoint 재시도",
        )
        if stop_event.wait(delay_s):
            return


def _run_opencv_stream(config: dict[str, Any], output_queue, stop_event, cv2, model, enable_detection: bool) -> None:
    transport = normalize_rtsp_transport(str(config.get("rtsp_transport", RTSP_TRANSPORT_UDP)))
    os.environ["OPENCV_FFMPEG_CAPTURE_OPTIONS"] = f"rtsp_transport;{transport}|stimeout;2000000|max_delay;500000"
    backend = normalize_stream_backend(str(config.get("stream_backend", STREAM_BACKEND_OPENCV)))
    config.update(recording_integrity_contract(backend))
    api_preference = cv2.CAP_ANY if backend == STREAM_BACKEND_OPENCV_AUTO else cv2.CAP_FFMPEG
    backend_name = "OpenCV Auto" if backend == STREAM_BACKEND_OPENCV_AUTO else "OpenCV/FFmpeg"
    cap = cv2.VideoCapture(config["url"], api_preference)
    try:
        cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
    except Exception:
        pass
    if not cap.isOpened():
        _emit_status(output_queue, f"{backend_name} 스트림 열기 실패")
        cap.release()
        return

    rate_limiter = AdaptiveYoloRateLimiter.from_config(config, enable_detection=enable_detection)
    next_frame_s = 0.0
    last_status_s = 0.0
    last_stats_s = time.monotonic()
    displayed_frames = 0
    inferred_frames = 0
    width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH) or 0)
    height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT) or 0)
    latest_lock = threading.Lock()
    latest_frame: dict[str, Any] | None = None
    latest_sequence = 0
    decoded_rate = RollingFrameRateEstimator()
    reader_stop = threading.Event()
    record_writer_state = None

    def reader_loop() -> None:
        nonlocal displayed_frames, height, latest_frame, latest_sequence, record_writer_state, width
        while not stop_event.is_set() and not reader_stop.is_set():
            ok, frame = cap.read()
            receive_monotonic_ns, receive_s = app_receive_monotonic_sample()
            if not ok or frame is None:
                time.sleep(0.02)
                continue
            with latest_lock:
                decoded_rate.add(receive_s)
            frame_height, frame_width = frame.shape[:2]
            if _record_video_path(config):
                if record_writer_state is None:
                    record_writer_state = _open_record_video_writer(cv2, config, output_queue, frame_width, frame_height)
                _write_record_video_frame(cv2, record_writer_state, frame)
            native_metadata = _publish_native_frame_to_ring(
                frame,
                config,
                output_queue,
                timestamp_s=receive_s,
                source_timestamp_s=None,
            )
            native_metadata.update(
                {
                    "app_receive_monotonic_ns": receive_monotonic_ns,
                    "app_receive_timestamp_role": "authoritative_distance",
                }
            )
            preview_emitted = _emit_preview_frame(
                cv2,
                frame,
                config,
                output_queue,
                timestamp_s=receive_s,
                timestamp_source=f"{backend}-read",
                extra_metadata=native_metadata,
            )
            with latest_lock:
                latest_sequence += 1
                displayed_frames += int(preview_emitted)
                width = frame_width
                height = frame_height
                latest_frame = {
                    "sequence": latest_sequence,
                    "frame": frame,
                    "timestamp_s": receive_s,
                    "native_metadata": native_metadata,
                    "width": frame_width,
                    "height": frame_height,
                }

    reader = threading.Thread(target=reader_loop, name=f"stereo-{backend}-reader", daemon=True)
    reader.start()
    status = f"{backend_name} YOLO 실행 중" if enable_detection else f"{backend_name} 카메라 스트림 실행 중"
    if _record_video_path(config):
        status += ": UNVERIFIED_REENCODE (decoded frame->MP4; RTP/AU 무결성 보장 없음)"
    else:
        status += ": UNVERIFIED_DECODE (RTP/AU 무결성 게이트 없음)"
    _emit_status(output_queue, status)
    last_processed_sequence = 0
    try:
        while not stop_event.is_set():
            now_s = time.monotonic()
            if now_s - last_stats_s >= 1.0:
                elapsed_s = now_s - last_stats_s
                with latest_lock:
                    interval_displayed_frames = displayed_frames
                    displayed_frames = 0
                    stats_width = width
                    stats_height = height
                    decoded_hz = decoded_rate.rate(now_s)
                displayed_hz = interval_displayed_frames / elapsed_s
                inferred_hz = inferred_frames / elapsed_s
                output_queue.put(
                    (
                        "stats",
                        decoded_hz,
                        displayed_hz,
                        inferred_hz,
                        stats_width,
                        stats_height,
                        rate_limiter.target_rate_hz,
                        *_yolo_device_stats(config),
                    )
                )
                rate_limiter.update(decoded_hz=decoded_hz, inferred_hz=inferred_hz)
                inferred_frames = 0
                last_stats_s = now_s
            with latest_lock:
                frame_record = None if latest_frame is None else dict(latest_frame)
            if frame_record is None:
                if now_s - last_status_s > 1.0:
                    _emit_status(output_queue, f"{backend_name} 프레임 수신 대기 중")
                    last_status_s = now_s
                time.sleep(0.02)
                continue
            sequence = int(frame_record["sequence"])
            if sequence == last_processed_sequence:
                time.sleep(0.005)
                continue
            if now_s < next_frame_s:
                last_processed_sequence = sequence
                time.sleep(min(0.02, max(0.0, next_frame_s - now_s)))
                continue
            last_processed_sequence = sequence
            next_frame_s = now_s + rate_limiter.interval_s

            frame = frame_record["frame"]
            detection_timestamp_metadata = {
                "timestamp_s": float(frame_record["timestamp_s"]),
                "source_timestamp_s": None,
                "timestamp_source": f"{backend}-read",
                "timestamp_reference": "app-receive",
            }
            detection_timestamp_metadata.update(
                dict(frame_record.get("native_metadata") or {})
            )
            annotated, best_detection, inferred = _process_frame_for_output(
                cv2,
                frame,
                model=model,
                config=config,
                output_queue=output_queue,
                now_s=float(frame_record["timestamp_s"]),
                enable_detection=enable_detection,
                draw_preview_overlay=True,
                timestamp_metadata=detection_timestamp_metadata,
            )
            if annotated is None:
                time.sleep(0.2)
                continue
            if config.get("_yolo_cpu_fallback_active"):
                _activate_cpu_fallback_rate_limiter(rate_limiter, config)
            if inferred:
                inferred_frames += 1
            last_status_s = _emit_stream_status(output_queue, enable_detection, best_detection, now_s, last_status_s)

    finally:
        reader_stop.set()
        reader.join(timeout=1.0)
        _release_record_video_writer(output_queue, record_writer_state)
        cap.release()
        _emit_status(output_queue, "중지됨")


def _run_pyav_stream(config: dict[str, Any], output_queue, stop_event, cv2, model, enable_detection: bool) -> None:
    try:
        import av
    except Exception as exc:
        _emit_status(output_queue, f"PyAV 사용 불가: {exc}")
        return

    transport = normalize_rtsp_transport(str(config.get("rtsp_transport", RTSP_TRANSPORT_UDP)))
    config.update(recording_integrity_contract(STREAM_BACKEND_PYAV))
    options = {
        "rtsp_transport": transport,
        "stimeout": "2000000",
        "max_delay": "500000",
        "fflags": "nobuffer",
        "flags": "low_delay",
    }
    try:
        container = av.open(config["url"], mode="r", options=options)
    except Exception as exc:
        _emit_status(output_queue, f"PyAV 스트림 열기 실패: {exc}")
        return

    video_stream = next((stream for stream in container.streams if stream.type == "video"), None)
    if video_stream is None:
        container.close()
        _emit_status(output_queue, "PyAV 비디오 스트림 없음")
        return

    rate_limiter = AdaptiveYoloRateLimiter.from_config(config, enable_detection=enable_detection)
    next_frame_s = 0.0
    last_status_s = 0.0
    last_stats_s = time.monotonic()
    displayed_frames = 0
    inferred_frames = 0
    width = int(video_stream.width or 0)
    height = int(video_stream.height or 0)
    latest_lock = threading.Lock()
    latest_frame: dict[str, Any] | None = None
    latest_sequence = 0
    decoded_rate = RollingFrameRateEstimator()
    reader_stop = threading.Event()
    record_writer_state = None

    def reader_loop() -> None:
        nonlocal displayed_frames, height, latest_frame, latest_sequence, record_writer_state, width
        try:
            for packet in container.demux(video_stream):
                if stop_event.is_set() or reader_stop.is_set():
                    break
                for av_frame in packet.decode():
                    if stop_event.is_set() or reader_stop.is_set():
                        break
                    receive_monotonic_ns, receive_s = app_receive_monotonic_sample()
                    with latest_lock:
                        decoded_rate.add(receive_s)
                    frame = av_frame.to_ndarray(format="bgr24")
                    frame_height, frame_width = frame.shape[:2]
                    if _record_video_path(config):
                        if record_writer_state is None:
                            record_writer_state = _open_record_video_writer(
                                cv2,
                                config,
                                output_queue,
                                frame_width,
                                frame_height,
                            )
                        _write_record_video_frame(cv2, record_writer_state, frame)
                    source_timestamp_s = None
                    timestamp_source = "pyav-read"
                    timestamp_reference = "app-receive"
                    if av_frame.pts is not None and av_frame.time_base is not None:
                        source_timestamp_s = float(av_frame.pts * av_frame.time_base)
                        timestamp_source = "pyav-pts"
                        timestamp_reference = "source"
                    native_metadata = _publish_native_frame_to_ring(
                        frame,
                        config,
                        output_queue,
                        timestamp_s=receive_s,
                        source_timestamp_s=source_timestamp_s,
                    )
                    native_metadata.update(
                        {
                            "app_receive_monotonic_ns": receive_monotonic_ns,
                            "app_receive_timestamp_role": "authoritative_distance",
                        }
                    )
                    preview_emitted = _emit_preview_frame(
                        cv2,
                        frame,
                        config,
                        output_queue,
                        timestamp_s=receive_s,
                        source_timestamp_s=source_timestamp_s,
                        timestamp_source=timestamp_source,
                        extra_metadata=native_metadata,
                    )
                    with latest_lock:
                        latest_sequence += 1
                        displayed_frames += int(preview_emitted)
                        width = frame_width
                        height = frame_height
                        latest_frame = {
                            "sequence": latest_sequence,
                            "frame": frame,
                            "timestamp_s": receive_s,
                            "source_timestamp_s": source_timestamp_s,
                            "timestamp_source": timestamp_source,
                            "timestamp_reference": timestamp_reference,
                            "native_metadata": native_metadata,
                            "width": frame_width,
                            "height": frame_height,
                        }
        except Exception as exc:
            if not stop_event.is_set() and not reader_stop.is_set():
                _emit_status(output_queue, f"PyAV 스트림 오류: {exc}")

    reader = threading.Thread(target=reader_loop, name="stereo-pyav-reader", daemon=True)
    reader.start()
    status = "PyAV YOLO 실행 중" if enable_detection else "PyAV 카메라 스트림 실행 중"
    if _record_video_path(config):
        status += ": UNVERIFIED_REENCODE (decoded frame->MP4; RTP/AU 무결성 보장 없음)"
    else:
        status += ": UNVERIFIED_DECODE (RTP/AU 무결성 게이트 없음)"
    _emit_status(output_queue, status)
    last_processed_sequence = 0
    try:
        while not stop_event.is_set():
            now_s = time.monotonic()
            if now_s - last_stats_s >= 1.0:
                elapsed_s = now_s - last_stats_s
                with latest_lock:
                    interval_displayed_frames = displayed_frames
                    displayed_frames = 0
                    stats_width = width
                    stats_height = height
                    decoded_hz = decoded_rate.rate(now_s)
                displayed_hz = interval_displayed_frames / elapsed_s
                inferred_hz = inferred_frames / elapsed_s
                output_queue.put(
                    (
                        "stats",
                        decoded_hz,
                        displayed_hz,
                        inferred_hz,
                        stats_width,
                        stats_height,
                        rate_limiter.target_rate_hz,
                        *_yolo_device_stats(config),
                    )
                )
                rate_limiter.update(decoded_hz=decoded_hz, inferred_hz=inferred_hz)
                inferred_frames = 0
                last_stats_s = now_s
            with latest_lock:
                frame_record = None if latest_frame is None else dict(latest_frame)
            if frame_record is None:
                if now_s - last_status_s > 1.0:
                    _emit_status(output_queue, "PyAV 프레임 수신 대기 중")
                    last_status_s = now_s
                time.sleep(0.02)
                continue
            sequence = int(frame_record["sequence"])
            if sequence == last_processed_sequence:
                time.sleep(0.005)
                continue
            if now_s < next_frame_s:
                last_processed_sequence = sequence
                time.sleep(min(0.02, max(0.0, next_frame_s - now_s)))
                continue
            last_processed_sequence = sequence
            next_frame_s = now_s + rate_limiter.interval_s

            frame = frame_record["frame"]
            detection_timestamp_metadata = {
                "timestamp_s": float(frame_record["timestamp_s"]),
                "source_timestamp_s": frame_record["source_timestamp_s"],
                "timestamp_source": frame_record["timestamp_source"],
                "timestamp_reference": frame_record["timestamp_reference"],
            }
            detection_timestamp_metadata.update(
                dict(frame_record.get("native_metadata") or {})
            )
            annotated, best_detection, inferred = _process_frame_for_output(
                cv2,
                frame,
                model=model,
                config=config,
                output_queue=output_queue,
                now_s=float(frame_record["timestamp_s"]),
                enable_detection=enable_detection,
                draw_preview_overlay=True,
                timestamp_metadata=detection_timestamp_metadata,
            )
            if annotated is None:
                time.sleep(0.2)
                continue
            if config.get("_yolo_cpu_fallback_active"):
                _activate_cpu_fallback_rate_limiter(rate_limiter, config)
            if inferred:
                inferred_frames += 1
            last_status_s = _emit_stream_status(output_queue, enable_detection, best_detection, now_s, last_status_s)

    finally:
        reader_stop.set()
        reader.join(timeout=1.0)
        _release_record_video_writer(output_queue, record_writer_state)
        container.close()
        _emit_status(output_queue, "중지됨")


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


class CameraIpcCounters:
    """Small shared counters for independently bounded camera IPC channels."""

    def __init__(self, mp_context: multiprocessing.context.BaseContext) -> None:
        self.preview_replaced = mp_context.Value("q", 0)
        self.preview_dropped = mp_context.Value("q", 0)
        self.metadata_replaced = mp_context.Value("q", 0)
        self.metadata_dropped = mp_context.Value("q", 0)
        self.control_dropped = mp_context.Value("q", 0)

    @staticmethod
    def _increment(counter) -> None:
        with counter.get_lock():
            counter.value += 1

    def snapshot(self) -> dict[str, int]:
        return {
            "preview_replaced": int(self.preview_replaced.value),
            "preview_dropped": int(self.preview_dropped.value),
            "metadata_replaced": int(self.metadata_replaced.value),
            "metadata_dropped": int(self.metadata_dropped.value),
            "control_dropped": int(self.control_dropped.value),
        }


class _BoundedProcessOutputQueue:
    METADATA_KINDS = frozenset({"detection", "stats"})
    CRITICAL_CONTROL_KINDS = frozenset(
        {
            "decoder_fallback",
            "fatal_error",
            "recording_finalized",
            "terminal_exit",
        }
    )

    def __init__(
        self,
        control_queue,
        frame_queue,
        metadata_queue=None,
        counters: CameraIpcCounters | None = None,
    ) -> None:
        # event_queue remains an alias for compatibility with tests and older
        # process targets. Actual Workstation runs provide a separate metadata
        # queue so image pressure cannot delay Stop/Error/control delivery.
        self.control_queue = control_queue
        self.event_queue = control_queue
        self.frame_queue = frame_queue
        self.metadata_queue = metadata_queue
        self.counters = counters

    def _count(self, name: str) -> None:
        if self.counters is not None:
            CameraIpcCounters._increment(getattr(self.counters, name))

    @staticmethod
    def _is_nal1_candidate_frame(message: object) -> bool:
        return bool(
            isinstance(message, tuple)
            and len(message) >= 2
            and message[0] == "frame"
            and isinstance(message[1], dict)
            and message[1].get("h264_nal1_non_idr_intra") is True
        )

    def _replace_latest(
        self,
        target_queue,
        message,
        *,
        replaced: str,
        dropped: str,
        protect_nal1_candidate: bool = False,
    ) -> None:
        try:
            target_queue.put_nowait(message)
            return
        except queue.Full:
            pass
        try:
            previous = target_queue.get_nowait()
        except queue.Empty:
            previous = None
        if (
            protect_nal1_candidate
            and self._is_nal1_candidate_frame(previous)
            and not self._is_nal1_candidate_frame(message)
        ):
            try:
                target_queue.put_nowait(previous)
            except queue.Full:
                pass
            self._count(dropped)
            return
        if previous is not None:
            self._count(replaced)
        try:
            target_queue.put_nowait(message)
        except queue.Full:
            self._count(dropped)

    def put(self, message) -> None:
        if message and message[0] == "frame":
            self._replace_latest(
                self.frame_queue,
                message,
                replaced="preview_replaced",
                dropped="preview_dropped",
                protect_nal1_candidate=True,
            )
            return
        kind = message[0] if message else ""
        if self.metadata_queue is not None and kind in self.METADATA_KINDS:
            self._replace_latest(
                self.metadata_queue,
                message,
                replaced="metadata_replaced",
                dropped="metadata_dropped",
            )
            return
        if kind in self.CRITICAL_CONTROL_KINDS:
            try:
                self.control_queue.put_nowait(message)
                return
            except queue.Full:
                try:
                    self.control_queue.get_nowait()
                    self._count("control_dropped")
                except queue.Empty:
                    pass
                try:
                    self.control_queue.put_nowait(message)
                except queue.Full:
                    self._count("control_dropped")
                return
        try:
            self.control_queue.put_nowait(message)
        except queue.Full:
            # Status is observational. Stop is an independent Event and fatal
            # messages use the bounded critical path above.
            self._count("control_dropped")


def _queue_has_pending_messages(message_queue) -> bool:
    if message_queue is None:
        return False
    try:
        return not message_queue.empty()
    except Exception:
        return False


def _get_queue_message(message_queue, *, timeout_s: float):
    if message_queue is None:
        raise queue.Empty
    return message_queue.get(timeout=timeout_s)


def _get_queue_message_nowait(message_queue):
    if message_queue is None:
        raise queue.Empty
    return message_queue.get_nowait()


class YoloStreamWorker(QtCore.QObject):
    frame_ready = QtCore.pyqtSignal(QtGui.QImage, object)
    measurement_frame_ready = QtCore.pyqtSignal(float, QtGui.QImage)
    native_measurement_frame_ready = QtCore.pyqtSignal(object, QtGui.QImage)
    detection_ready = QtCore.pyqtSignal(dict)
    stats_ready = QtCore.pyqtSignal(dict)
    recording_finalized = QtCore.pyqtSignal(dict)
    status_changed = QtCore.pyqtSignal(str)
    finished = QtCore.pyqtSignal()

    def __init__(
        self,
        *,
        url: str,
        weights_path: str,
        device: str,
        input_size: int,
        conf_threshold: float,
        iou_threshold: float,
        max_rate_hz: float,
        camera_id: str = "",
        stream_generation: int = 0,
        display_rate_hz: float = DEFAULT_PREVIEW_DISPLAY_RATE_HZ,
        preview_max_width: int = 960,
        rtsp_latency_ms: int = DEFAULT_RTSP_LATENCY_MS,
        rtsp_encoded_queue_mode: str = RTSP_ENCODED_QUEUE_MODE_LATEST,
        decoder: str = DECODER_AUTO,
        stream_backend: str = STREAM_BACKEND_GSTREAMER,
        rtsp_transport: str = RTSP_TRANSPORT_UDP,
        rtsp_reconnect_policy: str = RTSP_RECONNECT_MANUAL,
        enable_detection: bool = True,
        class_gate_enabled: bool = False,
        class_gate: str = "",
        record_video_path: str = "",
        record_video_fps: float = 30.0,
        event_queue_size: int = DEFAULT_EVENT_QUEUE_SIZE,
        metadata_queue_size: int = DEFAULT_METADATA_QUEUE_SIZE,
        frame_queue_size: int = DEFAULT_FRAME_QUEUE_SIZE,
        native_ring_capacity: int = DEFAULT_NATIVE_RING_CAPACITY,
        native_max_width: int = DEFAULT_NATIVE_MAX_WIDTH,
        native_max_height: int = DEFAULT_NATIVE_MAX_HEIGHT,
        process_target: Callable[..., None] = _run_yolo_process,
        mp_context: multiprocessing.context.BaseContext | None = None,
        crash_restart_delays_s: tuple[float, ...] = YOLO_PROCESS_CRASH_RESTART_DELAYS_S,
    ) -> None:
        super().__init__()
        effective_max_rate = effective_yolo_max_rate_hz(
            max_rate_hz,
            device=device,
            enable_detection=enable_detection,
        )
        adaptive_rate = enable_detection and _device_uses_cpu(device)
        self.config = {
            "url": url,
            "camera_id": str(camera_id or ""),
            "stream_generation": int(stream_generation),
            "weights_path": weights_path,
            "device": device,
            "_requested_yolo_device": device,
            "input_size": normalize_yolo_input_size(input_size),
            "conf_threshold": conf_threshold,
            "iou_threshold": iou_threshold,
            "max_rate_hz": effective_max_rate,
            "display_rate_hz": display_rate_hz,
            "preview_max_width": preview_max_width,
            "rtsp_latency_ms": rtsp_latency_ms,
            "rtsp_encoded_queue_mode": normalize_rtsp_encoded_queue_mode(
                rtsp_encoded_queue_mode
            ),
            "decoder": normalize_decoder_mode(decoder),
            "stream_backend": normalize_stream_backend(stream_backend),
            "rtsp_transport": normalize_rtsp_transport(rtsp_transport),
            "rtsp_reconnect_policy": normalize_rtsp_reconnect_policy(
                rtsp_reconnect_policy
            ),
            "enable_detection": enable_detection,
            "class_gate_enabled": bool(class_gate_enabled),
            "class_gate": str(class_gate or "").strip(),
            "emit_measurement_frames": False,
            "emit_native_reference_frames": False,
            "adaptive_rate": adaptive_rate,
            "adaptive_max_rate_hz": DEFAULT_CPU_YOLO_ADAPTIVE_MAX_RATE_HZ if adaptive_rate else 0.0,
            "worker_cpu_threads": _default_worker_cpu_threads(device),
            "record_video_path": str(record_video_path or ""),
            "record_video_fps": _record_video_fps({"record_video_fps": record_video_fps}),
            "event_queue_size": max(1, int(event_queue_size)),
            "metadata_queue_size": max(1, int(metadata_queue_size)),
            "frame_queue_size": max(1, int(frame_queue_size)),
        }
        self.config.update(recording_integrity_contract(self.config["stream_backend"]))
        self._mp_context = mp_context or multiprocessing.get_context("spawn")
        set_executable = getattr(self._mp_context, "set_executable", None)
        if callable(set_executable):
            set_executable(sys.executable)
        self._ipc_counters = CameraIpcCounters(self._mp_context)
        self.native_frame_ring: SharedNativeFrameRing | None = None
        if self.config["camera_id"]:
            self.native_frame_ring = SharedNativeFrameRing.create(
                camera_id=self.config["camera_id"],
                stream_generation=self.config["stream_generation"],
                capacity=max(1, int(native_ring_capacity)),
                max_width=max(1, int(native_max_width)),
                max_height=max(1, int(native_max_height)),
                channels=3,
                mp_context=self._mp_context,
            )
            self.config["_native_frame_ring"] = self.native_frame_ring
        self._process_target = process_target
        self._crash_restart_delays_s = tuple(
            max(0.0, float(delay_s)) for delay_s in crash_restart_delays_s
        )
        self._stop_event = None
        self._control_queue = None
        self._event_queue = None
        self._metadata_queue = None
        self._frame_queue = None
        self._process = None
        self._stop_requested = False
        self._stop_requested_monotonic_s: float | None = None
        self._finished = False
        self._child_terminal_exit_reason = ""

    @QtCore.pyqtSlot()
    def run(self) -> None:
        status = "YOLO worker 시작 중" if self.config["enable_detection"] else "카메라 worker 시작 중"
        self.status_changed.emit(status)
        self._stop_requested = False
        self._stop_requested_monotonic_s = None
        self._finished = False
        self._child_terminal_exit_reason = ""
        consecutive_crashes = 0
        try:
            while not self._stop_requested:
                self._stop_event = self._mp_context.Event()
                self._control_queue = self._mp_context.Queue(
                    maxsize=int(self.config["event_queue_size"])
                )
                self._event_queue = self._control_queue
                self._metadata_queue = self._mp_context.Queue(
                    maxsize=int(self.config["metadata_queue_size"])
                )
                self._frame_queue = self._mp_context.Queue(
                    maxsize=int(self.config["frame_queue_size"])
                )
                process = self._mp_context.Process(
                    target=self._process_target,
                    args=(
                        self.config,
                        _BoundedProcessOutputQueue(
                            self._control_queue,
                            self._frame_queue,
                            self._metadata_queue,
                            self._ipc_counters,
                        ),
                        self._stop_event,
                    ),
                    daemon=True,
                )
                self._process = process
                process_started_s = time.monotonic()
                launch_error = ""
                try:
                    process.start()
                    self.status_changed.emit(f"{status}: pid={process.pid}")
                    while process.is_alive() or self._queue_has_pending_messages():
                        self._drain_output_queue(timeout_s=0.1)
                    process.join(timeout=0.2)
                except Exception as exc:
                    launch_error = f"{type(exc).__name__}: {exc}"
                    self.status_changed.emit(f"스트림 프로세스 오류: {launch_error}")

                exitcode = getattr(process, "exitcode", None)
                runtime_s = max(0.0, time.monotonic() - process_started_s)
                child_stopped = self._stop_child_process()
                if not child_stopped:
                    self.status_changed.emit(
                        "스트림 자식 프로세스 종료 확인 실패; "
                        "재시작하지 않고 종료 검증을 대기합니다."
                    )
                    self._stop_requested = True
                    break
                if self._stop_requested:
                    break
                if self._child_terminal_exit_reason:
                    self.status_changed.emit(
                        "스트림 프로세스 영구 종료: "
                        f"{self._child_terminal_exit_reason}; 자동 재시작 안 함"
                    )
                    break
                if (
                    self.config["rtsp_reconnect_policy"]
                    == RTSP_RECONNECT_MANUAL
                ):
                    detail = launch_error or f"exitcode={exitcode}"
                    self.status_changed.emit(
                        f"스트림 프로세스 종료: {detail}; 자동 재시작 안 함"
                    )
                    break
                if runtime_s >= YOLO_PROCESS_STABLE_RUN_S:
                    consecutive_crashes = 0
                consecutive_crashes += 1
                if consecutive_crashes > len(self._crash_restart_delays_s):
                    detail = launch_error or f"exitcode={exitcode}"
                    self.status_changed.emit(
                        "스트림 프로세스 크래시 복구 한도 초과: "
                        f"{detail}; 수동 재시작이 필요합니다."
                    )
                    break
                delay_s = self._crash_restart_delays_s[consecutive_crashes - 1]
                detail = launch_error or f"exitcode={exitcode}"
                self.status_changed.emit(
                    "스트림 프로세스 크래시 복구 중 "
                    f"({consecutive_crashes}/{len(self._crash_restart_delays_s)}, "
                    f"{detail}, backoff={delay_s:.2f}s)"
                )
                retry_at_s = time.monotonic() + delay_s
                while not self._stop_requested and time.monotonic() < retry_at_s:
                    time.sleep(min(0.05, max(0.0, retry_at_s - time.monotonic())))
        finally:
            if self._stop_child_process():
                self._emit_finished_once()

    def stop(self) -> None:
        self.request_stop()
        process = self._process
        if process is None:
            return
        if not process.is_alive():
            if self._stop_child_process():
                self._emit_finished_once()
            return
        # Give the child a chance to release its video writer and queues after
        # observing the stop event.  Escalate only if cooperative shutdown
        # does not complete.
        stop_requested_s = self._stop_requested_monotonic_s or time.monotonic()
        cooperative_remaining_s = max(
            0.0,
            stop_requested_s + YOLO_COOPERATIVE_STOP_TIMEOUT_S - time.monotonic(),
        )
        process.join(timeout=cooperative_remaining_s)
        if process.is_alive():
            process.terminate()
            terminate_remaining_s = max(
                0.0,
                stop_requested_s + YOLO_KILL_AFTER_STOP_S - time.monotonic(),
            )
            process.join(timeout=terminate_remaining_s)
        if process.is_alive():
            process.kill()
            process.join(timeout=YOLO_KILL_JOIN_TIMEOUT_S)
        if self._stop_child_process():
            self._emit_finished_once()

    def request_stop(self) -> None:
        self._stop_requested = True
        if self._stop_requested_monotonic_s is None:
            self._stop_requested_monotonic_s = time.monotonic()
        if self._stop_event is not None:
            self._stop_event.set()

    def close_native_frame_ring(self, *, force: bool = False) -> bool:
        ring = self.native_frame_ring
        if ring is None:
            return True
        if not force and int(ring.metrics().get("active_leases", 0)) > 0:
            return False
        ring.close()
        ring.unlink()
        self.native_frame_ring = None
        self.config.pop("_native_frame_ring", None)
        return True

    def terminate_child_process(self) -> None:
        """Escalate shutdown without waiting on the caller's thread."""

        self.request_stop()
        process = self._process
        if process is not None and process.is_alive():
            process.terminate()

    def kill_child_process(self) -> None:
        """Last-resort non-blocking kill used by retryable UI shutdown."""

        self.request_stop()
        process = self._process
        if process is not None and process.is_alive():
            process.kill()
            return
        if process is not None and self._stop_child_process():
            self._emit_finished_once()

    def _queue_has_pending_messages(self) -> bool:
        return any(
            _queue_has_pending_messages(message_queue)
            for message_queue in (
                self._control_queue,
                self._metadata_queue,
                self._frame_queue,
            )
        )

    def _drain_output_queue(self, *, timeout_s: float) -> None:
        try:
            message = _get_queue_message(self._control_queue, timeout_s=0.0)
        except queue.Empty:
            try:
                message = _get_queue_message(self._metadata_queue, timeout_s=0.0)
            except queue.Empty:
                try:
                    message = _get_queue_message(
                        self._frame_queue,
                        timeout_s=timeout_s,
                    )
                except queue.Empty:
                    return
        self._handle_worker_message(message)
        for message_queue in (
            self._control_queue,
            self._metadata_queue,
            self._frame_queue,
        ):
            while True:
                try:
                    message = _get_queue_message_nowait(message_queue)
                except queue.Empty:
                    break
                self._handle_worker_message(message)

    def _stop_child_process(self) -> bool:
        process = self._process
        if process is not None and process.is_alive():
            if self._stop_requested_monotonic_s is None:
                self._stop_requested_monotonic_s = time.monotonic()
            stop_requested_s = self._stop_requested_monotonic_s
            if self._stop_event is not None:
                self._stop_event.set()
            cooperative_remaining_s = max(
                0.0,
                stop_requested_s + YOLO_COOPERATIVE_STOP_TIMEOUT_S - time.monotonic(),
            )
            process.join(timeout=cooperative_remaining_s)
            if process.is_alive():
                process.terminate()
                terminate_remaining_s = max(
                    0.0,
                    stop_requested_s + YOLO_KILL_AFTER_STOP_S - time.monotonic(),
                )
                process.join(timeout=terminate_remaining_s)
            if process.is_alive():
                process.kill()
                process.join(timeout=YOLO_KILL_JOIN_TIMEOUT_S)
            if process.is_alive():
                # Keep the handle and IPC owners reachable so the application
                # resource audit cannot mistake a kill request for confirmed
                # child death. A later forced-shutdown retry calls
                # kill_child_process() again and completes cleanup only after
                # is_alive() becomes false.
                return False
        self._process = None
        self._stop_event = None
        for message_queue in (
            self._control_queue,
            self._metadata_queue,
            self._frame_queue,
        ):
            if message_queue is None:
                continue
            try:
                message_queue.cancel_join_thread()
            except (AttributeError, OSError, ValueError):
                pass
            try:
                message_queue.close()
            except (AttributeError, OSError, ValueError):
                pass
        self._control_queue = None
        self._event_queue = None
        self._metadata_queue = None
        self._frame_queue = None
        return True

    def _handle_worker_message(self, message) -> None:
        kind = message[0]

        def ensure_app_receive_metadata(
            metadata: dict[str, object],
            timestamp_s: object,
        ) -> None:
            if metadata.get("app_receive_monotonic_ns") is None:
                metadata["app_receive_monotonic_ns"] = int(
                    round(float(timestamp_s) * 1_000_000_000.0)
                )
                metadata.setdefault(
                    "app_receive_timestamp_role", "legacy_float_derived"
                )
            else:
                metadata.setdefault(
                    "app_receive_timestamp_role", "authoritative_distance"
                )
        if kind == "decoder_fallback":
            payload = dict(message[1] or {}) if len(message) > 1 else {}
            reason = str(payload.get("reason") or "unknown decoder fallback")
            recording_restart_required = bool(
                payload.get("recording_restart_required", False)
            )
            if recording_restart_required:
                self._child_terminal_exit_reason = (
                    "recording-side decoder fallback requires a new side start"
                )
                prefix = DECODER_FALLBACK_STATUS_PREFIX
            else:
                prefix = DECODER_FALLBACK_ACTIVE_STATUS_PREFIX
            self.status_changed.emit(f"{prefix} {reason}")
            return
        if kind == "terminal_exit":
            payload = dict(message[1] or {}) if len(message) > 1 else {}
            self._child_terminal_exit_reason = str(
                payload.get("reason") or "child requested permanent exit"
            )
            return
        if kind == "status":
            self.status_changed.emit(message[1])
            return
        if kind == "recording_finalized":
            self.recording_finalized.emit(dict(message[1] or {}))
            return
        if kind == "frame":
            if len(message) == 5 and isinstance(message[1], dict):
                _kind, metadata, width, height, data = message
            elif len(message) == 5:
                _kind, timestamp_s, width, height, data = message
                metadata = {
                    "timestamp_s": float(timestamp_s),
                    "source_timestamp_s": None,
                    "timestamp_source": "legacy-frame",
                }
            else:
                _kind, width, height, data = message
                metadata = {
                    "timestamp_s": time.monotonic(),
                    "source_timestamp_s": None,
                    "timestamp_source": "legacy-frame",
                }
            ensure_app_receive_metadata(metadata, metadata.get("timestamp_s", 0.0))
            metadata.setdefault("rtsp_transport", self.config["rtsp_transport"])
            metadata.setdefault("stream_backend", self.config["stream_backend"])
            metadata.setdefault(
                "recording_encoding_mode",
                self.config["recording_encoding_mode"],
            )
            metadata.setdefault(
                "recording_integrity_verified",
                self.config["recording_integrity_verified"],
            )
            metadata.setdefault(
                "recording_integrity_scope",
                self.config["recording_integrity_scope"],
            )
            image = QtGui.QImage(
                data,
                width,
                height,
                width * 3,
                QtGui.QImage.Format_RGB888,
            ).copy()
            self.frame_ready.emit(image, metadata)
            return
        if kind == "measurement_frame":
            _kind, timestamp_value, width, height, data = message
            metadata = (
                dict(timestamp_value)
                if isinstance(timestamp_value, dict)
                else {
                    "timestamp_s": float(timestamp_value),
                    "native_width": int(width),
                    "native_height": int(height),
                    "native_lossless_source": False,
                    "overlay_applied": False,
                }
            )
            timestamp_s = float(metadata.get("timestamp_s", time.monotonic()))
            ensure_app_receive_metadata(metadata, timestamp_s)
            metadata.setdefault("rtsp_transport", self.config["rtsp_transport"])
            metadata.setdefault("stream_backend", self.config["stream_backend"])
            metadata.setdefault(
                "recording_encoding_mode",
                self.config["recording_encoding_mode"],
            )
            metadata.setdefault(
                "recording_integrity_verified",
                self.config["recording_integrity_verified"],
            )
            metadata.setdefault(
                "recording_integrity_scope",
                self.config["recording_integrity_scope"],
            )
            image = QtGui.QImage(
                data,
                width,
                height,
                width * 3,
                QtGui.QImage.Format_RGB888,
            ).copy()
            self.measurement_frame_ready.emit(float(timestamp_s), image)
            self.native_measurement_frame_ready.emit(metadata, image)
            return
        if kind == "detection":
            if len(message) >= 7:
                _kind, timestamp_s, width, height, best_detection, detections, timestamp_metadata = message
            else:
                _kind, timestamp_s, width, height, best_detection, detections = message
                timestamp_metadata = {}
            timestamp_metadata = dict(timestamp_metadata or {})
            ensure_app_receive_metadata(timestamp_metadata, timestamp_s)
            timestamp_metadata.setdefault("rtsp_transport", self.config["rtsp_transport"])
            timestamp_metadata.setdefault("stream_backend", self.config["stream_backend"])
            timestamp_metadata.setdefault(
                "recording_encoding_mode",
                self.config["recording_encoding_mode"],
            )
            timestamp_metadata.setdefault(
                "recording_integrity_verified",
                self.config["recording_integrity_verified"],
            )
            timestamp_metadata.setdefault(
                "recording_integrity_scope",
                self.config["recording_integrity_scope"],
            )
            self.detection_ready.emit(
                {
                    "timestamp_s": timestamp_s,
                    "width": width,
                    "height": height,
                    "best_detection": best_detection,
                    "detections": detections,
                    "timestamp_metadata": timestamp_metadata,
                }
            )
            return
        if kind == "stats":
            stream_metrics: dict[str, object] = {}
            if len(message) >= 10:
                (
                    _kind,
                    decoded_hz,
                    displayed_hz,
                    inferred_hz,
                    width,
                    height,
                    target_rate_hz,
                    requested_device,
                    active_device,
                    cpu_fallback,
                ) = message[:10]
                if len(message) >= 11 and isinstance(message[10], Mapping):
                    stream_metrics = dict(message[10])
            elif len(message) >= 7:
                _kind, decoded_hz, displayed_hz, inferred_hz, width, height, target_rate_hz = message
                requested_device = self.config.get("_requested_yolo_device", self.config["device"])
                active_device = ""
                cpu_fallback = False
            else:
                _kind, decoded_hz, displayed_hz, inferred_hz, width, height = message
                target_rate_hz = 0.0
                requested_device = self.config.get("_requested_yolo_device", self.config["device"])
                active_device = ""
                cpu_fallback = False
            native_ring_metrics = (
                self.native_frame_ring.metrics()
                if self.native_frame_ring is not None
                else {}
            )
            self.stats_ready.emit(
                {
                    "decoded_hz": decoded_hz,
                    "decoded_rate_window_s": DEFAULT_DECODE_RATE_WINDOW_S,
                    "displayed_hz": displayed_hz,
                    "inferred_hz": inferred_hz,
                    "width": width,
                    "height": height,
                    "decoder": self.config["decoder"],
                    "stream_backend": self.config["stream_backend"],
                    "rtsp_transport": self.config["rtsp_transport"],
                    "rtsp_reconnect_policy": self.config[
                        "rtsp_reconnect_policy"
                    ],
                    "recording_encoding_mode": self.config[
                        "recording_encoding_mode"
                    ],
                    "recording_integrity_verified": self.config[
                        "recording_integrity_verified"
                    ],
                    "recording_integrity_scope": self.config[
                        "recording_integrity_scope"
                    ],
                    "enable_detection": self.config["enable_detection"],
                    "yolo_target_hz": target_rate_hz,
                    "yolo_requested_device": requested_device,
                    "yolo_actual_device": active_device,
                    "yolo_cpu_fallback": cpu_fallback,
                    **stream_metrics,
                    "preview_queue_capacity": int(
                        self.config["frame_queue_size"]
                    ),
                    "metadata_queue_capacity": int(
                        self.config["metadata_queue_size"]
                    ),
                    "control_queue_capacity": int(
                        self.config["event_queue_size"]
                    ),
                    **self._ipc_counters.snapshot(),
                    **{
                        f"native_ring_{key}": value
                        for key, value in native_ring_metrics.items()
                        if key not in {"camera_id", "stream_generation"}
                    },
                }
            )

    def _emit_finished_once(self) -> None:
        if self._finished:
            return
        self._finished = True
        self.finished.emit()
