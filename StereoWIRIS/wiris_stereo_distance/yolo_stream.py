from __future__ import annotations

import os
import multiprocessing
import queue
import shlex
import shutil
import signal
import subprocess
import sys
import threading
import time
from collections import deque
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Callable

from PyQt5 import QtCore, QtGui


SASL_ROOT = Path(__file__).resolve().parents[2]
DEFAULT_YOLO_PYTHON = os.environ.get("PYTHON_BIN") or sys.executable
DEFAULT_YOLO_WEIGHTS = os.environ.get("YOLO_MODEL_PATH") or str(SASL_ROOT / "models" / "sasl_uav.pt")
DEFAULT_RTSP_LATENCY_MS = 50
DEFAULT_CPU_YOLO_MAX_RATE_HZ = 20.0
DEFAULT_CPU_YOLO_INITIAL_RATE_HZ = 15.0
DEFAULT_CPU_YOLO_ADAPTIVE_MAX_RATE_HZ = 20.0
DEFAULT_CPU_YOLO_MIN_RATE_HZ = 2.0
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
YOLO_INPUT_SIZE_STRIDE = 32
DECODER_AUTO = "auto"
DECODER_NVIDIA = "nvidia"
DECODER_SOFTWARE = "software"
STREAM_BACKEND_GSTREAMER = "gstreamer"
STREAM_BACKEND_OPENCV = "opencv"
STREAM_BACKEND_OPENCV_AUTO = "opencv_auto"
STREAM_BACKEND_PYAV = "pyav"
RTSP_TRANSPORT_UDP = "udp"
RTSP_TRANSPORT_TCP = "tcp"
I_FRAME_ONLY_ENV = "STEREO_GST_I_FRAME_ONLY"
NVIDIA_H264_DECODER = "nvh264dec"
SOFTWARE_H264_DECODER = "avdec_h264"
RED_BGR = (0, 0, 255)
GREEN_BGR = (0, 180, 0)
WHITE_BGR = (255, 255, 255)
RTP_CLOCK_RATE_HZ = 90_000
NTP_UNIX_EPOCH_OFFSET_S = 2_208_988_800
UINT32_MODULUS = 2**32
UINT32_HALF_RANGE = 2**31
MAX_RTP_TIMING_CACHE = 240
DEFAULT_RECORD_VIDEO_QUEUE_SIZE = 300
MAX_RTCP_SR_PC_OFFSET_S = 0.5


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
        color = RED_BGR if is_best else GREEN_BGR
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
    return shlex.quote(str(Path(value).expanduser()))


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
    ) -> None:
        self.cv2 = cv2
        self.path = str(path)
        self.fps = float(fps)
        self.width = int(width)
        self.height = int(height)
        self.output_queue = output_queue
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
        self.thread.join(timeout=5.0)
        if self.thread.is_alive():
            _emit_status(self.output_queue, f"영상 녹화 writer 종료 지연: {self.path}")

    def _run(self) -> None:
        writer = None
        try:
            output_path = Path(self.path).expanduser()
            output_path.parent.mkdir(parents=True, exist_ok=True)
            fourcc = self.cv2.VideoWriter_fourcc(*"mp4v")
            writer = self.cv2.VideoWriter(str(output_path), fourcc, self.fps, (self.width, self.height))
            if not writer.isOpened():
                writer.release()
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
        except Exception as exc:
            _emit_status(self.output_queue, f"영상 녹화 writer 오류: {exc}")
        finally:
            if writer is not None:
                writer.release()
            _emit_status(
                self.output_queue,
                f"영상 녹화 저장 완료: {self.path} "
                f"(written={self.written_frames}, dropped={self.dropped_frames})",
            )


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


def normalize_rtsp_transport(transport: str | None) -> str:
    value = (transport or RTSP_TRANSPORT_UDP).strip().lower()
    if value in {RTSP_TRANSPORT_UDP, "udp-mcast", "udp_multicast"}:
        return RTSP_TRANSPORT_UDP
    if value in {RTSP_TRANSPORT_TCP, "interleaved"}:
        return RTSP_TRANSPORT_TCP
    return RTSP_TRANSPORT_UDP


def _device_uses_cpu(device: str | None) -> bool:
    return not (device or "").strip() or (device or "").strip().lower() == "cpu"


def effective_yolo_max_rate_hz(max_rate_hz: float | int | str | None, *, device: str | None, enable_detection: bool) -> float:
    try:
        rate_hz = float(max_rate_hz or 0.0)
    except (TypeError, ValueError):
        rate_hz = 0.0
    if enable_detection and rate_hz <= 0.0 and _device_uses_cpu(device):
        return DEFAULT_CPU_YOLO_MAX_RATE_HZ
    return max(0.0, rate_hz)


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


def env_flag_enabled(name: str) -> bool:
    return (os.environ.get(name) or "").strip().lower() in {"1", "true", "yes", "on"}


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
        return f"{SOFTWARE_H264_DECODER} skip-frame=1 output-corrupt=false"
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
class RtcpSenderReportTiming:
    ssrc: int
    ntp_raw: int
    ntp_unix_s: float
    rtp_timestamp: int
    clock_rate_hz: int
    received_monotonic_s: float
    received_wall_s: float | None = None


def rtcp_sender_report_pc_offset_s(
    *,
    sr_ntp_unix_s: float,
    received_wall_s: float,
) -> float | None:
    offset_s = float(sr_ntp_unix_s) - float(received_wall_s)
    return offset_s if abs(offset_s) <= 60.0 else None


def rtcp_sender_report_is_pc_valid(
    *,
    sr_ntp_unix_s: float,
    received_wall_s: float,
    max_offset_s: float = MAX_RTCP_SR_PC_OFFSET_S,
) -> bool:
    offset_s = rtcp_sender_report_pc_offset_s(
        sr_ntp_unix_s=sr_ntp_unix_s,
        received_wall_s=received_wall_s,
    )
    return offset_s is not None and abs(offset_s) <= float(max_offset_s)


class GstRtpTimingTracker:
    def __init__(self, Gst, GstRtp, *, status_callback: Callable[[str], None] | None = None) -> None:
        self.Gst = Gst
        self.GstRtp = GstRtp
        self._status_callback = status_callback
        self._lock = threading.Lock()
        self._pending_rtp_frames: deque[RtpFrameTiming] = deque(maxlen=MAX_RTP_TIMING_CACHE)
        self._timing_by_pts_ns: dict[int, RtpFrameTiming] = {}
        self._pts_order: deque[int] = deque(maxlen=MAX_RTP_TIMING_CACHE)
        self._sr_by_ssrc: dict[int, RtcpSenderReportTiming] = {}
        self._manager = None
        self._last_rtcp_poll_s = 0.0

    def _emit_status(self, text: str) -> None:
        if self._status_callback is not None:
            self._status_callback(text)

    def _register_sender_report(self, timing: RtcpSenderReportTiming) -> str | None:
        with self._lock:
            previous = self._sr_by_ssrc.get(timing.ssrc)
            if previous is not None and previous.ntp_raw == timing.ntp_raw:
                # The poll re-reads the same report from the GStreamer session
                # stats every 250 ms. Keep the timestamps from when this report
                # first appeared, so rtcp_sr_age_s measures how stale the anchor
                # is instead of how recently we polled.
                return None
            self._sr_by_ssrc[timing.ssrc] = timing
        if previous is None:
            return None
        interval_s = timing.received_monotonic_s - previous.received_monotonic_s
        if interval_s <= 0.0:
            return None
        return (
            "[RTCP] Sender Report 갱신: "
            f"ssrc={timing.ssrc}, interval={interval_s:.2f}s, "
            f"ntp={timing.ntp_unix_s:.6f}"
        )

    def attach(self, pipeline) -> None:
        source = pipeline.get_by_name("src")
        depay = pipeline.get_by_name("depay")
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

    def frame_metadata(
        self,
        buffer,
        now_s: float,
        *,
        received_wall_s: float | None = None,
    ) -> dict[str, int | float | str] | None:
        received_wall_s = time.time() if received_wall_s is None else float(received_wall_s)
        self._poll_rtcp_sender_reports(now_s)
        pts_ns = None if buffer.pts == self.Gst.CLOCK_TIME_NONE else int(buffer.pts)
        frame_timing = self._pop_frame_timing(pts_ns)
        if frame_timing is None:
            return None
        metadata: dict[str, int | float | str] = {
            "rtp_timestamp": frame_timing.rtp_timestamp,
            "rtp_ssrc": frame_timing.rtp_ssrc,
            "rtp_marker_seq": frame_timing.rtp_marker_seq,
        }
        if frame_timing.rtp_pts_s is not None:
            metadata["rtp_pts_s"] = frame_timing.rtp_pts_s
        sr_timing = self._sr_by_ssrc.get(frame_timing.rtp_ssrc)
        if sr_timing is None:
            return metadata
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
            }
        )
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

    def _capture_rtp_packet(self, _pad, info):
        buffer = info.get_buffer()
        if buffer is None:
            return self.Gst.PadProbeReturn.OK
        success, rtp_buffer = self.GstRtp.RTPBuffer.map(buffer, self.Gst.MapFlags.READ)
        if not success:
            return self.Gst.PadProbeReturn.OK
        try:
            if rtp_buffer.get_marker():
                timing = RtpFrameTiming(
                    rtp_timestamp=int(rtp_buffer.get_timestamp()),
                    rtp_ssrc=int(rtp_buffer.get_ssrc()),
                    rtp_marker_seq=int(rtp_buffer.get_seq()),
                    rtp_pts_s=None,
                )
                with self._lock:
                    self._pending_rtp_frames.append(timing)
        finally:
            rtp_buffer.unmap()
        return self.Gst.PadProbeReturn.OK

    def _map_depay_buffer_pts(self, _pad, info):
        buffer = info.get_buffer()
        if buffer is None or buffer.pts == self.Gst.CLOCK_TIME_NONE:
            return self.Gst.PadProbeReturn.OK
        pts_ns = int(buffer.pts)
        pts_s = pts_ns / 1e9
        with self._lock:
            if not self._pending_rtp_frames:
                return self.Gst.PadProbeReturn.OK
            timing = self._pending_rtp_frames.popleft()
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
        if abs(nearest_pts - pts_ns) > 5_000_000:
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
) -> str:
    latency_ms = max(0, int(latency_ms))
    rtsp_transport = normalize_rtsp_transport(transport)
    decoder_element = decoder_element_text(decoder)
    h264parse_element = "h264parse"
    keyframe_filter = ""
    if keyframe_only:
        h264parse_element = "h264parse config-interval=-1 disable-passthrough=true"
        keyframe_filter = (
            "! video/x-h264,alignment=au "
            "! identity drop-buffer-flags=delta-unit silent=true "
            "! queue leaky=downstream max-size-buffers=1 max-size-bytes=0 max-size-time=0 "
        )
    record_path = str(record_video_path or "").strip()
    if record_path:
        return (
            f"rtspsrc name=src location={url} latency={latency_ms} protocols={rtsp_transport} "
            "drop-on-latency=true do-rtcp=true "
            "! rtph264depay name=depay source-info=true "
            "! queue leaky=downstream max-size-buffers=1 max-size-bytes=0 max-size-time=0 "
            f"! {h264parse_element} "
            f"{keyframe_filter}"
            "! tee name=encoded "
            "encoded. ! queue leaky=downstream max-size-buffers=1 max-size-bytes=0 max-size-time=0 "
            f"! {decoder_element} "
            "! queue leaky=downstream max-size-buffers=1 max-size-bytes=0 max-size-time=0 "
            "! videoconvert "
            "! video/x-raw,format=BGR "
            "! appsink name=sink emit-signals=false sync=false max-buffers=1 drop=true "
            "encoded. ! queue max-size-buffers=0 max-size-bytes=0 max-size-time=0 "
            "! h264parse config-interval=-1 "
            "! matroskamux "
            f"! filesink location={_gst_location(record_path)} sync=false"
        )
    return (
        f"rtspsrc name=src location={url} latency={latency_ms} protocols={rtsp_transport} "
        "drop-on-latency=true do-rtcp=true "
        "! rtph264depay name=depay source-info=true "
        "! queue leaky=downstream max-size-buffers=1 max-size-bytes=0 max-size-time=0 "
        f"! {h264parse_element} "
        f"{keyframe_filter}"
        "! queue leaky=downstream max-size-buffers=1 max-size-bytes=0 max-size-time=0 "
        f"! {decoder_element} "
        "! queue leaky=downstream max-size-buffers=1 max-size-bytes=0 max-size-time=0 "
        "! videoconvert "
        "! video/x-raw,format=BGR "
        "! appsink name=sink emit-signals=false sync=false max-buffers=1 drop=true"
    )


def build_gst_recording_pipeline_text(
    url: str,
    *,
    latency_ms: int = DEFAULT_RTSP_LATENCY_MS,
    transport: str = RTSP_TRANSPORT_UDP,
    record_video_path: str,
) -> str:
    latency_ms = max(0, int(latency_ms))
    rtsp_transport = normalize_rtsp_transport(transport)
    return (
        f"rtspsrc location={url} latency={latency_ms} protocols={rtsp_transport} "
        "drop-on-latency=true do-rtcp=true "
        "! rtph264depay "
        "! h264parse config-interval=-1 "
        "! matroskamux "
        f"! filesink location={_gst_location(record_video_path)} sync=false"
    )


def build_ffmpeg_recording_command(
    url: str,
    *,
    transport: str = RTSP_TRANSPORT_UDP,
    record_video_path: str,
) -> list[str]:
    rtsp_transport = "tcp" if normalize_rtsp_transport(transport) == RTSP_TRANSPORT_TCP else "udp"
    return [
        "ffmpeg",
        "-hide_banner",
        "-loglevel",
        "warning",
        "-rtsp_transport",
        rtsp_transport,
        "-use_wallclock_as_timestamps",
        "1",
        "-fflags",
        "+genpts",
        "-i",
        str(url),
        "-an",
        "-c:v",
        "copy",
        "-movflags",
        "+faststart",
        "-y",
        str(Path(record_video_path).expanduser()),
    ]


def _iter_ffmpeg_recording_pids(record_video_path: str) -> list[int]:
    target_path = str(Path(record_video_path).expanduser())
    proc_root = Path("/proc")
    if not proc_root.exists():
        return []
    pids: list[int] = []
    for pid_dir in proc_root.iterdir():
        if not pid_dir.name.isdigit():
            continue
        try:
            cmdline_bytes = (pid_dir / "cmdline").read_bytes()
        except OSError:
            continue
        if not cmdline_bytes:
            continue
        parts = [part.decode(errors="ignore") for part in cmdline_bytes.split(b"\0") if part]
        if not parts:
            continue
        if Path(parts[0]).name != "ffmpeg":
            continue
        if target_path in parts:
            pids.append(int(pid_dir.name))
    return pids


def _stop_stale_ffmpeg_recorders(record_video_path: str, output_queue) -> None:
    stale_pids = _iter_ffmpeg_recording_pids(record_video_path)
    if not stale_pids:
        return
    for pid in stale_pids:
        try:
            os.killpg(pid, signal.SIGINT)
        except ProcessLookupError:
            continue
        except OSError:
            try:
                os.kill(pid, signal.SIGINT)
            except OSError:
                continue
    deadline_s = time.monotonic() + 3.0
    while time.monotonic() < deadline_s:
        if not _iter_ffmpeg_recording_pids(record_video_path):
            _emit_status(output_queue, f"기존 ffmpeg MP4 녹화 정리 완료: {record_video_path}")
            return
        time.sleep(0.1)
    for pid in _iter_ffmpeg_recording_pids(record_video_path):
        try:
            os.killpg(pid, signal.SIGTERM)
        except ProcessLookupError:
            continue
        except OSError:
            try:
                os.kill(pid, signal.SIGTERM)
            except OSError:
                continue
    _emit_status(output_queue, f"기존 ffmpeg MP4 녹화 강제 정리 요청: {record_video_path}")


def _start_ffmpeg_encoded_recording_process(config: dict[str, Any], output_queue):
    record_path = _record_video_path(config)
    if not record_path:
        return None
    if shutil.which("ffmpeg") is None:
        _emit_status(output_queue, "ffmpeg 없음: MP4 원본 녹화를 시작할 수 없습니다.")
        return None
    try:
        Path(record_path).expanduser().parent.mkdir(parents=True, exist_ok=True)
        _stop_stale_ffmpeg_recorders(record_path, output_queue)
        command = build_ffmpeg_recording_command(
            str(config["url"]),
            transport=str(config.get("rtsp_transport", RTSP_TRANSPORT_UDP)),
            record_video_path=record_path,
        )
        process = subprocess.Popen(
            command,
            stdin=subprocess.PIPE,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
            start_new_session=True,
        )
        _emit_status(output_queue, f"ffmpeg H264 MP4 원본 녹화 시작: {record_path}")
        return process
    except Exception as exc:
        _emit_status(output_queue, f"ffmpeg H264 MP4 원본 녹화 시작 실패: {exc}")
        return None


def _stop_ffmpeg_encoded_recording_process(process, output_queue, record_video_path: str) -> None:
    if process is None:
        return
    try:
        if process.poll() is None:
            if process.stdin is not None:
                try:
                    process.stdin.write(b"q\n")
                    process.stdin.flush()
                    process.stdin.close()
                except Exception:
                    pass
            try:
                process.wait(timeout=8.0)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGINT)
                try:
                    process.wait(timeout=5.0)
                except subprocess.TimeoutExpired:
                    os.killpg(process.pid, signal.SIGTERM)
                    process.wait(timeout=2.0)
        _emit_status(output_queue, f"ffmpeg H264 MP4 원본 녹화 종료: {record_video_path} (exit={process.returncode})")
    except Exception as exc:
        _emit_status(output_queue, f"ffmpeg H264 MP4 원본 녹화 종료 실패: {exc}")


def _start_gst_encoded_recording_process(config: dict[str, Any], output_queue):
    record_path = _record_video_path(config)
    if not record_path:
        return None
    try:
        Path(record_path).expanduser().parent.mkdir(parents=True, exist_ok=True)
        pipeline_text = build_gst_recording_pipeline_text(
            str(config["url"]),
            latency_ms=int(config.get("rtsp_latency_ms", DEFAULT_RTSP_LATENCY_MS)),
            transport=str(config.get("rtsp_transport", RTSP_TRANSPORT_UDP)),
            record_video_path=record_path,
        )
        process = subprocess.Popen(
            ["gst-launch-1.0", "-e", *shlex.split(pipeline_text)],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
            start_new_session=True,
        )
        _emit_status(output_queue, f"GStreamer H264 원본 녹화 프로세스 시작: {record_path}")
        return process
    except Exception as exc:
        _emit_status(output_queue, f"GStreamer H264 원본 녹화 프로세스 시작 실패: {exc}")
        return None


def _stop_gst_encoded_recording_process(process, output_queue, record_video_path: str) -> None:
    if process is None:
        return
    try:
        if process.poll() is None:
            os.killpg(process.pid, signal.SIGINT)
            try:
                process.wait(timeout=5.0)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGTERM)
                process.wait(timeout=2.0)
        _emit_status(
            output_queue,
            f"GStreamer H264 원본 녹화 프로세스 종료: {record_video_path} (exit={process.returncode})",
        )
    except Exception as exc:
        _emit_status(output_queue, f"GStreamer H264 원본 녹화 프로세스 종료 실패: {exc}")


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
                raise RuntimeError(f"{decoder} failed to enter PLAYING state")
            _emit_status(output_queue, f"GStreamer 디코더: {decoder}")
            if record_video_path:
                _emit_status(output_queue, f"GStreamer H264 원본 녹화 시작: {record_video_path}")
            return pipeline, sink, bus, decoder
        except Exception as exc:
            last_error = f"{decoder}: {exc}"
            if pipeline is not None:
                pipeline.set_state(Gst.State.NULL)
            if normalize_decoder_mode(decoder_mode) == DECODER_AUTO and decoder == NVIDIA_H264_DECODER:
                _emit_status(output_queue, f"NVIDIA 디코더 사용 불가, software로 재시도: {exc}")
                continue
            break
    raise RuntimeError(last_error or "no usable decoder")


def _finalize_gst_recording(Gst, pipeline, bus, output_queue, record_video_path: str) -> None:
    if not record_video_path:
        return
    try:
        pipeline.send_event(Gst.Event.new_eos())
        deadline_s = time.monotonic() + 3.0
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
                return
            if message.type == Gst.MessageType.EOS:
                _emit_status(output_queue, f"GStreamer H264 원본 녹화 저장 완료: {record_video_path}")
                return
        _emit_status(output_queue, f"GStreamer 녹화 EOS 대기 시간 초과: {record_video_path}")
    except Exception as exc:
        _emit_status(output_queue, f"GStreamer 녹화 종료 실패: {exc}")


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
                last_error_s = float(config.get("_last_inference_error_s", 0.0) or 0.0)
                if now_s - last_error_s >= 1.0:
                    _emit_status(output_queue, f"추론 오류: {exc}")
                    config["_last_inference_error_s"] = now_s
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
            detections = detections_from_ultralytics_result(result, names=class_names)
            best_detection = highest_confidence_detection(detections)
            if draw_preview_overlay and best_detection is not None:
                annotated = _draw_detections(cv2, frame, [best_detection], best_detection)
            inferred = True
            if best_detection is not None and bool(config.get("emit_measurement_frames", False)):
                _emit_measurement_frame(cv2, annotated, config, output_queue, timestamp_s=now_s)
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
) -> None:
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
    }
    if extra_metadata:
        metadata.update(extra_metadata)
    metadata.setdefault(
        "timestamp_reference",
        "rtcp-invalid"
        if metadata.get("rtcp_timing_invalid_reason")
        else "rtcp-ntp"
        if metadata.get("rtcp_frame_ntp_unix_s") is not None
        else "source"
        if metadata.get("source_timestamp_s") is not None
        else "app-receive",
    )
    output_queue.put(("frame", metadata, rgb.shape[1], rgb.shape[0], rgb.tobytes()))


def _emit_measurement_frame(
    cv2,
    frame,
    config: dict[str, Any],
    output_queue,
    *,
    timestamp_s: float,
) -> None:
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
    output_queue.put(("measurement_frame", float(timestamp_s), rgb.shape[1], rgb.shape[0], rgb.tobytes()))


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


def _run_gstreamer_stream(config: dict[str, Any], output_queue, stop_event, cv2, model, enable_detection: bool) -> None:
    try:
        import gi
        import numpy as np

        gi.require_version("Gst", "1.0")
        gi.require_version("GstRtp", "1.0")
        from gi.repository import Gst
        from gi.repository import GstRtp
    except Exception as exc:
        _emit_status(output_queue, f"GStreamer 사용 불가: {exc}")
        return

    Gst.init(None)
    keyframe_only = bool(config.get("keyframe_only", False)) or env_flag_enabled(I_FRAME_ONLY_ENV)
    rtp_timing_tracker = GstRtpTimingTracker(
        Gst,
        GstRtp,
        status_callback=lambda text: _emit_status(output_queue, text),
    )
    record_video_path = _record_video_path(config)
    recording_process = _start_ffmpeg_encoded_recording_process(config, output_queue)
    try:
        pipeline, sink, bus, _decoder = _launch_gst_pipeline(
            Gst,
            config["url"],
            latency_ms=int(config.get("rtsp_latency_ms", DEFAULT_RTSP_LATENCY_MS)),
            decoder_mode=str(config.get("decoder", DECODER_AUTO)),
            transport=str(config.get("rtsp_transport", RTSP_TRANSPORT_UDP)),
            keyframe_only=keyframe_only,
            record_video_path="",
            output_queue=output_queue,
            rtp_timing_tracker=rtp_timing_tracker,
        )
    except Exception as exc:
        _emit_status(output_queue, f"GStreamer 오류: {exc}")
        _stop_ffmpeg_encoded_recording_process(recording_process, output_queue, record_video_path)
        return

    rate_limiter = AdaptiveYoloRateLimiter.from_config(config, enable_detection=enable_detection)
    next_frame_s = 0.0
    last_status_s = 0.0
    last_stats_s = time.monotonic()
    displayed_frames = 0
    inferred_frames = 0
    latest_lock = threading.Lock()
    latest_frame: dict[str, Any] | None = None
    latest_sequence = 0
    decoded_frames = 0
    latest_width = 0
    latest_height = 0
    reader_stop = threading.Event()
    def reader_loop() -> None:
        nonlocal decoded_frames, displayed_frames, latest_frame, latest_height, latest_sequence, latest_width
        while not stop_event.is_set() and not reader_stop.is_set():
            sample = sink.emit("try-pull-sample", 200_000_000)
            if sample is None:
                continue
            receive_s = time.monotonic()
            receive_wall_s = time.time()
            caps = sample.get_caps().get_structure(0)
            width = int(caps.get_value("width"))
            height = int(caps.get_value("height"))
            buffer = sample.get_buffer()
            source_timestamp_s = None if buffer.pts == Gst.CLOCK_TIME_NONE else buffer.pts / 1e9
            rtp_timing_metadata = rtp_timing_tracker.frame_metadata(
                buffer,
                receive_s,
                received_wall_s=receive_wall_s,
            ) or {}
            success, map_info = buffer.map(Gst.MapFlags.READ)
            if not success:
                continue
            try:
                frame = np.frombuffer(map_info.data, dtype=np.uint8).reshape((height, width, 3)).copy()
            finally:
                buffer.unmap(map_info)
            _emit_preview_frame(
                cv2,
                frame,
                config,
                output_queue,
                timestamp_s=receive_s,
                source_timestamp_s=source_timestamp_s,
                timestamp_source="gstreamer-pts",
                extra_metadata=rtp_timing_metadata,
            )
            with latest_lock:
                latest_sequence += 1
                decoded_frames += 1
                displayed_frames += 1
                latest_width = width
                latest_height = height
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
    if keyframe_only:
        _emit_status(output_queue, "GStreamer I-frame only 진단 모드")
    else:
        _emit_status(output_queue, "GStreamer YOLO 실행 중" if enable_detection else "GStreamer 카메라 스트림 실행 중")
    last_processed_sequence = 0
    try:
        while not stop_event.is_set():
            message = bus.pop_filtered(Gst.MessageType.ERROR | Gst.MessageType.EOS)
            if message is not None:
                if message.type == Gst.MessageType.ERROR:
                    error, debug = message.parse_error()
                    detail = error.message if debug is None else f"{error.message} ({debug})"
                    _emit_status(output_queue, f"스트림 오류: {detail}")
                else:
                    _emit_status(output_queue, "스트림 종료")
                break

            now_s = time.monotonic()
            if now_s - last_stats_s >= 1.0:
                elapsed_s = now_s - last_stats_s
                with latest_lock:
                    interval_decoded_frames = decoded_frames
                    decoded_frames = 0
                    interval_displayed_frames = displayed_frames
                    displayed_frames = 0
                    stats_width = latest_width
                    stats_height = latest_height
                decoded_hz = interval_decoded_frames / elapsed_s
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
            if inferred:
                inferred_frames += 1
            last_status_s = _emit_stream_status(output_queue, enable_detection, best_detection, now_s, last_status_s)

    finally:
        reader_stop.set()
        pipeline.set_state(Gst.State.NULL)
        reader.join(timeout=1.0)
        _stop_ffmpeg_encoded_recording_process(recording_process, output_queue, record_video_path)
        _emit_status(output_queue, "중지됨")


def _run_opencv_stream(config: dict[str, Any], output_queue, stop_event, cv2, model, enable_detection: bool) -> None:
    transport = normalize_rtsp_transport(str(config.get("rtsp_transport", RTSP_TRANSPORT_UDP)))
    os.environ["OPENCV_FFMPEG_CAPTURE_OPTIONS"] = f"rtsp_transport;{transport}|stimeout;2000000|max_delay;500000"
    backend = normalize_stream_backend(str(config.get("stream_backend", STREAM_BACKEND_OPENCV)))
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
    decoded_frames = 0
    reader_stop = threading.Event()
    record_writer_state = None

    def reader_loop() -> None:
        nonlocal decoded_frames, displayed_frames, height, latest_frame, latest_sequence, record_writer_state, width
        while not stop_event.is_set() and not reader_stop.is_set():
            ok, frame = cap.read()
            receive_s = time.monotonic()
            if not ok or frame is None:
                time.sleep(0.02)
                continue
            frame_height, frame_width = frame.shape[:2]
            if _record_video_path(config):
                if record_writer_state is None:
                    record_writer_state = _open_record_video_writer(cv2, config, output_queue, frame_width, frame_height)
                _write_record_video_frame(cv2, record_writer_state, frame)
            _emit_preview_frame(
                cv2,
                frame,
                config,
                output_queue,
                timestamp_s=receive_s,
                timestamp_source=f"{backend}-read",
            )
            with latest_lock:
                latest_sequence += 1
                decoded_frames += 1
                displayed_frames += 1
                width = frame_width
                height = frame_height
                latest_frame = {
                    "sequence": latest_sequence,
                    "frame": frame,
                    "timestamp_s": receive_s,
                    "width": frame_width,
                    "height": frame_height,
                }

    reader = threading.Thread(target=reader_loop, name=f"stereo-{backend}-reader", daemon=True)
    reader.start()
    _emit_status(output_queue, f"{backend_name} YOLO 실행 중" if enable_detection else f"{backend_name} 카메라 스트림 실행 중")
    last_processed_sequence = 0
    try:
        while not stop_event.is_set():
            now_s = time.monotonic()
            if now_s - last_stats_s >= 1.0:
                elapsed_s = now_s - last_stats_s
                with latest_lock:
                    interval_decoded_frames = decoded_frames
                    decoded_frames = 0
                    interval_displayed_frames = displayed_frames
                    displayed_frames = 0
                    stats_width = width
                    stats_height = height
                decoded_hz = interval_decoded_frames / elapsed_s
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
    decoded_frames = 0
    reader_stop = threading.Event()
    record_writer_state = None

    def reader_loop() -> None:
        nonlocal decoded_frames, displayed_frames, height, latest_frame, latest_sequence, record_writer_state, width
        try:
            for packet in container.demux(video_stream):
                if stop_event.is_set() or reader_stop.is_set():
                    break
                for av_frame in packet.decode():
                    if stop_event.is_set() or reader_stop.is_set():
                        break
                    receive_s = time.monotonic()
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
                    _emit_preview_frame(
                        cv2,
                        frame,
                        config,
                        output_queue,
                        timestamp_s=receive_s,
                        source_timestamp_s=source_timestamp_s,
                        timestamp_source=timestamp_source,
                    )
                    with latest_lock:
                        latest_sequence += 1
                        decoded_frames += 1
                        displayed_frames += 1
                        width = frame_width
                        height = frame_height
                        latest_frame = {
                            "sequence": latest_sequence,
                            "frame": frame,
                            "timestamp_s": receive_s,
                            "source_timestamp_s": source_timestamp_s,
                            "timestamp_source": timestamp_source,
                            "timestamp_reference": timestamp_reference,
                            "width": frame_width,
                            "height": frame_height,
                        }
        except Exception as exc:
            if not stop_event.is_set() and not reader_stop.is_set():
                _emit_status(output_queue, f"PyAV 스트림 오류: {exc}")

    reader = threading.Thread(target=reader_loop, name="stereo-pyav-reader", daemon=True)
    reader.start()
    _emit_status(output_queue, "PyAV YOLO 실행 중" if enable_detection else "PyAV 카메라 스트림 실행 중")
    last_processed_sequence = 0
    try:
        while not stop_event.is_set():
            now_s = time.monotonic()
            if now_s - last_stats_s >= 1.0:
                elapsed_s = now_s - last_stats_s
                with latest_lock:
                    interval_decoded_frames = decoded_frames
                    decoded_frames = 0
                    interval_displayed_frames = displayed_frames
                    displayed_frames = 0
                    stats_width = width
                    stats_height = height
                decoded_hz = interval_decoded_frames / elapsed_s
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


class _BoundedProcessOutputQueue:
    def __init__(self, event_queue, frame_queue) -> None:
        self.event_queue = event_queue
        self.frame_queue = frame_queue

    def put(self, message) -> None:
        if message and message[0] == "frame":
            try:
                self.frame_queue.put_nowait(message)
            except queue.Full:
                return
            return
        self.event_queue.put(message)


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
    detection_ready = QtCore.pyqtSignal(dict)
    stats_ready = QtCore.pyqtSignal(dict)
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
        display_rate_hz: float = 0.0,
        preview_max_width: int = 960,
        rtsp_latency_ms: int = DEFAULT_RTSP_LATENCY_MS,
        decoder: str = DECODER_NVIDIA,
        stream_backend: str = STREAM_BACKEND_GSTREAMER,
        rtsp_transport: str = RTSP_TRANSPORT_UDP,
        enable_detection: bool = True,
        record_video_path: str = "",
        record_video_fps: float = 30.0,
        event_queue_size: int = DEFAULT_EVENT_QUEUE_SIZE,
        frame_queue_size: int = DEFAULT_FRAME_QUEUE_SIZE,
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
            "decoder": normalize_decoder_mode(decoder),
            "stream_backend": normalize_stream_backend(stream_backend),
            "rtsp_transport": normalize_rtsp_transport(rtsp_transport),
            "enable_detection": enable_detection,
            "emit_measurement_frames": enable_detection,
            "adaptive_rate": adaptive_rate,
            "adaptive_max_rate_hz": DEFAULT_CPU_YOLO_ADAPTIVE_MAX_RATE_HZ if adaptive_rate else 0.0,
            "worker_cpu_threads": _default_worker_cpu_threads(device),
            "record_video_path": str(record_video_path or ""),
            "record_video_fps": _record_video_fps({"record_video_fps": record_video_fps}),
            "event_queue_size": max(1, int(event_queue_size)),
            "frame_queue_size": max(1, int(frame_queue_size)),
        }
        self._mp_context = multiprocessing.get_context("spawn")
        self._stop_event = None
        self._event_queue = None
        self._frame_queue = None
        self._process = None
        self._stop_requested = False
        self._finished = False

    @QtCore.pyqtSlot()
    def run(self) -> None:
        status = "YOLO worker 시작 중" if self.config["enable_detection"] else "카메라 worker 시작 중"
        self.status_changed.emit(status)
        self._stop_requested = False
        self._stop_event = self._mp_context.Event()
        self._event_queue = self._mp_context.Queue(maxsize=int(self.config["event_queue_size"]))
        self._frame_queue = self._mp_context.Queue(maxsize=int(self.config["frame_queue_size"]))
        process = self._mp_context.Process(
            target=_run_yolo_process,
            args=(self.config, _BoundedProcessOutputQueue(self._event_queue, self._frame_queue), self._stop_event),
            daemon=True,
        )
        self._process = process
        try:
            process.start()
            self.status_changed.emit(f"{status}: pid={process.pid}")
            while process.is_alive() or self._queue_has_pending_messages():
                self._drain_output_queue(timeout_s=0.1)
            process.join(timeout=0.2)
            if process.exitcode not in (0, None) and not self._stop_requested:
                self.status_changed.emit(f"스트림 프로세스 종료 코드: {process.exitcode}")
        except Exception as exc:
            self.status_changed.emit(f"스트림 프로세스 오류: {exc}")
        finally:
            self._stop_child_process()
            self._emit_finished_once()

    def stop(self) -> None:
        self._stop_requested = True
        if self._stop_event is not None:
            self._stop_event.set()
        process = self._process
        if process is not None and process.is_alive():
            process.terminate()
            process.join(timeout=2.0)
            if process.is_alive():
                process.kill()
                process.join(timeout=1.0)

    def request_stop(self) -> None:
        self._stop_requested = True
        if self._stop_event is not None:
            self._stop_event.set()

    def _queue_has_pending_messages(self) -> bool:
        return _queue_has_pending_messages(self._event_queue) or _queue_has_pending_messages(self._frame_queue)

    def _drain_output_queue(self, *, timeout_s: float) -> None:
        try:
            message = _get_queue_message(self._event_queue, timeout_s=0.0)
        except queue.Empty:
            try:
                message = _get_queue_message(self._frame_queue, timeout_s=timeout_s)
            except queue.Empty:
                return
        self._handle_worker_message(message)
        while True:
            try:
                message = _get_queue_message_nowait(self._event_queue)
            except queue.Empty:
                break
            self._handle_worker_message(message)
        while True:
            try:
                message = _get_queue_message_nowait(self._frame_queue)
            except queue.Empty:
                break
            self._handle_worker_message(message)

    def _stop_child_process(self) -> None:
        process = self._process
        if process is not None and process.is_alive():
            if self._stop_event is not None:
                self._stop_event.set()
            process.join(timeout=1.0)
            if process.is_alive():
                process.terminate()
                process.join(timeout=1.0)
        self._process = None
        self._stop_event = None
        self._event_queue = None
        self._frame_queue = None

    def _handle_worker_message(self, message) -> None:
        kind = message[0]
        if kind == "status":
            self.status_changed.emit(message[1])
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
            _kind, timestamp_s, width, height, data = message
            image = QtGui.QImage(
                data,
                width,
                height,
                width * 3,
                QtGui.QImage.Format_RGB888,
            ).copy()
            self.measurement_frame_ready.emit(float(timestamp_s), image)
            return
        if kind == "detection":
            if len(message) >= 7:
                _kind, timestamp_s, width, height, best_detection, detections, timestamp_metadata = message
            else:
                _kind, timestamp_s, width, height, best_detection, detections = message
                timestamp_metadata = {}
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
                ) = message
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
            self.stats_ready.emit(
                {
                    "decoded_hz": decoded_hz,
                    "displayed_hz": displayed_hz,
                    "inferred_hz": inferred_hz,
                    "width": width,
                    "height": height,
                    "decoder": self.config["decoder"],
                    "stream_backend": self.config["stream_backend"],
                    "rtsp_transport": self.config["rtsp_transport"],
                    "enable_detection": self.config["enable_detection"],
                    "yolo_target_hz": target_rate_hz,
                    "yolo_requested_device": requested_device,
                    "yolo_actual_device": active_device,
                    "yolo_cpu_fallback": cpu_fallback,
                }
            )

    def _emit_finished_once(self) -> None:
        if self._finished:
            return
        self._finished = True
        self.finished.emit()
