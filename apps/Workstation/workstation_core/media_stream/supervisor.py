"""supervisor owner extracted from the Workstation media runtime.

Existing queues, locks, clocks, source-frame identity and policies are preserved.
Native pixels use image u/v and RGB/BGR/NV12; no world/body transform is introduced.
"""

from __future__ import annotations

import multiprocessing
import queue
import sys
import time
from collections.abc import Mapping
from typing import Callable
from PyQt5 import QtCore, QtGui
from Workstation.workstation_core.mp4_integrity import validate_h264_copy_mp4_integrity
from Workstation.workstation_core.native_frame_ring import (
    DEFAULT_NATIVE_MAX_HEIGHT,
    DEFAULT_NATIVE_MAX_WIDTH,
    DEFAULT_NATIVE_RING_CAPACITY,
    SharedNativeFrameRing,
)
from Workstation.workstation_core.settings import RTSP_ENCODED_QUEUE_MODE_LATEST, normalize_rtsp_encoded_queue_mode
from .child_runtime import _run_yolo_process
from .configuration import (
    _device_uses_cpu,
    _record_video_fps,
    effective_yolo_max_rate_hz,
    normalize_rtsp_reconnect_policy,
    normalize_rtsp_transport,
    normalize_stream_backend,
    normalize_yolo_input_size,
    recording_integrity_contract,
)
from .decoder_policy import normalize_decoder_mode
from .ipc import (
    CameraIpcCounters,
    _BoundedProcessOutputQueue,
    _get_queue_message,
    _get_queue_message_nowait,
    _queue_has_pending_messages,
)
from .policy import (
    DECODER_AUTO,
    DECODER_FALLBACK_ACTIVE_STATUS_PREFIX,
    DECODER_FALLBACK_STATUS_PREFIX,
    DEFAULT_CPU_YOLO_ADAPTIVE_MAX_RATE_HZ,
    DEFAULT_DECODE_RATE_WINDOW_S,
    DEFAULT_EVENT_QUEUE_SIZE,
    DEFAULT_FRAME_QUEUE_SIZE,
    DEFAULT_METADATA_QUEUE_SIZE,
    DEFAULT_PREVIEW_DISPLAY_RATE_HZ,
    DEFAULT_RTSP_LATENCY_MS,
    RTSP_RECONNECT_MANUAL,
    RTSP_TRANSPORT_UDP,
    STREAM_BACKEND_GSTREAMER,
    YOLO_COOPERATIVE_STOP_TIMEOUT_S,
    YOLO_KILL_AFTER_STOP_S,
    YOLO_KILL_JOIN_TIMEOUT_S,
    YOLO_PROCESS_CRASH_RESTART_DELAYS_S,
    YOLO_PROCESS_STABLE_RUN_S,
)
from .worker_threads import _default_worker_cpu_threads


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
            "rtsp_encoded_queue_mode": normalize_rtsp_encoded_queue_mode(rtsp_encoded_queue_mode),
            "decoder": normalize_decoder_mode(decoder),
            "stream_backend": normalize_stream_backend(stream_backend),
            "rtsp_transport": normalize_rtsp_transport(rtsp_transport),
            "rtsp_reconnect_policy": normalize_rtsp_reconnect_policy(rtsp_reconnect_policy),
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
        # Ported from the now-deleted measurement_package_controller.py's
        # "package worker" (see the EOS handler around
        # _run_gstreamer_stream_once): full finalized-file H.264 copy
        # integrity validation was previously only performed by that
        # sessionless-migration-orphaned controller, so it never actually ran
        # in production. Wiring it in here closes that gap without waiting
        # for a separate post-hoc verification pass.
        self.config["_recording_integrity_validator"] = validate_h264_copy_mp4_integrity
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
        self._crash_restart_delays_s = tuple(max(0.0, float(delay_s)) for delay_s in crash_restart_delays_s)
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
                self._control_queue = self._mp_context.Queue(maxsize=int(self.config["event_queue_size"]))
                self._event_queue = self._control_queue
                self._metadata_queue = self._mp_context.Queue(maxsize=int(self.config["metadata_queue_size"]))
                self._frame_queue = self._mp_context.Queue(maxsize=int(self.config["frame_queue_size"]))
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
                        "스트림 자식 프로세스 종료 확인 실패; 재시작하지 않고 종료 검증을 대기합니다."
                    )
                    self._stop_requested = True
                    break
                if self._stop_requested:
                    break
                if self._child_terminal_exit_reason:
                    self.status_changed.emit(
                        f"스트림 프로세스 영구 종료: {self._child_terminal_exit_reason}; 자동 재시작 안 함"
                    )
                    break
                if self.config["rtsp_reconnect_policy"] == RTSP_RECONNECT_MANUAL:
                    detail = launch_error or f"exitcode={exitcode}"
                    self.status_changed.emit(f"스트림 프로세스 종료: {detail}; 자동 재시작 안 함")
                    break
                if runtime_s >= YOLO_PROCESS_STABLE_RUN_S:
                    consecutive_crashes = 0
                consecutive_crashes += 1
                if consecutive_crashes > len(self._crash_restart_delays_s):
                    detail = launch_error or f"exitcode={exitcode}"
                    self.status_changed.emit(
                        f"스트림 프로세스 크래시 복구 한도 초과: {detail}; 수동 재시작이 필요합니다."
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
        # _stop_child_process() already implements the full cooperative
        # join -> terminate -> join -> kill -> join escalation (using the
        # same YOLO_COOPERATIVE_STOP_TIMEOUT_S/YOLO_KILL_AFTER_STOP_S/
        # YOLO_KILL_JOIN_TIMEOUT_S budgets) plus queue/process-handle
        # cleanup, so there is nothing left for stop() to duplicate here.
        self.request_stop()
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
                metadata["app_receive_monotonic_ns"] = int(round(float(timestamp_s) * 1_000_000_000.0))
                metadata.setdefault("app_receive_timestamp_role", "legacy_float_derived")
            else:
                metadata.setdefault("app_receive_timestamp_role", "authoritative_distance")

        if kind == "decoder_fallback":
            payload = dict(message[1] or {}) if len(message) > 1 else {}
            reason = str(payload.get("reason") or "unknown decoder fallback")
            recording_restart_required = bool(payload.get("recording_restart_required", False))
            if recording_restart_required:
                self._child_terminal_exit_reason = "recording-side decoder fallback requires a new side start"
                prefix = DECODER_FALLBACK_STATUS_PREFIX
            else:
                prefix = DECODER_FALLBACK_ACTIVE_STATUS_PREFIX
            self.status_changed.emit(f"{prefix} {reason}")
            return
        if kind == "terminal_exit":
            payload = dict(message[1] or {}) if len(message) > 1 else {}
            self._child_terminal_exit_reason = str(payload.get("reason") or "child requested permanent exit")
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
            native_ring_metrics = self.native_frame_ring.metrics() if self.native_frame_ring is not None else {}
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
                    "rtsp_reconnect_policy": self.config["rtsp_reconnect_policy"],
                    "recording_encoding_mode": self.config["recording_encoding_mode"],
                    "recording_integrity_verified": self.config["recording_integrity_verified"],
                    "recording_integrity_scope": self.config["recording_integrity_scope"],
                    "enable_detection": self.config["enable_detection"],
                    "yolo_target_hz": target_rate_hz,
                    "yolo_requested_device": requested_device,
                    "yolo_actual_device": active_device,
                    "yolo_cpu_fallback": cpu_fallback,
                    **stream_metrics,
                    "preview_queue_capacity": int(self.config["frame_queue_size"]),
                    "metadata_queue_capacity": int(self.config["metadata_queue_size"]),
                    "control_queue_capacity": int(self.config["event_queue_size"]),
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


# Retain legacy pickle names without importing the facade.
YoloStreamWorker.__module__ = "Workstation.workstation_core.yolo_stream"
