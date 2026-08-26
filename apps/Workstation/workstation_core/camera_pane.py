from __future__ import annotations

import copy
import math
import os
import threading
import time
import weakref
from collections import deque
from pathlib import Path

from PyQt5 import QtCore, QtGui, QtWidgets

from Workstation.workstation_core.stereo import DetectionObservation
from Workstation.workstation_core.theme import ACCENT
from Workstation.workstation_core.native_frame_ring import (
    FrameKey,
    SharedNativeFrameRing,
)
from Workstation.workstation_core.yolo_stream import (
    DEFAULT_PREVIEW_DISPLAY_RATE_HZ,
    YoloStreamWorker,
)


DEFAULT_NATIVE_CALIBRATION_FRAME_MAX_AGE_S = 2.0
MAX_GUI_PREVIEW_RATE_HZ = DEFAULT_PREVIEW_DISPLAY_RATE_HZ
GUI_HEARTBEAT_INTERVAL_MS = 250
CAMERA_STOP_COOPERATIVE_WAIT_MS = 12_000
CAMERA_STOP_POST_ESCALATION_WAIT_MS = 1_000
CAMERA_STOP_TERMINATE_AFTER_S = 12.0
CAMERA_STOP_KILL_AFTER_S = 20.0


def infer_camera_timestamp_reference(metadata: dict) -> str:
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


class _AspectRatioPreviewContainer(QtWidgets.QWidget):
    """Sizes its single child to a locked width:height ratio, centered.

    The label and its pixmap both preserve the source ratio.  No render path
    is allowed to enlarge-and-crop the decoded camera image.
    """

    def __init__(self, child: QtWidgets.QWidget, parent: QtWidgets.QWidget | None = None) -> None:
        super().__init__(parent)
        self._child = child
        child.setParent(self)
        self._aspect_ratio: float | None = None
        self.setSizePolicy(QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Expanding)
        self.setMinimumSize(child.minimumSize())

    def set_aspect_ratio(self, ratio: float | None) -> None:
        if ratio is not None and (ratio <= 0 or ratio != ratio):  # reject <=0 and NaN
            ratio = None
        if ratio == self._aspect_ratio:
            return
        self._aspect_ratio = ratio
        self._apply_child_geometry(self.size())

    def resizeEvent(self, event) -> None:  # noqa: N802 - Qt override.
        self._apply_child_geometry(event.size())
        super().resizeEvent(event)

    def _apply_child_geometry(self, available: QtCore.QSize) -> None:
        # Deliberately setGeometry() rather than setFixedSize(): a fixed size
        # policy would make the child's current size a hard minimum for this
        # container (and everything above it), so the pane could grow to fit
        # a wide frame but could never shrink back down again.
        width = max(1, available.width())
        height = max(1, available.height())
        if self._aspect_ratio is None:
            target_w, target_h = width, height
        else:
            candidate_width = height * self._aspect_ratio
            if candidate_width <= width:
                target_w, target_h = max(1, round(candidate_width)), height
            else:
                target_w, target_h = width, max(1, round(width / self._aspect_ratio))
        x = (width - target_w) // 2
        y = (height - target_h) // 2
        self._child.setGeometry(x, y, target_w, target_h)


class _CameraStreamSignalAdapter(QtCore.QObject):
    """Give non-preview worker signals an explicit GUI-thread receiver."""

    def __init__(self, pane: "CameraPane", generation: int) -> None:
        super().__init__(pane)
        self._pane_ref = weakref.ref(pane)
        self._generation = int(generation)

    @QtCore.pyqtSlot(object, QtGui.QImage)
    def native_frame(self, metadata: object, image: QtGui.QImage) -> None:
        pane = self._pane_ref()
        if pane is not None:
            pane._emit_native_measurement_frame_if_current(
                self._generation,
                metadata,
                image,
            )

    @QtCore.pyqtSlot(dict)
    def detection(self, payload: dict) -> None:
        pane = self._pane_ref()
        if pane is not None:
            pane._handle_detection_payload_if_current(self._generation, payload)

    @QtCore.pyqtSlot(dict)
    def stats(self, payload: dict) -> None:
        pane = self._pane_ref()
        if pane is not None:
            pane._handle_stats_payload_if_current(self._generation, payload)

    @QtCore.pyqtSlot(str)
    def status(self, value: str) -> None:
        pane = self._pane_ref()
        if pane is not None:
            pane._set_status_if_current(self._generation, value)


class CameraPane(QtWidgets.QGroupBox):
    detection_changed = QtCore.pyqtSignal(str, object)
    frame_changed = QtCore.pyqtSignal(str, QtGui.QImage, object)
    measurement_frame_changed = QtCore.pyqtSignal(str, QtGui.QImage, object)
    native_measurement_frame_changed = QtCore.pyqtSignal(str, object, QtGui.QImage)
    stats_changed = QtCore.pyqtSignal(str, object)
    state_changed = QtCore.pyqtSignal(str, str)

    def __init__(self, label: str) -> None:
        super().__init__(label)
        self.setObjectName("cameraPane")
        self.camera_label = label
        self.worker: YoloStreamWorker | None = None
        self.thread: QtCore.QThread | None = None
        self.latest_image: QtGui.QImage | None = None
        self.latest_detection: DetectionObservation | None = None
        self.measurement_frame_images: dict[float, QtGui.QImage] = {}
        self.measurement_frame_metadata: dict[float, dict[str, object]] = {}
        # Only lightweight descriptors live in the GUI. Native rasters remain
        # in the bounded shared-memory ring until an exact FrameKey is pinned.
        self.measurement_frame_order: deque[float] = deque(maxlen=16)
        self.native_frame_ring: SharedNativeFrameRing | None = None
        self._retired_native_frame_rings: list[SharedNativeFrameRing] = []
        self.native_frame_descriptors: dict[str, dict[str, object]] = {}
        self.native_frame_descriptor_order: deque[str] = deque(maxlen=64)
        self.measurement_pair_display_active = False
        self.selected_measurement_frame_timestamp_s: float | None = None
        self.selected_measurement_frame_lookup_delta_s: float | None = None
        self.detection_stream_active = False
        self.stream_generation = 0
        self.current_status = "중지됨"
        self.recording_finalizations: dict[str, dict[str, object]] = {}
        self._stop_requested_monotonic_s: float | None = None
        self._stop_escalation_level = 0
        self._preview_lock = threading.Lock()
        self._pending_preview: tuple[int, QtGui.QImage, dict[str, object]] | None = None
        self._preview_submitted = 0
        self._preview_rendered = 0
        self._preview_replaced = 0
        self._last_preview_age_s: float | None = None
        self._max_preview_age_s = 0.0
        self._last_stream_stats: dict[str, object] = {}
        self._gui_heartbeat_count = 0
        self._gui_heartbeat_last_s: float | None = None
        self._gui_heartbeat_max_delay_s = 0.0
        self._preview_frame_callback = None
        self._stream_signal_adapter: _CameraStreamSignalAdapter | None = None
        self._preview_timer = QtCore.QTimer(self)
        self._preview_timer.setTimerType(QtCore.Qt.PreciseTimer)
        self._preview_timer.setInterval(round(1000.0 / MAX_GUI_PREVIEW_RATE_HZ))
        self._preview_timer.timeout.connect(self._drain_preview_frame)
        self._gui_heartbeat_timer = QtCore.QTimer(self)
        self._gui_heartbeat_timer.setTimerType(QtCore.Qt.PreciseTimer)
        self._gui_heartbeat_timer.setInterval(GUI_HEARTBEAT_INTERVAL_MS)
        self._gui_heartbeat_timer.timeout.connect(self._record_gui_heartbeat)
        self._gui_heartbeat_timer.start()

        layout = QtWidgets.QVBoxLayout(self)
        layout.setContentsMargins(10, 12, 10, 10)
        layout.setSpacing(8)
        self.image_label = QtWidgets.QLabel("중지됨")
        self.image_label.setObjectName("cameraPreview")
        self.image_label.setMinimumSize(360, 220)
        self.image_label.setSizePolicy(QtWidgets.QSizePolicy.Ignored, QtWidgets.QSizePolicy.Ignored)
        self.image_label.setAlignment(QtCore.Qt.AlignCenter)
        self._preview_container = _AspectRatioPreviewContainer(self.image_label)
        layout.addWidget(self._preview_container, 1)
        self.detection_label = QtWidgets.QLabel("최고 bbox: 없음")
        self.detection_label.setObjectName("cameraMeta")
        self.detection_label.setTextInteractionFlags(QtCore.Qt.TextSelectableByMouse)
        layout.addWidget(self.detection_label)

    def start_stream(self, url: str, *, enable_detection: bool, **kwargs) -> bool:
        if self.worker is not None or self.thread is not None:
            if not self.stop(wait=False):
                self._set_status("이전 카메라 worker 종료 대기 중")
                return False
        self.stream_generation += 1
        stream_generation = self.stream_generation
        self.latest_image = None
        self.latest_detection = None
        self.measurement_frame_images.clear()
        self.measurement_frame_metadata.clear()
        self.measurement_frame_order.clear()
        self.native_frame_descriptors.clear()
        self.native_frame_descriptor_order.clear()
        self._clear_pending_preview()
        self.measurement_pair_display_active = False
        self.selected_measurement_frame_timestamp_s = None
        self.selected_measurement_frame_lookup_delta_s = None
        self.detection_stream_active = bool(enable_detection)
        self._stop_requested_monotonic_s = None
        self._stop_escalation_level = 0
        self._configure_preview_timer(kwargs.get("display_rate_hz"))
        self._preview_timer.start()
        self.image_label.setPixmap(QtGui.QPixmap())
        self.image_label.setText("YOLO 시작 중..." if enable_detection else "카메라 연결 중...")
        self._set_detection_label(None)
        self.thread = QtCore.QThread(self)
        self.worker = YoloStreamWorker(
            url=url,
            camera_id=self.camera_label,
            stream_generation=stream_generation,
            enable_detection=enable_detection,
            **kwargs,
        )
        self.native_frame_ring = self.worker.native_frame_ring
        self.worker.moveToThread(self.thread)
        self.thread.started.connect(self.worker.run)
        pane_ref = weakref.ref(self)

        def submit_preview(
            image: QtGui.QImage,
            metadata: object,
            *,
            generation: int = stream_generation,
        ) -> None:
            pane = pane_ref()
            if pane is not None:
                pane._submit_preview_frame_for_generation(generation, image, metadata)

        self._preview_frame_callback = submit_preview
        self.worker.frame_ready.connect(
            submit_preview,
            QtCore.Qt.DirectConnection,
        )
        adapter = _CameraStreamSignalAdapter(self, stream_generation)
        self._stream_signal_adapter = adapter
        # Native rasters remain in the child/shared-memory ring. The GUI gets
        # only lightweight descriptors through preview/detection metadata.
        self.worker.detection_ready.connect(
            adapter.detection,
        )
        self.worker.stats_ready.connect(
            adapter.stats,
        )
        # Finalization can arrive after stop() has invalidated the preview
        # generation and deleted its adapter.  Bind it directly to the pane and
        # key by the unique recording path instead of treating it as live UI.
        self.worker.recording_finalized.connect(self._handle_recording_finalized)
        self.worker.status_changed.connect(
            adapter.status,
        )
        self.worker.finished.connect(self.thread.quit)
        self.worker.finished.connect(self.worker.deleteLater)
        self.thread.finished.connect(self.thread.deleteLater)
        self.thread.finished.connect(
            lambda generation=stream_generation, thread=self.thread: self._clear_thread_if_current(generation, thread)
        )
        self.thread.start()
        self._set_status("YOLO 시작 중" if enable_detection else "카메라 연결 중")
        return True

    def start_yolo(self, url: str, **kwargs) -> bool:
        return self.start_stream(url, enable_detection=True, **kwargs)

    def stop(self, *, wait: bool = False) -> bool:
        self.stream_generation += 1
        worker = self.worker
        thread = self.thread
        adapter = self._stream_signal_adapter
        self._stream_signal_adapter = None
        if adapter is not None:
            adapter.deleteLater()
        if worker is not None:
            callback = self._preview_frame_callback
            if callback is not None:
                try:
                    worker.frame_ready.disconnect(callback)
                except (TypeError, RuntimeError):
                    pass
            self._preview_frame_callback = None
            worker.request_stop()
            if self._stop_requested_monotonic_s is None:
                self._stop_requested_monotonic_s = time.monotonic()
        stopped = thread is None
        if thread is not None:
            if wait:
                stopped = bool(thread.wait(CAMERA_STOP_COOPERATIVE_WAIT_MS))
                if not stopped and worker is not None:
                    # The child process is the resource owner.  Stop it before
                    # considering a QThread fallback; terminating the Qt thread
                    # would skip the worker's finally block and lose its cleanup.
                    force_stop = getattr(worker, "stop", None)
                    if callable(force_stop):
                        force_stop()
                    stopped = bool(thread.wait(CAMERA_STOP_POST_ESCALATION_WAIT_MS))
            else:
                is_running = getattr(thread, "isRunning", None)
                stopped = bool(is_running is not None and callable(is_running) and not is_running())
                if not stopped and worker is not None and self._stop_requested_monotonic_s is not None:
                    elapsed_s = max(0.0, time.monotonic() - self._stop_requested_monotonic_s)
                    if elapsed_s >= CAMERA_STOP_KILL_AFTER_S and self._stop_escalation_level < 2:
                        kill_child = getattr(worker, "kill_child_process", None)
                        if callable(kill_child):
                            kill_child()
                        self._stop_escalation_level = 2
                    elif elapsed_s >= CAMERA_STOP_TERMINATE_AFTER_S and self._stop_escalation_level < 1:
                        terminate_child = getattr(worker, "terminate_child_process", None)
                        if callable(terminate_child):
                            terminate_child()
                        self._stop_escalation_level = 1
        if stopped:
            self._retire_native_frame_ring(self.native_frame_ring)
            self.native_frame_ring = None
            self.worker = None
            self.thread = None
            self._stop_requested_monotonic_s = None
            self._stop_escalation_level = 0
        self.latest_image = None
        self.latest_detection = None
        self.measurement_frame_images.clear()
        self.measurement_frame_metadata.clear()
        self.measurement_frame_order.clear()
        self.native_frame_descriptors.clear()
        self.native_frame_descriptor_order.clear()
        self._clear_pending_preview()
        self.measurement_pair_display_active = False
        self.selected_measurement_frame_timestamp_s = None
        self.selected_measurement_frame_lookup_delta_s = None
        self.detection_stream_active = False
        self._preview_timer.stop()
        self.image_label.setText("중지됨")
        self.image_label.setPixmap(QtGui.QPixmap())
        self._preview_container.set_aspect_ratio(None)
        self._set_status("중지됨")
        self._set_detection_label(None)
        self.detection_changed.emit(self.camera_label, None)
        if not stopped:
            self._set_status("종료 대기 중")
        return stopped

    def resizeEvent(self, event) -> None:  # noqa: N802 - Qt override.
        self._refresh_pixmap()
        super().resizeEvent(event)

    def showEvent(self, event) -> None:  # noqa: N802 - Qt override.
        super().showEvent(event)
        # Hidden panes still retain the latest QImage and publish frame
        # evidence, but they deliberately skip QPixmap conversion/scaling.
        # Render that latest frame once when the pane becomes observable.
        self._refresh_pixmap()

    def closeEvent(self, event) -> None:  # noqa: N802 - Qt override.
        if not self.stop(wait=False):
            event.ignore()
            QtCore.QTimer.singleShot(100, self.close)
            return
        super().closeEvent(event)

    def _show_frame(self, image: QtGui.QImage, metadata: object | None = None) -> None:
        frame_metadata = dict(metadata or {}) if isinstance(metadata, dict) else {}
        frame_metadata.setdefault("timestamp_s", time.monotonic())
        frame_metadata.setdefault("source_timestamp_s", None)
        frame_metadata.setdefault("timestamp_source", "app-receive")
        frame_metadata.setdefault("timestamp_reference", infer_camera_timestamp_reference(frame_metadata))
        self._remember_native_descriptor(frame_metadata)
        if self.measurement_pair_display_active and self.detection_stream_active:
            self.frame_changed.emit(self.camera_label, image, frame_metadata)
            return
        self.measurement_pair_display_active = False
        self.latest_image = image
        if image.height() > 0:
            self._preview_container.set_aspect_ratio(image.width() / image.height())
        self.frame_changed.emit(self.camera_label, image, frame_metadata)
        self._refresh_pixmap()

    @staticmethod
    def _frame_key_from_metadata(metadata: object) -> FrameKey | None:
        if not isinstance(metadata, dict):
            return None
        try:
            key = FrameKey.from_mapping(metadata)
        except (TypeError, ValueError):
            return None
        if not key.camera_id or key.stream_generation <= 0 or key.frame_id <= 0:
            return None
        return key

    def _remember_native_descriptor(self, metadata: object) -> None:
        frame_metadata = dict(metadata) if isinstance(metadata, dict) else {}
        key = self._frame_key_from_metadata(frame_metadata)
        if key is None:
            return
        key_text = key.text
        if key_text not in self.native_frame_descriptors:
            if (
                len(self.native_frame_descriptor_order)
                == self.native_frame_descriptor_order.maxlen
            ):
                old_key = self.native_frame_descriptor_order.popleft()
                self.native_frame_descriptors.pop(old_key, None)
            self.native_frame_descriptor_order.append(key_text)
        self.native_frame_descriptors[key_text] = frame_metadata

    def _show_frame_if_current(self, generation: int, image: QtGui.QImage, metadata: object) -> None:
        if generation != self.stream_generation:
            return
        frame_metadata = dict(metadata) if isinstance(metadata, dict) else {}
        frame_metadata["stream_generation"] = int(generation)
        self._show_frame(image, frame_metadata)

    def _configure_preview_timer(self, requested_rate_hz: object) -> None:
        try:
            requested = float(requested_rate_hz)
        except (TypeError, ValueError):
            requested = MAX_GUI_PREVIEW_RATE_HZ
        if not math.isfinite(requested) or requested <= 0.0:
            requested = MAX_GUI_PREVIEW_RATE_HZ
        effective = min(MAX_GUI_PREVIEW_RATE_HZ, requested)
        self._preview_timer.setInterval(max(1, round(1000.0 / effective)))

    def _submit_preview_frame_for_generation(
        self,
        generation: int,
        image: QtGui.QImage,
        metadata: object,
    ) -> None:
        # May run in a camera worker thread. Only replace one detached QImage
        # reference under a lock; QWidget access remains in the timer slot.
        if generation != self.stream_generation:
            return
        frame_metadata = dict(metadata) if isinstance(metadata, dict) else {}
        frame_metadata["stream_generation"] = int(generation)
        with self._preview_lock:
            self._preview_submitted += 1
            if self._pending_preview is not None:
                self._preview_replaced += 1
            self._pending_preview = (generation, image, frame_metadata)

    @QtCore.pyqtSlot()
    def _drain_preview_frame(self) -> None:
        with self._preview_lock:
            pending = self._pending_preview
            self._pending_preview = None
        if pending is None:
            return
        generation, image, metadata = pending
        if generation != self.stream_generation:
            return
        self._preview_rendered += 1
        try:
            frame_timestamp_s = float(metadata.get("timestamp_s"))
            preview_age_s = max(0.0, time.monotonic() - frame_timestamp_s)
        except (TypeError, ValueError):
            preview_age_s = None
        if preview_age_s is not None:
            self._last_preview_age_s = preview_age_s
            self._max_preview_age_s = max(self._max_preview_age_s, preview_age_s)
        self._show_frame(image, metadata)

    def _clear_pending_preview(self) -> None:
        with self._preview_lock:
            self._pending_preview = None

    def preview_delivery_snapshot(self) -> dict[str, int | bool]:
        with self._preview_lock:
            queued = self._pending_preview is not None
            submitted = self._preview_submitted
            replaced = self._preview_replaced
        return {
            "submitted": submitted,
            "rendered": self._preview_rendered,
            "replaced": replaced,
            "queued_latest": queued,
            "timer_interval_ms": self._preview_timer.interval(),
        }

    @staticmethod
    def _current_process_rss_bytes() -> int:
        try:
            resident_pages = int(
                (Path("/proc/self/statm").read_text(encoding="ascii").split())[1]
            )
            return resident_pages * int(os.sysconf("SC_PAGE_SIZE"))
        except (OSError, ValueError, IndexError):
            return 0

    @QtCore.pyqtSlot()
    def _record_gui_heartbeat(self) -> None:
        now_s = time.monotonic()
        previous_s = self._gui_heartbeat_last_s
        if previous_s is not None:
            expected_interval_s = GUI_HEARTBEAT_INTERVAL_MS / 1000.0
            delay_s = max(0.0, now_s - previous_s - expected_interval_s)
            self._gui_heartbeat_max_delay_s = max(
                self._gui_heartbeat_max_delay_s,
                delay_s,
            )
        self._gui_heartbeat_last_s = now_s
        self._gui_heartbeat_count += 1

    def backpressure_snapshot(self) -> dict[str, object]:
        ring_metrics = (
            self.native_frame_ring.metrics()
            if self.native_frame_ring is not None
            else {}
        )
        return {
            **self.preview_delivery_snapshot(),
            **self._last_stream_stats,
            **{
                f"native_ring_{key}": value
                for key, value in ring_metrics.items()
                if key not in {"camera_id", "stream_generation"}
            },
            "gui_heartbeat_count": self._gui_heartbeat_count,
            "gui_heartbeat_max_delay_s": self._gui_heartbeat_max_delay_s,
            "gui_preview_age_s": self._last_preview_age_s,
            "gui_preview_max_age_s": self._max_preview_age_s,
            "workstation_rss_bytes": self._current_process_rss_bytes(),
        }

    def _store_measurement_frame(
        self,
        timestamp_s: float,
        image: QtGui.QImage,
        metadata: dict[str, object] | None = None,
    ) -> None:
        key = float(timestamp_s)
        if key not in self.measurement_frame_images:
            if len(self.measurement_frame_order) == self.measurement_frame_order.maxlen:
                old_key = self.measurement_frame_order.popleft()
                self.measurement_frame_images.pop(old_key, None)
                self.measurement_frame_metadata.pop(old_key, None)
            self.measurement_frame_order.append(key)
        self.measurement_frame_images[key] = image.copy()
        if metadata is not None:
            self.measurement_frame_metadata[key] = copy.deepcopy(metadata)

    def _store_measurement_frame_if_current(self, generation: int, timestamp_s: float, image: QtGui.QImage) -> None:
        if generation != self.stream_generation:
            return
        self._store_measurement_frame(timestamp_s, image)

    def _emit_native_measurement_frame_if_current(
        self,
        generation: int,
        metadata: object,
        image: QtGui.QImage,
    ) -> None:
        if generation != self.stream_generation:
            return
        frame_metadata = dict(metadata) if isinstance(metadata, dict) else {}
        frame_metadata["stream_generation"] = int(generation)
        timestamp_s = float(frame_metadata.get("timestamp_s", time.monotonic()))
        # The compatibility signal above may arrive immediately before this
        # metadata-rich signal.  Store the native raster and metadata together
        # so a frozen burst works even when detection is disabled.
        self._store_measurement_frame(timestamp_s, image, frame_metadata)
        self.native_measurement_frame_changed.emit(self.camera_label, frame_metadata, image)

    def _native_ring_for_key(self, key: FrameKey) -> SharedNativeFrameRing | None:
        candidates = [self.native_frame_ring, *self._retired_native_frame_rings]
        for ring in candidates:
            if (
                ring is not None
                and ring.camera_id == key.camera_id
                and ring.stream_generation == key.stream_generation
            ):
                return ring
        return None

    def native_ring_for_frame_key(
        self,
        key: FrameKey,
    ) -> SharedNativeFrameRing | None:
        return self._native_ring_for_key(key)

    @staticmethod
    def _qimage_from_native_bgr(
        data: bytes,
        *,
        width: int,
        height: int,
    ) -> QtGui.QImage:
        bytes_per_line = int(width) * 3
        bgr_format = getattr(QtGui.QImage, "Format_BGR888", None)
        if bgr_format is not None:
            return QtGui.QImage(
                data,
                int(width),
                int(height),
                bytes_per_line,
                bgr_format,
            ).copy()
        return QtGui.QImage(
            data,
            int(width),
            int(height),
            bytes_per_line,
            QtGui.QImage.Format_RGB888,
        ).rgbSwapped()

    def native_frame_for_key(
        self,
        key: FrameKey,
    ) -> tuple[QtGui.QImage, dict[str, object]] | None:
        """Resolve exactly one native frame; never substitute a nearby frame."""

        ring = self._native_ring_for_key(key)
        if ring is None:
            return None
        lease = ring.pin(key)
        if lease is None:
            return None
        try:
            snapshot = ring.read(lease)
        except KeyError:
            return None
        finally:
            ring.release(lease)
        descriptor = snapshot.descriptor
        if descriptor.channels != 3:
            return None
        image = self._qimage_from_native_bgr(
            snapshot.data,
            width=descriptor.width,
            height=descriptor.height,
        )
        metadata = dict(self.native_frame_descriptors.get(key.text, {}))
        metadata.update(descriptor.to_mapping())
        metadata.update(
            {
                "source_native_frame_id": key.text,
                "native_lossless_source": True,
                "overlay_applied": False,
            }
        )
        return image, metadata

    def native_frame_for_detection(
        self,
        detection: DetectionObservation,
    ) -> tuple[QtGui.QImage, dict[str, object]] | None:
        key = self._frame_key_from_metadata(detection.timestamp_metadata)
        if key is None:
            return None
        return self.native_frame_for_key(key)

    def native_measurement_frame(
        self,
        timestamp_s: float,
        *,
        maximum_delta_s: float = 0.001,
    ) -> tuple[float, QtGui.QImage, dict[str, object]] | None:
        """Return a native, unannotated frame near a detection timestamp."""

        requested = float(timestamp_s)
        descriptor_candidates: list[tuple[float, FrameKey]] = []
        for metadata in self.native_frame_descriptors.values():
            key = self._frame_key_from_metadata(metadata)
            if key is None:
                continue
            try:
                candidate_s = float(metadata.get("timestamp_s"))
            except (TypeError, ValueError):
                continue
            descriptor_candidates.append((candidate_s, key))
        if descriptor_candidates:
            candidate_s, key = min(
                descriptor_candidates,
                key=lambda item: abs(item[0] - requested),
            )
            if abs(candidate_s - requested) <= float(maximum_delta_s):
                resolved = self.native_frame_for_key(key)
                if resolved is not None:
                    image, metadata = resolved
                    return candidate_s, image, metadata

        if not self.measurement_frame_images:
            return None
        key = min(self.measurement_frame_images, key=lambda candidate: abs(candidate - requested))
        if abs(key - requested) > float(maximum_delta_s):
            return None
        image = self.measurement_frame_images.get(key)
        if image is None:
            return None
        metadata = dict(self.measurement_frame_metadata.get(key, {}))
        metadata.setdefault("timestamp_s", key)
        metadata.setdefault("native_width", image.width())
        metadata.setdefault("native_height", image.height())
        metadata.setdefault("overlay_applied", False)
        return key, image.copy(), metadata

    def latest_native_measurement_frame(
        self,
    ) -> tuple[float, QtGui.QImage, dict[str, object]] | None:
        """Return the newest native, unannotated producer frame.

        This is intentionally distinct from ``latest_image``, which is a
        display preview and may be resized.  Optical-pose burst capture must
        call this method so annotations remain on the decoded source grid.
        """

        ring = self.native_frame_ring
        if ring is not None:
            descriptor = ring.latest_descriptor()
            if descriptor is not None:
                resolved = self.native_frame_for_key(descriptor.key)
                if resolved is not None:
                    image, metadata = resolved
                    return (
                        float(descriptor.timestamp_s or 0.0),
                        image,
                        metadata,
                    )
        if not self.measurement_frame_order:
            return None
        return self.native_measurement_frame(
            self.measurement_frame_order[-1],
            maximum_delta_s=0.0,
        )

    def calibration_native_frame_near(
        self,
        timestamp_s: float,
        *,
        maximum_delta_s: float = 0.001,
        maximum_age_s: float = DEFAULT_NATIVE_CALIBRATION_FRAME_MAX_AGE_S,
        now_monotonic_s: float | None = None,
    ) -> tuple[str, QtGui.QImage, dict[str, object]] | None:
        """Return the closest fresh native raster suitable for calibration.

        ``timestamp_s`` is the producer's app-receive monotonic timestamp.  A
        calibration caller can therefore resolve the native raster belonging
        to a preview/checkerboard observation without ever saving the resized
        preview.  The returned image and metadata are defensive copies.

        Frames fail closed when their producer provenance cannot prove that
        they are lossless, native-sized and unannotated.  This keeps a preview
        resize or UI overlay from silently changing the calibration pixel
        coordinate system.
        """

        requested = self._finite_nonnegative_limit(timestamp_s, "timestamp_s", allow_zero=True)
        delta_limit = self._finite_nonnegative_limit(
            maximum_delta_s,
            "maximum_delta_s",
            allow_zero=True,
        )
        descriptor_candidates: list[tuple[float, dict[str, object]]] = []
        for metadata in self.native_frame_descriptors.values():
            try:
                candidate_s = float(metadata.get("timestamp_s"))
            except (TypeError, ValueError):
                continue
            descriptor_candidates.append((candidate_s, metadata))
        if descriptor_candidates:
            candidate_s, metadata = min(
                descriptor_candidates,
                key=lambda item: abs(item[0] - requested),
            )
            if abs(candidate_s - requested) <= delta_limit:
                return self._native_calibration_frame_for_descriptor(
                    metadata,
                    maximum_age_s=maximum_age_s,
                    now_monotonic_s=now_monotonic_s,
                )
        if not self.measurement_frame_images:
            return None
        key = min(self.measurement_frame_images, key=lambda candidate: abs(candidate - requested))
        if abs(key - requested) > delta_limit:
            return None
        return self._native_calibration_frame_for_key(
            key,
            maximum_age_s=maximum_age_s,
            now_monotonic_s=now_monotonic_s,
        )

    def latest_native_calibration_frame(
        self,
        *,
        maximum_age_s: float = DEFAULT_NATIVE_CALIBRATION_FRAME_MAX_AGE_S,
        now_monotonic_s: float | None = None,
    ) -> tuple[str, QtGui.QImage, dict[str, object]] | None:
        """Return ``(frame_id, native_image, metadata)`` for calibration.

        Unlike :meth:`current_image`, this method never reads the resized UI
        preview.  Missing, stale, annotated, resized, or provenance-incomplete
        native frames are rejected with ``None``.
        """

        ring = self.native_frame_ring
        if ring is not None:
            descriptor = ring.latest_descriptor()
            if descriptor is not None:
                metadata = dict(
                    self.native_frame_descriptors.get(descriptor.key.text, {})
                )
                metadata.update(descriptor.to_mapping())
                return self._native_calibration_frame_for_descriptor(
                    metadata,
                    maximum_age_s=maximum_age_s,
                    now_monotonic_s=now_monotonic_s,
                )
        if not self.measurement_frame_order:
            return None
        return self._native_calibration_frame_for_key(
            self.measurement_frame_order[-1],
            maximum_age_s=maximum_age_s,
            now_monotonic_s=now_monotonic_s,
        )

    def _native_calibration_frame_for_descriptor(
        self,
        metadata: dict[str, object],
        *,
        maximum_age_s: float,
        now_monotonic_s: float | None,
    ) -> tuple[str, QtGui.QImage, dict[str, object]] | None:
        key = self._frame_key_from_metadata(metadata)
        if key is None:
            return None
        resolved = self.native_frame_for_key(key)
        if resolved is None:
            return None
        image, resolved_metadata = resolved
        age_limit = self._finite_nonnegative_limit(
            maximum_age_s,
            "maximum_age_s",
            allow_zero=True,
        )
        now_s = time.monotonic() if now_monotonic_s is None else float(now_monotonic_s)
        try:
            timestamp_s = float(resolved_metadata["timestamp_s"])
        except (KeyError, TypeError, ValueError):
            return None
        if not math.isfinite(now_s) or not math.isfinite(timestamp_s):
            return None
        age_s = now_s - timestamp_s
        if age_s < 0.0 or age_s > age_limit:
            return None
        if resolved_metadata.get("native_lossless_source") is not True:
            return None
        if resolved_metadata.get("overlay_applied") is not False:
            return None
        resolved_metadata["frame_id"] = key.text
        resolved_metadata.setdefault(
            "pixel_contract",
            "decoded native RGB raster; no resize, crop, or overlay",
        )
        return key.text, image, resolved_metadata

    @staticmethod
    def _finite_nonnegative_limit(
        value: float,
        name: str,
        *,
        allow_zero: bool,
    ) -> float:
        try:
            normalized = float(value)
        except (TypeError, ValueError) as exc:
            raise ValueError(f"{name} must be a finite non-negative number") from exc
        if not math.isfinite(normalized) or normalized < 0.0 or (not allow_zero and normalized == 0.0):
            raise ValueError(f"{name} must be a finite non-negative number")
        return normalized

    def _native_calibration_frame_for_key(
        self,
        key: float,
        *,
        maximum_age_s: float,
        now_monotonic_s: float | None,
    ) -> tuple[str, QtGui.QImage, dict[str, object]] | None:
        age_limit = self._finite_nonnegative_limit(
            maximum_age_s,
            "maximum_age_s",
            allow_zero=True,
        )
        now_s = time.monotonic() if now_monotonic_s is None else float(now_monotonic_s)
        if not math.isfinite(now_s):
            raise ValueError("now_monotonic_s must be finite")
        age_s = now_s - float(key)
        if age_s < 0.0 or age_s > age_limit:
            return None

        image = self.measurement_frame_images.get(key)
        if image is None or image.isNull():
            return None
        metadata = copy.deepcopy(self.measurement_frame_metadata.get(key, {}))
        try:
            metadata_timestamp_s = float(metadata["timestamp_s"])
            native_width = int(metadata["native_width"])
            native_height = int(metadata["native_height"])
        except (KeyError, TypeError, ValueError, OverflowError):
            return None
        if not math.isfinite(metadata_timestamp_s) or abs(metadata_timestamp_s - float(key)) > 1e-9:
            return None
        if (native_width, native_height) != (image.width(), image.height()):
            return None
        if metadata.get("native_lossless_source") is not True:
            return None
        if metadata.get("overlay_applied") is not False:
            return None
        frame_id = str(
            metadata.get("source_native_frame_id")
            or metadata.get("frame_id")
            or ""
        ).strip()
        if not frame_id:
            return None

        metadata["frame_id"] = frame_id
        metadata.setdefault(
            "pixel_contract",
            "decoded native RGB raster; no resize, crop, or overlay",
        )
        return frame_id, image.copy(), metadata

    def show_measurement_frame_for_detection(self, detection: DetectionObservation) -> bool:
        exact = self.native_frame_for_detection(detection)
        if exact is not None:
            image, metadata = exact
            self.selected_measurement_frame_timestamp_s = float(
                metadata.get("timestamp_s", detection.timestamp_s)
            )
            self.selected_measurement_frame_lookup_delta_s = 0.0
            self.show_selected_measurement_frame(image, metadata)
            return True
        if self._frame_key_from_metadata(detection.timestamp_metadata) is not None:
            self.measurement_pair_display_active = False
            self.selected_measurement_frame_timestamp_s = None
            self.selected_measurement_frame_lookup_delta_s = None
            return False
        detection_timestamp_s = float(detection.timestamp_s)
        image = self.measurement_frame_images.get(detection_timestamp_s)
        selected_timestamp_s = detection_timestamp_s if image is not None else None
        if image is None:
            nearest = self._nearest_measurement_frame(detection_timestamp_s)
            if nearest is not None:
                selected_timestamp_s, image = nearest
        if image is None:
            self.measurement_pair_display_active = False
            self.selected_measurement_frame_timestamp_s = None
            self.selected_measurement_frame_lookup_delta_s = None
            return False
        self.selected_measurement_frame_timestamp_s = selected_timestamp_s
        self.selected_measurement_frame_lookup_delta_s = (
            None if selected_timestamp_s is None else selected_timestamp_s - detection_timestamp_s
        )
        metadata = (
            dict(self.measurement_frame_metadata.get(float(selected_timestamp_s), {}))
            if selected_timestamp_s is not None
            else {}
        )
        metadata.setdefault("timestamp_s", selected_timestamp_s)
        self.show_selected_measurement_frame(image, metadata)
        return True

    def show_selected_measurement_frame(
        self,
        image: QtGui.QImage,
        metadata: object | None = None,
    ) -> None:
        self.measurement_pair_display_active = True
        self.latest_image = image.copy()
        if image.height() > 0:
            self._preview_container.set_aspect_ratio(image.width() / image.height())
        self._refresh_pixmap()
        frame_metadata = dict(metadata) if isinstance(metadata, dict) else {}
        frame_metadata.setdefault("timestamp_s", self.selected_measurement_frame_timestamp_s)
        frame_metadata.setdefault("display_mode", "detection")
        # MeasurementWorkspace has a separate preview widget from the legacy
        # CameraPane label.  Publish the frozen pair explicitly so the live
        # frame signal cannot replace it while detection mode is active.
        self.measurement_frame_changed.emit(self.camera_label, self.latest_image, frame_metadata)

    def clear_measurement_pair_display(self) -> None:
        self.measurement_pair_display_active = False
        self.selected_measurement_frame_timestamp_s = None
        self.selected_measurement_frame_lookup_delta_s = None

    def _nearest_measurement_frame(self, timestamp_s: float) -> tuple[float, QtGui.QImage] | None:
        if not self.measurement_frame_images:
            return None
        key = min(self.measurement_frame_images, key=lambda candidate: abs(candidate - timestamp_s))
        if abs(key - timestamp_s) > 0.001:
            return None
        image = self.measurement_frame_images.get(key)
        if image is None:
            return None
        return key, image

    def _refresh_pixmap(self) -> None:
        if self.latest_image is None or not self.isVisible():
            return
        target_size = self.image_label.size()
        source_pixmap = QtGui.QPixmap.fromImage(self.latest_image)
        display_pixmap = source_pixmap.scaled(
            target_size,
            QtCore.Qt.KeepAspectRatio,
            QtCore.Qt.SmoothTransformation,
        )
        if self.latest_detection is not None and not self.measurement_pair_display_active:
            display_pixmap = self._draw_detection_overlay(
                display_pixmap,
                self.latest_detection,
                scale_x=display_pixmap.width() / source_pixmap.width(),
                scale_y=display_pixmap.height() / source_pixmap.height(),
            )
        self.image_label.setPixmap(display_pixmap)

    def _draw_detection_overlay(
        self,
        pixmap: QtGui.QPixmap,
        detection: DetectionObservation,
        *,
        scale_x: float | None = None,
        scale_y: float | None = None,
        crop_x: int = 0,
        crop_y: int = 0,
    ) -> QtGui.QPixmap:
        if detection.width <= 0 or detection.height <= 0:
            return pixmap
        if scale_x is None:
            scale_x = pixmap.width() / detection.width
        if scale_y is None:
            scale_y = pixmap.height() / detection.height
        painter = QtGui.QPainter(pixmap)
        try:
            painter.setRenderHint(QtGui.QPainter.Antialiasing)
            color = QtGui.QColor(ACCENT)
            rect = QtCore.QRectF(
                detection.x1 * scale_x - crop_x,
                detection.y1 * scale_y - crop_y,
                (detection.x2 - detection.x1) * scale_x,
                (detection.y2 - detection.y1) * scale_y,
            )
            painter.setPen(QtGui.QPen(color, 2))
            painter.drawRect(rect)
            center_x, center_y = detection.center
            center = QtCore.QPointF(
                center_x * scale_x - crop_x,
                center_y * scale_y - crop_y,
            )
            marker_radius = max(3.0, min(6.0, min(pixmap.width(), pixmap.height()) * 0.012))
            painter.setBrush(QtCore.Qt.NoBrush)
            painter.setPen(QtGui.QPen(color, 2))
            painter.drawEllipse(center, marker_radius, marker_radius)
            label = detection.class_name or f"#{detection.class_id}"
            painter.drawText(
                QtCore.QPointF(rect.left(), max(10.0, rect.top() - 4.0)),
                f"{label} {detection.confidence:.2f}",
            )
        finally:
            painter.end()
        return pixmap

    def current_image(self) -> QtGui.QImage | None:
        return self.latest_image.copy() if self.latest_image is not None else None

    def _handle_detection_payload(self, payload: dict) -> None:
        observation = DetectionObservation.from_payload(self.camera_label, payload)
        self.latest_detection = observation
        self._set_detection_label(observation)
        if not self.measurement_pair_display_active:
            self._refresh_pixmap()
        self.detection_changed.emit(self.camera_label, observation)

    def _handle_detection_payload_if_current(self, generation: int, payload: dict) -> None:
        if generation != self.stream_generation:
            return
        current_payload = dict(payload)
        timestamp_metadata = current_payload.get("timestamp_metadata")
        timestamp_metadata = (
            dict(timestamp_metadata) if isinstance(timestamp_metadata, dict) else {}
        )
        timestamp_metadata["stream_generation"] = int(generation)
        self._remember_native_descriptor(timestamp_metadata)
        current_payload["timestamp_metadata"] = timestamp_metadata
        self._handle_detection_payload(current_payload)

    def _handle_stats_payload(self, payload: dict) -> None:
        merged = dict(payload)
        self._last_stream_stats = merged
        merged.update(self.backpressure_snapshot())
        self.stats_changed.emit(self.camera_label, merged)

    def _handle_stats_payload_if_current(self, generation: int, payload: dict) -> None:
        if generation != self.stream_generation:
            return
        self._handle_stats_payload(payload)

    @QtCore.pyqtSlot(dict)
    def _handle_recording_finalized(self, payload: dict) -> None:
        finalization = dict(payload or {})
        path = str(finalization.get("path") or "").strip()
        if not path:
            return
        self.recording_finalizations[path] = finalization
        while len(self.recording_finalizations) > 16:
            self.recording_finalizations.pop(next(iter(self.recording_finalizations)))

    def _set_status(self, status: str) -> None:
        self.current_status = status
        self.state_changed.emit(self.camera_label, status)

    def _set_status_if_current(self, generation: int, status: str) -> None:
        if generation != self.stream_generation:
            return
        self._set_status(status)

    def _set_detection_label(self, detection: DetectionObservation | None) -> None:
        if detection is None:
            self.detection_label.setText("최고 bbox: 없음")
            return
        cx, cy = detection.center
        class_label = detection.class_name or f"#{detection.class_id}"
        text = (
            f"최고 bbox: class={class_label} conf={detection.confidence:.2f} "
            f"center=({cx:.2f}, {cy:.2f}) px"
        )
        self.detection_label.setText(text)

    def _clear_thread_if_current(self, _generation: int, thread: QtCore.QThread) -> None:
        # A stop increments stream_generation so stale frames are ignored, but
        # the finished callback still has to release the stopped thread.  The
        # object identity prevents an old callback from clearing a replacement
        # stream that has already started.
        if self.thread is not thread:
            return
        self._retire_native_frame_ring(self.native_frame_ring)
        self.native_frame_ring = None
        self.thread = None
        self.worker = None
        self._preview_frame_callback = None
        self._stream_signal_adapter = None
        self._stop_requested_monotonic_s = None
        self._stop_escalation_level = 0

    def _retire_native_frame_ring(
        self,
        ring: SharedNativeFrameRing | None,
    ) -> None:
        if ring is not None and ring not in self._retired_native_frame_rings:
            self._retired_native_frame_rings.append(ring)
        CameraPane.sweep_retired_native_frame_rings(self)

    def sweep_retired_native_frame_rings(self) -> bool:
        """Unlink every retired ring whose evidence leases have drained."""

        survivors: list[SharedNativeFrameRing] = []
        for candidate in self._retired_native_frame_rings:
            try:
                active_leases = int(candidate.metrics().get("active_leases", 0))
            except RuntimeError:
                active_leases = 0
            if active_leases > 0:
                survivors.append(candidate)
                continue
            candidate.close()
            candidate.unlink()
        self._retired_native_frame_rings = survivors
        return not survivors
