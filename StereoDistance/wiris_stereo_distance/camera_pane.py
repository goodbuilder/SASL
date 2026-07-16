from __future__ import annotations

import threading
import time
from collections import deque
from dataclasses import dataclass
from types import MappingProxyType
from typing import Any, Mapping

from PyQt5 import QtCore, QtGui, QtWidgets

from wiris_stereo_distance.stereo import DetectionObservation
from wiris_stereo_distance.theme import ACCENT, SERIES_HIGHLIGHT
from wiris_stereo_distance.yolo_stream import YoloStreamWorker


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


@dataclass(frozen=True)
class MeasurementFrameSelection:
    timestamp_s: float
    frame_id: object | None
    image: QtGui.QImage
    lookup_delta_s: float
    metadata: Mapping[str, Any]


@dataclass(frozen=True)
class BilateralMeasurementFrameSelection:
    left: MeasurementFrameSelection
    right: MeasurementFrameSelection


class CameraPane(QtWidgets.QGroupBox):
    detection_changed = QtCore.pyqtSignal(str, object)
    frame_changed = QtCore.pyqtSignal(str, QtGui.QImage, object)
    stats_changed = QtCore.pyqtSignal(str, object)
    state_changed = QtCore.pyqtSignal(str, str)
    recording_ready = QtCore.pyqtSignal(object)
    recording_event_ready = QtCore.pyqtSignal(object)
    recording_failure = QtCore.pyqtSignal(object)
    recording_terminal = QtCore.pyqtSignal(object)
    stream_finished = QtCore.pyqtSignal(str)

    def __init__(self, label: str) -> None:
        super().__init__(label)
        self.setObjectName("cameraPane")
        self.camera_label = label
        self.worker: YoloStreamWorker | None = None
        self.thread: QtCore.QThread | None = None
        self.latest_image: QtGui.QImage | None = None
        self.latest_detection: DetectionObservation | None = None
        self.measurement_frame_images: dict[float, QtGui.QImage] = {}
        self.measurement_frame_order: deque[float] = deque(maxlen=120)
        self.measurement_frame_ids: dict[float, object] = {}
        self.measurement_frame_metadata: dict[float, dict[str, Any]] = {}
        self.measurement_frame_timestamps_by_id: dict[str, float] = {}
        self.measurement_pair_display_active = False
        self.selected_measurement_frame_timestamp_s: float | None = None
        self.selected_measurement_frame_lookup_delta_s: float | None = None
        self.selected_measurement_frame_id: object | None = None
        self.selected_measurement_frame_metadata: dict[str, Any] = {}
        self.detection_stream_active = False
        self.bbox_center_adjustment: tuple[tuple[float, float, float, float], tuple[float, float, float, float]] | None = None
        self.stream_generation = 0
        self.stop_requested = False
        self._force_stop_thread: threading.Thread | None = None
        self._force_stop_worker: object | None = None
        self.current_status = "중지됨"

        layout = QtWidgets.QVBoxLayout(self)
        layout.setContentsMargins(10, 12, 10, 10)
        layout.setSpacing(8)
        self.image_label = QtWidgets.QLabel("중지됨")
        self.image_label.setObjectName("cameraPreview")
        self.image_label.setMinimumSize(360, 220)
        self.image_label.setSizePolicy(QtWidgets.QSizePolicy.Ignored, QtWidgets.QSizePolicy.Ignored)
        self.image_label.setAlignment(QtCore.Qt.AlignCenter)
        layout.addWidget(self.image_label, 1)
        self.detection_label = QtWidgets.QLabel("최고 bbox: 없음")
        self.detection_label.setObjectName("cameraMeta")
        self.detection_label.setTextInteractionFlags(QtCore.Qt.TextSelectableByMouse)
        layout.addWidget(self.detection_label)

    def start_stream(self, url: str, *, enable_detection: bool, **kwargs) -> None:
        if self.worker is not None or self.thread is not None:
            raise RuntimeError(f"{self.camera_label} camera worker is still active")
        self.stream_generation += 1
        stream_generation = self.stream_generation
        self.stop_requested = False
        self.latest_image = None
        self.latest_detection = None
        self.measurement_frame_images.clear()
        self.measurement_frame_order.clear()
        self.measurement_frame_ids.clear()
        self.measurement_frame_metadata.clear()
        self.measurement_frame_timestamps_by_id.clear()
        self.measurement_pair_display_active = False
        self._clear_selected_measurement_frame_state()
        self.detection_stream_active = bool(enable_detection)
        self.image_label.setPixmap(QtGui.QPixmap())
        self.image_label.setText("동기화 프레임 대기 중" if enable_detection else "카메라 연결 중...")
        self._set_detection_label(None)
        self.thread = QtCore.QThread(self)
        self.worker = YoloStreamWorker(url=url, enable_detection=enable_detection, **kwargs)
        self.worker.moveToThread(self.thread)
        self.thread.started.connect(self.worker.run)
        self.worker.frame_ready.connect(
            lambda image, metadata, generation=stream_generation: self._show_frame_if_current(
                generation, image, metadata
            )
        )
        self.worker.measurement_frame_ready.connect(
            lambda timestamp_s, frame_id, image, metadata, generation=stream_generation: self._store_measurement_frame_if_current(
                generation, timestamp_s, frame_id, image, metadata
            )
        )
        self.worker.detection_ready.connect(
            lambda payload, generation=stream_generation: self._handle_detection_payload_if_current(
                generation, payload
            )
        )
        self.worker.stats_ready.connect(
            lambda payload, generation=stream_generation: self._handle_stats_payload_if_current(generation, payload)
        )
        self.worker.status_changed.connect(
            lambda status, generation=stream_generation: self._set_status_if_current(generation, status)
        )
        for worker_signal_name, pane_signal in (
            ("recording_ready", self.recording_ready),
            ("recording_event_ready", self.recording_event_ready),
            ("recording_failure", self.recording_failure),
            ("recording_terminal", self.recording_terminal),
        ):
            worker_signal = getattr(self.worker, worker_signal_name, None)
            if worker_signal is not None:
                worker_signal.connect(
                    lambda payload, signal=pane_signal, generation=stream_generation: self._emit_recording_signal_if_current(
                        generation, signal, payload
                    )
                )
        self.worker.finished.connect(self.thread.quit)
        self.worker.finished.connect(self.worker.deleteLater)
        self.thread.finished.connect(self.thread.deleteLater)
        self.thread.finished.connect(
            lambda generation=stream_generation, thread=self.thread: self._clear_thread_if_current(generation, thread)
        )
        self.thread.start()
        self._set_status("YOLO 시작 중" if enable_detection else "카메라 연결 중")

    def start_yolo(self, url: str, **kwargs) -> None:
        self.start_stream(url, enable_detection=True, **kwargs)

    def stop(self, *, wait: bool = False) -> None:
        worker = self.worker
        self.stop_requested = True
        if worker is not None:
            worker.request_stop()
        # ``wait`` is retained for source compatibility only.  Camera shutdown
        # is always asynchronous; references are released by the thread's
        # actual ``finished`` signal.
        self.latest_image = None
        self.latest_detection = None
        self.measurement_frame_images.clear()
        self.measurement_frame_order.clear()
        self.measurement_frame_ids.clear()
        self.measurement_frame_metadata.clear()
        self.measurement_frame_timestamps_by_id.clear()
        self.measurement_pair_display_active = False
        self._clear_selected_measurement_frame_state()
        self.detection_stream_active = False
        self.image_label.setText("중지됨")
        self.image_label.setPixmap(QtGui.QPixmap())
        self._set_status("중지됨")
        self._set_detection_label(None)
        self.detection_changed.emit(self.camera_label, None)

    def force_stop_async(self) -> None:
        """Clean up a hung worker process without blocking the GUI thread."""

        worker = self.worker
        qt_thread = self.thread
        if worker is not None:
            try:
                worker.request_stop()
            except Exception:
                pass
        if qt_thread is not None:
            for operation_name in ("requestInterruption", "quit"):
                operation = getattr(qt_thread, operation_name, None)
                if operation is None:
                    continue
                try:
                    operation()
                except Exception:
                    pass
        if worker is None:
            return
        helper = self._force_stop_thread
        if helper is not None and helper.is_alive() and self._force_stop_worker is worker:
            return

        def stop_worker_process() -> None:
            try:
                worker.stop()
            except Exception:
                pass
            finally:
                if self._force_stop_thread is helper:
                    self._force_stop_thread = None
                    self._force_stop_worker = None

        helper = threading.Thread(
            target=stop_worker_process,
            name=f"camera-force-stop-{self.camera_label}",
            daemon=True,
        )
        self._force_stop_thread = helper
        self._force_stop_worker = worker
        helper.start()

    def resizeEvent(self, event) -> None:  # noqa: N802 - Qt override.
        self._refresh_pixmap()
        super().resizeEvent(event)

    def closeEvent(self, event) -> None:  # noqa: N802 - Qt override.
        self.stop()
        super().closeEvent(event)

    def _show_frame(self, image: QtGui.QImage, metadata: object | None = None) -> None:
        frame_metadata = dict(metadata or {}) if isinstance(metadata, dict) else {}
        frame_metadata.setdefault("timestamp_s", time.monotonic())
        frame_metadata.setdefault("source_timestamp_s", None)
        frame_metadata.setdefault("timestamp_source", "app-receive")
        frame_metadata.setdefault("timestamp_reference", infer_camera_timestamp_reference(frame_metadata))
        if self.detection_stream_active:
            # Generic frames update receipt/status; only selected pairs repaint measurement panes.
            self.frame_changed.emit(self.camera_label, image, frame_metadata)
            return
        self.measurement_pair_display_active = False
        self.latest_image = image
        self.frame_changed.emit(self.camera_label, image, frame_metadata)
        self._refresh_pixmap()

    def _show_frame_if_current(self, generation: int, image: QtGui.QImage, metadata: object) -> None:
        if generation != self.stream_generation or self.stop_requested:
            return
        self._show_frame(image, metadata)

    @staticmethod
    def _measurement_frame_id_key(frame_id: object | None) -> str | None:
        return None if frame_id is None else str(frame_id)

    def _store_measurement_frame(
        self,
        timestamp_s: float,
        image: QtGui.QImage,
        frame_id: object | None = None,
        metadata: object | None = None,
    ) -> None:
        key = float(timestamp_s)
        if key not in self.measurement_frame_images:
            if len(self.measurement_frame_order) == self.measurement_frame_order.maxlen:
                old_key = self.measurement_frame_order.popleft()
                self.measurement_frame_images.pop(old_key, None)
                old_frame_id = self.measurement_frame_ids.pop(old_key, None)
                old_frame_id_key = self._measurement_frame_id_key(old_frame_id)
                if old_frame_id_key is not None:
                    self.measurement_frame_timestamps_by_id.pop(old_frame_id_key, None)
                self.measurement_frame_metadata.pop(old_key, None)
            self.measurement_frame_order.append(key)
        self.measurement_frame_images[key] = image.copy()
        frame_metadata = dict(metadata or {}) if isinstance(metadata, dict) else {}
        frame_metadata.setdefault("timestamp_s", key)
        if frame_id is not None:
            frame_metadata.setdefault("frame_id", frame_id)
        self.measurement_frame_metadata[key] = frame_metadata
        old_frame_id = self.measurement_frame_ids.pop(key, None)
        old_frame_id_key = self._measurement_frame_id_key(old_frame_id)
        if old_frame_id_key is not None:
            self.measurement_frame_timestamps_by_id.pop(old_frame_id_key, None)
        if frame_id is not None:
            self.measurement_frame_ids[key] = frame_id
            frame_id_key = self._measurement_frame_id_key(frame_id)
            if frame_id_key is not None:
                self.measurement_frame_timestamps_by_id[frame_id_key] = key

    def _store_measurement_frame_if_current(
        self,
        generation: int,
        timestamp_s: float,
        frame_id: object | None,
        image: QtGui.QImage,
        metadata: object | None = None,
    ) -> None:
        if generation != self.stream_generation or self.stop_requested:
            return
        self._store_measurement_frame(timestamp_s, image, frame_id, metadata)

    def resolve_measurement_frame_for_detection(
        self,
        detection: DetectionObservation,
    ) -> MeasurementFrameSelection | None:
        detection_timestamp_s = float(detection.timestamp_s)
        detection_metadata = dict(detection.timestamp_metadata or {})
        frame_id = detection_metadata.get("frame_id")
        if frame_id is not None:
            selected_timestamp_s = self.measurement_frame_timestamps_by_id.get(
                self._measurement_frame_id_key(frame_id) or ""
            )
        elif self.detection_stream_active:
            return None
        else:
            selected_timestamp_s = detection_timestamp_s if detection_timestamp_s in self.measurement_frame_images else None
        if selected_timestamp_s is None:
            return None
        image = self.measurement_frame_images.get(selected_timestamp_s)
        if image is None:
            return None
        return MeasurementFrameSelection(
            timestamp_s=selected_timestamp_s,
            frame_id=self.measurement_frame_ids.get(selected_timestamp_s),
            image=image,
            lookup_delta_s=selected_timestamp_s - detection_timestamp_s,
            metadata=MappingProxyType(dict(self.measurement_frame_metadata.get(selected_timestamp_s) or {})),
        )

    def show_measurement_frame_for_detection(self, detection: DetectionObservation) -> bool:
        selection = self.resolve_measurement_frame_for_detection(detection)
        if selection is None:
            self.measurement_pair_display_active = False
            self._clear_selected_measurement_frame_state()
            return False
        self.show_resolved_measurement_frame(selection)
        return True

    def show_resolved_measurement_frame(self, selection: MeasurementFrameSelection) -> None:
        self.show_selected_measurement_frame(selection.image)
        self.selected_measurement_frame_timestamp_s = selection.timestamp_s
        self.selected_measurement_frame_lookup_delta_s = selection.lookup_delta_s
        self.selected_measurement_frame_id = selection.frame_id
        self.selected_measurement_frame_metadata = dict(selection.metadata)

    def show_selected_measurement_frame(self, image: QtGui.QImage) -> None:
        self.measurement_pair_display_active = True
        self.selected_measurement_frame_id = None
        self.selected_measurement_frame_metadata = {}
        self.latest_image = image.copy()
        self._refresh_pixmap()

    def clear_measurement_pair_display(self, *, clear_image: bool = False) -> None:
        self.measurement_pair_display_active = False
        self._clear_selected_measurement_frame_state()
        if clear_image:
            self.latest_image = None
            self.image_label.setPixmap(QtGui.QPixmap())
            self.image_label.setText("동기화 프레임 대기 중")

    def _clear_selected_measurement_frame_state(self) -> None:
        self.selected_measurement_frame_timestamp_s = None
        self.selected_measurement_frame_lookup_delta_s = None
        self.selected_measurement_frame_id = None
        self.selected_measurement_frame_metadata = {}

    def _refresh_pixmap(self) -> None:
        if self.latest_image is None:
            return
        pixmap = QtGui.QPixmap.fromImage(self.latest_image)
        self._draw_bbox_center_adjustment_overlay(pixmap)
        self.image_label.setPixmap(
            pixmap.scaled(self.image_label.size(), QtCore.Qt.KeepAspectRatio, QtCore.Qt.SmoothTransformation)
        )

    def current_image(self) -> QtGui.QImage | None:
        return self.latest_image.copy() if self.latest_image is not None else None

    def set_bbox_center_adjustment(
        self,
        adjustment: tuple[tuple[float, float, float, float], tuple[float, float, float, float]] | None,
    ) -> None:
        self.bbox_center_adjustment = adjustment
        self._set_detection_label(self.latest_detection)
        self._refresh_pixmap()

    def adjusted_detection_center(self, detection: DetectionObservation | None = None) -> tuple[float, float] | None:
        observation = detection or self.latest_detection
        if observation is None or self.bbox_center_adjustment is None:
            return None
        u, v = observation.center
        row_u, row_v = self.bbox_center_adjustment
        return (
            row_u[0] * u + row_u[1] * v + row_u[2] + row_u[3],
            row_v[0] * u + row_v[1] * v + row_v[2] + row_v[3],
        )

    def _draw_bbox_center_adjustment_overlay(self, pixmap: QtGui.QPixmap) -> None:
        if self.latest_detection is None or self.bbox_center_adjustment is None:
            return
        raw_x, raw_y = self.latest_detection.center
        corrected = self.adjusted_detection_center(self.latest_detection)
        if corrected is None:
            return
        corrected_x, corrected_y = corrected
        painter = QtGui.QPainter(pixmap)
        painter.setRenderHint(QtGui.QPainter.Antialiasing)
        painter.setPen(QtGui.QPen(QtGui.QColor(ACCENT), 2))
        painter.setBrush(QtCore.Qt.NoBrush)
        painter.drawEllipse(QtCore.QPointF(raw_x, raw_y), 6, 6)
        painter.setPen(QtGui.QPen(QtGui.QColor(SERIES_HIGHLIGHT), 2))
        painter.drawLine(QtCore.QPointF(raw_x, raw_y), QtCore.QPointF(corrected_x, corrected_y))
        painter.setBrush(QtGui.QColor(SERIES_HIGHLIGHT))
        painter.drawEllipse(QtCore.QPointF(corrected_x, corrected_y), 4, 4)
        painter.setPen(QtGui.QColor(SERIES_HIGHLIGHT))
        painter.drawText(
            QtCore.QPointF(corrected_x + 8, corrected_y - 8),
            "bbox 보정",
        )
        painter.end()

    def _handle_detection_payload(self, payload: dict) -> None:
        observation = DetectionObservation.from_payload(self.camera_label, payload)
        self.latest_detection = observation
        self._set_detection_label(observation)
        self.detection_changed.emit(self.camera_label, observation)

    def _handle_detection_payload_if_current(self, generation: int, payload: dict) -> None:
        if generation != self.stream_generation or self.stop_requested:
            return
        self._handle_detection_payload(payload)

    def _handle_stats_payload(self, payload: dict) -> None:
        self.stats_changed.emit(self.camera_label, payload)

    def _handle_stats_payload_if_current(self, generation: int, payload: dict) -> None:
        if generation != self.stream_generation or self.stop_requested:
            return
        self._handle_stats_payload(payload)

    def _set_status(self, status: str) -> None:
        self.current_status = status
        self.state_changed.emit(self.camera_label, status)

    def _set_status_if_current(self, generation: int, status: str) -> None:
        if generation != self.stream_generation or self.stop_requested:
            return
        self._set_status(status)

    def _emit_recording_signal_if_current(self, generation: int, signal, payload: object) -> None:
        if generation != self.stream_generation:
            return
        signal.emit(payload)

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
        adjusted = self.adjusted_detection_center(detection)
        if adjusted is not None:
            text += f" → corrected=({adjusted[0]:.2f}, {adjusted[1]:.2f}) px"
        self.detection_label.setText(text)

    def _clear_thread_if_current(self, generation: int, thread: QtCore.QThread) -> None:
        if generation != self.stream_generation or self.thread is not thread:
            return
        self.thread = None
        self.worker = None
        self.stop_requested = False
        self.stream_finished.emit(self.camera_label)
