from __future__ import annotations

import time
from collections import deque

from PyQt5 import QtCore, QtGui, QtWidgets

from Workstation.workstation_core.stereo import DetectionObservation
from Workstation.workstation_core.theme import ACCENT
from Workstation.workstation_core.yolo_stream import YoloStreamWorker


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


class CameraPane(QtWidgets.QGroupBox):
    detection_changed = QtCore.pyqtSignal(str, object)
    frame_changed = QtCore.pyqtSignal(str, QtGui.QImage, object)
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
        self.measurement_frame_order: deque[float] = deque(maxlen=120)
        self.measurement_pair_display_active = False
        self.selected_measurement_frame_timestamp_s: float | None = None
        self.selected_measurement_frame_lookup_delta_s: float | None = None
        self.detection_stream_active = False
        self.stream_generation = 0
        self.current_status = "중지됨"
        self._stop_requested_monotonic_s: float | None = None
        self._stop_escalation_level = 0

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
        self.measurement_frame_order.clear()
        self.measurement_pair_display_active = False
        self.selected_measurement_frame_timestamp_s = None
        self.selected_measurement_frame_lookup_delta_s = None
        self.detection_stream_active = bool(enable_detection)
        self._stop_requested_monotonic_s = None
        self._stop_escalation_level = 0
        self.image_label.setPixmap(QtGui.QPixmap())
        self.image_label.setText("YOLO 시작 중..." if enable_detection else "카메라 연결 중...")
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
            lambda timestamp_s, image, generation=stream_generation: self._store_measurement_frame_if_current(
                generation, timestamp_s, image
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
        if worker is not None:
            worker.request_stop()
            if self._stop_requested_monotonic_s is None:
                self._stop_requested_monotonic_s = time.monotonic()
        stopped = thread is None
        if thread is not None:
            if wait:
                stopped = bool(thread.wait(1500))
                if not stopped and worker is not None:
                    # The child process is the resource owner.  Stop it before
                    # considering a QThread fallback; terminating the Qt thread
                    # would skip the worker's finally block and lose its cleanup.
                    force_stop = getattr(worker, "stop", None)
                    if callable(force_stop):
                        force_stop()
                    stopped = bool(thread.wait(1500))
            else:
                is_running = getattr(thread, "isRunning", None)
                stopped = bool(is_running is not None and callable(is_running) and not is_running())
                if not stopped and worker is not None and self._stop_requested_monotonic_s is not None:
                    elapsed_s = max(0.0, time.monotonic() - self._stop_requested_monotonic_s)
                    if elapsed_s >= 3.0 and self._stop_escalation_level < 2:
                        kill_child = getattr(worker, "kill_child_process", None)
                        if callable(kill_child):
                            kill_child()
                        self._stop_escalation_level = 2
                    elif elapsed_s >= 1.5 and self._stop_escalation_level < 1:
                        terminate_child = getattr(worker, "terminate_child_process", None)
                        if callable(terminate_child):
                            terminate_child()
                        self._stop_escalation_level = 1
        if stopped:
            self.worker = None
            self.thread = None
            self._stop_requested_monotonic_s = None
            self._stop_escalation_level = 0
        self.latest_image = None
        self.latest_detection = None
        self.measurement_frame_images.clear()
        self.measurement_frame_order.clear()
        self.measurement_pair_display_active = False
        self.selected_measurement_frame_timestamp_s = None
        self.selected_measurement_frame_lookup_delta_s = None
        self.detection_stream_active = False
        self.image_label.setText("중지됨")
        self.image_label.setPixmap(QtGui.QPixmap())
        self._set_status("중지됨")
        self._set_detection_label(None)
        self.detection_changed.emit(self.camera_label, None)
        if not stopped:
            self._set_status("종료 대기 중")
        return stopped

    def resizeEvent(self, event) -> None:  # noqa: N802 - Qt override.
        self._refresh_pixmap()
        super().resizeEvent(event)

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
        if self.measurement_pair_display_active and self.detection_stream_active:
            self.frame_changed.emit(self.camera_label, image, frame_metadata)
            return
        self.measurement_pair_display_active = False
        self.latest_image = image
        self.frame_changed.emit(self.camera_label, image, frame_metadata)
        self._refresh_pixmap()

    def _show_frame_if_current(self, generation: int, image: QtGui.QImage, metadata: object) -> None:
        if generation != self.stream_generation:
            return
        self._show_frame(image, metadata)

    def _store_measurement_frame(self, timestamp_s: float, image: QtGui.QImage) -> None:
        key = float(timestamp_s)
        if key not in self.measurement_frame_images:
            if len(self.measurement_frame_order) == self.measurement_frame_order.maxlen:
                old_key = self.measurement_frame_order.popleft()
                self.measurement_frame_images.pop(old_key, None)
            self.measurement_frame_order.append(key)
        self.measurement_frame_images[key] = image.copy()

    def _store_measurement_frame_if_current(self, generation: int, timestamp_s: float, image: QtGui.QImage) -> None:
        if generation != self.stream_generation:
            return
        self._store_measurement_frame(timestamp_s, image)

    def show_measurement_frame_for_detection(self, detection: DetectionObservation) -> bool:
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
        self.show_selected_measurement_frame(image)
        return True

    def show_selected_measurement_frame(self, image: QtGui.QImage) -> None:
        self.measurement_pair_display_active = True
        self.latest_image = image.copy()
        self._refresh_pixmap()

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
        if self.latest_image is None:
            return
        pixmap = QtGui.QPixmap.fromImage(self.latest_image)
        scaled = pixmap.scaled(self.image_label.size(), QtCore.Qt.KeepAspectRatio, QtCore.Qt.SmoothTransformation)
        if self.latest_detection is not None and not self.measurement_pair_display_active:
            scaled = self._draw_detection_overlay(scaled, self.latest_detection)
        self.image_label.setPixmap(scaled)

    def _draw_detection_overlay(
        self, pixmap: QtGui.QPixmap, detection: DetectionObservation
    ) -> QtGui.QPixmap:
        if detection.width <= 0 or detection.height <= 0:
            return pixmap
        scale_x = pixmap.width() / detection.width
        scale_y = pixmap.height() / detection.height
        painter = QtGui.QPainter(pixmap)
        try:
            painter.setRenderHint(QtGui.QPainter.Antialiasing)
            color = QtGui.QColor(ACCENT)
            rect = QtCore.QRectF(
                detection.x1 * scale_x,
                detection.y1 * scale_y,
                (detection.x2 - detection.x1) * scale_x,
                (detection.y2 - detection.y1) * scale_y,
            )
            painter.setPen(QtGui.QPen(color, 2))
            painter.drawRect(rect)
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
        self._handle_detection_payload(payload)

    def _handle_stats_payload(self, payload: dict) -> None:
        self.stats_changed.emit(self.camera_label, payload)

    def _handle_stats_payload_if_current(self, generation: int, payload: dict) -> None:
        if generation != self.stream_generation:
            return
        self._handle_stats_payload(payload)

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
        self.thread = None
        self.worker = None
        self._stop_requested_monotonic_s = None
        self._stop_escalation_level = 0
