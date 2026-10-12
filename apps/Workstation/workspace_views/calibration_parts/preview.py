"""CalibrationPreview owns its widgets and transient presentation state.

Domain formulas, native raster/timing admission and calibration solves remain with
the existing backend and core contracts. No workspace host is accepted.
"""

from __future__ import annotations

from collections.abc import Mapping
from pathlib import Path
from PyQt5 import QtCore, QtGui, QtWidgets
from ..camera_preview import CameraPreview, CameraPreviewPair


class _SidePreviewReceiver(QtCore.QObject):
    """Side identity and GUI receiver lifetime for one producer's three signals."""

    def __init__(self, preview, side):
        super().__init__(preview)
        self.preview, self.side = preview, side

    @QtCore.pyqtSlot(str, QtGui.QImage, object)
    def frame(self, _label, image, metadata):
        if not self.preview._disposed:
            self.preview._on_camera_frame(self.side, image, metadata)

    @QtCore.pyqtSlot(str, object)
    def detection(self, _label, value):
        if not self.preview._disposed:
            self.preview.previews[self.side].set_detection(value)

    @QtCore.pyqtSlot(str, object)
    def stats(self, _label, value):
        if not self.preview._disposed:
            self.preview.previews[self.side].set_metadata(value)


class CalibrationPreview(QtCore.QObject):
    def __init__(self, parent=None, *, preview_command=lambda: None, channels=(),
                 clear_signal=None, overlay_signal=None):
        super().__init__(parent)
        if not callable(preview_command):
            raise TypeError("Calibration preview requires a refresh-command reader")
        self._preview_command = preview_command
        self._channels = tuple(channels)
        self._clear_signal = clear_signal
        self._overlay_signal = overlay_signal
        self._signal_connections = []
        self._side_receivers = []
        self._camera_signals_connected = False
        self._overlay_signal_connected = False
        self._disposed = False
        self._camera_preview_metadata = {"left": {}, "right": {}}
        self._checkerboard_overlay_active = {"left": False, "right": False}
        self._checkerboard_preview_supported = callable(self._preview_command())
        self._checkerboard_preview_timer = QtCore.QTimer(self)
        self._checkerboard_preview_timer.setInterval(500)
        self._checkerboard_preview_timer.timeout.connect(
            self._request_checkerboard_preview
        )
        self._ftp_preview_timer = QtCore.QTimer(self)
        self._ftp_preview_timer.setSingleShot(True)
        self._ftp_preview_timer.setInterval(1000)
        self._ftp_preview_timer.timeout.connect(
            self._return_to_live_after_ftp_preview
        )
        self._ftp_auto_preview_active = False
        self._ftp_manual_hold = False
        self._camera_mode_programmatic_change = False
        self.camera_mode_tabs = QtWidgets.QTabWidget()
        self.camera_mode_tabs.setObjectName("calibrationCameraModeTabs")
        live_page = QtWidgets.QWidget()
        live_layout = QtWidgets.QVBoxLayout(live_page)
        live_layout.setContentsMargins(4, 4, 4, 4)
        live_layout.setSpacing(5)
        self.preview_splitter = CameraPreviewPair(live_page)
        self.previews = {
            side: CameraPreview(
                side,
                crop_to_fill=False,
                lock_chrome_height=True,
                lock_image_aspect=True,
                wheel_zoom=True,
            )
            for side in ("left", "right")
        }
        for side in ("left", "right"):
            self.preview_splitter.addWidget(self.previews[side])
        self.preview_splitter.setStretchFactor(0, 1)
        self.preview_splitter.setStretchFactor(1, 1)
        live_layout.addWidget(self.preview_splitter)

        ftp_page = QtWidgets.QWidget()
        ftp_layout = QtWidgets.QVBoxLayout(ftp_page)
        ftp_layout.setContentsMargins(4, 4, 4, 4)
        ftp_layout.setSpacing(5)
        self._ftp_preview_status_text = ""
        self.ftp_preview_splitter = CameraPreviewPair(ftp_page)
        self.ftp_previews = {
            side: CameraPreview(
                side,
                crop_to_fill=False,
                lock_chrome_height=True,
                lock_image_aspect=True,
                active_label="FTP STILL",
                empty_image_text="NO FTP IMAGE",
                wheel_zoom=True,
            )
            for side in ("left", "right")
        }
        for side in ("left", "right"):
            self.ftp_preview_splitter.addWidget(self.ftp_previews[side])
        self.ftp_preview_splitter.setStretchFactor(0, 1)
        self.ftp_preview_splitter.setStretchFactor(1, 1)
        ftp_layout.addWidget(self.ftp_preview_splitter)

        self.camera_mode_tabs.addTab(live_page, "Live")
        self.camera_mode_tabs.addTab(ftp_page, "FTP")
        self.camera_mode_tabs.currentChanged.connect(
            self._on_camera_mode_changed
        )
        self.camera_mode_tabs.tabBar().tabBarClicked.connect(
            self._on_camera_mode_tab_clicked
        )

    def build_simple_previews(self, parent):
        simple_preview_splitter = CameraPreviewPair(parent)
        simple_preview_splitter.setObjectName("simpleCalibrationPreviewPair")
        self.simple_calibration_previews = {
            side: CameraPreview(
                side,
                crop_to_fill=False,
                lock_chrome_height=True,
                lock_image_aspect=True,
                active_label="CHECKERBOARD",
                empty_image_text="NO CALIBRATION VIDEO",
                maximum_image_height=200,
            )
            for side in ("left", "right")
        }
        self._simple_calibration_detection_text = {
            "left": "보드: 프레임 없음",
            "right": "보드: 프레임 없음",
        }
        for side in ("left", "right"):
            simple_preview_splitter.addWidget(self.simple_calibration_previews[side])
            self.simple_calibration_previews[side].meta.hide()
        simple_preview_splitter.setStretchFactor(0, 1)
        simple_preview_splitter.setStretchFactor(1, 1)
        self.simple_calibration_preview_splitter = simple_preview_splitter

        return simple_preview_splitter

    def set_detection_text(self, side, text):
        self._simple_calibration_detection_text[side] = text
        self._set_simple_calibration_preview_meta(side)

    def stop(self):
        self._disposed = True
        self._checkerboard_preview_timer.stop()
        self._ftp_preview_timer.stop()
        # Disconnect only this view's receivers. Already queued deliveries are
        # fenced by _disposed, and QObject destruction retires their context.
        for signal, receiver in self._signal_connections:
            try:
                signal.disconnect(receiver)
            except (TypeError, RuntimeError):
                pass
        self._signal_connections.clear()

    def _connect_signal(self, signal, receiver):
        connect = getattr(signal, "connect", None)
        if not self._disposed and callable(connect):
            connect(receiver)
            self._signal_connections.append((signal, receiver))

    def _set_simple_calibration_preview_meta(self, side: str) -> None:
        previews = getattr(self, "simple_calibration_previews", {})
        preview = previews.get(side) if isinstance(previews, dict) else None
        if preview is None:
            return
        text = str(
            getattr(self, "_simple_calibration_detection_text", {}).get(
                side,
                "보드: 상태 없음",
            )
        )
        preview.meta.setText(text)
        preview.meta.setToolTip(text)


    def _set_ftp_preview_status(self, text: object) -> None:
        """Retain FTP preview detail without reserving a visible status strip."""

        self._ftp_preview_status_text = str(text or "")
        ftp_index = self.camera_mode_tabs.indexOf(self.ftp_preview_splitter.parentWidget())
        if ftp_index >= 0:
            self.camera_mode_tabs.setTabToolTip(ftp_index, self._ftp_preview_status_text)


    def _show_ftp_pair_preview(self, payload: Mapping[str, object]) -> None:
        raw_paths = payload.get("paths")
        paths = raw_paths if isinstance(raw_paths, Mapping) else {}
        images: dict[str, QtGui.QImage] = {}
        for side in ("left", "right"):
            path = str(paths.get(side) or "").strip()
            image = QtGui.QImage(path) if path else QtGui.QImage()
            if image.isNull():
                self._set_ftp_preview_status(
                    f"FTP preview failed: {side} image is missing or unreadable ({path or 'no path'})."
                )
                return
            images[side] = image

        capture_id = str(payload.get("capture_id") or "").strip() or "unknown"
        operation = str(payload.get("operation") or "FTP capture")
        for side in ("left", "right"):
            path = str(paths.get(side) or "")
            self.ftp_previews[side].set_frame(
                images[side],
                {
                    "capture_id": capture_id,
                    "path": path,
                    "width": images[side].width(),
                    "height": images[side].height(),
                },
            )
        left_name = Path(str(paths.get("left") or "")).name
        right_name = Path(str(paths.get("right") or "")).name
        self._set_ftp_preview_status(
            f"{operation} · {capture_id} · Left={left_name} · Right={right_name}"
        )

        self._ftp_preview_timer.stop()
        if self._ftp_manual_hold and self.camera_mode_tabs.currentIndex() == 1:
            self._ftp_auto_preview_active = False
            return
        self._ftp_auto_preview_active = True
        self._camera_mode_programmatic_change = True
        try:
            self.camera_mode_tabs.setCurrentIndex(1)
        finally:
            self._camera_mode_programmatic_change = False
        self._ftp_preview_timer.start()


    def _on_camera_mode_tab_clicked(self, index: int) -> None:
        self._ftp_manual_hold = index == 1
        self._ftp_auto_preview_active = False
        self._ftp_preview_timer.stop()


    def _on_camera_mode_changed(self, index: int) -> None:
        if self._camera_mode_programmatic_change:
            return
        self._ftp_manual_hold = index == 1
        self._ftp_auto_preview_active = False
        self._ftp_preview_timer.stop()


    def _return_to_live_after_ftp_preview(self) -> None:
        if self._disposed:
            return
        if not self._ftp_auto_preview_active:
            return
        self._ftp_auto_preview_active = False
        if self.camera_mode_tabs.currentIndex() != 1:
            return
        self._camera_mode_programmatic_change = True
        try:
            self.camera_mode_tabs.setCurrentIndex(0)
        finally:
            self._camera_mode_programmatic_change = False


    def _connect_camera_panes(self) -> None:
        if self._disposed or self._camera_signals_connected:
            return
        self._camera_signals_connected = True
        self._connect_signal(self._clear_signal, self._clear_camera_previews)
        for channel in self._channels:
            if channel.side not in self.previews:
                raise ValueError(f"Unknown calibration preview side: {channel.side}")
            receiver = _SidePreviewReceiver(self, channel.side)
            self._side_receivers.append(receiver)
            self._connect_signal(channel.frame_changed, receiver.frame)
            self._connect_signal(channel.detection_changed, receiver.detection)
            self._connect_signal(channel.stats_changed, receiver.stats)

    @QtCore.pyqtSlot()
    def _clear_camera_previews(self) -> None:
        if self._disposed:
            return
        self._checkerboard_overlay_active = {"left": False, "right": False}
        self._camera_preview_metadata = {"left": {}, "right": {}}
        for previews in (
            self.previews,
            getattr(self, "simple_calibration_previews", {}),
        ):
            for preview in previews.values():
                preview.clear()
        if hasattr(self, "_simple_calibration_detection_text"):
            self._simple_calibration_detection_text = {
                "left": "보드: 프레임 없음",
                "right": "보드: 프레임 없음",
            }
            for side in ("left", "right"):
                self._set_simple_calibration_preview_meta(side)


    def _connect_calibration_preview_signal(self) -> None:
        if self._disposed or self._overlay_signal_connected:
            return
        self._overlay_signal_connected = True
        self._connect_signal(self._overlay_signal, self._on_calibration_preview_changed)

    def _show_preview_frame(self, side: str, image: QtGui.QImage, metadata: object) -> None:
        """Route one frame to the advanced preview and, once built, the simple one."""

        self.previews[side].set_frame(image, metadata)
        # Frame signals are connected before the simple surface exists, so the
        # attribute lookup has to stay tolerant during construction.
        simple = getattr(self, "simple_calibration_previews", {}).get(side)
        if simple is not None:
            simple.set_frame(image, metadata)
            self._set_simple_calibration_preview_meta(side)


    def _on_camera_frame(self, side: str, image: QtGui.QImage, metadata: object) -> None:
        if self._disposed:
            return
        self._camera_preview_metadata[side] = metadata
        if not self._checkerboard_overlay_active[side]:
            self._show_preview_frame(side, image, metadata)


    @QtCore.pyqtSlot(object)
    def _on_calibration_preview_changed(self, payload: object) -> None:
        if self._disposed or not isinstance(payload, dict):
            return
        camera = str(payload.get("camera") or "")
        image = payload.get("image")
        if camera not in self.previews or not isinstance(image, QtGui.QImage) or image.isNull():
            return
        kind = str(payload.get("kind") or "raw")
        metadata = payload.get("metadata", self._camera_preview_metadata[camera])
        if kind == "overlay":
            # Board indicators and counters come from the 4 Hz refresh; an
            # overlay frame must not trigger another capture-root snapshot.
            self._checkerboard_overlay_active[camera] = True
            self._show_preview_frame(camera, image, metadata)
        elif not self._checkerboard_overlay_active[camera]:
            self._show_preview_frame(camera, image, metadata)


    def _request_checkerboard_preview(self) -> None:
        if self._disposed or not self._checkerboard_preview_supported:
            return
        self._checkerboard_overlay_active = {"left": False, "right": False}
        refresh = self._preview_command()
        if not callable(refresh):
            return
        try:
            refresh()
        except Exception as exc:
            for previews in (
                self.previews,
                getattr(self, "simple_calibration_previews", {}),
            ):
                for preview in previews.values():
                    preview.meta.setToolTip(f"Checkerboard preview unavailable: {exc}")


    def _start_checkerboard_preview(self) -> None:
        if self._disposed or not self._checkerboard_preview_supported:
            return
        if not self._checkerboard_preview_timer.isActive():
            self._checkerboard_preview_timer.start()
        self._request_checkerboard_preview()



    # Narrow sibling operations; underscore names remain at the legacy edge.
    show_ftp_pair = _show_ftp_pair_preview
    set_ftp_status = _set_ftp_preview_status
