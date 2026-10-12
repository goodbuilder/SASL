"""Independent camera preview widgets and pair layout helpers."""

from __future__ import annotations

import math
from collections.abc import Mapping
from pathlib import Path
from PyQt5 import QtCore, QtGui, QtWidgets
from Workstation.workstation_core.theme import ACCENT
from Workstation.workstation_core.stereo import DetectionObservation


def _balance_preview_pair(splitter: QtWidgets.QSplitter) -> None:
    """Give the two camera previews equal room before sizing them by aspect."""

    if splitter.width() <= 0 or splitter.height() <= 0:
        return
    if splitter.orientation() == QtCore.Qt.Horizontal:
        available = max(2, splitter.width() - splitter.handleWidth())
    else:
        available = max(2, splitter.height() - splitter.handleWidth())
    first = available // 2
    sizes = [first, available - first]
    if splitter.sizes() != sizes:
        splitter.setSizes(sizes)


def _preview_pair_height(
    splitter: QtWidgets.QSplitter,
    previews: Mapping[str, "CameraPreview"],
) -> int:
    """Return the height needed to show both previews without letterboxing."""

    _balance_preview_pair(splitter)
    heights = [
        max(0, preview.heightForWidth(max(1, preview.width())))
        for preview in previews.values()
    ]
    if not heights:
        return 0
    if splitter.orientation() == QtCore.Qt.Horizontal:
        return max(heights)
    return sum(heights) + splitter.handleWidth() * max(0, len(heights) - 1)


class CameraPreview(QtWidgets.QFrame):
    IMAGE_ASPECT_RATIO = 16.0 / 9.0
    # Same wheel-zoom contract as AnnotationImageView on the Alignment page.
    MIN_ZOOM_FACTOR = 1.0
    MAX_ZOOM_FACTOR = 8.0
    WHEEL_ZOOM_STEP = 1.2

    def __init__(
        self,
        side: str,
        parent: QtWidgets.QWidget | None = None,
        *,
        crop_to_fill: bool = True,
        lock_chrome_height: bool = False,
        lock_image_aspect: bool = False,
        active_label: str = "LIVE",
        empty_image_text: str = "NO VIDEO",
        maximum_image_height: int | None = None,
        wheel_zoom: bool = False,
    ) -> None:
        super().__init__(parent)
        self.side = side
        self.crop_to_fill = bool(crop_to_fill)
        # Plain-wheel zoom is opt-in: previews embedded in scrolling pages
        # must keep forwarding wheel input to the enclosing QScrollArea unless
        # the page explicitly makes the camera surface the wheel target.
        # Zoom is only implemented for the fit (non-crop) render path.
        self.wheel_zoom = bool(wheel_zoom) and not self.crop_to_fill
        self.lock_chrome_height = bool(lock_chrome_height)
        self.lock_image_aspect = bool(lock_image_aspect)
        self.active_label = str(active_label)
        self.empty_image_text = str(empty_image_text)
        self.maximum_image_height = (
            None
            if maximum_image_height is None
            else max(1, int(maximum_image_height))
        )
        self.setObjectName("cameraPreview")
        self.setMinimumSize(0, 0)
        self.setSizePolicy(QtWidgets.QSizePolicy.Ignored, QtWidgets.QSizePolicy.Expanding)
        layout = QtWidgets.QVBoxLayout(self)
        layout.setContentsMargins(4, 4, 4, 4)
        layout.setSpacing(3)
        title = QtWidgets.QLabel("Left camera" if side == "left" else "Right camera")
        title.setObjectName("cameraPreviewTitle")
        self.image = QtWidgets.QLabel(self.empty_image_text)
        self.image.setAlignment(QtCore.Qt.AlignCenter)
        self.image.setMinimumSize(0, 0)
        self.image.setSizePolicy(
            QtWidgets.QSizePolicy.Ignored,
            QtWidgets.QSizePolicy.Fixed
            if self.lock_image_aspect
            else QtWidgets.QSizePolicy.Expanding,
        )
        self.image.setObjectName("cameraPreviewImage")
        self.meta = QtWidgets.QLabel("WAITING")
        self.meta.setObjectName("cameraPreviewMeta")
        self.meta.setWordWrap(not self.lock_chrome_height)
        self.meta.setMinimumWidth(0)
        if self.lock_chrome_height:
            # Measurement receives live metadata and detection labels while
            # the image surface is being painted.  A wrapping metadata label
            # changes its sizeHint when a class name or status changes, which
            # lets the parent splitter resize the camera surface on every
            # update.  Keep one reserved line and expose the full text in the
            # tooltip so content changes cannot move the image.
            self.meta.setSizePolicy(QtWidgets.QSizePolicy.Ignored, QtWidgets.QSizePolicy.Fixed)
            self._locked_meta_height = max(1, self.meta.fontMetrics().lineSpacing())
            self.meta.setFixedHeight(self._locked_meta_height)
        else:
            self.meta.setSizePolicy(QtWidgets.QSizePolicy.Ignored, QtWidgets.QSizePolicy.Preferred)
        layout.addWidget(title)
        layout.addWidget(self.image, 0 if self.lock_image_aspect else 1)
        layout.addWidget(self.meta)
        if self.lock_image_aspect:
            layout.addStretch(1)
        self._image: QtGui.QImage | None = None
        self._detection: DetectionObservation | None = None
        self._metadata: object = {}
        self._center_line_visible = False
        self._zoom_factor = self.MIN_ZOOM_FACTOR
        # Native (u, v) pixel center kept at the viewport center while zoomed.
        self._view_center_source: tuple[float, float] | None = None
        if self.wheel_zoom:
            self.image.setToolTip(
                "Mouse wheel over the image zooms 1×–8× around the pointer."
            )

    def set_center_line_visible(self, visible: bool) -> None:
        """Toggle a 1 px red vertical guide at the transformed source midpoint.

        The guide is display-only chrome painted on the rendered copy; the
        source frame handed to capture/evidence paths is never modified.
        """

        visible = bool(visible)
        if visible == self._center_line_visible:
            return
        self._center_line_visible = visible
        if self.isVisible():
            self._render()

    def center_line_visible(self) -> bool:
        return self._center_line_visible

    def zoom_factor(self) -> float:
        return self._zoom_factor

    def reset_zoom(self) -> None:
        """Return to the fully zoomed-out, centered fit view."""

        changed = (
            self._zoom_factor != self.MIN_ZOOM_FACTOR
            or self._view_center_source is not None
        )
        self._zoom_factor = self.MIN_ZOOM_FACTOR
        self._view_center_source = None
        if changed and self.isVisible():
            self._render()

    def _fit_display_geometry(
        self,
    ) -> tuple[float, float, float, float, float] | None:
        """Fit-path transform as ``(scale, offset_x, offset_y, draw_w, draw_h)``.

        ``display = source * scale + offset`` in image-label coordinates.
        Only valid when ``crop_to_fill`` is False.
        """

        if self._image is None or self._image.isNull():
            return None
        source_width = self._image.width()
        source_height = self._image.height()
        target = self.image.size()
        if (
            source_width <= 0
            or source_height <= 0
            or target.width() <= 0
            or target.height() <= 0
        ):
            return None
        fit_scale = min(
            target.width() / source_width, target.height() / source_height
        )
        scale = fit_scale * self._zoom_factor
        draw_width = source_width * scale
        draw_height = source_height * scale
        if self._zoom_factor <= self.MIN_ZOOM_FACTOR or self._view_center_source is None:
            offset_x = (target.width() - draw_width) / 2.0
            offset_y = (target.height() - draw_height) / 2.0
        else:
            center_u = min(max(self._view_center_source[0], 0.0), float(source_width - 1))
            center_v = min(max(self._view_center_source[1], 0.0), float(source_height - 1))
            offset_x = target.width() / 2.0 - (center_u + 0.5) * scale
            offset_y = target.height() / 2.0 - (center_v + 0.5) * scale
        return scale, offset_x, offset_y, draw_width, draw_height

    def wheelEvent(self, event: QtGui.QWheelEvent) -> None:  # noqa: N802 - Qt API.
        """Zoom around the pointer while keeping the pointed native pixel fixed."""

        if not self.wheel_zoom:
            super().wheelEvent(event)
            return
        geometry = self._fit_display_geometry()
        if geometry is None:
            super().wheelEvent(event)
            return
        scale, offset_x, offset_y, draw_width, draw_height = geometry
        cursor = QtCore.QPointF(event.pos()) - QtCore.QPointF(self.image.pos())
        image_rect = QtCore.QRectF(0.0, 0.0, float(self.image.width()), float(self.image.height()))
        display_rect = QtCore.QRectF(offset_x, offset_y, draw_width, draw_height)
        if not image_rect.contains(cursor) or not display_rect.contains(cursor):
            super().wheelEvent(event)
            return
        delta_y = float(event.angleDelta().y())
        if delta_y == 0.0:
            delta_y = float(event.pixelDelta().y())
        if delta_y == 0.0:
            super().wheelEvent(event)
            return
        steps = delta_y / 120.0
        next_zoom = min(
            self.MAX_ZOOM_FACTOR,
            max(self.MIN_ZOOM_FACTOR, self._zoom_factor * self.WHEEL_ZOOM_STEP**steps),
        )
        if math.isclose(next_zoom, self._zoom_factor, rel_tol=0.0, abs_tol=1.0e-12):
            event.accept()
            return
        source_width = self._image.width()
        source_height = self._image.height()
        source_u = min(max((cursor.x() - offset_x) / scale - 0.5, 0.0), float(source_width - 1))
        source_v = min(max((cursor.y() - offset_y) / scale - 0.5, 0.0), float(source_height - 1))
        if next_zoom <= self.MIN_ZOOM_FACTOR:
            # Fully zoomed out must be the centered fit view, otherwise
            # repeated in/out wheel steps drift the image sideways.
            self._view_center_source = None
        else:
            next_scale = (scale / self._zoom_factor) * next_zoom
            viewport_cx = self.image.width() / 2.0
            viewport_cy = self.image.height() / 2.0
            self._view_center_source = (
                min(max(source_u - (cursor.x() - viewport_cx) / next_scale, 0.0), float(source_width - 1)),
                min(max(source_v - (cursor.y() - viewport_cy) / next_scale, 0.0), float(source_height - 1)),
            )
        self._zoom_factor = next_zoom
        self._render()
        event.accept()

    def hasHeightForWidth(self) -> bool:  # noqa: N802 - Qt override.
        return True

    def heightForWidth(self, width: int) -> int:  # noqa: N802 - Qt override.
        content_width = max(0, int(width) - self.layout().contentsMargins().left() - self.layout().contentsMargins().right())
        image_height = int(round(content_width / self.IMAGE_ASPECT_RATIO))
        if self.maximum_image_height is not None:
            image_height = min(image_height, self.maximum_image_height)
        title = self.findChild(QtWidgets.QLabel, "cameraPreviewTitle")
        chrome_height = title.sizeHint().height() if title is not None else 0
        meta_visible = not self.meta.isHidden()
        if meta_visible:
            chrome_height += (
                self._locked_meta_height
                if self.lock_chrome_height
                else self.meta.sizeHint().height()
            )
        spacing_count = 2 if meta_visible else 1
        return image_height + chrome_height + self.layout().spacing() * spacing_count + self.layout().contentsMargins().top() + self.layout().contentsMargins().bottom()

    def set_frame(self, image: QtGui.QImage, metadata: object) -> None:
        self._image = image
        if self.isVisible():
            self._render()
        self._set_metadata(metadata, live=True)

    def clear(self) -> None:
        """Clear the displayed frame when the camera Preview is stopped."""

        self._image = None
        self._detection = None
        self._metadata = {}
        if self.isVisible():
            self._render()
        self._set_metadata({}, live=False)

    def set_detection(self, detection: DetectionObservation | None) -> None:
        """Keep the latest image-space bbox for this display preview.

        Detection coordinates remain in the source image pixel contract
        (x1/y1/x2/y2, or u/v).  Only the copy rendered into the widget is
        scaled for presentation; no camera/world-frame conversion occurs.
        """

        self._detection = detection
        if self.isVisible():
            self._render()
        self._set_metadata(
            self._metadata,
            live=self._image is not None and not self._image.isNull(),
        )

    def set_metadata(self, metadata: object) -> None:
        live = self._image is not None and not self._image.isNull()
        self._set_metadata(metadata, live=live)

    def _set_metadata(self, metadata: object, *, live: bool) -> None:
        self._metadata = metadata
        detail = self._metadata_text(metadata, live=live)
        self.meta.setText(detail)
        self.meta.setToolTip(detail)

    def _metadata_text(self, metadata: object, *, live: bool) -> str:
        parts = [self.active_label if live else "WAITING"]
        if self.active_label != "LIVE":
            if isinstance(metadata, Mapping):
                path = str(metadata.get("path") or "")
                if path:
                    parts.append(Path(path).name)
                width = metadata.get("width")
                height = metadata.get("height")
                if width and height:
                    parts.append(f"{width}×{height}")
            return " | ".join(parts)
        if self._detection is None:
            parts.append("Class: --")
        else:
            class_name = self._detection.class_name or f"#{self._detection.class_id}"
            parts.append(f"Class: {class_name}")
        if (
            isinstance(metadata, Mapping)
            and bool(metadata.get("media_integrity_unverified"))
        ):
            parts.append("INPUT UNVERIFIED")
        return " | ".join(parts)

    def _render(self) -> None:
        if self._image is None or self._image.isNull():
            self.image.setText(self.empty_image_text)
            self.image.setPixmap(QtGui.QPixmap())
            return
        target_size = self.image.size()
        source_pixmap = QtGui.QPixmap.fromImage(self._image)
        if self.crop_to_fill:
            scaled = source_pixmap.scaled(
                target_size, QtCore.Qt.KeepAspectRatioByExpanding, QtCore.Qt.SmoothTransformation
            )
            crop_x = max(0, (scaled.width() - target_size.width()) // 2)
            crop_y = max(0, (scaled.height() - target_size.height()) // 2)
            pixmap = scaled.copy(crop_x, crop_y, target_size.width(), target_size.height())
        elif self._zoom_factor <= self.MIN_ZOOM_FACTOR:
            scaled = source_pixmap.scaled(
                target_size, QtCore.Qt.KeepAspectRatio, QtCore.Qt.SmoothTransformation
            )
            offset_x = max(0, (target_size.width() - scaled.width()) // 2)
            offset_y = max(0, (target_size.height() - scaled.height()) // 2)
            pixmap = QtGui.QPixmap(target_size)
            pixmap.fill(QtCore.Qt.black)
            painter = QtGui.QPainter(pixmap)
            painter.drawPixmap(offset_x, offset_y, scaled)
            painter.end()
            # _draw_detection_overlay uses display = source * scale - crop.
            crop_x = -offset_x
            crop_y = -offset_y
        else:
            # Zoomed fit view: paint the transformed source directly so the
            # off-screen part is clipped instead of materialised as a huge
            # scaled pixmap at 8x.
            geometry = self._fit_display_geometry()
            if geometry is None:
                self.image.setText(self.empty_image_text)
                self.image.setPixmap(QtGui.QPixmap())
                return
            scale, offset_x, offset_y, draw_width, draw_height = geometry
            pixmap = QtGui.QPixmap(target_size)
            pixmap.fill(QtCore.Qt.black)
            painter = QtGui.QPainter(pixmap)
            painter.setRenderHint(QtGui.QPainter.SmoothPixmapTransform, True)
            painter.drawPixmap(
                QtCore.QRectF(offset_x, offset_y, draw_width, draw_height),
                source_pixmap,
                QtCore.QRectF(source_pixmap.rect()),
            )
            painter.end()
            scaled = QtCore.QSizeF(draw_width, draw_height)
            crop_x = -offset_x
            crop_y = -offset_y
        if self._detection is not None:
            pixmap = self._draw_detection_overlay(
                pixmap,
                self._detection,
                scale_x=scaled.width() / source_pixmap.width(),
                scale_y=scaled.height() / source_pixmap.height(),
                crop_x=crop_x,
                crop_y=crop_y,
            )
        if self._center_line_visible:
            pixmap = self._draw_center_line(
                pixmap,
                center_x=scaled.width() / 2.0 - crop_x,
                top=max(0, int(math.floor(-crop_y))),
                bottom=min(pixmap.height(), int(math.ceil(scaled.height() - crop_y))),
            )
        self.image.setPixmap(pixmap)

    def _lock_image_height(self, preview_width: int) -> None:
        if not self.lock_image_aspect:
            return
        margins = self.layout().contentsMargins()
        content_width = max(0, int(preview_width) - margins.left() - margins.right())
        desired_height = int(round(content_width / self.IMAGE_ASPECT_RATIO))
        if self.maximum_image_height is not None:
            desired_height = min(desired_height, self.maximum_image_height)
        if desired_height > 0 and self.image.height() != desired_height:
            self.image.setFixedHeight(desired_height)

    @staticmethod
    def _draw_center_line(
        pixmap: QtGui.QPixmap,
        *,
        center_x: float,
        top: int,
        bottom: int,
    ) -> QtGui.QPixmap:
        """Paint the transformed native image center when it is in view."""

        if pixmap.isNull() or bottom <= top or not 0.0 <= center_x < pixmap.width():
            return pixmap
        # Cursor-centered zoom can move the source center outside the viewport.
        # Clamping it to an edge would show an incorrect installation guide.
        column = int(center_x)
        painter = QtGui.QPainter(pixmap)
        try:
            painter.setRenderHint(QtGui.QPainter.Antialiasing, False)
            pen = QtGui.QPen(QtGui.QColor(255, 0, 0))
            pen.setWidth(1)
            pen.setCosmetic(True)
            painter.setPen(pen)
            painter.drawLine(column, int(top), column, int(bottom) - 1)
        finally:
            painter.end()
        return pixmap

    @staticmethod
    def _draw_detection_overlay(
        pixmap: QtGui.QPixmap,
        detection: DetectionObservation,
        *,
        scale_x: float | None = None,
        scale_y: float | None = None,
        crop_x: int = 0,
        crop_y: int = 0,
    ) -> QtGui.QPixmap:
        """Draw a bbox and circular center marker on the display pixmap."""

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

    def resizeEvent(self, event: QtGui.QResizeEvent) -> None:  # noqa: N802
        self._lock_image_height(event.size().width())
        super().resizeEvent(event)
        if self.isVisible():
            self._render()

    def showEvent(self, event: QtGui.QShowEvent) -> None:  # noqa: N802
        super().showEvent(event)
        self._lock_image_height(self.width())
        self._render()


class CameraPreviewPair(QtWidgets.QSplitter):
    """Aspect-aware pair container for the calibration camera previews."""

    def __init__(self, parent: QtWidgets.QWidget | None = None) -> None:
        super().__init__(QtCore.Qt.Horizontal, parent)
        self._equalizing_sizes = False
        self.setChildrenCollapsible(False)
        self.setSizePolicy(QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Fixed)
        self.splitterMoved.connect(self._restore_equal_sizes)

    def _restore_equal_sizes(self, *_args: object) -> None:
        """Keep both calibration preview surfaces at the same display size."""

        if self._equalizing_sizes or self.orientation() != QtCore.Qt.Horizontal:
            return
        if self.count() != 2:
            return
        available_width = sum(self.sizes())
        if available_width <= 0:
            available_width = max(0, self.width() - self.handleWidth())
        left_width = available_width // 2
        equal_sizes = [left_width, available_width - left_width]
        if self.sizes() == equal_sizes:
            return
        self._equalizing_sizes = True
        try:
            self.setSizes(equal_sizes)
        finally:
            self._equalizing_sizes = False

    def hasHeightForWidth(self) -> bool:  # noqa: N802 - Qt override.
        return True

    def heightForWidth(self, width: int) -> int:  # noqa: N802 - Qt override.
        previews = [self.widget(index) for index in range(self.count())]
        if not previews:
            return 0
        if self.orientation() == QtCore.Qt.Horizontal:
            available_width = max(0, int(width) - self.handleWidth())
            first_width = available_width // len(previews)
            widths = [first_width] * len(previews)
            widths[-1] += available_width - sum(widths)
            heights = [
                int(preview.heightForWidth(max(0, preview_width)))
                if callable(getattr(preview, "heightForWidth", None))
                else 0
                for preview, preview_width in zip(previews, widths)
            ]
            return max(heights, default=0)
        preview_height = max(0, int(width))
        return sum(
            int(preview.heightForWidth(preview_height))
            if callable(getattr(preview, "heightForWidth", None))
            else 0
            for preview in previews
        ) + self.handleWidth() * max(0, len(previews) - 1)

    def sizeHint(self) -> QtCore.QSize:  # noqa: N802 - Qt override.
        hint = super().sizeHint()
        if hint.width() > 0:
            hint.setHeight(self.heightForWidth(hint.width()))
        return hint

    def resizeEvent(self, event: QtGui.QResizeEvent) -> None:  # noqa: N802 - Qt override.
        super().resizeEvent(event)
        self._restore_equal_sizes()
        desired_height = self.heightForWidth(event.size().width())
        if desired_height > 0 and self.minimumHeight() != desired_height:
            self.setMinimumHeight(desired_height)

    def setOrientation(self, orientation: QtCore.Qt.Orientation) -> None:  # noqa: N802 - Qt override.
        super().setOrientation(orientation)
        self.updateGeometry()
