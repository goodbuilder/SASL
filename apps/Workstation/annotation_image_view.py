"""Frozen-native-frame annotation widget for optical-pose evidence.

The widget deliberately has no live-preview input.  A frame can only be loaded
with a stable ``frame_id`` and image digest, and all annotations are stored in
the source image's native OpenCV pixel coordinates: ``(u, v)``, with ``u``
increasing right and ``v`` increasing down.  Widget coordinates are used only
for display and mouse interaction.
"""

from __future__ import annotations

import hashlib
import math
from dataclasses import dataclass
from enum import Enum
from typing import Iterable, Sequence

from PyQt5 import QtCore, QtGui, QtWidgets

from Workstation.workstation_core import theme


class AnnotationMode(str, Enum):
    """Supported annotations on one immutable native frame."""

    HORIZON = "horizon"
    RIDGE = "ridge"
    TARGET = "target"


@dataclass(frozen=True)
class SourcePixel:
    """OpenCV image pixel center, in native ``(u-right, v-down)`` pixels."""

    u: float
    v: float

    def to_mapping(self) -> dict[str, float]:
        return {"u_px": self.u, "v_px": self.v}


@dataclass(frozen=True)
class FrozenAnnotationPayload:
    """Immutable snapshot of every annotation tied to one frame identity."""

    frame_id: str
    image_sha256: str
    source_width_px: int
    source_height_px: int
    horizon_line: tuple[SourcePixel, ...]
    ridge_polyline: tuple[SourcePixel, ...]
    target_point: tuple[SourcePixel, ...]
    schema: str = "frozen-native-annotations-v1"
    pixel_contract: str = "OpenCV native image pixel centers: u right, v down"

    def to_mapping(self) -> dict[str, object]:
        """Return a fresh JSON-compatible copy; mutating it cannot alter us."""

        return {
            "schema": self.schema,
            "frame_id": self.frame_id,
            "image_sha256": self.image_sha256,
            "source_width_px": self.source_width_px,
            "source_height_px": self.source_height_px,
            "pixel_contract": self.pixel_contract,
            "horizon_line": [point.to_mapping() for point in self.horizon_line],
            "ridge_polyline": [point.to_mapping() for point in self.ridge_polyline],
            "target_point": [point.to_mapping() for point in self.target_point],
        }


def qimage_pixel_sha256(image: QtGui.QImage) -> str:
    """Hash decoded native pixels deterministically when no file hash exists.

    The hash includes dimensions and a tightly packed RGBA8888 raster.  A PNG
    artifact's byte hash may instead be supplied by the evidence store; both
    are valid stable identities as long as the producer records which one it
    uses.
    """

    if image.isNull():
        raise ValueError("cannot hash a null image")
    packed = image.convertToFormat(QtGui.QImage.Format_RGBA8888)
    byte_count = packed.bytesPerLine() * packed.height()
    bits = packed.constBits()
    bits.setsize(byte_count)
    digest = hashlib.sha256()
    digest.update(f"{packed.width()}x{packed.height()}:RGBA8888\n".encode("ascii"))
    for row in range(packed.height()):
        offset = row * packed.bytesPerLine()
        digest.update(bytes(bits[offset : offset + packed.width() * 4]))
    return digest.hexdigest()


def _coerce_point(point: SourcePixel | QtCore.QPointF | QtCore.QPoint | Sequence[float]) -> SourcePixel:
    if isinstance(point, SourcePixel):
        result = point
    elif isinstance(point, (QtCore.QPointF, QtCore.QPoint)):
        result = SourcePixel(float(point.x()), float(point.y()))
    else:
        if len(point) != 2:
            raise ValueError("a source point must contain exactly (u, v)")
        result = SourcePixel(float(point[0]), float(point[1]))
    if not (math.isfinite(result.u) and math.isfinite(result.v)):
        raise ValueError("source pixel coordinates must be finite")
    return result


def _parse_mode(mode: AnnotationMode | str) -> AnnotationMode:
    if isinstance(mode, AnnotationMode):
        return mode
    return AnnotationMode(str(mode).strip().lower())


def _confidence_color(confidence: float) -> QtGui.QColor:
    """Map a [0, 1] match-quality score onto the app's neutral/red palette.

    Stays within the existing gray-to-red contract (no new semantic hue):
    a weak match (0) reads as ``theme.WARNING`` red, a strong one (1) fades
    to the neutral ``theme.SERIES_Z`` gray already used for suggestions.
    """

    weak = QtGui.QColor(theme.WARNING)
    strong = QtGui.QColor(theme.SERIES_Z)
    t = min(1.0, max(0.0, float(confidence)))
    return QtGui.QColor(
        round(weak.red() + (strong.red() - weak.red()) * t),
        round(weak.green() + (strong.green() - weak.green()) * t),
        round(weak.blue() + (strong.blue() - weak.blue()) * t),
    )


class AnnotationImageView(QtWidgets.QWidget):
    """16:9 viewer/editor for a *frozen native* camera frame.

    ``load_frozen_native_frame`` copies the input image, so later producer-side
    mutations cannot change the displayed evidence.  Loading a different
    ``(frame_id, image_sha256)`` clears annotations by default.  The
    multi-view workflow may pass ``preserve_annotations=True``: old points
    remain visible, but their exported identity stays pinned to the previous
    frame until the operator explicitly clears them.  Loading the same identity
    preserves points, including across arbitrary widget resizes.  The display
    viewport is intentionally 16:9 and uses a center crop (never a horizontal
    letterbox) so a camera reference fills its column.  The source image is
    never cropped in storage: source/widget conversion accounts for the crop
    transform and all saved points remain native OpenCV ``(u, v)`` pixels.
    """

    DISPLAY_ASPECT_WIDTH = 16
    DISPLAY_ASPECT_HEIGHT = 9
    DISPLAY_ASPECT_RATIO = DISPLAY_ASPECT_WIDTH / DISPLAY_ASPECT_HEIGHT
    HOVER_DOT_RADIUS_PX = 3.0
    HOVER_DOT_COLOR = QtGui.QColor("#ef233c")
    MIN_ZOOM_FACTOR = 1.0
    MAX_ZOOM_FACTOR = 8.0
    WHEEL_ZOOM_STEP = 1.2

    annotation_changed = QtCore.pyqtSignal(object)
    frame_changed = QtCore.pyqtSignal(object)
    mode_changed = QtCore.pyqtSignal(str)
    hover_changed = QtCore.pyqtSignal(object)
    zoom_changed = QtCore.pyqtSignal(float)
    suggestion_point_right_clicked = QtCore.pyqtSignal(str, int)

    _COLORS = {
        AnnotationMode.HORIZON: QtGui.QColor(theme.WARNING),
        AnnotationMode.RIDGE: QtGui.QColor(theme.SERIES_RIGHT),
        AnnotationMode.TARGET: QtGui.QColor(theme.ACCENT),
    }
    # Automatic suggestions are deliberately visually distinct from the
    # operator's committed annotation colors.  They live in a separate buffer
    # and are never included by ``export_annotations`` until accepted by the
    # owning pane.
    _SUGGESTION_COLORS = {
        # Keep the Workstation's neutral/red palette contract.  The dashed
        # stroke and the lighter/darker neutral values distinguish suggestions
        # from committed annotations without introducing a new semantic hue.
        AnnotationMode.HORIZON: QtGui.QColor(theme.SERIES_RIGHT),
        AnnotationMode.RIDGE: QtGui.QColor(theme.SERIES_Z),
        AnnotationMode.TARGET: QtGui.QColor(theme.ACCENT_HOVER),
    }

    def __init__(
        self,
        parent: QtWidgets.QWidget | None = None,
        *,
        wheel_zoom_requires_control: bool = True,
    ) -> None:
        super().__init__(parent)
        self.setObjectName("annotationImageView")
        self.setMinimumSize(240, 135)
        size_policy = QtWidgets.QSizePolicy(
            QtWidgets.QSizePolicy.Expanding,
            QtWidgets.QSizePolicy.Fixed,
        )
        size_policy.setHeightForWidth(True)
        self.setSizePolicy(size_policy)
        self.setMouseTracking(True)
        self.setFocusPolicy(QtCore.Qt.StrongFocus)
        self._image: QtGui.QImage | None = None
        self._frame_id: str | None = None
        self._image_sha256: str | None = None
        self._annotation_frame_id: str | None = None
        self._annotation_image_sha256: str | None = None
        self._annotation_source_width_px: int | None = None
        self._annotation_source_height_px: int | None = None
        self._mode = AnnotationMode.HORIZON
        self._points: dict[AnnotationMode, list[SourcePixel]] = {
            mode: [] for mode in AnnotationMode
        }
        self._suggestion_points: dict[AnnotationMode, list[SourcePixel]] = {
            mode: [] for mode in AnnotationMode
        }
        # Optional per-point match-quality score in [0, 1], index-aligned
        # with ``_suggestion_points``.  Empty for modes/suggestions with no
        # natural per-point score (e.g. Horizon's single fitted line).
        self._suggestion_confidences: dict[AnnotationMode, list[float]] = {
            mode: [] for mode in AnnotationMode
        }
        self._suggestion_polyline_visible: dict[AnnotationMode, bool] = {
            mode: True for mode in AnnotationMode
        }
        self._hover_source_pixel: SourcePixel | None = None
        self._zoom_factor = self.MIN_ZOOM_FACTOR
        self._view_center_source: SourcePixel | None = None
        self._wheel_zoom_requires_control = bool(wheel_zoom_requires_control)

    def hasHeightForWidth(self) -> bool:  # noqa: N802 - Qt override.
        """Tell parent layouts that this camera viewport is fixed at 16:9."""

        return True

    def heightForWidth(self, width: int) -> int:  # noqa: N802 - Qt override.
        width = max(0, int(width))
        return int(round(width * self.DISPLAY_ASPECT_HEIGHT / self.DISPLAY_ASPECT_WIDTH))

    def sizeHint(self) -> QtCore.QSize:  # noqa: N802 - Qt override.
        return QtCore.QSize(640, self.heightForWidth(640))

    def minimumSizeHint(self) -> QtCore.QSize:  # noqa: N802 - Qt override.
        return QtCore.QSize(240, self.heightForWidth(240))

    def resizeEvent(self, event: QtGui.QResizeEvent) -> None:  # noqa: N802 - Qt API.
        """Keep the actual widget geometry at the declared 16:9 ratio.

        A normal layout honors ``heightForWidth``.  The explicit minimum-height
        refresh also covers nested group boxes that otherwise compress a fixed
        child when the whole Alignment page is shorter than its scrollable
        content; the page then grows/scrolls instead of distorting the camera
        viewport.
        """

        desired_height = self.heightForWidth(self.width())
        if self.minimumHeight() != desired_height:
            self.setMinimumHeight(desired_height)
        super().resizeEvent(event)

    @property
    def frame_id(self) -> str | None:
        return self._frame_id

    @property
    def image_sha256(self) -> str | None:
        return self._image_sha256

    @property
    def annotations_stale(self) -> bool:
        """Whether visible points remain bound to a previous frozen frame."""

        return bool(
            self._annotation_frame_id
            and self._annotation_image_sha256
            and (self._annotation_frame_id, self._annotation_image_sha256)
            != (self._frame_id, self._image_sha256)
            and any(self._points.values())
        )

    @property
    def annotation_mode(self) -> AnnotationMode:
        return self._mode

    @property
    def source_size(self) -> QtCore.QSize:
        if self._image is None:
            return QtCore.QSize()
        return self._image.size()

    @property
    def hover_source_pixel(self) -> SourcePixel | None:
        """The native pixel currently under the pointer, if visible."""

        return self._hover_source_pixel

    @property
    def zoom_factor(self) -> float:
        """Current display magnification; native annotations are unaffected."""

        return self._zoom_factor

    @property
    def wheel_zoom_requires_control(self) -> bool:
        """Whether wheel zoom is gated behind the Control modifier."""

        return self._wheel_zoom_requires_control

    def has_frozen_frame(self) -> bool:
        return self._image is not None and not self._image.isNull()

    def frozen_image_copy(self) -> QtGui.QImage | None:
        return None if self._image is None else self._image.copy()

    def load_frozen_native_frame(
        self,
        image: QtGui.QImage,
        *,
        frame_id: str,
        image_sha256: str | None = None,
        metadata: dict[str, object] | None = None,
        preserve_annotations: bool = False,
    ) -> None:
        """Load a copied native image, optionally retaining old visible points.

        Retained points remain bound to their original frame identity. They are
        display-only until ``clear_annotations`` is called, so a new frame
        cannot silently reinterpret old pixel coordinates.
        """

        if not isinstance(image, QtGui.QImage) or image.isNull():
            raise ValueError("frozen native frame must be a non-null QImage")
        identifier = str(frame_id).strip()
        if not identifier:
            raise ValueError("frame_id is required for frozen evidence")
        metadata_value = dict(metadata or {})
        if metadata_value.get("overlay_applied") not in (None, False):
            raise ValueError("a frame with display overlays is not native evidence")
        native_width = int(metadata_value.get("native_width", image.width()))
        native_height = int(metadata_value.get("native_height", image.height()))
        if (native_width, native_height) != (image.width(), image.height()):
            raise ValueError("native frame dimensions do not match metadata")
        digest = str(image_sha256 or qimage_pixel_sha256(image)).strip().lower()
        if not digest:
            raise ValueError("image_sha256 is required for frozen evidence")

        identity_changed = (identifier, digest) != (self._frame_id, self._image_sha256)
        if not identity_changed and self._image is not None and image.size() != self._image.size():
            raise ValueError("one frozen frame identity cannot have two native dimensions")
        retain_points = bool(
            preserve_annotations and identity_changed and any(self._points.values())
        )
        if identity_changed and not retain_points:
            self._clear_points(emit=False)
            self._clear_suggestions()
            self._set_hover_source_pixel(None)
            self._reset_view(emit=False)
        elif identity_changed:
            # Automatic suggestions are also frame-pinned and never carry over.
            self._clear_suggestions()
            self._set_hover_source_pixel(None)
            self._reset_view(emit=False)

        # QImage is implicitly shared.  Conversion followed by copy guarantees
        # this widget owns a detached snapshot, never the producer's live buffer.
        self._image = image.convertToFormat(QtGui.QImage.Format_RGBA8888).copy()
        self._image.setDevicePixelRatio(1.0)
        self._frame_id = identifier
        self._image_sha256 = digest
        if not retain_points:
            self._bind_annotations_to_current_frame(
                identifier,
                digest,
                self._image.width(),
                self._image.height(),
            )
        self.update()
        self.frame_changed.emit(
            {
                "frame_id": identifier,
                "image_sha256": digest,
                "source_width_px": self._image.width(),
                "source_height_px": self._image.height(),
            }
        )
        if identity_changed:
            self.annotation_changed.emit(self.export_annotations())

    # Short alias for integration code; it retains the frozen/native contract.
    set_frozen_frame = load_frozen_native_frame

    def clear_frame(self) -> None:
        self._image = None
        self._frame_id = None
        self._image_sha256 = None
        self._annotation_frame_id = None
        self._annotation_image_sha256 = None
        self._annotation_source_width_px = None
        self._annotation_source_height_px = None
        self._clear_points(emit=False)
        self._clear_suggestions()
        self._set_hover_source_pixel(None)
        self._reset_view(emit=False)
        self.update()
        self.frame_changed.emit(None)
        self.annotation_changed.emit(None)

    def reset_zoom(self) -> None:
        """Return to the full-frame view without changing native annotations."""

        self._reset_view(emit=True)
        self.update()

    def _reset_view(self, *, emit: bool) -> None:
        changed = self._zoom_factor != self.MIN_ZOOM_FACTOR or self._view_center_source is not None
        self._zoom_factor = self.MIN_ZOOM_FACTOR
        self._view_center_source = None
        if emit and changed:
            self.zoom_changed.emit(self._zoom_factor)

    def set_annotation_mode(self, mode: AnnotationMode | str) -> None:
        parsed = _parse_mode(mode)
        if parsed == self._mode:
            return
        self._mode = parsed
        self.mode_changed.emit(parsed.value)
        self.update()

    set_mode = set_annotation_mode

    def annotation_points(self, mode: AnnotationMode | str) -> tuple[SourcePixel, ...]:
        return tuple(self._points[_parse_mode(mode)])

    def suggestion_points(self, mode: AnnotationMode | str) -> tuple[SourcePixel, ...]:
        """Return non-committed automatic points currently drawn on the view."""

        return tuple(self._suggestion_points[_parse_mode(mode)])

    def set_suggestion_points(
        self,
        points: Iterable[SourcePixel | QtCore.QPointF | QtCore.QPoint | Sequence[float]],
        *,
        mode: AnnotationMode | str,
        confidences: Sequence[float] | None = None,
    ) -> None:
        """Replace the separate automatic overlay without changing annotation data.

        ``confidences`` is an optional per-point match-quality score in
        ``[0, 1]``, index-aligned with ``points`` -- e.g. Mountain's
        descriptor-distance-derived score. Omit it (or pass an empty
        sequence) for suggestions with no natural per-point score, such as
        Horizon's single fitted line; the dots then fall back to the plain
        suggestion color.
        """

        parsed_mode = _parse_mode(mode)
        converted = [_coerce_point(point) for point in points]
        if self._image is not None and any(not self._source_contains(point) for point in converted):
            raise ValueError("suggestion point is outside the native source image")
        if confidences is not None and len(confidences) not in (0, len(converted)):
            raise ValueError("confidences must be empty or match the point count")
        self._suggestion_points[parsed_mode] = converted
        self._suggestion_confidences[parsed_mode] = (
            [min(1.0, max(0.0, float(value))) for value in confidences] if confidences else []
        )
        self.update()

    def clear_suggestion_points(self, mode: AnnotationMode | str | None = None) -> None:
        if mode is None:
            changed = any(self._suggestion_points.values())
            self._clear_suggestions()
        else:
            parsed_mode = _parse_mode(mode)
            changed = bool(self._suggestion_points[parsed_mode])
            self._suggestion_points[parsed_mode].clear()
            self._suggestion_confidences[parsed_mode].clear()
        if changed:
            self.update()

    def set_suggestion_polyline_visible(
        self,
        visible: bool,
        *,
        mode: AnnotationMode | str,
    ) -> None:
        """Control only the line joining automatic points for one mode."""

        parsed_mode = _parse_mode(mode)
        visible = bool(visible)
        if self._suggestion_polyline_visible[parsed_mode] == visible:
            return
        self._suggestion_polyline_visible[parsed_mode] = visible
        self.update()

    def suggestion_point_index_at(
        self,
        widget_point: QtCore.QPointF | QtCore.QPoint | Sequence[float],
        *,
        mode: AnnotationMode | str | None = None,
        radius_px: float = 10.0,
    ) -> int | None:
        """Return the nearest automatic point within a widget-space radius."""

        parsed_mode = self._mode if mode is None else _parse_mode(mode)
        if isinstance(widget_point, (QtCore.QPointF, QtCore.QPoint)):
            cursor = QtCore.QPointF(widget_point)
        else:
            if len(widget_point) != 2:
                raise ValueError("a widget point must contain exactly (x, y)")
            cursor = QtCore.QPointF(float(widget_point[0]), float(widget_point[1]))
        radius_squared = max(0.0, float(radius_px)) ** 2
        nearest: tuple[float, int] | None = None
        for index, point in enumerate(self._suggestion_points[parsed_mode]):
            if not self._source_contains(point):
                continue
            drawn = self.source_pixel_to_widget(point)
            distance_squared = (drawn.x() - cursor.x()) ** 2 + (drawn.y() - cursor.y()) ** 2
            if distance_squared <= radius_squared and (
                nearest is None or distance_squared < nearest[0]
            ):
                nearest = (distance_squared, index)
        return None if nearest is None else nearest[1]

    def replace_source_points(
        self,
        points: Iterable[SourcePixel | QtCore.QPointF | QtCore.QPoint | Sequence[float]],
        *,
        mode: AnnotationMode | str,
        emit: bool = True,
    ) -> None:
        """Set committed native points in one operation.

        This is used only after an explicit ``Use suggestion`` approval.  It
        keeps the automatic overlay and the exported manual annotation buffers
        separate, while avoiding an intermediate partially accepted line.
        """

        parsed_mode = _parse_mode(mode)
        converted = [_coerce_point(point) for point in points]
        if self._image is None:
            raise RuntimeError("load a frozen native frame before annotating")
        if self.annotations_stale:
            raise RuntimeError(
                "clear the previous annotation before replacing points on a new frozen frame"
            )
        if any(not self._source_contains(point) for point in converted):
            raise ValueError("annotation is outside the native source image")
        self._points[parsed_mode] = converted
        self.update()
        if emit:
            self.annotation_changed.emit(self.export_annotations())

    def add_source_point(
        self,
        point: SourcePixel | QtCore.QPointF | QtCore.QPoint | Sequence[float],
        *,
        mode: AnnotationMode | str | None = None,
    ) -> None:
        if self._image is None:
            raise RuntimeError("load a frozen native frame before annotating")
        if self.annotations_stale:
            raise RuntimeError(
                "clear the previous annotation before adding points to a new frozen frame"
            )
        parsed_mode = self._mode if mode is None else _parse_mode(mode)
        source = _coerce_point(point)
        if not self._source_contains(source):
            raise ValueError("annotation is outside the native source image")
        points = self._points[parsed_mode]
        if parsed_mode == AnnotationMode.TARGET:
            points[:] = [source]
        elif parsed_mode == AnnotationMode.HORIZON:
            # A third click begins a fresh two-point line.
            if len(points) >= 2:
                points.clear()
            points.append(source)
        else:
            points.append(source)
        self.update()
        self.annotation_changed.emit(self.export_annotations())

    def undo_last_point(self, mode: AnnotationMode | str | None = None) -> bool:
        parsed_mode = self._mode if mode is None else _parse_mode(mode)
        points = self._points[parsed_mode]
        if not points:
            return False
        points.pop()
        self.update()
        self.annotation_changed.emit(self.export_annotations())
        return True

    def clear_annotations(self, mode: AnnotationMode | str | None = None) -> None:
        if mode is None:
            changed = any(self._points.values())
            self._clear_points(emit=False)
        else:
            parsed_mode = _parse_mode(mode)
            changed = bool(self._points[parsed_mode])
            self._points[parsed_mode].clear()
        if not any(self._points.values()) and self._image is not None:
            # Rebind before emitting the cleared payload so the owner records
            # that the current frame is now ready for fresh points.
            self._bind_annotations_to_current_frame(
                self._frame_id,
                self._image_sha256,
                self._image.width(),
                self._image.height(),
            )
        if changed:
            self.update()
            self.annotation_changed.emit(self.export_annotations())
        if not any(self._points.values()) and self._image is not None:
            self.update()

    def export_annotations(self) -> FrozenAnnotationPayload | None:
        if (
            self._image is None
            or self._annotation_frame_id is None
            or self._annotation_image_sha256 is None
            or self._annotation_source_width_px is None
            or self._annotation_source_height_px is None
        ):
            return None
        return FrozenAnnotationPayload(
            frame_id=self._annotation_frame_id,
            image_sha256=self._annotation_image_sha256,
            source_width_px=self._annotation_source_width_px,
            source_height_px=self._annotation_source_height_px,
            horizon_line=tuple(self._points[AnnotationMode.HORIZON]),
            ridge_polyline=tuple(self._points[AnnotationMode.RIDGE]),
            target_point=tuple(self._points[AnnotationMode.TARGET]),
        )

    export_payload = export_annotations

    def image_display_rect(self) -> QtCore.QRectF:
        """Continuous center-crop target rectangle in widget coordinates.

        ``KeepAspectRatioByExpanding`` is expressed as a floating-point target
        rectangle instead of a pre-cropped pixmap.  This keeps the exact
        source-to-widget transform available for annotation input and for the
        hover marker.  With the normal 16:9 widget geometry the target fills
        the viewport exactly for 16:9 native frames; non-16:9 frames crop only
        the excess dimension and never create side bars.
        """

        if self._image is None or self._image.width() <= 0 or self._image.height() <= 0:
            return QtCore.QRectF()
        available_width = max(0.0, float(self.width()))
        available_height = max(0.0, float(self.height()))
        if available_width == 0.0 or available_height == 0.0:
            return QtCore.QRectF()
        # Expand rather than fit: the camera column must not acquire horizontal
        # empty margins.  A 4:3 native frame is therefore cropped at top/bottom
        # in a 16:9 viewport, while its original pixels remain fully stored.
        scale = max(available_width / self._image.width(), available_height / self._image.height())
        scale *= self._zoom_factor
        draw_width = self._image.width() * scale
        draw_height = self._image.height() * scale
        center = self._view_center_source or SourcePixel(
            (self._image.width() - 1) / 2.0,
            (self._image.height() - 1) / 2.0,
        )
        viewport_center = QtCore.QPointF(available_width / 2.0, available_height / 2.0)
        return QtCore.QRectF(
            viewport_center.x() - (center.u + 0.5) * scale,
            viewport_center.y() - (center.v + 0.5) * scale,
            draw_width,
            draw_height,
        )

    display_rect = image_display_rect

    def source_pixel_to_widget(
        self,
        point: SourcePixel | QtCore.QPointF | QtCore.QPoint | Sequence[float],
    ) -> QtCore.QPointF:
        """Map a native pixel *center* to its widget-space center."""

        if self._image is None:
            raise RuntimeError("no frozen native frame is loaded")
        source = _coerce_point(point)
        if not self._source_contains(source):
            raise ValueError("source point is outside the native image")
        target = self.image_display_rect()
        scale_x = target.width() / self._image.width()
        scale_y = target.height() / self._image.height()
        return QtCore.QPointF(
            target.left() + (source.u + 0.5) * scale_x,
            target.top() + (source.v + 0.5) * scale_y,
        )

    source_to_widget = source_pixel_to_widget

    def widget_to_source_pixel(
        self,
        point: QtCore.QPointF | QtCore.QPoint | Sequence[float],
        *,
        clamp_to_edge_pixel: bool = True,
    ) -> SourcePixel | None:
        """Map widget coordinates to a native pixel center.

        The normal 16:9 viewport has no letterbox bars.  ``None`` is returned
        only when no frame is loaded or a coordinate falls outside the rendered
        source rectangle (which can matter for callers using a custom widget
        geometry).
        """

        if self._image is None:
            return None
        if isinstance(point, (QtCore.QPointF, QtCore.QPoint)):
            widget_point = QtCore.QPointF(point)
        else:
            if len(point) != 2:
                raise ValueError("a widget point must contain exactly (x, y)")
            widget_point = QtCore.QPointF(float(point[0]), float(point[1]))
        if not (math.isfinite(widget_point.x()) and math.isfinite(widget_point.y())):
            raise ValueError("widget coordinates must be finite")
        target = self.image_display_rect()
        if target.isEmpty() or not target.contains(widget_point):
            return None
        scale_x = target.width() / self._image.width()
        scale_y = target.height() / self._image.height()
        u = (widget_point.x() - target.left()) / scale_x - 0.5
        v = (widget_point.y() - target.top()) / scale_y - 0.5
        if clamp_to_edge_pixel:
            u = min(max(u, 0.0), float(self._image.width() - 1))
            v = min(max(v, 0.0), float(self._image.height() - 1))
        elif not (0.0 <= u <= self._image.width() - 1 and 0.0 <= v <= self._image.height() - 1):
            return None
        return SourcePixel(u, v)

    widget_to_source = widget_to_source_pixel

    def wheelEvent(self, event: QtGui.QWheelEvent) -> None:  # noqa: N802 - Qt API.
        """Zoom around the pointer while keeping the pointed native pixel fixed.

        Views embedded in ordinary scrolling pages require Ctrl by default so
        plain wheel input reaches the enclosing QScrollArea. Alignment opts
        into plain-wheel zoom explicitly because its camera previews are the
        intended wheel interaction target.
        """

        control_pressed = bool(event.modifiers() & QtCore.Qt.ControlModifier)
        if self._image is None or (self._wheel_zoom_requires_control and not control_pressed):
            super().wheelEvent(event)
            return
        cursor = QtCore.QPointF(event.pos())
        target = self.image_display_rect()
        if target.isEmpty() or not target.contains(cursor):
            super().wheelEvent(event)
            return
        delta_y = float(event.angleDelta().y())
        if delta_y == 0.0:
            delta_y = float(event.pixelDelta().y())
        if delta_y == 0.0:
            super().wheelEvent(event)
            return
        source_before = self.widget_to_source_pixel(cursor, clamp_to_edge_pixel=True)
        if source_before is None:
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

        base_scale = max(
            float(self.width()) / self._image.width(),
            float(self.height()) / self._image.height(),
        )
        next_scale = base_scale * next_zoom
        if next_zoom <= self.MIN_ZOOM_FACTOR:
            # At the fully-zoomed-out level the display rect exactly fills
            # the viewport, so it must be centered on the true image center
            # or one edge overhangs while the other falls short, leaving an
            # empty gap. Snap back to that centered state here -- the same
            # one reset_zoom() sets -- instead of keeping whatever off-center
            # point the cursor last anchored, which is what let repeated
            # wheel zoom in/out drift the image to one side.
            self._view_center_source = None
        else:
            viewport_center = QtCore.QPointF(self.width() / 2.0, self.height() / 2.0)
            next_center = SourcePixel(
                source_before.u - (cursor.x() - viewport_center.x()) / next_scale,
                source_before.v - (cursor.y() - viewport_center.y()) / next_scale,
            )
            self._view_center_source = SourcePixel(
                min(max(next_center.u, 0.0), float(self._image.width() - 1)),
                min(max(next_center.v, 0.0), float(self._image.height() - 1)),
            )
        self._zoom_factor = next_zoom
        self.update()
        self.zoom_changed.emit(self._zoom_factor)
        event.accept()

    def mousePressEvent(self, event: QtGui.QMouseEvent) -> None:  # noqa: N802 - Qt API.
        if event.button() == QtCore.Qt.LeftButton:
            source = self.widget_to_source_pixel(event.localPos())
            if source is not None:
                try:
                    self.add_source_point(source)
                except RuntimeError:
                    # A retained multi-view mark is still pinned to the
                    # previous frame. Keep it visible and require the explicit
                    # Clear current annotation action before accepting new
                    # points on this frame.
                    event.accept()
                    return
                event.accept()
                return
        elif event.button() == QtCore.Qt.RightButton:
            suggestion_index = self.suggestion_point_index_at(event.localPos())
            if suggestion_index is not None:
                self.suggestion_point_right_clicked.emit(self._mode.value, suggestion_index)
                event.accept()
                return
            if self.undo_last_point():
                event.accept()
                return
        super().mousePressEvent(event)

    def mouseMoveEvent(self, event: QtGui.QMouseEvent) -> None:  # noqa: N802 - Qt API.
        """Track the exact native pixel under the cursor without adding it."""

        source = self.widget_to_source_pixel(event.localPos(), clamp_to_edge_pixel=False)
        if source is not None and self._source_contains(source):
            next_hover = source
        else:
            next_hover = None
        self._set_hover_source_pixel(next_hover)
        event.accept()

    def leaveEvent(self, event: QtCore.QEvent) -> None:  # noqa: N802 - Qt API.
        self._set_hover_source_pixel(None)
        super().leaveEvent(event)

    def keyPressEvent(self, event: QtGui.QKeyEvent) -> None:  # noqa: N802 - Qt API.
        if event.key() in (QtCore.Qt.Key_Backspace, QtCore.Qt.Key_Delete):
            if self.undo_last_point():
                event.accept()
                return
        elif event.key() == QtCore.Qt.Key_Escape:
            self.clear_annotations(self._mode)
            event.accept()
            return
        super().keyPressEvent(event)

    def paintEvent(self, event: QtGui.QPaintEvent) -> None:  # noqa: N802 - Qt API.
        del event
        painter = QtGui.QPainter(self)
        painter.fillRect(self.rect(), QtGui.QColor(theme.INVERSE_BG))
        if self._image is None:
            painter.setPen(QtGui.QColor(theme.INVERSE_TEXT_MUTED))
            painter.drawText(self.rect(), QtCore.Qt.AlignCenter, "NO FROZEN NATIVE FRAME")
            return
        painter.setRenderHint(QtGui.QPainter.SmoothPixmapTransform, True)
        painter.drawImage(self.image_display_rect(), self._image)
        painter.setRenderHint(QtGui.QPainter.Antialiasing, True)
        for mode in AnnotationMode:
            self._paint_suggestion(painter, mode, self._suggestion_points[mode])
        for mode in AnnotationMode:
            self._paint_annotation(painter, mode, self._points[mode])
        self._paint_hover_dot(painter)

    def _paint_suggestion(
        self,
        painter: QtGui.QPainter,
        mode: AnnotationMode,
        points: Iterable[SourcePixel],
    ) -> None:
        """Draw automatic points as dashed overlays, never as saved points."""

        confidences = self._suggestion_confidences.get(mode, [])
        visible = [
            (index, self.source_pixel_to_widget(point))
            for index, point in enumerate(points)
            if self._source_contains(point)
        ]
        if not visible:
            return
        base_color = self._SUGGESTION_COLORS[mode]
        show_polyline = (
            self._suggestion_polyline_visible[mode]
            and mode in (AnnotationMode.HORIZON, AnnotationMode.RIDGE)
            and len(visible) >= 2
        )
        if show_polyline:
            line_pen = QtGui.QPen(base_color, 2.0)
            line_pen.setCosmetic(True)
            line_pen.setStyle(QtCore.Qt.DashLine)
            painter.setPen(line_pen)
            painter.drawPolyline(QtGui.QPolygonF([widget_point for _, widget_point in visible]))
        show_index = mode in (AnnotationMode.HORIZON, AnnotationMode.RIDGE)
        for index, widget_point in visible:
            # A per-point score (currently only Mountain's descriptor-distance
            # confidence) colors the dot itself; suggestions without one keep
            # the plain suggestion color.
            dot_color = _confidence_color(confidences[index]) if index < len(confidences) else base_color
            dot_pen = QtGui.QPen(dot_color, 2.0)
            dot_pen.setCosmetic(True)
            painter.setPen(dot_pen)
            painter.setBrush(QtGui.QBrush(dot_color))
            painter.drawEllipse(widget_point, 4.5, 4.5)
            if show_index:
                self._draw_point_label(painter, widget_point, index + 1)

    def _paint_hover_dot(self, painter: QtGui.QPainter) -> None:
        """Draw a small red, source-centered hover marker over the image."""

        if self._hover_source_pixel is None or self._image is None:
            return
        widget_point = self.source_pixel_to_widget(self._hover_source_pixel)
        if not self.rect().contains(widget_point.toPoint()):
            return
        # A dark one-pixel halo keeps the red center visible on bright snow or
        # sky while the center itself remains a small, unambiguous red dot.
        painter.setPen(QtGui.QPen(QtGui.QColor(0, 0, 0, 210), 1.0))
        painter.setBrush(QtGui.QBrush(self.HOVER_DOT_COLOR))
        painter.drawEllipse(widget_point, self.HOVER_DOT_RADIUS_PX + 1.0, self.HOVER_DOT_RADIUS_PX + 1.0)
        painter.setPen(QtCore.Qt.NoPen)
        painter.setBrush(QtGui.QBrush(self.HOVER_DOT_COLOR))
        painter.drawEllipse(widget_point, self.HOVER_DOT_RADIUS_PX, self.HOVER_DOT_RADIUS_PX)

    def _paint_annotation(
        self,
        painter: QtGui.QPainter,
        mode: AnnotationMode,
        points: Iterable[SourcePixel],
    ) -> None:
        visible = [
            (index, self.source_pixel_to_widget(point))
            for index, point in enumerate(points)
            if self._source_contains(point)
        ]
        if not visible:
            return
        color = self._COLORS[mode]
        pen = QtGui.QPen(color, 2.0 if mode == self._mode else 1.5)
        pen.setCosmetic(True)
        painter.setPen(pen)
        painter.setBrush(QtGui.QBrush(color))
        show_polyline = mode in (AnnotationMode.HORIZON, AnnotationMode.RIDGE) and len(visible) >= 2
        if show_polyline:
            painter.drawPolyline(QtGui.QPolygonF([widget_point for _, widget_point in visible]))
        show_index = mode in (AnnotationMode.HORIZON, AnnotationMode.RIDGE)
        for index, widget_point in visible:
            painter.drawEllipse(widget_point, 3.5, 3.5)
            if show_index:
                self._draw_point_label(painter, widget_point, index + 1)
        if mode == AnnotationMode.TARGET:
            point = visible[0][1]
            painter.drawLine(QtCore.QPointF(point.x() - 8.0, point.y()), QtCore.QPointF(point.x() + 8.0, point.y()))
            painter.drawLine(QtCore.QPointF(point.x(), point.y() - 8.0), QtCore.QPointF(point.x(), point.y() + 8.0))

    def _draw_point_label(self, painter: QtGui.QPainter, point: QtCore.QPointF, number: int) -> None:
        """Draw a small 1-based index next to a point, offset up-right.

        Matching numbers on the left/right views are the operator's only cue
        that a Horizon/Ridge correspondence pair is in the right order --
        without it, a swapped click only shows up as a vaguely wrong polyline
        shape instead of pointing at exactly which pair is swapped.
        """

        text = str(number)
        text_point = QtCore.QPointF(point.x() + 7.0, point.y() - 7.0)
        # Same dark-halo trick as the hover dot: keeps the digit legible on
        # bright sky and dark terrain alike.
        painter.setPen(QtGui.QPen(QtGui.QColor(0, 0, 0, 210)))
        for dx, dy in ((-1.0, 0.0), (1.0, 0.0), (0.0, -1.0), (0.0, 1.0)):
            painter.drawText(text_point + QtCore.QPointF(dx, dy), text)
        painter.setPen(QtGui.QColor(theme.INVERSE_TEXT))
        painter.drawText(text_point, text)

    def _source_contains(self, point: SourcePixel) -> bool:
        return bool(
            self._image is not None
            and 0.0 <= point.u <= self._image.width() - 1
            and 0.0 <= point.v <= self._image.height() - 1
        )

    def _set_hover_source_pixel(self, point: SourcePixel | None) -> None:
        if point == self._hover_source_pixel:
            return
        self._hover_source_pixel = point
        self.hover_changed.emit(point)
        self.update()

    def _clear_points(self, *, emit: bool) -> None:
        for points in self._points.values():
            points.clear()
        if emit:
            self.annotation_changed.emit(self.export_annotations())

    def _bind_annotations_to_current_frame(
        self,
        frame_id: str | None,
        image_sha256: str | None,
        width_px: int,
        height_px: int,
    ) -> None:
        self._annotation_frame_id = None if frame_id is None else str(frame_id)
        self._annotation_image_sha256 = (
            None if image_sha256 is None else str(image_sha256)
        )
        self._annotation_source_width_px = int(width_px) if frame_id is not None else None
        self._annotation_source_height_px = int(height_px) if frame_id is not None else None

    def _clear_suggestions(self) -> None:
        for points in self._suggestion_points.values():
            points.clear()
        for confidences in self._suggestion_confidences.values():
            confidences.clear()


__all__ = [
    "AnnotationImageView",
    "AnnotationMode",
    "FrozenAnnotationPayload",
    "SourcePixel",
    "qimage_pixel_sha256",
]
