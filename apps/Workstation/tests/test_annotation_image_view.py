from __future__ import annotations

from dataclasses import FrozenInstanceError

import pytest
from PyQt5 import QtCore, QtGui, QtTest, QtWidgets

from Workstation.annotation_image_view import (
    AnnotationImageView,
    AnnotationMode,
    FrozenAnnotationPayload,
    SourcePixel,
    qimage_pixel_sha256,
)


def _native_image(width: int, height: int, color: str = "#333333") -> QtGui.QImage:
    image = QtGui.QImage(width, height, QtGui.QImage.Format_RGB888)
    image.fill(QtGui.QColor(color))
    return image


@pytest.mark.parametrize(
    ("source_size", "widget_size"),
    [
        ((1920, 1080), (1000, 800)),  # center crop fills the viewport.
        ((1600, 1200), (1100, 600)),  # 4:3 source, top/bottom crop.
    ],
)
def test_native_pixel_roundtrip_at_top_left_center_and_bottom_right(
    source_size: tuple[int, int],
    widget_size: tuple[int, int],
) -> None:
    source_width, source_height = source_size
    view = AnnotationImageView()
    view.resize(*widget_size)
    view.load_frozen_native_frame(
        _native_image(*source_size),
        frame_id=f"frame-{source_width}x{source_height}",
        image_sha256="a" * 64,
    )

    points = (
        SourcePixel(0.0, 0.0),
        SourcePixel((source_width - 1) / 2.0, (source_height - 1) / 2.0),
        SourcePixel(float(source_width - 1), float(source_height - 1)),
    )
    for source in points:
        widget = view.source_pixel_to_widget(source)
        restored = view.widget_to_source_pixel(widget, clamp_to_edge_pixel=False)
        assert restored is not None
        assert restored.u == pytest.approx(source.u, abs=1e-9)
        assert restored.v == pytest.approx(source.v, abs=1e-9)

    display = view.image_display_rect()
    assert display.width() / display.height() == pytest.approx(source_width / source_height)
    # The expanding transform has no side bars: every viewport corner maps to
    # a visible native pixel (the non-16:9 source may be cropped vertically).
    corner = view.widget_to_source_pixel(QtCore.QPointF(0.0, 0.0))
    assert corner is not None
    assert 0.0 <= corner.u <= source_width - 1
    assert 0.0 <= corner.v <= source_height - 1


def test_resize_changes_only_display_transform_not_native_annotations() -> None:
    view = AnnotationImageView()
    view.resize(960, 540)
    view.set_frozen_frame(_native_image(1920, 1080), frame_id="burst-left", image_sha256="b" * 64)
    view.add_source_point((100.25, 200.75), mode=AnnotationMode.HORIZON)
    view.add_source_point((1800.5, 700.125), mode=AnnotationMode.HORIZON)
    before = view.export_annotations()
    before_widget = view.source_to_widget((100.25, 200.75))

    view.resize(700, 900)
    after = view.export_annotations()
    after_widget = view.source_to_widget((100.25, 200.75))

    assert after == before
    assert after_widget != before_widget
    restored = view.widget_to_source(after_widget, clamp_to_edge_pixel=False)
    assert restored is not None
    assert restored.u == pytest.approx(100.25)
    assert restored.v == pytest.approx(200.75)


def test_annotation_modes_have_expected_cardinality_and_order() -> None:
    view = AnnotationImageView()
    view.resize(640, 480)
    view.load_frozen_native_frame(_native_image(640, 480), frame_id="frame-modes", image_sha256="c" * 64)

    view.set_mode("horizon")
    view.add_source_point((10, 20))
    view.add_source_point((300, 25))
    assert view.annotation_points("horizon") == (SourcePixel(10, 20), SourcePixel(300, 25))
    view.add_source_point((40, 50))
    assert view.annotation_points("horizon") == (SourcePixel(40, 50),)

    view.set_mode("ridge")
    for point in ((5, 100), (80, 70), (150, 120)):
        view.add_source_point(point)
    assert view.annotation_points("ridge") == tuple(SourcePixel(*point) for point in ((5, 100), (80, 70), (150, 120)))

    view.set_mode("target")
    view.add_source_point((200, 210))
    view.add_source_point((220, 230))
    assert view.annotation_points("target") == (SourcePixel(220, 230),)


def test_mouse_input_uses_center_crop_transform_without_bars() -> None:
    view = AnnotationImageView()
    view.resize(800, 800)
    view.load_frozen_native_frame(_native_image(1600, 1200), frame_id="mouse", image_sha256="d" * 64)
    view.show()
    QtWidgets.QApplication.processEvents()

    display = view.image_display_rect()
    assert display.left() < 0.0
    assert display.top() == pytest.approx(0.0)

    expected = SourcePixel(799.5, 599.5)
    widget = view.source_pixel_to_widget(expected).toPoint()
    QtTest.QTest.mouseClick(view, QtCore.Qt.LeftButton, pos=widget)
    actual = view.annotation_points("horizon")[0]
    # QTest accepts integer widget positions, so at most half a source pixel is
    # lost during the intentional quantization at the UI boundary.
    assert actual.u == pytest.approx(expected.u, abs=1.0)
    assert actual.v == pytest.approx(expected.v, abs=1.0)

    view.close()


def test_camera_view_is_fixed_16_9_and_hover_reports_native_pixel_with_red_dot() -> None:
    view = AnnotationImageView()
    assert view.hasHeightForWidth()
    assert view.heightForWidth(640) == 360
    view.resize(640, view.heightForWidth(640))
    view.load_frozen_native_frame(
        _native_image(1280, 720, "#333333"),
        frame_id="hover",
        image_sha256="h" * 64,
    )
    view.show()
    QtWidgets.QApplication.processEvents()

    widget = view.source_pixel_to_widget(SourcePixel(639.5, 359.5)).toPoint()
    QtTest.QTest.mouseMove(view, widget)
    QtWidgets.QApplication.processEvents()
    assert view.hover_source_pixel is not None
    assert view.hover_source_pixel.u == pytest.approx(639.5, abs=1e-6)
    assert view.hover_source_pixel.v == pytest.approx(359.5, abs=1e-6)

    color = view.grab().toImage().pixelColor(widget)
    assert color.red() > 150
    assert color.red() > color.green() * 1.5
    assert color.red() > color.blue() * 1.5

    view.leaveEvent(QtCore.QEvent(QtCore.QEvent.Leave))
    assert view.hover_source_pixel is None
    view.close()


def test_frame_identity_pins_annotations_and_export_is_deeply_immutable() -> None:
    image = _native_image(320, 240)
    digest = qimage_pixel_sha256(image)
    view = AnnotationImageView()
    view.load_frozen_native_frame(image, frame_id="frozen-1", image_sha256=digest)
    view.add_source_point((1.5, 2.5), mode="target")

    first = view.export_payload()
    assert isinstance(first, FrozenAnnotationPayload)
    assert first.target_point == (SourcePixel(1.5, 2.5),)
    with pytest.raises(FrozenInstanceError):
        first.frame_id = "mutated"  # type: ignore[misc]
    with pytest.raises(FrozenInstanceError):
        first.target_point[0].u = 9.0  # type: ignore[misc]
    mutable_copy = first.to_mapping()
    mutable_copy["target_point"][0]["u_px"] = 99.0  # type: ignore[index]
    assert view.export_payload().target_point == (SourcePixel(1.5, 2.5),)

    # Repainting/reloading the same evidence identity preserves source points.
    view.load_frozen_native_frame(image, frame_id="frozen-1", image_sha256=digest)
    assert view.export_payload().target_point == (SourcePixel(1.5, 2.5),)

    # Either component changing identifies different evidence and clears them.
    view.load_frozen_native_frame(image, frame_id="frozen-1", image_sha256="e" * 64)
    assert view.export_payload().target_point == ()


def test_multiview_capture_can_keep_visible_points_until_explicit_clear() -> None:
    view = AnnotationImageView()
    first = _native_image(640, 480, "#333333")
    second = _native_image(640, 480, "#444444")
    view.load_frozen_native_frame(first, frame_id="burst-1", image_sha256="1" * 64)
    view.add_source_point((120.0, 140.0), mode=AnnotationMode.TARGET)

    view.load_frozen_native_frame(
        second,
        frame_id="burst-2",
        image_sha256="2" * 64,
        preserve_annotations=True,
    )

    assert view.annotation_points(AnnotationMode.TARGET) == (SourcePixel(120.0, 140.0),)
    assert view.annotations_stale is True
    assert view.export_annotations().frame_id == "burst-1"
    with pytest.raises(RuntimeError, match="clear the previous annotation"):
        view.add_source_point((200.0, 220.0), mode=AnnotationMode.TARGET)

    view.clear_annotations(AnnotationMode.TARGET)
    assert view.annotations_stale is False
    assert view.export_annotations().frame_id == "burst-2"
    view.add_source_point((200.0, 220.0), mode=AnnotationMode.TARGET)
    assert view.export_annotations().frame_id == "burst-2"


def test_rejects_overlay_or_non_native_dimensions() -> None:
    view = AnnotationImageView()
    image = _native_image(640, 480)

    with pytest.raises(ValueError, match="overlays"):
        view.load_frozen_native_frame(
            image,
            frame_id="bad-overlay",
            image_sha256="f" * 64,
            metadata={"overlay_applied": True},
        )
    with pytest.raises(ValueError, match="dimensions"):
        view.load_frozen_native_frame(
            image,
            frame_id="bad-size",
            image_sha256="f" * 64,
            metadata={"native_width": 320, "native_height": 240},
        )


class _WheelEvent:
    def __init__(self, position: QtCore.QPoint, delta_y: int) -> None:
        self._position = position
        self._delta_y = delta_y
        self.accepted = False

    def pos(self) -> QtCore.QPoint:
        return self._position

    def angleDelta(self) -> QtCore.QPoint:
        return QtCore.QPoint(0, self._delta_y)

    def pixelDelta(self) -> QtCore.QPoint:
        return QtCore.QPoint()

    def accept(self) -> None:
        self.accepted = True


def test_wheel_zoom_keeps_cursor_native_pixel_fixed() -> None:
    view = AnnotationImageView()
    view.resize(800, 600)
    view.load_frozen_native_frame(
        _native_image(1600, 1200),
        frame_id="zoom",
        image_sha256="z" * 64,
    )
    cursor = QtCore.QPoint(257, 183)
    before = view.widget_to_source_pixel(cursor, clamp_to_edge_pixel=False)
    assert before is not None
    view.set_mode(AnnotationMode.TARGET)
    view.add_source_point(before)

    event = _WheelEvent(cursor, 120)
    view.wheelEvent(event)  # type: ignore[arg-type]

    assert event.accepted
    assert view.zoom_factor == pytest.approx(1.2)
    after = view.widget_to_source_pixel(cursor, clamp_to_edge_pixel=False)
    assert after is not None
    assert after.u == pytest.approx(before.u, abs=1.0e-9)
    assert after.v == pytest.approx(before.v, abs=1.0e-9)
    assert view.export_annotations().target_point == (before,)


def test_wheel_zoom_out_to_min_recenters_with_no_gap() -> None:
    """Zooming in near one corner, then back out while the cursor has moved
    to a different point (the ordinary way a user actually zooms -- look at
    one feature, scroll in, move the mouse, scroll back out) must recenter
    the view once it reaches the fully-zoomed-out level, or the display rect
    stays off-center at a scale where it exactly fills the viewport --
    leaving an empty gap on one edge. Regression guard for that drift."""

    view = AnnotationImageView()
    view.resize(800, 600)
    view.load_frozen_native_frame(
        _native_image(1600, 1200),
        frame_id="zoom-recenter",
        image_sha256="r" * 64,
    )

    for _ in range(5):
        view.wheelEvent(_WheelEvent(QtCore.QPoint(40, 30), 120))  # type: ignore[arg-type]
    assert view.zoom_factor > 1.0

    for _ in range(5):
        view.wheelEvent(_WheelEvent(QtCore.QPoint(760, 570), -120))  # type: ignore[arg-type]

    assert view.zoom_factor == pytest.approx(1.0)
    rect = view.image_display_rect()
    assert rect.left() <= 1.0e-6
    assert rect.top() <= 1.0e-6
    assert rect.right() >= view.width() - 1.0e-6
    assert rect.bottom() >= view.height() - 1.0e-6
