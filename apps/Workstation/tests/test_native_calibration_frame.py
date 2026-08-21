from __future__ import annotations

from typing import Any

import pytest
from PyQt5 import QtGui

from Workstation.workstation_core.camera_pane import CameraPane


def _image(width: int, height: int, color: tuple[int, int, int]) -> QtGui.QImage:
    image = QtGui.QImage(width, height, QtGui.QImage.Format_RGB888)
    image.fill(QtGui.QColor(*color))
    return image


def _native_metadata(timestamp_s: float, frame_id: str = "decoded-native-001") -> dict[str, Any]:
    return {
        "timestamp_s": timestamp_s,
        "source_timestamp_s": 41.25,
        "timestamp_source": "gstreamer-pts",
        "timestamp_reference": "rtcp-ntp",
        "rtcp_frame_ntp_unix_s": 1_700_000_000.125,
        "native_width": 1920,
        "native_height": 1080,
        "native_lossless_source": True,
        "overlay_applied": False,
        "source_native_frame_id": frame_id,
        "capture": {"sequence": 7},
    }


def test_calibration_frame_uses_native_raster_when_preview_is_resized() -> None:
    pane = CameraPane("left")
    preview = _image(1280, 720, (200, 10, 10))
    native = _image(1920, 1080, (10, 20, 30))
    pane._show_frame(
        preview,
        {
            "timestamp_s": 100.0,
            "native_width": 1920,
            "native_height": 1080,
            "preview_width": 1280,
            "preview_height": 720,
        },
    )
    pane._emit_native_measurement_frame_if_current(
        pane.stream_generation,
        _native_metadata(100.0),
        native,
    )

    result = pane.latest_native_calibration_frame(now_monotonic_s=100.5)

    assert result is not None
    frame_id, calibration_image, metadata = result
    assert frame_id == "decoded-native-001"
    assert (pane.current_image().width(), pane.current_image().height()) == (1280, 720)
    assert (calibration_image.width(), calibration_image.height()) == (1920, 1080)
    assert calibration_image.pixelColor(0, 0) == QtGui.QColor(10, 20, 30)
    assert metadata["native_width"] == 1920
    assert metadata["native_height"] == 1080
    assert metadata["source_timestamp_s"] == 41.25
    assert metadata["rtcp_frame_ntp_unix_s"] == 1_700_000_000.125
    assert metadata["timestamp_reference"] == "rtcp-ntp"
    assert metadata["frame_id"] == frame_id
    assert "no resize" in str(metadata["pixel_contract"])


def test_calibration_frame_returns_defensive_image_and_metadata_copies() -> None:
    pane = CameraPane("right")
    source_metadata = _native_metadata(200.0)
    pane._emit_native_measurement_frame_if_current(
        pane.stream_generation,
        source_metadata,
        _image(1920, 1080, (4, 5, 6)),
    )
    source_metadata["capture"]["sequence"] = 999

    first = pane.latest_native_calibration_frame(now_monotonic_s=200.1)
    assert first is not None
    _frame_id, first_image, first_metadata = first
    first_image.fill(QtGui.QColor(250, 250, 250))
    first_metadata["capture"]["sequence"] = -1

    second = pane.latest_native_calibration_frame(now_monotonic_s=200.2)

    assert second is not None
    _frame_id, second_image, second_metadata = second
    assert second_image.pixelColor(0, 0) == QtGui.QColor(4, 5, 6)
    assert second_metadata["capture"]["sequence"] == 7


def test_calibration_frame_rejects_missing_stale_and_future_frames() -> None:
    pane = CameraPane("left")
    assert pane.latest_native_calibration_frame(now_monotonic_s=10.0) is None

    pane._emit_native_measurement_frame_if_current(
        pane.stream_generation,
        _native_metadata(10.0),
        _image(1920, 1080, (1, 2, 3)),
    )

    assert pane.latest_native_calibration_frame(
        maximum_age_s=2.0,
        now_monotonic_s=12.001,
    ) is None
    assert pane.latest_native_calibration_frame(
        maximum_age_s=2.0,
        now_monotonic_s=9.999,
    ) is None
    assert pane.latest_native_calibration_frame(
        maximum_age_s=2.0,
        now_monotonic_s=12.0,
    ) is not None


@pytest.mark.parametrize(
    "metadata_update",
    [
        {"native_width": 1280},
        {"overlay_applied": True},
        {"native_lossless_source": False},
        {"source_native_frame_id": ""},
    ],
)
def test_calibration_frame_rejects_unproven_native_contract(
    metadata_update: dict[str, object],
) -> None:
    pane = CameraPane("left")
    metadata = _native_metadata(300.0)
    metadata.update(metadata_update)
    pane._emit_native_measurement_frame_if_current(
        pane.stream_generation,
        metadata,
        _image(1920, 1080, (1, 2, 3)),
    )

    assert pane.latest_native_calibration_frame(now_monotonic_s=300.1) is None


def test_calibration_frame_near_selects_the_closest_native_timestamp() -> None:
    pane = CameraPane("right")
    for timestamp_s, frame_id, color in (
        (400.000, "frame-a", (10, 0, 0)),
        (400.020, "frame-b", (20, 0, 0)),
    ):
        pane._emit_native_measurement_frame_if_current(
            pane.stream_generation,
            _native_metadata(timestamp_s, frame_id),
            _image(1920, 1080, color),
        )

    result = pane.calibration_native_frame_near(
        400.018,
        maximum_delta_s=0.005,
        now_monotonic_s=400.1,
    )

    assert result is not None
    frame_id, image, metadata = result
    assert frame_id == "frame-b"
    assert image.pixelColor(0, 0) == QtGui.QColor(20, 0, 0)
    assert metadata["timestamp_s"] == 400.020
    assert pane.calibration_native_frame_near(
        400.010,
        maximum_delta_s=0.005,
        now_monotonic_s=400.1,
    ) is None
