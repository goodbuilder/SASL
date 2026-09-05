from __future__ import annotations

import multiprocessing

import pytest
from PyQt5 import QtGui, QtWidgets

from Workstation.workstation_core.camera_pane import CameraPane
from Workstation.workstation_core.native_frame_ring import SharedNativeFrameRing
from Workstation.workstation_core.stereo import DetectionObservation


def _image(red: int) -> QtGui.QImage:
    image = QtGui.QImage(8, 6, QtGui.QImage.Format_RGB888)
    image.fill(QtGui.QColor(red, 0, 0))
    return image


def test_preview_inbox_replaces_only_stale_ui_frames_and_renders_latest() -> None:
    _app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    pane = CameraPane("left")
    pane.stream_generation = 7
    delivered: list[tuple[str, int, int]] = []
    pane.frame_changed.connect(
        lambda side, _image, metadata: delivered.append(
            (side, int(metadata["sequence"]), int(metadata["stream_generation"]))
        )
    )

    pane._submit_preview_frame_for_generation(7, _image(10), {"sequence": 1})
    pane._submit_preview_frame_for_generation(7, _image(20), {"sequence": 2})
    pane._submit_preview_frame_for_generation(7, _image(30), {"sequence": 3})

    before = pane.preview_delivery_snapshot()
    assert before == {
        "submitted": 3,
        "rendered": 0,
        "replaced": 2,
        "queued_latest": True,
        "timer_interval_ms": 67,
    }
    assert pane.current_image() is None

    pane._drain_preview_frame()

    assert delivered == [("left", 3, 7)]
    assert pane.current_image().pixelColor(0, 0).red() == 30
    after = pane.preview_delivery_snapshot()
    assert after["submitted"] == 3
    assert after["rendered"] == 1
    assert after["replaced"] == 2
    assert after["queued_latest"] is False
    pane.deleteLater()


def test_preview_inbox_preserves_nal1_candidate_until_gui_drain() -> None:
    _app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    pane = CameraPane("left")
    pane.stream_generation = 7
    delivered: list[int] = []
    pane.frame_changed.connect(
        lambda _side, _image, metadata: delivered.append(int(metadata["sequence"]))
    )

    pane._submit_preview_frame_for_generation(
        7,
        _image(10),
        {"sequence": 1, "h264_nal1_non_idr_intra": True},
    )
    pane._submit_preview_frame_for_generation(
        7,
        _image(20),
        {"sequence": 2},
    )
    pane._drain_preview_frame()

    assert delivered == [1]
    assert pane.current_image().pixelColor(0, 0).red() == 10
    assert pane.preview_delivery_snapshot()["replaced"] == 1
    pane.deleteLater()


def test_hidden_camera_pane_retains_and_publishes_frame_without_pixmap_render() -> None:
    _app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    pane = CameraPane("left")
    pane.hide()
    delivered: list[int] = []
    pane.frame_changed.connect(
        lambda _side, _image, metadata: delivered.append(int(metadata["sequence"]))
    )

    pane._show_frame(_image(40), {"sequence": 4})

    assert delivered == [4]
    assert pane.current_image() is not None
    assert pane.image_label.pixmap() is None or pane.image_label.pixmap().isNull()
    pane.deleteLater()


def test_camera_preview_preserves_full_source_aspect_without_center_crop() -> None:
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    pane = CameraPane("left")
    pane.show()
    app.processEvents()
    pane.image_label.resize(200, 200)
    source = QtGui.QImage(160, 90, QtGui.QImage.Format_RGB888)
    source.fill(QtGui.QColor("blue"))

    pane._show_frame(source, {"sequence": 1})

    pixmap = pane.image_label.pixmap()
    assert pixmap is not None and not pixmap.isNull()
    assert pixmap.width() <= pane.image_label.width()
    assert pixmap.height() <= pane.image_label.height()
    assert pixmap.width() / pixmap.height() == pytest.approx(16.0 / 9.0, rel=0.02)
    assert (pixmap.width(), pixmap.height()) != (
        pane.image_label.width(),
        pane.image_label.height(),
    )
    pane.close()
    pane.deleteLater()


def test_preview_inbox_rejects_old_stream_generation() -> None:
    _app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    pane = CameraPane("left")
    pane.stream_generation = 9

    pane._submit_preview_frame_for_generation(8, _image(10), {"sequence": 1})

    assert pane.preview_delivery_snapshot()["submitted"] == 0
    assert pane.preview_delivery_snapshot()["queued_latest"] is False
    pane.deleteLater()


def test_preview_timer_is_capped_but_respects_lower_requested_rate() -> None:
    _app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    pane = CameraPane("left")

    pane._configure_preview_timer(60.0)
    assert pane.preview_delivery_snapshot()["timer_interval_ms"] == 67
    pane._configure_preview_timer(5.0)
    assert pane.preview_delivery_snapshot()["timer_interval_ms"] == 200
    pane._configure_preview_timer(0.0)
    assert pane.preview_delivery_snapshot()["timer_interval_ms"] == 67
    pane.deleteLater()


def test_native_measurement_frame_bypasses_preview_replacement() -> None:
    _app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    pane = CameraPane("left")
    pane.stream_generation = 3
    preview = _image(10)
    native = QtGui.QImage(16, 12, QtGui.QImage.Format_RGB888)
    native.fill(QtGui.QColor(90, 0, 0))
    metadata = {
        "timestamp_s": 12.5,
        "native_width": 16,
        "native_height": 12,
        "native_lossless_source": True,
        "overlay_applied": False,
        "source_native_frame_id": "native-1",
    }

    pane._submit_preview_frame_for_generation(3, preview, {"sequence": 1})
    pane._emit_native_measurement_frame_if_current(3, metadata, native)

    stored = pane.native_measurement_frame(12.5)
    assert stored is not None
    assert stored[0] == 12.5
    assert (stored[1].width(), stored[1].height()) == (16, 12)
    assert stored[2]["source_native_frame_id"] == "native-1"
    assert stored[2]["stream_generation"] == 3
    assert pane.preview_delivery_snapshot()["queued_latest"] is True
    pane.deleteLater()


def test_detection_metadata_carries_the_owning_stream_generation() -> None:
    _app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    pane = CameraPane("left")
    pane.stream_generation = 11
    delivered = []
    pane.detection_changed.connect(lambda _side, observation: delivered.append(observation))

    pane._handle_detection_payload_if_current(
        11,
        {
            "timestamp_s": 5.0,
            "width": 640,
            "height": 480,
            "best_detection": {
                "x1": 1.0,
                "y1": 2.0,
                "x2": 3.0,
                "y2": 4.0,
                "confidence": 0.9,
                "class_id": 1,
                "class_name": "target",
            },
            "detections": [{"class_id": 1}],
            "timestamp_metadata": {"rtp_ssrc": 101},
        },
    )

    assert len(delivered) == 1
    assert delivered[0].timestamp_metadata["stream_generation"] == 11
    pane.deleteLater()


def test_exact_frame_key_reads_native_ring_and_never_substitutes_overwritten_frame() -> None:
    _app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    pane = CameraPane("left")
    pane.stream_generation = 5
    ring = SharedNativeFrameRing.create(
        camera_id="left",
        stream_generation=5,
        capacity=1,
        max_width=2,
        max_height=1,
        channels=3,
        mp_context=multiprocessing.get_context("spawn"),
    )
    pane.native_frame_ring = ring
    try:
        first = ring.publish_bytes(
            bytes([255, 0, 0, 255, 0, 0]),
            width=2,
            height=1,
            channels=3,
            timestamp_s=1.0,
        )
        first_metadata = first.to_mapping()
        pane._remember_native_descriptor(first_metadata)
        detection = DetectionObservation.from_payload(
            "left",
            {
                "timestamp_s": 1.0,
                "width": 2,
                "height": 1,
                "best_detection": {
                    "x1": 0.0,
                    "y1": 0.0,
                    "x2": 1.0,
                    "y2": 1.0,
                    "confidence": 0.9,
                    "class_id": 0,
                    "class_name": "target",
                },
                "detections": [],
                "timestamp_metadata": first_metadata,
            },
        )
        resolved = pane.native_frame_for_detection(detection)
        assert resolved is not None
        image, metadata = resolved
        assert image.pixelColor(0, 0) == QtGui.QColor("blue")
        assert metadata["frame_key"] == "left:5:1"

        ring.publish_bytes(
            bytes([0, 0, 255, 0, 0, 255]),
            width=2,
            height=1,
            channels=3,
            timestamp_s=1.0001,
        )
        assert pane.native_frame_for_detection(detection) is None
    finally:
        pane.native_frame_ring = None
        ring.close()
        ring.unlink()
        pane.deleteLater()


def test_gui_heartbeat_records_a_two_second_event_loop_stall(monkeypatch) -> None:
    from Workstation.workstation_core import camera_pane

    _app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    pane = CameraPane("left")
    pane._gui_heartbeat_last_s = 10.0
    monkeypatch.setattr(camera_pane.time, "monotonic", lambda: 12.25)

    pane._record_gui_heartbeat()
    snapshot = pane.backpressure_snapshot()

    assert snapshot["gui_heartbeat_max_delay_s"] == pytest.approx(2.0)
    assert snapshot["gui_heartbeat_count"] == 1
    assert snapshot["workstation_rss_bytes"] > 0
    pane.deleteLater()
