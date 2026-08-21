from __future__ import annotations

from PyQt5 import QtGui, QtWidgets

from Workstation.workstation_core.camera_pane import CameraPane


def _image(red: int) -> QtGui.QImage:
    image = QtGui.QImage(8, 6, QtGui.QImage.Format_RGB888)
    image.fill(QtGui.QColor(red, 0, 0))
    return image


def test_preview_inbox_replaces_only_stale_ui_frames_and_renders_latest() -> None:
    _app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    pane = CameraPane("left")
    pane.stream_generation = 7
    delivered: list[tuple[str, int]] = []
    pane.frame_changed.connect(
        lambda side, _image, metadata: delivered.append((side, int(metadata["sequence"])))
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

    assert delivered == [("left", 3)]
    assert pane.current_image().pixelColor(0, 0).red() == 30
    after = pane.preview_delivery_snapshot()
    assert after["submitted"] == 3
    assert after["rendered"] == 1
    assert after["replaced"] == 2
    assert after["queued_latest"] is False
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
    assert pane.preview_delivery_snapshot()["queued_latest"] is True
    pane.deleteLater()
