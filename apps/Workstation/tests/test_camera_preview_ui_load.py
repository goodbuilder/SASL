from __future__ import annotations

from collections import deque
from types import SimpleNamespace

from PyQt5 import QtGui

from Workstation.standalone_gui import WirisStereoDistancePanel


class _SignalProbe:
    def __init__(self) -> None:
        self.payloads: list[object] = []

    def emit(self, payload: object) -> None:
        self.payloads.append(payload)


def _preview_panel(*, active: bool) -> SimpleNamespace:
    return SimpleNamespace(
        yolo_enabled=True,
        camera_frame_metadata={"left": {}, "right": {}},
        camera_frame_metadata_history={"left": deque(maxlen=8), "right": deque(maxlen=8)},
        camera_frame_times={"left": deque(maxlen=8), "right": deque(maxlen=8)},
        latest_calibration_preview_images={},
        calibration_preview_times={"left": 0.0, "right": 0.0},
        calibration_preview_changed=_SignalProbe(),
        _workstation_calibration_preview_active=active,
        _record_gui_camera_frame=lambda *_args, **_kwargs: None,
        _capture_recording_clock_sync_start_observation=lambda: None,
        _record_clock_offset_sample=lambda _now_s: None,
        _is_clock_sync_tab_visible=lambda: False,
        _is_checkerboard_calibration_visible=lambda: False,
        _refresh_calibration_preview=lambda camera: None,
    )


def test_raw_camera_frame_updates_latest_snapshot_without_duplicate_preview_signal() -> None:
    panel = _preview_panel(active=False)
    image = QtGui.QImage(64, 48, QtGui.QImage.Format_RGB888)
    image.fill(QtGui.QColor("blue"))

    WirisStereoDistancePanel._update_calibration_preview(
        panel,
        "left",
        image,
        {"timestamp_s": 12.5, "timestamp_source": "app-receive"},
    )

    cached = panel.latest_calibration_preview_images["left"]
    assert isinstance(cached, QtGui.QImage)
    assert cached.cacheKey() == image.cacheKey()
    assert panel.calibration_preview_changed.payloads == []
    assert panel.camera_frame_times["left"] == deque((12.5,), maxlen=8)


def test_active_checkerboard_preview_keeps_throttled_overlay_request_path() -> None:
    refreshed: list[str] = []
    panel = _preview_panel(active=True)
    panel._refresh_calibration_preview = lambda camera: refreshed.append(camera)
    image = QtGui.QImage(64, 48, QtGui.QImage.Format_RGB888)

    WirisStereoDistancePanel._update_calibration_preview(
        panel,
        "right",
        image,
        {"timestamp_s": 20.0},
    )

    assert refreshed == ["right"]
    assert panel.calibration_preview_changed.payloads == []
