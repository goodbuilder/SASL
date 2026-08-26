from __future__ import annotations

import json
import re
import time
from pathlib import Path

import pytest
from PyQt5 import QtCore, QtGui, QtWidgets

from Workstation.workstation_core.native_frame_store import (
    AsyncNativeFrameStore,
    NativeFrameStore,
)


def _image(width: int, height: int) -> QtGui.QImage:
    image = QtGui.QImage(width, height, QtGui.QImage.Format_RGB888)
    image.fill(QtGui.QColor(11, 22, 33))
    return image


def test_native_frame_is_lossless_png_with_native_dimensions_and_frame_id(tmp_path: Path) -> None:
    store = NativeFrameStore(tmp_path / "session")
    artifact = store.persist(
        side="left",
        purpose="pose-reference",
        frame_id="burst-004",
        image=_image(1920, 1080),
        metadata={
            "timestamp_s": 10.0,
            "source_timestamp_s": 9.5,
            "timestamp_reference": "rtcp-ntp",
            "native_width": 1920,
            "native_height": 1080,
            "overlay_applied": False,
        },
    )

    output = tmp_path / "session" / artifact.relative_path
    decoded = QtGui.QImage(str(output))
    assert (decoded.width(), decoded.height()) == (1920, 1080)
    assert artifact.frame_id == "burst-004"
    assert artifact.sha256
    row = json.loads((tmp_path / "session" / "native_frames.jsonl").read_text().strip())
    assert row["pixel_contract"].startswith("decoded native RGB")


def test_dimension_mismatch_and_overlay_are_rejected(tmp_path: Path) -> None:
    store = NativeFrameStore(tmp_path / "session")
    image = _image(960, 540)
    with pytest.raises(ValueError, match="dimensions"):
        store.persist(
            side="right",
            purpose="measurement",
            image=image,
            metadata={"native_width": 1920, "native_height": 1080},
        )
    with pytest.raises(ValueError, match="annotated"):
        store.persist(
            side="right",
            purpose="measurement",
            image=image,
            metadata={"native_width": 960, "native_height": 540, "overlay_applied": True},
        )


def test_missing_timestamp_reference_remains_unknown(tmp_path: Path) -> None:
    artifact = NativeFrameStore(tmp_path / "session").persist(
        side="left",
        purpose="imported-reference",
        image=_image(32, 24),
        metadata={
            "timestamp_s": 1_787_000_000.0,
            "native_width": 32,
            "native_height": 24,
            "overlay_applied": False,
        },
    )

    assert artifact.timestamp_reference == "unknown"


def test_native_frame_ledger_recovers_only_a_torn_final_record(tmp_path: Path) -> None:
    root = tmp_path / "session"
    root.mkdir()
    ledger = root / "native_frames.jsonl"
    prefix = b'{"frame_id":"valid"}\n'
    ledger.write_bytes(prefix + b'{"frame_id":')

    store = NativeFrameStore(root)

    assert ledger.read_bytes() == prefix
    assert store.ledger_recovery.quarantined_tail is not None


def test_nonfinite_native_timestamp_is_persisted_as_unknown_value(tmp_path: Path) -> None:
    artifact = NativeFrameStore(tmp_path / "session").persist(
        side="left",
        purpose="sync-rejection",
        image=_image(8, 6),
        metadata={
            "timestamp_s": float("nan"),
            "timestamp_reference": "app-receive",
            "native_width": 8,
            "native_height": 6,
            "overlay_applied": False,
        },
    )

    assert artifact.timestamp_s is None
    row = json.loads((tmp_path / "session" / "native_frames.jsonl").read_text())
    assert row["timestamp_s"] is None


def test_async_native_store_does_not_block_gui_timer(monkeypatch, tmp_path: Path) -> None:
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    original_persist = NativeFrameStore.persist

    def delayed_persist(self, **kwargs):
        time.sleep(0.2)
        return original_persist(self, **kwargs)

    monkeypatch.setattr(NativeFrameStore, "persist", delayed_persist)
    store = AsyncNativeFrameStore(tmp_path / "session")
    completed: list[dict[str, object]] = []
    store.completed.connect(completed.append)
    ticks: list[float] = []
    timer = QtCore.QTimer()
    timer.setInterval(10)
    timer.timeout.connect(lambda: ticks.append(time.monotonic()))
    timer.start()

    enqueue_started_s = time.monotonic()
    scheduled = store.enqueue_pair(
        request_id="request-1",
        pair_id="pair-1",
        purpose="measurement",
        frames=(
            (
                "left",
                _image(64, 48),
                {
                    "native_width": 64,
                    "native_height": 48,
                    "overlay_applied": False,
                },
            ),
        ),
    )
    assert time.monotonic() - enqueue_started_s < 0.05
    assert scheduled is not None and scheduled["native_pair_status"] == "pending"

    deadline_s = time.monotonic() + 2.0
    while not completed and time.monotonic() < deadline_s:
        app.processEvents()
        time.sleep(0.005)
    timer.stop()

    assert completed
    assert len(ticks) >= 5
    store.acknowledge("request-1")
    assert store.close() is True


def test_sessionless_async_native_paths_use_creation_timestamp(tmp_path: Path) -> None:
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    store = AsyncNativeFrameStore(
        tmp_path / "measurement_data",
        timestamp_filenames=True,
    )
    completed: list[dict[str, object]] = []
    store.completed.connect(completed.append)

    scheduled = store.enqueue_pair(
        request_id="request-time",
        pair_id="pair-1",
        purpose="measurement",
        frames=(
            (
                "left",
                _image(16, 12),
                {"native_width": 16, "native_height": 12},
            ),
        ),
    )
    assert scheduled is not None
    name = Path(str(scheduled["native_left_png_path"])).name
    assert re.fullmatch(
        r"\d{8}_\d{6}_\d{9}[+-]\d{4}_pair-1-left\.png",
        name,
    )

    deadline_s = time.monotonic() + 2.0
    while not completed and time.monotonic() < deadline_s:
        app.processEvents()
        time.sleep(0.005)
    assert completed
    persisted = completed[0]["fields"]["native_left_png_path"]
    assert persisted == scheduled["native_left_png_path"]
    assert (tmp_path / "measurement_data" / str(persisted)).is_file()
    store.acknowledge("request-time")
    assert store.close() is True


def test_native_frame_refuses_to_replace_existing_file(tmp_path: Path) -> None:
    store = NativeFrameStore(tmp_path / "measurement_data")
    kwargs = {
        "side": "left",
        "purpose": "measurement",
        "frame_id": "same-frame",
        "image": _image(8, 6),
        "metadata": {"native_width": 8, "native_height": 6},
    }
    first = store.persist(**kwargs)
    first_path = tmp_path / "measurement_data" / first.relative_path
    original = first_path.read_bytes()

    with pytest.raises(OSError, match="refusing to replace"):
        store.persist(**kwargs)

    assert first_path.read_bytes() == original
