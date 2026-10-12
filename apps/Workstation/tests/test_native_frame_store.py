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


def test_native_frame_store_can_share_timestamped_ledger_path(tmp_path: Path) -> None:
    root = tmp_path / "session"
    ledger = root / "native_frames_20260903_120000_000000001+0900.jsonl"
    artifact = NativeFrameStore(root, events_path=ledger).persist(
        side="left",
        purpose="measurement",
        frame_id="shared-ledger",
        image=_image(8, 6),
        metadata={"native_width": 8, "native_height": 6},
    )

    assert ledger.is_file()
    assert artifact.relative_path == "frames/measurement/shared-ledger-left.png"
    assert not (root / "native_frames.jsonl").exists()


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
    import threading

    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    original_persist = NativeFrameStore.persist
    writer_entered = threading.Event()
    release_writer = threading.Event()
    gui_progress = threading.Event()
    writer_threads: list[int] = []

    def delayed_persist(self, **kwargs):
        writer_threads.append(threading.get_ident())
        writer_entered.set()
        if not release_writer.wait(2.0):
            raise RuntimeError("GUI timer did not release the blocked native writer")
        return original_persist(self, **kwargs)

    monkeypatch.setattr(NativeFrameStore, "persist", delayed_persist)
    store = AsyncNativeFrameStore(tmp_path / "session")
    completed: list[dict[str, object]] = []
    store.completed.connect(completed.append)
    ticks: list[float] = []
    callback_threads: list[object] = []
    timer = QtCore.QTimer()
    timer.setInterval(10)

    def observe_gui_progress():
        # Count only GUI events delivered while the actual writer is blocked.
        # Qt may coalesce timer expirations under load, so a fixed 200 ms sleep
        # cannot prove how many callbacks the GUI had an opportunity to run.
        if writer_entered.is_set() and not release_writer.is_set():
            callback_threads.append(QtCore.QThread.currentThread())
            ticks.append(time.monotonic())
            if len(ticks) >= 5:
                gui_progress.set()
                release_writer.set()

    timer.timeout.connect(observe_gui_progress)
    timer.start()
    try:
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

        assert writer_entered.is_set()
        assert gui_progress.is_set()
        assert len(ticks) >= 5
        assert all(thread is app.thread() for thread in callback_threads)
        assert writer_threads == [store._writer_thread.ident]
        assert writer_threads[0] != threading.get_ident()
        assert completed
        assert completed[0]["fields"]["evidence_state"] == "SAVED"
        assert completed[0]["fields"]["native_pair_status"] == "complete"
        saved_path = tmp_path / "session" / completed[0]["fields"]["native_left_png_path"]
        assert saved_path.is_file()
    finally:
        timer.stop()
        # Failed assertions must not strand a blocked writer or a queued Qt
        # completion in the next test of the full regression process.
        release_writer.set()
        store.acknowledge("request-1")
        assert store.close() is True
        store._writer_thread.join(timeout=2.0)
        assert not store._writer_thread.is_alive()
        app.processEvents()
        store.deleteLater()


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
