from __future__ import annotations

import json
from pathlib import Path

import pytest
from PyQt5 import QtGui

from Workstation.workstation_core.native_frame_store import NativeFrameStore


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
