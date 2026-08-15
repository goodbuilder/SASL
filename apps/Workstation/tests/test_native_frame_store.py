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

