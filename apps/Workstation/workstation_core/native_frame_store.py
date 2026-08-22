"""Lossless native PNG persistence for pose and accepted-measurement evidence."""

from __future__ import annotations

import hashlib
import math
import os
import time
import uuid
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Mapping

from PyQt5 import QtGui

from Workstation.workstation_core.strict_jsonl import (
    append_jsonl_record,
    prepare_jsonl_append,
)


@dataclass(frozen=True)
class NativeFrameArtifact:
    frame_id: str
    side: str
    purpose: str
    relative_path: str
    width: int
    height: int
    size_bytes: int
    sha256: str
    timestamp_s: float | None
    source_timestamp_s: float | None
    timestamp_reference: str
    source_native_frame_id: str = ""

    def to_mapping(self) -> dict[str, Any]:
        return {
            "schema": "native-frame-artifact-v1",
            **self.__dict__,
            "pixel_contract": "decoded native RGB pixels; lossless PNG; no overlay; no resize",
        }


def _safe_token(value: object, fallback: str) -> str:
    text = "".join(character if character.isalnum() or character in "._-" else "_" for character in str(value or ""))
    return text.strip("._") or fallback


class NativeFrameStore:
    """Synchronous atomic writer intended for a dedicated controller/worker."""

    def __init__(self, session_root: Path | str) -> None:
        self.root = Path(session_root).expanduser().resolve()
        self.root.mkdir(parents=True, exist_ok=True)
        self.events_path = self.root / "native_frames.jsonl"
        self.ledger_recovery = prepare_jsonl_append(self.events_path)

    def persist(
        self,
        *,
        side: str,
        purpose: str,
        image: QtGui.QImage,
        metadata: Mapping[str, Any],
        frame_id: str | None = None,
    ) -> NativeFrameArtifact:
        side_value = str(side).lower()
        if side_value not in {"left", "right"}:
            raise ValueError("side must be left or right")
        if image.isNull():
            raise ValueError("native image is null")
        if metadata.get("overlay_applied") not in (False, None):
            raise ValueError("annotated frames cannot be persisted as native evidence")
        expected_width = int(metadata.get("native_width", image.width()))
        expected_height = int(metadata.get("native_height", image.height()))
        if (image.width(), image.height()) != (expected_width, expected_height):
            raise ValueError("native frame dimensions do not match producer metadata")
        identifier = _safe_token(frame_id or uuid.uuid4(), "frame")
        purpose_value = _safe_token(purpose, "unspecified")
        output_dir = self.root / "frames" / purpose_value
        output_dir.mkdir(parents=True, exist_ok=True)
        destination = output_dir / f"{identifier}-{side_value}.png"
        temporary = destination.with_name(f".{destination.name}.{uuid.uuid4().hex}.part")
        try:
            if not image.save(str(temporary), "PNG", 100):
                raise IOError("Qt failed to encode native PNG")
            with temporary.open("rb") as handle:
                os.fsync(handle.fileno())
            os.replace(temporary, destination)
            descriptor = os.open(output_dir, os.O_RDONLY)
            try:
                os.fsync(descriptor)
            finally:
                os.close(descriptor)
        except Exception:
            try:
                temporary.unlink()
            except FileNotFoundError:
                pass
            raise
        payload = destination.read_bytes()
        decoded = QtGui.QImage(str(destination))
        if decoded.isNull() or (decoded.width(), decoded.height()) != (image.width(), image.height()):
            raise IOError("persisted native PNG failed decode/dimension verification")
        raw_timestamp = metadata.get("timestamp_s")
        if raw_timestamp is None:
            timestamp_s: float | None = time.monotonic()
        else:
            try:
                timestamp_s = float(raw_timestamp)
            except (TypeError, ValueError):
                timestamp_s = None
            if timestamp_s is not None and not math.isfinite(timestamp_s):
                timestamp_s = None
        raw_source_timestamp = metadata.get("source_timestamp_s")
        try:
            source_timestamp_s = (
                None
                if raw_source_timestamp is None
                else float(raw_source_timestamp)
            )
        except (TypeError, ValueError):
            source_timestamp_s = None
        if source_timestamp_s is not None and not math.isfinite(source_timestamp_s):
            source_timestamp_s = None
        timestamp_reference = str(metadata.get("timestamp_reference") or "unknown")
        record = NativeFrameArtifact(
            frame_id=identifier,
            side=side_value,
            purpose=purpose_value,
            relative_path=destination.relative_to(self.root).as_posix(),
            width=image.width(),
            height=image.height(),
            size_bytes=len(payload),
            sha256=hashlib.sha256(payload).hexdigest(),
            timestamp_s=timestamp_s,
            source_timestamp_s=source_timestamp_s,
            # Missing provenance is unknown.  It must never be promoted to the
            # host monotonic clock merely because a caller omitted the field.
            timestamp_reference=timestamp_reference,
            source_native_frame_id=str(metadata.get("source_native_frame_id") or ""),
        )
        append_jsonl_record(
            self.events_path,
            record.to_mapping(),
            prepared=True,
        )
        return record


__all__ = ["NativeFrameArtifact", "NativeFrameStore"]
