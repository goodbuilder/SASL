"""Lossless native PNG persistence for pose and accepted-measurement evidence."""

from __future__ import annotations

import hashlib
import math
import os
import queue
import time
import uuid
from dataclasses import dataclass
from pathlib import Path
import threading
from typing import Any, Callable, Mapping, Sequence

from PyQt5 import QtCore, QtGui

from Workstation.workstation_core.strict_jsonl import (
    append_jsonl_record,
    prepare_jsonl_append,
)
from Workstation.workstation_core.measurement_data_store import (
    _safe_token,
    creation_timestamp_text,
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


class NativeFrameStore:
    """Synchronous atomic writer intended for a dedicated controller/worker."""

    def __init__(
        self,
        session_root: Path | str,
        *,
        events_path: Path | str | None = None,
        event_sink: Callable[[Mapping[str, Any]], bool] | None = None,
    ) -> None:
        self.root = Path(session_root).expanduser().resolve()
        self.root.mkdir(parents=True, exist_ok=True)
        self.events_path = (
            self.root / "native_frames.jsonl"
            if events_path is None
            else Path(events_path).expanduser().resolve()
        )
        self.event_sink = event_sink
        self.ledger_recovery = None if event_sink else prepare_jsonl_append(self.events_path)

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
        # These must come from the producer's own recorded frame size, not be
        # backfilled from the QImage being validated here -- defaulting to
        # image.width()/height() would make this check tautological (it can
        # never fail, since it would always compare the image against
        # itself). A caller with no genuine producer-side dimensions must
        # fail loudly instead of silently skipping the check.
        raw_width = metadata.get("native_width")
        raw_height = metadata.get("native_height")
        if raw_width is None or raw_height is None:
            raise ValueError(
                "metadata must include native_width/native_height from the frame "
                "producer; dimensions cannot be inferred from the image itself"
            )
        expected_width = int(raw_width)
        expected_height = int(raw_height)
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
            try:
                os.link(temporary, destination)
            except FileExistsError as exc:
                raise OSError(
                    f"refusing to replace existing native frame: {destination}"
                ) from exc
            temporary.unlink()
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
        if self.event_sink is not None:
            if not self.event_sink(record.to_mapping()):
                raise OSError("native image saved but artifact record enqueue failed")
        else:
            append_jsonl_record(self.events_path, record.to_mapping(), prepared=True)
        return record


class AsyncNativeFrameStore(QtCore.QObject):
    """Bounded single-writer queue that keeps PNG/fsync work off the GUI."""

    completed = QtCore.pyqtSignal(object)
    DEFAULT_MAX_PENDING = 16

    def __init__(
        self,
        session_root: Path | str,
        *,
        max_pending: int = DEFAULT_MAX_PENDING,
        timestamp_filenames: bool = False,
        events_path: Path | str | None = None,
        event_sink: Callable[[Mapping[str, Any], Mapping[str, Any]], bool] | None = None,
        parent: QtCore.QObject | None = None,
    ) -> None:
        super().__init__(parent)
        self.root = Path(session_root).expanduser().resolve()
        self.max_pending = max(1, int(max_pending))
        self.timestamp_filenames = bool(timestamp_filenames)
        self.event_sink = event_sink
        self.events_path = (
            None
            if events_path is None
            else Path(events_path).expanduser().resolve()
        )
        self._lock = threading.RLock()
        self._pending: set[str] = set()
        self._work_queue: queue.Queue[object] = queue.Queue(
            maxsize=self.max_pending
        )
        self._store: NativeFrameStore | None = None
        self._closed = False
        self._writer_thread = threading.Thread(
            target=self._run_writer,
            name="native-frame-store",
            daemon=True,
        )
        self._writer_thread.start()

    @property
    def pending_count(self) -> int:
        with self._lock:
            return len(self._pending)

    def enqueue_pair(
        self,
        *,
        request_id: str,
        pair_id: str,
        purpose: str,
        frames: Sequence[tuple[str, QtGui.QImage, Mapping[str, Any]]],
        missing_sides: Sequence[str] = (),
        context: Mapping[str, object] | None = None,
    ) -> dict[str, object] | None:
        request = str(request_id)
        with self._lock:
            if self._closed or request in self._pending:
                return None
            if len(self._pending) >= self.max_pending:
                return None
            frozen_frames = tuple(
                (str(side), QtGui.QImage(image), dict(metadata))
                for side, image, metadata in frames
            )
            stored_pair_id = str(pair_id)
            if self.timestamp_filenames:
                stamp = (context or {}).get("measurement_file_stamp") or creation_timestamp_text()
                stored_pair_id = (
                    f"{stamp}_"
                    f"{_safe_token(pair_id, 'frame')}"
                )
            task = (
                request,
                stored_pair_id,
                str(purpose),
                frozen_frames,
                tuple(str(side) for side in missing_sides),
                dict(context or {}),
            )
            self._pending.add(request)
            try:
                self._work_queue.put_nowait(task)
            except queue.Full:
                self._pending.discard(request)
                return None

        purpose_value = _safe_token(purpose, "unspecified")
        identifier = _safe_token(stored_pair_id, "frame")
        scheduled: dict[str, object] = {
            "native_pair_status": "pending",
            "native_pair_request_id": request,
            "evidence_state": "WRITING",
        }
        for side, image, metadata in frozen_frames:
            scheduled.update(
                {
                    f"native_{side}_frame_id": identifier,
                    f"native_{side}_png_path": (
                        Path("frames")
                        / purpose_value
                        / f"{identifier}-{side}.png"
                    ).as_posix(),
                    f"native_{side}_width_px": image.width(),
                    f"native_{side}_height_px": image.height(),
                    f"native_{side}_timestamp_reference": str(
                        metadata.get("timestamp_reference") or "unknown"
                    ),
                    f"native_{side}_source_frame_id": str(
                        metadata.get("source_native_frame_id") or ""
                    ),
                }
            )
        if missing_sides:
            scheduled["native_pair_missing_sides"] = ",".join(missing_sides)
        return scheduled

    def acknowledge(self, request_id: str) -> None:
        with self._lock:
            self._pending.discard(str(request_id))

    def close(self) -> bool:
        with self._lock:
            if self._pending:
                return False
            if self._closed:
                return True
            self._closed = True
        try:
            self._work_queue.put_nowait(None)
        except queue.Full:
            return False
        return True

    def _persist_pair(
        self,
        request_id: str,
        pair_id: str,
        purpose: str,
        frames: tuple[tuple[str, QtGui.QImage, dict[str, Any]], ...],
        missing_sides: tuple[str, ...],
        context: dict[str, object],
    ) -> dict[str, object]:
        root = Path(str(context.get("native_root") or self.root)).resolve()
        if self._store is None or self._store.root != root:
            sink = self.event_sink if context.get("measurement_id") else None
            self._store = NativeFrameStore(
                root,
                events_path=context.get("native_events_path") or self.events_path,
                event_sink=(lambda payload: sink(payload, context)) if sink else None,
            )
        fields: dict[str, object] = {
            "native_pair_status": "complete",
            "evidence_state": "SAVED",
        }
        failed = list(missing_sides)
        for side, image, metadata in frames:
            try:
                artifact = self._store.persist(
                    side=side,
                    purpose=purpose,
                    frame_id=pair_id,
                    image=image,
                    metadata=metadata,
                )
            except Exception as exc:  # noqa: BLE001 - one side remains explicit evidence.
                fields[f"native_{side}_error"] = str(exc)
                failed.append(side)
                continue
            fields.update(
                {
                    f"native_{side}_frame_id": artifact.frame_id,
                    f"native_{side}_png_path": artifact.relative_path,
                    f"native_{side}_png_sha256": artifact.sha256,
                    f"native_{side}_width_px": artifact.width,
                    f"native_{side}_height_px": artifact.height,
                    f"native_{side}_timestamp_reference": artifact.timestamp_reference,
                    f"native_{side}_source_frame_id": artifact.source_native_frame_id,
                }
            )
        if failed:
            fields["native_pair_status"] = "missing:" + ",".join(
                dict.fromkeys(failed)
            )
            fields["evidence_state"] = "FAILED"
        return {
            "request_id": request_id,
            "context": context,
            "fields": fields,
        }

    def _run_writer(self) -> None:
        while True:
            task = self._work_queue.get()
            if task is None:
                return
            (
                request_id,
                pair_id,
                purpose,
                frames,
                missing_sides,
                context,
            ) = task
            try:
                result = self._persist_pair(
                    request_id,
                    pair_id,
                    purpose,
                    frames,
                    missing_sides,
                    context,
                )
            except Exception as exc:  # noqa: BLE001 - surface executor/setup failures.
                result = {
                    "request_id": request_id,
                    "context": context,
                    "fields": {
                        "native_pair_status": "failed",
                        "evidence_state": "FAILED",
                        "native_pair_error": f"{type(exc).__name__}: {exc}",
                    },
                }
            self.completed.emit(result)


__all__ = ["AsyncNativeFrameStore", "NativeFrameArtifact", "NativeFrameStore"]
