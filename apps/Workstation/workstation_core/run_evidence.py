"""Append-only session evidence and session-owned artifacts.

This module owns no camera, MAVLink, or UI worker.  It provides the durable
boundary those producers use so an external analysis application can join
evidence by ``session_id`` and verify the actual bytes by SHA-256.

Coordinate contract
-------------------
This ledger does not reinterpret coordinates.  Event payloads must retain
their producer frame and axis contract (for example WGS84 latitude/longitude,
world NED ``[north,east,down]``, PX4 body FRD, or OpenCV optical RDF).  The
ledger adds provenance and integrity metadata only.
"""

from __future__ import annotations

import hashlib
import json
import os
import re
import time
import uuid
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Mapping

from Workstation.workstation_core.strict_jsonl import (
    append_jsonl_record,
    prepare_jsonl_append,
)


RUN_EVENT_SCHEMA = "workstation-run-event-v2"
ARTIFACT_SCHEMA = "workstation-session-artifact-v1"
SESSION_MANIFEST_SCHEMA = "session-manifest-v4"

_SAFE_COMPONENT = re.compile(r"[^A-Za-z0-9._-]+")


def _required_identifier(value: object, name: str) -> str:
    text = str(value or "").strip()
    if not text:
        raise ValueError(f"{name} is required")
    if any(character in text for character in ("/", "\\", "\x00")):
        raise ValueError(f"{name} must not contain path separators")
    return text


def _json_safe(value: Any) -> Any:
    if value is None or isinstance(value, (bool, int, str)):
        return value
    if isinstance(value, float):
        if not (float("-inf") < value < float("inf")):
            raise ValueError("event payload contains a non-finite number")
        return value
    if isinstance(value, Path):
        return str(value)
    if isinstance(value, Mapping):
        return {str(key): _json_safe(item) for key, item in value.items()}
    if isinstance(value, (tuple, list)):
        return [_json_safe(item) for item in value]
    to_mapping = getattr(value, "to_mapping", None)
    if callable(to_mapping):
        return _json_safe(to_mapping())
    raise TypeError(f"unsupported evidence value: {type(value).__name__}")


def canonical_json_bytes(value: Mapping[str, Any]) -> bytes:
    """Return stable UTF-8 JSON bytes used by content/signature hashes."""

    normalized = _json_safe(value)
    return json.dumps(
        normalized,
        ensure_ascii=False,
        allow_nan=False,
        sort_keys=True,
        separators=(",", ":"),
    ).encode("utf-8")


def sha256_file(path: Path, *, chunk_size: int = 1024 * 1024) -> str:
    digest = hashlib.sha256()
    with Path(path).open("rb") as handle:
        while True:
            chunk = handle.read(chunk_size)
            if not chunk:
                break
            digest.update(chunk)
    return digest.hexdigest()


@dataclass(frozen=True)
class ArtifactRecord:
    artifact_id: str
    role: str
    relative_path: str
    size_bytes: int
    sha256: str
    source_name: str
    media_type: str
    imported_utc_ns: int
    metadata: tuple[tuple[str, Any], ...] = ()

    def to_mapping(self) -> dict[str, Any]:
        return {
            "schema": ARTIFACT_SCHEMA,
            "artifact_id": self.artifact_id,
            "role": self.role,
            "relative_path": self.relative_path,
            "size_bytes": self.size_bytes,
            "sha256": self.sha256,
            "source_name": self.source_name,
            "media_type": self.media_type,
            "imported_utc_ns": self.imported_utc_ns,
            "metadata": {key: value for key, value in self.metadata},
        }


class RunEvidenceLedger:
    """Durable append-only evidence ledger for one Workstation session."""

    def __init__(self, root: Path | str) -> None:
        self.root = Path(root).expanduser().resolve()
        self.root.mkdir(parents=True, exist_ok=True)
        self.session_id = _required_identifier(self.root.name, "session_id")
        self._events_path = self.root / "run_events.jsonl"
        self._artifacts_path = self.root / "artifacts.jsonl"
        self._ledger_recovery = {
            "run_events": prepare_jsonl_append(self._events_path),
            "artifacts": prepare_jsonl_append(self._artifacts_path),
        }
        self._locked = self._events_path.exists() and self._events_path.stat().st_size > 0

    @property
    def locked(self) -> bool:
        return self._locked

    def append_event(
        self,
        event_type: str,
        payload: Mapping[str, Any],
        *,
        source: str,
        source_timestamp_ns: int | None = None,
        coordinate_contract: str = "not_applicable",
    ) -> dict[str, Any]:
        event_name = _required_identifier(event_type, "event_type")
        now_utc_ns = time.time_ns()
        event = {
            "schema": RUN_EVENT_SCHEMA,
            "event_id": str(uuid.uuid4()),
            "event_type": event_name,
            "session_id": self.session_id,
            "source": str(source),
            "source_timestamp_ns": None if source_timestamp_ns is None else int(source_timestamp_ns),
            "receive_utc_ns": now_utc_ns,
            "receive_monotonic_ns": time.monotonic_ns(),
            "coordinate_contract": str(coordinate_contract),
            "payload": _json_safe(payload),
        }
        self._append_json_line(self._events_path, event)
        self._locked = True
        return event

    def import_artifact(
        self,
        source_path: Path | str,
        *,
        role: str,
        media_type: str = "application/octet-stream",
        metadata: Mapping[str, Any] | None = None,
    ) -> ArtifactRecord:
        source = Path(source_path).expanduser().resolve(strict=True)
        if not source.is_file():
            raise ValueError("artifact source must be a regular file")
        role_text = str(role or "").strip()
        if not role_text or "\x00" in role_text:
            raise ValueError("role is required")
        safe_role = _SAFE_COMPONENT.sub("_", role_text).strip("._")
        if not safe_role:
            raise ValueError("role does not contain a safe path component")
        destination_dir = self.root / "assets" / safe_role
        destination_dir.mkdir(parents=True, exist_ok=True)
        temporary = destination_dir / f".{uuid.uuid4().hex}.part"
        digest = hashlib.sha256()
        size = 0
        try:
            with source.open("rb") as reader, temporary.open("xb") as writer:
                while True:
                    chunk = reader.read(1024 * 1024)
                    if not chunk:
                        break
                    writer.write(chunk)
                    digest.update(chunk)
                    size += len(chunk)
                writer.flush()
                os.fsync(writer.fileno())
            sha256 = digest.hexdigest()
            safe_stem = _SAFE_COMPONENT.sub("_", source.stem).strip("._") or "artifact"
            safe_suffix = _SAFE_COMPONENT.sub("", source.suffix.lower())
            destination = destination_dir / f"{safe_stem}-{sha256[:16]}{safe_suffix}"
            if destination.exists():
                if destination.stat().st_size != size or sha256_file(destination) != sha256:
                    raise RuntimeError("content-addressed artifact collision")
                temporary.unlink()
            else:
                os.replace(temporary, destination)
                self._fsync_directory(destination_dir)
        except Exception:
            try:
                temporary.unlink()
            except FileNotFoundError:
                pass
            raise

        relative = destination.relative_to(self.root).as_posix()
        record = ArtifactRecord(
            artifact_id=str(uuid.uuid4()),
            role=role_text,
            relative_path=relative,
            size_bytes=size,
            sha256=sha256,
            source_name=source.name,
            media_type=str(media_type),
            imported_utc_ns=time.time_ns(),
            metadata=tuple(sorted((str(key), _json_safe(value)) for key, value in (metadata or {}).items())),
        )
        self._record_artifact(
            record,
            "artifact_imported",
            source="RunEvidenceLedger.import_artifact",
        )
        return record

    def register_session_artifact(
        self,
        session_path: Path | str,
        *,
        role: str,
        media_type: str = "application/octet-stream",
        metadata: Mapping[str, Any] | None = None,
        expected_size_bytes: int | None = None,
        expected_sha256: str | None = None,
    ) -> ArtifactRecord:
        """Register already session-owned immutable bytes without copying them.

        ``import_artifact`` remains the boundary for external files: it copies
        those bytes into ``assets/`` before recording them.  This method is for
        producers such as the ULog and target-drone recorders that have already
        atomically sealed their output below this ledger's session root.
        """

        raw_path = Path(session_path).expanduser()
        candidate = raw_path if raw_path.is_absolute() else self.root / raw_path
        source = candidate.resolve(strict=True)
        try:
            relative = source.relative_to(self.root).as_posix()
        except ValueError as exc:
            raise ValueError("session artifact must be inside the session root") from exc
        if not source.is_file():
            raise ValueError("session artifact must be a regular file")

        role_text = str(role or "").strip()
        if not role_text or "\x00" in role_text:
            raise ValueError("role is required")
        size = source.stat().st_size
        if expected_size_bytes is not None:
            if isinstance(expected_size_bytes, bool) or int(expected_size_bytes) < 0:
                raise ValueError("expected_size_bytes must be a non-negative integer")
            if size != int(expected_size_bytes):
                raise ValueError("session artifact size does not match the producer record")
        sha256 = sha256_file(source)
        if expected_sha256 is not None:
            expected_digest = str(expected_sha256).strip().lower()
            if not re.fullmatch(r"[0-9a-f]{64}", expected_digest):
                raise ValueError("expected_sha256 must be a 64-character hexadecimal digest")
            if sha256 != expected_digest:
                raise ValueError("session artifact SHA-256 does not match the producer record")

        normalized_metadata = {
            str(key): _json_safe(value) for key, value in (metadata or {}).items()
        }
        normalized_metadata.update(
            {
                "registration": "session_reference",
                "storage_class": "sealed_immutable",
                "snapshot_policy": "reference_by_hash",
            }
        )
        existing = self._existing_registered_artifact(
            role=role_text,
            relative_path=relative,
            sha256=sha256,
        )
        if existing is not None:
            return existing
        record = ArtifactRecord(
            artifact_id=str(uuid.uuid4()),
            role=role_text,
            relative_path=relative,
            size_bytes=size,
            sha256=sha256,
            source_name=source.name,
            media_type=str(media_type),
            imported_utc_ns=time.time_ns(),
            metadata=tuple(sorted(normalized_metadata.items())),
        )
        self._record_artifact(
            record,
            "artifact_registered",
            source="RunEvidenceLedger.register_session_artifact",
        )
        return record

    def _existing_registered_artifact(
        self,
        *,
        role: str,
        relative_path: str,
        sha256: str,
    ) -> ArtifactRecord | None:
        if not self._artifacts_path.is_file():
            return None
        with self._artifacts_path.open("r", encoding="utf-8") as handle:
            for line in handle:
                if not line.strip():
                    continue
                try:
                    value = json.loads(line)
                except json.JSONDecodeError:
                    continue
                if not isinstance(value, Mapping):
                    continue
                if (
                    str(value.get("role") or "") != role
                    or str(value.get("relative_path") or "") != relative_path
                    or str(value.get("sha256") or "") != sha256
                ):
                    continue
                metadata = value.get("metadata")
                metadata = metadata if isinstance(metadata, Mapping) else {}
                return ArtifactRecord(
                    artifact_id=str(value.get("artifact_id") or ""),
                    role=role,
                    relative_path=relative_path,
                    size_bytes=int(value.get("size_bytes") or 0),
                    sha256=sha256,
                    source_name=str(value.get("source_name") or Path(relative_path).name),
                    media_type=str(value.get("media_type") or "application/octet-stream"),
                    imported_utc_ns=int(value.get("imported_utc_ns") or 0),
                    metadata=tuple(
                        sorted((str(key), _json_safe(item)) for key, item in metadata.items())
                    ),
                )
        return None

    def _record_artifact(
        self,
        record: ArtifactRecord,
        event_type: str,
        *,
        source: str,
    ) -> None:
        self._append_json_line(self._artifacts_path, record.to_mapping())
        self._locked = True
        self.append_event(
            event_type,
            {
                "artifact_id": record.artifact_id,
                "role": record.role,
                "relative_path": record.relative_path,
                "size_bytes": record.size_bytes,
                "sha256": record.sha256,
                "source_name": record.source_name,
            },
            source=source,
        )

    def resolve_artifact(self, relative_path: str) -> Path:
        candidate = (self.root / str(relative_path)).resolve()
        try:
            candidate.relative_to(self.root)
        except ValueError as exc:
            raise ValueError("artifact path escapes the session root") from exc
        return candidate

    def artifacts(self) -> tuple[ArtifactRecord, ...]:
        if not self._artifacts_path.exists():
            return ()
        records: list[ArtifactRecord] = []
        with self._artifacts_path.open("r", encoding="utf-8") as handle:
            for line in handle:
                if not line.strip():
                    continue
                value = json.loads(line)
                metadata = value.get("metadata")
                metadata_items = tuple(sorted((str(key), item) for key, item in (metadata or {}).items()))
                records.append(
                    ArtifactRecord(
                        artifact_id=str(value["artifact_id"]),
                        role=str(value["role"]),
                        relative_path=str(value["relative_path"]),
                        size_bytes=int(value["size_bytes"]),
                        sha256=str(value["sha256"]),
                        source_name=str(value.get("source_name") or ""),
                        media_type=str(value.get("media_type") or "application/octet-stream"),
                        imported_utc_ns=int(value["imported_utc_ns"]),
                        metadata=metadata_items,
                    )
                )
        return tuple(records)

    def manifest_snapshot(self) -> dict[str, Any]:
        artifacts = self.artifacts()
        return {
            "schema": SESSION_MANIFEST_SCHEMA,
            "session_id": self.session_id,
            "locked": self._locked,
            "event_log": {
                "path": self._events_path.relative_to(self.root).as_posix(),
                "sha256": sha256_file(self._events_path) if self._events_path.exists() else None,
            },
            "artifacts": [record.to_mapping() for record in artifacts],
        }

    @staticmethod
    def _append_json_line(path: Path, value: Mapping[str, Any]) -> None:
        append_jsonl_record(path, value, prepared=True)

    @staticmethod
    def _atomic_write_json(path: Path, value: Mapping[str, Any]) -> None:
        path.parent.mkdir(parents=True, exist_ok=True)
        temporary = path.with_name(f".{path.name}.{uuid.uuid4().hex}.tmp")
        try:
            with temporary.open("xb") as handle:
                handle.write(canonical_json_bytes(value) + b"\n")
                handle.flush()
                os.fsync(handle.fileno())
            os.replace(temporary, path)
            RunEvidenceLedger._fsync_directory(path.parent)
        except Exception:
            try:
                temporary.unlink()
            except FileNotFoundError:
                pass
            raise

    @staticmethod
    def _fsync_directory(path: Path) -> None:
        descriptor = os.open(path, os.O_RDONLY)
        try:
            os.fsync(descriptor)
        finally:
            os.close(descriptor)


__all__ = [
    "ARTIFACT_SCHEMA",
    "ArtifactRecord",
    "RUN_EVENT_SCHEMA",
    "RunEvidenceLedger",
    "SESSION_MANIFEST_SCHEMA",
    "canonical_json_bytes",
    "sha256_file",
]
