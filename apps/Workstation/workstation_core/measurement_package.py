"""Portable, revisioned evidence packages for one Workstation measurement.

The package manifest is deliberately producer-agnostic.  Camera, telemetry,
calibration, and result writers own their bytes; this module owns the durable
inventory, completeness decision, and integrity chain for those bytes.

All authoritative paths in a manifest are relative to the session root.
Symlinks are never authoritative: a path registered through an internal
symlink is stored as the resolved target path, while a link escaping the
session root is rejected.
"""

from __future__ import annotations

import hashlib
import json
import mimetypes
import os
import re
import stat
import time
import uuid
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Iterable, Mapping


PACKAGE_SCHEMA = "workstation-measurement-package-v1"
PACKAGE_SCHEMA_VERSION = 1
WORKING_STATE_SCHEMA = "workstation-measurement-package-working-v1"
LATEST_REVISION_SCHEMA = "workstation-measurement-package-latest-v1"

WORKING_STATE_NAME = "package_working_state.json"
LATEST_REVISION_NAME = "latest_revision.json"
REVISIONS_DIRECTORY_NAME = "revisions"
MANIFEST_NAME = "package_manifest.json"
MANIFEST_SIDECAR_NAME = "package_manifest.sha256"
SUMMARY_NAME = "summary.md"

_REVISION_RE = re.compile(r"^r([0-9]{4,})$")
_SEALED_STATES = frozenset({"SEALED_COMPLETE", "SEALED_INCOMPLETE"})
_MUTABLE_STATES = frozenset({"COLLECTING", "POST_CAPTURE", "RECOVERABLE"})
_ARTIFACT_STATUSES = frozenset({"present", "missing", "failed", "not_applicable"})
_SOURCE_STATUSES = frozenset({"active", "not_applicable"})
_TEMP_SUFFIXES = (".tmp", ".part", ".partial", ".lock")
_STORAGE_CLASS_POLICIES = {
    "appendable_ledger": "copy_on_revision",
    "sealed_immutable": "reference_by_hash",
    "derived": "reference_by_hash",
    "temporary": "exclude",
}
_SNAPSHOT_POLICIES = frozenset(_STORAGE_CLASS_POLICIES.values())
_KNOWN_APPENDABLE_LEDGER_PATHS = frozenset(
    {
        "audit/application_log.jsonl",
        "artifacts.jsonl",
        "hr_captures.jsonl",
        "hr_imports.jsonl",
        "native_frames.jsonl",
        "oz_captures.jsonl",
        "run_events.jsonl",
        "ulog_artifacts.jsonl",
    }
)


def _json_safe(value: Any) -> Any:
    """Return a JSON-safe value and reject non-finite or opaque values."""

    if value is None or isinstance(value, (bool, int, str)):
        return value
    if isinstance(value, float):
        if not (float("-inf") < value < float("inf")):
            raise ValueError("package data contains a non-finite number")
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
    raise TypeError(f"unsupported package value: {type(value).__name__}")


def canonical_json_bytes(value: Mapping[str, Any]) -> bytes:
    """Serialize a mapping in the canonical form used by package hashes."""

    normalized = _json_safe(value)
    return json.dumps(
        normalized,
        ensure_ascii=False,
        allow_nan=False,
        sort_keys=True,
        separators=(",", ":"),
    ).encode("utf-8")


def sha256_file(path: Path | str, *, chunk_size: int = 1024 * 1024) -> str:
    digest = hashlib.sha256()
    with Path(path).open("rb") as handle:
        while True:
            chunk = handle.read(chunk_size)
            if not chunk:
                break
            digest.update(chunk)
    return digest.hexdigest()


def _required_text(value: object, field: str) -> str:
    text = str(value or "").strip()
    if not text:
        raise ValueError(f"{field} is required")
    if "\x00" in text:
        raise ValueError(f"{field} must not contain NUL")
    return text


def _artifact_storage_metadata(
    relative_path: str,
    metadata: Mapping[str, Any] | None,
    *,
    storage_class: str | None,
    snapshot_policy: str | None,
) -> dict[str, Any]:
    """Normalize the explicit artifact mutability/snapshot contract.

    ``storage_class=None`` preserves legacy producer descriptors that predate
    the contract.  Such entries are treated as immutable references unless
    their exact session-relative path is one of the established append-only
    ledgers above.
    """

    normalized = dict(_json_safe(metadata or {}))
    stored_class = normalized.get("storage_class")
    stored_policy = normalized.get("snapshot_policy")
    if storage_class == "auto":
        if stored_class is not None:
            storage_class = str(stored_class)
        elif relative_path in _KNOWN_APPENDABLE_LEDGER_PATHS:
            storage_class = "appendable_ledger"
        else:
            storage_class = "sealed_immutable"
    elif storage_class is None and stored_class is not None:
        storage_class = str(stored_class)

    if snapshot_policy is None and stored_policy is not None:
        snapshot_policy = str(stored_policy)
    if storage_class is None:
        if snapshot_policy is not None:
            raise ValueError("snapshot_policy requires storage_class")
        return normalized
    storage_text = str(storage_class).strip().lower()
    if storage_text not in _STORAGE_CLASS_POLICIES:
        raise ValueError(f"unsupported artifact storage_class: {storage_class}")
    expected_policy = _STORAGE_CLASS_POLICIES[storage_text]
    policy_text = expected_policy if snapshot_policy is None else str(snapshot_policy).strip().lower()
    if policy_text not in _SNAPSHOT_POLICIES:
        raise ValueError(f"unsupported artifact snapshot_policy: {snapshot_policy}")
    if policy_text != expected_policy:
        raise ValueError(
            f"storage_class {storage_text} requires snapshot_policy {expected_policy}"
        )
    normalized["storage_class"] = storage_text
    normalized["snapshot_policy"] = policy_text
    return normalized


def _snapshot_policy_for_artifact(artifact: Mapping[str, Any]) -> str:
    metadata = artifact.get("metadata")
    if isinstance(metadata, Mapping):
        policy = str(metadata.get("snapshot_policy") or "").strip().lower()
        if policy in _SNAPSHOT_POLICIES:
            return policy
    relative = str(artifact.get("relative_path") or "")
    if relative in _KNOWN_APPENDABLE_LEDGER_PATHS:
        return "copy_on_revision"
    return "reference_by_hash"


def _session_id(value: object | None) -> str:
    if value is None:
        stamp = time.strftime("%Y%m%dT%H%M%SZ", time.gmtime())
        return f"{stamp}-{uuid.uuid4().hex[:8]}"
    text = _required_text(value, "session_id")
    if any(character in text for character in ("/", "\\")) or text in {".", ".."}:
        raise ValueError("session_id must be one portable path component")
    return text


def _revision_name(number: int) -> str:
    if int(number) < 1:
        raise ValueError("revision number must be >= 1")
    return f"r{int(number):04d}"


def _revision_number(value: int | str) -> int:
    if isinstance(value, int):
        number = value
    else:
        text = str(value).strip()
        match = _REVISION_RE.fullmatch(text)
        number = int(match.group(1)) if match else int(text)
    if number < 1:
        raise ValueError("revision number must be >= 1")
    return number


def _load_json(path: Path) -> dict[str, Any]:
    def reject_constant(value: str) -> None:
        raise ValueError(f"non-finite JSON constant {value}")

    with path.open("r", encoding="utf-8") as handle:
        value = json.load(handle, parse_constant=reject_constant)
    if not isinstance(value, dict):
        raise ValueError(f"{path.name} must contain a JSON object")
    return _json_safe(value)


def _fsync_directory(path: Path) -> None:
    descriptor = os.open(path, os.O_RDONLY)
    try:
        os.fsync(descriptor)
    finally:
        os.close(descriptor)


def _atomic_write_bytes(path: Path, data: bytes, *, replace: bool = True) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    if not replace and path.exists():
        raise FileExistsError(f"refusing to replace immutable file: {path}")
    temporary = path.with_name(f".{path.name}.{uuid.uuid4().hex}.tmp")
    try:
        with temporary.open("xb") as handle:
            handle.write(data)
            handle.flush()
            os.fsync(handle.fileno())
        if not replace and path.exists():
            raise FileExistsError(f"refusing to replace immutable file: {path}")
        os.replace(temporary, path)
        _fsync_directory(path.parent)
    except Exception:
        try:
            temporary.unlink()
        except FileNotFoundError:
            pass
        raise


def _atomic_write_json(path: Path, value: Mapping[str, Any], *, replace: bool = True) -> None:
    _atomic_write_bytes(path, canonical_json_bytes(value) + b"\n", replace=replace)


def _atomic_copy_file(source: Path, destination: Path, *, replace: bool = False) -> None:
    """Copy exact bytes into an immutable revision-owned snapshot."""

    destination.parent.mkdir(parents=True, exist_ok=True)
    temporary = destination.with_name(f".{destination.name}.{uuid.uuid4().hex}.tmp")
    try:
        with source.open("rb") as reader, temporary.open("xb") as writer:
            while True:
                chunk = reader.read(1024 * 1024)
                if not chunk:
                    break
                writer.write(chunk)
            writer.flush()
            os.fsync(writer.fileno())
        if destination.exists() and not replace:
            raise FileExistsError(f"refusing to replace immutable snapshot: {destination}")
        os.replace(temporary, destination)
        _fsync_directory(destination.parent)
    finally:
        temporary.unlink(missing_ok=True)


@dataclass(frozen=True)
class ArtifactEntry:
    artifact_id: str
    role: str
    classification: str
    relative_path: str | None
    producer: str
    source: str
    required: bool
    status: str
    media_type: str | None
    schema: str | None
    size_bytes: int | None
    sha256: str | None
    metadata: Mapping[str, Any]

    @classmethod
    def from_mapping(cls, value: Mapping[str, Any]) -> "ArtifactEntry":
        return cls(
            artifact_id=str(value.get("artifact_id") or ""),
            role=str(value.get("role") or ""),
            classification=str(value.get("classification") or "unclassified"),
            relative_path=(
                None if value.get("relative_path") in (None, "") else str(value["relative_path"])
            ),
            producer=str(value.get("producer") or "unknown"),
            source=str(value.get("source") or "unknown"),
            required=bool(value.get("required", False)),
            status=str(value.get("status") or "missing"),
            media_type=(None if value.get("media_type") is None else str(value["media_type"])),
            schema=(None if value.get("schema") is None else str(value["schema"])),
            size_bytes=(None if value.get("size_bytes") is None else int(value["size_bytes"])),
            sha256=(None if value.get("sha256") is None else str(value["sha256"])),
            metadata=_json_safe(value.get("metadata") or {}),
        )

    def to_mapping(self) -> dict[str, Any]:
        return {
            "artifact_id": self.artifact_id,
            "role": self.role,
            "classification": self.classification,
            "relative_path": self.relative_path,
            "producer": self.producer,
            "source": self.source,
            "required": self.required,
            "status": self.status,
            "media_type": self.media_type,
            "schema": self.schema,
            "size_bytes": self.size_bytes,
            "sha256": self.sha256,
            "metadata": _json_safe(self.metadata),
        }


@dataclass(frozen=True)
class VerificationIssue:
    code: str
    message: str
    path: str | None = None
    revision: str | None = None

    def to_mapping(self) -> dict[str, Any]:
        return {
            "code": self.code,
            "message": self.message,
            "path": self.path,
            "revision": self.revision,
        }


@dataclass(frozen=True)
class VerificationResult:
    valid: bool
    state: str
    session_id: str | None
    revision: str | None
    revision_number: int | None
    verified_manifests: int
    verified_artifacts: int
    issues: tuple[VerificationIssue, ...]

    @property
    def ok(self) -> bool:
        return self.valid

    def __bool__(self) -> bool:
        return self.valid

    def __getitem__(self, key: str) -> Any:
        return self.to_mapping()[key]

    def to_mapping(self) -> dict[str, Any]:
        return {
            "valid": self.valid,
            "state": self.state,
            "session_id": self.session_id,
            "revision": self.revision,
            "revision_number": self.revision_number,
            "verified_manifests": self.verified_manifests,
            "verified_artifacts": self.verified_artifacts,
            "issues": [issue.to_mapping() for issue in self.issues],
        }


class MeasurementPackage:
    """Mutable working inventory and immutable revision chain for one session."""

    def __init__(
        self,
        root: Path | str,
        *,
        session_id: str | None = None,
        metadata: Mapping[str, Any] | None = None,
    ) -> None:
        self.root = Path(root).expanduser().resolve()
        self._artifact_digest_cache: dict[tuple[str, int, int], str] = {}
        self.root.mkdir(parents=True, exist_ok=True)
        self._working_path = self.root / WORKING_STATE_NAME
        self._historical = False
        if self._working_path.exists():
            self._data = self._load_working_state()
            if self._data.get("state") == "VERIFYING":
                # A process can stop after entering VERIFYING but before an
                # immutable manifest exists.  Reopen it as explicitly
                # recoverable instead of leaving an unusable pseudo-state.
                self._data["state"] = "RECOVERABLE"
                self._data["metadata"] = {
                    **dict(self._data.get("metadata") or {}),
                    "recovery_reason": "interrupted_during_package_seal",
                }
                self._persist()
            if session_id is not None and self._data["session_id"] != _session_id(session_id):
                raise ValueError("session root already belongs to a different session_id")
            if metadata is not None:
                raise ValueError("metadata cannot replace persisted package metadata")
            return

        latest_path = self.root / LATEST_REVISION_NAME
        if latest_path.exists():
            latest = _load_json(latest_path)
            manifest_path = self.resolve_path(str(latest.get("manifest_path") or ""), must_exist=True)
            self._data = _load_json(manifest_path)
            self._historical = True
            if session_id is not None and self._data.get("session_id") != _session_id(session_id):
                raise ValueError("session root already belongs to a different session_id")
            if metadata is not None:
                raise ValueError("metadata cannot replace sealed package metadata")
            return

        now = time.time_ns()
        self._data = {
            "schema": PACKAGE_SCHEMA,
            "schema_version": PACKAGE_SCHEMA_VERSION,
            "working_state_schema": WORKING_STATE_SCHEMA,
            "session_id": _session_id(session_id),
            "revision": _revision_name(1),
            "revision_number": 1,
            "state": "COLLECTING",
            "created_utc_ns": now,
            "updated_utc_ns": now,
            "sealed_utc_ns": None,
            "metadata": _json_safe(metadata or {}),
            "measurement_outcome": None,
            "source_expectations": [],
            "artifacts": [],
            "evidence_completeness": {
                "status": "INCOMPLETE",
                "complete": False,
                "missing_required": [],
            },
            "previous_manifest": None,
        }
        self._persist()

    @classmethod
    def create(
        cls,
        root: Path | str,
        *,
        session_id: str | None = None,
        metadata: Mapping[str, Any] | None = None,
    ) -> "MeasurementPackage":
        resolved = Path(root).expanduser().resolve()
        if (resolved / WORKING_STATE_NAME).exists() or (resolved / LATEST_REVISION_NAME).exists():
            raise FileExistsError("a measurement package already exists at this session root")
        if (resolved / REVISIONS_DIRECTORY_NAME).exists():
            raise FileExistsError("revision bookkeeping already exists at this session root")
        return cls(resolved, session_id=session_id, metadata=metadata)

    @classmethod
    def open(cls, root: Path | str) -> "MeasurementPackage":
        resolved = Path(root).expanduser().resolve()
        if not (resolved / WORKING_STATE_NAME).exists() and not (resolved / LATEST_REVISION_NAME).exists():
            raise FileNotFoundError("no measurement package exists at this session root")
        return cls(resolved)

    @classmethod
    def open_revision(cls, root: Path | str, revision: int | str) -> "MeasurementPackage":
        resolved = Path(root).expanduser().resolve()
        number = _revision_number(revision)
        manifest_path = resolved / REVISIONS_DIRECTORY_NAME / _revision_name(number) / MANIFEST_NAME
        checked = _resolve_inside(resolved, manifest_path, must_exist=True)
        instance = cls.__new__(cls)
        instance.root = resolved
        instance._working_path = resolved / WORKING_STATE_NAME
        instance._historical = True
        instance._data = _load_json(checked)
        if instance._data.get("schema") != PACKAGE_SCHEMA:
            raise ValueError("unsupported package manifest schema")
        return instance

    @property
    def session_id(self) -> str:
        return str(self._data["session_id"])

    @property
    def state(self) -> str:
        return str(self._data["state"])

    @property
    def revision_number(self) -> int:
        return int(self._data["revision_number"])

    @property
    def revision(self) -> str:
        return str(self._data["revision"])

    @property
    def manifest_path(self) -> Path:
        return self.root / REVISIONS_DIRECTORY_NAME / self.revision / MANIFEST_NAME

    def snapshot(self) -> dict[str, Any]:
        """Return a detached, JSON-safe snapshot of the current package state."""

        return json.loads(canonical_json_bytes(self._data).decode("utf-8"))

    def resolve_path(self, relative_path: Path | str, *, must_exist: bool = False) -> Path:
        return _resolve_inside(self.root, relative_path, must_exist=must_exist)

    resolve_artifact = resolve_path

    def transition(self, state: str) -> str:
        self._ensure_mutable()
        target = str(state).strip().upper()
        allowed = {
            "COLLECTING": {"POST_CAPTURE", "RECOVERABLE"},
            "POST_CAPTURE": {"RECOVERABLE"},
            "RECOVERABLE": {"COLLECTING", "POST_CAPTURE"},
        }
        if target == self.state:
            return self.state
        if target not in allowed.get(self.state, set()):
            raise ValueError(f"invalid package state transition: {self.state} -> {target}")
        self._data["state"] = target
        self._persist()
        return target

    def mark_post_capture(self) -> str:
        return self.transition("POST_CAPTURE")

    def set_measurement_outcome(self, outcome: Any) -> None:
        self._ensure_mutable()
        self._data["measurement_outcome"] = _json_safe(outcome)
        self._persist()

    def update_metadata(self, values: Mapping[str, Any]) -> dict[str, Any]:
        """Merge durable controller/provenance state into a working revision."""

        self._ensure_mutable()
        metadata = dict(self._data.get("metadata") or {})
        metadata.update(dict(_json_safe(values)))
        self._data["metadata"] = metadata
        self._persist()
        return _json_safe(metadata)

    def set_source(
        self,
        source_id: str,
        *,
        active: bool | None = None,
        status: str | None = None,
        required_roles: Iterable[str] = (),
        metadata: Mapping[str, Any] | None = None,
    ) -> dict[str, Any]:
        self._ensure_mutable()
        source = _required_text(source_id, "source_id")
        if status is None:
            status_text = "active" if active is not False else "not_applicable"
        else:
            status_text = str(status).strip().lower()
            if active is not None and (status_text == "active") != bool(active):
                raise ValueError("active and status describe different source states")
        if status_text not in _SOURCE_STATUSES:
            raise ValueError("source status must be active or not_applicable")
        sources = list(self._data.get("source_expectations") or [])
        existing = next((item for item in sources if item.get("source_id") == source), None)
        if (
            isinstance(existing, Mapping)
            and (
                str(existing.get("status") or "") == "active"
                or bool(existing.get("active"))
            )
            and status_text == "not_applicable"
        ):
            raise ValueError("an active evidence source cannot become not_applicable")
        existing_roles = existing.get("required_roles", ()) if isinstance(existing, Mapping) else ()
        roles = sorted(
            {
                _required_text(role, "required_role")
                for role in (*tuple(existing_roles or ()), *tuple(required_roles))
            }
        )
        merged_metadata = dict(existing.get("metadata") or {}) if isinstance(existing, Mapping) else {}
        incoming_metadata = dict(_json_safe(metadata or {}))
        if "activated_utc_ns" in merged_metadata and "activated_utc_ns" in incoming_metadata:
            incoming_metadata["last_observed_utc_ns"] = incoming_metadata["activated_utc_ns"]
            incoming_metadata.pop("activated_utc_ns", None)
        merged_metadata.update(incoming_metadata)
        entry = {
            "source_id": source,
            "status": status_text,
            "active": status_text == "active",
            "required_roles": roles,
            "metadata": merged_metadata,
        }
        sources = [item for item in sources if item.get("source_id") != source]
        sources.append(entry)
        sources.sort(key=lambda item: str(item["source_id"]))
        self._data["source_expectations"] = sources
        self._refresh_completeness()
        self._persist()
        return _json_safe(entry)

    def register_artifact(
        self,
        path_or_role: Path | str,
        relative_path: Path | str | None = None,
        *,
        role: str | None = None,
        classification: str,
        producer: str,
        source: str,
        required: bool = False,
        status: str = "present",
        media_type: str | None = None,
        schema: str | None = None,
        data_schema: str | None = None,
        metadata: Mapping[str, Any] | None = None,
        artifact_id: str | None = None,
        storage_class: str | None = "auto",
        snapshot_policy: str | None = None,
    ) -> ArtifactEntry:
        self._ensure_mutable()
        if status != "present":
            raise ValueError("register_artifact only accepts status='present'")
        if relative_path is None:
            artifact_path = path_or_role
            artifact_role = _required_text(role, "role")
        else:
            artifact_path = relative_path
            artifact_role = _required_text(role if role is not None else path_or_role, "role")
        path = self.resolve_path(artifact_path, must_exist=True)
        if not path.is_file():
            raise ValueError("present artifact must be a regular file")
        relative = path.relative_to(self.root).as_posix()
        resolved_schema = _coalesce_schema(schema, data_schema)
        media = media_type or mimetypes.guess_type(path.name)[0] or "application/octet-stream"
        normalized_metadata = _artifact_storage_metadata(
            relative,
            metadata,
            storage_class=storage_class,
            snapshot_policy=snapshot_policy,
        )
        entry = ArtifactEntry(
            artifact_id=_required_text(artifact_id or uuid.uuid4(), "artifact_id"),
            role=artifact_role,
            classification=_required_text(classification, "classification"),
            relative_path=relative,
            producer=_required_text(producer, "producer"),
            source=_required_text(source, "source"),
            required=bool(required),
            status="present",
            media_type=_required_text(media, "media_type"),
            schema=resolved_schema,
            size_bytes=path.stat().st_size,
            sha256=self._cached_sha256(path),
            metadata=normalized_metadata,
        )
        self._upsert_artifact(entry)
        return entry

    def _cached_sha256(self, path: Path) -> str:
        stat_result = path.stat()
        key = (str(path), stat_result.st_size, stat_result.st_mtime_ns)
        digest = self._artifact_digest_cache.get(key)
        if digest is None:
            digest = sha256_file(path)
            self._artifact_digest_cache[key] = digest
        return digest

    def register_missing(
        self,
        role: str,
        *,
        relative_path: Path | str | None = None,
        classification: str = "unclassified",
        producer: str = "unknown",
        source: str = "unknown",
        required: bool = True,
        media_type: str | None = None,
        schema: str | None = None,
        data_schema: str | None = None,
        reason: str | None = None,
        metadata: Mapping[str, Any] | None = None,
        artifact_id: str | None = None,
    ) -> ArtifactEntry:
        return self._register_absent(
            "missing",
            role,
            relative_path=relative_path,
            classification=classification,
            producer=producer,
            source=source,
            required=required,
            media_type=media_type,
            schema=_coalesce_schema(schema, data_schema),
            reason=reason,
            metadata=metadata,
            artifact_id=artifact_id,
        )

    def register_failed(
        self,
        role: str,
        *,
        relative_path: Path | str | None = None,
        classification: str = "unclassified",
        producer: str = "unknown",
        source: str = "unknown",
        required: bool = True,
        media_type: str | None = None,
        schema: str | None = None,
        data_schema: str | None = None,
        reason: str | None = None,
        metadata: Mapping[str, Any] | None = None,
        artifact_id: str | None = None,
    ) -> ArtifactEntry:
        return self._register_absent(
            "failed",
            role,
            relative_path=relative_path,
            classification=classification,
            producer=producer,
            source=source,
            required=required,
            media_type=media_type,
            schema=_coalesce_schema(schema, data_schema),
            reason=reason,
            metadata=metadata,
            artifact_id=artifact_id,
        )

    def register_not_applicable(
        self,
        role: str,
        *,
        source: str,
        classification: str = "unclassified",
        producer: str = "unknown",
        metadata: Mapping[str, Any] | None = None,
    ) -> ArtifactEntry:
        return self._register_absent(
            "not_applicable",
            role,
            classification=classification,
            producer=producer,
            source=source,
            required=False,
            metadata=metadata,
        )

    def register_descriptor(self, descriptor: Mapping[str, Any]) -> ArtifactEntry:
        """Register a producer descriptor while independently hashing present bytes."""

        status = str(descriptor.get("status") or "present").lower()
        common = {
            "classification": str(descriptor.get("classification") or "unclassified"),
            "producer": str(descriptor.get("producer") or "unknown"),
            "source": str(descriptor.get("source") or "unknown"),
            "required": bool(descriptor.get("required", False)),
            "media_type": descriptor.get("media_type"),
            "schema": descriptor.get("schema"),
            "metadata": descriptor.get("metadata") or {},
            "artifact_id": descriptor.get("artifact_id"),
        }
        role = str(descriptor.get("role") or "")
        relative_path = descriptor.get("relative_path")
        if status == "present":
            if relative_path in (None, ""):
                raise ValueError("present descriptor requires relative_path")
            return self.register_artifact(
                relative_path,
                role=role,
                # Preserve byte-compatible legacy descriptors unless their
                # producer explicitly opts into the mutability contract.
                storage_class=descriptor.get("storage_class"),
                snapshot_policy=descriptor.get("snapshot_policy"),
                **common,
            )
        if status == "missing":
            return self.register_missing(role, relative_path=relative_path, **common)
        if status == "failed":
            return self.register_failed(role, relative_path=relative_path, **common)
        if status == "not_applicable":
            return self._register_absent(
                status,
                role,
                relative_path=relative_path,
                classification=common["classification"],
                producer=common["producer"],
                source=common["source"],
                required=False,
                media_type=common["media_type"],
                schema=common["schema"],
                metadata=common["metadata"],
                artifact_id=common["artifact_id"],
            )
        raise ValueError(f"unsupported artifact status: {status}")

    def seal(self, measurement_outcome: Any = None) -> dict[str, Any]:
        """Seal the working inventory as one immutable revision manifest."""

        self._ensure_mutable()
        if measurement_outcome is not None:
            self._data["measurement_outcome"] = _json_safe(measurement_outcome)
        target_dir = self.manifest_path.parent
        sidecar_path = target_dir / MANIFEST_SIDECAR_NAME
        if self.manifest_path.exists():
            try:
                return self._resume_existing_seal(measurement_outcome)
            except Exception:
                self._mark_recoverable()
                raise
        if sidecar_path.exists():
            self._mark_recoverable()
            raise FileExistsError("manifest sidecar exists without its revision manifest")

        self._data["state"] = "VERIFYING"
        self._persist(allow_verifying=True)
        try:
            target_dir.mkdir(parents=True, exist_ok=True)
            self._drop_generated_entries()
            self._refresh_present_artifacts()
            self._auto_inventory()
            self._freeze_appendable_evidence(target_dir)
            self._refresh_completeness()
            summary_path = target_dir / SUMMARY_NAME
            _atomic_write_bytes(summary_path, self._summary_text().encode("utf-8"), replace=True)
            self._add_generated_summary(summary_path)
            completeness = self._calculate_completeness()
            sealed_state = "SEALED_COMPLETE" if completeness["complete"] else "SEALED_INCOMPLETE"
            sealed_at = time.time_ns()
            manifest = self.snapshot()
            manifest.pop("working_state_schema", None)
            manifest.pop("updated_utc_ns", None)
            manifest["state"] = sealed_state
            manifest["sealed_utc_ns"] = sealed_at
            manifest["evidence_completeness"] = completeness
            manifest_bytes = canonical_json_bytes(manifest) + b"\n"
            manifest_sha256 = hashlib.sha256(manifest_bytes).hexdigest()
            _atomic_write_bytes(self.manifest_path, manifest_bytes, replace=False)
            sidecar = f"{manifest_sha256}  {MANIFEST_NAME}\n".encode("ascii")
            _atomic_write_bytes(sidecar_path, sidecar, replace=False)
            latest = {
                "schema": LATEST_REVISION_SCHEMA,
                "session_id": self.session_id,
                "revision": self.revision,
                "revision_number": self.revision_number,
                "manifest_path": self.manifest_path.relative_to(self.root).as_posix(),
                "manifest_sha256": manifest_sha256,
                "state": sealed_state,
                "evidence_complete": bool(completeness["complete"]),
                "updated_utc_ns": time.time_ns(),
            }
            _atomic_write_json(self.root / LATEST_REVISION_NAME, latest)
            self._data = dict(manifest)
            self._data["working_state_schema"] = WORKING_STATE_SCHEMA
            self._data["updated_utc_ns"] = time.time_ns()
            self._persist(allow_sealed=True)
            return manifest
        except Exception:
            self._mark_recoverable()
            raise

    def _resume_existing_seal(self, measurement_outcome: Any = None) -> dict[str, Any]:
        """Finish publishing an atomically-written manifest after a later write failed."""

        manifest = _load_json(self.manifest_path)
        if (
            manifest.get("schema") != PACKAGE_SCHEMA
            or manifest.get("session_id") != self.session_id
            or manifest.get("revision") != self.revision
            or int(manifest.get("revision_number") or 0) != self.revision_number
            or str(manifest.get("state") or "") not in _SEALED_STATES
        ):
            raise RuntimeError("existing revision manifest does not match recoverable working state")
        if measurement_outcome is not None and _json_safe(measurement_outcome) != manifest.get(
            "measurement_outcome"
        ):
            raise RuntimeError("existing revision manifest has a different measurement outcome")
        manifest_bytes = self.manifest_path.read_bytes()
        digest = hashlib.sha256(manifest_bytes).hexdigest()
        sidecar_path = self.manifest_path.parent / MANIFEST_SIDECAR_NAME
        sidecar = f"{digest}  {MANIFEST_NAME}\n".encode("ascii")
        if sidecar_path.exists():
            if sidecar_path.read_bytes() != sidecar:
                raise RuntimeError("existing revision sidecar does not match manifest bytes")
        else:
            _atomic_write_bytes(sidecar_path, sidecar, replace=False)
        verification = verify_package(self.root, revision=self.revision_number)
        if not verification.valid:
            codes = ", ".join(issue.code for issue in verification.issues)
            raise RuntimeError(f"existing revision manifest failed verification: {codes}")
        completeness = dict(manifest.get("evidence_completeness") or {})
        latest = {
            "schema": LATEST_REVISION_SCHEMA,
            "session_id": self.session_id,
            "revision": self.revision,
            "revision_number": self.revision_number,
            "manifest_path": self.manifest_path.relative_to(self.root).as_posix(),
            "manifest_sha256": digest,
            "state": str(manifest["state"]),
            "evidence_complete": bool(completeness.get("complete")),
            "updated_utc_ns": time.time_ns(),
        }
        _atomic_write_json(self.root / LATEST_REVISION_NAME, latest)
        self._data = dict(manifest)
        self._data["working_state_schema"] = WORKING_STATE_SCHEMA
        self._data["updated_utc_ns"] = time.time_ns()
        self._persist(allow_sealed=True)
        return manifest

    def create_revision(self) -> "MeasurementPackage":
        """Open a fresh working revision without modifying the sealed predecessor."""

        if self.state not in _SEALED_STATES:
            raise RuntimeError("create_revision requires a sealed package revision")
        verification = verify_package(self.root)
        if not verification.valid:
            raise RuntimeError("cannot extend an invalid package revision chain")
        if verification.revision_number != self.revision_number:
            raise RuntimeError("new revisions may only extend the latest sealed revision")
        latest_path = self.root / LATEST_REVISION_NAME
        latest = _load_json(latest_path)
        if int(latest.get("revision_number") or 0) != self.revision_number:
            raise RuntimeError("new revisions may only extend the latest sealed revision")

        next_number = self.revision_number + 1
        next_dir = self.root / REVISIONS_DIRECTORY_NAME / _revision_name(next_number)
        if next_dir.exists():
            raise FileExistsError(f"next revision directory already exists: {next_dir.name}")
        previous_sha = sha256_file(self.manifest_path)
        artifacts = [
            dict(item)
            for item in self._data.get("artifacts") or []
            if item.get("classification") != "auto_inventory"
            and not (
                item.get("producer") == "workstation.measurement_package"
                and item.get("role") == "report/summary"
            )
        ]
        now = time.time_ns()
        next_data = {
            "schema": PACKAGE_SCHEMA,
            "schema_version": PACKAGE_SCHEMA_VERSION,
            "working_state_schema": WORKING_STATE_SCHEMA,
            "session_id": self.session_id,
            "revision": _revision_name(next_number),
            "revision_number": next_number,
            "state": "COLLECTING",
            "created_utc_ns": now,
            "updated_utc_ns": now,
            "sealed_utc_ns": None,
            "metadata": _json_safe(self._data.get("metadata") or {}),
            "measurement_outcome": _json_safe(self._data.get("measurement_outcome")),
            "source_expectations": _json_safe(self._data.get("source_expectations") or []),
            "artifacts": _json_safe(artifacts),
            "evidence_completeness": _calculate_completeness(
                self._data.get("source_expectations") or [], artifacts
            ),
            "previous_manifest": {
                "revision": self.revision,
                "revision_number": self.revision_number,
                "relative_path": self.manifest_path.relative_to(self.root).as_posix(),
                "sha256": previous_sha,
            },
        }
        _atomic_write_json(self._working_path, next_data)
        return MeasurementPackage.open(self.root)

    def verify(self) -> VerificationResult:
        return verify_package(self.root, revision=self.revision_number)

    def _register_absent(
        self,
        status: str,
        role: str,
        *,
        relative_path: Path | str | None = None,
        classification: str,
        producer: str,
        source: str,
        required: bool,
        media_type: str | None = None,
        schema: str | None = None,
        reason: str | None = None,
        metadata: Mapping[str, Any] | None = None,
        artifact_id: str | None = None,
    ) -> ArtifactEntry:
        self._ensure_mutable()
        if status not in _ARTIFACT_STATUSES - {"present"}:
            raise ValueError("absent artifact status must be missing, failed, or not_applicable")
        relative: str | None = None
        if relative_path not in (None, ""):
            relative = self.resolve_path(relative_path, must_exist=False).relative_to(self.root).as_posix()
        normalized_metadata = dict(_json_safe(metadata or {}))
        if reason is not None:
            normalized_metadata["reason"] = _required_text(reason, "reason")
        entry = ArtifactEntry(
            artifact_id=_required_text(artifact_id or uuid.uuid4(), "artifact_id"),
            role=_required_text(role, "role"),
            classification=_required_text(classification, "classification"),
            relative_path=relative,
            producer=_required_text(producer, "producer"),
            source=_required_text(source, "source"),
            required=bool(required) if status != "not_applicable" else False,
            status=status,
            media_type=(None if media_type is None else _required_text(media_type, "media_type")),
            schema=schema,
            size_bytes=None,
            sha256=None,
            metadata=normalized_metadata,
        )
        self._upsert_artifact(entry)
        return entry

    def _upsert_artifact(self, entry: ArtifactEntry) -> None:
        artifacts = list(self._data.get("artifacts") or [])
        replacement_key = (entry.role, entry.source, entry.relative_path)
        artifacts = [
            item
            for item in artifacts
            if item.get("artifact_id") != entry.artifact_id
            and (item.get("role"), item.get("source"), item.get("relative_path")) != replacement_key
            and not (
                entry.relative_path is not None
                and item.get("relative_path") == entry.relative_path
                and item.get("classification") == "auto_inventory"
            )
        ]
        artifacts.append(entry.to_mapping())
        artifacts.sort(key=_artifact_sort_key)
        self._data["artifacts"] = artifacts
        self._refresh_completeness()
        self._persist()

    def _refresh_completeness(self) -> None:
        self._data["evidence_completeness"] = self._calculate_completeness()

    def _calculate_completeness(self) -> dict[str, Any]:
        return _calculate_completeness(
            self._data.get("source_expectations") or [], self._data.get("artifacts") or []
        )

    def _drop_generated_entries(self) -> None:
        self._data["artifacts"] = [
            item
            for item in self._data.get("artifacts") or []
            if item.get("classification") != "auto_inventory"
            and not (
                item.get("producer") == "workstation.measurement_package"
                and item.get("role") == "report/summary"
            )
        ]

    def _refresh_present_artifacts(self) -> None:
        """Hash the final drained bytes, downgrading disappeared files to missing."""

        refreshed: list[dict[str, Any]] = []
        digests: dict[str, tuple[int, str]] = {}
        for original in self._data.get("artifacts") or []:
            item = dict(original)
            if item.get("status") != "present":
                refreshed.append(item)
                continue
            relative = item.get("relative_path")
            if not relative:
                item.update(status="missing", size_bytes=None, sha256=None)
                refreshed.append(item)
                continue
            path = self.resolve_path(str(relative), must_exist=False)
            item["relative_path"] = path.relative_to(self.root).as_posix()
            if not path.exists():
                item.update(status="missing", size_bytes=None, sha256=None)
                metadata = dict(item.get("metadata") or {})
                metadata["seal_validation"] = "file disappeared before sealing"
                item["metadata"] = metadata
            elif not path.is_file():
                item.update(status="failed", size_bytes=None, sha256=None)
                metadata = dict(item.get("metadata") or {})
                metadata["seal_validation"] = "path is not a regular file"
                item["metadata"] = metadata
            else:
                relative_key = path.relative_to(self.root).as_posix()
                cached = digests.get(relative_key)
                if cached is None:
                    cached = (path.stat().st_size, self._cached_sha256(path))
                    digests[relative_key] = cached
                item["size_bytes"], item["sha256"] = cached
            refreshed.append(item)
        self._data["artifacts"] = sorted(refreshed, key=_artifact_sort_key)

    def _auto_inventory(self) -> None:
        registered = {
            str(item["relative_path"])
            for item in self._data.get("artifacts") or []
            if item.get("status") == "present" and item.get("relative_path")
        }
        for path in _regular_evidence_files(self.root):
            relative = path.relative_to(self.root).as_posix()
            if relative in registered:
                continue
            media = mimetypes.guess_type(path.name)[0] or "application/octet-stream"
            entry = ArtifactEntry(
                artifact_id=f"auto-{hashlib.sha256(relative.encode('utf-8')).hexdigest()[:24]}",
                role=f"auto_inventory/{relative}",
                classification="auto_inventory",
                relative_path=relative,
                producer="workstation.measurement_package",
                source="session_root",
                required=False,
                status="present",
                media_type=media,
                schema=None,
                size_bytes=path.stat().st_size,
                sha256=sha256_file(path),
                metadata=_artifact_storage_metadata(
                    relative,
                    {"registration": "automatic"},
                    storage_class="auto",
                    snapshot_policy=None,
                ),
            )
            self._data["artifacts"].append(entry.to_mapping())
        self._data["artifacts"].sort(key=_artifact_sort_key)

    def _freeze_appendable_evidence(self, target_dir: Path) -> None:
        """Snapshot mutable ledgers so a later revision cannot invalidate this one.

        Large immutable payloads such as video, images, ULogs, and workbooks
        remain in their session-owned paths.  Append-only ledgers are small
        enough to copy and are the files that legitimately gain rows when a
        sealed session is extended with a new evidence revision.
        """

        frozen: list[dict[str, Any]] = []
        snapshot_root = target_dir / "snapshots"
        snapshot_by_source: dict[str, Path] = {}
        for original in self._data.get("artifacts") or []:
            item = dict(original)
            relative_text = item.get("relative_path")
            if item.get("status") != "present" or not relative_text:
                frozen.append(item)
                continue
            relative = Path(str(relative_text))
            if (
                relative.parts[:1] == (REVISIONS_DIRECTORY_NAME,)
                or _snapshot_policy_for_artifact(item) != "copy_on_revision"
            ):
                frozen.append(item)
                continue
            source = self.resolve_path(relative, must_exist=True)
            source_key = relative.as_posix()
            destination = snapshot_by_source.get(source_key)
            if destination is None:
                destination = snapshot_root / relative
                _atomic_copy_file(source, destination, replace=True)
                snapshot_by_source[source_key] = destination
            metadata = dict(item.get("metadata") or {})
            metadata["live_relative_path"] = relative.as_posix()
            metadata["revision_snapshot"] = True
            item.update(
                relative_path=destination.relative_to(self.root).as_posix(),
                size_bytes=destination.stat().st_size,
                sha256=sha256_file(destination),
                metadata=metadata,
            )
            frozen.append(item)
        self._data["artifacts"] = sorted(frozen, key=_artifact_sort_key)

    def _add_generated_summary(self, path: Path) -> None:
        entry = ArtifactEntry(
            artifact_id=f"summary-{self.revision}",
            role="report/summary",
            classification="report",
            relative_path=path.relative_to(self.root).as_posix(),
            producer="workstation.measurement_package",
            source="package_manifest",
            required=False,
            status="present",
            media_type="text/markdown",
            schema=None,
            size_bytes=path.stat().st_size,
            sha256=sha256_file(path),
            metadata={"human_readable": True},
        )
        self._data["artifacts"].append(entry.to_mapping())
        self._data["artifacts"].sort(key=_artifact_sort_key)

    def _summary_text(self) -> str:
        completeness = self._calculate_completeness()
        lines = [
            f"# Measurement evidence package {self.session_id}",
            "",
            f"- Revision: `{self.revision}`",
            f"- Evidence completeness: **{completeness['status']}**",
            f"- Measurement outcome: `{json.dumps(_json_safe(self._data.get('measurement_outcome')), ensure_ascii=False, sort_keys=True)}`",
            "",
            "## Sources",
            "",
            "| Source | Status | Required roles |",
            "| --- | --- | --- |",
        ]
        for source in self._data.get("source_expectations") or []:
            roles = ", ".join(str(role) for role in source.get("required_roles") or []) or "-"
            lines.append(
                f"| {_markdown_cell(source.get('source_id'))} | {_markdown_cell(source.get('status'))} | {_markdown_cell(roles)} |"
            )
        lines.extend(
            [
                "",
                "## Artifacts",
                "",
                "| Role | Source | Status | Required | Path |",
                "| --- | --- | --- | --- | --- |",
            ]
        )
        for artifact in self._data.get("artifacts") or []:
            lines.append(
                "| "
                + " | ".join(
                    _markdown_cell(value)
                    for value in (
                        artifact.get("role"),
                        artifact.get("source"),
                        artifact.get("status"),
                        artifact.get("required"),
                        artifact.get("relative_path") or "-",
                    )
                )
                + " |"
            )
        if completeness["missing_required"]:
            lines.extend(["", "## Missing required evidence", ""])
            for item in completeness["missing_required"]:
                lines.append(
                    f"- `{item['source']}/{item['role']}`: {item['status']}"
                )
        lines.append("")
        return "\n".join(lines)

    def _ensure_mutable(self) -> None:
        if self._historical or self.state in _SEALED_STATES:
            raise RuntimeError("sealed package revisions are immutable; create a new revision")
        if self.state not in _MUTABLE_STATES:
            raise RuntimeError(f"package is not mutable while state is {self.state}")

    def _load_working_state(self) -> dict[str, Any]:
        data = _load_json(self._working_path)
        if data.get("schema") != PACKAGE_SCHEMA:
            raise ValueError("unsupported working package schema")
        if data.get("working_state_schema") != WORKING_STATE_SCHEMA:
            raise ValueError("unsupported package working-state schema")
        return data

    def _persist(self, *, allow_verifying: bool = False, allow_sealed: bool = False) -> None:
        if self._historical:
            raise RuntimeError("historical package revisions are read-only")
        if self.state == "VERIFYING" and not allow_verifying:
            raise RuntimeError("VERIFYING state may only be persisted by seal")
        if self.state in _SEALED_STATES and not allow_sealed:
            raise RuntimeError("sealed state may only be persisted by seal")
        self._data["updated_utc_ns"] = time.time_ns()
        _atomic_write_json(self._working_path, self._data)

    def _mark_recoverable(self) -> None:
        self._data["state"] = "RECOVERABLE"
        self._data["updated_utc_ns"] = time.time_ns()
        try:
            _atomic_write_json(self._working_path, self._data)
        except Exception:
            # Preserve the original sealing exception.  The in-memory state
            # still tells the caller recovery is required.
            pass


def _coalesce_schema(schema: str | None, data_schema: str | None) -> str | None:
    if schema is not None and data_schema is not None and str(schema) != str(data_schema):
        raise ValueError("schema and data_schema disagree")
    value = schema if schema is not None else data_schema
    return None if value is None else _required_text(value, "schema")


def _artifact_sort_key(item: Mapping[str, Any]) -> tuple[str, str, str, str]:
    return (
        str(item.get("source") or ""),
        str(item.get("role") or ""),
        str(item.get("relative_path") or ""),
        str(item.get("artifact_id") or ""),
    )


def _calculate_completeness(
    source_expectations: Iterable[Mapping[str, Any]], artifacts: Iterable[Mapping[str, Any]]
) -> dict[str, Any]:
    sources = list(source_expectations)
    entries = list(artifacts)
    active_sources = {
        str(item.get("source_id"))
        for item in sources
        if str(item.get("status") or "") == "active" or bool(item.get("active"))
    }
    not_applicable_sources = {
        str(item.get("source_id"))
        for item in sources
        if str(item.get("status") or "") == "not_applicable"
    }
    missing: list[dict[str, Any]] = []
    seen: set[tuple[str, str]] = set()

    for expectation in sources:
        source = str(expectation.get("source_id") or "")
        if source not in active_sources:
            continue
        for role_value in expectation.get("required_roles") or []:
            role = str(role_value)
            key = (source, role)
            matches = [
                item
                for item in entries
                if str(item.get("source") or "") == source and str(item.get("role") or "") == role
            ]
            if any(item.get("status") == "present" for item in matches):
                continue
            status = str(matches[-1].get("status") or "missing") if matches else "missing"
            missing.append({"source": source, "role": role, "status": status})
            seen.add(key)

    for artifact in entries:
        if not bool(artifact.get("required")) or artifact.get("status") == "present":
            continue
        source = str(artifact.get("source") or "unknown")
        if source in not_applicable_sources:
            continue
        role = str(artifact.get("role") or "unknown")
        if any(
            str(candidate.get("source") or "") == source
            and str(candidate.get("role") or "") == role
            and candidate.get("status") == "present"
            for candidate in entries
        ):
            # Retain the failed/missing attempt as provenance, but a later
            # exact artifact for the same required source/role satisfies the
            # current revision's completeness contract.
            continue
        key = (source, role)
        if key in seen:
            continue
        missing.append(
            {
                "source": source,
                "role": role,
                "status": str(artifact.get("status") or "missing"),
                "artifact_id": artifact.get("artifact_id"),
            }
        )
        seen.add(key)

    missing.sort(key=lambda item: (str(item["source"]), str(item["role"])))
    complete = not missing
    return {
        "status": "COMPLETE" if complete else "INCOMPLETE",
        "complete": complete,
        "missing_required": missing,
    }


def _resolve_inside(root: Path, value: Path | str, *, must_exist: bool) -> Path:
    try:
        raw = Path(value).expanduser()
        candidate = raw if raw.is_absolute() else root / raw
        resolved = candidate.resolve(strict=False)
        resolved.relative_to(root)
    except (OSError, RuntimeError, ValueError) as exc:
        raise ValueError("artifact path escapes the session root or is invalid") from exc
    if resolved == root:
        raise ValueError("artifact path must identify a file inside the session root")
    if must_exist and not resolved.exists():
        raise FileNotFoundError(resolved)
    return resolved


def _is_bookkeeping_or_temp(relative: Path) -> bool:
    parts = relative.parts
    if not parts:
        return True
    if parts[0] == REVISIONS_DIRECTORY_NAME:
        return True
    if len(parts) == 1 and parts[0] in {WORKING_STATE_NAME, LATEST_REVISION_NAME}:
        return True
    name = parts[-1]
    if name.startswith(f".{MANIFEST_NAME}.") or name.startswith(f".{WORKING_STATE_NAME}."):
        return True
    return name.endswith(_TEMP_SUFFIXES)


def _regular_evidence_files(root: Path) -> tuple[Path, ...]:
    files: list[Path] = []
    for current, directories, names in os.walk(root, followlinks=False):
        current_path = Path(current)
        relative_dir = current_path.relative_to(root)
        directories[:] = [
            name
            for name in directories
            if not _is_bookkeeping_or_temp(relative_dir / name)
            and not (current_path / name).is_symlink()
        ]
        for name in names:
            path = current_path / name
            relative = path.relative_to(root)
            if _is_bookkeeping_or_temp(relative):
                continue
            try:
                mode = path.lstat().st_mode
            except FileNotFoundError:
                continue
            if stat.S_ISREG(mode):
                files.append(path)
    return tuple(sorted(files, key=lambda path: path.relative_to(root).as_posix()))


def _markdown_cell(value: object) -> str:
    return str(value).replace("|", "\\|").replace("\r", " ").replace("\n", " ")


class _Verifier:
    def __init__(self, root: Path) -> None:
        self.root = root
        self.issues: list[VerificationIssue] = []
        self.verified_manifests = 0
        self.verified_artifacts = 0
        self.session_id: str | None = None
        self.revision: str | None = None
        self.revision_number: int | None = None
        self._visited: set[str] = set()

    def issue(self, code: str, message: str, *, path: str | None = None, revision: str | None = None) -> None:
        self.issues.append(VerificationIssue(code, message, path, revision))

    def verify(self, revision: int | str | None) -> VerificationResult:
        manifest_path: Path | None = None
        pointer: dict[str, Any] | None = None
        if revision is None:
            pointer_path = self.root / LATEST_REVISION_NAME
            if not pointer_path.is_file():
                self.issue("latest_pointer_missing", "latest_revision.json is missing", path=LATEST_REVISION_NAME)
            else:
                try:
                    pointer = _load_json(pointer_path)
                    if pointer.get("schema") != LATEST_REVISION_SCHEMA:
                        self.issue("latest_pointer_schema", "latest revision pointer schema is invalid", path=LATEST_REVISION_NAME)
                    manifest_path = _resolve_inside(
                        self.root, str(pointer.get("manifest_path") or ""), must_exist=True
                    )
                except Exception as exc:
                    self.issue("latest_pointer_invalid", str(exc), path=LATEST_REVISION_NAME)
        else:
            try:
                number = _revision_number(revision)
                manifest_path = _resolve_inside(
                    self.root,
                    Path(REVISIONS_DIRECTORY_NAME) / _revision_name(number) / MANIFEST_NAME,
                    must_exist=True,
                )
            except Exception as exc:
                self.issue("manifest_missing", str(exc), revision=str(revision))

        if manifest_path is not None:
            manifest = self._verify_manifest(manifest_path, expected_sha=None)
            if manifest is not None:
                self.session_id = str(manifest.get("session_id") or "") or None
                self.revision = str(manifest.get("revision") or "") or None
                self.revision_number = int(manifest.get("revision_number") or 0) or None
                if pointer is not None:
                    expected = str(pointer.get("manifest_sha256") or "")
                    actual = sha256_file(manifest_path)
                    if expected != actual:
                        self.issue(
                            "latest_pointer_hash_mismatch",
                            "latest pointer does not match manifest bytes",
                            path=LATEST_REVISION_NAME,
                            revision=self.revision,
                        )
                    if pointer.get("session_id") != manifest.get("session_id"):
                        self.issue("latest_pointer_session_mismatch", "latest pointer session_id is stale")
                    if int(pointer.get("revision_number") or 0) != int(manifest.get("revision_number") or 0):
                        self.issue("latest_pointer_revision_mismatch", "latest pointer revision is stale")
                if revision is None:
                    self._verify_no_unlisted_evidence(manifest)

        return VerificationResult(
            valid=not self.issues,
            state="VERIFIED" if not self.issues else "INTEGRITY_FAILED",
            session_id=self.session_id,
            revision=self.revision,
            revision_number=self.revision_number,
            verified_manifests=self.verified_manifests,
            verified_artifacts=self.verified_artifacts,
            issues=tuple(self.issues),
        )

    def _verify_no_unlisted_evidence(self, manifest: Mapping[str, Any]) -> None:
        listed: set[str] = set()
        for artifact in manifest.get("artifacts") or []:
            if not isinstance(artifact, Mapping) or artifact.get("status") != "present":
                continue
            relative = artifact.get("relative_path")
            if isinstance(relative, str) and relative:
                listed.add(relative)
            metadata = artifact.get("metadata")
            if isinstance(metadata, Mapping):
                live = metadata.get("live_relative_path")
                if isinstance(live, str) and live:
                    listed.add(live)
        for path in _regular_evidence_files(self.root):
            relative = path.relative_to(self.root).as_posix()
            if relative not in listed:
                self.issue(
                    "unlisted_evidence",
                    "session contains a regular evidence file absent from the latest manifest",
                    path=relative,
                    revision=str(manifest.get("revision") or ""),
                )

    def _verify_manifest(self, path: Path, *, expected_sha: str | None) -> dict[str, Any] | None:
        relative_path = path.relative_to(self.root).as_posix()
        if relative_path in self._visited:
            self.issue("revision_chain_cycle", "revision chain contains a cycle", path=relative_path)
            return None
        self._visited.add(relative_path)
        revision = path.parent.name
        try:
            data = _load_json(path)
        except Exception as exc:
            self.issue("manifest_invalid", str(exc), path=relative_path, revision=revision)
            return None
        self.verified_manifests += 1
        actual_sha = sha256_file(path)
        if expected_sha is not None and expected_sha != actual_sha:
            self.issue(
                "revision_chain_hash_mismatch",
                "previous manifest hash does not match the child revision",
                path=relative_path,
                revision=revision,
            )
        sidecar_path = path.parent / MANIFEST_SIDECAR_NAME
        if not sidecar_path.is_file():
            self.issue("manifest_sidecar_missing", "manifest SHA-256 sidecar is missing", path=sidecar_path.relative_to(self.root).as_posix(), revision=revision)
        else:
            try:
                sidecar_sha = sidecar_path.read_text(encoding="ascii").strip().split()[0]
            except Exception as exc:
                self.issue("manifest_sidecar_invalid", str(exc), path=sidecar_path.relative_to(self.root).as_posix(), revision=revision)
            else:
                if sidecar_sha != actual_sha:
                    self.issue("manifest_sidecar_mismatch", "manifest SHA-256 sidecar does not match", path=sidecar_path.relative_to(self.root).as_posix(), revision=revision)

        if data.get("schema") != PACKAGE_SCHEMA or int(data.get("schema_version") or 0) != PACKAGE_SCHEMA_VERSION:
            self.issue("manifest_schema", "unsupported package manifest schema", path=relative_path, revision=revision)
        if data.get("revision") != revision:
            self.issue("manifest_revision", "manifest revision does not match its directory", path=relative_path, revision=revision)
        session_id = str(data.get("session_id") or "")
        if self.session_id is not None and session_id != self.session_id:
            self.issue("revision_chain_session_mismatch", "revision chain crosses session_id values", path=relative_path, revision=revision)
        elif self.session_id is None:
            self.session_id = session_id or None

        for artifact in data.get("artifacts") or []:
            if not isinstance(artifact, Mapping):
                self.issue("artifact_entry_invalid", "artifact entry is not an object", path=relative_path, revision=revision)
                continue
            self._verify_artifact(artifact, revision)

        expected_completeness = _calculate_completeness(
            data.get("source_expectations") or [], data.get("artifacts") or []
        )
        if data.get("evidence_completeness") != expected_completeness:
            self.issue("completeness_mismatch", "stored evidence completeness is inconsistent", path=relative_path, revision=revision)
        expected_state = "SEALED_COMPLETE" if expected_completeness["complete"] else "SEALED_INCOMPLETE"
        if data.get("state") != expected_state:
            self.issue("sealed_state_mismatch", "sealed state does not match evidence completeness", path=relative_path, revision=revision)

        previous = data.get("previous_manifest")
        number = int(data.get("revision_number") or 0)
        if previous is None:
            if number != 1:
                self.issue("revision_chain_missing", "non-genesis revision has no previous manifest", path=relative_path, revision=revision)
        elif isinstance(previous, Mapping):
            try:
                previous_path = _resolve_inside(
                    self.root, str(previous.get("relative_path") or ""), must_exist=True
                )
                expected_previous_number = int(previous.get("revision_number") or 0)
                if expected_previous_number != number - 1:
                    self.issue("revision_chain_order", "previous revision is not the immediate predecessor", path=relative_path, revision=revision)
                if previous_path.parent.name != str(previous.get("revision") or ""):
                    self.issue("revision_chain_path", "previous revision path and id disagree", path=relative_path, revision=revision)
                self._verify_manifest(previous_path, expected_sha=str(previous.get("sha256") or ""))
            except Exception as exc:
                self.issue("revision_chain_path_escape", str(exc), path=relative_path, revision=revision)
        else:
            self.issue("revision_chain_invalid", "previous_manifest must be an object or null", path=relative_path, revision=revision)
        return data

    def _verify_artifact(self, artifact: Mapping[str, Any], revision: str) -> None:
        if artifact.get("status") != "present":
            return
        relative = artifact.get("relative_path")
        if not isinstance(relative, str) or not relative:
            self.issue("artifact_path_missing", "present artifact has no relative path", revision=revision)
            return
        try:
            raw = Path(relative)
            candidate = raw if raw.is_absolute() else self.root / raw
            resolved = candidate.resolve(strict=False)
            resolved.relative_to(self.root)
        except Exception:
            self.issue("artifact_path_escape", "artifact path escapes the session root", path=relative, revision=revision)
            return
        canonical = resolved.relative_to(self.root).as_posix()
        if raw.is_absolute() or canonical != relative:
            self.issue("artifact_path_noncanonical", "artifact path is not its authoritative resolved relative path", path=relative, revision=revision)
            return
        if not resolved.exists():
            self.issue("artifact_missing", "listed artifact is missing", path=relative, revision=revision)
            return
        if not resolved.is_file():
            self.issue("artifact_not_regular", "listed artifact is not a regular file", path=relative, revision=revision)
            return
        actual_size = resolved.stat().st_size
        recorded_size = artifact.get("size_bytes")
        if recorded_size is None or int(recorded_size) != actual_size:
            self.issue("artifact_size_mismatch", "artifact size does not match manifest", path=relative, revision=revision)
            return
        actual_sha = sha256_file(resolved)
        if str(artifact.get("sha256") or "") != actual_sha:
            self.issue("artifact_hash_mismatch", "artifact SHA-256 does not match manifest", path=relative, revision=revision)
            return
        self.verified_artifacts += 1


def verify_package(root: Path | str, revision: int | str | None = None) -> VerificationResult:
    """Verify a sealed revision, every listed artifact, and its full hash chain."""

    resolved = Path(root).expanduser().resolve()
    if not resolved.is_dir():
        issue = VerificationIssue("package_root_missing", "measurement package root is missing", str(resolved))
        return VerificationResult(False, "INTEGRITY_FAILED", None, None, None, 0, 0, (issue,))
    return _Verifier(resolved).verify(revision)


__all__ = [
    "ArtifactEntry",
    "LATEST_REVISION_NAME",
    "LATEST_REVISION_SCHEMA",
    "MANIFEST_NAME",
    "MANIFEST_SIDECAR_NAME",
    "MeasurementPackage",
    "PACKAGE_SCHEMA",
    "PACKAGE_SCHEMA_VERSION",
    "REVISIONS_DIRECTORY_NAME",
    "SUMMARY_NAME",
    "VerificationIssue",
    "VerificationResult",
    "WORKING_STATE_NAME",
    "WORKING_STATE_SCHEMA",
    "canonical_json_bytes",
    "sha256_file",
    "verify_package",
]
