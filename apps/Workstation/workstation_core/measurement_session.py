"""Explicit Workstation sessions containing independent measurement packages.

A :class:`MeasurementSession` is the operator-owned container created from the
Measurement workspace.  It owns stable session metadata and an ordered list of
child :class:`~Workstation.workstation_core.measurement_package.MeasurementPackage`
directories.  Camera/video/spool/result files remain measurement-owned, under
``measurements/<measurement_id>``.

The sealed session manifest does not duplicate child evidence.  It records the
relative path and SHA-256 of each latest child manifest, allowing a portable
session to prove both membership and each measurement's existing integrity
chain.
"""

from __future__ import annotations

import hashlib
import json
import math
import os
import subprocess
import time
import uuid
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Mapping

from Workstation.workstation_core.measurement_package import (
    LATEST_REVISION_NAME,
    MeasurementPackage,
    canonical_json_bytes,
    sha256_file,
    verify_package,
)


SESSION_SCHEMA = "workstation-measurement-session-v1"
SESSION_SCHEMA_VERSION = 2
LEGACY_SESSION_SCHEMA_VERSIONS = frozenset({1})
SESSION_STATE_NAME = "session.json"
SESSION_MANIFEST_NAME = "measurement_session_manifest.json"
SESSION_MANIFEST_SIDECAR_NAME = "measurement_session_manifest.sha256"
MEASUREMENTS_DIRECTORY_NAME = "measurements"

_OPEN_STATES = frozenset({"OPEN"})
_SEALED_MEASUREMENT_STATES = frozenset({"SEALED_COMPLETE", "SEALED_INCOMPLETE"})
_MUTABLE_MEASUREMENT_STATES = frozenset(
    {"COLLECTING", "POST_CAPTURE", "VERIFYING", "RECOVERABLE"}
)


def _software_summary() -> dict[str, Any]:
    repository = Path(__file__).resolve().parents[3]

    def git(*arguments: str) -> str | None:
        try:
            result = subprocess.run(
                ["git", *arguments],
                cwd=repository,
                stdout=subprocess.PIPE,
                stderr=subprocess.DEVNULL,
                check=False,
                timeout=2.0,
            )
        except (OSError, subprocess.SubprocessError):
            return None
        return (
            result.stdout.decode("utf-8", errors="replace").strip()
            if result.returncode == 0
            else None
        )

    status = git("status", "--porcelain=v1", "--untracked-files=normal")
    return {
        "branch": git("branch", "--show-current"),
        "commit": git("rev-parse", "HEAD"),
        "dirty": None if status is None else bool(status),
        "session_schema": SESSION_SCHEMA,
        "session_schema_version": SESSION_SCHEMA_VERSION,
    }


def _session_static_metadata(
    session_id: str,
    metadata: Mapping[str, Any] | None,
) -> dict[str, Any]:
    from Workstation.workstation_core.package_provenance import redact_secrets

    normalized = dict(_json_safe(redact_secrets(dict(metadata or {}))))
    existing_identity = normalized.get("identity")
    existing_identity = (
        dict(existing_identity) if isinstance(existing_identity, Mapping) else {}
    )
    experiment_type = existing_identity.get(
        "experiment_type",
        normalized.get("experiment_type"),
    )
    normalized["identity"] = {
        **existing_identity,
        "session_id": session_id,
        "application": "Workstation",
        "experiment_type": experiment_type,
    }
    normalized["software"] = _software_summary()
    normalized.setdefault(
        "recording_intervals",
        {"owner": "ordered child MeasurementPackage mNNNNNN entries"},
    )
    normalized.setdefault(
        "camera",
        {
            "requested_zoom": None,
            "observed_zoom": None,
            "requested_resolution_px": None,
            "requested_fps": None,
            "null_contract": "not observed at parent Session creation",
        },
    )
    normalized.setdefault("devices", {"observed_identities": []})
    normalized["credentials"] = {
        "secret_values_persisted": False,
        "allowed_reference": "credential environment-variable name only",
    }
    return normalized


def _json_safe(value: Any) -> Any:
    if value is None or isinstance(value, (bool, int, str)):
        return value
    if isinstance(value, float):
        if not math.isfinite(value):
            raise ValueError("session data contains a non-finite number")
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
    raise TypeError(f"unsupported session value: {type(value).__name__}")


def _required_component(value: object, field: str) -> str:
    text = str(value or "").strip()
    if not text:
        raise ValueError(f"{field} is required")
    if (
        text in {".", ".."}
        or Path(text).name != text
        or "\\" in text
        or "\x00" in text
    ):
        raise ValueError(f"{field} must be one safe path component")
    return text


def _load_json(path: Path) -> dict[str, Any]:
    def reject_constant(value: str) -> None:
        raise ValueError(f"non-finite JSON constant {value}")

    with path.open("r", encoding="utf-8") as handle:
        value = json.load(handle, parse_constant=reject_constant)
    if not isinstance(value, dict):
        raise ValueError(f"{path.name} must contain a JSON object")
    return dict(_json_safe(value))


def _fsync_directory(path: Path) -> None:
    descriptor = os.open(path, os.O_RDONLY)
    try:
        os.fsync(descriptor)
    finally:
        os.close(descriptor)


def _atomic_write_bytes(path: Path, payload: bytes, *, replace: bool = True) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_name(f".{path.name}.{uuid.uuid4().hex}.tmp")
    try:
        with temporary.open("xb") as handle:
            handle.write(payload)
            handle.flush()
            os.fsync(handle.fileno())
        if not replace and path.exists():
            raise FileExistsError(f"refusing to replace immutable file: {path}")
        os.replace(temporary, path)
        _fsync_directory(path.parent)
    finally:
        temporary.unlink(missing_ok=True)


def _atomic_write_json(path: Path, value: Mapping[str, Any], *, replace: bool = True) -> None:
    _atomic_write_bytes(path, canonical_json_bytes(dict(_json_safe(value))) + b"\n", replace=replace)


def _safe_child(root: Path, value: object, *, must_exist: bool = False) -> Path:
    text = str(value or "")
    raw = Path(text)
    if (
        raw.is_absolute()
        or "\\" in text
        or not raw.parts
        or any(part in {"", ".", ".."} for part in raw.parts)
    ):
        raise ValueError("measurement path must be a canonical relative path")
    resolved = (root / raw).resolve(strict=False)
    try:
        resolved.relative_to(root)
    except ValueError as exc:
        raise ValueError("measurement path escapes the session root") from exc
    if must_exist and not resolved.exists():
        raise FileNotFoundError(resolved)
    return resolved


def _recording_generation_summary(
    measurement_root: Path,
    package_snapshot: Mapping[str, Any],
) -> dict[str, object]:
    measurement_manifest_path = measurement_root / "measurement.json"
    if not measurement_manifest_path.is_file():
        return {
            "recording_generation": None,
            "authoritative_jsonl_fragments": [],
            "context_fragments": [],
            "binary_artifacts": [],
        }
    resolved_manifest = measurement_manifest_path.resolve(strict=True)
    resolved_manifest.relative_to(measurement_root)
    manifest = _load_json(measurement_manifest_path)
    details = manifest.get("artifact_details")
    details = details if isinstance(details, Mapping) else {}
    authoritative_jsonl: list[dict[str, object]] = []
    for key, value in details.items():
        if not isinstance(value, Mapping):
            continue
        if (
            value.get("authoritative") is not True
            or value.get("media_type") != "application/x-ndjson"
        ):
            continue
        authoritative_jsonl.append(
            {
                "artifact_key": str(key),
                "role": str(value.get("role") or ""),
                "relative_path": str(value.get("relative_path") or ""),
                "size_bytes": int(value.get("size_bytes") or 0),
                "sha256": str(value.get("sha256") or ""),
                "row_count": int(value.get("row_count") or 0),
                "schema_version": value.get("schema_version"),
            }
        )
    authoritative_jsonl.sort(
        key=lambda row: (str(row["role"]), str(row["relative_path"]))
    )
    current = measurement_root / ".workstation" / "publish" / "current"
    current_target = ""
    if current.is_symlink():
        target = Path(os.readlink(current))
        if (
            not target.is_absolute()
            and len(target.parts) == 2
            and target.parts[0] == "generations"
            and target.parts[1] not in {"", ".", ".."}
        ):
            current_target = target.as_posix()

    binary_artifacts: list[dict[str, object]] = []
    binary_suffixes = {
        ".bin",
        ".jpg",
        ".jpeg",
        ".mp4",
        ".png",
        ".rtcm",
        ".tif",
        ".tiff",
        ".ulg",
    }
    for raw in package_snapshot.get("artifacts", ()) or ():
        if not isinstance(raw, Mapping) or raw.get("status") != "present":
            continue
        relative = str(raw.get("relative_path") or "")
        media_type = str(raw.get("media_type") or "")
        if (
            Path(relative).suffix.lower() not in binary_suffixes
            and not media_type.startswith(("image/", "video/"))
            and media_type
            not in {"application/octet-stream", "application/vnd.px4.ulog"}
        ):
            continue
        binary_artifacts.append(
            {
                "role": str(raw.get("role") or ""),
                "relative_path": relative,
                "media_type": media_type,
                "size_bytes": int(raw.get("size_bytes") or 0),
                "sha256": str(raw.get("sha256") or ""),
            }
        )
    binary_artifacts.sort(
        key=lambda row: (str(row["role"]), str(row["relative_path"]))
    )
    return {
        "recording_generation": {
            "storage_schema": str(manifest.get("schema") or ""),
            "storage_schema_version": manifest.get("schema_version"),
            "generation_id": str(manifest.get("generation_id") or ""),
            "current_pointer_target": current_target,
            "measurement_manifest_relative_path": "measurement.json",
            "measurement_manifest_sha256": sha256_file(resolved_manifest),
        },
        "authoritative_jsonl_fragments": authoritative_jsonl,
        "context_fragments": [
            dict(row)
            for row in authoritative_jsonl
            if str(row["role"]).startswith("context/")
        ],
        "binary_artifacts": binary_artifacts,
    }


@dataclass(frozen=True)
class SessionVerificationIssue:
    code: str
    message: str
    path: str | None = None
    measurement_id: str | None = None

    def to_mapping(self) -> dict[str, Any]:
        return {
            "code": self.code,
            "message": self.message,
            "path": self.path,
            "measurement_id": self.measurement_id,
        }


@dataclass(frozen=True)
class SessionVerificationResult:
    valid: bool
    state: str
    session_id: str | None
    verified_measurements: int
    issues: tuple[SessionVerificationIssue, ...]

    @property
    def ok(self) -> bool:
        return self.valid

    def __bool__(self) -> bool:
        return self.valid

    def to_mapping(self) -> dict[str, Any]:
        return {
            "valid": self.valid,
            "state": self.state,
            "session_id": self.session_id,
            "verified_measurements": self.verified_measurements,
            "issues": [issue.to_mapping() for issue in self.issues],
        }


class MeasurementSession:
    """Mutable session index and immutable aggregate manifest."""

    def __init__(self, root: Path | str) -> None:
        self.root = Path(root).expanduser().resolve()
        self._state_path = self.root / SESSION_STATE_NAME
        if not self._state_path.is_file():
            raise FileNotFoundError(f"no measurement session exists at {self.root}")
        self._data = _load_json(self._state_path)
        if (
            self._data.get("schema") != SESSION_SCHEMA
            or int(self._data.get("schema_version") or 0)
            not in LEGACY_SESSION_SCHEMA_VERSIONS | {SESSION_SCHEMA_VERSION}
        ):
            raise ValueError("unsupported measurement session schema")
        if str(self._data.get("state") or "") != "SEALED":
            self.reconcile()

    @classmethod
    def create(
        cls,
        root: Path | str,
        *,
        session_id: str,
        metadata: Mapping[str, Any] | None = None,
        app_run_id: str | None = None,
    ) -> "MeasurementSession":
        resolved = Path(root).expanduser().resolve()
        normalized_session_id = _required_component(session_id, "session_id")
        if resolved.name != normalized_session_id:
            raise ValueError("session root directory must match session_id")
        if (resolved / SESSION_STATE_NAME).exists() or (resolved / SESSION_MANIFEST_NAME).exists():
            raise FileExistsError("a measurement session already exists at this root")
        measurements_root = resolved / MEASUREMENTS_DIRECTORY_NAME
        if measurements_root.exists() and any(measurements_root.iterdir()):
            raise FileExistsError("measurement directories already exist at this session root")
        resolved.mkdir(parents=True, exist_ok=True)
        now = time.time_ns()
        run_id = _required_component(app_run_id, "app_run_id") if app_run_id else ""
        data: dict[str, Any] = {
            "schema": SESSION_SCHEMA,
            "schema_version": SESSION_SCHEMA_VERSION,
            "session_id": normalized_session_id,
            "state": "OPEN",
            "created_utc_ns": now,
            "updated_utc_ns": now,
            "closing_utc_ns": None,
            "sealed_utc_ns": None,
            "metadata": _session_static_metadata(
                normalized_session_id,
                metadata,
            ),
            "app_runs": (
                [{"app_run_id": run_id, "joined_utc_ns": now}] if run_id else []
            ),
            "active_measurement_id": None,
            "measurements": [],
        }
        _atomic_write_json(resolved / SESSION_STATE_NAME, data, replace=False)
        return cls(resolved)

    @classmethod
    def open(cls, root: Path | str) -> "MeasurementSession":
        return cls(root)

    @property
    def session_id(self) -> str:
        return str(self._data["session_id"])

    @property
    def state(self) -> str:
        return str(self._data["state"])

    @property
    def active_measurement_id(self) -> str | None:
        value = self._data.get("active_measurement_id")
        return None if value in (None, "") else str(value)

    @property
    def measurements_root(self) -> Path:
        return self.root / MEASUREMENTS_DIRECTORY_NAME

    @property
    def manifest_path(self) -> Path:
        return self.root / SESSION_MANIFEST_NAME

    def snapshot(self) -> dict[str, Any]:
        return json.loads(canonical_json_bytes(self._data).decode("utf-8"))

    def add_app_run(self, app_run_id: str) -> dict[str, Any]:
        if self.state not in _OPEN_STATES:
            raise RuntimeError("sealed/closing sessions cannot accept another app run")
        run_id = _required_component(app_run_id, "app_run_id")
        rows = list(self._data.get("app_runs") or [])
        existing = next((row for row in rows if row.get("app_run_id") == run_id), None)
        if existing is not None:
            return dict(_json_safe(existing))
        row = {"app_run_id": run_id, "joined_utc_ns": time.time_ns()}
        rows.append(row)
        self._data["app_runs"] = rows
        self._persist()
        return dict(_json_safe(row))

    def measurement_root(self, measurement_id: str | None = None) -> Path:
        selected = measurement_id or self.active_measurement_id
        if not selected:
            raise RuntimeError("no active measurement is selected")
        component = _required_component(selected, "measurement_id")
        return self.measurements_root / component

    def measurement_package(self, measurement_id: str | None = None) -> MeasurementPackage:
        return MeasurementPackage.open(self.measurement_root(measurement_id))

    def begin_measurement(
        self,
        *,
        measurement_id: str | None = None,
        metadata: Mapping[str, Any] | None = None,
    ) -> MeasurementPackage:
        if self.state != "OPEN":
            raise RuntimeError(f"session is not open while state is {self.state}")
        self.reconcile()
        if self.active_measurement_id is not None:
            raise RuntimeError(
                f"measurement {self.active_measurement_id} must be sealed before starting another"
            )
        rows = list(self._data.get("measurements") or [])
        reserved_sequences = [int(row.get("sequence") or 0) for row in rows]
        if self.measurements_root.is_dir():
            for child in self.measurements_root.iterdir():
                name = child.name
                if child.is_dir() and name.startswith("m") and name[1:].isdigit():
                    reserved_sequences.append(int(name[1:]))
        sequence = max(reserved_sequences, default=0) + 1
        selected = _required_component(
            measurement_id or f"m{sequence:06d}", "measurement_id"
        )
        if any(str(row.get("measurement_id") or "") == selected for row in rows):
            raise FileExistsError(f"measurement already exists: {selected}")
        root = self.measurements_root / selected
        if root.exists():
            raise FileExistsError(f"measurement directory already exists: {selected}")
        measurement_metadata = {
            **dict(_json_safe(metadata or {})),
            "measurement_id": selected,
            "measurement_package_id": selected,
            "measurement_sequence": sequence,
            "parent_session_id": self.session_id,
        }
        package = MeasurementPackage.create(
            root,
            session_id=self.session_id,
            metadata=measurement_metadata,
        )
        row = self._measurement_row(package, selected, sequence=sequence)
        rows.append(row)
        rows.sort(key=lambda item: (int(item.get("sequence") or 0), str(item.get("measurement_id"))))
        self._data["measurements"] = rows
        self._data["active_measurement_id"] = selected
        self._persist()
        return package

    def refresh_measurement(
        self,
        measurement_id: str | None = None,
        *,
        package: MeasurementPackage | None = None,
    ) -> dict[str, Any]:
        selected = _required_component(
            measurement_id or self.active_measurement_id, "measurement_id"
        )
        package = package or self.measurement_package(selected)
        expected_root = self.measurement_root(selected).resolve()
        if package.root.resolve() != expected_root:
            raise ValueError("measurement package root does not match measurement_id")
        if package.session_id != self.session_id:
            raise ValueError("measurement package belongs to a different session")
        rows = list(self._data.get("measurements") or [])
        existing = next(
            (row for row in rows if str(row.get("measurement_id") or "") == selected),
            None,
        )
        sequence = int(existing.get("sequence") or 0) if isinstance(existing, Mapping) else 0
        if sequence < 1:
            sequence = max((int(row.get("sequence") or 0) for row in rows), default=0) + 1
        updated = self._measurement_row(package, selected, sequence=sequence)
        rows = [row for row in rows if str(row.get("measurement_id") or "") != selected]
        rows.append(updated)
        rows.sort(key=lambda item: (int(item.get("sequence") or 0), str(item.get("measurement_id"))))
        self._data["measurements"] = rows
        if package.state in _SEALED_MEASUREMENT_STATES and self.active_measurement_id == selected:
            self._data["active_measurement_id"] = None
        elif package.state in _MUTABLE_MEASUREMENT_STATES:
            active = self.active_measurement_id
            if active not in (None, selected):
                raise RuntimeError(f"multiple mutable measurements detected: {active}, {selected}")
            self._data["active_measurement_id"] = selected
        self._persist()
        return dict(_json_safe(updated))

    def reconcile(self) -> None:
        """Adopt child packages and repair a stale active-measurement pointer."""

        rows_by_id = {
            str(row.get("measurement_id") or ""): dict(row)
            for row in self._data.get("measurements") or []
            if isinstance(row, Mapping) and row.get("measurement_id")
        }
        mutable: list[str] = []
        changed = False
        if self.measurements_root.is_dir():
            for child in sorted(self.measurements_root.iterdir(), key=lambda path: path.name):
                if not child.is_dir() or child.is_symlink():
                    continue
                if not (
                    (child / "package_working_state.json").is_file()
                    or (child / LATEST_REVISION_NAME).is_file()
                ):
                    continue
                package = MeasurementPackage.open(child)
                if package.session_id != self.session_id:
                    raise ValueError(f"measurement {child.name} belongs to another session")
                existing = rows_by_id.get(child.name)
                metadata = package.snapshot().get("metadata") or {}
                sequence = int(
                    (existing or {}).get("sequence")
                    or (metadata.get("measurement_sequence") if isinstance(metadata, Mapping) else 0)
                    or 0
                )
                if sequence < 1:
                    sequence = max(
                        (int(row.get("sequence") or 0) for row in rows_by_id.values()),
                        default=0,
                    ) + 1
                refreshed = self._measurement_row(package, child.name, sequence=sequence)
                if refreshed != existing:
                    rows_by_id[child.name] = refreshed
                    changed = True
                if package.state in _MUTABLE_MEASUREMENT_STATES:
                    mutable.append(child.name)
        if len(mutable) > 1:
            raise RuntimeError("multiple mutable measurement packages require manual recovery")
        active = mutable[0] if mutable else None
        if self._data.get("active_measurement_id") != active:
            self._data["active_measurement_id"] = active
            changed = True
        ordered = sorted(
            rows_by_id.values(),
            key=lambda item: (int(item.get("sequence") or 0), str(item.get("measurement_id"))),
        )
        if ordered != list(self._data.get("measurements") or []):
            self._data["measurements"] = ordered
            changed = True
        if changed:
            self._persist()

    def seal(self) -> dict[str, Any]:
        """Seal the session membership after every child measurement is sealed."""

        if self.state == "SEALED":
            return _load_json(self.manifest_path)
        if self.state not in {"OPEN", "CLOSING"}:
            raise RuntimeError(f"session cannot be sealed while state is {self.state}")
        self.reconcile()
        if self.active_measurement_id is not None:
            raise RuntimeError(
                f"active measurement {self.active_measurement_id} must be sealed first"
            )
        invalid = [
            str(row.get("measurement_id") or "unknown")
            for row in self._data.get("measurements") or []
            if str(row.get("state") or "") not in _SEALED_MEASUREMENT_STATES
        ]
        if invalid:
            raise RuntimeError("unsealed measurements prevent session sealing: " + ", ".join(invalid))
        declared_ids = {
            str(row.get("measurement_id") or "")
            for row in self._data.get("measurements") or []
            if isinstance(row, Mapping)
        }
        if self.measurements_root.is_dir():
            for child in self.measurements_root.iterdir():
                if child.is_symlink() or not child.is_dir():
                    raise RuntimeError(
                        f"unexpected measurement entry prevents session sealing: {child.name}"
                    )
                if child.name not in declared_ids or not (
                    (child / "package_working_state.json").is_file()
                    or (child / LATEST_REVISION_NAME).is_file()
                ):
                    raise RuntimeError(
                        f"orphan measurement directory prevents session sealing: {child.name}"
                    )
        if self.state == "OPEN":
            self._data["state"] = "CLOSING"
            self._data["closing_utc_ns"] = time.time_ns()
            self._persist()

        manifest = self.snapshot()
        manifest["state"] = "SEALED"
        manifest["sealed_utc_ns"] = int(manifest.get("closing_utc_ns") or time.time_ns())
        manifest["shared_artifacts"] = self._shared_artifacts()
        measurement_rows = list(manifest.get("measurements") or [])
        complete_count = sum(bool(row.get("evidence_complete")) for row in measurement_rows)
        manifest["measurement_summary"] = {
            "measurement_count": len(measurement_rows),
            "complete_count": complete_count,
            "incomplete_count": len(measurement_rows) - complete_count,
            "evidence_complete": complete_count == len(measurement_rows),
        }
        manifest.pop("updated_utc_ns", None)
        manifest_bytes = canonical_json_bytes(manifest) + b"\n"
        digest = hashlib.sha256(manifest_bytes).hexdigest()
        if self.manifest_path.exists():
            existing = self.manifest_path.read_bytes()
            if existing != manifest_bytes:
                raise RuntimeError("existing session manifest disagrees with recoverable close state")
        else:
            _atomic_write_bytes(self.manifest_path, manifest_bytes, replace=False)
        sidecar_path = self.root / SESSION_MANIFEST_SIDECAR_NAME
        sidecar = f"{digest}  {SESSION_MANIFEST_NAME}\n".encode("ascii")
        if sidecar_path.exists():
            if sidecar_path.read_bytes() != sidecar:
                raise RuntimeError("existing session manifest sidecar disagrees with manifest")
        else:
            _atomic_write_bytes(sidecar_path, sidecar, replace=False)
        self._data = dict(manifest)
        self._persist(allow_sealed=True)
        return manifest

    def verify(self) -> SessionVerificationResult:
        return verify_measurement_session(self.root)

    def _measurement_row(
        self,
        package: MeasurementPackage,
        measurement_id: str,
        *,
        sequence: int,
    ) -> dict[str, Any]:
        snapshot = package.snapshot()
        state = str(snapshot.get("state") or package.state)
        row: dict[str, Any] = {
            "measurement_id": measurement_id,
            "measurement_package_id": measurement_id,
            "sequence": int(sequence),
            "relative_path": (
                Path(MEASUREMENTS_DIRECTORY_NAME) / measurement_id
            ).as_posix(),
            "state": state,
            "created_utc_ns": int(snapshot.get("created_utc_ns") or 0),
            "updated_utc_ns": int(snapshot.get("updated_utc_ns") or time.time_ns()),
            "sealed_utc_ns": snapshot.get("sealed_utc_ns"),
            "revision": str(snapshot.get("revision") or package.revision),
            "revision_number": int(snapshot.get("revision_number") or package.revision_number),
            "evidence_complete": bool(
                dict(snapshot.get("evidence_completeness") or {}).get("complete", False)
            ),
            "measurement_outcome": _json_safe(snapshot.get("measurement_outcome")),
            "manifest_relative_path": None,
            "manifest_sha256": None,
            **_recording_generation_summary(package.root, snapshot),
        }
        if state in _SEALED_MEASUREMENT_STATES:
            verification = verify_package(package.root)
            if not verification.valid:
                codes = ", ".join(issue.code for issue in verification.issues)
                raise RuntimeError(f"measurement {measurement_id} failed verification: {codes}")
            manifest_path = package.manifest_path
            row["manifest_relative_path"] = manifest_path.relative_to(self.root).as_posix()
            row["manifest_sha256"] = sha256_file(manifest_path)
        return row

    def _shared_artifacts(self) -> list[dict[str, Any]]:
        """Inventory session-owned files outside child measurement roots."""

        excluded_names = {
            SESSION_STATE_NAME,
            SESSION_MANIFEST_NAME,
            SESSION_MANIFEST_SIDECAR_NAME,
        }
        rows: list[dict[str, Any]] = []
        for current, directories, names in os.walk(self.root, followlinks=False):
            current_path = Path(current)
            relative_dir = current_path.relative_to(self.root)
            directories[:] = [
                name
                for name in directories
                if not (
                    relative_dir == Path(".")
                    and name == MEASUREMENTS_DIRECTORY_NAME
                )
            ]
            for directory in tuple(directories):
                if (current_path / directory).is_symlink():
                    raise ValueError("session shared evidence directories must not be symlinks")
            for name in names:
                path = current_path / name
                relative = path.relative_to(self.root)
                if len(relative.parts) == 1 and name in excluded_names:
                    continue
                if name.endswith((".tmp", ".part", ".partial", ".lock")):
                    continue
                if path.is_symlink():
                    raise ValueError("session shared evidence files must not be symlinks")
                if not path.is_file():
                    continue
                rows.append(
                    {
                        "artifact_id": (
                            "session-shared-"
                            + hashlib.sha256(relative.as_posix().encode("utf-8")).hexdigest()[:24]
                        ),
                        "role": f"session_shared/{relative.as_posix()}",
                        "relative_path": relative.as_posix(),
                        "size_bytes": path.stat().st_size,
                        "sha256": sha256_file(path),
                    }
                )
        rows.sort(key=lambda row: str(row["relative_path"]))
        return rows

    def _persist(self, *, allow_sealed: bool = False) -> None:
        if self.state == "SEALED" and not allow_sealed:
            raise RuntimeError("sealed sessions are immutable")
        self._data["updated_utc_ns"] = time.time_ns()
        _atomic_write_json(self._state_path, self._data)


def verify_measurement_session(root: Path | str) -> SessionVerificationResult:
    resolved = Path(root).expanduser().resolve()
    issues: list[SessionVerificationIssue] = []
    verified = 0
    manifest_path = resolved / SESSION_MANIFEST_NAME
    sidecar_path = resolved / SESSION_MANIFEST_SIDECAR_NAME
    if not manifest_path.is_file():
        issues.append(SessionVerificationIssue("session_manifest_missing", "session manifest is missing"))
        return SessionVerificationResult(False, "INTEGRITY_FAILED", None, 0, tuple(issues))
    try:
        manifest_bytes = manifest_path.read_bytes()
        manifest = json.loads(manifest_bytes.decode("utf-8"))
        if not isinstance(manifest, dict):
            raise ValueError("session manifest must contain a JSON object")
    except (OSError, UnicodeError, ValueError) as exc:
        issues.append(SessionVerificationIssue("session_manifest_invalid", str(exc), SESSION_MANIFEST_NAME))
        return SessionVerificationResult(False, "INTEGRITY_FAILED", None, 0, tuple(issues))
    session_id = str(manifest.get("session_id") or "") or None
    if session_id is not None and resolved.name != session_id:
        issues.append(SessionVerificationIssue("session_directory_mismatch", "session directory name differs from session_id"))
    actual_digest = hashlib.sha256(manifest_bytes).hexdigest()
    if not sidecar_path.is_file():
        issues.append(SessionVerificationIssue("session_sidecar_missing", "session sidecar is missing"))
    else:
        try:
            expected_digest = sidecar_path.read_text(encoding="ascii").strip().split()[0]
        except (OSError, UnicodeError, IndexError) as exc:
            issues.append(SessionVerificationIssue("session_sidecar_invalid", str(exc)))
        else:
            if expected_digest != actual_digest:
                issues.append(SessionVerificationIssue("session_sidecar_mismatch", "session sidecar hash does not match"))
    manifest_schema_version = int(manifest.get("schema_version") or 0)
    if (
        manifest.get("schema") != SESSION_SCHEMA
        or manifest_schema_version
        not in LEGACY_SESSION_SCHEMA_VERSIONS | {SESSION_SCHEMA_VERSION}
    ):
        issues.append(SessionVerificationIssue("session_schema", "unsupported session schema"))
    if manifest.get("state") != "SEALED":
        issues.append(SessionVerificationIssue("session_state", "session manifest is not sealed"))

    declared_ids: set[str] = set()
    declared_sequences: set[int] = set()
    for raw_row in manifest.get("measurements") or []:
        if not isinstance(raw_row, Mapping):
            issues.append(SessionVerificationIssue("measurement_entry_invalid", "measurement entry is not an object"))
            continue
        measurement_id = str(raw_row.get("measurement_id") or "")
        if not measurement_id or measurement_id in declared_ids:
            issues.append(SessionVerificationIssue("measurement_id_invalid", "measurement ID is empty or duplicated", measurement_id=measurement_id or None))
            continue
        measurement_package_id = str(
            raw_row.get("measurement_package_id") or measurement_id
        )
        if measurement_package_id != measurement_id:
            issues.append(
                SessionVerificationIssue(
                    "measurement_package_id_mismatch",
                    "measurement package alias differs from measurement_id",
                    measurement_id=measurement_id,
                )
            )
            continue
        declared_ids.add(measurement_id)
        try:
            sequence = int(raw_row.get("sequence") or 0)
        except (TypeError, ValueError):
            sequence = 0
        if sequence < 1 or sequence in declared_sequences:
            issues.append(SessionVerificationIssue("measurement_sequence_invalid", "measurement sequence is non-positive or duplicated", measurement_id=measurement_id))
            continue
        declared_sequences.add(sequence)
        expected_relative = (Path(MEASUREMENTS_DIRECTORY_NAME) / measurement_id).as_posix()
        if str(raw_row.get("relative_path") or "") != expected_relative:
            issues.append(SessionVerificationIssue("measurement_path_mismatch", "measurement path does not match its ID", measurement_id=measurement_id))
            continue
        try:
            child_root = _safe_child(resolved, expected_relative, must_exist=True)
        except Exception as exc:
            issues.append(SessionVerificationIssue("measurement_path_invalid", str(exc), expected_relative, measurement_id))
            continue
        result = verify_package(child_root)
        if not result.valid:
            issues.append(SessionVerificationIssue("measurement_integrity_failed", ", ".join(issue.code for issue in result.issues), expected_relative, measurement_id))
            continue
        expected_manifest_relative = (
            Path(expected_relative)
            / "revisions"
            / str(result.revision or "")
            / "package_manifest.json"
        ).as_posix()
        manifest_relative = str(raw_row.get("manifest_relative_path") or "")
        if manifest_relative != expected_manifest_relative:
            issues.append(SessionVerificationIssue("measurement_manifest_path_mismatch", "child manifest path is not the verified latest revision", manifest_relative or None, measurement_id))
            continue
        try:
            child_manifest = _safe_child(resolved, manifest_relative, must_exist=True)
        except Exception as exc:
            issues.append(SessionVerificationIssue("measurement_manifest_path_invalid", str(exc), manifest_relative or None, measurement_id))
            continue
        if str(raw_row.get("manifest_sha256") or "") != sha256_file(child_manifest):
            issues.append(SessionVerificationIssue("measurement_manifest_hash_mismatch", "child manifest hash does not match session manifest", manifest_relative, measurement_id))
            continue
        if result.session_id != session_id:
            issues.append(SessionVerificationIssue("measurement_session_mismatch", "child package session_id differs from parent", expected_relative, measurement_id))
            continue
        if str(raw_row.get("revision") or "") != str(result.revision or ""):
            issues.append(SessionVerificationIssue("measurement_revision_mismatch", "child revision differs from session manifest", expected_relative, measurement_id))
            continue
        if int(raw_row.get("revision_number") or 0) != int(result.revision_number or 0):
            issues.append(SessionVerificationIssue("measurement_revision_number_mismatch", "child revision number differs from session manifest", expected_relative, measurement_id))
            continue
        try:
            child_manifest_payload = _load_json(child_manifest)
        except Exception as exc:
            issues.append(SessionVerificationIssue("measurement_manifest_invalid", str(exc), manifest_relative, measurement_id))
            continue
        if str(raw_row.get("state") or "") != str(child_manifest_payload.get("state") or ""):
            issues.append(SessionVerificationIssue("measurement_state_mismatch", "child state differs from session manifest", manifest_relative, measurement_id))
            continue
        child_complete = bool(
            dict(child_manifest_payload.get("evidence_completeness") or {}).get("complete", False)
        )
        if bool(raw_row.get("evidence_complete")) != child_complete:
            issues.append(SessionVerificationIssue("measurement_completeness_mismatch", "child completeness differs from session manifest", manifest_relative, measurement_id))
            continue
        metadata = child_manifest_payload.get("metadata")
        metadata = metadata if isinstance(metadata, Mapping) else {}
        if (
            str(metadata.get("measurement_id") or "") != measurement_id
            or str(
                metadata.get("measurement_package_id") or measurement_id
            ) != measurement_id
            or str(metadata.get("parent_session_id") or "") != str(session_id or "")
            or int(metadata.get("measurement_sequence") or 0) != sequence
        ):
            issues.append(SessionVerificationIssue("measurement_identity_mismatch", "child metadata identity differs from parent index", manifest_relative, measurement_id))
            continue
        if _json_safe(raw_row.get("measurement_outcome")) != _json_safe(
            child_manifest_payload.get("measurement_outcome")
        ):
            issues.append(SessionVerificationIssue("measurement_outcome_mismatch", "child outcome differs from session manifest", manifest_relative, measurement_id))
            continue
        if manifest_schema_version >= 2:
            try:
                expected_recording = _recording_generation_summary(
                    child_root,
                    child_manifest_payload,
                )
            except Exception as exc:
                issues.append(
                    SessionVerificationIssue(
                        "measurement_recording_generation_invalid",
                        str(exc),
                        manifest_relative,
                        measurement_id,
                    )
                )
                continue
            if any(
                _json_safe(raw_row.get(field))
                != _json_safe(expected_recording[field])
                for field in (
                    "recording_generation",
                    "authoritative_jsonl_fragments",
                    "context_fragments",
                    "binary_artifacts",
                )
            ):
                issues.append(
                    SessionVerificationIssue(
                        "measurement_recording_generation_mismatch",
                        "child recording generation inventory differs from parent",
                        manifest_relative,
                        measurement_id,
                    )
                )
                continue
        verified += 1

    actual_ids: set[str] = set()
    measurements_root = resolved / MEASUREMENTS_DIRECTORY_NAME
    if measurements_root.is_dir():
        for child in measurements_root.iterdir():
            if child.is_symlink():
                issues.append(SessionVerificationIssue("measurement_symlink", "measurement entries must not be symlinks", child.relative_to(resolved).as_posix(), child.name))
                continue
            if not child.is_dir():
                issues.append(SessionVerificationIssue("measurement_unexpected_file", "unexpected file in measurements directory", child.relative_to(resolved).as_posix()))
                continue
            if not (
                (child / "package_working_state.json").is_file()
                or (child / LATEST_REVISION_NAME).is_file()
            ):
                issues.append(SessionVerificationIssue("measurement_orphan", "measurement directory has no package state", child.relative_to(resolved).as_posix(), child.name))
                continue
            actual_ids.add(child.name)
    for measurement_id in sorted(actual_ids - declared_ids):
        issues.append(SessionVerificationIssue("measurement_unlisted", "child measurement is absent from the sealed session manifest", measurement_id=measurement_id))

    summary = manifest.get("measurement_summary")
    summary = summary if isinstance(summary, Mapping) else {}
    rows = [row for row in manifest.get("measurements") or [] if isinstance(row, Mapping)]
    expected_complete_count = sum(bool(row.get("evidence_complete")) for row in rows)
    expected_summary = {
        "measurement_count": len(rows),
        "complete_count": expected_complete_count,
        "incomplete_count": len(rows) - expected_complete_count,
        "evidence_complete": expected_complete_count == len(rows),
    }
    if dict(summary) != expected_summary:
        issues.append(SessionVerificationIssue("session_summary_mismatch", "measurement summary is inconsistent"))

    declared_shared: set[str] = set()
    for raw_artifact in manifest.get("shared_artifacts") or []:
        if not isinstance(raw_artifact, Mapping):
            issues.append(SessionVerificationIssue("shared_artifact_invalid", "shared artifact entry is not an object"))
            continue
        relative = str(raw_artifact.get("relative_path") or "")
        try:
            path = _safe_child(resolved, relative, must_exist=True)
        except Exception as exc:
            issues.append(SessionVerificationIssue("shared_artifact_path_invalid", str(exc), relative or None))
            continue
        canonical = path.relative_to(resolved).as_posix()
        if canonical != relative or Path(relative).parts[0] == MEASUREMENTS_DIRECTORY_NAME:
            issues.append(SessionVerificationIssue("shared_artifact_scope_invalid", "shared artifact is outside session-shared scope", relative))
            continue
        if path.is_symlink() or not path.is_file():
            issues.append(SessionVerificationIssue("shared_artifact_not_regular", "shared artifact is not a regular file", relative))
            continue
        declared_shared.add(relative)
        if int(raw_artifact.get("size_bytes") or -1) != path.stat().st_size:
            issues.append(SessionVerificationIssue("shared_artifact_size_mismatch", "shared artifact size differs", relative))
            continue
        if str(raw_artifact.get("sha256") or "") != sha256_file(path):
            issues.append(SessionVerificationIssue("shared_artifact_hash_mismatch", "shared artifact hash differs", relative))

    actual_shared: set[str] = set()
    excluded_root_names = {
        SESSION_STATE_NAME,
        SESSION_MANIFEST_NAME,
        SESSION_MANIFEST_SIDECAR_NAME,
    }
    for current, directories, names in os.walk(resolved, followlinks=False):
        current_path = Path(current)
        relative_dir = current_path.relative_to(resolved)
        directories[:] = [
            name
            for name in directories
            if not (relative_dir == Path(".") and name == MEASUREMENTS_DIRECTORY_NAME)
        ]
        for name in names:
            path = current_path / name
            relative = path.relative_to(resolved)
            if len(relative.parts) == 1 and name in excluded_root_names:
                continue
            if name.endswith((".tmp", ".part", ".partial", ".lock")):
                continue
            if path.is_file() and not path.is_symlink():
                actual_shared.add(relative.as_posix())
    for relative in sorted(actual_shared - declared_shared):
        issues.append(SessionVerificationIssue("shared_artifact_unlisted", "session shared file is absent from manifest", relative))

    return SessionVerificationResult(
        not issues,
        "VERIFIED" if not issues else "INTEGRITY_FAILED",
        session_id,
        verified,
        tuple(issues),
    )


__all__ = [
    "MEASUREMENTS_DIRECTORY_NAME",
    "MeasurementSession",
    "SESSION_MANIFEST_NAME",
    "SESSION_MANIFEST_SIDECAR_NAME",
    "SESSION_SCHEMA",
    "SESSION_SCHEMA_VERSION",
    "SESSION_STATE_NAME",
    "SessionVerificationIssue",
    "SessionVerificationResult",
    "verify_measurement_session",
]
