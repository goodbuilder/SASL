"""Read-only artifact identity, manifest and path projections.

Paths are resolved against an explicit owned session root. Recorded NED/FRD/RDF
and time fields retain their producer labels; review never reinterprets them.
"""
from __future__ import annotations
import hashlib
import json
from pathlib import Path
from typing import Mapping
from .values import _integer

def resolve_session_artifact_path(
    session_root: Path | str,
    artifact_name: str,
    *,
    allow_published: bool = True,
) -> Path | None:
    """Resolve one session-owned artifact without discarding subdirectories.

    Manifest and run-ledger artifacts may live below ``.generations`` or
    ``assets``.  Basename-only resolution made those files impossible to
    review and could not distinguish two artifacts with the same filename.
    Absolute paths, ``..`` traversal, and symlinks escaping the session root
    are rejected.  A finalized generation is also searched through the
    session-owned ``published`` pointer for legacy basename manifests.
    """

    root = Path(session_root).expanduser().resolve()
    raw = str(artifact_name or "").strip()
    if not raw:
        return root if root.exists() else None
    relative = Path(raw)
    if relative.is_absolute() or "\x00" in raw:
        return None
    candidates = [root / relative]
    if allow_published:
        candidates.append(root / "published" / relative)
    for unresolved in candidates:
        try:
            candidate = unresolved.resolve()
            candidate.relative_to(root)
        except (OSError, RuntimeError, ValueError):
            continue
        if candidate.exists():
            return candidate
    return None

def _safe_session_relative_path(session_root: Path, value: object) -> str | None:
    """Return a normalized session-relative path only when it cannot escape.

    Package manifests are review input, not trusted filesystem authority.  Keep
    their nested paths intact while rejecting absolute paths, explicit parent
    traversal, NULs, and paths whose existing symlink prefix leaves the session.
    The target itself need not exist because missing artifacts are evidence too.
    """

    raw = str(value or "").strip()
    if not raw or "\x00" in raw:
        return None
    relative = Path(raw)
    if relative.is_absolute() or any(part == ".." for part in relative.parts):
        return None
    normalized = Path(*[part for part in relative.parts if part not in ("", ".")])
    if not normalized.parts:
        return None
    root = session_root.expanduser().resolve()
    try:
        (root / normalized).resolve().relative_to(root)
    except (OSError, RuntimeError, ValueError):
        return None
    return normalized.as_posix()

def _recording_session_directories(recordings_root: Path) -> tuple[Path, ...]:
    """Return v2 direct sessions followed by legacy ``sessions/`` entries."""

    root = recordings_root.expanduser().resolve()
    candidates: list[Path] = []
    if root.is_dir():
        candidates.extend(
            child
            for child in root.iterdir()
            if child.name != "sessions"
            and not child.name.startswith(".")
            and child.is_dir()
            and not child.is_symlink()
        )
    legacy = root / "sessions"
    if legacy.is_dir() and not legacy.is_symlink():
        candidates.extend(
            child
            for child in legacy.iterdir()
            if child.is_dir() and not child.is_symlink()
        )
    grouped: dict[str, list[Path]] = {}
    for candidate in candidates:
        grouped.setdefault(candidate.name, []).append(candidate)
    # Ambiguous direct/legacy IDs are deliberately absent from automatic
    # review.  Recovery reports the collision and requires an explicit choice.
    return tuple(values[0] for values in grouped.values() if len(values) == 1)

def _resolve_recording_session_directory(
    recordings_root: Path,
    session_id: object,
) -> Path | None:
    """Resolve a safe session ID, preferring the v2 direct location."""

    component = str(session_id or "")
    if (
        not component
        or Path(component).name != component
        or component in {".", "..", "sessions"}
        or "\\" in component
        or "\x00" in component
    ):
        return None
    from Workstation.workstation_core.session_storage.paths import resolve_session_dir

    root = recordings_root.expanduser().resolve()
    try:
        resolved = resolve_session_dir(root, component).resolve(strict=True)
        resolved.relative_to(root)
    except (FileExistsError, FileNotFoundError, OSError, RuntimeError, ValueError):
        return None
    return resolved if resolved.is_dir() and not resolved.is_symlink() else None

def _package_manifest_reference(
    session_root: Path,
) -> tuple[Path | None, dict[str, object], str, bool, str]:
    """Locate the authoritative revision manifest without following unsafe pointers.

    The return value is ``(path, latest_pointer, error, detected, relative_path)``.
    An existing latest pointer is authoritative; a malformed or unsafe pointer is
    reported rather than silently falling back to an arbitrary revision.
    """

    latest_path = session_root / "latest_revision.json"
    if latest_path.exists() or latest_path.is_symlink():
        safe_latest = _safe_session_relative_path(session_root, "latest_revision.json")
        if safe_latest is None or not latest_path.is_file():
            return None, {}, "latest_revision.json is not a safe regular file", True, ""
        try:
            latest_payload = json.loads(latest_path.read_text(encoding="utf-8"))
        except (OSError, UnicodeError, ValueError) as exc:
            return None, {}, f"malformed latest_revision.json: {exc}", True, ""
        if not isinstance(latest_payload, Mapping):
            return None, {}, "malformed latest_revision.json: expected a JSON object", True, ""
        latest = {str(key): value for key, value in latest_payload.items()}
        manifest_value = latest.get("manifest_path")
        if manifest_value in (None, ""):
            revision = latest.get("revision")
            if revision in (None, ""):
                return None, latest, "latest_revision.json has no manifest_path or revision", True, ""
            revision_text = str(revision)
            if revision_text.isdigit():
                revision_text = f"r{int(revision_text):04d}"
            manifest_value = f"revisions/{revision_text}/package_manifest.json"
        relative = _safe_session_relative_path(session_root, manifest_value)
        if relative is None:
            return None, latest, "latest_revision.json contains an unsafe manifest_path", True, ""
        parts = Path(relative).parts
        if len(parts) != 3 or parts[0] != "revisions" or parts[2] != "package_manifest.json":
            return None, latest, "latest_revision.json manifest_path is not a revision package manifest", True, relative
        manifest_path = session_root / relative
        if not manifest_path.is_file():
            return None, latest, f"latest package manifest is missing: {relative}", True, relative
        return manifest_path, latest, "", True, relative

    revisions_root = session_root / "revisions"
    if not revisions_root.is_dir():
        return None, {}, "", False, ""
    if _safe_session_relative_path(session_root, "revisions") is None:
        return None, {}, "package revisions directory escapes the session", True, ""
    candidates: list[tuple[str, Path]] = []
    unsafe_manifest_seen = False
    try:
        revision_dirs = tuple(revisions_root.iterdir())
    except OSError as exc:
        return None, {}, f"cannot scan package revisions: {exc}", True, ""
    for revision_dir in revision_dirs:
        candidate = revision_dir / "package_manifest.json"
        relative = _safe_session_relative_path(session_root, candidate.relative_to(session_root))
        if candidate.exists() and relative is None:
            unsafe_manifest_seen = True
        elif relative is not None and candidate.is_file():
            candidates.append((relative, candidate))
    if not candidates:
        if unsafe_manifest_seen:
            return None, {}, "revision package manifest escapes the session", True, ""
        return None, {}, "", False, ""
    def revision_sort_key(item: tuple[str, Path]) -> tuple[int, int, str]:
        revision_name = Path(item[0]).parts[1]
        if revision_name.startswith("r") and revision_name[1:].isdigit():
            return (1, int(revision_name[1:]), item[0])
        return (0, 0, item[0])

    relative, manifest_path = max(candidates, key=revision_sort_key)
    return manifest_path, {}, "", True, relative

def _mapping_state(value: object, *, default: str = "") -> str:
    if isinstance(value, Mapping):
        for key in ("state", "status", "value"):
            candidate = value.get(key)
            if candidate not in (None, ""):
                return str(candidate)
        return default
    return default if value in (None, "") else str(value)

def _reason_values(value: object) -> tuple[str, ...]:
    """Flatten list/mapping reason fields without inventing an interpretation."""

    reasons: list[str] = []

    def append(item: object, label: str = "") -> None:
        if item in (None, "", False):
            return
        if isinstance(item, str):
            reasons.append(f"{label}: {item}" if label else item)
            return
        if isinstance(item, Mapping):
            identity = next(
                (
                    str(item.get(key))
                    for key in ("artifact_id", "relative_path", "path", "role", "source_id")
                    if item.get(key) not in (None, "")
                ),
                label,
            )
            detail = next(
                (
                    item.get(key)
                    for key in ("reason", "message", "error", "detail")
                    if item.get(key) not in (None, "")
                ),
                None,
            )
            if detail is not None:
                append(detail, identity)
                return
            nested = next(
                (item.get(key) for key in ("reasons", "missing_required", "issues") if key in item),
                None,
            )
            if nested is not None:
                append(nested, identity)
                return
            source = item.get("source")
            role = item.get("role")
            status = item.get("status")
            if source not in (None, "") and role not in (None, "") and status not in (None, ""):
                reasons.append(f"{source}/{role}: {status}")
                return
            for key, nested_value in item.items():
                append(nested_value, str(key))
            return
        if isinstance(item, (list, tuple, set)):
            for nested_item in item:
                append(nested_item, label)
            return
        reasons.append(f"{label}: {item}" if label else str(item))

    append(value)
    return tuple(dict.fromkeys(reason for reason in reasons if reason))

def _artifact_reason(detail: Mapping[str, object]) -> str:
    metadata = detail.get("metadata")
    for source in (detail, metadata if isinstance(metadata, Mapping) else {}):
        for key in ("reason", "error", "message", "detail"):
            value = source.get(key)
            if value not in (None, ""):
                return str(value)
    return ""

def _package_artifact_projection(
    session_root: Path,
    value: object,
) -> tuple[tuple[dict[str, object], ...], tuple[str, ...], tuple[str, ...], int]:
    """Normalize list- or mapping-shaped artifact inventories."""

    raw_entries: list[tuple[str, object]] = []
    if isinstance(value, Mapping):
        raw_entries.extend((str(key), detail) for key, detail in value.items())
    elif isinstance(value, (list, tuple)):
        raw_entries.extend((str(index), detail) for index, detail in enumerate(value))
    elif value not in (None, ""):
        raw_entries.append(("0", value))

    entries: list[dict[str, object]] = []
    missing: list[str] = []
    failed: list[str] = []
    total_bytes = 0
    for key, raw_detail in raw_entries:
        if isinstance(raw_detail, Mapping):
            detail = {str(name): item for name, item in raw_detail.items()}
        elif isinstance(raw_detail, str):
            detail = (
                {"status": raw_detail}
                if isinstance(value, Mapping)
                else {"relative_path": raw_detail}
            )
        else:
            detail = {"status": raw_detail}
        artifact_id = str(detail.get("artifact_id") or key)
        declared_path = detail.get("relative_path", detail.get("path"))
        if (
            declared_path in (None, "")
            and isinstance(value, Mapping)
            and detail.get("artifact_id") in (None, "")
        ):
            declared_path = key
        relative_path = (
            _safe_session_relative_path(session_root, declared_path)
            if declared_path not in (None, "")
            else None
        )
        state = _mapping_state(detail.get("status", detail.get("state")), default="present")
        state_lower = state.lower()
        reason = _artifact_reason(detail)
        if declared_path not in (None, "") and relative_path is None:
            failed.append(f"{artifact_id}: unsafe artifact path rejected")
        elif state_lower == "present" and relative_path is None:
            failed.append(f"{artifact_id}: present artifact has no relative_path")
        if state_lower == "missing":
            missing.append(f"{artifact_id}: {reason}" if reason else artifact_id)
        elif state_lower == "failed":
            failed.append(f"{artifact_id}: {reason}" if reason else artifact_id)

        size_bytes = _integer(detail.get("size_bytes", detail.get("size")))
        if (
            size_bytes is not None
            and size_bytes >= 0
            and relative_path is not None
            and state_lower == "present"
        ):
            total_bytes += size_bytes
        else:
            size_bytes = None
        entries.append(
            {
                "artifact_id": artifact_id,
                "role": str(detail.get("role") or ""),
                "classification": str(detail.get("classification") or ""),
                "relative_path": relative_path or "",
                "producer": str(detail.get("producer") or ""),
                "source": str(detail.get("source") or detail.get("source_id") or ""),
                "required": bool(detail.get("required", False)),
                "status": state,
                "media_type": str(detail.get("media_type") or ""),
                "schema": detail.get("schema"),
                "size_bytes": size_bytes,
                "sha256": str(detail.get("sha256") or ""),
                "reason": reason,
            }
        )
    return (
        tuple(entries),
        tuple(dict.fromkeys(missing)),
        tuple(dict.fromkeys(failed)),
        total_bytes,
    )

def _package_error_projection(
    session_root: Path,
    *,
    error: str,
    relative_path: str,
    latest: Mapping[str, object] | None = None,
) -> dict[str, object]:
    latest = latest if isinstance(latest, Mapping) else {}
    revision = str(latest.get("revision") or (Path(relative_path).parent.name if relative_path else ""))
    revision_number = _integer(latest.get("revision_number"))
    hash_summary = {
        "status": "NOT_RUN",
        "manifest_status": "UNREADABLE",
        "expected_manifest_sha256": str(latest.get("manifest_sha256") or ""),
        "actual_manifest_sha256": "",
        "artifact_hashes_declared": 0,
        "artifacts_verified": 0,
        "artifacts_failed": 0,
        "artifacts_not_run": 0,
    }
    return {
        "session_id": session_root.name,
        "path": str(session_root),
        "status": "INTEGRITY_FAILED",
        "lifecycle_state": "UNKNOWN",
        "completeness_state": "UNKNOWN",
        "evidence_complete": False,
        "integrity_status": "INTEGRITY_FAILED",
        "package_schema": "",
        "package_schema_version": None,
        "revision": revision,
        "revision_number": revision_number,
        "package_manifest_path": relative_path,
        "missing_reasons": (),
        "failed_reasons": (error,),
        "total_bytes": 0,
        "hash_verification": hash_summary,
        "hash_verification_summary": hash_summary,
        "source_expectations": (),
        "artifact_details": (),
        "artifacts": (),
        "error": error,
        "package_manifest_error": True,
        "retryable": False,
        "published_target": "",
        "phase": "",
        "generation_id": "",
        "source": relative_path or "latest_revision.json",
        "time_axis": "recording_time_ns",
        "legacy": False,
    }

def _package_session_projection(
    session_root: Path,
    *,
    owning_session_id: str | None = None,
    measurement_id: str | None = None,
) -> dict[str, object] | None:
    expected_session_id = str(owning_session_id or session_root.name)
    expected_measurement_id = str(measurement_id or "")

    def identified(entry: dict[str, object]) -> dict[str, object]:
        entry["session_id"] = expected_session_id
        entry["measurement_id"] = expected_measurement_id
        entry["catalog_key"] = (
            f"{expected_session_id}::{expected_measurement_id}"
            if expected_measurement_id
            else expected_session_id
        )
        entry["layout"] = "nested-v2" if expected_measurement_id else "legacy-flat-v1"
        entry["session_root"] = str(
            session_root.parent.parent if expected_measurement_id else session_root
        )
        return entry

    manifest_path, latest, locate_error, detected, relative_path = _package_manifest_reference(session_root)
    if not detected:
        return None
    if locate_error or manifest_path is None:
        return identified(_package_error_projection(
            session_root,
            error=locate_error or "package manifest is unavailable",
            relative_path=relative_path,
            latest=latest,
        ))
    try:
        manifest_bytes = manifest_path.read_bytes()
        payload = json.loads(manifest_bytes.decode("utf-8"))
    except (OSError, UnicodeError, ValueError) as exc:
        return identified(_package_error_projection(
            session_root,
            error=f"malformed package manifest: {exc}",
            relative_path=relative_path,
            latest=latest,
        ))
    if not isinstance(payload, Mapping):
        return identified(_package_error_projection(
            session_root,
            error="malformed package manifest: expected a JSON object",
            relative_path=relative_path,
            latest=latest,
        ))
    manifest = {str(key): value for key, value in payload.items()}
    artifact_details, artifact_missing, artifact_failed, total_bytes = _package_artifact_projection(
        session_root,
        manifest.get("artifacts", manifest.get("files", manifest.get("inventory"))),
    )
    artifacts = tuple(
        sorted(
            {
                str(detail["relative_path"])
                for detail in artifact_details
                if detail.get("relative_path")
            }
        )
    )
    completeness = manifest.get("evidence_completeness", manifest.get("completeness"))
    completeness_state = _mapping_state(completeness, default="UNKNOWN")
    completeness_mapping = completeness if isinstance(completeness, Mapping) else {}
    complete_value = completeness_mapping.get("complete")
    if completeness_state == "UNKNOWN" and complete_value is not None:
        completeness_state = "COMPLETE" if bool(complete_value) else "INCOMPLETE"
    evidence_complete = (
        bool(complete_value)
        if complete_value is not None
        else completeness_state.upper() == "COMPLETE"
    )
    manifest_missing = _reason_values(
        completeness_mapping.get("missing_required", manifest.get("missing_reasons"))
    )
    manifest_failed = _reason_values(
        completeness_mapping.get("failed", manifest.get("failed_reasons"))
    )
    missing_reasons = tuple(dict.fromkeys((*manifest_missing, *artifact_missing)))
    failed_reasons = list(dict.fromkeys((*manifest_failed, *artifact_failed)))

    actual_manifest_sha256 = hashlib.sha256(manifest_bytes).hexdigest()
    expected_manifest_sha256 = str(latest.get("manifest_sha256") or "")
    manifest_hash_status = "UNAVAILABLE"
    integrity_status = "NOT_VERIFIED"
    if expected_manifest_sha256:
        if len(expected_manifest_sha256) != 64 or any(
            character not in "0123456789abcdefABCDEF" for character in expected_manifest_sha256
        ):
            manifest_hash_status = "INVALID_EXPECTED_HASH"
            integrity_status = "INTEGRITY_FAILED"
            failed_reasons.append("latest_revision.json manifest_sha256 is not a SHA-256 digest")
        elif actual_manifest_sha256.lower() == expected_manifest_sha256.lower():
            manifest_hash_status = "VERIFIED"
            integrity_status = "MANIFEST_VERIFIED"
        else:
            manifest_hash_status = "MISMATCH"
            integrity_status = "INTEGRITY_FAILED"
            failed_reasons.append("package manifest SHA-256 does not match latest_revision.json")
    declared_hashes = sum(bool(detail.get("sha256")) for detail in artifact_details)
    present_artifacts = sum(str(detail.get("status") or "").lower() == "present" for detail in artifact_details)
    hash_summary = {
        "status": (
            "INTEGRITY_FAILED"
            if integrity_status == "INTEGRITY_FAILED"
            else "ARTIFACTS_NOT_RUN"
        ),
        "manifest_status": manifest_hash_status,
        "expected_manifest_sha256": expected_manifest_sha256,
        "actual_manifest_sha256": actual_manifest_sha256,
        "artifact_hashes_declared": declared_hashes,
        "artifacts_verified": 0,
        "artifacts_failed": 0,
        "artifacts_not_run": present_artifacts,
    }

    schema = manifest.get("schema", manifest.get("package_schema", ""))
    schema_name = _mapping_state(schema)
    if isinstance(schema, Mapping):
        schema_name = str(schema.get("name") or schema.get("id") or schema_name)
    schema_version = _integer(
        manifest.get(
            "schema_version",
            schema.get("version") if isinstance(schema, Mapping) else None,
        )
    )
    lifecycle_state = _mapping_state(
        manifest.get("state", manifest.get("lifecycle", manifest.get("status"))),
        default="UNKNOWN",
    )
    manifest_session_id = str(manifest.get("session_id") or session_root.name)
    if manifest_session_id != expected_session_id:
        failed_reasons.append(
            f"package manifest session_id {manifest_session_id!r} does not match owning session {expected_session_id!r}"
        )
        integrity_status = "INTEGRITY_FAILED"
        hash_summary["status"] = "INTEGRITY_FAILED"
    manifest_metadata = manifest.get("metadata")
    manifest_metadata = manifest_metadata if isinstance(manifest_metadata, Mapping) else {}
    declared_measurement_id = str(manifest_metadata.get("measurement_id") or "")
    if expected_measurement_id and declared_measurement_id != expected_measurement_id:
        failed_reasons.append(
            f"package measurement_id {declared_measurement_id!r} does not match directory {expected_measurement_id!r}"
        )
        integrity_status = "INTEGRITY_FAILED"
        hash_summary["status"] = "INTEGRITY_FAILED"
    if schema_name and schema_name != "workstation-measurement-package-v1":
        failed_reasons.append(f"unsupported package schema: {schema_name}")
        integrity_status = "INTEGRITY_FAILED"
        hash_summary["status"] = "INTEGRITY_FAILED"
    elif not schema_name:
        failed_reasons.append("package manifest schema is missing")
        integrity_status = "INTEGRITY_FAILED"
        hash_summary["status"] = "INTEGRITY_FAILED"
    if schema_version != 1:
        failed_reasons.append(f"unsupported package schema version: {schema_version}")
        integrity_status = "INTEGRITY_FAILED"
        hash_summary["status"] = "INTEGRITY_FAILED"
    source_expectations = manifest.get("source_expectations", ())
    if isinstance(source_expectations, Mapping):
        source_expectations = tuple(source_expectations.values())
    elif isinstance(source_expectations, list):
        source_expectations = tuple(source_expectations)
    elif not isinstance(source_expectations, tuple):
        source_expectations = ()
    revision = str(manifest.get("revision") or latest.get("revision") or Path(relative_path).parent.name)
    revision_number = _integer(manifest.get("revision_number", latest.get("revision_number")))
    revision_directory = Path(relative_path).parent.name
    if revision != revision_directory:
        failed_reasons.append(
            f"package manifest revision {revision!r} does not match directory {revision_directory!r}"
        )
        integrity_status = "INTEGRITY_FAILED"
        hash_summary["status"] = "INTEGRITY_FAILED"
    if latest:
        pointer_schema = str(latest.get("schema") or "")
        if pointer_schema and pointer_schema != "workstation-measurement-package-latest-v1":
            failed_reasons.append(f"unsupported latest revision pointer schema: {pointer_schema}")
            integrity_status = "INTEGRITY_FAILED"
            hash_summary["status"] = "INTEGRITY_FAILED"
        pointer_session = str(latest.get("session_id") or "")
        if pointer_session and pointer_session != manifest_session_id:
            failed_reasons.append("latest revision pointer session_id does not match package manifest")
            integrity_status = "INTEGRITY_FAILED"
            hash_summary["status"] = "INTEGRITY_FAILED"
        pointer_revision = str(latest.get("revision") or "")
        if pointer_revision and pointer_revision != revision:
            failed_reasons.append("latest revision pointer revision does not match package manifest")
            integrity_status = "INTEGRITY_FAILED"
            hash_summary["status"] = "INTEGRITY_FAILED"
        pointer_number = _integer(latest.get("revision_number"))
        if pointer_number is not None and pointer_number != revision_number:
            failed_reasons.append("latest revision pointer revision_number does not match package manifest")
            integrity_status = "INTEGRITY_FAILED"
            hash_summary["status"] = "INTEGRITY_FAILED"
    status = "INTEGRITY_FAILED" if integrity_status == "INTEGRITY_FAILED" else lifecycle_state
    return identified({
        "session_id": expected_session_id,
        "measurement_id": expected_measurement_id,
        "path": str(session_root),
        "status": status,
        "lifecycle_state": lifecycle_state,
        "completeness_state": completeness_state,
        "evidence_complete": evidence_complete,
        "integrity_status": integrity_status,
        "package_schema": schema_name,
        "package_schema_version": schema_version,
        "revision": revision,
        "revision_number": revision_number,
        "package_manifest_path": relative_path,
        "missing_reasons": missing_reasons,
        "failed_reasons": tuple(dict.fromkeys(failed_reasons)),
        "total_bytes": total_bytes,
        "hash_verification": hash_summary,
        "hash_verification_summary": hash_summary,
        "source_expectations": source_expectations,
        "artifact_details": artifact_details,
        "artifacts": artifacts,
        "error": "; ".join(dict.fromkeys(failed_reasons)),
        "package_manifest_error": False,
        "retryable": lifecycle_state == "RECOVERABLE",
        "published_target": "",
        "phase": lifecycle_state,
        "generation_id": "",
        "source": f"{relative_path} package source of truth",
        "time_axis": str(manifest.get("time_axis", "recording_time_ns") or "recording_time_ns"),
        "legacy": False,
    })

def _run_ledger_artifact_names(session_root: Path) -> set[str]:
    """Return bounded, read-only projections of run-ledger artifact paths."""

    names = {
        name
        for name in (
            "run_events.jsonl",
            "artifacts.jsonl",
            "native_frames.jsonl",
            "hr_captures.jsonl",
            "oz_captures.jsonl",
            "hr_imports.jsonl",
        )
        if (session_root / name).is_file()
    }
    ledger = session_root / "artifacts.jsonl"
    if not ledger.is_file():
        return names
    try:
        with ledger.open("r", encoding="utf-8") as handle:
            for index, line in enumerate(handle):
                if index >= 4096:
                    break
                value = json.loads(line)
                relative = value.get("relative_path") if isinstance(value, Mapping) else None
                if relative and resolve_session_artifact_path(session_root, str(relative)) is not None:
                    names.add(str(relative))
    except (OSError, UnicodeError, ValueError):
        # The catalog remains usable even when a ledger needs forensic repair.
        pass
    return names

def _existing_artifact_byte_total(session_root: Path, artifacts: tuple[str, ...]) -> int:
    total = 0
    for name in artifacts:
        candidate = resolve_session_artifact_path(session_root, name)
        if candidate is None or not candidate.is_file():
            continue
        try:
            total += candidate.stat().st_size
        except OSError:
            continue
    return total
