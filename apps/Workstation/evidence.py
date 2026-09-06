from __future__ import annotations

"""Read-only projections of the existing StereoWIRIS panel state.

The workstation is a presentation shell.  This module deliberately keeps the
projection boundary boring: it reads existing model objects and widgets,
retains their timestamp/frame contracts, and never performs a measurement,
save, or calibration operation. Explicit presentation-only coordinate
derivations remain separate from their authoritative raw values.
"""

import time
import csv
import hashlib
import json
import math
from pathlib import Path
from typing import Any, Mapping

from Workstation.workstation_core.attitude_geometry import (
    relative_attitude,
    world_ned_vector_to_body_frd,
)
from Workstation.workstation_core.calibration import independent_zoom_selection_mode
from Workstation.workstation_core.drone_pixhawk_geometry import (
    calculate_horizontal_triangle_angles,
    subtract_vectors,
    vector_range_m,
)
from Workstation.workstation_core.gnss_baseline_center import (
    DEFAULT_MAXIMUM_GNSS_AGE_S,
    DEFAULT_MAXIMUM_GNSS_TIMESTAMP_SPREAD_S,
    estimate_gnss_baseline_center_frd,
)
from Workstation.workstation_core.gnss_monitor import evaluate_ntrip_health
from Workstation.workstation_core.magnetometer_yaw import (
    calibrated_magnetometer_yaw_deg,
    project_magnetometer_yaw,
)
from Workstation.workstation_core.mavlink_liveness import MAVLINK_QUALIFICATION_S
from Workstation.workstation_core.pixhawk_attitude import (
    MAVLINK_ATTITUDE_STALE_S,
    MAVLINK_IMU_STREAM_STALE_S,
    MAVLINK_MAG_STREAMS,
    magnetometer_value_units,
)
from Workstation.workstation_core.physical_alignment import (
    angle_error_deg,
    calculate_baseline_yaw_alignment,
    normalize_compass_angle_deg,
)
from Workstation.workstation_core.px4_mag_manager import (
    BOARD_INTERNAL_MAG_ID,
    GPS1_MAG_ID,
    GPS2_MAG_ID,
    INTERNAL_MAG_IDS,
    MAG_SLOTS,
    RM3100_BACKUP_IDS,
    RM3100_PRIMARY_IDS,
    identify_mag_device,
    mag_role_for_device_id,
    mag_priority_slot_key,
)
from Workstation.workstation_core.rtk_geometry import GpsPositionState
from Workstation.workstation_core.settings import normalize_live_distance_time_basis
from Workstation.workstation_core.strict_jsonl import sha256_file

from Workstation.workstation_core.coordinate_contract import (
    BASELINE_TRANSLATION_FRAME,
    BODY_FRAME,
    CAMERA_FRAME,
    MEASUREMENT_AXIS_CONTRACT,
    MEASUREMENT_FRAME,
    WORLD_FRAME,
)


def _widget_text(widget: object | None, default: str = "") -> str:
    method = getattr(widget, "text", None)
    if callable(method):
        try:
            return str(method())
        except Exception:
            return default
    return default


def _widget_value(widget: object | None, default: object = None) -> object:
    method = getattr(widget, "value", None)
    if callable(method):
        try:
            return method()
        except Exception:
            return default
    return default


def _widget_checked(widget: object | None, default: bool = False) -> bool:
    method = getattr(widget, "isChecked", None)
    if callable(method):
        try:
            return bool(method())
        except Exception:
            return default
    return default


def _combo_data(widget: object | None, default: object = None) -> object:
    method = getattr(widget, "currentData", None)
    if callable(method):
        try:
            return method()
        except Exception:
            return default
    return default


def _number(value: object, default: float | None = None) -> float | None:
    if value in (None, ""):
        return default
    try:
        return float(value)
    except (TypeError, ValueError):
        return default


def _integer(value: object, default: int | None = None) -> int | None:
    if value in (None, ""):
        return default
    try:
        return int(value)
    except (TypeError, ValueError):
        return default


def _age(timestamp_s: object, now_s: float) -> float | None:
    value = _number(timestamp_s)
    if value is None:
        return None
    return max(0.0, now_s - value)


def _safe_len(value: object) -> int:
    try:
        return len(value)  # type: ignore[arg-type]
    except (TypeError, AttributeError):
        return 0


def _matrix_value(matrix: object, row: int, column: int) -> float | None:
    try:
        return float(matrix[row][column])  # type: ignore[index]
    except (IndexError, KeyError, TypeError, ValueError):
        return None


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
    from Workstation.workstation_core.recording_session import resolve_session_dir

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


def mavlink_link_provenance(panel: object, side: str) -> dict[str, object]:
    """Return link identity without changing any telemetry value semantics."""

    subscriber = getattr(panel, "mavlink_pixhawk", None)
    if subscriber is None:
        return {}
    urls = getattr(subscriber, "urls", {}) or {}
    connections = getattr(subscriber, "connections", {}) or {}
    connection = connections.get(side) if isinstance(connections, dict) else None
    heartbeat_sides = getattr(subscriber, "_heartbeat_logged", set()) or set()

    def optional_id(value: object) -> int | None:
        try:
            parsed = int(value)
        except (TypeError, ValueError):
            return None
        return parsed if parsed > 0 else None

    return {
        "telemetry_transport": "MAVLink",
        "mavlink_endpoint": str(urls.get(side, "") or "") if isinstance(urls, dict) else "",
        "mavlink_link_open": connection is not None,
        "mavlink_heartbeat_seen": side in heartbeat_sides,
        "mavlink_target_system": optional_id(getattr(connection, "target_system", None)),
        "mavlink_target_component": optional_id(getattr(connection, "target_component", None)),
        "mavlink_identity_contract": (
            "endpoint + target system/component identify the link; packet source system/component/sequence "
            "are recorded when available; values retain producer frames/units"
        ),
    }


class EvidenceAdapter:
    """Expose source-backed, UI-safe snapshots for one panel owner."""

    def __init__(self, panel: object) -> None:
        self.panel = panel

    def _app_root(self) -> Path:
        """Return the data root owned by this panel's application runtime."""

        configured = getattr(self.panel, "app_root", None)
        if configured not in (None, ""):
            return Path(configured)
        # Lightweight test owners may omit app_root; the repository-owned
        # Workstation core root is the only fallback.
        from Workstation.workstation_core.settings import APP_ROOT

        return Path(APP_ROOT)

    def attitude(self, side: str, *, now_s: float | None = None) -> dict[str, object]:
        now = time.monotonic() if now_s is None else float(now_s)
        states = getattr(self.panel, "latest_extrinsics", {}) or {}
        state = states.get(side) if isinstance(states, dict) else None
        if state is None:
            return {
                "side": side,
                "state": "WAITING",
                "age_s": None,
                "frame_contract": "PX4 vehicle attitude quaternion rotates body FRD to world NED",
                "source": "MAVLink attitude telemetry",
            }
        provenance = (
            mavlink_link_provenance(self.panel, side)
            if str(getattr(state, "topic", "")).startswith("mavlink/")
            else {}
        )
        return {
            "side": side,
            "state": "LIVE",
            "age_s": _age(getattr(state, "timestamp_s", None), now),
            "roll_deg": _number(getattr(state, "roll_deg", None)),
            "pitch_deg": _number(getattr(state, "pitch_deg", None)),
            "yaw_deg": _number(getattr(state, "yaw_deg", None)),
            "q_wxyz": tuple(getattr(state, "q", ()) or ()),
            "app_timestamp_s": _number(getattr(state, "timestamp_s", None)),
            "px4_timestamp_s": _number(getattr(state, "px4_timestamp_s", None)),
            "px4_timestamp_us": _integer(getattr(state, "px4_timestamp_us", None)),
            "px4_timestamp_sample_us": _integer(getattr(state, "px4_timestamp_sample_us", None)),
            "mavlink_source_system": _integer(getattr(state, "mavlink_source_system", None)),
            "mavlink_source_component": _integer(getattr(state, "mavlink_source_component", None)),
            "mavlink_sequence": _integer(getattr(state, "mavlink_sequence", None)),
            "mavlink_message_id": _integer(getattr(state, "mavlink_message_id", None)),
            "source": "MAVLink attitude telemetry / CameraExtrinsicState",
            "clock": "app receive monotonic; PX4 timestamp when present",
            "frame_contract": "PX4 vehicle attitude quaternion rotates body FRD to world NED",
            "orientation_contract": f"orientation: {BODY_FRAME} → {WORLD_FRAME}; roll/pitch/yaw are degrees",
            **provenance,
        }

    def gps(
        self,
        side: str,
        *,
        now_s: float | None = None,
        source: str = "GPS1",
    ) -> dict[str, object]:
        now = time.monotonic() if now_s is None else float(now_s)
        states = getattr(self.panel, "latest_gps", {}) or {}
        source = str(source or "GPS1").upper()
        if source not in {"GPS1", "GPS2"}:
            source = "GPS1"
        state_key = side if source == "GPS1" else f"{side}_GPS2"
        state = states.get(state_key) if isinstance(states, dict) else None
        if state is None:
            return {
                "side": side,
                "receiver_source": source,
                "state": "NO DATA",
                "source": "GpsPositionState / GPS receiver",
                "position_contract": "WGS84 latitude/longitude degrees; altitude metres",
            }
        provenance = (
            mavlink_link_provenance(self.panel, side)
            if str(getattr(state, "topic", "")).startswith("mavlink/")
            else {}
        )
        fix_type = _integer(getattr(state, "fix_type", None))
        rtk_label = str(getattr(state, "rtk_label", "UNKNOWN") or "UNKNOWN")
        return {
            "side": side,
            "receiver_source": source,
            "state": rtk_label,
            "rtk_label": rtk_label,
            "fix_type": fix_type,
            "satellites": _integer(getattr(state, "satellites_used", None)),
            "eph_m": _number(getattr(state, "eph_m", None)),
            "epv_m": _number(getattr(state, "epv_m", None)),
            "hdop": _number(getattr(state, "hdop", None)),
            "vdop": _number(getattr(state, "vdop", None)),
            "h_acc_m": _number(getattr(state, "eph_m", None)),
            "v_acc_m": _number(getattr(state, "epv_m", None)),
            "latitude_deg": _number(getattr(state, "latitude_deg", None)),
            "longitude_deg": _number(getattr(state, "longitude_deg", None)),
            "altitude_m": _number(getattr(state, "altitude_m", None)),
            "altitude_ellipsoid_m": _number(getattr(state, "altitude_ellipsoid_m", None)),
            "heading_deg": _number(getattr(state, "heading_deg", None)),
            "timestamp_s": _number(getattr(state, "timestamp_s", None)),
            "age_s": _age(getattr(state, "timestamp_s", None), now),
            "px4_timestamp_s": _number(getattr(state, "px4_timestamp_s", None)),
            "px4_timestamp_us": _integer(getattr(state, "px4_timestamp_us", None)),
            "px4_timestamp_sample_us": _integer(getattr(state, "px4_timestamp_sample_us", None)),
            "gps_utc_us": _integer(getattr(state, "gps_utc_us", None)),
            "mavlink_source_system": _integer(getattr(state, "mavlink_source_system", None)),
            "mavlink_source_component": _integer(getattr(state, "mavlink_source_component", None)),
            "mavlink_sequence": _integer(getattr(state, "mavlink_sequence", None)),
            "mavlink_message_id": _integer(getattr(state, "mavlink_message_id", None)),
            "source": "MAVLink GPS telemetry / GpsPositionState",
            "clock": "GPS app receive monotonic; PX4/GPS source timestamps retained",
            "position_contract": "WGS84 latitude/longitude degrees; altitude metres",
            **provenance,
        }

    def _magnetometer_yaw_fields(
        self,
        side: str,
        *,
        xmag: object,
        ymag: object,
        sensor_device_id: int | None,
        sample_available: bool,
        now_s: float,
    ) -> dict[str, object]:
        """Return raw yaw plus an exact active-device PX4 reference."""

        active_ids = getattr(self.panel, "latest_vehicle_mag_device_ids", {}) or {}
        active_device_id = (
            _integer(active_ids.get(side)) if isinstance(active_ids, Mapping) else None
        )
        if not sample_available:
            return {
                "raw_yaw_deg": None,
                "px4_yaw_deg": None,
                "delta_yaw_deg": None,
                "px4_yaw_state": "WAITING",
                "px4_yaw_reason": "MAG sample waiting",
                "raw_yaw_contract": (
                    "RAW sensor-axis azimuth = atan2(-y, x); no CAL_MAG, sensor rotation, "
                    "board rotation, or tilt correction"
                ),
                "active_device_id": active_device_id,
                "px4_yaw_age_s": None,
            }
        attitude = self.attitude(side, now_s=now_s)
        projection = project_magnetometer_yaw(
            x=xmag,
            y=ymag,
            sensor_device_id=sensor_device_id,
            active_device_id=active_device_id,
            px4_yaw_deg=attitude.get("yaw_deg"),
            px4_age_s=attitude.get("age_s"),
            px4_stale_s=MAVLINK_ATTITUDE_STALE_S,
        )
        return {
            **projection.to_mapping(),
            "active_device_id": active_device_id,
            "px4_yaw_age_s": attitude.get("age_s"),
            "px4_yaw_source": attitude.get("source", "MAVLink attitude telemetry"),
            "px4_yaw_frame_contract": attitude.get(
                "frame_contract",
                "PX4 vehicle attitude quaternion rotates body FRD to world NED",
            ),
        }

    def _magnetometer_corrected_yaw_fields(
        self,
        side: str,
        *,
        xmag: object,
        ymag: object,
        zmag: object,
        sensor_device_id: int | None,
        sample_available: bool,
    ) -> dict[str, object]:
        if not sample_available:
            return {
                "corrected_yaw_deg": None,
                "corrected_yaw_state": "WAITING",
                "corrected_yaw_reason": "MAG sample waiting",
                "corrected_yaw_contract": "CAL_MAG-corrected sensor vector; PARAM readback required",
            }
        if sensor_device_id is None:
            return {
                "corrected_yaw_deg": None,
                "corrected_yaw_state": "IDENTITY UNKNOWN",
                "corrected_yaw_reason": "sensor device ID unavailable; corrected yaw not calculated",
                "corrected_yaw_contract": "CAL_MAG-corrected sensor vector; device identity required",
            }
        status_cache = getattr(self.panel, "_px4_mag_status_cache", {}) or {}
        status = status_cache.get(side) if isinstance(status_cache, Mapping) else None
        params = getattr(status, "params", None)
        params = params if isinstance(params, Mapping) else None
        slot = None
        if params is not None:
            for candidate in range(4):
                if _integer(params.get(f"CAL_MAG{candidate}_ID")) == int(sensor_device_id):
                    slot = candidate
                    break
        attitude_cache = getattr(self.panel, "_px4_attitude_cal_status_cache", {}) or {}
        attitude_status = attitude_cache.get(side) if isinstance(attitude_cache, Mapping) else None
        board_params = getattr(attitude_status, "params", None)
        board_params = board_params if isinstance(board_params, Mapping) else None
        calibration = calibrated_magnetometer_yaw_deg(
            x=xmag,
            y=ymag,
            z=zmag,
            calibration_params=params,
            calibration_slot=slot,
            board_params=board_params,
            internal_sensor=int(sensor_device_id) in INTERNAL_MAG_IDS,
        )
        return {
            **calibration,
            "corrected_yaw_contract": (
                "PX4 Magnetometer::Correct-equivalent: CAL_MAG offset/scale/off-diagonal/rotation; "
                "power compensation is withheld when no contemporaneous power sample exists"
            ),
        }

    def magnetometers(self, side: str, *, now_s: float | None = None) -> tuple[dict[str, object], ...]:
        """Project the ROS2-free MAVLink IMU magnetometer streams.

        ``SCALED_IMU*`` values retain the producer's xmag/ymag/zmag sensor
        axes and milligauss unit.  This adapter only presents those values; it
        does not reinterpret them as NED, ENU, PX4 FRD, or camera coordinates.
        """

        now = time.monotonic() if now_s is None else float(now_s)
        states_by_slot = getattr(self.panel, "latest_mag_sensors", {}) or {}
        states_by_slot = states_by_slot.get(side, {}) if isinstance(states_by_slot, Mapping) else {}
        rows: list[dict[str, object]] = []
        axis_contract = (
            "MAVLink SCALED_IMU* producer xmag/ymag/zmag sensor axes; "
            "Workstation applies no coordinate conversion"
        )
        for sensor_slot, imu_label, message_name in MAVLINK_MAG_STREAMS:
            expected_topic = f"mavlink/{message_name}"
            state = states_by_slot.get(sensor_slot) if isinstance(states_by_slot, Mapping) else None
            if state is None:
                yaw_fields = self._magnetometer_yaw_fields(
                    side,
                    xmag=None,
                    ymag=None,
                    sensor_device_id=None,
                    sample_available=False,
                    now_s=now,
                )
                rows.append(
                    {
                        "side": side,
                        "pixhawk": "P1" if side == "left" else "P2",
                        "sensor_slot": sensor_slot,
                        "imu": imu_label,
                        "state": "WAITING",
                        "xmag": None,
                        "ymag": None,
                        "zmag": None,
                        "norm": None,
                        "unit": "mG",
                        "age_s": None,
                        "timestamp_s": None,
                        "px4_timestamp_s": None,
                        "topic": expected_topic,
                        "device_id": None,
                        "axis_contract": axis_contract,
                        "source": "MAVLink SCALED_IMU telemetry",
                        **yaw_fields,
                        **self._magnetometer_corrected_yaw_fields(
                            side,
                            xmag=None,
                            ymag=None,
                            zmag=None,
                            sensor_device_id=None,
                            sample_available=False,
                        ),
                    }
                )
                continue

            topic = str(getattr(state, "topic", "") or expected_topic)
            values = {
                "xmag": _number(getattr(state, "x", None)),
                "ymag": _number(getattr(state, "y", None)),
                "zmag": _number(getattr(state, "z", None)),
                "norm": _number(getattr(state, "magnitude", None)),
            }
            complete = all(values[key] is not None for key in ("xmag", "ymag", "zmag"))
            age_s = _age(getattr(state, "timestamp_s", None), now)
            device_id = _integer(getattr(state, "device_id", None))
            yaw_fields = self._magnetometer_yaw_fields(
                side,
                xmag=values["xmag"],
                ymag=values["ymag"],
                sensor_device_id=device_id,
                sample_available=True,
                now_s=now,
            )
            rows.append(
                {
                    "side": side,
                    "pixhawk": "P1" if side == "left" else "P2",
                    "sensor_slot": sensor_slot,
                    "imu": imu_label,
                    "state": (
                        "PARTIAL"
                        if not complete
                        else "STALE"
                        if age_s is None or age_s > MAVLINK_IMU_STREAM_STALE_S
                        else "LIVE"
                    ),
                    **values,
                    "unit": magnetometer_value_units(topic) or "mG",
                    "age_s": age_s,
                    "timestamp_s": _number(getattr(state, "timestamp_s", None)),
                    "px4_timestamp_s": _number(getattr(state, "px4_timestamp_s", None)),
                    "topic": topic,
                    "device_id": device_id,
                    "axis_contract": axis_contract,
                    "source": (
                        "PX4 Shell sensor_mag readback"
                        if topic.lower().startswith("px4-shell/sensor_mag/")
                        else "MAVLink SCALED_IMU telemetry"
                    ),
                    **yaw_fields,
                    **self._magnetometer_corrected_yaw_fields(
                        side,
                        xmag=values["xmag"],
                        ymag=values["ymag"],
                        zmag=values["zmag"],
                        sensor_device_id=device_id,
                        sample_available=complete,
                    ),
                }
            )
        return tuple(rows)

    def can_magnetometers(self, side: str, *, now_s: float | None = None) -> tuple[dict[str, object], ...]:
        """Project firmware CAN_MAG telemetry, with a CAL_MAG identity fallback.

        xmag/ymag/zmag/Norm are never derived from CAL_MAG parameters -- only
        PX4's DEBUG_VECT CAN_MAG compatibility message proves a CAN sensor is
        currently online and sampling. When that message is absent (older
        firmware, or simply no packet yet on the active transport --
        USB/radio telemetry/Ethernet all work the same way here), Node
        ID/Device ID are instead read from the last CAL_MAG*_ID parameter
        readout in ``panel._px4_mag_status_cache`` (populated by the PX4 MAG
        calibration "Read" action). That is a fixed configuration value, not
        a live sample, so the row is marked "CONFIG" rather than LIVE/STALE.
        """

        now = time.monotonic() if now_s is None else float(now_s)
        states_by_slot = getattr(self.panel, "latest_can_mag_sensors", {}) or {}
        states_by_slot = states_by_slot.get(side, {}) if isinstance(states_by_slot, Mapping) else {}
        rows: list[dict[str, object]] = []
        axis_contract = (
            "firmware CAN magnetometer xmag/ymag/zmag producer sensor axes; "
            "Workstation applies no coordinate conversion"
        )

        def source_for(topic: str) -> str:
            if topic.lower().startswith("px4-shell/sensor_mag/"):
                return "PX4 Shell sensor_mag readback"
            return "PX4 DEBUG_VECT CAN_MAG compatibility telemetry"

        configured_status = getattr(self.panel, "_px4_mag_status_cache", {}) or {}
        configured_status = configured_status.get(side) if isinstance(configured_status, Mapping) else None
        configured_rm: dict[str, tuple[int, int | None]] = {}
        for slot in getattr(configured_status, "slots", ()) or ():
            device_id = _integer(getattr(slot, "device_id", None))
            if device_id in RM3100_PRIMARY_IDS:
                configured_rm["CAN1"] = (
                    device_id,
                    0x7D if device_id == RM3100_PRIMARY_IDS[0] else 0x7C,
                )
            elif device_id in RM3100_BACKUP_IDS:
                configured_rm["CAN2"] = (
                    device_id,
                    0x7B if device_id == RM3100_BACKUP_IDS[0] else None,
                )

        for sensor_slot in ("CAN1", "CAN2"):
            state = states_by_slot.get(sensor_slot) if isinstance(states_by_slot, Mapping) else None
            if state is None:
                configured = configured_rm.get(sensor_slot)
                yaw_fields = self._magnetometer_yaw_fields(
                    side,
                    xmag=None,
                    ymag=None,
                    sensor_device_id=None,
                    sample_available=False,
                    now_s=now,
                )
                rows.append(
                    {
                        "side": side,
                        "pixhawk": "P1" if side == "left" else "P2",
                        "can_port": sensor_slot,
                        "node_id": None if configured is None else configured[1],
                        "device_id": None if configured is None else configured[0],
                        "state": "CONFIG" if configured is not None else "WAITING",
                        "xmag": None,
                        "ymag": None,
                        "zmag": None,
                        "norm": None,
                        "unit": "G",
                        "age_s": None,
                        "topic": "px4-shell/sensor_mag or mavlink/DEBUG_VECT",
                        "axis_contract": axis_contract,
                        "source": (
                            "PX4 CAL_MAG configuration; waiting for Shell sensor_mag sample"
                            if configured is not None
                            else "PX4 Shell sensor_mag / DEBUG_VECT compatibility telemetry"
                        ),
                        **yaw_fields,
                        **self._magnetometer_corrected_yaw_fields(
                            side,
                            xmag=None,
                            ymag=None,
                            zmag=None,
                            sensor_device_id=None if configured is None else configured[0],
                            sample_available=False,
                        ),
                    }
                )
                continue

            topic = str(getattr(state, "topic", "") or "mavlink/DEBUG_VECT")
            age_s = _age(getattr(state, "timestamp_s", None), now)
            values = {
                "xmag": _number(getattr(state, "x", None)),
                "ymag": _number(getattr(state, "y", None)),
                "zmag": _number(getattr(state, "z", None)),
                "norm": _number(getattr(state, "magnitude", None)),
            }
            complete = all(values[key] is not None for key in ("xmag", "ymag", "zmag"))
            shell_source = topic.lower().startswith("px4-shell/sensor_mag/")
            if not complete:
                state_label = "PARTIAL"
            elif age_s is None or age_s > (15.0 if shell_source else 5.0):
                state_label = "DISCONNECTED"
            elif age_s > (6.0 if shell_source else 1.0):
                state_label = "STALE"
            else:
                state_label = "LIVE"
            device_id = _integer(getattr(state, "device_id", None))
            yaw_fields = self._magnetometer_yaw_fields(
                side,
                xmag=values["xmag"],
                ymag=values["ymag"],
                sensor_device_id=device_id,
                sample_available=True,
                now_s=now,
            )
            rows.append(
                {
                    "side": side,
                    "pixhawk": "P1" if side == "left" else "P2",
                    "can_port": sensor_slot,
                    "node_id": _integer(getattr(state, "node_id", None)),
                    "device_id": device_id,
                    "state": state_label,
                    **values,
                    "unit": magnetometer_value_units(topic) or "G",
                    "age_s": age_s,
                    "topic": topic,
                    "axis_contract": axis_contract,
                    "source": source_for(topic),
                    **yaw_fields,
                    **self._magnetometer_corrected_yaw_fields(
                        side,
                        xmag=values["xmag"],
                        ymag=values["ymag"],
                        zmag=values["zmag"],
                        sensor_device_id=device_id,
                        sample_available=complete,
                    ),
                }
            )
        return tuple(rows)

    def magnetometers_for_imu_view(
        self, side: str, *, now_s: float | None = None
    ) -> tuple[dict[str, object], ...]:
        """Project the IMU-table view without relabeling CAN data as SCALED_IMU.

        The first three rows are the standard ``SCALED_IMU*`` streams.  The
        final two rows are the existing dedicated ``DEBUG_VECT``
        ``CAN1_MAG``/``CAN2_MAG`` compatibility samples from the stable PX4
        path.  Keeping the two sources separate in the evidence dictionaries
        preserves their topic, units, identity, and yaw-correlation rules.
        """

        rows = list(self.magnetometers(side, now_s=now_s))
        for can_row in self.can_magnetometers(side, now_s=now_s):
            can_port = str(can_row.get("can_port") or "CAN?")
            rows.append(
                {
                    **can_row,
                    "imu": f"RM3100 {can_port}",
                    "sensor_slot": can_port,
                }
            )
        return tuple(rows)

    def magnetometers_for_cal_mag_view(
        self, side: str, *, now_s: float | None = None
    ) -> tuple[dict[str, object], ...]:
        """Project one row per PX4 ``CAL_MAGn`` slot in parameter order.

        ``sensor_mag`` instance numbers and MAVLink ``SCALED_IMU*`` stream
        numbers are transport/topology identities, not calibration-slot
        identities.  The operator-facing MAG table therefore uses the
        configured ``CAL_MAGn_ID`` as its row key and only attaches a live
        sample whose device ID matches that slot.
        """

        now = time.monotonic() if now_s is None else float(now_s)
        status_cache = getattr(self.panel, "_px4_mag_status_cache", {}) or {}
        status = status_cache.get(side) if isinstance(status_cache, Mapping) else None
        params = getattr(status, "params", {}) if status is not None else {}
        params = params if isinstance(params, Mapping) else {}
        slots_by_number = {
            int(getattr(slot, "slot")): slot
            for slot in (getattr(status, "slots", ()) or ())
            if getattr(slot, "slot", None) is not None
        }

        # Standard SCALED_IMU and dedicated CAN evidence are kept in separate
        # stores.  Prefer a live dedicated CAN sample when both paths expose
        # the same identity; do not let a CONFIG placeholder hide live data.
        source_rows: dict[int, dict[str, object]] = {}
        for row in self.magnetometers(side, now_s=now):
            device_id = _integer(row.get("device_id"))
            if device_id is not None:
                source_rows[device_id] = dict(row)
        for row in self.can_magnetometers(side, now_s=now):
            device_id = _integer(row.get("device_id"))
            if device_id is None:
                continue
            existing = source_rows.get(device_id)
            if existing is None or str(row.get("state")) not in {"CONFIG", "WAITING"}:
                source_rows[device_id] = dict(row)

        rows: list[dict[str, object]] = []
        for slot_number in MAG_SLOTS:
            slot = slots_by_number.get(slot_number)
            configured_device_id = _integer(
                getattr(slot, "device_id", None)
                if slot is not None
                else params.get(f"CAL_MAG{slot_number}_ID")
            )
            priority = _integer(
                getattr(slot, "priority", None)
                if slot is not None
                else params.get(f"CAL_MAG{slot_number}_PRIO")
            )
            info = identify_mag_device(configured_device_id)
            role = mag_role_for_device_id(configured_device_id)
            source_row = source_rows.get(configured_device_id) if configured_device_id is not None else None
            if source_row is None:
                yaw_fields = self._magnetometer_yaw_fields(
                    side,
                    xmag=None,
                    ymag=None,
                    sensor_device_id=configured_device_id,
                    sample_available=False,
                    now_s=now,
                )
                row = {
                    "side": side,
                    "pixhawk": "P1" if side == "left" else "P2",
                    "cal_slot": slot_number,
                    "cal_mag": mag_priority_slot_key(slot_number),
                    "sensor": info.device_type if configured_device_id is not None else "N/A",
                    "sensor_role": role or "UNKNOWN",
                    "state": "CONFIG" if configured_device_id is not None else "WAITING",
                    "xmag": None,
                    "ymag": None,
                    "zmag": None,
                    "norm": None,
                    "unit": "G",
                    "age_s": None,
                    "topic": "CAL_MAG configuration",
                    "device_id": configured_device_id,
                    "priority": priority,
                    "source": (
                        "PX4 CAL_MAG configuration; waiting for matching sensor sample"
                        if configured_device_id is not None
                        else "PX4 CAL_MAG parameter readback waiting"
                    ),
                    **yaw_fields,
                    **self._magnetometer_corrected_yaw_fields(
                        side,
                        xmag=None,
                        ymag=None,
                        zmag=None,
                        sensor_device_id=configured_device_id,
                        sample_available=False,
                    ),
                }
            else:
                row = dict(source_row)
                row.update(
                    {
                        "cal_slot": slot_number,
                        "cal_mag": mag_priority_slot_key(slot_number),
                        "sensor": info.device_type,
                        "sensor_role": role or "UNKNOWN",
                        "device_id": configured_device_id,
                        "priority": priority,
                    }
                )
            rows.append(row)
        return tuple(rows)

    def baseline(self) -> dict[str, object]:
        baseline = getattr(self.panel, "latest_rtk_baseline", None)
        projection = getattr(self.panel, "latest_p1_body_frd_baseline", None)
        quality = getattr(self.panel, "latest_baseline_quality", None)
        selector = getattr(self.panel, "_selected_baseline_gps_states", None)
        if callable(selector):
            selected_left, selected_right = selector()
        else:
            latest = getattr(self.panel, "latest_gps", {}) or {}
            selected_left = latest.get("left") if isinstance(latest, dict) else None
            selected_right = latest.get("right") if isinstance(latest, dict) else None
        source_reader = getattr(self.panel, "_pixhawk_gnss_source", None)
        left_source = str(source_reader("left")) if callable(source_reader) else "GPS1"
        right_source = str(source_reader("right")) if callable(source_reader) else "GPS1"
        quality_values: dict[str, object] = {
            "sample_count": _integer(getattr(quality, "sample_count", None)),
            "fixed_ratio": _number(getattr(quality, "rtk_fixed_ratio", None)),
            "length_std_m": _number(getattr(quality, "baseline_length_std_m", None)),
            "timestamp_delta_ms": _number(getattr(quality, "timestamp_delta_ms", None)),
            "reasons": tuple(getattr(quality, "reasons", ()) or ()),
        }
        projection_values: dict[str, object] = {
            "p1_body_frd_state": "WAITING",
            "p1_body_frd_forward_m": None,
            "p1_body_frd_right_m": None,
            "p1_body_frd_down_m": None,
            "p1_body_frd_attitude_age_s": None,
            "p1_body_frd_source": "P1 attitude-derived projection; waiting for P1 body FRD attitude",
            "p1_body_frd_frame": BODY_FRAME,
            "p1_body_frd_axis_contract": "P1->P2 in P1 body FRD: [Forward, Right, Down]; metres",
        }
        if projection is not None:
            projection_values.update(
                {
                    "p1_body_frd_state": "READY",
                    "p1_body_frd_forward_m": _number(getattr(projection, "forward_m", None)),
                    "p1_body_frd_right_m": _number(getattr(projection, "right_m", None)),
                    "p1_body_frd_down_m": _number(getattr(projection, "down_m", None)),
                    "p1_body_frd_attitude_age_s": _number(getattr(projection, "attitude_age_s", None)),
                    "p1_body_frd_source": "BodyFrdBaselineProjection from raw NED + P1 attitude",
                    "p1_body_frd_frame": str(getattr(projection, "target_frame", BODY_FRAME)),
                    "p1_body_frd_axis_contract": str(
                        getattr(
                            projection,
                            "axis_contract",
                            "P1->P2 in P1 body FRD: [Forward, Right, Down]; metres",
                        )
                    ),
                }
            )
        if baseline is None:
            return {
                "state": "NO DATA",
                "source": "RtkBaselineEstimate from left/right GpsPositionState",
                "axis_contract": "NED = north / east / down; metres",
                "quality": quality_values,
                "left_source": left_source,
                "right_source": right_source,
                "left_fix_type": _integer(getattr(selected_left, "fix_type", None)),
                "right_fix_type": _integer(getattr(selected_right, "fix_type", None)),
                **projection_values,
            }
        return {
            "state": str(getattr(baseline, "fix_quality", "UNKNOWN") or "UNKNOWN"),
            "north_m": _number(getattr(baseline, "north_m", None)),
            "east_m": _number(getattr(baseline, "east_m", None)),
            "down_m": _number(getattr(baseline, "down_m", None)),
            "length_m": _number(getattr(baseline, "baseline_m", None)),
            "left_age_s": _number(getattr(baseline, "left_age_s", None)),
            "right_age_s": _number(getattr(baseline, "right_age_s", None)),
            "warnings": tuple(getattr(baseline, "warnings", ()) or ()),
            "source": "RtkBaselineEstimate from left/right GpsPositionState",
            "axis_contract": "NED = north / east / down; metres",
            "position_frame": "GPS/RTK WGS84 delta projected to local NED; not camera or FRD",
            "quality": quality_values,
            "left_source": left_source,
            "right_source": right_source,
            "left_fix_type": _integer(getattr(selected_left, "fix_type", None)),
            "right_fix_type": _integer(getattr(selected_right, "fix_type", None)),
            **projection_values,
        }

    def baseline_yaw_alignment(self, *, now_s: float | None = None) -> dict[str, object]:
        """Project horizontal Baseline-P1/P2 angles with source freshness.

        This latest-state read model is an operator aid for stationary/slow
        physical alignment.  It does not claim that GNSS and ATTITUDE samples
        are simultaneous, and it never replaces the raw NED baseline or raw
        PX4 attitude evidence.
        """

        now = time.monotonic() if now_s is None else float(now_s)
        baseline = self.baseline()
        attitudes = {
            side: self.attitude(side, now_s=now)
            for side in ("left", "right")
        }
        sources = {
            "left": str(baseline.get("left_source") or "GPS1"),
            "right": str(baseline.get("right_source") or "GPS1"),
        }
        gps = {
            side: self.gps(side, source=sources[side], now_s=now)
            for side in ("left", "right")
        }

        manager = getattr(self.panel, "gnss_operation_manager", None)
        readiness = getattr(getattr(manager, "config", None), "readiness", None)
        try:
            maximum_gps_age_s = float(
                getattr(readiness, "maximum_gps_sample_age_s", 1.0)
            )
        except (TypeError, ValueError):
            maximum_gps_age_s = 1.0
        if not math.isfinite(maximum_gps_age_s) or maximum_gps_age_s < 0.0:
            maximum_gps_age_s = 1.0

        def finite_number(value: object) -> float | None:
            number = _number(value)
            return number if number is not None and math.isfinite(number) else None

        def freshness_state(value: float | None, age_s: float | None, maximum_age_s: float) -> str:
            if value is None:
                return "WAITING"
            if age_s is None or not math.isfinite(age_s) or age_s > maximum_age_s:
                return "STALE"
            return "CURRENT"

        ned_values = tuple(
            finite_number(baseline.get(key))
            for key in ("north_m", "east_m", "down_m")
        )
        raw_yaws = {
            side: finite_number(attitudes[side].get("yaw_deg"))
            for side in ("left", "right")
        }
        compass_yaws = {
            side: (
                None
                if raw_yaws[side] is None
                else normalize_compass_angle_deg(raw_yaws[side], f"{side}_yaw_deg")
            )
            for side in ("left", "right")
        }
        geometry = None
        baseline_reason = ""
        if all(value is not None for value in ned_values):
            geometry = calculate_baseline_yaw_alignment(
                ned_values,  # type: ignore[arg-type]
                p1_yaw_deg=raw_yaws["left"],
                p2_yaw_deg=raw_yaws["right"],
            )
            if geometry is None:
                baseline_reason = "P1→P2 horizontal baseline direction is undefined"
        else:
            baseline_reason = "P1→P2 NED baseline waiting"

        gps_ages = {
            side: finite_number(gps[side].get("age_s"))
            for side in ("left", "right")
        }
        gps_positions_present = {
            side: all(
                finite_number(gps[side].get(key)) is not None
                for key in ("latitude_deg", "longitude_deg", "altitude_m")
            )
            for side in ("left", "right")
        }
        gps_states = {
            side: freshness_state(
                1.0 if gps_positions_present[side] else None,
                gps_ages[side],
                maximum_gps_age_s,
            )
            for side in ("left", "right")
        }
        baseline_state = "WAITING"
        if geometry is not None:
            baseline_state = (
                "CURRENT"
                if all(state == "CURRENT" for state in gps_states.values())
                else "STALE"
            )

        attitude_ages = {
            side: finite_number(attitudes[side].get("age_s"))
            for side in ("left", "right")
        }
        side_states = {
            side: freshness_state(
                compass_yaws[side],
                attitude_ages[side],
                MAVLINK_ATTITUDE_STALE_S,
            )
            for side in ("left", "right")
        }
        if geometry is None:
            state = "WAITING"
        elif baseline_state == "STALE":
            state = "STALE"
        else:
            current_count = sum(value == "CURRENT" for value in side_states.values())
            if current_count == 2:
                state = "CURRENT"
            elif current_count == 1:
                state = "PARTIAL"
            elif any(value == "STALE" for value in side_states.values()):
                state = "STALE"
            else:
                state = "WAITING"

        reasons: list[str] = []
        if baseline_reason:
            reasons.append(baseline_reason)
        elif baseline_state == "STALE":
            reasons.append("selected P1/P2 GNSS baseline is stale")
        for side, label in (("left", "P1"), ("right", "P2")):
            if side_states[side] != "CURRENT":
                reasons.append(f"{label} ATTITUDE {side_states[side].lower()}")

        gps_timestamps = {
            side: finite_number(gps[side].get("timestamp_s"))
            for side in ("left", "right")
        }
        gps_pair_delta_ms = None
        if all(value is not None for value in gps_timestamps.values()):
            gps_pair_delta_ms = abs(
                gps_timestamps["right"] - gps_timestamps["left"]  # type: ignore[operator]
            ) * 1000.0
        baseline_age_values = [value for value in gps_ages.values() if value is not None]
        baseline_age_s = max(baseline_age_values) if baseline_age_values else None

        return {
            "state": state,
            "reason": "; ".join(reasons),
            "baseline_state": baseline_state,
            "p1_state": side_states["left"],
            "p2_state": side_states["right"],
            "baseline_azimuth_deg": (
                None if geometry is None else geometry.baseline_azimuth_deg
            ),
            "p1_yaw_deg": compass_yaws["left"],
            "p2_yaw_deg": compass_yaws["right"],
            "baseline_p1_deg": None if geometry is None else geometry.baseline_p1_deg,
            "baseline_p2_deg": None if geometry is None else geometry.baseline_p2_deg,
            "p2_minus_p1_deg": (
                geometry.p2_minus_p1_deg
                if geometry is not None
                else (
                    None
                    if raw_yaws["left"] is None or raw_yaws["right"] is None
                    else angle_error_deg(raw_yaws["left"], raw_yaws["right"])
                )
            ),
            "baseline_age_s": baseline_age_s,
            "p1_attitude_age_s": attitude_ages["left"],
            "p2_attitude_age_s": attitude_ages["right"],
            "gps_pair_delta_ms": gps_pair_delta_ms,
            "maximum_gps_age_s": maximum_gps_age_s,
            "attitude_stale_age_s": MAVLINK_ATTITUDE_STALE_S,
            "p1_gps_source": sources["left"],
            "p2_gps_source": sources["right"],
            "p1_fix_type": gps["left"].get("fix_type"),
            "p2_fix_type": gps["right"].get("fix_type"),
            "p1_rtk_label": gps["left"].get("rtk_label", gps["left"].get("state", "NO DATA")),
            "p2_rtk_label": gps["right"].get("rtk_label", gps["right"].get("state", "NO DATA")),
            "p1_gps_age_s": gps_ages["left"],
            "p2_gps_age_s": gps_ages["right"],
            "baseline_quality": baseline.get("state", "NO DATA"),
            "baseline_length_m": finite_number(baseline.get("length_m")),
            "source": "selected P1/P2 GpsPositionState + P1/P2 MAVLink ATTITUDE",
            "clock": "latest app receive monotonic snapshots; not simultaneous measurement",
            "frame_contract": "P1→P2 horizontal local NED; P1/P2 body FRD yaw in world NED",
            "angle_contract": (
                "Baseline-Pn = wrap180(azimuth(P1→P2) - Pn yaw), Pn=P1/P2; "
                "+ is clockwise/right, - is counter-clockwise/left"
            ),
        }

    def drone_pixhawk_geometry(self, *, now_s: float | None = None) -> dict[str, object]:
        """Project Dr/P1/P2 relative vectors and horizontal triangle angles.

        The midpoint vector uses the existing GNSS local-level FRD geometry.
        The P1/P2 vectors are the same GNSS world-NED vectors expressed in the
        corresponding PX4 body-FRD frame.  The two point angles use only the
        horizontal North/East components and are independent of Pixhawk yaw.
        """

        now = time.monotonic() if now_s is None else float(now_s)
        if not math.isfinite(now):
            raise ValueError("now_s must be finite")

        latest_gps = getattr(self.panel, "latest_gps", {}) or {}
        latest_gps = latest_gps if isinstance(latest_gps, Mapping) else {}

        def selected_gps(side: str) -> tuple[GpsPositionState | None, str]:
            selector = getattr(self.panel, "_selected_rover_gps_state", None)
            if callable(selector):
                try:
                    selected = selector(side)
                except Exception:
                    selected = None
                if not isinstance(selected, GpsPositionState):
                    selected = None
                if side == "drone":
                    if selected is None:
                        selected = latest_gps.get("drone")
                    return (
                        selected if isinstance(selected, GpsPositionState) else None,
                        "DRONE",
                    )
                source_reader = getattr(self.panel, "_pixhawk_gnss_source", None)
                source = (
                    str(source_reader(side)).upper()
                    if callable(source_reader)
                    else "GPS1"
                )
                return selected, source if source in {"GPS1", "GPS2"} else "GPS1"
            if side == "drone":
                selected = latest_gps.get("drone")
                return (
                    selected if isinstance(selected, GpsPositionState) else None,
                    "DRONE",
                )
            source_reader = getattr(self.panel, "_pixhawk_gnss_source", None)
            source = (
                str(source_reader(side)).upper()
                if callable(source_reader)
                else "GPS1"
            )
            source = source if source in {"GPS1", "GPS2"} else "GPS1"
            key = side if source == "GPS1" else f"{side}_GPS2"
            selected = latest_gps.get(key)
            return selected if isinstance(selected, GpsPositionState) else None, source

        p1, p1_source = selected_gps("left")
        p2, p2_source = selected_gps("right")
        drone, _drone_source = selected_gps("drone")
        if not callable(getattr(self.panel, "_selected_rover_gps_state", None)):
            drone = latest_gps.get("drone")

        estimate = estimate_gnss_baseline_center_frd(
            p1,
            p2,
            drone,
            now_s=now,
            maximum_age_s=DEFAULT_MAXIMUM_GNSS_AGE_S,
            maximum_timestamp_spread_s=DEFAULT_MAXIMUM_GNSS_TIMESTAMP_SPREAD_S,
        )
        geometry = estimate.geometry
        geometry_state = (
            "WAITING"
            if geometry is None
            else "STALE"
            if estimate.state == "STALE"
            else "CURRENT"
        )

        def finite_age(state: object | None) -> float | None:
            if state is None:
                return None
            age = _age(getattr(state, "timestamp_s", None), now)
            return age if age is not None and math.isfinite(age) else None

        p1_age = finite_age(p1)
        p2_age = finite_age(p2)
        drone_age = finite_age(drone)
        gps_ages = tuple(
            age for age in (p1_age, p2_age, drone_age) if age is not None
        )
        gps_age = max(gps_ages) if gps_ages else None

        def attitude_snapshot(side: str) -> dict[str, object]:
            return self.attitude(side, now_s=now)

        attitudes = {
            "p1": attitude_snapshot("left"),
            "p2": attitude_snapshot("right"),
        }

        def body_attitude_state(data: Mapping[str, object]) -> str:
            q = data.get("q_wxyz")
            age = data.get("age_s")
            if not isinstance(q, (tuple, list)) or len(q) != 4:
                return "WAITING"
            try:
                finite_q = all(math.isfinite(float(value)) for value in q)
                finite_age_value = age is not None and math.isfinite(float(age))
            except (TypeError, ValueError):
                return "WAITING"
            if not finite_q or not finite_age_value:
                return "WAITING"
            return (
                "STALE"
                if float(age) > MAVLINK_ATTITUDE_STALE_S
                else "CURRENT"
            )

        p1_attitude_state = body_attitude_state(attitudes["p1"])
        p2_attitude_state = body_attitude_state(attitudes["p2"])

        def vector_row(
            key: str,
            label: str,
            vector: tuple[float, float, float] | None,
            *,
            state: str,
            frame: str,
            age_s: float | None,
            source: str,
            reason: str,
        ) -> dict[str, object]:
            if vector is None:
                return {
                    "key": key,
                    "label": label,
                    "state": state,
                    "forward_m": None,
                    "right_m": None,
                    "down_m": None,
                    "range_m": None,
                    "frame": frame,
                    "age_s": age_s,
                    "source": source,
                    "reason": reason,
                }
            return {
                "key": key,
                "label": label,
                "state": state,
                "forward_m": vector[0],
                "right_m": vector[1],
                "down_m": vector[2],
                "range_m": vector_range_m(vector),
                "frame": frame,
                "age_s": age_s,
                "source": source,
                "reason": reason,
            }

        def combined_body_state(base_state: str, attitude_state: str) -> str:
            if base_state == "WAITING":
                return "WAITING"
            if base_state == "STALE" or attitude_state == "STALE":
                return "STALE"
            if attitude_state == "CURRENT":
                return "CURRENT"
            return "WAITING"

        vectors: list[dict[str, object]] = []
        if geometry is None:
            midpoint_vector = None
            p1_vector_ned = None
            p2_vector_ned = None
        else:
            midpoint_vector = geometry.vector_m
            p1_vector_ned = geometry.drone_from_p1_ned_m
            p2_vector_ned = subtract_vectors(
                geometry.drone_from_p1_ned_m,
                geometry.baseline_ned_m,
            )

        midpoint_reason = (
            estimate.reason
            if geometry is None or geometry_state != "CURRENT"
            else "Three GNSS positions are current"
        )
        vectors.append(
            vector_row(
                "baseline_center",
                "Dr from Baseline Center",
                midpoint_vector,
                state=geometry_state,
                frame=(
                    str(getattr(geometry, "frame", "gnss_baseline_center_local_level_frd"))
                    if geometry is not None
                    else "gnss_baseline_center_local_level_frd"
                ),
                age_s=gps_age,
                source="selected P1/P2 GNSS + Drone GNSS; midpoint geometry",
                reason=midpoint_reason,
            )
        )

        def transformed_body_vector(
            vector_ned: tuple[float, float, float] | None,
            attitude: Mapping[str, object],
        ) -> tuple[float, float, float] | None:
            if vector_ned is None:
                return None
            q = attitude.get("q_wxyz")
            if not isinstance(q, (tuple, list)) or len(q) != 4:
                return None
            try:
                return world_ned_vector_to_body_frd(vector_ned, q)
            except (TypeError, ValueError):
                return None

        p1_body_vector = transformed_body_vector(p1_vector_ned, attitudes["p1"])
        p2_body_vector = transformed_body_vector(p2_vector_ned, attitudes["p2"])
        p1_body_state = combined_body_state(geometry_state, p1_attitude_state)
        p2_body_state = combined_body_state(geometry_state, p2_attitude_state)
        p1_body_reason = (
            estimate.reason
            if p1_attitude_state in {"CURRENT", "STALE"}
            else "P1 ATTITUDE waiting or invalid"
        )
        p2_body_reason = (
            estimate.reason
            if p2_attitude_state in {"CURRENT", "STALE"}
            else "P2 ATTITUDE waiting or invalid"
        )
        vectors.append(
            vector_row(
                "p1",
                "Dr from P1",
                p1_body_vector,
                state=p1_body_state,
                frame=f"{BODY_FRAME} (P1 body)",
                age_s=max(
                    (age for age in (gps_age, attitudes["p1"].get("age_s")) if age is not None),
                    default=None,
                ),
                source="P1→Dr GNSS vector + P1 MAVLink ATTITUDE",
                reason=p1_body_reason,
            )
        )
        vectors.append(
            vector_row(
                "p2",
                "Dr from P2",
                p2_body_vector,
                state=p2_body_state,
                frame=f"{BODY_FRAME} (P2 body)",
                age_s=max(
                    (age for age in (gps_age, attitudes["p2"].get("age_s")) if age is not None),
                    default=None,
                ),
                source="P2→Dr GNSS vector + P2 MAVLink ATTITUDE",
                reason=p2_body_reason,
            )
        )

        angles = None
        if geometry is not None:
            angles = calculate_horizontal_triangle_angles(
                (0.0, 0.0, 0.0),
                geometry.baseline_ned_m,
                geometry.drone_from_p1_ned_m,
            )

        def angle_row(
            key: str,
            label: str,
            value: float | None,
            *,
            vertex: str,
        ) -> dict[str, object]:
            state = geometry_state if value is not None else "WAITING"
            reason = (
                estimate.reason
                if value is not None
                else "horizontal ray is undefined"
                if geometry is not None
                else estimate.reason
            )
            return {
                "key": key,
                "label": label,
                "angle_deg": value,
                "state": state,
                "vertex": vertex,
                "age_s": gps_age,
                "source": "P1/P2/Drone GNSS horizontal positions",
                "frame": "horizontal local NED (North/East only)",
                "reason": reason,
            }

        angles_rows = (
            angle_row(
                "dr_p1_p2",
                "∠Dr-P1-P2",
                None if angles is None else angles.at_p1_deg,
                vertex="P1",
            ),
            angle_row(
                "dr_p2_p1",
                "∠Dr-P2-P1",
                None if angles is None else angles.at_p2_deg,
                vertex="P2",
            ),
        )

        angle_values = tuple(row.get("angle_deg") for row in angles_rows)
        if geometry_state == "STALE":
            overall_state = "STALE"
        elif geometry_state != "CURRENT":
            overall_state = geometry_state
        else:
            current_attitudes = sum(
                state == "CURRENT"
                for state in (p1_attitude_state, p2_attitude_state)
            )
            if current_attitudes == 2:
                overall_state = "CURRENT" if all(value is not None for value in angle_values) else "PARTIAL"
            elif current_attitudes == 1:
                overall_state = "PARTIAL"
            elif any(
                state == "STALE"
                for state in (p1_attitude_state, p2_attitude_state)
            ):
                overall_state = "STALE"
            else:
                overall_state = "WAITING"

        reasons: list[str] = []
        if estimate.reason and geometry_state != "CURRENT":
            reasons.append(estimate.reason)
        if p1_attitude_state not in {"CURRENT"}:
            reasons.append(f"P1 ATTITUDE {p1_attitude_state.lower()}")
        if p2_attitude_state not in {"CURRENT"}:
            reasons.append(f"P2 ATTITUDE {p2_attitude_state.lower()}")
        if geometry_state == "CURRENT" and any(value is None for value in angle_values):
            reasons.append("one or both horizontal triangle angles are undefined")
        return {
            "state": overall_state,
            "reason": "; ".join(dict.fromkeys(reasons)) or "all geometry inputs current",
            "vectors": tuple(vectors),
            "angles": angles_rows,
            "p1_gps_source": p1_source,
            "p2_gps_source": p2_source,
            "p1_gps_age_s": p1_age,
            "p2_gps_age_s": p2_age,
            "drone_gps_age_s": drone_age,
            "source": "selected P1/P2 GNSS + Drone GNSS + P1/P2 MAVLink ATTITUDE",
            "clock": "latest app receive monotonic snapshots; not simultaneous measurement",
            "frame_contract": (
                "midpoint→Dr is GNSS baseline-center local-level FRD; "
                "P1/P2→Dr are respective PX4 body FRD vectors"
            ),
            "angle_contract": (
                "∠Dr-P1-P2 and ∠Dr-P2-P1 use horizontal North/East only; "
                "unsigned internal angles in [0,180] degrees"
            ),
        }

    def gnss_operation(self, *, now_s: float | None = None) -> dict[str, object]:
        """Project Base GNSS, Survey-in, NTRIP, and RTCM evidence.

        Base coordinates retain their producer WGS84/height contract. Survey-in
        ECEF means are explicitly named and never moved into NED, FRD, or camera
        fields. Rover position and baseline evidence remain owned by ``gps`` and
        ``baseline`` respectively.
        """

        now = time.monotonic() if now_s is None else float(now_s)
        stats = getattr(self.panel, "gnss_base_rtcm_stats", {}) or {}
        stats = stats if isinstance(stats, Mapping) else {}
        injection = stats.get("injection", {})
        injection = injection if isinstance(injection, Mapping) else {}
        routes: dict[str, dict[str, object]] = {}
        for side in ("left", "right", "drone"):
            route = injection.get(side, {})
            route = route if isinstance(route, Mapping) else {}
            last_delivery = _number(route.get("last_delivery_s"))
            routes[side] = {
                "frames": _integer(route.get("frames"), 0),
                "bytes": _integer(route.get("bytes"), 0),
                "packets": _integer(route.get("packets"), 0),
                "fail": _integer(route.get("fail"), 0),
                "delivery_age_s": None if last_delivery is None else max(0.0, now - last_delivery),
            }
        manager = getattr(self.panel, "gnss_operation_manager", None)
        metadata_method = getattr(manager, "metadata", None)
        try:
            metadata = metadata_method() if callable(metadata_method) else {}
        except Exception:
            metadata = {}
        metadata = metadata if isinstance(metadata, Mapping) else {}
        config = getattr(manager, "config", None)
        config_method = getattr(config, "to_mapping", None)
        try:
            config_values = config_method() if callable(config_method) else {}
        except Exception:
            config_values = {}
        config_values = config_values if isinstance(config_values, Mapping) else {}
        local_config = config_values.get("local_base", {})
        local_config = local_config if isinstance(local_config, Mapping) else {}
        network_config = config_values.get("network", {})
        network_config = network_config if isinstance(network_config, Mapping) else {}
        readiness_config = config_values.get("readiness", {})
        readiness_config = readiness_config if isinstance(readiness_config, Mapping) else {}
        fixed_config = config_values.get("fixed_base", {})
        fixed_config = fixed_config if isinstance(fixed_config, Mapping) else {}

        candidate_rows = getattr(self.panel, "_gnss_base_candidate_rows", ()) or ()
        candidates: list[dict[str, object]] = []
        for row in candidate_rows:
            if not isinstance(row, Mapping):
                continue
            messages = row.get("messages", "")
            if isinstance(messages, (tuple, list)):
                messages = ", ".join(str(message) for message in messages)
            candidates.append(
                {
                    "port": str(row.get("port", "") or ""),
                    "target": str(row.get("target", "") or ""),
                    "baud": _integer(row.get("baud")),
                    "status": str(row.get("status", "") or ""),
                    "messages": str(messages or ""),
                    "ubx": bool(row.get("ubx", False)),
                }
            )

        source_identity = metadata.get("correction_source")
        source_identity = source_identity if isinstance(source_identity, Mapping) else {}
        survey = getattr(self.panel, "latest_gnss_base_survey_in", None)
        survey = survey if isinstance(survey, Mapping) else {}
        survey_samples = getattr(self.panel, "gnss_base_survey_samples", ()) or ()
        if not survey and survey_samples:
            try:
                _sample_time, duration_s, accuracy_m, active, valid = survey_samples[-1]
                survey = {
                    "duration_s": duration_s,
                    "mean_accuracy_m": accuracy_m,
                    "active": active,
                    "valid": valid,
                }
            except (IndexError, TypeError, ValueError):
                survey = {}

        coordinate_samples = getattr(self.panel, "gnss_coordinate_samples", ()) or ()
        latest_coordinate = None
        try:
            if coordinate_samples:
                latest_coordinate = coordinate_samples[-1]
        except (IndexError, TypeError):
            latest_coordinate = None
        coordinate = {
            "latitude_deg": _number(getattr(latest_coordinate, "latitude_deg", None)),
            "longitude_deg": _number(getattr(latest_coordinate, "longitude_deg", None)),
            "ellipsoid_height_m": _number(getattr(latest_coordinate, "ellipsoid_height_m", None)),
            "msl_altitude_m": _number(getattr(latest_coordinate, "msl_altitude_m", None)),
            "selected_height_m": _number(getattr(latest_coordinate, "selected_height_m", None)),
            "height_type": str(getattr(getattr(latest_coordinate, "height_type", None), "value", "UNKNOWN")),
            "fixed": None if latest_coordinate is None else bool(getattr(latest_coordinate, "fixed", False)),
            "device_id": str(getattr(latest_coordinate, "device_id", "") or ""),
            "age_s": _age(getattr(latest_coordinate, "timestamp_s", None), now),
            "position_contract": "WGS84 latitude/longitude degrees; heights metres with explicit height type",
            "fix_type": getattr(latest_coordinate, "fix_type", None),
            "carrier_solution": getattr(latest_coordinate, "carrier_solution", None),
            "fix_stage_label": (
                "--" if latest_coordinate is None else str(getattr(latest_coordinate, "fix_stage_label", "UNKNOWN"))
            ),
        }
        satellite_quality = getattr(self.panel, "gnss_base_satellite_quality", None)
        satellite_quality = satellite_quality if isinstance(satellite_quality, Mapping) else {}
        satellites = {
            "count": getattr(self.panel, "gnss_base_satellite_count", None),
            "count_source": str(getattr(self.panel, "gnss_base_satellite_count_source", "") or ""),
            "used": _integer(satellite_quality.get("used")),
            "tracked": _integer(satellite_quality.get("tracked")),
            "avg_cno_dbhz": _number(satellite_quality.get("avg_cno_dbhz")),
        }

        profile = getattr(self.panel, "gnss_fixed_base_profile", None)
        profile_method = getattr(profile, "to_mapping", None)
        try:
            profile_values = profile_method() if callable(profile_method) else {}
        except Exception:
            profile_values = {}
        profile_values = profile_values if isinstance(profile_values, Mapping) else {}
        live_profile = getattr(self.panel, "gnss_fixed_base_live_profile", None)
        live_profile_method = getattr(live_profile, "to_mapping", None)
        try:
            live_profile_values = live_profile_method() if callable(live_profile_method) else {}
        except Exception:
            live_profile_values = {}
        live_profile_values = live_profile_values if isinstance(live_profile_values, Mapping) else {}
        readiness = None
        readiness_method = getattr(manager, "readiness", None)
        if callable(readiness_method):
            try:
                readiness = readiness_method()
            except Exception:
                readiness = None
        state = getattr(manager, "state", "UNKNOWN")
        state = str(getattr(state, "value", state) or "UNKNOWN")
        elapsed_s = max(0.0, now - float(stats.get("start_s") or now))
        operation_mode = str(
            metadata.get("gnss_operation_mode", config_values.get("operation_mode", "UNKNOWN"))
            or "UNKNOWN"
        )
        network_stats = getattr(self.panel, "gnss_network_stats", None)
        ntrip_health = evaluate_ntrip_health(
            network_mode=operation_mode in {"NETWORK_ASSISTED_FIXED_BASE", "NETWORK_RTK_DIRECT"},
            connected=bool(getattr(self.panel, "gnss_network_connected", False)),
            running=getattr(self.panel, "gnss_network_thread", None) is not None,
            phase=str(getattr(self.panel, "gnss_network_phase", "DISCONNECTED") or "DISCONNECTED"),
            connected_at_s=getattr(network_stats, "connected_at_monotonic_s", None),
            last_chunk_s=getattr(network_stats, "last_chunk_monotonic_s", None),
            last_rtcm_s=stats.get("last_frame_s"),
            now_s=now,
            transport_stale_after_s=_number(network_config.get("read_timeout_s"), 10.0) or 10.0,
            correction_stale_after_s=_number(readiness_config.get("maximum_correction_age_s"), 3.0) or 3.0,
            bytes_received=getattr(network_stats, "bytes_received", 0),
            chunks_received=getattr(network_stats, "chunks_received", 0),
            reconnect_count=getattr(self.panel, "gnss_network_reconnect_count", 0),
            last_error=str(getattr(self.panel, "gnss_network_last_error", "") or ""),
        )
        ntrip_message_types = stats.get("message_types")
        ntrip_message_types = (
            {str(key): _integer(value, 0) for key, value in ntrip_message_types.items()}
            if isinstance(ntrip_message_types, Mapping)
            else {}
        )
        # A fresh, high-rate RTCM stream can still be useless for RTK: 1005/1006
        # (station coordinate) and 1070-1129 (MSM observables) are the message
        # classes an RTK solution actually needs. Ephemeris/antenna-descriptor
        # messages (1019/1020/1033/1042/1007/...) alone will never converge.
        ntrip_base_position_present = any(
            int(key) in {1005, 1006} for key in ntrip_message_types if key.lstrip("-").isdigit()
        )
        ntrip_msm_present = any(
            1070 <= int(key) <= 1129 for key in ntrip_message_types if key.lstrip("-").isdigit()
        )
        rover_fixed_since = getattr(self.panel, "gnss_rover_fixed_since", {}) or {}
        rover_fixed_since = rover_fixed_since if isinstance(rover_fixed_since, Mapping) else {}
        rover: dict[str, dict[str, object]] = {}
        for side in ("left", "right"):
            gps = self.gps(side, now_s=now)
            fix_type = gps.get("fix_type")
            fixed_since_s = _number(rover_fixed_since.get(side))
            rover[side] = {
                "state": str(gps.get("state", "NO DATA") or "NO DATA"),
                "rtk_label": str(gps.get("rtk_label", "UNKNOWN") or "UNKNOWN"),
                "fix_type": fix_type,
                "rtk_fixed": None if fix_type is None else fix_type == 6,
                "age_s": gps.get("age_s"),
                "fixed_age_s": None if fixed_since_s is None else max(0.0, now - fixed_since_s),
            }
        return {
            "state": state,
            "mode": operation_mode,
            "experiment_site": {
                "label": str(config_values.get("site_label", "") or ""),
                "address": str(config_values.get("site_address", "") or ""),
                "coordinate_contract": "WGS84 geodetic latitude/longitude; Base height is explicit ellipsoid/MSL",
            },
            "source_identity": dict(source_identity),
            "invalidation_reason": str(metadata.get("invalidation_reason", "") or ""),
            "base_serial": {
                "connected": bool(getattr(self.panel, "gnss_base_serial_connected", False)),
                "device": str(source_identity.get("base_device", local_config.get("device", "")) or ""),
                "baud": _integer(local_config.get("baud")),
            },
            "base_candidates": tuple(candidates),
            "base_discovery_active": getattr(self.panel, "gnss_base_detect_thread", None) is not None,
            "base_discovery_status": str(
                getattr(self.panel, "workstation_gnss_base_discovery_status", "NOT RUN") or "NOT RUN"
            ),
            "survey_in": {
                "active": bool(survey.get("active", False)),
                "valid": bool(survey.get("valid", False)),
                "duration_s": _integer(survey.get("duration_s"), 0),
                "target_duration_s": _number(local_config.get("survey_in_min_duration_s")),
                "mean_accuracy_m": _number(survey.get("mean_accuracy_m")),
                "target_accuracy_m": _number(local_config.get("survey_in_accuracy_limit_m")),
                "observations": _integer(survey.get("observations")),
                "mean_ecef_m": (
                    _number(survey.get("mean_x_m")),
                    _number(survey.get("mean_y_m")),
                    _number(survey.get("mean_z_m")),
                ),
                "axis_contract": "ECEF [X,Y,Z] metres from UBX NAV-SVIN; not NED, FRD, or camera RDF",
            },
            "ntrip": {
                "connected": bool(getattr(self.panel, "gnss_network_connected", False)),
                "running": getattr(self.panel, "gnss_network_thread", None) is not None,
                "health_state": ntrip_health.state.value,
                "status": str(getattr(self.panel, "gnss_network_status_line", "") or ""),
                "caster_host": str(source_identity.get("caster_host", network_config.get("caster_host", "")) or ""),
                "caster_port": _integer(source_identity.get("caster_port", network_config.get("caster_port"))),
                "mountpoint": str(source_identity.get("mountpoint", network_config.get("mountpoint", "")) or ""),
                "gga_policy": str(source_identity.get("gga_policy", network_config.get("gga_policy", "")) or ""),
                "tls": bool(network_config.get("tls", False)),
                # Username is plain config; the password is never present in
                # network_config (NetworkConfig has no such field) and must
                # never be added here -- see workstation_core/ntrip_credentials.py.
                "username": str(network_config.get("username", "") or ""),
                "username_env": str(network_config.get("username_env", "") or ""),
                "password_env": str(network_config.get("password_env", "") or ""),
                "session_age_s": ntrip_health.session_age_s,
                "last_chunk_age_s": ntrip_health.transport_age_s,
                "last_gga_age_s": _age(getattr(network_stats, "last_gga_monotonic_s", None), now),
                "gga_sent_count": _integer(getattr(network_stats, "gga_sent_count", 0), 0),
                # A VRS caster withholds RTCM without periodic GGA while the
                # transport itself stays "connected" -- this is the reason
                # _gnss_gga_provider() last returned None instead of a
                # sentence, empty once it succeeds again.
                "gga_skip_reason": str(getattr(self.panel, "_gnss_last_gga_skip_reason", "") or ""),
                "gga_skip_age_s": _age(getattr(self.panel, "_gnss_last_gga_skip_monotonic_s", None), now),
                "bytes_received": ntrip_health.bytes_received,
                "chunks_received": ntrip_health.chunks_received,
                "byte_rate_bps": (
                    float(ntrip_health.bytes_received) / ntrip_health.session_age_s
                    if ntrip_health.session_age_s and ntrip_health.session_age_s > 0.0
                    else 0.0
                ),
                "rtcm_age_s": ntrip_health.rtcm_age_s,
                "rtcm_frames": _integer(stats.get("frames"), 0),
                "rtcm_message_types": ntrip_message_types,
                "rtcm_base_position_present": ntrip_base_position_present,
                "rtcm_msm_present": ntrip_msm_present,
                "reconnect_count": ntrip_health.reconnect_count,
                "last_error": ntrip_health.last_error,
                "transport_stale_after_s": _number(network_config.get("read_timeout_s"), 10.0),
                "correction_stale_after_s": _number(readiness_config.get("maximum_correction_age_s"), 3.0),
            },
            "base_coordinate": coordinate,
            "satellites": satellites,
            "fixed_base": {
                "sample_count": _safe_len(coordinate_samples),
                "profile": dict(profile_values),
                "live_profile": dict(live_profile_values),
                "target_acquisition_duration_s": _number(fixed_config.get("acquisition_duration_s")),
                "target_fixed_duration_s": _number(fixed_config.get("minimum_fixed_duration_s")),
                "target_fixed_ratio": _number(fixed_config.get("minimum_fixed_ratio")),
                "maximum_horizontal_scatter_m": _number(fixed_config.get("maximum_horizontal_scatter_m")),
                "maximum_vertical_scatter_m": _number(fixed_config.get("maximum_vertical_scatter_m")),
                "height_type": str(fixed_config.get("height_type", "UNKNOWN") or "UNKNOWN"),
                "reconfigure_ack": bool(getattr(self.panel, "gnss_fixed_base_reconfigure_ack", False)),
                "reconfigure_readback": bool(getattr(self.panel, "gnss_fixed_base_reconfigure_readback", False)),
                "reconfigure_rtcm": bool(getattr(self.panel, "gnss_fixed_base_reconfigure_rtcm", False)),
            },
            "base_frames": _integer(stats.get("frames"), 0),
            "base_bytes": _integer(stats.get("bytes"), 0),
            "average_frame_rate_hz": float(stats.get("frames") or 0) / elapsed_s if elapsed_s > 0.0 else 0.0,
            "average_byte_rate_bps": float(stats.get("bytes") or 0) / elapsed_s if elapsed_s > 0.0 else 0.0,
            "last_frame_age_s": _age(stats.get("last_frame_s"), now),
            "routes": routes,
            "rover": rover,
            "readiness_limits": {
                "minimum_fix_type": _integer(readiness_config.get("minimum_fix_type"), 3),
                "minimum_fixed_duration_s": _number(readiness_config.get("minimum_fixed_duration_s"), 30.0),
                "maximum_correction_age_s": _number(readiness_config.get("maximum_correction_age_s"), 3.0),
            },
            "readiness": (
                readiness.to_metadata()
                if readiness is not None and callable(getattr(readiness, "to_metadata", None))
                else {}
            ),
            "contract": "RTCM base stream → MAVLink GPS_RTCM_DATA route; bytes/packets are producer counters",
        }

    def camera_connection_candidates(self) -> dict[str, object]:
        """Project discovered camera endpoints without exposing owner-only objects."""

        rows = getattr(self.panel, "_camera_candidate_rows", ()) or ()
        candidates: list[dict[str, object]] = []
        for row in rows:
            if not isinstance(row, Mapping):
                continue
            interface = row.get("interface")
            candidates.append(
                {
                    "kind": str(row.get("kind", "") or ""),
                    "address": str(row.get("address", "") or ""),
                    "host": str(row.get("host", "") or ""),
                    "port": str(row.get("port", "") or ""),
                    "status": str(row.get("status", "") or ""),
                    "interface": str(getattr(interface, "name", "") or ""),
                    "mac": str(getattr(interface, "mac", "") or ""),
                    "host_ip": str(row.get("host_ip", "") or ""),
                }
            )
        raw_network_roles = getattr(self.panel, "network_role_assignments", {}) or {}
        if not isinstance(raw_network_roles, Mapping) or not raw_network_roles:
            interface_edit = getattr(self.panel, "interface_edit", None)
            interface_name = (
                interface_edit.text().strip()
                if interface_edit is not None and hasattr(interface_edit, "text")
                else ""
            )
            raw_network_roles = {
                "camera": {"interface": interface_name, "stable_id": ""}
            }
        return {
            "candidates": tuple(candidates),
            "discovery_active": any(
                getattr(self.panel, name, None) is not None
                for name in ("camera_network_discovery_thread", "camera_auto_detect_thread")
            ),
            "status": str(getattr(self.panel, "workstation_camera_discovery_status", "NOT RUN") or "NOT RUN"),
            **self._network_role_projection(raw_network_roles),
            "contract": "Network/RTSP/control endpoint metadata; no image coordinates or camera-frame values",
        }

    def pixhawk_connection_candidates(self) -> dict[str, object]:
        """Project PC-interface, USB, and MAVLink candidates for explicit P1/P2 assignment."""

        grouped = getattr(self.panel, "_pixhawk_candidate_rows_by_source", None)
        if not isinstance(grouped, Mapping):
            grouped = {"mavlink": getattr(self.panel, "_pixhawk_candidate_rows", ()) or ()}
        candidates: list[dict[str, object]] = []
        for source in ("network", "pc", "mavlink"):
            rows = grouped.get(source, ()) or ()
            for source_row, row in enumerate(rows):
                if not isinstance(row, Mapping):
                    continue
                candidates.append(
                    {
                        "source": source,
                        "source_row": source_row,
                        "kind": str(row.get("kind", "") or ""),
                        "value": str(row.get("value", "") or ""),
                        "resolved_device": str(row.get("resolved_device", "") or ""),
                        "baud": _integer(row.get("baud")),
                        "system_id": _integer(row.get("system_id")),
                        "component_id": _integer(row.get("component_id")),
                        "observed_monotonic_s": _number(row.get("observed_monotonic_s")),
                        "status": str(row.get("status", "") or ""),
                        "interface": str(row.get("interface", "") or ""),
                        "candidate_id": str(row.get("candidate_id", "") or ""),
                        "carrier": str(row.get("carrier", "") or ""),
                        "local_cidr": str(row.get("local_cidr", "") or ""),
                        "local_ip": str(row.get("local_ip", "") or ""),
                        "p1_route": str(row.get("p1_route", "") or ""),
                        "p2_route": str(row.get("p2_route", "") or ""),
                        "p1_route_state": str(row.get("p1_route_state", "") or ""),
                        "p2_route_state": str(row.get("p2_route_state", "") or ""),
                        "p1_endpoint": str(row.get("p1_endpoint", "") or ""),
                        "p2_endpoint": str(row.get("p2_endpoint", "") or ""),
                        "mac": str(row.get("mac", "") or ""),
                        "driver": str(row.get("driver", "") or ""),
                        "device_path": str(row.get("device_path", "") or ""),
                    }
                )
        pixhawk_assignments = getattr(self.panel, "pixhawk_network_assignments", {}) or {}
        raw_network_roles = getattr(self.panel, "network_role_assignments", {}) or {}
        if not isinstance(raw_network_roles, Mapping) or not raw_network_roles:
            raw_network_roles = {
                "p1": {
                    "interface": str(
                        (pixhawk_assignments.get("left") or {}).get("interface") or ""
                    ),
                    "stable_id": "",
                },
                "p2": {
                    "interface": str(
                        (pixhawk_assignments.get("right") or {}).get("interface") or ""
                    ),
                    "stable_id": "",
                },
            }
        return {
            "candidates": tuple(candidates),
            "discovery_active": any(
                getattr(self.panel, name, None) is not None
                for name in (
                    "pixhawk_usb_discovery_thread",
                    "pixhawk_host_ip_discovery_thread",
                    "pixhawk_network_discovery_thread",
                )
            ),
            "status": str(getattr(self.panel, "workstation_pixhawk_discovery_status", "NOT RUN") or "NOT RUN"),
            "revision": int(getattr(self.panel, "pixhawk_discovery_revision", 0) or 0),
            "assignments": {
                side: {
                    **dict(
                        pixhawk_assignments.get(side)
                        or {}
                    ),
                    **dict(
                        (getattr(self.panel, "pixhawk_network_assignment_status", {}) or {}).get(side)
                        or {}
                    ),
                }
                for side in ("left", "right")
            },
            **self._network_role_projection(raw_network_roles),
            "contract": "Host/serial/MAVLink endpoint metadata; no attitude, position, or baseline values",
        }

    def _network_role_projection(
        self, raw_network_roles: Mapping[str, object]
    ) -> dict[str, object]:
        """Network-role state shared by the camera and Pixhawk candidate evidence."""

        return {
            "network_role_status": dict(
                getattr(self.panel, "network_role_status", {}) or {}
            ),
            "network_role_active": getattr(self.panel, "network_role_operation", None) is not None,
            "network_roles": {
                role: dict(value or {})
                for role, value in raw_network_roles.items()
                if isinstance(value, Mapping)
            },
        }

    def pixhawk_recovery(self, side: str, *, now_s: float | None = None) -> dict[str, object]:
        """Project ETH MAV recovery controls and post-recovery telemetry evidence.

        The USB values identify the temporary PX4 shell/control transport. The
        telemetry values remain independent MAVLink receive evidence; this
        projection does not move or reinterpret attitude, GPS, or baseline
        coordinates.
        """

        if side not in {"left", "right"}:
            raise ValueError(f"Unknown Pixhawk side: {side}")
        now = time.monotonic() if now_s is None else float(now_s)
        panel = self.panel
        url_widget = getattr(panel, f"{side}_pixhawk_bootstrap_url_edit", None)
        baud_widget = getattr(panel, f"{side}_pixhawk_bootstrap_baud_spin", None)
        status_by_side = getattr(panel, "pixhawk_usb_recovery_status", {}) or {}
        record = status_by_side.get(side, {}) if isinstance(status_by_side, Mapping) else {}
        record = record if isinstance(record, Mapping) else {}
        active = getattr(panel, "pixhawk_usb_recovery_thread", None) is not None
        state = str(record.get("state", "IDLE") or "IDLE").upper()
        if active:
            state = "RUNNING"
        detail = str(record.get("detail", "") or "")
        updated_s = _number(record.get("updated_s"))
        subscriber = getattr(panel, "mavlink_pixhawk", None)
        health_reader = getattr(subscriber, "mavlink_health_snapshot", None)
        try:
            health = health_reader(side, now_s=now) if callable(health_reader) else None
        except Exception:
            health = None
        health_state = str(
            getattr(getattr(health, "state", None), "value", getattr(health, "state", ""))
            if health is not None
            else ""
        )
        qualification_required_s = float(
            getattr(health, "qualification_required_s", None) or MAVLINK_QUALIFICATION_S
        )
        qualification_text = f"{qualification_required_s:.0f}초"
        if not active and state == "COMMAND_SENT" and health_state == "RECOVERING":
            state = "QUALIFYING"
        elif not active and state in {"COMMAND_SENT", "QUALIFYING"} and health_state == "LIVE":
            state = "RECOVERED"
        provenance = mavlink_link_provenance(panel, side)
        if health is not None:
            attitude_age = _number(getattr(health, "attitude_rx_age_s", None))
            heartbeat_age = _number(getattr(health, "heartbeat_rx_age_s", None))
            attitude_ok = attitude_age is not None and attitude_age <= 2.0
            heartbeat_ok = heartbeat_age is not None and heartbeat_age <= 3.0
        else:
            attitude = self.attitude(side, now_s=now)
            attitude_age = _number(attitude.get("age_s"))
            heartbeat_age = None
            attitude_ok = (
                str(attitude.get("state", "WAITING")).upper() == "LIVE"
                and attitude_age is not None
                and attitude_age < 2.0
            )
            heartbeat_ok = bool(provenance.get("mavlink_heartbeat_seen", False))
        link_open = bool(getattr(health, "listener_open", False)) if health is not None else bool(
            provenance.get("mavlink_link_open", False)
        )
        heartbeat_state = "OK" if heartbeat_ok else "WAITING" if link_open else "NO LINK"
        attitude_state = "OK" if attitude_ok else "WAITING"
        expected_system_id = 1 if side == "left" else 2
        identities = getattr(panel, "pixhawk_usb_identity_evidence", ()) or ()
        matching = [
            item
            for item in identities
            if isinstance(item, Mapping)
            and _integer(item.get("system_id")) == expected_system_id
        ]
        identity = matching[0] if len(matching) == 1 else {}
        usb_url = str(identity.get("device") or _widget_text(url_widget, "--") or "--")
        usb_baud = _integer(identity.get("baud"))
        if usb_baud is None:
            usb_baud = _integer(_widget_value(baud_widget))
        if not detail:
            detail = {
                "IDLE": "대기",
                "RUNNING": "USB identity 재검증 및 one-shot reboot 진행 중",
                "COMMAND_SENT": "USB reboot command 전송; Ethernet packet 재개 대기",
                "QUALIFYING": f"Ethernet packet 재개; {qualification_text} qualification 진행 중",
                "RECOVERED": f"{qualification_text} qualification 통과",
                "CANCELLED": "operator 취소",
                "ERROR": "결과 없음",
            }.get(state, "결과 없음")
        return {
            "side": side,
            "usb_url": usb_url,
            "usb_baud": usb_baud,
            "usb_system_id": _integer(identity.get("system_id")),
            "usb_component_id": _integer(identity.get("component_id")),
            "usb_identity_count": len(matching),
            "state": state,
            "detail": detail,
            "updated_age_s": None if updated_s is None else max(0.0, now - updated_s),
            "active": active,
            "mavlink_link_open": link_open,
            "heartbeat_state": heartbeat_state,
            "attitude_state": attitude_state,
            "telemetry": f"Heartbeat {heartbeat_state} · ATTITUDE {attitude_state}",
            "mavlink_endpoint": str(provenance.get("mavlink_endpoint", "") or ""),
            "liveness_state": health_state,
            "qualification_elapsed_s": _number(getattr(health, "qualification_elapsed_s", None)),
            "qualification_required_s": qualification_required_s,
            "contract": (
                "unique current USB heartbeat sysid + operator confirmation + one-shot reboot; "
                f"success requires Ethernet HB/ATT {qualification_required_s:.0f}-second qualification"
            ),
        }

    def physical_alignment(self) -> dict[str, object]:
        """Return physical-alignment evidence and iteration history."""

        snapshot_method = getattr(self.panel, "workstation_physical_alignment_snapshot", None)
        if callable(snapshot_method):
            try:
                snapshot = snapshot_method()
            except Exception:
                snapshot = None
            if isinstance(snapshot, Mapping):
                return dict(snapshot)

        quality = getattr(self.panel, "latest_physical_alignment_quality", None)
        quality_mapping = (
            quality.to_mapping()
            if quality is not None and callable(getattr(quality, "to_mapping", None))
            else dict(quality)
            if isinstance(quality, Mapping)
            else {}
        )
        left = (getattr(self.panel, "latest_extrinsics", {}) or {}).get("left")
        right = (getattr(self.panel, "latest_extrinsics", {}) or {}).get("right")
        relative_rpy: tuple[float, float, float] | None = None
        if left is not None and right is not None:
            try:
                relative = relative_attitude(left.q, right.q)
                relative_rpy = (relative.roll_deg, relative.pitch_deg, relative.yaw_deg)
            except (AttributeError, TypeError, ValueError):
                relative_rpy = None
        baseline = self.baseline()
        current_frd = tuple(
            baseline.get(key)
            for key in (
                "p1_body_frd_forward_m",
                "p1_body_frd_right_m",
                "p1_body_frd_down_m",
            )
        )
        if not all(value is not None for value in current_frd):
            current_frd = None
        gps = {side: self.gps(side) for side in ("left", "right")}
        return {
            "state": "LIVE" if relative_rpy is not None and baseline.get("state") not in {"NO DATA", "UNKNOWN"} else "WAITING",
            "sampling_active": bool(getattr(self.panel, "baseline_sampling_active", False)),
            "sample_count": _safe_len(getattr(self.panel, "physical_alignment_samples", ())),
            "current_baseline_ned_m": tuple(
                baseline.get(key) for key in ("north_m", "east_m", "down_m")
            ),
            "current_baseline_frd_m": current_frd,
            "relative_rpy_deg": relative_rpy,
            "fix_type": {side: gps[side].get("fix_type") for side in ("left", "right")},
            "quality": quality_mapping,
            "iterations": tuple(getattr(self.panel, "alignment_iteration_history", ()) or ()),
            "physical_adjustment_only": True,
            "coordinate_contract": (
                "raw baseline NED [north,east,down] is preserved; P1→P2 FRD [forward,right,down] "
                "is an attitude-derived observation; ΔR/P/Y is [roll,pitch,yaw]"
            ),
        }

    def detection(self, side: str, *, now_s: float | None = None) -> dict[str, object]:
        now = time.monotonic() if now_s is None else float(now_s)
        observation = getattr(self.panel, f"latest_{side}", None)
        stats_all = getattr(self.panel, "camera_stream_stats", {}) or {}
        stats = stats_all.get(side, {}) if isinstance(stats_all, dict) else {}
        stats = stats if isinstance(stats, dict) else {}
        result: dict[str, object] = {
            "side": side,
            "runtime_state": "TARGET PRESENT" if observation is not None else "NO TARGET",
            "enabled": bool(getattr(self.panel, "yolo_enabled", False)),
            "decoded_hz": _number(stats.get("decoded_hz")),
            "inferred_hz": _number(stats.get("inferred_hz")),
            "displayed_hz": _number(stats.get("displayed_hz")),
            "actual_device": str(stats.get("yolo_actual_device", "") or ""),
            "requested_device": str(stats.get("yolo_requested_device", "") or ""),
            "cpu_fallback": bool(stats.get("yolo_cpu_fallback", False)),
            "source": "DetectionObservation / YoloStreamWorker stats",
            "clock": "detection app receive monotonic; source timestamp metadata retained",
            "pixel_contract": "bbox x1/y1/x2/y2 and center are image pixels (u/v), not metres",
        }
        if observation is None:
            return result
        metadata = getattr(observation, "timestamp_metadata", {}) or {}
        result.update(
            {
                "timestamp_s": _number(getattr(observation, "timestamp_s", None)),
                "age_s": _age(getattr(observation, "timestamp_s", None), now),
                "width_px": _integer(getattr(observation, "width", None)),
                "height_px": _integer(getattr(observation, "height", None)),
                "x1_px": _number(getattr(observation, "x1", None)),
                "y1_px": _number(getattr(observation, "y1", None)),
                "x2_px": _number(getattr(observation, "x2", None)),
                "y2_px": _number(getattr(observation, "y2", None)),
                "center_u_px": _number(getattr(observation, "center", (None, None))[0]),
                "center_v_px": _number(getattr(observation, "center", (None, None))[1]),
                "confidence": _number(getattr(observation, "confidence", None)),
                "class_id": _integer(getattr(observation, "class_id", None)),
                "class_name": str(getattr(observation, "class_name", "") or ""),
                "candidate_count": _safe_len(getattr(observation, "candidate_class_ids", ())),
                "source_timestamp_s": _number(metadata.get("source_timestamp_s")),
                "timestamp_source": str(metadata.get("timestamp_source", "") or ""),
                "timestamp_reference": str(metadata.get("timestamp_reference", "") or ""),
            }
        )
        return result

    def detection_config(self) -> dict[str, object]:
        settings = getattr(self.panel, "settings", None)

        def setting(name: str, default: object = None) -> object:
            value = getattr(settings, name, default)
            value = default if value is None else value
            resolver = getattr(self.panel, "_workstation_setting", None)
            return resolver(name, value) if callable(resolver) else value

        weights = _widget_text(getattr(self.panel, "weights_edit", None), str(setting("yolo_weights_path", "")))
        device = _combo_data(getattr(self.panel, "device_combo", None), setting("yolo_device", "cpu"))
        device = str(device or "cpu")
        gate_names = []
        gate_values = {
            "class": _widget_checked(getattr(self.panel, "yolo_gate_class_checkbox", None), bool(setting("yolo_gate_class_enabled", False))),
            "time": _widget_checked(getattr(self.panel, "yolo_gate_matching_checkbox", None), bool(setting("yolo_gate_matching_enabled", False))),
            "Δt": _widget_checked(getattr(self.panel, "yolo_gate_time_delta_checkbox", None), bool(setting("yolo_gate_time_delta_enabled", False))),
            "vertical Δ": _widget_checked(getattr(self.panel, "yolo_gate_vertical_delta_checkbox", None), bool(setting("yolo_gate_vertical_delta_enabled", False))),
        }
        gate_names.extend(name for name, enabled in gate_values.items() if enabled)
        return {
            "weights_path": weights,
            "device": device,
            "input_size_px": _integer(_widget_value(getattr(self.panel, "input_size_spin", None), setting("yolo_input_size"))),
            "confidence": _number(_widget_value(getattr(self.panel, "conf_spin", None), setting("yolo_conf_threshold"))),
            "iou": _number(_widget_value(getattr(self.panel, "iou_spin", None), setting("yolo_iou_threshold"))),
            "class_gate": _widget_text(getattr(self.panel, "yolo_gate_class_edit", None), str(setting("yolo_gate_class_text", ""))),
            "gate_enabled": tuple(gate_names),
            "max_delta_s": min(
                0.100,
                max(
                    0.0,
                    _number(
                        _widget_value(
                            getattr(self.panel, "yolo_gate_time_delta_spin", None),
                            setting("yolo_gate_max_time_delta_s"),
                        ),
                        0.100,
                    ),
                ),
            ),
            "time_basis": normalize_live_distance_time_basis(
                _combo_data(
                    getattr(self.panel, "yolo_gate_time_basis_combo", None),
                    setting("yolo_gate_time_basis", "app_receive_monotonic"),
                )
            ),
            "time_basis_role": "preferred_with_app_receive_fallback",
            "ntp_evidence_role": "preferred_when_selected_and_verified",
            "max_vertical_delta_px": _number(_widget_value(getattr(self.panel, "yolo_gate_vertical_delta_spin", None), setting("yolo_gate_max_vertical_delta_px"))),
            "source": "AppSettings + existing YOLO configuration widgets",
        }

    def measurement_settings(self) -> dict[str, object]:
        """Project editable Measurement and Time/Sync settings.

        The values are read from the panel's existing controls and settings
        object.  Workstation may send this same shape back through the
        backend facade, but this projection never mutates the owner.
        Units are explicit: focal values are pixels, timing thresholds
        are seconds or milliseconds as labelled, and calibration/extrinsic
        selectors name their frame source rather than converting coordinates.
        """

        settings = getattr(self.panel, "settings", None)

        def setting(name: str, default: object = None) -> object:
            value = getattr(settings, name, default)
            value = default if value is None else value
            resolver = getattr(self.panel, "_workstation_setting", None)
            return resolver(name, value) if callable(resolver) else value

        intrinsics_source = _combo_data(
            getattr(self.panel, "intrinsics_source_combo", None),
            "manual" if _widget_checked(getattr(self.panel, "manual_focal_checkbox", None), bool(setting("manual_focal_override", False))) else "checkerboard",
        )
        distance_mode = _combo_data(
            getattr(self.panel, "distance_mode_combo", None),
            setting("triangulation_extrinsics_source", "pixhawk"),
        )
        return {
            "intrinsics_source": str(intrinsics_source or "checkerboard"),
            "manual_focal_override": _widget_checked(
                getattr(self.panel, "manual_focal_checkbox", None),
                bool(setting("manual_focal_override", False)),
            ),
            "focal_px": _number(
                _widget_value(getattr(self.panel, "focal_spin", None), None),
                900.0,
            ),
            "distance_mode": str(distance_mode or "pixhawk"),
            "measurement_preferred_method": str(
                setting("measurement_preferred_method", "pixhawk_rtk_live")
            ),
            "measurement_method_fallback_order": list(
                setting("measurement_method_fallback_order", ()) or ()
            ),
            "measurement_baseline_yaw_reverse": bool(
                setting("measurement_baseline_yaw_reverse", False)
            ),
            "pixhawk_left_gnss_source": str(
                setting("pixhawk_left_gnss_source", "GPS1")
            ),
            "pixhawk_right_gnss_source": str(
                setting("pixhawk_right_gnss_source", "GPS1")
            ),
            "yolo_input_size_px": _integer(
                _widget_value(getattr(self.panel, "input_size_spin", None), setting("yolo_input_size"))
            ),
            "yolo_confidence": _number(
                _widget_value(getattr(self.panel, "conf_spin", None), setting("yolo_conf_threshold"))
            ),
            "yolo_iou": _number(
                _widget_value(getattr(self.panel, "iou_spin", None), setting("yolo_iou_threshold"))
            ),
            "class_gate_enabled": _widget_checked(
                getattr(self.panel, "yolo_gate_class_checkbox", None),
                bool(setting("yolo_gate_class_enabled", False)),
            ),
            "class_gate": _widget_text(
                getattr(self.panel, "yolo_gate_class_edit", None),
                str(setting("yolo_gate_class_text", "")),
            ),
            "matching_gate_enabled": True,
            "time_basis": normalize_live_distance_time_basis(
                _combo_data(
                    getattr(self.panel, "yolo_gate_time_basis_combo", None),
                    setting("yolo_gate_time_basis", "app_receive_monotonic"),
                )
                or "app_receive_monotonic"
            ),
            "time_basis_role": "preferred_with_app_receive_fallback",
            "ntp_evidence_role": "preferred_when_selected_and_verified",
            "time_delta_gate_enabled": True,
            "max_pair_delta_s": min(
                0.100,
                max(
                    0.0,
                    _number(
                        _widget_value(
                            getattr(self.panel, "yolo_gate_time_delta_spin", None),
                            setting("yolo_gate_max_time_delta_s"),
                        ),
                        0.100,
                    ),
                ),
            ),
            "vertical_delta_gate_enabled": _widget_checked(
                getattr(self.panel, "yolo_gate_vertical_delta_checkbox", None),
                bool(setting("yolo_gate_vertical_delta_enabled", False)),
            ),
            "max_vertical_delta_px": _number(
                _widget_value(
                    getattr(self.panel, "yolo_gate_vertical_delta_spin", None),
                    setting("yolo_gate_max_vertical_delta_px"),
                )
            ),
            "camera_sync_history_window_s": _number(
                _widget_value(
                    getattr(self.panel, "camera_sync_history_window_spin", None),
                    setting("camera_sync_history_window_s"),
                )
            ),
            "camera_sync_receive_pair_max_delta_ms": _number(
                _widget_value(
                    getattr(self.panel, "camera_sync_receive_pair_max_delta_spin", None),
                    _number(setting("camera_sync_receive_pair_max_delta_s"), 0.0) * 1000.0,
                )
            ),
            "camera_sync_rtcp_pair_max_delta_ms": _number(
                _widget_value(
                    getattr(self.panel, "camera_sync_rtcp_pair_max_delta_spin", None),
                    _number(setting("camera_sync_rtcp_pair_max_delta_s"), 0.0) * 1000.0,
                )
            ),
            "camera_sync_max_sr_age_s": _number(
                _widget_value(
                    getattr(self.panel, "camera_sync_max_sr_age_spin", None),
                    setting("camera_sync_max_sr_age_s"),
                )
            ),
            "camera_sync_max_pc_offset_ms": _number(
                _widget_value(
                    getattr(self.panel, "camera_sync_max_pc_offset_spin", None),
                    _number(setting("camera_sync_max_pc_offset_s"), 0.0) * 1000.0,
                )
            ),
            "camera_sync_max_pc_offset_role": "diagnostic-only",
            "camera_sync_clock_rate_error_pct": _number(
                _widget_value(
                    getattr(self.panel, "camera_sync_clock_rate_error_spin", None),
                    _number(setting("camera_sync_max_clock_rate_error"), 0.0) * 100.0,
                )
            ),
            "camera_sync_required_sr_count": _integer(
                _widget_value(
                    getattr(self.panel, "camera_sync_required_sr_count_spin", None),
                    setting("camera_sync_required_sr_count"),
                )
            ),
            "camera_sync_warmup_timeout_s": _number(
                _widget_value(
                    getattr(self.panel, "camera_sync_warmup_timeout_spin", None),
                    setting("camera_sync_warmup_timeout_s"),
                )
            ),
            "camera_sync_mode": str(
                _combo_data(
                    getattr(self.panel, "camera_sync_mode_combo", None),
                    setting("camera_sync_mode", "manual"),
                )
                or "manual"
            ),
            "camera_sync_interval_s": _number(
                _widget_value(
                    getattr(self.panel, "camera_sync_interval_spin", None),
                    _number(setting("camera_sync_interval_s"), 1800.0) / 60.0,
                )
            )
            * 60.0,
            "source": "existing Measurement/Time-Sync owner widgets + AppSettings",
            "coordinate_contract": f"intrinsics/extrinsics source selectors; public output {MEASUREMENT_FRAME} [{MEASUREMENT_AXIS_CONTRACT}]",
        }

    def image_format_status(self) -> dict[str, object]:
        """Latest left/right GIVI/GIRJ/GISR/GIHR/GILC snapshot, if queried."""

        return dict(getattr(self.panel, "wiris_image_format_status", {}) or {})

    def calibration_setup(self) -> dict[str, object]:
        """Project editable calibration setup fields without changing them."""

        settings = getattr(self.panel, "settings", None)
        pair_max_delta_s = _number(
            getattr(settings, "calibration_pair_max_delta_s", 0.100),
            0.100,
        )
        return {
            "zoom_level": _integer(_widget_value(getattr(self.panel, "zoom_level_spin", None))),
            "inner_cols": _integer(_widget_value(getattr(self.panel, "calib_cols_spin", None))),
            "inner_rows": _integer(_widget_value(getattr(self.panel, "calib_rows_spin", None))),
            "square_size_mm": _number(_widget_value(getattr(self.panel, "calib_square_spin", None))),
            "min_images": _integer(_widget_value(getattr(self.panel, "calib_min_images_spin", None))),
            "calculation_pair_count": _integer(
                _widget_value(getattr(self.panel, "calib_calculation_pairs_spin", None))
            ),
            "pair_max_delta_ms": _number(
                _widget_value(
                    getattr(self.panel, "calibration_pair_max_delta_spin", None),
                    float(pair_max_delta_s or 0.0) * 1000.0,
                )
            ),
            "auto_interval_s": _number(
                _widget_value(getattr(self.panel, "calibration_auto_interval_spin", None))
            ),
            "calibration_root": _widget_text(getattr(self.panel, "calib_dir_edit", None)),
            "calibration_root_resolved": str(self.panel._calibration_dirs()[0])
            if hasattr(self.panel, "_calibration_dirs")
            else "",
            "validation_mode": str(
                _combo_data(getattr(self.panel, "checkerboard_validation_mode_combo", None), "pixhawk")
                or "pixhawk"
            ),
            "validation_root": _widget_text(getattr(self.panel, "checkerboard_validation_root_edit", None)),
            "validation_json": _widget_text(getattr(self.panel, "checkerboard_validation_json_edit", None)),
            "validation_runs": _integer(
                _widget_value(getattr(self.panel, "checkerboard_validation_random_runs_spin", None))
            ),
            "validation_pair_count": _integer(
                _widget_value(getattr(self.panel, "checkerboard_validation_random_pair_count_spin", None))
            ),
            "validation_seed": _integer(
                _widget_value(getattr(self.panel, "checkerboard_validation_seed_spin", None))
            ),
            "source": "existing calibration setup widgets",
            "frame_contract": f"checkerboard image pixels/{CAMERA_FRAME}; stereo R/T remains left-camera optical RDF → right-camera optical RDF",
        }

    def session_catalog(self) -> tuple[dict[str, object], ...]:
        """List finalized/retryable session directories for review only."""

        recordings_root = self._app_root() / "recordings"
        session_directories = _recording_session_directories(recordings_root)
        if not session_directories:
            return ()
        entries: list[dict[str, object]] = []
        for path in sorted(
            session_directories,
            key=lambda item: item.name,
            reverse=True,
        ):
            hierarchy_state_path = path / "session.json"
            if hierarchy_state_path.is_file():
                try:
                    hierarchy = json.loads(hierarchy_state_path.read_text(encoding="utf-8"))
                    if not isinstance(hierarchy, Mapping):
                        raise ValueError("session.json must contain a JSON object")
                except (OSError, UnicodeError, ValueError) as exc:
                    entries.append(
                        {
                            "session_id": path.name,
                            "measurement_id": "",
                            "catalog_key": path.name,
                            "layout": "nested-v2",
                            "path": str(path),
                            "session_root": str(path),
                            "status": "INTEGRITY_FAILED",
                            "lifecycle_state": "RECOVERABLE",
                            "completeness_state": "UNKNOWN",
                            "integrity_status": "INTEGRITY_FAILED",
                            "artifacts": (),
                            "artifact_details": (),
                            "source_expectations": (),
                            "error": f"malformed session.json: {exc}",
                        }
                    )
                    continue
                session_id = str(hierarchy.get("session_id") or path.name)
                measurement_rows = {
                    str(row.get("measurement_id") or ""): dict(row)
                    for row in hierarchy.get("measurements", ()) or ()
                    if isinstance(row, Mapping) and row.get("measurement_id")
                }
                child_entries = 0
                measurements_root = path / "measurements"
                if measurements_root.is_dir() and not measurements_root.is_symlink():
                    for child in sorted(
                        (
                            item
                            for item in measurements_root.iterdir()
                            if item.is_dir() and not item.is_symlink()
                        ),
                        key=lambda item: item.name,
                        reverse=True,
                    ):
                        projection = _package_session_projection(
                            child,
                            owning_session_id=session_id,
                            measurement_id=child.name,
                        )
                        indexed_row = measurement_rows.get(child.name, {})
                        if projection is None:
                            row = indexed_row
                            projection = {
                                "session_id": session_id,
                                "measurement_id": child.name,
                                "catalog_key": f"{session_id}::{child.name}",
                                "layout": "nested-v2",
                                "session_root": str(path),
                                "path": str(child),
                                "status": str(row.get("state") or "COLLECTING"),
                                "lifecycle_state": str(row.get("state") or "COLLECTING"),
                                "completeness_state": "PENDING",
                                "evidence_complete": False,
                                "integrity_status": "NOT_VERIFIED",
                                "revision": str(row.get("revision") or ""),
                                "revision_number": row.get("revision_number"),
                                "artifacts": (),
                                "artifact_details": (),
                                "source_expectations": (),
                                "error": "",
                                "retryable": str(row.get("state") or "") == "RECOVERABLE",
                                "legacy": False,
                            }
                        for field in (
                            "measurement_package_id",
                            "recording_generation",
                            "authoritative_jsonl_fragments",
                            "context_fragments",
                            "binary_artifacts",
                        ):
                            if field in indexed_row:
                                projection[field] = indexed_row[field]
                        entries.append(projection)
                        child_entries += 1
                if child_entries == 0:
                    entries.append(
                        {
                            "session_id": session_id,
                            "measurement_id": "",
                            "catalog_key": session_id,
                            "layout": "nested-v2",
                            "session_root": str(path),
                            "path": str(path),
                            "status": str(hierarchy.get("state") or "OPEN"),
                            "lifecycle_state": str(hierarchy.get("state") or "OPEN"),
                            "completeness_state": "PENDING",
                            "evidence_complete": False,
                            "integrity_status": "NOT_VERIFIED",
                            "artifacts": (),
                            "artifact_details": (),
                            "source_expectations": (),
                            "error": "",
                            "retryable": False,
                            "legacy": False,
                        }
                    )
                continue
            package_entry = _package_session_projection(path)
            if package_entry is not None:
                entries.append(package_entry)
                continue
            v2_manifest_path = path / "measurement.json"
            legacy_manifest_path = path / "session_manifest.json"
            manifest_path = (
                v2_manifest_path
                if v2_manifest_path.is_file()
                else legacy_manifest_path
            )
            manifest_is_v2 = manifest_path == v2_manifest_path and manifest_path.is_file()
            manifest: dict[str, object] = {}
            manifest_error = ""
            if manifest_path.is_file():
                try:
                    payload = json.loads(manifest_path.read_text(encoding="utf-8"))
                    if isinstance(payload, dict):
                        manifest = payload
                    else:
                        manifest_error = f"malformed {manifest_path.name}: expected a JSON object"
                except (OSError, UnicodeError, ValueError) as exc:
                    manifest = {}
                    manifest_error = f"malformed {manifest_path.name}: {exc}"
            published = (
                path / ".workstation" / "publish" / "current"
                if manifest_is_v2
                else path / "published"
            )
            published_target = ""
            if published.is_symlink():
                try:
                    published_target = str(published.resolve().relative_to(path.resolve()))
                except (OSError, ValueError):
                    published_target = str(published)
            resource_files = manifest.get("resource_files", {})
            if not isinstance(resource_files, dict):
                resource_files = {}
            artifact_roles = manifest.get("artifact_roles", {})
            if not isinstance(artifact_roles, dict):
                artifact_roles = {}
            artifact_states = manifest.get("artifacts", {})
            if not isinstance(artifact_states, dict):
                artifact_states = {}
            artifact_details = manifest.get("artifact_details", {})
            if not isinstance(artifact_details, dict):
                artifact_details = {}
            artifact_names = {
                str(name)
                for name in (*resource_files.values(), *artifact_roles.keys(), *artifact_states.keys())
                if name not in (None, "")
            }
            for name, detail in artifact_details.items():
                if isinstance(detail, Mapping) and (
                    detail.get("relative_path") or detail.get("path")
                ):
                    artifact_names.add(
                        str(detail.get("relative_path") or detail.get("path"))
                    )
                else:
                    artifact_names.add(str(name))
            artifact_names.update(_run_ledger_artifact_names(path))
            artifacts = tuple(
                sorted(
                    {
                        normalized
                        for name in artifact_names
                        if (normalized := _safe_session_relative_path(path, name)) is not None
                    }
                )
            )
            entries.append(
                {
                    "session_id": path.name,
                    "path": str(path),
                    "status": (
                        str(manifest.get("status") or "unknown")
                        if manifest_is_v2
                        else "LEGACY_UNVERIFIED" if manifest_path.is_file() else "unknown"
                    ),
                    "lifecycle_state": str(manifest.get("status", "unknown") or "unknown"),
                    "completeness_state": (
                        "VERIFIED" if manifest_is_v2 else "LEGACY_UNVERIFIED" if manifest_path.is_file() else "UNKNOWN"
                    ),
                    "evidence_complete": False,
                    "integrity_status": (
                        "VERIFIED" if manifest_is_v2 else "LEGACY_UNVERIFIED" if manifest_path.is_file() else "NOT_VERIFIED"
                    ),
                    "package_schema": "",
                    "package_schema_version": None,
                    "revision": "",
                    "revision_number": None,
                    "package_manifest_path": "",
                    "missing_reasons": (),
                    "failed_reasons": (manifest_error,) if manifest_error else (),
                    "total_bytes": _existing_artifact_byte_total(path, artifacts),
                    "hash_verification": {
                        "status": "VERIFIED" if manifest_is_v2 else "LEGACY_UNVERIFIED" if manifest_path.is_file() else "NOT_RUN",
                        "manifest_status": "AVAILABLE" if manifest_is_v2 else "NOT_AVAILABLE",
                        "artifact_hashes_declared": 0,
                        "artifacts_verified": 0,
                        "artifacts_failed": 0,
                        "artifacts_not_run": len(artifacts),
                    },
                    "hash_verification_summary": {
                        "status": "VERIFIED" if manifest_is_v2 else "LEGACY_UNVERIFIED" if manifest_path.is_file() else "NOT_RUN",
                        "manifest_status": "AVAILABLE" if manifest_is_v2 else "NOT_AVAILABLE",
                        "artifact_hashes_declared": 0,
                        "artifacts_verified": 0,
                        "artifacts_failed": 0,
                        "artifacts_not_run": len(artifacts),
                    },
                    "source_expectations": (),
                    "artifact_details": tuple(
                        {"relative_path": name, "status": "unverified"} for name in artifacts
                    ),
                    "phase": str(manifest.get("phase", "") or ""),
                    "generation_id": str(manifest.get("generation_id", "") or ""),
                    "error": manifest_error or str(manifest.get("error", "") or ""),
                    "package_manifest_error": False,
                    "retryable": bool(manifest.get("retryable", False)),
                    "published_target": published_target,
                    "artifacts": artifacts,
                    "source": (
                        "measurement.json + authoritative JSONL generation"
                        if manifest_is_v2
                        else "session_manifest.json legacy unverified + published generation pointer"
                        if manifest_path.is_file()
                        else "session directory without manifest"
                    ),
                    "time_axis": str(manifest.get("time_axis", "recording_time_ns") or "recording_time_ns"),
                    "legacy": manifest_path.is_file() and not manifest_is_v2,
                }
            )
        return tuple(entries)

    def session_artifact_rows(
        self,
        session_id: str,
        artifact_name: str,
        *,
        measurement_id: str | None = None,
        limit: int = 100,
    ) -> dict[str, object]:
        """Read stored Parquet/CSV/JSONL rows without frame reinterpretation."""

        recordings_root = (self._app_root() / "recordings").resolve()
        resolved_session = _resolve_recording_session_directory(
            recordings_root,
            session_id,
        )
        if resolved_session is None:
            return {"columns": (), "rows": (), "error": "unsafe or missing session ID"}
        output_dir = resolved_session
        if measurement_id:
            component = Path(str(measurement_id))
            if (
                component.name != str(measurement_id)
                or str(measurement_id) in {".", ".."}
                or "\\" in str(measurement_id)
            ):
                return {"columns": (), "rows": (), "error": "unsafe measurement ID"}
            output_dir = (output_dir / "measurements" / component).resolve()
            try:
                output_dir.relative_to(recordings_root)
            except ValueError:
                return {"columns": (), "rows": (), "error": "measurement path outside recordings root"}
        package_entry = _package_session_projection(
            output_dir,
            owning_session_id=str(session_id) if measurement_id else None,
            measurement_id=str(measurement_id or "") or None,
        )
        logical_aliases = {
            "": "raw/stereo_measurements",
            "measurement_pairs": "raw/stereo_measurements",
            "authoritative_measurements": "raw/stereo_measurements",
        }
        try:
            from Workstation.workstation_core.recording_reader import ROLE_SOURCES

            logical_aliases.update({role: role for role in ROLE_SOURCES})
        except ImportError:
            pass
        logical_role = logical_aliases.get(str(artifact_name or ""))
        if measurement_id and logical_role is not None:
            if package_entry is not None and str(
                package_entry.get("integrity_status") or ""
            ) == "INTEGRITY_FAILED":
                return {
                    "columns": (),
                    "rows": (),
                    "error": "package integrity verification failed",
                    "logical_role": logical_role,
                }
            try:
                from Workstation.workstation_core.recording_reader import (
                    read_measurement_stream,
                )

                stream = read_measurement_stream(output_dir, logical_role)
                rows = stream.rows[: max(0, int(limit))]
                columns = tuple(
                    dict.fromkeys(key for row in rows for key in row)
                )
            except (FileNotFoundError, OSError, TypeError, ValueError) as exc:
                return {
                    "columns": (),
                    "rows": (),
                    "error": str(exc),
                    "logical_role": logical_role,
                }
            return {
                "columns": columns,
                "rows": rows,
                "error": "",
                "path": str(stream.path),
                "logical_role": logical_role,
                "authoritative": stream.authoritative,
                "legacy_fallback": stream.legacy_fallback,
                "time_axis": "producer clock domains preserved per row",
                "coordinate_contract": (
                    "producer RDF/NED/FRD/baseline_center_frd fields preserved; "
                    "no frame reinterpretation"
                ),
            }
        normalized_artifact = _safe_session_relative_path(output_dir, artifact_name)
        if normalized_artifact is None:
            return {
                "columns": (),
                "rows": (),
                "error": f"unsafe or missing session artifact: {artifact_name}",
            }
        declared_detail: Mapping[str, object] | None = None
        if package_entry is not None:
            declared_artifacts = {
                str(value) for value in package_entry.get("artifacts", ()) or ()
            }
            if normalized_artifact not in declared_artifacts:
                detail = (
                    "package manifest is unreadable"
                    if package_entry.get("package_manifest_error")
                    else "artifact is not declared by package manifest"
                )
                return {
                    "columns": (),
                    "rows": (),
                    "error": f"{detail}: {normalized_artifact}",
                }
            declared_detail = next(
                (
                    detail
                    for detail in package_entry.get("artifact_details", ()) or ()
                    if isinstance(detail, Mapping)
                    and str(detail.get("relative_path") or "") == normalized_artifact
                ),
                None,
            )
        candidate = resolve_session_artifact_path(
            output_dir,
            normalized_artifact,
            allow_published=package_entry is None,
        )
        if candidate is None or not candidate.is_file():
            return {
                "columns": (),
                "rows": (),
                "error": f"unsafe or missing session artifact: {normalized_artifact}",
            }
        if package_entry is not None:
            if declared_detail is None:
                return {
                    "columns": (),
                    "rows": (),
                    "error": f"package artifact descriptor is missing: {normalized_artifact}",
                }
            if str(declared_detail.get("status") or "").lower() != "present":
                return {
                    "columns": (),
                    "rows": (),
                    "error": f"package artifact is not present: {normalized_artifact}",
                }
            expected_size = declared_detail.get("size_bytes")
            if expected_size is not None and candidate.stat().st_size != int(expected_size):
                return {
                    "columns": (),
                    "rows": (),
                    "error": f"artifact size does not match package manifest: {normalized_artifact}",
                }
            expected_sha256 = str(declared_detail.get("sha256") or "").lower()
            if len(expected_sha256) != 64 or any(
                character not in "0123456789abcdef" for character in expected_sha256
            ):
                return {
                    "columns": (),
                    "rows": (),
                    "error": f"artifact SHA-256 is missing or invalid in package manifest: {normalized_artifact}",
                }
            try:
                actual_sha256 = sha256_file(candidate)
            except OSError as exc:
                return {"columns": (), "rows": (), "error": str(exc)}
            if actual_sha256 != expected_sha256:
                return {
                    "columns": (),
                    "rows": (),
                    "error": f"artifact SHA-256 does not match package manifest: {normalized_artifact}",
                }
        if candidate.suffix.lower() == ".jsonl":
            try:
                rows = []
                with candidate.open("r", encoding="utf-8") as handle:
                    for _, line in zip(range(max(0, int(limit))), handle):
                        payload = json.loads(line)
                        if isinstance(payload, dict):
                            rows.append(payload)
                columns = tuple(dict.fromkeys(key for row in rows for key in row))
            except (OSError, UnicodeError, ValueError) as exc:
                return {"columns": (), "rows": (), "error": str(exc), "path": str(candidate)}
            return {
                "columns": columns,
                "rows": tuple(rows),
                "error": "",
                "path": str(candidate),
                "time_axis": "recorded_unix_ns; camera response timestamps preserved",
                "coordinate_contract": "Capture HR Still evidence as stored; no image payload or conversion",
            }
        if candidate.suffix.lower() == ".parquet":
            from Workstation.workstation_core.session_parquet import read_lossless_rows

            artifact_keys = {
                "measurement_pairs.parquet": "measurement_pairs",
                "camera_timeline.parquet": "camera_timeline",
                "pixhawk_timeline.parquet": "pixhawk_timeline",
                "gnss_base_timeline.parquet": "gnss_base_timeline",
                "geometry_comparison.parquet": "geometry_comparison",
            }
            artifact_key = artifact_keys.get(candidate.name)
            if artifact_key is None:
                return {
                    "columns": (),
                    "rows": (),
                    "error": f"unknown Workstation Parquet artifact: {candidate.name}",
                    "path": str(candidate),
                }
            try:
                all_rows = read_lossless_rows(candidate, artifact_key)
                rows = all_rows[: max(0, int(limit))]
                columns = tuple(dict.fromkeys(key for row in rows for key in row))
            except (OSError, TypeError, ValueError) as exc:
                return {"columns": (), "rows": (), "error": str(exc), "path": str(candidate)}
            return {
                "columns": columns,
                "rows": tuple(rows),
                "error": "",
                "path": str(candidate),
                "time_axis": "recording_time_ns; exact row_json values preserved",
                "coordinate_contract": "typed Parquet projection plus lossless canonical row_json",
            }
        if candidate.suffix.lower() != ".csv":
            return {
                "columns": (),
                "rows": (),
                "error": "only Parquet/CSV/JSONL artifacts are previewed; binary/video/XLSX outputs remain unchanged",
                "path": str(candidate),
            }
        try:
            with candidate.open("r", encoding="utf-8-sig", newline="") as handle:
                reader = csv.DictReader(handle)
                columns = tuple(reader.fieldnames or ())
                rows = tuple(dict(row) for _, row in zip(range(max(0, int(limit))), reader))
        except (OSError, UnicodeError, csv.Error) as exc:
            return {"columns": (), "rows": (), "error": str(exc), "path": str(candidate)}
        return {
            "columns": columns,
            "rows": rows,
            "error": "",
            "path": str(candidate),
            "time_axis": "recording_time_ns; source columns preserved",
            "coordinate_contract": "stored CSV fields are displayed as saved; no NED/ENU/camera-frame conversion",
        }

    def recording(self, *, now_s: float | None = None) -> dict[str, object]:
        now = time.monotonic() if now_s is None else float(now_s)
        if not bool(getattr(self.panel, "_measurement_session_hierarchy_enabled", True)):
            storage_reader = getattr(self.panel, "data_storage_snapshot", None)
            run_reader = getattr(self.panel, "measurement_run_snapshot", None)
            try:
                storage = storage_reader() if callable(storage_reader) else {}
            except Exception as exc:
                storage = {"state": "UNAVAILABLE", "error": str(exc), "streams": {}}
            try:
                run = run_reader() if callable(run_reader) else {}
            except Exception:
                run = {}
            active = bool(
                getattr(self.panel, "distance_recording_enabled", False)
                or getattr(self.panel, "video_recording_enabled", False)
            )
            elapsed_method = getattr(self.panel, "_recording_elapsed_s", None)
            try:
                elapsed = _number(elapsed_method(now)) if callable(elapsed_method) else None
            except Exception:
                elapsed = None
            paths = getattr(self.panel, "video_recording_paths", {}) or {}
            return {
                "state": "ACTIVE" if active else str(storage.get("state") or "IDLE"),
                "active": active,
                "measurement_state": str(run.get("state") or "IDLE"),
                "recording_mode": str(
                    getattr(self.panel, "_recording_mode", "manual") or "manual"
                ),
                "output_dir": str(storage.get("root") or ""),
                "video_paths": {
                    str(side): str(path) for side, path in paths.items()
                }
                if isinstance(paths, dict)
                else {},
                "artifacts": (),
                "elapsed_s": elapsed,
                "status_label": _widget_text(
                    getattr(self.panel, "recording_status_label", None), ""
                ),
                "retry_required": False,
                "finalized": False,
                "storage": storage,
                "sample_counts": {
                    "pairs": _safe_len(getattr(self.panel, "distance_recording_rows", ())),
                    "camera_left": _safe_len(
                        (getattr(self.panel, "camera_recording_rows", {}) or {}).get("left", ())
                    ),
                    "camera_right": _safe_len(
                        (getattr(self.panel, "camera_recording_rows", {}) or {}).get("right", ())
                    ),
                    "px4_left": _safe_len(
                        (getattr(self.panel, "px4_recording_rows", {}) or {}).get("left", ())
                    ),
                    "px4_right": _safe_len(
                        (getattr(self.panel, "px4_recording_rows", {}) or {}).get("right", ())
                    ),
                    "gnss_base": _safe_len(
                        getattr(self.panel, "gnss_base_recording_rows", ())
                    ),
                },
                "source": "MeasurementDataStore + transient capture state",
                "clock": "per-record UTC/source clocks",
            }
        session_id = str(getattr(self.panel, "recording_session_id", "") or "")
        output_dir: Path | None = None
        if session_id:
            method = getattr(self.panel, "_recording_session_dir", None)
            if callable(method):
                try:
                    output_dir = Path(method(session_id))
                except (OSError, TypeError, ValueError):
                    output_dir = None
        paths = getattr(self.panel, "video_recording_paths", {}) or {}
        video_paths = {str(side): str(path) for side, path in paths.items()} if isinstance(paths, dict) else {}
        artifacts: list[str] = []
        if output_dir is not None and output_dir.exists():
            try:
                artifacts = sorted(path.name for path in output_dir.iterdir() if path.is_file())[:64]
            except OSError:
                artifacts = []
        active = bool(getattr(self.panel, "distance_recording_enabled", False) or getattr(self.panel, "video_recording_enabled", False))
        active_method = getattr(self.panel, "_session_recording_active", None)
        if callable(active_method):
            try:
                active = bool(active_method())
            except Exception:
                pass
        elapsed_method = getattr(self.panel, "_recording_elapsed_s", None)
        elapsed = None
        if callable(elapsed_method):
            try:
                elapsed = _number(elapsed_method(now))
            except Exception:
                elapsed = None
        status_label = _widget_text(getattr(self.panel, "recording_status_label", None), "")
        save_thread = getattr(self.panel, "_session_save_thread", None)
        retry = bool(getattr(self.panel, "_session_save_retry_required", False))
        finalized = bool(getattr(self.panel, "_session_save_finalized", False))
        if save_thread is not None:
            finalization = "FINALIZING"
        elif retry:
            finalization = "RETRYABLE"
        elif finalized:
            finalization = "FINALIZED"
        elif active:
            finalization = "ACTIVE"
        else:
            finalization = "IDLE"
        return {
            "state": finalization,
            "active": active,
            "session_id": session_id,
            "measurement_id": str(getattr(self.panel, "_measurement_id", "") or ""),
            "recording_mode": str(
                getattr(self.panel, "_recording_mode", "manual") or "manual"
            ),
            "output_dir": str(output_dir) if output_dir is not None else "",
            "video_paths": video_paths,
            "artifacts": tuple(artifacts),
            "elapsed_s": elapsed,
            "status_label": status_label,
            "retry_required": retry,
            "finalized": finalized,
            "sample_counts": {
                "pairs": _safe_len(getattr(self.panel, "distance_recording_rows", ())),
                "camera_left": _safe_len((getattr(self.panel, "camera_recording_rows", {}) or {}).get("left", ())),
                "camera_right": _safe_len((getattr(self.panel, "camera_recording_rows", {}) or {}).get("right", ())),
                "px4_left": _safe_len((getattr(self.panel, "px4_recording_rows", {}) or {}).get("left", ())),
                "px4_right": _safe_len((getattr(self.panel, "px4_recording_rows", {}) or {}).get("right", ())),
                "gnss_base": _safe_len(getattr(self.panel, "gnss_base_recording_rows", ())),
            },
            "recovered": getattr(self.panel, "_recovered_session_metadata", None) is not None,
            "source": "session spool rows + recording session/save worker",
            "clock": "session monotonic anchor; wall-clock ID retained",
        }

    @staticmethod
    def _camera_intrinsics(camera: object | None) -> dict[str, object]:
        if camera is None:
            return {"state": "NO DATA"}
        matrix = getattr(camera, "camera_matrix", None)
        return {
            "state": "AVAILABLE",
            "width_px": _integer(getattr(camera, "width", None)),
            "height_px": _integer(getattr(camera, "height", None)),
            "fx_px": _matrix_value(matrix, 0, 0),
            "fy_px": _matrix_value(matrix, 1, 1),
            "cx_px": _matrix_value(matrix, 0, 2),
            "cy_px": _matrix_value(matrix, 1, 2),
            "reprojection_error_px": _number(getattr(camera, "reprojection_error_px", None)),
            "image_count": _integer(getattr(camera, "image_count", None)),
            "distortion_coefficients": tuple(getattr(camera, "distortion_coefficients", ()) or ()),
            "source": "StereoIntrinsicsResult / CameraIntrinsics",
            "frame_contract": f"{CAMERA_FRAME}; camera matrix units are pixels",
        }

    def calibration(self) -> dict[str, object]:
        active = getattr(self.panel, "active_intrinsics", None)
        latest = getattr(self.panel, "last_calibration_result", None)
        result = active if active is not None else latest
        calculation_active = getattr(self.panel, "checkerboard_calibration_thread", None) is not None
        calibration_status = _widget_text(getattr(self.panel, "calibration_status_label", None), "")
        calculation_status = _widget_text(
            getattr(self.panel, "checkerboard_validation_status_label", None),
            "",
        )
        status_label = calculation_status if calculation_active and calculation_status else calibration_status
        active_profile_path = str(
            getattr(self.panel, "_active_intrinsics_profile_path", "") or ""
        )
        candidate_profile_path = str(
            getattr(self.panel, "_candidate_intrinsics_profile_path", "") or ""
        )
        profile_path = (
            active_profile_path if active is not None else candidate_profile_path
        )
        if not profile_path and result is not None:
            profile_path = (
                "memory/active-calibration"
                if active is not None
                else "memory/calibration-candidate"
            )
        zoom = (
            _integer(getattr(self.panel, "_active_intrinsics_source_zoom_level", None))
            if active is not None
            else _integer(getattr(self.panel, "last_calibration_zoom_level", None))
        )
        if zoom is None:
            zoom_widget = getattr(self.panel, "zoom_level_spin", None)
            zoom = _integer(_widget_value(zoom_widget))
        candidate_extrinsics_profile = getattr(
            self.panel, "selected_extrinsics_profile", None
        )
        if active is not None and hasattr(self.panel, "active_extrinsics_profile"):
            selected_extrinsics_profile = getattr(
                self.panel, "active_extrinsics_profile", None
            )
        else:
            selected_extrinsics_profile = candidate_extrinsics_profile
        extrinsics = None if result is None else getattr(result, "extrinsics", None)
        if extrinsics is None and selected_extrinsics_profile is not None:
            extrinsics = getattr(selected_extrinsics_profile, "extrinsics", None)
        extrinsics_payload: dict[str, object] = {}
        if extrinsics is not None:
            translation = tuple(getattr(extrinsics, "translation_left_to_right_m", ()) or ())
            rotation = tuple(
                tuple(row)
                for row in (getattr(extrinsics, "rotation_left_to_right", ()) or ())
            )
            right_origin = None
            if len(rotation) == 3 and all(len(row) == 3 for row in rotation) and len(translation) == 3:
                right_origin = tuple(
                    -sum(
                        float(rotation[column][row]) * float(translation[column])
                        for column in range(3)
                    )
                    for row in range(3)
                )
            extrinsics_payload = {
                "state": "AVAILABLE",
                "pair_count": _integer(getattr(extrinsics, "pair_count", None)),
                "reprojection_error_px": _number(getattr(extrinsics, "reprojection_error_px", None)),
                "rotation_left_to_right": rotation,
                "translation_left_to_right_m": translation,
                "right_camera_origin_in_left_rdf_m": right_origin,
                "frame_contract": str(getattr(extrinsics, "frame_contract", "") or f"{CAMERA_FRAME}; left→right R/T"),
                "baseline_m": sum(float(value) * float(value) for value in translation) ** 0.5 if translation else None,
            }
        active_intrinsics_zoom = _integer(
            getattr(self.panel, "_active_intrinsics_source_zoom_level", None)
        )
        if active_intrinsics_zoom is None:
            active_intrinsics_zoom = zoom
        active_extrinsics_zoom = _integer(
            getattr(self.panel, "_active_extrinsics_source_zoom_level", None)
        )
        selected_extrinsics_zoom = _integer(
            getattr(selected_extrinsics_profile, "source_zoom_level", None)
        )
        extrinsics_source_zoom = (
            active_extrinsics_zoom
            if active is not None and active_extrinsics_zoom is not None
            else selected_extrinsics_zoom
        )
        extrinsics_profile_path = str(
            getattr(self.panel, "_active_extrinsics_profile_path", "")
            if active is not None
            else getattr(self.panel, "_selected_extrinsics_profile_path", "")
            or ""
        )
        if not extrinsics_profile_path and selected_extrinsics_profile is not None:
            profile_id = str(getattr(selected_extrinsics_profile, "profile_id", "") or "stereo_extrinsics")
            extrinsics_profile_path = f"memory:{profile_id}"
        extrinsics_method = str(
            getattr(selected_extrinsics_profile, "method", "") or ""
        )
        extrinsics_profile_id = str(
            getattr(selected_extrinsics_profile, "profile_id", "") or ""
        )
        extrinsics_metadata = dict(
            getattr(selected_extrinsics_profile, "metadata", {}) or {}
        )
        cross_zoom = (
            active_intrinsics_zoom is not None
            and extrinsics_source_zoom is not None
            and active_intrinsics_zoom != extrinsics_source_zoom
        )
        selection_mode = independent_zoom_selection_mode(
            active_intrinsics_zoom,
            extrinsics_source_zoom,
        )
        selection_label = {
            "manual_cross_zoom": "cross-zoom/manual",
            "manual_same_zoom": "same-zoom/manual",
            "manual_independent_zoom": "independent/manual; source zoom unknown",
        }[selection_mode]
        combination = (
            f"K,D zoom {active_intrinsics_zoom} + R,t zoom {extrinsics_source_zoom} "
            f"({selection_label})"
            if extrinsics is not None
            else f"K,D zoom {active_intrinsics_zoom}; R,t not selected"
        )
        actual_zoom = getattr(self.panel, "actual_visible_zoom", {}) or {}
        actual_parts: list[str] = []
        for side in ("left", "right"):
            state = actual_zoom.get(side, {}) if isinstance(actual_zoom, dict) else {}
            index = state.get("index") if isinstance(state, dict) else None
            ratio = _number(state.get("ratio")) if isinstance(state, dict) else None
            if index is None:
                readback = "unknown"
            elif ratio is not None and math.isfinite(ratio) and ratio > 0.0:
                readback = f"{index} ({ratio:g}×)"
            else:
                readback = str(index)
            actual_parts.append(f"{side}={readback}")
        zoom_readback = ", ".join(actual_parts)
        capture_controller = getattr(self.panel, "_calibration_workflow_controller_owner", None)
        auto_capture_active = bool(getattr(capture_controller, "_auto_capture_active", False))
        auto_capture_saved_pairs = _integer(
            getattr(capture_controller, "_auto_capture_saved_pairs", None)
        )
        valid_pair_count = extrinsics_payload.get("pair_count")
        if auto_capture_saved_pairs is not None and (
            auto_capture_active or auto_capture_saved_pairs > 0
        ):
            valid_pair_count = auto_capture_saved_pairs
        selected_validation_mode = str(
            _combo_data(getattr(self.panel, "checkerboard_validation_mode_combo", None), "pixhawk")
            or "pixhawk"
        )
        result_validation_mode = str(
            getattr(self.panel, "checkerboard_validation_result_mode", "") or ""
        )
        validation_mode = (
            result_validation_mode
            if result is not None and result_validation_mode
            else selected_validation_mode
        )
        validation_run_results = getattr(self.panel, "checkerboard_validation_run_results", ()) or ()
        if not isinstance(validation_run_results, (tuple, list)):
            validation_run_results = ()
        raw_progress = getattr(self.panel, "calibration_progress", {}) or {}
        if isinstance(raw_progress, dict):
            progress = dict(raw_progress)
        else:
            progress = {}
        try:
            progress_percent = float(progress.get("percent", 0.0))
        except (TypeError, ValueError):
            progress_percent = 0.0
        progress_text = str(progress.get("message") or progress.get("phase") or "Idle")
        if progress:
            progress_text = f"{progress_text} ({max(0.0, min(100.0, progress_percent)):.0f}%)"
        calibration_quality = getattr(self.panel, "calibration_quality", None)
        if not isinstance(calibration_quality, Mapping) and result is not None:
            calibration_quality = getattr(result, "calibration_quality", None)
        calibration_quality = (
            dict(calibration_quality)
            if isinstance(calibration_quality, Mapping)
            else {
                "state": "UNVERIFIED",
                "reason_codes": ["quality_snapshot_unavailable"],
            }
        )
        capture_status: dict[str, object] = {}
        capture_status_error = ""
        for owner in (self.panel, capture_controller):
            reader = getattr(owner, "workstation_calibration_capture_snapshot", None)
            if not callable(reader):
                continue
            try:
                value = reader()
            except Exception as exc:
                capture_status_error = str(exc)
                break
            if isinstance(value, Mapping):
                capture_status = dict(value)
                break
        if not capture_status:
            capture_status = {
                "schema": "workstation-calibration-capture-status-v1",
                "capture_root": "",
                "capture_set_id": "",
                "detection": {},
                "run": {},
                "capture_set": {
                    "state": "UNAVAILABLE",
                    "error": capture_status_error
                    or "calibration capture snapshot unavailable",
                },
                "last": {"state": "UNKNOWN", "message": calibration_status},
            }
        return {
            "state": (
                "CALCULATING"
                if calculation_active
                else "ACTIVE"
                if active is not None
                else "AVAILABLE"
                if result is not None
                else "NO DATA"
            ),
            "active": active is not None,
            "calculation_active": calculation_active,
            "profile_path": profile_path,
            "active_profile_path": active_profile_path,
            "candidate_profile_path": candidate_profile_path,
            "candidate_available": latest is not None,
            "extrinsics_candidate_profile_id": str(
                getattr(candidate_extrinsics_profile, "profile_id", "") or ""
            ),
            "intrinsics_profile_hash": str(
                getattr(self.panel, "_active_intrinsics_profile_hash", "") or ""
            ),
            "zoom": zoom,
            "intrinsics_source_zoom": active_intrinsics_zoom,
            "extrinsics_profile_path": extrinsics_profile_path,
            "extrinsics_profile_id": extrinsics_profile_id,
            "extrinsics_method": extrinsics_method,
            "extrinsics_metadata": extrinsics_metadata,
            "extrinsics_profile_hash": str(
                getattr(self.panel, "_active_extrinsics_profile_hash", "")
                if active is not None
                else getattr(self.panel, "_selected_extrinsics_profile_hash", "")
                or ""
            ),
            "extrinsics_source_zoom": extrinsics_source_zoom,
            "combination": combination,
            "cross_zoom": cross_zoom,
            "selection_mode": str(
                getattr(self.panel, "_active_calibration_selection_mode", "")
                or ("manual_candidate" if extrinsics is not None else "intrinsics_only")
            ),
            "validation_gate": "disabled",
            "quality_gate": "calibration_quality_v1",
            "calibration_quality": calibration_quality,
            "capture_status": capture_status,
            "zoom_readback": zoom_readback,
            "status_label": status_label,
            "progress": progress_text,
            "valid_pair_count": valid_pair_count,
            "auto_capture_active": auto_capture_active,
            "auto_capture_saved_pairs": auto_capture_saved_pairs,
            "source": (
                "checkerboard calculation worker + live auto-capture controller + "
                "active_intrinsics/last_calibration_result"
            ),
            "left": self._camera_intrinsics(None if result is None else getattr(result, "left", None)),
            "right": self._camera_intrinsics(None if result is None else getattr(result, "right", None)),
            "extrinsics": extrinsics_payload,
            "validation_mode": validation_mode,
            "validation_summary": dict(getattr(self.panel, "checkerboard_validation_summary", {}) or {}),
            "validation_run_results": tuple(
                dict(row) for row in validation_run_results if isinstance(row, dict)
            ),
            "frame_contract": f"{CAMERA_FRAME}; calibration intrinsics/extrinsics stay in OpenCV optical RDF",
            "measurement_contract": f"public output {MEASUREMENT_FRAME} [{MEASUREMENT_AXIS_CONTRACT}]; baseline {BASELINE_TRANSLATION_FRAME}",
        }

    def summary(self, *, now_s: float | None = None) -> dict[str, object]:
        now = time.monotonic() if now_s is None else float(now_s)
        return {
            "attitude": {side: self.attitude(side, now_s=now) for side in ("left", "right")},
            "gps": {side: self.gps(side, now_s=now) for side in ("left", "right")},
            "baseline": self.baseline(),
            "detection_config": self.detection_config(),
            "detection": {side: self.detection(side, now_s=now) for side in ("left", "right")},
            "recording": self.recording(now_s=now),
            "calibration": self.calibration(),
        }
