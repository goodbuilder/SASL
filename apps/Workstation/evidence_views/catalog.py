"""Session review queries with an explicit data root and no live UI access.

The outer adapter retains synchronous callers; GUI composition may schedule the
same read-only query on its owned worker without changing the result contract.
"""
from __future__ import annotations
import csv
import json
from pathlib import Path
from typing import Mapping
from Workstation.workstation_core.strict_jsonl import sha256_file
from .artifacts import (
    resolve_session_artifact_path,
    _safe_session_relative_path,
    _recording_session_directories,
    _resolve_recording_session_directory,
    _package_manifest_reference,
    _mapping_state,
    _reason_values,
    _artifact_reason,
    _package_artifact_projection,
    _package_error_projection,
    _package_session_projection,
    _run_ledger_artifact_names,
    _existing_artifact_byte_total,
)


class SessionEvidenceQuery:
    def __init__(self, data_root: Path | str):
        self.data_root = Path(data_root)

    def session_catalog(self) -> tuple[dict[str, object], ...]:
        """List finalized/retryable session directories for review only."""

        recordings_root = self.data_root / "recordings"
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

        recordings_root = (self.data_root / "recordings").resolve()
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
