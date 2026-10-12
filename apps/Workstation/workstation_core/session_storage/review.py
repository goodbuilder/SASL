"""Session review responsibility extracted without changing saved data contracts.

Position/attitude frames, axis aliases, units, timestamps, schema and hashes
retain the producer/recording contract. This module does not convert camera
origins, optical RDF, baseline-center FRD or world NED implicitly.
"""
from __future__ import annotations

import json
from collections.abc import Mapping, Sequence
from pathlib import Path
from Workstation.workstation_core.measurement_methods import MEASUREMENT_METHOD_IDS, PIXHAWK_RTK_LIVE
from . import manifest as _storage_manifest


_LEGACY_REVIEW_METHOD_SCHEMAS = frozenset(
    {
        "",
        "stereowiris-paired-frame-v1",
        "stereowiris-paired-frame-v2",
    }
)


_LEGACY_REVIEW_METHOD_ALIASES = {
    "pixhawk": "pixhawk_rtk_live",
    "checkerboard": "checkerboard_stereo",
    "calibration": "checkerboard_stereo",
}


def _review_row_is_explicitly_blocked(row: Mapping[str, object]) -> bool:
    pair_status = str(row.get("pair_status") or "").strip().lower()
    geometry_kind = str(row.get("measurement_geometry_kind") or "").strip().lower()
    return bool(
        row.get("blocked_pair_sha256")
        or geometry_kind == "unresolved"
        or "blocked" in pair_status
        or "no_ready" in pair_status
    )


def _review_measurement_method_id(row: Mapping[str, object]) -> str:
    """Resolve current and legacy method identity without truthiness fallback."""

    if "measurement_method_id" in row:
        return str(row.get("measurement_method_id") or "").strip()
    if _review_row_is_explicitly_blocked(row):
        return ""
    schema = str(row.get("schema_version") or "").strip()
    if schema not in _LEGACY_REVIEW_METHOD_SCHEMAS:
        return ""
    legacy = str(row.get("distance_mode") or "").strip().lower()
    legacy = _LEGACY_REVIEW_METHOD_ALIASES.get(legacy, legacy)
    return legacy if legacy in MEASUREMENT_METHOD_IDS else ""


def _review_result_row(row: Mapping[str, object]) -> dict[str, object]:
    return {
        "pair_id": row.get("pair_id") or row.get("measurement_attempt_id"),
        "utc_time": row.get("estimated_utc_time_ns") or row.get("recorded_unix_ns"),
        "recording_time_ns": row.get("recording_time_ns") or row.get("pair_app_mid_monotonic_ns"),
        "object_class": row.get("class_name"),
        "distance_m": row.get("distance_m"),
        "forward_m": row.get("baseline_center_forward_m"),
        "right_m": row.get("baseline_center_right_m"),
        "down_m": row.get("baseline_center_down_m"),
        **{name: row.get(name) for name in (
            "target_latitude_deg", "target_longitude_deg", "target_altitude_ellipsoid_m", "target_altitude_msl_m",
        )},
        "measurement_method_id": _review_measurement_method_id(row),
        "measurement_method_instance_id": row.get("measurement_method_instance_id"),
        "measurement_geometry_kind": row.get("measurement_geometry_kind"),
        "measurement_geometry_sha256": row.get("measurement_geometry_sha256"),
        "measurement_selection_reason": row.get("measurement_selection_reason"),
        "measurement_component_sources": row.get("measurement_component_sources"),
        "valid": row.get("measurement_valid"),
        "warning_code": row.get("warning_code") or row.get("warnings"),
    }


def _review_quality_row(row: Mapping[str, object]) -> dict[str, object]:
    return {
        "pair_id": row.get("pair_id") or row.get("measurement_attempt_id"),
        "sync_delta_ns": row.get("sync_delta_ns") or row.get("pair_delta_ns"),
        "left_confidence": row.get("left_confidence") or row.get("left_bbox_confidence"),
        "right_confidence": row.get("right_confidence") or row.get("right_bbox_confidence"),
        "vertical_disparity_px": row.get("vertical_disparity_px") or row.get("vertical_delta_px"),
        "reprojection_error_px": row.get("checkerboard_extrinsics_reprojection_error_px"),
        "gnss_fix_type": row.get("rtk_fix_quality") or row.get("left_gps_fix_type"),
        "geometry_state": row.get("measurement_geometry_kind"),
        "status": row.get("pair_status") or row.get("quality"),
    }


def _review_warning_rows(rows: Sequence[Mapping[str, object]]) -> list[dict[str, object]]:
    warnings: list[dict[str, object]] = []
    for row in rows:
        value = str(row.get("warnings") or row.get("warning_code") or "").strip()
        if not value:
            continue
        warnings.append(
            {
                "utc_time": row.get("estimated_utc_time_ns") or row.get("recorded_unix_ns"),
                "pair_id": row.get("pair_id") or row.get("measurement_attempt_id"),
                "subsystem": "measurement",
                "severity": "WARNING",
                "code": row.get("warning_code") or "MEASUREMENT_WARNING",
                "description": value,
                "impact": row.get("pair_status"),
            }
        )
    return warnings


def _review_capture_rows(metadata: Mapping[str, object]) -> list[dict[str, object]]:
    rows: list[dict[str, object]] = []
    for key, capture_type in (("pnp_hr_photo_capture", "HR"), ("oz_photo_capture", "OZ")):
        section = metadata.get(key)
        values = section.get("captures", ()) if isinstance(section, Mapping) else ()
        for capture in values if isinstance(values, (list, tuple)) else ():
            if not isinstance(capture, Mapping):
                continue
            cameras = capture.get("cameras")
            if isinstance(cameras, Mapping):
                for side, camera in cameras.items():
                    camera_row = camera if isinstance(camera, Mapping) else {}
                    rows.append(
                        {
                            "capture_id": capture.get("capture_id"),
                            "capture_type": capture_type,
                            "purpose": capture.get("purpose"),
                            "side": side,
                            "requested_at": camera_row.get("capture_requested_unix_ns") or capture.get("request_utc_ns"),
                            "completed_at": camera_row.get("completed_unix_ns"),
                            "relative_path": camera_row.get("ftp_local_path"),
                            "sha256": camera_row.get("sha256"),
                            "status": camera_row.get("state") or capture.get("state"),
                        }
                    )
            else:
                rows.append(
                    {
                        "capture_id": capture.get("capture_id"),
                        "capture_type": capture_type,
                        "purpose": capture.get("purpose"),
                        "status": capture.get("state"),
                    }
                )
    return rows


def _measurement_review_mapping(
    measurement_root: Path,
    *,
    session_root: Path,
    row_factory=list,
) -> dict[str, object] | None:
    from Workstation.workstation_core.session_parquet import (
        verify_parquet_artifact,
    )
    from Workstation.workstation_core.recording_reader import (
        iter_measurement_stream,
    )

    data = measurement_root / "data"
    pairs = data / "measurement_pairs.parquet"
    context_path = data / "context.json"
    manifest_path = measurement_root / "measurement.json"
    if not pairs.exists() and not context_path.exists():
        return None
    if not pairs.is_file() or not context_path.is_file() or not manifest_path.is_file():
        raise OSError(
            f"incomplete finalized measurement data view: {measurement_root}"
        )
    try:
        manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
        if not isinstance(manifest, Mapping) or manifest.get("status") != "finalized":
            raise ValueError("measurement manifest is not finalized")
        declared_details = manifest.get("artifact_details")
        if not isinstance(declared_details, Mapping):
            raise ValueError("measurement manifest has no artifact details")
        rows = row_factory()
        rows.extend(iter_measurement_stream(measurement_root, "raw/stereo_measurements"))
        method_results: dict[str, list[dict[str, object]]] = {
            method_id: [] for method_id in MEASUREMENT_METHOD_IDS
        }
        from Workstation.workstation_core.method_evaluation import (
            MethodEvaluationOutcome,
        )

        for method_id in MEASUREMENT_METHOD_IDS:
            artifact_key = f"method_result_{method_id}"
            if not isinstance(declared_details.get(artifact_key), Mapping):
                continue
            method_rows = row_factory()
            method_rows.extend(iter_measurement_stream(measurement_root, f"raw/method_results/{method_id}"))
            for method_row in method_rows:
                outcome = MethodEvaluationOutcome.from_mapping(method_row)
                if outcome.method_id != method_id:
                    raise ValueError("method result stream contains the wrong method")
            method_results[method_id] = method_rows
        context_payload = json.loads(context_path.read_text(encoding="utf-8"))
    except (OSError, TypeError, ValueError, json.JSONDecodeError) as exc:
        raise OSError(
            f"cannot rebuild review from measurement {measurement_root.name}: {exc}"
        ) from exc
    metadata = context_payload.get("metadata", {}) if isinstance(context_payload, Mapping) else {}
    metadata = metadata if isinstance(metadata, Mapping) else {}
    measurement_id = str(metadata.get("measurement_id") or measurement_root.name)
    valid_count = sum(
        str(row.get("measurement_valid", "")).lower() in {"true", "1", "yes"}
        or row.get("measurement_valid") is True
        for row in rows
    )
    artifacts: list[dict[str, object]] = []
    nested_measurement = measurement_root.parent.name == "measurements"
    artifact_keys = {
        "measurement_pairs.parquet": "measurement_pairs",
        "camera_timeline.parquet": "camera_timeline",
        "pixhawk_timeline.parquet": "pixhawk_timeline",
        "gnss_base_timeline.parquet": "gnss_base_timeline",
        "geometry_comparison.parquet": "geometry_comparison",
    }
    for path in sorted(data.glob("*.parquet")):
        artifact_key = artifact_keys.get(path.name)
        if artifact_key is None:
            continue
        verified = verify_parquet_artifact(path, artifact_key)
        declared = declared_details.get(artifact_key)
        if not isinstance(declared, Mapping):
            raise OSError(f"manifest does not declare {artifact_key}")
        expected_values = {
            "size_bytes": verified.size_bytes,
            "row_count": verified.row_count,
            "sha256": verified.sha256,
            "schema_sha256": verified.schema_sha256,
        }
        mismatched = [
            key
            for key, actual in expected_values.items()
            if declared.get(key) != actual
        ]
        if mismatched:
            raise OSError(
                f"{path.name} disagrees with measurement manifest: {', '.join(mismatched)}"
            )
        relative = path.relative_to(measurement_root).as_posix()
        artifact_row: dict[str, object] = {
            "role": path.stem,
            "format": "parquet",
            "size_bytes": verified.size_bytes,
            "sha256": verified.sha256,
            "required": path.name == "measurement_pairs.parquet",
            "status": "VERIFIED",
        }
        artifact_row[
            "measurement_relative_path" if nested_measurement else "relative_path"
        ] = relative
        artifacts.append(artifact_row)
    context_relative = context_path.relative_to(measurement_root).as_posix()
    declared_context = declared_details.get("context")
    context_sha256 = _storage_manifest._sha256_path(context_path)
    if (
        not isinstance(declared_context, Mapping)
        or declared_context.get("size_bytes") != context_path.stat().st_size
        or declared_context.get("sha256") != context_sha256
    ):
        raise OSError("context.json disagrees with measurement manifest")
    context_artifact: dict[str, object] = {
        "role": "measurement_context",
        "format": "json",
        "size_bytes": context_path.stat().st_size,
        "sha256": context_sha256,
        "required": True,
        "status": "VERIFIED",
    }
    context_artifact[
        "measurement_relative_path" if nested_measurement else "relative_path"
    ] = context_relative
    artifacts.append(context_artifact)
    latest_path = measurement_root / "latest_revision.json"
    if latest_path.is_file():
        try:
            latest = json.loads(latest_path.read_text(encoding="utf-8"))
            relative_manifest = str(latest.get("manifest_path") or "")
            package_manifest_path = (measurement_root / relative_manifest).resolve(
                strict=True
            )
            package_manifest_path.relative_to(measurement_root.resolve())
            package_manifest = json.loads(
                package_manifest_path.read_text(encoding="utf-8")
            )
            package_artifacts = package_manifest.get("artifacts", ())
        except (OSError, TypeError, ValueError, json.JSONDecodeError) as exc:
            raise OSError(
                f"cannot read sealed artifact inventory for {measurement_root.name}: {exc}"
            ) from exc
        existing_keys = {
            (str(row.get("role") or ""), str(row.get("measurement_relative_path") or row.get("relative_path") or ""))
            for row in artifacts
        }
        for descriptor in package_artifacts if isinstance(package_artifacts, list) else ():
            if not isinstance(descriptor, Mapping) or descriptor.get("status") != "present":
                continue
            relative = str(descriptor.get("relative_path") or "")
            if not relative:
                continue
            role = str(descriptor.get("role") or "artifact")
            display_relative = relative
            data_marker = "/data/"
            if data_marker in relative and relative.endswith(
                (".parquet", "context.json")
            ):
                display_relative = "data/" + relative.rsplit(data_marker, 1)[1]
            key = (role, display_relative)
            if key in existing_keys:
                continue
            file_row: dict[str, object] = {
                "role": role,
                "format": Path(display_relative).suffix.lstrip(".") or "file",
                "size_bytes": descriptor.get("size_bytes"),
                "sha256": descriptor.get("sha256"),
                "required": descriptor.get("required"),
                "status": "VERIFIED",
            }
            file_row[
                "measurement_relative_path" if nested_measurement else "relative_path"
            ] = display_relative
            artifacts.append(file_row)
            existing_keys.add(key)
    calculation = metadata.get("calculation")
    calculation = calculation if isinstance(calculation, Mapping) else {}
    intrinsics = metadata.get("intrinsics")
    intrinsics = intrinsics if isinstance(intrinsics, Mapping) else {}
    geometry = [
        {
            "method_id": calculation.get("measurement_method_id"),
            "profile_id": calculation.get("measurement_method_instance_id"),
            "geometry_sha256": metadata.get("active_geometry_sha256")
            or intrinsics.get("content_sha256"),
            "zoom": intrinsics.get("active_zoom_level"),
            "baseline_m": calculation.get("baseline_m"),
            "rotation_source": calculation.get("mount_assumption_model"),
            "translation_source": calculation.get("baseline_source"),
            "status": "READY" if valid_count else "NO_VALID_RESULT",
        }
    ]
    devices = [
        {
            "device_id": name,
            "device_type": kind,
            "sample_count": metadata.get(count_key),
            "status": "RECORDED" if int(metadata.get(count_key) or 0) else "NO_DATA",
        }
        for name, kind, count_key in (
            ("camera-left", "camera", "camera_left_gui_frame_count"),
            ("camera-right", "camera", "camera_right_gui_frame_count"),
            ("pixhawk-left", "pixhawk", "px4_left_sample_count"),
            ("pixhawk-right", "pixhawk", "px4_right_sample_count"),
            ("pixhawk-drone", "pixhawk", "px4_drone_sample_count"),
            ("gnss-base", "gnss", "gnss_base_sample_count"),
        )
    ]
    result_rows = row_factory()
    quality_rows = row_factory()
    warning_rows = row_factory()
    for row in rows:
        result_rows.append(_review_result_row(row))
        quality_rows.append(_review_quality_row(row))
        warning_rows.extend(_review_warning_rows((row,)))
    return {
        "measurement_id": measurement_id,
        "summary": {
            "status": "VALID" if valid_count else "NO_VALID_RESULT",
            "method": PIXHAWK_RTK_LIVE,
            "valid_pair_count": valid_count,
            "blocked_pair_count": len(rows) - valid_count,
            "warning_count": len(warning_rows),
        },
        "results": result_rows,
        "quality": quality_rows,
        "geometry": geometry,
        "devices": devices,
        "captures": _review_capture_rows(metadata),
        "warnings": warning_rows,
        "artifacts": artifacts,
        "method_results": method_results,
    }


def _rebuild_session_review(session_root: Path, session_id: str) -> Path:
    from .disk_index import DiskRowTable, disk_index

    with disk_index(session_root) as connection:
        ordinal = 0

        def row_factory():
            nonlocal ordinal
            ordinal += 1
            return DiskRowTable(connection, f"measurement_review_{ordinal}")

        return _rebuild_indexed_session_review(session_root, session_id, row_factory)


def _rebuild_indexed_session_review(session_root, session_id, row_factory):
    from Workstation.workstation_core.session_review import ReplayableReviewRows, write_session_review_xlsx

    measurements: list[dict[str, object]] = []
    session_summary: dict[str, object] = {}
    state_path = session_root / "session.json"
    if state_path.is_file():
        try:
            value = json.loads(state_path.read_text(encoding="utf-8"))
            if isinstance(value, Mapping):
                session_summary = dict(value)
        except (OSError, ValueError):
            session_summary = {}
    required_measurements = {
        str(row.get("measurement_id") or "")
        for row in session_summary.get("measurements", ()) or ()
        if isinstance(row, Mapping)
        and str(row.get("state") or "") in {"SEALED_COMPLETE", "SEALED_INCOMPLETE"}
        and row.get("measurement_id")
    }
    measurement_parent = session_root / "measurements"
    if measurement_parent.is_dir():
        roots = sorted(
            child for child in measurement_parent.iterdir() if child.is_dir()
        )
    else:
        roots = [session_root]
    for root in roots:
        review = _measurement_review_mapping(root, session_root=session_root, row_factory=row_factory)
        if review is not None:
            for name in ("results", "quality", "warnings"):
                review[name] = ReplayableReviewRows(review[name])
            review["method_results"] = {method_id: ReplayableReviewRows(rows) for method_id, rows in review["method_results"].items()}
            measurements.append(review)
    included = {str(row.get("measurement_id") or "") for row in measurements}
    missing = sorted(required_measurements - included)
    if missing:
        raise OSError(
            "finalized measurements are missing from review inputs: "
            + ", ".join(missing)
        )
    return write_session_review_xlsx(
        session_root,
        session_id=session_id,
        measurements=measurements,
        session_summary=session_summary,
    )


def rebuild_session_review(session_root: Path, session_id: str) -> Path:
    """Regenerate the derived session workbook and persist its status."""

    session_root = Path(session_root)
    status_path = session_root / ".workstation" / "review.json"
    try:
        output = _rebuild_session_review(session_root, session_id)
    except Exception as exc:
        _storage_manifest._write_manifest(
            status_path,
            {
                "status": "failed",
                "path": "review.xlsx",
                "error": str(exc),
                "regenerable": True,
            },
        )
        _storage_manifest._fsync_directory_path(status_path.parent)
        raise
    _storage_manifest._write_manifest(
        status_path,
        {
            "status": "generated",
            "regenerable": True,
            "artifact": {
                "role": "derived/review",
                "media_type": (
                    "application/vnd.openxmlformats-officedocument.spreadsheetml.sheet"
                ),
                "relative_path": output.relative_to(session_root).as_posix(),
                "size_bytes": output.stat().st_size,
                "sha256": _storage_manifest._sha256_path(output),
                "row_count": None,
                "schema_version": "workstation-session-review-xlsx-v1",
                "authoritative": False,
                "derived_from": [],
                "verified": True,
            },
        },
    )
    _storage_manifest._fsync_directory_path(status_path.parent)
    return output
