"""Deterministic static geometry contexts for terminal measurement records.

The context registry is an index/deduplication layer.  Canonical
``MeasurementAttempt`` evidence remains self-contained and continues to carry
its live attitude, GPS, and baseline snapshot.  Those dynamic values are
deliberately excluded from the context identity.
"""

from __future__ import annotations

from collections.abc import Mapping, Sequence
from dataclasses import asdict
import hashlib
import json
from typing import Any

from Workstation.workstation_core.measurement_attempt import MeasurementAttempt


GEOMETRY_CONTEXT_SCHEMA_VERSION = "workstation-geometry-context-v1"


def _canonical_bytes(value: Mapping[str, Any]) -> bytes:
    return json.dumps(
        value,
        ensure_ascii=False,
        allow_nan=False,
        sort_keys=True,
        separators=(",", ":"),
    ).encode("utf-8")


def _context_record(payload: Mapping[str, Any]) -> dict[str, object]:
    normalized = json.loads(_canonical_bytes(payload).decode("utf-8"))
    digest = hashlib.sha256(_canonical_bytes(normalized)).hexdigest()
    return {
        "geometry_context_id": f"gc:{digest}",
        "geometry_context_sha256": digest,
        "context": normalized,
    }


def _camera_calibration(value: object) -> dict[str, object] | None:
    if value is None:
        return None
    return {
        "width_px": int(value.width_px),
        "height_px": int(value.height_px),
        "K": [list(row) for row in value.camera_matrix],
        "D": list(value.distortion_coefficients),
        "reprojection_error_px": float(value.reprojection_error_px),
        "image_count": int(value.image_count),
        "pixel_contract": "u-right/v-down; px",
        "camera_frame": str(value.camera_frame),
    }


def geometry_context_record_for_attempt(
    attempt: MeasurementAttempt,
) -> dict[str, object]:
    """Return one static context record without live pose/GPS/baseline values."""

    intrinsics = attempt.intrinsics
    component_sources = {
        str(name): str(source)
        for name, source in attempt.measurement_component_sources
    }
    uses_live_gnss = "live_gnss" in component_sources.values()
    uses_live_pixhawk = "live_pixhawk" in component_sources.values()
    checkerboard = (
        None if intrinsics is None else intrinsics.checkerboard_extrinsics
    )
    applied_checkerboard = (
        checkerboard
        if attempt.measurement_method_id == "checkerboard_stereo"
        else None
    )
    static_translation = None
    if (
        not uses_live_gnss
        and attempt.baseline is not None
        and attempt.baseline.translation_right_in_left_rdf_m is not None
    ):
        static_translation = list(
            attempt.baseline.translation_right_in_left_rdf_m
        )

    payload: dict[str, object] = {
        "schema_version": GEOMETRY_CONTEXT_SCHEMA_VERSION,
        "context_kind": "measurement_attempt_static",
        "calibration": {
            "left": None if intrinsics is None else _camera_calibration(intrinsics.left),
            "right": None if intrinsics is None else _camera_calibration(intrinsics.right),
            "source_resolution_px": (
                None
                if intrinsics is None
                else {
                    "left": [intrinsics.left.width_px, intrinsics.left.height_px],
                    "right": [intrinsics.right.width_px, intrinsics.right.height_px],
                }
            ),
            "profile": (
                None
                if intrinsics is None
                else {
                    "combined_path": intrinsics.selected_profile_path,
                    "combined_sha256": intrinsics.content_sha256,
                    "intrinsics_path": intrinsics.selected_profile_path,
                    "intrinsics_sha256": intrinsics.intrinsics_content_sha256,
                    "extrinsics_path": intrinsics.selected_extrinsics_profile_path,
                    "extrinsics_sha256": intrinsics.extrinsics_content_sha256,
                    "path_usage_contract": "historical_provenance_only",
                }
            ),
            "zoom": (
                None
                if intrinsics is None
                else {
                    "intrinsics_source_zoom": intrinsics.intrinsics_source_zoom_level,
                    "extrinsics_source_zoom": intrinsics.extrinsics_source_zoom_level,
                    "operating_zoom": intrinsics.active_zoom_level,
                    "observed_zoom": None,
                    "observed_zoom_status": "not_available_in_measurement_attempt",
                    "selection_mode": intrinsics.selection_mode,
                }
            ),
            "available_relative_transform": (
                None
                if checkerboard is None
                else {
                    "rotation_left_to_right": [
                        list(row) for row in checkerboard.rotation_left_to_right
                    ],
                    "translation_left_to_right_m": list(
                        checkerboard.translation_left_to_right_m
                    ),
                    "frame_contract": checkerboard.frame_contract,
                    "rotation_alias_contract": (
                        "R_right_left: v_right = R_right_left @ v_left"
                    ),
                    "point_contract": (
                        "p_right = R_right_left @ p_left + T_right_left_m"
                    ),
                }
            ),
            "applied_relative_transform": (
                None
                if applied_checkerboard is None
                else {
                    "rotation_left_to_right": [
                        list(row)
                        for row in applied_checkerboard.rotation_left_to_right
                    ],
                    "translation_left_to_right_m": list(
                        applied_checkerboard.translation_left_to_right_m
                    ),
                    "frame_contract": applied_checkerboard.frame_contract,
                }
            ),
        },
        "measurement_method": {
            "method_id": attempt.measurement_method_id,
            "method_instance_id": attempt.measurement_method_instance_id,
            "geometry_kind": attempt.measurement_geometry_kind,
            "component_sources": component_sources,
            "static_measurement_geometry_sha256": (
                (attempt.measurement_geometry_sha256 or None)
                if not (uses_live_gnss or uses_live_pixhawk)
                else None
            ),
            "applied_transform_status": (
                "embedded_checkerboard_r_t"
                if applied_checkerboard is not None
                else "dynamic_components_retained_in_measurement_attempt"
                if uses_live_gnss or uses_live_pixhawk
                else "referenced_by_static_measurement_geometry_sha256"
                if attempt.measurement_geometry_sha256
                else "not_available"
            ),
        },
        "geometry_config": asdict(attempt.geometry_config),
        "baseline_model": {
            "source_contract": (
                None if attempt.baseline is None else attempt.baseline.source
            ),
            "uses_live_gnss": uses_live_gnss,
            "uses_live_pixhawk": uses_live_pixhawk,
            "static_translation_right_in_left_rdf_m": static_translation,
            "dynamic_baseline_values_in_context": False,
        },
        "transform_contract": {
            "rotation_notation": "R_B_A means v_B = R_B_A @ v_A",
            "point": "rotation_plus_translation",
            "ray_direction": "rotation_only",
            "world_frame": attempt.world_frame,
            "body_frame": attempt.body_frame,
            "camera_frame": attempt.camera_frame,
            "camera_mount_profile": attempt.camera_mount_profile,
        },
        "public_output_contract": {
            "frame": attempt.measurement_frame,
            "origin": "midpoint of left/right optical centers",
            "axis_contract": attempt.measurement_axis_contract,
            "vector_contract": attempt.public_vector_contract,
            "unit": attempt.measurement_unit,
        },
        "dynamic_values_retained_only_in_measurement_attempt": [
            "left_attitude",
            "right_attitude",
            "left_gps",
            "right_gps",
            "live_dynamic_baseline_snapshot",
            "measurement_geometry_sha256_when_live_components_are_used",
        ],
    }
    return _context_record(payload)


def _terminal_fallback_context(row: Mapping[str, object]) -> dict[str, object]:
    blocked = bool(row.get("blocked_pair_sha256"))
    static_field_names = (
        "measurement_method_id",
        "measurement_method_instance_id",
        "measurement_geometry_kind",
        "measurement_component_sources",
        "intrinsics_profile_sha256",
        "extrinsics_profile_sha256",
        "intrinsics_source_zoom_level",
        "extrinsics_source_zoom_level",
        "intrinsics_zoom_level",
        "left_intr_width",
        "left_intr_height",
        "left_intr_fx",
        "left_intr_fy",
        "left_intr_cx",
        "left_intr_cy",
        "left_intr_distortion",
        "right_intr_width",
        "right_intr_height",
        "right_intr_fx",
        "right_intr_fy",
        "right_intr_cx",
        "right_intr_cy",
        "right_intr_distortion",
        "checkerboard_extrinsics_frame_contract",
        "measurement_frame",
        "measurement_axis_contract",
        "public_vector_contract",
    )
    return _context_record(
        {
            "schema_version": GEOMETRY_CONTEXT_SCHEMA_VERSION,
            "context_kind": "blocked_unresolved" if blocked else "legacy_terminal_static",
            "available_static_fields": {
                name: row.get(name)
                for name in static_field_names
                if row.get(name) not in (None, "")
            },
            "dynamic_values_retained_only_in_terminal_row": True,
            "public_output_contract": {
                "frame": row.get("measurement_frame") or "baseline_center_frd",
                "axis_contract": row.get("measurement_axis_contract")
                or "X=Front,Y=Right,Z=Down",
                "unit": "m",
            },
        }
    )


def _strict_json_mapping(value: object, *, field: str) -> dict[str, object]:
    if not isinstance(value, str) or not value:
        raise ValueError(f"{field} is missing")

    def reject_constant(constant: str) -> None:
        raise ValueError(f"non-finite JSON constant is forbidden: {constant}")

    decoded = json.loads(value, parse_constant=reject_constant)
    if not isinstance(decoded, dict):
        raise ValueError(f"{field} must contain a JSON object")
    return decoded


def geometry_context_record_for_terminal_row(
    row: Mapping[str, object],
) -> dict[str, object]:
    attempt_json = row.get("measurement_attempt_canonical_json")
    if attempt_json not in (None, ""):
        attempt_payload = _strict_json_mapping(
            attempt_json,
            field="measurement_attempt_canonical_json",
        )
        attempt = MeasurementAttempt.from_canonical_payload(attempt_payload)
        expected_sha256 = str(row.get("measurement_attempt_sha256") or "")
        if expected_sha256 and attempt.canonical_sha256() != expected_sha256:
            raise ValueError("measurement attempt hash changed before context indexing")
        return geometry_context_record_for_attempt(attempt)
    return _terminal_fallback_context(row)


def materialize_geometry_contexts(
    terminal_rows: Sequence[dict[str, object]],
) -> list[dict[str, object]]:
    """Attach resolvable context IDs and return first-seen unique contexts."""

    registry: dict[str, dict[str, object]] = {}
    for row in terminal_rows:
        record = geometry_context_record_for_terminal_row(row)
        context_id = str(record["geometry_context_id"])
        declared = str(row.get("geometry_context_id") or "")
        if declared and declared != context_id:
            raise ValueError("geometry_context_id disagrees with static context")
        row["geometry_context_id"] = context_id
        previous = registry.get(context_id)
        if previous is not None and previous != record:
            raise ValueError("geometry_context_id collision")
        registry.setdefault(context_id, record)
    unresolved = {
        str(row.get("geometry_context_id") or "") for row in terminal_rows
    } - set(registry)
    if unresolved:
        raise ValueError("terminal geometry context reference is unresolved")
    return list(registry.values())


__all__ = [
    "GEOMETRY_CONTEXT_SCHEMA_VERSION",
    "geometry_context_record_for_attempt",
    "geometry_context_record_for_terminal_row",
    "materialize_geometry_contexts",
]
