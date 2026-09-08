"""Create the single operator-facing workbook for a recording session.

The workbook is deliberately a derived view.  Finalized canonical JSONL and
native binary evidence remain authoritative; Parquet is an intermediate
derived analysis view.  This module only combines the small,
operator-relevant rows supplied by each measurement.  It does not read
recording state and has no dependency on a controller or on
``recording_session``.
"""

from __future__ import annotations

from collections.abc import Mapping, Sequence
from dataclasses import dataclass, field
from datetime import date, datetime, timezone
import json
import math
import os
from pathlib import Path, PurePosixPath, PureWindowsPath
import re
import tempfile
from typing import Any
import zipfile

from Workstation.workstation_core.strict_jsonl import fsync_directory as _fsync_directory
from xml.etree import ElementTree

import xlsxwriter

from Workstation.workstation_core.measurement_methods import (
    CHECKERBOARD_STEREO,
    CLOUD_GNSS,
    COMBINED_HYBRID,
    DRONE_RTK_MULTIVIEW,
    DRONE_RTK_ROTATION_HYBRID,
    HORIZON_HYBRID,
    MEASUREMENT_METHOD_IDS,
    MEASUREMENT_METHOD_LABELS,
    MOUNTAIN_GNSS,
    PIXHAWK_RTK_LIVE,
    PIXHAWK_RP_GPS_BASELINE_YAW,
    TERRAIN_PNP,
)


REVIEW_WORKBOOK_NAME = "review.xlsx"
EXCEL_MAX_ROWS = 1_048_576
EXCEL_MAX_COLUMNS = 16_384
EXCEL_MAX_TEXT_LENGTH = 32_767

BASE_SHEET_NAMES = (
    "ReadMe",
    "Session",
    "Measurements",
    "Results",
    "Quality",
    "Geometry",
    "Devices",
    "Captures",
    "Warnings",
    "Files",
)

# Excel sheet names are capped at 31 characters.  Keep these stable and
# compact while preserving a one-to-one mapping to the canonical method IDs.
METHOD_SHEET_NAMES: dict[str, str] = {
    PIXHAWK_RTK_LIVE: "M_Pixhawk_RTK",
    PIXHAWK_RP_GPS_BASELINE_YAW: "M_Pixhawk_RP_Yaw",
    CHECKERBOARD_STEREO: "M_Checkerboard",
    HORIZON_HYBRID: "M_Horizon",
    MOUNTAIN_GNSS: "M_Mountain_GNSS",
    DRONE_RTK_ROTATION_HYBRID: "M_Drone_RTK_Rotation",
    DRONE_RTK_MULTIVIEW: "M_Drone_RTK_Multiview",
    TERRAIN_PNP: "M_Terrain_PnP",
    COMBINED_HYBRID: "M_Combined",
    CLOUD_GNSS: "M_Cloud_GNSS",
}
if tuple(METHOD_SHEET_NAMES) != MEASUREMENT_METHOD_IDS:
    raise RuntimeError("method worksheet mapping must cover every canonical method in order")
if len(set(METHOD_SHEET_NAMES.values())) != len(METHOD_SHEET_NAMES) or any(
    len(name) > 31 for name in METHOD_SHEET_NAMES.values()
):
    raise RuntimeError("method worksheet names must be unique and Excel-compatible")

METHOD_SHEET_COLUMNS = (
    "record_type",
    "measurement_id",
    "pair_id",
    "method_id",
    "method_label",
    "profile_id",
    "geometry_sha256",
    "utc_time",
    "recording_time_ns",
    "object_class",
    "distance_m",
    "forward_m",
    "right_m",
    "down_m",
    "valid",
    "status",
    "warning_code",
    "sync_delta_ns",
    "left_confidence",
    "right_confidence",
    "vertical_disparity_px",
    "reprojection_error_px",
    "gnss_fix_type",
    "geometry_state",
    "zoom",
    "resolution",
    "baseline_m",
    "rotation_source",
    "translation_source",
    "measurement_method_instance_id",
    "measurement_geometry_kind",
    "measurement_selection_reason",
    "measurement_component_sources",
    "reasons",
    "warnings",
    "reused_inputs",
    "queue_wait_ms",
    "evaluation_duration_ms",
    "snapshot_sha256",
    "attempt_sha256",
    "method_evaluation_sha256",
    "method_evaluation_bundle_sha256",
)

_TABLE_NAMES = (
    "results",
    "quality",
    "geometry",
    "devices",
    "captures",
    "warnings",
)

_DEFAULT_COLUMNS: dict[str, tuple[str, ...]] = {
    "Measurements": (
        "measurement_id",
        "status",
        "method",
        "valid_pair_count",
        "blocked_pair_count",
        "representative_distance_m",
        "warning_count",
    ),
    "Results": (
        "measurement_id",
        "pair_id",
        "utc_time",
        "recording_time_ns",
        "object_class",
        "distance_m",
        "forward_m",
        "right_m",
        "down_m",
        "measurement_method_id",
        "valid",
        "warning_code",
    ),
    "Quality": (
        "measurement_id",
        "pair_id",
        "sync_delta_ns",
        "left_confidence",
        "right_confidence",
        "vertical_disparity_px",
        "reprojection_error_px",
        "gnss_fix_type",
        "geometry_state",
        "status",
    ),
    "Geometry": (
        "measurement_id",
        "method_id",
        "profile_id",
        "geometry_sha256",
        "zoom",
        "resolution",
        "baseline_m",
        "rotation_source",
        "translation_source",
        "status",
    ),
    "Devices": (
        "measurement_id",
        "device_id",
        "device_type",
        "source_id",
        "sample_count",
        "connected",
        "quality",
        "warning_count",
        "status",
    ),
    "Captures": (
        "measurement_id",
        "capture_id",
        "capture_type",
        "purpose",
        "side",
        "requested_at",
        "completed_at",
        "relative_path",
        "sha256",
        "status",
    ),
    "Warnings": (
        "measurement_id",
        "utc_time",
        "pair_id",
        "subsystem",
        "severity",
        "code",
        "description",
        "impact",
        "relative_path",
    ),
    "Files": (
        "measurement_id",
        "role",
        "relative_path",
        "format",
        "size_bytes",
        "sha256",
        "required",
        "status",
    ),
}

_PATH_COLUMNS = frozenset(
    {
        "relative_path",
        "file_path",
        "evidence_path",
        "solver_input_path",
    }
)
_HASH_COLUMN_RE = re.compile(r"(?:^|_)(?:sha(?:256)?|hash)(?:$|_)", re.IGNORECASE)


@dataclass(frozen=True)
class MeasurementReview:
    """Finalized, operator-facing rows contributed by one measurement.

    All artifact ``relative_path`` values are relative to the session root.
    Complex authoritative payloads should stay in the source data and be
    represented here only by a compact summary, relative path, and hash.
    """

    measurement_id: str
    summary: Mapping[str, object] = field(default_factory=dict)
    results: Sequence[Mapping[str, object]] = ()
    quality: Sequence[Mapping[str, object]] = ()
    geometry: Sequence[Mapping[str, object]] = ()
    devices: Sequence[Mapping[str, object]] = ()
    captures: Sequence[Mapping[str, object]] = ()
    warnings: Sequence[Mapping[str, object]] = ()
    artifacts: Sequence[Mapping[str, object]] = ()
    method_results: Mapping[str, Sequence[Mapping[str, object]]] = field(
        default_factory=dict
    )

    @classmethod
    def from_mapping(cls, value: Mapping[str, object]) -> "MeasurementReview":
        measurement_id = str(value.get("measurement_id") or "").strip()
        tables_value = value.get("tables", {})
        if tables_value is None:
            tables_value = {}
        if not isinstance(tables_value, Mapping):
            raise TypeError("measurement 'tables' must be a mapping")

        def table(name: str) -> Sequence[Mapping[str, object]]:
            rows = value.get(name, tables_value.get(name, ()))
            return _require_rows(rows, label=f"{measurement_id or '<missing>'}.{name}")

        summary = value.get("summary", {})
        if summary is None:
            summary = {}
        if not isinstance(summary, Mapping):
            raise TypeError("measurement 'summary' must be a mapping")
        artifacts = value.get("artifacts", ())
        raw_method_results = value.get("method_results", {})
        if raw_method_results is None:
            raw_method_results = {}
        if not isinstance(raw_method_results, Mapping):
            raise TypeError("measurement 'method_results' must be a mapping")
        return cls(
            measurement_id=measurement_id,
            summary=dict(summary),
            results=table("results"),
            quality=table("quality"),
            geometry=table("geometry"),
            devices=table("devices"),
            captures=table("captures"),
            warnings=table("warnings"),
            artifacts=_require_rows(
                artifacts, label=f"{measurement_id or '<missing>'}.artifacts"
            ),
            method_results={
                str(method_id): _require_rows(
                    rows,
                    label=f"{measurement_id or '<missing>'}.method_results.{method_id}",
                )
                for method_id, rows in raw_method_results.items()
            },
        )


def write_session_review_xlsx(
    session_root: Path | str,
    *,
    session_id: str,
    measurements: Sequence[MeasurementReview | Mapping[str, object]],
    session_summary: Mapping[str, object] | None = None,
    generated_at_utc: str | None = None,
    max_rows_per_sheet: int = EXCEL_MAX_ROWS,
) -> Path:
    """Atomically create ``<session_root>/review.xlsx`` and return its path.

    ``measurements`` may contain :class:`MeasurementReview` instances or plain
    mappings with ``measurement_id``, ``summary``, ``tables`` and ``artifacts``.
    A table can also be supplied directly under its lower-case name.  The six
    accepted table names are ``results``, ``quality``, ``geometry``, ``devices``,
    ``captures`` and ``warnings``.

    The row limit argument exists primarily for deterministic boundary tests;
    production callers should leave it at Excel's native limit.  The header
    consumes one row, and oversized tables become ``Results_002`` and so on.
    """

    resolved_session_id = str(session_id).strip()
    if not resolved_session_id:
        raise ValueError("session_id must not be empty")
    if not 2 <= int(max_rows_per_sheet) <= EXCEL_MAX_ROWS:
        raise ValueError(
            f"max_rows_per_sheet must be between 2 and {EXCEL_MAX_ROWS}"
        )
    if session_summary is None:
        session_summary = {}
    if not isinstance(session_summary, Mapping):
        raise TypeError("session_summary must be a mapping")
    if isinstance(measurements, (str, bytes, bytearray)) or not isinstance(
        measurements, Sequence
    ):
        raise TypeError("measurements must be a sequence")

    normalized = tuple(_normalize_measurement(item) for item in measurements)
    ids = [item.measurement_id for item in normalized]
    if any(not item for item in ids):
        raise ValueError("every measurement must have a non-empty measurement_id")
    if len(set(ids)) != len(ids):
        raise ValueError("measurement_id values must be unique within a session")

    generated = generated_at_utc or datetime.now(timezone.utc).isoformat()
    root = Path(session_root)
    root.mkdir(parents=True, exist_ok=True)
    output = root / REVIEW_WORKBOOK_NAME

    fd, temporary_name = tempfile.mkstemp(
        dir=root, prefix=f".{REVIEW_WORKBOOK_NAME}.", suffix=".tmp.xlsx"
    )
    os.close(fd)
    temporary = Path(temporary_name)
    try:
        _write_workbook(
            temporary,
            session_root=root,
            session_id=resolved_session_id,
            session_summary=dict(session_summary),
            measurements=normalized,
            generated_at_utc=generated,
            max_rows_per_sheet=int(max_rows_per_sheet),
        )
        _validate_workbook(temporary)
        with temporary.open("rb") as handle:
            os.fsync(handle.fileno())
        os.replace(temporary, output)
        _fsync_directory(root)
        return output
    finally:
        temporary.unlink(missing_ok=True)


def _normalize_measurement(
    value: MeasurementReview | Mapping[str, object],
) -> MeasurementReview:
    if isinstance(value, MeasurementReview):
        return MeasurementReview(
            measurement_id=str(value.measurement_id).strip(),
            summary=_require_mapping(value.summary, label="summary"),
            results=_require_rows(value.results, label="results"),
            quality=_require_rows(value.quality, label="quality"),
            geometry=_require_rows(value.geometry, label="geometry"),
            devices=_require_rows(value.devices, label="devices"),
            captures=_require_rows(value.captures, label="captures"),
            warnings=_require_rows(value.warnings, label="warnings"),
            artifacts=_require_rows(value.artifacts, label="artifacts"),
            method_results={
                str(method_id): _require_rows(
                    rows, label=f"method_results.{method_id}"
                )
                for method_id, rows in value.method_results.items()
            },
        )
    if not isinstance(value, Mapping):
        raise TypeError("each measurement must be MeasurementReview or a mapping")
    return MeasurementReview.from_mapping(value)


def _require_mapping(value: object, *, label: str) -> dict[str, object]:
    if not isinstance(value, Mapping):
        raise TypeError(f"{label} must be a mapping")
    return {str(key): item for key, item in value.items()}


def _require_rows(value: object, *, label: str) -> tuple[dict[str, object], ...]:
    if value is None:
        return ()
    if isinstance(value, (str, bytes, bytearray)) or not isinstance(value, Sequence):
        raise TypeError(f"{label} must be a sequence of mappings")
    rows: list[dict[str, object]] = []
    for index, row in enumerate(value):
        if not isinstance(row, Mapping):
            raise TypeError(f"{label}[{index}] must be a mapping")
        rows.append({str(key): item for key, item in row.items()})
    return tuple(rows)


def _write_workbook(
    output: Path,
    *,
    session_root: Path,
    session_id: str,
    session_summary: Mapping[str, object],
    measurements: Sequence[MeasurementReview],
    generated_at_utc: str,
    max_rows_per_sheet: int,
) -> None:
    rows_by_sheet = _collect_rows(measurements)
    rows_by_method = _collect_method_rows(measurements)
    with xlsxwriter.Workbook(
        output,
        {
            "constant_memory": True,
            "strings_to_formulas": False,
            "strings_to_urls": False,
            "nan_inf_to_errors": False,
        },
    ) as workbook:
        workbook.use_zip64()
        formats = _workbook_formats(workbook)
        _write_readme_sheet(
            workbook,
            formats=formats,
            session_id=session_id,
            generated_at_utc=generated_at_utc,
        )
        session_rows = [
            {"field": "session_id", "value": session_id},
            {"field": "generated_at_utc", "value": generated_at_utc},
            {"field": "measurement_count", "value": len(measurements)},
        ]
        session_rows.extend(
            {"field": key, "value": value}
            for key, value in _flatten_mapping(session_summary)
        )
        _write_table_shards(
            workbook,
            base_name="Session",
            rows=session_rows,
            default_columns=("field", "value"),
            formats=formats,
            session_root=session_root,
            output_parent=output.parent,
            max_rows_per_sheet=max_rows_per_sheet,
        )
        for sheet_name in BASE_SHEET_NAMES[2:]:
            _write_table_shards(
                workbook,
                base_name=sheet_name,
                rows=rows_by_sheet[sheet_name],
                default_columns=_DEFAULT_COLUMNS[sheet_name],
                formats=formats,
                session_root=session_root,
                output_parent=output.parent,
                max_rows_per_sheet=max_rows_per_sheet,
            )
        for method_id in MEASUREMENT_METHOD_IDS:
            _write_table_shards(
                workbook,
                base_name=METHOD_SHEET_NAMES[method_id],
                rows=rows_by_method[method_id],
                default_columns=METHOD_SHEET_COLUMNS,
                formats=formats,
                session_root=session_root,
                output_parent=output.parent,
                max_rows_per_sheet=max_rows_per_sheet,
            )


def _collect_rows(
    measurements: Sequence[MeasurementReview],
) -> dict[str, list[dict[str, object]]]:
    result = {name: [] for name in BASE_SHEET_NAMES[2:]}
    for measurement in measurements:
        summary = dict(measurement.summary)
        _set_measurement_id(summary, measurement.measurement_id, label="summary")
        result["Measurements"].append(summary)
        for table_name in _TABLE_NAMES:
            sheet_name = table_name.title()
            for row in getattr(measurement, table_name):
                resolved = dict(row)
                _set_measurement_id(
                    resolved, measurement.measurement_id, label=table_name
                )
                result[sheet_name].append(resolved)
        for artifact in measurement.artifacts:
            resolved = dict(artifact)
            _set_measurement_id(
                resolved, measurement.measurement_id, label="artifacts"
            )
            measurement_path = resolved.pop("measurement_relative_path", None)
            if "relative_path" not in resolved and measurement_path is not None:
                resolved["relative_path"] = str(
                    PurePosixPath("measurements")
                    / measurement.measurement_id
                    / str(measurement_path).replace("\\", "/")
                )
            result["Files"].append(resolved)
    return result


def _collect_method_rows(
    measurements: Sequence[MeasurementReview],
) -> dict[str, list[dict[str, object]]]:
    """Build one derived review table for each canonical Measurement method.

    A method tab contains only geometry summaries and terminal pair results
    that identify that method.  Empty tabs remain header-only so absence of
    use is explicit without fabricating readiness or candidate evidence.
    """

    result = {method_id: [] for method_id in MEASUREMENT_METHOD_IDS}
    for measurement in measurements:
        quality_by_pair = {
            str(row.get("pair_id") or ""): dict(row)
            for row in measurement.quality
            if row.get("pair_id") not in (None, "")
        }
        for method_id, outcome_rows in measurement.method_results.items():
            if method_id not in result:
                continue
            for outcome in outcome_rows:
                row = {
                    "record_type": "method_outcome",
                    "pair_id": outcome.get("pair_id"),
                    "method_id": method_id,
                    "method_label": MEASUREMENT_METHOD_LABELS[method_id],
                    "profile_id": outcome.get("method_instance_id"),
                    "measurement_method_instance_id": outcome.get(
                        "method_instance_id"
                    ),
                    "geometry_sha256": outcome.get("geometry_sha256"),
                    "status": outcome.get("status"),
                    "valid": str(outcome.get("status") or "")
                    in {"VALID", "ESTIMATED"},
                    "reasons": "; ".join(
                        str(value) for value in outcome.get("reasons", ()) or ()
                    ),
                    "warnings": "; ".join(
                        str(value) for value in outcome.get("warnings", ()) or ()
                    ),
                    "reused_inputs": "; ".join(
                        str(value)
                        for value in outcome.get("reused_inputs", ()) or ()
                    ),
                    "queue_wait_ms": outcome.get("queue_wait_ms"),
                    "evaluation_duration_ms": outcome.get(
                        "evaluation_duration_ms"
                    ),
                    "snapshot_sha256": outcome.get("snapshot_sha256"),
                    "attempt_sha256": outcome.get("attempt_sha256"),
                    "method_evaluation_sha256": outcome.get(
                        "method_evaluation_sha256"
                    ),
                    "method_evaluation_bundle_sha256": outcome.get(
                        "method_evaluation_bundle_sha256"
                    ),
                }
                row["warning_code"] = row["warnings"]
                triangulated = outcome.get("result")
                triangulated = (
                    triangulated.get("triangulated")
                    if isinstance(triangulated, Mapping)
                    else None
                )
                if isinstance(triangulated, Mapping):
                    point = triangulated.get("point_m")
                    if isinstance(point, (list, tuple)) and len(point) == 3:
                        row.update(
                            forward_m=point[0],
                            right_m=point[1],
                            down_m=point[2],
                        )
                    row["distance_m"] = triangulated.get("range_m")
                _set_measurement_id(
                    row,
                    measurement.measurement_id,
                    label="method outcome",
                )
                result[method_id].append(row)
        for geometry in measurement.geometry:
            method_id = str(geometry.get("method_id") or "")
            if method_id not in result:
                continue
            row = {
                **dict(geometry),
                "record_type": "geometry",
                "method_id": method_id,
                "method_label": MEASUREMENT_METHOD_LABELS[method_id],
            }
            _set_measurement_id(row, measurement.measurement_id, label="method geometry")
            result[method_id].append(row)
        for measurement_result in measurement.results:
            # Terminal blocked pairs deliberately carry no method: geometry
            # was never selected. Do not inherit the Measurement summary's
            # preferred method and misclassify them in a method worksheet.
            pair_status = str(measurement_result.get("pair_status") or "").lower()
            geometry_kind = str(
                measurement_result.get("measurement_geometry_kind") or ""
            ).lower()
            if (
                geometry_kind == "unresolved"
                or "blocked" in pair_status
                or "no_ready" in pair_status
            ):
                continue
            method_id = str(measurement_result.get("measurement_method_id") or "")
            if method_id not in result:
                continue
            pair_id = str(measurement_result.get("pair_id") or "")
            row = {
                **dict(quality_by_pair.get(pair_id, {})),
                **dict(measurement_result),
                "record_type": "result",
                "method_id": method_id,
                "method_label": MEASUREMENT_METHOD_LABELS[method_id],
            }
            _set_measurement_id(row, measurement.measurement_id, label="method result")
            result[method_id].append(row)
    return result


def _set_measurement_id(
    row: dict[str, object], measurement_id: str, *, label: str
) -> None:
    supplied = row.get("measurement_id")
    if supplied is not None and str(supplied) != measurement_id:
        raise ValueError(
            f"{label} row measurement_id {supplied!r} does not match {measurement_id!r}"
        )
    row["measurement_id"] = measurement_id


def _flatten_mapping(
    value: Mapping[str, object], prefix: str = ""
) -> list[tuple[str, object]]:
    rows: list[tuple[str, object]] = []
    for key, item in value.items():
        name = f"{prefix}.{key}" if prefix else str(key)
        if isinstance(item, Mapping):
            rows.extend(_flatten_mapping(item, name))
        else:
            rows.append((name, item))
    return rows


def _workbook_formats(workbook: xlsxwriter.Workbook) -> dict[str, Any]:
    return {
        "header": workbook.add_format(
            {
                "bold": True,
                "font_color": "#FFFFFF",
                "bg_color": "#333333",
                "border": 1,
                "align": "center",
                "valign": "vcenter",
            }
        ),
        "text": workbook.add_format({"valign": "top"}),
        "wrap": workbook.add_format({"text_wrap": True, "valign": "top"}),
        "link": workbook.add_format(
            {"font_color": "#666666", "underline": True, "valign": "top"}
        ),
        "integer": workbook.add_format({"num_format": "0"}),
        "decimal": workbook.add_format({"num_format": "0.###############"}),
        "title": workbook.add_format(
            {"bold": True, "font_size": 16, "font_color": "#990000"}
        ),
        "note": workbook.add_format(
            {"font_color": "#595959", "text_wrap": True, "valign": "top"}
        ),
    }


def _write_readme_sheet(
    workbook: xlsxwriter.Workbook,
    *,
    formats: Mapping[str, Any],
    session_id: str,
    generated_at_utc: str,
) -> None:
    worksheet = workbook.add_worksheet("ReadMe")
    worksheet.hide_gridlines(2)
    worksheet.set_column(0, 0, 22)
    worksheet.set_column(1, 1, 92)
    worksheet.write_string(0, 0, "Session review", formats["title"])
    worksheet.write_string(1, 0, "session_id", formats["header"])
    _write_cell(worksheet, 1, 1, session_id, "session_id", formats)
    worksheet.write_string(2, 0, "generated_at_utc", formats["header"])
    _write_cell(worksheet, 2, 1, generated_at_utc, "generated_at_utc", formats)
    worksheet.write_string(4, 0, "Sheet", formats["header"])
    worksheet.write_string(4, 1, "Purpose", formats["header"])
    descriptions = (
        ("Session", "Session identity, lifecycle, and common settings."),
        ("Measurements", "One summary row per measurement."),
        ("Results", "Operator-facing measurement results, including blocked rows."),
        ("Quality", "Synchronization, detection, geometry, and GNSS quality."),
        ("Geometry", "Calibration and alignment identities and source summaries."),
        ("Devices", "Compact camera, Pixhawk, GNSS, and telemetry health."),
        ("Captures", "HR, OZ, alignment, and checkpoint capture inventory."),
        ("Warnings", "Warnings and errors with their measurement impact."),
        ("Files", "Relative links and hashes for evidence and authoritative data."),
        *(
            (
                sheet_name,
                f"{MEASUREMENT_METHOD_LABELS[method_id]} geometry summaries and "
                "terminal pair results. Header-only means the method was not used.",
            )
            for method_id, sheet_name in METHOD_SHEET_NAMES.items()
        ),
    )
    for row_index, (name, description) in enumerate(descriptions, start=5):
        worksheet.write_string(row_index, 0, name, formats["text"])
        worksheet.write_string(row_index, 1, description, formats["wrap"])
    note_row = 5 + len(descriptions) + 1
    worksheet.write_string(note_row, 0, "Source of truth", formats["header"])
    worksheet.write_string(
        note_row,
        1,
        "This XLSX is a derived operator report. Finalized measurement data and "
        "evidence files referenced by the Files sheet remain authoritative.",
        formats["note"],
    )


def _write_table_shards(
    workbook: xlsxwriter.Workbook,
    *,
    base_name: str,
    rows: Sequence[Mapping[str, object]],
    default_columns: Sequence[str],
    formats: Mapping[str, Any],
    session_root: Path,
    output_parent: Path,
    max_rows_per_sheet: int,
) -> None:
    columns = _ordered_columns(rows, default_columns)
    if len(columns) > EXCEL_MAX_COLUMNS:
        raise ValueError(
            f"{base_name} has {len(columns)} columns; Excel supports {EXCEL_MAX_COLUMNS}"
        )
    data_rows_per_sheet = max_rows_per_sheet - 1
    shard_count = max(1, math.ceil(len(rows) / data_rows_per_sheet))
    for shard_index in range(shard_count):
        start = shard_index * data_rows_per_sheet
        stop = start + data_rows_per_sheet
        shard_rows = rows[start:stop]
        sheet_name = base_name if shard_index == 0 else f"{base_name}_{shard_index + 1:03d}"
        worksheet = workbook.add_worksheet(sheet_name)
        worksheet.freeze_panes(1, 0)
        worksheet.hide_gridlines(2)
        for column_index, column in enumerate(columns):
            worksheet.write_string(0, column_index, column, formats["header"])
        for row_index, row in enumerate(shard_rows, start=1):
            for column_index, column in enumerate(columns):
                _write_cell(
                    worksheet,
                    row_index,
                    column_index,
                    row.get(column),
                    column,
                    formats,
                    hyperlink=_relative_hyperlink(
                        row.get(column),
                        column=column,
                        session_root=session_root,
                        output_parent=output_parent,
                    ),
                )
        if columns:
            last_row = max(0, len(shard_rows))
            worksheet.autofilter(0, 0, last_row, len(columns) - 1)
            for column_index, column in enumerate(columns):
                width = _column_width(column, shard_rows)
                worksheet.set_column(column_index, column_index, width)


def _ordered_columns(
    rows: Sequence[Mapping[str, object]], default_columns: Sequence[str]
) -> list[str]:
    columns = list(dict.fromkeys(str(column) for column in default_columns))
    known = set(columns)
    for row in rows:
        for key in row:
            name = str(key)
            if name not in known:
                known.add(name)
                columns.append(name)
    return columns


def _column_width(column: str, rows: Sequence[Mapping[str, object]]) -> int:
    maximum = len(column)
    for row in rows[:200]:
        value = row.get(column)
        if value is not None:
            maximum = max(maximum, min(80, len(_display_text(value))))
    if column.lower() in _PATH_COLUMNS or _HASH_COLUMN_RE.search(column):
        return min(72, max(24, maximum + 2))
    return min(40, max(12, maximum + 2))


def _write_cell(
    worksheet: Any,
    row: int,
    column_index: int,
    value: object,
    column_name: str,
    formats: Mapping[str, Any],
    *,
    hyperlink: str | None = None,
) -> None:
    if value is None:
        return
    if hyperlink is not None:
        display = _safe_excel_text(_display_text(value))
        result = worksheet.write_url(
            row,
            column_index,
            f"external:{hyperlink}",
            formats["link"],
            display,
        )
        if result == 0:
            return
    if _must_be_text(column_name, value):
        worksheet.write_string(
            row, column_index, _safe_excel_text(_display_text(value)), formats["text"]
        )
    elif isinstance(value, bool):
        worksheet.write_boolean(row, column_index, value)
    elif isinstance(value, int):
        if abs(value) > 9_007_199_254_740_991:
            worksheet.write_string(row, column_index, str(value), formats["text"])
        else:
            worksheet.write_number(row, column_index, value, formats["integer"])
    elif isinstance(value, float) and math.isfinite(value):
        worksheet.write_number(row, column_index, value, formats["decimal"])
    else:
        worksheet.write_string(
            row, column_index, _safe_excel_text(_display_text(value)), formats["wrap"]
        )


def _must_be_text(column_name: str, value: object) -> bool:
    name = column_name.lower()
    if name.endswith("_ns") or "timestamp_ns" in name:
        return True
    if _HASH_COLUMN_RE.search(name):
        return True
    return isinstance(value, (str, Path, datetime, date, bytes, bytearray))


def _display_text(value: object) -> str:
    if isinstance(value, (bytes, bytearray)):
        text = bytes(value).hex()
    elif isinstance(value, (datetime, date)):
        text = value.isoformat()
    elif isinstance(value, Path):
        text = value.as_posix()
    elif isinstance(value, str):
        text = value
    elif isinstance(value, float) and not math.isfinite(value):
        text = str(value)
    elif isinstance(value, (Mapping, list, tuple, set)):
        normalized = sorted(value, key=str) if isinstance(value, set) else value
        try:
            text = json.dumps(
                normalized,
                ensure_ascii=False,
                sort_keys=True,
                separators=(",", ":"),
                default=str,
            )
        except (TypeError, ValueError):
            text = str(value)
    else:
        text = str(value)
    if len(text) > EXCEL_MAX_TEXT_LENGTH:
        suffix = "…[truncated]"
        return text[: EXCEL_MAX_TEXT_LENGTH - len(suffix)] + suffix
    return text


def _safe_excel_text(value: str) -> str:
    stripped = value.lstrip(" \t\r\n")
    if stripped.startswith(("=", "+", "-", "@")):
        return "'" + value
    return value


def _relative_hyperlink(
    value: object,
    *,
    column: str,
    session_root: Path,
    output_parent: Path,
) -> str | None:
    if column.lower() not in _PATH_COLUMNS or value is None:
        return None
    raw = str(value).strip().replace("\\", "/")
    if not raw or "\x00" in raw or any(ord(character) < 32 for character in raw):
        return None
    posix = PurePosixPath(raw)
    windows = PureWindowsPath(raw)
    if (
        posix.is_absolute()
        or windows.is_absolute()
        or bool(windows.drive)
        or ":" in posix.parts[0]
        or ".." in posix.parts
    ):
        return None
    target = session_root.joinpath(*posix.parts)
    try:
        resolved_root = session_root.resolve(strict=True)
        resolved_target = target.resolve(strict=True)
        resolved_target.relative_to(resolved_root)
    except (FileNotFoundError, OSError, RuntimeError, ValueError):
        return None
    if not resolved_target.is_file():
        return None
    relative = os.path.relpath(target, start=output_parent).replace(os.sep, "/")
    if len(relative) > 2_000:
        return None
    return relative


def _validate_workbook(path: Path) -> None:
    if not zipfile.is_zipfile(path):
        raise ValueError("generated review workbook is not a valid XLSX archive")
    with zipfile.ZipFile(path) as archive:
        required_members = {"[Content_Types].xml", "xl/workbook.xml"}
        if not required_members.issubset(archive.namelist()):
            raise ValueError("generated review workbook is missing required XLSX members")
        root = ElementTree.fromstring(archive.read("xl/workbook.xml"))
        namespace = {"m": "http://schemas.openxmlformats.org/spreadsheetml/2006/main"}
        sheet_names = {
            element.attrib.get("name", "")
            for element in root.findall("m:sheets/m:sheet", namespace)
        }
        required_sheet_names = {
            *BASE_SHEET_NAMES,
            *METHOD_SHEET_NAMES.values(),
        }
        missing = required_sheet_names.difference(sheet_names)
        if missing:
            raise ValueError(
                "generated review workbook is missing sheets: " + ", ".join(sorted(missing))
            )


__all__ = [
    "BASE_SHEET_NAMES",
    "EXCEL_MAX_ROWS",
    "METHOD_SHEET_NAMES",
    "MeasurementReview",
    "REVIEW_WORKBOOK_NAME",
    "write_session_review_xlsx",
]
