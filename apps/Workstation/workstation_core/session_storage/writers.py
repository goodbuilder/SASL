"""Session writers responsibility extracted without changing saved data contracts.

Position/attitude frames, axis aliases, units, timestamps, schema and hashes
retain the producer/recording contract. This module does not convert camera
origins, optical RDF, baseline-center FRD or world NED implicitly.
"""
from __future__ import annotations

import csv
import os
import tempfile
from collections.abc import Sequence
from dataclasses import field
from pathlib import Path
from typing import Iterable
import pyarrow as pa
import pyarrow.parquet as pq
import xlsxwriter
from . import manifest as _storage_manifest
from . import projection as _storage_projection
from . import schema as _storage_schema
from . import values as _storage_values
from .models import SessionSheetPreview


def _projected_columns(fieldnames: list[str], keywords: tuple[str, ...]) -> list[str]:
    selected = list(_storage_schema.PAIR_AXIS_COLUMNS)
    selected.extend(
        field
        for field in fieldnames
        if field not in _storage_schema.PAIR_AXIS_COLUMNS
        and any(keyword in field.lower() for keyword in keywords)
    )
    return selected


def _sanitize_xlsx_sheet_name(name: str, *, used: set[str]) -> str:
    """Fit an arbitrary method string into Excel's 31-char, symbol-free sheet name rule."""

    cleaned = "".join("_" if ch in _storage_schema._INVALID_XLSX_SHEET_CHARS else ch for ch in str(name)).strip()
    cleaned = (cleaned or "Sheet")[:31]
    candidate = cleaned
    suffix = 1
    while candidate in used:
        suffix_text = f"~{suffix}"
        candidate = cleaned[: max(1, 31 - len(suffix_text))] + suffix_text
        suffix += 1
    used.add(candidate)
    return candidate


def _extrinsics_comparison_sheet_groups(
    rows: Sequence[dict[str, object]],
) -> list[tuple[str, str, list[dict[str, object]]]]:
    """Group extrinsics-comparison rows by method into one sheet each.

    Method names are per-session dynamic (an ad-hoc PnP candidate id/method
    string, not a fixed enum), so sheets are discovered from the data itself
    rather than declared up front like the other fixed-category sheets.
    """

    grouped: dict[str, list[dict[str, object]]] = {}
    for row in rows:
        method = str(row.get("method") or "unknown")
        grouped.setdefault(method, []).append(row)
    used_names: set[str] = set()
    return [
        (_sanitize_xlsx_sheet_name(f"Ext_{method}", used=used_names), method, method_rows)
        for method, method_rows in grouped.items()
    ]


def session_sheet_previews(
    metadata: dict[str, object],
    rows: Sequence[dict[str, object]],
    *,
    camera_rows: dict[str, Sequence[dict[str, object]]] | None = None,
    px4_rows: dict[str, Sequence[dict[str, object]]] | None = None,
    gnss_base_rows: Sequence[dict[str, object]] | None = None,
    extrinsics_comparison_rows: Sequence[dict[str, object]] | None = None,
) -> list[SessionSheetPreview]:
    """Describe the sheets ``write_session_workbook`` would produce.

    The workbook writer consumes this same description, so an on-screen
    preview cannot drift from the saved file's sheet names or columns.
    """

    metadata_entries = _flatten_metadata(metadata)
    camera_rows = camera_rows or {}
    px4_rows = px4_rows or {}
    previews = [
        SessionSheetPreview(name="Summary", kind="key_value", entries=tuple(metadata_entries))
    ]
    previews.append(
        SessionSheetPreview(
            name="DistanceMeasurements",
            kind="table",
            columns=tuple(_storage_schema.DISTANCE_MEASUREMENT_COLUMNS),
        )
    )
    for sheet_name in ("CameraLeft", "CameraRight"):
        previews.append(
            SessionSheetPreview(
                name=sheet_name,
                kind="table",
                columns=tuple(_storage_schema.CAMERA_TIMELINE_COMPACT_COLUMNS),
            )
        )
    for sheet_name in ("Px4Left", "Px4Right"):
        previews.append(
            SessionSheetPreview(
                name=sheet_name,
                kind="table",
                columns=tuple(_storage_schema.PX4_TIMELINE_COMPACT_COLUMNS),
            )
        )
    previews.append(
        SessionSheetPreview(
            name="GnssBase",
            kind="table",
            columns=tuple(_storage_schema.GNSS_BASE_COMPACT_COLUMNS),
        )
    )
    previews.extend(
        SessionSheetPreview(
            name=f"{name}Static",
            kind="key_value",
            entries=tuple(
                entry
                for entry in metadata_entries
                if any(keyword in entry[0].lower() for keyword in _storage_schema.CALIBRATION_KEYWORDS)
            ),
        ) for name in ("Calibration",)
    )
    for sheet_name, _method, _method_rows in _extrinsics_comparison_sheet_groups(
        extrinsics_comparison_rows or ()
    ):
        previews.append(
            SessionSheetPreview(
                name=sheet_name,
                kind="table",
                columns=tuple(_storage_schema.EXTRINSICS_COMPARISON_COMPACT_COLUMNS),
            )
        )
    return previews


def _flatten_metadata(
    metadata: dict[str, object], prefix: str = ""
) -> list[tuple[str, object]]:
    flattened: list[tuple[str, object]] = []
    for key, value in metadata.items():
        qualified_key = f"{prefix}.{key}" if prefix else key
        if isinstance(value, dict):
            flattened.extend(_flatten_metadata(value, qualified_key))
        else:
            flattened.append((qualified_key, _storage_values._serializable_value(value)))
    return flattened


def _write_table(
    worksheet: xlsxwriter.worksheet.Worksheet,
    columns: list[str],
    rows: Iterable[dict[str, object]],
    header_format: xlsxwriter.format.Format,
) -> None:
    worksheet.freeze_panes(1, 0)
    for column_index, name in enumerate(columns):
        worksheet.write(0, column_index, name, header_format)
    for row_index, row in enumerate(rows, start=1):
        for column_index, name in enumerate(columns):
            value = row.get(name)
            if value is not None:
                worksheet.write(row_index, column_index, value)
    for column_index, name in enumerate(columns):
        worksheet.set_column(
            column_index, column_index, max(14, min(32, len(name) + 2))
        )


def _write_px4_workbook(output: Path, rows: Sequence[dict[str, object]]) -> None:
    buckets: dict[str, list[dict[str, object]]] = {name: [] for name in _storage_schema.PX4_XLSX_TAB_COLUMNS}
    for row in rows:
        sample_type = str(row.get("sample_type") or "")
        topic = str(row.get("topic") or "")
        if sample_type == "pixhawk_attitude":
            bucket = "ATTITUDE"
        elif sample_type == "gps_rtk":
            bucket = "GPS2" if "GPS2" in topic.upper() else "GPS1"
        elif sample_type == "magnetometer":
            bucket = "MAG"
        elif sample_type == "mag_slot_status":
            bucket = "MAG_CONFIG"
        elif sample_type == "mavlink_global_position_int":
            bucket = "GLOBAL_POSITION"
        elif sample_type == "mavlink_local_position_ned":
            bucket = "LOCAL_POSITION_NED"
        elif sample_type == "mavlink_raw_imu":
            bucket = "RAW_IMU"
        else:
            continue
        buckets[bucket].append(row)

    output.parent.mkdir(parents=True, exist_ok=True)
    fd, temp_name = tempfile.mkstemp(dir=output.parent, prefix=f".{output.name}.", suffix=".tmp")
    os.close(fd)
    temp_path = Path(temp_name)
    try:
        with xlsxwriter.Workbook(temp_path) as workbook:
            header_format = workbook.add_format({"bold": True, "bg_color": "#e0e0e0"})
            for name, columns in _storage_schema.PX4_XLSX_TAB_COLUMNS.items():
                worksheet = workbook.add_worksheet(name)
                _write_table(worksheet, list(columns), buckets[name], header_format)
        os.replace(temp_path, output)
    finally:
        temp_path.unlink(missing_ok=True)


def _compact_rtk_rows(
    rows: dict[str, Sequence[dict[str, object]]], metadata: dict[str, object]
) -> list[dict[str, object]]:
    compact: list[dict[str, object]] = []
    for side, side_rows in rows.items():
        for row in side_rows:
            if row.get("sample_type") != "gnss_relative":
                continue
            recording_time_ns = _storage_projection._recording_time_ns(row, metadata)
            compact.append(
                {
                    "side": side,
                    "recording_time_ns": recording_time_ns,
                    "session_utc_time_ns": _storage_projection._estimated_utc_time_ns(row, metadata, recording_time_ns),
                    "topic": row.get("topic"),
                    "heading_deg": row.get("heading_deg"),
                    "heading_accuracy_deg": row.get("heading_accuracy_deg"),
                    "position_length_m": row.get("position_length_m"),
                    "accuracy_length_m": row.get("accuracy_length_m"),
                    "gnss_fix_ok": row.get("gnss_fix_ok"),
                    "differential_solution": row.get("differential_solution"),
                    "relative_position_valid": row.get("relative_position_valid"),
                    "carrier_solution_floating": row.get("carrier_solution_floating"),
                    "carrier_solution_fixed": row.get("carrier_solution_fixed"),
                    "moving_base_mode": row.get("moving_base_mode"),
                    "heading_valid": row.get("heading_valid"),
                    "device_id": row.get("device_id"),
                }
            )
    return compact


def _write_rtk_workbook(output: Path, rows: Sequence[dict[str, object]]) -> None:
    output.parent.mkdir(parents=True, exist_ok=True)
    fd, temp_name = tempfile.mkstemp(dir=output.parent, prefix=f".{output.name}.", suffix=".tmp")
    os.close(fd)
    temp_path = Path(temp_name)
    try:
        with xlsxwriter.Workbook(temp_path) as workbook:
            header_format = workbook.add_format({"bold": True, "bg_color": "#e0e0e0"})
            worksheet = workbook.add_worksheet("GPS_RTK")
            _write_table(worksheet, list(_storage_schema.RTK_XLSX_COLUMNS), rows, header_format)
        os.replace(temp_path, output)
    finally:
        temp_path.unlink(missing_ok=True)


def _write_key_value_sheet(
    worksheet: xlsxwriter.worksheet.Worksheet,
    entries: Iterable[tuple[str, object]],
    header_format: xlsxwriter.format.Format,
) -> None:
    worksheet.write_row(0, 0, ["key", "value"], header_format)
    for row_index, (key, value) in enumerate(entries, start=1):
        worksheet.write(row_index, 0, key)
        if value is not None:
            worksheet.write(row_index, 1, value)
    worksheet.set_column(0, 0, 36)
    worksheet.set_column(1, 1, 64)


def write_dynamic_csv(
    output: Path,
    rows: Iterable[dict[str, object]],
    *,
    required_columns: Iterable[str] = _storage_schema.PAIR_AXIS_COLUMNS,
) -> None:
    output.parent.mkdir(parents=True, exist_ok=True)
    if not isinstance(rows, Sequence):
        rows = list(rows)
    fieldnames = _storage_projection._ordered_fieldnames(rows, required_columns)
    temp_path: Path | None = None
    try:
        with tempfile.NamedTemporaryFile(
            "w", newline="", encoding="utf-8", dir=output.parent,
            prefix=f".{output.name}.", suffix=".tmp", delete=False,
        ) as handle:
            temp_path = Path(handle.name)
            writer = csv.DictWriter(handle, fieldnames=fieldnames, extrasaction="ignore")
            writer.writeheader()
            writer.writerows(rows)
            handle.flush()
            os.fsync(handle.fileno())
        os.replace(temp_path, output)
    finally:
        if temp_path is not None:
            temp_path.unlink(missing_ok=True)


def write_parquet(
    output: Path,
    rows: Iterable[dict[str, object]],
    *,
    columns: Iterable[str],
) -> None:
    output.parent.mkdir(parents=True, exist_ok=True)
    columns = list(columns)
    records = [
        {column: row.get(column) for column in columns}
        for row in rows
    ]
    if records:
        table = pa.Table.from_pylist(records)
        table = table.select(columns)
    else:
        table = pa.Table.from_pydict({column: [] for column in columns})
    temp_path: Path | None = None
    try:
        fd, temp_name = tempfile.mkstemp(
            dir=output.parent, prefix=f".{output.name}.", suffix=".tmp"
        )
        os.close(fd)
        temp_path = Path(temp_name)
        pq.write_table(table, temp_path)
        os.replace(temp_path, output)
    finally:
        if temp_path is not None:
            temp_path.unlink(missing_ok=True)


def _write_session_static_json(
    output: Path,
    metadata: dict[str, object],
    *,
    resource_files: dict[str, str],
) -> None:
    payload = {
        "file_type": "stereowiris_session_static",
        "schema_version": "stereowiris-resource-session-static-v1",
        "time_axis": {
            "primary": "recording_time_ns",
            "definition": "nanoseconds elapsed from the session recording start",
        },
        "resource_files": resource_files,
        "metadata": metadata,
    }
    _storage_manifest._write_manifest(output, payload)


def _write_xlsx(
    output: Path,
    metadata: dict[str, object],
    distance_rows: list[dict[str, object]],
    *,
    camera_rows: dict[str, list[dict[str, object]]] | None = None,
    px4_rows: dict[str, list[dict[str, object]]] | None = None,
    gnss_base_rows: list[dict[str, object]] | None = None,
    extrinsics_comparison_rows: Sequence[dict[str, object]] | None = None,
) -> None:
    camera_rows = camera_rows or {}
    px4_rows = px4_rows or {}
    gnss_base_rows = gnss_base_rows or []
    extrinsics_comparison_rows = extrinsics_comparison_rows or ()
    sheet_rows = {
        "DistanceMeasurements": distance_rows,
        "CameraLeft": camera_rows.get("left", []),
        "CameraRight": camera_rows.get("right", []),
        "Px4Left": px4_rows.get("left", []),
        "Px4Right": px4_rows.get("right", []),
        "GnssBase": gnss_base_rows,
    }
    for sheet_name, _method, method_rows in _extrinsics_comparison_sheet_groups(
        extrinsics_comparison_rows
    ):
        sheet_rows[sheet_name] = method_rows
    previews = session_sheet_previews(
        metadata,
        distance_rows,
        camera_rows=camera_rows,
        px4_rows=px4_rows,
        gnss_base_rows=gnss_base_rows,
        extrinsics_comparison_rows=extrinsics_comparison_rows,
    )
    output.parent.mkdir(parents=True, exist_ok=True)
    fd, temp_name = tempfile.mkstemp(dir=output.parent, prefix=f".{output.name}.", suffix=".tmp")
    os.close(fd)
    temp_path = Path(temp_name)
    try:
        with xlsxwriter.Workbook(temp_path) as workbook:
            header_format = workbook.add_format({"bold": True, "bg_color": "#e0e0e0"})
            for preview in previews:
                worksheet = workbook.add_worksheet(preview.name)
                if preview.kind == "key_value":
                    _write_key_value_sheet(worksheet, list(preview.entries), header_format)
                else:
                    _write_table(
                        worksheet,
                        list(preview.columns),
                        sheet_rows.get(preview.name, []),
                        header_format,
                    )
        os.replace(temp_path, output)
    finally:
        temp_path.unlink(missing_ok=True)
