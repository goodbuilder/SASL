"""Derived XLSX view for the latest sessionless Workstation Measurement.

Timestamped JSONL remains authoritative.  This module reads an immutable
byte-cutoff view of timestamped or legacy files, selects one exact runtime
generation, and atomically publishes a spreadsheet that can be regenerated at
any time.
"""

from __future__ import annotations

from collections import OrderedDict
from collections.abc import Callable, Iterable, Iterator, Mapping, Sequence
from dataclasses import dataclass, field
from datetime import datetime, timedelta, timezone
import hashlib
import json
import math
import os
from pathlib import Path
import re
import tempfile
import threading
from typing import Any
import zipfile
from xml.etree import ElementTree

import xlsxwriter

from Workstation.workstation_core.measurement_data_store import (
    STREAM_PATHS,
    creation_timestamp_text,
)


LATEST_MEASUREMENT_WORKBOOK_NAME = "latest_measurement.xlsx"
LATEST_MEASUREMENT_WORKBOOK_STEM = "latest_measurement"
LATEST_MEASUREMENT_WORKBOOK_SUFFIX = ".xlsx"
LATEST_MEASUREMENT_WORKBOOK_SCHEMA = "workstation-latest-measurement-xlsx-v1"
EXCEL_MAX_ROWS = 1_048_576
EXCEL_MAX_COLUMNS = 16_384
EXCEL_MAX_TEXT_LENGTH = 32_767
CELL_TEXT_CHUNK = 32_000
MAX_RECORDED_ISSUES = 10_000

KST = timezone(timedelta(hours=9), name="KST")
UTC = timezone.utc

BASE_SHEET_NAMES = (
    "Summary",
    "Events",
    "Results",
    "ResultHistory",
    "MethodBundles",
    "MethodOutcomes",
    "MethodInputs",
    "CameraLeft",
    "CameraRight",
    "PixhawkP1",
    "PixhawkP2",
    "PixhawkDrone",
    "GnssBase",
    "GnssBaselineCenter",
    "Geometry",
    "NativeFrames",
    "Files",
    "Issues",
)

_IDENTITY_COLUMNS = (
    "runtime_generation",
    "measurement_id",
    "_record_index",
    "_record_operation",
    "pair_id",
    "pair_sequence",
    "method_id",
    "event_type",
    "recorded_utc_ns",
    "recorded_utc_ns_kst_iso",
)

_DEFAULT_COLUMNS: dict[str, tuple[str, ...]] = {
    "Events": ("runtime_generation", "event_type", "recorded_utc_ns"),
    "Results": ("measurement_id", "_record_index", "pair_id", "recorded_utc_ns"),
    "ResultHistory": (
        "measurement_id",
        "_record_index",
        "_record_operation",
        "pair_id",
        "recorded_utc_ns",
    ),
    "MethodBundles": (
        "measurement_id",
        "pair_id",
        "pair_sequence",
        "outcome_count",
        "recorded_utc_ns",
    ),
    "MethodOutcomes": (
        "measurement_id",
        "pair_id",
        "pair_sequence",
        "method_id",
        "status",
    ),
    "MethodInputs": ("measurement_id", "pair_id", "pair_sequence", "schema"),
    "CameraLeft": ("measurement_id", "sequence", "sample_type", "recorded_utc_ns"),
    "CameraRight": ("measurement_id", "sequence", "sample_type", "recorded_utc_ns"),
    "PixhawkP1": ("measurement_id", "sequence", "sample_type", "recorded_utc_ns"),
    "PixhawkP2": ("measurement_id", "sequence", "sample_type", "recorded_utc_ns"),
    "PixhawkDrone": ("measurement_id", "sequence", "sample_type", "recorded_utc_ns"),
    "GnssBase": ("measurement_id", "sequence", "sample_type", "recorded_utc_ns"),
    "GnssBaselineCenter": (
        "measurement_id",
        "sequence",
        "geometry_state",
        "gnss_baseline_center_forward_m",
        "gnss_baseline_center_right_m",
        "gnss_baseline_center_down_m",
        "gnss_baseline_center_range_m",
        "three_source_timestamp_spread_ms",
        "recorded_utc_ns",
    ),
    "Geometry": ("measurement_id", "method", "recorded_utc_ns"),
    "NativeFrames": ("runtime_generation", "request_id", "event_type", "recorded_utc_ns"),
    "Files": ("runtime_generation", "event_type", "side", "path", "recorded_utc_ns"),
    "Issues": ("stream", "line_number", "byte_offset", "error", "sha256"),
}

_INVALID_XML = re.compile(
    "[\x00-\x08\x0B\x0C\x0E-\x1F\uD800-\uDFFF\uFFFE\uFFFF]"
)
_EPOCH_FIELD = re.compile(r"(?:utc|unix)_(ns|us|s)$", re.IGNORECASE)


class LatestMeasurementExportCancelled(RuntimeError):
    """Raised cooperatively before a generated workbook is published."""


def timestamped_workbook_name(
    utc_ns: int | None = None,
    *,
    sequence: int = 0,
) -> str:
    """Return a non-overwriting filename for one derived workbook."""

    suffix = "" if int(sequence) == 0 else f"-{int(sequence):02d}"
    return (
        f"{LATEST_MEASUREMENT_WORKBOOK_STEM}_"
        f"{creation_timestamp_text(utc_ns)}"
        f"{suffix}"
        f"{LATEST_MEASUREMENT_WORKBOOK_SUFFIX}"
    )


@dataclass(frozen=True)
class LatestMeasurementSelection:
    generation: str
    started_utc_ns: int | None = None
    stopped_utc_ns: int | None = None
    completion_evidence: str = "explicit"


@dataclass(frozen=True)
class LatestMeasurementWorkbookResult:
    path: Path
    generation: str
    generated_utc_ns: int
    row_counts: Mapping[str, int]
    issue_count: int
    checkpoint_settled: bool
    checkpoint_pending: int
    completion_evidence: str


@dataclass(frozen=True)
class _SheetSpec:
    name: str
    stream: str
    project: Callable[[Mapping[str, object]], Iterable[Mapping[str, object]]]


@dataclass
class _SheetProfile:
    name: str
    default_columns: tuple[str, ...]
    row_count: int = 0
    _columns: OrderedDict[str, None] = field(default_factory=OrderedDict)

    def observe(self, row: Mapping[str, object]) -> None:
        self.row_count += 1
        for key in row:
            self._columns.setdefault(str(key), None)

    def columns(self) -> tuple[str, ...]:
        present = set(self._columns)
        ordered: list[str] = []
        for name in (*_IDENTITY_COLUMNS, *self.default_columns):
            if name in present and name not in ordered:
                ordered.append(name)
        remaining = present.difference(ordered)
        bases = sorted(
            {name.removesuffix("_kst_iso") for name in remaining},
            key=str.casefold,
        )
        for base in bases:
            if base in remaining:
                ordered.append(base)
            companion = f"{base}_kst_iso"
            if companion in remaining:
                ordered.append(companion)
        return tuple(ordered or self.default_columns or ("_source_status",))


@dataclass(frozen=True)
class _JsonlIssue:
    stream: str
    line_number: int
    byte_offset: int
    error: str
    sha256: str

    def to_row(self) -> dict[str, object]:
        return {
            "stream": self.stream,
            "line_number": self.line_number,
            "byte_offset": self.byte_offset,
            "error": self.error,
            "sha256": self.sha256,
        }


def _identity(row: Mapping[str, object]) -> str:
    for key in ("runtime_generation", "measurement_id"):
        value = row.get(key)
        if value not in (None, ""):
            return str(value)
    snapshot = row.get("snapshot_payload")
    if isinstance(snapshot, Mapping) and snapshot.get("measurement_id") not in (None, ""):
        return str(snapshot["measurement_id"])
    return ""


def _project_raw(row: Mapping[str, object]) -> Iterable[Mapping[str, object]]:
    yield row


def _project_method_bundle(row: Mapping[str, object]) -> Iterable[Mapping[str, object]]:
    projected = {
        str(key): value
        for key, value in row.items()
        if str(key) not in {"outcomes", "snapshot_payload"}
    }
    outcomes = row.get("outcomes")
    projected["outcome_count"] = len(outcomes) if isinstance(outcomes, list) else 0
    yield projected


def _project_method_outcomes(row: Mapping[str, object]) -> Iterable[Mapping[str, object]]:
    outcomes = row.get("outcomes")
    if not isinstance(outcomes, list):
        return
    parent = {
        "measurement_id": row.get("measurement_id"),
        "pair_id": row.get("pair_id"),
        "pair_sequence": row.get("pair_sequence"),
        "bundle_snapshot_sha256": row.get("snapshot_sha256"),
        "bundle_sha256": row.get("method_evaluation_bundle_sha256"),
        "bundle_recorded_utc_ns": row.get("recorded_utc_ns"),
    }
    for outcome_index, value in enumerate(outcomes):
        if not isinstance(value, Mapping):
            continue
        yield {**dict(value), **parent, "outcome_index": outcome_index}


def _project_method_inputs(row: Mapping[str, object]) -> Iterable[Mapping[str, object]]:
    snapshot = row.get("snapshot_payload")
    if not isinstance(snapshot, Mapping):
        return
    yield {
        **dict(snapshot),
        "measurement_id": row.get("measurement_id"),
        "pair_id": row.get("pair_id"),
        "pair_sequence": row.get("pair_sequence"),
        "bundle_snapshot_sha256": row.get("snapshot_sha256"),
    }


_SHEET_SPECS: tuple[_SheetSpec, ...] = (
    _SheetSpec("Events", "events", _project_raw),
    _SheetSpec("ResultHistory", "measurements", _project_raw),
    _SheetSpec("MethodBundles", "method_evaluations", _project_method_bundle),
    _SheetSpec("MethodOutcomes", "method_evaluations", _project_method_outcomes),
    _SheetSpec("MethodInputs", "method_evaluations", _project_method_inputs),
    _SheetSpec("CameraLeft", "camera.left", _project_raw),
    _SheetSpec("CameraRight", "camera.right", _project_raw),
    _SheetSpec("PixhawkP1", "px4.left", _project_raw),
    _SheetSpec("PixhawkP2", "px4.right", _project_raw),
    _SheetSpec("PixhawkDrone", "px4.drone", _project_raw),
    _SheetSpec("GnssBase", "gnss.base", _project_raw),
    _SheetSpec("GnssBaselineCenter", "gnss.baseline.center", _project_raw),
    _SheetSpec("Geometry", "geometry.comparisons", _project_raw),
    _SheetSpec("NativeFrames", "native.frames", _project_raw),
    _SheetSpec("Files", "artifacts", _project_raw),
)


def _reject_json_constant(value: str) -> None:
    raise ValueError(f"non-finite JSON constant: {value}")


def _iter_jsonl(
    root: Path,
    stream: str,
    *,
    byte_limit: int | None,
    issues: list[_JsonlIssue] | None,
    cancel_event: threading.Event | None,
    path: Path | None = None,
) -> Iterator[dict[str, object]]:
    path = root / STREAM_PATHS[stream] if path is None else Path(path)
    if not path.is_file() or path.is_symlink():
        return
    limit = max(0, path.stat().st_size if byte_limit is None else int(byte_limit))
    with path.open("rb") as handle:
        line_number = 0
        while handle.tell() < limit:
            if cancel_event is not None and cancel_event.is_set():
                raise LatestMeasurementExportCancelled("latest Measurement export cancelled")
            offset = handle.tell()
            line = handle.readline(limit - offset)
            if not line:
                break
            line_number += 1
            if not line.endswith(b"\n"):
                if issues is not None and len(issues) < MAX_RECORDED_ISSUES:
                    issues.append(
                        _JsonlIssue(
                            stream,
                            line_number,
                            offset,
                            "incomplete final JSONL record excluded",
                            hashlib.sha256(line).hexdigest(),
                        )
                    )
                break
            try:
                decoded = line.decode("utf-8")
                value = json.loads(decoded, parse_constant=_reject_json_constant)
                if not isinstance(value, dict):
                    raise ValueError("JSONL record is not an object")
            except (UnicodeDecodeError, ValueError, TypeError) as exc:
                if issues is not None and len(issues) < MAX_RECORDED_ISSUES:
                    issues.append(
                        _JsonlIssue(
                            stream,
                            line_number,
                            offset,
                            str(exc),
                            hashlib.sha256(line).hexdigest(),
                        )
                    )
                continue
            yield {str(key): item for key, item in value.items()}


def _stream_candidate_paths(
    root: Path,
    stream: str,
    stream_paths: Mapping[str, object] | None,
) -> tuple[Path, ...]:
    """Resolve current, legacy, and timestamped files for one stream."""

    relative = STREAM_PATHS[stream]
    values: list[Path] = []
    provided = None if stream_paths is None else stream_paths.get(stream)
    if isinstance(provided, (str, Path)):
        provided_values = (provided,)
    elif isinstance(provided, Sequence):
        provided_values = tuple(provided)
    else:
        provided_values = ()
    for value in provided_values:
        candidate = Path(value)
        if not candidate.is_absolute():
            candidate = root / candidate
        values.append(candidate)
    values.append(root / relative)
    values.extend(
        sorted(
            root.glob(f"{relative.stem}_*{relative.suffix}"),
            key=lambda path: path.name,
        )
    )
    unique: list[Path] = []
    for candidate in values:
        if candidate not in unique:
            unique.append(candidate)
    return tuple(unique)


def _stream_path_limit(
    root: Path,
    stream: str,
    path: Path,
    *,
    stream_byte_limits: Mapping[str, int] | None,
    stream_byte_limits_by_path: Mapping[str, int] | None,
) -> int | None:
    """Return a checkpoint limit for one physical stream path."""

    try:
        key = path.relative_to(root).as_posix()
    except ValueError:
        key = path.as_posix()
    if stream_byte_limits_by_path is not None and key in stream_byte_limits_by_path:
        return max(0, int(stream_byte_limits_by_path[key]))
    legacy = root / STREAM_PATHS[stream]
    if path == legacy and stream_byte_limits is not None and stream in stream_byte_limits:
        return max(0, int(stream_byte_limits[stream]))
    return None


def _iter_stream_jsonl(
    root: Path,
    stream: str,
    *,
    stream_paths: Mapping[str, object] | None,
    stream_byte_limits: Mapping[str, int] | None,
    stream_byte_limits_by_path: Mapping[str, int] | None,
    issues: list[_JsonlIssue] | None,
    cancel_event: threading.Event | None,
) -> Iterator[dict[str, object]]:
    for path in _stream_candidate_paths(root, stream, stream_paths):
        yield from _iter_jsonl(
            root,
            stream,
            byte_limit=_stream_path_limit(
                root,
                stream,
                path,
                stream_byte_limits=stream_byte_limits,
                stream_byte_limits_by_path=stream_byte_limits_by_path,
            ),
            issues=issues,
            cancel_event=cancel_event,
            path=path,
        )


def discover_latest_measurement(
    root: Path | str,
    *,
    stream_byte_limits: Mapping[str, int] | None = None,
    stream_paths: Mapping[str, object] | None = None,
    stream_byte_limits_by_path: Mapping[str, int] | None = None,
    cancel_event: threading.Event | None = None,
) -> LatestMeasurementSelection:
    """Discover the newest completed generation, with a transparent fallback."""

    resolved = Path(root).expanduser().resolve(strict=False)
    limits = dict(stream_byte_limits or {})
    latest_stop: tuple[int, dict[str, object]] | None = None
    for row in _iter_stream_jsonl(
        resolved,
        "events",
        stream_paths=stream_paths,
        stream_byte_limits=limits,
        stream_byte_limits_by_path=stream_byte_limits_by_path,
        issues=None,
        cancel_event=cancel_event,
    ):
        if row.get("event_type") != "measurement_stopped":
            continue
        generation = str(row.get("runtime_generation") or "")
        if not generation:
            continue
        clock = int(row.get("stopped_utc_ns") or row.get("recorded_utc_ns") or 0)
        if latest_stop is None or clock >= latest_stop[0]:
            latest_stop = (clock, row)
    if latest_stop is not None:
        stopped = latest_stop[1]
        generation = str(stopped["runtime_generation"])
        started_utc_ns: int | None = None
        for row in _iter_stream_jsonl(
            resolved,
            "events",
            stream_paths=stream_paths,
            stream_byte_limits=limits,
            stream_byte_limits_by_path=stream_byte_limits_by_path,
            issues=None,
            cancel_event=cancel_event,
        ):
            if (
                row.get("event_type") == "measurement_start_requested"
                and str(row.get("runtime_generation") or "") == generation
            ):
                started_utc_ns = _optional_int(
                    row.get("started_utc_ns") or row.get("recorded_utc_ns")
                )
        return LatestMeasurementSelection(
            generation=generation,
            started_utc_ns=started_utc_ns,
            stopped_utc_ns=_optional_int(
                stopped.get("stopped_utc_ns") or stopped.get("recorded_utc_ns")
            ),
            completion_evidence="measurement_stopped",
        )

    newest: tuple[int, str] | None = None
    for stream in (
        "measurements",
        "method_evaluations",
        "camera.left",
        "camera.right",
        "px4.left",
        "px4.right",
        "px4.drone",
        "gnss.base",
        "gnss.baseline.center",
        "geometry.comparisons",
        "native.frames",
        "artifacts",
    ):
        for row in _iter_stream_jsonl(
            resolved,
            stream,
            stream_paths=stream_paths,
            stream_byte_limits=limits,
            stream_byte_limits_by_path=stream_byte_limits_by_path,
            issues=None,
            cancel_event=cancel_event,
        ):
            generation = _identity(row)
            if not generation:
                continue
            clock = int(row.get("recorded_utc_ns") or 0)
            if newest is None or clock >= newest[0]:
                newest = (clock, generation)
    if newest is None:
        raise LookupError("no saved Measurement generation is available")
    return LatestMeasurementSelection(
        generation=newest[1],
        completion_evidence="inferred_saved_identity",
    )


def _optional_int(value: object) -> int | None:
    if value in (None, ""):
        return None
    try:
        return int(value)
    except (TypeError, ValueError, OverflowError):
        return None


def _escape_key(value: object) -> str:
    return str(value).replace("\\", "\\\\").replace(".", "\\.")


def _flatten(value: Mapping[str, object], prefix: str = "") -> Iterator[tuple[str, object]]:
    for raw_key, item in value.items():
        key = _escape_key(raw_key)
        name = f"{prefix}.{key}" if prefix else key
        if isinstance(item, Mapping):
            yield from _flatten({str(k): v for k, v in item.items()}, name)
        elif isinstance(item, (list, tuple)):
            yield name, json.dumps(
                item,
                ensure_ascii=False,
                allow_nan=False,
                sort_keys=True,
                separators=(",", ":"),
                default=str,
            )
        else:
            yield name, item


def _epoch_unit(name: str) -> str | None:
    match = _EPOCH_FIELD.search(name)
    return None if match is None else match.group(1).lower()


def _kst_iso(value: object, unit: str) -> str | None:
    try:
        numeric = float(value) if unit == "s" else int(value)
        if not math.isfinite(float(numeric)):
            return None
        nanoseconds = (
            int(round(float(numeric) * 1_000_000_000))
            if unit == "s"
            else int(numeric) * (1_000 if unit == "us" else 1)
        )
        seconds, remainder = divmod(nanoseconds, 1_000_000_000)
        local = datetime.fromtimestamp(seconds, tz=UTC).astimezone(KST)
    except (OverflowError, OSError, TypeError, ValueError):
        return None
    offset = local.strftime("%z")
    offset = f"{offset[:3]}:{offset[3:]}" if len(offset) == 5 else offset
    return f"{local:%Y-%m-%dT%H:%M:%S}.{remainder:09d}{offset}"


def _split_text(name: str, value: object) -> Iterator[tuple[str, object]]:
    if not isinstance(value, str) or len(value) <= CELL_TEXT_CHUNK:
        yield name, value
        return
    for index in range(0, len(value), CELL_TEXT_CHUNK):
        part = index // CELL_TEXT_CHUNK + 1
        yield f"{name}__part{part:03d}", value[index : index + CELL_TEXT_CHUNK]


def prepare_excel_row(row: Mapping[str, object]) -> OrderedDict[str, object]:
    """Flatten one JSON object without losing exact scalar values."""

    prepared: OrderedDict[str, object] = OrderedDict()
    for name, value in _flatten({str(key): item for key, item in row.items()}):
        for part_name, part_value in _split_text(name, value):
            prepared[part_name] = part_value
        unit = _epoch_unit(name)
        if unit is not None:
            prepared[f"{name}_kst_iso"] = _kst_iso(value, unit)
    return prepared


def _materialized_results(
    root: Path,
    generation: str,
    *,
    byte_limit: int | None,
    stream_paths: Mapping[str, object] | None,
    stream_byte_limits: Mapping[str, int] | None,
    stream_byte_limits_by_path: Mapping[str, int] | None,
    cancel_event: threading.Event | None,
) -> list[OrderedDict[str, object]]:
    current: OrderedDict[object, OrderedDict[str, object]] = OrderedDict()
    fallback = 0
    for row in _iter_stream_jsonl(
        root,
        "measurements",
        stream_paths=stream_paths,
        stream_byte_limits=stream_byte_limits,
        stream_byte_limits_by_path=stream_byte_limits_by_path,
        issues=None,
        cancel_event=cancel_event,
    ):
        if _identity(row) != generation:
            continue
        index = row.get("_record_index")
        if index is None:
            index = f"fallback:{fallback}"
            fallback += 1
        current[index] = prepare_excel_row(row)
    return list(current.values())


def _profile_export(
    root: Path,
    generation: str,
    *,
    stream_byte_limits: Mapping[str, int],
    stream_paths: Mapping[str, object] | None,
    stream_byte_limits_by_path: Mapping[str, int] | None,
    cancel_event: threading.Event | None,
) -> tuple[
    dict[str, _SheetProfile],
    list[_JsonlIssue],
    list[OrderedDict[str, object]],
]:
    profiles = {
        name: _SheetProfile(name, _DEFAULT_COLUMNS.get(name, ("_source_status",)))
        for name in BASE_SHEET_NAMES
        if name != "Summary"
    }
    issues: list[_JsonlIssue] = []
    specs_by_stream: dict[str, list[_SheetSpec]] = {}
    for spec in _SHEET_SPECS:
        specs_by_stream.setdefault(spec.stream, []).append(spec)

    for stream, specs in specs_by_stream.items():
        for row in _iter_stream_jsonl(
            root,
            stream,
            stream_paths=stream_paths,
            stream_byte_limits=stream_byte_limits,
            stream_byte_limits_by_path=stream_byte_limits_by_path,
            issues=issues,
            cancel_event=cancel_event,
        ):
            if _identity(row) != generation:
                continue
            for spec in specs:
                for projected in spec.project(row):
                    profiles[spec.name].observe(prepare_excel_row(projected))

    result_rows = _materialized_results(
        root,
        generation,
        byte_limit=stream_byte_limits.get("measurements"),
        stream_paths=stream_paths,
        stream_byte_limits=stream_byte_limits,
        stream_byte_limits_by_path=stream_byte_limits_by_path,
        cancel_event=cancel_event,
    )
    for row in result_rows:
        profiles["Results"].observe(row)
    for issue in issues:
        profiles["Issues"].observe(prepare_excel_row(issue.to_row()))
    return profiles, issues, result_rows


def _iter_sheet_rows(
    spec: _SheetSpec,
    root: Path,
    generation: str,
    *,
    byte_limit: int | None,
    stream_paths: Mapping[str, object] | None,
    stream_byte_limits: Mapping[str, int] | None,
    stream_byte_limits_by_path: Mapping[str, int] | None,
    cancel_event: threading.Event | None,
) -> Iterator[OrderedDict[str, object]]:
    for row in _iter_stream_jsonl(
        root,
        spec.stream,
        stream_paths=stream_paths,
        stream_byte_limits=stream_byte_limits,
        stream_byte_limits_by_path=stream_byte_limits_by_path,
        issues=None,
        cancel_event=cancel_event,
    ):
        if _identity(row) != generation:
            continue
        for projected in spec.project(row):
            yield prepare_excel_row(projected)


def _column_partitions(columns: Sequence[str], maximum: int) -> tuple[tuple[str, ...], ...]:
    if len(columns) <= maximum:
        return (tuple(columns),)
    repeated = tuple(name for name in _IDENTITY_COLUMNS if name in columns)
    if len(repeated) >= maximum:
        repeated = repeated[: max(1, maximum - 1)]
    remaining = [name for name in columns if name not in repeated]
    capacity = max(1, maximum - len(repeated))
    return tuple(
        tuple((*repeated, *remaining[index : index + capacity]))
        for index in range(0, len(remaining), capacity)
    )


def _shard_name(base: str, row_index: int, column_index: int, rows: int, columns: int) -> str:
    if rows == 1 and columns == 1:
        return base
    suffix = f"_r{row_index + 1:03d}c{column_index + 1:02d}"
    return f"{base[: 31 - len(suffix)]}{suffix}"


def _formats(workbook: xlsxwriter.Workbook) -> dict[str, Any]:
    return {
        "header": workbook.add_format(
            {
                "bold": True,
                "font_color": "#FFFFFF",
                "bg_color": "#333333",
                "border": 1,
                "align": "center",
            }
        ),
        "text": workbook.add_format({"valign": "top"}),
        "wrap": workbook.add_format({"valign": "top", "text_wrap": True}),
        "integer": workbook.add_format({"num_format": "0"}),
        "decimal": workbook.add_format({"num_format": "0.###############"}),
        "title": workbook.add_format({"bold": True, "font_size": 15, "font_color": "#990000"}),
        "note": workbook.add_format({"font_color": "#595959", "text_wrap": True}),
    }


def _clean_text(value: str) -> str:
    return _INVALID_XML.sub("�", value)[:EXCEL_MAX_TEXT_LENGTH]


def _write_cell(
    worksheet: Any,
    row_index: int,
    column_index: int,
    value: object,
    formats: Mapping[str, Any],
) -> None:
    if value is None:
        worksheet.write_formula(row_index, column_index, "=NA()", None, "#N/A")
    elif isinstance(value, bool):
        worksheet.write_boolean(row_index, column_index, value)
    elif isinstance(value, int):
        if abs(value) > 999_999_999_999_999:
            worksheet.write_string(row_index, column_index, str(value), formats["text"])
        else:
            worksheet.write_number(row_index, column_index, value, formats["integer"])
    elif isinstance(value, float):
        if math.isfinite(value):
            worksheet.write_number(row_index, column_index, value, formats["decimal"])
        else:
            worksheet.write_formula(row_index, column_index, "=NA()", None, "#N/A")
    else:
        worksheet.write_string(
            row_index,
            column_index,
            _clean_text(str(value)),
            formats["wrap"] if len(str(value)) > 80 else formats["text"],
        )


def _write_summary(
    workbook: xlsxwriter.Workbook,
    *,
    formats: Mapping[str, Any],
    selection: LatestMeasurementSelection,
    profiles: Mapping[str, _SheetProfile],
    generated_utc_ns: int,
    checkpoint_settled: bool,
    checkpoint_pending: int,
    storage_snapshot: Mapping[str, object],
    source_states: Mapping[str, object],
) -> None:
    worksheet = workbook.add_worksheet("Summary")
    worksheet.hide_gridlines(2)
    worksheet.set_column(0, 0, 24)
    worksheet.set_column(1, 1, 58)
    worksheet.set_column(2, 3, 22)
    worksheet.write_string(0, 0, "Latest Measurement XLSX", formats["title"])
    worksheet.write_string(
        1,
        0,
        "Derived view only; timestamped JSONL remains authoritative. Manual edits may be replaced.",
        formats["note"],
    )
    fields: tuple[tuple[str, object], ...] = (
        ("workbook_schema", LATEST_MEASUREMENT_WORKBOOK_SCHEMA),
        ("generation", selection.generation),
        ("completion_evidence", selection.completion_evidence),
        ("started_utc_ns", selection.started_utc_ns),
        ("started_kst_iso", _kst_iso(selection.started_utc_ns, "ns")),
        ("stopped_utc_ns", selection.stopped_utc_ns),
        ("stopped_kst_iso", _kst_iso(selection.stopped_utc_ns, "ns")),
        ("generated_utc_ns", generated_utc_ns),
        ("generated_kst_iso", _kst_iso(generated_utc_ns, "ns")),
        ("checkpoint_settled", checkpoint_settled),
        ("checkpoint_pending", checkpoint_pending),
        ("storage_state", storage_snapshot.get("state")),
        ("storage_dropped_records_cumulative", storage_snapshot.get("dropped_records")),
    )
    for row_index, (name, value) in enumerate(fields, start=3):
        worksheet.write_string(row_index, 0, name, formats["header"])
        _write_cell(worksheet, row_index, 1, value, formats)

    start = 3 + len(fields) + 2
    for column_index, name in enumerate(("sheet", "saved_rows", "source_state", "note")):
        worksheet.write_string(start, column_index, name, formats["header"])
    stream_states = storage_snapshot.get("streams")
    stream_states = stream_states if isinstance(stream_states, Mapping) else {}
    spec_by_name = {spec.name: spec for spec in _SHEET_SPECS}
    source_by_sheet = {
        "CameraLeft": "camera.left",
        "CameraRight": "camera.right",
        "PixhawkP1": "pixhawk.left",
        "PixhawkP2": "pixhawk.right",
        "PixhawkDrone": "target.drone",
        "GnssBase": "gnss.base",
    }
    for row_index, name in enumerate(BASE_SHEET_NAMES[1:-1], start=start + 1):
        profile = profiles[name]
        spec = spec_by_name.get(name)
        stream_value = stream_states.get(spec.stream, {}) if spec is not None else {}
        stream_value = stream_value if isinstance(stream_value, Mapping) else {}
        source_value = source_states.get(source_by_sheet.get(name, ""), {})
        source_value = source_value if isinstance(source_value, Mapping) else {}
        worksheet.write_string(row_index, 0, name, formats["text"])
        worksheet.write_number(row_index, 1, profile.row_count, formats["integer"])
        _write_cell(
            worksheet,
            row_index,
            2,
            source_value.get("state") or stream_value.get("state")
            if profile.row_count
            else None,
            formats,
        )
        note = "no saved rows" if not profile.row_count else "generation matched"
        worksheet.write_string(row_index, 3, note, formats["text"])


def _write_table(
    workbook: xlsxwriter.Workbook,
    *,
    profile: _SheetProfile,
    rows: Iterable[Mapping[str, object]],
    formats: Mapping[str, Any],
    max_rows_per_sheet: int,
    max_columns_per_sheet: int,
    cancel_event: threading.Event | None,
) -> None:
    columns = profile.columns()
    partitions = _column_partitions(columns, max_columns_per_sheet)
    data_rows = max_rows_per_sheet - 1
    row_shards = max(1, math.ceil(profile.row_count / data_rows))
    worksheets: list[list[Any]] = []
    for row_shard in range(row_shards):
        row_group: list[Any] = []
        rows_in_shard = min(data_rows, max(0, profile.row_count - row_shard * data_rows))
        for column_shard, partition in enumerate(partitions):
            name = _shard_name(
                profile.name,
                row_shard,
                column_shard,
                row_shards,
                len(partitions),
            )
            worksheet = workbook.add_worksheet(name)
            worksheet.freeze_panes(1, 0)
            worksheet.hide_gridlines(2)
            for column_index, column in enumerate(partition):
                worksheet.write_string(0, column_index, column, formats["header"])
                width = 48 if "path" in column.lower() else min(38, max(13, len(column) + 2))
                worksheet.set_column(column_index, column_index, width)
            worksheet.autofilter(0, 0, rows_in_shard, len(partition) - 1)
            row_group.append(worksheet)
        worksheets.append(row_group)

    written = 0
    for written, value in enumerate(rows, start=1):
        if cancel_event is not None and cancel_event.is_set():
            raise LatestMeasurementExportCancelled("latest Measurement export cancelled")
        zero_index = written - 1
        row_shard = zero_index // data_rows
        worksheet_row = zero_index % data_rows + 1
        for column_shard, partition in enumerate(partitions):
            worksheet = worksheets[row_shard][column_shard]
            for column_index, column in enumerate(partition):
                _write_cell(worksheet, worksheet_row, column_index, value.get(column), formats)
    if written != profile.row_count:
        raise ValueError(
            f"{profile.name} row count changed during export: "
            f"profile={profile.row_count}, written={written}"
        )


def _write_workbook(
    path: Path,
    *,
    root: Path,
    selection: LatestMeasurementSelection,
    stream_byte_limits: Mapping[str, int],
    stream_paths: Mapping[str, object] | None,
    stream_byte_limits_by_path: Mapping[str, int] | None,
    storage_snapshot: Mapping[str, object],
    source_states: Mapping[str, object],
    checkpoint_settled: bool,
    checkpoint_pending: int,
    cancel_event: threading.Event | None,
    max_rows_per_sheet: int,
    max_columns_per_sheet: int,
) -> tuple[dict[str, int], int, int]:
    profiles, issues, result_rows = _profile_export(
        root,
        selection.generation,
        stream_byte_limits=stream_byte_limits,
        stream_paths=stream_paths,
        stream_byte_limits_by_path=stream_byte_limits_by_path,
        cancel_event=cancel_event,
    )
    generated_utc_ns = time_ns()
    with xlsxwriter.Workbook(
        path,
        {
            "constant_memory": True,
            "strings_to_formulas": False,
            "strings_to_urls": False,
            "nan_inf_to_errors": False,
        },
    ) as workbook:
        workbook.use_zip64()
        workbook.set_properties(
            {
                "title": "Workstation latest Measurement",
                "comments": "Derived view; timestamped JSONL remains authoritative.",
            }
        )
        workbook.set_custom_property(
            "workstation_schema", LATEST_MEASUREMENT_WORKBOOK_SCHEMA
        )
        workbook.set_custom_property("runtime_generation", selection.generation)
        formats = _formats(workbook)
        _write_summary(
            workbook,
            formats=formats,
            selection=selection,
            profiles=profiles,
            generated_utc_ns=generated_utc_ns,
            checkpoint_settled=checkpoint_settled,
            checkpoint_pending=checkpoint_pending,
            storage_snapshot=storage_snapshot,
            source_states=source_states,
        )
        specs = {spec.name: spec for spec in _SHEET_SPECS}
        for name in BASE_SHEET_NAMES[1:]:
            profile = profiles[name]
            if name == "Results":
                rows: Iterable[Mapping[str, object]] = result_rows
            elif name == "Issues":
                rows = (prepare_excel_row(issue.to_row()) for issue in issues)
            else:
                spec = specs[name]
                rows = _iter_sheet_rows(
                    spec,
                    root,
                    selection.generation,
                    byte_limit=stream_byte_limits.get(spec.stream),
                    stream_paths=stream_paths,
                    stream_byte_limits=stream_byte_limits,
                    stream_byte_limits_by_path=stream_byte_limits_by_path,
                    cancel_event=cancel_event,
                )
            _write_table(
                workbook,
                profile=profile,
                rows=rows,
                formats=formats,
                max_rows_per_sheet=max_rows_per_sheet,
                max_columns_per_sheet=max_columns_per_sheet,
                cancel_event=cancel_event,
            )
    return (
        {name: profile.row_count for name, profile in profiles.items()},
        len(issues),
        generated_utc_ns,
    )


def time_ns() -> int:
    # Kept as one patchable boundary for deterministic tests.
    import time

    return time.time_ns()


def _read_custom_properties(path: Path) -> dict[str, str]:
    if not zipfile.is_zipfile(path):
        return {}
    with zipfile.ZipFile(path) as archive:
        try:
            payload = archive.read("docProps/custom.xml")
        except KeyError:
            return {}
    root = ElementTree.fromstring(payload)
    result: dict[str, str] = {}
    for element in root:
        name = str(element.attrib.get("name") or "")
        value = "" if len(element) == 0 else str(element[0].text or "")
        if name:
            result[name] = value
    return result


def workbook_identity(path: Path | str) -> dict[str, str]:
    return _read_custom_properties(Path(path))


def _validate_workbook(path: Path) -> None:
    if not zipfile.is_zipfile(path):
        raise ValueError("generated latest Measurement workbook is not a valid XLSX")
    properties = _read_custom_properties(path)
    if properties.get("workstation_schema") != LATEST_MEASUREMENT_WORKBOOK_SCHEMA:
        raise ValueError("generated workbook ownership schema is missing")
    with zipfile.ZipFile(path) as archive:
        required = {"[Content_Types].xml", "xl/workbook.xml", "docProps/custom.xml"}
        if not required.issubset(archive.namelist()):
            raise ValueError("generated workbook is missing required XLSX members")
        root = ElementTree.fromstring(archive.read("xl/workbook.xml"))
        namespace = {"m": "http://schemas.openxmlformats.org/spreadsheetml/2006/main"}
        names = {
            str(element.attrib.get("name") or "")
            for element in root.findall("m:sheets/m:sheet", namespace)
        }
    missing = [
        base
        for base in BASE_SHEET_NAMES
        if base not in names and not any(name.startswith(f"{base}_r") for name in names)
    ]
    if missing:
        raise ValueError("generated workbook is missing sheets: " + ", ".join(missing))


def _fsync_directory(path: Path) -> None:
    flags = os.O_RDONLY | getattr(os, "O_DIRECTORY", 0)
    try:
        descriptor = os.open(path, flags)
    except OSError:
        return
    try:
        os.fsync(descriptor)
    finally:
        os.close(descriptor)


def _publish_owned_workbook(temporary: Path, output: Path) -> None:
    if output.is_symlink() or (output.exists() and not output.is_file()):
        raise FileExistsError(f"latest Measurement XLSX target is not an owned file: {output}")
    if output.exists():
        identity = _read_custom_properties(output)
        if identity.get("workstation_schema") != LATEST_MEASUREMENT_WORKBOOK_SCHEMA:
            raise FileExistsError(
                f"refusing to replace unowned latest Measurement XLSX: {output}"
            )
        os.replace(temporary, output)
    else:
        try:
            os.link(temporary, output)
        except FileExistsError:
            raise FileExistsError(
                f"latest Measurement XLSX appeared during publication: {output}"
            ) from None
        temporary.unlink()
    _fsync_directory(output.parent)


def _validated_workbook_name(value: str | None) -> str:
    name = LATEST_MEASUREMENT_WORKBOOK_NAME if value is None else str(value).strip()
    path = Path(name)
    if (
        not name
        or path.name != name
        or path.suffix.lower() != LATEST_MEASUREMENT_WORKBOOK_SUFFIX
        or any(part in {"", ".", ".."} for part in path.parts)
        or "\x00" in name
    ):
        raise ValueError("latest Measurement workbook name must be one safe .xlsx filename")
    return name


def write_latest_measurement_xlsx(
    root: Path | str,
    *,
    generation: str | None = None,
    started_utc_ns: int | None = None,
    stopped_utc_ns: int | None = None,
    completion_evidence: str = "explicit",
    output_name: str | None = None,
    stream_byte_limits: Mapping[str, int] | None = None,
    stream_paths: Mapping[str, object] | None = None,
    stream_byte_limits_by_path: Mapping[str, int] | None = None,
    storage_snapshot: Mapping[str, object] | None = None,
    source_states: Mapping[str, object] | None = None,
    checkpoint_settled: bool = True,
    checkpoint_pending: int = 0,
    cancel_event: threading.Event | None = None,
    max_rows_per_sheet: int = EXCEL_MAX_ROWS,
    max_columns_per_sheet: int = EXCEL_MAX_COLUMNS,
) -> LatestMeasurementWorkbookResult:
    """Create and atomically publish the latest Measurement workbook."""

    resolved = Path(root).expanduser().resolve(strict=False)
    resolved.mkdir(parents=True, exist_ok=True)
    limits = {
        stream: max(0, int(limit))
        for stream, limit in dict(stream_byte_limits or {}).items()
        if stream in STREAM_PATHS
    }
    for stream, relative in STREAM_PATHS.items():
        if stream not in limits:
            path = resolved / relative
            limits[stream] = path.stat().st_size if path.is_file() and not path.is_symlink() else 0
    if generation is None:
        selection = discover_latest_measurement(
            resolved,
            stream_byte_limits=limits,
            stream_paths=stream_paths,
            stream_byte_limits_by_path=stream_byte_limits_by_path,
            cancel_event=cancel_event,
        )
    else:
        selected = str(generation).strip()
        if not selected or not re.fullmatch(r"[A-Za-z0-9._-]+", selected):
            raise ValueError("latest Measurement generation must be a safe non-empty token")
        selection = LatestMeasurementSelection(
            generation=selected,
            started_utc_ns=_optional_int(started_utc_ns),
            stopped_utc_ns=_optional_int(stopped_utc_ns),
            completion_evidence=str(completion_evidence or "explicit"),
        )
    if cancel_event is not None and cancel_event.is_set():
        raise LatestMeasurementExportCancelled("latest Measurement export cancelled")
    output = resolved / _validated_workbook_name(output_name)
    descriptor, temporary_name = tempfile.mkstemp(
        dir=resolved,
        prefix=f".{LATEST_MEASUREMENT_WORKBOOK_NAME}.",
        suffix=".tmp.xlsx",
    )
    os.close(descriptor)
    temporary = Path(temporary_name)
    try:
        row_counts, issue_count, generated_utc_ns = _write_workbook(
            temporary,
            root=resolved,
            selection=selection,
            stream_byte_limits=limits,
            stream_paths=stream_paths,
            stream_byte_limits_by_path=stream_byte_limits_by_path,
            storage_snapshot=dict(storage_snapshot or {}),
            source_states=dict(source_states or {}),
            checkpoint_settled=bool(checkpoint_settled),
            checkpoint_pending=max(0, int(checkpoint_pending)),
            cancel_event=cancel_event,
            max_rows_per_sheet=max(2, min(EXCEL_MAX_ROWS, int(max_rows_per_sheet))),
            max_columns_per_sheet=max(2, min(EXCEL_MAX_COLUMNS, int(max_columns_per_sheet))),
        )
        _validate_workbook(temporary)
        with temporary.open("rb") as handle:
            os.fsync(handle.fileno())
        if cancel_event is not None and cancel_event.is_set():
            raise LatestMeasurementExportCancelled("latest Measurement export cancelled")
        _publish_owned_workbook(temporary, output)
        return LatestMeasurementWorkbookResult(
            path=output,
            generation=selection.generation,
            generated_utc_ns=generated_utc_ns,
            row_counts=row_counts,
            issue_count=issue_count,
            checkpoint_settled=bool(checkpoint_settled),
            checkpoint_pending=max(0, int(checkpoint_pending)),
            completion_evidence=selection.completion_evidence,
        )
    finally:
        temporary.unlink(missing_ok=True)


__all__ = [
    "BASE_SHEET_NAMES",
    "EXCEL_MAX_COLUMNS",
    "EXCEL_MAX_ROWS",
    "LATEST_MEASUREMENT_WORKBOOK_NAME",
    "LATEST_MEASUREMENT_WORKBOOK_STEM",
    "LATEST_MEASUREMENT_WORKBOOK_SCHEMA",
    "LatestMeasurementExportCancelled",
    "LatestMeasurementSelection",
    "LatestMeasurementWorkbookResult",
    "discover_latest_measurement",
    "prepare_excel_row",
    "workbook_identity",
    "timestamped_workbook_name",
    "write_latest_measurement_xlsx",
]
