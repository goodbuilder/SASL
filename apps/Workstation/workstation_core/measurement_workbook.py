"""Per-Measurement XLSX workbook: sheet layout, atomic publish, background writer.

The workbook is the operator-facing, standalone record of one Measurement.
Every data value is a literal; reader sheets show missing values as ``—``
and compatibility evidence tables retain ``N/A``. R,t appears both in the
dedicated comparison sheet and expanded beside every Method result;
nanosecond/microsecond integers are written as text so Excel cannot round
them. A complete temporary workbook is written, closed, fsynced and then
atomically renamed over the previous version, so a crash always leaves the
last good workbook in place.
"""

from __future__ import annotations

from Workstation.workstation_core.target_level_coordinates import LEVEL_OUTPUT_FIELDS

import json
import math
import os
import platform
import subprocess
import sys
import threading
import time
import unicodedata
from dataclasses import replace
from collections.abc import Iterable, Mapping, Sequence
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

from Workstation.workstation_core.strict_jsonl import fsync_directory as _fsync_directory

import xlsxwriter

from Workstation.workstation_core.measurement_methods import (
    MEASUREMENT_METHOD_IDS,
    MEASUREMENT_METHOD_LABELS,
)
from Workstation.workstation_core.measurement_record import (
    ALWAYS_PRESENT_TELEMETRY_SHEETS,
    METHOD_SHEET_NAMES,
    STATUS_NOT_ATTEMPTED,
    STATUS_PENDING,
    MeasurementRecord,
    MeasurementSnapshot,
    MethodResultRow,
)
from Workstation.workstation_core.method_result_evidence import (
    flat_transform_cells,
    source_evidence_rows,
)
from Workstation.workstation_core.measurement_report import (
    FRONT_SHEETS, append_trend_chart, rt_id, rt_sources, write_front_sheets,
)
from Workstation.workstation_core.measurement_artifacts import KST

WORKBOOK_SCHEMA = "workstation-measurement-workbook-v2"
NA = "N/A"
MAX_SHEET_ROWS = 1_048_575
_EXACT_INT_LIMIT = 2**53
_TEXT_INT_SUFFIXES = ("_ns", "_us", "time_boot_ms", "_utc_ms")
_STATUS_ORDER = ("VALID", "ESTIMATED", "BLOCKED", "FAILED", "BACKPRESSURE_BLOCKED")

SECTION_A = "[A] Geometry / Result"
SECTION_B = "[B] Selected Detection"
SECTION_C = "[C] Actual ray input"
SECTION_D = "[D] R,t source evidence"
SECTION_E = "[E] Geometry diagnostics"

SECTION_A_COLUMNS: tuple[str, ...] = (
    "pair_sequence", "elapsed_s", "Range", "Forward", "Right", "Down", "Status", "Reason",
    "target_latitude_deg", "target_longitude_deg", "target_altitude_ellipsoid_m", "target_altitude_msl_m", *LEVEL_OUTPUT_FIELDS,
    "R11", "R12", "R13", "R21", "R22", "R23", "R31", "R32", "R33",
    "T_R", "T_D", "T_F",
    "rt_id", "R_source", "T_source",
    "measurement_id", "pair_id", "time_local", "time_utc_ns", "Attempted", "time_delta_s",
    "method_instance_id",
    "worker_started_monotonic_ns",
    "completed_monotonic_ns",
)
SECTION_B_COLUMNS: tuple[str, ...] = (
    "pair_id",
    "L_detection_id", "L_x1", "L_y1", "L_x2", "L_y2", "L_confidence", "L_class_id", "L_class_name",
    "R_detection_id", "R_x1", "R_y1", "R_x2", "R_y2", "R_confidence", "R_class_id", "R_class_name",
)
SECTION_C_COLUMNS: tuple[str, ...] = (
    "pair_id",
    "L_u", "L_v", "L_ray_R", "L_ray_D", "L_ray_F",
    "R_u", "R_v", "R_ray_R", "R_ray_D", "R_ray_F",
)
SECTION_D_COLUMNS: tuple[str, ...] = ("pair_id", "field", "value", "source", "unit_or_frame")
SECTION_E_COLUMNS: tuple[str, ...] = (
    "pair_id",
    "ray_gap_m",
    "ray_angle_deg",
    "left_depth_m",
    "right_depth_m",
    "triangulation_reason",
    "warnings",
)
FLAT_COLUMNS: tuple[str, ...] = (
    "measurement_id", "pair_id", "pair_sequence", "method_id", "Attempted", "Status",
    "L_detection_id", "L_x1", "L_y1", "L_x2", "L_y2", "L_confidence", "L_class_id", "L_class_name",
    "R_detection_id", "R_x1", "R_y1", "R_x2", "R_y2", "R_confidence", "R_class_id", "R_class_name",
    "L_u", "L_v", "R_u", "R_v",
    "R11", "R12", "R13", "R21", "R22", "R23", "R31", "R32", "R33",
    "T_R", "T_D", "T_F",
    "Forward", "Right", "Down", "Range",
    "target_latitude_deg", "target_longitude_deg", "target_altitude_ellipsoid_m", "target_altitude_msl_m", *LEVEL_OUTPUT_FIELDS,
    "ray_gap_m", "ray_angle_deg", "left_depth_m", "right_depth_m",
    "time_delta_s", "Reason",
)
DETECTION_COLUMNS: tuple[str, ...] = (
    "measurement_id", "pair_id", "pair_sequence", "frame_id", "frame_key", "detection_id",
    "selected", "timestamp_s", "app_receive_monotonic_ns",
    "x1", "y1", "x2", "y2", "confidence", "class_id", "class_name",
    "image_width", "image_height", "stream_generation", "rtcp_frame_ntp_unix_s",
)
CALIBRATION_COLUMNS: tuple[str, ...] = (
    "measurement_id", "side", "profile_sha256", "source_path", "image_width", "image_height",
    "K11", "K12", "K13", "K21", "K22", "K23", "K31", "K32", "K33",
    "distortion_model", "D_count",
    "D1", "D2", "D3", "D4", "D5", "D6", "D7", "D8",
    "reprojection_error_px", "image_count", "recorded_utc_ns",
)
ALIGNMENT_COLUMNS: tuple[str, ...] = (
    "measurement_id", "timestamp_utc_ns", "event_type", "method_id", "source",
    "R11", "R12", "R13", "R21", "R22", "R23", "R31", "R32", "R33",
    "T_R", "T_D", "T_F", "source_sha256", "detail",
)
EVENT_COLUMNS: tuple[str, ...] = ("measurement_id", "event_type", "utc_ns", "utc_local", "monotonic_ns", "detail", "payload")
METHOD_COMPARE_FRONT_COLUMNS: tuple[str, ...] = (
    "measurement_id", "pair_id", "pair_sequence", "time_local", "time_utc_ns",
    "disparity_px", "time_delta_s",
)
METHOD_COMPARE_FIELDS: tuple[str, ...] = ("status", "range_m", "forward_m", "right_m", "down_m", "ray_angle_deg",
    "target_latitude_deg", "target_longitude_deg", "target_altitude_ellipsoid_m", "target_altitude_msl_m", *LEVEL_OUTPUT_FIELDS)
OPTICAL_POSE_COLUMNS: tuple[str, ...] = (
    "measurement_id", "record_source", "event_type", "utc_ns", "utc_local",
    "mode", "suggestion_id", "state", "eligible", "inlier_count", "match_count",
    "residual_px", "confidence",
    "relative_yaw_right_minus_left_deg", "relative_pitch_right_minus_left_deg",
    "relative_roll_right_minus_left_deg", "relative_fit_inlier_count", "relative_fit_residual_px",
    "far_field_du_median_px", "far_field_dv_median_px",
    "gnss_baseline_m", "gnss_baseline_fix_quality", "left_image_sha256", "right_image_sha256",
)
EXACT_COLUMNS: tuple[str, ...] = ("measurement_id", "pair_id", "method_id", "field", "value", "value_repr", "value_hex")
TELEMETRY_FRONT_COLUMNS: tuple[str, ...] = (
    "measurement_id", "side", "sample_type", "sequence",
    "sample_monotonic_ns", "sample_monotonic_s", "host_receive_monotonic_ns", "host_receive_utc_ns",
    "session_utc_ns", "source_utc_ns", "session_elapsed_s",
)
TELEMETRY_TECHNICAL_COLUMNS: tuple[str, ...] = (
    "namespace", "topic", "mavlink_message_name", "mavlink_source_system", "mavlink_source_component",
    "mavlink_sequence", "mavlink_message_id", "telemetry_transport", "mavlink_endpoint",
    "mavlink_link_open", "mavlink_heartbeat_seen", "mavlink_target_system", "mavlink_target_component",
    "row_contract", "session_id", "session_utc_method", "session_utc_anchor_uncertainty_ns",
    "source_utc_method", "sample_timestamp_source", "sample_timestamp_fallback_reason",
    "camera", "triangulation_extrinsics_source", "distance_mode", "schema",
    "_record_index", "_record_operation", "recorded_utc_ns", "recorded_monotonic_ns",
    "nonfinite_source_fields", "nonfinite_source_reasons",
)

DEFINITIONS: tuple[tuple[str, str], ...] = (
    ("Measurement", "One operator interval from Measurement Start to Measurement Stop. One workbook per Measurement; several stereo pairs may exist inside it."),
    ("Stereo pair", "One Left detection + one Right detection used together for distance. pair_id and pair_sequence identify it in every sheet."),
    ("Attempted", "YES when the Method was actually submitted to a solver for that pair; NO when it was not ready (Status N/A)."),
    ("Status", "VALID: computed without warnings. ESTIMATED: computed with warnings. BLOCKED: geometry undefined (see Reason). FAILED: worker/solver failure. BACKPRESSURE_BLOCKED: worker queue full. PENDING: submitted, result not yet received. N/A: not attempted."),
    ("N/A", "Value not available / Method not attempted. Analysis code must treat it as a missing value."),
    ("RDF", "OpenCV optical camera frame: axes Right, Down, Forward (metres for points, unit vectors for rays)."),
    ("FRD", "Forward, Right, Down. Public result frame baseline_center_frd: origin at the midpoint between the two optical centres, Forward = average optical axis, Right = left->right baseline direction."),
    ("NED", "World frame North, East, Down used by Pixhawk attitude (body FRD -> NED) and GNSS baselines."),
    ("R_right_from_left (R11..R33)", "3x3 rotation, row-major. X_right = R_right_from_left * X_left + T_right_from_left (both in optical RDF). Equals OpenCV stereoCalibrate R."),
    ("T_right_from_left (T_R, T_D, T_F)", "Translation of the canonical transform in metres, components in the right-camera RDF axes. It is NOT the right camera position in the left frame; that is C = -R^T T."),
    ("u, v", "Actual solver input pixel on the native image grid (bbox centre today). Rays are undistorted unit vectors from u,v and the side's K,D."),
    ("Forward / Right / Down / Range", "Triangulated target in baseline_center_frd metres; Range = Euclidean distance from that origin."),
    ("time_delta_s", "Absolute difference between the Left and Right timestamps actually used to select the pair."),
    ("ray_gap_m", "Closest distance between the two rays (called closest_gap_m in JSONL). Large values mean the rays barely meet."),
    ("ray_angle_deg", "Angle between the two rays."),
    ("left_depth_m / right_depth_m", "Signed distance along each ray to its closest point; negative means the closest point lies behind that camera. Kept as evidence, never used to drop a result."),
    ("Reproduction", "Undistort u,v with K,D -> unit rays; kernel inputs R_left_from_right = R^T, C_right_in_left = -R^T T; closest-ray midpoint; convert to baseline_center_frd. 91_Exact holds float.hex() of every input for bit-exact replay."),
    ("PX4 sheets", "Every telemetry sample the Workstation consumed between Start and Stop, one row per sample, no downsampling. *_ns/*_us columns are text to keep full integer precision."),
)


def _kst_or_local_iso(utc_ns: int | None) -> str:
    if utc_ns is None:
        return NA
    try:
        seconds, nanos = divmod(int(utc_ns), 1_000_000_000)
        return datetime.fromtimestamp(seconds, tz=KST).replace(microsecond=nanos // 1000).isoformat(timespec="milliseconds")
    except (OverflowError, OSError, ValueError):
        return NA


def _compact_json(value: object, limit: int = 32_000) -> str:
    try:
        text = json.dumps(value, ensure_ascii=False, sort_keys=True, separators=(",", ":"), default=str)
    except (TypeError, ValueError):
        text = str(value)
    return text if len(text) <= limit else text[: limit - 3] + "..."


class _SheetWriter:
    """Explicit-type cell writer with column formats and width tracking."""

    def __init__(self, workbook: xlsxwriter.Workbook, name: str, formats: Mapping[str, Any]) -> None:
        self._workbook = workbook
        self._base_name = name
        self._part = 0
        self.worksheet = workbook.add_worksheet(name)
        self.sheets = [self.worksheet]
        self.table_headers: dict[Any, int] = {}
        self.end_rows: dict[Any, int] = {}
        self.name = name
        self.formats = formats
        self.row = 0
        self.widths: dict[int, int] = {}
        self.rt_links: Mapping[str, tuple[str, int]] = {}
        self.display_na = "—" if name in FRONT_SHEETS else NA
        self._row_heights: dict[int, float] = {}

    @property
    def max_row(self) -> int:
        return MAX_SHEET_ROWS

    def _continue(self) -> None:
        self.end_rows[self.worksheet] = self.row - 1
        self.finish()
        self._part += 1
        self.name = f"{self._base_name[:26]}_{self._part:03d}"
        self.worksheet = self._workbook.add_worksheet(self.name)
        self.sheets.append(self.worksheet)
        self.row = 0
        self.widths = {}
        self._row_heights = {}

    def ensure_room(self, rows: int = 1) -> None:
        if self.row + rows - 1 > self.max_row:
            self._continue()

    def data_row(self, columns: Sequence[str]) -> None:
        if self.row > self.max_row:
            previous = self.worksheet
            header = self.table_headers.get(previous, 0)
            previous.autofilter(header, 0, self.max_row, len(columns) - 1)
            previous.freeze_panes(header + 1, 2)
            self._continue()
            self.header(columns)

    def _note_width(self, column: int, text: str) -> None:
        width = min(60, max(4, len(text) + 1))
        if width > self.widths.get(column, 0):
            self.widths[column] = width

    def cell(self, row: int, column: int, value: object, *, header: str = "") -> None:
        worksheet = self.worksheet
        if value is None:
            worksheet.write_string(row, column, self.display_na, self.formats["na"])
            return
        if isinstance(value, bool):
            worksheet.write_string(row, column, "YES" if value else "NO", self.formats["body"])
            self._note_width(column, "YES")
            return
        if isinstance(value, int):
            if header.endswith(_TEXT_INT_SUFFIXES) or abs(value) >= _EXACT_INT_LIMIT:
                text = str(value)
                worksheet.write_string(row, column, text, self.formats["text_int"])
                self._note_width(column, text)
            else:
                worksheet.write_number(row, column, value, self.formats["integer"])
                self._note_width(column, str(value))
            return
        if isinstance(value, float):
            if not math.isfinite(value):
                worksheet.write_string(row, column, NA if math.isnan(value) else repr(value), self.formats["na"])
                return
            if header.endswith(_TEXT_INT_SUFFIXES) and value.is_integer():
                text = str(int(value))
                worksheet.write_string(row, column, text, self.formats["text_int"])
                self._note_width(column, text)
                return
            worksheet.write_number(row, column, value, self._number_format(header) or self.formats["number"])
            self._note_width(column, f"{value:.6f}")
            return
        if isinstance(value, (list, tuple, dict)):
            text = _compact_json(value)
        else:
            text = str(value)
        if header in {"rt_id", "R,t 번호", "Method"} and text in self.rt_links:
            name, target_row = self.rt_links[text]
            worksheet.write_url(row, column, f"internal:'{name}'!A{target_row + 1}", self.formats["link"], string=text)
        else:
            wrap_width = {"Reason": 37, "사유": 36, "경고": 40, "R_source": 48, "T_source": 48, "R 출처": 31, "t 출처": 31, "term": 37, "definition": 78}.get(header)
            worksheet.write_string(row, column, text, self.formats["wrap"] if wrap_width else self.formats["body"])
            if wrap_width:
                lines = sum(max(1, math.ceil(sum(2 if unicodedata.east_asian_width(c) in "WF" else 1 for c in line) / wrap_width)) for line in text.splitlines() or [""])
                height = max(self._row_heights.get(row, 18), min(409, 18 * lines))
                if height > 18:
                    worksheet.set_row(row, height)
                    self._row_heights[row] = height
        self._note_width(column, text)

    def _number_format(self, header: str) -> Any:
        if header.endswith(("target_latitude_deg", "target_longitude_deg")):
            return self.formats["matrix"]
        if header.startswith("R") and len(header) == 3 and header[1:].isdigit():
            return self.formats["matrix"]
        if header.endswith(("_ray_R", "_ray_D", "_ray_F")):
            return self.formats["matrix"]
        if header in {"Forward", "Right", "Down", "Range", "T_R", "T_D", "T_F"} or header.endswith(("_m", "(m)")):
            return self.formats["metre"]
        if header.endswith(("_deg", "(deg)")):
            return self.formats["degree"]
        if header.endswith(("_s", "(s)")):
            return self.formats["elapsed"]
        return None

    def header(self, columns: Sequence[str]) -> int:
        self.ensure_room(2)
        for index, name in enumerate(columns):
            self.worksheet.write_string(self.row, index, str(name), self.formats["header"])
            self._note_width(index, str(name))
        header_row = self.row
        self.table_headers[self.worksheet] = header_row
        self.row += 1
        return header_row

    def table(
        self,
        columns: Sequence[str],
        rows: Iterable[Mapping[str, object]],
        *,
        freeze: bool = True,
        autofilter: bool = True,
    ) -> int:
        header_row = self.header(columns)
        count = 0
        part_count = 0
        for row in rows:
            if self.row > self.max_row:
                if autofilter and part_count:
                    self.worksheet.autofilter(header_row, 0, self.row - 1, max(0, len(columns) - 1))
                if freeze:
                    self.worksheet.freeze_panes(header_row + 1, 2)
                self._continue()
                header_row = self.header(columns)
                part_count = 0
            for index, name in enumerate(columns):
                self.cell(self.row, index, row.get(name), header=name)
            self.row += 1
            count += 1
            part_count += 1
        if freeze:
            self.worksheet.freeze_panes(header_row + 1, 0)
        if autofilter and count:
            self.worksheet.autofilter(header_row, 0, header_row + part_count, max(0, len(columns) - 1))
        return count

    def marker(self, text: str) -> None:
        self.ensure_room()
        self.worksheet.write_string(self.row, 0, text, self.formats["section"])
        self.row += 1

    def blank(self) -> None:
        self.row += 1

    def key_values(self, items: Iterable[tuple[str, object]]) -> None:
        for key, value in items:
            self.ensure_room()
            self.worksheet.write_string(self.row, 0, str(key), self.formats["key"])
            self._note_width(0, str(key))
            self.cell(self.row, 1, value, header=str(key))
            self.row += 1

    def finish(self) -> None:
        for column, width in self.widths.items():
            self.worksheet.set_column(column, column, width)
        self.worksheet.set_default_row(18)
        self.worksheet.hide_gridlines(2)


def _formats(workbook: xlsxwriter.Workbook) -> dict[str, Any]:
    base = {"font_name": "Noto Sans CJK KR", "font_size": 11, "font_color": "#202020"}
    styles = {
        "body": {"align": "left", "indent": 1}, "number": {"align": "right"},
        "integer": {"num_format": "0", "align": "right"},
        "wrap": {"text_wrap": True, "valign": "top", "indent": 1},
        "header": {"bold": True, "align": "center", "bg_color": "#EAEAEA", "bottom": 1, "bottom_color": "#C8C8C8", "text_wrap": True},
        "section": {"bold": True, "font_size": 12}, "key": {"bold": True},
        "na": {"font_color": "#737373", "align": "center"}, "text_int": {"num_format": "@"},
        "metre": {"num_format": "0.000"}, "degree": {"num_format": "0.00"},
        "matrix": {"num_format": "0.000000000"}, "elapsed": {"num_format": "0.000"},
        "estimated_metre": {"num_format": '0.000" †"', "font_color": "#A02828"},
        "band": {"bg_color": "#F7F7F7"}, "link": {"font_color": "#5D2020", "underline": 1},
    }
    return {name: workbook.add_format(style if name == "band" else {**base, **style}) for name, style in styles.items()}


def _pair_snapshot_detection_cells(prefix: str, detection: Mapping[str, Any], detection_id: str) -> dict[str, object]:
    bbox = detection.get("bbox_xyxy_px") or (None, None, None, None)
    return {
        f"{prefix}_detection_id": detection_id or None,
        f"{prefix}_x1": bbox[0],
        f"{prefix}_y1": bbox[1],
        f"{prefix}_x2": bbox[2],
        f"{prefix}_y2": bbox[3],
        f"{prefix}_confidence": detection.get("confidence"),
        f"{prefix}_class_id": detection.get("class_id"),
        f"{prefix}_class_name": detection.get("class_name"),
    }


def _method_rows(snapshot: MeasurementSnapshot, method_id: str) -> list[MethodResultRow]:
    return [row for row in snapshot.method_results if row.method_id == method_id]


def _section_a_row(snapshot: MeasurementSnapshot, pair_by_id: Mapping[str, Any], row: MethodResultRow) -> dict[str, object]:
    pair = pair_by_id.get(row.pair_id)
    derived = row.derived or {}
    cells: dict[str, object] = {
        "measurement_id": snapshot.measurement_id,
        "pair_id": row.pair_id,
        "pair_sequence": row.pair_sequence,
        "elapsed_s": None if pair is None else (pair.created_monotonic_ns - snapshot.started_monotonic_ns) / 1e9,
        "rt_id": rt_id(row) or None,
        "R_source": rt_sources(row)[0], "T_source": rt_sources(row)[1],
        "time_local": _kst_or_local_iso(None if pair is None else pair.created_unix_ns),
        "time_utc_ns": None if pair is None else pair.created_unix_ns,
        "Attempted": bool(row.attempted),
        "Status": row.status,
        "Reason": row.reason or None,
        "time_delta_s": derived.get("time_delta_s") if derived.get("time_delta_s") is not None else (None if pair is None else pair.time_delta_s),
        "method_instance_id": derived.get("method_instance_id") or (row.evidence.get("instance_id") if row.evidence else None),
        "worker_started_monotonic_ns": derived.get("worker_started_monotonic_ns") or None,
        "completed_monotonic_ns": derived.get("completed_monotonic_ns") or None,
        "Forward": derived.get("forward_m"),
        "Right": derived.get("right_m"),
        "Down": derived.get("down_m"),
        "Range": derived.get("range_m"),
        **{name: derived.get(name) for name in (
            "target_latitude_deg", "target_longitude_deg", "target_altitude_ellipsoid_m", "target_altitude_msl_m", *LEVEL_OUTPUT_FIELDS,
        )},
    }
    cells.update(flat_transform_cells(derived))
    return cells


def _section_b_row(pair_by_id: Mapping[str, Any], row: MethodResultRow) -> dict[str, object]:
    pair = pair_by_id.get(row.pair_id)
    cells: dict[str, object] = {"pair_id": row.pair_id}
    if pair is None:
        return cells
    cells.update(_pair_snapshot_detection_cells("L", pair.left_detection, pair.left_detection_id))
    cells.update(_pair_snapshot_detection_cells("R", pair.right_detection, pair.right_detection_id))
    return cells


def _section_c_row(row: MethodResultRow) -> dict[str, object]:
    derived = row.derived or {}
    cells: dict[str, object] = {"pair_id": row.pair_id}
    for prefix, side in (("L", "left"), ("R", "right")):
        uv = derived.get(f"{side}_uv")
        ray = derived.get(f"{side}_ray")
        cells[f"{prefix}_u"] = None if uv is None else uv[0]
        cells[f"{prefix}_v"] = None if uv is None else uv[1]
        cells[f"{prefix}_ray_R"] = None if ray is None else ray[0]
        cells[f"{prefix}_ray_D"] = None if ray is None else ray[1]
        cells[f"{prefix}_ray_F"] = None if ray is None else ray[2]
    return cells


def _section_e_row(row: MethodResultRow) -> dict[str, object]:
    derived = row.derived or {}
    return {
        "pair_id": row.pair_id,
        "ray_gap_m": derived.get("ray_gap_m"),
        "ray_angle_deg": derived.get("ray_angle_deg"),
        "left_depth_m": derived.get("left_depth_m"),
        "right_depth_m": derived.get("right_depth_m"),
        "triangulation_reason": derived.get("triangulation_reason") or None,
        "warnings": "; ".join(derived.get("warnings", ()) or ()) or None,
    }


def _write_method_sheet(workbook: xlsxwriter.Workbook, formats: Mapping[str, Any], snapshot: MeasurementSnapshot, method_id: str, *, rt_links=None) -> None:
    sheet = _SheetWriter(workbook, METHOD_SHEET_NAMES[method_id], formats)
    sheet.rt_links = rt_links or {}
    rows = _method_rows(snapshot, method_id)
    pair_by_id = {pair.pair_id: pair for pair in snapshot.pairs}
    sheet.key_values(
        (
            ("Method", f"{method_id} — {MEASUREMENT_METHOD_LABELS.get(method_id, method_id)}"),
            ("Transform", "X_right = R_right_from_left_rdf * X_left + T_right_from_left_rdf_m (optical RDF)"),
            ("Result frame", "baseline_center_frd [Forward, Right, Down, Range] metres"),
        )
    )
    sheet.blank()
    sheet.marker(SECTION_A)
    header_row = sheet.row
    sheet.table(SECTION_A_COLUMNS, (_section_a_row(snapshot, pair_by_id, row) for row in rows))
    sheet.worksheet.freeze_panes(header_row + 1, 2)
    sheet.blank()
    sheet.marker(SECTION_B)
    sheet.table(SECTION_B_COLUMNS, (_section_b_row(pair_by_id, row) for row in rows), freeze=False, autofilter=False)
    sheet.blank()
    sheet.marker(SECTION_C)
    sheet.table(SECTION_C_COLUMNS, (_section_c_row(row) for row in rows), freeze=False, autofilter=False)
    sheet.blank()
    sheet.marker(SECTION_D)

    def evidence_rows() -> Iterable[dict[str, object]]:
        for row in rows:
            if row.outcome is None:
                for name, value in sorted((row.evidence or {}).items()):
                    yield {"pair_id": row.pair_id, "field": f"method.{name}", "value": _compact_json(value) if isinstance(value, (dict, list, tuple)) else value, "source": "method_status", "unit_or_frame": ""}
                continue
            for field, value, source, unit in source_evidence_rows(row.outcome, row.evidence):
                if unit in {"ns", "us"} and isinstance(value, int) and not isinstance(value, bool):
                    value = str(value)
                yield {"pair_id": row.pair_id, "field": field, "value": value, "source": source, "unit_or_frame": unit}

    sheet.table(SECTION_D_COLUMNS, evidence_rows(), freeze=False, autofilter=False)
    sheet.blank()
    sheet.marker(SECTION_E)
    sheet.table(SECTION_E_COLUMNS, (_section_e_row(row) for row in rows), freeze=False, autofilter=False)
    sheet.finish()
    for column in range(8, 20):
        sheet.worksheet.set_column(column, column, 16, None, {"hidden": False, "level": 0})
    sheet.worksheet.set_column(0, 1, 16)
    sheet.worksheet.set_column(2, 5, 15)
    sheet.worksheet.set_column(6, 6, 22)
    sheet.worksheet.set_column(7, 7, 40)
    if not any(row.attempted for row in rows):
        for worksheet in sheet.sheets:
            worksheet.hide()


def _calibration_rows(snapshot: MeasurementSnapshot) -> list[dict[str, object]]:
    rows: list[dict[str, object]] = []
    seen: set[tuple[str, str]] = set()

    def add(side: str, camera: Mapping[str, Any], *, profile_sha256: str, source_path: str, recorded_utc_ns: int | None) -> None:
        matrix = camera.get("camera_matrix") or ((None,) * 3,) * 3
        distortion = tuple(camera.get("distortion_coefficients") or ())
        key = (side, _compact_json({"m": matrix, "d": distortion}))
        if key in seen:
            return
        seen.add(key)
        row: dict[str, object] = {
            "measurement_id": snapshot.measurement_id,
            "side": side,
            "profile_sha256": profile_sha256 or None,
            "source_path": source_path or None,
            "image_width": camera.get("width_px", camera.get("image_width")),
            "image_height": camera.get("height_px", camera.get("image_height")),
            "distortion_model": camera.get("distortion_model", "opencv_plumb_bob(k1,k2,p1,p2,k3[,k4,k5,k6])"),
            "D_count": len(distortion),
            "reprojection_error_px": camera.get("reprojection_error_px"),
            "image_count": camera.get("image_count"),
            "recorded_utc_ns": recorded_utc_ns,
        }
        for r in range(3):
            for c in range(3):
                row[f"K{r + 1}{c + 1}"] = matrix[r][c]
        for index in range(8):
            row[f"D{index + 1}"] = distortion[index] if index < len(distortion) else None
        rows.append(row)

    calibration = snapshot.calibration or {}
    for side in ("left", "right"):
        camera = calibration.get(side)
        if isinstance(camera, Mapping):
            add(side, camera, profile_sha256=str(calibration.get("profile_sha256") or ""), source_path=str(calibration.get("source_path") or ""), recorded_utc_ns=snapshot.started_utc_ns)
    for result in snapshot.method_results:
        attempt = (result.outcome or {}).get("attempt_payload") if result.outcome else None
        intrinsics = attempt.get("intrinsics") if isinstance(attempt, Mapping) else None
        if not isinstance(intrinsics, Mapping):
            continue
        for side in ("left", "right"):
            camera = intrinsics.get(side)
            if isinstance(camera, Mapping):
                add(side, camera, profile_sha256=str(intrinsics.get("intrinsics_content_sha256") or intrinsics.get("content_sha256") or ""), source_path=str(intrinsics.get("selected_profile_path") or ""), recorded_utc_ns=None)
    return rows


def _alignment_rows(snapshot: MeasurementSnapshot) -> list[dict[str, object]]:
    rows: list[dict[str, object]] = []
    for raw in snapshot.alignment_rows:
        row: dict[str, object] = {"measurement_id": snapshot.measurement_id}
        if "R11" in raw:
            row.update({name: raw.get(name) for name in ALIGNMENT_COLUMNS if name in raw})
            row.setdefault("detail", raw.get("detail"))
        else:
            row.update(
                {
                    "timestamp_utc_ns": raw.get("updated_utc_ns") or raw.get("reset_utc_ns") or raw.get("recorded_utc_ns"),
                    "event_type": raw.get("event_type"),
                    "method_id": raw.get("method_id"),
                    "source": raw.get("source") or raw.get("previous_active_selection_id"),
                    "source_sha256": raw.get("source_sha256") or raw.get("previous_active_profile_hash"),
                    "detail": _compact_json({key: value for key, value in raw.items() if key not in {"schema"}}),
                }
            )
        rows.append(row)
    return rows


def _telemetry_columns(rows: Sequence[Mapping[str, Any]]) -> tuple[str, ...]:
    seen: list[str] = []
    present: set[str] = set()
    for row in rows:
        for key in row:
            if key not in present:
                present.add(key)
                seen.append(key)
    front = [name for name in TELEMETRY_FRONT_COLUMNS if name in present]
    technical = [name for name in TELEMETRY_TECHNICAL_COLUMNS if name in present]
    middle = [name for name in seen if name not in front and name not in technical]
    return tuple(front + middle + technical)


def _write_telemetry_sheet(workbook: xlsxwriter.Workbook, formats: Mapping[str, Any], name: str, rows: Sequence[Mapping[str, Any]]) -> None:
    columns = _telemetry_columns(rows) or ("measurement_id", "sample_type")
    for part, start in enumerate(range(0, max(1, len(rows)), MAX_SHEET_ROWS)):
        sheet_name = name if part == 0 else f"{name[:28]}_{part + 1}"
        sheet = _SheetWriter(workbook, sheet_name, formats)
        sheet.table(columns, rows[start : start + MAX_SHEET_ROWS])
        sheet.finish()
        if not rows:
            sheet.worksheet.hide()


_RT_CELL_NAMES: tuple[str, ...] = (
    "R11", "R12", "R13", "R21", "R22", "R23", "R31", "R32", "R33",
    "T_R", "T_D", "T_F",
)


def _finite_transform_cells(derived: Mapping[str, Any]) -> dict[str, float] | None:
    """Return the R,t cells of one method row only when all twelve are finite."""

    cells = flat_transform_cells(derived)
    values: dict[str, float] = {}
    for name in _RT_CELL_NAMES:
        value = cells.get(name)
        if not isinstance(value, (int, float)) or isinstance(value, bool) or not math.isfinite(float(value)):
            return None
        values[name] = float(value)
    return values


def _rotation_angle_deg(a: Mapping[str, float], b: Mapping[str, float]) -> float:
    """Geodesic angle between two rotation matrices given as R11..R33 cells."""

    trace = sum(a[f"R{i}{j}"] * b[f"R{i}{j}"] for i in (1, 2, 3) for j in (1, 2, 3))
    return math.degrees(math.acos(max(-1.0, min(1.0, (trace - 1.0) / 2.0))))


def _translation_distance_m(a: Mapping[str, float], b: Mapping[str, float]) -> float:
    return math.sqrt(sum((a[name] - b[name]) ** 2 for name in ("T_R", "T_D", "T_F")))


def _summary_method_rows(snapshot: MeasurementSnapshot) -> list[dict[str, object]]:
    """Legacy v1 projection; v2 reports use measurement_report instead."""
    rows: list[dict[str, object]] = []
    for method_id in MEASUREMENT_METHOD_IDS:
        results = _method_rows(snapshot, method_id)
        counts = {status: 0 for status in (*_STATUS_ORDER, STATUS_PENDING, STATUS_NOT_ATTEMPTED)}
        extents: dict[str, list[float]] = {"Forward": [], "Right": [], "Down": [], "Range": []}
        # R,t of every VALID row, in pair order, for the latest value and its spread.
        valid_transforms: list[tuple[int, dict[str, float]]] = []
        for row in results:
            counts[row.status] = counts.get(row.status, 0) + 1
            derived = row.derived or {}
            for column, key in (("Forward", "forward_m"), ("Right", "right_m"), ("Down", "down_m"), ("Range", "range_m")):
                value = derived.get(key)
                if isinstance(value, (int, float)) and math.isfinite(float(value)):
                    extents[column].append(float(value))
            if row.status == _STATUS_ORDER[0]:  # "VALID"
                transform = _finite_transform_cells(derived)
                if transform is not None:
                    valid_transforms.append((int(row.pair_sequence), transform))
        row: dict[str, object] = {
            "Method": method_id,
            "Sheet": METHOD_SHEET_NAMES[method_id],
            "Attempted count": sum(1 for item in results if item.attempted),
            **{f"{status} count": counts.get(status, 0) for status in (*_STATUS_ORDER, STATUS_PENDING, STATUS_NOT_ATTEMPTED)},
        }
        for column, values in extents.items():
            row[f"{column} min"] = min(values) if values else None
            row[f"{column} max"] = max(values) if values else None
        if valid_transforms:
            valid_transforms.sort(key=lambda item: item[0])
            latest_sequence, latest = valid_transforms[-1]
            row["R,t pair"] = latest_sequence
            row["R,t rows"] = len(valid_transforms)
            row.update(latest)
            row["R spread deg"] = max(
                (_rotation_angle_deg(latest, other) for _seq, other in valid_transforms),
                default=0.0,
            )
            row["T spread m"] = max(
                (_translation_distance_m(latest, other) for _seq, other in valid_transforms),
                default=0.0,
            )
        else:
            row["R,t pair"] = None
            row["R,t rows"] = 0
            for name in _RT_CELL_NAMES:
                row[name] = None
            row["R spread deg"] = None
            row["T spread m"] = None
        rows.append(row)
    return rows


SUMMARY_METHOD_COLUMNS: tuple[str, ...] = (
    "Method", "Sheet", "Attempted count",
    *(f"{status} count" for status in (*_STATUS_ORDER, STATUS_PENDING, STATUS_NOT_ATTEMPTED)),
    "Forward min", "Forward max", "Right min", "Right max", "Down min", "Down max", "Range min", "Range max",
    # Latest VALID R_right_from_left / T (right-camera RDF metres) per method, plus how far
    # the other VALID rows of this Measurement deviate from it.
    "R,t pair", "R,t rows", *_RT_CELL_NAMES, "R spread deg", "T spread m",
)


def _write_summary(workbook: xlsxwriter.Workbook, formats: Mapping[str, Any], snapshot: MeasurementSnapshot, *, write_utc_ns: int) -> None:
    sheet = _SheetWriter(workbook, "00_Summary", formats)
    detections = snapshot.detections
    sheet.key_values(
        (
            ("Workbook schema", WORKBOOK_SCHEMA),
            ("Measurement ID", snapshot.measurement_id),
            ("File prefix", snapshot.file_prefix),
            ("State", snapshot.state),
            ("Start local time", snapshot.started_local_iso),
            ("Start UTC ns", snapshot.started_utc_ns),
            ("Start monotonic ns", snapshot.started_monotonic_ns),
            ("Stop local time", snapshot.stopped_local_iso or None),
            ("Stop UTC ns", snapshot.stopped_utc_ns),
            ("Stop monotonic ns", snapshot.stopped_monotonic_ns),
            ("Duration s", snapshot.duration_s),
            ("Stereo pair count", len(snapshot.pairs)),
            ("Left detection rows", len(detections.get("left", ()))),
            ("Right detection rows", len(detections.get("right", ()))),
            ("Pending method jobs", sum(1 for row in snapshot.method_results if row.status == STATUS_PENDING)),
            ("Workbook written UTC", _kst_or_local_iso(write_utc_ns)),
            ("Workbook revision", snapshot.revision),
            ("Storage state", snapshot.storage.get("state")),
            ("Storage failures", snapshot.storage.get("failure_count")),
            ("Storage last error", snapshot.storage.get("last_error") or None),
        )
    )
    sheet.blank()
    sheet.marker(
        "Method summary (min/max over rows with numeric results only; R,t = latest VALID row's "
        "R_right_from_left and T in right-camera RDF metres, spread = max deviation of other VALID rows)"
    )
    header_row = sheet.header(SUMMARY_METHOD_COLUMNS)
    for index, row in enumerate(_summary_method_rows(snapshot)):
        target_row = header_row + 1 + index
        for column, name in enumerate(SUMMARY_METHOD_COLUMNS):
            value = row.get(name)
            if name == "Method":
                sheet.worksheet.write_url(target_row, column, f"internal:'{row['Sheet']}'!A1", string=str(value))
                sheet._note_width(column, str(value))
                continue
            sheet.cell(target_row, column, value, header=name)
        sheet.row = target_row + 1
    sheet.finish()


def _exact_rows(snapshot: MeasurementSnapshot) -> Iterable[dict[str, object]]:
    def emit(pair_id: str, method_id: str, field: str, value: object) -> dict[str, object] | None:
        if not isinstance(value, (int, float)) or isinstance(value, bool):
            return None
        number = float(value)
        if not math.isfinite(number):
            return None
        return {
            "measurement_id": snapshot.measurement_id,
            "pair_id": pair_id,
            "method_id": method_id,
            "field": field,
            "value": number,
            "value_repr": repr(number),
            "value_hex": number.hex(),
        }

    for row in _calibration_rows(snapshot):
        side = str(row["side"])
        for name in ("K11", "K12", "K13", "K21", "K22", "K23", "K31", "K32", "K33", *(f"D{index}" for index in range(1, 9))):
            item = emit("", "calibration", f"{side}.{name}", row.get(name))
            if item is not None:
                yield item
    for result in snapshot.method_results:
        derived = result.derived
        if not derived:
            continue
        cells = flat_transform_cells(derived)
        fields: list[tuple[str, object]] = list(cells.items())
        for side in ("left", "right"):
            uv = derived.get(f"{side}_uv")
            if uv is not None:
                fields.extend(((f"{side}.u", uv[0]), (f"{side}.v", uv[1])))
            ray = derived.get(f"{side}_ray")
            if ray is not None:
                fields.extend(((f"{side}.ray_R", ray[0]), (f"{side}.ray_D", ray[1]), (f"{side}.ray_F", ray[2])))
            quaternion = derived.get(f"{side}_quaternion_wxyz")
            if quaternion is not None:
                fields.extend((f"{side}.q_{name}", quaternion[index]) for index, name in enumerate("wxyz"))
            camera = derived.get(f"{side}_camera")
            if camera:
                matrix = camera["camera_matrix"]
                fields.extend((f"{side}.K{r + 1}{c + 1}", matrix[r][c]) for r in range(3) for c in range(3))
                fields.extend((f"{side}.D{index + 1}", value) for index, value in enumerate(camera["distortion_coefficients"]))
        fields.extend(
            (
                ("Forward", derived.get("forward_m")),
                ("Right", derived.get("right_m")),
                ("Down", derived.get("down_m")),
                ("Range", derived.get("range_m")),
                ("ray_gap_m", derived.get("ray_gap_m")),
                ("ray_angle_deg", derived.get("ray_angle_deg")),
                ("left_depth_m", derived.get("left_depth_m")),
                ("right_depth_m", derived.get("right_depth_m")),
            )
        )
        from Workstation.workstation_core.target_coordinates import TARGET_COORDINATE_FIELDS
        fields.extend((name, derived.get(name)) for name in (*TARGET_COORDINATE_FIELDS, *LEVEL_OUTPUT_FIELDS))
        level_evidence = derived.get("target_level_evidence") or {}
        fields.append(("target_level_evidence", json.dumps(level_evidence, ensure_ascii=False, sort_keys=True)))
        evidence = derived.get("target_coordinate_evidence") or {}
        for key in ("origin_wgs84", "target_from_left_ned_m", "source_point_left_rdf_m", "body_to_ned_quaternion_wxyz"):
            fields.extend((f"target.{key}.{i}", value) for i, value in enumerate(evidence.get(key, ())))
        for i, row in enumerate(evidence.get("rotation_ned_from_left_optical", ())):
            fields.extend((f"target.rotation.{i}.{j}", value) for j, value in enumerate(row))
        for field, value in fields:
            item = emit(result.pair_id, result.method_id, field, value)
            if item is not None:
                yield item


def _method_compare_label(method_id: str) -> str:
    name = METHOD_SHEET_NAMES.get(method_id, method_id)
    head, _, tail = str(name).partition("_")
    return tail if head.isdigit() and tail else str(name)


def _method_compare_table(snapshot: MeasurementSnapshot) -> tuple[tuple[str, ...], list[dict[str, object]]]:
    """One row per pair with every attempted method's F/R/D/Range side by side.

    The per-method sheets (``10_``..``19_``) already hold the full evidence;
    this sheet exists so two methods can be compared without a VLOOKUP.
    """

    attempted_methods = [
        method_id
        for method_id in MEASUREMENT_METHOD_IDS
        if any(row.method_id == method_id and row.attempted for row in snapshot.method_results)
    ]
    labels = {method_id: _method_compare_label(method_id) for method_id in attempted_methods}
    columns: list[str] = list(METHOD_COMPARE_FRONT_COLUMNS)
    for method_id in attempted_methods:
        columns.extend(f"{labels[method_id]}_{field}" for field in METHOD_COMPARE_FIELDS)
    if "pixhawk_rtk_live" in labels and "pixhawk_rp_gps_baseline_yaw" in labels:
        columns.append("yaw90_over_rtk_range_ratio")
    pair_by_id = {pair.pair_id: pair for pair in snapshot.pairs}
    by_pair: dict[str, dict[str, object]] = {}
    order: list[str] = []
    for row in snapshot.method_results:
        if row.method_id not in labels:
            continue
        cells = by_pair.get(row.pair_id)
        if cells is None:
            pair = pair_by_id.get(row.pair_id)
            cells = {
                "measurement_id": snapshot.measurement_id,
                "pair_id": row.pair_id,
                "pair_sequence": row.pair_sequence,
                "time_local": _kst_or_local_iso(None if pair is None else pair.created_unix_ns),
                "time_utc_ns": None if pair is None else pair.created_unix_ns,
                "time_delta_s": None if pair is None else pair.time_delta_s,
            }
            by_pair[row.pair_id] = cells
            order.append(row.pair_id)
        derived = row.derived or {}
        label = labels[row.method_id]
        cells[f"{label}_status"] = row.status
        for field in METHOD_COMPARE_FIELDS[1:]:
            cells[f"{label}_{field}"] = derived.get(field)
        outcome = row.outcome or {}
        result = outcome.get("result") if isinstance(outcome, Mapping) else None
        match = result.get("match") if isinstance(result, Mapping) else None
        if isinstance(match, Mapping) and cells.get("disparity_px") is None:
            cells["disparity_px"] = match.get("disparity_px")
    rows: list[dict[str, object]] = []
    for pair_id in order:
        cells = by_pair[pair_id]
        rtk = cells.get(f"{labels.get('pixhawk_rtk_live', '')}_range_m")
        yaw90 = cells.get(f"{labels.get('pixhawk_rp_gps_baseline_yaw', '')}_range_m")
        if "yaw90_over_rtk_range_ratio" in columns:
            try:
                cells["yaw90_over_rtk_range_ratio"] = (
                    None if not rtk or yaw90 is None else float(yaw90) / float(rtk)
                )
            except (TypeError, ValueError, ZeroDivisionError):
                cells["yaw90_over_rtk_range_ratio"] = None
        rows.append(cells)
    return tuple(columns), rows


def _optical_pose_rows(snapshot: MeasurementSnapshot) -> list[dict[str, object]]:
    """Accepted/created optical-pose suggestions and their relative rotation."""

    rows: list[dict[str, object]] = []

    def _from_suggestion(suggestion: Mapping[str, Any]) -> dict[str, object]:
        rotation = suggestion.get("relative_rotation")
        rotation = rotation if isinstance(rotation, Mapping) else {}
        frames = suggestion.get("source_frames")
        frames = frames if isinstance(frames, Mapping) else {}
        baseline = suggestion.get("gnss_baseline")
        baseline = baseline if isinstance(baseline, Mapping) else {}

        def _sha(side: str) -> object:
            value = frames.get(side)
            return value.get("image_sha256") if isinstance(value, Mapping) else None

        return {
            "mode": suggestion.get("mode"),
            "suggestion_id": suggestion.get("suggestion_id"),
            "state": suggestion.get("state"),
            "eligible": suggestion.get("eligible"),
            "inlier_count": suggestion.get("inlier_count"),
            "match_count": suggestion.get("match_count"),
            "residual_px": suggestion.get("residual_px"),
            "confidence": suggestion.get("confidence"),
            "relative_yaw_right_minus_left_deg": rotation.get("yaw_right_minus_left_deg"),
            "relative_pitch_right_minus_left_deg": rotation.get("pitch_right_minus_left_deg"),
            "relative_roll_right_minus_left_deg": rotation.get("roll_right_minus_left_deg"),
            "relative_fit_inlier_count": rotation.get("fit_inlier_count"),
            "relative_fit_residual_px": rotation.get("fit_residual_px"),
            "far_field_du_median_px": rotation.get("far_field_du_median_px"),
            "far_field_dv_median_px": rotation.get("far_field_dv_median_px"),
            "gnss_baseline_m": baseline.get("baseline_m"),
            "gnss_baseline_fix_quality": baseline.get("fix_quality"),
            "left_image_sha256": _sha("left"),
            "right_image_sha256": _sha("right"),
        }

    for raw in snapshot.alignment_rows:
        event_type = str(raw.get("event_type") or "")
        if not event_type.startswith("optical_pose_auto_suggestion"):
            continue
        utc_ns = raw.get("updated_utc_ns") or raw.get("recorded_utc_ns")
        row: dict[str, object] = {
            "measurement_id": snapshot.measurement_id,
            "record_source": "alignment",
            "event_type": event_type,
            "utc_ns": utc_ns,
            "utc_local": _kst_or_local_iso(utc_ns if isinstance(utc_ns, int) else None),
            "suggestion_id": raw.get("source"),
        }
        row.update({name: raw.get(name) for name in OPTICAL_POSE_COLUMNS if name in raw and name not in row})
        rows.append(row)
    for event in snapshot.events:
        if not str(event.event_type).startswith("optical_pose_auto_suggestion"):
            continue
        payload = event.payload if isinstance(event.payload, Mapping) else {}
        suggestion = payload.get("suggestion")
        source = suggestion if isinstance(suggestion, Mapping) else payload
        row = {
            "measurement_id": snapshot.measurement_id,
            "record_source": "event",
            "event_type": event.event_type,
            "utc_ns": event.utc_ns,
            "utc_local": _kst_or_local_iso(event.utc_ns),
        }
        row.update(_from_suggestion(source))
        if not row.get("mode"):
            row["mode"] = payload.get("mode")
        if not row.get("suggestion_id"):
            row["suggestion_id"] = payload.get("suggestion_id")
        rows.append(row)
    rows.sort(key=lambda item: (item.get("utc_ns") or 0))
    return rows


def _flat_rows(snapshot: MeasurementSnapshot) -> Iterable[dict[str, object]]:
    pair_by_id = {pair.pair_id: pair for pair in snapshot.pairs}
    for row in snapshot.method_results:
        cells = _section_a_row(snapshot, pair_by_id, row)
        cells.update(_section_b_row(pair_by_id, row))
        cells.update({key: value for key, value in _section_c_row(row).items() if key.endswith(("_u", "_v"))})
        cells.update(_section_e_row(row))
        cells["method_id"] = row.method_id
        yield cells


def _git_revision() -> str:
    try:
        result = subprocess.run(
            ["git", "rev-parse", "HEAD"],
            cwd=Path(__file__).resolve().parent,
            capture_output=True,
            text=True,
            timeout=2.0,
            check=False,
        )
    except (OSError, subprocess.SubprocessError):
        return ""
    return result.stdout.strip() if result.returncode == 0 else ""


def _library_versions() -> dict[str, str]:
    versions = {"python": sys.version.split()[0], "xlsxwriter": getattr(xlsxwriter, "__version__", "")}
    for name in ("numpy", "cv2"):
        try:
            module = __import__(name)
            versions[name] = str(getattr(module, "__version__", ""))
        except Exception:  # noqa: BLE001 - optional at write time.
            versions[name] = "unavailable"
    return versions


def _provenance_rows(snapshot: MeasurementSnapshot, *, write_utc_ns: int) -> list[dict[str, object]]:
    rows: list[dict[str, object]] = [
        {"key": "workbook_schema", "value": WORKBOOK_SCHEMA},
        {"key": "measurement_id", "value": snapshot.measurement_id},
        {"key": "file_prefix", "value": snapshot.file_prefix},
        {"key": "workbook_path", "value": str(snapshot.workbook_path or "")},
        {"key": "workbook_written_utc_ns", "value": write_utc_ns},
        {"key": "workbook_revision", "value": snapshot.revision},
        {"key": "git_revision", "value": _git_revision() or None},
        {"key": "hostname", "value": platform.node()},
        {"key": "platform", "value": platform.platform()},
    ]
    rows.extend({"key": f"library.{name}", "value": version} for name, version in _library_versions().items())
    rows.extend({"key": f"metadata.{key}", "value": _compact_json(value) if isinstance(value, (dict, list, tuple)) else value} for key, value in sorted(snapshot.metadata.items()))
    for artifact in snapshot.artifacts:
        rows.append(
            {
                "key": f"artifact.{artifact.get('event_type', 'artifact')}",
                "value": _compact_json({key: value for key, value in artifact.items() if key not in {"schema"}}),
            }
        )
    for name, value in sorted(snapshot.storage.items()):
        rows.append({"key": f"storage.{name}", "value": value})
    return rows


def write_measurement_workbook(snapshot: MeasurementSnapshot, path: Path, *, write_utc_ns: int | None = None) -> Path:
    """Write a complete workbook to a temp file and atomically replace ``path``."""

    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    write_utc_ns = int(time.time_ns() if write_utc_ns is None else write_utc_ns)
    temporary = path.with_name(f".{path.name}.tmp-{os.getpid()}-{write_utc_ns}")
    try:
        workbook = xlsxwriter.Workbook(
            str(temporary),
            {"constant_memory": True, "strings_to_formulas": False, "strings_to_urls": False, "nan_inf_to_errors": False},
        )
        workbook.set_properties({"title": f"Measurement {snapshot.file_prefix}", "subject": WORKBOOK_SCHEMA, "comments": snapshot.measurement_id})
        formats = _formats(workbook)
        report_info = write_front_sheets(workbook, formats, snapshot, _SheetWriter)
        for method_id in MEASUREMENT_METHOD_IDS:
            _write_method_sheet(workbook, formats, snapshot, method_id, rt_links=report_info["rt_locations"])
        for camera, name in (("left", "01_Left_Detections"), ("right", "02_Right_Detections")):
            sheet = _SheetWriter(workbook, name, formats)
            sheet.table(
                DETECTION_COLUMNS,
                (
                    {
                        "measurement_id": snapshot.measurement_id,
                        "pair_id": row.pair_id or None,
                        "pair_sequence": row.pair_sequence,
                        "frame_id": row.frame_id,
                        "frame_key": row.frame_key,
                        "detection_id": row.detection_id,
                        "selected": row.selected,
                        "timestamp_s": row.timestamp_s,
                        "app_receive_monotonic_ns": row.app_receive_monotonic_ns,
                        "x1": row.x1, "y1": row.y1, "x2": row.x2, "y2": row.y2,
                        "confidence": row.confidence,
                        "class_id": row.class_id,
                        "class_name": row.class_name,
                        "image_width": row.image_width,
                        "image_height": row.image_height,
                        "stream_generation": row.stream_generation,
                        "rtcp_frame_ntp_unix_s": row.rtcp_frame_ntp_unix_s,
                    }
                    for row in snapshot.detections.get(camera, ())
                ),
            )
            sheet.finish()
        sheet = _SheetWriter(workbook, "03_Calibration", formats)
        sheet.table(CALIBRATION_COLUMNS, _calibration_rows(snapshot))
        sheet.finish()
        sheet = _SheetWriter(workbook, "04_Alignment", formats)
        sheet.table(ALIGNMENT_COLUMNS, _alignment_rows(snapshot))
        sheet.finish()
        sheet = _SheetWriter(workbook, "06_Optical_Pose", formats)
        sheet.table(OPTICAL_POSE_COLUMNS, _optical_pose_rows(snapshot))
        sheet.finish()
        telemetry_names = sorted(set(ALWAYS_PRESENT_TELEMETRY_SHEETS) | set(snapshot.telemetry))
        for name in telemetry_names:
            _write_telemetry_sheet(workbook, formats, name, snapshot.telemetry.get(name, ()))
        sheet = _SheetWriter(workbook, "50_Events", formats)
        sheet.table(
            EVENT_COLUMNS,
            (
                {
                    "measurement_id": snapshot.measurement_id,
                    "event_type": event.event_type,
                    "utc_ns": event.utc_ns,
                    "utc_local": _kst_or_local_iso(event.utc_ns),
                    "monotonic_ns": event.monotonic_ns,
                    "detail": event.detail or None,
                    "payload": _compact_json(event.payload) if event.payload else None,
                }
                for event in snapshot.events
            ),
        )
        sheet.finish()
        sheet = _SheetWriter(workbook, "90_Flat", formats)
        sheet.table(FLAT_COLUMNS, _flat_rows(snapshot))
        sheet.finish()
        for worksheet in sheet.sheets:
            worksheet.hide()
        sheet = _SheetWriter(workbook, "91_Exact", formats)
        sheet.table(EXACT_COLUMNS, _exact_rows(snapshot))
        sheet.finish()
        for worksheet in sheet.sheets:
            worksheet.hide()
        sheet = _SheetWriter(workbook, "98_Provenance", formats)
        sheet.table(("key", "value"), _provenance_rows(snapshot, write_utc_ns=write_utc_ns), autofilter=False)
        append_trend_chart(workbook, sheet, snapshot, report_info)
        sheet.finish()
        for worksheet in sheet.sheets:
            worksheet.hide()
        sheet = _SheetWriter(workbook, "99_Definitions", formats)
        sheet.table(("term", "definition"), ({"term": term, "definition": text} for term, text in DEFINITIONS), autofilter=False)
        sheet.finish()
        sheet.worksheet.set_column(0, 0, 40)
        sheet.worksheet.set_column(1, 1, 85)
        workbook.close()
        with open(temporary, "rb") as handle:
            os.fsync(handle.fileno())
        os.replace(temporary, path)
        _fsync_directory(path.parent)
        return path
    except BaseException:
        try:
            temporary.unlink()
        except OSError:
            pass
        raise


class MeasurementWorkbookWriter:
    """Single background thread that writes coalesced workbook snapshots.

    Requests never block the caller. While a Measurement is RUNNING the
    workbook is rewritten at most every ``checkpoint_interval_s``; a Stop
    and every outcome arriving after Stop rewrite as soon as the worker is
    free. Failures are reported into the record and the previous good file
    is left untouched.
    """

    def __init__(self, *, checkpoint_interval_s: float = 30.0, writer=write_measurement_workbook, source_storage_reader=None) -> None:
        self._checkpoint_interval_s = max(0.0, float(checkpoint_interval_s))
        self._write = writer
        self._source_storage_reader = source_storage_reader
        self._condition = threading.Condition()
        self._pending: dict[str, tuple[MeasurementRecord, str]] = {}
        self._tracked: dict[str, MeasurementRecord] = {}
        self._last_write_monotonic: dict[str, float] = {}
        self._closed = False
        self._state = "IDLE"
        self._last_error = ""
        self._last_path = ""
        self._last_write_utc_ns: int | None = None
        self._write_count = 0
        self._failure_count = 0
        self._thread = threading.Thread(target=self._run, name="measurement-workbook-writer", daemon=True)
        self._thread.start()

    def request(self, record: MeasurementRecord, *, reason: str = "") -> None:
        with self._condition:
            if self._closed:
                return
            self._tracked[record.measurement_id] = record
            self._pending[record.measurement_id] = (record, reason)
            self._condition.notify_all()

    def write_now(self, record: MeasurementRecord, *, reason: str = "", timeout_s: float = 5.0) -> bool:
        """Synchronous write used at shutdown; falls back to inline writing."""

        if not self._thread.is_alive():
            return self._write_record(record, reason)
        with self._condition:
            self._pending[record.measurement_id] = (record, f"{reason}:now")
            self._condition.notify_all()
        deadline = time.monotonic() + max(0.0, float(timeout_s))
        while time.monotonic() < deadline:
            with self._condition:
                if record.measurement_id not in self._pending and record.written_revision >= record.revision:
                    return record.storage_snapshot().get("state") == "READY"
                self._condition.wait(timeout=0.05)
        return False

    def snapshot(self) -> dict[str, Any]:
        with self._condition:
            return {
                "state": self._state,
                "last_path": self._last_path,
                "last_write_utc_ns": self._last_write_utc_ns,
                "last_error": self._last_error,
                "write_count": self._write_count,
                "failure_count": self._failure_count,
                "pending": sorted(self._pending),
                "writer_alive": self._thread.is_alive(),
            }

    def close(self, *, timeout_s: float = 5.0) -> bool:
        with self._condition:
            self._closed = True
            self._condition.notify_all()
        self._thread.join(timeout=max(0.0, float(timeout_s)))
        return not self._thread.is_alive()

    def _due(self, record: MeasurementRecord, reason: str, now: float) -> bool:
        if reason.endswith(":now") or reason in {"start", "stop", "shutdown"}:
            return True
        if not record.running:
            return True
        last = self._last_write_monotonic.get(record.measurement_id)
        return last is None or now - last >= self._checkpoint_interval_s

    def _run(self) -> None:
        while True:
            with self._condition:
                while not self._pending and not self._closed:
                    self._condition.wait(timeout=1.0)
                    self._enqueue_checkpoints_locked()
                if self._closed and not self._pending:
                    return
                now = time.monotonic()
                ready = [key for key, (record, reason) in self._pending.items() if self._due(record, reason, now)]
                if not ready:
                    self._condition.wait(timeout=0.5)
                    continue
                key = ready[0]
                record, reason = self._pending.pop(key)
                self._state = "WRITING"
            self._write_record(record, reason)
            with self._condition:
                self._condition.notify_all()

    def _enqueue_checkpoints_locked(self) -> None:
        """Queue a periodic rewrite for tracked records that changed since their last write."""

        now = time.monotonic()
        for measurement_id, record in tuple(self._tracked.items()):
            if measurement_id in self._pending:
                continue
            if record.revision <= record.written_revision:
                if record.stopped and record.pending_method_count() == 0:
                    self._tracked.pop(measurement_id, None)
                continue
            last = self._last_write_monotonic.get(measurement_id)
            if record.running and last is not None and now - last < self._checkpoint_interval_s:
                continue
            self._pending[measurement_id] = (record, "checkpoint")

    def _write_record(self, record: MeasurementRecord, reason: str) -> bool:
        path = record.workbook_path
        if path is None:
            record.note_storage_result(ok=False, error="workbook path unavailable")
            with self._condition:
                self._state = "FAILED"
                self._last_error = "workbook path unavailable"
                self._failure_count += 1
            return False
        snapshot = record.snapshot()
        if self._source_storage_reader is not None:
            try:
                raw = self._source_storage_reader(record.measurement_id)
            except Exception as exc:
                raw = {"state": "UNAVAILABLE", "last_error": str(exc)}
            snapshot = replace(snapshot, storage={**snapshot.storage, "raw": raw})
        write_utc_ns = time.time_ns()
        try:
            self._write(snapshot, path, write_utc_ns=write_utc_ns)
        except Exception as exc:  # noqa: BLE001 - failures become evidence, never exceptions.
            error = f"{type(exc).__name__}: {exc}"
            record.note_storage_result(ok=False, error=error)
            with self._condition:
                self._state = "FAILED"
                self._last_error = error
                self._failure_count += 1
                self._last_write_monotonic[record.measurement_id] = time.monotonic()
            return False
        record.note_storage_result(ok=True, utc_ns=write_utc_ns)
        record.note_written_revision(snapshot.revision)
        with self._condition:
            self._state = "READY"
            self._last_error = ""
            self._last_path = str(path)
            self._last_write_utc_ns = write_utc_ns
            self._write_count += 1
            self._last_write_monotonic[record.measurement_id] = time.monotonic()
            if record.revision > snapshot.revision and record.measurement_id not in self._pending:
                # Evidence arrived while writing; keep the file current.
                self._pending[record.measurement_id] = (record, reason if not record.running else "checkpoint")
        return True


__all__ = [
    "DEFINITIONS",
    "FLAT_COLUMNS",
    "MAX_SHEET_ROWS",
    "NA",
    "SECTION_A",
    "SECTION_A_COLUMNS",
    "SECTION_B",
    "SECTION_C",
    "SECTION_D",
    "SECTION_E",
    "WORKBOOK_SCHEMA",
    "MeasurementWorkbookWriter",
    "write_measurement_workbook",
]
