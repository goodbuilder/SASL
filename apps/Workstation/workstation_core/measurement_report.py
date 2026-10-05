"""Reader-facing Measurement tables; all projections share frozen Method rows."""

from __future__ import annotations

from Workstation.workstation_core.target_level_coordinates import LEVEL_OUTPUT_FIELDS

import math
from datetime import datetime
from statistics import median
from typing import Any, Mapping

from Workstation.workstation_core.measurement_methods import MEASUREMENT_METHOD_IDS, MEASUREMENT_METHOD_LABELS
from Workstation.workstation_core.measurement_record import METHOD_SHEET_NAMES, MeasurementSnapshot, MethodResultRow
from Workstation.workstation_core.method_result_evidence import flat_transform_cells
from Workstation.workstation_core.measurement_artifacts import KST

SUMMARY = "00_요약"
COMPARE = "01_거리비교"
COORDINATES = "02_좌표결과"
DIAGNOSTICS = "03_진단"
RT = "04_Rt"
GRAPHS = "05_그래프"
FRONT_SHEETS = (SUMMARY, COMPARE, COORDINATES, DIAGNOSTICS, RT)
# One plot per result field; names/units intentionally distinguish the frames.
RESULT_CHART_FIELDS = (
    ("forward_m", "카메라 FRD Forward", "m"),
    ("right_m", "카메라 FRD Right", "m"),
    ("down_m", "카메라 FRD Down", "m"),
    ("range_m", "카메라 FRD Range", "m"),
    ("target_level_forward_m", "수평 FRD Forward", "m"),
    ("target_level_right_m", "수평 FRD Right", "m"),
    ("target_level_down_m", "수평 FRD Down", "m"),
    ("target_level_range_m", "수평 FRD Range", "m"),
    ("target_latitude_deg", "WGS84 위도", "deg"),
    ("target_longitude_deg", "WGS84 경도", "deg"),
    ("target_altitude_ellipsoid_m", "Target 타원체고", "m"),
    ("target_altitude_msl_m", "Target MSL 고도", "m"),
)
SHORT_METHODS = {
    "pixhawk_rtk_live": "Pixhawk RTK", "pixhawk_rp_gps_baseline_yaw": "GPS Yaw 90°",
    "checkerboard_stereo": "Checkerboard", "horizon_hybrid": "Horizon", "mountain_gnss": "Mountain",
    "drone_rtk_rotation_hybrid": "Drone Rot", "drone_rtk_multiview": "Drone MV",
    "terrain_pnp": "Terrain PnP", "combined_hybrid": "Combined", "cloud_gnss": "Cloud",
}
STATUS_LABELS = {
    "VALID": "정상", "ESTIMATED": "추정", "BLOCKED": "계산 불가",
    "FAILED": "실패", "BACKPRESSURE_BLOCKED": "처리 용량 초과",
    "PENDING": "계산 대기", "N/A": "미시도",
}
RUN_LABELS = {"RUNNING": "측정 중", "STOPPED_DRAINING": "측정 종료 · 계산 대기", "STOPPED": "측정 종료", "INTERRUPTED": "중단된 기록"}
RT_FIELDS = tuple(f"R{r}{c}" for r in (1, 2, 3) for c in (1, 2, 3)) + ("T_R", "T_D", "T_F")


def finite(value: object) -> bool:
    return isinstance(value, (int, float)) and not isinstance(value, bool) and math.isfinite(value)


def rt_id(row: MethodResultRow) -> str:
    if not row.attempted:
        return ""
    method_number = MEASUREMENT_METHOD_IDS.index(row.method_id) + 1
    return f"RT-{row.pair_sequence:06d}-{method_number:02d}"


def rt_sources(row: MethodResultRow) -> tuple[str, str]:
    attempt = (row.outcome or {}).get("attempt_payload") or {}
    sources = dict((row.evidence or {}).get("component_sources") or {})
    sources.update(dict(attempt.get("measurement_component_sources") or ()))
    rotation = sorted({str(v) for k, v in sources.items() if any(axis in str(k) for axis in ("roll", "pitch", "yaw", "rotation"))})
    translation = sorted({str(v) for k, v in sources.items() if any(axis in str(k) for axis in ("north", "east", "down", "translation"))})
    if attempt.get("triangulation_source") == "checkerboard":
        fallback = str(attempt.get("measurement_method_instance_id") or (row.outcome or {}).get("method_instance_id") or "checkerboard profile")
        return ", ".join(rotation) or fallback, ", ".join(translation) or fallback
    baseline = attempt.get("baseline") or {}
    return (
        ", ".join(rotation) or str(attempt.get("measurement_yaw_source") or attempt.get("triangulation_source") or "—"),
        ", ".join(translation) or str(baseline.get("source") or "—"),
    )


def attempted_methods(snapshot: MeasurementSnapshot) -> list[str]:
    used = {row.method_id for row in snapshot.method_results if row.attempted}
    return [method for method in MEASUREMENT_METHOD_IDS if method in used]


def transform_summary(snapshot: MeasurementSnapshot, method: str) -> dict[str, object]:
    results = [r for r in snapshot.method_results if r.method_id == method and r.attempted]
    cells = [flat_transform_cells(r.derived or {}) for r in results]
    def change(fields):
        values = [tuple(c.get(k) for k in fields) for c in cells if all(finite(c.get(k)) for k in fields)]
        return "미확보" if not values else "변화" if any(v != values[0] for v in values[1:]) else "고정"
    return {
        "Method": SHORT_METHODS.get(method, method),
        "계산 시도": len(results), "R": change(RT_FIELDS[:9]), "t": change(RT_FIELDS[9:]),
        "R,t 확보": sum(all(finite(c.get(k)) for k in RT_FIELDS) for c in cells),
    }


def _elapsed(snapshot: MeasurementSnapshot, pair) -> float | None:
    return None if pair is None else (pair.created_monotonic_ns - snapshot.started_monotonic_ns) / 1e9


def _title(sheet, title: str, snapshot: MeasurementSnapshot, note: str = "") -> None:
    sheet.marker(title)
    sheet.marker(f"측정 시작 {_display_time(snapshot.started_local_iso)} · {RUN_LABELS.get(snapshot.state, snapshot.state)}")
    sheet.marker(note or "† 추정 결과 · — 값 없음")
    sheet.blank()


def _display_time(text: str) -> str:
    try:
        value = datetime.fromisoformat(text).astimezone(KST)
        return value.strftime("%Y-%m-%d %H:%M:%S.%f")[:-3] + " KST"
    except (ValueError, TypeError):
        return text or "—"


def _finish(sheet, *, widths: Mapping[int, float] | None = None, header: int = 4, freeze_columns: int = 2) -> None:
    sheet.finish()
    for ws in sheet.sheets:
        local_header = sheet.table_headers.get(ws, header)
        end = sheet.end_rows.get(ws, sheet.row - 1)
        ws.hide_gridlines(2)
        ws.set_zoom(100)
        ws.set_landscape()
        if sheet._base_name in {SUMMARY, COMPARE}:
            ws.fit_to_pages(1, 0)
        else:
            ws.repeat_columns(0, max(0, freeze_columns - 1))
        ws.repeat_rows(local_header)
        ws.freeze_panes(local_header + 1, freeze_columns)
        ws.set_margins(0.3, 0.3, 0.4, 0.4)
        for col, width in (widths or {}).items():
            ws.set_column(col, col, width)
        if end > local_header:
            ws.conditional_format(local_header + 1, 0, end, max(0, len(sheet.widths) - 1), {
                "type": "formula", "criteria": "=MOD(ROW(),2)=0", "format": sheet.formats["band"],
            })


def rt_locations(snapshot: MeasurementSnapshot, max_row: int = 1_048_575) -> dict[str, tuple[str, int]]:
    # Four title rows, summary header+rows, blank, section title, history header.
    start = 8 + len(attempted_methods(snapshot))
    result = {}
    first_capacity = max_row - start + 1
    for index, row in enumerate(r for r in snapshot.method_results if r.attempted):
        if index < first_capacity:
            location = RT, start + index
        else:
            offset = index - first_capacity
            part = 1 + offset // max_row
            location = f"{RT[:26]}_{part:03d}", 1 + offset % max_row
        result[rt_id(row)] = location
    return result


def result_cells(snapshot: MeasurementSnapshot, row: MethodResultRow, pair) -> dict[str, object]:
    d = row.derived or {}
    return {
        "영상 쌍": row.pair_sequence, "경과시간 (s)": _elapsed(snapshot, pair),
        "Method": SHORT_METHODS.get(row.method_id, row.method_id),
        "거리 (m)": d.get("range_m"), "전방 (m)": d.get("forward_m"),
        "우측 (m)": d.get("right_m"), "하방 (m)": d.get("down_m"),
        **{name: d.get(name) for name in ("target_latitude_deg", "target_longitude_deg", "target_altitude_ellipsoid_m", "target_altitude_msl_m", *LEVEL_OUTPUT_FIELDS)},
        "상태": STATUS_LABELS.get(row.status, row.status), "사유": row.reason or None,
        "R,t 번호": rt_id(row) or None,
    }


def write_front_sheets(workbook, formats, snapshot: MeasurementSnapshot, writer_type):
    pairs = {pair.pair_id: pair for pair in snapshot.pairs}
    summary = writer_type(workbook, SUMMARY, formats)
    summary.rt_links = {SHORT_METHODS[m]: (METHOD_SHEET_NAMES[m], 0) for m in MEASUREMENT_METHOD_IDS}
    _title(summary, "Measurement 결과", snapshot, "좌표: 스테레오 중심 FRD · 거리와 좌표 단위: m")
    summary.key_values((
        ("종료 시각", _display_time(snapshot.stopped_local_iso) if snapshot.stopped_local_iso else "측정 중" if snapshot.state == "RUNNING" else "—"),
        ("측정 시간 (s)", snapshot.duration_s), ("영상 쌍 수", len(snapshot.pairs)),
        ("계산 대기", sum(r.status == "PENDING" for r in snapshot.method_results)),
        ("원본 저장 상태", (snapshot.storage.get("raw") or {}).get("state") or (snapshot.storage.get("state") if "raw_read_errors" in snapshot.storage else "확인 정보 없음")),
    ))
    summary.blank()
    header = summary.row
    columns = ("Method", "시도", "정상", "추정", "미산출", "미시도", "중앙값 (m)", "최소 (m)", "최대 (m)")
    rows = []
    for method in MEASUREMENT_METHOD_IDS:
        items = [r for r in snapshot.method_results if r.method_id == method]
        ranges = [r.derived.get("range_m") for r in items if r.status in {"VALID", "ESTIMATED"} and r.derived and finite(r.derived.get("range_m"))]
        rows.append({
            "Method": SHORT_METHODS[method], "시도": sum(r.attempted for r in items),
            "정상": sum(r.status == "VALID" for r in items), "추정": sum(r.status == "ESTIMATED" for r in items),
            "미산출": sum(r.attempted and r.status not in {"VALID", "ESTIMATED"} for r in items),
            "미시도": sum(not r.attempted for r in items),
            "중앙값 (m)": median(ranges) if ranges else None,
            "최소 (m)": min(ranges) if ranges else None, "최대 (m)": max(ranges) if ranges else None,
        })
    summary.table(columns, rows)
    summary.blank()
    summary.marker("거리 통계: 정상·추정 결과의 유한한 거리 값. 상태별 건수는 별도 집계.")
    chart_row = summary.row + 1
    summary.worksheet.write_url(summary.row, 0, f"internal:'{GRAPHS}'!A1", formats["link"], string="12개 좌표·거리 그래프 보기")
    _finish(summary, widths={0: 19, **{i: 10 for i in range(1, 6)}, **{i: 15 for i in range(6, 9)}}, header=header, freeze_columns=0)
    summary.worksheet.freeze_panes(0, 0)

    compare = writer_type(workbook, COMPARE, formats)
    _title(compare, "방법별 거리 비교", snapshot, "같은 영상 쌍 기준 · 거리 단위 m · † 추정 결과 · — 값 없음")
    methods = attempted_methods(snapshot)
    columns = ("영상 쌍", "경과시간 (s)", *(SHORT_METHODS[m] for m in methods))
    by_pair = {(r.pair_id, r.method_id): r for r in snapshot.method_results}
    compare.header(columns)
    for pair in snapshot.pairs:
        compare.data_row(columns)
        compare.cell(compare.row, 0, pair.pair_sequence)
        compare.cell(compare.row, 1, _elapsed(snapshot, pair), header="경과시간 (s)")
        for index, method in enumerate(methods, 2):
            result = by_pair.get((pair.pair_id, method))
            value = None if result is None else (result.derived or {}).get("range_m")
            if result is not None and result.status in {"VALID", "ESTIMATED"} and finite(value):
                compare.worksheet.write_number(compare.row, index, value, formats["estimated_metre"] if result.status == "ESTIMATED" else formats["metre"])
            else:
                compare.cell(compare.row, index, None)
        compare.row += 1
    compare_header = compare.table_headers[compare.worksheet]
    compare.worksheet.autofilter(compare_header, 0, max(compare_header, compare.row - 1), max(1, len(columns) - 1))
    _finish(compare, widths={0: 10, 1: 16, **{i: 14 for i in range(2, len(columns))}})

    coordinate = writer_type(workbook, COORDINATES, formats)
    coordinate.rt_links = rt_locations(snapshot, coordinate.max_row)
    _title(coordinate, "방법별 좌표 결과", snapshot, "스테레오 중심 FRD: 전방·우측·하방 · 단위 m")
    columns = ("영상 쌍", "경과시간 (s)", "Method", "거리 (m)", "전방 (m)", "우측 (m)", "하방 (m)", "상태", "사유", "R,t 번호")
    columns += ("target_latitude_deg", "target_longitude_deg", "target_altitude_ellipsoid_m", "target_altitude_msl_m", *LEVEL_OUTPUT_FIELDS)
    coordinate.table(columns, (result_cells(snapshot, r, pairs.get(r.pair_id)) for r in snapshot.method_results))
    _finish(coordinate, widths={0: 10, 1: 16, 2: 18, **{i: 13 for i in range(3, 7)}, 7: 16, 8: 38, 9: 20})

    diagnostic = writer_type(workbook, DIAGNOSTICS, formats)
    diagnostic.rt_links = rt_locations(snapshot, diagnostic.max_row)
    _title(diagnostic, "측정 진단", snapshot, "계산 결과의 상태, 입력 경고, 저장 오류를 확인합니다.")
    issues = []
    for r in snapshot.method_results:
        warnings = (r.derived or {}).get("warnings") or ()
        if r.status != "VALID" or warnings:
            issues.append({**result_cells(snapshot, r, pairs.get(r.pair_id)), "경고": "; ".join(map(str, warnings)),
                "광선 간격 (m)": (r.derived or {}).get("ray_gap_m"), "광선 각도 (deg)": (r.derived or {}).get("ray_angle_deg")})
    for event in snapshot.events:
        if any(token in event.event_type.upper() for token in ("FAIL", "ERROR", "DROP", "INCOMPLETE")):
            issues.append({"상태": event.event_type, "사유": event.detail})
    if snapshot.storage.get("last_error"):
        issues.append({"상태": "저장 오류", "사유": snapshot.storage["last_error"]})
    raw = snapshot.storage.get("raw") or {}
    if raw.get("last_error"):
        issues.append({"상태": "원본 저장 오류", "사유": raw["last_error"]})
    if raw.get("pending_records"):
        issues.append({"상태": "원본 저장 대기", "사유": f"{raw['pending_records']}개 기록 저장 대기"})
    for stream, state in raw.get("streams", {}).items():
        for label, value in (("JSONL", state), ("CSV", state.get("csv") or {})):
            if value.get("state") == "FAILED" or value.get("dropped_records") or value.get("unmirrored_records"):
                issues.append({"Method": stream, "상태": f"{label} 저장 문제",
                    "사유": value.get("last_error") or f"누락 {value.get('dropped_records', 0)}, 미반영 {value.get('unmirrored_records', 0)}"})
    diagnostic.table(("영상 쌍", "경과시간 (s)", "Method", "상태", "사유", "경고", "광선 간격 (m)", "광선 각도 (deg)", "R,t 번호"), issues)
    _finish(diagnostic, widths={0: 10, 1: 16, 2: 18, 3: 18, 4: 45, 5: 45, 6: 17, 7: 18, 8: 20})

    rt = writer_type(workbook, RT, formats)
    _title(rt, "계산에 사용된 R,t", snapshot, "X_right = R × X_left + t · 카메라 optical RDF · t 단위 m")
    rt.table(("Method", "계산 시도", "R", "t", "R,t 확보"), (transform_summary(snapshot, m) for m in methods), autofilter=False)
    rt.blank()
    rt.marker("영상 쌍·Method별 기록 · R과 t는 확보된 값만 표시 · 결과 상태와 별도")
    rt_header = rt.row
    columns = ("R,t 번호", "영상 쌍", "경과시간 (s)", "Method", *RT_FIELDS, "R 출처", "t 출처", "결과 상태")
    rt.table(columns, (
        {"R,t 번호": rt_id(r), "영상 쌍": r.pair_sequence, "경과시간 (s)": _elapsed(snapshot, pairs.get(r.pair_id)),
         "Method": SHORT_METHODS.get(r.method_id, r.method_id), **flat_transform_cells(r.derived or {}),
         "R 출처": rt_sources(r)[0], "t 출처": rt_sources(r)[1], "결과 상태": STATUS_LABELS.get(r.status, r.status)}
        for r in snapshot.method_results if r.attempted
    ))
    _finish(rt, widths={0: 21, 1: 10, 2: 16, 3: 18, **{i: 16 for i in range(4, 16)}, 16: 34, 17: 34, 18: 18}, header=rt_header)
    graphs = workbook.add_worksheet(GRAPHS)
    graphs.hide_gridlines(2)
    graphs.set_zoom(85)
    graphs.set_default_row(18)
    graphs.set_column(0, 15, 9)
    graphs.merge_range(0, 0, 0, 15, "좌표·거리 추세 · 12개 결과 필드", formats["section"])
    graphs.merge_range(1, 0, 1, 15, "X축: 측정 시작 후 경과시간(s) · 선: Method · 정상·추정 결과 · 결측 구간은 빈칸", formats["body"])
    graphs.merge_range(2, 0, 2, 15, "카메라 FRD 4개 / 수평 FRD 4개 / WGS84 4개 · 위경도 deg, 거리·고도 m", formats["body"])
    graphs.freeze_panes(3, 0)
    graphs.set_landscape()
    graphs.set_paper(9)
    graphs.fit_to_pages(1, 0)
    graphs.repeat_rows(0, 2)
    graphs.set_margins(.3, .3, .4, .4)
    graphs.print_area(0, 0, 87, 15)
    graphs.set_h_pagebreaks([32, 60])
    return {"summary": summary.sheets[0], "chart_row": chart_row, "graphs": graphs,
            "rt_locations": rt_locations(snapshot, rt.max_row)}


def _chart_points(points, limit=2000):
    if len(points) <= limit:
        return points
    size = math.ceil(len(points) / max(1, limit // 5))
    selected = []
    for start in range(0, len(points), size):
        end = min(len(points), start + size)
        indexes = {start, end - 1}
        valid = [i for i in range(start, end) if finite(points[i][1])]
        if valid:
            indexes.update((min(valid, key=lambda i: points[i][1]), max(valid, key=lambda i: points[i][1])))
        missing = next((i for i in range(start, end) if not finite(points[i][1])), None)
        if missing is not None:
            indexes.add(missing)
        selected.extend(points[i] for i in sorted(indexes))
    return selected


def append_trend_chart(workbook, provenance, snapshot, report_info):
    """Twelve native XY charts backed by frozen results, including empty slots."""
    pairs = {p.pair_id: p for p in snapshot.pairs}
    methods = attempted_methods(snapshot)
    by_method = {method: [] for method in methods}
    for row in snapshot.method_results:
        if row.method_id in by_method:
            by_method[row.method_id].append(row)
    for field_index, (field, title, unit) in enumerate(RESULT_CHART_FIELDS):
        chart = workbook.add_chart({"type": "scatter", "subtype": "straight"})
        series_count = 0
        shortened = False
        for method in methods:
            rows = by_method[method]
            points = [(_elapsed(snapshot, pairs.get(r.pair_id)), (r.derived or {}).get(field)
                       if r.status in {"VALID", "ESTIMATED"} else None) for r in rows]
            points = [(x, y) for x, y in points if finite(x)]
            if not any(finite(y) for _, y in points):
                continue
            sampled = _chart_points(points, min(2000, provenance.max_row - 3))
            shortened |= len(sampled) < len(points)
            provenance.ensure_room(len(sampled) + 2)
            provenance.blank()
            provenance.marker(SHORT_METHODS[method] + f" chart values: elapsed_s, {field}")
            start = provenance.row
            for x, y in sampled:
                provenance.worksheet.write_number(provenance.row, 0, x)
                if finite(y):
                    provenance.worksheet.write_number(provenance.row, 1, y)
                else:
                    provenance.worksheet.write_blank(provenance.row, 1, None)
                provenance.row += 1
            # Stable method identity across all fields, even when other methods
            # have no geographic/level inputs in one of the charts.
            style = MEASUREMENT_METHOD_IDS.index(method)
            color = ("#333333", "#A02828", "#777777")[style % 3]
            chart.add_series({
                "name": SHORT_METHODS[method],
                "categories": [provenance.name, start, 0, provenance.row - 1, 0],
                "values": [provenance.name, start, 1, provenance.row - 1, 1],
                "line": {"width": 1.4, "color": color,
                         "dash_type": ("solid", "dash", "dash_dot", "long_dash")[style // 3]},
                "marker": {"type": "circle", "size": 3, "border": {"color": color}, "fill": {"color": color}},
            })
            series_count += 1
        if not series_count:
            # XlsxWriter requires a series. Blank referenced cells keep the
            # empty chart editable without introducing a fabricated zero.
            provenance.ensure_room(3)
            provenance.blank()
            provenance.marker(f"{field} chart: no finite result")
            start = provenance.row
            provenance.worksheet.write_blank(start, 0, None)
            provenance.worksheet.write_blank(start, 1, None)
            provenance.row += 1
            chart.add_series({"name": "데이터 없음", "categories": [provenance.name,start,0,start,0],
                              "values": [provenance.name,start,1,start,1],
                              "marker": {"type": "none"}, "line": {"none": True}})
        suffix = " · 데이터 없음" if not series_count else " (표시점 축약)" if shortened else ""
        chart.set_title({"name": f"{title} ({unit}){suffix}", "name_font": {"size": 12}})
        # Keep elapsed-time labels below negative Down/height values. Empty
        # charts have no measured scale, so don't render arbitrary axis ticks.
        chart.set_x_axis({"name": "경과시간 (s)", "num_format": "0.0",
                          "label_position": "low", "visible": bool(series_count)})
        chart.set_y_axis({"name": unit, "num_format": "0.000000000" if unit == "deg" else "0.000",
                          "crossing": "min", "visible": bool(series_count)})
        chart.set_legend({"position": "bottom", "font": {"size": 9}} if series_count else {"none": True})
        chart.show_blanks_as("gap")
        chart.show_hidden_data()  # chart backing ranges live on hidden provenance sheets
        chart.set_size({"width": 520, "height": 320})
        report_info["graphs"].insert_chart(4 + (field_index // 2) * 14, (field_index % 2) * 8, chart)
