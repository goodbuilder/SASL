from dataclasses import replace
import zipfile
from xml.etree import ElementTree as ET

from Workstation.workstation_core.measurement_methods import MEASUREMENT_METHOD_IDS
from Workstation.workstation_core.measurement_record import METHOD_SHEET_NAMES
from Workstation.workstation_core.measurement_report import FRONT_SHEETS, RT_FIELDS, _chart_points, rt_locations
from Workstation.workstation_core.measurement_workbook import SECTION_A, write_measurement_workbook
from Workstation.workstation_core.measurement_xlsx_read import find_marker_row, read_workbook, table_rows
from Workstation.tests.test_measurement_workbook import _populated_record


def test_all_methods_show_expanded_rt_matching_dedicated_tab(tmp_path):
    _, record = _populated_record(tmp_path)
    snapshot = record.snapshot()
    source = next(r for r in snapshot.method_results if r.status == "VALID")
    # Distinct, hand-authored axes expose swaps/transposes and lost precision.
    geometry = {**source.derived, "rotation": ((0.0, -1.0, 0.0), (1.0, 0.0, 0.0), (0.0, 0.0, 1.0)),
                "translation": (1.234567890123, -2.345678901234, 3.456789012345)}
    rows = tuple(replace(source, method_id=m, status="ESTIMATED", derived=geometry) for m in MEASUREMENT_METHOD_IDS)
    snapshot = replace(snapshot, method_results=rows)
    path = write_measurement_workbook(snapshot, record.workbook_path)
    sheets = read_workbook(path)
    assert tuple(sheets)[:5] == FRONT_SHEETS
    assert len(sheets) == 40
    assert "05_그래프" in sheets
    grid = sheets["04_Rt"]
    header = next(i for i, row in enumerate(grid) if row and row[0] == "R,t 번호")
    rt = {r["R,t 번호"]: r for r in table_rows(grid, header)}
    assert len(rt) == 10
    locations = rt_locations(snapshot)
    for method in MEASUREMENT_METHOD_IDS:
        method_grid = sheets[METHOD_SHEET_NAMES[method]]
        result = table_rows(method_grid, find_marker_row(method_grid, SECTION_A) + 1)[0]
        reference = rt[result["rt_id"]]
        assert reference["결과 상태"] == "추정"
        assert reference["R12"] == -1 and reference["R21"] == 1
        assert reference["T_D"] == -2.345678901234
        assert all(result[name] == reference[name] for name in RT_FIELDS)
        target_sheet, target_row = locations[result["rt_id"]]
        assert sheets[target_sheet][target_row][0] == result["rt_id"]
    with zipfile.ZipFile(path) as archive:
        ns = {"m": "http://schemas.openxmlformats.org/spreadsheetml/2006/main"}
        workbook = ET.fromstring(archive.read("xl/workbook.xml"))
        for index, sheet in enumerate(workbook.find("m:sheets", ns), 1):
            if sheet.attrib["name"] in METHOD_SHEET_NAMES.values():
                assert sheet.attrib.get("state", "visible") == "visible"
                xml = ET.fromstring(archive.read(f"xl/worksheets/sheet{index}.xml"))
                for col in xml.findall("m:cols/m:col", ns):
                    if int(col.attrib["max"]) >= 9 and int(col.attrib["min"]) <= 20:
                        assert col.attrib.get("hidden", "0") == "0"
                assert xml.find("m:autoFilter", ns) is not None
                assert xml.find("m:sheetViews/m:sheetView/m:pane", ns).attrib["xSplit"] == "2"


def test_chart_sampling_is_bounded_and_keeps_extremes_and_gaps():
    points = [(i / 10, 100.0) for i in range(20_000)]
    points[25] = (2.5, -123.0)
    points[38] = (3.8, 9876.0)
    points[31] = (3.1, None)
    result = _chart_points(points)
    assert len(result) <= 2000
    assert points[25] in result and points[38] in result and points[31] in result
    assert result[0] == points[0] and result[-1] == points[-1]


def test_continuation_sheets_preserve_flat_rows_and_exact_reproduction(tmp_path, monkeypatch):
    import Workstation.workstation_core.measurement_workbook as module
    from Workstation.workstation_core.measurement_reproduce import reproduce_workbook_exact
    from Workstation.workstation_core.measurement_xlsx_read import table_rows_with_parts

    _, record = _populated_record(tmp_path)
    monkeypatch.setattr(module, "MAX_SHEET_ROWS", 64)
    path = write_measurement_workbook(record.snapshot(), record.workbook_path)
    sheets = read_workbook(path)
    assert any(name.startswith("91_Exact_") for name in sheets)
    assert len(table_rows_with_parts(sheets, "90_Flat")) == 30
    results = reproduce_workbook_exact(path)
    assert len(results) == 3
    assert all(row.bit_equal for row in results)
