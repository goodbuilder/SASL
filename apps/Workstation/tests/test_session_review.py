from __future__ import annotations

from pathlib import Path
from zipfile import ZipFile
from xml.etree import ElementTree

import pytest

from Workstation.workstation_core import session_review
from Workstation.workstation_core.session_review import (
    BASE_SHEET_NAMES,
    MeasurementReview,
    write_session_review_xlsx,
)


MAIN_NS = "http://schemas.openxmlformats.org/spreadsheetml/2006/main"
DOC_REL_NS = "http://schemas.openxmlformats.org/officeDocument/2006/relationships"
PACKAGE_REL_NS = "http://schemas.openxmlformats.org/package/2006/relationships"


def _sheet_names(archive: ZipFile) -> list[str]:
    root = ElementTree.fromstring(archive.read("xl/workbook.xml"))
    return [
        element.attrib["name"]
        for element in root.findall(f"{{{MAIN_NS}}}sheets/{{{MAIN_NS}}}sheet")
    ]


def _sheet_member(archive: ZipFile, sheet_name: str) -> str:
    workbook = ElementTree.fromstring(archive.read("xl/workbook.xml"))
    sheet = next(
        element
        for element in workbook.findall(
            f"{{{MAIN_NS}}}sheets/{{{MAIN_NS}}}sheet"
        )
        if element.attrib["name"] == sheet_name
    )
    relationship_id = sheet.attrib[f"{{{DOC_REL_NS}}}id"]
    relationships = ElementTree.fromstring(
        archive.read("xl/_rels/workbook.xml.rels")
    )
    target = next(
        element.attrib["Target"]
        for element in relationships.findall(f"{{{PACKAGE_REL_NS}}}Relationship")
        if element.attrib["Id"] == relationship_id
    )
    return "xl/" + target.lstrip("/")


def _sheet_cells(archive: ZipFile, sheet_name: str) -> dict[str, tuple[str | None, str]]:
    root = ElementTree.fromstring(archive.read(_sheet_member(archive, sheet_name)))
    cells: dict[str, tuple[str | None, str]] = {}
    for cell in root.findall(f".//{{{MAIN_NS}}}c"):
        cell_type = cell.attrib.get("t")
        if cell_type == "inlineStr":
            value = "".join(
                item.text or "" for item in cell.findall(f".//{{{MAIN_NS}}}t")
            )
        else:
            value_node = cell.find(f"{{{MAIN_NS}}}v")
            value = "" if value_node is None else value_node.text or ""
        cells[cell.attrib["r"]] = (cell_type, value)
    return cells


def _sheet_relationship_targets(archive: ZipFile, sheet_name: str) -> list[str]:
    member = _sheet_member(archive, sheet_name)
    path = Path(member)
    relationship_member = str(path.parent / "_rels" / f"{path.name}.rels")
    if relationship_member not in archive.namelist():
        return []
    root = ElementTree.fromstring(archive.read(relationship_member))
    return [
        element.attrib["Target"]
        for element in root.findall(f"{{{PACKAGE_REL_NS}}}Relationship")
        if element.attrib.get("TargetMode") == "External"
    ]


def test_integrated_review_workbook_contains_all_operator_sheets(tmp_path: Path) -> None:
    data_path = tmp_path / "measurements" / "m000001" / "data" / "measurement_pairs.parquet"
    data_path.parent.mkdir(parents=True)
    data_path.write_bytes(b"parquet-placeholder")
    output = write_session_review_xlsx(
        tmp_path,
        session_id="session-20260821-001",
        session_summary={
            "state": "SEALED",
            "equipment": {"camera": "StereoWIRIS", "pixhawk_count": 2},
        },
        measurements=(
            MeasurementReview(
                measurement_id="m000001",
                summary={
                    "status": "VALID",
                    "method": "horizon_hybrid",
                    "valid_pair_count": 1,
                },
                results=(
                    {
                        "pair_id": "pair-0001",
                        "recording_time_ns": 1_700_000_000_000_000_123,
                        "distance_m": 128.43,
                        "valid": True,
                    },
                ),
                quality=({"pair_id": "pair-0001", "status": "PASS"},),
                geometry=(
                    {
                        "method_id": "horizon_hybrid",
                        "geometry_sha256": "00112233445566778899aabbccddeeff",
                        "status": "PASS",
                    },
                ),
                devices=({"device_id": "left-pixhawk", "connected": True},),
                captures=({"capture_id": "capture-1", "status": "COMPLETE"},),
                warnings=({"severity": "WARNING", "code": "LEVER_ARM"},),
                artifacts=(
                    {
                        "role": "measurement_pairs",
                        "measurement_relative_path": "data/measurement_pairs.parquet",
                        "sha256": "0" * 64,
                        "status": "VERIFIED",
                    },
                ),
            ),
        ),
        generated_at_utc="2026-08-21T12:34:56+00:00",
    )

    assert output == tmp_path / "review.xlsx"
    assert output.is_file()
    assert not list(tmp_path.glob(".review.xlsx.*.tmp.xlsx"))

    with ZipFile(output) as archive:
        assert _sheet_names(archive) == list(BASE_SHEET_NAMES)
        session_cells = _sheet_cells(archive, "Session")
        assert (None, "2") in session_cells.values()
        assert ("inlineStr", "equipment.camera") in session_cells.values()

        result_cells = _sheet_cells(archive, "Results")
        # Nanosecond values stay lossless text; ordinary metrics stay numeric.
        assert result_cells["D2"] == ("inlineStr", "1700000000000000123")
        assert result_cells["F2"] == (None, "128.43")

        geometry_cells = _sheet_cells(archive, "Geometry")
        assert ("inlineStr", "00112233445566778899aabbccddeeff") in (
            geometry_cells.values()
        )

        files_cells = _sheet_cells(archive, "Files")
        expected_path = "measurements/m000001/data/measurement_pairs.parquet"
        assert ("inlineStr", expected_path) in files_cells.values()
        assert _sheet_relationship_targets(archive, "Files") == [
            "measurements\\m000001\\data\\measurement_pairs.parquet"
        ]


def test_untrusted_text_is_escaped_and_only_safe_relative_paths_are_linked(
    tmp_path: Path,
) -> None:
    safe_path = tmp_path / "measurements" / "m000001" / "evidence" / "left.mp4"
    safe_path.parent.mkdir(parents=True)
    safe_path.write_bytes(b"video")
    outside = tmp_path.parent / "outside-review.txt"
    outside.write_text("outside", encoding="utf-8")
    (tmp_path / "escape-link").symlink_to(outside)
    output = write_session_review_xlsx(
        tmp_path,
        session_id="session-safe-text",
        measurements=(
            {
                "measurement_id": "m000001",
                "summary": {"status": "=HYPERLINK(\"https://invalid\")"},
                "tables": {
                    "results": [
                        {
                            "pair_id": "pair-1",
                            "warning_code": "  +SUM(1,1)",
                        }
                    ]
                },
                "artifacts": [
                    {
                        "role": "safe",
                        "relative_path": "measurements/m000001/evidence/left.mp4",
                    },
                    {"role": "escape", "relative_path": "../outside.txt"},
                    {"role": "absolute", "relative_path": "/etc/passwd"},
                    {"role": "missing", "relative_path": "missing/file.txt"},
                    {"role": "drive", "relative_path": "C:relative.txt"},
                    {"role": "symlink_escape", "relative_path": "escape-link"},
                ],
            },
        ),
        generated_at_utc="2026-08-21T00:00:00Z",
    )

    with ZipFile(output) as archive:
        measurements = _sheet_cells(archive, "Measurements")
        results = _sheet_cells(archive, "Results")
        assert ("inlineStr", "'=HYPERLINK(\"https://invalid\")") in (
            measurements.values()
        )
        assert ("inlineStr", "'  +SUM(1,1)") in results.values()
        assert b"<f>" not in archive.read(_sheet_member(archive, "Measurements"))
        assert b"<f>" not in archive.read(_sheet_member(archive, "Results"))
        assert _sheet_relationship_targets(archive, "Files") == [
            "measurements\\m000001\\evidence\\left.mp4"
        ]


def test_large_tables_are_split_without_losing_rows(tmp_path: Path) -> None:
    output = write_session_review_xlsx(
        tmp_path,
        session_id="session-split",
        measurements=(
            {
                "measurement_id": "m000001",
                "summary": {"status": "VALID"},
                "results": [
                    {"pair_id": f"pair-{index:02d}", "distance_m": float(index)}
                    for index in range(9)
                ],
            },
        ),
        generated_at_utc="2026-08-21T00:00:00Z",
        max_rows_per_sheet=5,
    )

    with ZipFile(output) as archive:
        names = _sheet_names(archive)
        assert "Results" in names
        assert "Results_002" in names
        assert "Results_003" in names
        pair_ids = []
        for name in ("Results", "Results_002", "Results_003"):
            pair_ids.extend(
                value
                for _cell_type, value in _sheet_cells(archive, name).values()
                if value.startswith("pair-")
            )
        assert pair_ids == [f"pair-{index:02d}" for index in range(9)]


def test_failed_generation_preserves_previous_workbook_and_removes_temp_file(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    existing = tmp_path / "review.xlsx"
    existing.write_bytes(b"previous-complete-workbook")

    def fail_write(*args: object, **kwargs: object) -> None:
        raise RuntimeError("simulated workbook generation failure")

    monkeypatch.setattr(session_review, "_write_workbook", fail_write)

    with pytest.raises(RuntimeError, match="simulated workbook generation failure"):
        write_session_review_xlsx(
            tmp_path,
            session_id="session-atomic",
            measurements=(),
            generated_at_utc="2026-08-21T00:00:00Z",
        )

    assert existing.read_bytes() == b"previous-complete-workbook"
    assert not list(tmp_path.glob(".review.xlsx.*.tmp.xlsx"))


def test_conflicting_measurement_identity_is_rejected_before_publish(
    tmp_path: Path,
) -> None:
    with pytest.raises(ValueError, match="does not match"):
        write_session_review_xlsx(
            tmp_path,
            session_id="session-conflict",
            measurements=(
                {
                    "measurement_id": "m000001",
                    "results": [
                        {"measurement_id": "m999999", "pair_id": "pair-1"}
                    ],
                },
            ),
        )

    assert not (tmp_path / "review.xlsx").exists()
