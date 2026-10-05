from __future__ import annotations

import json
from pathlib import Path
from zipfile import ZipFile
from xml.etree import ElementTree

import pytest

from Workstation.workstation_core import session_review
from Workstation.workstation_core.recording_session import _review_result_row
from Workstation.workstation_core.session_review import (
    BASE_SHEET_NAMES,
    METHOD_SHEET_NAMES,
    MeasurementReview,
    write_session_review_xlsx,
)
from Workstation.workstation_core.measurement_methods import MEASUREMENT_METHOD_IDS


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
                    "valid_pair_count": 2,
                },
                results=(
                    {
                        "pair_id": "pair-0001",
                        "recording_time_ns": 1_700_000_000_000_000_123,
                        "distance_m": 128.43,
                        "measurement_method_id": "horizon_hybrid",
                        "measurement_method_instance_id": "candidate:horizon-1",
                        "measurement_geometry_sha256": "1" * 64,
                        "measurement_geometry_kind": "relative_pose",
                        "valid": True,
                    },
                    {
                        "pair_id": "pair-0002",
                        "recording_time_ns": 1_700_000_000_000_000_456,
                        "distance_m": 64.25,
                        "measurement_method_id": "checkerboard_stereo",
                        "measurement_method_instance_id": "profile:checkerboard-1",
                        "measurement_geometry_sha256": "2" * 64,
                        "measurement_geometry_kind": "relative_pose",
                        "valid": True,
                    },
                ),
                quality=(
                    {"pair_id": "pair-0001", "status": "PASS", "sync_delta_ns": 10},
                    {"pair_id": "pair-0002", "status": "PASS", "sync_delta_ns": 20},
                ),
                geometry=(
                    {
                        "method_id": "horizon_hybrid",
                        "geometry_sha256": "00112233445566778899aabbccddeeff",
                        "status": "PASS",
                    },
                    {
                        "method_id": "checkerboard_stereo",
                        "profile_id": "checkerboard-1",
                        "geometry_sha256": "2" * 64,
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
        assert _sheet_names(archive) == [
            *BASE_SHEET_NAMES,
            *METHOD_SHEET_NAMES.values(),
        ]
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

        horizon_values = set(_sheet_cells(archive, "M_Horizon").values())
        checkerboard_values = set(
            _sheet_cells(archive, "M_Checkerboard").values()
        )
        assert ("inlineStr", "pair-0001") in horizon_values
        assert ("inlineStr", "pair-0002") not in horizon_values
        assert ("inlineStr", "pair-0002") in checkerboard_values
        assert ("inlineStr", "pair-0001") not in checkerboard_values
        assert ("inlineStr", "10") in horizon_values
        assert ("inlineStr", "20") in checkerboard_values

        unused_cells = _sheet_cells(archive, "M_Terrain_PnP")
        assert unused_cells
        assert all(reference.endswith("1") for reference in unused_cells)


def test_blocked_result_without_method_stays_out_of_method_tabs(tmp_path: Path) -> None:
    output = write_session_review_xlsx(
        tmp_path,
        session_id="session-blocked-method",
        measurements=(
            MeasurementReview(
                measurement_id="m000001",
                summary={"status": "BLOCKED", "method": "checkerboard_stereo"},
                results=(
                    {
                        "pair_id": "blocked-pair-0001",
                        "pair_status": "matched_no_ready_measurement_method",
                        "measurement_method_id": "",
                        "measurement_geometry_kind": "unresolved",
                        "valid": False,
                    },
                ),
            ),
        ),
        generated_at_utc="2026-08-24T00:00:00Z",
    )

    with ZipFile(output) as archive:
        result_values = set(_sheet_cells(archive, "Results").values())
        assert ("inlineStr", "blocked-pair-0001") in result_values
        for sheet_name in METHOD_SHEET_NAMES.values():
            assert ("inlineStr", "blocked-pair-0001") not in set(
                _sheet_cells(archive, sheet_name).values()
            )


def test_each_method_outcome_stream_populates_its_own_review_tab(tmp_path: Path) -> None:
    method_results = {
        method_id: (
            {
                "pair_id": f"pair-{index}",
                "method_id": method_id,
                "method_instance_id": f"{method_id}:frozen",
                "status": "BLOCKED",
                "reasons": ["input unavailable"],
                "warnings": [],
                "result": {},
                "snapshot_sha256": f"{index + 1:x}" * 64,
            },
        )
        for index, method_id in enumerate(MEASUREMENT_METHOD_IDS)
    }
    output = write_session_review_xlsx(
        tmp_path,
        session_id="session-method-streams",
        measurements=(
            MeasurementReview(
                measurement_id="m000001",
                method_results=method_results,
            ),
        ),
        generated_at_utc="2026-08-24T00:00:00Z",
    )

    with ZipFile(output) as archive:
        for index, method_id in enumerate(MEASUREMENT_METHOD_IDS):
            values = set(
                _sheet_cells(archive, METHOD_SHEET_NAMES[method_id]).values()
            )
            assert ("inlineStr", f"pair-{index}") in values
            assert ("inlineStr", method_id) in values


def test_canonical_blocked_jsonl_explicit_empty_never_uses_distance_mode_fallback(
    tmp_path: Path,
) -> None:
    canonical_row = {
        "schema_version": "stereowiris-paired-frame-v3",
        "pair_id": "blocked-pair-v3",
        "pair_status": "matched_no_ready_measurement_method",
        "measurement_method_id": "",
        "measurement_geometry_kind": "unresolved",
        "distance_mode": "checkerboard_stereo",
        "measurement_valid": False,
    }
    source = tmp_path / "stereo_measurements.jsonl"
    source.write_text(json.dumps(canonical_row) + "\n", encoding="utf-8")
    recovered = json.loads(source.read_text(encoding="utf-8").splitlines()[0])
    result = _review_result_row(recovered)

    assert result["measurement_method_id"] == ""
    output = write_session_review_xlsx(
        tmp_path,
        session_id="session-blocked-v3",
        measurements=(
            MeasurementReview(
                measurement_id="m000001",
                summary={"status": "BLOCKED", "method": "checkerboard_stereo"},
                results=(result,),
            ),
        ),
        generated_at_utc="2026-08-24T00:00:00Z",
    )
    with ZipFile(output) as archive:
        assert ("inlineStr", "blocked-pair-v3") in set(
            _sheet_cells(archive, "Results").values()
        )
        for sheet_name in METHOD_SHEET_NAMES.values():
            assert ("inlineStr", "blocked-pair-v3") not in set(
                _sheet_cells(archive, sheet_name).values()
            )


def test_only_missing_legacy_method_key_can_migrate_distance_mode_to_method_tab(
    tmp_path: Path,
) -> None:
    legacy_missing = _review_result_row(
        {
            "schema_version": "stereowiris-paired-frame-v2",
            "pair_id": "legacy-missing-method",
            "pair_status": "measured",
            "measurement_geometry_kind": "relative_pose",
            "distance_mode": "checkerboard",
            "measurement_valid": True,
        }
    )
    legacy_explicit_empty = _review_result_row(
        {
            "schema_version": "stereowiris-paired-frame-v2",
            "pair_id": "legacy-explicit-empty",
            "pair_status": "measured",
            "measurement_method_id": "",
            "measurement_geometry_kind": "relative_pose",
            "distance_mode": "checkerboard",
            "measurement_valid": False,
        }
    )
    unknown_schema = _review_result_row(
        {
            "schema_version": "unknown-future-schema",
            "pair_id": "unknown-schema",
            "pair_status": "measured",
            "distance_mode": "checkerboard",
        }
    )

    assert legacy_missing["measurement_method_id"] == "checkerboard_stereo"
    assert legacy_explicit_empty["measurement_method_id"] == ""
    assert unknown_schema["measurement_method_id"] == ""
    output = write_session_review_xlsx(
        tmp_path,
        session_id="session-legacy-method",
        measurements=(
            MeasurementReview(
                measurement_id="m000001",
                summary={"status": "LEGACY"},
                results=(legacy_missing, legacy_explicit_empty, unknown_schema),
            ),
        ),
        generated_at_utc="2026-08-24T00:00:00Z",
    )
    with ZipFile(output) as archive:
        checkerboard = set(_sheet_cells(archive, "M_Checkerboard").values())
        assert ("inlineStr", "legacy-missing-method") in checkerboard
        assert ("inlineStr", "legacy-explicit-empty") not in checkerboard
        assert ("inlineStr", "unknown-schema") not in checkerboard


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
