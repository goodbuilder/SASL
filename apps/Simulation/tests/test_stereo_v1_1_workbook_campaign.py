from __future__ import annotations

import json
import posixpath
import sys
import xml.etree.ElementTree as ET
import zipfile
from pathlib import Path

import pytest


SIM_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(SIM_ROOT))

from campaigns.stereo_v1_1.workbook_campaign import (
    DEFAULT_WORKBOOK,
    LEGACY_DEFAULT_WORKBOOK,
    build_workbook_plan,
    initialize_workbook_campaign,
    read_numeric_workbook,
    reference_from_parameters,
    scenario_environment,
    validate_supported_run,
)


NS = {
    "m": "http://schemas.openxmlformats.org/spreadsheetml/2006/main",
    "r": "http://schemas.openxmlformats.org/officeDocument/2006/relationships",
}


def _truncate_to_runs(source: Path, destination: Path, run_count: int) -> None:
    with zipfile.ZipFile(source) as original:
        workbook = ET.fromstring(original.read("xl/workbook.xml"))
        relationships = ET.fromstring(original.read("xl/_rels/workbook.xml.rels"))
        targets = {item.attrib["Id"]: item.attrib["Target"] for item in relationships}
        sheet_members = {}
        for sheet in workbook.findall("m:sheets/m:sheet", NS):
            target = targets[sheet.attrib[f"{{{NS['r']}}}id"]]
            member = posixpath.normpath(target.lstrip("/") if target.startswith("/")
                                        else posixpath.join("xl", target))
            sheet_members[member] = sheet.attrib["name"]
        v3 = "Defaults" in sheet_members.values()
        with zipfile.ZipFile(destination, "w", compression=zipfile.ZIP_DEFLATED) as edited:
            for item in original.infolist():
                data = original.read(item.filename)
                if item.filename in sheet_members:
                    sheet_name = sheet_members[item.filename]
                    root = ET.fromstring(data)
                    sheet_data = root.find("m:sheetData", NS)
                    for row in list(sheet_data.findall("m:row", NS)):
                        if int(row.attrib["r"]) <= 1 or (v3 and sheet_name == "Defaults"):
                            continue
                        if sheet_name == "Runs" or not v3:
                            keep = int(row.attrib["r"]) <= run_count + 1
                        else:
                            sequence_cell = row.find("m:c[@r='A%d']/m:v" % int(row.attrib["r"]), NS)
                            keep = sequence_cell is not None and 1 <= int(float(sequence_cell.text)) <= run_count
                        if not keep:
                            sheet_data.remove(row)
                    dimension = root.find("m:dimension", NS)
                    if dimension is not None:
                        header = sheet_data.find("m:row", NS)
                        last_header_cell = header.findall("m:c", NS)[-1].attrib["r"]
                        end_column = "".join(char for char in last_header_cell if char.isalpha())
                        end_row = max(int(row.attrib["r"]) for row in sheet_data.findall("m:row", NS))
                        dimension.set("ref", f"A1:{end_column}{end_row}")
                    data = ET.tostring(root, encoding="utf-8", xml_declaration=True)
                edited.writestr(item, data)


def _remove_v3_run(source: Path, destination: Path, sequence: int) -> None:
    with zipfile.ZipFile(source) as original:
        members = _sheet_member_map(original)
        with zipfile.ZipFile(destination, "w", compression=zipfile.ZIP_DEFLATED) as edited:
            for item in original.infolist():
                data = original.read(item.filename)
                name = next((sheet for sheet, member in members.items()
                             if member == item.filename), None)
                if name is not None and name != "Defaults":
                    root = ET.fromstring(data)
                    sheet_data = root.find("m:sheetData", NS)
                    for row in list(sheet_data.findall("m:row", NS)):
                        if int(row.attrib["r"]) == 1:
                            continue
                        cell = row.find("m:c[@r='A%d']/m:v" % int(row.attrib["r"]), NS)
                        if cell is not None and int(float(cell.text)) == sequence:
                            sheet_data.remove(row)
                    dimension = root.find("m:dimension", NS)
                    if dimension is not None:
                        header = sheet_data.find("m:row", NS)
                        end_cell = header.findall("m:c", NS)[-1].attrib["r"]
                        end_column = "".join(character for character in end_cell if character.isalpha())
                        end_row = max(int(row.attrib["r"]) for row in sheet_data.findall("m:row", NS))
                        dimension.set("ref", f"A1:{end_column}{end_row}")
                    data = ET.tostring(root, encoding="utf-8", xml_declaration=True)
                edited.writestr(item, data)


def _sheet_member_map(archive: zipfile.ZipFile) -> dict[str, str]:
    workbook = ET.fromstring(archive.read("xl/workbook.xml"))
    relationships = ET.fromstring(archive.read("xl/_rels/workbook.xml.rels"))
    targets = {item.attrib["Id"]: item.attrib["Target"] for item in relationships}
    result = {}
    for sheet in workbook.findall("m:sheets/m:sheet", NS):
        target = targets[sheet.attrib[f"{{{NS['r']}}}id"]]
        result[sheet.attrib["name"]] = posixpath.normpath(
            target.lstrip("/") if target.startswith("/") else posixpath.join("xl", target)
        )
    return result


def _shared_strings(archive: zipfile.ZipFile) -> list[str]:
    if "xl/sharedStrings.xml" not in archive.namelist():
        return []
    root = ET.fromstring(archive.read("xl/sharedStrings.xml"))
    return ["".join(text.text or "" for text in item.findall(".//m:t", NS))
            for item in root.findall("m:si", NS)]


def _cell_text(cell: ET.Element | None, shared: list[str]) -> str | None:
    if cell is None:
        return None
    if cell.attrib.get("t") == "s":
        return shared[int(cell.findtext("m:v", namespaces=NS))]
    if cell.attrib.get("t") == "inlineStr":
        return "".join(text.text or "" for text in cell.findall(".//m:t", NS))
    return cell.findtext("m:v", namespaces=NS)


def _column_number(reference: str) -> int:
    number = 0
    for character in reference:
        if character.isalpha():
            number = number * 26 + ord(character.upper()) - ord("A") + 1
    return number


def _set_numeric_cell(root: ET.Element, reference: str, value: float) -> None:
    row_number = int("".join(character for character in reference if character.isdigit()))
    data = root.find("m:sheetData", NS)
    row = next((item for item in data.findall("m:row", NS)
                if int(item.attrib["r"]) == row_number), None)
    if row is None:
        row = ET.Element(f"{{{NS['m']}}}row", {"r": str(row_number)})
        data.append(row)
    cell = next((item for item in row.findall("m:c", NS)
                 if item.attrib.get("r") == reference), None)
    if cell is None:
        cell = ET.Element(f"{{{NS['m']}}}c", {"r": reference, "t": "n"})
        row.append(cell)
    else:
        cell.attrib.pop("t", None)
    value_node = cell.find("m:v", NS)
    if value_node is None:
        value_node = ET.SubElement(cell, f"{{{NS['m']}}}v")
    value_node.text = str(value)
    row[:] = sorted(row, key=lambda item: _column_number(item.attrib.get("r", "A1")))


def _edit_manifest_sheet(source: Path, destination: Path, sheet_name: str, editor) -> None:
    with zipfile.ZipFile(source) as original:
        members = _sheet_member_map(original)
        strings = _shared_strings(original)
        root = ET.fromstring(original.read(members[sheet_name]))
        editor(root, strings)
        sheet_data = root.find("m:sheetData", NS)
        dimension = root.find("m:dimension", NS)
        header = sheet_data.find("m:row", NS)
        last_header_cell = max(header.findall("m:c", NS), key=lambda cell: _column_number(cell.attrib["r"]))
        end_column = "".join(char for char in last_header_cell.attrib["r"] if char.isalpha())
        end_row = max(int(row.attrib["r"]) for row in sheet_data.findall("m:row", NS))
        if dimension is not None:
            dimension.set("ref", f"A1:{end_column}{end_row}")
        member = members[sheet_name]
        edited_sheet = ET.tostring(root, encoding="utf-8", xml_declaration=True)
        with zipfile.ZipFile(destination, "w", compression=zipfile.ZIP_DEFLATED) as edited:
            for item in original.infolist():
                edited.writestr(item, edited_sheet if item.filename == member else original.read(item.filename))


def _set_text_cell(root: ET.Element, reference: str, value: str) -> None:
    row_number = int("".join(char for char in reference if char.isdigit()))
    data = root.find("m:sheetData", NS)
    row = next(item for item in data.findall("m:row", NS) if int(item.attrib["r"]) == row_number)
    cell = next(item for item in row.findall("m:c", NS) if item.attrib.get("r") == reference)
    cell.attrib["t"] = "inlineStr"
    value_node = cell.find("m:v", NS)
    if value_node is not None:
        cell.remove(value_node)
    inline = cell.find("m:is", NS)
    if inline is None:
        inline = ET.SubElement(cell, f"{{{NS['m']}}}is")
    else:
        inline.clear()
    text_node = ET.SubElement(inline, f"{{{NS['m']}}}t")
    text_node.text = value


def _edit_v3_workbook(source: Path, destination: Path, *, shared_speed=None,
                      motion_override=None, motion_override_sequence=1,
                      remove_motion_override=False, path_bow_zero=False) -> None:
    with zipfile.ZipFile(source) as original:
        members = _sheet_member_map(original)
        strings = _shared_strings(original)
        modified = {}
        for name, member in members.items():
            if name not in {"Defaults", "Motion", "Path"}:
                continue
            root = ET.fromstring(original.read(member))
            if name == "Defaults" and shared_speed is not None:
                for row in root.findall("m:sheetData/m:row", NS):
                    category = _cell_text(row.find("m:c[@r='A%d']" % int(row.attrib["r"]), NS), strings)
                    parameter = _cell_text(row.find("m:c[@r='B%d']" % int(row.attrib["r"]), NS), strings)
                    if category == "Motion" and parameter and parameter.startswith("Cruise speed v along 3D path arc length"):
                        _set_numeric_cell(root, f"C{row.attrib['r']}", shared_speed)
                        break
                else:
                    raise AssertionError("Motion cruise-speed default was not found")
            if name == "Motion" and motion_override is not None:
                _set_numeric_cell(root, "A2", motion_override_sequence)
                _set_numeric_cell(root, "E2", motion_override)
            if name == "Motion" and remove_motion_override:
                data = root.find("m:sheetData", NS)
                for row in list(data.findall("m:row", NS)):
                    if int(row.attrib["r"]) == 1:
                        continue
                    sequence = row.find("m:c[@r='A%d']/m:v" % int(row.attrib["r"]), NS)
                    if sequence is not None and int(float(sequence.text)) == 1:
                        data.remove(row)
            if name == "Path" and path_bow_zero:
                for row in root.findall("m:sheetData/m:row", NS):
                    if int(row.attrib["r"]) == 1:
                        continue
                    sequence = row.find("m:c[@r='A%d']/m:v" % int(row.attrib["r"]), NS)
                    if sequence is not None and int(float(sequence.text)) == 2:
                        _set_numeric_cell(root, f"D{row.attrib['r']}", 0)
                        break
                else:
                    raise AssertionError("Path run 2 override was not found")
            modified[member] = ET.tostring(root, encoding="utf-8", xml_declaration=True)

        with zipfile.ZipFile(destination, "w", compression=zipfile.ZIP_DEFLATED) as edited:
            for item in original.infolist():
                edited.writestr(item, modified.get(item.filename, original.read(item.filename)))


def test_default_manifest_workbook_compiles_exact_order_seeds_scopes_and_references():
    plan = build_workbook_plan(DEFAULT_WORKBOOK)
    assert plan["planned_run_count"] == 39
    assert [row["sequence"] for row in plan["planned_runs"]] == list(range(1, 40))
    assert [row["run_id"] for row in plan["planned_runs"][:3]] == [
        "G0-P0-R01", "G0-P1-R01", "G0-P2-R01",
    ]
    assert [row["seed_requested"] for row in plan["planned_runs"]] == list(range(2026100501, 2026100540))
    assert plan["seed_policy"] == "manifest_required"
    assert all(row["attempt_id"] == "a01" for row in plan["planned_runs"])
    assert all(row["data_split"] == "development_calibration" for row in plan["planned_runs"]
               if row["role"] == "calibration")
    assert all(row["data_split"] == "locked_final_test" for row in plan["planned_runs"]
               if row["role"] == "final_evaluation")
    assert plan["planned_runs"][5]["reference_preview"]["minimum_camera_edge_margin_px"] >= 32

    parameters = plan["planned_runs"][0]["parameters"]
    reference = reference_from_parameters(parameters)
    assert reference["reference_update_hz"] == 50
    assert reference["samples"][0]["t_science_s"] == 0
    assert reference["samples"][0]["heading_deg_clockwise_from_north"] == 180
    environment = scenario_environment(parameters, 1)
    assert environment["CAMERA_CLIP_FAR_M"] == "700.0"
    assert environment["CAM_L_OPTICAL_PITCH_DEG"] == "20.0"
    assert plan["workbook_schema_version"] == "sweep_manifest_v2"
    contract = plan["environment_contract"]
    assert {row["setting_id"] for row in contract} == {
        "source_world_asset", "installed_world_asset", "world_name",
        "lighting_background", "weather", "wind_setting",
    }
    assert all(row["scope"] == "campaign_fixed" and row["applies_to"] == "all_runs"
               for row in contract)
    assert plan["planned_runs"][0]["environment_contract"] == contract
    assert parameters["Vehicle"]["model_id"] == "x500_2m"
    assert parameters["Vehicle"]["reference_link"] == "vehicle_reference_link"
    model_catalog = {row["parameter_id"]: row for row in read_numeric_workbook(DEFAULT_WORKBOOK)["catalog"]}
    assert model_catalog["Vehicle.model_id"]["data_type"] == "text"
    assert model_catalog["Vehicle.model_id"]["per_run_allowed"] is False
    assert model_catalog["Vehicle.reference_link"]["per_run_allowed"] is False
    assert plan["planned_runs"][0]["parameter_sources"]["Vehicle"]["reference_link"] == "campaign_fixed"
    assert plan["planned_runs"][0]["parameter_sources"]["Vehicle"]["model_id"] == "campaign_fixed"
    assert plan["planned_runs"][0]["parameter_sources"]["Motion"]["cruise_speed_mps"] == "campaign_fixed"
    assert plan["planned_runs"][0]["parameter_sources"]["Vehicle"]["model_id"] == "campaign_fixed"
    assert plan["planned_runs"][1]["parameter_sources"]["Path"]["bow_amplitude_m"] == "run_variable"
    assert {row["status"] for row in plan["requirement_evaluations"]
            if row["stage"] == "plan"} == {"PASS"}


def test_environment_contract_rejects_unbound_per_run_world_changes(tmp_path: Path):
    modified = tmp_path / "run_specific_world_contract.xlsx"
    _edit_manifest_sheet(
        DEFAULT_WORKBOOK, modified, "EnvironmentContract",
        lambda root, strings: _set_text_cell(root, "B2", "run_variable"),
    )

    with pytest.raises(ValueError, match="campaign_fixed/all_runs only"):
        build_workbook_plan(modified)


def test_manifest_workbook_rejects_missing_or_fixed_run_values(tmp_path: Path):
    missing = tmp_path / "missing_run_parameter.xlsx"
    def remove_required_value(root, strings):
        data = root.find("m:sheetData", NS)
        for row in list(data.findall("m:row", NS)):
            if int(row.attrib["r"]) == 1:
                continue
            number = int(row.attrib["r"])
            run_id = _cell_text(row.find(f"m:c[@r='A{number}']", NS), strings)
            parameter_id = _cell_text(row.find(f"m:c[@r='B{number}']", NS), strings)
            if run_id == "G0-P0-R01" and parameter_id == "Path.bow_amplitude_m":
                data.remove(row)
                return
        raise AssertionError("expected RunParameters row was not found")
    _edit_manifest_sheet(DEFAULT_WORKBOOK, missing, "RunParameters", remove_required_value)
    with pytest.raises(ValueError, match="RunParameters for G0-P0-R01 is incomplete"):
        build_workbook_plan(missing)

    fixed_override = tmp_path / "fixed_parameter_override.xlsx"
    _edit_manifest_sheet(
        DEFAULT_WORKBOOK, fixed_override, "RunParameters",
        lambda root, strings: _set_text_cell(root, "B2", "Motion.cruise_speed_mps"),
    )
    with pytest.raises(ValueError, match="overrides campaign_fixed Motion.cruise_speed_mps"):
        build_workbook_plan(fixed_override)


def test_manifest_workbook_rejects_duplicate_requested_seeds(tmp_path: Path):
    edited = tmp_path / "duplicate_seed.xlsx"
    _edit_manifest_sheet(
        DEFAULT_WORKBOOK, edited, "RunManifest",
        lambda root, strings: _set_numeric_cell(root, "M3", 2026100501),
    )
    with pytest.raises(ValueError, match="manifest_required seeds must be nonnegative and unique"):
        build_workbook_plan(edited)


def test_workbook_declares_vehicle_model_and_rejects_unsupported_model(tmp_path: Path):
    edited = tmp_path / "unsupported_model.xlsx"
    def replace_model(root, strings):
        data = root.find("m:sheetData", NS)
        for row in data.findall("m:row", NS):
            if int(row.attrib["r"]) == 1:
                continue
            number = int(row.attrib["r"])
            parameter_id = _cell_text(row.find(f"m:c[@r='A{number}']", NS), strings)
            if parameter_id == "Vehicle.model_id":
                _set_text_cell(root, f"C{number}", "x500_2m_unavailable")
                return
        raise AssertionError("Vehicle.model_id parameter row was not found")
    _edit_manifest_sheet(DEFAULT_WORKBOOK, edited, "Parameters", replace_model)
    with pytest.raises(ValueError, match="REQ-VEHICLE-MODEL failed"):
        build_workbook_plan(edited)

    unsupported_reference = tmp_path / "unsupported_reference_link.xlsx"
    def replace_reference(root, strings):
        data = root.find("m:sheetData", NS)
        for row in data.findall("m:row", NS):
            if int(row.attrib["r"]) == 1:
                continue
            number = int(row.attrib["r"])
            parameter_id = _cell_text(row.find(f"m:c[@r='A{number}']", NS), strings)
            if parameter_id == "Vehicle.reference_link":
                _set_text_cell(root, f"C{number}", "base_link")
                return
        raise AssertionError("Vehicle.reference_link parameter row was not found")
    _edit_manifest_sheet(DEFAULT_WORKBOOK, unsupported_reference, "Parameters", replace_reference)
    with pytest.raises(ValueError, match="REQ-VEHICLE-REFERENCE failed"):
        build_workbook_plan(unsupported_reference)


def test_shared_to_run_specific_parameter_edit_keeps_one_default_location(tmp_path: Path):
    edited = tmp_path / "shared_to_run_specific.xlsx"
    _edit_v3_workbook(LEGACY_DEFAULT_WORKBOOK, edited, shared_speed=4, motion_override=5)
    plan = build_workbook_plan(edited)
    first, second = plan["planned_runs"][:2]
    assert first["parameters"]["Motion"]["cruise_speed_mps"] == 5
    assert second["parameters"]["Motion"]["cruise_speed_mps"] == 4
    assert first["parameter_sources"]["Motion"]["cruise_speed_mps"] == "run_override"
    assert second["parameter_sources"]["Motion"]["cruise_speed_mps"] == "shared_default"


def test_run_specific_override_can_be_cleared_to_make_parameter_shared(tmp_path: Path):
    individual = tmp_path / "individual.xlsx"
    _edit_v3_workbook(LEGACY_DEFAULT_WORKBOOK, individual, shared_speed=4, motion_override=5)
    common_again = tmp_path / "common_again.xlsx"
    _edit_v3_workbook(individual, common_again, shared_speed=5, remove_motion_override=True)

    plan = build_workbook_plan(common_again)
    for row in plan["planned_runs"][:2]:
        assert row["parameters"]["Motion"]["cruise_speed_mps"] == 5
        assert row["parameter_sources"]["Motion"]["cruise_speed_mps"] == "shared_default"


def test_override_sequence_must_exist_in_runs_list(tmp_path: Path):
    edited = tmp_path / "unknown_override_sequence.xlsx"
    _edit_v3_workbook(LEGACY_DEFAULT_WORKBOOK, edited, motion_override=5,
                      motion_override_sequence=40)
    with pytest.raises(ValueError, match="override sequence 40 is not listed in Runs"):
        build_workbook_plan(edited)


def test_numeric_zero_in_override_is_not_treated_as_inherit(tmp_path: Path):
    edited = tmp_path / "explicit_zero_override.xlsx"
    _edit_v3_workbook(LEGACY_DEFAULT_WORKBOOK, edited, path_bow_zero=True)
    plan = build_workbook_plan(edited)
    second = plan["planned_runs"][1]
    assert second["parameters"]["Path"]["bow_amplitude_m"] == 0
    assert second["parameter_sources"]["Path"]["bow_amplitude_m"] == "run_override"
    assert second["shape"] == "straight"


def test_legacy_v2_workbook_remains_readable_with_identical_effective_parameters():
    legacy_path = SIM_ROOT / "config/campaign/v1_1/multi_flight_parameters_v1_1_numeric_v2.xlsx"
    legacy = read_numeric_workbook(legacy_path)
    current = read_numeric_workbook(LEGACY_DEFAULT_WORKBOOK)
    assert legacy["schema_version"] == "stereo_v1_1_numeric_v2"
    assert current["schema_version"] == "stereo_v1_1_numeric_v3"
    assert legacy["sequences"] == current["sequences"]
    for name in legacy["sheets"]:
        assert legacy["sheets"][name] == current["sheets"][name]


def test_run_count_comes_from_runs_list_and_parameter_tabs_may_be_sparse(tmp_path: Path):
    one_run = tmp_path / "one_run.xlsx"
    _truncate_to_runs(LEGACY_DEFAULT_WORKBOOK, one_run, 1)
    workbook = read_numeric_workbook(one_run)
    assert workbook["sequences"] == [1]
    plan = build_workbook_plan(one_run)
    assert plan["planned_run_count"] == 1
    assert plan["planned_runs"][0]["run_id"] == "XLSX-R0001"

    output = tmp_path / "campaign"
    summary = initialize_workbook_campaign(output, one_run)
    status = json.loads((output / "run_status.json").read_text(encoding="utf-8"))
    assert summary["planned_run_count"] == 1
    assert status["planned_run_count"] == 1
    assert status["schema_version"] == "stereo_v1_1_workbook_run_status_v1"
    changes = (output / "changes_log.jsonl").read_text(encoding="utf-8")
    assert '"source": "numeric_xlsx_workbook"' in changes
    assert status["run_slots"][0]["parameters"] == plan["planned_runs"][0]["parameters"]
    assert (output / "frozen_protocol" / "input_parameters.xlsx").is_file()
    assert status["workbook_sha256"] == plan["workbook_sha256"]


def test_removing_a_middle_run_preserves_other_numeric_ids(tmp_path: Path):
    edited = tmp_path / "without_run_2.xlsx"
    _remove_v3_run(LEGACY_DEFAULT_WORKBOOK, edited, 2)
    plan = build_workbook_plan(edited)
    assert plan["planned_run_count"] == 38
    assert [row["sequence"] for row in plan["planned_runs"][:3]] == [1, 3, 4]
    assert [row["run_id"] for row in plan["planned_runs"][:3]] == [
        "XLSX-R0001", "XLSX-R0003", "XLSX-R0004",
    ]


def test_workbook_attempt_persists_shared_and_override_sources(tmp_path: Path):
    from campaigns.stereo_v1_1 import campaign_run

    one_run = tmp_path / "one_run.xlsx"
    _truncate_to_runs(LEGACY_DEFAULT_WORKBOOK, one_run, 1)
    plan = build_workbook_plan(one_run)
    output = tmp_path / "campaign"
    initialize_workbook_campaign(output, one_run)

    attempt_id, attempt_dir, run_id = campaign_run._prepare_attempt(
        output, plan["planned_runs"][0], development=False,
    )

    sources = json.loads((attempt_dir / "workbook_parameter_sources.json").read_text(encoding="utf-8"))
    manifest = json.loads((attempt_dir / "run_manifest.json").read_text(encoding="utf-8"))
    assert run_id == "XLSX-R0001"
    assert sources["Motion"]["cruise_speed_mps"] == "shared_default"
    assert manifest["workbook_schema_version"] == "stereo_v1_1_numeric_v3"
    assert manifest["workbook_parameter_sources_file"] == "workbook_parameter_sources.json"
    assert "status" not in manifest
    assert manifest["attempt_id"] == attempt_id


def test_workbook_campaign_executes_rows_in_sequence_and_commits_each_slot(tmp_path, monkeypatch):
    from campaigns.stereo_v1_1 import campaign_run
    from campaigns.stereo_v1_1.campaign_status import begin_attempt, finish_attempt
    from campaigns.stereo_v1_1.workbook_campaign import execute_campaign

    two_runs = tmp_path / "two_runs.xlsx"
    _truncate_to_runs(LEGACY_DEFAULT_WORKBOOK, two_runs, 2)
    plan = build_workbook_plan(two_runs)
    output = tmp_path / "campaign"
    output.mkdir()
    (output / "environment_report.json").write_text(
        json.dumps({"status": "PREFLIGHT_PASS", "workbook_sha256": plan["workbook_sha256"]}),
        encoding="utf-8",
    )
    executed = []

    def fake_run_one(output_root, source):
        executed.append(source["sequence"])
        attempt, _ = begin_attempt(output_root, source["run_id"])
        qc = {"run_id": source["run_id"], "attempt_id": attempt["attempt_id"],
              "status": "VALID", "infrastructure_errors": [], "protocol_errors": [],
              "warnings": [], "evidence": {"controller_status": "science_completed"}}
        finish_attempt(output_root, source["run_id"], attempt["attempt_id"], qc)
        return qc

    monkeypatch.setattr(campaign_run, "_run_one", fake_run_one)
    monkeypatch.setattr(campaign_run, "_existing_sitl_processes", lambda: [])
    monkeypatch.setattr(campaign_run, "_audit_valid", lambda root: {"planned": 2, "valid_readback": 2})

    result = execute_campaign("run", two_runs, output)

    status = json.loads((output / "run_status.json").read_text(encoding="utf-8"))
    assert executed == [1, 2]
    assert result["status"] == "WORKBOOK_CAMPAIGN_COMPLETE"
    assert [row["status"] for row in status["run_slots"]] == ["VALID", "VALID"]


def test_unsupported_parameter_combination_is_rejected_before_flight():
    plan = build_workbook_plan(DEFAULT_WORKBOOK)
    parameters = json.loads(json.dumps(plan["planned_runs"][0]["parameters"]))
    parameters["Motion"]["intermediate_hold_s"] = 2
    with pytest.raises(ValueError, match="specifies where along s"):
        validate_supported_run(parameters)


def test_parameter_workbook_contains_no_seed_input():
    workbook = read_numeric_workbook(DEFAULT_WORKBOOK)
    assert workbook["campaign"]["seed_policy"] == "manifest_required"
    assert [row["seed_requested"] for row in workbook["run_manifest"]] == list(range(2026100501, 2026100540))


def test_requested_seed_policy_is_applied_and_legacy_environment_seed_is_cleared():
    from campaigns.stereo_v1_1.campaign_run import _apply_requested_seed

    seeded = {"SIM_GAZEBO_SEED": "stale"}
    _apply_requested_seed(seeded, {
        "seed_policy": "manifest_required", "seed_requested": 2026100501,
    }, workbook_source=True)
    assert seeded["SIM_GAZEBO_SEED"] == "2026100501"

    unseeded = {"SIM_GAZEBO_SEED": "stale"}
    _apply_requested_seed(unseeded, {
        "seed_policy": "none", "seed_requested": None,
    }, workbook_source=True)
    assert "SIM_GAZEBO_SEED" not in unseeded

    legacy = {"SIM_GAZEBO_SEED": "stale"}
    _apply_requested_seed(legacy, {"seed_requested": 123}, workbook_source=False)
    assert legacy["SIM_GAZEBO_SEED"] == "123"
