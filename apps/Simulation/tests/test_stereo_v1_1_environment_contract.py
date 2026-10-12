from __future__ import annotations

import copy
import hashlib
import json
from pathlib import Path

import pytest

from campaigns.stereo_v1_1.campaign_preflight import evaluate_environment_contract
from campaigns.stereo_v1_1 import campaign_run
from campaigns.stereo_v1_1.workbook_campaign import (
    DEFAULT_WORKBOOK,
    build_workbook_plan,
    initialize_workbook_campaign,
)


SIM_ROOT = Path(__file__).resolve().parents[1]


def test_manifest_environment_contract_matches_world_assets_and_runtime_name():
    plan = build_workbook_plan(DEFAULT_WORKBOOK)

    report = evaluate_environment_contract(
        plan["environment_contract"], SIM_ROOT,
        runtime_world_name="stereo_ground_drone",
    )

    assert report["status"] == "PASS"
    assert all(row["status"] == "PASS" for row in report["checks"])
    assert report["observed_world"]["world_name"] == "stereo_ground_drone"
    assert report["observed_world"]["atmosphere_type"] == "adiabatic"
    assert report["observed_world"]["wind_nodes"] == []
    assert report["observed_world"]["wind_plugins"] == []
    assert report["observed_world"]["lights"][0]["type"] == "directional"


@pytest.mark.parametrize("change,expected_check", [
    ("hash", "environment_contract_source_world_asset"),
    ("runtime_world", "environment_contract_world_name"),
])
def test_manifest_environment_contract_blocks_hash_or_world_drift(change, expected_check):
    contract = copy.deepcopy(build_workbook_plan(DEFAULT_WORKBOOK)["environment_contract"])
    runtime_world_name = "stereo_ground_drone"
    if change == "hash":
        next(row for row in contract if row["setting_id"] == "source_world_asset")[
            "expected_sha256"
        ] = "0" * 64
    else:
        runtime_world_name = "unexpected_world"

    report = evaluate_environment_contract(contract, SIM_ROOT, runtime_world_name)

    assert report["status"] == "FAIL"
    assert expected_check in {row["name"] for row in report["checks"]
                              if row["status"] == "FAIL"}


def test_missing_environment_contract_is_a_preflight_failure():
    report = evaluate_environment_contract([], SIM_ROOT, "stereo_ground_drone")

    assert report["status"] == "FAIL"
    assert report["checks"][0]["name"] == "environment_contract_present"


def test_attempt_records_world_and_preflight_environment_hash(tmp_path):
    plan = build_workbook_plan(DEFAULT_WORKBOOK)
    output_root = tmp_path / "campaign"
    initialize_workbook_campaign(output_root, DEFAULT_WORKBOOK)
    report_path = output_root / "environment_report.json"
    report_path.write_text(json.dumps({"status": "PREFLIGHT_PASS"}), encoding="utf-8")

    _, attempt_dir, _ = campaign_run._prepare_attempt(
        output_root, plan["planned_runs"][0], development=False,
    )

    run_manifest = json.loads((attempt_dir / "run_manifest.json").read_text(encoding="utf-8"))
    effective = json.loads((attempt_dir / "effective_config.json").read_text(encoding="utf-8"))
    expected_report_hash = hashlib.sha256(report_path.read_bytes()).hexdigest()
    assert run_manifest["gazebo_world_name"] == "stereo_ground_drone"
    assert effective["gazebo_world_name"] == "stereo_ground_drone"
    assert effective["environment_report_sha256"] == expected_report_hash
