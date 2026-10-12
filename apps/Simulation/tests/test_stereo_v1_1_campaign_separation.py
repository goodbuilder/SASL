"""The v1.1 plan is immutable and cannot write into legacy sweep output."""

from __future__ import annotations

import importlib.util
import json
from pathlib import Path

import pytest


SIM_ROOT = Path(__file__).resolve().parents[1]
SCRIPT = SIM_ROOT / "campaigns" / "stereo_v1_1" / "plan_campaign.py"


def _planner():
    spec = importlib.util.spec_from_file_location("stereo_v1_1_plan", SCRIPT)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_frozen_manifest_has_39_unexecuted_slots_and_distinct_profiles():
    planner = _planner()
    plan = planner.build_plan()
    assert plan["planned_run_count"] == 39
    assert plan["group_counts"] == {"G0": 30, "G1": 9}
    assert plan["execution_enabled"] is False
    assert plan["flight_results_present"] is False
    model_manifest = json.loads((planner.CAMPAIGN_ROOT / "model_manifest.json").read_text())
    assert plan["yolo_model"]["sha256"] == model_manifest["sha256"]
    assert {row["run_id"] for row in plan["planned_runs"]} == {
        row["run_id"]
        for row in json.loads(
            (planner.PROTOCOL_ROOT / "04_run_manifest.json").read_text(encoding="utf-8-sig")
        )["runs"]
    }
    assert all(row["attempt_count"] == 0 and row["recording_path"] is None
               for row in plan["planned_runs"])
    assert plan["profiles"]["G0"]["world_heading_deg"] == {"heading_left_deg": 0, "heading_right_deg": 0}
    assert plan["profiles"]["G1"]["world_heading_deg"] == {"heading_left_deg": 0.5, "heading_right_deg": -0.5}
    assert plan["profiles"]["G0"]["sha256"] != plan["profiles"]["G1"]["sha256"]


def test_plan_output_is_new_and_confined_to_v1_1_root(tmp_path, monkeypatch):
    planner = _planner()
    recordings = tmp_path / "recordings"
    legacy = recordings / "auto-attitude-sweep-old"
    legacy.mkdir(parents=True)
    legacy_sentinel = legacy / "keep.txt"
    legacy_sentinel.write_text("legacy", encoding="utf-8")
    monkeypatch.setattr(planner, "RECORDINGS_ROOT", recordings / "stereo_v1_1")
    plan = planner.build_plan()
    with pytest.raises(ValueError, match="new child"):
        planner.write_plan(plan, legacy)
    with pytest.raises(ValueError, match="new child"):
        planner.write_plan(plan, recordings / "stereo_v1_1")
    assert legacy_sentinel.read_text(encoding="utf-8") == "legacy"

    target = recordings / "stereo_v1_1" / "campaign-001"
    written = planner.write_plan(plan, target)
    assert written == target / "campaign_plan.json"
    saved = json.loads(written.read_text(encoding="utf-8"))
    assert saved == plan
    with pytest.raises(FileExistsError, match="already exists"):
        planner.write_plan(plan, target)
    assert legacy_sentinel.read_text(encoding="utf-8") == "legacy"


def test_protocol_source_change_requires_explicit_revision(tmp_path, monkeypatch):
    planner = _planner()
    copied = tmp_path / "protocol"
    copied.mkdir()
    for name in (*planner.PROTOCOL_FILES, "provenance.json"):
        (copied / name).write_bytes((planner.PROTOCOL_ROOT / name).read_bytes())
    (copied / "04_run_manifest.json").write_bytes(
        (copied / "04_run_manifest.json").read_bytes() + b"\n"
    )
    monkeypatch.setattr(planner, "PROTOCOL_ROOT", copied)
    with pytest.raises(ValueError, match="hash mismatch"):
        planner.build_plan()
