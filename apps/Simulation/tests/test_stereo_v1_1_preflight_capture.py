from __future__ import annotations

import math
from pathlib import Path

from campaigns.stereo_v1_1.preflight_capture import _operator_command, _write_preflight_scenario
from campaigns.stereo_v1_1.workbook_campaign import build_workbook_plan


SIM_ROOT = Path(__file__).resolve().parents[1]


def test_no_flight_preflight_parks_target_at_workbook_path_start_in_world_enu(tmp_path):
    plan = build_workbook_plan(SIM_ROOT / "config/campaign/v1_1/sweep_manifest_v1_1.xlsx")
    first_run = plan["planned_runs"][0]
    original_spawn = dict(first_run["parameters"]["Vehicle"])

    fixture = _write_preflight_scenario(tmp_path, first_run)

    expected_up = 400.0 * math.tan(math.radians(20.0))
    assert fixture == {
        "source": "first_point_of_workbook_reference_path",
        "world_frame": "gazebo_world_enu",
        "axis_order": ["east", "north", "up"],
        "position_m": [0.0, 400.0, expected_up],
        "vehicle_motion": "static; no arming or flight command",
    }
    contents = (tmp_path / "preflight_scenario.env").read_text(encoding="utf-8")
    assert "DRONE_SPAWN_X_M=0.0\n" in contents
    assert "DRONE_SPAWN_Y_M=400.0\n" in contents
    assert f"DRONE_SPAWN_Z_M={expected_up}\n" in contents
    assert first_run["parameters"]["Vehicle"] == original_spawn


def test_no_flight_preflight_holds_target_static():
    command = _operator_command("x500_2m")

    assert command[-1] == "target_static_for_research:=true"
