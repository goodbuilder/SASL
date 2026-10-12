"""Real campaign ordering begins with the four prescribed manifest slots."""

from __future__ import annotations

import sys
import json
from pathlib import Path


SIM_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(SIM_ROOT))

from campaigns.stereo_v1_1.campaign_run import PILOT_IDS, _plan_rows, _reference, _ros_command, _reconcile_interrupted
from campaigns.stereo_v1_1.campaign_status import begin_attempt, initialize


def test_selected_pilots_are_part_of_39_and_reference_is_continuous():
    rows = _plan_rows()
    assert len(rows) == 39
    assert tuple(row["run_id"] for row in rows[:4]) == PILOT_IDS
    assert len({row["run_id"] for row in rows}) == 39
    path = _reference("P4")
    assert path["reference_update_hz"] == 50
    assert path["samples"][0]["position_world_enu_m"][1] == 400
    assert path["samples"][-1]["position_world_enu_m"][1] == 300
    assert max(row["position_world_enu_m"][0] for row in path["samples"]) > 5.99
    assert min(row["position_world_enu_m"][0] for row in path["samples"]) < -5.99


def test_ros_wrapper_passes_arguments_as_argv_not_shell_text():
    command = _ros_command(Path("/bin/python3"), Path("script.py"), "literal;$(touch bad)")
    assert command[:4] == [
        "bash", "-c",
        'source "${SASL_ROS_WORKSPACE:-workspace/main}/install/setup.bash"\nexec "$@"',
        "_",
    ]
    assert command[-1] == "literal;$(touch bad)"


def test_resume_preserves_incomplete_attempt_and_reopens_slot(tmp_path):
    initialize(tmp_path)
    _, attempt_path = begin_attempt(tmp_path, "G0-P0-R01")
    (attempt_path / "raw_fragment.txt").write_text("keep me")
    slot = next(row for row in json.loads((tmp_path / "run_status.json").read_text())["run_slots"]
                if row["run_id"] == "G0-P0-R01")
    _reconcile_interrupted(tmp_path, slot)
    status = json.loads((tmp_path / "run_status.json").read_text())
    row = next(row for row in status["run_slots"] if row["run_id"] == "G0-P0-R01")
    assert row["status"] == "INFRASTRUCTURE_FAILED"
    assert row["attempt_count"] == 1
    assert (attempt_path / "raw_fragment.txt").read_text() == "keep me"
    assert (attempt_path / "resume_reconciliation.json").is_file()
    next_attempt, next_path = begin_attempt(tmp_path, "G0-P0-R01")
    assert next_attempt["attempt_id"] == "a02"
    assert next_attempt["seed_requested"] == slot["seed_requested"]
    assert next_path != attempt_path
