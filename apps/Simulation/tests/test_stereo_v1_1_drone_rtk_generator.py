from __future__ import annotations

import hashlib
import json
from pathlib import Path

from campaigns.stereo_v1_1.generate_drone_rtk_observations import generate_attempt


def test_drone_rtk_generator_is_deterministic_10hz_enu_and_resume_safe(tmp_path):
    run_id, attempt_id = "G0-P0-R01", "a01"
    attempt_dir = tmp_path / "raw" / run_id / attempt_id
    attempt_dir.mkdir(parents=True)
    status = {
        "run_status": 39,
        "run_slots": [{
            "run_id": run_id, "status": "VALID",
            "attempts": [{"attempt_id": attempt_id, "recording_path": f"raw/{run_id}/{attempt_id}"}],
        }],
    }
    (tmp_path / "run_status.json").write_text(json.dumps(status), encoding="utf-8")
    with (attempt_dir / "ground_truth.jsonl").open("w", encoding="utf-8") as handle:
        for index in range(21):
            stamp = index * 20_000_000
            row = {
                "simulation_epoch": 0,
                "source_timestamp_ns": stamp,
                "position_frame": "gazebo_world_enu",
                "position_axis_order": "east,north,up",
                "position_world_enu_m": [0.1 * index, 100.0 + 0.2 * index, 36.0 + 0.1 * index],
            }
            handle.write(json.dumps(row) + "\n")

    output, provenance = generate_attempt(tmp_path, run_id, attempt_id, 7)
    first_hash = hashlib.sha256(output.read_bytes()).hexdigest()
    output_again, provenance_again = generate_attempt(tmp_path, run_id, attempt_id, 7)
    assert output_again == output
    assert provenance_again == provenance
    assert hashlib.sha256(output_again.read_bytes()).hexdigest() == first_hash
    rows = [json.loads(line) for line in output.read_text(encoding="utf-8").splitlines()]
    assert [row["timestamp_ns"] for row in rows] == [0, 100_000_000, 200_000_000, 300_000_000, 400_000_000]
    assert all(row["position_frame"] == "gazebo_world_enu" for row in rows)
    assert all(row["position_axis_order"] == "east,north,up" for row in rows)
    assert all(row["synthetic"] is True and row["reference_point"] == "vehicle_reference_link" for row in rows)
    assert json.loads(provenance.read_text(encoding="utf-8"))["estimator_input"] is False
