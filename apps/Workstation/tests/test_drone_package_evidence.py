from __future__ import annotations

import json
from pathlib import Path

from Workstation.workstation_core.recording_session import save_session_files
from Workstation.workstation_core.session_parquet import read_lossless_rows


def test_active_drone_px4_rows_are_authoritative_jsonl_and_derived_parquet(
    tmp_path: Path,
) -> None:
    output = tmp_path / "recordings" / "drone-session"
    sentinel = {
        "sequence": 7,
        "side": "drone",
        "sample_type": "gps_rtk",
        "mavlink_source_system": 42,
        "latitude_deg": 36.123456789,
        "unknown_future_field": {"kept": True},
    }

    saved = save_session_files(
        output,
        metadata={"session_id": "drone-session"},
        pair_rows=[],
        camera_rows={"left": [], "right": []},
        px4_rows={"left": [], "right": [], "drone": [sentinel]},
        gnss_base_rows=[],
        extrinsics_comparison_rows=[],
    )

    manifest = json.loads(saved.manifest.read_text(encoding="utf-8"))
    raw_name = manifest["resource_files"]["raw_px4_drone"]
    raw_detail = manifest["artifact_details"]["raw_px4_drone"]
    name = manifest["resource_files"]["pixhawk_timeline_parquet"]
    detail = manifest["artifact_details"]["pixhawk_timeline"]
    assert raw_name == "data/raw/px4_drone.jsonl"
    assert raw_detail["authoritative"] is True
    assert raw_detail["row_count"] == 1
    assert json.loads((output / raw_name).read_text(encoding="utf-8")) == {
        **sentinel,
        "measurement_id": "drone-session",
        "measurement_package_id": "drone-session",
    }
    assert detail["verified"] is True
    assert detail["authoritative"] is False
    assert detail["row_count"] == 1
    assert name == "data/pixhawk_timeline.parquet"
    row = read_lossless_rows(saved.pixhawk_timeline_parquet, "pixhawk_timeline")[0]
    assert row["source_id"] == "drone"
    assert row["latitude_deg"] == sentinel["latitude_deg"]
    assert row["mavlink_source_system"] == 42
    assert row["unknown_future_field"] == {"kept": True}
