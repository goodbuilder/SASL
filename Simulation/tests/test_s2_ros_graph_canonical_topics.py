from pathlib import Path


def test_canonical_pointstamped_publishers_remain_distinct() -> None:
    source = (
        Path(__file__).resolve().parents[1]
        / "stereo_ground_sim/stereo_ground_sim/stereo_distance_node.py"
    ).read_text()
    assert '"/stereo/raw_position"' in source
    assert '"/stereo/attitude_corrected_position"' in source
    assert '"/stereo/ground_truth_position"' in source
    assert "self.raw_position_pub.publish" in source
    assert "self.attitude_corrected_position_pub.publish" in source
    assert "self.canonical_ground_truth_position_pub.publish" in source
    assert '"pair_midpoint_stamp_ns": metrics.get("pair_midpoint_stamp_ns")' in source
    assert '"pair_id": metrics.get("pair_id")' in source
