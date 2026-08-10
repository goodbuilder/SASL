from pathlib import Path


def test_canonical_pointstamped_publishers_remain_distinct() -> None:
    package = (
        Path(__file__).resolve().parents[1]
        / "stereo_ground_sim/stereo_ground_sim"
    )
    source = (package / "stereo_distance_node.py").read_text()
    pair_results = (package / "pair_result_coordinator.py").read_text()
    publication = (package / "measurement_publication.py").read_text()
    assert '"/stereo/nominal_pose_position"' in source
    assert '"/stereo/attitude_corrected_position"' in source
    assert '"/stereo/ground_truth_position"' in source
    assert "self.node.nominal_pose_position_pub.publish" in publication
    assert "self.node.attitude_corrected_position_pub.publish" in publication
    assert "self.node.canonical_ground_truth_position_pub.publish" in pair_results
    assert '"pair_midpoint_stamp_ns": metrics.get("pair_midpoint_stamp_ns")' in publication
    assert '"pair_id": metrics.get("pair_id")' in publication
