from __future__ import annotations

import ast
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
NODE = ROOT / "stereo_ground_sim" / "stereo_ground_sim" / "stereo_distance_node.py"


def test_canonical_point_stamped_topics_and_pair_stamp_contract() -> None:
    source = NODE.read_text(encoding="utf-8")
    pair_results = (NODE.parent / "pair_result_coordinator.py").read_text(encoding="utf-8")
    publication = (NODE.parent / "measurement_publication.py").read_text(encoding="utf-8")
    ast.parse(source)
    assert '"/stereo/nominal_pose_position"' in source
    assert '"/stereo/attitude_corrected_position"' in source
    assert '"/stereo/ground_truth_position"' in source
    assert 'PointStamped, "/stereo/nominal_pose_position"' in source
    assert 'Float32MultiArray, "/stereo/nominal_pose_position"' not in source
    assert "message.header.frame_id = BASELINE_CENTER_FRD_FRAME" in publication
    assert "pair_midpoint_stamp_ns" in publication
    assert '"correction_source": "gazebo_observed_optical_pose"' in source + pair_results
    assert "corrected_publisher.publish" in publication
    assert "self.node.attitude_corrected_position_pub.publish" in publication
    assert 'Float32MultiArray, "/stereo/nominal_pose_plot"' in source
