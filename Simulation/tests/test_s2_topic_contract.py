from __future__ import annotations

import ast
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
NODE = ROOT / "stereo_ground_sim" / "stereo_ground_sim" / "stereo_distance_node.py"


def test_canonical_point_stamped_topics_and_pair_stamp_contract() -> None:
    source = NODE.read_text(encoding="utf-8")
    ast.parse(source)
    assert '"/stereo/raw_position"' in source
    assert '"/stereo/attitude_corrected_position"' in source
    assert '"/stereo/ground_truth_position"' in source
    assert "message.header.frame_id = BASELINE_CENTER_FRD_FRAME" in source
    assert "pair_midpoint_stamp_ns" in source
    assert '"correction_source": "injected_attitude_oracle"' in source
    assert "self.corrected_position_pub.publish" in source
    assert "self.attitude_corrected_position_pub.publish" in source
