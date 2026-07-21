from pathlib import Path


def test_invalid_ideal_projection_has_explicit_terminal_diagnostics():
    text = (Path(__file__).resolve().parents[1] / "stereo_ground_sim" / "stereo_ground_sim" / "stereo_distance_node.py").read_text(encoding="utf-8")
    assert "ideal_projection_rejected_count" in text
    assert "projection_invalid_or_out_of_fov" in text
    assert "ideal_projection_success_count" in text
    assert "accepted_pair_discard_count += 1" in text
    assert "0.0 <= float(pixel[0]) < float(width)" in text
