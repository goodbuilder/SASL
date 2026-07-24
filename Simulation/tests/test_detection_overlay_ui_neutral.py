from pathlib import Path


ROOT_DIR = Path(__file__).resolve().parents[1]
SOURCE = ROOT_DIR / "stereo_ground_sim_overlay" / "src" / "detection_overlay_node.cpp"


def test_flight_overlay_does_not_assign_quality_colours_to_bbox_observations() -> None:
    source = SOURCE.read_text(encoding="utf-8")

    assert "pair_has_positive_disparity" not in source
    assert "drawMissingBorder" not in source
    assert "const std::array<uint8_t, 3> color{230, 230, 230};" in source
    assert "quality judgement" in source
