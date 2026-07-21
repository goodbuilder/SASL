from pathlib import Path


NODE = Path(__file__).resolve().parents[1] / "stereo_ground_sim" / "stereo_ground_sim" / "stereo_distance_node.py"
LAUNCH = Path(__file__).resolve().parents[1] / "stereo_ground_sim" / "launch" / "stereo_ground_sim.launch.py"


def test_observation_modes_are_explicit_and_bbox_is_default():
    text = NODE.read_text(encoding="utf-8")
    launch = LAUNCH.read_text(encoding="utf-8")
    assert 'declare_parameter("observation_mode", "bbox_yolo")' in text
    assert 'DeclareLaunchArgument("observation_mode", default_value="bbox_yolo")' in launch
    assert '"bbox_yolo"' in text and '"ideal_reference_point"' in text
    assert "yolo_bbox_center" in text
    assert "gazebo_reference_point_projection" in text
    assert "measurement_gt_independent" in text
    assert "observation_mode is immutable for one run" in text
    assert 'if getattr(self, "observation_mode", "bbox_yolo") == "bbox_yolo":' in text


def test_ideal_mode_does_not_load_yolo_or_publish_fake_detection():
    text = NODE.read_text(encoding="utf-8")
    assert "def _should_load_yolo_model" in text
    assert "self.observation_mode == \"bbox_yolo\"" in text
    assert "_ideal_detection_pair_from_reference_world" in text
