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


def test_bbox_and_ideal_loader_subscription_preconditions_are_separate():
    text = NODE.read_text(encoding="utf-8")
    assert 'and bool(self.yolo_model_path.strip())' in text
    assert 'and not self.yolo_runtime_disabled' in text
    assert 'if self.observation_mode == "ideal_reference_point":' in text
    assert '"reference_source": VEHICLE_REFERENCE_LINK if ideal_mode else "not_applicable"' in text


def test_loader_and_subscription_preconditions_by_mode():
    from stereo_ground_sim.stereo_distance_node import StereoDistanceNode
    def check(mode, hz, path, disabled):
        n = type("Harness", (), {})()
        n.observation_mode, n.yolo_target_hz = mode, hz
        n.yolo_model_path, n.yolo_runtime_disabled = path, disabled
        return StereoDistanceNode._should_load_yolo_model(n), StereoDistanceNode._should_subscribe_camera_images(n)
    assert check("bbox_yolo", 40.0, "/tmp/model.pt", False) == (True, True)
    assert check("bbox_yolo", 40.0, "", False) == (False, False)
    assert check("bbox_yolo", 0.0, "/tmp/model.pt", False) == (False, False)
    assert check("bbox_yolo", 40.0, "/tmp/model.pt", True) == (False, False)
    assert check("ideal_reference_point", 40.0, "", False) == (False, True)
