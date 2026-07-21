from pathlib import Path


def test_mode_metadata_contract_is_explicit():
    text = (Path(__file__).resolve().parents[1] / "stereo_ground_sim" / "stereo_ground_sim" / "stereo_distance_node.py").read_text(encoding="utf-8")
    assert '"observation_source": "gazebo_reference_point_projection" if ideal_mode else "yolo_bbox_center"' in text
    assert '"measurement_gt_independent": not ideal_mode' in text
    assert '"reference_source": VEHICLE_REFERENCE_LINK if ideal_mode else "not_applicable"' in text
    assert '"evaluation_gt_source": "gazebo_reference_link_world_pose"' in text
    assert '"projection_camera_pose_source": "actual" if ideal_mode else "rendered_image_observation"' in text
