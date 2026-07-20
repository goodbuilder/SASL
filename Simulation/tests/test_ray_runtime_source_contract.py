from __future__ import annotations

from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
NODE = ROOT / "stereo_ground_sim" / "stereo_ground_sim" / "stereo_distance_node.py"
RAY_MEASUREMENT = ROOT / "stereo_ground_sim" / "stereo_ground_sim" / "ray_measurement.py"
LAUNCH = ROOT / "stereo_ground_sim" / "launch" / "stereo_ground_sim.launch.py"
RQT_PATCH = ROOT / "python_patches" / "sitecustomize.py"


def test_runtime_node_uses_camera_info_and_nominal_ray_adapter() -> None:
    source = NODE.read_text(encoding="utf-8")
    ray_source = RAY_MEASUREMENT.read_text(encoding="utf-8")

    assert '"/left_camera/camera_info"' in source
    assert '"/right_camera/camera_info"' in source
    assert "triangulate_nominal_stereo_pixels(" in source
    assert "project_nominal_stereo_point(" in source
    assert "gazebo_world_xyz_to_baseline_center_frd(" in source
    assert "distortion=msg.d" in source
    assert "pixel_grid_contract_reason(" in source
    assert '"pixel_grid_mismatch"' in ray_source
    assert '"unsupported_camera_distortion"' in ray_source
    assert "abs(world_forward_m)" not in source
    assert '"/stereo/triangulation_status"' in source
    assert '"/stereo/estimated_position"' in source
    assert "data=[truth_range_m, estimate_value, error_value]" in source


def test_attitude_error_values_modify_launch_sdf_but_are_not_estimator_inputs() -> None:
    node_source = NODE.read_text(encoding="utf-8")
    launch_source = LAUNCH.read_text(encoding="utf-8")

    for name in (
        "left_roll_error_deg",
        "left_pitch_error_deg",
        "left_yaw_error_deg",
        "right_roll_error_deg",
        "right_pitch_error_deg",
        "right_yaw_error_deg",
    ):
        assert name in launch_source
        assert name not in node_source
    assert "left_pose" in launch_source
    assert "right_pose" in launch_source
    assert "world = world.replace" in launch_source


def test_operator_rqt_patch_plots_only_truth_and_raw_position_topics() -> None:
    source = RQT_PATCH.read_text(encoding="utf-8")

    assert '"/ground_truth_position/data[0]"' in source
    assert '"/stereo/estimated_position/data[0]"' in source
    assert "/stereo/corrected_position" not in source
    assert "truth vs raw vs corrected" not in source
