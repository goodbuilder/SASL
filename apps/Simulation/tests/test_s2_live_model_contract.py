from pathlib import Path


def test_user_model_is_external_runtime_input_not_packaged_or_hard_coded() -> None:
    root = Path(__file__).resolve().parents[3]
    launch = (root / "apps/Simulation/stereo_ground_sim/launch/stereo_ground_sim.launch.py").read_text()
    setup = (root / "apps/Simulation/stereo_ground_sim/setup.py").read_text()
    assert 'DeclareLaunchArgument("yolo_model_path"' in launch
    assert '"yolo_model_path"' in launch
    assert "yolov8n_uav.pt" not in launch
    assert "yolov8n_uav.pt" not in setup
