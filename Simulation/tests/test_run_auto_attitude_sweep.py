from pathlib import Path
import subprocess


ROOT = Path(__file__).resolve().parents[1]
RUNNER = ROOT / "run_auto_attitude_sweep.sh"


def test_auto_attitude_sweep_help_is_available_without_ros_runtime():
    result = subprocess.run(
        [str(RUNNER), "--help"],
        cwd=ROOT,
        check=True,
        capture_output=True,
        text=True,
    )
    assert "fresh" in result.stdout
    assert "camera_optical_rdf" in result.stdout
    assert "baseline_center_frd" in result.stdout
    assert "--seed" in result.stdout
    assert "--ordered" in result.stdout


def test_auto_attitude_sweep_preserves_interactive_auto_recording_contract():
    source = RUNNER.read_text(encoding="utf-8")
    auto_source = (ROOT / "run_auto.sh").read_text(encoding="utf-8")
    assert '"enable_research_recording:=true"' in source
    assert '"research_output_dir:=$recording_dir"' in source
    assert 'effective_config.json' in source
    assert 'research_effective_config_sha256:=$effective_config_sha256' in source
    assert 'research_session_id:=$session_id' in source
    assert 'research_attempt_id:=$case_id' in source
    assert "Save CSV" in auto_source
    assert "Save MP4" in auto_source
    assert "run_auto_attitude_sweep.sh" in auto_source
    assert "SWEEP_RANDOMIZE" in source
    assert "baseline-before" in source
    assert "baseline-after" in source
    assert 'kill -TERM "$pid"' in source
    assert 'kill -INT "$pid"' not in source


def test_research_node_has_an_explicit_graceful_signal_path():
    node = (ROOT / "stereo_ground_sim" / "stereo_ground_sim" / "stereo_distance_node.py").read_text(
        encoding="utf-8"
    )
    operator = (ROOT / "run_operator_view.sh").read_text(encoding="utf-8")
    assert "signal.signal(signal.SIGINT, _graceful_shutdown_signal)" in node
    assert "signal.signal(signal.SIGTERM, _graceful_shutdown_signal)" in node
    assert "terminate_matching INT \"stereo_ground_sim.stereo_distance_node\"" in operator


def test_auto_attitude_sweep_declares_coordinate_and_truth_contract():
    source = RUNNER.read_text(encoding="utf-8")
    assert "cam_r_optical_roll_deg" in source
    assert "cam_r_optical_pitch_deg" in source
    assert "cam_r_optical_yaw_deg" in source
    assert "baseline_center_frd[forward,right,down]" in source
    assert "Gazebo Target GT (offline calibration/validation only)" in source
    assert "execution_order" in source
    assert "effective_config_sha256" in source


def test_auto_attitude_sweep_exposes_stable_defaults_through_simulation_config():
    source = RUNNER.read_text(encoding="utf-8")
    config = (ROOT / "config" / "simulation.env").read_text(encoding="utf-8")
    for name in (
        "SWEEP_AXES",
        "SWEEP_VALUES_DEG",
        "SWEEP_REPETITIONS",
        "SWEEP_OUTPUT_DIR",
        "SWEEP_WAIT_S",
        "SWEEP_CAMERA_UPDATE_RATE_HZ",
        "SWEEP_YOLO_TARGET_HZ",
        "SWEEP_SEED",
        "SWEEP_RANDOMIZE",
        "SWEEP_SKIP_COLCON_BUILD",
        "SWEEP_KEEP_GOING",
        "SWEEP_FORCE_EXISTING_SIM",
    ):
        assert name in source
        assert name in config


def test_auto_mission_phase_is_recorded_without_using_gazebo_gt_as_runtime_input():
    mission = (ROOT / "scripts" / "px4_auto_mission.py").read_text(encoding="utf-8")
    node = (ROOT / "stereo_ground_sim" / "stereo_ground_sim" / "stereo_distance_node.py").read_text(encoding="utf-8")
    assert '"/stereo/auto_mission_phase"' in mission
    assert 'publish_phase("takeoff")' in mission
    assert 'publish_phase("approach")' in mission
    assert 'publish_phase("landing")' in mission
    assert '"/stereo/auto_mission_phase"' in node
    assert '"uam_raw_gps_reference_world_enu_m"' in node
