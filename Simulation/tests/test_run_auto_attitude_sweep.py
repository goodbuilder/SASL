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
    assert "--headless" not in source  # RViz/rqt_gui stay visible for every case.
    assert 'setsid env SIM_CONFIG="$SIM_CONFIG"' in source
    assert 'Operator stack exited during Auto' in source
    assert "validate_attitude_sweep_output.py" in source
    assert "SWEEP_FINALIZED_VALID" in source
    assert "quick_requested" in source
    assert 'auto_args+=(--target-distance-m "${DRONE_TARGET_DISTANCE_M:-500.0}")' in source


def test_auto_attitude_sweep_supports_combined_axis_and_geometry_diversity():
    source = RUNNER.read_text(encoding="utf-8")
    mission = (ROOT / "scripts" / "px4_auto_mission.py").read_text(encoding="utf-8")
    mission_runner = (ROOT / "run_mission.sh").read_text(encoding="utf-8")
    assert "roll,pitch,yaw,combined" in source
    assert 'combined) roll="$angle"; pitch="$angle"; yaw="$angle" ;;' in source
    assert "--altitudes-m" in source
    assert "--lateral-biases-m" in source
    assert "SWEEP_ALTITUDES_M" in source
    assert "SWEEP_LATERAL_BIASES_M" in source
    assert "geometry_altitude_m" in source
    assert "geometry_lateral_bias_m" in source
    assert "altitude_requested" in source
    assert "lateral_bias_requested" in source
    assert '"${case_auto_args[@]}"' in source
    assert "--lateral-bias-m" in mission
    assert "--lateral-bias-m" in mission_runner


def test_auto_attitude_sweep_baseline_before_shares_geometry_with_its_condition():
    source = RUNNER.read_text(encoding="utf-8")
    assert 'local case_id="$1" axis="$2" angle="$3" repetition="$4" geometry_index="$5"' in source
    assert (
        'run_case "case-$(printf \'%03d\' "$case_index")-baseline-before" baseline 0 0 "$pair_index"'
        in source
    )
    assert (
        'run_case "$case_id" "$axis" "$value" "$repetition" "$pair_index"' in source
    )
    # pair_index advances once per condition (after the paired case runs),
    # not once per case, so baseline-before and its condition share geometry.
    assert source.count("pair_index=$((pair_index + 1))") == 1


def test_auto_attitude_sweep_supports_active_lateral_sweep_within_fov_margin():
    source = RUNNER.read_text(encoding="utf-8")
    mission = (ROOT / "scripts" / "px4_auto_mission.py").read_text(encoding="utf-8")
    mission_runner = (ROOT / "run_mission.sh").read_text(encoding="utf-8")
    assert "--lateral-sweep-amplitude-m" in source
    assert "--lateral-sweep-period-s" in source
    assert "SWEEP_LATERAL_SWEEP_AMPLITUDE_M" in source
    assert "SWEEP_LATERAL_SWEEP_PERIOD_S" in source
    assert "geometry_lateral_sweep_amplitude_m" in source
    assert "geometry_lateral_sweep_period_s" in source
    assert "lateral_sweep_requested" in source
    assert "def triangle_wave(elapsed_s, period_s, amplitude_m):" in mission
    assert "lateral_reference = args.lateral_bias_m + triangle_wave(" in mission
    # The safe-FOV guard remains the hard stop; the sweep only changes the
    # lateral reference the P-controller tracks, not the guard itself.
    assert "lateral_error = raw_lateral - lateral_reference" in mission
    assert "--lateral-sweep-amplitude-m" in mission_runner
    assert "--lateral-sweep-period-s" in mission_runner
    assert "geometry_index % ${#altitudes_m[@]}" in source
    assert "geometry_index % ${#lateral_biases_m[@]}" in source


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
    assert "intrinsic_profile_sha256" in source
    assert "rig_profile_sha256" not in source  # Effective config uses rig.profile_sha256.
    assert '"profile_sha256": rig_sha256' in source
    assert '"hash_algorithm": "sha256"' in source


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
        "SWEEP_ALTITUDES_M",
        "SWEEP_LATERAL_BIASES_M",
        "SWEEP_LATERAL_SWEEP_AMPLITUDE_M",
        "SWEEP_LATERAL_SWEEP_PERIOD_S",
    ):
        assert name in source
        assert name in config


def test_auto_mission_phase_is_recorded_without_using_gazebo_gt_as_runtime_input():
    mission = (ROOT / "scripts" / "px4_auto_mission.py").read_text(encoding="utf-8")
    package = ROOT / "stereo_ground_sim" / "stereo_ground_sim"
    node = (package / "stereo_distance_node.py").read_text(encoding="utf-8")
    pair_results = (package / "pair_result_coordinator.py").read_text(encoding="utf-8")
    assert '"/stereo/auto_mission_phase"' in mission
    assert 'publish_phase("takeoff")' in mission
    assert 'publish_phase("approach")' in mission
    assert 'publish_phase("landing")' in mission
    assert '"/stereo/auto_mission_phase"' in node
    assert '"uam_raw_gps_reference_world_enu_m"' in node + pair_results
