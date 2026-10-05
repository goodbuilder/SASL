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
    assert "--baseline-only" in result.stdout


def test_auto_attitude_sweep_preserves_interactive_auto_recording_contract():
    source = RUNNER.read_text(encoding="utf-8")
    auto_source = (ROOT / "run_auto.sh").read_text(encoding="utf-8")
    assert '"enable_research_recording:=true"' in source
    assert '"research_output_dir:=$recording_dir"' in source
    assert 'effective_config.json' in source
    assert 'research_effective_config_sha256:=$effective_config_sha256' in source
    assert 'research_session_id:=$session_id' in source
    assert 'research_attempt_id:=$case_id' in source
    assert "stereo_ground_sim.operator_recorder" in auto_source
    assert "--mode full" in auto_source
    assert 'auto_$(date +%Y%m%d_%H%M%S)_Full' in auto_source
    assert "run_auto_attitude_sweep.sh" in auto_source
    assert "SWEEP_RANDOMIZE" in source
    assert "baseline-before" not in source
    assert "baseline-after" not in source
    assert 'kill -TERM "$pid"' in source
    assert 'kill -INT "$pid"' not in source
    assert "--headless" not in source  # RViz/rqt_gui stay visible for every case.
    assert 'setsid env SIM_CONFIG="$SIM_CONFIG"' in source
    assert 'Operator stack exited during the mission' in source
    assert "validate_attitude_sweep_output.py" in source
    assert "SWEEP_FINALIZED_VALID" in source
    assert 'if [[ "$record_wait_for_bbox" == true ]]' in source
    assert 'operator_cmd+=(--record-wait-for-bbox)' in source
    assert '"require_operator_full_recording": True' in source
    assert "target_model_protected_geometry_sha256" in source
    # The --quick/--target-distance-m shim existed only to patch around
    # run_mission.sh --quick's stop distance conflicting with FOV-guard;
    # irrelevant now that the mission runner has no FOV concept at all.
    assert "quick_requested" not in source
    assert 'auto_args+=(--target-distance-m "${DRONE_TARGET_DISTANCE_M:-500.0}")' not in source


def test_auto_attitude_sweep_flies_one_shared_deterministic_waypoint_path():
    # Every attitude case within one trajectory must fly the identical
    # physical trajectory; center and zigzag are generated once each.
    source = RUNNER.read_text(encoding="utf-8")
    # Each trajectory is generated from baseline attitude before its cases.
    assert source.count("scripts/generate_fov_constrained_waypoints.py") == 1
    assert 'for trajectory in "${trajectories[@]}"; do' in source
    assert 'shared_path_files["$trajectory"]' in source
    assert '"$trajectory_dir/validation_mission.csv"' in source
    assert '--range-count "$waypoint_count"' in source
    assert '--lateral-pattern "$trajectory"' in source
    # Per-case invocation: the deterministic, camera/FOV-independent MAVLink
    # waypoint follower -- no --fov-guard concept exists for it to take.
    assert '"$ROOT_DIR/run_px4_waypoint_mission.sh"' in source
    assert '--path-file "$shared_path_file"' in source
    assert "--no-record" in source
    assert '--hold-s "$waypoint_hold_s"' in source
    assert "--ramp-s 0" in source
    assert "--vertical-ramp-s 0" in source
    assert '--guidance-log "$guidance_trace_log"' in source
    assert '--guidance-mode "$guidance_mode"' in source
    assert '"phase_pattern": "waypoint_{index:03d}_hold_analysis"' in source
    assert '"primary_estimate_field": "nominal_pose_range_m"' in source
    assert '"ground_truth_field": "gt_range_m"' in source
    assert 'guidance_mode="${SWEEP_GUIDANCE_MODE:-velocity_only}"' in source
    assert "--fov-guard" not in source
    assert '"$ROOT_DIR/run_auto.sh"' not in source
    assert '"$ROOT_DIR/run_px4_waypoint_mission.sh" --check-connection' in source


def test_auto_attitude_sweep_supports_combined_axis():
    source = RUNNER.read_text(encoding="utf-8")
    assert "roll,pitch,yaw,combined" in source
    assert 'combined) roll_offset="$angle"; pitch_offset="$angle"; yaw_offset="$angle" ;;' in source
    # Sweep angles are errors added to the configured right-camera baseline
    # (CAM_R_OPTICAL_*_DEG), not absolute attitudes -- see CAM_L/R_OPTICAL_*
    # wiring below. The left camera stays at its own configured baseline.
    assert 'left_roll="${CAM_L_OPTICAL_ROLL_DEG:-0.0}"' in source
    assert 'roll="$(awk -v b="${CAM_R_OPTICAL_ROLL_DEG:-0.0}" -v o="$roll_offset"' in source
    assert '"cam_l_optical_roll_deg:=$left_roll"' in source
    assert '"${case_auto_args[@]}"' in source
    # The FOV-guard waypoint path px4_auto_mission.py takes when run_auto.sh
    # supplies --path-file never reaches args.altitude_m/args.lateral_bias_m
    # (those only apply to the older straight-line approach), so the sweep
    # no longer pretends to vary them per case -- geometry_altitude_m,
    # geometry_lateral_bias_m, --altitudes-m, --lateral-biases-m, and their
    # SWEEP_* env vars were removed as dead configuration.
    for dead_token in (
        "--altitudes-m",
        "--lateral-biases-m",
        "--lateral-sweep-amplitude-m",
        "--lateral-sweep-period-s",
        "SWEEP_ALTITUDES_M",
        "SWEEP_LATERAL_BIASES_M",
        "SWEEP_LATERAL_SWEEP_AMPLITUDE_M",
        "SWEEP_LATERAL_SWEEP_PERIOD_S",
        "geometry_altitude_m",
        "geometry_lateral_bias_m",
        "geometry_lateral_sweep_amplitude_m",
        "geometry_index",
        "pair_index",
        "altitude_requested",
        "lateral_bias_requested",
        "lateral_sweep_requested",
    ):
        assert dead_token not in source, f"{dead_token!r} should have been removed as dead configuration"


def test_auto_attitude_sweep_run_case_takes_no_geometry_index():
    source = RUNNER.read_text(encoding="utf-8")
    assert 'local trajectory="$1" axis="$2" angle="$3" repetition="$4"' in source
    assert 'run_case "$trajectory" baseline 0 0' in source
    assert 'run_case "$trajectory" "$axis" "$value" "$repetition"' in source


def test_auto_attitude_sweep_can_limit_rate_smoke_to_baselines():
    source = RUNNER.read_text(encoding="utf-8")
    assert 'baseline_only="$(normalize_bool SWEEP_BASELINE_ONLY' in source
    assert '--baseline-only) baseline_only=true' in source
    assert 'if [[ "$baseline_only" != true ]]; then' in source
    assert '"baseline_only=$baseline_only"' in source


def test_auto_attitude_sweep_wires_measured_rate_acceptance_into_each_case():
    source = RUNNER.read_text(encoding="utf-8")

    assert 'overlay_publish_hz="${SWEEP_OVERLAY_PUBLISH_HZ:-$camera_update_rate_hz}"' in source
    assert '"overlay_publish_hz:=$overlay_publish_hz"' in source
    assert '"performance_acceptance": {' in source
    assert '"min_yolo_update_hz": float(min_yolo_update_hz)' in source
    assert '"max_yolo_inference_p95_ms": float(max_yolo_inference_p95_ms)' in source
    assert '"min_overlay_hz": float(min_overlay_hz)' in source


def test_auto_attitude_sweep_case_id_encodes_absolute_camera_attitude():
    # case_id is built inside run_case from the actual absolute L/R camera
    # attitude (not the axis+delta abstraction), so a directory listing is
    # self-describing without cross-referencing the sweep's baseline config.
    # "_" is the only structural separator; "-" and "." stay literal inside
    # a value (e.g. "y-0.2") since they never collide with a delimiter.
    source = RUNNER.read_text(encoding="utf-8")
    assert "format_deg() {" in source
    assert 'awk -v v="$1" \'BEGIN { printf "%g", v }\'' in source
    assert 'attitude_token() {' in source
    assert (
        "printf '%sr%sp%sy%s' \"$prefix\" \"$(format_deg \"$roll\")\" \"$(format_deg \"$pitch\")\" \"$(format_deg \"$yaw\")\""
        in source
    )
    assert "compute_offset_attitude() {" in source
    assert (
        'case_id="case-${trajectory}-$(printf \'%03d\' "$case_index")_$(attitude_token L "$left_roll" "$left_pitch" "$left_yaw")_'
        '$(attitude_token R "$roll" "$pitch" "$yaw")_repetition$(printf \'%02d\' "$repetition")"' in source
    )
    assert "format_value" not in source  # superseded by format_deg/attitude_token


def test_auto_attitude_sweep_baseline_runs_once_up_front():
    # Each trajectory has exactly one 0 deg baseline before its conditions.
    source = RUNNER.read_text(encoding="utf-8")
    assert source.count('run_case "$trajectory" baseline 0 0') == 1
    assert "completed_case=baseline" in source
    assert 'expected_case_count=%s' in source
    assert 'for trajectory in "${trajectories[@]}"; do' in source
    assert source.count('for condition_index in "${condition_order[@]}"; do') >= 2


def test_auto_mission_still_supports_lateral_sweep_for_the_straight_line_path():
    # --lateral-sweep-amplitude-m/-period-s remain a legitimate feature of
    # px4_auto_mission.py's older straight-line approach (used when no
    # --path-file/--waypoints-xy-alt is given); only run_auto_attitude_sweep.sh's
    # dead per-case wiring of them was removed.
    mission = (ROOT / "scripts" / "px4_auto_mission.py").read_text(encoding="utf-8")
    mission_runner = (ROOT / "run_mission.sh").read_text(encoding="utf-8")
    assert "--lateral-bias-m" in mission
    assert "--lateral-bias-m" in mission_runner
    assert "def triangle_wave(elapsed_s, period_s, amplitude_m):" in mission
    assert "lateral_reference = args.lateral_bias_m + triangle_wave(" in mission
    # The safe-FOV guard remains the hard stop; the sweep only changes the
    # lateral reference the P-controller tracks, not the guard itself.
    assert "lateral_error = raw_lateral - lateral_reference" in mission
    assert "--lateral-sweep-amplitude-m" in mission_runner
    assert "--lateral-sweep-period-s" in mission_runner


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
        "SWEEP_TRAJECTORIES",
        "SWEEP_WAYPOINT_COUNT",
        "SWEEP_WAYPOINT_HOLD_S",
        "SWEEP_FINAL_HOVER_S",
        "SWEEP_WAYPOINTS_RFU_M",
        "SWEEP_ZIGZAG_OVERLAP_RATIO",
        "SWEEP_PIXEL_MARGIN_PX",
        "SWEEP_ENTRY_MIN_WORLD_FRONT_M",
        "SWEEP_RECORD_WAIT_FOR_BBOX",
        "SWEEP_STABILITY_WINDOW_S",
        "SWEEP_STABILITY_POSITION_P95_MAX_M",
        "SWEEP_STABILITY_ATTITUDE_MAX_DEG",
        "SWEEP_GUIDANCE_MODE",
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
    package = ROOT / "stereo_ground_sim" / "stereo_ground_sim"
    node = (package / "stereo_distance_node.py").read_text(encoding="utf-8")
    pair_results = (package / "pair_result_coordinator.py").read_text(encoding="utf-8")
    assert '"/stereo/auto_mission_phase"' in mission
    assert 'publish_phase("takeoff")' in mission
    assert 'publish_phase("approach")' in mission
    assert 'publish_phase("landing")' in mission
    assert '"/stereo/auto_mission_phase"' in node
    assert '"uam_raw_gps_reference_world_enu_m"' in node + pair_results
