from __future__ import annotations

from pathlib import Path


def test_operator_launcher_discovers_machine_local_yolo_weights_before_bare_fallback() -> None:
    source = (Path(__file__).resolve().parents[1] / "run_operator_view.sh").read_text(encoding="utf-8")

    assert '"$ROOT_DIR/$candidate"' in source
    assert '"$ROOT_DIR/../$candidate"' in source
    assert '"$HOME/Downloads/$candidate"' in source
    assert '"$HOME/다운로드/$candidate"' in source
    assert 'bare_fallback=""' in source
    assert '"$INTEGRATION_ROOT/.local/legacy/Simulation/yolov8n_uav.pt"' in source
    assert source.index('if [[ -n "$bare_fallback" ]]') > source.index("for candidate in \"$@\"")


def test_operator_launcher_auto_selects_a_kconfiglib_capable_px4_python() -> None:
    source = (Path(__file__).resolve().parents[1] / "run_operator_view.sh").read_text(encoding="utf-8")

    assert '"$ROOT_DIR/.venv-px4-local/bin/python"' in source
    assert '"$ROOT_DIR/.venv-px4-v1.17.0/bin/python"' in source
    assert "-c 'import kconfiglib'" in source


def test_operator_recorder_video_fps_matches_the_overlay_publish_rate() -> None:
    # operator_recorder writes MP4 frames from detection_image topics, which
    # publish at overlay_publish_hz -- not at --video-fps's own 20.0 default.
    # A mismatch there compresses the recorded MP4 into a fraction of the
    # real session length (cv2.VideoWriter assumes frames arrive at
    # --video-fps). run_operator_view.sh must pass the actual configured
    # overlay_publish_hz through so playback speed matches wall-clock time.
    source = (Path(__file__).resolve().parents[1] / "run_operator_view.sh").read_text(encoding="utf-8")

    assert (
        'overlay_publish_hz_value="$(get_launch_arg_value overlay_publish_hz "${launch_args[@]}" || true)"'
        in source
    )
    assert 'recorder_cmd+=(--video-fps "$overlay_publish_hz_value")' in source
    # Must be wired up after launch_args is fully resolved (defaults merged
    # in), not the raw launch_input the caller passed.
    assert source.index('overlay_publish_hz_value="$(get_launch_arg_value') > source.index(
        'launch_args+=("${launch_input[@]}")'
    )


def test_operator_launcher_can_request_an_isolated_yolo_runtime() -> None:
    source = (Path(__file__).resolve().parents[1] / "run_operator_view.sh").read_text(
        encoding="utf-8"
    )
    assert "YOLO_VENV_ISOLATED" in source
    assert "ensure_yolo_args+=(--isolated)" in source


def test_resolved_launch_rates_are_exported_to_recording_provenance() -> None:
    source = (Path(__file__).resolve().parents[1] / "run_operator_view.sh").read_text(
        encoding="utf-8"
    )

    for name, launch_name in (
        ("CAMERA_UPDATE_RATE_HZ", "camera_update_rate_hz"),
        ("YOLO_TARGET_HZ", "yolo_target_hz"),
        ("YOLO_IMGSZ", "yolo_imgsz"),
        ("YOLO_DEVICE", "yolo_device"),
        ("OVERLAY_PUBLISH_HZ", "overlay_publish_hz"),
    ):
        assert f'export {name}="$(get_launch_arg_value {launch_name}' in source


def test_operator_launcher_exposes_overlay_rate_as_a_first_class_cli_option() -> None:
    source = (Path(__file__).resolve().parents[1] / "run_operator_view.sh").read_text(
        encoding="utf-8"
    )

    assert "--overlay-hz VALUE" in source
    assert '--overlay-hz)' in source
    assert 'DEFAULT_OVERLAY_HZ="${2:?missing value for --overlay-hz}"' in source


def test_operator_launcher_fails_closed_when_requested_yolo_runtime_is_unavailable() -> None:
    source = (Path(__file__).resolve().parents[1] / "run_operator_view.sh").read_text(
        encoding="utf-8"
    )

    assert "refusing an unverified CPU/system-Python fallback" in source
    failed_prepare = source.index("error: failed to prepare the requested YOLO runtime")
    assert "exit 2" in source[failed_prepare : failed_prepare + 400]


def test_operator_recorder_is_not_coupled_to_the_isolated_yolo_python() -> None:
    source = (Path(__file__).resolve().parents[1] / "run_operator_view.sh").read_text(
        encoding="utf-8"
    )

    assert "resolve_operator_recorder_python" in source
    assert "'import cv2, xlsxwriter'" in source
    assert source.index("/usr/bin/python3") < source.index('"$ROOT_DIR/.venv/bin/python"', source.index("resolve_operator_recorder_python"))


def test_cleanup_kills_rqt_process_group_even_after_its_wrapper_exits() -> None:
    source = (Path(__file__).resolve().parents[1] / "run_operator_view.sh").read_text(encoding="utf-8")

    assert "Signal the recorded process group even if that original wrapper PID has" in source
    assert 'kill -KILL "-$pid" >/dev/null 2>&1 || kill -KILL "$pid"' in source


def test_cleanup_waits_for_long_operator_recording_finalization() -> None:
    source = (Path(__file__).resolve().parents[1] / "run_operator_view.sh").read_text(
        encoding="utf-8"
    )

    assert "for _ in {1..180}; do" in source
    assert '"state"[[:space:]]*:[[:space:]]*"finalized"' in source
    assert '"${record_dir}/progress.json"' in source


def test_mission_launcher_selects_a_pymavlink_capable_python() -> None:
    source = (Path(__file__).resolve().parents[1] / "run_mission.sh").read_text(encoding="utf-8")

    assert "resolve_pymavlink_python" in source
    assert '"$ROOT_DIR/.venv-px4-local/bin/python"' in source
    assert '"$ROOT_DIR/.venv-px4-v1.17.0/bin/python"' in source
    assert "-c 'import pymavlink'" in source


def test_auto_launcher_starts_full_recording_and_enables_fov_guard() -> None:
    source = (Path(__file__).resolve().parents[1] / "run_auto.sh").read_text(encoding="utf-8")

    assert '"$ROOT_DIR/run_mission.sh"' in source
    assert "--fov-guard" in source
    assert "--bbox-margin-ratio" in source
    assert "--fov-detection-timeout-s" in source
    assert "Recording Auto Full session to:" in source
    assert '--px4-origin-world-east-m "${DRONE_SPAWN_X_M:-0.0}"' in source
    assert '--px4-origin-world-north-m "${DRONE_SPAWN_Y_M:-0.0}"' in source
    assert '--px4-origin-world-up-m "${DRONE_SPAWN_Z_M:-0.0}"' in source
    assert '"$ROOT_DIR/Recordings/auto_$(date +%Y%m%d_%H%M%S)_Full"' in source
    assert "stereo_ground_sim.operator_recorder" in source
    assert "--mode full" in source
    assert '--output-dir "$waypoints_dir"' in source
    assert 'kill -INT -- "-$record_pid"' in source


def test_runtime_outputs_share_recordings_root() -> None:
    root = Path(__file__).resolve().parents[1]
    operator_source = (root / "run_operator_view.sh").read_text(encoding="utf-8")
    waypoint_source = (root / "run_px4_waypoint_mission.sh").read_text(encoding="utf-8")
    panel_source = (root / "stereo_ground_sim" / "stereo_ground_sim" / "recorder_panel.py").read_text(
        encoding="utf-8"
    )

    assert '"$ROOT_DIR/Recordings/operator_$(date +%Y%m%d_%H%M%S)_$record_mode"' in operator_source
    assert 'export OPERATOR_BAG_DIR="${OPERATOR_BAG_DIR:-$ROOT_DIR/Recordings}"' in operator_source
    assert '"$ROOT_DIR/Recordings/waypoint_$(date +%Y%m%d_%H%M%S)_full"' in waypoint_source
    assert 'Path.cwd() / "Recordings"' in panel_source


def test_operator_launcher_prefers_repo_local_rqt_python_sources() -> None:
    source = (Path(__file__).resolve().parents[1] / "run_operator_view.sh").read_text(encoding="utf-8")

    assert 'RQT_PYTHONPATH="$ROOT_DIR/python_patches:$ROOT_DIR/stereo_ground_sim' in source


def test_operator_launcher_can_reuse_existing_install_when_colcon_build_is_unavailable() -> None:
    source = (Path(__file__).resolve().parents[1] / "run_operator_view.sh").read_text(encoding="utf-8")

    assert "--skip-colcon-build" in source
    assert 'skip_colcon_build=true' in source
    assert "Skipping colcon build" in source


def test_operator_launcher_builds_simulation_interfaces_before_python_nodes() -> None:
    source = (Path(__file__).resolve().parents[1] / "run_operator_view.sh").read_text(encoding="utf-8")

    assert '--base-paths "$ROOT_DIR"' in source
    assert "--packages-select simulation_interfaces stereo_ground_sim stereo_ground_sim_overlay" in source
    assert 'operator_colcon_build_base="$(mktemp -d /tmp/sasl-operator-colcon-build.XXXXXX)"' in source
    assert '--build-base "$operator_colcon_build_base"' in source
