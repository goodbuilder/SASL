from __future__ import annotations

from pathlib import Path


def test_operator_launcher_discovers_machine_local_yolo_weights_before_bare_fallback() -> None:
    source = (Path(__file__).resolve().parents[1] / "run_operator_view.sh").read_text(encoding="utf-8")

    assert '"$ROOT_DIR/$candidate"' in source
    assert '"$ROOT_DIR/../$candidate"' in source
    assert '"$HOME/Downloads/$candidate"' in source
    assert '"$HOME/다운로드/$candidate"' in source


def test_operator_launcher_auto_selects_a_kconfiglib_capable_px4_python() -> None:
    source = (Path(__file__).resolve().parents[1] / "run_operator_view.sh").read_text(encoding="utf-8")

    assert '"$ROOT_DIR/.venv-px4-local/bin/python"' in source
    assert '"$ROOT_DIR/.venv-px4-v1.17.0/bin/python"' in source
    assert "-c 'import kconfiglib'" in source


def test_cleanup_kills_rqt_process_group_even_after_its_wrapper_exits() -> None:
    source = (Path(__file__).resolve().parents[1] / "run_operator_view.sh").read_text(encoding="utf-8")

    assert "Signal the recorded process group even if that original wrapper PID has" in source
    assert 'kill -KILL "-$pid" >/dev/null 2>&1 || kill -KILL "$pid"' in source


def test_mission_launcher_selects_a_pymavlink_capable_python() -> None:
    source = (Path(__file__).resolve().parents[1] / "run_mission.sh").read_text(encoding="utf-8")

    assert "resolve_pymavlink_python" in source
    assert '"$ROOT_DIR/.venv-px4-local/bin/python"' in source
    assert '"$ROOT_DIR/.venv-px4-v1.17.0/bin/python"' in source
    assert "-c 'import pymavlink'" in source


def test_auto_launcher_does_not_start_recording_and_enables_fov_guard() -> None:
    source = (Path(__file__).resolve().parents[1] / "run_auto.sh").read_text(encoding="utf-8")

    assert '"$ROOT_DIR/run_mission.sh"' in source
    assert "--fov-guard" in source
    assert "--bbox-margin-ratio" in source
    assert "--fov-detection-timeout-s" in source
    assert "Save CSV or Save MP4 while Auto is running" in source
    assert "operator_recorder" not in source
    assert "--record-dir" not in source


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
