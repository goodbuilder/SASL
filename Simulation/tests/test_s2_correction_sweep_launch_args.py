from __future__ import annotations

from decimal import Decimal
import importlib.util
from pathlib import Path
import sys


PACKAGE_ROOT = Path(__file__).resolve().parents[1] / "stereo_ground_sim"
sys.path.insert(0, str(PACKAGE_ROOT))

from stereo_ground_sim.correction_sweep import (
    ATTITUDE_AXES,
    CameraContract,
    build_launch_command,
    build_sweep_plan,
)


def _case():
    plan, cases = build_sweep_plan(
        sweep_id="sweep",
        commit_sha="a" * 40,
        branch="sim/test",
        repository_name="goodbuilder/SASL",
        attitude_mode="ofat",
        axis_ranges=(("common_optical_roll_error_deg", (Decimal("0.1"),)),),
        range_values=(Decimal("500"),),
        bearing_values=(Decimal("1"),),
        elevation_values=(Decimal("0.5"),),
        repetitions=1,
        seed=1,
        case_order="canonical",
        invalid_spatial_policy="fail",
        max_cases=4,
        contract=CameraContract(),
    )
    del plan
    return cases[1]


def _arguments(command):
    return dict(item.split(":=", 1) for item in command[4:] if ":=" in item)


def _cli_module():
    path = Path(__file__).resolve().parents[1] / "scripts" / "run_correction_search_sweep.py"
    name = "s2_r3_cli_launch_test"
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    sys.modules[name] = module
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return module


def test_command_builder_explicitly_selects_ideal_r2_and_provenance(tmp_path: Path) -> None:
    command = build_launch_command(
        repository_root=tmp_path,
        case=_case(),
        attempt_dir=tmp_path / "attempt-0001",
        sweep_id="sweep",
        repository_name="goodbuilder/SASL",
        branch="sim/test",
        commit_sha="a" * 40,
        gazebo_version="8.14.0",
    )
    assert command[:4] == ["ros2", "launch", "stereo_ground_sim", "stereo_ground_sim.launch.py"]
    args = _arguments(command)
    assert args["observation_mode"] == "ideal_reference_point"
    assert args["enable_research_recording"] == "true"
    assert Path(args["research_output_dir"]).is_absolute()
    assert args["research_commit_sha"] == "a" * 40
    assert args["research_git_status"] == "clean"
    assert args["research_px4_version"] == "not_applicable"
    assert "yolo_model_path" not in args


def test_command_builder_passes_camera_static_target_and_disables_ui(tmp_path: Path) -> None:
    command = build_launch_command(
        repository_root=tmp_path,
        case=_case(),
        attempt_dir=tmp_path / "attempt",
        sweep_id="s",
        repository_name="repo",
        branch="sim/test",
        commit_sha="b" * 40,
        gazebo_version="8",
    )
    args = _arguments(command)
    assert (args["camera_image_width_px"], args["camera_image_height_px"]) == ("1280", "720")
    assert args["camera_update_rate_hz"] == args["yolo_target_hz"] == "20.0"
    assert args["mission_profile"] == "manual"
    assert args["fly_target_in_gazebo"] == "false"
    assert args["gz_gui"] == args["launch_rviz"] == "false"
    assert args["record_calibration_csv"] == "false"
    assert float(args["target_spawn_x_m"]) == _case()["payload"]["spatial"]["target_world_east_m"]
    assert float(args["target_spawn_y_m"]) == _case()["payload"]["spatial"]["target_world_north_m"]
    assert float(args["target_spawn_z_m"]) == _case()["payload"]["spatial"]["target_world_up_m"]


def test_command_builder_passes_all_twelve_attitude_sources_including_zero(tmp_path: Path) -> None:
    args = _arguments(
        build_launch_command(
            repository_root=tmp_path,
            case=_case(),
            attempt_dir=tmp_path / "attempt",
            sweep_id="s",
            repository_name="repo",
            branch="sim/test",
            commit_sha="c" * 40,
            gazebo_version="8",
        )
    )
    assert set(ATTITUDE_AXES) <= set(args)
    assert float(args["common_optical_roll_error_deg"]) == 0.1
    assert sum(float(args[axis]) != 0.0 for axis in ATTITUDE_AXES) == 1


def test_process_launcher_uses_exact_cwd_owned_session_and_copied_environment(
    tmp_path: Path, monkeypatch
) -> None:
    module = _cli_module()
    captured = {}

    class FakePopen:
        pid = 123

        def __init__(self, command, **kwargs):
            captured["command"] = command
            captured.update(kwargs)

    monkeypatch.setattr(module.subprocess, "Popen", FakePopen)
    environment = {"TOKEN": "original"}
    process = module.ProcessLauncher().launch(
        ["command"],
        cwd=tmp_path,
        environment=environment,
        log_path=tmp_path / "simulation.log",
    )
    process.close_log()
    assert captured["cwd"] == tmp_path
    assert captured["start_new_session"] is True
    assert captured["env"] == environment
    assert captured["env"] is not environment
