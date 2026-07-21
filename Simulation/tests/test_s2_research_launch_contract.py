from __future__ import annotations

import importlib.util
import sys
import threading
from pathlib import Path
from types import SimpleNamespace

import yaml
from launch import LaunchContext
from launch.actions import DeclareLaunchArgument

ROOT = Path(__file__).resolve().parents[1]
PACKAGE_ROOT = ROOT / "stereo_ground_sim"
sys.path.insert(0, str(PACKAGE_ROOT))

from stereo_ground_sim.stereo_distance_node import StereoDistanceNode


RECORDING_DEFAULTS = {
    "enable_research_recording": "false",
    "research_output_dir": "",
    "research_run_id": "",
    "research_case_id": "",
    "research_repository": "",
    "research_branch": "",
    "research_commit_sha": "",
    "research_git_status": "",
    "research_gazebo_version": "",
    "research_px4_version": "not_applicable",
    "research_record_queue_capacity": "1024",
}


def _launch_module():
    path = PACKAGE_ROOT / "launch" / "stereo_ground_sim.launch.py"
    spec = importlib.util.spec_from_file_location("stereo_ground_sim_launch_r2_test", path)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _default_context(module):
    context = LaunchContext()
    declarations = {
        entity.name: entity
        for entity in module.generate_launch_description().entities
        if isinstance(entity, DeclareLaunchArgument)
    }
    for name, declaration in declarations.items():
        context.launch_configurations[name] = "".join(
            substitution.perform(context) for substitution in declaration.default_value
        )
    return context, declarations


def test_launch_declares_disabled_recording_and_camera_sampling_defaults() -> None:
    module = _launch_module()
    context, declarations = _default_context(module)
    for name, expected in RECORDING_DEFAULTS.items():
        assert name in declarations
        assert context.launch_configurations[name] == expected
    assert context.launch_configurations["camera_image_width_px"] == "1280"
    assert context.launch_configurations["camera_image_height_px"] == "720"
    assert context.launch_configurations["camera_update_rate_hz"] == "20.0"


def test_launch_writes_all_recording_parameters_with_native_types(tmp_path: Path, monkeypatch) -> None:
    module = _launch_module()
    context, _declarations = _default_context(module)
    context.launch_configurations.update(
        {
            "enable_research_recording": "true",
            "research_output_dir": str(tmp_path / "run"),
            "research_run_id": "run-id",
            "research_case_id": "case-id",
            "research_repository": "goodbuilder/SASL",
            "research_branch": "sim/r2",
            "research_commit_sha": "c" * 40,
            "research_git_status": "clean",
            "research_gazebo_version": "8.0",
            "research_px4_version": "not_applicable",
            "research_record_queue_capacity": "2048",
        }
    )
    params_path = tmp_path / "params.yaml"
    monkeypatch.setattr(module.tempfile, "gettempdir", lambda: str(tmp_path))
    actual_path = Path(module._write_distance_node_params(context))
    assert actual_path == params_path.with_name("stereo_distance_node_params.yaml")
    params = yaml.safe_load(actual_path.read_text(encoding="utf-8"))[
        "/stereo_distance_node"
    ]["ros__parameters"]
    assert params["enable_research_recording"] is True
    assert params["research_record_queue_capacity"] == 2048
    assert params["camera_image_width_px"] == 1280
    assert params["camera_image_height_px"] == 720
    assert params["camera_update_rate_hz"] == 20.0
    assert params["research_commit_sha"] == "c" * 40
    assert params["research_output_dir"] == str(tmp_path / "run")


def test_disabled_node_initialization_creates_no_thread_or_files(tmp_path: Path) -> None:
    node = StereoDistanceNode.__new__(StereoDistanceNode)
    node.enable_research_recording = False
    node.research_output_dir = str(tmp_path / "disabled")
    node.research_recorder = None
    before = {thread.ident for thread in threading.enumerate()}
    node._initialize_research_recording()
    after = {thread.ident for thread in threading.enumerate()}
    assert node.research_recorder is None
    assert before == after
    assert not (tmp_path / "disabled").exists()


def test_recording_and_sampling_parameters_are_batch_immutable() -> None:
    node = StereoDistanceNode.__new__(StereoDistanceNode)
    result = node._on_param_update(
        [
            SimpleNamespace(name="mission_profile", value="changed"),
            SimpleNamespace(name="research_run_id", value="changed"),
            SimpleNamespace(name="camera_update_rate_hz", value=30.0),
        ]
    )
    assert result.successful is False
    assert "immutable" in result.reason
