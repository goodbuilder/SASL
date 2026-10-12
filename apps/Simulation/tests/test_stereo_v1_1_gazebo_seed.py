"""Manifest seed reaches Gazebo CLI only when explicitly supplied."""

from __future__ import annotations

import importlib.util
from pathlib import Path

import pytest
from launch import LaunchContext
from launch.actions import DeclareLaunchArgument


SIM_ROOT = Path(__file__).resolve().parents[1]


def _launch():
    path = SIM_ROOT / "stereo_ground_sim" / "launch" / "stereo_ground_sim.launch.py"
    spec = importlib.util.spec_from_file_location("v1_1_seed_launch", path)
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(module)
    context = LaunchContext()
    for action in module.generate_launch_description().entities:
        if isinstance(action, DeclareLaunchArgument):
            context.launch_configurations[action.name] = "".join(
                value.perform(context) for value in action.default_value
            )
    return module, context


def test_unset_seed_preserves_old_gazebo_command_and_manifest_seed_is_explicit():
    module, context = _launch()
    context.launch_configurations["gz_gui"] = "false"
    assert module._gazebo_args(context, "/tmp/world.sdf", "/tmp/gui.config") == "-r -s /tmp/world.sdf"
    context.launch_configurations["gazebo_seed"] = "2026100501"
    assert module._gazebo_args(context, "/tmp/world.sdf", "/tmp/gui.config") == (
        "-r -s --seed 2026100501 /tmp/world.sdf"
    )
    for invalid in ("NaN", "-1", "4294967296"):
        context.launch_configurations["gazebo_seed"] = invalid
        with pytest.raises(ValueError, match="unsigned 32-bit"):
            module._gazebo_args(context, "/tmp/world.sdf", "/tmp/gui.config")
