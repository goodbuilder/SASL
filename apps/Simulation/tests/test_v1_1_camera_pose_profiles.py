"""Protocol v1.1 physical optical pose, checked at the generated SDF boundary."""

from __future__ import annotations

import importlib.util
import json
import math
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

import pytest
from launch import LaunchContext
from launch.actions import DeclareLaunchArgument


SIM_ROOT = Path(__file__).resolve().parents[1]
PACKAGE_ROOT = SIM_ROOT / "stereo_ground_sim"
sys.path.insert(0, str(PACKAGE_ROOT))

from stereo_ground_sim.coordinate_frames import rpy_to_rotation_world_from_link


def _launch_module():
    path = PACKAGE_ROOT / "launch" / "stereo_ground_sim.launch.py"
    spec = importlib.util.spec_from_file_location("v1_1_camera_pose_launch", path)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _profile(group: str) -> dict[str, str]:
    path = SIM_ROOT / "config" / "campaign" / "v1_1" / f"{group}.env"
    values = {}
    for line in path.read_text(encoding="utf-8").splitlines():
        line = line.strip()
        if line and not line.startswith("#"):
            key, value = line.split("=", 1)
            values[key] = value
    return values


def _context(module, profile: dict[str, str]) -> LaunchContext:
    context = LaunchContext()
    for entity in module.generate_launch_description().entities:
        if isinstance(entity, DeclareLaunchArgument):
            context.launch_configurations[entity.name] = "".join(
                item.perform(context) for item in entity.default_value
            )
    mapping = {
        "CAMERA_BASELINE_M": "baseline_m",
        "CAMERA_HEIGHT_M": "camera_height_m",
        "CAMERA_HORIZONTAL_FOV_RAD": "horizontal_fov_rad",
        "CAMERA_IMAGE_WIDTH_PX": "camera_image_width_px",
        "CAMERA_IMAGE_HEIGHT_PX": "camera_image_height_px",
        "CAMERA_UPDATE_RATE_HZ": "camera_update_rate_hz",
    }
    for key, value in profile.items():
        name = mapping.get(key, key.lower())
        if key == "SIM_SCENARIO_NAME":
            name = "scenario_name"
        context.launch_configurations[name] = value
    # A camera-mounted PX4 body pose may differ from the optical pose.
    context.launch_configurations["cam_l_px4_attitude_pitch_deg"] = "8.0"
    context.launch_configurations["cam_r_px4_attitude_pitch_deg"] = "8.0"
    return context


def _matmul(left, right):
    return [
        [sum(left[i][k] * right[k][j] for k in range(3)) for j in range(3)]
        for i in range(3)
    ]


@pytest.mark.parametrize("group", ["G0", "G1"])
def test_physical_camera_pose_matches_protocol_world_heading(group, tmp_path, monkeypatch):
    spec = json.loads((SIM_ROOT / "작업지시서" / "03_experiment_spec.json").read_text())
    module = _launch_module()
    monkeypatch.setattr(module, "get_package_share_directory", lambda _: str(PACKAGE_ROOT))
    monkeypatch.setattr(module.tempfile, "gettempdir", lambda: str(tmp_path))
    profile = _profile(group)
    generated = module._generate_world(_context(module, profile))
    root = ET.parse(generated).getroot()
    R0 = spec["coordinates"]["R_WC_nominal"]
    group_headings = spec["camera"]["physical_pose_groups"][group]

    for side in ("left", "right"):
        model = root.find(f"./world/model[@name='{side}_stereo_camera']")
        assert model is not None
        model_xyzrpy = [float(v) for v in model.findtext("pose").split()]
        optical = model.find("./link[@name='optical_link']")
        assert optical is not None
        relative_xyzrpy = [float(v) for v in optical.findtext("pose").split()]
        assert model_xyzrpy[:3] == pytest.approx(
            spec["camera"][f"{side}_center_world_m"], abs=1e-12
        )
        assert relative_xyzrpy[:3] == pytest.approx([0, 0, 0], abs=1e-12)

        body_R = rpy_to_rotation_world_from_link(*model_xyzrpy[3:])
        mount_R = rpy_to_rotation_world_from_link(*relative_xyzrpy[3:])
        link_R = _matmul(body_R, mount_R)
        # Link +X/+Y/+Z represent optical Forward/-Right/-Down.
        optical_R = [
            [-link_R[i][1], -link_R[i][2], link_R[i][0]] for i in range(3)
        ]
        psi = math.radians(group_headings[f"heading_{side}_deg"])
        world_heading_R = [
            [math.cos(psi), math.sin(psi), 0],
            [-math.sin(psi), math.cos(psi), 0],
            [0, 0, 1],
        ]
        expected_R = _matmul(world_heading_R, R0)
        for actual_row, expected_row in zip(optical_R, expected_R):
            assert actual_row == pytest.approx(expected_row, abs=1e-9)
        assert max(
            abs(body_R[i][j] - link_R[i][j]) for i in range(3) for j in range(3)
        ) > 0.01
