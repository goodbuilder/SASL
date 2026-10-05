from __future__ import annotations

import importlib.util
from pathlib import Path
import sys
import xml.etree.ElementTree as ET

import pytest

PACKAGE_ROOT = Path(__file__).resolve().parents[1] / "stereo_ground_sim"
from launch import LaunchContext
from launch.actions import DeclareLaunchArgument


sys.path.insert(0, str(PACKAGE_ROOT))


def _launch_module():
    path = PACKAGE_ROOT / "launch" / "stereo_ground_sim.launch.py"
    spec = importlib.util.spec_from_file_location("s2_r3_static_target_launch", path)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _default_context(module, *, static_target: bool) -> LaunchContext:
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
    context.launch_configurations.update(
        {
            "target_static_for_research": "true" if static_target else "false",
            "target_spawn_x_m": "123.25",
            "target_spawn_y_m": "456.5",
            "target_spawn_z_m": "7.75",
        }
    )
    return context


def _target_include(root: ET.Element) -> ET.Element:
    matches = [
        include
        for include in root.findall("./world/include")
        if include.findtext("uri") == "model://x500_15m"
    ]
    assert len(matches) == 1
    return matches[0]


@pytest.mark.parametrize(("enabled", "expected"), ((False, "false"), (True, "true")))
def test_generated_world_changes_only_research_target_static_contract(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch, enabled: bool, expected: str
) -> None:
    module = _launch_module()
    monkeypatch.setattr(module, "get_package_share_directory", lambda _: str(PACKAGE_ROOT))
    monkeypatch.setattr(module.tempfile, "gettempdir", lambda: str(tmp_path))
    generated = module._generate_world(_default_context(module, static_target=enabled))
    root = ET.parse(generated).getroot()
    target = _target_include(root)
    assert target.findtext("static") == expected
    assert tuple(float(value) for value in target.findtext("pose").split()[:3]) == pytest.approx(
        (123.25, 456.5, 7.75), abs=1e-12
    )
    cameras = {
        model.attrib["name"]: model.findtext("static")
        for model in root.findall("./world/model")
        if model.attrib.get("name") in {"left_stereo_camera", "right_stereo_camera"}
    }
    assert cameras == {"left_stereo_camera": "true", "right_stereo_camera": "true"}
    assert root.findtext("./world/gravity").strip() == "0 0 -9.8"


def test_target_static_transform_rejects_missing_duplicate_and_malformed_contract() -> None:
    module = _launch_module()
    valid = "<sdf><world><include><uri>model://target</uri><static>false</static></include></world></sdf>"
    assert "<static>true</static>" in module._set_target_include_static(
        valid, target_model_name="target", static_enabled=True
    )
    for invalid in (
        "<sdf><world /></sdf>",
        "<sdf><world><include><uri>model://target</uri><static>false</static></include>"
        "<include><uri>model://target</uri><static>false</static></include></world></sdf>",
        "<sdf><world><include><uri>model://target</uri></include></world></sdf>",
        "<sdf><world><include><uri>model://target</uri><static>maybe</static></include></world></sdf>",
    ):
        with pytest.raises(ValueError):
            module._set_target_include_static(
                invalid, target_model_name="target", static_enabled=True
            )
