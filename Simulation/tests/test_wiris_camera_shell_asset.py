from __future__ import annotations

import struct
import xml.etree.ElementTree as ET
from pathlib import Path


ROOT_DIR = Path(__file__).resolve().parents[1]
WORLD_PATH = ROOT_DIR / "stereo_ground_sim" / "worlds" / "stereo_ground_drone.sdf"
ASSET_PATH = (
    ROOT_DIR
    / "stereo_ground_sim"
    / "models"
    / "wiris_camera"
    / "meshes"
    / "WIRIS.stl"
)


def test_wiris_shell_asset_is_packaged_and_sensor_pose_is_unchanged() -> None:
    assert ASSET_PATH.is_file()
    assert ASSET_PATH.stat().st_size > 1_000_000
    with ASSET_PATH.open("rb") as handle:
        header = handle.read(80)
        triangle_count = struct.unpack("<I", handle.read(4))[0]
    assert header.startswith(b"SASL WIRIS Enterprise binary STL")
    assert ASSET_PATH.stat().st_size == 84 + triangle_count * 50

    root = ET.parse(WORLD_PATH).getroot()
    camera_models = {
        model.attrib["name"]: model
        for model in root.findall(".//model")
        if model.attrib.get("name") in {"left_stereo_camera", "right_stereo_camera"}
    }
    assert set(camera_models) == {"left_stereo_camera", "right_stereo_camera"}

    for model in camera_models.values():
        shell = model.find("./link/visual[@name='wiris_shell']")
        assert shell is not None
        assert shell.findtext("./geometry/mesh/uri") == "model://wiris_camera/meshes/WIRIS.stl"
        assert shell.findtext("./geometry/mesh/scale") == "0.001 0.001 0.001"
        assert shell.findtext("./pose") == "-0.5185 0.051 -0.038103205 0 0 0"

        sensor = model.find("./link/sensor[@name='camera']")
        assert sensor is not None
        assert sensor.findtext("./pose") == "0 0 0 0 0 0"
        assert sensor.findtext("./topic") in {"/left_camera/image", "/right_camera/image"}
