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


def test_wiris_shell_asset_is_packaged_and_visual_yaw_does_not_change_sensor_pose() -> None:
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
        optical_link = model.find("./link[@name='optical_link']")
        assert optical_link is not None
        shell = optical_link.find("./visual[@name='wiris_shell']")
        assert shell is not None
        assert shell.findtext("./geometry/mesh/uri") == "model://wiris_camera/meshes/WIRIS.stl"
        assert shell.findtext("./geometry/mesh/scale") == "0.001 0.001 0.001"
        # This keeps the mesh origin at the optical-frame origin and compensates
        # only the STL's authored +Y forward axis.  The camera sensor remains at
        # that same origin, so its image geometry and PX4/body attachment are
        # unaffected by the visual correction.
        assert shell.findtext("./pose") == "0 0 0 0 0 1.570796327"

        sensor = optical_link.find("./sensor[@name='camera']")
        assert sensor is not None
        assert sensor.findtext("./pose") == "0 0 0 0 0 0"
        assert sensor.findtext("./topic") in {"/left_camera/image", "/right_camera/image"}
