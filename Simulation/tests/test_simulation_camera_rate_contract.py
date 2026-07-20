from __future__ import annotations

import re
import xml.etree.ElementTree as ET
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
WORLD = ROOT / "stereo_ground_sim" / "worlds" / "stereo_ground_drone.sdf"
LAUNCH = ROOT / "stereo_ground_sim" / "launch" / "stereo_ground_sim.launch.py"
CAMERA_ENV = ROOT / "config" / "camera_model.env"


def test_visible_camera_sdf_is_1280x720_at_20_hz() -> None:
    root = ET.parse(WORLD).getroot()
    cameras = root.findall(".//sensor[@type='camera']")
    assert len(cameras) == 2
    for sensor in cameras:
        assert sensor.findtext("update_rate") == "20"
        assert sensor.findtext("camera/image/width") == "1280"
        assert sensor.findtext("camera/image/height") == "720"


def test_camera_defaults_and_frame_interval_are_20_hz() -> None:
    env = CAMERA_ENV.read_text(encoding="utf-8")
    assert re.search(r"^CAMERA_IMAGE_WIDTH_PX=1280$", env, re.MULTILINE)
    assert re.search(r"^CAMERA_IMAGE_HEIGHT_PX=720$", env, re.MULTILINE)
    assert re.search(r"^CAMERA_UPDATE_RATE_HZ=20\.0$", env, re.MULTILINE)
    launch = LAUNCH.read_text(encoding="utf-8")
    assert 'DeclareLaunchArgument("camera_update_rate_hz", default_value="20.0")' in launch
    assert "50,000,000" not in launch  # runtime interval is implied by 20 Hz, not a timer


def test_active_camera_rate_has_no_30_or_60_default() -> None:
    sources = [WORLD.read_text(encoding="utf-8"), LAUNCH.read_text(encoding="utf-8"), CAMERA_ENV.read_text(encoding="utf-8")]
    joined = "\n".join(sources)
    assert "<update_rate>60</update_rate>" not in joined
    assert "<update_rate>30</update_rate>" not in joined
    assert "CAMERA_UPDATE_RATE_HZ=30" not in joined
    assert 'DeclareLaunchArgument("camera_update_rate_hz", default_value="60.0")' not in joined
