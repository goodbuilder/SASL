from __future__ import annotations

import xml.etree.ElementTree as ET
from pathlib import Path
import sys

import pytest

PACKAGE_ROOT = Path(__file__).resolve().parents[1] / "stereo_ground_sim"
if str(PACKAGE_ROOT) not in sys.path:
    sys.path.insert(0, str(PACKAGE_ROOT))

from stereo_ground_sim.coordinate_frames import (
    REFERENCE_OFFSET_PARENT_M,
    REFERENCE_OFFSET_PARENT_RPY_RAD,
    SOURCE_GAZEBO_ENTITY,
    VEHICLE_REFERENCE_DEFINITION,
    VEHICLE_REFERENCE_LINK,
    WORLD_FRAME,
)
from stereo_ground_sim.gazebo_ground_truth_adapter import (
    OUTPUT_TOPIC,
    POSE_TOPIC,
    PosePayloadError,
    PoseVState,
    parse_pose_v_payload,
)


ROOT = Path(__file__).resolve().parents[1]
MODEL = ROOT / "stereo_ground_sim" / "models" / "x500_15m" / "model.sdf"


def test_vehicle_reference_is_zero_offset_sdf_frame_not_physics_link() -> None:
    root = ET.parse(MODEL).getroot()
    model = root.find("model")
    assert model is not None
    frames = model.findall("frame[@name='vehicle_reference_link']")
    assert len(frames) == 1
    frame = frames[0]
    assert frame.attrib["attached_to"] == "base_link"
    assert (frame.findtext("pose") or "").split() == ["0", "0", "0", "0", "0", "0"]
    assert model.findall("link[@name='vehicle_reference_link']") == []


def test_reference_metadata_contract() -> None:
    assert VEHICLE_REFERENCE_LINK == "vehicle_reference_link"
    assert VEHICLE_REFERENCE_DEFINITION == "project_defined_vehicle_reference_at_base_link_origin"
    assert SOURCE_GAZEBO_ENTITY == "base_link"
    assert REFERENCE_OFFSET_PARENT_M == (0.0, 0.0, 0.0)
    assert REFERENCE_OFFSET_PARENT_RPY_RAD == (0.0, 0.0, 0.0)
    assert WORLD_FRAME == "gazebo_world_enu"
    assert POSE_TOPIC == "/world/stereo_ground_drone/pose/info"
    assert OUTPUT_TOPIC == "/simulation/ground_truth/vehicle_reference_link"


def _payload(stamp_sec: int, stamp_nsec: int, *, entity_id: int = 17) -> dict:
    return {
        "header": {"stamp": {"sec": stamp_sec, "nsec": stamp_nsec}},
        "pose": [
            {
                "name": "base_link",
                "id": entity_id,
                "position": {"x": 1.25, "y": -2.5, "z": 3.75},
                "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0},
            }
        ],
    }


def test_pose_v_parser_preserves_top_level_stamp_and_entity_identity() -> None:
    record = parse_pose_v_payload(_payload(12, 345))
    assert record.stamp_ns == 12_000_000_345
    assert record.entity_name == "base_link"
    assert record.entity_id == 17
    assert record.position_world_enu_m == (1.25, -2.5, 3.75)


def test_pose_v_state_requires_name_and_id_and_records_reset() -> None:
    state = PoseVState()
    assert state.consume(_payload(5, 0)) is not None
    assert state.consume(_payload(4, 999)) is not None
    assert state.reset_count == 1
    assert state.consume(_payload(5, 0, entity_id=18)) is None
    assert "changed" in state.unavailable_reason


def test_duplicate_authoritative_name_is_unavailable() -> None:
    payload = _payload(1, 0)
    payload["pose"].append(dict(payload["pose"][0]))
    with pytest.raises(PosePayloadError, match="duplicate"):
        parse_pose_v_payload(payload)
