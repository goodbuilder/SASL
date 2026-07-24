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
    pose_record_world_from_payload,
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
    assert POSE_TOPIC == "/model/x500_15m/pose"
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


def test_pose_v_parser_accepts_gazebo_sparse_zero_scalar_json() -> None:
    record = parse_pose_v_payload(
        {
            "header": {"stamp": {"nsec": 12_000_000}},
            "pose": [
                {
                    "name": "base_link",
                    "id": 17,
                    "position": {"y": 2000.0},
                    "orientation": {"z": -0.707106781, "w": 0.707106781},
                }
            ],
        }
    )
    assert record.stamp_ns == 12_000_000
    assert record.position_world_enu_m == (0.0, 2000.0, 0.0)
    assert record.orientation_world_from_entity_xyzw == (0.0, 0.0, -0.707106781, 0.707106781)


def test_model_scoped_pose_publisher_payload_uses_entity_stamp() -> None:
    payload = {
        "pose": [
            {
                "header": {"stamp": {"sec": 4, "nsec": 5}},
                "name": "x500_15m::base_link",
                "position": {},
                "orientation": {"w": 1.0},
            },
            {
                "header": {"stamp": {"sec": 4, "nsec": 5}},
                "name": "x500_15m",
                "position": {"y": 2000.0, "z": 0.75},
                "orientation": {"w": 1.0},
            },
        ]
    }
    link = parse_pose_v_payload(payload, entity_name="x500_15m::base_link")
    world = pose_record_world_from_payload(payload, link, parent_model_name="x500_15m")
    assert world.stamp_ns == 4_000_000_005
    assert world.entity_id == -1
    assert world.position_world_enu_m == (0.0, 2000.0, 0.75)


def test_target_model_owns_scoped_pose_publisher() -> None:
    root = ET.parse(MODEL).getroot()
    plugin = root.find("./model/plugin[@name='gz::sim::systems::PosePublisher']")
    assert plugin is not None
    assert plugin.findtext("publish_model_pose") == "true"
    assert plugin.findtext("publish_link_pose") == "true"
    assert plugin.findtext("use_pose_vector_msg") == "true"
    assert plugin.findtext("static_publisher") == "false"


def test_pose_v_world_resolution_composes_scene_model_and_base_link() -> None:
    payload = {
        "header": {"stamp": {"sec": 2, "nsec": 3}},
        "pose": [
            {
                "name": "x500_15m",
                "id": 16,
                "position": {"y": 2000.0, "z": 0.75},
                "orientation": {"z": -0.707106781, "w": 0.707106781},
            },
            {
                "name": "base_link",
                "id": 17,
                "position": {},
                "orientation": {"w": 1.0},
            },
        ],
    }
    record = parse_pose_v_payload(payload)
    world = pose_record_world_from_payload(payload, record)
    assert world.entity_name == "base_link"
    assert world.entity_id == 17
    assert world.position_world_enu_m == pytest.approx((0.0, 2000.0, 0.75), abs=1.0e-9)
    assert world.orientation_world_from_entity_xyzw == pytest.approx(
        (0.0, 0.0, -0.707106781, 0.707106781), abs=1.0e-9
    )


def test_pose_v_world_resolution_does_not_fallback_without_parent_model() -> None:
    payload = _payload(3, 0)
    record = parse_pose_v_payload(payload)
    with pytest.raises(PosePayloadError, match="parent model"):
        pose_record_world_from_payload(payload, record)


def test_pose_v_state_relocks_id_only_after_stamp_reset() -> None:
    state = PoseVState()
    assert state.consume(_payload(5, 0)) is not None
    assert state.consume(_payload(5, 999, entity_id=18)) is None
    assert state.entity_id == 17
    assert state.consume(_payload(4, 999, entity_id=18)) is not None
    assert state.reset_count == 1
    assert state.entity_id == 18
    assert state.consume(_payload(5, 0, entity_id=18)) is not None


def test_pose_v_state_reset_with_same_id_is_accepted() -> None:
    state = PoseVState()
    assert state.consume(_payload(8, 0, entity_id=17)) is not None
    assert state.consume(_payload(7, 0, entity_id=17)) is not None
    assert state.reset_count == 1


def test_duplicate_authoritative_name_is_unavailable() -> None:
    payload = _payload(1, 0)
    payload["pose"].append(dict(payload["pose"][0]))
    with pytest.raises(PosePayloadError, match="duplicate"):
        parse_pose_v_payload(payload)
