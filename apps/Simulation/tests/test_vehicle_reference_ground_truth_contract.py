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
    vehicle_reference_offset_parent_m,
    vehicle_reference_definition,
)
from stereo_ground_sim.gazebo_ground_truth_adapter import (
    OUTPUT_TOPIC,
    POSE_TOPIC,
    PosePayloadError,
    PoseVState,
    parse_pose_v_payload,
    pose_record_world_from_payload,
    pose_record_to_vehicle_reference,
)


ROOT = Path(__file__).resolve().parents[1]
MODEL = ROOT / "stereo_ground_sim" / "models" / "x500_15m" / "model.sdf"


def test_vehicle_reference_is_height_midpoint_sdf_frame_not_physics_link() -> None:
    root = ET.parse(MODEL).getroot()
    model = root.find("model")
    assert model is not None
    frames = model.findall("frame[@name='vehicle_reference_link']")
    assert len(frames) == 1
    frame = frames[0]
    assert frame.attrib["attached_to"] == "base_link"
    assert (frame.findtext("pose") or "").split() == ["0", "0", "-2.297088218875", "0", "0", "0"]
    assert model.findall("link[@name='vehicle_reference_link']") == []


def test_reference_metadata_contract() -> None:
    assert VEHICLE_REFERENCE_LINK == "vehicle_reference_link"
    assert VEHICLE_REFERENCE_DEFINITION == "project_defined_vehicle_reference_at_visual_height_midpoint_v1"
    assert SOURCE_GAZEBO_ENTITY == "base_link"
    assert REFERENCE_OFFSET_PARENT_M == (0.0, 0.0, -2.297088218875)
    assert REFERENCE_OFFSET_PARENT_RPY_RAD == (0.0, 0.0, 0.0)
    assert WORLD_FRAME == "gazebo_world_enu"
    assert POSE_TOPIC == "/model/x500_15m/pose"
    assert OUTPUT_TOPIC == "/simulation/ground_truth/vehicle_reference_link"


@pytest.mark.parametrize("model_name,bottom_z,top_z", [
    ("x500_2m", -1.0265138481, 0.3083),
    ("x500_15m", -6.19487643775, 1.6007),
])
def test_reference_matches_visual_height_midpoint(model_name, bottom_z, top_z) -> None:
    import numpy as np

    model_dir = MODEL.parent.parent / model_name
    model = ET.parse(model_dir / "model.sdf").getroot().find("model")
    mesh = ET.parse(model_dir / "meshes/NXP-HGD-CF.dae").getroot()
    ns = {"c": "http://www.collada.org/2005/11/COLLADASchema"}
    # The current body DAE scene nodes are identity; reject new transforms so
    # a geometry edit cannot silently invalidate this independent vertex check.
    identity = list(np.eye(4).ravel())
    for matrix in mesh.findall(".//c:visual_scene//c:matrix", ns):
        assert list(map(float, matrix.text.split())) == identity
    assert mesh.findtext("c:asset/c:up_axis", namespaces=ns) == "Z_UP"
    assert float(mesh.find("c:asset/c:unit", ns).get("meter")) == 1.0
    vertices = []
    for geometry in mesh.findall(".//c:geometry", ns):
        source_id = geometry.find("c:mesh/c:vertices/c:input[@semantic='POSITION']", ns).get("source")[1:]
        values = geometry.find(f"c:mesh/c:source[@id='{source_id}']/c:float_array", ns)
        vertices.extend(np.fromstring(values.text, sep=" ").reshape(-1, 3)[:, 2])
    visual = model.find("link[@name='base_link']/visual[@name='base_link_visual']")
    scale_z = float(visual.findtext("geometry/mesh/scale").split()[2])
    bottom = min(vertices) * scale_z
    rotor = model.find("link[@name='rotor_0']")
    hub = rotor.find("visual[@name='rotor_0_visual_hub']")
    top = (float(rotor.findtext("pose").split()[2])
           + float(hub.findtext("pose").split()[2])
           + float(hub.findtext("geometry/cylinder/length")) / 2)
    assert bottom == pytest.approx(bottom_z, abs=1e-12)
    assert top == pytest.approx(top_z, abs=1e-12)
    frame = model.find("frame[@name='vehicle_reference_link']")
    pose = tuple(map(float, frame.findtext("pose").split()))
    assert frame.get("attached_to") == "base_link"
    assert pose == pytest.approx((0, 0, (bottom + top) / 2, 0, 0, 0), abs=1e-12)
    assert pose[:3] == vehicle_reference_offset_parent_m(model_name)
    assert model.find("link[@name='vehicle_reference_link']") is None


@pytest.mark.parametrize("quaternion,expected", [
    ((0, 0, 0, 1), (10, 20, 30 - 0.35910692405)),
    ((0, 2**-0.5, 0, 2**-0.5), (10 - 0.35910692405, 20, 30)),
    ((2**-0.5, 0, 0, 2**-0.5), (10, 20 + 0.35910692405, 30)),
])
def test_reference_shift_rotates_with_vehicle_and_preserves_stamp(quaternion, expected):
    payload = _payload(12, 345)
    payload["pose"][0]["position"] = {}
    payload["pose"].append({
        "name": "x500_2m", "id": 16,
        "position": dict(zip("xyz", (10, 20, 30))),
        "orientation": dict(zip("xyzw", quaternion)),
    })
    base = pose_record_world_from_payload(payload, parse_pose_v_payload(payload), parent_model_name="x500_2m")
    reference = pose_record_to_vehicle_reference(base, "x500_2m")
    assert reference.position_world_enu_m == pytest.approx(expected, abs=1e-12)
    assert reference.orientation_world_from_entity_xyzw == base.orientation_world_from_entity_xyzw
    assert reference.stamp_ns == 12_000_000_345
    assert base.position_world_enu_m == (10, 20, 30)  # mesh source is unchanged
    large = pose_record_to_vehicle_reference(base, "x500_15m")
    expected_large = tuple(p + (e - p) * (2.297088218875 / 0.35910692405)
                           for p, e in zip((10, 20, 30), expected))
    assert large.position_world_enu_m == pytest.approx(expected_large, abs=1e-12)


def test_unknown_reference_model_is_rejected():
    with pytest.raises(ValueError, match="unknown vehicle reference"):
        vehicle_reference_definition("unknown")


def test_historical_origin_qualification_cannot_certify_midpoint_runtime():
    from stereo_ground_sim.correction_qualification import (
        FREEZE_ANCHOR_REPOSITORY_PATH, _validate_coordinate_contract_sources,
        load_freeze_anchor,
    )

    repository = ROOT.parents[1]
    anchor = load_freeze_anchor(repository / FREEZE_ANCHOR_REPOSITORY_PATH)
    # Isolate the semantic gate; the real verifier also rejects stale hashes
    # before reaching this gate. Preserve the anchor's historical GT fields.
    anchor["coordinate_contract_sources"]["files"] = {}
    with pytest.raises(ValueError, match="ground_truth_source_constant_mismatch"):
        _validate_coordinate_contract_sources(anchor, repository)


def test_reference_consumer_recovers_mesh_origin():
    from types import SimpleNamespace
    from geometry_msgs.msg import Point
    from stereo_ground_sim.rviz_publisher import RvizVisualizationPublisher

    published = []
    node = SimpleNamespace(
        target_model_name="x500_2m",
        ray_marker_pub=SimpleNamespace(publish=published.append),
        _marker_point=lambda p: Point(x=float(p[0]), y=float(p[1]), z=float(p[2])),
        _mesh_visual_scale=4.0638, _mesh_resource="test_mesh",
    )
    publisher = RvizVisualizationPublisher(node, world_frame=WORLD_FRAME, ray_display_names={})
    publisher._publish_actual_drone_marker(
        (9.64089307595, 20, 30), (0, 2**-0.5, 0, 2**-0.5), 12_000_000_345
    )
    position = published[0].markers[0].pose.position
    assert (position.x, position.y, position.z) == pytest.approx((10, 20, 30), abs=1e-12)


@pytest.mark.parametrize("model_name,reference_x", [
    ("x500_2m", 9.64089307595), ("x500_15m", 7.702911781125),
])
def test_live_ros_adapter_keeps_mesh_at_base_and_publishes_shifted_reference(monkeypatch, model_name, reference_x):
    """Real adapter thread/ROS serialization with synthetic Pose_V, no flight."""
    import json
    import time

    rclpy = pytest.importorskip("rclpy")
    from geometry_msgs.msg import PoseStamped
    from visualization_msgs.msg import MarkerArray
    from stereo_ground_sim.gazebo_ground_truth_adapter import GazeboGroundTruthAdapter, RVIZ_MARKER_TOPIC

    payload = {"header": {"stamp": {"sec": 12, "nsec": 345}}, "pose": [
        {"name": f"{model_name}::base_link", "position": {}, "orientation": {"w": 1}},
        {"name": model_name, "position": {"x": 10, "y": 20, "z": 30},
         "orientation": {"y": 2**-0.5, "w": 2**-0.5}},
    ]}
    producer = "import sys,time\nfor _ in range(100):\n print(sys.argv[1],flush=True)\n time.sleep(0.05)"
    monkeypatch.setenv("ROS_DOMAIN_ID", "87")
    monkeypatch.setenv("ROS_LOCALHOST_ONLY", "1")
    rclpy.init()
    node = rclpy.create_node("reference_midpoint_test")
    poses, markers = [], []
    node.create_subscription(PoseStamped, OUTPUT_TOPIC, poses.append, 10)
    node.create_subscription(MarkerArray, RVIZ_MARKER_TOPIC, markers.append, 10)
    adapter = None
    try:
        adapter = GazeboGroundTruthAdapter(
            parent_model_name=model_name,
            command=(sys.executable, "-u", "-c", producer, json.dumps(payload)),
        )
        deadline = time.monotonic() + 6
        while time.monotonic() < deadline and not (poses and markers):
            rclpy.spin_once(node, timeout_sec=0.1)
        assert poses and markers, "adapter did not publish both ROS outputs"
        pose = poses[-1]
        assert pose.header.frame_id == "gazebo_world_enu"
        assert (pose.header.stamp.sec, pose.header.stamp.nanosec) == (12, 345)
        assert (pose.pose.position.x, pose.pose.position.y, pose.pose.position.z) == pytest.approx(
            (reference_x, 20, 30), abs=1e-12
        )
        mesh = markers[-1].markers[0]
        assert (mesh.pose.position.x, mesh.pose.position.y, mesh.pose.position.z) == (10, 20, 30)
    finally:
        if adapter is not None:
            adapter.shutdown()
        node.destroy_node()
        rclpy.shutdown()


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
