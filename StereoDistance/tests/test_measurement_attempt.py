from dataclasses import asdict, replace
from math import cos, degrees, radians, sin
from types import SimpleNamespace

import pytest

from wiris_stereo_distance.calibration import (
    CameraIntrinsics,
    CheckerboardConfig,
    StereoExtrinsics,
    StereoIntrinsicsResult,
)
from wiris_stereo_distance.measurement_attempt import (
    AttitudeSnapshot,
    CameraIntrinsicsSnapshot,
    CheckerboardExtrinsicsSnapshot,
    DetectionSnapshot,
    GeometryConfigSnapshot,
    GpsSnapshot,
    MeasurementAttempt,
    StereoIntrinsicsSnapshot,
    build_checkerboard_baseline_snapshot,
    build_pixhawk_baseline_snapshot,
    detection_pair_key,
)
from wiris_stereo_distance.measurement_controller import calculate_measurement_geometry
from wiris_stereo_distance.rtk_geometry import EARTH_RADIUS_M
from wiris_stereo_distance.stereo import DetectionObservation, correspondence_rejection_reason


def detection(camera: str, center_x: float, *, timestamp_s: float = 10.0, metadata=None):
    return DetectionObservation(
        camera=camera,
        timestamp_s=timestamp_s,
        width=1280,
        height=720,
        x1=center_x - 10.0,
        y1=350.0,
        x2=center_x + 10.0,
        y2=370.0,
        confidence=0.95,
        class_id=2,
        class_name="drone",
        timestamp_metadata={} if metadata is None else metadata,
    )


def calibration_result(*, right_fx: float = 900.0, checkerboard: bool = False, distorted: bool = False):
    left_k = [[900.0, 0.0, 640.0], [0.0, 900.0, 360.0], [0.0, 0.0, 1.0]]
    right_k = [[right_fx, 0.0, 640.0], [0.0, right_fx, 360.0], [0.0, 0.0, 1.0]]
    left_distortion = [0.01, -0.02, 0.0, 0.0, 0.0] if distorted else [0.0]
    right_distortion = [0.03, -0.04, 0.0, 0.0, 0.0] if distorted else [0.0]
    left = CameraIntrinsics(1280, 720, left_k, left_distortion, 0.2, 20)
    right = CameraIntrinsics(1280, 720, right_k, right_distortion, 0.3, 21)
    extrinsics = None
    if checkerboard:
        extrinsics = StereoExtrinsics(
            rotation_left_to_right=[[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]],
            translation_left_to_right_m=[-0.3, 0.0, 0.0],
            reprojection_error_px=0.1,
            pair_count=12,
        )
    return StereoIntrinsicsResult(
        left=left,
        right=right,
        board=CheckerboardConfig(9, 6, 0.03),
        extrinsics=extrinsics,
    )


def attitude(camera: str, q=(1.0, 0.0, 0.0, 0.0), timestamp_s: float = 10.0):
    return AttitudeSnapshot(
        camera=camera,
        app_receive_monotonic_s=timestamp_s,
        quaternion_wxyz=tuple(q),
        namespace=camera,
        topic=f"/{camera}/fmu/out/vehicle_attitude",
        px4_timestamp_s=timestamp_s - 1.0,
        roll_deg=0.0,
        pitch_deg=0.0,
        yaw_deg=0.0,
    )


def gps(camera: str, *, longitude_deg: float, timestamp_s: float):
    return GpsSnapshot(
        camera=camera,
        app_receive_monotonic_s=timestamp_s,
        latitude_deg=0.0,
        longitude_deg=longitude_deg,
        altitude_m=100.0,
        fix_type=6,
        satellites_used=20,
        eph_m=0.01,
        epv_m=0.02,
        rtk_label="RTK_FIXED",
        namespace=camera,
        topic=f"/{camera}/fmu/out/vehicle_gps_position",
        px4_timestamp_s=timestamp_s - 1.0,
    )


def pixhawk_attempt(*, intrinsics_source=None) -> MeasurementAttempt:
    left_source = detection("left", 640.0)
    right_source = detection("right", 613.0)
    left_detection = DetectionSnapshot.from_observation(
        left_source, pairing_timestamp_basis="detection", pairing_timestamp_s=10.0
    )
    right_detection = DetectionSnapshot.from_observation(
        right_source, pairing_timestamp_basis="detection", pairing_timestamp_s=10.0
    )
    left_attitude = attitude("left")
    right_attitude = attitude("right")
    longitude_delta_deg = degrees(0.3 / EARTH_RADIUS_M)
    left_gps = gps("left", longitude_deg=0.0, timestamp_s=9.8)
    right_gps = gps("right", longitude_deg=longitude_delta_deg, timestamp_s=9.9)
    baseline = build_pixhawk_baseline_snapshot(
        left_gps=left_gps,
        right_gps=right_gps,
        selected_left_quaternion_wxyz=left_attitude.q,
        selected_right_quaternion_wxyz=right_attitude.q,
        selected_left_attitude_monotonic_s=left_attitude.timestamp_s,
        created_monotonic_s=10.2,
        detection_midpoint_monotonic_s=10.0,
    )
    result = calibration_result() if intrinsics_source is None else intrinsics_source
    intrinsics = StereoIntrinsicsSnapshot.from_result(
        result, active_zoom_level=8, selected_profile_path="zoom_08.json"
    )
    return MeasurementAttempt(
        attempt_id=1,
        attempt_session_id="session-a",
        detection_pair_key=detection_pair_key(
            triangulation_source="pixhawk",
            timestamp_basis="detection",
            left=left_source,
            right=right_source,
            left_pairing_timestamp_s=10.0,
            right_pairing_timestamp_s=10.0,
        ),
        created_monotonic_s=10.2,
        created_wall_s=1000.0,
        detection_midpoint_monotonic_s=10.0,
        timestamp_basis="detection",
        triangulation_source="pixhawk",
        left_detection=left_detection,
        right_detection=right_detection,
        left_attitude=left_attitude,
        right_attitude=right_attitude,
        left_gps=left_gps,
        right_gps=right_gps,
        intrinsics=intrinsics,
        baseline=baseline,
        geometry_config=GeometryConfigSnapshot(900.0, None, None, False, "detection", False),
        left_attitude_age_s=0.0,
        right_attitude_age_s=0.0,
        attitude_pair_delta_s=0.0,
        relative_roll_deg=0.0,
        relative_pitch_deg=0.0,
        relative_yaw_deg=0.0,
        source_quality="GOOD",
        warnings=(),
    )


def direct_detection_snapshot(metadata_items) -> DetectionSnapshot:
    return DetectionSnapshot(
        camera="left",
        app_receive_monotonic_s=10.0,
        width_px=1280,
        height_px=720,
        bbox_xyxy_px=(630.0, 350.0, 650.0, 370.0),
        center_px=(640.0, 360.0),
        confidence=0.95,
        class_id=2,
        class_name="drone",
        pairing_timestamp_basis="detection",
        pairing_timestamp_s=10.0,
        timestamp_metadata_items=metadata_items,
        candidate_class_ids=(2,),
    )


class MetadataMappingSubclass(dict):
    pass


class MetadataCustomIterable:
    def __iter__(self):
        yield "rtp_timestamp"
        yield 123


class MetadataTupleSubclass(tuple):
    pass


class MetadataListSubclass(list):
    pass


class BypassDetectionSnapshot(DetectionSnapshot):
    def __post_init__(self) -> None:
        pass


def malformed_mapping_item():
    return {"rtp_timestamp": 999, "timestamp_source": "rtcp"}


def detection_payload(metadata) -> dict:
    return {
        "timestamp_s": 10.0,
        "width": 1280,
        "height": 720,
        "best_detection": {
            "x1": 600.0,
            "y1": 300.0,
            "x2": 680.0,
            "y2": 380.0,
            "confidence": 0.9,
            "class_id": 1,
            "class_name": "target",
        },
        "detections": [],
        "timestamp_metadata": metadata,
    }


def bypass_detection_snapshot(metadata_items) -> BypassDetectionSnapshot:
    return BypassDetectionSnapshot(
        camera="left",
        app_receive_monotonic_s=10.0,
        width_px=1280,
        height_px=720,
        bbox_xyxy_px=(630.0, 350.0, 650.0, 370.0),
        center_px=(640.0, 360.0),
        confidence=0.95,
        class_id=2,
        class_name="drone",
        pairing_timestamp_basis="detection",
        pairing_timestamp_s=10.0,
        timestamp_metadata_items=metadata_items,
        candidate_class_ids=(2,),
    )


def test_from_payload_rejects_malformed_mapping_item_before_dict_coercion() -> None:
    with pytest.raises((TypeError, ValueError)):
        DetectionObservation.from_payload(
            "left",
            detection_payload((malformed_mapping_item(),)),
        )


def test_from_payload_malformed_metadata_cannot_reach_snapshot_attempt_or_sha() -> None:
    observation_created = False
    snapshot_created = False
    attempt_created = False
    canonical_sha_called = False

    with pytest.raises((TypeError, ValueError)):
        observation = DetectionObservation.from_payload(
            "left",
            detection_payload((malformed_mapping_item(),)),
        )
        observation_created = True
        snapshot = DetectionSnapshot.from_observation(
            observation,
            pairing_timestamp_basis="detection",
            pairing_timestamp_s=10.0,
        )
        snapshot_created = True
        attempt = replace(pixhawk_attempt(), left_detection=snapshot)
        attempt_created = True
        canonical_sha_called = True
        attempt.canonical_sha256()

    assert observation_created is False
    assert snapshot_created is False
    assert attempt_created is False
    assert canonical_sha_called is False


def test_from_payload_normal_mapping_is_detached_and_hash_stable() -> None:
    source_metadata = {
        "rtp_timestamp": 123,
        "timestamp_source": "rtcp",
        "unknown_vendor_field": [1, 2],
    }
    payload = detection_payload(source_metadata)
    observation = DetectionObservation.from_payload("left", payload)
    assert observation is not None
    snapshot = DetectionSnapshot.from_observation(
        observation,
        pairing_timestamp_basis="detection",
        pairing_timestamp_s=10.0,
    )
    attempt = replace(pixhawk_attempt(), left_detection=snapshot)
    before_snapshot_metadata = snapshot.timestamp_metadata_items
    before_sha = attempt.canonical_sha256()

    source_metadata["rtp_timestamp"] = 999
    source_metadata["unknown_vendor_field"].append(3)
    payload["timestamp_metadata"] = {"rtp_timestamp": 456}

    assert observation.timestamp_metadata["rtp_timestamp"] == 123
    assert observation.timestamp_metadata["unknown_vendor_field"] == [1, 2], (
        "S3-B1 nested observation metadata must remain deeply detached from source mutation"
    )
    assert observation.timestamp_metadata is not source_metadata
    assert snapshot.timestamp_metadata_items == (
        ("timestamp_source", "rtcp"),
        ("rtp_timestamp", 123),
    )
    assert snapshot.timestamp_metadata_items == before_snapshot_metadata
    assert attempt.canonical_sha256() == before_sha


def test_from_payload_accepts_explicit_tuple_and_list_pairs() -> None:
    observation = DetectionObservation.from_payload(
        "left",
        detection_payload(
            (
                ("rtp_timestamp", 123),
                ["timestamp_source", "rtcp"],
            )
        ),
    )
    assert observation is not None
    assert observation.timestamp_metadata == {
        "rtp_timestamp": 123,
        "timestamp_source": "rtcp",
    }

    snapshot = DetectionSnapshot.from_observation(
        observation,
        pairing_timestamp_basis="detection",
        pairing_timestamp_s=10.0,
    )

    assert snapshot.timestamp_metadata_items == (
        ("timestamp_source", "rtcp"),
        ("rtp_timestamp", 123),
    )


@pytest.mark.parametrize("empty_metadata", (None, (), []), ids=("none", "tuple", "list"))
def test_payload_adapter_allows_explicit_empty_values(empty_metadata) -> None:
    observation = DetectionObservation.from_payload(
        "left",
        detection_payload(empty_metadata),
    )

    assert observation is not None
    assert observation.timestamp_metadata == {}


@pytest.mark.parametrize(
    "invalid_metadata_factory",
    (
        lambda: "",
        lambda: b"",
        bytearray,
        lambda: memoryview(b""),
        set,
        frozenset,
        lambda: (item for item in ()),
        MetadataCustomIterable,
        MetadataTupleSubclass,
        MetadataListSubclass,
        lambda: 0,
        lambda: 1,
        lambda: False,
        lambda: True,
    ),
    ids=(
        "empty-str",
        "empty-bytes",
        "empty-bytearray",
        "empty-memoryview",
        "empty-set",
        "empty-frozenset",
        "generator",
        "custom-iterable",
        "tuple-subclass",
        "list-subclass",
        "zero-int",
        "int",
        "false-bool",
        "bool",
    ),
)
def test_payload_adapter_rejects_malformed_outer_types(invalid_metadata_factory) -> None:
    with pytest.raises((TypeError, ValueError)):
        DetectionObservation.from_payload(
            "left",
            detection_payload(invalid_metadata_factory()),
        )


@pytest.mark.parametrize(
    "invalid_item_factory",
    (
        malformed_mapping_item,
        lambda: MetadataMappingSubclass(malformed_mapping_item()),
        lambda: {"rtp_timestamp", 123},
        lambda: frozenset(("rtp_timestamp", 123)),
        lambda: "ab",
        lambda: b"ab",
        lambda: bytearray(b"ab"),
        lambda: memoryview(b"ab"),
        lambda: (value for value in ("rtp_timestamp", 123)),
        MetadataCustomIterable,
        lambda: MetadataTupleSubclass(("rtp_timestamp", 123)),
        lambda: MetadataListSubclass(("rtp_timestamp", 123)),
    ),
    ids=(
        "mapping",
        "mapping-subclass",
        "set",
        "frozenset",
        "str",
        "bytes",
        "bytearray",
        "memoryview",
        "generator",
        "custom-iterable",
        "tuple-subclass",
        "list-subclass",
    ),
)
def test_payload_adapter_rejects_malformed_pair_items(invalid_item_factory) -> None:
    with pytest.raises((TypeError, ValueError)):
        DetectionObservation.from_payload(
            "left",
            detection_payload((invalid_item_factory(),)),
        )


@pytest.mark.parametrize(
    "invalid_metadata",
    (
        ((),),
        (("rtp_timestamp",),),
        (("rtp_timestamp", 1, 2),),
        ([],),
        (["rtp_timestamp"],),
        (["rtp_timestamp", 1, 2],),
        (("rtp_timestamp", 1), ("rtp_timestamp", 2)),
        ((1, 123),),
    ),
    ids=(
        "empty-tuple",
        "one-tuple",
        "three-tuple",
        "empty-list",
        "one-list",
        "three-list",
        "duplicate",
        "non-string-key",
    ),
)
def test_payload_adapter_rejects_invalid_pair_lengths_and_keys(invalid_metadata) -> None:
    with pytest.raises((TypeError, ValueError)):
        DetectionObservation.from_payload(
            "left",
            detection_payload(invalid_metadata),
        )


def test_snapshot_subclass_cannot_enter_measurement_attempt() -> None:
    bypass = bypass_detection_snapshot((malformed_mapping_item(),))

    with pytest.raises(TypeError, match="DetectionSnapshot"):
        replace(pixhawk_attempt(), left_detection=bypass)


def test_snapshot_subclass_factory_is_rejected() -> None:
    observation = detection("left", 640.0, metadata={"rtp_timestamp": 123})

    with pytest.raises(TypeError, match="subclass"):
        BypassDetectionSnapshot.from_observation(
            observation,
            pairing_timestamp_basis="detection",
            pairing_timestamp_s=10.0,
        )


def test_snapshot_subclass_replace_cannot_reach_attempt_or_sha() -> None:
    bypass = bypass_detection_snapshot((("rtp_timestamp", 123),))
    replaced_bypass = replace(
        bypass,
        timestamp_metadata_items=(malformed_mapping_item(),),
    )
    attempt_created = False
    canonical_sha_called = False

    with pytest.raises(TypeError, match="DetectionSnapshot"):
        attempt = replace(pixhawk_attempt(), left_detection=replaced_bypass)
        attempt_created = True
        canonical_sha_called = True
        attempt.canonical_sha256()

    assert attempt_created is False
    assert canonical_sha_called is False


def test_base_snapshot_factory_replace_attempt_and_sha_remain_supported() -> None:
    observation = DetectionObservation.from_payload(
        "left",
        detection_payload({"rtp_timestamp": 123, "timestamp_source": "rtcp"}),
    )
    assert observation is not None
    snapshot = DetectionSnapshot.from_observation(
        observation,
        pairing_timestamp_basis="detection",
        pairing_timestamp_s=10.0,
    )
    replaced_snapshot = replace(
        snapshot,
        timestamp_metadata_items=(("timestamp_source", "rtcp"), ("rtp_timestamp", 123)),
    )
    attempt = replace(pixhawk_attempt(), left_detection=replaced_snapshot)

    assert type(snapshot) is DetectionSnapshot
    assert type(replaced_snapshot) is DetectionSnapshot
    assert len(attempt.canonical_sha256()) == 64


@pytest.mark.parametrize(
    "invalid_item_factory",
    (
        malformed_mapping_item,
        lambda: MetadataMappingSubclass(
            {"rtp_timestamp": 999, "timestamp_source": "rtcp"}
        ),
        lambda: {"rtp_timestamp", 123},
        lambda: frozenset(("rtp_timestamp", 123)),
        lambda: "ab",
        lambda: b"ab",
        lambda: bytearray(b"ab"),
        lambda: memoryview(b"ab"),
        lambda: (value for value in ("rtp_timestamp", 123)),
        MetadataCustomIterable,
        lambda: MetadataTupleSubclass(("rtp_timestamp", 123)),
        lambda: MetadataListSubclass(("rtp_timestamp", 123)),
    ),
    ids=(
        "mapping-item",
        "mapping-subclass-item",
        "set-item",
        "frozenset-item",
        "str-item",
        "bytes-item",
        "bytearray-item",
        "memoryview-item",
        "generator-item",
        "custom-iterable-item",
        "tuple-subclass-item",
        "list-subclass-item",
    ),
)
def test_detection_metadata_pair_shape_rejects_non_explicit_pair_items(
    invalid_item_factory,
) -> None:
    with pytest.raises((TypeError, ValueError), match="tuple or list"):
        direct_detection_snapshot((invalid_item_factory(),))


@pytest.mark.parametrize(
    "invalid_item",
    (
        (),
        ("rtp_timestamp",),
        ("rtp_timestamp", 1, 2),
        [],
        ["rtp_timestamp"],
        ["rtp_timestamp", 1, 2],
    ),
    ids=(
        "empty-tuple",
        "one-tuple",
        "three-tuple",
        "empty-list",
        "one-list",
        "three-list",
    ),
)
def test_detection_metadata_pair_shape_rejects_invalid_lengths(invalid_item) -> None:
    with pytest.raises(ValueError, match="exactly two"):
        direct_detection_snapshot((invalid_item,))


@pytest.mark.parametrize(
    "invalid_outer_factory",
    (
        lambda: "",
        lambda: "ab",
        lambda: b"",
        lambda: b"ab",
        bytearray,
        dict,
        lambda: {"rtp_timestamp": 123},
        set,
        lambda: (item for item in (("rtp_timestamp", 123),)),
        MetadataCustomIterable,
        lambda: memoryview(b""),
    ),
    ids=(
        "empty-str",
        "str",
        "empty-bytes",
        "bytes",
        "bytearray",
        "empty-mapping",
        "mapping",
        "set",
        "generator",
        "custom-iterable",
        "memoryview",
    ),
)
def test_direct_detection_metadata_outer_container_rejects_malformed_types(
    invalid_outer_factory,
) -> None:
    with pytest.raises((TypeError, ValueError), match="tuple or list"):
        direct_detection_snapshot(invalid_outer_factory())


@pytest.mark.parametrize("empty_metadata", (None, (), []), ids=("none", "tuple", "list"))
def test_direct_detection_metadata_outer_container_allows_explicit_empty_values(
    empty_metadata,
) -> None:
    assert direct_detection_snapshot(empty_metadata).timestamp_metadata_items == ()


def test_detection_metadata_pair_shape_accepts_explicit_tuple_and_list_pairs() -> None:
    snapshot = direct_detection_snapshot(
        (
            ("rtp_timestamp", 123),
            ["timestamp_source", "rtcp"],
        )
    )

    assert snapshot.timestamp_metadata_items == (
        ("timestamp_source", "rtcp"),
        ("rtp_timestamp", 123),
    )
    assert all(type(item) is tuple for item in snapshot.timestamp_metadata_items)


def test_factory_detection_metadata_mapping_item_is_rejected() -> None:
    observation = detection(
        "left",
        640.0,
        metadata=(malformed_mapping_item(),),
    )

    with pytest.raises((TypeError, ValueError), match="tuple or list"):
        DetectionSnapshot.from_observation(
            observation,
            pairing_timestamp_basis="detection",
            pairing_timestamp_s=10.0,
        )


@pytest.mark.parametrize(
    "invalid_outer_factory",
    (
        lambda: "ab",
        lambda: b"ab",
        lambda: {"rtp_timestamp", 123},
        lambda: (item for item in (("rtp_timestamp", 123),)),
        MetadataCustomIterable,
    ),
    ids=("str", "bytes", "set", "generator", "custom-iterable"),
)
def test_factory_detection_metadata_outer_container_rejects_arbitrary_iterables(
    invalid_outer_factory,
) -> None:
    observation = detection(
        "left",
        640.0,
        metadata=invalid_outer_factory(),
    )

    with pytest.raises((TypeError, ValueError), match="Mapping, tuple, or list"):
        DetectionSnapshot.from_observation(
            observation,
            pairing_timestamp_basis="detection",
            pairing_timestamp_s=10.0,
        )


def test_factory_detection_metadata_normal_mapping_is_projected() -> None:
    snapshot = DetectionSnapshot.from_observation(
        detection(
            "left",
            640.0,
            metadata={
                "rtp_timestamp": 123,
                "timestamp_source": "rtcp",
                "unknown_vendor_field": [1, 2],
            },
        ),
        pairing_timestamp_basis="detection",
        pairing_timestamp_s=10.0,
    )

    assert snapshot.timestamp_metadata_items == (
        ("timestamp_source", "rtcp"),
        ("rtp_timestamp", 123),
    )
    assert type(snapshot.timestamp_metadata["rtp_timestamp"]) is int
    assert type(snapshot.timestamp_metadata["timestamp_source"]) is str


def test_replace_rejects_malformed_detection_metadata_mapping_item() -> None:
    snapshot = direct_detection_snapshot((("rtp_timestamp", 123),))

    with pytest.raises((TypeError, ValueError), match="tuple or list"):
        replace(
            snapshot,
            timestamp_metadata_items=(malformed_mapping_item(),),
        )


def test_malformed_mapping_item_cannot_reach_attempt_canonical_sha() -> None:
    canonical_sha_called = False
    with pytest.raises((TypeError, ValueError), match="tuple or list"):
        snapshot = direct_detection_snapshot((malformed_mapping_item(),))
        attempt = replace(pixhawk_attempt(), left_detection=snapshot)
        canonical_sha_called = True
        attempt.canonical_sha256()

    assert canonical_sha_called is False


@pytest.mark.parametrize(
    "invalid_value_factory",
    (
        lambda: [123],
        lambda: {"value": 123},
        lambda: {123},
        lambda: (123,),
        lambda: b"123",
        lambda: bytearray(b"123"),
        lambda: SimpleNamespace(value=123),
        lambda: (value for value in (123,)),
    ),
    ids=("list", "dict", "set", "tuple", "bytes", "bytearray", "custom-object", "generator"),
)
def test_direct_detection_metadata_rejects_non_scalar_values(invalid_value_factory) -> None:
    with pytest.raises((TypeError, ValueError), match="scalar"):
        direct_detection_snapshot((("rtp_timestamp", invalid_value_factory()),))


@pytest.mark.parametrize("invalid_value", (float("nan"), float("inf"), float("-inf")))
def test_direct_detection_metadata_rejects_nonfinite_float_at_construction(invalid_value: float) -> None:
    with pytest.raises(ValueError, match="finite"):
        direct_detection_snapshot((("rtcp_sr_age_s", invalid_value),))


def test_direct_detection_metadata_rejects_duplicate_keys() -> None:
    with pytest.raises(ValueError, match="duplicate"):
        direct_detection_snapshot((("rtp_timestamp", 1), ("rtp_timestamp", 2)))


def test_factory_detection_metadata_rejects_duplicate_allowlisted_keys() -> None:
    observation = detection(
        "left",
        640.0,
        metadata=(("rtp_timestamp", 1), ("rtp_timestamp", 2)),
    )

    with pytest.raises(ValueError, match="duplicate"):
        DetectionSnapshot.from_observation(
            observation,
            pairing_timestamp_basis="detection",
            pairing_timestamp_s=10.0,
        )


@pytest.mark.parametrize(
    "invalid_items",
    (
        ((1, 123),),
        (("not_allowlisted", 123),),
        (("rtp_timestamp", 1, 2),),
        ("rtp_timestamp", 1),
        123,
    ),
    ids=("non-string-key", "unknown-key", "three-item-pair", "flat-pair", "non-iterable"),
)
def test_direct_detection_metadata_rejects_invalid_keys_and_container_shape(invalid_items) -> None:
    with pytest.raises((TypeError, ValueError)):
        direct_detection_snapshot(invalid_items)


def test_direct_detection_metadata_has_canonical_allowlist_order_and_hash() -> None:
    first = direct_detection_snapshot(
        (
            ("rtp_timestamp", 123),
            ("timestamp_source", "rtcp"),
            ("rtcp_sr_age_s", 0.1),
            ("source_timestamp_s", None),
            ("rtp_marker_seq", True),
        )
    )
    second = direct_detection_snapshot(
        (
            ("rtp_marker_seq", True),
            ("source_timestamp_s", None),
            ("rtcp_sr_age_s", 0.1),
            ("rtp_timestamp", 123),
            ("timestamp_source", "rtcp"),
        )
    )
    expected = (
        ("source_timestamp_s", None),
        ("timestamp_source", "rtcp"),
        ("rtp_timestamp", 123),
        ("rtp_marker_seq", True),
        ("rtcp_sr_age_s", 0.1),
    )
    template = pixhawk_attempt()
    first_attempt = replace(template, left_detection=first)
    second_attempt = replace(template, left_detection=second)

    assert first.timestamp_metadata_items == expected
    assert second.timestamp_metadata_items == expected
    assert type(dict(first.timestamp_metadata_items)["rtp_timestamp"]) is int
    assert type(dict(first.timestamp_metadata_items)["rtp_marker_seq"]) is bool
    assert first_attempt.canonical_payload() == second_attempt.canonical_payload()
    assert first_attempt.canonical_sha256() == second_attempt.canonical_sha256()


def test_direct_detection_metadata_is_stable_after_source_container_mutation() -> None:
    source = [("rtp_timestamp", 123), ("timestamp_source", "rtcp")]
    snapshot = direct_detection_snapshot(source)
    attempt = replace(pixhawk_attempt(), left_detection=snapshot)
    before_items = snapshot.timestamp_metadata_items
    before_payload = attempt.canonical_payload()
    before_sha = attempt.canonical_sha256()

    source[0] = ("rtp_timestamp", 999)
    source.append(("rtcp_sr_age_s", 99.0))

    assert snapshot.timestamp_metadata_items == before_items
    assert attempt.canonical_payload() == before_payload
    assert attempt.canonical_sha256() == before_sha


@pytest.mark.parametrize(
    "invalid_items",
    (
        (("rtp_timestamp", [123]),),
        (("rtcp_sr_age_s", float("nan")),),
        (("rtp_timestamp", 1), ("rtp_timestamp", 2)),
    ),
    ids=("list", "nan", "duplicate"),
)
def test_replace_revalidates_invalid_detection_metadata(invalid_items) -> None:
    snapshot = direct_detection_snapshot((("rtp_timestamp", 123),))

    with pytest.raises((TypeError, ValueError)):
        replace(snapshot, timestamp_metadata_items=invalid_items)


def test_asdict_mutation_does_not_change_snapshot_or_attempt_hash() -> None:
    snapshot = direct_detection_snapshot(
        (("timestamp_source", "rtcp"), ("rtp_timestamp", 123))
    )
    attempt = replace(pixhawk_attempt(), left_detection=snapshot)
    before_items = snapshot.timestamp_metadata_items
    before_sha = attempt.canonical_sha256()
    snapshot_payload = asdict(snapshot)
    attempt_payload = attempt.canonical_payload()

    snapshot_payload["timestamp_metadata_items"] = (("rtp_timestamp", 999),)
    attempt_payload["left_detection"]["timestamp_metadata_items"] = (("rtp_timestamp", 999),)

    assert snapshot.timestamp_metadata_items == before_items
    assert attempt.canonical_sha256() == before_sha


def test_factory_and_direct_detection_metadata_canonicalization_match() -> None:
    metadata = {
        "rtp_timestamp": 123,
        "timestamp_source": "rtcp",
        "rtcp_sr_age_s": 0.1,
    }
    factory = DetectionSnapshot.from_observation(
        detection("left", 640.0, metadata=metadata),
        pairing_timestamp_basis="detection",
        pairing_timestamp_s=10.0,
    )
    direct = direct_detection_snapshot(tuple(reversed(tuple(metadata.items()))))

    assert factory.timestamp_metadata_items == direct.timestamp_metadata_items


def test_snapshots_are_deeply_immutable_and_hash_stable_after_source_mutation() -> None:
    metadata = {"rtp_timestamp": 123, "timestamp_source": "rtcp", "ignored": [1, 2]}
    observation = detection("left", 640.0, metadata=metadata)
    camera_matrix = [[900.0, 0.0, 640.0], [0.0, 900.0, 360.0], [0.0, 0.0, 1.0]]
    distortion = [0.01, -0.02]
    raw_message = {"mutable": [1, 2]}
    attitude_q = [1.0, 0.0, 0.0, 0.0]
    attitude_state = SimpleNamespace(
        camera="left",
        timestamp_s=9.7,
        q=attitude_q,
        namespace="left",
        topic="/attitude",
        px4_timestamp_s=8.7,
        roll_deg=0.0,
        pitch_deg=0.0,
        yaw_deg=0.0,
    )
    gps_state = SimpleNamespace(
        camera="left",
        timestamp_s=9.8,
        latitude_deg=0.0,
        longitude_deg=0.0,
        altitude_m=100.0,
        fix_type=6,
        satellites_used=20,
        eph_m=0.01,
        epv_m=0.02,
        rtk_label="RTK_FIXED",
        namespace="left",
        topic="/gps",
        px4_timestamp_s=8.8,
        heading_deg=123.0,
        heading_accuracy_deg=0.5,
        raw=raw_message,
    )
    detection_snapshot = DetectionSnapshot.from_observation(
        observation, pairing_timestamp_basis="detection", pairing_timestamp_s=10.0
    )
    intrinsics_snapshot = StereoIntrinsicsSnapshot.from_result(
        StereoIntrinsicsResult(
            left=CameraIntrinsics(1280, 720, camera_matrix, distortion, 0.2, 20),
            right=CameraIntrinsics(1280, 720, camera_matrix, distortion, 0.2, 20),
            board=CheckerboardConfig(9, 6, 0.03),
        ),
        active_zoom_level=8,
        selected_profile_path="zoom_08.json",
    )
    gps_snapshot = GpsSnapshot.from_state(gps_state)
    attitude_snapshot = AttitudeSnapshot.from_state(attitude_state)
    before = (
        hash(detection_snapshot),
        hash(intrinsics_snapshot),
        hash(gps_snapshot),
        hash(attitude_snapshot),
    )

    metadata["rtp_timestamp"] = 999
    metadata["ignored"].append(3)
    camera_matrix[0][0] = 1.0
    distortion.append(99.0)
    raw_message["mutable"].append(3)
    gps_state.longitude_deg = 12.0
    gps_state.altitude_m = -50.0
    gps_state.heading_deg = 321.0
    attitude_q[0] = 0.0
    attitude_q[3] = 1.0

    assert detection_snapshot.timestamp_metadata["rtp_timestamp"] == 123
    assert intrinsics_snapshot.left.camera_matrix[0][0] == 900.0
    assert intrinsics_snapshot.left.distortion_coefficients == (0.01, -0.02)
    assert not hasattr(gps_snapshot, "raw")
    assert gps_snapshot.longitude_deg == 0.0
    assert gps_snapshot.altitude_m == 100.0
    assert gps_snapshot.heading_deg == 123.0
    assert attitude_snapshot.quaternion_wxyz == (1.0, 0.0, 0.0, 0.0)
    assert before == (
        hash(detection_snapshot),
        hash(intrinsics_snapshot),
        hash(gps_snapshot),
        hash(attitude_snapshot),
    )


def test_pixhawk_attempt_uses_one_left_quaternion_basis_and_reconstructs_known_point() -> None:
    attempt = pixhawk_attempt()
    assert attempt.baseline is not None
    assert attempt.baseline.left_basis_quaternion_wxyz == attempt.left_attitude.quaternion_wxyz
    assert attempt.translation_right_in_left_m == (0.3, 0.0, 0.0)

    geometry = calculate_measurement_geometry(attempt)

    assert geometry.triangulated is not None
    reconstructed = geometry.triangulated.point_left_rdf_m
    expected = (0.0, 0.0, 10.0)
    error_m = sum((actual - target) ** 2 for actual, target in zip(reconstructed, expected)) ** 0.5
    assert error_m <= 1e-9
    assert geometry.triangulated.range_m == 10.0
    assert geometry.triangulated.output_frame == "left_camera_rdf"


def test_selected_gps_pair_and_intrinsics_content_are_stable_and_independent() -> None:
    source = calibration_result(right_fx=910.0, distorted=True)
    attempt = pixhawk_attempt(intrinsics_source=source)
    before_hash = attempt.canonical_sha256()
    source.left.camera_matrix[0][0] = 1.0
    source.right.camera_matrix[0][0] = 2.0
    source.left.distortion_coefficients.append(88.0)

    assert attempt.left_gps.timestamp_s == 9.8
    assert attempt.right_gps.timestamp_s == 9.9
    assert attempt.baseline.ned_right_from_left_m == (0.0, 0.3, 0.0)
    assert attempt.intrinsics.left.camera_matrix[0][0] == 900.0
    assert attempt.intrinsics.right.camera_matrix[0][0] == 910.0
    assert attempt.intrinsics.left.distortion_coefficients == (0.01, -0.02, 0.0, 0.0, 0.0)
    assert attempt.intrinsics.right.distortion_coefficients == (0.03, -0.04, 0.0, 0.0, 0.0)
    assert attempt.canonical_sha256() == before_hash


def test_intrinsics_content_hash_is_deterministic_and_checkerboard_snapshot_is_self_contained() -> None:
    source = calibration_result(checkerboard=True)
    first = StereoIntrinsicsSnapshot.from_result(source, active_zoom_level=8, selected_profile_path="a.json")
    second = StereoIntrinsicsSnapshot.from_result(source, active_zoom_level=8, selected_profile_path="b.json")
    baseline = build_checkerboard_baseline_snapshot(first)

    assert first.content_sha256 == second.content_sha256
    assert first.left is not first.right
    assert baseline.translation_right_in_left_rdf_m == (0.3, 0.0, 0.0)
    source.extrinsics.translation_left_to_right_m[0] = -99.0
    assert first.checkerboard_extrinsics.translation_left_to_right_m == (-0.3, 0.0, 0.0)


def test_pair_id_is_collision_free_for_10000_sequences_and_scoped_by_session() -> None:
    template = pixhawk_attempt()
    same_session_ids = {
        replace(template, attempt_id=sequence).pair_id for sequence in range(1, 10_001)
    }

    assert len(same_session_ids) == 10_000
    assert replace(template, attempt_id=1, attempt_session_id="session-a").pair_id != replace(
        template, attempt_id=1, attempt_session_id="session-b"
    ).pair_id


@pytest.mark.parametrize(
    ("field_name", "wrong_side"),
    (
        ("left_attitude", "right"),
        ("right_attitude", "left"),
        ("left_gps", "right"),
        ("right_gps", "left"),
    ),
)
def test_attempt_rejects_cross_camera_state_slots(field_name: str, wrong_side: str) -> None:
    attempt = pixhawk_attempt()
    current = getattr(attempt, field_name)

    with pytest.raises(ValueError, match=f"{field_name} must preserve"):
        replace(attempt, **{field_name: replace(current, camera=wrong_side)})


def test_detection_payload_preserves_candidate_identity_for_fail_closed_matching() -> None:
    payload = {
        "timestamp_s": 10.0,
        "width": 1280,
        "height": 720,
        "best_detection": {
            "x1": 630.0,
            "y1": 350.0,
            "x2": 650.0,
            "y2": 370.0,
            "confidence": 0.95,
            "class_id": 2,
            "class_name": "drone",
        },
        "detections": [
            {"class_id": 2},
            {"class_id": 2},
            {"class_id": 7},
        ],
    }
    left = DetectionObservation.from_payload("left", payload)
    right = DetectionObservation.from_payload(
        "right",
        {
            **payload,
            "detections": [{"class_id": 2}],
        },
    )

    assert left is not None and right is not None
    assert left.candidate_class_ids == (2, 2, 7)
    assert correspondence_rejection_reason(left, right) == "ambiguous_left_candidates"
    assert correspondence_rejection_reason(None, right) == "missing_left_detection"
    assert correspondence_rejection_reason(left, None) == "missing_right_detection"


def test_nonidentity_selected_left_attitude_is_shared_by_ray_and_origin() -> None:
    def transpose(matrix):
        return tuple(tuple(matrix[row][column] for row in range(3)) for column in range(3))

    def matrix_vector(matrix, vector):
        return tuple(sum(matrix[row][column] * vector[column] for column in range(3)) for row in range(3))

    def yaw_matrix(angle_deg):
        angle = radians(angle_deg)
        return (
            (cos(angle), -sin(angle), 0.0),
            (sin(angle), cos(angle), 0.0),
            (0.0, 0.0, 1.0),
        )

    camera_to_body = ((0.0, 0.0, 1.0), (1.0, 0.0, 0.0), (0.0, 1.0, 0.0))
    body_to_camera = transpose(camera_to_body)
    left_yaw_deg = 30.0
    right_yaw_deg = 20.0
    left_world_from_body = yaw_matrix(left_yaw_deg)
    right_world_from_body = yaw_matrix(right_yaw_deg)
    baseline_ned = (0.2, 0.3, -0.1)
    translation_left_rdf = matrix_vector(
        body_to_camera,
        matrix_vector(transpose(left_world_from_body), baseline_ned),
    )
    expected_point = (1.2, 0.4, 12.0)
    right_to_target_left = tuple(
        expected_point[index] - translation_left_rdf[index] for index in range(3)
    )
    right_ray_camera = matrix_vector(
        body_to_camera,
        matrix_vector(
            transpose(right_world_from_body),
            matrix_vector(
                left_world_from_body,
                matrix_vector(camera_to_body, right_to_target_left),
            ),
        ),
    )
    left_pixel = (
        640.0 + 900.0 * expected_point[0] / expected_point[2],
        360.0 + 900.0 * expected_point[1] / expected_point[2],
    )
    right_pixel = (
        640.0 + 900.0 * right_ray_camera[0] / right_ray_camera[2],
        360.0 + 900.0 * right_ray_camera[1] / right_ray_camera[2],
    )
    left_source = detection("left", left_pixel[0])
    right_source = detection("right", right_pixel[0])
    left_source = replace(left_source, y1=left_pixel[1] - 10.0, y2=left_pixel[1] + 10.0)
    right_source = replace(right_source, y1=right_pixel[1] - 10.0, y2=right_pixel[1] + 10.0)
    left_q = (cos(radians(left_yaw_deg) / 2.0), 0.0, 0.0, sin(radians(left_yaw_deg) / 2.0))
    right_q = (cos(radians(right_yaw_deg) / 2.0), 0.0, 0.0, sin(radians(right_yaw_deg) / 2.0))
    left_attitude = attitude("left", left_q, timestamp_s=9.7)
    right_attitude = attitude("right", right_q, timestamp_s=9.8)
    left_gps = GpsSnapshot(
        "left", 9.6, 0.0, 0.0, 100.0, 6, 20, 0.01, 0.02,
        "RTK_FIXED", "left", "/left/gps", 8.6,
    )
    right_gps = GpsSnapshot(
        "right",
        9.65,
        degrees(baseline_ned[0] / EARTH_RADIUS_M),
        degrees(baseline_ned[1] / EARTH_RADIUS_M),
        100.0 - baseline_ned[2],
        6,
        20,
        0.01,
        0.02,
        "RTK_FIXED",
        "right",
        "/right/gps",
        8.65,
    )
    baseline = build_pixhawk_baseline_snapshot(
        left_gps=left_gps,
        right_gps=right_gps,
        selected_left_quaternion_wxyz=left_q,
        selected_right_quaternion_wxyz=right_q,
        selected_left_attitude_monotonic_s=left_attitude.timestamp_s,
        created_monotonic_s=10.1,
        detection_midpoint_monotonic_s=10.0,
    )
    attempt = replace(
        pixhawk_attempt(),
        left_detection=DetectionSnapshot.from_observation(
            left_source, pairing_timestamp_basis="detection", pairing_timestamp_s=10.0
        ),
        right_detection=DetectionSnapshot.from_observation(
            right_source, pairing_timestamp_basis="detection", pairing_timestamp_s=10.0
        ),
        left_attitude=left_attitude,
        right_attitude=right_attitude,
        left_gps=left_gps,
        right_gps=right_gps,
        baseline=baseline,
    )

    geometry = calculate_measurement_geometry(attempt)

    assert attempt.baseline.left_basis_quaternion_wxyz == left_attitude.quaternion_wxyz
    assert attempt.baseline.left_basis_attitude_monotonic_s == left_attitude.timestamp_s
    assert geometry.triangulated is not None
    error_m = sum(
        (actual - expected) ** 2
        for actual, expected in zip(geometry.triangulated.point_left_rdf_m, expected_point)
    ) ** 0.5
    assert error_m <= 1e-9
    assert geometry.triangulated.output_frame == "left_camera_rdf"
    assert attempt.measurement_unit == "m"


def test_checkerboard_attempt_keeps_not_used_inputs_and_golden_point() -> None:
    source = calibration_result(checkerboard=True)
    intrinsics = StereoIntrinsicsSnapshot.from_result(
        source, active_zoom_level=8, selected_profile_path="zoom_08.json"
    )
    baseline = build_checkerboard_baseline_snapshot(intrinsics)
    template = pixhawk_attempt()
    checkerboard = replace(
        template,
        triangulation_source="checkerboard",
        left_detection=DetectionSnapshot.from_observation(
            detection("left", 640.0), pairing_timestamp_basis="detection", pairing_timestamp_s=10.0
        ),
        right_detection=DetectionSnapshot.from_observation(
            detection("right", 613.0), pairing_timestamp_basis="detection", pairing_timestamp_s=10.0
        ),
        left_attitude=None,
        right_attitude=None,
        left_gps=None,
        right_gps=None,
        intrinsics=intrinsics,
        baseline=baseline,
    )

    geometry = calculate_measurement_geometry(checkerboard)

    assert checkerboard.left_attitude is None and checkerboard.right_attitude is None
    assert checkerboard.left_gps is None and checkerboard.right_gps is None
    assert checkerboard.baseline.fix_quality == "NOT_USED"
    assert geometry.triangulated is not None
    assert geometry.triangulated.point_left_rdf_m == pytest.approx((0.0, 0.0, 10.0), abs=1e-9)
    assert geometry.triangulated.range_m == pytest.approx(10.0, abs=1e-9)


@pytest.mark.parametrize(
    "factory",
    (
        lambda: replace(attitude("left"), quaternion_wxyz=(0.0, 0.0, 0.0, 0.0)),
        lambda: GpsSnapshot(
            "left", 10.0, float("nan"), 0.0, 100.0, 6, 20, 0.01, 0.02,
            "RTK_FIXED", "left", "/gps", 9.0,
        ),
        lambda: CameraIntrinsicsSnapshot(
            1280,
            720,
            ((900.0, 0.0, 640.0), (0.0, 900.0, 360.0), (0.0, 0.0, 1.0)),
            (),
            0.2,
            20,
        ),
        lambda: CheckerboardExtrinsicsSnapshot(
            ((-1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0)),
            (-0.3, 0.0, 0.0),
            0.1,
            10,
            "opencv-rdf",
        ),
        lambda: CheckerboardExtrinsicsSnapshot(
            ((1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0)),
            (-0.3, 0.0, 0.0),
            0.1,
            10,
            "ned",
        ),
        lambda: replace(
            pixhawk_attempt(),
            measurement_frame="baseline_center_frd",
        ),
        lambda: replace(
            pixhawk_attempt(),
            baseline=replace(pixhawk_attempt().baseline, translation_frame="ned"),
        ),
    ),
)
def test_attempt_inputs_fail_closed_on_invalid_semantics(factory) -> None:
    with pytest.raises(ValueError):
        factory()
