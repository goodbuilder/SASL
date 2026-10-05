from __future__ import annotations

import hashlib
import json
from dataclasses import FrozenInstanceError
from math import cos, sin
from pathlib import Path

import pytest

from Workstation.workstation_core.hr_evidence import (
    BridgeCameraIntrinsics,
    BridgeCompatibilityEvidence,
    BridgeCorrespondenceView,
    BridgeObservationModel,
    CapturePurpose,
    HrCaptureEvent,
    HrOzBridgeProfile,
    HrOzSideBridge,
    create_hr_capture_event,
    evaluate_bridge_applicability,
    hr_frame,
    import_hr_image_file,
    oz_frame,
    solve_full_se3_hr_to_oz_bridge,
    solve_rotation_only_hr_to_oz_bridge,
    transfer_hr_stereo_pose_to_oz,
)
from Workstation.workstation_core.optical_pose import SE3Transform, StereoPoseRdf


def _identity():
    return ((1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0))


def _rotation_z(angle_rad: float):
    return (
        (cos(angle_rad), -sin(angle_rad), 0.0),
        (sin(angle_rad), cos(angle_rad), 0.0),
        (0.0, 0.0, 1.0),
    )


def _matvec(matrix, vector):
    return tuple(
        sum(matrix[row][column] * vector[column] for column in range(3))
        for row in range(3)
    )


def _side_bridge(
    side: str,
    *,
    observation_model: BridgeObservationModel = BridgeObservationModel.FULL_SE3,
    hr_serial: str | None = None,
    oz_serial: str | None = None,
    hr_hash: str = "a" * 64,
    oz_hash: str = "b" * 64,
) -> HrOzSideBridge:
    return HrOzSideBridge(
        side=side,
        transform_oz_from_hr=SE3Transform(
            output_frame=oz_frame(side),
            input_frame=hr_frame(side),
            rotation_output_from_input=_identity(),
            translation_output_from_input_m=(
                (0.0, 0.0, 0.0)
                if observation_model is BridgeObservationModel.ROTATION_ONLY
                else (0.01, -0.02, 0.03)
            ),
        ),
        observation_model=observation_model,
        hr_camera_serial=hr_serial or f"{side.upper()}-HR",
        oz_camera_serial=oz_serial or f"{side.upper()}-OZ",
        hr_intrinsics_sha256=hr_hash,
        oz_intrinsics_sha256=oz_hash,
        method="synthetic",
        view_count=6,
        point_count=54,
        rms_reprojection_error_px=(
            None
            if observation_model is BridgeObservationModel.ROTATION_ONLY
            else 0.1
        ),
        angular_rms_error_deg=(
            0.01
            if observation_model is BridgeObservationModel.ROTATION_ONLY
            else None
        ),
    )


@pytest.mark.parametrize(
    "purpose",
    (
        CapturePurpose.PRE,
        CapturePurpose.POST,
        CapturePurpose.CHECKPOINT,
        CapturePurpose.UNSPECIFIED,
    ),
)
def test_hr_import_preserves_exact_bytes_hash_resolution_serial_and_purpose(
    tmp_path: Path,
    purpose: CapturePurpose,
) -> None:
    cv2 = pytest.importorskip("cv2")
    np = pytest.importorskip("numpy")
    pixels = np.zeros((7, 11, 3), dtype=np.uint8)
    pixels[:, :, 1] = 137
    ok, encoded = cv2.imencode(".png", pixels)
    assert ok
    exact_bytes = bytes(encoded) + b"HR-EVIDENCE-TRAILER"
    source = tmp_path / "left_hr.png"
    source.write_bytes(exact_bytes)

    imported = import_hr_image_file(
        source,
        side="left",
        camera_serial="WIRIS-L-001",
        expected_sha256=hashlib.sha256(exact_bytes).hexdigest(),
        expected_width_px=11,
        expected_height_px=7,
        imported_utc_ns=123,
    )
    event = create_hr_capture_event(
        "capture-01",
        purpose=purpose,
        left=imported,
        captured_utc_ns=456,
    )

    assert imported.file_bytes == exact_bytes
    assert imported.byte_size == len(exact_bytes)
    assert imported.sha256 == hashlib.sha256(exact_bytes).hexdigest()
    assert (imported.width_px, imported.height_px) == (11, 7)
    assert imported.camera_serial == "WIRIS-L-001"
    assert event.purpose is purpose
    mapping = event.to_mapping()
    assert mapping["left"]["byte_size"] == len(exact_bytes)
    assert mapping["left"]["sha256"] == imported.sha256
    assert "file_bytes" not in mapping["left"]
    json.dumps(mapping, allow_nan=False)

    materialized = imported.write_exact(tmp_path / "stored" / "left_hr.png")
    assert materialized.read_bytes() == exact_bytes


def test_hr_import_rejects_hash_and_resolution_mismatch(tmp_path: Path) -> None:
    cv2 = pytest.importorskip("cv2")
    np = pytest.importorskip("numpy")
    ok, encoded = cv2.imencode(".png", np.zeros((4, 6), dtype=np.uint8))
    assert ok
    source = tmp_path / "hr.png"
    source.write_bytes(bytes(encoded))

    with pytest.raises(ValueError, match="SHA-256 mismatch"):
        import_hr_image_file(
            source,
            side="right",
            camera_serial="R",
            expected_sha256="0" * 64,
        )
    with pytest.raises(ValueError, match="width mismatch"):
        import_hr_image_file(
            source,
            side="right",
            camera_serial="R",
            expected_width_px=7,
        )


def test_capture_event_and_bridge_profile_are_immutable_and_hash_round_trip() -> None:
    event = HrCaptureEvent(
        event_id="event-01",
        capture_id="capture-01",
        purpose="checkpoint",
        captured_utc_ns=100,
    )
    profile = HrOzBridgeProfile(
        profile_id="bridge-01",
        left=_side_bridge("left"),
        right=_side_bridge("right"),
        created_utc_ns=200,
        source_event_ids=(event.event_id,),
    )
    payload = profile.to_mapping()
    loaded = HrOzBridgeProfile.from_mapping(json.loads(json.dumps(payload)))

    assert loaded == profile
    assert loaded.profile_sha256 == profile.profile_sha256
    with pytest.raises(FrozenInstanceError):
        event.capture_id = "changed"  # type: ignore[misc]
    with pytest.raises(FrozenInstanceError):
        profile.profile_id = "changed"  # type: ignore[misc]

    payload["left"]["method"] = "tampered"
    with pytest.raises(ValueError, match="profile_sha256 mismatch"):
        HrOzBridgeProfile.from_mapping(payload)


def test_bridge_applicability_blocks_serial_and_intrinsics_hash_mismatch() -> None:
    profile = HrOzBridgeProfile(
        profile_id="bridge-identity",
        left=_side_bridge("left"),
        right=_side_bridge("right"),
        created_utc_ns=1,
    )
    left = BridgeCompatibilityEvidence(
        side="left",
        hr_camera_serial="WRONG-HR",
        oz_camera_serial="LEFT-OZ",
        hr_intrinsics_sha256="c" * 64,
        oz_intrinsics_sha256="b" * 64,
    )
    right = BridgeCompatibilityEvidence(
        side="right",
        hr_camera_serial="RIGHT-HR",
        oz_camera_serial="WRONG-OZ",
        hr_intrinsics_sha256="a" * 64,
        oz_intrinsics_sha256="d" * 64,
    )

    applicability = evaluate_bridge_applicability(profile, left, right)

    assert applicability.state == "BLOCKED"
    assert applicability.geometry_allowed is False
    assert applicability.stereo_translation_transfer_allowed is False
    assert applicability.raw_evidence_allowed is True
    assert applicability.reasons == (
        "left_hr_camera_serial_mismatch",
        "left_hr_intrinsics_sha256_mismatch",
        "right_oz_camera_serial_mismatch",
        "right_oz_intrinsics_sha256_mismatch",
    )


def test_rotation_only_solver_recovers_direction_and_marks_translation_unobservable() -> None:
    expected_rotation = _rotation_z(0.17)
    hr_rays = (
        (0.0, 0.0, 1.0),
        (0.4, 0.1, 1.0),
        (-0.3, 0.25, 0.9),
        (0.2, -0.5, 1.2),
        (-0.6, -0.2, 1.4),
    )
    oz_rays = tuple(_matvec(expected_rotation, ray) for ray in hr_rays)

    bridge = solve_rotation_only_hr_to_oz_bridge(
        hr_rays,
        oz_rays,
        side="left",
        hr_camera_serial="LEFT-HR",
        oz_camera_serial="LEFT-OZ",
        hr_intrinsics_sha256="a" * 64,
        oz_intrinsics_sha256="b" * 64,
    )

    assert bridge.observation_model is BridgeObservationModel.ROTATION_ONLY
    assert bridge.translation_observable is False
    assert bridge.translation_contract == "zero_serialization_sentinel_translation_not_observable"
    assert bridge.transform_oz_from_hr.translation_output_from_input_m == (0.0, 0.0, 0.0)
    for actual_row, expected_row in zip(
        bridge.transform_oz_from_hr.rotation_output_from_input,
        expected_rotation,
    ):
        assert actual_row == pytest.approx(expected_row, abs=1.0e-12)
    assert bridge.angular_rms_error_deg == pytest.approx(0.0, abs=1.0e-6)

    profile = HrOzBridgeProfile(
        profile_id="rotation-only",
        left=bridge,
        right=_side_bridge("right"),
        created_utc_ns=2,
    )
    assert profile.stereo_translation_transfer_allowed is False


def test_rotation_only_contract_rejects_invented_translation() -> None:
    with pytest.raises(ValueError, match="translation is not observable"):
        HrOzSideBridge(
            side="left",
            transform_oz_from_hr=SE3Transform(
                output_frame=oz_frame("left"),
                input_frame=hr_frame("left"),
                rotation_output_from_input=_identity(),
                translation_output_from_input_m=(0.001, 0.0, 0.0),
            ),
            observation_model="rotation_only",
            hr_camera_serial="LEFT-HR",
            oz_camera_serial="LEFT-OZ",
            hr_intrinsics_sha256="a" * 64,
            oz_intrinsics_sha256="b" * 64,
            method="invalid",
            view_count=1,
            point_count=3,
        )


def test_rotation_only_profile_cannot_invent_stereo_translation_transfer() -> None:
    profile = HrOzBridgeProfile(
        profile_id="mixed-observability",
        left=_side_bridge("left", observation_model=BridgeObservationModel.ROTATION_ONLY),
        right=_side_bridge("right"),
        created_utc_ns=3,
    )
    hr_stereo = StereoPoseRdf(
        rotation_right_from_left_rdf=_identity(),
        translation_right_from_left_rdf_m=(-25.0, 0.0, 0.0),
    )

    with pytest.raises(ValueError, match="translation is not observable"):
        transfer_hr_stereo_pose_to_oz(hr_stereo, profile)


def test_full_se3_opencv_solver_recovers_x_oz_from_hr_direction() -> None:
    cv2 = pytest.importorskip("cv2")
    np = pytest.importorskip("numpy")

    hr_intrinsics = BridgeCameraIntrinsics(
        side="right",
        channel="hr",
        width_px=1920,
        height_px=1080,
        camera_matrix=((1700.0, 0.0, 960.0), (0.0, 1680.0, 540.0), (0.0, 0.0, 1.0)),
        distortion_coefficients=(0.0, 0.0, 0.0, 0.0, 0.0),
        camera_serial="RIGHT-HR",
        intrinsics_sha256="a" * 64,
    )
    oz_intrinsics = BridgeCameraIntrinsics(
        side="right",
        channel="oz",
        width_px=1280,
        height_px=720,
        camera_matrix=((1120.0, 0.0, 640.0), (0.0, 1135.0, 360.0), (0.0, 0.0, 1.0)),
        distortion_coefficients=(0.0, 0.0, 0.0, 0.0, 0.0),
        camera_serial="RIGHT-OZ",
        intrinsics_sha256="b" * 64,
    )
    expected_rotation, _ = cv2.Rodrigues(np.asarray((0.012, -0.018, 0.025), dtype=np.float64))
    expected_translation = np.asarray((0.038, -0.006, 0.011), dtype=np.float64).reshape(3, 1)
    object_points = np.asarray(
        [(column * 0.035, row * 0.035, 0.0) for row in range(6) for column in range(9)],
        dtype=np.float64,
    )
    views = []
    for index in range(8):
        board_rvec_hr = np.asarray(
            (0.025 * (index - 3), -0.018 * (index % 4), 0.012 * index),
            dtype=np.float64,
        )
        board_rotation_hr, _ = cv2.Rodrigues(board_rvec_hr)
        board_translation_hr = np.asarray(
            (-0.16 + 0.045 * index, -0.10 + 0.025 * (index % 3), 3.2 + 0.23 * index),
            dtype=np.float64,
        ).reshape(3, 1)
        board_rotation_oz = expected_rotation @ board_rotation_hr
        board_translation_oz = expected_rotation @ board_translation_hr + expected_translation
        board_rvec_oz, _ = cv2.Rodrigues(board_rotation_oz)
        hr_pixels, _ = cv2.projectPoints(
            object_points,
            board_rvec_hr,
            board_translation_hr,
            np.asarray(hr_intrinsics.camera_matrix),
            np.asarray(hr_intrinsics.distortion_coefficients),
        )
        oz_pixels, _ = cv2.projectPoints(
            object_points,
            board_rvec_oz,
            board_translation_oz,
            np.asarray(oz_intrinsics.camera_matrix),
            np.asarray(oz_intrinsics.distortion_coefficients),
        )
        views.append(
            BridgeCorrespondenceView(
                object_points_m=tuple(tuple(float(v) for v in point) for point in object_points),
                hr_pixels_px=tuple(tuple(float(v) for v in point) for point in hr_pixels.reshape(-1, 2)),
                oz_pixels_px=tuple(tuple(float(v) for v in point) for point in oz_pixels.reshape(-1, 2)),
            )
        )

    bridge = solve_full_se3_hr_to_oz_bridge(
        views,
        hr_intrinsics=hr_intrinsics,
        oz_intrinsics=oz_intrinsics,
    )

    assert bridge.observation_model is BridgeObservationModel.FULL_SE3
    assert bridge.translation_observable is True
    assert bridge.transform_oz_from_hr.input_frame == "right_hr_optical_rdf"
    assert bridge.transform_oz_from_hr.output_frame == "right_oz_optical_rdf"
    actual_rotation = np.asarray(bridge.transform_oz_from_hr.rotation_output_from_input)
    actual_translation = np.asarray(bridge.transform_oz_from_hr.translation_output_from_input_m)
    assert actual_rotation == pytest.approx(expected_rotation, abs=2.0e-5)
    assert actual_translation == pytest.approx(expected_translation.ravel(), abs=2.0e-5)

    point_hr = np.asarray((0.7, -0.4, 8.0), dtype=np.float64)
    expected_point_oz = (expected_rotation @ point_hr.reshape(3, 1) + expected_translation).ravel()
    assert bridge.transform_point(point_hr) == pytest.approx(expected_point_oz, abs=2.0e-5)
    assert bridge.rms_reprojection_error_px is not None
    assert bridge.rms_reprojection_error_px < 1.0e-3
