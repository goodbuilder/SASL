from __future__ import annotations

import json
from dataclasses import FrozenInstanceError

import pytest
from types import SimpleNamespace

from Workstation.optical_pose_controller import OpticalPoseController
from Workstation.workstation_core.calibration import (
    CameraIntrinsics,
    CheckerboardConfig,
    StereoIntrinsicsResult,
)
from Workstation.workstation_core.operating_intrinsics import (
    NOMINAL_INTRINSICS_QUALITY,
    OperatingCameraEvidence,
    OperatingCameraIntrinsics,
    OperatingIntrinsicsProfile,
    build_nominal_operating_intrinsics_profile,
    evaluate_intrinsics_applicability,
    nominal_profile_to_stereo_result,
)


def test_nominal_zoom_8_builds_hand_checked_1280x720_pinhole() -> None:
    profile = build_nominal_operating_intrinsics_profile(
        zoom_index=8,
        left_width_px=1280,
        left_height_px=720,
        right_width_px=1280,
        right_height_px=720,
    )

    assert profile.left.camera_matrix == (
        (8242.0, 0.0, 639.5),
        (0.0, 8242.0, 359.5),
        (0.0, 0.0, 1.0),
    )
    assert profile.left.distortion_coefficients == (0.0,) * 5
    assert profile.provenance["quality"] == NOMINAL_INTRINSICS_QUALITY
    assert profile.provenance["precision_qualified"] is False
    assert profile.provenance["runtime_truth_used"] is False


def test_nominal_intrinsics_scale_axes_and_preserve_rdf_profile_hash() -> None:
    profile = build_nominal_operating_intrinsics_profile(
        zoom_index=1,
        left_width_px=640,
        left_height_px=360,
        right_width_px=1920,
        right_height_px=1080,
    )

    assert profile.left.camera_matrix[0][0] == pytest.approx(618.0)
    assert profile.left.camera_matrix[1][1] == pytest.approx(618.0)
    assert profile.right.camera_matrix[0][0] == pytest.approx(1854.0)
    assert profile.right.camera_matrix[1][1] == pytest.approx(1854.0)
    assert len(profile.profile_sha256) == 64
    result = nominal_profile_to_stereo_result(profile)
    assert result.extrinsics is None
    assert result.calibration_quality["state"] == NOMINAL_INTRINSICS_QUALITY


def test_nominal_intrinsics_reject_unknown_zoom_and_invalid_raster() -> None:
    with pytest.raises(ValueError, match="no nominal focal"):
        build_nominal_operating_intrinsics_profile(
            zoom_index=99,
            left_width_px=1280,
            left_height_px=720,
            right_width_px=1280,
            right_height_px=720,
        )


def test_optical_pose_uses_nominal_automatically_without_active_calibration(monkeypatch) -> None:
    panel = SimpleNamespace(
        active_intrinsics=None,
        settings=SimpleNamespace(),
    )
    controller = OpticalPoseController(panel)
    evidence = {
        "left": OperatingCameraEvidence("left", 8, 8.0, 99.0, 1280, 720, None, None),
        "right": OperatingCameraEvidence("right", 8, 8.0, 99.0, 1280, 720, None, None),
    }
    monkeypatch.setattr(controller, "_camera_evidence", lambda side, **_kwargs: evidence[side])

    profile = controller._operating_intrinsics_profile()
    assert profile is not None
    assert profile.provenance["quality"] == NOMINAL_INTRINSICS_QUALITY


def test_optical_pose_nominal_fallback_rejects_left_right_zoom_mismatch(monkeypatch) -> None:
    panel = SimpleNamespace(
        active_intrinsics=None,
        settings=SimpleNamespace(),
    )
    controller = OpticalPoseController(panel)
    evidence = {
        "left": OperatingCameraEvidence("left", 8, 8.0, 99.0, 1280, 720, None, None),
        "right": OperatingCameraEvidence("right", 9, 10.0, 99.0, 1280, 720, None, None),
    }
    monkeypatch.setattr(controller, "_camera_evidence", lambda side, **_kwargs: evidence[side])

    assert controller._operating_intrinsics_profile() is None


def test_calibrated_zoom_mismatch_selects_fresh_nominal_profile(monkeypatch) -> None:
    camera = CameraIntrinsics(
        width=1280,
        height=720,
        camera_matrix=[[7000.0, 0.0, 639.5], [0.0, 7000.0, 359.5], [0.0, 0.0, 1.0]],
        distortion_coefficients=[0.01, 0.0, 0.0, 0.0, 0.0],
        reprojection_error_px=0.2,
        image_count=20,
    )
    active = StereoIntrinsicsResult(
        left=camera,
        right=camera,
        board=CheckerboardConfig(8, 6, 0.04),
    )
    panel = SimpleNamespace(
        active_intrinsics=active,
        _active_intrinsics_source_zoom_level=7,
        _active_intrinsics_profile_hash="a" * 64,
        _active_intrinsics_profile_path="calibration/zoom-07.json",
        operating_intrinsics_identity={},
        selected_extrinsics_profile=None,
    )
    controller = OpticalPoseController(panel)
    evidence = {
        "left": OperatingCameraEvidence("left", 8, 8.0, 99.0, 1280, 720, None, None),
        "right": OperatingCameraEvidence("right", 8, 8.0, 99.0, 1280, 720, None, None),
    }
    monkeypatch.setattr(controller, "_camera_evidence", lambda side, **_kwargs: evidence[side])
    monkeypatch.setattr("Workstation.optical_pose_controller.time.monotonic", lambda: 100.0)

    snapshot = controller._intrinsics_snapshot(use_frozen=False)

    assert snapshot["applicability"]["geometry_allowed"] is True
    assert snapshot["profile"]["provenance"]["quality"] == NOMINAL_INTRINSICS_QUALITY
    assert snapshot["profile"]["left"]["source_zoom_index"] == 8


def test_calibrated_zoom_mismatch_does_not_use_stale_nominal_profile(monkeypatch) -> None:
    camera = CameraIntrinsics(
        width=1280,
        height=720,
        camera_matrix=[[7000.0, 0.0, 639.5], [0.0, 7000.0, 359.5], [0.0, 0.0, 1.0]],
        distortion_coefficients=[0.01, 0.0, 0.0, 0.0, 0.0],
        reprojection_error_px=0.2,
        image_count=20,
    )
    panel = SimpleNamespace(
        active_intrinsics=StereoIntrinsicsResult(
            left=camera,
            right=camera,
            board=CheckerboardConfig(8, 6, 0.04),
        ),
        _active_intrinsics_source_zoom_level=7,
        _active_intrinsics_profile_hash="a" * 64,
        _active_intrinsics_profile_path="calibration/zoom-07.json",
        operating_intrinsics_identity={},
        selected_extrinsics_profile=None,
    )
    controller = OpticalPoseController(panel)
    stale = {
        side: OperatingCameraEvidence(side, 8, 8.0, 90.0, 1280, 720, None, None)
        for side in ("left", "right")
    }
    monkeypatch.setattr(controller, "_camera_evidence", lambda side, **_kwargs: stale[side])
    monkeypatch.setattr("Workstation.optical_pose_controller.time.monotonic", lambda: 100.0)

    snapshot = controller._intrinsics_snapshot(use_frozen=False)

    assert snapshot["applicability"]["geometry_allowed"] is False
    assert "left_operating_zoom_mismatch" in snapshot["applicability"]["reasons"]
    with pytest.raises(ValueError, match="positive native raster"):
        build_nominal_operating_intrinsics_profile(
            zoom_index=8,
            left_width_px=0,
            left_height_px=720,
            right_width_px=1280,
            right_height_px=720,
        )


def _camera(side: str, zoom: int | None = 12) -> OperatingCameraIntrinsics:
    return OperatingCameraIntrinsics(
        side=side,
        width_px=1920,
        height_px=1080,
        camera_matrix=(
            (7200.0, 0.0, 960.0),
            (0.0, 7195.0, 540.0),
            (0.0, 0.0, 1.0),
        ),
        distortion_coefficients=(0.01, -0.02, 0.0, 0.0, 0.0),
        source_zoom_index=zoom,
        source_zoom_ratio=None if zoom is None else float(zoom),
        camera_serial=f"SERIAL-{side.upper()}",
        stabilization_state="enabled",
    )


def _profile(zoom: int | None = 12) -> OperatingIntrinsicsProfile:
    return OperatingIntrinsicsProfile(
        left=_camera("left", zoom),
        right=_camera("right", zoom),
        profile_id="field-operating-profile",
        source_path="intrinsics_zoom_12.json",
        provenance_json=json.dumps(
            {
                "operator": "test",
                "capture_ids": [3, 1, 2],
                "nested": {"method": "checkerboard"},
            }
        ),
    )


def _evidence(side: str, zoom: int = 12, *, updated: float = 99.0) -> OperatingCameraEvidence:
    return OperatingCameraEvidence(
        side=side,
        zoom_index=zoom,
        zoom_ratio=float(zoom),
        zoom_updated_monotonic_s=updated,
        frame_width_px=1920,
        frame_height_px=1080,
        camera_serial=f"SERIAL-{side.upper()}",
        stabilization_state="ENABLED",
    )


def test_cross_zoom_3_to_12_keeps_exact_operating_kd_ready() -> None:
    result = evaluate_intrinsics_applicability(
        _profile(12),
        _evidence("left", 12),
        _evidence("right", 12),
        now_monotonic_s=100.0,
        extrinsics_source_zoom_index=3,
    )

    assert result.state == "READY"
    assert result.geometry_allowed is True
    assert result.raw_recording_allowed is True
    assert result.reasons == ()
    assert result.operating_zoom_index == 12
    assert result.extrinsics_source_zoom_index == 3
    assert result.cross_zoom is True


def test_same_zoom_7_to_7_is_ready_and_not_cross_zoom() -> None:
    result = evaluate_intrinsics_applicability(
        _profile(7),
        _evidence("left", 7),
        _evidence("right", 7),
        now_monotonic_s=100.0,
        extrinsics_source_zoom_index=7,
    )

    assert result.state == "READY"
    assert result.geometry_allowed is True
    assert result.cross_zoom is False


def test_operating_mismatch_blocks_geometry_but_not_raw_recording() -> None:
    left = _evidence("left", 11, updated=80.0)
    right = OperatingCameraEvidence(
        side="right",
        zoom_index=12,
        zoom_ratio=12.0,
        zoom_updated_monotonic_s=99.0,
        frame_width_px=1280,
        frame_height_px=720,
        camera_serial="WRONG-SERIAL",
        stabilization_state="disabled",
    )

    result = evaluate_intrinsics_applicability(
        _profile(12),
        left,
        right,
        now_monotonic_s=100.0,
        max_zoom_age_s=5.0,
        extrinsics_source_zoom_index=3,
    )

    assert result.state == "BLOCKED"
    assert result.geometry_allowed is False
    assert result.raw_recording_allowed is True
    assert "left_zoom_readback_stale" in result.reasons
    assert "left_operating_zoom_mismatch" in result.reasons
    assert "left_right_zoom_disagree" in result.reasons
    assert "right_frame_resolution_mismatch" in result.reasons
    assert "right_camera_serial_mismatch" in result.reasons
    assert "right_camera_stabilization_mismatch" in result.reasons
    assert result.cross_zoom is None


def test_legacy_unknown_zoom_is_not_replaced_by_requested_zoom() -> None:
    camera = CameraIntrinsics(
        width=1920,
        height=1080,
        camera_matrix=[
            [7200.0, 0.0, 960.0],
            [0.0, 7195.0, 540.0],
            [0.0, 0.0, 1.0],
        ],
        distortion_coefficients=[0.0, 0.0, 0.0, 0.0, 0.0],
        reprojection_error_px=0.2,
        image_count=20,
    )
    legacy = StereoIntrinsicsResult(
        left=camera,
        right=camera,
        board=CheckerboardConfig(9, 6, 0.04),
    )
    profile = OperatingIntrinsicsProfile.from_stereo_result(
        legacy,
        source_zoom_index=None,
        left_camera_serial="SERIAL-LEFT",
        right_camera_serial="SERIAL-RIGHT",
        left_stabilization_state="enabled",
        right_stabilization_state="enabled",
    )

    assert profile.left.source_zoom_index is None
    assert profile.right.source_zoom_index is None
    result = evaluate_intrinsics_applicability(
        profile,
        _evidence("left", 12),
        _evidence("right", 12),
        now_monotonic_s=100.0,
        extrinsics_source_zoom_index=3,
    )
    assert result.state == "BLOCKED"
    assert "left_intrinsics_zoom_unknown" in result.reasons
    assert "right_intrinsics_zoom_unknown" in result.reasons
    assert result.raw_recording_allowed is True


def test_profile_hashes_round_trip_and_nested_inputs_cannot_mutate_profile() -> None:
    profile = _profile()
    math_hash = profile.intrinsics_math_sha256
    profile_hash = profile.profile_sha256
    payload = profile.to_mapping()

    loaded = OperatingIntrinsicsProfile.from_mapping(
        json.loads(json.dumps(payload, ensure_ascii=False))
    )

    assert loaded.intrinsics_math_sha256 == math_hash
    assert loaded.profile_sha256 == profile_hash
    assert len(math_hash) == 64
    assert len(profile_hash) == 64
    payload["left"]["camera_matrix"][0][0] = 1.0
    payload["provenance"]["nested"]["method"] = "changed"
    detached = loaded.provenance
    detached["nested"]["method"] = "also changed"
    assert loaded.left.camera_matrix[0][0] == 7200.0
    assert loaded.provenance["nested"]["method"] == "checkerboard"
    assert loaded.profile_sha256 == profile_hash
    with pytest.raises(FrozenInstanceError):
        loaded.profile_id = "mutated"  # type: ignore[misc]


def test_profile_round_trip_detects_hash_tampering() -> None:
    payload = _profile().to_mapping()
    payload["left"]["camera_matrix"][0][0] = 7100.0

    with pytest.raises(ValueError, match="intrinsics_math_sha256"):
        OperatingIntrinsicsProfile.from_mapping(payload)


@pytest.mark.parametrize(
    "factory",
    (
        lambda: OperatingCameraIntrinsics(
            side="left",
            width_px=1920,
            height_px=1080,
            camera_matrix=((float("nan"), 0.0, 960.0), (0.0, 1.0, 540.0), (0.0, 0.0, 1.0)),
            distortion_coefficients=(0.0, 0.0, 0.0, 0.0, 0.0),
            source_zoom_index=12,
        ),
        lambda: OperatingCameraEvidence(
            side="left",
            zoom_index=12,
            zoom_ratio=float("nan"),
            zoom_updated_monotonic_s=1.0,
            frame_width_px=1920,
            frame_height_px=1080,
            camera_serial="SERIAL-LEFT",
            stabilization_state="enabled",
        ),
        lambda: OperatingIntrinsicsProfile(
            left=_camera("left"),
            right=_camera("right"),
            provenance_json='{"bad": NaN}',
        ),
    ),
)
def test_nan_is_rejected(factory) -> None:
    with pytest.raises(ValueError, match="finite"):
        factory()


def test_missing_profile_blocks_geometry_and_retains_raw_capture() -> None:
    result = evaluate_intrinsics_applicability(
        None,
        _evidence("left"),
        _evidence("right"),
        now_monotonic_s=100.0,
        extrinsics_source_zoom_index=3,
    )

    assert result.state == "BLOCKED"
    assert result.reasons == ("operating_intrinsics_profile_missing",)
    assert result.geometry_allowed is False
    assert result.raw_recording_allowed is True
    assert result.intrinsics_math_sha256 == ""
    assert result.profile_sha256 == ""
