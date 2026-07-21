from __future__ import annotations

import math
from pathlib import Path
import sys

import pytest


PACKAGE_ROOT = Path(__file__).resolve().parents[1] / "stereo_ground_sim"
sys.path.insert(0, str(PACKAGE_ROOT))

from stereo_ground_sim.coordinate_frames import (
    apply_optical_rpy_error,
    build_baseline_center_frd,
    nominal_camera_poses,
    optical_rpy_error_rotation,
    project_world_point_to_pixel,
)
from stereo_ground_sim.correction_qualification import (
    CandidateProfile,
    IDENTITY_MATRIX,
    predict_candidate,
    score_prediction,
)
from stereo_ground_sim.correction_sweep import CameraContract


def _intrinsics(contract: CameraContract) -> tuple[float, float, float, float]:
    focal = (contract.image_width_px * 0.5) / math.tan(contract.horizontal_fov_rad * 0.5)
    return focal, focal, contract.image_width_px * 0.5, contract.image_height_px * 0.5


def _pixels(
    target_frd_m: tuple[float, float, float],
    left_rpy_deg: tuple[float, float, float],
    right_rpy_deg: tuple[float, float, float],
    contract: CameraContract,
) -> tuple[tuple[float, float], tuple[float, float]]:
    nominal_left, nominal_right = nominal_camera_poses(contract.baseline_m, contract.camera_height_m)
    baseline = build_baseline_center_frd(
        nominal_left.center_world_enu_m,
        nominal_right.center_world_enu_m,
        nominal_left.forward_world_enu,
        nominal_right.forward_world_enu,
    )
    target_world = baseline.point_to_world(target_frd_m)
    actual_left = apply_optical_rpy_error(
        nominal_left, *(math.radians(value) for value in left_rpy_deg)
    )
    actual_right = apply_optical_rpy_error(
        nominal_right, *(math.radians(value) for value in right_rpy_deg)
    )
    fx, fy, cx, cy = _intrinsics(contract)
    left = project_world_point_to_pixel(target_world, actual_left, fx_px=fx, fy_px=fy, cx_px=cx, cy_px=cy)
    right = project_world_point_to_pixel(target_world, actual_right, fx_px=fx, fy_px=fy, cx_px=cx, cy_px=cy)
    assert left is not None and right is not None
    return left, right


def _profile(
    family: str,
    left_rpy_deg: tuple[float, float, float],
    right_rpy_deg: tuple[float, float, float],
) -> CandidateProfile:
    left = optical_rpy_error_rotation(*(math.radians(value) for value in left_rpy_deg))
    right = optical_rpy_error_rotation(*(math.radians(value) for value in right_rpy_deg))
    return CandidateProfile(family, "profile", left, right, None, None)


def test_dual_candidate_recovers_direct_common_relative_pose_without_moving_centers() -> None:
    contract = CameraContract()
    target = (1000.0, 50.0, -5.0)
    left_rpy = (0.045, -0.040, 0.060)
    right_rpy = (0.005, 0.010, 0.130)
    left_pixel, right_pixel = _pixels(target, left_rpy, right_rpy, contract)
    prediction = predict_candidate(
        _profile("dual_camera_wahba_so3", left_rpy, right_rpy),
        left_pixel,
        right_pixel,
        contract,
    )
    score = score_prediction(prediction, target)
    assert prediction.output_frame == "baseline_center_frd"
    assert prediction.position_unit == "m"
    assert prediction.computation_status == "computed"
    assert score["position_error_m"] <= 1.0e-6
    assert prediction.ray_gap_m is not None and prediction.ray_gap_m <= 1.0e-6
    assert prediction.left_depth_m is not None and prediction.left_depth_m > 0.0
    assert prediction.right_depth_m is not None and prediction.right_depth_m > 0.0


def test_raw_is_biased_but_shared_candidate_recovers_common_rotation() -> None:
    contract = CameraContract()
    target = (1000.0, 50.0, -5.0)
    common = (0.05, -0.08, 0.12)
    pixels = _pixels(target, common, common, contract)
    raw = CandidateProfile("raw_nominal", "profile", IDENTITY_MATRIX, IDENTITY_MATRIX, None, None)
    raw_score = score_prediction(predict_candidate(raw, *pixels, contract), target)
    shared_score = score_prediction(
        predict_candidate(_profile("shared_wahba_so3", common, common), *pixels, contract), target
    )
    assert raw_score["position_error_m"] > 1.0e-3
    assert shared_score["position_error_m"] <= 1.0e-6
    assert shared_score["position_error_m"] < raw_score["position_error_m"]


def test_one_shared_rotation_cannot_fully_correct_relative_camera_error() -> None:
    contract = CameraContract()
    target = (1000.0, 50.0, -5.0)
    left_rpy = (-0.05, 0.0, 0.0)
    right_rpy = (0.05, 0.0, 0.0)
    pixels = _pixels(target, left_rpy, right_rpy, contract)
    shared = CandidateProfile("shared_wahba_so3", "profile", IDENTITY_MATRIX, IDENTITY_MATRIX, None, None)
    dual = _profile("dual_camera_wahba_so3", left_rpy, right_rpy)
    shared_score = score_prediction(predict_candidate(shared, *pixels, contract), target)
    dual_score = score_prediction(predict_candidate(dual, *pixels, contract), target)
    assert dual_score["position_error_m"] <= 1.0e-6
    assert shared_score["position_error_m"] > dual_score["position_error_m"] + 1.0e-4


def test_zero_profile_does_not_degrade_nominal_geometry() -> None:
    contract = CameraContract()
    target = (1000.0, 50.0, -5.0)
    pixels = _pixels(target, (0.0, 0.0, 0.0), (0.0, 0.0, 0.0), contract)
    raw = CandidateProfile("raw_nominal", "profile", IDENTITY_MATRIX, IDENTITY_MATRIX, None, None)
    prediction = predict_candidate(raw, *pixels, contract)
    assert score_prediction(prediction, target)["position_error_m"] <= 1.0e-9


def test_candidate_prediction_rejects_nonfinite_pixels() -> None:
    contract = CameraContract()
    raw = CandidateProfile("raw_nominal", "profile", IDENTITY_MATRIX, IDENTITY_MATRIX, None, None)
    with pytest.raises(ValueError, match="finite"):
        predict_candidate(raw, (math.nan, 1.0), (2.0, 3.0), contract)
