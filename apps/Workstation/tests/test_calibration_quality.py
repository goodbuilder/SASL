import json
from dataclasses import FrozenInstanceError, replace
from math import sqrt

import pytest

from Workstation.workstation_core.calibration_quality import (
    CalibrationQualityPolicy,
    CalibrationSubsetEstimate,
    CenteredRoiPolicy,
    evaluate_camera_calibration_quality,
    extract_view_geometry,
    pose_novelty,
    pose_signature,
    select_pose_representatives,
)


IMAGE_SIZE = (1280, 720)
PATTERN_SIZE = (5, 4)


def _bilinear_grid(
    top_left,
    top_right,
    bottom_left,
    bottom_right,
    *,
    pattern_size=PATTERN_SIZE,
):
    columns, rows = pattern_size
    points = []
    for row in range(rows):
        v = row / (rows - 1)
        left = (
            top_left[0] * (1.0 - v) + bottom_left[0] * v,
            top_left[1] * (1.0 - v) + bottom_left[1] * v,
        )
        right = (
            top_right[0] * (1.0 - v) + bottom_right[0] * v,
            top_right[1] * (1.0 - v) + bottom_right[1] * v,
        )
        for column in range(columns):
            u = column / (columns - 1)
            points.append(
                (
                    left[0] * (1.0 - u) + right[0] * u,
                    left[1] * (1.0 - u) + right[1] * u,
                )
            )
    return points


def _view(
    view_id,
    center_normalized,
    *,
    width_fraction=0.22,
    height_fraction=0.18,
    horizontal_perspective=0.0,
    vertical_perspective=0.0,
):
    image_width, image_height = IMAGE_SIZE
    center_x = center_normalized[0] * image_width
    center_y = center_normalized[1] * image_height
    top_width = width_fraction * image_width * (1.0 + horizontal_perspective)
    bottom_width = width_fraction * image_width * (1.0 - horizontal_perspective)
    left_height = height_fraction * image_height * (1.0 + vertical_perspective)
    right_height = height_fraction * image_height * (1.0 - vertical_perspective)
    corners = _bilinear_grid(
        (center_x - top_width / 2.0, center_y - left_height / 2.0),
        (center_x + top_width / 2.0, center_y - right_height / 2.0),
        (center_x - bottom_width / 2.0, center_y + left_height / 2.0),
        (center_x + bottom_width / 2.0, center_y + right_height / 2.0),
    )
    return extract_view_geometry(
        corners,
        IMAGE_SIZE,
        pattern_size=PATTERN_SIZE,
        view_id=view_id,
    )


def _diverse_views():
    centers = [
        (x, y)
        for y in (0.14, 0.50, 0.86)
        for x in (0.12, 0.38, 0.62, 0.88)
    ]
    horizontal = (-0.18, -0.06, 0.06, 0.18)
    vertical = (-0.16, 0.0, 0.16)
    return tuple(
        _view(
            f"diverse-{index:02d}",
            center,
            width_fraction=0.18 + 0.025 * (index % 4),
            height_fraction=0.15 + 0.025 * (index % 3),
            horizontal_perspective=horizontal[index % 4],
            vertical_perspective=vertical[index % 3],
        )
        for index, center in enumerate(centers)
    )


def _stable_subsets():
    return (
        CalibrationSubsetEstimate("subset-a", 900.0, 905.0, 638.0, 358.0, 10),
        CalibrationSubsetEstimate("subset-b", 910.0, 897.0, 642.0, 362.0, 10),
        CalibrationSubsetEstimate("subset-c", 895.0, 910.0, 640.0, 360.0, 10),
    )


def _camera_matrix(cx=639.5, cy=359.5):
    return ((900.0, 0.0, cx), (0.0, 905.0, cy), (0.0, 0.0, 1.0))


def test_extract_view_geometry_has_explicit_pixel_normalization_and_json_contract():
    geometry = extract_view_geometry(
        ((100.0, 100.0), (300.0, 100.0), (100.0, 200.0), (300.0, 200.0)),
        (400, 300),
        pattern_size=(2, 2),
        view_id="rectangular-board",
    )

    assert geometry.center_normalized == pytest.approx((0.5, 0.5))
    assert geometry.bbox_normalized == pytest.approx((0.25, 1.0 / 3.0, 0.75, 2.0 / 3.0))
    assert geometry.bbox_extent_normalized == pytest.approx((0.5, 1.0 / 3.0))
    assert geometry.coverage_fraction == pytest.approx(1.0 / 6.0)
    assert geometry.scale == pytest.approx(sqrt(1.0 / 6.0))
    assert geometry.roll_deg == pytest.approx(0.0)
    assert geometry.top_bottom_ratio == pytest.approx(1.0)
    assert geometry.left_right_ratio == pytest.approx(1.0)
    json.dumps(geometry.to_mapping(), allow_nan=False)
    with pytest.raises(FrozenInstanceError):
        geometry.scale = 2.0


def test_pose_signature_novelty_and_representative_selection_collapse_duplicates_deterministically():
    small = _view("small", (0.31, 0.35), width_fraction=0.20, height_fraction=0.16)
    best = _view("best", (0.311, 0.351), width_fraction=0.204, height_fraction=0.163)
    distinct = _view("distinct", (0.72, 0.70), horizontal_perspective=0.18)

    assert pose_novelty(best, (small,)) <= 1.0
    assert pose_novelty(distinct, (small,)) > 1.0
    assert pose_signature(small) == pose_signature(small)
    forward = select_pose_representatives((small, distinct, best))
    reverse = select_pose_representatives((best, distinct, small))

    assert tuple(view.view_id for view in forward) == ("best", "distinct")
    assert tuple(view.view_id for view in reverse) == ("best", "distinct")


def test_current_degenerate_like_central_repeated_views_fail_despite_finite_intrinsics():
    repeated = tuple(
        _view(
            f"central-{index:02d}",
            (0.355 + (index % 3) * 0.003, 0.365 + (index % 2) * 0.003),
            width_fraction=0.18 + (index % 2) * 0.002,
            height_fraction=0.15,
            horizontal_perspective=0.005 * (index % 2),
        )
        for index in range(18)
    )
    unstable_subsets = (
        CalibrationSubsetEstimate("a", 900.0, 900.0, 639.0, 359.0, 12),
        CalibrationSubsetEstimate("b", 940.0, 860.0, 872.0, 463.0, 12),
        CalibrationSubsetEstimate("c", 870.0, 930.0, 700.0, 400.0, 12),
    )

    quality = evaluate_camera_calibration_quality(
        _camera_matrix(cx=866.9, cy=422.0),
        IMAGE_SIZE,
        repeated,
        unstable_subsets,
    )

    assert quality.state == "FAIL"
    assert quality.effective_pose_count < 12
    assert "insufficient_effective_poses" in quality.reason_codes
    assert "insufficient_center_grid_coverage" in quality.reason_codes
    assert "insufficient_top_bottom_perspective_diversity" in quality.reason_codes
    assert "subset_principal_x_span_exceeds_limit" in quality.reason_codes
    assert "principal_point_x_offset_exceeds_failure_limit" in quality.reason_codes
    json.dumps(quality.to_mapping(), allow_nan=False)


def test_diverse_views_and_stable_subsets_pass_default_policy():
    quality = evaluate_camera_calibration_quality(
        _camera_matrix(),
        IMAGE_SIZE,
        (view.to_mapping() for view in _diverse_views()),
        (subset.to_mapping() for subset in _stable_subsets()),
    )

    assert quality.state == "PASS"
    assert quality.reason_codes == ()
    assert quality.effective_pose_count == 12
    assert len(quality.center_grid_occupied) == 9
    assert quality.union_extent_normalized[0] >= 0.60
    assert quality.union_extent_normalized[1] >= 0.60
    assert quality.subset_principal_span_px == pytest.approx((4.0, 4.0))
    assert quality.subset_focal_relative_span < 0.03


def test_principal_point_offsets_warn_then_fail_and_documented_roi_can_override_expectation():
    views = _diverse_views()
    subsets = _stable_subsets()
    warning_cx = 639.5 + 0.06 * IMAGE_SIZE[0]
    failing_cx = 639.5 + 0.11 * IMAGE_SIZE[0]

    warning = evaluate_camera_calibration_quality(
        _camera_matrix(cx=warning_cx), IMAGE_SIZE, views, subsets
    )
    failure = evaluate_camera_calibration_quality(
        _camera_matrix(cx=failing_cx), IMAGE_SIZE, views, subsets
    )
    documented_roi_policy = replace(
        CalibrationQualityPolicy(),
        centered_roi_policy=CenteredRoiPolicy(
            expected_normalized=(failing_cx / (IMAGE_SIZE[0] - 1), 0.5)
        ),
    )
    overridden = evaluate_camera_calibration_quality(
        _camera_matrix(cx=failing_cx),
        IMAGE_SIZE,
        views,
        subsets,
        policy=documented_roi_policy,
    )

    assert warning.state == "WARNING"
    assert warning.reason_codes == ("principal_point_x_offset_exceeds_warning_limit",)
    assert failure.state == "FAIL"
    assert "principal_point_x_offset_exceeds_failure_limit" in failure.reason_codes
    assert overridden.state == "PASS"


@pytest.mark.parametrize(
    ("camera_matrix", "reason_code"),
    (
        (((float("nan"), 0.0, 640.0), (0.0, 900.0, 360.0), (0.0, 0.0, 1.0)), "invalid_camera_matrix"),
        (((0.0, 0.0, 640.0), (0.0, 900.0, 360.0), (0.0, 0.0, 1.0)), "non_positive_fx"),
        (((900.0, 0.0, 1280.0), (0.0, 900.0, 360.0), (0.0, 0.0, 1.0)), "cx_out_of_frame"),
    ),
)
def test_hard_intrinsic_validity_is_a_failure(camera_matrix, reason_code):
    quality = evaluate_camera_calibration_quality(
        camera_matrix,
        IMAGE_SIZE,
        _diverse_views(),
        _stable_subsets(),
    )

    assert quality.state == "FAIL"
    assert reason_code in quality.reason_codes


def test_missing_subset_stability_cannot_be_reported_as_pass():
    quality = evaluate_camera_calibration_quality(
        _camera_matrix(),
        IMAGE_SIZE,
        _diverse_views(),
    )

    assert quality.state == "WARNING"
    assert quality.reason_codes == ("subset_stability_not_evaluated",)


def test_extended_solver_uncertainty_and_per_view_error_are_quality_gates():
    quality = evaluate_camera_calibration_quality(
        _camera_matrix(),
        IMAGE_SIZE,
        _diverse_views(),
        _stable_subsets(),
        intrinsic_standard_deviations={
            "fx_px": 40.0,
            "fy_px": 2.0,
            "cx_px": 30.0,
            "cy_px": 1.0,
        },
        per_view_errors_px=(0.15, 0.22, 1.25),
    )

    assert quality.state == "FAIL"
    assert "intrinsic_focal_stddev_exceeds_limit" in quality.reason_codes
    assert "intrinsic_principal_x_stddev_exceeds_limit" in quality.reason_codes
    assert "per_view_reprojection_error_exceeds_limit" in quality.reason_codes
    assert quality.intrinsic_principal_stddev_px == pytest.approx((30.0, 1.0))
    assert quality.per_view_error_max_px == pytest.approx(1.25)
