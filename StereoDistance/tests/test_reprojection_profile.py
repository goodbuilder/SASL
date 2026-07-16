from types import SimpleNamespace

from wiris_stereo_distance.calibration import CameraIntrinsics, CheckerboardConfig, StereoIntrinsicsResult
from wiris_stereo_distance.measurement_controller import MeasurementGeometryInput, calculate_measurement_geometry
from wiris_stereo_distance.reprojection_profile import (
    FALLBACK_EXACT,
    FALLBACK_NO_CELL,
    OriginBBoxReprojectionProfile,
    RectifiedStereoCameraModel,
    ReprojectionResidualSample,
    calibration_envelope_from_intrinsics,
    load_reprojection_profile,
    residual_sample_from_row,
)
from wiris_stereo_distance.stereo import DetectionObservation, StereoCalibration


def detection(camera: str, *, cx: float, cy: float = 360.0) -> DetectionObservation:
    return DetectionObservation(
        camera=camera,
        timestamp_s=10.0,
        width=1280,
        height=720,
        x1=cx - 10.0,
        y1=cy - 10.0,
        x2=cx + 10.0,
        y2=cy + 10.0,
        confidence=0.9,
        class_id=1,
        class_name="target",
    )


def intrinsics_result() -> StereoIntrinsicsResult:
    intrinsics = CameraIntrinsics(
        width=1280,
        height=720,
        camera_matrix=[[900.0, 0.0, 640.0], [0.0, 900.0, 360.0], [0.0, 0.0, 1.0]],
        distortion_coefficients=[0.0],
        reprojection_error_px=0.2,
        image_count=20,
        coverage_summary={"status": "ok", "warnings": []},
    )
    return StereoIntrinsicsResult(
        left=intrinsics,
        right=intrinsics,
        board=CheckerboardConfig(inner_cols=9, inner_rows=6, square_size_m=0.03),
        stereo_calibration={
            "method": "cv2.stereoCalibrate",
            "pair_count": 18,
            "rms_error_px": 0.22,
            "epipolar_mean_error_px": 0.08,
            "epipolar_max_error_px": 0.2,
        },
    )


def pixhawk_attitude() -> SimpleNamespace:
    return SimpleNamespace(q=(1.0, 0.0, 0.0, 0.0), timestamp_s=10.0)


def test_reprojection_residual_sample_uses_gt_pixel_label() -> None:
    camera = RectifiedStereoCameraModel(
        image_width_px=1280.0,
        image_height_px=720.0,
        focal_length_px=900.0,
        baseline_m=0.3,
    )
    projected = camera.project_truth_center(right_m=0.0, down_m=0.0, forward_m=9.0)
    row = {
        "left_cx": str(projected["left_u"] - 2.0),
        "left_cy": str(projected["left_v"] + 3.0),
        "right_cx": str(projected["right_u"] + 4.0),
        "right_cy": str(projected["right_v"] - 5.0),
        "truth_right_m": "0.0",
        "truth_down_m": "0.0",
        "truth_forward_m": "9.0",
        "raw_disparity_px": "30.0",
    }

    sample = residual_sample_from_row(row, camera)

    assert sample.du_left == 2.0
    assert sample.dv_left == -3.0
    assert sample.du_right == -4.0
    assert sample.dv_right == 5.0


def test_rectified_camera_model_triangulates_centers_with_rays() -> None:
    camera = RectifiedStereoCameraModel(
        image_width_px=1280.0,
        image_height_px=720.0,
        focal_length_px=900.0,
        baseline_m=0.3,
    )

    result = camera.triangulate_centers(
        left_u=665.0,
        left_v=360.0,
        right_u=615.0,
        right_v=360.0,
    )

    assert tuple(round(result[key], 6) for key in ("right_m", "down_m", "forward_m")) == (0.0, 0.0, 5.4)
    assert round(result["range_m"], 6) == 5.4
    assert result["disparity_px"] == 50.0


def test_profile_lookup_uses_raw_disparity_without_range() -> None:
    sample = {
        "du_left": 2.0,
        "dv_left": -1.0,
        "du_right": -2.0,
        "dv_right": 1.0,
        "sample_count": 5,
    }
    profile = OriginBBoxReprojectionProfile(
        profile_id="disparity_lookup",
        image_width_px=1280.0,
        image_height_px=720.0,
        u_edges=(0.0, 1280.0),
        v_edges=(0.0, 720.0),
        depth_edges=(0.5, 100.0),
        cells={"0:0:0": sample},
        bin_axis="disparity",
    )

    result = profile.apply(650.0, 360.0, 620.0, 360.0, raw_disparity_px=30.0)

    assert result.fallback_level == FALLBACK_EXACT
    assert result.corrected_left_u == 652.0
    assert result.corrected_right_u == 618.0


def test_profile_disparity_only_applies_separation_without_screen_shift() -> None:
    profile = OriginBBoxReprojectionProfile(
        profile_id="disparity_only",
        image_width_px=1280.0,
        image_height_px=720.0,
        u_edges=(0.0, 1280.0),
        v_edges=(0.0, 720.0),
        depth_edges=(0.5, 100.0),
        cells={
            "0:0:0": {
                "du_left": 6.0,
                "dv_left": 10.0,
                "du_right": 2.0,
                "dv_right": -2.0,
                "sample_count": 5,
            }
        },
        bin_axis="disparity",
        correction_mode="disparity_only",
    )

    result = profile.apply(650.0, 360.0, 620.0, 360.0, raw_disparity_px=30.0)

    assert result.fallback_level == FALLBACK_EXACT
    assert result.corrected_left_u == 652.0
    assert result.corrected_right_u == 618.0
    assert result.corrected_left_v == 360.0
    assert result.corrected_right_v == 360.0
    assert result.du_left == 2.0
    assert result.du_right == -2.0
    assert result.dv_left == 0.0
    assert result.dv_right == 0.0


def test_profile_build_defaults_to_conservative_component_gain_policy() -> None:
    profile = OriginBBoxReprojectionProfile.build(
        [
            ReprojectionResidualSample(
                left_u=650.0,
                left_v=360.0,
                right_u=620.0,
                right_v=360.0,
                raw_disparity_px=30.0,
                raw_range_m=None,
                du_left=6.0,
                dv_left=10.0,
                du_right=2.0,
                dv_right=-2.0,
            )
        ],
        image_width_px=1280.0,
        image_height_px=720.0,
        u_bins=1,
        v_bins=1,
        depth_bins=1,
        min_samples_per_cell=1,
    )

    result = profile.apply(650.0, 360.0, 620.0, 360.0, raw_disparity_px=30.0)

    assert profile.correction_mode == "component_gains"
    assert profile.output_policy == "range_only_forward_fusion"
    assert result.corrected_left_u == 650.25
    assert result.corrected_right_u == 619.75
    assert result.corrected_left_v == 360.1
    assert result.corrected_right_v == 360.1


def test_profile_load_defaults_old_json_to_conservative_component_gain_policy(tmp_path) -> None:
    path = tmp_path / "legacy_reprojection.json"
    path.write_text(
        """
{
  "profile_id": "legacy",
  "image_width_px": 1280,
  "image_height_px": 720,
  "bin_axis": "disparity",
  "grid": {
    "u_edges": [0, 1280],
    "v_edges": [0, 720],
    "depth_edges": [0.5, 100]
  },
  "cells": {
    "0:0:0": {
      "du_left": 6,
      "dv_left": 10,
      "du_right": 2,
      "dv_right": -2,
      "sample_count": 5
    }
  }
}
""",
        encoding="utf-8",
    )

    profile = load_reprojection_profile(path)
    result = profile.apply(650.0, 360.0, 620.0, 360.0, raw_disparity_px=30.0)

    assert profile.correction_mode == "component_gains"
    assert profile.calibration_envelope == {}
    assert result.corrected_left_u == 650.25
    assert result.corrected_right_u == 619.75


def test_profile_serializes_calibration_envelope() -> None:
    envelope = calibration_envelope_from_intrinsics(
        intrinsics_result(),
        intrinsics_path="intrinsics_zoom_08.json",
        profile_source="test-profile",
    )
    profile = OriginBBoxReprojectionProfile(
        profile_id="enveloped",
        image_width_px=1280.0,
        image_height_px=720.0,
        u_edges=(0.0, 1280.0),
        v_edges=(0.0, 720.0),
        depth_edges=(0.5, 100.0),
        cells={},
        bin_axis="disparity",
        calibration_envelope=envelope,
    )

    payload = profile.to_dict()
    loaded = OriginBBoxReprojectionProfile.from_dict(payload)

    assert payload["calibration_envelope"]["schema"] == "reprojection-profile-calibration-envelope-v1"
    assert payload["calibration_envelope"]["intrinsics_path"] == "intrinsics_zoom_08.json"
    assert payload["calibration_envelope"]["stereo_calibration"]["epipolar_mean_error_px"] == 0.08
    assert payload["calibration_envelope"]["warnings"] == []
    assert loaded.calibration_envelope == payload["calibration_envelope"]


def test_calibration_envelope_warns_when_intrinsics_missing() -> None:
    envelope = calibration_envelope_from_intrinsics(None)

    assert envelope["intrinsics_available"] is False
    assert envelope["warnings"] == ["no active intrinsics attached to reprojection profile"]


def test_profile_loads_and_serializes_range_only_output_policy(tmp_path) -> None:
    path = tmp_path / "range_only_reprojection.json"
    path.write_text(
        """
{
  "profile_id": "range_only",
  "image_width_px": 1280,
  "image_height_px": 720,
  "bin_axis": "disparity",
  "output_policy": "range_only_forward_fusion",
  "grid": {
    "u_edges": [0, 1280],
    "v_edges": [0, 720],
    "depth_edges": [0.5, 100]
  },
  "cells": {
    "0:0:0": {
      "du_left": 6,
      "dv_left": 10,
      "du_right": 2,
      "dv_right": -2,
      "sample_count": 5
    }
  }
}
""",
        encoding="utf-8",
    )

    profile = load_reprojection_profile(path)

    assert profile.output_policy == "range_only_forward_fusion"
    assert profile.to_dict()["output_policy"] == "range_only_forward_fusion"


def test_profile_noops_when_cell_missing_or_out_of_range() -> None:
    profile = OriginBBoxReprojectionProfile(
        profile_id="noop",
        image_width_px=1280.0,
        image_height_px=720.0,
        u_edges=(0.0, 640.0, 1280.0),
        v_edges=(0.0, 720.0),
        depth_edges=(0.5, 100.0),
        cells={},
        bin_axis="disparity",
    )

    missing = profile.apply(650.0, 360.0, 620.0, 360.0, raw_disparity_px=30.0)
    outside = profile.apply(1400.0, 360.0, 1370.0, 360.0, raw_disparity_px=30.0)

    assert missing.fallback_level == FALLBACK_NO_CELL
    assert missing.corrected_left_u == 650.0
    assert outside.fallback_level == FALLBACK_NO_CELL
    assert outside.corrected_right_u == 1370.0


def test_measurement_controller_retriangulates_corrected_bbox_center() -> None:
    profile = OriginBBoxReprojectionProfile(
        profile_id="controller_reprojection",
        image_width_px=1280.0,
        image_height_px=720.0,
        u_edges=(0.0, 1280.0),
        v_edges=(0.0, 720.0),
        depth_edges=(0.5, 100.0),
        cells={
            "0:0:0": {
                "du_left": 0.0,
                "dv_left": 0.0,
                "du_right": -30.0,
                "dv_right": 0.0,
                "sample_count": 10,
            }
        },
        bin_axis="disparity",
    )

    result = calculate_measurement_geometry(
        MeasurementGeometryInput(
            left=detection("left", cx=700.0),
            right=detection("right", cx=670.0),
            calibration=StereoCalibration(focal_length_px=900.0, baseline_m=0.3),
            intrinsics=intrinsics_result(),
            triangulation_source="pixhawk",
            left_extrinsic=pixhawk_attitude(),
            right_extrinsic=pixhawk_attitude(),
            translation_right_in_left_m=(0.3, 0.0, 0.0),
            reprojection_profile=profile,
        )
    )

    assert result.triangulated is not None
    assert result.reprojection is not None
    assert result.reprojection.lookup.corrected_right_u == 640.0
    assert result.reprojection.triangulated is not None
    assert result.reprojection.point_center_m is not None
    assert result.reprojection.triangulated.distance_m < result.triangulated.distance_m
