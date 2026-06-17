from __future__ import annotations

import math
import sys
import tempfile
import unittest
from pathlib import Path

import numpy as np

ROOT_DIR = Path(__file__).resolve().parents[1]
PACKAGE_DIR = ROOT_DIR / "stereo_ground_sim"
if str(PACKAGE_DIR) not in sys.path:
    sys.path.insert(0, str(PACKAGE_DIR))

from stereo_ground_sim.stereo_distance_node import StereoDistanceNode
from stereo_ground_sim.runtime_guard_policy import (
    BoundedGainForwardCellGuardPolicy,
    EstimatedToTruthAffineProfile,
    EstimatedToTruthDisparityProfile,
    EstimatedToTruthSim3Profile,
)


def make_policy_case():
    width = 1280.0
    height = 720.0
    horizontal_fov = 0.154997710698
    baseline = 25.0
    fx = (width * 0.5) / math.tan(horizontal_fov * 0.5)

    left = {"cx": 700.0, "cy": 367.0, "bbox": (650.0, 340.0, 750.0, 394.0)}
    right = {"cx": 528.0, "cy": 367.0, "bbox": (478.0, 340.0, 578.0, 394.0)}
    disparity = abs(left["cx"] - right["cx"])
    forward = fx * baseline / disparity
    raw = {
        "right_m": ((left["cx"] + right["cx"]) * 0.5 - width * 0.5) * forward / fx,
        "down_m": ((left["cy"] + right["cy"]) * 0.5 - height * 0.5) * forward / fx,
        "forward_m": forward,
        "range_m": forward,
        "disparity_px": disparity,
    }

    policy = BoundedGainForwardCellGuardPolicy.from_json_dict(
        {
            "mode": "bounded_gain_forward_cell_guard",
            "profile_id": "unit-test",
            "profile_family": "disparity_bin",
            "anchor_profile": {
                "mode": "disparity_bin",
                "global_anchor": {
                    "left_u": 0.5,
                    "left_v": 0.5,
                    "right_u": 0.5,
                    "right_v": 0.5,
                },
                "disparity_edges_px": [0.0, 100.0, 150.0, 170.0, 300.0],
                "cells": {
                    "3": {
                        "anchor": {
                            "left_u": 0.5,
                            "left_v": 0.8,
                            "right_u": 0.5,
                            "right_v": 0.8,
                        }
                    }
                },
            },
            "right_actual_to_nominal_rotation": [
                [1.0, 0.0, 0.0],
                [0.0, 1.0, 0.0],
                [0.0, 0.0, 1.0],
            ],
            "forward_policy": {
                "candidate": "anchor_gain_1",
                "min_range_m": 500.0,
                "raw_fallback_anchor_cells": ["disparity_bin_3"],
            },
            "down_policy": {
                "cells": {
                    "disparity_bin|3|3|1": {
                        "candidate": "anchor_gain_0.25",
                        "sample_count": 20,
                    }
                }
            },
        }
    )
    return policy, left, right, raw, {
        "width_px": width,
        "height_px": height,
        "horizontal_fov_rad": horizontal_fov,
        "baseline_m": baseline,
    }


class RuntimeGuardPolicyTests(unittest.TestCase):
    def test_stereo_distance_node_loads_estimated_to_truth_sim3_profile(self) -> None:
        with tempfile.TemporaryDirectory() as tmpdir:
            profile_path = Path(tmpdir) / "sim3_profile.json"
            profile_path.write_text(
                """{
  "mode": "estimated_to_truth_sim3",
  "profile_id": "node-loader-unit",
  "scale": 1.0,
  "rotation_matrix_rdf": [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]],
  "translation_rdf_m": [1.0, 2.0, 3.0]
}
""",
                encoding="utf-8",
            )
            node = StereoDistanceNode.__new__(StereoDistanceNode)
            node.enable_bbox_center_adjustment = True
            node.bbox_center_adjustment_path = str(profile_path)
            node.get_logger = lambda: type(
                "Logger",
                (),
                {
                    "warning": lambda self, msg: None,
                    "info": lambda self, msg: None,
                },
            )()

            loaded = StereoDistanceNode._load_bbox_center_adjustment(node)

        self.assertIsInstance(loaded, EstimatedToTruthSim3Profile)
        self.assertEqual(loaded.profile_id, "node-loader-unit")

    def test_stereo_distance_node_loads_estimated_to_truth_disparity_profile(self) -> None:
        with tempfile.TemporaryDirectory() as tmpdir:
            profile_path = Path(tmpdir) / "disparity_profile.json"
            profile_path.write_text(
                """{
  "mode": "estimated_to_truth_disparity_poly",
  "profile_id": "node-loader-disparity-unit",
  "correction_space": "raw_rdf_ray_scale",
  "d_corr_coefficients_high_to_low": [1.0, 0.0]
}
""",
                encoding="utf-8",
            )
            node = StereoDistanceNode.__new__(StereoDistanceNode)
            node.enable_bbox_center_adjustment = True
            node.bbox_center_adjustment_path = str(profile_path)
            node.get_logger = lambda: type(
                "Logger",
                (),
                {
                    "warning": lambda self, msg: None,
                    "info": lambda self, msg: None,
                },
            )()

            loaded = StereoDistanceNode._load_bbox_center_adjustment(node)

        self.assertIsInstance(loaded, EstimatedToTruthDisparityProfile)
        self.assertEqual(loaded.profile_id, "node-loader-disparity-unit")

    def test_stereo_distance_node_loads_estimated_to_truth_affine_profile(self) -> None:
        with tempfile.TemporaryDirectory() as tmpdir:
            profile_path = Path(tmpdir) / "affine_profile.json"
            profile_path.write_text(
                """{
  "mode": "estimated_to_truth_affine_rdf",
  "profile_id": "node-loader-affine-unit",
  "affine_matrix_rdf": [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]],
  "translation_rdf_m": [1.0, 2.0, 3.0]
}
""",
                encoding="utf-8",
            )
            node = StereoDistanceNode.__new__(StereoDistanceNode)
            node.enable_bbox_center_adjustment = True
            node.bbox_center_adjustment_path = str(profile_path)
            node.get_logger = lambda: type(
                "Logger",
                (),
                {
                    "warning": lambda self, msg: None,
                    "info": lambda self, msg: None,
                },
            )()

            loaded = StereoDistanceNode._load_bbox_center_adjustment(node)

        self.assertIsInstance(loaded, EstimatedToTruthAffineProfile)
        self.assertEqual(loaded.profile_id, "node-loader-affine-unit")

    def test_stereo_distance_node_applies_sim3_profile_to_metrics_rdf(self) -> None:
        node = StereoDistanceNode.__new__(StereoDistanceNode)
        node.horizontal_fov_rad = 0.154997710698
        node.baseline_m = 25.0
        node.yolo_load_error = ""
        node.yolo_class_name = "drone"
        node.bbox_center_adjustment = EstimatedToTruthSim3Profile.from_json_dict(
            {
                "mode": "estimated_to_truth_sim3",
                "profile_id": "node-apply-unit",
                "scale": 1.0,
                "rotation_matrix_rdf": [
                    [1.0, 0.0, 0.0],
                    [0.0, 1.0, 0.0],
                    [0.0, 0.0, 1.0],
                ],
                "translation_rdf_m": [1.0, 2.0, 3.0],
            }
        )
        left = {"cx": 700.0, "cy": 367.0, "bbox": (650.0, 340.0, 750.0, 394.0)}
        right = {"cx": 528.0, "cy": 367.0, "bbox": (478.0, 340.0, 578.0, 394.0)}
        image = np.zeros((720, 1280, 3), dtype=np.uint8)

        metrics = StereoDistanceNode._calculate_metrics(node, image, left, right, 1000.0, "unit")

        self.assertEqual(metrics["profile_status"]["profile_mode"], "estimated_to_truth_sim3")
        self.assertAlmostEqual(metrics["corrected_right_m"], metrics["estimated_right_m"] + 1.0)
        self.assertAlmostEqual(metrics["corrected_down_m"], metrics["estimated_down_m"] + 2.0)
        self.assertAlmostEqual(metrics["corrected_forward_m"], metrics["estimated_forward_m"] + 3.0)
        self.assertAlmostEqual(
            metrics["corrected_range_m"],
            math.sqrt(
                metrics["corrected_right_m"] ** 2
                + metrics["corrected_down_m"] ** 2
                + metrics["corrected_forward_m"] ** 2
            ),
        )

    def test_stereo_distance_node_passes_bbox_center_to_left_disparity_profile(self) -> None:
        node = StereoDistanceNode.__new__(StereoDistanceNode)
        node.horizontal_fov_rad = 0.154997710698
        node.baseline_m = 25.0
        node.yolo_load_error = ""
        node.yolo_class_name = "drone"
        width = 1280.0
        height = 720.0
        fx = (width * 0.5) / math.tan(node.horizontal_fov_rad * 0.5)
        node.bbox_center_adjustment = EstimatedToTruthSim3Profile.from_json_dict(
            {
                "mode": "estimated_to_truth_sim3",
                "profile_id": "node-left-disparity-unit",
                "scale": 1.0,
                "rotation_matrix_rdf": [
                    [1.0, 0.0, 0.0],
                    [0.0, 1.0, 0.0],
                    [0.0, 0.0, 1.0],
                ],
                "translation_rdf_m": [0.0, 0.0, 0.0],
                "left_disparity_triangulation_model": {
                    "fx_px": fx,
                    "fy_px": fx,
                    "cx_px": width * 0.5,
                    "cy_px": height * 0.5,
                    "baseline_m": node.baseline_m,
                    "disparity_correction_coefficients_high_to_low": [1.0, 0.0],
                },
            }
        )
        left = {"cx": 700.0, "cy": 367.0, "bbox": (650.0, 340.0, 750.0, 394.0)}
        right = {"cx": 528.0, "cy": 367.0, "bbox": (478.0, 340.0, 578.0, 394.0)}
        image = np.zeros((int(height), int(width), 3), dtype=np.uint8)

        metrics = StereoDistanceNode._calculate_metrics(node, image, left, right, 1000.0, "unit")

        disparity = abs(left["cx"] - right["cx"])
        forward = fx * node.baseline_m / disparity
        expected_right = (left["cx"] - width * 0.5) * forward / fx - node.baseline_m * 0.5
        expected_down = (left["cy"] - height * 0.5) * forward / fx
        self.assertEqual(metrics["profile_status"]["profile_mode"], "estimated_to_truth_sim3")
        self.assertTrue(metrics["profile_status"]["left_disparity_triangulation_model_applied"])
        self.assertAlmostEqual(metrics["corrected_right_m"], expected_right)
        self.assertAlmostEqual(metrics["corrected_down_m"], expected_down)
        self.assertAlmostEqual(metrics["corrected_forward_m"], forward)

    def test_stereo_distance_node_applies_affine_profile_to_metrics_rdf(self) -> None:
        node = StereoDistanceNode.__new__(StereoDistanceNode)
        node.horizontal_fov_rad = 0.154997710698
        node.baseline_m = 25.0
        node.yolo_load_error = ""
        node.yolo_class_name = "drone"
        node.bbox_center_adjustment = EstimatedToTruthAffineProfile.from_json_dict(
            {
                "mode": "estimated_to_truth_affine_rdf",
                "profile_id": "node-affine-apply-unit",
                "affine_matrix_rdf": [
                    [1.0, 0.0, 0.0],
                    [0.0, 1.0, 0.0],
                    [0.0, 0.0, 1.0],
                ],
                "translation_rdf_m": [1.0, 2.0, 3.0],
            }
        )
        left = {"cx": 700.0, "cy": 367.0, "bbox": (650.0, 340.0, 750.0, 394.0)}
        right = {"cx": 528.0, "cy": 367.0, "bbox": (478.0, 340.0, 578.0, 394.0)}
        image = np.zeros((720, 1280, 3), dtype=np.uint8)

        metrics = StereoDistanceNode._calculate_metrics(node, image, left, right, 1000.0, "unit")

        self.assertEqual(metrics["profile_status"]["profile_mode"], "estimated_to_truth_affine_rdf")
        self.assertAlmostEqual(metrics["corrected_right_m"], metrics["estimated_right_m"] + 1.0)
        self.assertAlmostEqual(metrics["corrected_down_m"], metrics["estimated_down_m"] + 2.0)
        self.assertAlmostEqual(metrics["corrected_forward_m"], metrics["estimated_forward_m"] + 3.0)

    def test_estimated_to_truth_sim3_profile_applies_rdf_transform(self) -> None:
        profile = EstimatedToTruthSim3Profile.from_json_dict(
            {
                "mode": "estimated_to_truth_sim3",
                "profile_id": "unit-sim3",
                "scale": 2.0,
                "rotation_matrix_rdf": [
                    [0.0, -1.0, 0.0],
                    [1.0, 0.0, 0.0],
                    [0.0, 0.0, 1.0],
                ],
                "translation_rdf_m": [1.0, 2.0, 3.0],
            }
        )

        result = profile.apply(
            {
                "right_m": 4.0,
                "down_m": 5.0,
                "forward_m": 6.0,
                "range_m": 9.0,
                "disparity_px": 10.0,
            }
        )

        self.assertIsNotNone(result.corrected)
        assert result.corrected is not None
        self.assertEqual(result.status["profile_mode"], "estimated_to_truth_sim3")
        self.assertAlmostEqual(result.corrected["right_m"], -9.0)
        self.assertAlmostEqual(result.corrected["down_m"], 10.0)
        self.assertAlmostEqual(result.corrected["forward_m"], 15.0)
        self.assertAlmostEqual(
            result.corrected["range_m"],
            math.sqrt((-9.0) ** 2 + 10.0 ** 2 + 15.0 ** 2),
        )

    def test_estimated_to_truth_sim3_profile_applies_target_disparity_blend_guard(self) -> None:
        profile = EstimatedToTruthSim3Profile.from_json_dict(
            {
                "mode": "estimated_to_truth_sim3",
                "profile_id": "unit-target-disparity-blend",
                "scale": 1.0,
                "rotation_matrix_rdf": [
                    [1.0, 0.0, 0.0],
                    [0.0, 1.0, 0.0],
                    [0.0, 0.0, 1.0],
                ],
                "translation_rdf_m": [4.0, -2.0, 8.0],
                "target_disparity_blend_guard": {
                    "target_disparity_px": 5.0,
                    "half_width_px": 0.2,
                    "alpha_inside": 0.75,
                    "alpha_outside": 1.0,
                },
            }
        )

        result = profile.apply(
            {
                "right_m": 10.0,
                "down_m": 20.0,
                "forward_m": 1000.0,
                "range_m": 1000.25,
                "disparity_px": 5.1,
            }
        )

        self.assertIsNotNone(result.corrected)
        assert result.corrected is not None
        self.assertTrue(result.status["target_disparity_blend_guard_applied"])
        self.assertAlmostEqual(result.status["target_disparity_blend_alpha"], 0.75)
        self.assertAlmostEqual(result.corrected["right_m"], 13.0)
        self.assertAlmostEqual(result.corrected["down_m"], 18.5)
        self.assertAlmostEqual(result.corrected["forward_m"], 1006.0)

        outside = profile.apply(
            {
                "right_m": 10.0,
                "down_m": 20.0,
                "forward_m": 1000.0,
                "range_m": 1000.25,
                "disparity_px": 5.5,
            }
        )

        self.assertIsNotNone(outside.corrected)
        assert outside.corrected is not None
        self.assertFalse(outside.status["target_disparity_blend_guard_applied"])
        self.assertAlmostEqual(outside.status["target_disparity_blend_alpha"], 1.0)
        self.assertAlmostEqual(outside.corrected["right_m"], 14.0)
        self.assertAlmostEqual(outside.corrected["down_m"], 18.0)
        self.assertAlmostEqual(outside.corrected["forward_m"], 1008.0)

    def test_estimated_to_truth_sim3_profile_applies_target_range_away_blend_guard(self) -> None:
        profile = EstimatedToTruthSim3Profile.from_json_dict(
            {
                "mode": "estimated_to_truth_sim3",
                "profile_id": "unit-target-range-away",
                "scale": 1.0,
                "rotation_matrix_rdf": [
                    [1.0, 0.0, 0.0],
                    [0.0, 1.0, 0.0],
                    [0.0, 0.0, 1.0],
                ],
                "translation_rdf_m": [0.0, 0.0, 10.0],
                "target_range_away_blend_guard": {
                    "target_range_m": 1000.0,
                    "window_m": 40.0,
                    "alpha_away": 0.5,
                },
            }
        )

        away = profile.apply(
            {
                "right_m": 0.0,
                "down_m": 0.0,
                "forward_m": 1010.0,
                "range_m": 1010.0,
                "disparity_px": 10.0,
            }
        )

        self.assertIsNotNone(away.corrected)
        assert away.corrected is not None
        self.assertTrue(away.status["target_range_away_blend_guard_applied"])
        self.assertAlmostEqual(away.status["target_range_away_blend_alpha"], 0.5)
        self.assertAlmostEqual(away.corrected["forward_m"], 1015.0)

        toward = profile.apply(
            {
                "right_m": 0.0,
                "down_m": 0.0,
                "forward_m": 990.0,
                "range_m": 990.0,
                "disparity_px": 10.0,
            }
        )

        self.assertIsNotNone(toward.corrected)
        assert toward.corrected is not None
        self.assertFalse(toward.status["target_range_away_blend_guard_applied"])
        self.assertAlmostEqual(toward.corrected["forward_m"], 1000.0)

    def test_estimated_to_truth_sim3_profile_applies_target_disparity_range_delta_cap_guard(self) -> None:
        profile = EstimatedToTruthSim3Profile.from_json_dict(
            {
                "mode": "estimated_to_truth_sim3",
                "profile_id": "unit-target-disparity-cap",
                "scale": 1.0,
                "rotation_matrix_rdf": [
                    [1.0, 0.0, 0.0],
                    [0.0, 1.0, 0.0],
                    [0.0, 0.0, 1.0],
                ],
                "translation_rdf_m": [0.0, 0.0, 10.0],
                "target_disparity_range_delta_cap_guard": {
                    "target_disparity_px": 5.0,
                    "half_width_px": 0.2,
                    "range_delta_cap_m": 4.0,
                },
            }
        )

        capped = profile.apply(
            {
                "right_m": 0.0,
                "down_m": 0.0,
                "forward_m": 1000.0,
                "range_m": 1000.0,
                "disparity_px": 5.1,
            }
        )

        self.assertIsNotNone(capped.corrected)
        assert capped.corrected is not None
        self.assertTrue(capped.status["target_disparity_range_delta_cap_guard_applied"])
        self.assertAlmostEqual(capped.status["target_disparity_range_delta_cap_m"], 4.0)
        self.assertAlmostEqual(capped.corrected["forward_m"], 1004.0)

        outside = profile.apply(
            {
                "right_m": 0.0,
                "down_m": 0.0,
                "forward_m": 1000.0,
                "range_m": 1000.0,
                "disparity_px": 5.5,
            }
        )

        self.assertIsNotNone(outside.corrected)
        assert outside.corrected is not None
        self.assertFalse(outside.status["target_disparity_range_delta_cap_guard_applied"])
        self.assertAlmostEqual(outside.corrected["forward_m"], 1010.0)

    def test_estimated_to_truth_sim3_profile_applies_ray_disparity_correction_before_transform(self) -> None:
        profile = EstimatedToTruthSim3Profile.from_json_dict(
            {
                "mode": "estimated_to_truth_sim3",
                "profile_id": "unit-sim3-ray-disparity",
                "scale": 2.0,
                "rotation_matrix_rdf": [
                    [1.0, 0.0, 0.0],
                    [0.0, 1.0, 0.0],
                    [0.0, 0.0, 1.0],
                ],
                "translation_rdf_m": [1.0, 2.0, 3.0],
                "raw_rdf_ray_disparity_correction_coefficients_high_to_low": [2.0, 0.0],
            }
        )

        result = profile.apply(
            {
                "right_m": 4.0,
                "down_m": 6.0,
                "forward_m": 100.0,
                "range_m": 100.3,
                "disparity_px": 10.0,
            }
        )

        self.assertIsNotNone(result.corrected)
        assert result.corrected is not None
        # d_corr=2*d_meas, so the raw RDF ray is scaled by 0.5 before Sim(3).
        self.assertAlmostEqual(result.corrected["right_m"], 5.0)
        self.assertAlmostEqual(result.corrected["down_m"], 8.0)
        self.assertAlmostEqual(result.corrected["forward_m"], 103.0)

    def test_estimated_to_truth_sim3_profile_triangulates_left_disparity_before_transform(self) -> None:
        profile = EstimatedToTruthSim3Profile.from_json_dict(
            {
                "mode": "estimated_to_truth_sim3",
                "profile_id": "unit-sim3-left-disparity",
                "scale": 1.0,
                "rotation_matrix_rdf": [
                    [1.0, 0.0, 0.0],
                    [0.0, 1.0, 0.0],
                    [0.0, 0.0, 1.0],
                ],
                "translation_rdf_m": [1.0, 2.0, 3.0],
                "left_disparity_triangulation_model": {
                    "fx_px": 100.0,
                    "fy_px": 100.0,
                    "cx_px": 50.0,
                    "cy_px": 40.0,
                    "baseline_m": 10.0,
                    "disparity_correction_coefficients_high_to_low": [1.0, -2.0],
                },
            }
        )

        result = profile.apply(
            {
                "right_m": 999.0,
                "down_m": 999.0,
                "forward_m": 999.0,
                "range_m": 999.0,
                "disparity_px": 12.0,
                "left_u_px": 65.0,
                "left_v_px": 46.0,
            }
        )

        self.assertIsNotNone(result.corrected)
        assert result.corrected is not None
        # d_corr=10, forward=100, right=(65-50)*100/100 - 5, down=(46-40)*100/100.
        self.assertTrue(result.status["left_disparity_triangulation_model_applied"])
        self.assertAlmostEqual(result.corrected["right_m"], 11.0)
        self.assertAlmostEqual(result.corrected["down_m"], 8.0)
        self.assertAlmostEqual(result.corrected["forward_m"], 103.0)
        self.assertAlmostEqual(result.corrected["disparity_px"], 10.0)

    def test_estimated_to_truth_sim3_profile_applies_disparity_product_ray_scale_before_transform(self) -> None:
        profile = EstimatedToTruthSim3Profile.from_json_dict(
            {
                "mode": "estimated_to_truth_sim3",
                "profile_id": "unit-sim3-product-ray",
                "scale": 1.0,
                "rotation_matrix_rdf": [
                    [1.0, 0.0, 0.0],
                    [0.0, 1.0, 0.0],
                    [0.0, 0.0, 1.0],
                ],
                "translation_rdf_m": [1.0, 2.0, 3.0],
                "raw_rdf_ray_disparity_forward_product_px_m": 100.0,
            }
        )

        result = profile.apply(
            {
                "right_m": 2.0,
                "down_m": 4.0,
                "forward_m": 8.0,
                "range_m": 9.0,
                "disparity_px": 10.0,
            }
        )

        self.assertIsNotNone(result.corrected)
        assert result.corrected is not None
        # product/disparity sets forward to 10 m, so the raw RDF ray is scaled by 1.25 before Sim(3).
        self.assertAlmostEqual(result.corrected["right_m"], 3.5)
        self.assertAlmostEqual(result.corrected["down_m"], 7.0)
        self.assertAlmostEqual(result.corrected["forward_m"], 13.0)

    def test_estimated_to_truth_sim3_profile_applies_inverse_depth_ray_correction_before_transform(self) -> None:
        profile = EstimatedToTruthSim3Profile.from_json_dict(
            {
                "mode": "estimated_to_truth_sim3",
                "profile_id": "unit-sim3-ray-inverse-depth",
                "scale": 1.0,
                "rotation_matrix_rdf": [
                    [1.0, 0.0, 0.0],
                    [0.0, 1.0, 0.0],
                    [0.0, 0.0, 1.0],
                ],
                "translation_rdf_m": [1.0, 2.0, 3.0],
                "raw_rdf_ray_inverse_depth_correction_coefficients_high_to_low": [0.5, 0.0],
            }
        )

        result = profile.apply(
            {
                "right_m": 4.0,
                "down_m": 6.0,
                "forward_m": 100.0,
                "range_m": 100.3,
                "disparity_px": 10.0,
            }
        )

        self.assertIsNotNone(result.corrected)
        assert result.corrected is not None
        self.assertAlmostEqual(result.corrected["right_m"], 9.0)
        self.assertAlmostEqual(result.corrected["down_m"], 14.0)
        self.assertAlmostEqual(result.corrected["forward_m"], 203.0)

    def test_estimated_to_truth_sim3_profile_applies_piecewise_raw_forward_transform(self) -> None:
        profile = EstimatedToTruthSim3Profile.from_json_dict(
            {
                "mode": "estimated_to_truth_sim3",
                "profile_id": "unit-sim3-piecewise",
                "scale": 1.0,
                "rotation_matrix_rdf": [
                    [1.0, 0.0, 0.0],
                    [0.0, 1.0, 0.0],
                    [0.0, 0.0, 1.0],
                ],
                "translation_rdf_m": [1000.0, 1000.0, 1000.0],
                "piecewise_sim3_raw_forward": {
                    "x_values_m": [100.0, 200.0],
                    "transforms": [
                        {
                            "scale": 1.0,
                            "rotation_matrix_rdf": [
                                [1.0, 0.0, 0.0],
                                [0.0, 1.0, 0.0],
                                [0.0, 0.0, 1.0],
                            ],
                            "translation_rdf_m": [10.0, 0.0, 0.0],
                        },
                        {
                            "scale": 2.0,
                            "rotation_matrix_rdf": [
                                [1.0, 0.0, 0.0],
                                [0.0, 1.0, 0.0],
                                [0.0, 0.0, 1.0],
                            ],
                            "translation_rdf_m": [0.0, 20.0, 0.0],
                        },
                    ],
                },
            }
        )

        result = profile.apply(
            {
                "right_m": 4.0,
                "down_m": 5.0,
                "forward_m": 150.0,
                "range_m": 150.2,
                "disparity_px": 10.0,
            }
        )

        self.assertIsNotNone(result.corrected)
        assert result.corrected is not None
        self.assertTrue(result.status["piecewise_sim3_raw_forward_active"])
        # x=150m is halfway between the two transforms:
        # 0.5 * ([4,5,150] + [10,0,0]) + 0.5 * (2*[4,5,150] + [0,20,0]).
        self.assertAlmostEqual(result.corrected["right_m"], 11.0)
        self.assertAlmostEqual(result.corrected["down_m"], 17.5)
        self.assertAlmostEqual(result.corrected["forward_m"], 225.0)

    def test_estimated_to_truth_sim3_profile_blends_global_and_piecewise_transform(self) -> None:
        profile = EstimatedToTruthSim3Profile.from_json_dict(
            {
                "mode": "estimated_to_truth_sim3",
                "profile_id": "unit-sim3-blended-piecewise",
                "scale": 1.0,
                "rotation_matrix_rdf": [
                    [1.0, 0.0, 0.0],
                    [0.0, 1.0, 0.0],
                    [0.0, 0.0, 1.0],
                ],
                "translation_rdf_m": [100.0, 0.0, 0.0],
                "piecewise_sim3_blend_alpha": 0.25,
                "piecewise_sim3_raw_forward": {
                    "x_values_m": [100.0],
                    "transforms": [
                        {
                            "scale": 1.0,
                            "rotation_matrix_rdf": [
                                [1.0, 0.0, 0.0],
                                [0.0, 1.0, 0.0],
                                [0.0, 0.0, 1.0],
                            ],
                            "translation_rdf_m": [0.0, 40.0, 0.0],
                        }
                    ],
                },
            }
        )

        result = profile.apply(
            {
                "right_m": 4.0,
                "down_m": 5.0,
                "forward_m": 150.0,
                "range_m": 150.2,
                "disparity_px": 10.0,
            }
        )

        self.assertIsNotNone(result.corrected)
        assert result.corrected is not None
        self.assertTrue(result.status["piecewise_sim3_raw_forward_active"])
        self.assertAlmostEqual(result.status["piecewise_sim3_blend_alpha"], 0.25)
        self.assertAlmostEqual(result.corrected["right_m"], 79.0)
        self.assertAlmostEqual(result.corrected["down_m"], 15.0)
        self.assertAlmostEqual(result.corrected["forward_m"], 150.0)

    def test_estimated_to_truth_affine_profile_applies_rdf_transform(self) -> None:
        profile = EstimatedToTruthAffineProfile.from_json_dict(
            {
                "mode": "estimated_to_truth_affine_rdf",
                "profile_id": "unit-affine",
                "affine_matrix_rdf": [
                    [1.0, 0.1, 0.0],
                    [0.0, 2.0, 0.0],
                    [0.0, 0.0, 0.5],
                ],
                "translation_rdf_m": [1.0, 2.0, 3.0],
            }
        )

        result = profile.apply(
            {
                "right_m": 4.0,
                "down_m": 5.0,
                "forward_m": 6.0,
                "range_m": 9.0,
                "disparity_px": 10.0,
            }
        )

        self.assertIsNotNone(result.corrected)
        assert result.corrected is not None
        self.assertEqual(result.status["profile_mode"], "estimated_to_truth_affine_rdf")
        self.assertAlmostEqual(result.corrected["right_m"], 5.5)
        self.assertAlmostEqual(result.corrected["down_m"], 12.0)
        self.assertAlmostEqual(result.corrected["forward_m"], 6.0)

    def test_estimated_to_truth_affine_profile_applies_residual_luts(self) -> None:
        profile = EstimatedToTruthAffineProfile.from_json_dict(
            {
                "mode": "estimated_to_truth_affine_rdf",
                "profile_id": "unit-affine-lut",
                "affine_matrix_rdf": [
                    [1.0, 0.0, 0.0],
                    [0.0, 1.0, 0.0],
                    [0.0, 0.0, 1.0],
                ],
                "translation_rdf_m": [0.0, 0.0, 0.0],
                "forward_residual_disparity_lut": {
                    "x_values_px": [10.0, 20.0],
                    "residual_values_m": [2.0, 4.0],
                },
                "rdf_residual_disparity_lut": {
                    "right": {"x_values_px": [10.0, 20.0], "residual_values_m": [1.0, 3.0]},
                    "down": {"x_values_px": [10.0, 20.0], "residual_values_m": [-1.0, -3.0]},
                    "forward": {"x_values_px": [10.0, 20.0], "residual_values_m": [0.5, 1.5]},
                },
            }
        )

        result = profile.apply(
            {
                "right_m": 4.0,
                "down_m": 5.0,
                "forward_m": 6.0,
                "range_m": 9.0,
                "disparity_px": 15.0,
            }
        )

        self.assertIsNotNone(result.corrected)
        assert result.corrected is not None
        self.assertEqual(result.status["profile_mode"], "estimated_to_truth_affine_rdf")
        self.assertAlmostEqual(result.corrected["right_m"], 6.0)
        self.assertAlmostEqual(result.corrected["down_m"], 3.0)
        self.assertAlmostEqual(result.corrected["forward_m"], 10.0)

    def test_estimated_to_truth_disparity_profile_scales_raw_rdf_ray(self) -> None:
        profile = EstimatedToTruthDisparityProfile.from_json_dict(
            {
                "mode": "estimated_to_truth_disparity_poly",
                "profile_id": "unit-disparity",
                "d_corr_coefficients_high_to_low": [2.0, 0.0],
            }
        )

        result = profile.apply(
            {
                "right_m": 4.0,
                "down_m": 5.0,
                "forward_m": 100.0,
                "range_m": 100.2,
                "disparity_px": 10.0,
            }
        )

        self.assertIsNotNone(result.corrected)
        assert result.corrected is not None
        self.assertEqual(result.status["profile_mode"], "estimated_to_truth_disparity_poly")
        self.assertAlmostEqual(result.corrected["right_m"], 2.0)
        self.assertAlmostEqual(result.corrected["down_m"], 2.5)
        self.assertAlmostEqual(result.corrected["forward_m"], 50.0)
        self.assertAlmostEqual(result.corrected["disparity_px"], 20.0)

    def test_estimated_to_truth_disparity_profile_scales_raw_rdf_ray_by_inverse_depth(self) -> None:
        profile = EstimatedToTruthDisparityProfile.from_json_dict(
            {
                "mode": "estimated_to_truth_disparity_poly",
                "profile_id": "unit-inverse-depth-ray",
                "correction_space": "raw_rdf_inverse_depth_ray_scale",
                "inverse_depth_coefficients_high_to_low": [0.5, 0.0],
            }
        )

        result = profile.apply(
            {
                "right_m": 4.0,
                "down_m": 5.0,
                "forward_m": 100.0,
                "range_m": 100.2,
                "disparity_px": 10.0,
            }
        )

        self.assertIsNotNone(result.corrected)
        assert result.corrected is not None
        # inv_corr = 0.5 * (1 / 100), so corrected forward doubles.
        self.assertEqual(result.status["profile_mode"], "estimated_to_truth_disparity_poly")
        self.assertAlmostEqual(result.corrected["right_m"], 8.0)
        self.assertAlmostEqual(result.corrected["down_m"], 10.0)
        self.assertAlmostEqual(result.corrected["forward_m"], 200.0)

    def test_estimated_to_truth_sim3_profile_applies_forward_disparity_residual(self) -> None:
        profile = EstimatedToTruthSim3Profile.from_json_dict(
            {
                "mode": "estimated_to_truth_sim3",
                "profile_id": "unit-sim3-residual",
                "scale": 1.0,
                "rotation_matrix_rdf": [
                    [1.0, 0.0, 0.0],
                    [0.0, 1.0, 0.0],
                    [0.0, 0.0, 1.0],
                ],
                "translation_rdf_m": [0.5, -0.25, 2.0],
                "forward_residual_disparity_coefficients_high_to_low": [0.1, -1.0, 4.0],
            }
        )

        result = profile.apply(
            {
                "right_m": 4.0,
                "down_m": 5.0,
                "forward_m": 100.0,
                "range_m": 100.2,
                "disparity_px": 10.0,
            }
        )

        self.assertIsNotNone(result.corrected)
        assert result.corrected is not None
        self.assertEqual(result.status["profile_mode"], "estimated_to_truth_sim3")
        # Residual is polyval([0.1, -1.0, 4.0], disparity_px=10) = 4m,
        # applied only to the RDF Forward axis after the Sim(3) transform.
        self.assertAlmostEqual(result.corrected["right_m"], 4.5)
        self.assertAlmostEqual(result.corrected["down_m"], 4.75)
        self.assertAlmostEqual(result.corrected["forward_m"], 106.0)
        self.assertAlmostEqual(
            result.corrected["range_m"],
            math.sqrt(4.5**2 + 4.75**2 + 106.0**2),
        )

    def test_estimated_to_truth_sim3_profile_applies_rdf_disparity_residual(self) -> None:
        profile = EstimatedToTruthSim3Profile.from_json_dict(
            {
                "mode": "estimated_to_truth_sim3",
                "profile_id": "unit-sim3-rdf-residual",
                "scale": 1.0,
                "rotation_matrix_rdf": [
                    [1.0, 0.0, 0.0],
                    [0.0, 1.0, 0.0],
                    [0.0, 0.0, 1.0],
                ],
                "translation_rdf_m": [0.0, 0.0, 0.0],
                "rdf_residual_disparity_coefficients_high_to_low": {
                    "right": [0.1, 0.0],
                    "down": [-0.2],
                    "forward": [0.0, 1.5],
                },
            }
        )

        result = profile.apply(
            {
                "right_m": 4.0,
                "down_m": 5.0,
                "forward_m": 100.0,
                "range_m": 100.2,
                "disparity_px": 10.0,
            }
        )

        self.assertIsNotNone(result.corrected)
        assert result.corrected is not None
        self.assertAlmostEqual(result.corrected["right_m"], 5.0)
        self.assertAlmostEqual(result.corrected["down_m"], 4.8)
        self.assertAlmostEqual(result.corrected["forward_m"], 101.5)
        self.assertAlmostEqual(
            result.corrected["range_m"],
            math.sqrt(5.0**2 + 4.8**2 + 101.5**2),
        )

    def test_estimated_to_truth_sim3_profile_clamps_rdf_forward_residual(self) -> None:
        profile = EstimatedToTruthSim3Profile.from_json_dict(
            {
                "mode": "estimated_to_truth_sim3",
                "profile_id": "unit-sim3-rdf-forward-residual-clamp",
                "scale": 1.0,
                "rotation_matrix_rdf": [
                    [1.0, 0.0, 0.0],
                    [0.0, 1.0, 0.0],
                    [0.0, 0.0, 1.0],
                ],
                "translation_rdf_m": [0.0, 0.0, 0.0],
                "rdf_residual_disparity_coefficients_high_to_low": {
                    "right": [0.0],
                    "down": [0.0],
                    "forward": [4.0],
                },
                "forward_residual_clamp_max_abs_m": 1.5,
            }
        )

        result = profile.apply(
            {
                "right_m": 4.0,
                "down_m": 5.0,
                "forward_m": 100.0,
                "range_m": 100.2,
                "disparity_px": 10.0,
            }
        )

        self.assertIsNotNone(result.corrected)
        assert result.corrected is not None
        self.assertAlmostEqual(result.corrected["right_m"], 4.0)
        self.assertAlmostEqual(result.corrected["down_m"], 5.0)
        self.assertAlmostEqual(result.corrected["forward_m"], 101.5)
        self.assertEqual(result.status["rdf_forward_residual_unclamped_m"], 4.0)
        self.assertEqual(result.status["rdf_forward_residual_applied_m"], 1.5)

    def test_estimated_to_truth_sim3_profile_applies_raw_rdf_residual_polynomial(self) -> None:
        profile = EstimatedToTruthSim3Profile.from_json_dict(
            {
                "mode": "estimated_to_truth_sim3",
                "profile_id": "unit-sim3-raw-rdf-poly",
                "scale": 1.0,
                "rotation_matrix_rdf": [
                    [1.0, 0.0, 0.0],
                    [0.0, 1.0, 0.0],
                    [0.0, 0.0, 1.0],
                ],
                "translation_rdf_m": [0.0, 0.0, 0.0],
                "rdf_residual_raw_rdf_polynomial": {
                    "degree": 1,
                    "feature_names": [
                        "1",
                        "raw_right_over_raw_forward",
                        "raw_down_over_raw_forward",
                        "raw_forward_normalized",
                    ],
                    "forward_center_m": 100.0,
                    "forward_scale_m": 20.0,
                    "coefficient_matrix_m": [
                        [1.0, -2.0, 3.0],
                        [10.0, 0.0, 0.0],
                        [0.0, 20.0, 0.0],
                        [0.0, 0.0, 2.0],
                    ],
                    "residual_norm_cap_m": None,
                },
            }
        )

        result = profile.apply(
            {
                "right_m": 10.0,
                "down_m": 5.0,
                "forward_m": 120.0,
                "range_m": 120.5,
                "disparity_px": 10.0,
            }
        )

        self.assertIsNotNone(result.corrected)
        assert result.corrected is not None
        # Features are [1, right/forward=1/12, down/forward=1/24, forward_norm=1].
        self.assertAlmostEqual(result.corrected["right_m"], 10.0 + 1.0 + 10.0 / 12.0)
        self.assertAlmostEqual(result.corrected["down_m"], 5.0 - 2.0 + 20.0 / 24.0)
        self.assertAlmostEqual(result.corrected["forward_m"], 120.0 + 3.0 + 2.0)

    def test_estimated_to_truth_sim3_profile_applies_forward_residual_lut(self) -> None:
        profile = EstimatedToTruthSim3Profile.from_json_dict(
            {
                "mode": "estimated_to_truth_sim3",
                "profile_id": "unit-sim3-lut-residual",
                "scale": 1.0,
                "rotation_matrix_rdf": [
                    [1.0, 0.0, 0.0],
                    [0.0, 1.0, 0.0],
                    [0.0, 0.0, 1.0],
                ],
                "translation_rdf_m": [0.0, 0.0, 0.0],
                "forward_residual_disparity_lut": {
                    "x_values_px": [10.0, 20.0],
                    "residual_values_m": [3.0, 5.0],
                },
            }
        )

        result = profile.apply(
            {
                "right_m": 4.0,
                "down_m": 5.0,
                "forward_m": 100.0,
                "range_m": 100.2,
                "disparity_px": 15.0,
            }
        )

        self.assertIsNotNone(result.corrected)
        assert result.corrected is not None
        self.assertAlmostEqual(result.corrected["right_m"], 4.0)
        self.assertAlmostEqual(result.corrected["down_m"], 5.0)
        self.assertAlmostEqual(result.corrected["forward_m"], 104.0)
        self.assertAlmostEqual(
            result.corrected["range_m"],
            math.sqrt(4.0**2 + 5.0**2 + 104.0**2),
        )

    def test_estimated_to_truth_sim3_profile_applies_forward_raw_forward_lut(self) -> None:
        profile = EstimatedToTruthSim3Profile.from_json_dict(
            {
                "mode": "estimated_to_truth_sim3",
                "profile_id": "unit-sim3-raw-forward-lut-residual",
                "scale": 1.0,
                "rotation_matrix_rdf": [
                    [1.0, 0.0, 0.0],
                    [0.0, 1.0, 0.0],
                    [0.0, 0.0, 1.0],
                ],
                "translation_rdf_m": [0.0, 0.0, 0.0],
                "forward_residual_raw_forward_lut": {
                    "x_values_m": [80.0, 120.0],
                    "residual_values_m": [2.0, 6.0],
                },
            }
        )

        result = profile.apply(
            {
                "right_m": 4.0,
                "down_m": 5.0,
                "forward_m": 100.0,
                "range_m": 100.2,
                "disparity_px": 999.0,
            }
        )

        self.assertIsNotNone(result.corrected)
        assert result.corrected is not None
        self.assertAlmostEqual(result.corrected["right_m"], 4.0)
        self.assertAlmostEqual(result.corrected["down_m"], 5.0)
        self.assertAlmostEqual(result.corrected["forward_m"], 104.0)
        self.assertAlmostEqual(
            result.corrected["range_m"],
            math.sqrt(4.0**2 + 5.0**2 + 104.0**2),
        )

    def test_estimated_to_truth_sim3_profile_suppresses_disagreeing_forward_luts(self) -> None:
        profile = EstimatedToTruthSim3Profile.from_json_dict(
            {
                "mode": "estimated_to_truth_sim3",
                "profile_id": "unit-sim3-agreement-gated-lut-residual",
                "scale": 1.0,
                "rotation_matrix_rdf": [
                    [1.0, 0.0, 0.0],
                    [0.0, 1.0, 0.0],
                    [0.0, 0.0, 1.0],
                ],
                "translation_rdf_m": [0.0, 0.0, 0.0],
                "forward_residual_disparity_lut": {
                    "x_values_px": [10.0, 20.0],
                    "residual_values_m": [4.0, 4.0],
                },
                "forward_residual_raw_forward_lut": {
                    "x_values_m": [80.0, 120.0],
                    "residual_values_m": [1.0, 1.0],
                },
                "forward_residual_agreement_gate": {
                    "max_abs_difference_m": 1.0,
                    "alpha_disagree": 0.0,
                },
            }
        )

        result = profile.apply(
            {
                "right_m": 4.0,
                "down_m": 5.0,
                "forward_m": 100.0,
                "range_m": 100.2,
                "disparity_px": 15.0,
            }
        )

        self.assertIsNotNone(result.corrected)
        assert result.corrected is not None
        self.assertAlmostEqual(result.corrected["forward_m"], 100.0)
        self.assertIn("forward_residual_agreement_gate", result.status)

    def test_estimated_to_truth_sim3_profile_applies_rdf_residual_lut(self) -> None:
        profile = EstimatedToTruthSim3Profile.from_json_dict(
            {
                "mode": "estimated_to_truth_sim3",
                "profile_id": "unit-sim3-rdf-lut-residual",
                "scale": 1.0,
                "rotation_matrix_rdf": [
                    [1.0, 0.0, 0.0],
                    [0.0, 1.0, 0.0],
                    [0.0, 0.0, 1.0],
                ],
                "translation_rdf_m": [0.0, 0.0, 0.0],
                "rdf_residual_disparity_lut": {
                    "right": {"x_values_px": [10.0, 20.0], "residual_values_m": [1.0, 2.0]},
                    "down": {"x_values_px": [10.0, 20.0], "residual_values_m": [-1.0, -2.0]},
                    "forward": {"x_values_px": [10.0, 20.0], "residual_values_m": [3.0, 5.0]},
                },
            }
        )

        result = profile.apply(
            {
                "right_m": 4.0,
                "down_m": 5.0,
                "forward_m": 100.0,
                "range_m": 100.2,
                "disparity_px": 15.0,
            }
        )

        self.assertIsNotNone(result.corrected)
        assert result.corrected is not None
        self.assertAlmostEqual(result.corrected["right_m"], 5.5)
        self.assertAlmostEqual(result.corrected["down_m"], 3.5)
        self.assertAlmostEqual(result.corrected["forward_m"], 104.0)
        self.assertAlmostEqual(
            result.corrected["range_m"],
            math.sqrt(5.5**2 + 3.5**2 + 104.0**2),
        )

    def test_estimated_to_truth_sim3_profile_can_suppress_forward_residual_by_disparity(self) -> None:
        profile = EstimatedToTruthSim3Profile.from_json_dict(
            {
                "mode": "estimated_to_truth_sim3",
                "profile_id": "unit-sim3-residual-band",
                "scale": 1.0,
                "rotation_matrix_rdf": [
                    [1.0, 0.0, 0.0],
                    [0.0, 1.0, 0.0],
                    [0.0, 0.0, 1.0],
                ],
                "translation_rdf_m": [0.0, 0.0, 0.0],
                "forward_residual_disparity_coefficients_high_to_low": [4.0],
                "forward_residual_disparity_suppression_band": {
                    "center_disparity_px": 114.0,
                    "half_width_px": 8.0,
                    "alpha_inside": 0.0,
                    "alpha_outside": 1.0,
                },
            }
        )

        inside = profile.apply(
            {
                "right_m": 1.0,
                "down_m": 2.0,
                "forward_m": 100.0,
                "range_m": 100.0,
                "disparity_px": 114.0,
            }
        )
        outside = profile.apply(
            {
                "right_m": 1.0,
                "down_m": 2.0,
                "forward_m": 100.0,
                "range_m": 100.0,
                "disparity_px": 130.0,
            }
        )

        self.assertIsNotNone(inside.corrected)
        self.assertIsNotNone(outside.corrected)
        assert inside.corrected is not None
        assert outside.corrected is not None
        self.assertAlmostEqual(inside.corrected["forward_m"], 100.0)
        self.assertAlmostEqual(outside.corrected["forward_m"], 104.0)
        self.assertEqual(inside.status["forward_residual_alpha"], 0.0)
        self.assertEqual(outside.status["forward_residual_alpha"], 1.0)

    def test_estimated_to_truth_sim3_profile_clamps_forward_residual_before_alpha(self) -> None:
        profile = EstimatedToTruthSim3Profile.from_json_dict(
            {
                "mode": "estimated_to_truth_sim3",
                "profile_id": "unit-sim3-residual-clamp",
                "scale": 1.0,
                "rotation_matrix_rdf": [
                    [1.0, 0.0, 0.0],
                    [0.0, 1.0, 0.0],
                    [0.0, 0.0, 1.0],
                ],
                "translation_rdf_m": [0.0, 0.0, 0.0],
                "forward_residual_disparity_coefficients_high_to_low": [4.0],
                "forward_residual_clamp_max_abs_m": 1.5,
                "forward_residual_disparity_suppression_band": {
                    "center_disparity_px": 114.0,
                    "half_width_px": 2.0,
                    "alpha_inside": 0.0,
                    "alpha_outside": 1.0,
                },
            }
        )

        inside = profile.apply(
            {
                "right_m": 1.0,
                "down_m": 2.0,
                "forward_m": 100.0,
                "range_m": 100.0,
                "disparity_px": 114.0,
            }
        )
        outside = profile.apply(
            {
                "right_m": 1.0,
                "down_m": 2.0,
                "forward_m": 100.0,
                "range_m": 100.0,
                "disparity_px": 130.0,
            }
        )

        self.assertIsNotNone(inside.corrected)
        self.assertIsNotNone(outside.corrected)
        assert inside.corrected is not None
        assert outside.corrected is not None
        self.assertAlmostEqual(inside.corrected["forward_m"], 100.0)
        self.assertAlmostEqual(outside.corrected["forward_m"], 101.5)
        self.assertEqual(inside.status["forward_residual_alpha"], 0.0)
        self.assertEqual(outside.status["forward_residual_alpha"], 1.0)
        self.assertEqual(outside.status["forward_residual_unclamped_m"], 4.0)
        self.assertEqual(outside.status["forward_residual_applied_m"], 1.5)

    def test_bounded_gain_policy_applies_down_cell_and_forward_anchor_fallback(self) -> None:
        policy, left, right, raw, camera_kwargs = make_policy_case()

        result = policy.apply(
            left,
            right,
            raw,
            **camera_kwargs,
        )

        self.assertIsNotNone(result.corrected)
        assert result.corrected is not None
        self.assertEqual(result.status["anchor_cell"], "disparity_bin_3")
        self.assertEqual(result.status["candidate_forward"], "raw")
        self.assertEqual(result.status["candidate_down"], "anchor_gain_0.25")
        self.assertAlmostEqual(result.corrected["forward_m"], raw["forward_m"])
        self.assertNotAlmostEqual(result.corrected["down_m"], raw["down_m"])

    def test_bounded_gain_policy_rejects_raw_points_outside_policy_bins(self) -> None:
        policy, left, right, raw, camera_kwargs = make_policy_case()
        out_of_range_raw = dict(raw)
        out_of_range_raw["forward_m"] = 2500.0
        out_of_range_raw["range_m"] = 2500.0

        result = policy.apply(
            left,
            right,
            out_of_range_raw,
            **camera_kwargs,
        )

        self.assertIsNone(result.corrected)
        self.assertEqual(result.status["fallback_reason"], "outside_policy_range")


if __name__ == "__main__":
    unittest.main()
