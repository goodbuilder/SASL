from __future__ import annotations

import importlib.util
import math
import sys
import tempfile
import unittest
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation


ROOT_DIR = Path(__file__).resolve().parents[1]


def load_pipeline_module():
    spec = importlib.util.spec_from_file_location(
        "rtk_stereo_calibration_pipeline_under_test",
        ROOT_DIR / "scripts" / "rtk_stereo_calibration_pipeline.py",
    )
    if spec is None or spec.loader is None:
        raise RuntimeError("cannot load rtk_stereo_calibration_pipeline.py")
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


class RtkStereoCalibrationPipelineTests(unittest.TestCase):
    def test_synthetic_truth_uses_explicit_non_alias_ned_to_rdf_transform(self) -> None:
        module = load_pipeline_module()
        truth_rdf = np.array([20.0, 5.0, 1852.0], dtype=float)

        truth_ned = module.stereo_rdf_to_ned_explicit(truth_rdf)
        roundtrip_rdf = module.ned_to_stereo_rdf_explicit(truth_ned)

        np.testing.assert_allclose(roundtrip_rdf, truth_rdf, atol=1.0e-9)
        self.assertFalse(
            np.allclose(
                truth_rdf,
                np.array([truth_ned[1], truth_ned[2], truth_ned[0]], dtype=float),
                atol=1.0e-9,
                rtol=0.0,
            )
        )
        self.assertIn("explicit_transform", module.SYNTHETIC_NED_TO_RDF_CONTRACT)
        self.assertIn("forbidden", module.SYNTHETIC_NED_TO_RDF_CONTRACT)

    def test_split_preserves_truth_transform_metadata_for_reports(self) -> None:
        module = load_pipeline_module()
        source = module.SyntheticCalibrationDataSource(
            camera=module.CameraModel(),
            injection=module.ErrorInjection(),
            sample_count=8,
            seed=123,
            target_range_m=1852.0,
            target_window_m=30.0,
            target_oversample_count=2,
        )
        loaded = source.load_samples()

        train, validation = module.split_samples(
            loaded,
            train_ratio=0.75,
            seed=321,
            target_range_m=1852.0,
            target_window_m=30.0,
        )
        rows = module.rows_for_samples([*train, *validation])

        self.assertTrue(rows)
        for row in rows:
            self.assertEqual(row["truth_source_frame"], "px4_local_position_ned")
            self.assertEqual(row["truth_source_axis_order"], "north,east,down")
            self.assertIn("explicit_transform", row["truth_rdf_transform_contract"])
            self.assertNotIn("synthetic_bridge: rdf_right=east", row["truth_rdf_transform_contract"])
            self.assertTrue(row["truth_source_to_stereo_rdf_rotation_matrix"])

    def test_triangulation_uses_rdf_axis_contract(self) -> None:
        module = load_pipeline_module()
        camera = module.CameraModel()
        truth_rdf = np.array([7.0, 3.0, 1000.0])
        left_u = camera.cx_px + (truth_rdf[0] + camera.baseline_m * 0.5) * camera.fx_true_px / truth_rdf[2]
        left_v = camera.cy_px + truth_rdf[1] * camera.fy_true_px / truth_rdf[2]
        disparity = camera.fx_true_px * camera.baseline_m / truth_rdf[2]

        raw = module.triangulate_left_disparity(
            left_u_px=left_u,
            left_v_px=left_v,
            disparity_px=disparity,
            fx_px=camera.fx_true_px,
            fy_px=camera.fy_true_px,
            cx_px=camera.cx_px,
            cy_px=camera.cy_px,
            baseline_m=camera.baseline_m,
        )

        np.testing.assert_allclose(raw, truth_rdf, atol=1.0e-9)

    def test_umeyama_recovers_sim3_mapping_for_rdf_rows(self) -> None:
        module = load_pipeline_module()
        source = np.array(
            [
                [2.0, 1.0, 100.0],
                [-3.0, 4.0, 150.0],
                [6.0, -2.0, 220.0],
                [8.0, 5.0, 310.0],
            ],
            dtype=float,
        )
        scale = 1.015
        rotation = Rotation.from_euler("zyx", [0.4, -0.2, 0.1], degrees=True)
        translation = np.array([1.5, -0.7, 5.0])
        target = scale * module.apply_rotation_to_rows(source, rotation) + translation

        fitted_scale, fitted_rotation, fitted_translation = module.fit_umeyama_sim3(source, target)
        corrected = fitted_scale * module.apply_rotation_to_rows(source, fitted_rotation) + fitted_translation

        self.assertTrue(math.isclose(fitted_scale, scale, rel_tol=1.0e-9, abs_tol=1.0e-9))
        np.testing.assert_allclose(corrected, target, atol=1.0e-8)

    def test_truncated_umeyama_sim3_rejects_large_rdf_outlier(self) -> None:
        module = load_pipeline_module()
        source = np.array(
            [
                [2.0, 1.0, 100.0],
                [-3.0, 4.0, 150.0],
                [6.0, -2.0, 220.0],
                [8.0, 5.0, 310.0],
                [-4.0, -3.0, 420.0],
                [9.0, 2.0, 520.0],
            ],
            dtype=float,
        )
        scale = 1.01
        rotation = Rotation.from_euler("zyx", [0.3, -0.1, 0.2], degrees=True)
        translation = np.array([1.0, -0.5, 4.0])
        clean_target = scale * module.apply_rotation_to_rows(source, rotation) + translation
        outlier_target = clean_target.copy()
        outlier_target[-1] += np.array([80.0, -50.0, 120.0])

        plain_scale, plain_rotation, plain_translation = module.fit_umeyama_sim3(source, outlier_target)
        tls_scale, tls_rotation, tls_translation, tls_weights, cutoff = module.fit_truncated_umeyama_sim3(
            source,
            outlier_target,
            threshold_m=10.0,
        )

        plain_error = np.linalg.norm(
            module.predict_sim3(source[:-1], plain_scale, plain_rotation, plain_translation) - clean_target[:-1],
            axis=1,
        )
        tls_error = np.linalg.norm(
            module.predict_sim3(source[:-1], tls_scale, tls_rotation, tls_translation) - clean_target[:-1],
            axis=1,
        )

        self.assertEqual(float(cutoff), 10.0)
        self.assertEqual(int(np.sum(tls_weights > 0.0)), 5)
        self.assertLess(float(np.mean(tls_error)), float(np.mean(plain_error)))

    def test_affine_rdf_recovers_linear_mapping_for_rdf_rows(self) -> None:
        module = load_pipeline_module()
        source = np.array(
            [
                [2.0, 1.0, 100.0],
                [-3.0, 4.0, 150.0],
                [6.0, -2.0, 220.0],
                [8.0, 5.0, 310.0],
                [-4.0, -3.0, 420.0],
                [9.0, 2.0, 520.0],
            ],
            dtype=float,
        )
        matrix = np.array(
            [
                [1.01, 0.02, -0.0005],
                [-0.01, 0.99, 0.0003],
                [0.2, -0.1, 1.004],
            ],
            dtype=float,
        )
        translation = np.array([1.5, -0.7, 5.0])
        target = source @ matrix.T + translation

        fitted_matrix, fitted_translation = module.fit_affine_rdf(source, target)
        corrected = module.predict_affine_rdf(source, fitted_matrix, fitted_translation)

        np.testing.assert_allclose(fitted_matrix, matrix, atol=1.0e-9)
        np.testing.assert_allclose(fitted_translation, translation, atol=1.0e-9)
        np.testing.assert_allclose(corrected, target, atol=1.0e-9)

    def test_diagonal_affine_rdf_recovers_axis_gain_bias_mapping(self) -> None:
        module = load_pipeline_module()
        source = np.array(
            [
                [2.0, 1.0, 100.0],
                [-3.0, 4.0, 150.0],
                [6.0, -2.0, 220.0],
                [8.0, 5.0, 310.0],
                [-4.0, -3.0, 420.0],
                [9.0, 2.0, 520.0],
            ],
            dtype=float,
        )
        gains = np.array([1.02, 0.98, 1.004], dtype=float)
        translation = np.array([1.5, -0.7, 5.0], dtype=float)
        matrix = np.diag(gains)
        target = source @ matrix.T + translation

        fitted_matrix, fitted_translation = module.fit_diagonal_affine_rdf(source, target)
        corrected = module.predict_affine_rdf(source, fitted_matrix, fitted_translation)

        np.testing.assert_allclose(fitted_matrix, matrix, atol=1.0e-9)
        np.testing.assert_allclose(fitted_translation, translation, atol=1.0e-9)
        np.testing.assert_allclose(corrected, target, atol=1.0e-9)

    def test_smoothed_binned_residual_lut_shrinks_noisy_bins(self) -> None:
        module = load_pipeline_module()
        disparity = np.arange(1.0, 13.0)
        residual = np.array([0.0, 0.1, 0.2, 10.0, 0.3, 0.4, 0.5, -8.0, 0.6, 0.7, 0.8, 0.9])

        raw_lut = module.fit_binned_residual_lut(disparity, residual, bin_count=6, min_samples_per_bin=1)
        smoothed_lut = module.fit_smoothed_binned_residual_lut(
            disparity,
            residual,
            bin_count=6,
            min_samples_per_bin=1,
            prior_strength=4.0,
            neighbor_weight=0.25,
        )

        self.assertEqual(len(raw_lut["residual_values_m"]), len(smoothed_lut["residual_values_m"]))
        self.assertIn("raw_residual_values_m", smoothed_lut)
        self.assertLess(
            max(abs(value) for value in smoothed_lut["residual_values_m"]),
            max(abs(value) for value in raw_lut["residual_values_m"]),
        )
        self.assertEqual(
            smoothed_lut["fit_basis"],
            "quantile-binned median residual LUT with empirical-Bayes shrinkage "
            "toward the global median and nearest-neighbor smoothing",
        )

    def test_kernel_smoothed_residual_lut_tracks_smooth_curve(self) -> None:
        module = load_pipeline_module()
        x_values = np.linspace(0.0, 100.0, 21)
        residual = 0.02 * x_values + 0.3

        lut = module.fit_kernel_smoothed_residual_lut(
            x_values,
            residual,
            support_count=5,
            bandwidth=20.0,
        )

        self.assertEqual(len(lut["x_values_px"]), 5)
        self.assertEqual(len(lut["residual_values_m"]), 5)
        self.assertIn("Nadaraya-Watson", lut["fit_basis"])
        center_index = len(lut["x_values_px"]) // 2
        self.assertAlmostEqual(lut["x_values_px"][center_index], 50.0)
        self.assertAlmostEqual(lut["residual_values_m"][center_index], 1.3, delta=0.05)

    def test_local_linear_residual_lut_tracks_boundary_trend(self) -> None:
        module = load_pipeline_module()
        x_values = np.linspace(0.0, 100.0, 21)
        residual = 0.03 * x_values - 0.4

        lut = module.fit_local_linear_residual_lut(
            x_values,
            residual,
            support_count=5,
            bandwidth=20.0,
        )

        self.assertEqual(len(lut["x_values_px"]), 5)
        self.assertEqual(len(lut["residual_values_m"]), 5)
        self.assertEqual(len(lut["local_linear_slopes_m_per_x"]), 5)
        self.assertIn("local-linear", lut["fit_basis"])
        self.assertAlmostEqual(lut["residual_values_m"][0], -0.4, delta=0.05)
        self.assertAlmostEqual(lut["local_linear_slopes_m_per_x"][0], 0.03, delta=0.01)

    def test_smoothing_spline_residual_lut_exports_smooth_runtime_lut(self) -> None:
        module = load_pipeline_module()
        x_values = np.array([0.0, 0.0, 10.0, 20.0, 30.0, 40.0, 50.0, 60.0], dtype=float)
        residual = 0.01 * x_values + np.array([0.0, 0.2, -0.1, 0.05, -0.04, 0.03, -0.02, 0.0])

        lut = module.fit_smoothing_spline_residual_lut(
            x_values,
            residual,
            support_count=5,
            smoothing_factor_scale=1.0,
        )

        self.assertEqual(len(lut["x_values_px"]), len(lut["residual_values_m"]))
        self.assertLessEqual(len(lut["x_values_px"]), 5)
        self.assertEqual(lut["spline_degree"], 3)
        self.assertGreater(lut["spline_smoothing_factor"], 0.0)
        self.assertIn("smoothing-spline", lut["fit_basis"])
        np.testing.assert_allclose(lut["x_values_px"], sorted(lut["x_values_px"]))

    def test_capped_binned_residual_lut_limits_extreme_bins(self) -> None:
        module = load_pipeline_module()
        disparity = np.arange(1.0, 13.0)
        residual = np.array([0.0, 0.1, 0.2, 11.0, 0.3, 0.4, 0.5, -9.0, 0.6, 0.7, 0.8, 0.9])

        raw_lut = module.fit_binned_residual_lut(disparity, residual, bin_count=6, min_samples_per_bin=1)
        capped_lut = module.fit_capped_binned_residual_lut(
            disparity,
            residual,
            bin_count=6,
            min_samples_per_bin=1,
            mad_sigma=1.5,
            quantile=0.75,
        )

        self.assertEqual(len(raw_lut["residual_values_m"]), len(capped_lut["residual_values_m"]))
        self.assertIn("raw_residual_values_m", capped_lut)
        self.assertIn("cap_half_width_m", capped_lut)
        self.assertLess(
            max(abs(value) for value in capped_lut["residual_values_m"]),
            max(abs(value) for value in raw_lut["residual_values_m"]),
        )
        self.assertEqual(
            capped_lut["fit_basis"],
            "quantile-binned median residual LUT with robust winsorized cap "
            "from training residual MAD and absolute-residual quantile",
        )

    def test_monotone_binned_residual_lut_projects_with_pava(self) -> None:
        module = load_pipeline_module()
        forward = np.arange(1.0, 9.0)
        residual = np.array([0.0, 1.0, 0.5, 1.5, 1.2, 2.0, 1.8, 2.5])

        lut = module.fit_monotone_binned_residual_lut(
            forward,
            residual,
            bin_count=8,
            min_samples_per_bin=1,
        )
        residual_lut = np.asarray(lut["residual_values_m"], dtype=float)

        self.assertEqual(len(residual_lut), 8)
        self.assertTrue(
            np.all(np.diff(residual_lut) >= -1.0e-12) or np.all(np.diff(residual_lut) <= 1.0e-12)
        )
        self.assertIn("raw_residual_values_m", lut)
        self.assertIn(lut["monotone_direction"], {"nondecreasing", "nonincreasing"})
        self.assertIn("PAVA isotonic", lut["fit_basis"])

    def test_scale_residual_lut_preserves_x_contract(self) -> None:
        module = load_pipeline_module()
        lut = {
            "x_values_m": [100.0, 200.0],
            "residual_values_m": [4.0, -2.0],
            "fit_basis": "unit raw-forward LUT",
        }

        scaled = module.scale_residual_lut(lut, residual_scale=0.25, fit_basis_suffix="unit blend")

        self.assertEqual(scaled["x_values_m"], [100.0, 200.0])
        self.assertEqual(scaled["residual_values_m"], [1.0, -0.5])
        self.assertEqual(scaled["residual_scale"], 0.25)
        self.assertIn("unit blend", scaled["fit_basis"])

    def test_range_normalized_residual_lut_exports_meter_residuals(self) -> None:
        module = load_pipeline_module()
        forward = np.array([100.0, 200.0, 300.0, 400.0, 500.0, 600.0], dtype=float)
        residual = 0.01 * forward

        lut = module.fit_range_normalized_residual_lut(
            forward,
            residual,
            bin_count=3,
            min_samples_per_bin=1,
            prior_strength=0.0,
        )

        self.assertEqual(len(lut["x_values_m"]), 3)
        np.testing.assert_allclose(
            np.asarray(lut["residual_values_m"], dtype=float),
            0.01 * np.asarray(lut["x_values_m"], dtype=float),
            atol=1.0e-12,
        )
        self.assertIn("normalized_residual_values", lut)
        self.assertIn("raw RDF forward_m", lut["fit_basis"])

    def test_inverse_depth_residual_lut_exports_meter_residuals(self) -> None:
        module = load_pipeline_module()
        raw_forward = np.array([100.0, 200.0, 300.0, 400.0, 500.0, 600.0], dtype=float)
        base_forward = raw_forward * 1.01
        inv_delta = 2.0e-5
        truth_forward = 1.0 / ((1.0 / base_forward) + inv_delta)

        lut = module.fit_inverse_depth_residual_lut(
            raw_forward,
            base_forward,
            truth_forward,
            bin_count=3,
            min_samples_per_bin=1,
            prior_strength=0.0,
        )

        self.assertEqual(len(lut["x_values_m"]), 3)
        expected = 1.0 / (
            (1.0 / np.asarray(lut["base_forward_values_m"], dtype=float))
            + np.asarray(lut["inverse_depth_delta_values_1_per_m"], dtype=float)
        ) - np.asarray(lut["base_forward_values_m"], dtype=float)
        np.testing.assert_allclose(np.asarray(lut["residual_values_m"], dtype=float), expected, atol=1.0e-12)
        np.testing.assert_allclose(
            np.asarray(lut["inverse_depth_delta_values_1_per_m"], dtype=float),
            inv_delta,
            atol=1.0e-12,
        )
        self.assertIn("inverse-depth delta", lut["fit_basis"])

    def test_monotone_inverse_depth_residual_lut_projects_delta(self) -> None:
        module = load_pipeline_module()
        raw_forward = np.array([100.0, 200.0, 300.0, 400.0, 500.0, 600.0], dtype=float)
        base_forward = raw_forward.copy()
        inv_delta = np.array([1.0e-5, 3.0e-5, 2.0e-5, 4.0e-5, 3.5e-5, 5.0e-5], dtype=float)
        truth_forward = 1.0 / ((1.0 / base_forward) + inv_delta)

        lut = module.fit_monotone_inverse_depth_residual_lut(
            raw_forward,
            base_forward,
            truth_forward,
            bin_count=6,
            min_samples_per_bin=1,
            prior_strength=0.0,
        )

        projected_delta = np.asarray(lut["inverse_depth_delta_values_1_per_m"], dtype=float)
        self.assertTrue(
            np.all(np.diff(projected_delta) >= -1.0e-12)
            or np.all(np.diff(projected_delta) <= 1.0e-12)
        )
        self.assertIn("pre_monotone_inverse_depth_delta_values_1_per_m", lut)
        self.assertIn("isotonic", lut["fit_basis"])

    def test_weighted_raw_forward_residual_lut_keeps_meter_contract(self) -> None:
        module = load_pipeline_module()
        raw_forward = np.array([1500.0, 1750.0, 1850.0, 1860.0, 1950.0, 2100.0], dtype=float)
        residual = 0.0001 * np.square(raw_forward - 1852.0) - 0.5
        weights = np.array([0.05, 0.3, 1.0, 1.0, 0.4, 0.05], dtype=float)

        lut = module.fit_polynomial_raw_forward_residual_lut(
            raw_forward,
            residual,
            degree=2,
            ridge_alpha=10.0,
            bin_count=3,
            min_samples_per_bin=1,
            weights=weights,
        )

        self.assertEqual(len(lut["x_values_m"]), len(lut["residual_values_m"]))
        self.assertTrue(lut["weighted_fit"])
        self.assertGreater(lut["mean_weight"], 0.0)
        self.assertIn("raw RDF forward_m", lut["fit_basis"])

    def test_polynomial_raw_forward_residual_lut_exports_quadratic_meter_residuals(self) -> None:
        module = load_pipeline_module()
        forward = np.array([100.0, 200.0, 300.0, 400.0, 500.0, 600.0], dtype=float)
        residual = 0.0001 * forward**2 - 0.02 * forward + 3.0

        lut = module.fit_polynomial_raw_forward_residual_lut(
            forward,
            residual,
            degree=2,
            ridge_alpha=0.0,
            bin_count=3,
            min_samples_per_bin=1,
        )

        self.assertEqual(len(lut["x_values_m"]), 3)
        expected = 0.0001 * np.asarray(lut["x_values_m"], dtype=float) ** 2 - 0.02 * np.asarray(
            lut["x_values_m"],
            dtype=float,
        ) + 3.0
        np.testing.assert_allclose(np.asarray(lut["residual_values_m"], dtype=float), expected, atol=1.0e-8)
        self.assertEqual(lut["polynomial_degree"], 2)
        self.assertIn("Z^2", lut["fit_basis"])

    def test_polynomial_plus_binned_raw_forward_residual_lut_exports_single_runtime_lut(self) -> None:
        module = load_pipeline_module()
        forward = np.array([100.0, 200.0, 300.0, 400.0, 500.0, 600.0], dtype=float)
        polynomial = 0.0001 * forward**2 - 0.02 * forward + 3.0
        local = np.array([0.5, -0.5, 0.25, -0.25, 0.1, -0.1], dtype=float)
        residual = polynomial + local

        lut = module.fit_polynomial_plus_binned_raw_forward_residual_lut(
            forward,
            residual,
            degree=2,
            ridge_alpha=0.0,
            bin_count=3,
            min_samples_per_bin=1,
            residual_shrink=0.5,
        )

        self.assertEqual(len(lut["x_values_m"]), 3)
        self.assertIn("polynomial_residual_values_m", lut)
        self.assertIn("local_residual_values_m", lut)
        self.assertEqual(lut["local_residual_shrink"], 0.5)
        self.assertEqual(len(lut["residual_values_m"]), len(lut["x_values_m"]))
        self.assertIn("residual-of-residual", lut["fit_basis"])

    def test_split_samples_preserves_target_window_in_train_and_validation(self) -> None:
        module = load_pipeline_module()
        samples = []
        for index, forward_m in enumerate([500.0, 650.0, 900.0, 1200.0, 1845.0, 1855.0, 1900.0, 2100.0]):
            truth_rdf = np.array([0.0, 0.0, forward_m], dtype=float)
            samples.append(
                module.CalibrationSample(
                    sample_id=index,
                    t_s=float(index),
                    split="",
                    truth_ned_m=np.array([forward_m, 0.0, 0.0], dtype=float),
                    truth_rdf_m=truth_rdf,
                    raw_rdf_m=truth_rdf.copy(),
                    left_u_px=0.0,
                    left_v_px=0.0,
                    measured_disparity_px=1.0,
                    true_disparity_px=1.0,
                )
            )

        train, validation = module.split_samples(
            samples,
            train_ratio=0.8,
            seed=20260613,
            target_range_m=1852.0,
            target_window_m=30.0,
        )

        self.assertGreaterEqual(sum(abs(sample.truth_range_m - 1852.0) <= 30.0 for sample in train), 1)
        self.assertGreaterEqual(sum(abs(sample.truth_range_m - 1852.0) <= 30.0 for sample in validation), 1)

    def test_target_window_sample_count_uses_truth_range(self) -> None:
        module = load_pipeline_module()
        samples = []
        for index, forward_m in enumerate([1820.0, 1840.0, 1852.0, 1885.0]):
            truth_rdf = np.array([0.0, 0.0, forward_m], dtype=float)
            samples.append(
                module.CalibrationSample(
                    sample_id=index,
                    t_s=float(index),
                    split="",
                    truth_ned_m=np.array([forward_m, 0.0, 0.0], dtype=float),
                    truth_rdf_m=truth_rdf,
                    raw_rdf_m=truth_rdf.copy(),
                    left_u_px=0.0,
                    left_v_px=0.0,
                    measured_disparity_px=1.0,
                    true_disparity_px=1.0,
                )
            )

        self.assertEqual(
            module.target_window_sample_count(samples, target_range_m=1852.0, target_window_m=20.0),
            2,
        )

    def test_synthetic_outlier_injection_changes_raw_rdf_without_changing_truth_contract(self) -> None:
        module = load_pipeline_module()
        camera = module.CameraModel()
        common_kwargs = {
            "camera": camera,
            "sample_count": 8,
            "seed": 123,
            "target_range_m": module.DEFAULT_TARGET_RANGE_M,
            "target_window_m": 30.0,
            "target_oversample_count": 2,
        }
        nominal = module.SyntheticCalibrationDataSource(
            injection=module.ErrorInjection(
                disparity_bias_px=0.0,
                disparity_noise_px=0.0,
                bbox_noise_px=0.0,
                down_bias_px=0.0,
            ),
            **common_kwargs,
        ).load_samples()
        outlier = module.SyntheticCalibrationDataSource(
            injection=module.ErrorInjection(
                disparity_bias_px=0.0,
                disparity_noise_px=0.0,
                bbox_noise_px=0.0,
                disparity_outlier_rate=1.0,
                disparity_outlier_px=0.25,
                bbox_outlier_rate=1.0,
                bbox_outlier_px=2.0,
                down_bias_px=0.0,
            ),
            **common_kwargs,
        ).load_samples()

        nominal_truth = np.vstack([sample.truth_rdf_m for sample in nominal])
        outlier_truth = np.vstack([sample.truth_rdf_m for sample in outlier])
        nominal_raw = np.vstack([sample.raw_rdf_m for sample in nominal])
        outlier_raw = np.vstack([sample.raw_rdf_m for sample in outlier])

        np.testing.assert_allclose(outlier_truth, nominal_truth)
        self.assertGreater(float(np.max(np.linalg.norm(outlier_raw - nominal_raw, axis=1))), 0.1)
        self.assertEqual(module.RDF_AXIS_CONTRACT, "stereo_center_opencv_optical:RDF(right,down,forward)")

    def test_choose_recommendation_keeps_raw_without_validation_target_window(self) -> None:
        module = load_pipeline_module()
        rows = [
            {
                "method": "raw",
                "validation_range_rmse_m": 5.0,
                "validation_right_rmse_m": 5.0,
                "validation_down_rmse_m": 5.0,
                "validation_forward_rmse_m": 5.0,
                "validation_target_abs_mean_error_m": float("inf"),
                "validation_target_window_count": 0,
                "generalization_gap_range_rmse_m": 0.0,
                "model_complexity": 0,
            },
            {
                "method": "candidate",
                "validation_range_rmse_m": 1.0,
                "validation_right_rmse_m": 1.0,
                "validation_down_rmse_m": 1.0,
                "validation_forward_rmse_m": 1.0,
                "validation_target_abs_mean_error_m": float("inf"),
                "validation_target_window_count": 0,
                "generalization_gap_range_rmse_m": 0.0,
                "model_complexity": 3,
            },
        ]

        recommendation, reason = module.choose_recommendation(rows)

        self.assertEqual(recommendation, "raw")
        self.assertIn("No correction candidate passed", reason)

    def test_choose_recommendation_rejects_rdf_axis_regression(self) -> None:
        module = load_pipeline_module()
        rows = [
            {
                "method": "raw",
                "validation_range_rmse_m": 5.0,
                "validation_right_rmse_m": 1.0,
                "validation_down_rmse_m": 1.0,
                "validation_forward_rmse_m": 5.0,
                "validation_target_abs_mean_error_m": 5.0,
                "validation_target_window_count": 2,
                "generalization_gap_range_rmse_m": 0.0,
                "model_complexity": 0,
            },
            {
                "method": "lower_range_but_axis_regresses",
                "validation_range_rmse_m": 0.8,
                "validation_right_rmse_m": 0.5,
                "validation_down_rmse_m": 1.1,
                "validation_forward_rmse_m": 0.7,
                "validation_target_abs_mean_error_m": 0.5,
                "validation_target_window_count": 2,
                "generalization_gap_range_rmse_m": 0.0,
                "model_complexity": 1,
            },
            {
                "method": "axis_safe",
                "validation_range_rmse_m": 1.2,
                "validation_right_rmse_m": 0.6,
                "validation_down_rmse_m": 0.9,
                "validation_forward_rmse_m": 1.0,
                "validation_target_abs_mean_error_m": 0.8,
                "validation_target_window_count": 2,
                "generalization_gap_range_rmse_m": 0.0,
                "model_complexity": 1,
            },
        ]

        recommendation, reason = module.choose_recommendation(rows)

        self.assertEqual(recommendation, "axis_safe")
        self.assertIn("RDF axis non-regression", reason)

    def test_robust_sim3_variants_reduce_outlier_influence_in_rdf_rows(self) -> None:
        module = load_pipeline_module()
        source = np.array(
            [
                [2.0, 1.0, 100.0],
                [-3.0, 4.0, 150.0],
                [6.0, -2.0, 220.0],
                [8.0, 5.0, 310.0],
                [-4.0, -3.0, 420.0],
                [9.0, 2.0, 530.0],
                [1.5, -5.0, 640.0],
                [-7.0, 3.5, 750.0],
            ],
            dtype=float,
        )
        scale = 1.01
        rotation = Rotation.from_euler("zyx", [0.3, -0.15, 0.05], degrees=True)
        translation = np.array([1.0, -0.3, 4.0])
        clean_target = module.predict_sim3(source, scale, rotation, translation)
        target_with_outlier = clean_target.copy()
        target_with_outlier[-1] += np.array([80.0, -45.0, 180.0])

        plain_scale, plain_rotation, plain_translation = module.fit_umeyama_sim3(source, target_with_outlier)
        trimmed_scale, trimmed_rotation, trimmed_translation, trimmed_mask = module.fit_trimmed_umeyama_sim3(
            source,
            target_with_outlier,
            keep_fraction=0.75,
            iterations=4,
        )
        ransac_scale, ransac_rotation, ransac_translation, ransac_mask = module.fit_ransac_umeyama_sim3(
            source,
            target_with_outlier,
            iterations=80,
            threshold_m=2.0,
        )
        huber_scale, huber_rotation, huber_translation, huber_weights = module.fit_huber_umeyama_sim3(
            source,
            target_with_outlier,
            iterations=6,
        )
        tukey_scale, tukey_rotation, tukey_translation, tukey_weights = module.fit_tukey_umeyama_sim3(
            source,
            target_with_outlier,
            iterations=6,
            tuning_m=20.0,
        )
        soft_l1_scale, soft_l1_rotation, soft_l1_translation, soft_l1_info = module.fit_robust_nonlinear_sim3(
            source,
            target_with_outlier,
            loss="soft_l1",
            f_scale_m=2.0,
        )
        clean_rows = slice(0, -1)
        plain_error = np.linalg.norm(
            module.predict_sim3(source[clean_rows], plain_scale, plain_rotation, plain_translation)
            - clean_target[clean_rows],
            axis=1,
        ).mean()
        trimmed_error = np.linalg.norm(
            module.predict_sim3(source[clean_rows], trimmed_scale, trimmed_rotation, trimmed_translation)
            - clean_target[clean_rows],
            axis=1,
        ).mean()
        ransac_error = np.linalg.norm(
            module.predict_sim3(source[clean_rows], ransac_scale, ransac_rotation, ransac_translation)
            - clean_target[clean_rows],
            axis=1,
        ).mean()
        huber_error = np.linalg.norm(
            module.predict_sim3(source[clean_rows], huber_scale, huber_rotation, huber_translation)
            - clean_target[clean_rows],
            axis=1,
        ).mean()
        tukey_error = np.linalg.norm(
            module.predict_sim3(source[clean_rows], tukey_scale, tukey_rotation, tukey_translation)
            - clean_target[clean_rows],
            axis=1,
        ).mean()
        soft_l1_error = np.linalg.norm(
            module.predict_sim3(source[clean_rows], soft_l1_scale, soft_l1_rotation, soft_l1_translation)
            - clean_target[clean_rows],
            axis=1,
        ).mean()

        self.assertLess(trimmed_error, plain_error)
        self.assertLess(ransac_error, plain_error)
        self.assertLess(huber_error, plain_error)
        self.assertLess(soft_l1_error, plain_error)
        self.assertLess(tukey_error, plain_error)
        self.assertFalse(bool(trimmed_mask[-1]))
        self.assertFalse(bool(ransac_mask[-1]))
        self.assertLess(huber_weights[-1], huber_weights[0])
        self.assertLess(tukey_weights[-1], tukey_weights[0])
        self.assertTrue(soft_l1_info["success"])

    def test_runtime_profile_exports_new_sim3_variant_contract(self) -> None:
        module = load_pipeline_module()
        method = module.FittedMethod(
            name="huber_sim3",
            predict=lambda samples: np.empty((0, 3)),
            parameters={
                "scale": 1.0,
                "rotation_matrix_rdf": np.eye(3).tolist(),
                "rotation_euler_zyx_deg": [0.0, 0.0, 0.0],
                "translation_rdf_m": [0.0, 0.0, 0.0],
            },
            model_complexity=8,
        )

        profile = module.runtime_profile_for_method(method, experiment_id="EXP-test")

        self.assertIsNotNone(profile)
        self.assertEqual(profile["mode"], "estimated_to_truth_sim3")
        self.assertEqual(profile["method"], "huber_sim3")
        self.assertIn("RDF(right,down,forward)", profile["axis_contract"])

    def test_runtime_profile_exports_disparity_product_ray_scale_before_sim3(self) -> None:
        module = load_pipeline_module()
        method = module.FittedMethod(
            name="disparity_product_ray_scale_sim3",
            predict=lambda samples: np.empty((0, 3)),
            parameters={
                "raw_rdf_ray_disparity_forward_product_px_m": 100.0,
                "scale": 1.0,
                "rotation_matrix_rdf": np.eye(3).tolist(),
                "rotation_euler_zyx_deg": [0.0, 0.0, 0.0],
                "translation_rdf_m": [0.0, 0.0, 0.0],
            },
            model_complexity=9,
        )

        profile = module.runtime_profile_for_method(method, experiment_id="EXP-test")

        self.assertIsNotNone(profile)
        self.assertEqual(profile["mode"], "estimated_to_truth_sim3")
        self.assertEqual(profile["raw_rdf_ray_disparity_forward_product_px_m"], 100.0)
        self.assertIn("measured disparity", profile["training_contract"])

    def test_synthetic_source_and_methods_improve_validation_range_rmse(self) -> None:
        module = load_pipeline_module()
        camera = module.CameraModel()
        injection = module.ErrorInjection()
        source = module.SyntheticCalibrationDataSource(
            camera=camera,
            injection=injection,
            sample_count=120,
            seed=20260613,
            target_range_m=module.DEFAULT_TARGET_RANGE_M,
            target_window_m=30.0,
            target_oversample_count=10,
        )
        samples = source.load_samples()
        train, validation = module.split_samples(samples, train_ratio=0.8, seed=11)
        methods = module.fit_methods(train, camera=camera)
        raw_method = next(method for method in methods if method.name == "raw")
        corrected_method = next(method for method in methods if method.name == "sim3_residual_disparity")
        method_names = {method.name for method in methods}
        raw_metrics = module.evaluate_predictions(
            validation,
            raw_method.predict(validation),
            target_range_m=module.DEFAULT_TARGET_RANGE_M,
            target_window_m=60.0,
        )
        corrected_metrics = module.evaluate_predictions(
            validation,
            corrected_method.predict(validation),
            target_range_m=module.DEFAULT_TARGET_RANGE_M,
            target_window_m=60.0,
        )

        self.assertLess(corrected_metrics["range_rmse_m"], raw_metrics["range_rmse_m"])
        self.assertGreaterEqual(
            module.target_window_sample_count(
                samples,
                target_range_m=module.DEFAULT_TARGET_RANGE_M,
                target_window_m=30.0,
            ),
            8,
        )
        self.assertIn("sim3_rdf_residual_disparity", method_names)
        self.assertIn("sim3_residual_disparity_deg0", method_names)
        self.assertIn("sim3_residual_disparity_deg1", method_names)
        self.assertIn("sim3_target_weighted_residual_disparity_deg1", method_names)
        self.assertIn("sim3_target_weighted_residual_disparity", method_names)
        self.assertIn("target_weighted_huber_left_disparity_camera_geometry_sim3_sigma080", method_names)
        self.assertIn("target_weighted_huber_left_disparity_camera_geometry_sim3_sigma160", method_names)
        self.assertIn("target_weighted_huber_left_disparity_camera_geometry_sim3_sigma250", method_names)
        self.assertIn("target_weighted_huber_left_disparity_camera_geometry_sim3_sigma400", method_names)
        self.assertIn("target_weighted_huber_left_disparity_camera_geometry_sim3_sigma600", method_names)
        self.assertIn("target_weighted_huber_left_disparity_camera_geometry_sim3_sigma800", method_names)
        self.assertIn(
            "huber_left_disparity_camera_geometry_sim3_quadratic_residual_raw_forward_lut_shrink025",
            method_names,
        )
        self.assertIn(
            "huber_left_disparity_camera_geometry_sim3_quadratic_residual_raw_forward_lut_shrink050",
            method_names,
        )
        self.assertIn(
            "huber_left_disparity_camera_geometry_sim3_quadratic_residual_raw_forward_lut_shrink075",
            method_names,
        )
        self.assertIn(
            "target_weighted_huber_left_disparity_camera_geometry_sim3_quadratic_residual_raw_forward_lut_sigma400_shrink025",
            method_names,
        )
        self.assertIn(
            "target_weighted_huber_left_disparity_camera_geometry_sim3_quadratic_residual_raw_forward_lut_sigma400_shrink050",
            method_names,
        )
        self.assertIn(
            "target_weighted_huber_left_disparity_camera_geometry_sim3_quadratic_residual_raw_forward_lut_sigma400_shrink075",
            method_names,
        )
        sigma_grid_method = next(
            method
            for method in methods
            if method.name == "target_weighted_huber_left_disparity_camera_geometry_sim3_sigma080"
        )
        sigma_grid_profile = module.runtime_profile_for_method(sigma_grid_method, experiment_id="EXP-test")
        self.assertIsNotNone(sigma_grid_profile)
        self.assertIn("left_u_px", sigma_grid_profile["runtime_inputs"])
        self.assertIn("left_v_px", sigma_grid_profile["runtime_inputs"])
        self.assertIn("RDF(right,down,forward)", sigma_grid_profile["axis_contract"])
        self.assertIn("left bbox center", sigma_grid_profile["training_contract"])
        self.assertIn("left_disparity_triangulation_model", sigma_grid_profile)
        shrink_method = next(
            method
            for method in methods
            if method.name
            == "target_weighted_huber_left_disparity_camera_geometry_sim3_quadratic_residual_raw_forward_lut_sigma400_shrink025"
        )
        shrink_profile = module.runtime_profile_for_method(shrink_method, experiment_id="EXP-test")
        self.assertIsNotNone(shrink_profile)
        self.assertIn("left_u_px", shrink_profile["runtime_inputs"])
        self.assertIn("left_v_px", shrink_profile["runtime_inputs"])
        self.assertIn("raw_forward_m", shrink_profile["runtime_inputs"])
        self.assertIn("forward_residual_raw_forward_lut", shrink_profile)
        self.assertEqual(shrink_profile["forward_residual_raw_forward_lut"]["residual_scale"], 0.25)
        unweighted_shrink_method = next(
            method
            for method in methods
            if method.name == "huber_left_disparity_camera_geometry_sim3_quadratic_residual_raw_forward_lut_shrink050"
        )
        unweighted_shrink_profile = module.runtime_profile_for_method(
            unweighted_shrink_method,
            experiment_id="EXP-test",
        )
        self.assertIsNotNone(unweighted_shrink_profile)
        self.assertIn("left_u_px", unweighted_shrink_profile["runtime_inputs"])
        self.assertIn("left_v_px", unweighted_shrink_profile["runtime_inputs"])
        self.assertIn("raw_forward_m", unweighted_shrink_profile["runtime_inputs"])
        self.assertEqual(unweighted_shrink_profile["forward_residual_raw_forward_lut"]["residual_scale"], 0.5)
        self.assertIn("sim3_target_weighted_rdf_residual_disparity", method_names)
        self.assertIn("sim3_piecewise_raw_forward_bins3", method_names)
        self.assertIn("sim3_piecewise_raw_forward_bins5", method_names)
        self.assertIn("sim3_blended_piecewise_raw_forward_bins3_alpha025", method_names)
        self.assertIn("sim3_blended_piecewise_raw_forward_bins3_alpha050", method_names)
        self.assertIn("sim3_blended_piecewise_raw_forward_bins3_alpha075", method_names)
        self.assertIn("sim3_blended_piecewise_raw_forward_bins5_alpha025", method_names)
        self.assertIn("sim3_blended_piecewise_raw_forward_bins5_alpha050", method_names)
        self.assertIn("sim3_blended_piecewise_raw_forward_bins5_alpha075", method_names)
        self.assertIn("tukey_sim3", method_names)
        self.assertIn("tukey_range_weighted_sim3", method_names)
        self.assertIn("diagonal_affine_rdf", method_names)
        self.assertIn("identity_shrunk_diagonal_affine_rdf_lambda0p01", method_names)
        self.assertIn("identity_shrunk_diagonal_affine_rdf_lambda1", method_names)
        self.assertIn("affine_rdf", method_names)
        self.assertIn("ridge_affine_rdf", method_names)
        self.assertIn("disparity_linear_then_sim3", method_names)
        self.assertIn("disparity_quadratic_then_sim3", method_names)
        self.assertIn("joint_disparity_ray_rotation_nls", method_names)
        self.assertIn("huber_joint_disparity_ray_rotation_nls", method_names)
        self.assertIn("huber_joint_disparity_ray_rotation_nls_fscale0p75", method_names)
        self.assertIn("huber_joint_disparity_ray_rotation_nls_fscale1p0", method_names)
        self.assertIn("huber_joint_disparity_ray_rotation_nls_fscale2p0", method_names)
        self.assertIn("huber_joint_disparity_ray_rotation_nls_fscale3p0", method_names)
        self.assertIn("target_weighted_huber_joint_disparity_ray_rotation_nls_sigma030", method_names)
        self.assertIn("target_weighted_huber_joint_disparity_ray_rotation_nls_sigma045", method_names)
        self.assertIn("target_weighted_huber_joint_disparity_ray_rotation_nls_sigma060", method_names)
        self.assertIn("target_weighted_huber_joint_disparity_ray_rotation_nls_sigma060_fscale0p75", method_names)
        self.assertIn("target_weighted_huber_joint_disparity_ray_rotation_nls_sigma060_fscale1p0", method_names)
        self.assertIn("target_weighted_huber_joint_disparity_ray_rotation_nls_sigma060_fscale2p0", method_names)
        self.assertIn("target_weighted_huber_joint_disparity_ray_rotation_nls_sigma060_fscale3p0", method_names)
        self.assertIn("target_weighted_huber_joint_disparity_ray_rotation_nls_sigma090", method_names)
        self.assertIn("target_weighted_huber_joint_disparity_ray_rotation_nls_sigma120", method_names)
        self.assertIn("target_weighted_huber_joint_disparity_ray_rotation_nls_sigma160", method_names)
        self.assertIn("target_weighted_huber_joint_disparity_ray_rotation_nls_sigma250", method_names)
        self.assertIn("global_inverse_depth_linear", method_names)
        self.assertIn("inverse_depth_linear_then_sim3", method_names)
        self.assertIn("huber_global_inverse_depth_linear", method_names)
        self.assertIn("huber_inverse_depth_linear_then_sim3", method_names)
        self.assertIn("ridge_global_inverse_depth_linear_lambda0p1", method_names)
        self.assertIn("ridge_global_inverse_depth_linear_lambda1p0", method_names)
        self.assertIn("ridge_global_inverse_depth_linear_lambda10p0", method_names)
        self.assertIn("target_weighted_global_inverse_depth_linear", method_names)
        self.assertIn("target_weighted_inverse_depth_linear_then_sim3", method_names)
        self.assertIn("target_weighted_huber_global_inverse_depth_linear", method_names)
        self.assertIn("target_weighted_huber_inverse_depth_linear_then_sim3", method_names)
        self.assertIn("theil_sen_global_inverse_depth_linear", method_names)
        self.assertIn("theil_sen_inverse_depth_linear_then_sim3", method_names)
        self.assertIn("target_weighted_theil_sen_global_inverse_depth_linear", method_names)
        self.assertIn("target_weighted_theil_sen_inverse_depth_linear_then_sim3", method_names)
        self.assertIn("target_weighted_ridge_global_inverse_depth_linear_lambda0p1", method_names)
        self.assertIn("target_weighted_ridge_global_inverse_depth_linear_lambda1p0", method_names)
        self.assertIn("target_weighted_ridge_global_inverse_depth_linear_lambda10p0", method_names)
        self.assertIn("target_weighted_global_inverse_depth_linear_sigma060", method_names)
        self.assertIn("target_weighted_global_inverse_depth_linear_sigma250", method_names)
        self.assertIn("target_weighted_inverse_depth_linear_then_sim3_sigma060", method_names)
        self.assertIn("target_weighted_inverse_depth_linear_then_sim3_sigma250", method_names)
        self.assertIn("target_shrunk_global_inverse_depth_linear_shrink025", method_names)
        self.assertIn("target_shrunk_global_inverse_depth_linear_shrink040", method_names)
        self.assertIn("target_shrunk_global_inverse_depth_linear_shrink050", method_names)
        self.assertIn("target_shrunk_global_inverse_depth_linear_shrink060", method_names)
        self.assertIn("target_shrunk_global_inverse_depth_linear_shrink075", method_names)
        self.assertIn("target_shrunk_inverse_depth_linear_then_sim3_shrink025", method_names)
        self.assertIn("target_shrunk_inverse_depth_linear_then_sim3_shrink040", method_names)
        self.assertIn("target_shrunk_inverse_depth_linear_then_sim3_shrink050", method_names)
        self.assertIn("target_shrunk_inverse_depth_linear_then_sim3_shrink060", method_names)
        self.assertIn("target_shrunk_inverse_depth_linear_then_sim3_shrink075", method_names)
        self.assertIn("target_shrunk_huber_global_inverse_depth_linear_shrink025", method_names)
        self.assertIn("target_shrunk_huber_global_inverse_depth_linear_shrink040", method_names)
        self.assertIn("target_shrunk_huber_global_inverse_depth_linear_shrink050", method_names)
        self.assertIn("target_shrunk_huber_global_inverse_depth_linear_shrink060", method_names)
        self.assertIn("target_shrunk_huber_global_inverse_depth_linear_shrink075", method_names)
        self.assertIn("target_shrunk_huber_inverse_depth_linear_then_sim3_shrink025", method_names)
        self.assertIn("target_shrunk_huber_inverse_depth_linear_then_sim3_shrink040", method_names)
        self.assertIn("target_shrunk_huber_inverse_depth_linear_then_sim3_shrink050", method_names)
        self.assertIn("target_shrunk_huber_inverse_depth_linear_then_sim3_shrink060", method_names)
        self.assertIn("target_shrunk_huber_inverse_depth_linear_then_sim3_shrink075", method_names)
        self.assertIn("disparity_linear_then_sim3_quadratic_residual_raw_forward_lut", method_names)
        self.assertIn("disparity_quadratic_then_sim3_quadratic_residual_raw_forward_lut", method_names)
        self.assertIn("disparity_linear_then_sim3_inverse_depth_residual_raw_forward_lut", method_names)
        self.assertIn("disparity_quadratic_then_sim3_inverse_depth_residual_raw_forward_lut", method_names)
        self.assertIn("target_weighted_disparity_linear_then_sim3", method_names)
        self.assertIn("target_weighted_disparity_quadratic_then_sim3", method_names)
        self.assertIn("target_weighted_disparity_linear_then_sim3_quadratic_residual_raw_forward_lut", method_names)
        self.assertIn("target_weighted_disparity_quadratic_then_sim3_quadratic_residual_raw_forward_lut", method_names)
        self.assertIn("target_weighted_disparity_linear_then_sim3_inverse_depth_residual_raw_forward_lut", method_names)
        self.assertIn("target_weighted_disparity_quadratic_then_sim3_inverse_depth_residual_raw_forward_lut", method_names)
        self.assertIn(
            "target_weighted_disparity_linear_then_sim3_target_weighted_quadratic_residual_raw_forward_lut",
            method_names,
        )
        self.assertIn(
            "target_weighted_disparity_linear_then_sim3_target_weighted_quadratic_residual_raw_forward_lut_sigma040",
            method_names,
        )
        self.assertIn(
            "target_weighted_disparity_linear_then_sim3_target_weighted_quadratic_residual_raw_forward_lut_sigma160_shrink075",
            method_names,
        )
        self.assertIn(
            "target_weighted_disparity_quadratic_then_sim3_target_weighted_quadratic_residual_raw_forward_lut",
            method_names,
        )
        self.assertIn(
            "target_weighted_disparity_quadratic_then_sim3_target_weighted_quadratic_residual_raw_forward_lut_sigma040",
            method_names,
        )
        self.assertIn(
            "target_weighted_disparity_quadratic_then_sim3_target_weighted_quadratic_residual_raw_forward_lut_shrink075",
            method_names,
        )
        self.assertIn(
            "target_weighted_disparity_linear_then_sim3_target_weighted_quadratic_residual_raw_forward_lut_shrink025",
            method_names,
        )
        self.assertIn(
            "target_weighted_disparity_quadratic_then_sim3_target_weighted_quadratic_residual_raw_forward_lut_shrink050",
            method_names,
        )
        self.assertIn("affine_binned_residual_disparity", method_names)
        self.assertIn("affine_binned_residual_raw_forward", method_names)
        self.assertIn("affine_binned_residual_raw_forward_bins11", method_names)
        self.assertIn("affine_monotone_binned_residual_raw_forward", method_names)
        self.assertIn("affine_monotone_binned_residual_raw_forward_bins11", method_names)
        self.assertIn("affine_range_normalized_binned_residual_raw_forward", method_names)
        self.assertIn("affine_range_normalized_binned_residual_raw_forward_bins11", method_names)
        self.assertIn("affine_inverse_depth_binned_residual_raw_forward", method_names)
        self.assertIn("affine_inverse_depth_binned_residual_raw_forward_bins11", method_names)
        self.assertIn("affine_monotone_inverse_depth_binned_residual_raw_forward", method_names)
        self.assertIn("affine_monotone_inverse_depth_binned_residual_raw_forward_bins11", method_names)
        self.assertIn("affine_quadratic_residual_raw_forward_lut", method_names)
        self.assertIn("affine_quadratic_local_residual_raw_forward_lut_bins3_shrink025", method_names)
        self.assertIn("affine_quadratic_local_residual_raw_forward_lut_bins3", method_names)
        self.assertIn("affine_quadratic_local_residual_raw_forward_lut_bins5_shrink025", method_names)
        self.assertIn("affine_quadratic_local_residual_raw_forward_lut_bins5", method_names)
        self.assertIn("affine_quadratic_local_residual_raw_forward_lut_shrink025", method_names)
        self.assertIn("affine_quadratic_local_residual_raw_forward_lut", method_names)
        self.assertIn("affine_quadratic_local_residual_raw_forward_lut_shrink075", method_names)
        self.assertIn("affine_binned_residual_disparity_bins5", method_names)
        self.assertIn("affine_binned_residual_disparity_bins9", method_names)
        self.assertIn("affine_binned_residual_disparity_bins11", method_names)
        self.assertIn("affine_smoothed_binned_residual_disparity", method_names)
        self.assertIn("affine_smoothed_binned_residual_disparity_bins11", method_names)
        self.assertIn("affine_capped_binned_residual_disparity", method_names)
        self.assertIn("affine_capped_binned_residual_disparity_bins11", method_names)
        self.assertIn("affine_binned_rdf_residual_disparity", method_names)
        self.assertIn("sim3_ridge_residual_disparity", method_names)
        self.assertIn("sim3_binned_residual_disparity", method_names)
        self.assertIn("sim3_binned_residual_raw_forward", method_names)
        self.assertIn("sim3_binned_residual_raw_forward_bins11", method_names)
        self.assertIn("sim3_kernel_smoothed_residual_raw_forward_sigma060", method_names)
        self.assertIn("sim3_kernel_smoothed_residual_raw_forward_sigma120", method_names)
        self.assertIn("sim3_kernel_smoothed_residual_raw_forward_sigma250", method_names)
        self.assertIn("sim3_monotone_binned_residual_raw_forward", method_names)
        self.assertIn("sim3_monotone_binned_residual_raw_forward_bins11", method_names)
        self.assertIn("sim3_range_normalized_binned_residual_raw_forward", method_names)
        self.assertIn("sim3_range_normalized_binned_residual_raw_forward_bins11", method_names)
        self.assertIn("sim3_inverse_depth_binned_residual_raw_forward", method_names)
        self.assertIn("sim3_inverse_depth_binned_residual_raw_forward_bins11", method_names)
        self.assertIn("sim3_monotone_inverse_depth_binned_residual_raw_forward", method_names)
        self.assertIn("sim3_monotone_inverse_depth_binned_residual_raw_forward_bins11", method_names)
        self.assertIn("sim3_quadratic_residual_raw_forward_lut", method_names)
        self.assertIn("sim3_ridge_raw_rdf_residual_field_deg1_alpha10", method_names)
        self.assertIn("sim3_ridge_raw_rdf_residual_field_deg1_alpha100_cap4m", method_names)
        self.assertIn("sim3_ridge_raw_rdf_residual_field_deg2_alpha100_cap4m", method_names)
        self.assertIn("sim3_ridge_raw_rdf_residual_field_deg2_alpha1000_cap4m", method_names)
        self.assertIn("sim3_quadratic_local_residual_raw_forward_lut_bins3_shrink025", method_names)
        self.assertIn("sim3_quadratic_local_residual_raw_forward_lut_bins3", method_names)
        self.assertIn("sim3_quadratic_local_residual_raw_forward_lut_bins5_shrink025", method_names)
        self.assertIn("sim3_quadratic_local_residual_raw_forward_lut_bins5", method_names)
        self.assertIn("sim3_quadratic_local_residual_raw_forward_lut_shrink025", method_names)
        self.assertIn("sim3_quadratic_local_residual_raw_forward_lut", method_names)
        self.assertIn("sim3_quadratic_local_residual_raw_forward_lut_shrink075", method_names)
        self.assertIn("sim3_blended_binned_residual_disparity_raw_forward", method_names)
        self.assertIn("sim3_blended_binned_residual_disparity_raw_forward_bins11", method_names)
        self.assertIn("sim3_blended_binned_residual_disparity_raw_forward_alpha025", method_names)
        self.assertIn("sim3_blended_binned_residual_disparity_raw_forward_alpha075", method_names)
        self.assertIn("sim3_blended_binned_residual_disparity_raw_forward_bins11_alpha025", method_names)
        self.assertIn("sim3_blended_binned_residual_disparity_raw_forward_bins11_alpha075", method_names)
        self.assertIn("sim3_agreement_gated_binned_residual_disparity_raw_forward", method_names)
        self.assertIn("sim3_agreement_gated_binned_residual_disparity_raw_forward_bins11", method_names)
        self.assertIn("sim3_binned_residual_disparity_bins5", method_names)
        self.assertIn("sim3_binned_residual_disparity_bins9", method_names)
        self.assertIn("sim3_binned_residual_disparity_bins11", method_names)
        self.assertIn("sim3_kernel_smoothed_binned_residual_disparity_sigma020", method_names)
        self.assertIn("sim3_kernel_smoothed_binned_residual_disparity_sigma040", method_names)
        self.assertIn("sim3_kernel_smoothed_binned_residual_disparity_sigma080", method_names)
        self.assertIn("sim3_kernel_smoothed_binned_residual_disparity_sigma020_support5", method_names)
        self.assertIn("sim3_kernel_smoothed_binned_residual_disparity_sigma040_support5", method_names)
        self.assertIn("sim3_kernel_smoothed_binned_residual_disparity_sigma080_support5", method_names)
        self.assertIn("sim3_kernel_smoothed_binned_residual_disparity_sigma020_support7", method_names)
        self.assertIn("sim3_kernel_smoothed_binned_residual_disparity_sigma040_support7", method_names)
        self.assertIn("sim3_kernel_smoothed_binned_residual_disparity_sigma080_support7", method_names)
        self.assertIn("sim3_smoothed_binned_residual_disparity", method_names)
        self.assertIn("sim3_smoothed_binned_residual_disparity_bins11", method_names)
        self.assertIn("sim3_capped_binned_residual_disparity", method_names)
        self.assertIn("sim3_capped_binned_residual_disparity_bins11", method_names)
        self.assertIn("sim3_binned_rdf_residual_disparity", method_names)
        self.assertIn("sim3_ridge_rdf_residual_disparity", method_names)
        global_rotation = next(method for method in methods if method.name == "global_rotation")
        global_scale_rotation = next(method for method in methods if method.name == "global_scale_rotation")
        global_rotation_profile = module.runtime_profile_for_method(global_rotation, experiment_id="EXP-test")
        global_scale_profile = module.runtime_profile_for_method(global_scale_rotation, experiment_id="EXP-test")
        self.assertIsNotNone(global_rotation_profile)
        self.assertIsNotNone(global_scale_profile)
        assert global_rotation_profile is not None
        assert global_scale_profile is not None
        self.assertEqual(global_rotation_profile["translation_rdf_m"], [0.0, 0.0, 0.0])
        self.assertEqual(global_scale_profile["translation_rdf_m"], [0.0, 0.0, 0.0])
        self.assertEqual(module.RDF_AXIS_CONTRACT, "stereo_center_opencv_optical:RDF(right,down,forward)")

    def test_runtime_profile_exports_rdf_residual_disparity_coefficients(self) -> None:
        module = load_pipeline_module()
        method = module.FittedMethod(
            name="sim3_rdf_residual_disparity",
            predict=lambda samples: np.empty((0, 3)),
            parameters={
                "scale": 1.0,
                "rotation_matrix_rdf": np.eye(3).tolist(),
                "rotation_euler_zyx_deg": [0.0, 0.0, 0.0],
                "translation_rdf_m": [0.0, 0.0, 0.0],
                "rdf_residual_disparity_coefficients_high_to_low": {
                    "right": [0.1, 0.0],
                    "down": [-0.2],
                    "forward": [0.0, 1.5],
                },
            },
            model_complexity=13,
        )

        profile = module.runtime_profile_for_method(method, experiment_id="EXP-test")

        self.assertIsNotNone(profile)
        assert profile is not None
        self.assertEqual(profile["method"], "sim3_rdf_residual_disparity")
        self.assertEqual(
            profile["rdf_residual_disparity_coefficients_high_to_low"]["forward"],
            [0.0, 1.5],
        )

    def test_runtime_profile_exports_raw_rdf_residual_polynomial(self) -> None:
        module = load_pipeline_module()
        residual_model = {
            "degree": 1,
            "alpha": 10.0,
            "feature_names": [
                "1",
                "raw_right_over_raw_forward",
                "raw_down_over_raw_forward",
                "raw_forward_normalized",
            ],
            "forward_center_m": 1800.0,
            "forward_scale_m": 100.0,
            "coefficient_matrix_m": [
                [1.0, 2.0, 3.0],
                [0.1, 0.2, 0.3],
                [-0.1, -0.2, -0.3],
                [0.0, 0.0, 0.5],
            ],
            "residual_norm_cap_m": 4.0,
        }
        method = module.FittedMethod(
            name="sim3_ridge_raw_rdf_residual_field_deg1_alpha10",
            predict=lambda samples: np.empty((0, 3)),
            parameters={
                "scale": 1.0,
                "rotation_matrix_rdf": np.eye(3).tolist(),
                "rotation_euler_zyx_deg": [0.0, 0.0, 0.0],
                "translation_rdf_m": [0.0, 0.0, 0.0],
                "rdf_residual_raw_rdf_polynomial": residual_model,
            },
            model_complexity=20,
        )

        profile = module.runtime_profile_for_method(method, experiment_id="EXP-test")

        self.assertIsNotNone(profile)
        assert profile is not None
        self.assertEqual(profile["mode"], "estimated_to_truth_sim3")
        self.assertEqual(
            profile["rdf_residual_raw_rdf_polynomial"]["coefficient_matrix_m"][0],
            [1.0, 2.0, 3.0],
        )
        self.assertIn("runtime apply uses only raw stereo RDF", profile["training_contract"])

    def test_runtime_profile_exports_ray_disparity_correction_before_sim3(self) -> None:
        module = load_pipeline_module()
        method = module.FittedMethod(
            name="disparity_linear_then_sim3",
            predict=lambda samples: np.empty((0, 3)),
            parameters={
                "scale": 1.0,
                "rotation_matrix_rdf": np.eye(3).tolist(),
                "rotation_euler_zyx_deg": [0.0, 0.0, 0.0],
                "translation_rdf_m": [0.0, 0.0, 0.0],
                "raw_rdf_ray_disparity_correction_coefficients_high_to_low": [1.1, -0.2],
            },
            model_complexity=9,
        )

        profile = module.runtime_profile_for_method(method, experiment_id="EXP-test")

        self.assertIsNotNone(profile)
        assert profile is not None
        self.assertEqual(profile["mode"], "estimated_to_truth_sim3")
        self.assertEqual(
            profile["raw_rdf_ray_disparity_correction_coefficients_high_to_low"],
            [1.1, -0.2],
        )
        self.assertIn("measured disparity", profile["training_contract"])

    def test_runtime_profile_exports_piecewise_raw_forward_sim3(self) -> None:
        module = load_pipeline_module()
        piecewise_model = {
            "x_values_m": [1000.0, 2000.0],
            "transforms": [
                {
                    "scale": 1.0,
                    "rotation_matrix_rdf": np.eye(3).tolist(),
                    "translation_rdf_m": [1.0, 0.0, 0.0],
                    "sample_count": 8,
                },
                {
                    "scale": 1.01,
                    "rotation_matrix_rdf": np.eye(3).tolist(),
                    "translation_rdf_m": [0.0, 2.0, 0.0],
                    "sample_count": 8,
                },
            ],
        }
        method = module.FittedMethod(
            name="sim3_piecewise_raw_forward_bins3",
            predict=lambda samples: np.empty((0, 3)),
            parameters={
                "scale": 1.0,
                "rotation_matrix_rdf": np.eye(3).tolist(),
                "rotation_euler_zyx_deg": [0.0, 0.0, 0.0],
                "translation_rdf_m": [0.0, 0.0, 0.0],
                "piecewise_sim3_raw_forward": piecewise_model,
            },
            model_complexity=21,
        )

        profile = module.runtime_profile_for_method(method, experiment_id="EXP-test")

        self.assertIsNotNone(profile)
        assert profile is not None
        self.assertEqual(profile["mode"], "estimated_to_truth_sim3")
        self.assertEqual(profile["piecewise_sim3_raw_forward"]["x_values_m"], [1000.0, 2000.0])
        self.assertIn("raw_forward_m", profile["training_contract"])

    def test_runtime_profile_exports_blended_piecewise_raw_forward_sim3(self) -> None:
        module = load_pipeline_module()
        piecewise_model = {
            "x_values_m": [1000.0],
            "transforms": [
                {
                    "scale": 1.0,
                    "rotation_matrix_rdf": np.eye(3).tolist(),
                    "translation_rdf_m": [10.0, 0.0, 0.0],
                    "sample_count": 8,
                }
            ],
        }
        method = module.FittedMethod(
            name="sim3_blended_piecewise_raw_forward_bins3_alpha050",
            predict=lambda samples: np.empty((0, 3)),
            parameters={
                "scale": 1.0,
                "rotation_matrix_rdf": np.eye(3).tolist(),
                "rotation_euler_zyx_deg": [0.0, 0.0, 0.0],
                "translation_rdf_m": [0.0, 20.0, 0.0],
                "piecewise_sim3_raw_forward": piecewise_model,
                "piecewise_sim3_blend_alpha": 0.5,
            },
            model_complexity=10,
        )

        profile = module.runtime_profile_for_method(method, experiment_id="EXP-test")

        self.assertIsNotNone(profile)
        assert profile is not None
        self.assertEqual(profile["mode"], "estimated_to_truth_sim3")
        self.assertEqual(profile["piecewise_sim3_blend_alpha"], 0.5)
        self.assertEqual(profile["piecewise_sim3_raw_forward"]["x_values_m"], [1000.0])
        self.assertIn("convex-blends", profile["training_contract"])

    def test_runtime_profile_exports_ray_disparity_correction_with_raw_forward_lut(self) -> None:
        module = load_pipeline_module()
        method = module.FittedMethod(
            name="disparity_quadratic_then_sim3_quadratic_residual_raw_forward_lut",
            predict=lambda samples: np.empty((0, 3)),
            parameters={
                "scale": 1.0,
                "rotation_matrix_rdf": np.eye(3).tolist(),
                "rotation_euler_zyx_deg": [0.0, 0.0, 0.0],
                "translation_rdf_m": [0.0, 0.0, 0.0],
                "raw_rdf_ray_disparity_correction_coefficients_high_to_low": [0.01, 1.0, -0.2],
                "forward_residual_raw_forward_lut": {
                    "x_values_m": [1000.0, 2000.0],
                    "residual_values_m": [3.0, 5.0],
                },
            },
            model_complexity=13,
        )

        profile = module.runtime_profile_for_method(method, experiment_id="EXP-test")

        self.assertIsNotNone(profile)
        assert profile is not None
        self.assertEqual(profile["mode"], "estimated_to_truth_sim3")
        self.assertEqual(profile["forward_residual_raw_forward_lut"]["x_values_m"], [1000.0, 2000.0])
        self.assertEqual(
            profile["raw_rdf_ray_disparity_correction_coefficients_high_to_low"],
            [0.01, 1.0, -0.2],
        )

    def test_runtime_profile_exports_disparity_polynomial_method(self) -> None:
        module = load_pipeline_module()
        method = module.FittedMethod(
            name="global_disparity_linear",
            predict=lambda samples: np.empty((0, 3)),
            parameters={"d_corr_coefficients_high_to_low": [1.1, -0.2]},
            model_complexity=2,
        )

        profile = module.runtime_profile_for_method(method, experiment_id="EXP-test")

        self.assertIsNotNone(profile)
        assert profile is not None
        self.assertEqual(profile["mode"], "estimated_to_truth_disparity_poly")
        self.assertEqual(profile["correction_space"], "raw_rdf_ray_scale")
        self.assertEqual(profile["d_corr_coefficients_high_to_low"], [1.1, -0.2])

    def test_runtime_profile_exports_inverse_depth_ray_method(self) -> None:
        module = load_pipeline_module()
        method = module.FittedMethod(
            name="huber_global_inverse_depth_linear",
            predict=lambda samples: np.empty((0, 3)),
            parameters={
                "inverse_depth_coefficients_high_to_low": [0.98, 1.0e-6],
                "correction_space": "raw_rdf_inverse_depth_ray_scale",
                "huber_inverse_depth_fit": {"mean_irls_weight": 0.9},
            },
            model_complexity=2,
        )

        profile = module.runtime_profile_for_method(method, experiment_id="EXP-test")

        self.assertIsNotNone(profile)
        assert profile is not None
        self.assertEqual(profile["mode"], "estimated_to_truth_disparity_poly")
        self.assertEqual(profile["correction_space"], "raw_rdf_inverse_depth_ray_scale")
        self.assertEqual(profile["inverse_depth_coefficients_high_to_low"], [0.98, 1.0e-6])

    def test_huber_inverse_depth_ray_fit_downweights_outlier(self) -> None:
        module = load_pipeline_module()
        raw_forward = np.array([500.0, 700.0, 900.0, 1100.0, 1300.0, 1500.0, 1700.0])
        truth_forward = raw_forward * 1.01 + 2.0
        truth_forward[-1] += 180.0

        least_squares_coeff = module.fit_inverse_depth_ray_coefficients(raw_forward, truth_forward)
        huber_coeff, info = module.fit_huber_inverse_depth_ray_coefficients(raw_forward, truth_forward)
        nominal_inverse_depth = 1.0 / (raw_forward[:-1] * 1.01 + 2.0)
        x_values = 1.0 / raw_forward[:-1]

        least_squares_error = float(np.mean(np.abs(np.polyval(least_squares_coeff, x_values) - nominal_inverse_depth)))
        huber_error = float(np.mean(np.abs(np.polyval(huber_coeff, x_values) - nominal_inverse_depth)))

        self.assertLess(huber_error, least_squares_error)
        self.assertLess(info["min_irls_weight"], 1.0)

    def test_theil_sen_inverse_depth_ray_fit_resists_outlier(self) -> None:
        module = load_pipeline_module()
        raw_forward = np.array([500.0, 700.0, 900.0, 1100.0, 1300.0, 1500.0, 1700.0])
        truth_forward = raw_forward * 1.01 + 2.0
        truth_forward[-1] += 180.0

        least_squares_coeff = module.fit_inverse_depth_ray_coefficients(raw_forward, truth_forward)
        theil_sen_coeff, info = module.fit_theil_sen_inverse_depth_ray_coefficients(raw_forward, truth_forward)
        nominal_inverse_depth = 1.0 / (raw_forward[:-1] * 1.01 + 2.0)
        x_values = 1.0 / raw_forward[:-1]

        least_squares_error = float(np.mean(np.abs(np.polyval(least_squares_coeff, x_values) - nominal_inverse_depth)))
        theil_sen_error = float(np.mean(np.abs(np.polyval(theil_sen_coeff, x_values) - nominal_inverse_depth)))

        self.assertLess(theil_sen_error, least_squares_error)
        self.assertGreater(info["pair_count"], 0)

    def test_ridge_inverse_depth_ray_fit_shrinks_toward_identity(self) -> None:
        module = load_pipeline_module()
        raw_forward = np.array([500.0, 700.0, 900.0, 1100.0, 1300.0, 1500.0, 1700.0])
        truth_forward = raw_forward * 1.04 + 8.0

        loose_coeff, loose_info = module.fit_ridge_inverse_depth_ray_coefficients(
            raw_forward,
            truth_forward,
            ridge_lambda=0.01,
        )
        tight_coeff, tight_info = module.fit_ridge_inverse_depth_ray_coefficients(
            raw_forward,
            truth_forward,
            ridge_lambda=100.0,
        )
        identity = np.array([1.0, 0.0])

        self.assertLess(np.linalg.norm(tight_coeff - identity), np.linalg.norm(loose_coeff - identity))
        self.assertEqual(loose_info["valid_count"], raw_forward.size)
        self.assertEqual(tight_info["prior_coefficients_high_to_low"], [1.0, 0.0])

    def test_pipeline_uses_shared_method_contract_taxonomy(self) -> None:
        module = load_pipeline_module()

        fields = module.method_contract_fields("target_weighted_global_inverse_depth_linear_sigma250")

        self.assertEqual(fields["registration_model"], "raw_rdf_inverse_depth_ray_scale")
        self.assertEqual(fields["runtime_profile_family"], "estimated_to_truth_disparity_poly")
        self.assertIn("target_range_weighting", fields["regularization_model"])
        huber_fields = module.method_contract_fields("target_weighted_huber_global_inverse_depth_linear")
        self.assertEqual(huber_fields["registration_model"], "raw_rdf_inverse_depth_ray_scale")
        self.assertIn("huber_irls", huber_fields["regularization_model"])
        self.assertIn("target_range_weighting", huber_fields["regularization_model"])
        shrunk_huber_fields = module.method_contract_fields(
            "target_shrunk_huber_global_inverse_depth_linear_shrink050"
        )
        self.assertEqual(shrunk_huber_fields["registration_model"], "raw_rdf_inverse_depth_ray_scale")
        ridge_fields = module.method_contract_fields("target_weighted_ridge_global_inverse_depth_linear_lambda1p0")
        self.assertEqual(ridge_fields["registration_model"], "raw_rdf_inverse_depth_ray_scale")
        self.assertIn("ridge", ridge_fields["regularization_model"])
        self.assertIn("huber_irls", shrunk_huber_fields["regularization_model"])
        self.assertIn("target_range_weighting", shrunk_huber_fields["regularization_model"])
        self.assertIn("target_weight_shrinkage", shrunk_huber_fields["regularization_model"])
        shrunk_fields = module.method_contract_fields("identity_shrunk_diagonal_affine_rdf_lambda1")
        self.assertEqual(shrunk_fields["registration_model"], "affine_rdf_estimated_to_truth")
        self.assertEqual(shrunk_fields["runtime_profile_family"], "estimated_to_truth_affine_rdf")
        self.assertIn("identity_gain_shrinkage", shrunk_fields["regularization_model"])
        blend_fields = module.method_contract_fields(
            "sim3_blended_binned_residual_disparity_raw_forward_bins11_alpha025"
        )
        self.assertEqual(blend_fields["registration_model"], "sim3_estimated_to_truth")
        self.assertIn("dual_key_residual_blend", blend_fields["regularization_model"])
        self.assertIn("dual_key_blend_alpha_grid", blend_fields["regularization_model"])
        piecewise_blend_fields = module.method_contract_fields(
            "sim3_blended_piecewise_raw_forward_bins5_alpha050"
        )
        self.assertEqual(
            piecewise_blend_fields["registration_model"],
            "piecewise_raw_forward_sim3_estimated_to_truth",
        )
        self.assertEqual(piecewise_blend_fields["residual_model"], "none")
        self.assertIn("global_piecewise_registration_blend", piecewise_blend_fields["regularization_model"])
        self.assertIn("piecewise_blend_alpha_grid", piecewise_blend_fields["regularization_model"])
        product_ray_fields = module.method_contract_fields("disparity_product_ray_scale_sim3")
        self.assertEqual(
            product_ray_fields["registration_model"],
            "disparity_product_ray_scale_then_sim3_estimated_to_truth",
        )
        self.assertEqual(product_ray_fields["residual_model"], "calibrated_disparity_forward_product_ray_scale")
        self.assertEqual(product_ray_fields["runtime_profile_family"], "estimated_to_truth_sim3")
        self.assertIn("stereo_depth_product", product_ray_fields["regularization_model"])
        joint_fields = module.method_contract_fields("target_weighted_huber_joint_disparity_ray_rotation_nls_sigma160")
        self.assertEqual(
            joint_fields["registration_model"],
            "joint_disparity_ray_rotation_estimated_to_truth",
        )
        self.assertEqual(joint_fields["residual_model"], "corrected_disparity_linear_plus_rdf_rotation")
        self.assertEqual(joint_fields["runtime_profile_family"], "estimated_to_truth_sim3")
        self.assertIn("nonlinear_least_squares", joint_fields["regularization_model"])
        self.assertIn("huber_m_estimator", joint_fields["regularization_model"])
        kernel_fields = module.method_contract_fields("sim3_kernel_smoothed_residual_raw_forward_sigma120")
        self.assertEqual(kernel_fields["residual_model"], "forward_raw_rdf_forward_lut")
        self.assertIn("kernel_smoothing", kernel_fields["regularization_model"])
        disparity_kernel_fields = module.method_contract_fields(
            "sim3_kernel_smoothed_binned_residual_disparity_sigma040_support5"
        )
        self.assertEqual(disparity_kernel_fields["residual_model"], "forward_disparity_lut")
        self.assertIn("kernel_smoothing", disparity_kernel_fields["regularization_model"])
        self.assertIn("lut_bin_count_grid", disparity_kernel_fields["regularization_model"])
        target_residual_grid_fields = module.method_contract_fields(
            "target_weighted_disparity_quadratic_then_sim3_"
            "target_weighted_quadratic_residual_raw_forward_lut_sigma040_shrink075"
        )
        self.assertIn("target_range_weighting", target_residual_grid_fields["regularization_model"])
        self.assertIn("target_residual_sigma_grid", target_residual_grid_fields["regularization_model"])
        self.assertIn("target_residual_shrinkage", target_residual_grid_fields["regularization_model"])

    def test_runtime_profile_exports_inverse_depth_ray_before_sim3(self) -> None:
        module = load_pipeline_module()
        method = module.FittedMethod(
            name="target_weighted_inverse_depth_linear_then_sim3",
            predict=lambda samples: np.empty((0, 3)),
            parameters={
                "scale": 1.0,
                "rotation_matrix_rdf": np.eye(3).tolist(),
                "rotation_euler_zyx_deg": [0.0, 0.0, 0.0],
                "translation_rdf_m": [0.0, 0.0, 0.0],
                "raw_rdf_ray_inverse_depth_correction_coefficients_high_to_low": [0.98, 1.0e-6],
            },
            model_complexity=10,
        )

        profile = module.runtime_profile_for_method(method, experiment_id="EXP-test")

        self.assertIsNotNone(profile)
        assert profile is not None
        self.assertEqual(profile["mode"], "estimated_to_truth_sim3")
        self.assertEqual(
            profile["raw_rdf_ray_inverse_depth_correction_coefficients_high_to_low"],
            [0.98, 1.0e-6],
        )
        self.assertIn("Inverse-depth ray-scale correction", profile["training_contract"])

    def test_joint_disparity_ray_rotation_fit_recovers_rotation_and_disparity_bias(self) -> None:
        module = load_pipeline_module()
        camera = module.CameraModel()
        samples = []
        rotation_error = module.Rotation.from_euler("zyx", [0.5, -0.2, 0.1], degrees=True)
        disparity_bias_px = 0.4
        for index, forward in enumerate(np.linspace(300.0, 1900.0, 20)):
            truth = np.array(
                [
                    8.0 * np.sin(index * 0.4),
                    4.0 * np.cos(index * 0.3),
                    forward,
                ],
                dtype=float,
            )
            true_disparity = camera.fx_true_px * camera.baseline_m / truth[2]
            measured_disparity = true_disparity + disparity_bias_px
            ray_scaled = truth * (true_disparity / measured_disparity)
            raw = module.apply_rotation_to_rows(ray_scaled[None, :], rotation_error)[0]
            samples.append(
                module.CalibrationSample(
                    sample_id=index,
                    t_s=float(index),
                    split="train",
                    truth_ned_m=np.array([truth[2], truth[0], truth[1]], dtype=float),
                    truth_rdf_m=truth,
                    raw_rdf_m=raw,
                    left_u_px=0.0,
                    left_v_px=0.0,
                    measured_disparity_px=float(measured_disparity),
                    true_disparity_px=float(true_disparity),
                )
            )

        coefficients, rotation, info = module.fit_nonlinear_disparity_ray_rotation(samples)
        corrected = module.predict_disparity_ray_rotation(samples, coefficients, rotation)

        self.assertTrue(info["success"])
        self.assertLess(float(np.sqrt(np.mean((corrected - module.stack(samples, "truth_rdf_m")) ** 2))), 1.0e-4)
        self.assertAlmostEqual(float(coefficients[0]), 1.0, places=5)
        self.assertAlmostEqual(float(coefficients[1]), -disparity_bias_px, places=4)

    def test_runtime_profile_exports_joint_disparity_ray_rotation_method(self) -> None:
        module = load_pipeline_module()
        method = module.FittedMethod(
            name="huber_joint_disparity_ray_rotation_nls",
            predict=lambda samples: np.empty((0, 3)),
            parameters={
                "scale": 1.0,
                "rotation_matrix_rdf": np.eye(3).tolist(),
                "rotation_euler_zyx_deg": [0.0, 0.0, 0.0],
                "translation_rdf_m": [0.0, 0.0, 0.0],
                "raw_rdf_ray_disparity_correction_coefficients_high_to_low": [1.0, 0.4],
                "joint_disparity_ray_rotation_fit": {"loss": "huber"},
            },
            model_complexity=5,
        )

        profile = module.runtime_profile_for_method(method, experiment_id="EXP-test")

        self.assertIsNotNone(profile)
        assert profile is not None
        self.assertEqual(profile["mode"], "estimated_to_truth_sim3")
        self.assertEqual(profile["raw_rdf_ray_disparity_correction_coefficients_high_to_low"], [1.0, 0.4])
        self.assertIn("Measured-disparity ray-scale correction", profile["training_contract"])

    def test_runtime_profile_exports_affine_rdf_method(self) -> None:
        module = load_pipeline_module()
        method = module.FittedMethod(
            name="affine_rdf",
            predict=lambda samples: np.empty((0, 3)),
            parameters={
                "affine_matrix_rdf": np.eye(3).tolist(),
                "translation_rdf_m": [1.0, 2.0, 3.0],
            },
            model_complexity=12,
        )

        profile = module.runtime_profile_for_method(method, experiment_id="EXP-test")

        self.assertIsNotNone(profile)
        assert profile is not None
        self.assertEqual(profile["mode"], "estimated_to_truth_affine_rdf")
        self.assertEqual(profile["method"], "affine_rdf")
        self.assertEqual(profile["translation_rdf_m"], [1.0, 2.0, 3.0])
        self.assertIn("raw_right_m", profile["runtime_inputs"])
        self.assertIn("px4_rtk_truth", profile["forbidden_runtime_inputs"])

    def test_runtime_profile_exports_affine_forward_residual_lut(self) -> None:
        module = load_pipeline_module()
        method = module.FittedMethod(
            name="affine_binned_residual_disparity",
            predict=lambda samples: np.empty((0, 3)),
            parameters={
                "affine_matrix_rdf": np.eye(3).tolist(),
                "translation_rdf_m": [1.0, 2.0, 3.0],
                "forward_residual_disparity_lut": {
                    "x_values_px": [10.0, 20.0],
                    "residual_values_m": [3.0, 5.0],
                },
            },
            model_complexity=14,
        )

        profile = module.runtime_profile_for_method(method, experiment_id="EXP-test")

        self.assertIsNotNone(profile)
        assert profile is not None
        self.assertEqual(profile["mode"], "estimated_to_truth_affine_rdf")
        self.assertEqual(profile["method"], "affine_binned_residual_disparity")
        self.assertEqual(profile["forward_residual_disparity_lut"]["residual_values_m"], [3.0, 5.0])

    def test_runtime_profile_exports_affine_rdf_residual_lut(self) -> None:
        module = load_pipeline_module()
        method = module.FittedMethod(
            name="affine_binned_rdf_residual_disparity",
            predict=lambda samples: np.empty((0, 3)),
            parameters={
                "affine_matrix_rdf": np.eye(3).tolist(),
                "translation_rdf_m": [1.0, 2.0, 3.0],
                "rdf_residual_disparity_lut": {
                    "right": {"x_values_px": [10.0, 20.0], "residual_values_m": [1.0, 2.0]},
                    "down": {"x_values_px": [10.0, 20.0], "residual_values_m": [-1.0, -2.0]},
                    "forward": {"x_values_px": [10.0, 20.0], "residual_values_m": [3.0, 5.0]},
                },
            },
            model_complexity=18,
        )

        profile = module.runtime_profile_for_method(method, experiment_id="EXP-test")

        self.assertIsNotNone(profile)
        assert profile is not None
        self.assertEqual(profile["mode"], "estimated_to_truth_affine_rdf")
        self.assertEqual(profile["method"], "affine_binned_rdf_residual_disparity")
        self.assertEqual(profile["rdf_residual_disparity_lut"]["forward"]["residual_values_m"], [3.0, 5.0])

    def test_runtime_profile_exports_forward_residual_lut(self) -> None:
        module = load_pipeline_module()
        method = module.FittedMethod(
            name="sim3_binned_residual_disparity",
            predict=lambda samples: np.empty((0, 3)),
            parameters={
                "sim3_scale": 1.0,
                "sim3_rotation_matrix_rdf": np.eye(3).tolist(),
                "sim3_rotation_euler_zyx_deg": [0.0, 0.0, 0.0],
                "sim3_translation_rdf_m": [0.0, 0.0, 0.0],
                "forward_residual_disparity_lut": {
                    "x_values_px": [10.0, 20.0],
                    "residual_values_m": [3.0, 5.0],
                },
            },
            model_complexity=10,
        )

        profile = module.runtime_profile_for_method(method, experiment_id="EXP-test")

        self.assertIsNotNone(profile)
        assert profile is not None
        self.assertEqual(profile["mode"], "estimated_to_truth_sim3")
        self.assertEqual(profile["method"], "sim3_binned_residual_disparity")
        self.assertEqual(profile["forward_residual_disparity_lut"]["residual_values_m"], [3.0, 5.0])

    def test_runtime_profile_exports_forward_residual_raw_forward_lut(self) -> None:
        module = load_pipeline_module()
        method = module.FittedMethod(
            name="sim3_binned_residual_raw_forward",
            predict=lambda samples: np.empty((0, 3)),
            parameters={
                "sim3_scale": 1.0,
                "sim3_rotation_matrix_rdf": np.eye(3).tolist(),
                "sim3_rotation_euler_zyx_deg": [0.0, 0.0, 0.0],
                "sim3_translation_rdf_m": [0.0, 0.0, 0.0],
                "forward_residual_raw_forward_lut": {
                    "x_values_m": [1000.0, 2000.0],
                    "residual_values_m": [3.0, 5.0],
                },
            },
            model_complexity=10,
        )

        profile = module.runtime_profile_for_method(method, experiment_id="EXP-test")

        self.assertIsNotNone(profile)
        assert profile is not None
        self.assertEqual(profile["mode"], "estimated_to_truth_sim3")
        self.assertEqual(profile["method"], "sim3_binned_residual_raw_forward")
        self.assertEqual(profile["forward_residual_raw_forward_lut"]["x_values_m"], [1000.0, 2000.0])
        self.assertEqual(profile["forward_residual_raw_forward_lut"]["residual_values_m"], [3.0, 5.0])

    def test_runtime_profile_exports_blended_forward_residual_luts(self) -> None:
        module = load_pipeline_module()
        method = module.FittedMethod(
            name="sim3_blended_binned_residual_disparity_raw_forward",
            predict=lambda samples: np.empty((0, 3)),
            parameters={
                "sim3_scale": 1.0,
                "sim3_rotation_matrix_rdf": np.eye(3).tolist(),
                "sim3_rotation_euler_zyx_deg": [0.0, 0.0, 0.0],
                "sim3_translation_rdf_m": [0.0, 0.0, 0.0],
                "forward_residual_disparity_lut": {
                    "x_values_px": [10.0, 20.0],
                    "residual_values_m": [1.5, 2.5],
                },
                "forward_residual_raw_forward_lut": {
                    "x_values_m": [1000.0, 2000.0],
                    "residual_values_m": [1.5, 2.5],
                },
            },
            model_complexity=14,
        )

        profile = module.runtime_profile_for_method(method, experiment_id="EXP-test")

        self.assertIsNotNone(profile)
        assert profile is not None
        self.assertEqual(profile["mode"], "estimated_to_truth_sim3")
        self.assertEqual(profile["method"], "sim3_blended_binned_residual_disparity_raw_forward")
        self.assertEqual(profile["forward_residual_disparity_lut"]["x_values_px"], [10.0, 20.0])
        self.assertEqual(profile["forward_residual_raw_forward_lut"]["x_values_m"], [1000.0, 2000.0])

    def test_runtime_profile_exports_forward_residual_agreement_gate(self) -> None:
        module = load_pipeline_module()
        method = module.FittedMethod(
            name="sim3_agreement_gated_binned_residual_disparity_raw_forward",
            predict=lambda samples: np.empty((0, 3)),
            parameters={
                "sim3_scale": 1.0,
                "sim3_rotation_matrix_rdf": np.eye(3).tolist(),
                "sim3_rotation_euler_zyx_deg": [0.0, 0.0, 0.0],
                "sim3_translation_rdf_m": [0.0, 0.0, 0.0],
                "forward_residual_disparity_lut": {
                    "x_values_px": [10.0, 20.0],
                    "residual_values_m": [1.5, 2.5],
                },
                "forward_residual_raw_forward_lut": {
                    "x_values_m": [1000.0, 2000.0],
                    "residual_values_m": [1.5, 2.5],
                },
                "forward_residual_agreement_gate": {
                    "max_abs_difference_m": 0.75,
                    "alpha_disagree": 0.0,
                },
            },
            model_complexity=16,
        )

        profile = module.runtime_profile_for_method(method, experiment_id="EXP-test")

        self.assertIsNotNone(profile)
        assert profile is not None
        self.assertEqual(profile["method"], "sim3_agreement_gated_binned_residual_disparity_raw_forward")
        self.assertEqual(profile["forward_residual_agreement_gate"]["max_abs_difference_m"], 0.75)
        self.assertEqual(profile["forward_residual_agreement_gate"]["alpha_disagree"], 0.0)

    def test_runtime_profile_exports_rdf_residual_lut(self) -> None:
        module = load_pipeline_module()
        method = module.FittedMethod(
            name="sim3_binned_rdf_residual_disparity",
            predict=lambda samples: np.empty((0, 3)),
            parameters={
                "sim3_scale": 1.0,
                "sim3_rotation_matrix_rdf": np.eye(3).tolist(),
                "sim3_rotation_euler_zyx_deg": [0.0, 0.0, 0.0],
                "sim3_translation_rdf_m": [0.0, 0.0, 0.0],
                "rdf_residual_disparity_lut": {
                    "right": {"x_values_px": [10.0, 20.0], "residual_values_m": [1.0, 2.0]},
                    "down": {"x_values_px": [10.0, 20.0], "residual_values_m": [-1.0, -2.0]},
                    "forward": {"x_values_px": [10.0, 20.0], "residual_values_m": [3.0, 5.0]},
                },
            },
            model_complexity=20,
        )

        profile = module.runtime_profile_for_method(method, experiment_id="EXP-test")

        self.assertIsNotNone(profile)
        assert profile is not None
        self.assertEqual(profile["mode"], "estimated_to_truth_sim3")
        self.assertEqual(profile["method"], "sim3_binned_rdf_residual_disparity")
        self.assertEqual(profile["rdf_residual_disparity_lut"]["right"]["residual_values_m"], [1.0, 2.0])

    def test_ridge_polyfit_returns_original_disparity_basis_coefficients(self) -> None:
        module = load_pipeline_module()
        disparity = np.array([10.0, 20.0, 30.0, 40.0, 50.0], dtype=float)
        residual = 0.01 * disparity**2 - 0.4 * disparity + 2.0

        coefficients = module.fit_ridge_polyfit(disparity, residual, degree=2, alpha=0.0)

        np.testing.assert_allclose(np.polyval(coefficients, disparity), residual, atol=1.0e-9)

    def test_weighted_polyfit_prioritizes_high_weight_samples(self) -> None:
        module = load_pipeline_module()
        disparity = np.array([10.0, 20.0, 30.0, 40.0], dtype=float)
        residual = np.array([0.0, 0.0, 10.0, 10.0], dtype=float)
        weights = np.array([1.0, 1.0, 100.0, 100.0], dtype=float)

        weighted_coeff = module.fit_weighted_polyfit(disparity, residual, degree=0, weights=weights)
        unweighted_coeff = np.polyfit(disparity, residual, deg=0)

        self.assertGreater(float(weighted_coeff[-1]), float(unweighted_coeff[-1]))

    def test_pareto_frontier_marks_nondominated_methods(self) -> None:
        module = load_pipeline_module()
        rows = [
            {
                "method": "raw",
                "validation_range_rmse_m": 5.0,
                "validation_position_rmse_m": 6.0,
                "validation_right_rmse_m": 5.0,
                "validation_down_rmse_m": 5.0,
                "validation_forward_rmse_m": 5.0,
                "validation_target_abs_mean_error_m": 5.0,
                "validation_target_rmse_m": 5.5,
                "validation_target_max_abs_error_m": 6.0,
                "validation_target_window_count": 1,
                "generalization_gap_range_rmse_m": 0.0,
                "model_complexity": 0,
            },
            {
                "method": "balanced",
                "validation_range_rmse_m": 1.0,
                "validation_position_rmse_m": 1.2,
                "validation_right_rmse_m": 0.7,
                "validation_down_rmse_m": 0.8,
                "validation_forward_rmse_m": 1.0,
                "validation_target_abs_mean_error_m": 1.0,
                "validation_target_rmse_m": 1.3,
                "validation_target_max_abs_error_m": 1.8,
                "validation_target_window_count": 1,
                "generalization_gap_range_rmse_m": 0.1,
                "model_complexity": 7,
            },
            {
                "method": "target_specialist",
                "validation_range_rmse_m": 1.5,
                "validation_position_rmse_m": 1.6,
                "validation_right_rmse_m": 0.8,
                "validation_down_rmse_m": 0.9,
                "validation_forward_rmse_m": 1.4,
                "validation_target_abs_mean_error_m": 0.2,
                "validation_target_rmse_m": 0.8,
                "validation_target_max_abs_error_m": 0.7,
                "validation_target_window_count": 1,
                "generalization_gap_range_rmse_m": 0.2,
                "model_complexity": 8,
            },
            {
                "method": "dominated",
                "validation_range_rmse_m": 1.6,
                "validation_position_rmse_m": 1.7,
                "validation_target_abs_mean_error_m": 1.1,
                "validation_target_rmse_m": 1.4,
                "generalization_gap_range_rmse_m": 0.3,
                "model_complexity": 9,
            },
        ]

        pareto_rows = module.pareto_frontier_rows(rows, recommendation="balanced")
        by_method = {row["method"]: row for row in pareto_rows}

        self.assertTrue(by_method["balanced"]["is_pareto_frontier"])
        self.assertTrue(by_method["balanced"]["is_recommendation"])
        self.assertTrue(by_method["target_specialist"]["is_pareto_frontier"])
        self.assertFalse(by_method["dominated"]["is_pareto_frontier"])

    def test_policy_recommendations_capture_different_operating_priorities(self) -> None:
        module = load_pipeline_module()
        rows = [
            {
                "method": "raw",
                "validation_range_rmse_m": 5.0,
                "validation_position_rmse_m": 6.0,
                "validation_right_rmse_m": 5.0,
                "validation_down_rmse_m": 5.0,
                "validation_forward_rmse_m": 5.0,
                "validation_target_abs_mean_error_m": 5.0,
                "validation_target_rmse_m": 5.5,
                "validation_target_max_abs_error_m": 6.0,
                "validation_target_window_count": 1,
                "generalization_gap_range_rmse_m": 0.0,
                "model_complexity": 0,
            },
            {
                "method": "balanced",
                "validation_range_rmse_m": 1.0,
                "validation_position_rmse_m": 1.2,
                "validation_right_rmse_m": 0.7,
                "validation_down_rmse_m": 0.8,
                "validation_forward_rmse_m": 1.0,
                "validation_target_abs_mean_error_m": 1.0,
                "validation_target_rmse_m": 1.3,
                "validation_target_max_abs_error_m": 1.8,
                "validation_target_window_count": 1,
                "generalization_gap_range_rmse_m": 0.1,
                "model_complexity": 7,
            },
            {
                "method": "target_specialist",
                "validation_range_rmse_m": 1.5,
                "validation_position_rmse_m": 1.6,
                "validation_right_rmse_m": 0.8,
                "validation_down_rmse_m": 0.9,
                "validation_forward_rmse_m": 1.4,
                "validation_target_abs_mean_error_m": 0.2,
                "validation_target_rmse_m": 0.8,
                "validation_target_max_abs_error_m": 0.7,
                "validation_target_window_count": 1,
                "generalization_gap_range_rmse_m": 0.2,
                "model_complexity": 8,
            },
            {
                "method": "simple",
                "validation_range_rmse_m": 2.0,
                "validation_position_rmse_m": 2.1,
                "validation_right_rmse_m": 1.2,
                "validation_down_rmse_m": 1.4,
                "validation_forward_rmse_m": 1.7,
                "validation_target_abs_mean_error_m": 2.0,
                "validation_target_rmse_m": 2.2,
                "validation_target_max_abs_error_m": 1.1,
                "validation_target_window_count": 1,
                "generalization_gap_range_rmse_m": 0.1,
                "model_complexity": 2,
            },
        ]

        policy_rows = module.policy_recommendation_rows(rows, final_recommendation="balanced")
        by_policy = {row["policy"]: row for row in policy_rows}

        self.assertEqual(by_policy["balanced"]["method"], "balanced")
        self.assertEqual(by_policy["axis_balanced"]["method"], "balanced")
        self.assertEqual(by_policy["target_specialist"]["method"], "target_specialist")
        self.assertEqual(by_policy["target_max_stable"]["method"], "target_specialist")
        self.assertEqual(by_policy["target_max_stable"]["validation_target_max_abs_error_m"], 0.7)
        self.assertEqual(by_policy["lowest_complexity_pass"]["method"], "simple")
        self.assertEqual(by_policy["lowest_validation_rmse"]["method"], "balanced")

    def test_policy_recommendations_include_final_recommendation_row_when_needed(self) -> None:
        module = load_pipeline_module()
        rows = [
            {
                "method": "raw",
                "validation_range_rmse_m": 5.0,
                "validation_position_rmse_m": 6.0,
                "validation_right_rmse_m": 5.0,
                "validation_down_rmse_m": 5.0,
                "validation_forward_rmse_m": 5.0,
                "validation_target_abs_mean_error_m": 5.0,
                "validation_target_rmse_m": 5.5,
                "validation_target_max_abs_error_m": 6.0,
                "validation_target_window_count": 1,
                "generalization_gap_range_rmse_m": 0.0,
                "model_complexity": 0,
            },
            {
                "method": "balanced",
                "validation_range_rmse_m": 1.0,
                "validation_position_rmse_m": 1.2,
                "validation_right_rmse_m": 0.7,
                "validation_down_rmse_m": 0.8,
                "validation_forward_rmse_m": 1.0,
                "validation_target_abs_mean_error_m": 1.0,
                "validation_target_rmse_m": 1.3,
                "validation_target_max_abs_error_m": 1.8,
                "validation_target_window_count": 1,
                "generalization_gap_range_rmse_m": 0.1,
                "model_complexity": 7,
            },
            {
                "method": "axis_safe_final",
                "validation_range_rmse_m": 1.4,
                "validation_position_rmse_m": 4.2,
                "validation_right_rmse_m": 4.0,
                "validation_down_rmse_m": 4.0,
                "validation_forward_rmse_m": 1.2,
                "validation_target_abs_mean_error_m": 1.2,
                "validation_target_rmse_m": 1.4,
                "validation_target_max_abs_error_m": 2.0,
                "validation_target_window_count": 1,
                "generalization_gap_range_rmse_m": 0.0,
                "model_complexity": 8,
            },
        ]

        policy_rows = module.policy_recommendation_rows(rows, final_recommendation="axis_safe_final")
        by_policy = {row["policy"]: row for row in policy_rows}

        self.assertEqual(by_policy["final_recommendation"]["method"], "axis_safe_final")
        self.assertTrue(by_policy["final_recommendation"]["is_final_recommendation"])

    def test_csv_source_accepts_recorded_style_gt_alias_and_center_disparity(self) -> None:
        module = load_pipeline_module()
        row = (
            "trial,trial_t_s,gt_forward_m,gt_right_m,gt_down_m,"
            "estimated_forward_m,estimated_right_m,estimated_down_m,"
            "left_cx_px,left_cy_px,right_cx_px,right_cy_px\n"
            "1,3.0,1000.0,8.0,4.0,1002.0,7.5,4.2,700.0,390.0,500.0,389.0\n"
        )
        with tempfile.TemporaryDirectory() as tmpdir:
            path = Path(tmpdir) / "calibration_samples.csv"
            path.write_text(row, encoding="utf-8")
            samples = module.CsvCalibrationDataSource(path, camera=module.CameraModel()).load_samples()

        self.assertEqual(len(samples), 1)
        np.testing.assert_allclose(samples[0].truth_rdf_m, np.array([8.0, 4.0, 1000.0]))
        self.assertTrue(np.all(np.isnan(samples[0].truth_ned_m)))
        np.testing.assert_allclose(samples[0].raw_rdf_m, np.array([7.5, 4.2, 1002.0]))
        self.assertEqual(samples[0].measured_disparity_px, 200.0)

    def test_csv_source_rejects_ned_only_truth_without_explicit_rdf(self) -> None:
        module = load_pipeline_module()
        row = (
            "truth_north_m,truth_east_m,truth_down_ned_m,"
            "estimated_forward_m,estimated_right_m,estimated_down_m,"
            "measured_disparity_px\n"
            "1000.0,8.0,4.0,1002.0,7.5,4.2,200.0\n"
        )
        with tempfile.TemporaryDirectory() as tmpdir:
            path = Path(tmpdir) / "ned_only.csv"
            path.write_text(row, encoding="utf-8")
            with self.assertRaisesRegex(ValueError, "no usable calibration rows"):
                module.CsvCalibrationDataSource(path, camera=module.CameraModel()).load_samples()

    def test_csv_source_preserves_truth_transform_metadata_in_sample_rows(self) -> None:
        module = load_pipeline_module()
        row = (
            "truth_right_m,truth_down_m,truth_forward_m,"
            "truth_source_frame,truth_source_axis_order,truth_source_fields,"
            "truth_rdf_transform_contract,truth_source_origin_m,"
            "truth_source_to_stereo_rdf_translation_m,truth_source_to_stereo_rdf_rotation_matrix,"
            "estimated_right_m,estimated_down_m,estimated_forward_m,measured_disparity_px\n"
            "8.0,4.0,1000.0,"
            "rtk_enu,\"east,north,up\",\"rtk_e,rtk_n,rtk_u\","
            "T_source_to_stereo_rdf: explicit transform,\"1.0,2.0,3.0\","
            "\"0.0,0.0,0.0\",\"[[1,0,0],[0,1,0],[0,0,1]]\","
            "7.5,4.2,1002.0,200.0\n"
        )
        with tempfile.TemporaryDirectory() as tmpdir:
            path = Path(tmpdir) / "converted.csv"
            path.write_text(row, encoding="utf-8")
            samples = module.CsvCalibrationDataSource(path, camera=module.CameraModel()).load_samples()

        sample_rows = module.rows_for_samples(samples)
        self.assertEqual(sample_rows[0]["truth_source_frame"], "rtk_enu")
        self.assertEqual(sample_rows[0]["truth_source_axis_order"], "east,north,up")
        self.assertIn("T_source_to_stereo_rdf", sample_rows[0]["truth_rdf_transform_contract"])
        self.assertNotIn("explicit_transform: truth_rdf = R_ned_to_stereo_rdf", sample_rows[0]["truth_rdf_transform_contract"])

    def test_csv_source_accepts_ros2_calibration_recorder_contract(self) -> None:
        module = load_pipeline_module()
        row = (
            "time_sec,ros_time_sec,source_contract,ned_axis_contract,rdf_axis_contract,"
            "public_topic_contract,truth_north_m,truth_east_m,truth_down_ned_m,"
            "truth_right_m,truth_down_m,truth_forward_m,truth_range_m,"
            "raw_forward_m,raw_right_m,raw_down_m,raw_range_m,"
            "estimated_forward_m,estimated_right_m,estimated_down_m,estimated_range_m,"
            "measured_disparity_px,left_cx_px,left_cy_px,right_cx_px,right_cy_px\n"
            "0.0,0.0,px4_ned_truth+stereodistance_frd_estimate,"
            "px4_local_position_ned:north/east/down,"
            "stereo_center_opencv_optical:RDF(right/down/forward),"
            "stereodistance_position_topics:FRD_range(forward/right/down/range),"
            "1000.0,8.0,4.0,8.0,4.0,1000.0,1000.04,"
            "1002.0,7.5,4.2,1002.04,1002.0,7.5,4.2,1002.04,"
            "200.0,700.0,390.0,500.0,389.0\n"
        )
        with tempfile.TemporaryDirectory() as tmpdir:
            path = Path(tmpdir) / "telemetry.csv"
            path.write_text(row, encoding="utf-8")
            samples = module.CsvCalibrationDataSource(path, camera=module.CameraModel()).load_samples()

        self.assertEqual(len(samples), 1)
        np.testing.assert_allclose(samples[0].truth_ned_m, np.array([1000.0, 8.0, 4.0]))
        np.testing.assert_allclose(samples[0].truth_rdf_m, np.array([8.0, 4.0, 1000.0]))
        np.testing.assert_allclose(samples[0].raw_rdf_m, np.array([7.5, 4.2, 1002.0]))
        self.assertEqual(samples[0].measured_disparity_px, 200.0)


if __name__ == "__main__":
    unittest.main()
