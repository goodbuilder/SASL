from __future__ import annotations

import csv
import importlib.util
import json
import sys
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch


ROOT_DIR = Path(__file__).resolve().parents[1]


def load_guard_module():
    spec = importlib.util.spec_from_file_location(
        "select_stereodistance_profile_by_runtime_measurement_guard_under_test",
        ROOT_DIR / "scripts" / "select_stereodistance_profile_by_runtime_measurement_guard.py",
    )
    if spec is None or spec.loader is None:
        raise RuntimeError("cannot load select_stereodistance_profile_by_runtime_measurement_guard.py")
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def sample_row(forward: float, disparity: float, truth_offset: float = 0.0) -> dict[str, str]:
    return {
        "raw_right_m": "1.0",
        "raw_down_m": "2.0",
        "raw_forward_m": f"{forward}",
        "raw_range_m": f"{forward}",
        "measured_disparity_px": f"{disparity}",
        "left_u_px": "640.0",
        "left_v_px": "360.0",
        "truth_right_m": f"{truth_offset + 100.0}",
        "truth_down_m": f"{truth_offset + 200.0}",
        "truth_forward_m": f"{truth_offset + 300.0}",
        "truth_range_m": f"{truth_offset + 400.0}",
        "true_disparity_px": f"{truth_offset + 500.0}",
        "raw_disparity_residual_px": f"{truth_offset + 600.0}",
    }


def sample_row_with_product(forward: float, product_px_m: float = 20000.0) -> dict[str, str]:
    return sample_row(forward, product_px_m / forward)


class SelectStereoDistanceProfileByRuntimeMeasurementGuardTests(unittest.TestCase):
    def test_runtime_measurement_features_ignore_truth_fields(self) -> None:
        module = load_guard_module()
        base = [sample_row(1000.0, 20.0, truth_offset=0.0), sample_row(1100.0, 18.0, truth_offset=0.0)]
        changed_truth = [
            sample_row(1000.0, 20.0, truth_offset=10000.0),
            sample_row(1100.0, 18.0, truth_offset=-10000.0),
        ]

        self.assertEqual(module.runtime_measurement_features(base), module.runtime_measurement_features(changed_truth))

    def test_runtime_measurement_features_include_nominal_depth_scale_product(self) -> None:
        module = load_guard_module()
        rows = [sample_row(1000.0, 20.0), sample_row(1000.0, 20.0)]

        features = module.runtime_measurement_features(rows)
        expected_ratio = 20000.0 / module.DEFAULT_NOMINAL_DISPARITY_FORWARD_PRODUCT_PX_M

        self.assertAlmostEqual(features["disparity_forward_product_mean_px_m"], 20000.0)
        self.assertAlmostEqual(features["disparity_forward_product_ratio_to_nominal"], expected_ratio)
        self.assertAlmostEqual(
            features["disparity_forward_product_abs_fractional_error"],
            abs(expected_ratio - 1.0),
        )
        self.assertAlmostEqual(features["disparity_forward_product_ratio_median_to_nominal"], expected_ratio)
        self.assertAlmostEqual(features["disparity_forward_product_ratio_p10_to_nominal"], expected_ratio)
        self.assertAlmostEqual(features["disparity_forward_product_ratio_iqr"], 0.0)
        self.assertAlmostEqual(
            features["disparity_forward_product_abs_fractional_error_median"],
            abs(expected_ratio - 1.0),
        )

    def test_select_source_by_runtime_measurement_uses_nearest_raw_centroid(self) -> None:
        module = load_guard_module()
        near_rows = [sample_row(1000.0, 20.0), sample_row(1030.0, 18.0)]
        library = [
            {
                "scenario": "near",
                "seed": "seed_1",
                "csv_path": "near.csv",
                "features": module.runtime_measurement_features([sample_row(1000.0, 20.0), sample_row(1030.0, 18.0)]),
                "feature_vector": module.feature_vector(
                    module.runtime_measurement_features([sample_row(1000.0, 20.0), sample_row(1030.0, 18.0)])
                ),
            },
            {
                "scenario": "far",
                "seed": "seed_1",
                "csv_path": "far.csv",
                "features": module.runtime_measurement_features([sample_row(2200.0, 9.0), sample_row(2300.0, 8.0)]),
                "feature_vector": module.feature_vector(
                    module.runtime_measurement_features([sample_row(2200.0, 9.0), sample_row(2300.0, 8.0)])
                ),
            },
        ]

        selection = module.select_source_by_runtime_measurement(rows=near_rows, library=library)

        self.assertEqual(selection["selected_source_scenario"], "near")

    def test_robust_family_fallback_uses_disparity_profile_when_bbox_profile_is_unavailable(self) -> None:
        module = load_guard_module()
        rows = [sample_row_with_product(1000.0), sample_row_with_product(1030.0)]
        library = []
        for scenario, forward in (
            ("nominal", 1000.0),
            ("disparity_noise_stress", 2200.0),
        ):
            features = module.runtime_measurement_features(
                [sample_row_with_product(forward), sample_row_with_product(forward + 5.0)]
            )
            library.append(
                {
                    "scenario": scenario,
                    "seed": "seed_1",
                    "csv_path": f"{scenario}.csv",
                    "features": features,
                    "feature_vector": module.feature_vector(features),
                }
            )

        selection = module.select_source_by_runtime_measurement(
            rows=rows,
            library=library,
            available_source_scenarios=["nominal", "disparity_noise_stress"],
            selection_policy="robust_family_fallback",
        )

        self.assertEqual(selection["preliminary_source_scenario"], "nominal")
        self.assertEqual(selection["selected_source_scenario"], "disparity_noise_stress")
        self.assertEqual(
            selection["fallback_reason"],
            "bbox_profile_unavailable_low_range_family_uses_disparity_noise_profile",
        )

    def test_summarize_audit_reports_validation_only_pass_count_separately(self) -> None:
        module = load_guard_module()
        rows = [
            {
                "pass_runtime_measurement_guard_replay": False,
                "validation_only_pass_runtime_measurement_guard_replay": True,
                "validation_range_rmse_corrected_m": 1.0,
                "target_abs_mean_error_corrected_m": 2.0,
                "target_max_abs_error_corrected_m": 3.0,
            },
            {
                "pass_runtime_measurement_guard_replay": False,
                "validation_only_pass_runtime_measurement_guard_replay": False,
                "validation_range_rmse_corrected_m": 1.5,
                "target_abs_mean_error_corrected_m": 2.5,
                "target_max_abs_error_corrected_m": 4.0,
            },
        ]

        summary = module.summarize_audit(rows)

        self.assertEqual(summary["pass_count"], 0)
        self.assertEqual(summary["validation_only_pass_count"], 1)
        self.assertFalse(summary["pass_all"])
        self.assertFalse(summary["validation_only_pass_all"])

    def test_transfer_risk_fallback_selects_lowest_risk_confusion_cluster_source(self) -> None:
        module = load_guard_module()
        rows = [sample_row_with_product(1000.0), sample_row_with_product(1030.0)]
        library = []
        for scenario, forward in (
            ("nominal", 1000.0),
            ("yaw_pitch_stress", 1010.0),
            ("bbox_outlier_stress", 1020.0),
            ("disparity_noise_stress", 2200.0),
        ):
            features = module.runtime_measurement_features(
                [sample_row_with_product(forward), sample_row_with_product(forward + 5.0)]
            )
            library.append(
                {
                    "scenario": scenario,
                    "seed": "seed_1",
                    "csv_path": f"{scenario}.csv",
                    "features": features,
                    "feature_vector": module.feature_vector(features),
                }
            )
        matrix = {}
        cluster = ("nominal", "yaw_pitch_stress", "bbox_outlier_stress")
        for source in cluster:
            for target in cluster:
                matrix[(source, target)] = {
                    "source_scenario": source,
                    "target_scenario": target,
                    "pass_all": "True",
                    "mean_validation_range_rmse_corrected_m": "1.0",
                    "max_target_abs_mean_error_corrected_m": "2.0",
                    "min_validation_range_rmse_improvement_m": "1.0",
                }
        for target in cluster:
            matrix[("bbox_outlier_stress", target)]["mean_validation_range_rmse_corrected_m"] = "0.7"
        for source in ("nominal", "yaw_pitch_stress", "bbox_outlier_stress", "disparity_noise_stress"):
            matrix[(source, "disparity_noise_stress")] = {
                "source_scenario": source,
                "target_scenario": "disparity_noise_stress",
                "pass_all": "False" if source != "disparity_noise_stress" else "True",
                "mean_validation_range_rmse_corrected_m": "4.0" if source != "disparity_noise_stress" else "0.8",
                "max_target_abs_mean_error_corrected_m": "5.0" if source != "disparity_noise_stress" else "1.0",
                "min_validation_range_rmse_improvement_m": "-1.0" if source != "disparity_noise_stress" else "1.0",
            }
            matrix[("disparity_noise_stress", source)] = matrix[(source, "disparity_noise_stress")]

        selection = module.select_source_by_runtime_measurement(
            rows=rows,
            library=library,
            available_source_scenarios=["nominal", "yaw_pitch_stress", "bbox_outlier_stress", "disparity_noise_stress"],
            selection_policy="transfer_risk_fallback",
            transfer_matrix=matrix,
            transfer_low_risk_rmse_limit_m=1.3,
            transfer_low_risk_target_limit_m=2.5,
            transfer_risk_target_weight=0.10,
        )

        self.assertEqual(selection["selected_source_scenario"], "bbox_outlier_stress")
        self.assertIn("nominal", selection["transfer_confusion_cluster"])
        self.assertIn("bbox_outlier_stress", selection["transfer_confusion_cluster"])
        self.assertEqual(selection["fallback_reason"], "transfer_matrix_low_risk_cluster_prefers_lower_risk_source")

    def test_feature_vote_transfer_risk_reports_raw_feature_subset_votes(self) -> None:
        module = load_guard_module()
        rows = [sample_row(1000.0, 20.0), sample_row(1030.0, 18.0)]
        library = []
        for scenario, forward, disparity, right, down, u, v in (
            ("range_like", 1000.0, 8.0, 20.0, 20.0, 700.0, 400.0),
            ("disparity_like", 2030.0, 10.0, 25.0, 25.0, 710.0, 410.0),
            ("bearing_like", 2300.0, 7.0, 1.0, 2.0, 640.0, 360.0),
        ):
            training_rows = []
            for offset in (0.0, 5.0):
                row = sample_row(forward + offset, disparity)
                row["raw_right_m"] = str(right)
                row["raw_down_m"] = str(down)
                row["left_u_px"] = str(u)
                row["left_v_px"] = str(v)
                training_rows.append(row)
            features = module.runtime_measurement_features(training_rows)
            library.append(
                {
                    "scenario": scenario,
                    "seed": "seed_1",
                    "csv_path": f"{scenario}.csv",
                    "features": features,
                    "feature_vector": module.feature_vector(features),
                }
            )
        matrix = {}
        for source in ("range_like", "disparity_like", "bearing_like"):
            for target in ("range_like", "disparity_like", "bearing_like"):
                matrix[(source, target)] = {
                    "source_scenario": source,
                    "target_scenario": target,
                    "pass_all": "True",
                    "mean_validation_range_rmse_corrected_m": "0.9" if source == "bearing_like" else "1.1",
                    "max_target_abs_mean_error_corrected_m": "1.5" if source == "bearing_like" else "2.0",
                    "min_validation_range_rmse_improvement_m": "1.0",
                }

        selection = module.select_source_by_runtime_measurement(
            rows=rows,
            library=library,
            available_source_scenarios=["range_like", "disparity_like", "bearing_like"],
            selection_policy="feature_vote_transfer_risk",
            transfer_matrix=matrix,
            transfer_low_risk_rmse_limit_m=1.3,
            transfer_low_risk_target_limit_m=2.5,
            transfer_risk_target_weight=0.10,
        )

        vote_winners = {row["feature_subset"]: row["winner_scenario"] for row in selection["feature_vote_rows"]}
        self.assertEqual(set(vote_winners), {"range_geometry", "disparity_depth", "bearing_image"})
        self.assertEqual(vote_winners["range_geometry"], "range_like")
        self.assertEqual(vote_winners["bearing_image"], "bearing_like")
        self.assertEqual(selection["selected_source_scenario"], "bearing_like")
        self.assertEqual(selection["fallback_reason"], "feature_vote_transfer_cluster_prefers_lower_risk_source")

    def test_intrinsic_scale_gap_transfer_risk_prefers_scale_profile_when_product_ratio_is_high(self) -> None:
        module = load_guard_module()
        high_product = module.DEFAULT_NOMINAL_DISPARITY_FORWARD_PRODUCT_PX_M * 1.004
        nominal_product = module.DEFAULT_NOMINAL_DISPARITY_FORWARD_PRODUCT_PX_M * 1.0
        rows = [sample_row_with_product(1000.0, high_product), sample_row_with_product(1010.0, high_product)]
        library = []
        for scenario, product in (
            ("nominal", nominal_product),
            ("intrinsic_scale_stress", high_product),
        ):
            features = module.runtime_measurement_features(
                [sample_row_with_product(1000.0, product), sample_row_with_product(1010.0, product)]
            )
            library.append(
                {
                    "scenario": scenario,
                    "seed": "seed_1",
                    "csv_path": f"{scenario}.csv",
                    "features": features,
                    "feature_vector": module.feature_vector(features),
                }
            )

        selection = module.select_source_by_runtime_measurement(
            rows=rows,
            library=library,
            available_source_scenarios=["nominal", "intrinsic_scale_stress"],
            selection_policy="intrinsic_scale_gap_transfer_risk",
            transfer_matrix={("nominal", "nominal"): {"pass_all": "True"}},
            intrinsic_scale_gap_product_ratio_p10_threshold=1.0037,
        )

        self.assertEqual(selection["selected_source_scenario"], "intrinsic_scale_stress")
        self.assertEqual(selection["fallback_reason"], "intrinsic_scale_product_ratio_guard_selects_scale_profile")

    def test_knn_transfer_risk_uses_nearest_raw_measurement_runs(self) -> None:
        module = load_guard_module()
        rows = [sample_row_with_product(1000.0), sample_row_with_product(1010.0)]
        library = []
        for scenario, seed, forward in (
            ("near_a", "seed_1", 1000.0),
            ("near_b", "seed_2", 1015.0),
            ("far", "seed_1", 2200.0),
        ):
            features = module.runtime_measurement_features(
                [sample_row_with_product(forward), sample_row_with_product(forward + 5.0)]
            )
            library.append(
                {
                    "scenario": scenario,
                    "seed": seed,
                    "csv_path": f"{scenario}_{seed}.csv",
                    "features": features,
                    "feature_vector": module.feature_vector(features),
                }
            )
        matrix = {}
        for source in ("near_a", "near_b", "far"):
            for target in ("near_a", "near_b", "far"):
                matrix[(source, target)] = {
                    "source_scenario": source,
                    "target_scenario": target,
                    "pass_all": "True",
                    "mean_validation_range_rmse_corrected_m": "0.7" if source == "near_b" else "1.0",
                    "max_target_abs_mean_error_corrected_m": "1.0",
                    "min_validation_range_rmse_improvement_m": "1.0",
                }

        selection = module.select_source_by_runtime_measurement(
            rows=rows,
            library=library,
            available_source_scenarios=["near_a", "near_b", "far"],
            selection_policy="knn_transfer_risk",
            transfer_matrix=matrix,
            transfer_low_risk_rmse_limit_m=1.3,
            transfer_low_risk_target_limit_m=2.5,
            transfer_risk_target_weight=0.10,
            knn_neighbor_count=2,
        )

        self.assertEqual(selection["preliminary_source_scenario"], "near_a")
        self.assertEqual(selection["selected_source_scenario"], "near_b")
        self.assertEqual(len(selection["knn_neighbor_rows"]), 2)
        self.assertEqual(selection["fallback_reason"], "knn_transfer_cluster_prefers_lower_risk_source")

    def test_centroid_knn_union_transfer_risk_penalizes_cross_domain_knn_candidate(self) -> None:
        module = load_guard_module()
        rows = [sample_row_with_product(2200.0), sample_row_with_product(2210.0)]
        library = []
        for scenario, seed, forward in (
            ("disparity_noise_stress", "seed_1", 2200.0),
            ("bbox_outlier_stress", "seed_1", 2210.0),
            ("nominal", "seed_1", 1000.0),
        ):
            features = module.runtime_measurement_features(
                [sample_row_with_product(forward), sample_row_with_product(forward + 5.0)]
            )
            library.append(
                {
                    "scenario": scenario,
                    "seed": seed,
                    "csv_path": f"{scenario}_{seed}.csv",
                    "features": features,
                    "feature_vector": module.feature_vector(features),
                }
            )
        matrix = {}
        scenarios = ("disparity_noise_stress", "bbox_outlier_stress", "nominal")
        for source in scenarios:
            for target in scenarios:
                same = source == target
                cross_disparity_bbox = {source, target} == {"disparity_noise_stress", "bbox_outlier_stress"}
                matrix[(source, target)] = {
                    "source_scenario": source,
                    "target_scenario": target,
                    "pass_all": "False" if cross_disparity_bbox else "True",
                    "mean_validation_range_rmse_corrected_m": "5.0" if cross_disparity_bbox else ("0.7" if source == "disparity_noise_stress" else "1.0"),
                    "max_target_abs_mean_error_corrected_m": "5.0" if cross_disparity_bbox else "1.0",
                    "min_validation_range_rmse_improvement_m": "-1.0" if cross_disparity_bbox else "1.0",
                }

        selection = module.select_source_by_runtime_measurement(
            rows=rows,
            library=library,
            available_source_scenarios=list(scenarios),
            selection_policy="centroid_knn_union_transfer_risk",
            transfer_matrix=matrix,
            transfer_low_risk_rmse_limit_m=1.3,
            transfer_low_risk_target_limit_m=2.5,
            transfer_risk_target_weight=0.10,
            knn_neighbor_count=2,
        )

        self.assertNotEqual(selection["selected_source_scenario"], "bbox_outlier_stress")
        self.assertIn("bbox_outlier_stress", selection["transfer_confusion_cluster"])

    def test_centroid_knn_veto_transfer_risk_excludes_unsafe_knn_candidate(self) -> None:
        module = load_guard_module()
        rows = [sample_row_with_product(2200.0), sample_row_with_product(2210.0)]
        library = []
        for scenario, seed, forward in (
            ("disparity_noise_stress", "seed_1", 2200.0),
            ("bbox_outlier_stress", "seed_1", 2210.0),
            ("nominal", "seed_1", 1000.0),
        ):
            features = module.runtime_measurement_features(
                [sample_row_with_product(forward), sample_row_with_product(forward + 5.0)]
            )
            library.append(
                {
                    "scenario": scenario,
                    "seed": seed,
                    "csv_path": f"{scenario}_{seed}.csv",
                    "features": features,
                    "feature_vector": module.feature_vector(features),
                }
            )
        matrix = {}
        scenarios = ("disparity_noise_stress", "bbox_outlier_stress", "nominal")
        for source in scenarios:
            for target in scenarios:
                unsafe = {source, target} == {"disparity_noise_stress", "bbox_outlier_stress"}
                matrix[(source, target)] = {
                    "source_scenario": source,
                    "target_scenario": target,
                    "pass_all": "False" if unsafe else "True",
                    "mean_validation_range_rmse_corrected_m": "5.0" if unsafe else "1.0",
                    "max_target_abs_mean_error_corrected_m": "5.0" if unsafe else "1.0",
                    "min_validation_range_rmse_improvement_m": "-1.0" if unsafe else "1.0",
                }

        selection = module.select_source_by_runtime_measurement(
            rows=rows,
            library=library,
            available_source_scenarios=list(scenarios),
            selection_policy="centroid_knn_veto_transfer_risk",
            transfer_matrix=matrix,
            transfer_low_risk_rmse_limit_m=1.3,
            transfer_low_risk_target_limit_m=2.5,
            transfer_risk_target_weight=0.10,
            knn_neighbor_count=2,
        )

        self.assertNotIn("bbox_outlier_stress", selection["transfer_confusion_cluster"])
        self.assertTrue(selection["transfer_score_rows"])

    def test_run_guard_audits_all_sweep_seed_csvs_with_raw_selection_contract(self) -> None:
        module = load_guard_module()
        with tempfile.TemporaryDirectory() as tmpdir:
            root = Path(tmpdir)
            sweep_dir = root / "sweep"
            for scenario, forward, disparity in (("near", 1000.0, 20.0), ("far", 2200.0, 9.0)):
                csv_dir = sweep_dir / scenario / "seed_1"
                csv_dir.mkdir(parents=True)
                csv_dir.joinpath("calibration_samples.csv").write_text(
                    "raw_right_m,raw_down_m,raw_forward_m,raw_range_m,measured_disparity_px,left_u_px,left_v_px,split\n"
                    f"1,2,{forward},{forward},{disparity},640,360,validation\n",
                    encoding="utf-8",
                )
            catalog_dir = sweep_dir / "scenario_profile_catalog"
            catalog_dir.mkdir()
            (catalog_dir / "near_runtime_profile.json").write_text('{"profile":"near"}\n', encoding="utf-8")
            (catalog_dir / "far_runtime_profile.json").write_text('{"profile":"far"}\n', encoding="utf-8")
            catalog_json = catalog_dir / "manifest.json"
            catalog_json.write_text(
                json.dumps(
                    {
                        "mode": "scenario_profile_catalog",
                        "profiles": [
                            {
                                "scenario": "near",
                                "method": "sim3",
                                "catalog_profile_path": "scenario_profile_catalog/near_runtime_profile.json",
                                "profile_exists": True,
                                "pass_cross_seed_replay": True,
                            },
                            {
                                "scenario": "far",
                                "method": "sim3",
                                "catalog_profile_path": "scenario_profile_catalog/far_runtime_profile.json",
                                "profile_exists": True,
                                "pass_cross_seed_replay": True,
                            },
                        ],
                    }
                ),
                encoding="utf-8",
            )

            def fake_replay_candidate(**kwargs):
                scenario = kwargs["scenario"]
                return {
                    "scenario": scenario,
                    "method": "sim3",
                    "profile_path": str(catalog_dir / f"{scenario}_runtime_profile.json"),
                    "pass_replay": True,
                    "pass_reason": "pass",
                    "validation_range_rmse_raw_m": 3.0,
                    "validation_range_rmse_corrected_m": 1.0,
                    "validation_range_rmse_improvement_m": 2.0,
                    "target_window_count": 1,
                    "target_abs_mean_error_corrected_m": 0.5,
                    "target_max_abs_error_corrected_m": 0.7,
                }

            args = type(
                "Args",
                (),
                {
                    "sweep_dir": sweep_dir,
                    "catalog_json": catalog_json,
                    "minimal_set_json": None,
                    "input_csv": None,
                    "output_dir": root / "guard",
                    "output_profile": None,
                    "split": "validation",
                    "target_range_m": 1852.0,
                    "target_window_m": 30.0,
                    "target_error_limit_m": 10.0,
                    "candidate_source_mode": "all_catalog",
                    "selection_policy": "nearest_centroid",
                    "transfer_matrix_csv": None,
                    "transfer_low_risk_rmse_limit_m": 1.3,
                    "transfer_low_risk_target_limit_m": 2.5,
                    "transfer_risk_target_weight": 0.10,
                    "knn_neighbor_count": 5,
                    "audit_all_sweep_seeds": True,
                    "include_target_in_centroids": True,
                    "allow_unvalidated": False,
                },
            )()

            with patch.object(module, "replay_candidate", side_effect=fake_replay_candidate):
                summary = module.run_guard(args)

            self.assertTrue(summary["pass_all"])
            audit_csv = args.output_dir / "runtime_measurement_guard_audit.csv"
            self.assertTrue(audit_csv.is_file())
            text = audit_csv.read_text(encoding="utf-8")
            self.assertIn("selection_input_fields", text)
            self.assertIn("forbidden_selection_fields", text)

    def test_run_guard_can_exclude_target_scenario_from_runtime_library(self) -> None:
        module = load_guard_module()
        with tempfile.TemporaryDirectory() as tmpdir:
            root = Path(tmpdir)
            sweep_dir = root / "sweep"
            for scenario, seed, forward, disparity in (
                ("near", "seed_1", 1000.0, 20.0),
                ("near", "seed_2", 1010.0, 19.5),
                ("far", "seed_1", 2200.0, 9.0),
            ):
                csv_dir = sweep_dir / scenario / seed
                csv_dir.mkdir(parents=True)
                csv_dir.joinpath("calibration_samples.csv").write_text(
                    "raw_right_m,raw_down_m,raw_forward_m,raw_range_m,measured_disparity_px,left_u_px,left_v_px,split\n"
                    f"1,2,{forward},{forward},{disparity},640,360,validation\n",
                    encoding="utf-8",
                )
            catalog_dir = sweep_dir / "scenario_profile_catalog"
            catalog_dir.mkdir()
            (catalog_dir / "near_runtime_profile.json").write_text('{"profile":"near"}\n', encoding="utf-8")
            (catalog_dir / "far_runtime_profile.json").write_text('{"profile":"far"}\n', encoding="utf-8")
            catalog_json = catalog_dir / "manifest.json"
            catalog_json.write_text(
                json.dumps(
                    {
                        "mode": "scenario_profile_catalog",
                        "profiles": [
                            {
                                "scenario": "near",
                                "method": "sim3",
                                "catalog_profile_path": "scenario_profile_catalog/near_runtime_profile.json",
                                "profile_exists": True,
                                "pass_cross_seed_replay": True,
                            },
                            {
                                "scenario": "far",
                                "method": "sim3",
                                "catalog_profile_path": "scenario_profile_catalog/far_runtime_profile.json",
                                "profile_exists": True,
                                "pass_cross_seed_replay": True,
                            },
                        ],
                    }
                ),
                encoding="utf-8",
            )

            def fake_replay_candidate(**kwargs):
                return {
                    "scenario": kwargs["scenario"],
                    "method": "sim3",
                    "profile_path": str(catalog_dir / f"{kwargs['scenario']}_runtime_profile.json"),
                    "pass_replay": True,
                    "pass_reason": "pass",
                    "validation_range_rmse_raw_m": 3.0,
                    "validation_range_rmse_corrected_m": 1.0,
                    "validation_range_rmse_improvement_m": 2.0,
                    "target_window_count": 1,
                    "target_abs_mean_error_corrected_m": 0.5,
                    "target_max_abs_error_corrected_m": 0.7,
                }

            args = type(
                "Args",
                (),
                {
                    "sweep_dir": sweep_dir,
                    "catalog_json": catalog_json,
                    "minimal_set_json": None,
                    "input_csv": None,
                    "output_dir": root / "guard_loso",
                    "output_profile": None,
                    "split": "validation",
                    "target_range_m": 1852.0,
                    "target_window_m": 30.0,
                    "target_error_limit_m": 10.0,
                    "candidate_source_mode": "all_catalog",
                    "selection_policy": "nearest_centroid",
                    "transfer_matrix_csv": None,
                    "transfer_low_risk_rmse_limit_m": 1.3,
                    "transfer_low_risk_target_limit_m": 2.5,
                    "transfer_risk_target_weight": 0.10,
                    "knn_neighbor_count": 5,
                    "audit_all_sweep_seeds": True,
                    "include_target_in_centroids": True,
                    "exclude_target_scenario_from_library": True,
                    "allow_unvalidated": False,
                },
            )()

            with patch.object(module, "replay_candidate", side_effect=fake_replay_candidate):
                summary = module.run_guard(args)

            self.assertTrue(summary["exclude_target_scenario_from_library"])
            with (args.output_dir / "runtime_measurement_guard_audit.csv").open(newline="", encoding="utf-8") as handle:
                audit_rows = list(csv.DictReader(handle))
            near_rows = [row for row in audit_rows if row["target_scenario"] == "near"]
            self.assertEqual(len(near_rows), 2)
            self.assertTrue(all(row["selected_source_scenario"] == "far" for row in near_rows))
            self.assertTrue(all(row["eligible_source_scenarios"] == "far" for row in near_rows))


if __name__ == "__main__":
    unittest.main()
