from __future__ import annotations

import importlib.util
import json
import sys
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch


ROOT_DIR = Path(__file__).resolve().parents[1]


def load_replay_selector_module():
    spec = importlib.util.spec_from_file_location(
        "select_stereodistance_profile_by_calibration_replay_under_test",
        ROOT_DIR / "scripts" / "select_stereodistance_profile_by_calibration_replay.py",
    )
    if spec is None or spec.loader is None:
        raise RuntimeError("cannot load select_stereodistance_profile_by_calibration_replay.py")
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


class SelectStereoDistanceProfileByCalibrationReplayTests(unittest.TestCase):
    def test_select_by_calibration_replay_uses_minimal_set_candidates_and_copies_best_profile(self) -> None:
        module = load_replay_selector_module()
        with tempfile.TemporaryDirectory() as tmpdir:
            root = Path(tmpdir)
            catalog_dir = root / "scenario_profile_catalog"
            catalog_dir.mkdir()
            nominal_profile = catalog_dir / "nominal_runtime_profile.json"
            nominal_profile.write_text('{"profile": "nominal"}\n', encoding="utf-8")
            intrinsic_profile = catalog_dir / "intrinsic_scale_stress_runtime_profile.json"
            intrinsic_profile.write_text('{"profile": "intrinsic"}\n', encoding="utf-8")
            ignored_profile = catalog_dir / "ignored_runtime_profile.json"
            ignored_profile.write_text('{"profile": "ignored"}\n', encoding="utf-8")
            catalog_json = catalog_dir / "manifest.json"
            catalog_json.write_text(
                json.dumps(
                    {
                        "mode": "scenario_profile_catalog",
                        "profiles": [
                            {
                                "scenario": "nominal",
                                "method": "sim3_binned_residual_disparity",
                                "catalog_profile_path": "scenario_profile_catalog/nominal_runtime_profile.json",
                                "profile_exists": True,
                                "pass_cross_seed_replay": True,
                            },
                            {
                                "scenario": "intrinsic_scale_stress",
                                "method": "sim3_binned_residual_disparity",
                                "catalog_profile_path": "scenario_profile_catalog/intrinsic_scale_stress_runtime_profile.json",
                                "profile_exists": True,
                                "pass_cross_seed_replay": True,
                            },
                            {
                                "scenario": "ignored",
                                "method": "sim3",
                                "catalog_profile_path": "scenario_profile_catalog/ignored_runtime_profile.json",
                                "profile_exists": True,
                                "pass_cross_seed_replay": True,
                            },
                        ],
                    }
                ),
                encoding="utf-8",
            )
            minimal_set_json = root / "scenario_profile_catalog_minimal_set.json"
            minimal_set_json.write_text(
                json.dumps(
                    {
                        "pass_minimal_catalog_profile_set": True,
                        "rows": [
                            {"source_scenario": "nominal"},
                            {"source_scenario": "intrinsic_scale_stress"},
                        ],
                    }
                ),
                encoding="utf-8",
            )
            input_csv = root / "calibration_samples.csv"
            input_csv.write_text("split\nvalidation\n", encoding="utf-8")
            output_profile = root / "active_runtime_profile.json"
            output_dir = root / "selection"
            seen_scenarios: list[str] = []

            def fake_replay_candidate(**kwargs):
                scenario = kwargs["scenario"]
                seen_scenarios.append(scenario)
                profile_path = nominal_profile if scenario == "nominal" else intrinsic_profile
                corrected = 0.8 if scenario == "intrinsic_scale_stress" else 1.2
                return {
                    "scenario": scenario,
                    "method": "sim3_binned_residual_disparity",
                    **module.method_contract_fields("sim3_binned_residual_disparity"),
                    "profile_path": str(profile_path),
                    "pass_replay": True,
                    "validation_range_rmse_raw_m": 3.0,
                    "validation_range_rmse_corrected_m": corrected,
                    "validation_range_rmse_improvement_m": 3.0 - corrected,
                    "target_window_count": 2,
                    "target_abs_mean_error_corrected_m": 0.5,
                    "target_max_abs_error_corrected_m": 0.7,
                    "axis_contract": module.RDF_AXIS_CONTRACT,
                }

            with patch.object(module, "replay_candidate", side_effect=fake_replay_candidate):
                summary = module.select_by_calibration_replay(
                    catalog_json=catalog_json,
                    input_csv=input_csv,
                    output_profile=output_profile,
                    output_dir=output_dir,
                    minimal_set_json=minimal_set_json,
                )

            self.assertEqual(seen_scenarios, ["nominal", "intrinsic_scale_stress"])
            self.assertEqual(output_profile.read_text(encoding="utf-8"), intrinsic_profile.read_text(encoding="utf-8"))
            self.assertEqual(summary["scenario"], "intrinsic_scale_stress")
            self.assertEqual(summary["pass_count"], 2)
            self.assertEqual(summary["registration_model"], "sim3_estimated_to_truth")
            self.assertEqual(summary["residual_model"], "forward_disparity_lut")
            self.assertEqual(summary["runtime_profile_family"], "estimated_to_truth_sim3")
            self.assertEqual(summary["replay_selection_rank"], 1)
            self.assertIn("replay_selection_score", summary)
            self.assertIn("model_complexity_estimate", summary)
            self.assertEqual(summary["target_max_abs_error_corrected_m"], 0.7)
            self.assertTrue((output_dir / "calibration_replay_profile_candidates.csv").is_file())
            self.assertTrue((output_dir / "calibration_replay_profile_selection.json").is_file())
            self.assertEqual(summary["axis_contract"], module.RDF_AXIS_CONTRACT)

    def test_select_by_calibration_replay_all_catalog_mode_ignores_minimal_set_limit(self) -> None:
        module = load_replay_selector_module()
        with tempfile.TemporaryDirectory() as tmpdir:
            root = Path(tmpdir)
            catalog_dir = root / "scenario_profile_catalog"
            catalog_dir.mkdir()
            nominal_profile = catalog_dir / "nominal_runtime_profile.json"
            nominal_profile.write_text('{"profile": "nominal"}\n', encoding="utf-8")
            ignored_profile = catalog_dir / "ignored_runtime_profile.json"
            ignored_profile.write_text('{"profile": "ignored"}\n', encoding="utf-8")
            catalog_json = catalog_dir / "manifest.json"
            catalog_json.write_text(
                json.dumps(
                    {
                        "mode": "scenario_profile_catalog",
                        "profiles": [
                            {
                                "scenario": "nominal",
                                "method": "sim3",
                                "catalog_profile_path": "scenario_profile_catalog/nominal_runtime_profile.json",
                                "profile_exists": True,
                                "pass_cross_seed_replay": True,
                            },
                            {
                                "scenario": "ignored",
                                "method": "sim3",
                                "catalog_profile_path": "scenario_profile_catalog/ignored_runtime_profile.json",
                                "profile_exists": True,
                                "pass_cross_seed_replay": True,
                            },
                        ],
                    }
                ),
                encoding="utf-8",
            )
            minimal_set_json = root / "scenario_profile_catalog_minimal_set.json"
            minimal_set_json.write_text(
                json.dumps({"pass_minimal_catalog_profile_set": True, "rows": [{"source_scenario": "nominal"}]}),
                encoding="utf-8",
            )
            input_csv = root / "calibration_samples.csv"
            input_csv.write_text("split\nvalidation\n", encoding="utf-8")
            output_profile = root / "active_runtime_profile.json"
            seen_scenarios: list[str] = []

            def fake_replay_candidate(**kwargs):
                scenario = kwargs["scenario"]
                seen_scenarios.append(scenario)
                return {
                    "scenario": scenario,
                    "method": "sim3",
                    **module.method_contract_fields("sim3"),
                    "profile_path": str(ignored_profile if scenario == "ignored" else nominal_profile),
                    "pass_replay": True,
                    "validation_range_rmse_raw_m": 3.0,
                    "validation_range_rmse_corrected_m": 0.4 if scenario == "ignored" else 1.4,
                    "validation_range_rmse_improvement_m": 2.6 if scenario == "ignored" else 1.6,
                    "target_window_count": 2,
                    "target_abs_mean_error_corrected_m": 0.5,
                    "target_max_abs_error_corrected_m": 0.7,
                    "axis_contract": module.RDF_AXIS_CONTRACT,
                }

            with patch.object(module, "replay_candidate", side_effect=fake_replay_candidate):
                summary = module.select_by_calibration_replay(
                    catalog_json=catalog_json,
                    input_csv=input_csv,
                    output_profile=output_profile,
                    output_dir=root / "selection",
                    minimal_set_json=minimal_set_json,
                    candidate_source_mode="all_catalog",
                )

            self.assertEqual(seen_scenarios, ["nominal", "ignored"])
            self.assertEqual(summary["scenario"], "ignored")
            self.assertEqual(summary["candidate_source_mode"], "all_catalog")
            self.assertEqual(summary["candidate_source_scenarios"], "nominal,ignored")
            self.assertEqual(output_profile.read_text(encoding="utf-8"), ignored_profile.read_text(encoding="utf-8"))

    def test_replay_selection_score_penalizes_target_error_and_complexity(self) -> None:
        module = load_replay_selector_module()
        simple = {
            "validation_range_rmse_corrected_m": 1.0,
            "target_abs_mean_error_corrected_m": 0.5,
            "target_max_abs_error_corrected_m": 0.7,
            "model_complexity_estimate": 7,
        }
        complex_target_bad = {
            "validation_range_rmse_corrected_m": 0.95,
            "target_abs_mean_error_corrected_m": 2.0,
            "target_max_abs_error_corrected_m": 4.0,
            "model_complexity_estimate": 22,
        }
        low_mean_high_worst_case = {
            "validation_range_rmse_corrected_m": 1.0,
            "target_abs_mean_error_corrected_m": 0.1,
            "target_max_abs_error_corrected_m": 8.0,
            "model_complexity_estimate": 7,
        }

        self.assertLess(
            module.replay_selection_score(simple),
            module.replay_selection_score(complex_target_bad),
        )
        self.assertLess(
            module.replay_selection_score(simple),
            module.replay_selection_score(low_mean_high_worst_case),
        )
        self.assertEqual(module.method_complexity_estimate("sim3"), 7)
        self.assertGreater(
            module.method_complexity_estimate("sim3_agreement_gated_binned_residual_disparity_raw_forward"),
            module.method_complexity_estimate("sim3_binned_residual_raw_forward"),
        )
        self.assertLess(
            module.method_complexity_estimate("sim3_quadratic_residual_raw_forward_lut"),
            module.method_complexity_estimate("sim3_binned_residual_raw_forward"),
        )

    def test_replay_candidate_requires_requested_stability_splits_when_present(self) -> None:
        module = load_replay_selector_module()
        with tempfile.TemporaryDirectory() as tmpdir:
            root = Path(tmpdir)
            catalog_dir = root / "scenario_profile_catalog"
            catalog_dir.mkdir()
            profile_path = catalog_dir / "nominal_runtime_profile.json"
            profile_path.write_text("{}", encoding="utf-8")
            catalog_json = catalog_dir / "manifest.json"
            catalog = {
                "profiles": [
                    {
                        "scenario": "nominal",
                        "method": "sim3_binned_residual_raw_forward",
                        "catalog_profile_path": "scenario_profile_catalog/nominal_runtime_profile.json",
                        "profile_exists": True,
                        "pass_cross_seed_replay": True,
                    }
                ]
            }
            rows = [{"split": "train"}, {"split": "validation"}]

            def fake_replay_rows(_rows, _profile, *, split_filter, target_range_m, target_window_m):
                del _rows, _profile, target_range_m, target_window_m
                if split_filter == "train":
                    return [], {
                        "evaluated_row_count": 20,
                        "validation_range_rmse_raw_m": 1.0,
                        "validation_range_rmse_corrected_m": 1.5,
                        "target_window_count": 2,
                        "target_abs_mean_error_corrected_m": 0.5,
                        "target_max_abs_error_corrected_m": 0.7,
                    }
                return [], {
                    "evaluated_row_count": 10,
                    "validation_range_rmse_raw_m": 2.0,
                    "validation_range_rmse_corrected_m": 1.0,
                    "target_window_count": 2,
                    "target_abs_mean_error_corrected_m": 0.5,
                    "target_max_abs_error_corrected_m": 0.7,
                }

            with patch.object(module, "load_profile", return_value=object()), patch.object(
                module,
                "replay_rows",
                side_effect=fake_replay_rows,
            ):
                row = module.replay_candidate(
                    catalog_json=catalog_json,
                    catalog=catalog,
                    scenario="nominal",
                    rows=rows,
                    split="validation",
                    target_range_m=1852.0,
                    target_window_m=30.0,
                    target_error_limit_m=10.0,
                    allow_unvalidated=False,
                    stability_splits=["train", "validation"],
                )

            self.assertFalse(row["pass_replay"])
            self.assertFalse(row["stability_pass"])
            self.assertIn("train:no_positive_range_rmse_improvement", row["pass_reason"])
            self.assertEqual(row["stability_splits_evaluated"], "train,validation")

    def test_replay_candidate_rejects_target_window_worst_case_over_limit(self) -> None:
        module = load_replay_selector_module()
        with tempfile.TemporaryDirectory() as tmpdir:
            root = Path(tmpdir)
            catalog_dir = root / "scenario_profile_catalog"
            catalog_dir.mkdir()
            profile_path = catalog_dir / "nominal_runtime_profile.json"
            profile_path.write_text("{}", encoding="utf-8")
            catalog_json = catalog_dir / "manifest.json"
            catalog = {
                "profiles": [
                    {
                        "scenario": "nominal",
                        "method": "sim3",
                        "catalog_profile_path": "scenario_profile_catalog/nominal_runtime_profile.json",
                        "profile_exists": True,
                        "pass_cross_seed_replay": True,
                    }
                ]
            }
            rows = [{"split": "validation"}]

            def fake_replay_rows(_rows, _profile, *, split_filter, target_range_m, target_window_m):
                del _rows, _profile, split_filter, target_range_m, target_window_m
                return [], {
                    "evaluated_row_count": 8,
                    "validation_range_rmse_raw_m": 4.0,
                    "validation_range_rmse_corrected_m": 1.0,
                    "target_window_count": 2,
                    "target_abs_mean_error_corrected_m": 0.5,
                    "target_max_abs_error_corrected_m": 12.0,
                }

            with patch.object(module, "load_profile", return_value=object()), patch.object(
                module,
                "replay_rows",
                side_effect=fake_replay_rows,
            ):
                row = module.replay_candidate(
                    catalog_json=catalog_json,
                    catalog=catalog,
                    scenario="nominal",
                    rows=rows,
                    split="validation",
                    target_range_m=1852.0,
                    target_window_m=30.0,
                    target_error_limit_m=10.0,
                    allow_unvalidated=False,
                    stability_splits=["validation"],
                )

            self.assertFalse(row["pass_replay"])
            self.assertIn("target_max_error_over_limit", row["pass_reason"])
            self.assertEqual(row["target_max_abs_error_corrected_m"], 12.0)

    def test_select_by_calibration_replay_rejects_when_no_candidate_passes(self) -> None:
        module = load_replay_selector_module()
        with tempfile.TemporaryDirectory() as tmpdir:
            root = Path(tmpdir)
            catalog_dir = root / "scenario_profile_catalog"
            catalog_dir.mkdir()
            (catalog_dir / "nominal_runtime_profile.json").write_text("{}", encoding="utf-8")
            catalog_json = catalog_dir / "manifest.json"
            catalog_json.write_text(
                json.dumps(
                    {
                        "mode": "scenario_profile_catalog",
                        "profiles": [
                            {
                                "scenario": "nominal",
                                "catalog_profile_path": "scenario_profile_catalog/nominal_runtime_profile.json",
                                "profile_exists": True,
                                "pass_cross_seed_replay": True,
                            }
                        ],
                    }
                ),
                encoding="utf-8",
            )
            input_csv = root / "calibration_samples.csv"
            input_csv.write_text("split\nvalidation\n", encoding="utf-8")
            output_dir = root / "selection"

            with patch.object(
                module,
                "replay_candidate",
                return_value={
                    "scenario": "nominal",
                    "method": "sim3",
                    "profile_path": str(catalog_dir / "nominal_runtime_profile.json"),
                    "pass_replay": False,
                    "validation_range_rmse_raw_m": 1.0,
                    "validation_range_rmse_corrected_m": 2.0,
                    "target_window_count": 1,
                    "target_abs_mean_error_corrected_m": 0.5,
                },
            ):
                with self.assertRaisesRegex(ValueError, "no catalog candidate passed"):
                    module.select_by_calibration_replay(
                        catalog_json=catalog_json,
                        input_csv=input_csv,
                        output_profile=root / "active_runtime_profile.json",
                        output_dir=output_dir,
                    )

            self.assertTrue((output_dir / "calibration_replay_profile_candidates.csv").is_file())
            self.assertTrue((output_dir / "calibration_replay_profile_selection.json").is_file())


if __name__ == "__main__":
    unittest.main()
