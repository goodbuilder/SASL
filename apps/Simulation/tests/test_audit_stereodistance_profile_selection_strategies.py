from __future__ import annotations

import importlib.util
import json
import sys
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch


ROOT_DIR = Path(__file__).resolve().parents[1]


def load_audit_module():
    spec = importlib.util.spec_from_file_location(
        "audit_stereodistance_profile_selection_strategies_under_test",
        ROOT_DIR / "scripts" / "audit_stereodistance_profile_selection_strategies.py",
    )
    if spec is None or spec.loader is None:
        raise RuntimeError("cannot load audit_stereodistance_profile_selection_strategies.py")
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


class AuditStereoDistanceProfileSelectionStrategiesTests(unittest.TestCase):
    def test_audit_compares_label_mapping_with_calibration_replay_selection(self) -> None:
        module = load_audit_module()
        with tempfile.TemporaryDirectory() as tmpdir:
            root = Path(tmpdir)
            sweep_dir = root / "sweep"
            sweep_dir.mkdir()
            catalog_dir = sweep_dir / "scenario_profile_catalog"
            catalog_dir.mkdir()
            (catalog_dir / "nominal_runtime_profile.json").write_text("{}", encoding="utf-8")
            (catalog_dir / "intrinsic_scale_stress_runtime_profile.json").write_text("{}", encoding="utf-8")
            (catalog_dir / "manifest.json").write_text(
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
                        ],
                    }
                ),
                encoding="utf-8",
            )
            (sweep_dir / "scenario_profile_catalog_minimal_set.json").write_text(
                json.dumps(
                    {
                        "pass_minimal_catalog_profile_set": True,
                        "rows": [
                            {
                                "source_scenario": "nominal",
                                "newly_covered_target_scenarios": "combined_sensor_stress",
                                "all_pass_target_scenarios": "combined_sensor_stress",
                            },
                            {
                                "source_scenario": "intrinsic_scale_stress",
                                "newly_covered_target_scenarios": "intrinsic_scale_stress",
                                "all_pass_target_scenarios": "intrinsic_scale_stress",
                            },
                        ],
                    }
                ),
                encoding="utf-8",
            )
            (sweep_dir / "scenario_runs.csv").write_text(
                "scenario,pass_run\ncombined_sensor_stress,True\n",
                encoding="utf-8",
            )
            scenario_dir = sweep_dir / "combined_sensor_stress"
            seed_dir = scenario_dir / "seed_1"
            seed_dir.mkdir(parents=True)
            (seed_dir / "calibration_samples.csv").write_text("split\nvalidation\n", encoding="utf-8")
            (scenario_dir / "seed_runs.csv").write_text(
                f"seed,run_dir,exit_code,pass_run\n1,{seed_dir},0,True\n",
                encoding="utf-8",
            )
            output_dir = root / "audit"

            def fake_replay_candidate(**kwargs):
                scenario = kwargs["scenario"]
                corrected = 1.5 if scenario == "nominal" else 0.9
                return {
                    "scenario": scenario,
                    "method": "sim3_binned_residual_disparity",
                    "profile_path": str(catalog_dir / f"{scenario}_runtime_profile.json"),
                    "pass_replay": True,
                    "evaluated_row_count": 10,
                    "validation_range_rmse_raw_m": 3.0,
                    "validation_range_rmse_corrected_m": corrected,
                    "validation_range_rmse_improvement_m": 3.0 - corrected,
                    "target_window_count": 2,
                    "target_abs_mean_error_corrected_m": 0.4,
                    "axis_contract": module.RDF_AXIS_CONTRACT,
                }

            with patch.object(module, "replay_candidate", side_effect=fake_replay_candidate):
                run_rows, scenario_rows, summary = module.audit_selection_strategies(
                    sweep_dir=sweep_dir,
                    output_dir=output_dir,
                )

            self.assertEqual(len(run_rows), 1)
            self.assertEqual(run_rows[0]["label_source_scenario"], "nominal")
            self.assertEqual(run_rows[0]["calibration_source_scenario"], "intrinsic_scale_stress")
            self.assertTrue(run_rows[0]["calibration_beats_label_rmse"])
            self.assertEqual(scenario_rows[0]["calibration_beats_label_count"], 1)
            self.assertEqual(summary["calibration_beats_label_count"], 1)
            self.assertTrue((output_dir / "profile_selection_strategy_runs.csv").is_file())
            self.assertTrue((output_dir / "profile_selection_strategy_scenarios.csv").is_file())
            self.assertTrue((output_dir / "profile_selection_strategy_summary.json").is_file())
            self.assertEqual(summary["axis_contract"], module.RDF_AXIS_CONTRACT)

    def test_all_catalog_candidate_mode_searches_beyond_minimal_set_sources(self) -> None:
        module = load_audit_module()
        with tempfile.TemporaryDirectory() as tmpdir:
            root = Path(tmpdir)
            sweep_dir = root / "sweep"
            catalog_dir = sweep_dir / "scenario_profile_catalog"
            catalog_dir.mkdir(parents=True)
            for name in ("nominal", "bbox_outlier_stress"):
                (catalog_dir / f"{name}_runtime_profile.json").write_text("{}", encoding="utf-8")
            (catalog_dir / "manifest.json").write_text(
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
                                "scenario": "bbox_outlier_stress",
                                "method": "sim3",
                                "catalog_profile_path": "scenario_profile_catalog/bbox_outlier_stress_runtime_profile.json",
                                "profile_exists": True,
                                "pass_cross_seed_replay": True,
                            },
                        ],
                    }
                ),
                encoding="utf-8",
            )
            (sweep_dir / "scenario_profile_catalog_minimal_set.json").write_text(
                json.dumps(
                    {
                        "pass_minimal_catalog_profile_set": True,
                        "rows": [
                            {
                                "source_scenario": "nominal",
                                "newly_covered_target_scenarios": "bbox_outlier_stress",
                                "all_pass_target_scenarios": "bbox_outlier_stress",
                            }
                        ],
                    }
                ),
                encoding="utf-8",
            )
            (sweep_dir / "scenario_runs.csv").write_text("scenario,pass_run\nbbox_outlier_stress,True\n", encoding="utf-8")
            scenario_dir = sweep_dir / "bbox_outlier_stress"
            seed_dir = scenario_dir / "seed_1"
            seed_dir.mkdir(parents=True)
            (seed_dir / "calibration_samples.csv").write_text("split\nvalidation\n", encoding="utf-8")
            (scenario_dir / "seed_runs.csv").write_text(f"seed,run_dir,exit_code,pass_run\n1,{seed_dir},0,True\n", encoding="utf-8")

            seen_sources: list[str] = []

            def fake_replay_candidate(**kwargs):
                scenario = kwargs["scenario"]
                seen_sources.append(scenario)
                corrected = 0.5 if scenario == "bbox_outlier_stress" else 1.5
                return {
                    "scenario": scenario,
                    "method": "sim3",
                    "profile_path": str(catalog_dir / f"{scenario}_runtime_profile.json"),
                    "pass_replay": True,
                    "evaluated_row_count": 10,
                    "validation_range_rmse_raw_m": 3.0,
                    "validation_range_rmse_corrected_m": corrected,
                    "validation_range_rmse_improvement_m": 3.0 - corrected,
                    "target_window_count": 2,
                    "target_abs_mean_error_corrected_m": 0.4,
                    "target_max_abs_error_corrected_m": 0.6,
                    "axis_contract": module.RDF_AXIS_CONTRACT,
                }

            with patch.object(module, "replay_candidate", side_effect=fake_replay_candidate):
                run_rows, _, summary = module.audit_selection_strategies(
                    sweep_dir=sweep_dir,
                    output_dir=root / "audit",
                    candidate_source_mode="all_catalog",
                )

            self.assertEqual(seen_sources, ["nominal", "bbox_outlier_stress"])
            self.assertEqual(run_rows[0]["label_source_scenario"], "nominal")
            self.assertEqual(run_rows[0]["calibration_source_scenario"], "bbox_outlier_stress")
            self.assertEqual(summary["candidate_source_mode"], "all_catalog")

    def test_can_exclude_target_scenario_from_calibration_replay_candidates(self) -> None:
        module = load_audit_module()
        with tempfile.TemporaryDirectory() as tmpdir:
            root = Path(tmpdir)
            sweep_dir = root / "sweep"
            catalog_dir = sweep_dir / "scenario_profile_catalog"
            catalog_dir.mkdir(parents=True)
            for name in ("nominal", "bbox_outlier_stress"):
                (catalog_dir / f"{name}_runtime_profile.json").write_text("{}", encoding="utf-8")
            (catalog_dir / "manifest.json").write_text(
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
                                "scenario": "bbox_outlier_stress",
                                "method": "sim3",
                                "catalog_profile_path": "scenario_profile_catalog/bbox_outlier_stress_runtime_profile.json",
                                "profile_exists": True,
                                "pass_cross_seed_replay": True,
                            },
                        ],
                    }
                ),
                encoding="utf-8",
            )
            (sweep_dir / "scenario_profile_catalog_minimal_set.json").write_text(
                json.dumps(
                    {
                        "pass_minimal_catalog_profile_set": True,
                        "rows": [
                            {
                                "source_scenario": "bbox_outlier_stress",
                                "newly_covered_target_scenarios": "bbox_outlier_stress",
                                "all_pass_target_scenarios": "bbox_outlier_stress",
                            }
                        ],
                    }
                ),
                encoding="utf-8",
            )
            (sweep_dir / "scenario_runs.csv").write_text("scenario,pass_run\nbbox_outlier_stress,True\n", encoding="utf-8")
            scenario_dir = sweep_dir / "bbox_outlier_stress"
            seed_dir = scenario_dir / "seed_1"
            seed_dir.mkdir(parents=True)
            (seed_dir / "calibration_samples.csv").write_text("split\nvalidation\n", encoding="utf-8")
            (scenario_dir / "seed_runs.csv").write_text(f"seed,run_dir,exit_code,pass_run\n1,{seed_dir},0,True\n", encoding="utf-8")

            seen_sources: list[str] = []

            def fake_replay_candidate(**kwargs):
                scenario = kwargs["scenario"]
                seen_sources.append(scenario)
                return {
                    "scenario": scenario,
                    "method": "sim3",
                    "profile_path": str(catalog_dir / f"{scenario}_runtime_profile.json"),
                    "pass_replay": True,
                    "evaluated_row_count": 10,
                    "validation_range_rmse_raw_m": 3.0,
                    "validation_range_rmse_corrected_m": 1.2,
                    "validation_range_rmse_improvement_m": 1.8,
                    "target_window_count": 2,
                    "target_abs_mean_error_corrected_m": 0.4,
                    "target_max_abs_error_corrected_m": 0.6,
                    "axis_contract": module.RDF_AXIS_CONTRACT,
                }

            with patch.object(module, "replay_candidate", side_effect=fake_replay_candidate):
                run_rows, _, summary = module.audit_selection_strategies(
                    sweep_dir=sweep_dir,
                    output_dir=root / "audit",
                    candidate_source_mode="all_catalog",
                    exclude_target_scenario_from_candidates=True,
                )

            self.assertEqual(seen_sources, ["nominal"])
            self.assertFalse(run_rows[0]["label_source_available"])
            self.assertEqual(run_rows[0]["eligible_candidate_source_scenarios"], "nominal")
            self.assertEqual(run_rows[0]["calibration_source_scenario"], "nominal")
            self.assertTrue(summary["exclude_target_scenario_from_candidates"])


if __name__ == "__main__":
    unittest.main()
