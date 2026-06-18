from __future__ import annotations

import csv
import importlib.util
import json
import sys
import tempfile
import unittest
from pathlib import Path


ROOT_DIR = Path(__file__).resolve().parents[1]
SCRIPT_PATH = ROOT_DIR / "scripts" / "export_observation_fusion_runtime_manifest.py"
SPEC = importlib.util.spec_from_file_location("export_observation_fusion_runtime_manifest_under_test", SCRIPT_PATH)
module = importlib.util.module_from_spec(SPEC)
assert SPEC is not None and SPEC.loader is not None
sys.modules[SPEC.name] = module
SPEC.loader.exec_module(module)


class ExportObservationFusionRuntimeManifestTests(unittest.TestCase):
    def test_adapter_config_for_repeated_observation(self) -> None:
        config = module.adapter_config(
            "RepeatedObservationFusion",
            {"repeat_count": "15", "fusion_mode": "trimmed_mean", "trim_fraction": "0.2"},
        )

        self.assertEqual(config["class"], "RepeatedObservationFusion")
        self.assertEqual(config["min_observations"], 15)
        self.assertEqual(config["mode"], "trimmed_mean")
        self.assertAlmostEqual(config["trim_fraction"], 0.2)

    def test_adapter_config_for_multiburst_observation(self) -> None:
        config = module.adapter_config(
            "MultiBurstObservationFusion",
            {
                "burst_count": "3",
                "observations_per_burst": "5",
                "inner_mode": "trimmed_mean",
                "outer_mode": "median",
                "trim_fraction": "0.2",
            },
        )

        self.assertEqual(config["class"], "MultiBurstObservationFusion")
        self.assertEqual(config["min_bursts"], 3)
        self.assertEqual(config["min_observations_per_burst"], 5)
        self.assertEqual(config["outer_mode"], "median")

    def test_build_manifest_uses_common_mode_default(self) -> None:
        with tempfile.TemporaryDirectory() as tmpdir:
            root = Path(tmpdir)
            repeated_csv = root / "repeated.csv"
            multiburst_csv = root / "multiburst.csv"
            fieldnames = [
                "method",
                "status",
                "pass_count",
                "trial_count",
                "repeat_count",
                "fusion_mode",
                "burst_count",
                "observations_per_burst",
                "inner_mode",
                "outer_mode",
                "trim_fraction",
                "range_rmse_corrected_mean_m",
                "range_rmse_corrected_max_m",
                "position_rmse_corrected_mean_m",
                "position_rmse_corrected_max_m",
                "target_max_abs_error_corrected_max_m",
                "target_rmse_corrected_mean_m",
                "runtime_truth_used",
            ]
            with repeated_csv.open("w", newline="", encoding="utf-8") as handle:
                writer = csv.DictWriter(handle, fieldnames=fieldnames)
                writer.writeheader()
                writer.writerow(
                    {
                        "method": "repeated_observation_trimmed_mean_n15_trim0p2",
                        "status": "STRICT_PASS",
                        "pass_count": "50",
                        "trial_count": "50",
                        "repeat_count": "15",
                        "fusion_mode": "trimmed_mean",
                        "trim_fraction": "0.2",
                        "range_rmse_corrected_mean_m": "0.2",
                        "range_rmse_corrected_max_m": "0.3",
                        "position_rmse_corrected_mean_m": "0.25",
                        "position_rmse_corrected_max_m": "0.35",
                        "target_max_abs_error_corrected_max_m": "1.2",
                        "target_rmse_corrected_mean_m": "0.4",
                        "runtime_truth_used": "False",
                    }
                )
            with multiburst_csv.open("w", newline="", encoding="utf-8") as handle:
                writer = csv.DictWriter(handle, fieldnames=fieldnames)
                writer.writeheader()
                writer.writerow(
                    {
                        "method": "multiburst_observation_inner_trimmed_mean_outer_median_b3x5_trim0p2_common1",
                        "status": "STRICT_PASS",
                        "pass_count": "50",
                        "trial_count": "50",
                        "burst_count": "3",
                        "observations_per_burst": "5",
                        "inner_mode": "trimmed_mean",
                        "outer_mode": "median",
                        "trim_fraction": "0.2",
                        "range_rmse_corrected_mean_m": "0.52",
                        "range_rmse_corrected_max_m": "0.73",
                        "position_rmse_corrected_mean_m": "0.54",
                        "position_rmse_corrected_max_m": "0.74",
                        "target_max_abs_error_corrected_max_m": "4.2",
                        "target_rmse_corrected_mean_m": "0.85",
                        "runtime_truth_used": "False",
                    }
                )

            manifest = module.build_manifest(
                root,
                [
                    {
                        "tier": "balanced_independent",
                        "purpose": "test",
                        "source_csv": "repeated.csv",
                        "method": "repeated_observation_trimmed_mean_n15_trim0p2",
                        "adapter_class": "RepeatedObservationFusion",
                        "runtime_condition": "independent",
                    },
                    {
                        "tier": "common_mode_target_gate",
                        "purpose": "test",
                        "source_csv": "multiburst.csv",
                        "method": "multiburst_observation_inner_trimmed_mean_outer_median_b3x5_trim0p2_common1",
                        "adapter_class": "MultiBurstObservationFusion",
                        "runtime_condition": "common",
                    },
                ],
            )

        self.assertEqual(manifest["default_recommendation"], "common_mode_target_gate")
        self.assertFalse(manifest["runtime_truth_used"])
        self.assertEqual(len(manifest["tiers"]), 2)

    def test_build_manifest_can_use_all_scenario_aggregate_source(self) -> None:
        with tempfile.TemporaryDirectory() as tmpdir:
            root = Path(tmpdir)
            aggregate_csv = root / "aggregate.csv"
            fieldnames = [
                "method",
                "scenario_count",
                "strict_pass_scenario_count",
                "all_scenarios_strict_pass",
                "total_observation_count",
                "burst_count",
                "observations_per_burst",
                "inner_mode",
                "outer_mode",
                "trim_fraction",
                "common_fraction",
                "target_max_abs_error_worst_m",
                "range_rmse_corrected_mean_across_scenarios_m",
                "position_rmse_corrected_mean_across_scenarios_m",
                "runtime_truth_used",
            ]
            with aggregate_csv.open("w", newline="", encoding="utf-8") as handle:
                writer = csv.DictWriter(handle, fieldnames=fieldnames)
                writer.writeheader()
                writer.writerow(
                    {
                        "method": "multiburst_observation_inner_trimmed_mean_outer_trimmed_mean_b7x3_trim0p2_common0p5",
                        "scenario_count": "4",
                        "strict_pass_scenario_count": "4",
                        "all_scenarios_strict_pass": "True",
                        "total_observation_count": "21",
                        "burst_count": "7",
                        "observations_per_burst": "3",
                        "inner_mode": "trimmed_mean",
                        "outer_mode": "trimmed_mean",
                        "trim_fraction": "0.2",
                        "common_fraction": "0.5",
                        "target_max_abs_error_worst_m": "5.064",
                        "range_rmse_corrected_mean_across_scenarios_m": "1.309",
                        "position_rmse_corrected_mean_across_scenarios_m": "3.624",
                        "runtime_truth_used": "False",
                    }
                )

            manifest = module.build_manifest(
                root,
                [
                    {
                        "tier": "fresh_common_mode_b7x3_aggregate_best",
                        "purpose": "aggregate",
                        "source_csv": "aggregate.csv",
                        "method": "multiburst_observation_inner_trimmed_mean_outer_trimmed_mean_b7x3_trim0p2_common0p5",
                        "adapter_class": "MultiBurstObservationFusion",
                        "runtime_condition": "fresh aggregate",
                    }
                ],
            )

        tier = manifest["tiers"][0]
        self.assertEqual(manifest["default_recommendation"], "fresh_common_mode_b7x3_aggregate_best")
        self.assertEqual(tier["adapter"]["min_bursts"], 7)
        self.assertEqual(tier["adapter"]["min_observations_per_burst"], 3)
        self.assertEqual(tier["adapter"]["outer_mode"], "trimmed_mean")
        self.assertEqual(tier["evidence"]["status"], "STRICT_PASS")
        self.assertEqual(tier["evidence"]["pass_count"], 4)
        self.assertEqual(tier["evidence"]["trial_count"], 4)
        self.assertAlmostEqual(tier["evidence"]["target_max_abs_error_corrected_max_m"], 5.064)
        self.assertFalse(tier["evidence"]["runtime_truth_used"])

    def test_load_tier_specs_from_json(self) -> None:
        with tempfile.TemporaryDirectory() as tmpdir:
            spec_path = Path(tmpdir) / "tiers.json"
            spec_path.write_text(
                json.dumps(
                    [
                        {
                            "tier": "fresh_common_mode_balanced",
                            "purpose": "fresh multi-burst result",
                            "source_csv": "multiburst.csv",
                            "method": "multiburst_observation_inner_trimmed_mean_outer_median_b5x5_trim0p2_common0p5",
                            "adapter_class": "MultiBurstObservationFusion",
                            "runtime_condition": "common-mode fresh-seed validated",
                        }
                    ]
                ),
                encoding="utf-8",
            )

            specs = module.load_tier_specs(spec_path)

        self.assertEqual(len(specs), 1)
        self.assertEqual(specs[0]["tier"], "fresh_common_mode_balanced")
        self.assertEqual(specs[0]["adapter_class"], "MultiBurstObservationFusion")


if __name__ == "__main__":
    unittest.main()
