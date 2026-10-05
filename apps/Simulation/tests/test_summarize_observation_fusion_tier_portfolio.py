from __future__ import annotations

import importlib.util
import json
import sys
import tempfile
import unittest
from pathlib import Path


ROOT_DIR = Path(__file__).resolve().parents[1]
SCRIPT_PATH = ROOT_DIR / "scripts" / "summarize_observation_fusion_tier_portfolio.py"


def load_module():
    spec = importlib.util.spec_from_file_location("summarize_observation_fusion_tier_portfolio_under_test", SCRIPT_PATH)
    if spec is None or spec.loader is None:
        raise RuntimeError(f"cannot load {SCRIPT_PATH}")
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def write_scenario_summary(path: Path, *, scenario: str, rows: list[tuple[str, float, float, int]]) -> None:
    path.mkdir(parents=True)
    path.joinpath("multiburst_observation_metrics.json").write_text(
        json.dumps({"scenario": scenario}),
        encoding="utf-8",
    )
    lines = [
        "method,status,pass_count,trial_count,burst_count,observations_per_burst,total_observation_count,inner_mode,outer_mode,trim_fraction,common_fraction,range_rmse_raw_mean_m,range_rmse_corrected_mean_m,position_rmse_raw_mean_m,position_rmse_corrected_mean_m,target_max_abs_error_corrected_max_m,target_rmse_corrected_mean_m,runtime_truth_used"
    ]
    for method, target, range_rmse, obs in rows:
        bursts = obs // 3
        lines.append(
            f"{method},STRICT_PASS,2,2,{bursts},3,{obs},trimmed_mean,trimmed_mean,0.2,0.5,3.0,{range_rmse},5.0,2.0,{target},1.0,False"
        )
    path.joinpath("multiburst_observation_method_summary.csv").write_text("\n".join(lines) + "\n", encoding="utf-8")


class SummarizeObservationFusionTierPortfolioTests(unittest.TestCase):
    def test_builds_oracle_selection_from_manifest_tiers(self) -> None:
        module = load_module()
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            result_root = root / "results" / "probe"
            write_scenario_summary(
                result_root / "bbox",
                scenario="bbox",
                rows=[("m_default", 3.0, 1.0, 21), ("m_slow", 2.0, 1.2, 45)],
            )
            write_scenario_summary(
                result_root / "disparity",
                scenario="disparity",
                rows=[("m_default", 5.0, 2.0, 21), ("m_slow", 6.0, 2.1, 45)],
            )
            aggregate_dir = result_root / "aggregate_summary"
            aggregate_dir.mkdir()
            aggregate_csv = aggregate_dir / "multiburst_observation_method_aggregate.csv"
            aggregate_csv.write_text(
                "\n".join(
                    [
                        "method,scenario_count,strict_pass_scenario_count,all_scenarios_strict_pass,total_observation_count,burst_count,observations_per_burst,inner_mode,outer_mode,trim_fraction,common_fraction,target_max_abs_error_worst_m,range_rmse_corrected_mean_across_scenarios_m,position_rmse_corrected_mean_across_scenarios_m,runtime_truth_used,status",
                        "m_default,2,2,True,21,7,3,trimmed_mean,trimmed_mean,0.2,0.5,5.0,1.5,2.0,False,STRICT_PASS",
                        "m_slow,2,2,True,45,15,3,trimmed_mean,trimmed_mean,0.2,0.5,6.0,1.6,2.2,False,STRICT_PASS",
                    ]
                )
                + "\n",
                encoding="utf-8",
            )
            manifest = {
                "default_recommendation": "default",
                "tiers": [
                    {
                        "tier": "default",
                        "method": "m_default",
                        "selection_scope": "all_scenario_aggregate",
                        "source_csv": str(aggregate_csv),
                        "adapter": {
                            "class": "MultiBurstObservationFusion",
                            "min_bursts": 7,
                            "min_observations_per_burst": 3,
                        },
                    },
                    {
                        "tier": "slow",
                        "method": "m_slow",
                        "selection_scope": "all_scenario_aggregate",
                        "source_csv": str(aggregate_csv),
                        "adapter": {
                            "class": "MultiBurstObservationFusion",
                            "min_bursts": 15,
                            "min_observations_per_burst": 3,
                        },
                    },
                    {
                        "tier": "single_source_bbox",
                        "method": "m_default",
                        "selection_scope": "single_source",
                        "source_csv": str(result_root / "bbox" / "multiburst_observation_method_summary.csv"),
                        "adapter": {
                            "class": "MultiBurstObservationFusion",
                            "min_bursts": 7,
                            "min_observations_per_burst": 3,
                        },
                    },
                ],
            }

            scenario_rows = module.tier_scenario_rows(manifest, root_dir=root)
            oracle_rows = module.best_rows_by_scenario(scenario_rows)
            summary = module.summary_payload(manifest=manifest, scenario_rows=scenario_rows, oracle_rows=oracle_rows)

            self.assertEqual(len(scenario_rows), 4)
            self.assertEqual([row["tier"] for row in oracle_rows], ["slow", "default"])
            self.assertEqual(summary["default_target_worst_m"], 5.0)
            self.assertEqual(summary["oracle_target_worst_m"], 5.0)
            self.assertEqual(summary["included_selection_scopes"], ["all_scenario_aggregate"])
            self.assertFalse(summary["include_single_source_tiers"])
            self.assertFalse(summary["runtime_truth_used"])
            self.assertIn("left_u_px", summary["runtime_contract"])

            all_rows = module.tier_scenario_rows(manifest, root_dir=root, include_single_source=True)
            all_summary = module.summary_payload(
                manifest=manifest,
                scenario_rows=all_rows,
                oracle_rows=module.best_rows_by_scenario(all_rows),
                include_single_source=True,
            )
            self.assertEqual(len(all_rows), 5)
            self.assertEqual(
                all_summary["included_selection_scopes"],
                ["all_scenario_aggregate", "single_source"],
            )
            self.assertTrue(all_summary["include_single_source_tiers"])


if __name__ == "__main__":
    unittest.main()
