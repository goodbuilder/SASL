from __future__ import annotations

import csv
import importlib.util
import json
import sys
import tempfile
import unittest
from pathlib import Path


ROOT_DIR = Path(__file__).resolve().parents[1]


def load_module():
    spec = importlib.util.spec_from_file_location(
        "summarize_runtime_deployment_strategy_under_test",
        ROOT_DIR / "scripts" / "summarize_runtime_deployment_strategy.py",
    )
    if spec is None or spec.loader is None:
        raise RuntimeError("cannot load summarize_runtime_deployment_strategy.py")
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def write_csv(path: Path, rows: list[dict[str, object]]) -> None:
    fieldnames = list(dict.fromkeys(key for row in rows for key in row.keys()))
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)


def write_json(path: Path, payload: dict[str, object]) -> None:
    path.write_text(json.dumps(payload), encoding="utf-8")


class SummarizeRuntimeDeploymentStrategyTests(unittest.TestCase):
    def test_recommends_known_override_and_unknown_single_pooled(self) -> None:
        module = load_module()
        with tempfile.TemporaryDirectory() as tmpdir:
            root = Path(tmpdir)
            known_catalog = root / "known_catalog.json"
            known_override = root / "known_override.json"
            loso_override = root / "loso_override.json"
            single = root / "single.csv"
            loso_single = root / "loso_single.csv"
            write_json(
                known_catalog,
                {
                    "audit_count": 2,
                    "pass_count": 2,
                    "pass_all": True,
                    "validation_range_rmse_corrected_mean_m": 1.0,
                    "validation_range_rmse_corrected_max_m": 1.6,
                    "target_max_abs_error_max_m": 9.6,
                },
            )
            write_json(
                known_override,
                {
                    "audit_count": 2,
                    "pass_count": 2,
                    "pass_all": True,
                    "validation_range_rmse_corrected_mean_m": 1.1,
                    "validation_range_rmse_corrected_max_m": 1.7,
                    "target_max_abs_error_max_m": 8.4,
                    "baseline_label": "catalog_guard",
                    "override_label": "single_pooled",
                    "rule_feature_name": "feature_left_v_std_px",
                    "rule_direction": "ge",
                    "rule_threshold": 129.7,
                },
            )
            write_json(
                loso_override,
                {
                    "audit_count": 2,
                    "pass_count": 2,
                    "pass_all": True,
                    "validation_range_rmse_corrected_mean_m": 1.3,
                    "validation_range_rmse_corrected_max_m": 2.5,
                    "target_max_abs_error_max_m": 9.7,
                },
            )
            write_csv(
                single,
                [
                    {
                        "pass_runtime_measurement_guard_replay": True,
                        "validation_range_rmse_corrected_m": 1.4,
                        "target_abs_mean_error_corrected_m": 1.0,
                        "target_max_abs_error_corrected_m": 8.4,
                    },
                    {
                        "pass_runtime_measurement_guard_replay": True,
                        "validation_range_rmse_corrected_m": 1.2,
                        "target_abs_mean_error_corrected_m": 0.8,
                        "target_max_abs_error_corrected_m": 7.0,
                    },
                ],
            )
            write_csv(
                loso_single,
                [
                    {
                        "pass_runtime_measurement_guard_replay": True,
                        "validation_range_rmse_corrected_m": 1.4,
                        "target_abs_mean_error_corrected_m": 1.0,
                        "target_max_abs_error_corrected_m": 8.4,
                    }
                ],
            )

            summary = module.summarize_runtime_deployment_strategy(
                known_catalog_summary_json=known_catalog,
                known_override_summary_json=known_override,
                single_pooled_candidate_csv=single,
                loso_override_summary_json=loso_override,
                loso_single_pooled_candidate_csv=loso_single,
                output_dir=root / "out",
                target_error_limit_m=10.0,
            )

            self.assertEqual(
                summary["known_catalog_recommendation"],
                "known_catalog_guard_with_single_pooled_override",
            )
            self.assertEqual(summary["unknown_regime_recommendation"], "single_pooled_runtime_profile")
            self.assertIn("feature_left_v_std_px", summary["known_catalog_runtime_rule"])
            self.assertEqual(summary["axis_contract"], module.RDF_AXIS_CONTRACT)
            self.assertTrue((root / "out" / "runtime_deployment_strategy_candidates.csv").is_file())
            self.assertTrue((root / "out" / "runtime_deployment_strategy_recommendation.txt").is_file())

    def test_strategy_score_prioritizes_pass_all_before_lower_target_error(self) -> None:
        module = load_module()
        rows = [
            {"strategy_label": "failing", "pass_all": False, "target_max_abs_error_max_m": 1.0},
            {
                "strategy_label": "passing",
                "pass_all": True,
                "target_max_abs_error_max_m": 8.0,
                "validation_range_rmse_corrected_mean_m": 2.0,
            },
        ]

        selected = module.choose_strategy(rows, target_error_limit_m=10.0)

        self.assertEqual(selected["strategy_label"], "passing")


if __name__ == "__main__":
    unittest.main()
