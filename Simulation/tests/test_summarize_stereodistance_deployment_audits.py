from __future__ import annotations

import csv
import importlib.util
import json
import sys
import tempfile
import unittest
from pathlib import Path


ROOT_DIR = Path(__file__).resolve().parents[1]


def load_summary_module():
    spec = importlib.util.spec_from_file_location(
        "summarize_stereodistance_deployment_audits_under_test",
        ROOT_DIR / "scripts" / "summarize_stereodistance_deployment_audits.py",
    )
    if spec is None or spec.loader is None:
        raise RuntimeError("cannot load summarize_stereodistance_deployment_audits.py")
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def write_csv(path: Path, rows: list[dict[str, object]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    fieldnames = list(dict.fromkeys(key for row in rows for key in row.keys()))
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)


class SummarizeStereoDistanceDeploymentAuditsTests(unittest.TestCase):
    def test_summarize_prefers_lower_score_blend_policy(self) -> None:
        module = load_summary_module()
        with tempfile.TemporaryDirectory() as tmpdir:
            root = Path(tmpdir)
            sweep_dir = root / "sweep"
            selection_dir = sweep_dir / "profile_selection_strategy_audit_all_catalog_target_max_score"
            blend_dir = sweep_dir / "profile_blend_strategy_audit_all_catalog_target_max_score"
            write_csv(
                selection_dir / "profile_selection_strategy_runs.csv",
                [
                    {
                        "calibration_pass": "True",
                        "calibration_validation_range_rmse_corrected_m": 0.8,
                        "calibration_target_abs_mean_error_corrected_m": 0.3,
                    },
                    {
                        "calibration_pass": "True",
                        "calibration_validation_range_rmse_corrected_m": 1.0,
                        "calibration_target_abs_mean_error_corrected_m": 0.4,
                    },
                ],
            )
            write_csv(
                blend_dir / "blend_strategy_policy_summary.csv",
                [
                    {
                        "policy": "balanced",
                        "run_count": 2,
                        "pass_count": 2,
                        "validation_range_rmse_corrected_mean_m": 0.7,
                        "validation_range_rmse_corrected_max_m": 0.9,
                        "target_abs_mean_error_mean_m": 0.1,
                        "target_abs_mean_error_max_m": 0.2,
                    },
                    {
                        "policy": "target_guard",
                        "run_count": 2,
                        "pass_count": 2,
                        "validation_range_rmse_corrected_mean_m": 0.9,
                        "validation_range_rmse_corrected_max_m": 1.1,
                        "target_abs_mean_error_mean_m": 0.05,
                        "target_abs_mean_error_max_m": 0.1,
                    },
                ],
            )
            (blend_dir / "blend_strategy_summary.json").write_text(
                json.dumps({"candidate_source_mode": "all_catalog"}),
                encoding="utf-8",
            )

            rows, recommendation = module.summarize_deployment_audits(
                sweep_dir=sweep_dir,
                output_dir=root / "out",
            )

            self.assertEqual(len(rows), 3)
            self.assertEqual(recommendation["recommended_strategy"], "blend_policy_calibration_replay_all_catalog")
            self.assertEqual(recommendation["recommended_policy"], "balanced")
            self.assertEqual(recommendation["axis_contract"], module.RDF_AXIS_CONTRACT)
            self.assertTrue((root / "out" / "deployment_audit_strategy_candidates.csv").is_file())
            self.assertTrue((root / "out" / "deployment_audit_strategy_recommendation.json").is_file())

    def test_selection_strategy_row_uses_candidate_source_mode_from_summary(self) -> None:
        module = load_summary_module()
        with tempfile.TemporaryDirectory() as tmpdir:
            root = Path(tmpdir)
            selection_dir = root / "selection"
            write_csv(
                selection_dir / "profile_selection_strategy_runs.csv",
                [
                    {
                        "calibration_pass": "True",
                        "calibration_validation_range_rmse_corrected_m": 0.8,
                        "calibration_target_abs_mean_error_corrected_m": 0.3,
                    }
                ],
            )
            (selection_dir / "profile_selection_strategy_summary.json").write_text(
                json.dumps({"candidate_source_mode": "minimal_set"}),
                encoding="utf-8",
            )

            row = module.selection_strategy_row(
                selection_dir / "profile_selection_strategy_runs.csv",
                selection_dir / "profile_selection_strategy_summary.json",
            )

            self.assertEqual(row["strategy"], "single_profile_calibration_replay_minimal_set")
            self.assertEqual(row["candidate_source_mode"], "minimal_set")

    def test_choose_recommendation_prefers_simpler_strategy_inside_score_margin(self) -> None:
        module = load_summary_module()
        rows = [
            {
                "strategy": "single_profile_calibration_replay_minimal_set",
                "policy": "single_best",
                "strategy_complexity": 1,
                "pass_count": 6,
                "deployment_audit_score": 2.1484,
                "validation_range_rmse_corrected_mean_m": 0.985,
                "target_abs_mean_error_mean_m": 1.149,
            },
            {
                "strategy": "blend_policy_calibration_replay_minimal_set",
                "policy": "validation_best",
                "strategy_complexity": 2,
                "pass_count": 6,
                "deployment_audit_score": 2.1480,
                "validation_range_rmse_corrected_mean_m": 0.984,
                "target_abs_mean_error_mean_m": 1.148,
            },
        ]

        recommendation = module.choose_recommendation(rows, simplicity_tie_margin=0.01)

        self.assertEqual(
            recommendation["recommended_strategy"],
            "single_profile_calibration_replay_minimal_set",
        )
        self.assertEqual(recommendation["best_score_strategy"], "blend_policy_calibration_replay_minimal_set")
        self.assertGreater(recommendation["score_margin_to_best_m"], 0.0)
        self.assertIn("lower-complexity", recommendation["recommendation_reason"])


if __name__ == "__main__":
    unittest.main()
