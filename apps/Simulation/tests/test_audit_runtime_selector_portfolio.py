from __future__ import annotations

import csv
import importlib.util
import sys
import tempfile
import unittest
from pathlib import Path


ROOT_DIR = Path(__file__).resolve().parents[1]


def load_module():
    spec = importlib.util.spec_from_file_location(
        "audit_runtime_selector_portfolio_under_test",
        ROOT_DIR / "scripts" / "audit_runtime_selector_portfolio.py",
    )
    if spec is None or spec.loader is None:
        raise RuntimeError("cannot load audit_runtime_selector_portfolio.py")
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def write_candidate(path: Path, rows: list[dict[str, object]]) -> None:
    fieldnames = list(dict.fromkeys(key for row in rows for key in row.keys()))
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)


def row(scenario: str, seed: str, feature: float, rmse: float, target_max: float, passed: bool) -> dict[str, object]:
    return {
        "target_scenario": scenario,
        "target_seed": seed,
        "feature_raw_forward_mean_m": feature,
        "feature_measured_disparity_mean_px": 10.0,
        "pass_runtime_measurement_guard_replay": passed,
        "validation_range_rmse_corrected_m": rmse,
        "validation_range_rmse_improvement_m": 1.0,
        "target_abs_mean_error_corrected_m": min(target_max, 9.0),
        "target_max_abs_error_corrected_m": target_max,
        "selected_source_scenario": scenario,
        "selected_source_seed": seed,
        "selected_method": "unit",
    }


class AuditRuntimeSelectorPortfolioTests(unittest.TestCase):
    def test_portfolio_selects_candidate_with_raw_feature_threshold(self) -> None:
        module = load_module()
        with tempfile.TemporaryDirectory() as tmpdir:
            root = Path(tmpdir)
            policy = root / "policy.csv"
            affine = root / "affine.csv"
            write_candidate(
                policy,
                [
                    row("near", "seed_1", 1.0, 1.0, 8.0, True),
                    row("near", "seed_2", 2.0, 1.0, 8.0, True),
                    row("far", "seed_1", 10.0, 1.5, 7.0, True),
                ],
            )
            write_candidate(
                affine,
                [
                    row("near", "seed_1", 1.0, 0.8, 6.0, True),
                    row("near", "seed_2", 2.0, 0.8, 6.0, True),
                    row("far", "seed_1", 10.0, 3.0, 12.0, False),
                ],
            )

            summary = module.audit_selector_portfolio(
                [f"policy={policy}", f"affine={affine}"],
                root / "out",
                target_error_limit_m=10.0,
                include_target_run=True,
            )

            self.assertEqual(summary["pass_count"], 3)
            self.assertEqual(summary["selected_oracle_match_count"], 3)
            rows = module.read_csv(root / "out" / "runtime_selector_portfolio_audit.csv")
            selected = {(item["target_scenario"], item["target_seed"]): item["selected_candidate"] for item in rows}
            self.assertEqual(selected[("near", "seed_1")], "affine")
            self.assertEqual(selected[("near", "seed_2")], "affine")
            self.assertEqual(selected[("far", "seed_1")], "policy")
            self.assertTrue(all(item["selected_matches_oracle"] == "True" for item in rows))
            self.assertIn("raw RDF/disparity", (root / "out" / "runtime_selector_portfolio_summary.txt").read_text())

    def test_portfolio_accepts_target_cap_depth_two_rule(self) -> None:
        module = load_module()
        with tempfile.TemporaryDirectory() as tmpdir:
            root = Path(tmpdir)
            policy = root / "policy.csv"
            affine = root / "affine.csv"
            write_candidate(
                policy,
                [
                    row("near", "seed_1", 1.0, 1.0, 8.0, True),
                    row("near", "seed_2", 2.0, 1.0, 8.0, True),
                    row("far", "seed_1", 10.0, 1.5, 7.0, True),
                    row("far", "seed_2", 11.0, 1.5, 7.0, True),
                ],
            )
            write_candidate(
                affine,
                [
                    row("near", "seed_1", 1.0, 0.8, 6.0, True),
                    row("near", "seed_2", 2.0, 0.8, 6.0, True),
                    row("far", "seed_1", 10.0, 3.0, 12.0, False),
                    row("far", "seed_2", 11.0, 3.0, 12.0, False),
                ],
            )

            summary = module.audit_selector_portfolio(
                [f"policy={policy}", f"affine={affine}"],
                root / "out",
                target_error_limit_m=10.0,
                objective="target_cap_then_rmse",
                include_target_run=True,
                max_depth=2,
            )

            self.assertEqual(summary["pass_count"], 4)
            self.assertEqual(summary["max_depth"], 2)
            self.assertIn("rmse_regret_max_m", summary)
            rows = module.read_csv(root / "out" / "runtime_selector_portfolio_audit.csv")
            self.assertTrue(all(item["rule_json"] for item in rows))
            self.assertTrue(all("oracle_candidate" in item for item in rows))
            self.assertIn("target_cap_then_rmse", (root / "out" / "runtime_selector_portfolio_summary.txt").read_text())

    def test_portfolio_writes_oracle_separability_diagnostics(self) -> None:
        module = load_module()
        with tempfile.TemporaryDirectory() as tmpdir:
            root = Path(tmpdir)
            policy = root / "policy.csv"
            affine = root / "affine.csv"
            write_candidate(
                policy,
                [
                    row("near", "seed_1", 1.0, 1.0, 8.0, True),
                    row("near", "seed_2", 2.0, 1.0, 8.0, True),
                    row("far", "seed_1", 10.0, 1.5, 7.0, True),
                ],
            )
            write_candidate(
                affine,
                [
                    row("near", "seed_1", 1.0, 0.8, 6.0, True),
                    row("near", "seed_2", 2.0, 0.8, 6.0, True),
                    row("far", "seed_1", 10.0, 3.0, 12.0, False),
                ],
            )

            summary = module.audit_selector_portfolio(
                [f"policy={policy}", f"affine={affine}"],
                root / "out",
                target_error_limit_m=10.0,
                include_target_run=True,
            )

            self.assertEqual(summary["oracle_separability_best_label"], "affine")
            self.assertEqual(summary["oracle_separability_best_accuracy"], 1.0)
            rows = module.read_csv(root / "out" / "runtime_selector_portfolio_audit.csv")
            self.assertIn("feature_raw_forward_mean_m", rows[0])
            separability_rows = module.read_csv(root / "out" / "runtime_selector_portfolio_oracle_separability.csv")
            self.assertEqual(separability_rows[0]["oracle_label"], "affine")
            self.assertIn(
                "oracle labels are calibration/replay truth only",
                (root / "out" / "runtime_selector_portfolio_oracle_separability.txt").read_text(),
            )

    def test_portfolio_precision_override_uses_zero_false_positive_rule(self) -> None:
        module = load_module()
        with tempfile.TemporaryDirectory() as tmpdir:
            root = Path(tmpdir)
            policy = root / "policy.csv"
            affine = root / "affine.csv"
            write_candidate(
                policy,
                [
                    row("near", "seed_1", 1.0, 1.0, 8.0, True),
                    row("near", "seed_2", 2.0, 1.0, 8.0, True),
                    row("far", "seed_1", 10.0, 1.5, 7.0, True),
                ],
            )
            write_candidate(
                affine,
                [
                    row("near", "seed_1", 1.0, 0.8, 6.0, True),
                    row("near", "seed_2", 2.0, 0.8, 6.0, True),
                    row("far", "seed_1", 10.0, 3.0, 12.0, False),
                ],
            )

            summary = module.audit_selector_portfolio(
                [f"policy={policy}", f"affine={affine}"],
                root / "out",
                target_error_limit_m=10.0,
                include_target_run=True,
                precision_override_label="affine",
                precision_override_baseline_label="policy",
            )

            self.assertEqual(summary["precision_override_label"], "affine")
            self.assertEqual(summary["precision_override_count"], 2)
            self.assertEqual(summary["precision_override_pass_count"], 3)
            override_summary = module.read_csv(root / "out" / "runtime_selector_portfolio_precision_override_audit.csv")
            selected = {(item["target_scenario"], item["target_seed"]): item["selected_candidate"] for item in override_summary}
            self.assertEqual(selected[("near", "seed_1")], "affine")
            self.assertEqual(selected[("near", "seed_2")], "affine")
            self.assertEqual(selected[("far", "seed_1")], "policy")
            self.assertIn(
                "conservative runtime-safe override",
                (root / "out" / "runtime_selector_portfolio_precision_override_summary.txt").read_text(),
            )


if __name__ == "__main__":
    unittest.main()
