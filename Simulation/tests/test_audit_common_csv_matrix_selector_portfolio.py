from __future__ import annotations

import csv
import importlib.util
import sys
import tempfile
import unittest
from pathlib import Path


ROOT_DIR = Path(__file__).resolve().parents[1]
SCRIPT_PATH = ROOT_DIR / "scripts" / "audit_common_csv_matrix_selector_portfolio.py"


def load_module():
    spec = importlib.util.spec_from_file_location("audit_common_csv_matrix_selector_portfolio_under_test", SCRIPT_PATH)
    if spec is None or spec.loader is None:
        raise RuntimeError(f"cannot load {SCRIPT_PATH}")
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


def add_run(matrix: Path, scenario: str, seed: str, rows: list[dict[str, object]]) -> None:
    enriched = []
    for row in rows:
        payload = {
            "candidate": row["method"],
            "profile_json": f"{row['method']}.json",
            "profile_mode": "estimated_to_truth_sim3",
            "pass_common_replay": "True",
            "validation_range_rmse_corrected_m": row["rmse"],
            "target_max_abs_error_corrected_m": row["target"],
            "common_csv_score": row["score"],
            "runtime_truth_used": "False",
        }
        payload.update(row)
        enriched.append(payload)
    write_csv(matrix / "runs" / scenario / seed / "common_csv_profile_comparison.csv", enriched)


class AuditCommonCsvMatrixSelectorPortfolioTests(unittest.TestCase):
    def test_selector_modes_are_summarized_without_runtime_truth(self) -> None:
        module = load_module()
        with tempfile.TemporaryDirectory() as tmpdir:
            root = Path(tmpdir)
            matrix = root / "matrix"
            add_run(
                matrix,
                "scenario_a",
                "seed_1",
                [
                    {"method": "global", "rmse": 2.0, "target": 3.0, "score": 5.0},
                    {"method": "a_profile", "rmse": 1.0, "target": 2.0, "score": 3.0},
                    {"method": "b_profile", "rmse": 4.0, "target": 5.0, "score": 9.0},
                ],
            )
            add_run(
                matrix,
                "scenario_a",
                "seed_2",
                [
                    {"method": "global", "rmse": 2.2, "target": 3.2, "score": 5.4},
                    {"method": "a_profile", "rmse": 1.1, "target": 2.1, "score": 3.2},
                    {"method": "b_profile", "rmse": 4.1, "target": 5.1, "score": 9.1},
                ],
            )
            add_run(
                matrix,
                "scenario_b",
                "seed_1",
                [
                    {"method": "global", "rmse": 2.0, "target": 3.0, "score": 5.0},
                    {"method": "a_profile", "rmse": 4.0, "target": 5.0, "score": 9.0},
                    {"method": "b_profile", "rmse": 1.0, "target": 2.0, "score": 3.0},
                ],
            )
            add_run(
                matrix,
                "scenario_b",
                "seed_2",
                [
                    {"method": "global", "rmse": 2.2, "target": 3.2, "score": 5.4},
                    {"method": "a_profile", "rmse": 4.1, "target": 5.1, "score": 9.1},
                    {"method": "b_profile", "rmse": 1.1, "target": 2.1, "score": 3.2},
                ],
            )

            result = module.audit_selector_portfolio(
                matrix_dirs=[matrix],
                output_dir=root / "out",
                objective="score",
            )

            self.assertEqual(result["status"], "PASS")
            self.assertFalse(result["runtime_truth_used"])
            self.assertEqual(result["recommended_deployable_selector_mode"], "scenario_loso")
            self.assertEqual(result["upper_bound_selector_mode"], "per_run_oracle")
            modes = {row["selector_mode"]: row for row in result["selector_modes"]}
            self.assertEqual(modes["global_single"]["run_count"], 4)
            self.assertLess(
                float(modes["scenario_loso"]["validation_range_rmse_corrected_mean_m"]),
                float(modes["global_single"]["validation_range_rmse_corrected_mean_m"]),
            )
            self.assertLessEqual(
                float(modes["per_run_oracle"]["validation_range_rmse_corrected_mean_m"]),
                float(modes["scenario_loso"]["validation_range_rmse_corrected_mean_m"]),
            )
            self.assertTrue((root / "out" / "selector_portfolio_modes.csv").is_file())


if __name__ == "__main__":
    unittest.main()
