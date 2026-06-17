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
        "build_single_profile_runtime_audit_candidate_under_test",
        ROOT_DIR / "scripts" / "build_single_profile_runtime_audit_candidate.py",
    )
    if spec is None or spec.loader is None:
        raise RuntimeError("cannot load build_single_profile_runtime_audit_candidate.py")
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


class BuildSingleProfileRuntimeAuditCandidateTests(unittest.TestCase):
    def test_builds_portfolio_compatible_candidate_rows(self) -> None:
        module = load_module()
        with tempfile.TemporaryDirectory() as tmpdir:
            root = Path(tmpdir)
            input_csv = root / "calibration_samples.csv"
            guard_csv = root / "runtime_measurement_guard_audit.csv"
            profile_json = root / "profile.json"
            output_csv = root / "single_profile_candidate.csv"
            write_csv(
                input_csv,
                [
                    {
                        "split": "validation",
                        "truth_right_m": 0.0,
                        "truth_down_m": 0.0,
                        "truth_forward_m": 100.0,
                        "raw_right_m": 0.0,
                        "raw_down_m": 0.0,
                        "raw_forward_m": 101.0,
                        "measured_disparity_px": 12.0,
                    }
                ],
            )
            write_csv(
                guard_csv,
                [
                    {
                        "target_scenario": "nominal",
                        "target_seed": "seed_1",
                        "input_csv": str(input_csv),
                        "selection_input_fields": "raw_right_m,raw_down_m,raw_forward_m,measured_disparity_px",
                        "forbidden_selection_fields": "truth_right_m,truth_down_m,truth_forward_m",
                        "feature_raw_forward_mean_m": 101.0,
                        "feature_measured_disparity_mean_px": 12.0,
                    }
                ],
            )
            profile_json.write_text("{}", encoding="utf-8")

            class FakeProfile:
                profile_id = "fixed_profile"

            def fake_load_profile(path: Path) -> FakeProfile:
                self.assertEqual(path, profile_json)
                return FakeProfile()

            def fake_replay_rows(rows, profile, *, split_filter, target_range_m, target_window_m):
                self.assertEqual(profile.profile_id, "fixed_profile")
                return [], {
                    "evaluated_row_count": len(rows),
                    "validation_range_rmse_raw_m": 5.0,
                    "validation_range_rmse_corrected_m": 1.5,
                    "target_window_count": 1,
                    "target_abs_mean_error_corrected_m": 1.0,
                    "target_max_abs_error_corrected_m": 2.0,
                }

            module.load_profile = fake_load_profile
            module.replay_rows = fake_replay_rows

            rows = module.build_single_profile_candidate(
                guard_audit_csv=guard_csv,
                profile_json=profile_json,
                output_csv=output_csv,
                label="pooled",
                stability_splits=("validation",),
                target_error_limit_m=10.0,
            )

            self.assertEqual(len(rows), 1)
            self.assertTrue(rows[0]["pass_runtime_measurement_guard_replay"])
            self.assertEqual(rows[0]["selected_source_scenario"], "pooled")
            self.assertEqual(rows[0]["feature_raw_forward_mean_m"], "101.0")
            self.assertIn("truth-free runtime features", rows[0]["selection_contract"])
            saved = module.read_csv(output_csv)
            self.assertEqual(saved[0]["target_max_abs_error_corrected_m"], "2.0")
            self.assertEqual(saved[0]["axis_contract"], module.RDF_AXIS_CONTRACT)


if __name__ == "__main__":
    unittest.main()
