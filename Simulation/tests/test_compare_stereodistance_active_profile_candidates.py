from __future__ import annotations

import importlib.util
import json
import sys
import tempfile
import unittest
from pathlib import Path


ROOT_DIR = Path(__file__).resolve().parents[1]


def load_module():
    spec = importlib.util.spec_from_file_location(
        "compare_stereodistance_active_profile_candidates_under_test",
        ROOT_DIR / "scripts" / "compare_stereodistance_active_profile_candidates.py",
    )
    if spec is None or spec.loader is None:
        raise RuntimeError("cannot load compare_stereodistance_active_profile_candidates.py")
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def write_candidate(
    root: Path,
    *,
    label: str,
    method: str,
    dashboard_status: str = "PASS",
    readiness_status: str = "PASS",
    axis_status: str = "PASS",
    runtime_truth_used: bool = False,
    robust_target_max: float = 1.0,
    robust_rmse_mean: float = 1.0,
    robust_axis_regression: float = 0.0,
    profile_rmse: float | None = None,
    robust_method: str | None = None,
    sample_offset: float = 0.0,
) -> tuple[str, Path, Path]:
    dashboard = root / label / "dashboard"
    dashboard.mkdir(parents=True)
    dashboard.joinpath("active_runtime_profile.json").write_text(
        json.dumps(
            {
                "method": method,
                "profile_id": f"rtk_stereo_calibration-{method}",
                "mode": "estimated_to_truth_sim3",
                "validation_summary": {
                    "validation_range_rmse_raw_m": 3.0,
                    "validation_range_rmse_corrected_m": (
                        robust_rmse_mean if profile_rmse is None else profile_rmse
                    ),
                    "target_1852m_abs_mean_error_m": 0.5,
                },
            },
            ensure_ascii=False,
        ),
        encoding="utf-8",
    )
    dashboard.joinpath("rtk_stereo_acceptance_dashboard_with_promotion_summary.json").write_text(
        json.dumps(
            {
                "status": dashboard_status,
                "final_rdf_axis_nonregression_status": axis_status,
                "runtime_truth_used": runtime_truth_used,
                "promotion_sample_apply_count": 12,
            },
            ensure_ascii=False,
        ),
        encoding="utf-8",
    )
    sample_csv = root / label / "calibration_samples.csv"
    sample_csv.write_text(
        (
            "sample_id,split,truth_right_m,truth_down_m,truth_forward_m,truth_range_m,"
            "raw_right_m,raw_down_m,raw_forward_m,raw_range_m,measured_disparity_px\n"
            f"0,validation,1,2,{100 + sample_offset},{100.025 + sample_offset},"
            f"1.1,2.1,{101 + sample_offset},{101.027 + sample_offset},200\n"
            "1,train,1,2,200,200.012,1.1,2.1,201,201.014,100\n"
        ),
        encoding="utf-8",
    )
    promotion_dir = dashboard / "active_profile_promotion_check"
    promotion_dir.mkdir()
    promotion_dir.joinpath("promotion_check.json").write_text(
        json.dumps(
            {
                "sample_csv": str(sample_csv),
                "sample_split": "validation",
                "sample_apply_count": 1,
                "sample_invalid_count": 0,
            },
            ensure_ascii=False,
        ),
        encoding="utf-8",
    )
    dashboard.joinpath("deployment_readiness.json").write_text(
        json.dumps(
            {
                "status": readiness_status,
                "active_profile_sha256": f"sha-{label}",
            },
            ensure_ascii=False,
        ),
        encoding="utf-8",
    )
    robustness = root / label / "robustness_summary.json"
    robustness.write_text(
        json.dumps(
            {
                "robust_recommendation": {
                    "method": method if robust_method is None else robust_method,
                    "validation_range_rmse_mean_m": robust_rmse_mean,
                    "validation_range_rmse_max_m": robust_rmse_mean + 0.2,
                    "target_abs_mean_error_mean_m": 0.4,
                    "target_abs_mean_error_max_m": robust_target_max,
                    "axis_rmse_regression_sum_mean_m": robust_axis_regression,
                }
            },
            ensure_ascii=False,
        ),
        encoding="utf-8",
    )
    return label, dashboard, robustness


class CompareStereoDistanceActiveProfileCandidatesTests(unittest.TestCase):
    def test_selects_best_balanced_validation_target_candidate(self) -> None:
        module = load_module()
        with tempfile.TemporaryDirectory() as tmpdir:
            root = Path(tmpdir)
            candidate_a = write_candidate(
                root,
                label="current",
                method="sigma060",
                robust_target_max=1.6,
                robust_rmse_mean=1.3,
            )
            candidate_b = write_candidate(
                root,
                label="new",
                method="sigma045",
                robust_target_max=2.1,
                robust_rmse_mean=1.0,
            )

            summary = module.compare_candidates(
                candidate_specs=[
                    f"{candidate_a[0]}:{candidate_a[1]}:{candidate_a[2]}",
                    f"{candidate_b[0]}:{candidate_b[1]}:{candidate_b[2]}",
                ],
                output_dir=root / "out",
            )

            self.assertEqual(summary["status"], "PASS")
            self.assertTrue(summary["promotion_samples_comparable"])
            self.assertEqual(summary["selected_candidate"], "current")
            self.assertEqual(summary["selected_method"], "sigma060")
            self.assertIn("balanced validation/1852m target score", summary["reason"])
            txt = (root / "out" / "profile_candidate_comparison.txt").read_text(encoding="utf-8")
            self.assertIn("stereo-center RDF", txt)
            self.assertIn("profile_matches_robust", txt)
            self.assertIn("sample_hash", txt)
            csv_text = (root / "out" / "profile_candidate_comparison.csv").read_text(encoding="utf-8")
            self.assertIn("deployment_ready", csv_text)
            self.assertIn("balanced_validation_target_score", csv_text)
            self.assertIn("profile_matches_robust_recommendation", csv_text)
            self.assertIn("promotion_sample_fingerprint_sha256", csv_text)
            rows = {row["candidate"]: row for row in summary["rows"]}
            self.assertEqual(rows["current"]["promotion_sample_fingerprint_count"], 1)
            self.assertTrue(rows["current"]["promotion_sample_fingerprint_sha256"])

    def test_does_not_promote_tiny_target_gain_with_large_validation_regression(self) -> None:
        module = load_module()
        with tempfile.TemporaryDirectory() as tmpdir:
            root = Path(tmpdir)
            current = write_candidate(
                root,
                label="current",
                method="active_sigma060",
                robust_target_max=1.601,
                robust_rmse_mean=1.367,
                profile_rmse=1.154,
            )
            target_only = write_candidate(
                root,
                label="target_only",
                method="fscale_probe",
                robust_target_max=1.598,
                robust_rmse_mean=1.516,
                profile_rmse=1.629,
                sample_offset=10.0,
            )

            summary = module.compare_candidates(
                candidate_specs=[
                    f"{current[0]}:{current[1]}:{current[2]}",
                    f"{target_only[0]}:{target_only[1]}:{target_only[2]}",
                ],
                output_dir=root / "out",
            )

            self.assertEqual(summary["status"], "PASS")
            self.assertFalse(summary["promotion_samples_comparable"])
            self.assertIn("evidence only", summary["promotion_sample_comparison_note"])
            self.assertEqual(summary["selected_candidate"], "current")
            self.assertEqual(summary["selected_method"], "active_sigma060")

    def test_records_exported_profile_when_robust_recommendation_differs(self) -> None:
        module = load_module()
        with tempfile.TemporaryDirectory() as tmpdir:
            root = Path(tmpdir)
            candidate = write_candidate(
                root,
                label="mismatch",
                method="exported_runtime_profile",
                robust_method="frontier_diagnostic_method",
                robust_target_max=1.0,
                robust_rmse_mean=1.0,
            )

            summary = module.compare_candidates(
                candidate_specs=[f"{candidate[0]}:{candidate[1]}:{candidate[2]}"],
                output_dir=root / "out",
            )

            self.assertEqual(summary["status"], "PASS")
            self.assertEqual(summary["selected_method"], "exported_runtime_profile")
            row = summary["rows"][0]
            self.assertEqual(row["robust_recommendation_method"], "frontier_diagnostic_method")
            self.assertFalse(row["profile_matches_robust_recommendation"])
            self.assertIn("differs", row["method_selection_scope"])

    def test_rejects_runtime_truth_and_axis_regression_candidates(self) -> None:
        module = load_module()
        with tempfile.TemporaryDirectory() as tmpdir:
            root = Path(tmpdir)
            truth_candidate = write_candidate(
                root,
                label="truth",
                method="truth_bad",
                runtime_truth_used=True,
                robust_target_max=0.1,
            )
            axis_candidate = write_candidate(
                root,
                label="axis",
                method="axis_bad",
                robust_axis_regression=0.2,
                robust_target_max=0.2,
            )
            valid_candidate = write_candidate(
                root,
                label="valid",
                method="valid_method",
                robust_target_max=1.0,
            )

            summary = module.compare_candidates(
                candidate_specs=[
                    f"{truth_candidate[0]}:{truth_candidate[1]}:{truth_candidate[2]}",
                    f"{axis_candidate[0]}:{axis_candidate[1]}:{axis_candidate[2]}",
                    f"{valid_candidate[0]}:{valid_candidate[1]}:{valid_candidate[2]}",
                ],
                output_dir=root / "out",
            )

            self.assertEqual(summary["selected_candidate"], "valid")
            rows = {row["candidate"]: row for row in summary["rows"]}
            self.assertFalse(rows["truth"]["deployment_ready"])
            self.assertFalse(rows["axis"]["deployment_ready"])
            self.assertTrue(rows["valid"]["deployment_ready"])

    def test_reports_failure_when_no_candidate_is_ready(self) -> None:
        module = load_module()
        with tempfile.TemporaryDirectory() as tmpdir:
            root = Path(tmpdir)
            candidate = write_candidate(
                root,
                label="failed",
                method="failed_method",
                dashboard_status="FAIL",
            )

            summary = module.compare_candidates(
                candidate_specs=[f"{candidate[0]}:{candidate[1]}:{candidate[2]}"],
                output_dir=root / "out",
            )

            self.assertEqual(summary["status"], "FAIL")
            self.assertEqual(summary["selected_candidate"], "")


if __name__ == "__main__":
    unittest.main()
