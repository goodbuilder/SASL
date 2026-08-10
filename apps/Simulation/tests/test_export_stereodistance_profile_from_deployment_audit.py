from __future__ import annotations

import importlib.util
import json
import sys
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch


ROOT_DIR = Path(__file__).resolve().parents[1]


def load_export_module():
    spec = importlib.util.spec_from_file_location(
        "export_stereodistance_profile_from_deployment_audit_under_test",
        ROOT_DIR / "scripts" / "export_stereodistance_profile_from_deployment_audit.py",
    )
    if spec is None or spec.loader is None:
        raise RuntimeError("cannot load export_stereodistance_profile_from_deployment_audit.py")
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def write_json(path: Path, payload: object) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(payload, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")


class ExportStereoDistanceProfileFromDeploymentAuditTests(unittest.TestCase):
    def test_blend_recommendation_exports_policy_and_runs_promotion_check(self) -> None:
        module = load_export_module()
        with tempfile.TemporaryDirectory() as tmpdir:
            root = Path(tmpdir)
            recommendation = root / "recommendation.json"
            write_json(
                recommendation,
                {
                    "recommended_strategy": module.BLEND_POLICY_STRATEGY,
                    "recommended_policy": "balanced",
                },
            )
            output_profile = root / "active.json"

            def fake_explore_blends(**kwargs):
                kwargs["output_profile"].write_text('{"mode": "estimated_to_truth_sim3"}\n', encoding="utf-8")
                return {
                    "validation_range_rmse_corrected_m": 1.2,
                    "target_abs_mean_error_corrected_m": 0.3,
                    "target_max_abs_error_corrected_m": 0.7,
                    "replay_selection_score": 1.4,
                }

            with patch.object(module, "explore_blends", side_effect=fake_explore_blends) as explore, patch.object(
                module,
                "select_by_calibration_replay",
            ) as select, patch.object(
                module,
                "run_check",
                return_value={"pass_promotion_check": True},
            ) as promotion:
                summary = module.export_profile_from_deployment_audit(
                    deployment_audit_json=recommendation,
                    catalog_json=root / "catalog.json",
                    input_csv=root / "calibration.csv",
                    output_profile=output_profile,
                    output_dir=root / "out",
                    candidate_source_mode="from_recommendation",
                    selection_policy="from_recommendation",
                )

            self.assertTrue(explore.called)
            self.assertFalse(select.called)
            self.assertTrue(promotion.called)
            self.assertEqual(explore.call_args.kwargs["candidate_source_mode"], "all_catalog")
            self.assertEqual(explore.call_args.kwargs["selection_policy"], "balanced")
            self.assertEqual(summary["selected_strategy"], module.BLEND_POLICY_STRATEGY)
            self.assertEqual(summary["selected_policy"], "balanced")
            self.assertTrue(summary["promotion_check_passed"])
            self.assertEqual(summary["axis_contract"], module.RDF_AXIS_CONTRACT)
            self.assertTrue((root / "out" / "deployment_audit_profile_export_summary.json").is_file())

    def test_single_profile_recommendation_uses_calibration_replay_selector(self) -> None:
        module = load_export_module()
        with tempfile.TemporaryDirectory() as tmpdir:
            root = Path(tmpdir)
            recommendation = root / "recommendation.json"
            write_json(
                recommendation,
                {
                    "recommended_strategy": module.SINGLE_PROFILE_STRATEGY,
                    "recommended_policy": "single_best",
                },
            )

            def fake_select(**kwargs):
                kwargs["output_profile"].write_text('{"mode": "estimated_to_truth_sim3"}\n', encoding="utf-8")
                return {
                    "validation_range_rmse_corrected_m": 0.9,
                    "target_abs_mean_error_corrected_m": 0.2,
                    "target_max_abs_error_corrected_m": 0.4,
                    "replay_selection_score": 1.0,
                }

            with patch.object(module, "select_by_calibration_replay", side_effect=fake_select) as select, patch.object(
                module,
                "explore_blends",
            ) as explore:
                summary = module.export_profile_from_deployment_audit(
                    deployment_audit_json=recommendation,
                    catalog_json=root / "catalog.json",
                    input_csv=root / "calibration.csv",
                    output_profile=root / "active.json",
                    output_dir=root / "out",
                    candidate_source_mode="minimal_set",
                    run_promotion=False,
                )

            self.assertTrue(select.called)
            self.assertFalse(explore.called)
            self.assertEqual(select.call_args.kwargs["candidate_source_mode"], "minimal_set")
            self.assertEqual(summary["selected_policy"], "single_best")
            self.assertEqual(summary["promotion_check_passed"], "")
            self.assertIn("no runtime truth", summary["runtime_input_contract"])

    def test_minimal_set_recommendation_derives_candidate_source_mode(self) -> None:
        module = load_export_module()
        with tempfile.TemporaryDirectory() as tmpdir:
            root = Path(tmpdir)
            recommendation = root / "recommendation.json"
            write_json(
                recommendation,
                {
                    "recommended_strategy": module.SINGLE_PROFILE_MINIMAL_SET_STRATEGY,
                    "recommended_policy": "single_best",
                },
            )

            def fake_select(**kwargs):
                kwargs["output_profile"].write_text('{"mode": "estimated_to_truth_sim3"}\n', encoding="utf-8")
                return {
                    "validation_range_rmse_corrected_m": 0.9,
                    "target_abs_mean_error_corrected_m": 0.2,
                    "target_max_abs_error_corrected_m": 0.4,
                    "replay_selection_score": 1.0,
                }

            with patch.object(module, "select_by_calibration_replay", side_effect=fake_select) as select:
                summary = module.export_profile_from_deployment_audit(
                    deployment_audit_json=recommendation,
                    catalog_json=root / "catalog.json",
                    input_csv=root / "calibration.csv",
                    output_profile=root / "active.json",
                    output_dir=root / "out",
                    candidate_source_mode="from_recommendation",
                    run_promotion=False,
                )

            self.assertEqual(select.call_args.kwargs["candidate_source_mode"], "minimal_set")
            self.assertEqual(summary["selected_strategy"], module.SINGLE_PROFILE_MINIMAL_SET_STRATEGY)
            self.assertEqual(summary["candidate_source_mode"], "minimal_set")


if __name__ == "__main__":
    unittest.main()
