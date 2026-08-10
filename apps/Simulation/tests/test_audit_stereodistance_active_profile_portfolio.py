from __future__ import annotations

import importlib.util
import json
import sys
import tempfile
import unittest
from pathlib import Path


ROOT_DIR = Path(__file__).resolve().parents[1]


def load_portfolio_module():
    spec = importlib.util.spec_from_file_location(
        "audit_stereodistance_active_profile_portfolio_under_test",
        ROOT_DIR / "scripts" / "audit_stereodistance_active_profile_portfolio.py",
    )
    if spec is None or spec.loader is None:
        raise RuntimeError("cannot load audit_stereodistance_active_profile_portfolio.py")
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


class AuditStereoDistanceActiveProfilePortfolioTests(unittest.TestCase):
    def test_portfolio_ranks_fixed_profiles_by_transfer_pass_and_target_worst_case(self) -> None:
        module = load_portfolio_module()
        with tempfile.TemporaryDirectory() as tmpdir:
            root = Path(tmpdir)
            sweep_dir = root / "sweep"
            for scenario, raw_forward, truth_forward in (
                ("nominal", 100.0, 105.0),
                ("stress", 120.0, 126.0),
            ):
                seed_dir = sweep_dir / scenario / "seed_1"
                seed_dir.mkdir(parents=True)
                seed_dir.joinpath("calibration_samples.csv").write_text(
                    "split,raw_right_m,raw_down_m,raw_forward_m,truth_right_m,truth_down_m,truth_forward_m,measured_disparity_px\n"
                    f"validation,0,0,{raw_forward},0,0,{truth_forward},10\n",
                    encoding="utf-8",
                )
            good_profile = root / "good.json"
            good_profile.write_text(
                json.dumps(
                    {
                        "mode": "estimated_to_truth_disparity_poly",
                        "profile_id": "good-inverse-depth",
                        "method": "global_inverse_depth_linear",
                        "correction_space": "raw_rdf_inverse_depth_ray_scale",
                        "inverse_depth_coefficients_high_to_low": [1.0 / 1.05, 0.0],
                        "runtime_inputs": ["raw_rdf_m", "measured_disparity_px"],
                        "forbidden_runtime_inputs": ["calibration_truth", "px4_rtk_truth"],
                    }
                ),
                encoding="utf-8",
            )
            weak_profile = root / "weak.json"
            weak_profile.write_text(
                json.dumps(
                    {
                        "mode": "estimated_to_truth_sim3",
                        "profile_id": "weak-identity",
                        "scale": 1.0,
                        "rotation_matrix_rdf": [[1, 0, 0], [0, 1, 0], [0, 0, 1]],
                        "translation_rdf_m": [0, 0, 0],
                        "runtime_inputs": ["raw_rdf_m", "measured_disparity_px"],
                        "forbidden_runtime_inputs": ["calibration_truth", "px4_rtk_truth"],
                    }
                ),
                encoding="utf-8",
            )

            result = module.audit_active_profile_portfolio(
                sweep_dir=sweep_dir,
                profile_candidates=(("weak", weak_profile), ("good", good_profile)),
                output_dir=root / "out",
                split="validation",
                target_range_m=105.0,
                target_window_m=30.0,
                target_error_limit_m=10.0,
            )

            recommendation = result["recommendation"]
            self.assertTrue(recommendation["selected"])
            self.assertEqual(recommendation["recommended_profile_label"], "good")
            self.assertEqual(result["axis_contract"], module.RDF_AXIS_CONTRACT)
            self.assertTrue((root / "out" / "active_profile_portfolio_runs.csv").is_file())
            self.assertTrue((root / "out" / "active_profile_portfolio_summary.csv").is_file())
            self.assertTrue((root / "out" / "active_profile_portfolio_recommendation.json").is_file())
            self.assertTrue((root / "out" / "active_profile_portfolio_summary.png").is_file())


if __name__ == "__main__":
    unittest.main()
