from __future__ import annotations

import importlib.util
import json
import sys
import tempfile
import unittest
from pathlib import Path


ROOT_DIR = Path(__file__).resolve().parents[1]


def load_matrix_module():
    spec = importlib.util.spec_from_file_location(
        "build_policy_catalog_transfer_matrix_under_test",
        ROOT_DIR / "scripts" / "build_policy_catalog_transfer_matrix.py",
    )
    if spec is None or spec.loader is None:
        raise RuntimeError("cannot load build_policy_catalog_transfer_matrix.py")
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


class BuildPolicyCatalogTransferMatrixTests(unittest.TestCase):
    def test_builds_pair_matrix_from_policy_catalog_profiles(self) -> None:
        module = load_matrix_module()
        with tempfile.TemporaryDirectory() as tmpdir:
            root = Path(tmpdir)
            sweep = root / "sweep"
            for scenario, seed, raw_forward, truth_forward in (
                ("a", "seed_1", 100.0, 105.0),
                ("b", "seed_1", 110.0, 115.0),
            ):
                run_dir = sweep / scenario / seed
                run_dir.mkdir(parents=True)
                run_dir.joinpath("calibration_samples.csv").write_text(
                    "split,raw_right_m,raw_down_m,raw_forward_m,truth_right_m,truth_down_m,truth_forward_m,measured_disparity_px\n"
                    f"validation,0,0,{raw_forward},0,0,{truth_forward},10\n",
                    encoding="utf-8",
                )
            catalog_dir = root / "catalog"
            catalog_dir.mkdir()
            profile = catalog_dir / "a_runtime_profile.json"
            profile.write_text(
                json.dumps(
                    {
                        "mode": "estimated_to_truth_disparity_poly",
                        "profile_id": "scale",
                        "method": "global_inverse_depth_linear",
                        "correction_space": "raw_rdf_inverse_depth_ray_scale",
                        "inverse_depth_coefficients_high_to_low": [1.0 / 1.05, 0.0],
                        "runtime_inputs": ["raw_rdf_m", "measured_disparity_px"],
                        "forbidden_runtime_inputs": ["calibration_truth"],
                    }
                ),
                encoding="utf-8",
            )
            catalog = catalog_dir / "manifest.json"
            catalog.write_text(
                json.dumps(
                    {
                        "mode": "scenario_profile_catalog",
                        "profiles": [
                            {
                                "scenario": "a",
                                "method": "global_inverse_depth_linear",
                                "source_policy": "test_policy",
                                "catalog_profile_path": "a_runtime_profile.json",
                                "profile_exists": True,
                                "pass_cross_seed_replay": True,
                            }
                        ],
                    }
                ),
                encoding="utf-8",
            )
            output = root / "out"
            summary = module.build_transfer_matrix(
                sweep_dir=sweep,
                catalog_json=catalog,
                output_dir=output,
                split="validation",
                target_range_m=105.0,
                target_window_m=30.0,
                target_error_limit_m=10.0,
            )

            self.assertEqual(summary["profile_replay_count"], 2)
            self.assertEqual(summary["profile_pair_count"], 2)
            self.assertTrue((output / "policy_catalog_transfer_replay.csv").is_file())
            matrix_text = (output / "policy_catalog_transfer_matrix.csv").read_text(encoding="utf-8")
            self.assertIn("max_target_abs_error_corrected_m", matrix_text)
            self.assertTrue((output / "policy_catalog_transfer_summary.json").is_file())


if __name__ == "__main__":
    unittest.main()
