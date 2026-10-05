from __future__ import annotations

import importlib.util
import json
import sys
import tempfile
import unittest
from pathlib import Path


ROOT_DIR = Path(__file__).resolve().parents[1]


def load_transfer_module():
    spec = importlib.util.spec_from_file_location(
        "audit_stereodistance_active_profile_transfer_under_test",
        ROOT_DIR / "scripts" / "audit_stereodistance_active_profile_transfer.py",
    )
    if spec is None or spec.loader is None:
        raise RuntimeError("cannot load audit_stereodistance_active_profile_transfer.py")
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


class AuditStereoDistanceActiveProfileTransferTests(unittest.TestCase):
    def test_audit_active_profile_transfer_replays_every_seed_csv(self) -> None:
        module = load_transfer_module()
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
            profile_json = root / "profile.json"
            profile_json.write_text(
                json.dumps(
                    {
                        "mode": "estimated_to_truth_disparity_poly",
                        "profile_id": "identity-inverse-depth",
                        "method": "global_inverse_depth_linear",
                        "correction_space": "raw_rdf_inverse_depth_ray_scale",
                        "inverse_depth_coefficients_high_to_low": [1.0 / 1.05, 0.0],
                        "runtime_inputs": ["raw_rdf_m", "measured_disparity_px"],
                        "forbidden_runtime_inputs": [
                            "calibration_truth",
                            "px4_rtk_truth",
                            "true_disparity",
                            "true_image_feature_location",
                        ],
                    }
                ),
                encoding="utf-8",
            )

            summary = module.audit_active_profile_transfer(
                sweep_dir=sweep_dir,
                profile_json=profile_json,
                output_dir=root / "out",
                split="validation",
                target_range_m=105.0,
                target_window_m=30.0,
                target_error_limit_m=10.0,
            )

            self.assertEqual(summary["transfer_replay_count"], 2)
            self.assertEqual(summary["transfer_replay_pass_count"], 2)
            self.assertTrue(summary["pass_all_transfer_replays"])
            self.assertEqual(summary["axis_contract"], module.RDF_AXIS_CONTRACT)
            self.assertTrue((root / "out" / "active_profile_transfer_replay.csv").is_file())
            self.assertTrue((root / "out" / "active_profile_transfer_summary.json").is_file())
            self.assertTrue((root / "out" / "active_profile_transfer_summary.png").is_file())

            filtered = module.audit_active_profile_transfer(
                sweep_dir=sweep_dir,
                profile_json=profile_json,
                output_dir=root / "out_filtered",
                split="validation",
                target_range_m=105.0,
                target_window_m=30.0,
                target_error_limit_m=10.0,
                target_scenarios=["stress"],
            )

            self.assertEqual(filtered["target_scenarios"], "stress")
            self.assertEqual(filtered["transfer_replay_count"], 1)
            filtered_text = (root / "out_filtered" / "active_profile_transfer_replay.csv").read_text(
                encoding="utf-8"
            )
            self.assertIn("stress", filtered_text)
            self.assertNotIn("nominal", filtered_text)


if __name__ == "__main__":
    unittest.main()
