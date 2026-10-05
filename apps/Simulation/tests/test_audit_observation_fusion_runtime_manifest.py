from __future__ import annotations

import importlib.util
import json
import sys
import tempfile
import unittest
from pathlib import Path


ROOT_DIR = Path(__file__).resolve().parents[1]
SCRIPT_PATH = ROOT_DIR / "scripts" / "audit_observation_fusion_runtime_manifest.py"
SPEC = importlib.util.spec_from_file_location("audit_observation_fusion_runtime_manifest_under_test", SCRIPT_PATH)
module = importlib.util.module_from_spec(SPEC)
assert SPEC is not None and SPEC.loader is not None
sys.modules[SPEC.name] = module
SPEC.loader.exec_module(module)


class AuditObservationFusionRuntimeManifestTests(unittest.TestCase):
    def test_observation_row_for_rdf_round_trips_through_camera(self) -> None:
        camera = module.RuntimeStereoCameraContract()
        row = module.observation_row_for_rdf(
            camera=camera,
            right_m=4.0,
            down_m=1.5,
            forward_m=800.0,
        )
        observation = module.RuntimeImageObservation.from_mapping(row)
        assert observation is not None
        sample = camera.triangulate_left_observation(observation)

        self.assertIsNotNone(sample)
        assert sample is not None
        self.assertAlmostEqual(sample.raw_right_m, 4.0)
        self.assertAlmostEqual(sample.raw_down_m, 1.5)
        self.assertAlmostEqual(sample.raw_forward_m, 800.0)

    def test_manifest_tier_audit_passes_repeated_and_multiburst(self) -> None:
        with tempfile.TemporaryDirectory() as tmpdir:
            path = Path(tmpdir) / "manifest.json"
            path.write_text(
                json.dumps(
                    {
                        "default_recommendation": "common_mode_target_gate",
                        "tiers": [
                            {
                                "tier": "balanced_independent",
                                "method": "repeated",
                                "adapter": {
                                    "class": "RepeatedObservationFusion",
                                    "min_observations": 3,
                                    "mode": "median",
                                    "trim_fraction": 0.2,
                                },
                            },
                            {
                                "tier": "common_mode_target_gate",
                                "method": "multiburst",
                                "adapter": {
                                    "class": "MultiBurstObservationFusion",
                                    "min_bursts": 3,
                                    "min_observations_per_burst": 5,
                                    "inner_mode": "trimmed_mean",
                                    "outer_mode": "median",
                                    "trim_fraction": 0.2,
                                },
                            },
                        ],
                    }
                ),
                encoding="utf-8",
            )
            camera = module.RuntimeStereoCameraContract()
            repeated = module.audit_tier(
                manifest_path=path,
                tier_name="balanced_independent",
                camera=camera,
                right_m=4.0,
                down_m=1.5,
                forward_m=800.0,
                tolerance_m=0.2,
            )
            multiburst = module.audit_tier(
                manifest_path=path,
                tier_name="common_mode_target_gate",
                camera=camera,
                right_m=4.0,
                down_m=1.5,
                forward_m=800.0,
                tolerance_m=0.2,
            )

        self.assertEqual(repeated["status"], "PASS")
        self.assertEqual(multiburst["status"], "PASS")
        self.assertFalse(repeated["runtime_truth_used"])
        self.assertFalse(multiburst["runtime_truth_used"])


if __name__ == "__main__":
    unittest.main()
