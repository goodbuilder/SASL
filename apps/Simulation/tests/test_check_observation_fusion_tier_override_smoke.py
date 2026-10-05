from __future__ import annotations

import importlib.util
import json
import sys
import tempfile
import unittest
from pathlib import Path


ROOT_DIR = Path(__file__).resolve().parents[1]
SCRIPT_PATH = ROOT_DIR / "scripts" / "check_observation_fusion_tier_override_smoke.py"
SPEC = importlib.util.spec_from_file_location("check_observation_fusion_tier_override_smoke_under_test", SCRIPT_PATH)
module = importlib.util.module_from_spec(SPEC)
assert SPEC is not None and SPEC.loader is not None
sys.modules[SPEC.name] = module
SPEC.loader.exec_module(module)


def write_manifest(path: Path) -> None:
    payload = {
        "default_recommendation": "unit_multiburst",
        "runtime_truth_used": False,
        "tiers": [
            {
                "tier": "unit_multiburst",
                "method": "unit",
                "adapter": {
                    "class": "MultiBurstObservationFusion",
                    "min_bursts": 2,
                    "min_observations_per_burst": 3,
                    "inner_mode": "median",
                    "outer_mode": "median",
                    "trim_fraction": 0.0,
                },
            },
            {
                "tier": "unit_repeated",
                "method": "unit",
                "adapter": {
                    "class": "RepeatedObservationFusion",
                    "min_observations": 3,
                    "mode": "median",
                    "trim_fraction": 0.0,
                },
            },
        ],
    }
    path.write_text(json.dumps(payload), encoding="utf-8")


class CheckObservationFusionTierOverrideSmokeTests(unittest.TestCase):
    def test_smokes_every_manifest_tier(self) -> None:
        with tempfile.TemporaryDirectory() as tmpdir:
            root = Path(tmpdir)
            manifest = root / "manifest.json"
            write_manifest(manifest)
            rows = module.smoke_rows(manifest_json=manifest)
            payload = module.summary_payload(manifest_json=manifest, profile_json=None, rows=rows)

        self.assertEqual(payload["status"], "PASS")
        self.assertEqual(payload["tier_count"], 2)
        self.assertEqual(payload["pass_count"], 2)
        self.assertEqual(payload["default_tier_count"], 1)
        self.assertFalse(payload["runtime_truth_used"])
        self.assertEqual({row["adapter_class"] for row in rows}, {"MultiBurstObservationFusion", "RepeatedObservationFusion"})

    def test_rejects_missing_default_row(self) -> None:
        rows = [
            {
                "tier": "other",
                "is_default": False,
                "status": "PASS",
                "failure_count": 0,
                "runtime_truth_used": False,
            }
        ]

        payload = module.summary_payload(manifest_json=Path("manifest.json"), profile_json=None, rows=rows)

        self.assertEqual(payload["status"], "FAIL")
        self.assertIn("default_tier_count:not_one", payload["failures"])


if __name__ == "__main__":
    unittest.main()
