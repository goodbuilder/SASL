from __future__ import annotations

import importlib.util
import sys
import unittest
from pathlib import Path


ROOT_DIR = Path(__file__).resolve().parents[1]
SCRIPT_PATH = ROOT_DIR / "scripts" / "apply_observation_fusion_with_runtime_profile.py"
SPEC = importlib.util.spec_from_file_location("apply_observation_fusion_with_runtime_profile_under_test", SCRIPT_PATH)
module = importlib.util.module_from_spec(SPEC)
assert SPEC is not None and SPEC.loader is not None
sys.modules[SPEC.name] = module
SPEC.loader.exec_module(module)


class FakeProfileResult:
    def __init__(self) -> None:
        self.corrected = {"right_m": 5.0, "down_m": 2.0, "forward_m": 900.0}
        self.status = {"fallback_level": 0, "fallback_reason": ""}


class FakeProfile:
    def apply(self, payload: dict[str, float]) -> FakeProfileResult:
        self.last_payload = payload
        return FakeProfileResult()


class ApplyObservationFusionWithRuntimeProfileTests(unittest.TestCase):
    def test_rdf_to_public_frd_reorders_axes(self) -> None:
        public = module.rdf_to_public_frd(right_m=5.0, down_m=2.0, forward_m=900.0)

        self.assertAlmostEqual(public["forward_m"], 900.0)
        self.assertAlmostEqual(public["right_m"], 5.0)
        self.assertAlmostEqual(public["down_m"], 2.0)
        self.assertAlmostEqual(public["range_m"], (5.0**2 + 2.0**2 + 900.0**2) ** 0.5)

    def test_profile_payload_from_fused_row(self) -> None:
        payload = module.profile_payload_from_fused_row(
            {
                "raw_right_m": "4.0",
                "raw_down_m": "1.5",
                "raw_forward_m": "800.0",
                "measured_disparity_px": "257.5",
                "left_u_px": "810.0",
                "left_v_px": "375.0",
            }
        )

        assert payload is not None
        self.assertAlmostEqual(payload["right_m"], 4.0)
        self.assertAlmostEqual(payload["range_m"], (4.0**2 + 1.5**2 + 800.0**2) ** 0.5)
        self.assertAlmostEqual(payload["disparity_px"], 257.5)

    def test_apply_profile_to_fused_rows_adds_corrected_rdf(self) -> None:
        profile = FakeProfile()
        rows = [
            {
                "group_id": "g0",
                "sample_valid": True,
                "raw_right_m": 4.0,
                "raw_down_m": 1.5,
                "raw_forward_m": 800.0,
                "measured_disparity_px": 257.5,
                "left_u_px": 810.0,
                "left_v_px": 375.0,
            }
        ]

        output = module.apply_profile_to_fused_rows(profile=profile, fused_rows=rows)

        self.assertTrue(output[0]["profile_valid"])
        self.assertAlmostEqual(output[0]["corrected_right_m"], 5.0)
        self.assertAlmostEqual(output[0]["corrected_down_m"], 2.0)
        self.assertAlmostEqual(output[0]["corrected_forward_m"], 900.0)
        self.assertAlmostEqual(output[0]["raw_public_forward_m"], 800.0)
        self.assertAlmostEqual(output[0]["raw_public_right_m"], 4.0)
        self.assertAlmostEqual(output[0]["raw_public_down_m"], 1.5)
        self.assertAlmostEqual(output[0]["corrected_public_forward_m"], 900.0)
        self.assertAlmostEqual(output[0]["corrected_public_right_m"], 5.0)
        self.assertAlmostEqual(output[0]["corrected_public_down_m"], 2.0)
        self.assertFalse(output[0]["runtime_truth_used"])
        self.assertEqual(profile.last_payload["forward_m"], 800.0)

    def test_invalid_fused_row_is_marked_invalid(self) -> None:
        output = module.apply_profile_to_fused_rows(profile=FakeProfile(), fused_rows=[{"group_id": "g0"}])

        self.assertFalse(output[0]["profile_valid"])
        self.assertEqual(output[0]["profile_fallback_reason"], "invalid_fused_payload")


if __name__ == "__main__":
    unittest.main()
