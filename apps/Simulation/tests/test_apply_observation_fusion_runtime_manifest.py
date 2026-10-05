from __future__ import annotations

import csv
import importlib.util
import json
import sys
import tempfile
import unittest
from pathlib import Path


ROOT_DIR = Path(__file__).resolve().parents[1]
SCRIPT_PATH = ROOT_DIR / "scripts" / "apply_observation_fusion_runtime_manifest.py"
SPEC = importlib.util.spec_from_file_location("apply_observation_fusion_runtime_manifest_under_test", SCRIPT_PATH)
module = importlib.util.module_from_spec(SPEC)
assert SPEC is not None and SPEC.loader is not None
sys.modules[SPEC.name] = module
SPEC.loader.exec_module(module)


class ApplyObservationFusionRuntimeManifestTests(unittest.TestCase):
    def manifest_path(self, tmpdir: str) -> Path:
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
                                "min_observations_per_burst": 2,
                                "inner_mode": "median",
                                "outer_mode": "median",
                                "trim_fraction": 0.2,
                            },
                        },
                    ],
                }
            ),
            encoding="utf-8",
        )
        return path

    def row_for_rdf(self, *, right_m: float, down_m: float, forward_m: float) -> dict[str, float]:
        camera = module.RuntimeStereoCameraContract()
        disparity_px = camera.fx_px * camera.baseline_m / forward_m
        left_u_px = camera.cx_px + (right_m + camera.baseline_m * 0.5) * camera.fx_px / forward_m
        left_v_px = camera.cy_px + down_m * camera.fy_px / forward_m
        return {
            "left_u_px": left_u_px,
            "left_v_px": left_v_px,
            "measured_disparity_px": disparity_px,
        }

    def test_apply_repeated_tier_to_grouped_rows(self) -> None:
        with tempfile.TemporaryDirectory() as tmpdir:
            manifest = self.manifest_path(tmpdir)
            adapter, tier = module.load_observation_fusion_adapter_from_manifest(
                manifest,
                tier="balanced_independent",
            )
            base = self.row_for_rdf(right_m=4.0, down_m=1.5, forward_m=800.0)
            rows = [
                {"group_id": "g0", **base},
                {"group_id": "g0", **base},
                {"group_id": "g0", **base},
            ]

            output = module.apply_adapter_to_groups(
                adapter=adapter,
                tier_payload=tier,
                rows=rows,
                group_field="group_id",
                burst_field="burst_id",
            )

        self.assertEqual(len(output), 1)
        self.assertTrue(output[0]["sample_valid"])
        self.assertAlmostEqual(float(output[0]["raw_right_m"]), 4.0)
        self.assertAlmostEqual(float(output[0]["raw_down_m"]), 1.5)
        self.assertAlmostEqual(float(output[0]["raw_forward_m"]), 800.0)

    def test_apply_multiburst_tier_to_grouped_rows(self) -> None:
        with tempfile.TemporaryDirectory() as tmpdir:
            manifest = self.manifest_path(tmpdir)
            adapter, tier = module.load_observation_fusion_adapter_from_manifest(manifest)
            base = self.row_for_rdf(right_m=4.0, down_m=1.5, forward_m=800.0)
            rows = []
            for burst_id in ("b0", "b1", "b2"):
                rows.append({"group_id": "g0", "burst_id": burst_id, **base})
                rows.append({"group_id": "g0", "burst_id": burst_id, **base})

            output = module.apply_adapter_to_groups(
                adapter=adapter,
                tier_payload=tier,
                rows=rows,
                group_field="group_id",
                burst_field="burst_id",
            )

        self.assertEqual(len(output), 1)
        self.assertTrue(output[0]["sample_valid"])
        self.assertEqual(output[0]["adapter_class"], "MultiBurstObservationFusion")
        self.assertAlmostEqual(float(output[0]["raw_forward_m"]), 800.0)

    def test_write_csv_outputs_rows(self) -> None:
        with tempfile.TemporaryDirectory() as tmpdir:
            path = Path(tmpdir) / "out.csv"
            module.write_csv(path, [{"group_id": "g0", "sample_valid": True}])
            with path.open(newline="", encoding="utf-8") as handle:
                rows = list(csv.DictReader(handle))

        self.assertEqual(rows[0]["group_id"], "g0")
        self.assertEqual(rows[0]["sample_valid"], "True")


if __name__ == "__main__":
    unittest.main()
