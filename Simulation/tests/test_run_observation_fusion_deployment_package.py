from __future__ import annotations

import importlib.util
import sys
import unittest
from pathlib import Path


ROOT_DIR = Path(__file__).resolve().parents[1]
SCRIPT_PATH = ROOT_DIR / "scripts" / "run_observation_fusion_deployment_package.py"
SPEC = importlib.util.spec_from_file_location("run_observation_fusion_deployment_package_under_test", SCRIPT_PATH)
module = importlib.util.module_from_spec(SPEC)
assert SPEC is not None and SPEC.loader is not None
sys.modules[SPEC.name] = module
SPEC.loader.exec_module(module)


class RunObservationFusionDeploymentPackageTests(unittest.TestCase):
    def test_synthetic_multiburst_rows_have_default_shape_and_contract_fields(self) -> None:
        rows = module.synthetic_multiburst_rows(camera=module.RuntimeStereoCameraContract())

        self.assertEqual(len(rows), 15)
        self.assertEqual({row["group_id"] for row in rows}, {"sample0"})
        self.assertEqual({row["burst_id"] for row in rows}, {"b0", "b1", "b2"})
        self.assertTrue(all("left_u_px" in row for row in rows))
        self.assertTrue(all("left_v_px" in row for row in rows))
        self.assertTrue(all("measured_disparity_px" in row for row in rows))
        self.assertTrue(all(float(row["measured_disparity_px"]) > 0.0 for row in rows))

    def test_synthetic_multiburst_rows_accept_fresh_b5x5_shape(self) -> None:
        rows = module.synthetic_multiburst_rows(
            camera=module.RuntimeStereoCameraContract(),
            burst_count=5,
            observations_per_burst=5,
        )

        self.assertEqual(len(rows), 25)
        self.assertEqual({row["burst_id"] for row in rows}, {"b0", "b1", "b2", "b3", "b4"})

    def test_default_smoke_shape_uses_manifest_default_adapter(self) -> None:
        manifest = {
            "default_recommendation": "fresh_common_mode_b7x3_aggregate_best",
            "tiers": [
                {
                    "tier": "fresh_common_mode_b7x3_aggregate_best",
                    "adapter": {
                        "class": "MultiBurstObservationFusion",
                        "min_bursts": 7,
                        "min_observations_per_burst": 3,
                    },
                }
            ],
        }

        self.assertEqual(module.default_smoke_shape(manifest), (7, 3))

    def test_synthetic_multiburst_rows_pass_input_schema_gate(self) -> None:
        rows = module.synthetic_multiburst_rows(camera=module.RuntimeStereoCameraContract())
        columns = list(rows[0].keys())

        result = module.input_validator.validate_rows(
            columns=columns,
            rows=rows,
            require_burst_id=True,
        )

        self.assertEqual(result["status"], "PASS")
        self.assertEqual(result["burst_count_by_group"], {"sample0": 3})

    def test_synthetic_bbox_rows_convert_to_fusion_input(self) -> None:
        rows = module.synthetic_multiburst_rows(camera=module.RuntimeStereoCameraContract())
        bbox_rows = module.synthetic_bbox_rows(rows)

        converted, failures = module.bbox_converter.convert_rows(
            bbox_rows,
            columns=list(bbox_rows[0].keys()),
        )

        self.assertEqual(failures, [])
        self.assertEqual(len(converted), len(rows))
        self.assertAlmostEqual(float(converted[0]["left_u_px"]), float(rows[0]["left_u_px"]))
        self.assertAlmostEqual(
            float(converted[0]["measured_disparity_px"]),
            float(rows[0]["measured_disparity_px"]),
        )

    def test_synthetic_calibration_rows_drop_truth_for_runtime_input(self) -> None:
        rows = module.synthetic_multiburst_rows(camera=module.RuntimeStereoCameraContract())
        calibration_rows = module.synthetic_calibration_rows(rows)

        extracted, failures, dropped = module.calibration_extractor.extract_rows(
            calibration_rows,
            columns=list(calibration_rows[0].keys()),
        )

        self.assertEqual(failures, [])
        self.assertEqual(len(extracted), len(rows))
        self.assertIn("truth_north_m", dropped)
        self.assertFalse(extracted[0]["runtime_truth_used"])
        self.assertNotIn("truth_north_m", extracted[0])

    def test_topic_contract_manifest_marks_truth_offline_only(self) -> None:
        manifest = module.topic_contract_export.build_manifest(module.topic_contract_export.DEFAULT_TOPICS)

        self.assertFalse(manifest["runtime_truth_used"])
        self.assertNotIn("calibration_truth", manifest["runtime_input_interfaces"])
        self.assertIn("stereo_bbox_observation", manifest["runtime_input_interfaces"])

    def test_package_acceptance_checker_rejects_truth_runtime_topic(self) -> None:
        import json
        import tempfile

        with tempfile.TemporaryDirectory() as tmpdir:
            root = Path(tmpdir)
            checker_test = module.acceptance_check
            from Simulation.tests.test_check_observation_fusion_deployment_package import write_json
            from Simulation.tests.test_check_observation_fusion_deployment_package import (
                CheckObservationFusionDeploymentPackageTests,
            )

            CheckObservationFusionDeploymentPackageTests().write_minimal_package(root)
            path = root / "topic_contract" / "observation_fusion_topic_contract_manifest.json"
            payload = json.loads(path.read_text(encoding="utf-8"))
            payload["runtime_input_interfaces"].append("simulation_truth")
            write_json(path, payload)

            result = checker_test.check_package(root)

        self.assertEqual(result["status"], "FAIL")
        self.assertIn("topic_contract:simulation_truth_marked_runtime_input", result["failures"])

    def test_write_rows_round_trips_csv(self) -> None:
        import csv
        import tempfile

        with tempfile.TemporaryDirectory() as tmpdir:
            path = Path(tmpdir) / "rows.csv"
            module.write_rows(path, [{"group_id": "sample0", "left_u_px": 1.0}])
            with path.open(newline="", encoding="utf-8") as handle:
                rows = list(csv.DictReader(handle))

        self.assertEqual(rows[0]["group_id"], "sample0")
        self.assertEqual(rows[0]["left_u_px"], "1.0")


if __name__ == "__main__":
    unittest.main()
