from __future__ import annotations

import importlib.util
import sys
import unittest
from pathlib import Path


ROOT_DIR = Path(__file__).resolve().parents[1]
SCRIPT_PATH = ROOT_DIR / "scripts" / "export_observation_fusion_topic_contract_manifest.py"
SPEC = importlib.util.spec_from_file_location("export_observation_fusion_topic_contract_manifest_under_test", SCRIPT_PATH)
module = importlib.util.module_from_spec(SPEC)
assert SPEC is not None and SPEC.loader is not None
sys.modules[SPEC.name] = module
SPEC.loader.exec_module(module)


class ExportObservationFusionTopicContractManifestTests(unittest.TestCase):
    def test_manifest_separates_runtime_inputs_from_truth(self) -> None:
        manifest = module.build_manifest(module.DEFAULT_TOPICS)

        self.assertFalse(manifest["runtime_truth_used"])
        runtime_inputs = set(manifest["runtime_input_interfaces"])
        self.assertIn("stereo_bbox_observation", runtime_inputs)
        self.assertIn("measured_disparity", runtime_inputs)
        self.assertNotIn("calibration_truth", runtime_inputs)
        self.assertNotIn("simulation_truth", runtime_inputs)
        self.assertEqual(manifest["frames"]["internal_measurement"], "stereo-center RDF right/down/forward")
        self.assertEqual(manifest["frames"]["public_stereodistance"], "FRD+range forward/right/down/range")
        self.assertIn("T_source_to_stereo_rdf", manifest["frames"]["px4_truth"])

        truth = next(row for row in manifest["interfaces"] if row["interface"] == "calibration_truth")
        self.assertEqual(truth["output_fields"], "truth_north_m,truth_east_m,truth_down_ned_m")
        self.assertIn("T_source_to_stereo_rdf", truth["contract"])
        self.assertIn("axis aliasing is forbidden", truth["contract"])
        self.assertNotIn("right=east", truth["contract"])

    def test_bbox_contract_outputs_fusion_input_fields(self) -> None:
        rows = module.topic_contract_rows(module.DEFAULT_TOPICS)
        bbox = next(row for row in rows if row["interface"] == "stereo_bbox_observation")

        self.assertTrue(bbox["runtime_input"])
        self.assertEqual(bbox["topic"], "/stereo/detections")
        self.assertIn("left_u_px", bbox["output_fields"])
        self.assertIn("measured_disparity_px", bbox["output_fields"])
        self.assertIn("left_u_px-right_u_px", bbox["contract"])


if __name__ == "__main__":
    unittest.main()
