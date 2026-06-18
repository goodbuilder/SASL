from __future__ import annotations

import csv
import importlib.util
import json
import sys
import tempfile
import unittest
from pathlib import Path


ROOT_DIR = Path(__file__).resolve().parents[1]
SCRIPT_PATH = ROOT_DIR / "scripts" / "export_stereodistance_public_topic_payload_from_profile.py"


def load_module():
    spec = importlib.util.spec_from_file_location(
        "export_stereodistance_public_topic_payload_from_profile_under_test",
        SCRIPT_PATH,
    )
    if spec is None or spec.loader is None:
        raise RuntimeError(f"cannot load {SCRIPT_PATH}")
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


class ExportStereoDistancePublicTopicPayloadFromProfileTests(unittest.TestCase):
    def write_profile(self, path: Path) -> None:
        path.write_text(
            json.dumps(
                {
                    "profile_id": "unit",
                    "method": "sim3",
                    "mode": "estimated_to_truth_sim3",
                    "axis_contract": "RDF(right,down,forward)",
                    "runtime_inputs": ["raw_right_m", "raw_down_m", "raw_forward_m", "raw_disparity_px"],
                    "forbidden_runtime_inputs": ["calibration_truth", "px4_rtk_truth"],
                    "scale": 1.0,
                    "rotation_matrix_rdf": [
                        [1.0, 0.0, 0.0],
                        [0.0, 1.0, 0.0],
                        [0.0, 0.0, 1.0],
                    ],
                    "translation_rdf_m": [1.0, 2.0, 3.0],
                }
            ),
            encoding="utf-8",
        )

    def write_samples(self, path: Path) -> None:
        path.write_text(
            "\n".join(
                [
                    "split,raw_right_m,raw_down_m,raw_forward_m,measured_disparity_px,left_u_px,left_v_px",
                    "validation,10.0,20.0,30.0,5.0,640.0,360.0",
                    "train,1.0,2.0,3.0,4.0,640.0,360.0",
                ]
            )
            + "\n",
            encoding="utf-8",
        )

    def test_exports_public_frd_range_payload(self) -> None:
        module = load_module()
        with tempfile.TemporaryDirectory() as tmpdir:
            root = Path(tmpdir)
            profile = root / "active_runtime_profile.json"
            samples = root / "samples.csv"
            output_dir = root / "public_payload"
            self.write_profile(profile)
            self.write_samples(samples)

            summary = module.export_payloads(
                profile_json=profile,
                sample_csv=samples,
                output_dir=output_dir,
                split="validation",
                max_samples=5,
            )
            csv_path = output_dir / "public_corrected_position_payload.csv"
            with csv_path.open(encoding="utf-8") as handle:
                rows = list(csv.DictReader(handle))

        self.assertEqual(summary["status"], "PASS")
        self.assertEqual(summary["sample_count"], 1)
        self.assertEqual(summary["invalid_count"], 0)
        self.assertEqual(summary["node_public_payload_equivalence_status"], "PASS")
        self.assertEqual(summary["node_float32_equivalence_tolerance_m"], 1.0e-3)
        self.assertFalse(summary["runtime_truth_used"])
        self.assertEqual(rows[0]["topic"], "/stereo/corrected_position")
        self.assertEqual(rows[0]["data_axis_order"], "forward_m,right_m,down_m,range_m")
        self.assertEqual(rows[0]["node_public_payload_equivalent"], "True")
        self.assertEqual(float(rows[0]["node_float32_equivalence_tolerance_m"]), 1.0e-3)
        data = json.loads(rows[0]["data"])
        self.assertEqual(data[0], float(rows[0]["public_forward_m"]))
        self.assertEqual(data[1], float(rows[0]["public_right_m"]))
        self.assertEqual(data[2], float(rows[0]["public_down_m"]))
        self.assertEqual(data[3], float(rows[0]["public_range_m"]))
        self.assertIn("RDF(right,down,forward)", rows[0]["input_coordinate_contract"])
        self.assertIn("[forward_m,right_m,down_m,range_m]", rows[0]["public_topic_contract"])

    def test_missing_validation_samples_fails(self) -> None:
        module = load_module()
        with tempfile.TemporaryDirectory() as tmpdir:
            root = Path(tmpdir)
            profile = root / "active_runtime_profile.json"
            samples = root / "samples.csv"
            self.write_profile(profile)
            self.write_samples(samples)

            summary = module.export_payloads(
                profile_json=profile,
                sample_csv=samples,
                output_dir=root / "public_payload",
                split="missing",
                max_samples=5,
            )

        self.assertEqual(summary["status"], "FAIL")
        self.assertEqual(summary["sample_count"], 0)


if __name__ == "__main__":
    unittest.main()
