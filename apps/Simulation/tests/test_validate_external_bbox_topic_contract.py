from __future__ import annotations

import importlib.util
import sys
import tempfile
import unittest
from pathlib import Path


ROOT_DIR = Path(__file__).resolve().parents[1]
SCRIPT_PATH = ROOT_DIR / "scripts" / "validate_external_bbox_topic_contract.py"


def load_module():
    spec = importlib.util.spec_from_file_location(
        "validate_external_bbox_topic_contract_under_test",
        SCRIPT_PATH,
    )
    if spec is None or spec.loader is None:
        raise RuntimeError(f"cannot load {SCRIPT_PATH}")
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


class ValidateExternalBBoxTopicContractTests(unittest.TestCase):
    def test_passes_valid_float32_multi_array_values(self) -> None:
        module = load_module()
        values = [
            1,
            650,
            340,
            750,
            394,
            0.9,
            700,
            367,
            1,
            478,
            340,
            578,
            394,
            0.8,
            528,
            367,
        ]

        payload = module.validate_values(values=values)

        self.assertEqual(payload["status"], "PASS")
        self.assertEqual(payload["valid_pair_count"], 1)
        self.assertEqual(payload["disparity_min_px"], 172.0)
        self.assertFalse(payload["runtime_truth_used"])
        self.assertIn("stereo-center RDF", payload["coordinate_contract"])

    def test_fails_reversed_disparity(self) -> None:
        module = load_module()
        values = [
            1,
            450,
            340,
            550,
            394,
            0.9,
            500,
            367,
            1,
            650,
            340,
            750,
            394,
            0.8,
            700,
            367,
        ]

        payload = module.validate_values(values=values)

        self.assertEqual(payload["status"], "FAIL")
        self.assertIn("row0:left_minus_right_disparity_below_min:-200", payload["failures"])

    def test_csv_forbids_truth_columns(self) -> None:
        module = load_module()
        with tempfile.TemporaryDirectory() as tmpdir:
            path = Path(tmpdir) / "bbox.csv"
            path.write_text(
                ",".join(module.FIELD_NAMES + ("ground_truth_m",))
                + "\n"
                + "1,650,340,750,394,0.9,700,367,1,478,340,578,394,0.8,528,367,1000\n",
                encoding="utf-8",
            )

            payload = module.validate_csv(input_csv=path)

        self.assertEqual(payload["status"], "FAIL")
        self.assertIn("forbidden_truth_column:ground_truth_m", payload["failures"])
        self.assertEqual(payload["valid_pair_count"], 1)

    def test_csv_accepts_single_data_column(self) -> None:
        module = load_module()
        with tempfile.TemporaryDirectory() as tmpdir:
            path = Path(tmpdir) / "bbox_data.csv"
            path.write_text(
                "data\n"
                "\"1 650 340 750 394 0.9 700 367 1 478 340 578 394 0.8 528 367\"\n",
                encoding="utf-8",
            )

            payload = module.validate_csv(input_csv=path)

        self.assertEqual(payload["status"], "PASS")
        self.assertEqual(payload["valid_pair_count"], 1)


if __name__ == "__main__":
    unittest.main()
