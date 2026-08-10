from __future__ import annotations

import importlib.util
import csv
import json
import os
import sys
import tempfile
import unittest
from pathlib import Path


ROOT_DIR = Path(__file__).resolve().parents[1]
SCRIPT_PATH = ROOT_DIR / "scripts" / "run_operator_latest_telemetry_residual_gate.py"


def load_module():
    spec = importlib.util.spec_from_file_location("run_operator_latest_telemetry_residual_gate_under_test", SCRIPT_PATH)
    if spec is None or spec.loader is None:
        raise RuntimeError(f"cannot load {SCRIPT_PATH}")
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


class RunOperatorLatestTelemetryResidualGateTests(unittest.TestCase):
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
                    "rotation_matrix_rdf": [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]],
                    "translation_rdf_m": [0.0, 0.0, 0.0],
                }
            )
            + "\n",
            encoding="utf-8",
        )

    def write_selector(self, root: Path) -> tuple[Path, Path]:
        profile = root / "profiles" / "unit.json"
        profile.parent.mkdir(parents=True, exist_ok=True)
        profile.write_text(
            json.dumps(
                {
                    "profile_type": "DisparityResidualRuntimeProfile",
                    "profile_id": "unit",
                    "runtime_inputs": ["raw_right_m", "raw_down_m", "raw_forward_m", "measured_disparity_px"],
                    "runtime_truth_used": False,
                    "rdf_axis_contract": "stereo_center_opencv_optical:RDF(right,down,forward)",
                    "public_topic_contract": "stereodistance_position_topics:FRD_range(forward,right,down,range)",
                }
            )
            + "\n",
            encoding="utf-8",
        )
        model = root / "selector_model.json"
        model.write_text(
            json.dumps(
                {
                    "feature_names": ["raw_forward_m_mean"],
                    "mean": [0.0],
                    "scale": [1.0],
                    "centroids": {"unit": [1002.0]},
                }
            )
            + "\n",
            encoding="utf-8",
        )
        catalog = root / "catalog.json"
        catalog.write_text(json.dumps({"profile_paths": {"unit": "profiles/unit.json"}}) + "\n", encoding="utf-8")
        return model, catalog

    def write_current_schema_telemetry(self, path: Path) -> None:
        columns = [
            "time_sec",
            "axis_contract",
            "left_cx",
            "left_cy",
            "raw_disparity_px",
            "raw_forward_m",
            "raw_right_m",
            "raw_down_m",
            "corrected_forward_m",
            "corrected_right_m",
            "corrected_down_m",
        ]
        rows = []
        for index in range(5):
            rows.append(
                {
                    "time_sec": index * 0.1,
                    "axis_contract": "stereodistance_public_frd:forward,right,down,range",
                    "left_cx": 700.0 + index,
                    "left_cy": 390.0 + index,
                    "raw_disparity_px": 200.0,
                    "raw_forward_m": 1000.0 + index,
                    "raw_right_m": 7.5,
                    "raw_down_m": 4.2,
                    "corrected_forward_m": 1001.0 + index,
                    "corrected_right_m": 7.8,
                    "corrected_down_m": 4.1,
                }
            )
        path.parent.mkdir(parents=True, exist_ok=True)
        with path.open("w", newline="", encoding="utf-8") as handle:
            writer = csv.DictWriter(handle, fieldnames=columns)
            writer.writeheader()
            writer.writerows(rows)

    def test_latest_gate_runs_readiness_when_preflight_passes(self) -> None:
        module = load_module()
        with tempfile.TemporaryDirectory() as tmpdir:
            root = Path(tmpdir)
            profile = root / "active_runtime_profile.json"
            bags = root / "bags"
            telemetry = bags / "operator_new_lite" / "telemetry.csv"
            output_dir = root / "gate"
            self.write_profile(profile)
            model, catalog = self.write_selector(root)
            self.write_current_schema_telemetry(telemetry)

            result = module.run(
                [
                    "--profile-json",
                    str(profile),
                    "--latest-telemetry-root",
                    str(bags),
                    "--selector-model-json",
                    str(model),
                    "--catalog-json",
                    str(catalog),
                    "--output-dir",
                    str(output_dir),
                    "--window-size",
                    "5",
                    "--min-samples",
                    "5",
                    "--expected-label",
                    "unit",
                    "--require-stable-label",
                ]
            )

        self.assertEqual(result["status"], "PASS")
        self.assertEqual(result["input_telemetry_csv"], str(telemetry))
        self.assertEqual(result["preflight_status"], "PASS")
        self.assertEqual(result["readiness_status"], "PASS")
        self.assertTrue(result["apply_residual_allowed"])
        self.assertFalse(result["runtime_truth_used"])

    def test_latest_gate_reports_old_schema_without_readiness(self) -> None:
        module = load_module()
        with tempfile.TemporaryDirectory() as tmpdir:
            root = Path(tmpdir)
            profile = root / "active_runtime_profile.json"
            bags = root / "bags"
            old_telemetry = bags / "operator_old_lite" / "telemetry.csv"
            new_telemetry = bags / "operator_new_lite" / "telemetry.csv"
            output_dir = root / "gate"
            self.write_profile(profile)
            model, catalog = self.write_selector(root)
            self.write_current_schema_telemetry(old_telemetry)
            new_telemetry.parent.mkdir(parents=True)
            new_telemetry.write_text("left_cx,left_cy,raw_disparity_px,range_m\n700,390,200,1000\n", encoding="utf-8")
            os.utime(old_telemetry, (1000.0, 1000.0))
            os.utime(new_telemetry, (2000.0, 2000.0))

            result = module.run(
                [
                    "--profile-json",
                    str(profile),
                    "--latest-telemetry-root",
                    str(bags),
                    "--selector-model-json",
                    str(model),
                    "--catalog-json",
                    str(catalog),
                    "--output-dir",
                    str(output_dir),
                    "--window-size",
                    "5",
                    "--min-samples",
                    "5",
                ]
            )

        self.assertEqual(result["status"], "FAIL")
        self.assertEqual(result["input_telemetry_csv"], str(new_telemetry))
        self.assertEqual(result["preflight_status"], "FAIL")
        self.assertEqual(result["readiness_status"], "SKIPPED")
        self.assertFalse(result["apply_residual_allowed"])
        self.assertIn("readiness:preflight_not_passed", result["failures"])
        self.assertIn("Record a fresh", result["next_action"])


if __name__ == "__main__":
    unittest.main()
