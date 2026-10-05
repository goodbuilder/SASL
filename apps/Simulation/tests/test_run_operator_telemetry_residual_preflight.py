from __future__ import annotations

import importlib.util
import json
import os
import sys
import tempfile
import unittest
from pathlib import Path


ROOT_DIR = Path(__file__).resolve().parents[1]
SCRIPT_PATH = ROOT_DIR / "scripts" / "run_operator_telemetry_residual_preflight.py"


def load_module():
    spec = importlib.util.spec_from_file_location(
        "run_operator_telemetry_residual_preflight_under_test",
        SCRIPT_PATH,
    )
    if spec is None or spec.loader is None:
        raise RuntimeError(f"cannot load {SCRIPT_PATH}")
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


class RunOperatorTelemetryResidualPreflightTests(unittest.TestCase):
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
                    "method": "unit_residual",
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

    def write_telemetry(self, path: Path, *, include_corrected: bool = True) -> None:
        columns = [
            "left_cx",
            "left_cy",
            "raw_disparity_px",
            "raw_forward_m",
            "raw_right_m",
            "raw_down_m",
            "drone_forward_m",
            "ground_truth_m",
        ]
        if include_corrected:
            columns.extend(["corrected_forward_m", "corrected_right_m", "corrected_down_m"])
        rows = [columns]
        for index in range(5):
            row = [
                str(700.0 + index),
                str(390.0 + index),
                "200.0",
                str(1000.0 + index),
                "7.5",
                "4.2",
                "999.0",
                "1000.0",
            ]
            if include_corrected:
                row.extend([str(1001.0 + index), "7.8", "4.1"])
            rows.append(row)
        path.write_text("\n".join(",".join(row) for row in rows) + "\n", encoding="utf-8")

    def test_preflight_runs_selector_decision_and_operator_launch(self) -> None:
        module = load_module()
        with tempfile.TemporaryDirectory() as tmpdir:
            root = Path(tmpdir)
            profile = root / "active_runtime_profile.json"
            telemetry = root / "telemetry.csv"
            output_dir = root / "preflight"
            self.write_profile(profile)
            model, catalog = self.write_selector(root)
            self.write_telemetry(telemetry)

            result = module.run(
                [
                    "--profile-json",
                    str(profile),
                    "--input-telemetry-csv",
                    str(telemetry),
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
            summary_exists = (output_dir / "operator_telemetry_residual_preflight_summary.json").is_file()
            decision_exists = (output_dir / "telemetry_selector_replay" / "operator_telemetry_selector_decision.json").is_file()
            launch_exists = (output_dir / "operator_launch_check_from_decision.json").is_file()

        self.assertEqual(result["status"], "PASS")
        self.assertEqual(result["schema_audit_status"], "PASS")
        self.assertEqual(result["schema_audit_valid_row_count"], 5)
        self.assertTrue(summary_exists)
        self.assertTrue(decision_exists)
        self.assertTrue(launch_exists)
        self.assertTrue(result["apply_residual_allowed"])
        self.assertEqual(result["recommended_residual_label"], "unit")
        self.assertTrue(str(result["resolved_disparity_residual_profile"]).endswith("profiles/unit.json"))
        self.assertFalse(result["runtime_truth_used"])
        self.assertIn("stereo-center RDF", result["coordinate_contract"])

    def test_preflight_fails_before_launch_when_selector_gate_fails(self) -> None:
        module = load_module()
        with tempfile.TemporaryDirectory() as tmpdir:
            root = Path(tmpdir)
            profile = root / "active_runtime_profile.json"
            telemetry = root / "telemetry.csv"
            output_dir = root / "preflight"
            self.write_profile(profile)
            model, catalog = self.write_selector(root)
            self.write_telemetry(telemetry)

            result = module.run(
                [
                    "--profile-json",
                    str(profile),
                    "--input-telemetry-csv",
                    str(telemetry),
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
                    "--min-distance-margin",
                    "1000000001.0",
                ]
            )
            launch_exists = (output_dir / "operator_launch_check_from_decision.json").is_file()

        self.assertEqual(result["status"], "FAIL")
        self.assertFalse(result["apply_residual_allowed"])
        self.assertFalse(launch_exists)
        self.assertIn("selector_decision:apply_not_allowed", result["failures"])

    def test_preflight_stops_before_selector_replay_when_schema_audit_fails(self) -> None:
        module = load_module()
        with tempfile.TemporaryDirectory() as tmpdir:
            root = Path(tmpdir)
            profile = root / "active_runtime_profile.json"
            telemetry = root / "telemetry.csv"
            output_dir = root / "preflight"
            self.write_profile(profile)
            model, catalog = self.write_selector(root)
            telemetry.write_text(
                "\n".join(
                    [
                        "left_cx,left_cy,raw_disparity_px,range_m,ground_truth_m",
                        "700,390,200,1000,1000",
                    ]
                )
                + "\n",
                encoding="utf-8",
            )

            result = module.run(
                [
                    "--profile-json",
                    str(profile),
                    "--input-telemetry-csv",
                    str(telemetry),
                    "--selector-model-json",
                    str(model),
                    "--catalog-json",
                    str(catalog),
                    "--output-dir",
                    str(output_dir),
                    "--window-size",
                    "5",
                    "--min-samples",
                    "1",
                ]
            )
            audit_exists = (output_dir / "operator_telemetry_schema_audit.json").is_file()
            replay_summary_exists = (
                output_dir / "telemetry_selector_replay" / "operator_telemetry_selector_replay_summary.json"
            ).is_file()
            decision_json = output_dir / "telemetry_selector_replay" / "operator_telemetry_selector_decision.json"
            decision = json.loads(decision_json.read_text(encoding="utf-8"))

        self.assertEqual(result["status"], "FAIL")
        self.assertEqual(result["schema_audit_status"], "FAIL")
        self.assertTrue(audit_exists)
        self.assertTrue(replay_summary_exists)
        self.assertEqual(decision["status"], "SKIPPED")
        self.assertFalse(decision["apply_residual_allowed"])
        self.assertFalse(result["apply_residual_allowed"])
        self.assertIn("schema_audit:missing_required_group:nominal_pose_forward_m", result["failures"])

    def test_preflight_can_discover_latest_telemetry_csv(self) -> None:
        module = load_module()
        with tempfile.TemporaryDirectory() as tmpdir:
            root = Path(tmpdir)
            profile = root / "active_runtime_profile.json"
            bags = root / "bags"
            old_telemetry = bags / "operator_old_lite" / "telemetry.csv"
            new_telemetry = bags / "operator_new_lite" / "telemetry.csv"
            output_dir = root / "preflight"
            old_telemetry.parent.mkdir(parents=True)
            new_telemetry.parent.mkdir(parents=True)
            self.write_profile(profile)
            model, catalog = self.write_selector(root)
            self.write_telemetry(old_telemetry)
            self.write_telemetry(new_telemetry)
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
                    "--expected-label",
                    "unit",
                ]
            )

        self.assertEqual(result["status"], "PASS")
        self.assertEqual(result["input_telemetry_csv"], str(new_telemetry))
        self.assertEqual(result["latest_telemetry_root"], str(bags))


if __name__ == "__main__":
    unittest.main()
