from __future__ import annotations

import csv
import importlib.util
import json
import os
import sys
import tempfile
import time
import unittest
from pathlib import Path
from unittest.mock import patch

import pytest


ROOT_DIR = Path(__file__).resolve().parents[1]
SCRIPT_PATH = ROOT_DIR / "scripts" / "run_operator_record_lite_and_residual_suite.py"


def load_module():
    spec = importlib.util.spec_from_file_location("run_operator_record_lite_and_residual_suite_under_test", SCRIPT_PATH)
    if spec is None or spec.loader is None:
        raise RuntimeError(f"cannot load {SCRIPT_PATH}")
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


@pytest.mark.usefixtures("isolated_operator_overlay_root")
class RunOperatorRecordLiteAndResidualSuiteTests(unittest.TestCase):
    def run_with_isolated_overlay(self, module, argv):
        suite = module.enablement_suite
        original_readiness = suite.install_overlay_readiness.check_readiness
        original_smoke = suite.installed_csv_smoke.run

        def check_isolated_overlay(_root):
            return original_readiness(self.isolated_operator_overlay_root)

        def run_isolated_smoke(smoke_argv):
            return original_smoke(
                ["--root-dir", str(self.isolated_operator_overlay_root), *smoke_argv]
            )

        with patch.object(
            suite.install_overlay_readiness,
            "check_readiness",
            side_effect=check_isolated_overlay,
        ), patch.object(suite.installed_csv_smoke, "run", side_effect=run_isolated_smoke):
            return module.run(argv)

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
        path.parent.mkdir(parents=True, exist_ok=True)
        with path.open("w", newline="", encoding="utf-8") as handle:
            writer = csv.DictWriter(handle, fieldnames=columns)
            writer.writeheader()
            for index in range(5):
                writer.writerow(
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

    def write_external_bbox_samples(self, path: Path, *, valid_disparity: bool = True) -> None:
        path.parent.mkdir(parents=True, exist_ok=True)
        right_x1 = 478.0 if valid_disparity else 680.0
        right_x2 = 578.0 if valid_disparity else 780.0
        right_cx = 528.0 if valid_disparity else 730.0
        with path.open("w", newline="", encoding="utf-8") as handle:
            writer = csv.writer(handle)
            writer.writerow(
                [
                    "left_valid",
                    "left_x1",
                    "left_y1",
                    "left_x2",
                    "left_y2",
                    "left_confidence",
                    "left_cx",
                    "left_cy",
                    "right_valid",
                    "right_x1",
                    "right_y1",
                    "right_x2",
                    "right_y2",
                    "right_confidence",
                    "right_cx",
                    "right_cy",
                ]
            )
            writer.writerow([1, 650, 340, 750, 394, 0.9, 700, 367, 1, right_x1, 340, right_x2, 394, 0.8, right_cx, 367])

    def test_print_only_builds_record_and_suite_commands(self) -> None:
        module = load_module()
        with tempfile.TemporaryDirectory() as tmpdir:
            root = Path(tmpdir)
            profile = root / "active_runtime_profile.json"
            output_dir = root / "out"
            record_dir = root / "bags" / "operator_unit_lite"
            self.write_profile(profile)
            model, catalog = self.write_selector(root)

            result = self.run_with_isolated_overlay(
                module,
                [
                    "--profile-json",
                    str(profile),
                    "--latest-telemetry-root",
                    str(root / "bags"),
                    "--record-dir",
                    str(record_dir),
                    "--selector-model-json",
                    str(model),
                    "--catalog-json",
                    str(catalog),
                    "--output-dir",
                    str(output_dir),
                    "--print-only",
                ]
            )

        self.assertEqual(result["status"], "PASS")
        self.assertEqual(result["record_status"], "PRINT_ONLY")
        self.assertIn("--record-lite", result["record_command"])
        self.assertIn(str(record_dir), result["record_command"])
        self.assertNotIn("--no-px4", result["record_command"])
        self.assertIn("--latest-telemetry-root", result["suite_args"])
        self.assertEqual(result["record_telemetry_csv"], str(record_dir / "telemetry.csv"))
        self.assertFalse(result["record_telemetry_fresh"])
        self.assertFalse(result["suite_used_record_telemetry"])
        self.assertFalse(result["runtime_truth_used"])

    def test_print_only_builds_external_bbox_sidecar_command(self) -> None:
        module = load_module()
        with tempfile.TemporaryDirectory() as tmpdir:
            root = Path(tmpdir)
            profile = root / "active_runtime_profile.json"
            output_dir = root / "out"
            record_dir = root / "bags" / "operator_external_bbox_lite"
            self.write_profile(profile)
            model, catalog = self.write_selector(root)

            result = self.run_with_isolated_overlay(
                module,
                [
                    "--profile-json",
                    str(profile),
                    "--latest-telemetry-root",
                    str(root / "bags"),
                    "--record-dir",
                    str(record_dir),
                    "--selector-model-json",
                    str(model),
                    "--catalog-json",
                    str(catalog),
                    "--output-dir",
                    str(output_dir),
                    "--external-bbox-publisher",
                    "--external-bbox-topic",
                    "/yolo/stereo_bbox",
                    "--print-only",
                ]
            )

        self.assertEqual(result["status"], "PASS")
        self.assertIn("stereo_ground_sim.external_bbox_publisher", result["sidecar_command"])
        self.assertIn("/yolo/stereo_bbox", result["sidecar_command"])

    def test_skip_record_runs_suite_on_existing_latest_current_schema(self) -> None:
        module = load_module()
        with tempfile.TemporaryDirectory() as tmpdir:
            root = Path(tmpdir)
            profile = root / "active_runtime_profile.json"
            bags = root / "bags"
            telemetry = bags / "operator_new_lite" / "telemetry.csv"
            output_dir = root / "out"
            self.write_profile(profile)
            model, catalog = self.write_selector(root)
            self.write_current_schema_telemetry(telemetry)

            result = self.run_with_isolated_overlay(
                module,
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
                    "--current-schema-expected-label",
                    "unit",
                    "--skip-record",
                ]
            )

        self.assertEqual(result["status"], "PASS")
        self.assertEqual(result["record_status"], "SKIPPED")
        self.assertEqual(result["suite_status"], "PASS")
        self.assertFalse(result["runtime_truth_used"])

    def test_non_skip_record_requires_fresh_record_dir_telemetry(self) -> None:
        module = load_module()
        with tempfile.TemporaryDirectory() as tmpdir:
            root = Path(tmpdir)
            profile = root / "active_runtime_profile.json"
            bags = root / "bags"
            telemetry = bags / "operator_new_lite" / "telemetry.csv"
            record_dir = bags / "operator_expected_lite"
            output_dir = root / "out"
            self.write_profile(profile)
            model, catalog = self.write_selector(root)
            self.write_current_schema_telemetry(telemetry)

            result = self.run_with_isolated_overlay(
                module,
                [
                    "--profile-json",
                    str(profile),
                    "--latest-telemetry-root",
                    str(bags),
                    "--record-dir",
                    str(record_dir),
                    "--record-seconds",
                    "0",
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
                    "--current-schema-expected-label",
                    "unit",
                ]
            )

        self.assertEqual(result["status"], "FAIL")
        self.assertEqual(result["record_status"], "SKIPPED")
        self.assertEqual(result["suite_status"], "PASS")
        self.assertFalse(result["record_telemetry_exists"])
        self.assertFalse(result["record_telemetry_fresh"])
        self.assertFalse(result["suite_used_record_telemetry"])
        self.assertIn("record:not_executed", result["failures"])
        self.assertIn("record:telemetry_missing", result["failures"])

    def test_synthetic_record_writes_fresh_record_dir_telemetry_and_passes_suite(self) -> None:
        module = load_module()
        with tempfile.TemporaryDirectory() as tmpdir:
            root = Path(tmpdir)
            profile = root / "active_runtime_profile.json"
            bags = root / "bags"
            record_dir = bags / "operator_expected_lite"
            output_dir = root / "out"
            self.write_profile(profile)
            model, catalog = self.write_selector(root)

            result = self.run_with_isolated_overlay(
                module,
                [
                    "--profile-json",
                    str(profile),
                    "--latest-telemetry-root",
                    str(bags),
                    "--record-dir",
                    str(record_dir),
                    "--synthetic-record",
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
                    "--current-schema-expected-label",
                    "unit",
                ]
            )

        self.assertEqual(result["status"], "PASS")
        self.assertEqual(result["record_status"], "SYNTHETIC")
        self.assertTrue(result["synthetic_record"])
        self.assertEqual(result["suite_status"], "PASS")
        self.assertTrue(result["record_telemetry_exists"])
        self.assertTrue(result["record_telemetry_fresh"])
        self.assertTrue(result["suite_used_record_telemetry"])
        self.assertFalse(result["runtime_truth_used"])
        self.assertEqual(result["failures"], [])

    def test_external_bbox_preflight_passes_before_synthetic_record_suite(self) -> None:
        module = load_module()
        with tempfile.TemporaryDirectory() as tmpdir:
            root = Path(tmpdir)
            profile = root / "active_runtime_profile.json"
            bags = root / "bags"
            record_dir = bags / "operator_expected_lite"
            output_dir = root / "out"
            sample_csv = root / "external_bbox_samples.csv"
            self.write_profile(profile)
            model, catalog = self.write_selector(root)
            self.write_external_bbox_samples(sample_csv, valid_disparity=True)

            result = self.run_with_isolated_overlay(
                module,
                [
                    "--profile-json",
                    str(profile),
                    "--latest-telemetry-root",
                    str(bags),
                    "--record-dir",
                    str(record_dir),
                    "--synthetic-record",
                    "--external-bbox-preflight",
                    "--external-bbox-preflight-input-csv",
                    str(sample_csv),
                    "--external-bbox-preflight-min-valid-pairs",
                    "1",
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
                    "--current-schema-expected-label",
                    "unit",
                ]
            )

        self.assertEqual(result["status"], "PASS")
        self.assertEqual(result["external_bbox_preflight_status"], "PASS")
        self.assertEqual(result["record_status"], "SYNTHETIC")
        self.assertEqual(result["suite_status"], "PASS")
        self.assertFalse(result["runtime_truth_used"])

    def test_external_bbox_preflight_min_valid_pairs_failure_skips_record_and_suite(self) -> None:
        module = load_module()
        with tempfile.TemporaryDirectory() as tmpdir:
            root = Path(tmpdir)
            profile = root / "active_runtime_profile.json"
            bags = root / "bags"
            record_dir = bags / "operator_expected_lite"
            output_dir = root / "out"
            sample_csv = root / "external_bbox_samples.csv"
            self.write_profile(profile)
            model, catalog = self.write_selector(root)
            self.write_external_bbox_samples(sample_csv, valid_disparity=True)

            result = self.run_with_isolated_overlay(
                module,
                [
                    "--profile-json",
                    str(profile),
                    "--latest-telemetry-root",
                    str(bags),
                    "--record-dir",
                    str(record_dir),
                    "--synthetic-record",
                    "--external-bbox-preflight",
                    "--external-bbox-preflight-input-csv",
                    str(sample_csv),
                    "--external-bbox-preflight-min-valid-pairs",
                    "2",
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
                    "--current-schema-expected-label",
                    "unit",
                ]
            )

        self.assertEqual(result["status"], "FAIL")
        self.assertEqual(result["external_bbox_preflight_status"], "FAIL")
        self.assertEqual(result["record_status"], "SKIPPED_BY_EXTERNAL_BBOX_PREFLIGHT")
        self.assertEqual(result["suite_status"], "SKIPPED_BY_EXTERNAL_BBOX_PREFLIGHT")
        self.assertIn("external_bbox_preflight:valid_pair_count_below_min:1<2", result["failures"])

    def test_external_bbox_preflight_failure_skips_record_and_suite(self) -> None:
        module = load_module()
        with tempfile.TemporaryDirectory() as tmpdir:
            root = Path(tmpdir)
            profile = root / "active_runtime_profile.json"
            bags = root / "bags"
            record_dir = bags / "operator_expected_lite"
            output_dir = root / "out"
            sample_csv = root / "external_bbox_samples.csv"
            self.write_profile(profile)
            model, catalog = self.write_selector(root)
            self.write_external_bbox_samples(sample_csv, valid_disparity=False)

            result = module.run(
                [
                    "--profile-json",
                    str(profile),
                    "--latest-telemetry-root",
                    str(bags),
                    "--record-dir",
                    str(record_dir),
                    "--synthetic-record",
                    "--external-bbox-preflight",
                    "--external-bbox-preflight-input-csv",
                    str(sample_csv),
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
                    "--current-schema-expected-label",
                    "unit",
                ]
            )

        self.assertEqual(result["status"], "FAIL")
        self.assertEqual(result["external_bbox_preflight_status"], "FAIL")
        self.assertEqual(result["record_status"], "SKIPPED_BY_EXTERNAL_BBOX_PREFLIGHT")
        self.assertEqual(result["suite_status"], "SKIPPED_BY_EXTERNAL_BBOX_PREFLIGHT")
        self.assertFalse(result["record_telemetry_exists"])
        self.assertIn("external_bbox_preflight:validation:row0:left_minus_right_disparity_below_min:-30", result["failures"])

    def test_non_skip_record_rejects_stale_record_dir_telemetry(self) -> None:
        module = load_module()
        with tempfile.TemporaryDirectory() as tmpdir:
            root = Path(tmpdir)
            profile = root / "active_runtime_profile.json"
            bags = root / "bags"
            record_dir = bags / "operator_expected_lite"
            telemetry = record_dir / "telemetry.csv"
            output_dir = root / "out"
            self.write_profile(profile)
            model, catalog = self.write_selector(root)
            self.write_current_schema_telemetry(telemetry)
            stale_time = time.time() - 1000.0
            os.utime(telemetry, (stale_time, stale_time))

            result = self.run_with_isolated_overlay(
                module,
                [
                    "--profile-json",
                    str(profile),
                    "--latest-telemetry-root",
                    str(bags),
                    "--record-dir",
                    str(record_dir),
                    "--record-seconds",
                    "0",
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
                    "--current-schema-expected-label",
                    "unit",
                ]
            )

        self.assertEqual(result["status"], "FAIL")
        self.assertEqual(result["suite_status"], "PASS")
        self.assertTrue(result["record_telemetry_exists"])
        self.assertFalse(result["record_telemetry_fresh"])
        self.assertTrue(result["suite_used_record_telemetry"])
        self.assertIn("record:telemetry_stale", result["failures"])


if __name__ == "__main__":
    unittest.main()
