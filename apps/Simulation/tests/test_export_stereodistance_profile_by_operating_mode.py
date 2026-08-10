from __future__ import annotations

import importlib.util
import json
import sys
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch


ROOT_DIR = Path(__file__).resolve().parents[1]


def load_export_module():
    spec = importlib.util.spec_from_file_location(
        "export_stereodistance_profile_by_operating_mode_under_test",
        ROOT_DIR / "scripts" / "export_stereodistance_profile_by_operating_mode.py",
    )
    if spec is None or spec.loader is None:
        raise RuntimeError("cannot load export_stereodistance_profile_by_operating_mode.py")
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


class ExportStereoDistanceProfileByOperatingModeTests(unittest.TestCase):
    def test_auto_mode_uses_calibration_replay_when_truth_fields_exist(self) -> None:
        module = load_export_module()
        with tempfile.TemporaryDirectory() as tmpdir:
            root = Path(tmpdir)
            input_csv = root / "calibration.csv"
            input_csv.write_text(
                "raw_right_m,raw_down_m,raw_forward_m,measured_disparity_px,truth_right_m\n"
                "1,2,3,4,5\n",
                encoding="utf-8",
            )
            output_profile = root / "active.json"

            def fake_select_by_calibration_replay(**kwargs):
                output_profile.write_text('{"profile":"calibration"}\n', encoding="utf-8")
                return {
                    "scenario": "nominal",
                    "method": "sim3",
                    "validation_range_rmse_corrected_m": 1.0,
                    "target_abs_mean_error_corrected_m": 0.5,
                    "target_max_abs_error_corrected_m": 0.7,
                }

            with patch.object(module, "select_by_calibration_replay", side_effect=fake_select_by_calibration_replay):
                summary = module.export_by_operating_mode(
                    catalog_json=root / "catalog.json",
                    input_csv=input_csv,
                    output_profile=output_profile,
                    output_dir=root / "out",
                    mode="auto",
                )

            self.assertEqual(summary["selected_operating_mode"], "calibration_replay")
            self.assertEqual(summary["selected_source_scenario"], "nominal")
            self.assertTrue((root / "out" / "operating_mode_profile_export_summary.json").is_file())

    def test_auto_mode_uses_runtime_guard_without_truth_fields(self) -> None:
        module = load_export_module()
        with tempfile.TemporaryDirectory() as tmpdir:
            root = Path(tmpdir)
            input_csv = root / "runtime.csv"
            input_csv.write_text(
                "raw_right_m,raw_down_m,raw_forward_m,measured_disparity_px\n"
                "1,2,3,4\n",
                encoding="utf-8",
            )
            output_profile = root / "active.json"

            def fake_runtime_selection(**kwargs):
                self.assertEqual(kwargs["selection_policy"], "intrinsic_scale_gap_transfer_risk")
                self.assertEqual(kwargs["intrinsic_scale_gap_source_scenario"], "intrinsic_scale_stress")
                self.assertAlmostEqual(kwargs["intrinsic_scale_gap_product_ratio_p10_threshold"], 1.0037)
                kwargs["output_profile"].write_text('{"profile":"runtime"}\n', encoding="utf-8")
                return {
                    "selected": True,
                    "scenario": "bbox",
                    "selection_mode": "runtime_measurement_guard_selection_only",
                    "fallback_reason": "intrinsic_scale_gap_detected_but_source_unavailable",
                }

            with patch.object(
                module,
                "select_runtime_guard_profile_without_truth",
                side_effect=fake_runtime_selection,
            ):
                summary = module.export_by_operating_mode(
                    catalog_json=root / "scenario_profile_catalog" / "manifest.json",
                    input_csv=input_csv,
                    output_profile=output_profile,
                    output_dir=root / "out",
                    sweep_dir=root / "sweep",
                    mode="auto",
                    runtime_selection_policy="intrinsic_scale_gap_transfer_risk",
                )

            self.assertEqual(summary["selected_operating_mode"], "runtime_guard")
            self.assertEqual(summary["selected_source_scenario"], "bbox")
            self.assertEqual(summary["selected_method"], "")
            self.assertEqual(
                summary["selection_summary"]["fallback_reason"],
                "intrinsic_scale_gap_detected_but_source_unavailable",
            )

    def test_runtime_guard_can_block_calibration_required_export(self) -> None:
        module = load_export_module()
        with tempfile.TemporaryDirectory() as tmpdir:
            root = Path(tmpdir)
            sweep_dir = root / "sweep"
            for scenario, forward, product in (
                ("nominal", 1000.0, 20000.0),
                ("combined_sensor_stress", 1200.0, 20000.0),
            ):
                csv_dir = sweep_dir / scenario / "seed_1"
                csv_dir.mkdir(parents=True)
                csv_dir.joinpath("calibration_samples.csv").write_text(
                    "raw_right_m,raw_down_m,raw_forward_m,raw_range_m,measured_disparity_px,left_u_px,left_v_px,split\n"
                    f"1,2,{forward},{forward},{product / forward},640,360,validation\n",
                    encoding="utf-8",
                )
            catalog_dir = sweep_dir / "scenario_profile_catalog"
            catalog_dir.mkdir()
            for scenario in ("nominal", "combined_sensor_stress"):
                (catalog_dir / f"{scenario}_runtime_profile.json").write_text(
                    json.dumps({"profile": scenario}) + "\n",
                    encoding="utf-8",
                )
            catalog_json = catalog_dir / "manifest.json"
            catalog_json.write_text(
                json.dumps(
                    {
                        "mode": "scenario_profile_catalog",
                        "profiles": [
                            {
                                "scenario": "nominal",
                                "method": "sim3",
                                "catalog_profile_path": "scenario_profile_catalog/nominal_runtime_profile.json",
                                "profile_exists": True,
                                "pass_cross_seed_replay": True,
                            },
                            {
                                "scenario": "combined_sensor_stress",
                                "method": "sim3",
                                "catalog_profile_path": "scenario_profile_catalog/combined_sensor_stress_runtime_profile.json",
                                "profile_exists": True,
                                "pass_cross_seed_replay": True,
                            },
                        ],
                    }
                ),
                encoding="utf-8",
            )
            runtime_csv = root / "runtime.csv"
            runtime_csv.write_text(
                "raw_right_m,raw_down_m,raw_forward_m,raw_range_m,measured_disparity_px,left_u_px,left_v_px\n"
                "1,2,1000,1000,20.1,640,360\n",
                encoding="utf-8",
            )
            output_profile = root / "active.json"

            summary = module.export_by_operating_mode(
                catalog_json=catalog_json,
                input_csv=runtime_csv,
                output_profile=output_profile,
                output_dir=root / "out",
                sweep_dir=sweep_dir,
                mode="runtime_guard",
                runtime_selection_policy="nearest_centroid",
                intrinsic_scale_gap_source_scenario="intrinsic_scale_stress",
                intrinsic_scale_gap_product_ratio_p10_threshold=0.0001,
                block_calibration_required_runtime_export=True,
            )

            self.assertFalse(summary["selected"])
            self.assertTrue(summary["calibration_required_detected"])
            self.assertTrue(summary["calibration_required_blocked_export"])
            self.assertEqual(summary["output_profile"], "")
            self.assertFalse(output_profile.exists())
            selection_summary = summary["selection_summary"]
            self.assertEqual(
                selection_summary["calibration_required_block_reason"],
                "intrinsic_scale_gap_detected_required_profile_unavailable",
            )
            self.assertEqual(selection_summary["output_profile"], "")


if __name__ == "__main__":
    unittest.main()
