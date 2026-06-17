from __future__ import annotations

import importlib.util
import json
import sys
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch


ROOT_DIR = Path(__file__).resolve().parents[1]


def load_module():
    spec = importlib.util.spec_from_file_location(
        "audit_method_family_runtime_feature_selector_under_test",
        ROOT_DIR / "scripts" / "audit_method_family_runtime_feature_selector.py",
    )
    if spec is None or spec.loader is None:
        raise RuntimeError("cannot load audit_method_family_runtime_feature_selector.py")
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def sample_csv_text(forward: float, disparity: float) -> str:
    return (
        "split,raw_right_m,raw_down_m,raw_forward_m,raw_range_m,measured_disparity_px,left_u_px,left_v_px,"
        "truth_right_m,truth_down_m,truth_forward_m\n"
        f"validation,1.0,2.0,{forward},{forward},{disparity},640,360,1.0,2.0,{forward}\n"
        f"train,1.1,2.1,{forward + 1.0},{forward + 1.0},{disparity},641,361,1.1,2.1,{forward + 1.0}\n"
    )


class AuditMethodFamilyRuntimeFeatureSelectorTests(unittest.TestCase):
    def test_nearest_runtime_feature_selector_replays_selected_source_profile(self) -> None:
        module = load_module()
        with tempfile.TemporaryDirectory() as tmpdir:
            root = Path(tmpdir)
            sweep = root / "sweep"
            selected_rows = []
            for scenario, forward, disparity in (
                ("nominal", 1000.0, 20.0),
                ("stress", 1500.0, 14.0),
            ):
                run_dir = sweep / scenario / "seed_1"
                run_dir.mkdir(parents=True)
                run_dir.joinpath("calibration_samples.csv").write_text(
                    sample_csv_text(forward, disparity),
                    encoding="utf-8",
                )
                profile = run_dir / f"{scenario}_affine.json"
                profile.write_text(json.dumps({"scenario": scenario}), encoding="utf-8")
                selected_rows.append(
                    {
                        "scenario": scenario,
                        "seed": "seed_1",
                        "selected": "True",
                        "selected_method": f"{scenario}_affine",
                        "selected_profile_path": str(profile),
                    }
                )
            selected_csv = root / "selected.csv"
            module.write_csv(selected_csv, selected_rows)

            def fake_replay_rows(rows, profile, *, split_filter, target_range_m, target_window_m):
                del rows, profile, split_filter, target_range_m, target_window_m
                return [], {
                    "validation_range_rmse_raw_m": 3.0,
                    "validation_range_rmse_corrected_m": 1.0,
                    "target_window_count": 1,
                    "target_abs_mean_error_corrected_m": 0.5,
                    "target_max_abs_error_corrected_m": 0.8,
                }

            with patch.object(module, "load_profile", side_effect=lambda path: path), patch.object(
                module,
                "replay_rows",
                side_effect=fake_replay_rows,
            ):
                summary = module.audit_runtime_feature_selector(
                    sweep,
                    selected_csv,
                    root / "out",
                    feature_subset="range_geometry",
                    include_target_run=False,
                )

            self.assertEqual(summary["audit_count"], 2)
            self.assertEqual(summary["pass_count"], 2)
            rows_csv = (root / "out" / "runtime_feature_selector_audit.csv").read_text(encoding="utf-8")
            self.assertIn("selected_source_scenario", rows_csv)
            self.assertIn("runtime feature selector uses raw RDF/disparity", rows_csv)
            summary_json = json.loads((root / "out" / "runtime_feature_selector_summary.json").read_text(encoding="utf-8"))
            self.assertEqual(summary_json["axis_contract"], module.RDF_AXIS_CONTRACT)

    def test_robust_bbox_fallback_uses_bbox_source_for_low_range_family(self) -> None:
        module = load_module()
        with tempfile.TemporaryDirectory() as tmpdir:
            root = Path(tmpdir)
            sweep = root / "sweep"
            selected_rows = []
            for scenario, seed, forward, disparity in (
                ("nominal", "seed_1", 1000.0, 20.0),
                ("bbox_outlier_stress", "seed_2", 1002.0, 19.9),
                ("far_range", "seed_3", 1800.0, 11.0),
            ):
                run_dir = sweep / scenario / seed
                run_dir.mkdir(parents=True)
                run_dir.joinpath("calibration_samples.csv").write_text(
                    sample_csv_text(forward, disparity),
                    encoding="utf-8",
                )
                profile = run_dir / f"{scenario}_affine.json"
                profile.write_text(json.dumps({"scenario": scenario}), encoding="utf-8")
                selected_rows.append(
                    {
                        "scenario": scenario,
                        "seed": seed,
                        "selected": "True",
                        "selected_method": f"{scenario}_affine",
                        "selected_profile_path": str(profile),
                    }
                )
            selected_csv = root / "selected.csv"
            module.write_csv(selected_csv, selected_rows)

            def fake_replay_rows(rows, profile, *, split_filter, target_range_m, target_window_m):
                del rows, profile, split_filter, target_range_m, target_window_m
                return [], {
                    "validation_range_rmse_raw_m": 3.0,
                    "validation_range_rmse_corrected_m": 1.0,
                    "target_window_count": 1,
                    "target_abs_mean_error_corrected_m": 0.5,
                    "target_max_abs_error_corrected_m": 0.8,
                }

            with patch.object(module, "load_profile", side_effect=lambda path: path), patch.object(
                module,
                "replay_rows",
                side_effect=fake_replay_rows,
            ):
                summary = module.audit_runtime_feature_selector(
                    sweep,
                    selected_csv,
                    root / "out",
                    feature_subset="all",
                    include_target_run=False,
                    selection_policy="robust_bbox_fallback",
                )

            self.assertEqual(summary["pass_count"], 3)
            rows = module.read_csv(root / "out" / "runtime_feature_selector_audit.csv")
            nominal_row = next(row for row in rows if row["target_scenario"] == "nominal")
            self.assertEqual(nominal_row["selected_source_scenario"], "bbox_outlier_stress")
            self.assertIn("low_range_family_bbox_fallback", nominal_row["feature_vote_rows_json"])
            summary_json = json.loads((root / "out" / "runtime_feature_selector_summary.json").read_text(encoding="utf-8"))
            self.assertEqual(summary_json["selection_policy"], "robust_bbox_fallback")
            self.assertIn("Runtime feature selector uses raw RDF/disparity", summary_json["selection_contract"])


if __name__ == "__main__":
    unittest.main()
