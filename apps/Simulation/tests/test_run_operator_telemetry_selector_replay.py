from __future__ import annotations

import csv
import importlib.util
import json
import sys
import tempfile
import unittest
from pathlib import Path


ROOT_DIR = Path(__file__).resolve().parents[1]
SCRIPT_PATH = ROOT_DIR / "scripts" / "run_operator_telemetry_selector_replay.py"


def load_module():
    spec = importlib.util.spec_from_file_location(
        "run_operator_telemetry_selector_replay_under_test",
        SCRIPT_PATH,
    )
    if spec is None or spec.loader is None:
        raise RuntimeError(f"cannot load {SCRIPT_PATH}")
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


class RunOperatorTelemetrySelectorReplayTests(unittest.TestCase):
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
            "time_sec",
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
                str(index * 0.1),
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

    def test_operator_telemetry_selector_replay_writes_truth_free_outputs(self) -> None:
        module = load_module()
        with tempfile.TemporaryDirectory() as tmpdir:
            root = Path(tmpdir)
            telemetry = root / "telemetry.csv"
            output_dir = root / "out"
            model, catalog = self.write_selector(root)
            self.write_telemetry(telemetry)

            result = module.run(
                [
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
                    "--stride",
                    "1",
                    "--min-samples",
                    "5",
                    "--expected-label",
                    "unit",
                ]
            )
            summary_path = output_dir / "operator_telemetry_selector_replay_summary.json"
            decision_path = output_dir / "operator_telemetry_selector_decision.json"
            selector_csv = output_dir / "selector_observations.csv"
            with selector_csv.open(newline="", encoding="utf-8") as handle:
                selector_rows = list(csv.DictReader(handle))
            summary = json.loads(summary_path.read_text(encoding="utf-8"))
            decision = json.loads(decision_path.read_text(encoding="utf-8"))

        self.assertEqual(result["status"], "PASS")
        self.assertEqual(summary["status"], "PASS")
        self.assertEqual(decision["status"], "PASS")
        self.assertFalse(result["runtime_truth_used"])
        self.assertEqual(result["dropped_truth_column_count"], 2)
        self.assertEqual(result["window_count"], 1)
        self.assertEqual(result["label_match_count"], 1)
        self.assertEqual(result["profile_exists_count"], 1)
        self.assertFalse(result["require_stable_label"])
        self.assertEqual(result["min_distance_margin"], 0.0)
        self.assertEqual(result["selected_label_counts"], {"unit": 1})
        self.assertEqual(result["dominant_selected_label"], "unit")
        self.assertTrue(result["selector_label_stable"])
        self.assertTrue(result["apply_residual_allowed"])
        self.assertEqual(result["recommended_residual_label"], "unit")
        self.assertTrue(result["recommended_residual_profile"].endswith("profiles/unit.json"))
        self.assertTrue(decision["apply_residual_allowed"])
        self.assertEqual(decision["recommended_residual_label"], "unit")
        self.assertTrue(decision["recommended_residual_profile"].endswith("profiles/unit.json"))
        self.assertEqual(selector_rows[0]["raw_forward_m"], "1000.0")
        self.assertEqual(selector_rows[0]["raw_right_m"], "7.5")
        self.assertEqual(selector_rows[0]["observation_forward_m"], "1001.0")
        self.assertNotIn("ground_truth_m", selector_rows[0])
        self.assertIn("RDF(right,down,forward)", result["coordinate_contract"])

    def test_operator_telemetry_selector_replay_fails_when_export_fails(self) -> None:
        module = load_module()
        with tempfile.TemporaryDirectory() as tmpdir:
            root = Path(tmpdir)
            telemetry = root / "telemetry.csv"
            output_dir = root / "out"
            model, catalog = self.write_selector(root)
            self.write_telemetry(telemetry, include_corrected=False)

            result = module.run(
                [
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
                ]
            )

        self.assertEqual(result["status"], "FAIL")
        self.assertTrue(any(str(failure).startswith("export:") for failure in result["failures"]))

    def test_operator_telemetry_selector_replay_propagates_replay_gates(self) -> None:
        module = load_module()
        with tempfile.TemporaryDirectory() as tmpdir:
            root = Path(tmpdir)
            telemetry = root / "telemetry.csv"
            output_dir = root / "out"
            model, catalog = self.write_selector(root)
            self.write_telemetry(telemetry)

            result = module.run(
                [
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
                    "--min-distance-margin",
                    "1000000001.0",
                ]
            )
            decision = json.loads((output_dir / "operator_telemetry_selector_decision.json").read_text(encoding="utf-8"))

        self.assertEqual(result["status"], "FAIL")
        self.assertTrue(result["require_stable_label"])
        self.assertEqual(result["min_distance_margin"], 1000000001.0)
        self.assertFalse(result["apply_residual_allowed"])
        self.assertEqual(result["recommended_residual_profile"], "")
        self.assertFalse(decision["apply_residual_allowed"])
        self.assertEqual(decision["recommended_residual_profile"], "")
        self.assertTrue(any(str(failure).startswith("replay:selector:distance_margin_below_threshold") for failure in result["failures"]))


if __name__ == "__main__":
    unittest.main()
