from __future__ import annotations

import importlib.util
import json
import sys
import tempfile
import unittest
from pathlib import Path


ROOT_DIR = Path(__file__).resolve().parents[1]
SCRIPT_PATH = ROOT_DIR / "scripts" / "check_disparity_residual_selector_on_observation_windows.py"


def load_module():
    spec = importlib.util.spec_from_file_location(
        "check_disparity_residual_selector_on_observation_windows_under_test",
        SCRIPT_PATH,
    )
    if spec is None or spec.loader is None:
        raise RuntimeError(f"cannot load {SCRIPT_PATH}")
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


class CheckDisparityResidualSelectorOnObservationWindowsTests(unittest.TestCase):
    def write_selector(self, root: Path) -> tuple[Path, Path]:
        model = root / "selector_model.json"
        catalog = root / "catalog.json"
        profile = root / "profiles" / "unit.json"
        profile.parent.mkdir(parents=True, exist_ok=True)
        profile.write_text(
            json.dumps(
                {
                    "profile_type": "DisparityResidualRuntimeProfile",
                    "profile_id": "unit",
                    "method": "unit",
                    "runtime_inputs": ["raw_right_m", "raw_down_m", "raw_forward_m", "measured_disparity_px"],
                    "runtime_truth_used": False,
                }
            )
            + "\n",
            encoding="utf-8",
        )
        model.write_text(
            json.dumps(
                {
                    "feature_names": ["raw_forward_m_mean"],
                    "mean": [0.0],
                    "scale": [1.0],
                    "centroids": {"unit": [800.0]},
                }
            )
            + "\n",
            encoding="utf-8",
        )
        catalog.write_text(json.dumps({"profile_paths": {"unit": "profiles/unit.json"}}) + "\n", encoding="utf-8")
        return model, catalog

    def write_two_label_selector(self, root: Path) -> tuple[Path, Path]:
        model = root / "selector_model.json"
        catalog = root / "catalog.json"
        profile_dir = root / "profiles"
        profile_dir.mkdir(parents=True, exist_ok=True)
        for label in ("near", "far"):
            (profile_dir / f"{label}.json").write_text(
                json.dumps(
                    {
                        "profile_type": "DisparityResidualRuntimeProfile",
                        "profile_id": label,
                        "runtime_inputs": ["raw_right_m", "raw_down_m", "raw_forward_m", "measured_disparity_px"],
                        "runtime_truth_used": False,
                    }
                )
                + "\n",
                encoding="utf-8",
            )
        model.write_text(
            json.dumps(
                {
                    "feature_names": ["raw_forward_m_mean"],
                    "mean": [0.0],
                    "scale": [1.0],
                    "centroids": {"near": [100.0], "far": [300.0]},
                }
            )
            + "\n",
            encoding="utf-8",
        )
        catalog.write_text(
            json.dumps({"profile_paths": {"near": "profiles/near.json", "far": "profiles/far.json"}}) + "\n",
            encoding="utf-8",
        )
        return model, catalog

    def write_observations(self, path: Path, *, include_truth: bool = False) -> None:
        header = [
            "left_u_px",
            "left_v_px",
            "measured_disparity_px",
            "fused_disparity_px",
            "raw_right_m",
            "raw_down_m",
            "raw_forward_m",
            "observation_right_m",
            "observation_down_m",
            "observation_forward_m",
        ]
        if include_truth:
            header.append("px4_ned_north_m")
        rows = [",".join(header)]
        for index in range(5):
            values = [
                str(640.0 + index),
                str(360.0 + index),
                "10.0",
                "10.1",
                "1.0",
                "2.0",
                str(798.0 + index),
                "1.1",
                "2.1",
                str(798.5 + index),
            ]
            if include_truth:
                values.append("800.0")
            rows.append(",".join(values))
        path.write_text("\n".join(rows) + "\n", encoding="utf-8")

    def write_unstable_observations(self, path: Path) -> None:
        header = [
            "left_u_px",
            "left_v_px",
            "measured_disparity_px",
            "fused_disparity_px",
            "raw_right_m",
            "raw_down_m",
            "raw_forward_m",
            "observation_right_m",
            "observation_down_m",
            "observation_forward_m",
        ]
        rows = [",".join(header)]
        for forward in (100.0, 300.0):
            for index in range(3):
                rows.append(
                    ",".join(
                        [
                            str(640.0 + index),
                            str(360.0 + index),
                            "10.0",
                            "10.1",
                            "1.0",
                            "2.0",
                            str(forward),
                            "1.1",
                            "2.1",
                            str(forward),
                        ]
                    )
                )
        path.write_text("\n".join(rows) + "\n", encoding="utf-8")

    def test_selector_windows_passes_truth_free_csv(self) -> None:
        module = load_module()
        with tempfile.TemporaryDirectory() as tmpdir:
            root = Path(tmpdir)
            model, catalog = self.write_selector(root)
            observations = root / "observations.csv"
            self.write_observations(observations)

            result = module.selector_windows(
                input_csv=observations,
                selector_model_json=model,
                catalog_json=catalog,
                window_size=3,
                stride=2,
                min_samples=3,
                expected_label="unit",
            )

        self.assertEqual(result["status"], "PASS")
        self.assertEqual(result["window_count"], 2)
        self.assertEqual(result["label_match_count"], 2)
        self.assertEqual(result["profile_exists_count"], 2)
        self.assertEqual(result["selected_label_counts"], {"unit": 2})
        self.assertEqual(result["dominant_selected_label"], "unit")
        self.assertTrue(result["selector_label_stable"])
        self.assertFalse(result["runtime_truth_used"])
        self.assertIn("RDF(right,down,forward)", result["coordinate_contract"])

    def test_selector_windows_rejects_truth_column(self) -> None:
        module = load_module()
        with tempfile.TemporaryDirectory() as tmpdir:
            root = Path(tmpdir)
            model, catalog = self.write_selector(root)
            observations = root / "observations.csv"
            self.write_observations(observations, include_truth=True)

            result = module.selector_windows(
                input_csv=observations,
                selector_model_json=model,
                catalog_json=catalog,
                window_size=3,
                stride=2,
                min_samples=3,
                expected_label="unit",
            )

        self.assertEqual(result["status"], "FAIL")
        self.assertIn("forbidden_truth_column:px4_ned_north_m", result["failures"])

    def test_run_writes_outputs(self) -> None:
        module = load_module()
        with tempfile.TemporaryDirectory() as tmpdir:
            root = Path(tmpdir)
            model, catalog = self.write_selector(root)
            observations = root / "observations.csv"
            out = root / "out"
            self.write_observations(observations)

            result = module.run(
                [
                    "--input-csv",
                    str(observations),
                    "--selector-model-json",
                    str(model),
                    "--catalog-json",
                    str(catalog),
                    "--output-dir",
                    str(out),
                    "--window-size",
                    "3",
                    "--stride",
                    "2",
                    "--min-samples",
                    "3",
                    "--expected-label",
                    "unit",
                ]
            )
            summary_exists = (out / "observation_window_selector_replay_summary.json").is_file()
            replay_exists = (out / "observation_window_selector_replay.csv").is_file()
            txt = (out / "observation_window_selector_replay_summary.txt").read_text(encoding="utf-8")

        self.assertEqual(result["status"], "PASS")
        self.assertTrue(summary_exists)
        self.assertTrue(replay_exists)
        self.assertIn("status: PASS", txt)

    def test_selector_windows_can_require_stable_label(self) -> None:
        module = load_module()
        with tempfile.TemporaryDirectory() as tmpdir:
            root = Path(tmpdir)
            model, catalog = self.write_two_label_selector(root)
            observations = root / "observations.csv"
            self.write_unstable_observations(observations)

            result = module.selector_windows(
                input_csv=observations,
                selector_model_json=model,
                catalog_json=catalog,
                window_size=3,
                stride=3,
                min_samples=3,
                require_stable_label=True,
            )

        self.assertEqual(result["status"], "FAIL")
        self.assertFalse(result["selector_label_stable"])
        self.assertEqual(result["selected_label_counts"], {"far": 1, "near": 1})
        self.assertIn("selector:unstable_label", result["failures"])

    def test_selector_windows_can_require_min_distance_margin(self) -> None:
        module = load_module()
        with tempfile.TemporaryDirectory() as tmpdir:
            root = Path(tmpdir)
            model, catalog = self.write_selector(root)
            observations = root / "observations.csv"
            self.write_observations(observations)

            result = module.selector_windows(
                input_csv=observations,
                selector_model_json=model,
                catalog_json=catalog,
                window_size=3,
                stride=2,
                min_samples=3,
                expected_label="unit",
                min_distance_margin=1000000001.0,
            )

        self.assertEqual(result["status"], "FAIL")
        self.assertEqual(result["min_distance_margin"], 1000000001.0)
        self.assertTrue(any(str(failure).startswith("selector:distance_margin_below_threshold") for failure in result["failures"]))


if __name__ == "__main__":
    unittest.main()
