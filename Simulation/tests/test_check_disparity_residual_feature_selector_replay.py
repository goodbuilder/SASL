from __future__ import annotations

import importlib.util
import json
import sys
import tempfile
import unittest
from pathlib import Path


ROOT_DIR = Path(__file__).resolve().parents[1]
SCRIPT_PATH = ROOT_DIR / "scripts" / "check_disparity_residual_feature_selector_replay.py"


def load_module():
    spec = importlib.util.spec_from_file_location(
        "check_disparity_residual_feature_selector_replay_under_test",
        SCRIPT_PATH,
    )
    if spec is None or spec.loader is None:
        raise RuntimeError(f"cannot load {SCRIPT_PATH}")
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


class CheckDisparityResidualFeatureSelectorReplayTests(unittest.TestCase):
    def write_residual_profile(self, path: Path) -> None:
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(
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

    def write_selector(self, root: Path) -> tuple[Path, Path, Path]:
        model = root / "selector_model.json"
        catalog = root / "catalog.json"
        profile = root / "profiles" / "unit.json"
        self.write_residual_profile(profile)
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
        return model, catalog, profile

    def test_replay_passes_truth_free_validation_rows(self) -> None:
        module = load_module()
        with tempfile.TemporaryDirectory() as tmpdir:
            root = Path(tmpdir)
            model, catalog, _ = self.write_selector(root)
            features = root / "features.csv"
            features.write_text(
                "\n".join(
                    [
                        "scenario,selector_split,raw_forward_m_mean,runtime_truth_used",
                        "unit,train,800.0,False",
                        "unit,validation,800.2,False",
                    ]
                )
                + "\n",
                encoding="utf-8",
            )

            result = module.replay_selector(
                selector_model_json=model,
                catalog_json=catalog,
                feature_csv=features,
            )

        self.assertEqual(result["status"], "PASS")
        self.assertEqual(result["replay_row_count"], 1)
        self.assertEqual(result["label_match_count"], 1)
        self.assertEqual(result["profile_exists_count"], 1)
        self.assertFalse(result["runtime_truth_used"])
        self.assertIn("RDF(right,down,forward)", result["coordinate_contract"])

    def test_replay_rejects_forbidden_truth_column(self) -> None:
        module = load_module()
        with tempfile.TemporaryDirectory() as tmpdir:
            root = Path(tmpdir)
            model, catalog, _ = self.write_selector(root)
            features = root / "features.csv"
            features.write_text(
                "\n".join(
                    [
                        "scenario,selector_split,raw_forward_m_mean,px4_truth_range_m,runtime_truth_used",
                        "unit,validation,800.2,799.0,False",
                    ]
                )
                + "\n",
                encoding="utf-8",
            )

            result = module.replay_selector(
                selector_model_json=model,
                catalog_json=catalog,
                feature_csv=features,
            )

        self.assertEqual(result["status"], "FAIL")
        self.assertIn("feature_csv:forbidden_column", result["failures"])

    def test_run_writes_outputs(self) -> None:
        module = load_module()
        with tempfile.TemporaryDirectory() as tmpdir:
            root = Path(tmpdir)
            model, catalog, _ = self.write_selector(root)
            features = root / "features.csv"
            output_json = root / "replay.json"
            output_csv = root / "replay.csv"
            output_txt = root / "replay.txt"
            features.write_text(
                "\n".join(
                    [
                        "scenario,selector_split,raw_forward_m_mean,runtime_truth_used",
                        "unit,validation,800.2,False",
                    ]
                )
                + "\n",
                encoding="utf-8",
            )

            result = module.run(
                [
                    "--selector-model-json",
                    str(model),
                    "--catalog-json",
                    str(catalog),
                    "--feature-csv",
                    str(features),
                    "--output-json",
                    str(output_json),
                    "--output-csv",
                    str(output_csv),
                    "--output-txt",
                    str(output_txt),
                ]
            )
            output_json_exists = output_json.is_file()
            output_csv_exists = output_csv.is_file()
            output_txt_text = output_txt.read_text(encoding="utf-8")

        self.assertEqual(result["status"], "PASS")
        self.assertTrue(output_json_exists)
        self.assertTrue(output_csv_exists)
        self.assertIn("status: PASS", output_txt_text)


if __name__ == "__main__":
    unittest.main()
