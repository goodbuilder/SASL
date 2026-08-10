from __future__ import annotations

import importlib.util
import sys
import tempfile
import unittest
from pathlib import Path


ROOT_DIR = Path(__file__).resolve().parents[1]
SCRIPT_PATH = ROOT_DIR / "scripts" / "export_disparity_residual_feature_row_from_observations.py"
SPEC = importlib.util.spec_from_file_location("export_disparity_residual_feature_row_from_observations_under_test", SCRIPT_PATH)
module = importlib.util.module_from_spec(SPEC)
assert SPEC is not None and SPEC.loader is not None
sys.modules[SPEC.name] = module
SPEC.loader.exec_module(module)


class ExportDisparityResidualFeatureRowFromObservationsTests(unittest.TestCase):
    def test_exports_truth_free_feature_row_with_rdf_contract(self) -> None:
        with tempfile.TemporaryDirectory() as tmpdir:
            root = Path(tmpdir)
            csv_path = root / "observations.csv"
            csv_path.write_text(
                "\n".join(
                    [
                        "left_u_px,left_v_px,measured_disparity_px,fused_disparity_px,raw_right_m,raw_down_m,raw_forward_m,observation_right_m,observation_down_m,observation_forward_m",
                        "640,360,10,9.8,1,2,100,1.1,2.1,101",
                        "650,362,8,8.2,2,3,120,2.2,3.2,121",
                    ]
                )
                + "\n",
                encoding="utf-8",
            )

            summary = module.export_feature_row(input_csv=csv_path, output_dir=root / "out", feature_label="unit")
            feature = module.json.loads(Path(summary["feature_json"]).read_text(encoding="utf-8"))

        self.assertEqual(summary["status"], "PASS")
        self.assertEqual(feature["feature_label"], "unit")
        self.assertEqual(feature["observation_rdf_source"], "observation_rdf_columns")
        self.assertAlmostEqual(feature["measured_disparity_px_mean"], 9.0)
        self.assertIn("RDF(right,down,forward)", feature["axis_contract"])
        self.assertFalse(feature["runtime_truth_used"])

    def test_rejects_truth_column(self) -> None:
        with tempfile.TemporaryDirectory() as tmpdir:
            root = Path(tmpdir)
            csv_path = root / "observations.csv"
            csv_path.write_text(
                "\n".join(
                    [
                        "left_u_px,left_v_px,measured_disparity_px,raw_right_m,raw_down_m,raw_forward_m,px4_north_m",
                        "640,360,10,1,2,100,100",
                    ]
                )
                + "\n",
                encoding="utf-8",
            )

            summary = module.export_feature_row(
                input_csv=csv_path,
                output_dir=root / "out",
                allow_missing_observation_rdf=True,
            )

        self.assertEqual(summary["status"], "FAIL")
        self.assertIn("forbidden_truth_column:px4_north_m", summary["failures"])


if __name__ == "__main__":
    unittest.main()
