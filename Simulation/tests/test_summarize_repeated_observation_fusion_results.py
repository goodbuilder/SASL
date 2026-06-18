from __future__ import annotations

import importlib.util
import json
import sys
import tempfile
import unittest
from pathlib import Path


ROOT_DIR = Path(__file__).resolve().parents[1]
SCRIPT_PATH = ROOT_DIR / "scripts" / "summarize_repeated_observation_fusion_results.py"


def load_module():
    spec = importlib.util.spec_from_file_location("summarize_repeated_observation_fusion_results_under_test", SCRIPT_PATH)
    if spec is None or spec.loader is None:
        raise RuntimeError(f"cannot load {SCRIPT_PATH}")
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def write_result(
    result_dir: Path,
    *,
    scenario: str,
    status: str,
    pass_count: int,
    target_max: float,
) -> None:
    result_dir.mkdir(parents=True)
    result_dir.joinpath("repeated_observation_metrics.json").write_text(
        json.dumps(
            {
                "experiment_id": f"exp-{scenario}",
                "scenario": scenario,
                "profile_json": "profile.json",
                "runtime_input_contract": "runtime pixels and disparity only",
                "rdf_axis_contract": "stereo_center_opencv_optical:RDF(right,down,forward)",
                "public_topic_contract": "stereodistance_position_topics:FRD_range(forward,right,down,range)",
                "best_method": {
                    "method": "repeated_observation_trimmed_mean_n7_trim0p2",
                    "status": status,
                    "pass_count": pass_count,
                    "trial_count": 10,
                    "repeat_count": 7,
                    "fusion_mode": "trimmed_mean",
                    "trim_fraction": 0.2,
                    "range_rmse_raw_mean_m": 3.0,
                    "range_rmse_corrected_mean_m": 1.0,
                    "position_rmse_raw_mean_m": 5.0,
                    "position_rmse_corrected_mean_m": 2.0,
                    "target_max_abs_error_corrected_max_m": target_max,
                    "target_rmse_corrected_mean_m": 0.8,
                    "runtime_truth_used": False,
                    "rdf_axis_contract": "stereo_center_opencv_optical:RDF(right,down,forward)",
                    "public_topic_contract": "stereodistance_position_topics:FRD_range(forward,right,down,range)",
                },
            }
        ),
        encoding="utf-8",
    )


class SummarizeRepeatedObservationFusionResultsTests(unittest.TestCase):
    def test_discovers_results_and_writes_summary_outputs(self) -> None:
        module = load_module()
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            write_result(root / "nominal", scenario="nominal", status="STRICT_PASS", pass_count=10, target_max=2.0)
            write_result(root / "candidate", scenario="intrinsic", status="FAIL", pass_count=0, target_max=8.0)
            rows = [module.row_from_result_dir(path) for path in module.discover_result_dirs(root)]
            output_dir = root / "out"

            module.write_csv(output_dir / "summary.csv", rows)
            module.write_summary_txt(output_dir / "summary.txt", rows)

            self.assertEqual(len(rows), 2)
            self.assertEqual(rows[0]["runtime_truth_used"], False)
            self.assertIn("RDF(right,down,forward)", rows[0]["rdf_axis_contract"])
            self.assertTrue((output_dir / "summary.csv").exists())
            summary_text = (output_dir / "summary.txt").read_text(encoding="utf-8")
            self.assertIn("strict_pass_count: 1/2", summary_text)
            self.assertIn("runtime_truth_used: False", summary_text)


if __name__ == "__main__":
    unittest.main()
