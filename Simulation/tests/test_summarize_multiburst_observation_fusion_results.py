from __future__ import annotations

import importlib.util
import json
import sys
import tempfile
import unittest
from pathlib import Path


ROOT_DIR = Path(__file__).resolve().parents[1]
SCRIPT_PATH = ROOT_DIR / "scripts" / "summarize_multiburst_observation_fusion_results.py"


def load_module():
    spec = importlib.util.spec_from_file_location("summarize_multiburst_observation_fusion_results_under_test", SCRIPT_PATH)
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
    result_dir.joinpath("multiburst_observation_metrics.json").write_text(
        json.dumps(
            {
                "experiment_id": f"exp-{scenario}",
                "scenario": scenario,
                "profile_json": "profile.json",
                "runtime_input_contract": "runtime burst pixels and disparity only",
                "rdf_axis_contract": "stereo_center_opencv_optical:RDF(right,down,forward)",
                "public_topic_contract": "stereodistance_position_topics:FRD_range(forward,right,down,range)",
                "best_method": {
                    "method": "multiburst_observation_inner_trimmed_mean_outer_median_b5x5_trim0p2_common0p5",
                    "status": status,
                    "pass_count": pass_count,
                    "trial_count": 10,
                    "burst_count": 5,
                    "observations_per_burst": 5,
                    "total_observation_count": 25,
                    "inner_mode": "trimmed_mean",
                    "outer_mode": "median",
                    "trim_fraction": 0.2,
                    "common_fraction": 0.5,
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


def write_method_summary(result_dir: Path, *, scenario: str) -> None:
    result_dir.joinpath("multiburst_observation_method_summary.csv").write_text(
        "\n".join(
            [
                "method,status,pass_count,trial_count,burst_count,observations_per_burst,total_observation_count,inner_mode,outer_mode,trim_fraction,common_fraction,range_rmse_raw_mean_m,range_rmse_corrected_mean_m,range_rmse_corrected_max_m,position_rmse_raw_mean_m,position_rmse_corrected_mean_m,position_rmse_corrected_max_m,target_max_abs_error_corrected_max_m,target_rmse_corrected_mean_m,runtime_truth_used,runtime_input_contract,image_contract,rdf_axis_contract,public_topic_contract,scenario",
                f"b3,STRICT_PASS,10,10,3,5,15,trimmed_mean,median,0.2,0.5,3.0,1.2,1.4,5.0,2.4,2.6,3.0,1.0,False,runtime,image,RDF,FRD,{scenario}",
                f"b5,STRICT_PASS,10,10,5,5,25,trimmed_mean,median,0.2,0.5,3.0,0.9,1.0,5.0,2.0,2.2,2.0,0.8,False,runtime,image,RDF,FRD,{scenario}",
            ]
        )
        + "\n",
        encoding="utf-8",
    )


class SummarizeMultiburstObservationFusionResultsTests(unittest.TestCase):
    def test_discovers_results_and_writes_summary_outputs(self) -> None:
        module = load_module()
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            write_result(root / "bbox", scenario="bbox", status="STRICT_PASS", pass_count=10, target_max=2.0)
            write_result(root / "candidate", scenario="candidate", status="FAIL", pass_count=0, target_max=8.0)
            rows = [module.row_from_result_dir(path) for path in module.discover_result_dirs(root)]
            output_dir = root / "out"

            module.write_csv(output_dir / "summary.csv", rows)
            module.write_summary_txt(output_dir / "summary.txt", rows)

            self.assertEqual(len(rows), 2)
            self.assertEqual(rows[0]["runtime_truth_used"], False)
            self.assertEqual(rows[0]["total_observation_count"], 25)
            self.assertIn("RDF(right,down,forward)", rows[0]["rdf_axis_contract"])
            self.assertTrue((output_dir / "summary.csv").exists())
            summary_text = (output_dir / "summary.txt").read_text(encoding="utf-8")
            self.assertIn("strict_pass_count: 1/2", summary_text)
            self.assertIn("runtime_truth_used: False", summary_text)

    def test_writes_latency_frontier_and_method_aggregate(self) -> None:
        module = load_module()
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            write_result(root / "bbox", scenario="bbox", status="STRICT_PASS", pass_count=10, target_max=2.0)
            write_method_summary(root / "bbox", scenario="bbox")
            method_rows = [
                row
                for path in module.discover_result_dirs(root)
                for row in module.method_rows_from_result_dir(path)
            ]
            frontier = module.latency_frontier_rows(method_rows)
            aggregates = module.aggregate_method_rows(method_rows)
            output_dir = root / "out"

            module.write_csv(output_dir / "frontier.csv", frontier)
            module.write_csv(output_dir / "aggregate.csv", aggregates)
            module.write_frontier_txt(output_dir / "frontier.txt", frontier_rows=frontier, aggregate_rows=aggregates)

            self.assertEqual([row["total_observation_count"] for row in frontier], [15, 25])
            self.assertEqual(aggregates[0]["method"], "b5")
            self.assertEqual(aggregates[0]["target_max_abs_error_worst_m"], 2.0)
            frontier_text = (output_dir / "frontier.txt").read_text(encoding="utf-8")
            self.assertIn("runtime_truth_used: False", frontier_text)
            self.assertIn("Best aggregate method:", frontier_text)


if __name__ == "__main__":
    unittest.main()
