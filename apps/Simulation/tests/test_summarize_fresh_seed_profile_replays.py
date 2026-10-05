from __future__ import annotations

import importlib.util
import json
import sys
import tempfile
import unittest
from pathlib import Path


ROOT_DIR = Path(__file__).resolve().parents[1]
SCRIPT_PATH = ROOT_DIR / "scripts" / "summarize_fresh_seed_profile_replays.py"


def load_module():
    spec = importlib.util.spec_from_file_location("summarize_fresh_seed_profile_replays_under_test", SCRIPT_PATH)
    if spec is None or spec.loader is None:
        raise RuntimeError("cannot load summarize_fresh_seed_profile_replays.py")
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def write_replay(
    replay_dir: Path,
    *,
    experiment_id: str,
    status: str,
    pass_count: int,
    trial_count: int,
    target_max: float,
    range_mean: float,
    position_mean: float,
    runtime_truth_used: bool = False,
) -> None:
    replay_dir.mkdir(parents=True)
    replay_dir.joinpath("fresh_seed_replay_metrics.json").write_text(
        json.dumps(
            {
                "experiment_id": experiment_id,
                "profile_json": "profile.json",
                "profile_id": f"profile-{experiment_id}",
                "status": status,
                "pass_count": pass_count,
                "trial_count": trial_count,
                "range_rmse_raw_mean_m": 3.0,
                "range_rmse_corrected_mean_m": range_mean,
                "range_rmse_corrected_max_m": range_mean + 0.5,
                "position_rmse_raw_mean_m": 8.0,
                "position_rmse_corrected_mean_m": position_mean,
                "position_rmse_corrected_max_m": position_mean + 0.5,
                "target_max_abs_error_corrected_max_m": target_max,
                "target_rmse_corrected_mean_m": target_max / 4.0,
                "invalid_count_total": 0,
                "fallback_count_total": 0,
                "runtime_truth_used": runtime_truth_used,
                "rdf_axis_contract": "stereo_center_opencv_optical:RDF(right,down,forward)",
                "public_topic_contract": "stereodistance_position_topics:FRD_range(forward,right,down,range)",
            }
        ),
        encoding="utf-8",
    )
    replay_dir.joinpath("fresh_seed_replay_scenario_summary.csv").write_text(
        "scenario,status,pass_count,trial_count,target_max_abs_error_corrected_max_m\n"
        f"nominal,{status},{pass_count},{trial_count},{target_max}\n",
        encoding="utf-8",
    )


class SummarizeFreshSeedProfileReplayTests(unittest.TestCase):
    def test_summary_recommends_strict_pass_before_lower_rmse_fail(self) -> None:
        module = load_module()
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            write_replay(
                root / "strict",
                experiment_id="strict",
                status="STRICT_PASS",
                pass_count=10,
                trial_count=10,
                target_max=8.0,
                range_mean=2.0,
                position_mean=4.0,
            )
            write_replay(
                root / "fail",
                experiment_id="fail",
                status="FAIL",
                pass_count=9,
                trial_count=10,
                target_max=4.0,
                range_mean=1.0,
                position_mean=2.0,
            )

            row_a, _ = module.summarize_replay_dir(root / "strict")
            row_b, _ = module.summarize_replay_dir(root / "fail")
            summary = module.build_summary([row_b, row_a])

            self.assertEqual(summary["status"], "FAIL")
            self.assertEqual(summary["strict_pass_count"], 1)
            self.assertEqual(summary["recommended_replay_label"], "strict")
            self.assertEqual(summary["runtime_truth_used"], False)

    def test_summary_flags_target_tail_generalization_gap(self) -> None:
        module = load_module()
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            write_replay(
                root / "candidate",
                experiment_id="candidate",
                status="FAIL",
                pass_count=8,
                trial_count=10,
                target_max=12.5,
                range_mean=1.5,
                position_mean=3.0,
            )

            row, scenarios = module.summarize_replay_dir(root / "candidate")
            summary = module.build_summary([row])

            self.assertEqual(len(scenarios), 1)
            self.assertEqual(summary["status"], "FAIL")
            self.assertFalse(summary["all_target_max_within_10m"])
            self.assertEqual(summary["recommendation"], "fresh_seed_generalization_gap_continue_search")


if __name__ == "__main__":
    unittest.main()
