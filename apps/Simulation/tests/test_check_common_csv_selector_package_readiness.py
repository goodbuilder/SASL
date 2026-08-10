from __future__ import annotations

import importlib.util
import json
import sys
import tempfile
import unittest
from pathlib import Path


ROOT_DIR = Path(__file__).resolve().parents[1]
SCRIPT_PATH = ROOT_DIR / "scripts" / "check_common_csv_selector_package_readiness.py"


def load_module():
    spec = importlib.util.spec_from_file_location("check_common_csv_selector_package_readiness_under_test", SCRIPT_PATH)
    if spec is None or spec.loader is None:
        raise RuntimeError(f"cannot load {SCRIPT_PATH}")
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def write_json(path: Path, payload: object) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(payload, ensure_ascii=False), encoding="utf-8")


class CheckCommonCsvSelectorPackageReadinessTests(unittest.TestCase):
    def test_readiness_passes_for_truth_free_selector_package(self) -> None:
        module = load_module()
        with tempfile.TemporaryDirectory() as tmpdir:
            root = Path(tmpdir)
            package = root / "selector_package"
            package.mkdir()
            write_json(
                package / "selector_package_export_summary.json",
                {"status": "PASS", "runtime_truth_used": False},
            )
            write_json(
                package / "runtime_profile_label_selector.json",
                {
                    "mode": "stereodistance_runtime_profile_label_selector",
                    "runtime_truth_used": False,
                    "axis_contract": "stereo_center_opencv_optical:RDF(right,down,forward)",
                    "public_topic_contract": "stereodistance_position_topics:FRD_range(forward,right,down,range)",
                },
            )
            (package / "selector_package_profiles.csv").write_text("profile_label\np\n", encoding="utf-8")
            (package / "selector_package_labels.csv").write_text("deployment_label\nscenario\n", encoding="utf-8")
            portfolio = root / "portfolio.json"
            write_json(
                portfolio,
                {
                    "status": "PASS",
                    "recommended_deployable_selector_mode": "scenario_loso",
                    "runtime_truth_used": False,
                    "selector_modes": [
                        {
                            "selector_mode": "global_single",
                            "pass_all_runs": True,
                            "validation_range_rmse_corrected_mean_m": 2.0,
                            "validation_range_rmse_corrected_max_m": 3.0,
                            "target_max_abs_error_corrected_max_m": 4.0,
                        },
                        {
                            "selector_mode": "scenario_loso",
                            "pass_all_runs": True,
                            "target_limit_pass": True,
                            "validation_range_rmse_corrected_mean_m": 1.5,
                            "validation_range_rmse_corrected_max_m": 2.5,
                            "target_max_abs_error_corrected_max_m": 3.5,
                        },
                    ],
                },
            )
            frontier = root / "frontier.json"
            write_json(
                frontier,
                {
                    "summary": {
                        "status": "PASS",
                        "runtime_truth_used": False,
                        "truth_free_runtime_selection": True,
                        "profile_count": 2,
                        "deployment_label_count": 2,
                    }
                },
            )
            operator = root / "operator.json"
            write_json(
                operator,
                {
                    "status": "PASS",
                    "source_mode": "selector",
                    "label_selection_status": "PASS",
                    "package_check_status": "PASS",
                    "bbox_adjustment_arg_present": True,
                    "runtime_truth_used": False,
                    "selected_label": "scenario_a",
                    "package_type": "lightweight_profile_package",
                },
            )
            selection = root / "selection.json"
            write_json(
                selection,
                {
                    "status": "PASS",
                    "deployment_label": "scenario_a",
                    "label_selection_applied": True,
                    "runtime_truth_used": False,
                    "axis_contract": "stereo_center_opencv_optical:RDF(right,down,forward)",
                },
            )

            result = module.check_readiness(
                selector_package_dir=package,
                portfolio_summary_json=portfolio,
                frontier_json=frontier,
                operator_check_json=operator,
                selection_jsons=[selection],
            )

            self.assertEqual(result["status"], "PASS")
            self.assertEqual(result["recommended_deployable_selector_mode"], "scenario_loso")
            self.assertFalse(result["runtime_truth_used"])


if __name__ == "__main__":
    unittest.main()
