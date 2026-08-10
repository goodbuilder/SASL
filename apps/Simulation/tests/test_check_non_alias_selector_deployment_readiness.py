from __future__ import annotations

import csv
import importlib.util
import json
import sys
import tempfile
import unittest
from pathlib import Path


ROOT_DIR = Path(__file__).resolve().parents[1]
SCRIPT_PATH = ROOT_DIR / "scripts" / "check_non_alias_selector_deployment_readiness.py"


def load_module():
    spec = importlib.util.spec_from_file_location("check_non_alias_selector_deployment_readiness_under_test", SCRIPT_PATH)
    if spec is None or spec.loader is None:
        raise RuntimeError(f"cannot load {SCRIPT_PATH}")
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def write_json(path: Path, payload: object) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(payload, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")


def write_csv(path: Path, rows: list[dict[str, object]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    fieldnames = list(dict.fromkeys(key for row in rows for key in row.keys()))
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)


def make_fixture(root: Path) -> tuple[Path, Path, Path]:
    robustness = root / "robustness"
    selector = root / "selector"
    readme = root / "README.md"
    scenario_rows = [
        {
            "scenario": "nominal",
            "profile_exists": True,
            "pass_cross_seed_replay": True,
            "axis_contract": "stereo_center_opencv_optical:RDF(right,down,forward)",
            "runtime_input_contract": "raw stereo-center RDF plus measured disparity; no runtime truth",
        },
        {
            "scenario": "stress",
            "profile_exists": True,
            "pass_cross_seed_replay": True,
            "axis_contract": "stereo_center_opencv_optical:RDF(right,down,forward)",
            "runtime_input_contract": "raw stereo-center RDF plus measured disparity; no runtime truth",
        },
    ]
    write_json(
        robustness / "robustness_summary.json",
        {
            "scenario_count": 2,
            "coordinate_contract_summary": {
                "loaded_contract_count": 4,
                "direct_axis_alias_forbidden_count": 4,
                "all_scenarios_forbid_direct_axis_alias": True,
                "truth_rdf_transform_contract": (
                    "truth_rdf must be stereo_center_RDF from explicit T_source_to_stereo_rdf "
                    "or an already stereo-frame truth producer; direct NED-to-RDF axis alias is forbidden"
                ),
            },
        },
    )
    write_json(
        robustness / "scenario_profile_catalog.json",
        {
            "pass_scenario_profile_catalog": True,
            "catalog_profile_count": 2,
            "profile_exists_count": 2,
            "cross_seed_pass_count": 2,
            "rows": scenario_rows,
        },
    )
    write_csv(
        robustness / "scenario_coordinate_contracts.csv",
        [
            {
                "scenario": "nominal",
                "all_loaded_contracts_forbid_direct_axis_alias": True,
                "truth_rdf_transform_contract": (
                    "truth_rdf must be stereo_center_RDF from explicit T_source_to_stereo_rdf "
                    "or an already stereo-frame truth producer; direct NED-to-RDF axis alias is forbidden"
                ),
                "axis_contract": "stereo_center_opencv_optical:RDF(right,down,forward)",
            },
            {
                "scenario": "stress",
                "all_loaded_contracts_forbid_direct_axis_alias": True,
                "truth_rdf_transform_contract": (
                    "truth_rdf must be stereo_center_RDF from explicit T_source_to_stereo_rdf "
                    "or an already stereo-frame truth producer; direct NED-to-RDF axis alias is forbidden"
                ),
                "axis_contract": "stereo_center_opencv_optical:RDF(right,down,forward)",
            },
        ],
    )
    write_json(
        selector / "selector_package_export_summary.json",
        {
            "status": "PASS",
            "profile_count": 2,
            "deployment_label_count": 6,
            "runtime_truth_used": False,
        },
    )
    write_json(
        selector / "runtime_profile_label_selector.json",
        {
            "mode": "stereodistance_runtime_profile_label_selector",
            "label_profile_map": {"nominal": "scenario_nominal", "stress": "scenario_stress"},
            "profile_paths": {"scenario_nominal": "packages/nominal/active_runtime_profile.json", "scenario_stress": "packages/stress/active_runtime_profile.json"},
            "package_paths": {"scenario_nominal": "packages/nominal", "scenario_stress": "packages/stress"},
            "selection_input_fields": ["deployment_label", "scenario_label", "scenario_name"],
            "axis_contract": "stereo_center_opencv_optical:RDF(right,down,forward)",
        },
    )
    write_json(
        selector / "runtime_deployment_frontier" / "runtime_deployment_frontier.json",
        {
            "summary": {
                "status": "PASS",
                "truth_free_runtime_selection": True,
                "runtime_truth_used": False,
                "profile_count": 2,
                "deployment_label_count": 7,
                "axis_contract": "stereo_center_opencv_optical:RDF(right,down,forward)",
            }
        },
    )
    operator_rows = [
        {"deployment_label": "__default__", "status": "PASS", "bbox_adjustment_arg_present": True, "runtime_truth_used": False},
        {"deployment_label": "nominal", "status": "PASS", "bbox_adjustment_arg_present": True, "runtime_truth_used": False},
        {"deployment_label": "stress", "status": "PASS", "bbox_adjustment_arg_present": True, "runtime_truth_used": False},
    ]
    write_json(
        selector / "operator_view_launch_checks" / "operator_view_launch_checks.json",
        {
            "summary": {
                "status": "PASS",
                "check_count": 3,
                "pass_count": 3,
                "bbox_adjustment_arg_count": 3,
                "runtime_truth_used_count": 0,
                "runtime_truth_used": False,
            },
            "rows": operator_rows,
        },
    )
    readme.write_text(
        "PX4/RTK/NED truth, validation error, target-window label, replay metric\n"
        "synthetic mode에서도 camera optical frame forward=north right=east down=down 가정하면 안 된다\n"
        "stereo-center RDF(right,down,forward)\n"
        "FRD+range(forward,right,down,range)\n",
        encoding="utf-8",
    )
    return robustness, selector, readme


class CheckNonAliasSelectorDeploymentReadinessTests(unittest.TestCase):
    def test_readiness_passes_complete_fixture(self) -> None:
        module = load_module()
        with tempfile.TemporaryDirectory() as tmpdir:
            robustness, selector, readme = make_fixture(Path(tmpdir))

            result = module.check_readiness(
                robustness_dir=robustness,
                selector_package_dir=selector,
                readme=readme,
            )

        self.assertEqual(result["status"], "PASS")
        self.assertEqual(result["pass_count"], result["requirement_count"])
        self.assertFalse(result["runtime_truth_used"])

    def test_readiness_fails_when_operator_launch_uses_truth(self) -> None:
        module = load_module()
        with tempfile.TemporaryDirectory() as tmpdir:
            robustness, selector, readme = make_fixture(Path(tmpdir))
            payload = json.loads(
                (selector / "operator_view_launch_checks" / "operator_view_launch_checks.json").read_text(encoding="utf-8")
            )
            payload["summary"]["runtime_truth_used_count"] = 1
            payload["summary"]["runtime_truth_used"] = True
            payload["rows"][0]["runtime_truth_used"] = True
            write_json(selector / "operator_view_launch_checks" / "operator_view_launch_checks.json", payload)

            result = module.check_readiness(
                robustness_dir=robustness,
                selector_package_dir=selector,
                readme=readme,
            )

        self.assertEqual(result["status"], "FAIL")
        self.assertIn("REQ-05-operator-launch", result["failures"])
        self.assertIn("REQ-06-runtime-truth-free", result["failures"])


if __name__ == "__main__":
    unittest.main()
