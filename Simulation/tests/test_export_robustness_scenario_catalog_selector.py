from __future__ import annotations

import csv
import importlib.util
import json
import sys
import tempfile
import unittest
from pathlib import Path


ROOT_DIR = Path(__file__).resolve().parents[1]
SCRIPT_PATH = ROOT_DIR / "scripts" / "export_robustness_scenario_catalog_selector.py"
SELECT_SCRIPT_PATH = ROOT_DIR / "scripts" / "select_stereodistance_runtime_profile_by_label.py"


def load_module(path: Path, name: str):
    spec = importlib.util.spec_from_file_location(name, path)
    if spec is None or spec.loader is None:
        raise RuntimeError(f"cannot load {path}")
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


class ExportRobustnessScenarioCatalogSelectorTests(unittest.TestCase):
    def write_profile(self, path: Path, method: str) -> None:
        write_json(
            path,
            {
                "profile_id": f"rtk_stereo_calibration-{method}",
                "method": method,
                "mode": "estimated_to_truth_sim3",
                "axis_contract": "internal_policy_vector=RDF(right,down,forward)",
                "runtime_inputs": ["raw_right_m", "raw_down_m", "raw_forward_m", "raw_disparity_px"],
                "forbidden_runtime_inputs": ["calibration_truth", "px4_rtk_truth"],
                "scale": 1.0,
                "rotation_matrix_rdf": [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]],
                "translation_rdf_m": [0.0, 0.0, 0.0],
            },
        )

    def make_robustness_fixture(self, root: Path) -> Path:
        robustness = root / "robustness"
        catalog_dir = robustness / "scenario_profile_catalog"
        self.write_profile(catalog_dir / "nominal_runtime_profile.json", "sim3_nominal")
        self.write_profile(catalog_dir / "stress_runtime_profile.json", "sim3_stress")
        rows = [
            {
                "scenario": "nominal",
                "method": "sim3_nominal",
                "catalog_profile_path": "scenario_profile_catalog/nominal_runtime_profile.json",
                "profile_exists": True,
                "pass_cross_seed_replay": True,
                "axis_contract": "stereo_center_opencv_optical:RDF(right,down,forward)",
                "runtime_input_contract": (
                    "scenario-specific raw stereo-center RDF(right,down,forward) plus measured disparity; "
                    "no runtime truth"
                ),
            },
            {
                "scenario": "stress",
                "method": "sim3_stress",
                "catalog_profile_path": "scenario_profile_catalog/stress_runtime_profile.json",
                "profile_exists": True,
                "pass_cross_seed_replay": True,
                "axis_contract": "stereo_center_opencv_optical:RDF(right,down,forward)",
                "runtime_input_contract": (
                    "scenario-specific raw stereo-center RDF(right,down,forward) plus measured disparity; "
                    "no runtime truth"
                ),
            },
        ]
        write_json(
            robustness / "scenario_profile_catalog.json",
            {
                "catalog_profile_count": 2,
                "profile_exists_count": 2,
                "cross_seed_pass_count": 2,
                "pass_scenario_profile_catalog": True,
                "rows": rows,
            },
        )
        write_json(
            robustness / "robustness_summary.json",
            {
                "scenario_count": 2,
                "robustness_acceptance_pass": True,
                "runtime_truth_used": False,
                "coordinate_contract_summary": {
                    "scenario_count": 2,
                    "summary_loaded_scenario_count": 2,
                    "scenario_direct_axis_alias_forbidden_count": 2,
                    "loaded_contract_count": 2,
                    "direct_axis_alias_forbidden_count": 2,
                    "all_scenarios_forbid_direct_axis_alias": True,
                    "truth_rdf_transform_contract": (
                        "truth_rdf must be stereo_center_RDF from explicit T_source_to_stereo_rdf "
                        "or an already stereo-frame truth producer; direct NED-to-RDF axis alias is forbidden"
                    ),
                },
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
        return robustness

    def test_exports_truth_free_label_selector_from_robustness_catalog(self) -> None:
        module = load_module(SCRIPT_PATH, "export_robustness_scenario_catalog_selector_under_test")
        selector_module = load_module(SELECT_SCRIPT_PATH, "select_runtime_profile_by_label_for_catalog_export_test")
        with tempfile.TemporaryDirectory() as tmpdir:
            root = Path(tmpdir)
            robustness = self.make_robustness_fixture(root)
            output_dir = root / "selector_package"

            summary = module.export_selector_package(
                robustness_dir=robustness,
                output_dir=output_dir,
                baseline_scenario="nominal",
            )
            selector_json = Path(summary["selector_json"])
            manifest = json.loads(selector_json.read_text(encoding="utf-8"))
            stress_selection = selector_module.select_runtime_profile_by_label(
                selector_json=selector_json,
                deployment_label="stress",
            )
            fallback_selection = selector_module.select_runtime_profile_by_label(
                selector_json=selector_json,
                deployment_label="unknown",
            )

        self.assertEqual(summary["status"], "PASS")
        self.assertEqual(summary["profile_count"], 2)
        self.assertFalse(summary["runtime_truth_used"])
        self.assertEqual(manifest["selection_input_fields"], ["deployment_label", "scenario_label", "scenario_name"])
        self.assertNotIn("truth", " ".join(manifest["selection_input_fields"]))
        self.assertEqual(stress_selection["status"], "PASS")
        self.assertEqual(stress_selection["method"], "sim3_stress")
        self.assertFalse(stress_selection["runtime_truth_used"])
        self.assertEqual(fallback_selection["method"], "sim3_nominal")

    def test_rejects_profile_with_truth_runtime_input(self) -> None:
        module = load_module(SCRIPT_PATH, "export_robustness_scenario_catalog_selector_bad_profile_test")
        with tempfile.TemporaryDirectory() as tmpdir:
            root = Path(tmpdir)
            robustness = self.make_robustness_fixture(root)
            bad_profile = robustness / "scenario_profile_catalog" / "stress_runtime_profile.json"
            payload = json.loads(bad_profile.read_text(encoding="utf-8"))
            payload["runtime_inputs"] = ["raw_right_m", "truth_forward_m"]
            write_json(bad_profile, payload)

            with self.assertRaises(ValueError):
                module.export_selector_package(
                    robustness_dir=robustness,
                    output_dir=root / "selector_package",
                    baseline_scenario="nominal",
                )

    def test_exports_frontier_and_operator_launch_checks_when_requested(self) -> None:
        module = load_module(SCRIPT_PATH, "export_robustness_scenario_catalog_selector_bundle_test")
        with tempfile.TemporaryDirectory() as tmpdir:
            root = Path(tmpdir)
            robustness = self.make_robustness_fixture(root)
            output_dir = root / "selector_package"
            readme = root / "README.md"
            readme.write_text(
                "PX4/RTK/NED truth, validation error, target-window label, replay metric\n"
                "synthetic mode에서도 camera optical frame forward=north right=east down=down 가정하면 안 된다\n"
                "stereo-center RDF(right,down,forward)\n"
                "FRD+range(forward,right,down,range)\n",
                encoding="utf-8",
            )

            summary = module.export_selector_package(
                robustness_dir=robustness,
                output_dir=output_dir,
                baseline_scenario="nominal",
                with_deployment_frontier=True,
                with_operator_launch_checks=True,
                with_readiness_check=True,
                readme=readme,
            )
            frontier_json = output_dir / "runtime_deployment_frontier" / "runtime_deployment_frontier.json"
            launch_json = output_dir / "operator_view_launch_checks" / "operator_view_launch_checks.json"
            readiness_json = output_dir / "non_alias_selector_deployment_readiness.json"

            self.assertEqual(summary["status"], "PASS")
            self.assertEqual(summary["deployment_frontier_status"], "PASS")
            self.assertEqual(summary["operator_launch_status"], "PASS")
            self.assertEqual(summary["readiness_status"], "PASS")
            self.assertEqual(summary["operator_launch_check_count"], summary["operator_launch_pass_count"])
            self.assertGreater(summary["operator_launch_check_count"], 0)
            self.assertEqual(summary["readiness_pass_count"], summary["readiness_requirement_count"])
            self.assertFalse(summary["runtime_truth_used"])
            self.assertTrue(frontier_json.is_file())
            self.assertTrue(launch_json.is_file())
            self.assertTrue(readiness_json.is_file())


if __name__ == "__main__":
    unittest.main()
