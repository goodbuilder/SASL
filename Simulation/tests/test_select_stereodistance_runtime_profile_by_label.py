from __future__ import annotations

import importlib.util
import json
import sys
import tempfile
import unittest
from pathlib import Path


ROOT_DIR = Path(__file__).resolve().parents[1]
SCRIPT_PATH = ROOT_DIR / "scripts" / "select_stereodistance_runtime_profile_by_label.py"


def load_module():
    spec = importlib.util.spec_from_file_location(
        "select_stereodistance_runtime_profile_by_label_under_test",
        SCRIPT_PATH,
    )
    if spec is None or spec.loader is None:
        raise RuntimeError(f"cannot load {SCRIPT_PATH}")
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def write_json(path: Path, payload: object) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(payload, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")


class SelectStereoDistanceRuntimeProfileByLabelTests(unittest.TestCase):
    def write_profile(self, path: Path, profile_id: str, method: str) -> None:
        write_json(
            path,
            {
                "profile_id": profile_id,
                "method": method,
                "mode": "estimated_to_truth_sim3",
                "axis_contract": "internal_policy_vector=RDF(right,down,forward)",
                "runtime_inputs": ["raw_right_m", "raw_down_m", "raw_forward_m", "raw_disparity_px"],
                "forbidden_runtime_inputs": ["calibration_truth", "px4_rtk_truth"],
                "scale": 1.0,
                "rotation_matrix_rdf": [
                    [1.0, 0.0, 0.0],
                    [0.0, 1.0, 0.0],
                    [0.0, 0.0, 1.0],
                ],
                "translation_rdf_m": [0.0, 0.0, 0.0],
            },
        )

    def make_selector_fixture(self, root: Path) -> Path:
        current_package = root / "current_package"
        stress_package = root / "stress_package"
        current_profile = current_package / "active_runtime_profile.json"
        stress_profile = stress_package / "active_runtime_profile.json"
        self.write_profile(current_profile, "current", "sim3_binned")
        self.write_profile(stress_profile, "stress", "target_weighted_huber")
        evidence = root / "evidence"
        (evidence / "current_readiness.txt").parent.mkdir(parents=True, exist_ok=True)
        (evidence / "current_readiness.txt").write_text("status: PASS\n", encoding="utf-8")
        (evidence / "stress_readiness.txt").write_text("status: PASS\n", encoding="utf-8")
        selector = root / "selector.json"
        write_json(
            selector,
            {
                "mode": "stereodistance_runtime_profile_label_selector",
                "baseline_selector": "current_active",
                "label_profile_map": {
                    "nominal": "current_active",
                    "runtime_observation_stress": "stress_active",
                },
                "profile_paths": {
                    "current_active": "current_package/active_runtime_profile.json",
                    "stress_active": "stress_package/active_runtime_profile.json",
                },
                "package_paths": {
                    "current_active": "current_package",
                    "stress_active": "stress_package",
                },
                "validated_evidence": {
                    "current_readiness": "evidence/current_readiness.txt",
                    "stress_readiness": "evidence/stress_readiness.txt",
                },
                "selection_input_fields": ["deployment_label", "scenario_label", "scenario_name"],
                "forbidden_runtime_fields": ["truth_right_m", "px4_rtk_truth", "validation_range_rmse_corrected_m"],
                "axis_contract": "stereo_center_opencv_optical:RDF(right,down,forward)",
                "public_topic_contract": "stereodistance_position_topics:FRD_range(forward,right,down,range)",
                "selection_contract": (
                    "Runtime profile selection may use explicit operator/deployment scenario labels only. "
                    "It must not inspect PX4/RTK truth."
                ),
            },
        )
        return selector

    def test_selects_stress_profile_from_deployment_label(self) -> None:
        module = load_module()
        with tempfile.TemporaryDirectory() as tmpdir:
            selector = self.make_selector_fixture(Path(tmpdir))

            result = module.select_runtime_profile_by_label(
                selector_json=selector,
                deployment_label="runtime_observation_stress",
            )

        self.assertEqual(result["status"], "PASS")
        self.assertEqual(result["selected_label"], "stress_active")
        self.assertEqual(result["profile_id"], "stress")
        self.assertTrue(result["label_selection_applied"])
        self.assertFalse(result["runtime_truth_used"])
        self.assertIn("RDF(right,down,forward)", result["coordinate_contract"])

    def test_unknown_label_falls_back_to_current_active(self) -> None:
        module = load_module()
        with tempfile.TemporaryDirectory() as tmpdir:
            selector = self.make_selector_fixture(Path(tmpdir))

            result = module.select_runtime_profile_by_label(
                selector_json=selector,
                deployment_label="unlabeled",
            )

        self.assertEqual(result["status"], "PASS")
        self.assertEqual(result["selected_label"], "current_active")
        self.assertEqual(result["profile_id"], "current")
        self.assertFalse(result["label_selection_applied"])

    def test_rejects_selector_that_allows_truth_selection_inputs(self) -> None:
        module = load_module()
        with tempfile.TemporaryDirectory() as tmpdir:
            root = Path(tmpdir)
            selector = self.make_selector_fixture(root)
            payload = json.loads(selector.read_text(encoding="utf-8"))
            payload["selection_input_fields"] = ["deployment_label", "truth_forward_m"]
            write_json(selector, payload)

            result = module.select_runtime_profile_by_label(
                selector_json=selector,
                deployment_label="runtime_observation_stress",
            )

        self.assertEqual(result["status"], "FAIL")
        self.assertIn("selector:selection_inputs_truth", result["failures"])

    def test_run_writes_json_and_text(self) -> None:
        module = load_module()
        with tempfile.TemporaryDirectory() as tmpdir:
            root = Path(tmpdir)
            selector = self.make_selector_fixture(root)
            output_json = root / "selection.json"
            output_txt = root / "selection.txt"

            result = module.run(
                [
                    "--selector-json",
                    str(selector),
                    "--deployment-label",
                    "nominal",
                    "--output-json",
                    str(output_json),
                    "--output-txt",
                    str(output_txt),
                ]
            )
            output_json_exists = output_json.is_file()
            text = output_txt.read_text(encoding="utf-8")

        self.assertEqual(result["status"], "PASS")
        self.assertTrue(output_json_exists)
        self.assertIn("selected_label: current_active", text)


if __name__ == "__main__":
    unittest.main()
