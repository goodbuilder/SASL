from __future__ import annotations

import importlib.util
import sys
import tempfile
import unittest
from pathlib import Path


ROOT_DIR = Path(__file__).resolve().parents[1]
SCRIPT_PATH = ROOT_DIR / "scripts" / "check_stereodistance_launch_profile_contract.py"


def load_module():
    spec = importlib.util.spec_from_file_location(
        "check_stereodistance_launch_profile_contract_under_test",
        SCRIPT_PATH,
    )
    if spec is None or spec.loader is None:
        raise RuntimeError(f"cannot load {SCRIPT_PATH}")
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


class CheckStereoDistanceLaunchProfileContractTests(unittest.TestCase):
    def test_current_launch_file_passes_profile_contract(self) -> None:
        module = load_module()

        result = module.check_launch_contract()

        self.assertEqual(result["status"], "PASS")
        self.assertIn("enable_bbox_center_adjustment", result["declared_profile_arguments"])
        self.assertIn("bbox_center_adjustment_path", result["distance_node_profile_parameters"])
        self.assertIn("enable_observation_fusion", result["declared_profile_arguments"])
        self.assertIn("observation_fusion_manifest_path", result["distance_node_profile_parameters"])
        self.assertIn("enable_disparity_residual_feature_selector", result["declared_profile_arguments"])
        self.assertTrue(result["enable_bbox_center_adjustment_is_bool"])
        self.assertTrue(result["enable_observation_fusion_is_bool"])
        self.assertTrue(result["enable_disparity_residual_feature_selector_is_bool"])
        self.assertFalse(result["runtime_truth_used"])

    def test_missing_profile_param_fails(self) -> None:
        module = load_module()
        with tempfile.TemporaryDirectory() as tmpdir:
            launch_file = Path(tmpdir) / "bad.launch.py"
            launch_file.write_text(
                """
from launch.actions import DeclareLaunchArgument

def generate_launch_description():
    return [DeclareLaunchArgument("enable_bbox_center_adjustment", default_value="false")]

def _write_distance_node_params(context):
    bool_params = set()
    names = ["enable_bbox_center_adjustment"]
    handle.write("/stereo_distance_node:\\n")
    args = ["--params-file", "stereo_ground_sim.stereo_distance_node"]
""",
                encoding="utf-8",
            )

            result = module.check_launch_contract(launch_file)

        self.assertEqual(result["status"], "FAIL")
        self.assertIn("launch_argument:missing:bbox_center_adjustment_path", result["failures"])
        self.assertIn("launch_argument:missing:enable_observation_fusion", result["failures"])
        self.assertIn("bool_param:missing:enable_bbox_center_adjustment", result["failures"])
        self.assertIn("bool_param:missing:enable_observation_fusion", result["failures"])

    def test_run_writes_json_and_text(self) -> None:
        module = load_module()
        with tempfile.TemporaryDirectory() as tmpdir:
            root = Path(tmpdir)
            output_json = root / "launch_contract.json"
            output_txt = root / "launch_contract.txt"

            result = module.run(["--output-json", str(output_json), "--output-txt", str(output_txt)])
            output_json_exists = output_json.is_file()
            txt = output_txt.read_text(encoding="utf-8")

        self.assertEqual(result["status"], "PASS")
        self.assertTrue(output_json_exists)
        self.assertIn("status: PASS", txt)


if __name__ == "__main__":
    unittest.main()
