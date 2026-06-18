from __future__ import annotations

import csv
import importlib.util
import json
import sys
import tempfile
import unittest
from pathlib import Path


ROOT_DIR = Path(__file__).resolve().parents[1]
SCRIPT_PATH = ROOT_DIR / "scripts" / "export_stereodistance_runtime_deployment_frontier.py"


def load_module():
    spec = importlib.util.spec_from_file_location("export_stereodistance_runtime_deployment_frontier_under_test", SCRIPT_PATH)
    if spec is None or spec.loader is None:
        raise RuntimeError(f"cannot load {SCRIPT_PATH}")
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


class ExportStereoDistanceRuntimeDeploymentFrontierTests(unittest.TestCase):
    def make_profile(self, package_dir: Path, method: str) -> Path:
        package_dir.mkdir(parents=True)
        profile = package_dir / "active_runtime_profile.json"
        profile.write_text(
            json.dumps(
                {
                    "profile_id": method,
                    "method": method,
                    "mode": "estimated_to_truth_sim3",
                    "axis_contract": "stereo_center_opencv_optical:RDF(right,down,forward)",
                    "runtime_inputs": ["raw_right_m", "raw_down_m", "raw_forward_m", "raw_disparity_px"],
                }
            )
            + "\n",
            encoding="utf-8",
        )
        (package_dir.parent / "deployment_readiness.json").write_text(
            '{"status":"PASS","failure_count":0,"runtime_truth_used":false}\n',
            encoding="utf-8",
        )
        return profile

    def test_exports_frontier_profiles_and_label_routes(self) -> None:
        module = load_module()
        with tempfile.TemporaryDirectory() as tmpdir:
            root = Path(tmpdir)
            current_profile = self.make_profile(root / "current_package", "current_method")
            stress_profile = self.make_profile(root / "stress_package", "stress_method")
            evidence = root / "evidence.txt"
            evidence.write_text("PASS\n", encoding="utf-8")
            selector_json = root / "runtime_profile_label_selector.json"
            selector_json.write_text(
                json.dumps(
                    {
                        "mode": "stereodistance_runtime_profile_label_selector",
                        "baseline_selector": "current_active",
                        "label_profile_map": {
                            "nominal": "current_active",
                            "runtime_observation_stress": "stress_active",
                        },
                        "profile_paths": {
                            "current_active": str(current_profile),
                            "stress_active": str(stress_profile),
                        },
                        "package_paths": {
                            "current_active": str(current_profile.parent),
                            "stress_active": str(stress_profile.parent),
                        },
                        "validated_evidence": {"unit": str(evidence)},
                        "selection_input_fields": ["deployment_label", "scenario_label", "scenario_name"],
                        "forbidden_runtime_fields": ["truth", "px4", "rtk", "ned"],
                        "axis_contract": "stereo_center_opencv_optical:RDF(right,down,forward)",
                        "public_topic_contract": "stereodistance_position_topics:FRD_range(forward,right,down,range)",
                    }
                )
                + "\n",
                encoding="utf-8",
            )
            output_dir = root / "frontier"

            summary = module.export_frontier(selector_json=selector_json, output_dir=output_dir)
            with (output_dir / "runtime_deployment_frontier_profiles.csv").open(encoding="utf-8") as handle:
                profile_rows = list(csv.DictReader(handle))
            with (output_dir / "runtime_deployment_frontier_labels.csv").open(encoding="utf-8") as handle:
                label_rows = list(csv.DictReader(handle))
            frontier = json.loads((output_dir / "runtime_deployment_frontier.json").read_text(encoding="utf-8"))
            text = (output_dir / "runtime_deployment_frontier.txt").read_text(encoding="utf-8")

        self.assertEqual(summary["status"], "PASS")
        self.assertEqual(summary["profile_count"], 2)
        self.assertEqual(summary["deployment_label_count"], 3)
        self.assertEqual({row["profile_label"] for row in profile_rows}, {"current_active", "stress_active"})
        by_label = {row["deployment_label"]: row for row in label_rows}
        self.assertEqual(by_label["__default__"]["selected_profile_label"], "current_active")
        self.assertEqual(by_label["runtime_observation_stress"]["selected_profile_label"], "stress_active")
        self.assertEqual(frontier["summary"]["status"], "PASS")
        self.assertIn("runtime_observation_stress -> stress_active", text)

    def test_truth_runtime_input_fails_frontier(self) -> None:
        module = load_module()
        with tempfile.TemporaryDirectory() as tmpdir:
            root = Path(tmpdir)
            package = root / "package"
            profile = self.make_profile(package, "bad_method")
            payload = json.loads(profile.read_text(encoding="utf-8"))
            payload["runtime_inputs"] = ["raw_right_m", "truth_right_m"]
            profile.write_text(json.dumps(payload) + "\n", encoding="utf-8")
            evidence = root / "evidence.txt"
            evidence.write_text("PASS\n", encoding="utf-8")
            selector_json = root / "runtime_profile_label_selector.json"
            selector_json.write_text(
                json.dumps(
                    {
                        "mode": "stereodistance_runtime_profile_label_selector",
                        "baseline_selector": "current_active",
                        "label_profile_map": {"nominal": "current_active"},
                        "profile_paths": {"current_active": str(profile)},
                        "package_paths": {"current_active": str(package)},
                        "validated_evidence": {"unit": str(evidence)},
                        "selection_input_fields": ["deployment_label"],
                        "axis_contract": "stereo_center_opencv_optical:RDF(right,down,forward)",
                        "public_topic_contract": "stereodistance_position_topics:FRD_range(forward,right,down,range)",
                    }
                )
                + "\n",
                encoding="utf-8",
            )

            summary = module.export_frontier(selector_json=selector_json, output_dir=root / "frontier")

        self.assertEqual(summary["status"], "FAIL")
        self.assertIn("profile:current_active:runtime_inputs_truth", summary["failures"])


if __name__ == "__main__":
    unittest.main()
