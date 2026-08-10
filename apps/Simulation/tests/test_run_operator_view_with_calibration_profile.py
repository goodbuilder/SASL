from __future__ import annotations

import subprocess
import tempfile
import unittest
import json
import os
from pathlib import Path


ROOT_DIR = Path(__file__).resolve().parents[1]
SCRIPT = ROOT_DIR / "run_operator_view_with_calibration_profile.sh"


class RunOperatorViewWithCalibrationProfileTests(unittest.TestCase):
    def run_script(self, *args: str) -> subprocess.CompletedProcess[str]:
        return subprocess.run(
            [str(SCRIPT), *args],
            cwd=ROOT_DIR,
            check=False,
            text=True,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
        )

    def test_print_only_uses_newest_active_runtime_profile(self) -> None:
        with tempfile.TemporaryDirectory() as tmpdir:
            root = Path(tmpdir)
            older = root / "legacy" / "sim3_runtime_profile.json"
            newer = root / "deployment" / "active_runtime_profile.json"
            older.parent.mkdir(parents=True)
            newer.parent.mkdir(parents=True)
            older.write_text("{}\n", encoding="utf-8")
            newer.write_text("{}\n", encoding="utf-8")
            os.utime(older, (1000, 1000))
            os.utime(newer, (2000, 2000))

            result = self.run_script("--results-root", str(root), "--print-only")

        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertIn(f"Runtime profile: {newer}", result.stdout)
        self.assertIn("--bbox-adjustment", result.stdout)
        self.assertIn(str(newer), result.stdout)
        self.assertNotIn("--no-px4", result.stdout)

    def test_print_only_accepts_deployment_package_dir(self) -> None:
        with tempfile.TemporaryDirectory() as tmpdir:
            package_dir = Path(tmpdir) / "deployment_package"
            profile = package_dir / "active_runtime_profile.json"
            package_dir.mkdir()
            profile.write_text("{}\n", encoding="utf-8")

            result = self.run_script("--package-dir", str(package_dir), "--print-only")

        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertIn(f"Runtime profile: {profile}", result.stdout)
        self.assertIn(str(profile), result.stdout)

    def test_print_only_passes_headless_operator_arg(self) -> None:
        with tempfile.TemporaryDirectory() as tmpdir:
            profile = Path(tmpdir) / "active_runtime_profile.json"
            profile.write_text("{}\n", encoding="utf-8")

            result = self.run_script(
                "--profile-json",
                str(profile),
                "--print-only",
                "--",
                "--headless",
            )

        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertIn("--headless", result.stdout)
        self.assertNotIn("--no-px4", result.stdout)

    def test_print_only_accepts_observation_fusion_manifest(self) -> None:
        with tempfile.TemporaryDirectory() as tmpdir:
            root = Path(tmpdir)
            profile = root / "active_runtime_profile.json"
            manifest = root / "observation_fusion_runtime_manifest.json"
            profile.write_text("{}\n", encoding="utf-8")
            manifest.write_text(
                json.dumps(
                    {
                        "default_recommendation": "unit",
                        "tiers": [
                            {
                                "tier": "unit",
                                "adapter": {
                                    "class": "RepeatedObservationFusion",
                                    "min_observations": 3,
                                    "mode": "median",
                                    "trim_fraction": 0.0,
                                },
                            }
                        ],
                    }
                )
                + "\n",
                encoding="utf-8",
            )

            result = self.run_script(
                "--profile-json",
                str(profile),
                "--observation-fusion-manifest",
                str(manifest),
                "--observation-fusion-tier",
                "unit",
                "--print-only",
            )

        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertIn(f"Runtime profile: {profile}", result.stdout)
        self.assertIn(f"Observation fusion manifest: {manifest}", result.stdout)
        self.assertIn("--observation-fusion-manifest", result.stdout)
        self.assertIn("--observation-fusion-tier", result.stdout)

    def test_profile_json_and_package_dir_are_mutually_exclusive(self) -> None:
        with tempfile.TemporaryDirectory() as tmpdir:
            root = Path(tmpdir)
            profile = root / "active_runtime_profile.json"
            package_dir = root / "deployment_package"
            profile.write_text("{}\n", encoding="utf-8")
            package_dir.mkdir()

            result = self.run_script(
                "--profile-json",
                str(profile),
                "--package-dir",
                str(package_dir),
                "--print-only",
            )

        self.assertEqual(result.returncode, 2)
        self.assertIn("only one of --profile-json, --package-dir, or --selector-json", result.stderr)

    def test_print_only_accepts_runtime_profile_label_selector(self) -> None:
        with tempfile.TemporaryDirectory() as tmpdir:
            root = Path(tmpdir)
            package_dir = root / "stress_package"
            profile = package_dir / "active_runtime_profile.json"
            evidence = root / "evidence.txt"
            selector_json = root / "runtime_profile_label_selector.json"
            package_dir.mkdir()
            profile.write_text(
                json.dumps(
                    {
                        "profile_id": "stress_profile",
                        "method": "unit_test_selector_profile",
                        "mode": "estimated_to_truth_sim3",
                        "axis_contract": "stereo_center_opencv_optical:RDF(right,down,forward)",
                        "runtime_inputs": [
                            "raw_right_m",
                            "raw_down_m",
                            "raw_forward_m",
                            "raw_disparity_px",
                        ],
                    }
                )
                + "\n",
                encoding="utf-8",
            )
            evidence.write_text("PASS\n", encoding="utf-8")
            selector_json.write_text(
                json.dumps(
                    {
                        "mode": "stereodistance_runtime_profile_label_selector",
                        "axis_contract": "stereo_center_opencv_optical:RDF(right,down,forward)",
                        "public_topic_contract": "stereodistance_position_topics:FRD_range(forward,right,down,range)",
                        "selection_input_fields": ["deployment_label", "scenario_label", "scenario_name"],
                        "forbidden_runtime_fields": ["truth", "px4", "rtk", "ned"],
                        "baseline_selector": "current_active",
                        "label_profile_map": {"runtime_observation_stress": "stress_active"},
                        "profile_paths": {"stress_active": str(profile), "current_active": str(profile)},
                        "package_paths": {"stress_active": str(package_dir), "current_active": str(package_dir)},
                        "validated_evidence": {"unit": str(evidence)},
                        "selection_contract": "truth-free deployment/scenario label selection only",
                    }
                )
                + "\n",
                encoding="utf-8",
            )

            result = self.run_script(
                "--selector-json",
                str(selector_json),
                "--deployment-label",
                "runtime_observation_stress",
                "--print-only",
            )

        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertIn(f"Selector: {selector_json}", result.stdout)
        self.assertIn("Selected label: stress_active", result.stdout)
        self.assertIn(f"Runtime profile: {profile}", result.stdout)
        self.assertIn(f"Deployment package: {package_dir}", result.stdout)
        self.assertIn("--bbox-adjustment", result.stdout)
        self.assertNotIn("--no-px4", result.stdout)


if __name__ == "__main__":
    unittest.main()
