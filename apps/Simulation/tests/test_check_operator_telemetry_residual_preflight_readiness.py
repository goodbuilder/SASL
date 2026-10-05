from __future__ import annotations

import importlib.util
import json
import sys
import tempfile
import unittest
from pathlib import Path


ROOT_DIR = Path(__file__).resolve().parents[1]
SCRIPT_PATH = ROOT_DIR / "scripts" / "check_operator_telemetry_residual_preflight_readiness.py"


def load_module():
    spec = importlib.util.spec_from_file_location(
        "check_operator_telemetry_residual_preflight_readiness_under_test",
        SCRIPT_PATH,
    )
    if spec is None or spec.loader is None:
        raise RuntimeError(f"cannot load {SCRIPT_PATH}")
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


class CheckOperatorTelemetryResidualPreflightReadinessTests(unittest.TestCase):
    def write_artifacts(self, root: Path, *, launch_status: str = "PASS") -> Path:
        residual = root / "profiles" / "unit.json"
        residual.parent.mkdir(parents=True, exist_ok=True)
        residual.write_text("{}\n", encoding="utf-8")
        decision = root / "telemetry_selector_replay" / "operator_telemetry_selector_decision.json"
        decision.parent.mkdir(parents=True, exist_ok=True)
        decision.write_text(
            json.dumps(
                {
                    "status": "PASS",
                    "apply_residual_allowed": True,
                    "recommended_residual_label": "unit",
                    "recommended_residual_profile": str(residual),
                    "runtime_truth_used": False,
                }
            )
            + "\n",
            encoding="utf-8",
        )
        launch = root / "operator_launch_check_from_decision.json"
        launch.write_text(
            json.dumps(
                {
                    "status": launch_status,
                    "resolved_disparity_residual_profile": str(residual),
                    "disparity_residual_arg_present": True,
                    "runtime_truth_used": False,
                }
            )
            + "\n",
            encoding="utf-8",
        )
        summary = root / "operator_telemetry_residual_preflight_summary.json"
        summary.write_text(
            json.dumps(
                {
                    "status": "PASS",
                    "selector_decision_json": str(decision),
                    "operator_launch_check_json": str(launch),
                    "apply_residual_allowed": True,
                    "recommended_residual_label": "unit",
                    "recommended_residual_profile": str(residual),
                    "operator_launch_status": launch_status,
                    "resolved_disparity_residual_profile": str(residual),
                    "runtime_truth_used": False,
                    "coordinate_contract": (
                        "operator telemetry source is public FRD; selector replay emits stereo-center RDF "
                        "observation features; public output remains FRD+range"
                    ),
                    "failure_count": 0,
                    "failures": [],
                }
            )
            + "\n",
            encoding="utf-8",
        )
        return summary

    def test_readiness_passes_complete_preflight_artifacts(self) -> None:
        module = load_module()
        with tempfile.TemporaryDirectory() as tmpdir:
            root = Path(tmpdir)
            summary = self.write_artifacts(root)

            result = module.check_readiness(summary)

        self.assertEqual(result["status"], "PASS")
        self.assertTrue(result["apply_residual_allowed"])
        self.assertTrue(result["operator_launch_residual_arg_present"])
        self.assertFalse(result["runtime_truth_used"])
        self.assertIn("FRD+range", result["coordinate_contract"])

    def test_readiness_fails_when_launch_artifact_failed(self) -> None:
        module = load_module()
        with tempfile.TemporaryDirectory() as tmpdir:
            root = Path(tmpdir)
            summary = self.write_artifacts(root, launch_status="FAIL")

            result = module.check_readiness(summary)

        self.assertEqual(result["status"], "FAIL")
        self.assertIn("preflight:operator_launch_status", result["failures"])
        self.assertIn("operator_launch:status", result["failures"])

    def test_run_writes_outputs(self) -> None:
        module = load_module()
        with tempfile.TemporaryDirectory() as tmpdir:
            root = Path(tmpdir)
            summary = self.write_artifacts(root)
            output_json = root / "readiness.json"
            output_txt = root / "readiness.txt"

            result = module.run(
                [
                    "--preflight-summary-json",
                    str(summary),
                    "--output-json",
                    str(output_json),
                    "--output-txt",
                    str(output_txt),
                ]
            )
            output_json_exists = output_json.is_file()
            output_txt_text = output_txt.read_text(encoding="utf-8")

        self.assertEqual(result["status"], "PASS")
        self.assertTrue(output_json_exists)
        self.assertIn("status: PASS", output_txt_text)


if __name__ == "__main__":
    unittest.main()
