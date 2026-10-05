from __future__ import annotations

import importlib.util
import sys
import tempfile
import unittest
from pathlib import Path


ROOT_DIR = Path(__file__).resolve().parents[1]
SCRIPT_PATH = ROOT_DIR / "scripts" / "audit_operator_telemetry_residual_preflight_schema.py"


def load_module():
    spec = importlib.util.spec_from_file_location(
        "audit_operator_telemetry_residual_preflight_schema_under_test",
        SCRIPT_PATH,
    )
    if spec is None or spec.loader is None:
        raise RuntimeError(f"cannot load {SCRIPT_PATH}")
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


class AuditOperatorTelemetryResidualPreflightSchemaTests(unittest.TestCase):
    def test_passes_current_public_frd_schema(self) -> None:
        module = load_module()
        with tempfile.TemporaryDirectory() as tmpdir:
            telemetry = Path(tmpdir) / "telemetry.csv"
            telemetry.write_text(
                "\n".join(
                    [
                        ",".join(
                            [
                                "left_cx",
                                "left_cy",
                                "raw_disparity_px",
                                "raw_forward_m",
                                "raw_right_m",
                                "raw_down_m",
                                "corrected_forward_m",
                                "corrected_right_m",
                                "corrected_down_m",
                                "ground_truth_m",
                            ]
                        ),
                        "700,390,200,1000,7,4,999,6,3,1000",
                    ]
                )
                + "\n",
                encoding="utf-8",
            )

            payload = module.audit_schema(input_csv=telemetry)

        self.assertEqual(payload["status"], "PASS")
        self.assertEqual(payload["valid_row_count"], 1)
        self.assertFalse(payload["runtime_truth_used"])
        self.assertIn("public FRD", payload["coordinate_contract"])
        self.assertEqual(payload["dropped_truth_column_count"], 1)

    def test_fails_old_schema_before_selector_replay(self) -> None:
        module = load_module()
        with tempfile.TemporaryDirectory() as tmpdir:
            telemetry = Path(tmpdir) / "telemetry.csv"
            telemetry.write_text(
                "\n".join(
                    [
                        "left_cx,left_cy,raw_disparity_px,range_m,ground_truth_m",
                        "700,390,200,1000,1000",
                    ]
                )
                + "\n",
                encoding="utf-8",
            )

            payload = module.audit_schema(input_csv=telemetry)

        self.assertEqual(payload["status"], "FAIL")
        self.assertIn("missing_required_group:nominal_pose_forward_m", payload["failures"])
        self.assertIn("missing_required_group:nominal_pose_right_m", payload["failures"])
        self.assertIn("missing_required_group:nominal_pose_down_m", payload["failures"])
        self.assertIn("valid_row_count_below_min:0<1", payload["failures"])
        self.assertGreater(payload["row_failure_sample_count"], 0)

    def test_allow_missing_corrected_accepts_raw_observation_fallback(self) -> None:
        module = load_module()
        with tempfile.TemporaryDirectory() as tmpdir:
            telemetry = Path(tmpdir) / "telemetry.csv"
            telemetry.write_text(
                "\n".join(
                    [
                        "left_cx,left_cy,raw_disparity_px,raw_forward_m,raw_right_m,raw_down_m",
                        "700,390,200,1000,7,4",
                    ]
                )
                + "\n",
                encoding="utf-8",
            )

            payload = module.audit_schema(input_csv=telemetry, allow_missing_corrected=True)

        self.assertEqual(payload["status"], "PASS")
        self.assertEqual(payload["valid_row_count"], 1)
        self.assertNotIn("observation_forward_m", payload["required_groups"])


if __name__ == "__main__":
    unittest.main()
