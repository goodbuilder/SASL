from __future__ import annotations

import importlib.util
import json
import sys
import tempfile
import unittest
from pathlib import Path


ROOT_DIR = Path(__file__).resolve().parents[1]
SCRIPT_PATH = ROOT_DIR / "scripts" / "check_runtime_guard_deployment_readiness.py"
SPEC = importlib.util.spec_from_file_location("check_runtime_guard_deployment_readiness_under_test", SCRIPT_PATH)
module = importlib.util.module_from_spec(SPEC)
assert SPEC is not None and SPEC.loader is not None
sys.modules[SPEC.name] = module
SPEC.loader.exec_module(module)


def write_json(path: Path, payload: object) -> None:
    path.write_text(json.dumps(payload, ensure_ascii=False), encoding="utf-8")


class CheckRuntimeGuardDeploymentReadinessTests(unittest.TestCase):
    def test_readiness_passes_when_all_evidence_passes(self) -> None:
        with tempfile.TemporaryDirectory() as tmpdir:
            root = Path(tmpdir)
            known = root / "known.json"
            loso = root / "loso.json"
            batch = root / "batch.json"
            operator = root / "operator.json"
            write_json(
                known,
                {
                    "mode": "runtime_measurement_guard_audit",
                    "audit_count": 2,
                    "pass_count": 2,
                    "pass_all": True,
                    "validation_only_pass_all": True,
                    "target_max_abs_error_max_m": 3.0,
                    "runtime_truth_used": False,
                    "axis_contract": module.RDF_AXIS_CONTRACT,
                    "public_topic_contract": module.PUBLIC_TOPIC_CONTRACT,
                    "selection_input_fields": "raw_right_m,raw_down_m,raw_forward_m,measured_disparity_px",
                },
            )
            write_json(
                loso,
                {
                    "mode": "runtime_measurement_guard_policy_audit",
                    "recommended_selection_policy": "nearest_centroid",
                    "recommended_audit_count": 2,
                    "recommended_pass_count": 2,
                    "recommended_target_max_abs_error_max_m": 5.0,
                    "axis_contract": module.RDF_AXIS_CONTRACT,
                    "public_topic_contract": module.PUBLIC_TOPIC_CONTRACT,
                },
            )
            write_json(
                batch,
                {
                    "mode": "runtime_guard_operating_mode_batch_smoke",
                    "status": "PASS",
                    "run_count": 2,
                    "pass_count": 2,
                    "runtime_truth_used": False,
                    "axis_contract": module.RDF_AXIS_CONTRACT,
                    "public_topic_contract": module.PUBLIC_TOPIC_CONTRACT,
                },
            )
            write_json(
                operator,
                {
                    "mode": "runtime_guard_batch_operator_view_readiness",
                    "status": "PASS",
                    "run_count": 2,
                    "pass_count": 2,
                    "runtime_truth_used": False,
                    "axis_contract": module.RDF_AXIS_CONTRACT,
                    "public_topic_contract": module.PUBLIC_TOPIC_CONTRACT,
                },
            )

            result = module.check_readiness(
                known_guard_summary_json=known,
                loso_policy_summary_json=loso,
                batch_smoke_summary_json=batch,
                operator_readiness_summary_json=operator,
            )

            self.assertEqual(result["status"], "PASS")
            self.assertEqual(
                result["recommendation"],
                "runtime_guard_candidate_ready_for_real_calibration_csv_replay",
            )
            self.assertFalse(result["runtime_truth_used"])

    def test_readiness_fails_when_operator_count_mismatches_batch(self) -> None:
        with tempfile.TemporaryDirectory() as tmpdir:
            root = Path(tmpdir)
            known = root / "known.json"
            loso = root / "loso.json"
            batch = root / "batch.json"
            operator = root / "operator.json"
            write_json(
                known,
                {
                    "mode": "runtime_measurement_guard_audit",
                    "audit_count": 1,
                    "pass_count": 1,
                    "pass_all": True,
                    "validation_only_pass_all": True,
                    "target_max_abs_error_max_m": 3.0,
                    "runtime_truth_used": False,
                    "axis_contract": module.RDF_AXIS_CONTRACT,
                    "public_topic_contract": module.PUBLIC_TOPIC_CONTRACT,
                    "selection_input_fields": "raw_right_m",
                },
            )
            write_json(
                loso,
                {
                    "mode": "runtime_measurement_guard_policy_audit",
                    "recommended_audit_count": 1,
                    "recommended_pass_count": 1,
                    "recommended_target_max_abs_error_max_m": 3.0,
                    "axis_contract": module.RDF_AXIS_CONTRACT,
                    "public_topic_contract": module.PUBLIC_TOPIC_CONTRACT,
                },
            )
            write_json(
                batch,
                {
                    "mode": "runtime_guard_operating_mode_batch_smoke",
                    "status": "PASS",
                    "run_count": 2,
                    "pass_count": 2,
                    "runtime_truth_used": False,
                    "axis_contract": module.RDF_AXIS_CONTRACT,
                    "public_topic_contract": module.PUBLIC_TOPIC_CONTRACT,
                },
            )
            write_json(
                operator,
                {
                    "mode": "runtime_guard_batch_operator_view_readiness",
                    "status": "PASS",
                    "run_count": 1,
                    "pass_count": 1,
                    "runtime_truth_used": False,
                    "axis_contract": module.RDF_AXIS_CONTRACT,
                    "public_topic_contract": module.PUBLIC_TOPIC_CONTRACT,
                },
            )

            result = module.check_readiness(
                known_guard_summary_json=known,
                loso_policy_summary_json=loso,
                batch_smoke_summary_json=batch,
                operator_readiness_summary_json=operator,
            )

            self.assertEqual(result["status"], "FAIL")
            self.assertIn("operator_readiness:run_count_matches_batch", result["failures"])


if __name__ == "__main__":
    unittest.main()
