from __future__ import annotations

import importlib.util
import json
import sys
import tempfile
import unittest
from pathlib import Path


ROOT_DIR = Path(__file__).resolve().parents[1]
SCRIPT_PATH = ROOT_DIR / "scripts" / "audit_rtk_stereo_method_search_catalog.py"


def load_module():
    spec = importlib.util.spec_from_file_location("audit_rtk_stereo_method_search_catalog_under_test", SCRIPT_PATH)
    if spec is None or spec.loader is None:
        raise RuntimeError(f"cannot load {SCRIPT_PATH}")
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def write_json(path: Path, payload: object) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(payload, ensure_ascii=False), encoding="utf-8")


def make_fixture(root: Path) -> tuple[Path, Path, Path]:
    methods = [
        "global_rotation",
        "global_scale_rotation",
        "sim3",
        "linear_disparity",
        "inverse_depth_linear",
        "quadratic_disparity",
        "poly_residual",
        "binned_residual",
        "sim3_residual",
        "sim3_disparity",
        "focal_scale_profile",
        "baseline_scale_profile",
    ]
    matrix = root / "frontier_profile_matrix_summary.json"
    write_json(
        matrix,
        {
            "status": "PASS",
            "axis_contract": "stereo_center_opencv_optical:RDF(right,down,forward)",
            "public_topic_contract": "stereodistance_position_topics:FRD_range(forward,right,down,range)",
            "runtime_truth_used": False,
            "methods": [{"method": method} for method in methods],
        },
    )
    selection = root / "deployable_profile_policy_selection.json"
    write_json(
        selection,
        {
            "status": "PASS",
            "recommended_policy": "range_priority",
            "recommended_method": "sim3_residual",
            "axis_contract": "stereo_center_opencv_optical:RDF(right,down,forward)",
            "public_topic_contract": "stereodistance_position_topics:FRD_range(forward,right,down,range)",
            "runtime_truth_used": False,
            "candidates": [
                {
                    "method": method,
                    "profile_exists": True,
                    "runtime_truth_free": True,
                    "hard_gate_pass": method == "sim3_residual",
                    "pass_count": 3 if method == "sim3_residual" else 2,
                    "expected_run_count": 3,
                    "axis_pass_count": 3,
                    "target_max_abs_error_corrected_max_m": 4.0 if method == "sim3_residual" else 12.0,
                    "validation_range_rmse_corrected_mean_m": 1.8 if method == "sim3_residual" else 3.0,
                    "validation_range_rmse_corrected_max_m": 2.8 if method == "sim3_residual" else 5.0,
                }
                for method in methods
            ],
        },
    )
    bundle = root / "rtk_stereo_goal_acceptance_bundle_manifest.json"
    write_json(
        bundle,
        {
            "status": "PASS",
            "selected_policy": "range_priority",
            "selected_method": "sim3_residual",
            "axis_contract": "stereo_center_opencv_optical:RDF(right,down,forward)",
            "public_topic_contract": "stereodistance_position_topics:FRD_range(forward,right,down,range)",
            "runtime_truth_used": False,
        },
    )
    return matrix, selection, bundle


class AuditRtkStereoMethodSearchCatalogTests(unittest.TestCase):
    def test_catalog_audit_passes_broad_truth_free_catalog(self) -> None:
        module = load_module()
        with tempfile.TemporaryDirectory() as tmpdir:
            matrix, selection, bundle = make_fixture(Path(tmpdir))
            result = module.audit_catalog(
                matrix_summary_json=matrix,
                policy_selection_json=selection,
                bundle_manifest_json=bundle,
                min_method_count=12,
                target_error_limit_m=10.0,
            )

        self.assertEqual(result["status"], "PASS")
        self.assertTrue(result["required_family_pass"])
        self.assertEqual(result["hard_gate_pass_count"], 1)
        self.assertFalse(result["runtime_truth_used"])

    def test_catalog_audit_fails_when_runtime_truth_is_used(self) -> None:
        module = load_module()
        with tempfile.TemporaryDirectory() as tmpdir:
            root = Path(tmpdir)
            matrix, selection, bundle = make_fixture(root)
            payload = json.loads(selection.read_text(encoding="utf-8"))
            payload["runtime_truth_used"] = True
            write_json(selection, payload)
            result = module.audit_catalog(
                matrix_summary_json=matrix,
                policy_selection_json=selection,
                bundle_manifest_json=bundle,
                min_method_count=12,
                target_error_limit_m=10.0,
            )

        self.assertEqual(result["status"], "FAIL")
        self.assertTrue(result["runtime_truth_used"])


if __name__ == "__main__":
    unittest.main()
