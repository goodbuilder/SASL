from __future__ import annotations

import importlib.util
import json
import sys
import tempfile
import unittest
from pathlib import Path


ROOT_DIR = Path(__file__).resolve().parents[1]
SCRIPT_PATH = ROOT_DIR / "scripts" / "export_deployable_profile_candidate_from_policy_selection.py"


def load_module():
    spec = importlib.util.spec_from_file_location(
        "export_deployable_profile_candidate_from_policy_selection_under_test",
        SCRIPT_PATH,
    )
    if spec is None or spec.loader is None:
        raise RuntimeError(f"cannot load {SCRIPT_PATH}")
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def write_profile(path: Path) -> None:
    path.write_text(
        json.dumps(
            {
                "profile_id": "candidate",
                "method": "range_best",
                "mode": "estimated_to_truth_sim3",
                "axis_contract": "stereo_center_opencv_optical:RDF(right,down,forward)",
                "runtime_inputs": ["raw_right_m", "raw_down_m", "raw_forward_m", "raw_disparity_px"],
                "forbidden_runtime_inputs": [
                    "calibration_truth",
                    "px4_rtk_truth",
                    "true_disparity",
                    "true_image_feature_location",
                ],
                "scale": 1.0,
                "rotation_matrix_rdf": [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]],
                "translation_rdf_m": [1.0, 2.0, 3.0],
            },
            ensure_ascii=False,
        ),
        encoding="utf-8",
    )


def write_samples(path: Path) -> None:
    path.write_text(
        "\n".join(
            [
                "split,raw_right_m,raw_down_m,raw_forward_m,measured_disparity_px,left_u_px,left_v_px",
                "validation,10.0,20.0,30.0,5.0,640.0,360.0",
                "validation,11.0,21.0,31.0,5.1,641.0,361.0",
            ]
        )
        + "\n",
        encoding="utf-8",
    )


class ExportDeployableProfileCandidateFromPolicySelectionTests(unittest.TestCase):
    def test_exports_active_candidate_with_promotion_and_public_payload(self) -> None:
        module = load_module()
        with tempfile.TemporaryDirectory() as tmpdir:
            root = Path(tmpdir)
            source = root / "profiles" / "range_best.json"
            source.parent.mkdir()
            write_profile(source)
            samples = root / "samples.csv"
            write_samples(samples)
            source_sha = module.sha256_file(source)
            policy = root / "policy.json"
            policy.write_text(
                json.dumps(
                    {
                        "status": "PASS",
                        "recommended_policy": "range_priority",
                        "recommended_method": "range_best",
                        "recommended_profile_json": str(source),
                        "recommended_profile_sha256": source_sha,
                        "matrix_summary_json": "matrix.json",
                        "matrix_selected_method": "score_best",
                        "hard_gate_pass_count": 2,
                        "policies": [
                            {
                                "policy": "range_priority",
                                "selected_method": "range_best",
                                "validation_range_rmse_corrected_mean_m": 1.0,
                                "validation_range_rmse_corrected_max_m": 1.5,
                                "target_max_abs_error_corrected_max_m": 2.0,
                            }
                        ],
                        "candidates": [
                            {
                                "method": "range_best",
                                "pass_count": 2,
                                "expected_run_count": 2,
                                "axis_pass_count": 2,
                                "target_pass_count": 2,
                            }
                        ],
                    },
                    ensure_ascii=False,
                ),
                encoding="utf-8",
            )

            summary = module.export_candidate(
                policy_selection_json=policy,
                output_active_profile=root / "active_runtime_profile.json",
                output_dir=root / "out",
                sample_csv=samples,
                split="validation",
                max_samples=2,
            )

            export_json_exists = (root / "out" / "deployable_profile_candidate_export.json").exists()
            public_csv_exists = (root / "out" / "public_topic_payload" / "public_corrected_position_payload.csv").exists()

        self.assertEqual(summary["status"], "PASS")
        self.assertEqual(summary["method"], "range_best")
        self.assertTrue(summary["profile_sha256_matches_policy"])
        self.assertTrue(summary["promotion_check_pass"])
        self.assertEqual(summary["promotion_sample_apply_count"], 2)
        self.assertEqual(summary["promotion_sample_invalid_count"], 0)
        self.assertEqual(summary["public_topic_payload_status"], "PASS")
        self.assertEqual(summary["public_topic_payload_sample_count"], 2)
        self.assertFalse(summary["runtime_truth_used"])
        self.assertTrue(export_json_exists)
        self.assertTrue(public_csv_exists)


if __name__ == "__main__":
    unittest.main()
