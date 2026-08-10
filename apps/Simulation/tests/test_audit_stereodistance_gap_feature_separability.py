from __future__ import annotations

import csv
import importlib.util
import json
import sys
import tempfile
import unittest
from pathlib import Path


ROOT_DIR = Path(__file__).resolve().parents[1]


def load_separability_module():
    spec = importlib.util.spec_from_file_location(
        "audit_stereodistance_gap_feature_separability_under_test",
        ROOT_DIR / "scripts" / "audit_stereodistance_gap_feature_separability.py",
    )
    if spec is None or spec.loader is None:
        raise RuntimeError("cannot load audit_stereodistance_gap_feature_separability.py")
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


class AuditStereoDistanceGapFeatureSeparabilityTests(unittest.TestCase):
    def test_audit_finds_raw_feature_threshold_for_full_gap_family(self) -> None:
        module = load_separability_module()
        with tempfile.TemporaryDirectory() as tmpdir:
            root = Path(tmpdir)
            sweep = root / "sweep"
            gap_dir = sweep / "loso_coverage_gap_summary"
            runtime_dir = sweep / "runtime_measurement_guard_policy_audit_loso" / "runs" / "all_catalog_transfer_risk_fallback"
            gap_dir.mkdir(parents=True)
            runtime_dir.mkdir(parents=True)
            gap_dir.joinpath("loso_coverage_gap_summary.json").write_text(
                json.dumps({"full_gap_scenarios": ["intrinsic_scale_stress"]}),
                encoding="utf-8",
            )
            runtime_dir.joinpath("runtime_measurement_guard_audit.csv").write_text(
                "target_scenario,feature_raw_forward_mean_m,feature_disparity_forward_product_mean_px_m\n"
                "intrinsic_scale_stress,100,500\n"
                "intrinsic_scale_stress,101,510\n"
                "nominal,100,200\n"
                "bbox_outlier_stress,99,210\n",
                encoding="utf-8",
            )

            summary = module.audit_gap_feature_separability(sweep_dir=sweep, output_dir=root / "out")

            self.assertEqual(summary["best_target_scenario"], "intrinsic_scale_stress")
            self.assertEqual(summary["best_feature"], "feature_disparity_forward_product_mean_px_m")
            self.assertEqual(summary["detection_recommendation"], "raw_feature_gap_guard_candidate")
            with (root / "out" / "gap_feature_separability.csv").open(newline="", encoding="utf-8") as handle:
                rows = list(csv.DictReader(handle))
            self.assertEqual(rows[0]["feature"], "feature_disparity_forward_product_mean_px_m")
            self.assertTrue((root / "out" / "gap_feature_separability_summary.json").is_file())
            self.assertTrue((root / "out" / "gap_feature_separability_top_features.png").is_file())


if __name__ == "__main__":
    unittest.main()
