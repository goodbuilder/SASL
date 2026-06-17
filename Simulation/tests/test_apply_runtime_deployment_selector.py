from __future__ import annotations

import importlib.util
import csv
import json
import sys
import tempfile
import unittest
from pathlib import Path


ROOT_DIR = Path(__file__).resolve().parents[1]


def load_module():
    spec = importlib.util.spec_from_file_location(
        "apply_runtime_deployment_selector_under_test",
        ROOT_DIR / "scripts" / "apply_runtime_deployment_selector.py",
    )
    if spec is None or spec.loader is None:
        raise RuntimeError("cannot load apply_runtime_deployment_selector.py")
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


class ApplyRuntimeDeploymentSelectorTests(unittest.TestCase):
    def test_override_rule_uses_feature_prefix_contract(self) -> None:
        module = load_module()
        rule = module.parse_override_rule(
            "use catalog_guard unless feature_left_v_std_px ge 129.7294559911429, then use single_pooled"
        )

        self.assertTrue(module.override_applies({"left_v_std_px": 130.0}, rule))
        self.assertFalse(module.override_applies({"left_v_std_px": 120.0}, rule))

    def test_apply_selector_to_rows_returns_override_label_without_truth(self) -> None:
        module = load_module()

        def fake_read_json(path: Path) -> dict[str, object]:
            return {
                "scenario_catalog_json": "catalog.json",
                "transfer_matrix_csv": "transfer.csv",
                "known_selector_policy": "intrinsic_scale_gap_transfer_risk",
                "runtime_rule": "use catalog_guard unless feature_left_v_std_px ge 10, then use single_pooled",
                "override_profile_label": "single_pooled",
                "override_profile_json": "profiles/single.json",
                "selection_contract": "raw features only",
            }

        def fake_runtime_measurement_features(rows):
            return {"left_v_std_px": 11.0, "raw_forward_mean_m": 100.0}

        def fake_select_source_by_runtime_measurement(**kwargs):
            return {
                "selected_source_scenario": "bbox_outlier_stress",
                "preliminary_source_scenario": "nominal",
            }

        module.read_json = fake_read_json
        module.load_catalog = lambda path: {"profiles": [{"scenario": "nominal"}]}
        module.source_scenarios_for_mode = lambda **kwargs: ["nominal", "bbox_outlier_stress"]
        module.build_feature_library = lambda **kwargs: [{"scenario": "nominal"}]
        module.load_transfer_matrix = lambda path: {}
        module.runtime_measurement_features = fake_runtime_measurement_features
        module.select_source_by_runtime_measurement = fake_select_source_by_runtime_measurement

        result = module.apply_selector_to_rows(
            selector_json=Path("/tmp/package/known_catalog_runtime_selector.json"),
            sweep_dir=Path("/tmp/sweep"),
            rows=[{"raw_forward_m": "100", "measured_disparity_px": "10"}],
        )

        self.assertEqual(result["selected_label"], "single_pooled")
        self.assertEqual(result["baseline_source_scenario"], "bbox_outlier_stress")
        self.assertTrue(result["override_applied"])
        self.assertEqual(result["axis_contract"], module.RDF_AXIS_CONTRACT)
        self.assertIn("feature_left_v_std_px", json.loads(result["override_rule_json"])["feature_name"])

    def test_apply_selector_to_sweep_compares_reference_audit(self) -> None:
        module = load_module()
        with tempfile.TemporaryDirectory() as tmpdir:
            root = Path(tmpdir)
            sweep = root / "sweep"
            csv_dir = sweep / "nominal" / "seed_1"
            csv_dir.mkdir(parents=True)
            sample_csv = csv_dir / "calibration_samples.csv"
            sample_csv.write_text(
                "raw_right_m,raw_down_m,raw_forward_m,raw_range_m,measured_disparity_px,left_u_px,left_v_px\n"
                "0,0,100,100,10,640,360\n",
                encoding="utf-8",
            )
            selector_json = root / "selector.json"
            selector_json.write_text("{}", encoding="utf-8")
            reference_csv = root / "reference.csv"
            with reference_csv.open("w", newline="", encoding="utf-8") as handle:
                writer = csv.DictWriter(
                    handle,
                    fieldnames=["target_scenario", "target_seed", "selected_candidate", "override_applied"],
                )
                writer.writeheader()
                writer.writerow(
                    {
                        "target_scenario": "nominal",
                        "target_seed": "seed_1",
                        "selected_candidate": "catalog_guard",
                        "override_applied": "False",
                    }
                )

            def fake_apply_selector_to_rows(**kwargs):
                return {
                    "selected_label": "catalog_guard",
                    "baseline_source_scenario": "nominal",
                    "override_applied": False,
                    "axis_contract": module.RDF_AXIS_CONTRACT,
                    "public_topic_contract": module.PUBLIC_TOPIC_CONTRACT,
                }

            module.apply_selector_to_rows = fake_apply_selector_to_rows

            summary = module.apply_selector_to_sweep(
                selector_json=selector_json,
                sweep_dir=sweep,
                output_dir=root / "out",
                reference_audit_csv=reference_csv,
            )

            self.assertTrue(summary["selected_candidate_match_all"])
            self.assertTrue(summary["override_match_all"])
            rows = module.read_csv(root / "out" / "runtime_deployment_selector_sweep_audit.csv")
            self.assertEqual(rows[0]["matches_reference_selected_candidate"], "True")


if __name__ == "__main__":
    unittest.main()
