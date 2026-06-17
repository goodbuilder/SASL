from __future__ import annotations

import importlib.util
import json
import sys
import tempfile
import unittest
from pathlib import Path


ROOT_DIR = Path(__file__).resolve().parents[1]


def load_module():
    spec = importlib.util.spec_from_file_location(
        "export_runtime_deployment_package_under_test",
        ROOT_DIR / "scripts" / "export_runtime_deployment_package.py",
    )
    if spec is None or spec.loader is None:
        raise RuntimeError("cannot load export_runtime_deployment_package.py")
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


class ExportRuntimeDeploymentPackageTests(unittest.TestCase):
    def test_exports_active_profile_and_selector_manifest(self) -> None:
        module = load_module()
        with tempfile.TemporaryDirectory() as tmpdir:
            root = Path(tmpdir)
            strategy = root / "strategy.json"
            profile = root / "single_profile.json"
            catalog = root / "catalog.json"
            transfer = root / "transfer.csv"
            output = root / "package"
            strategy.write_text(
                json.dumps(
                    {
                        "known_catalog_recommendation": "known_catalog_guard_with_single_pooled_override",
                        "known_catalog_runtime_rule": (
                            "use catalog_guard unless feature_left_v_std_px ge 129.7, then use single_pooled"
                        ),
                        "unknown_regime_recommendation": "single_pooled_runtime_profile",
                        "unknown_regime_runtime_rule": "use single pooled",
                        "known_catalog_target_max_abs_error_max_m": 8.4,
                        "unknown_regime_target_max_abs_error_max_m": 8.4,
                    }
                ),
                encoding="utf-8",
            )
            profile.write_text('{"mode":"estimated_to_truth_sim3","profile_id":"unit"}\n', encoding="utf-8")
            catalog.write_text('{"mode":"scenario_profile_catalog","profiles":[]}\n', encoding="utf-8")
            transfer.write_text("source_scenario,target_scenario\nnominal,nominal\n", encoding="utf-8")

            manifest = module.export_runtime_deployment_package(
                strategy_recommendation_json=strategy,
                single_pooled_profile_json=profile,
                scenario_catalog_json=catalog,
                transfer_matrix_csv=transfer,
                output_dir=output,
                target_range_m=1852.0,
                target_error_limit_m=10.0,
            )

            self.assertEqual(manifest["unknown_active_profile"], "active_runtime_profile_unknown.json")
            self.assertTrue((output / "active_runtime_profile_unknown.json").is_file())
            self.assertTrue((output / "profiles" / "single_pooled_runtime_profile.json").is_file())
            selector = json.loads((output / "known_catalog_runtime_selector.json").read_text(encoding="utf-8"))
            self.assertEqual(selector["axis_contract"], module.RDF_AXIS_CONTRACT)
            self.assertIn("raw_forward_m", selector["selection_input_fields"])
            self.assertIn("truth_forward_m", selector["forbidden_runtime_fields"])
            self.assertIn("feature_left_v_std_px", selector["runtime_rule"])
            self.assertTrue((output / "runtime_deployment_package_manifest.txt").is_file())


if __name__ == "__main__":
    unittest.main()
