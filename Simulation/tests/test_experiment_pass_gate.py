from __future__ import annotations

import importlib.util
import sys
import unittest
from pathlib import Path


ROOT_DIR = Path(__file__).resolve().parents[1]


def load_gate_module():
    spec = importlib.util.spec_from_file_location(
        "experiment_pass_gate_under_test",
        ROOT_DIR / "scripts" / "experiment_pass_gate.py",
    )
    if spec is None or spec.loader is None:
        raise RuntimeError("cannot load experiment_pass_gate.py")
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def base_metrics() -> dict[str, object]:
    return {
        "truth_point_defined": True,
        "runtime_feature_defined": True,
        "runtime_truth_used": False,
        "automated_search": True,
        "search_space_defined": True,
        "selection_metric_defined": True,
        "stereodistance_applicable": True,
        "runtime_profile_exportable": True,
        "offline_only": False,
        "coordinate_contracts_explicit": True,
        "alias_contracts_explicit": True,
        "truth_rdf_transform_contract": (
            "explicit_transform_required: evaluate in stereo_center_RDF; direct axis alias "
            "rdf_right=east/rdf_down=down/rdf_forward=north is forbidden"
        ),
        "direct_ned_to_rdf_axis_alias_allowed": False,
        "validation_range_rmse_raw_m": 5.0,
        "validation_range_rmse_corrected_m": 3.0,
        "validation_position_rmse_raw_m": 6.0,
        "validation_position_rmse_corrected_m": 4.0,
        "validation_right_rmse_raw_m": 2.0,
        "validation_right_rmse_corrected_m": 1.0,
        "validation_down_rmse_raw_m": 2.0,
        "validation_down_rmse_corrected_m": 1.0,
        "validation_forward_rmse_raw_m": 5.0,
        "validation_forward_rmse_corrected_m": 3.0,
        "target_range_m": 1852.0,
        "target_range_sample_count": 4,
        "target_range_min_sample_count": 3,
        "target_range_abs_mean_error_m": 6.0,
        "target_range_tolerance_m": 10.0,
        "train_validation_gap_range_rmse_m": 0.5,
        "max_train_validation_gap_range_rmse_m": 2.0,
    }


class ExperimentPassGateTests(unittest.TestCase):
    def test_strict_pass_when_all_required_checks_pass(self) -> None:
        module = load_gate_module()

        result = module.evaluate(base_metrics())

        self.assertEqual(result["verdict"], "STRICT_PASS")
        self.assertTrue(result["checks"]["axis_strict_improvement"])

    def test_runtime_truth_use_forces_fail(self) -> None:
        module = load_gate_module()
        metrics = base_metrics()
        metrics["runtime_truth_used"] = True

        result = module.evaluate(metrics)

        self.assertEqual(result["verdict"], "FAIL")
        self.assertIn("runtime_truth_used", result["reasons"])

    def test_low_target_sample_count_is_conditional_when_core_metrics_pass(self) -> None:
        module = load_gate_module()
        metrics = base_metrics()
        metrics["target_range_sample_count"] = 1

        result = module.evaluate(metrics)

        self.assertEqual(result["verdict"], "CONDITIONAL_PASS")
        self.assertIn("target_range_under_sampled", result["reasons"])

    def test_legacy_1852m_fields_remain_supported(self) -> None:
        module = load_gate_module()
        metrics = base_metrics()
        for key in list(metrics):
            if key.startswith("target_range"):
                del metrics[key]
        metrics.update(
            {
                "target_1852m_sample_count": 4,
                "target_1852m_min_sample_count": 3,
                "target_1852m_abs_mean_error_m": 6.0,
                "target_1852m_tolerance_m": 10.0,
            }
        )

        result = module.evaluate(metrics)

        self.assertEqual(result["verdict"], "STRICT_PASS")
        self.assertTrue(result["checks"]["target_range_error_within_tolerance"])

    def test_direct_ned_to_rdf_alias_forces_fail(self) -> None:
        module = load_gate_module()
        metrics = base_metrics()
        metrics["truth_rdf_transform_contract"] = (
            "synthetic_bridge: rdf_right=east, rdf_down=down, rdf_forward=north"
        )

        result = module.evaluate(metrics)

        self.assertEqual(result["verdict"], "FAIL")
        self.assertFalse(result["checks"]["direct_axis_alias_forbidden"])
        self.assertIn("direct_axis_alias_not_forbidden", result["reasons"])

    def test_direct_alias_allowed_flag_forces_fail(self) -> None:
        module = load_gate_module()
        metrics = base_metrics()
        metrics["direct_ned_to_rdf_axis_alias_allowed"] = True

        result = module.evaluate(metrics)

        self.assertEqual(result["verdict"], "FAIL")
        self.assertIn("direct_axis_alias_not_forbidden", result["reasons"])


if __name__ == "__main__":
    unittest.main()
