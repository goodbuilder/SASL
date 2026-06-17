from __future__ import annotations

import importlib.util
import sys
import unittest
from pathlib import Path


ROOT_DIR = Path(__file__).resolve().parents[1]


def load_oracle_label_module():
    spec = importlib.util.spec_from_file_location(
        "audit_runtime_oracle_label_selector_under_test",
        ROOT_DIR / "scripts" / "audit_runtime_oracle_label_selector.py",
    )
    if spec is None or spec.loader is None:
        raise RuntimeError("cannot load audit_runtime_oracle_label_selector.py")
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def sample_row(forward: float, disparity: float, truth_offset: float = 0.0) -> dict[str, str]:
    return {
        "raw_right_m": "1.0",
        "raw_down_m": "2.0",
        "raw_forward_m": f"{forward}",
        "raw_range_m": f"{forward}",
        "measured_disparity_px": f"{disparity}",
        "left_u_px": "640.0",
        "left_v_px": "360.0",
        "truth_right_m": f"{truth_offset + 100.0}",
        "truth_down_m": f"{truth_offset + 200.0}",
        "truth_forward_m": f"{truth_offset + 300.0}",
    }


def sample_row_with_product(forward: float, product_px_m: float = 20000.0, truth_offset: float = 0.0) -> dict[str, str]:
    return sample_row(forward, product_px_m / forward, truth_offset=truth_offset)


class AuditRuntimeOracleLabelSelectorTests(unittest.TestCase):
    def test_select_label_uses_raw_features_not_target_truth(self) -> None:
        module = load_oracle_label_module()
        guard = sys.modules["select_stereodistance_profile_by_runtime_measurement_guard"]
        training = []
        for label, forward in (("near_profile", 1000.0), ("far_profile", 2200.0)):
            rows = [sample_row_with_product(forward), sample_row_with_product(forward + 10.0)]
            features = guard.runtime_measurement_features(rows)
            training.append(
                {
                    "label_source_scenario": label,
                    "features": features,
                    "feature_vector": guard.feature_vector(features),
                }
            )
        base_rows = [sample_row_with_product(1010.0, truth_offset=0.0), sample_row_with_product(1020.0, truth_offset=0.0)]
        changed_truth_rows = [
            sample_row_with_product(1010.0, truth_offset=10000.0),
            sample_row_with_product(1020.0, truth_offset=-10000.0),
        ]

        base_selection = module.select_label(
            rows=base_rows,
            training=training,
            policy="nearest_label_centroid",
            transfer_matrix={},
            transfer_low_risk_rmse_limit_m=1.3,
            transfer_low_risk_target_limit_m=2.5,
            transfer_risk_target_weight=0.10,
            top_label_count=3,
        )
        changed_truth_selection = module.select_label(
            rows=changed_truth_rows,
            training=training,
            policy="nearest_label_centroid",
            transfer_matrix={},
            transfer_low_risk_rmse_limit_m=1.3,
            transfer_low_risk_target_limit_m=2.5,
            transfer_risk_target_weight=0.10,
            top_label_count=3,
        )

        self.assertEqual(base_selection["selected_source_scenario"], "near_profile")
        self.assertEqual(base_selection["selected_source_scenario"], changed_truth_selection["selected_source_scenario"])
        self.assertIn("label_distance_rows", base_selection)


if __name__ == "__main__":
    unittest.main()
