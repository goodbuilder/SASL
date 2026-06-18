from __future__ import annotations

import importlib.util
import sys
import unittest
from pathlib import Path


ROOT_DIR = Path(__file__).resolve().parents[1]


def load_script(name: str):
    script_path = ROOT_DIR / "scripts" / f"{name}.py"
    spec = importlib.util.spec_from_file_location(f"{name}_under_test", script_path)
    if spec is None or spec.loader is None:
        raise RuntimeError(f"cannot load {script_path}")
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


class RuntimeTruthRdfTransformContractTests(unittest.TestCase):
    def test_runtime_modules_expose_source_truth_transform_contract(self) -> None:
        for name in (
            "run_runtime_image_prefilter_sweep",
            "run_runtime_motion_fusion_sweep",
            "run_runtime_multiburst_disparity_residual_sweep",
            "audit_runtime_profile_fresh_seed_replay",
            "continue_bbox_outlier_runtime_guards",
        ):
            with self.subTest(name=name):
                module = load_script(name)
                self.assertEqual(module.NED_TO_RDF_CONTRACT, module.TRUTH_RDF_TRANSFORM_CONTRACT)
                self.assertIn("T_source_to_stereo_rdf", module.TRUTH_RDF_TRANSFORM_CONTRACT)
                self.assertIn("axis alias", module.TRUTH_RDF_TRANSFORM_CONTRACT)


if __name__ == "__main__":
    unittest.main()
