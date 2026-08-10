from __future__ import annotations

import importlib.util
import sys
import tempfile
import unittest
from pathlib import Path


ROOT_DIR = Path(__file__).resolve().parents[1]
PORTABILITY_PATH = ROOT_DIR / "scripts" / "check_deployable_profile_candidate_package_portability.py"
PACKAGE_TEST_PATH = ROOT_DIR / "tests" / "test_deployable_profile_candidate_package.py"


def load_module(name: str, path: Path):
    spec = importlib.util.spec_from_file_location(name, path)
    if spec is None or spec.loader is None:
        raise RuntimeError(f"cannot load {path}")
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


class CheckDeployableProfileCandidatePackagePortabilityTests(unittest.TestCase):
    def test_copied_candidate_package_remains_portable(self) -> None:
        portability = load_module("check_candidate_package_portability_under_test", PORTABILITY_PATH)
        package_fixture = load_module("candidate_package_fixture_for_portability", PACKAGE_TEST_PATH)
        exporter = load_module(
            "export_candidate_package_for_portability",
            ROOT_DIR / "scripts" / "export_deployable_profile_candidate_package.py",
        )
        with tempfile.TemporaryDirectory() as tmpdir:
            root = Path(tmpdir)
            readiness = package_fixture.make_candidate_fixture(root)
            package = root / "package"
            exporter.export_package(candidate_readiness_json=readiness, output_dir=package)
            result = portability.check_portability(package_dir=package)

        self.assertEqual(result["status"], "PASS")
        self.assertEqual(result["copied_package_check_status"], "PASS")
        self.assertEqual(result["copied_operator_view_status"], "PASS")
        self.assertEqual(result["copied_operator_view_package_type"], "candidate_package")
        self.assertTrue(result["copied_package_dir_ephemeral"])
        self.assertFalse(result["runtime_truth_used"])
        self.assertIn("RDF(right,down,forward)", result["coordinate_contract"])

    def test_missing_package_fails(self) -> None:
        portability = load_module("check_candidate_package_portability_missing_under_test", PORTABILITY_PATH)
        with tempfile.TemporaryDirectory() as tmpdir:
            result = portability.check_portability(package_dir=Path(tmpdir) / "missing")

        self.assertEqual(result["status"], "FAIL")
        self.assertIn("package_dir:missing", result["failures"])


if __name__ == "__main__":
    unittest.main()
