from pathlib import Path
import ast
import math
import sys
import unittest


ROOT = Path(__file__).resolve().parents[1]
PANEL = ROOT / "stereo_ground_sim" / "stereo_ground_sim" / "operator_panel.py"
VIS = ROOT / "stereo_ground_sim" / "stereo_ground_sim" / "simulation_visualization_node.py"
RVIZ = ROOT / "stereo_ground_sim" / "rviz" / "stereo_ground_sim.rviz"


class OperatorPanelLayoutContractTests(unittest.TestCase):
    def test_panel_source_declares_side_by_side_read_only_sections(self):
        source = PANEL.read_text(encoding="utf-8")
        tree = ast.parse(source)
        self.assertIn("QHBoxLayout", source)
        for tab in ("Range & Error", "FRD", "Geometry", "Attitude", "Event Log"):
            self.assertIn(tab, source)
        for field in ("truth", "raw", "corrected", "disparity", "vertical_disparity", "ray_gap", "intersection_angle"):
            self.assertIn(field, source)
        self.assertTrue(any(isinstance(node, ast.ClassDef) and node.name == "OperatorPanel" for node in ast.walk(tree)))

    def test_panel_does_not_fake_frame_or_sample_timestamp(self):
        source = PANEL.read_text(encoding="utf-8")
        self.assertIn('Message-level frame: unavailable', source)
        self.assertIn('Message sample timestamp: unavailable', source)
        self.assertIn('samples not synchronized', source)
        self.assertNotIn('sample_time = time.monotonic()', source)

    def test_pure_numeric_helpers_reject_non_finite_values(self):
        sys.path.insert(0, str(ROOT / "stereo_ground_sim"))
        try:
            from stereo_ground_sim.operator_panel import finite_float, improvement
        except Exception as exc:  # Qt is optional in non-rqt test environments.
            self.skipTest(f"operator panel dependencies unavailable: {exc}")
        self.assertEqual(finite_float(1.5), 1.5)
        self.assertIsNone(finite_float(math.nan))
        self.assertIsNone(finite_float(math.inf))
        self.assertIsNone(finite_float(True))
        self.assertEqual(improvement(0.0, 0.1), "N/A — samples not synchronized")

    def test_visualization_ray_namespaces_are_explicit(self):
        source = VIS.read_text(encoding="utf-8")
        self.assertIn('"rays/analytic/left"', source)
        self.assertIn('"rays/analytic/right"', source)
        self.assertIn('rays/observed: unavailable', source)
        self.assertNotIn('"rays", left_ray', source)

    def test_rviz_keeps_observed_rays_disabled_and_legacy_display(self):
        source = RVIZ.read_text(encoding="utf-8")
        self.assertIn("Value: /stereo/distance_marker", source)
        self.assertIn("rays/analytic/left: true", source)
        self.assertIn("rays/analytic/right: true", source)
        self.assertIn("rays/observed: false", source)


if __name__ == "__main__":
    unittest.main()
