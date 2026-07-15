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
        for tab in ("Live Trends", "Diagnostics", "Event Log"):
            self.assertIn(tab, source)
        for field in (
            "truth",
            "raw",
            "corrected",
            "disparity",
            "vertical_disparity",
            "ray_gap",
            "intersection_angle",
        ):
            self.assertIn(field, source)
        for label in (
            "EXPERIMENT CONDITION",
            "CURRENT MEASUREMENT — baseline_center_frd [m]",
            "TRIANGULATION QUALITY",
            "Observed/BBox Raw",
            "Analytic Attitude Raw",
            "observed_disparity_px",
            "analytic_disparity_px",
            "Difference Δd [px]",
        ):
            self.assertIn(label, source)
        for label in ("PX4 LOCAL NED", "AUXILIARY, NOT GROUND TRUTH"):
            self.assertIn(label, source)
        self.assertNotIn('QGroupBox("FLIGHT AUTHORITY"', source)
        self.assertNotIn('QGroupBox("PX4 FLIGHT STATE"', source)
        self.assertNotIn("_top_status_labels", source)
        self.assertNotIn("font-size: 8px", source)
        for source_name in ("ground_truth_range", "observed_disparity_px", "analytic_disparity_px", "pixels", "ray_quality"):
            self.assertIn(source_name, source)
        self.assertTrue(any(isinstance(node, ast.ClassDef) and node.name == "OperatorPanel" for node in ast.walk(tree)))

    def test_panel_does_not_fake_frame_or_sample_timestamp(self):
        source = PANEL.read_text(encoding="utf-8")
        self.assertIn('Message-level frame: unavailable', source)
        self.assertIn('Message sample timestamp: unavailable', source)
        self.assertIn('UNSYNCHRONIZED', source)
        self.assertNotIn('sample_time = time.monotonic()', source)
        self.assertIn("synchronized=False", source)
        self.assertIn("/simulation/px4/status", source)
        self.assertNotIn("--reload-import", (ROOT / "run_operator_view.sh").read_text(encoding="utf-8"))

    def test_panel_image_mode_matches_the_requested_camera_topics(self):
        source = PANEL.read_text(encoding="utf-8")
        self.assertIn('os.environ.get("RQT_IMAGE_TOPIC_MODE", "detection")', source)
        self.assertIn('if IMAGE_MODE == "raw":', source)
        self.assertIn('("Left Camera", "/left_camera/image")', source)
        self.assertIn('("Right Camera", "/right_camera/image")', source)
        self.assertIn('("Left Detection", "/left_camera/detection_image")', source)
        self.assertIn('("Right Detection", "/right_camera/detection_image")', source)

    def test_plot_updates_are_source_local(self):
        source = PANEL.read_text(encoding="utf-8")
        self.assertIn("self._bridge.plot.emit(key, value or [])", source)
        self.assertNotIn("for name, _topic in POSITION_TOPICS:\n            self._plot_stack.append", source)

    def test_main_position_contract_has_no_receive_age_column(self):
        source = PANEL.read_text(encoding="utf-8")
        self.assertIn('headers = ("Source", "F", "R", "D", "Range", "State")', source)
        self.assertNotIn('headers = ("Source", "F", "R", "D", "Range", "State", "Receive age")', source)

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
