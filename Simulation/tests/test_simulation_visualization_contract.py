from __future__ import annotations

import math
import sys
import unittest
from pathlib import Path


PACKAGE_DIR = Path(__file__).resolve().parents[1] / "stereo_ground_sim"
if str(PACKAGE_DIR) not in sys.path:
    sys.path.insert(0, str(PACKAGE_DIR))

from stereo_ground_sim.simulation_visualization_node import (  # noqa: E402
    BASELINE_FRAME,
    CORRECTED_PATH_TOPIC,
    GROUND_TRUTH_PATH_TOPIC,
    MARKER_TOPIC,
    RAW_PATH_TOPIC,
    camera_centers_baseline_frd,
    finite_vector,
    safe_marker_vector,
    stable_marker_identity,
)


class SimulationVisualizationContractTests(unittest.TestCase):
    def test_baseline_centers_are_symmetric_and_exact(self) -> None:
        left, right = camera_centers_baseline_frd(25.0)
        self.assertEqual(left, (0.0, -12.5, 0.0))
        self.assertEqual(right, (0.0, 12.5, 0.0))
        self.assertAlmostEqual(math.dist(left, right), 25.0, delta=1.0e-9)

    def test_vector_rejects_non_finite_values_without_producing_marker_data(self) -> None:
        self.assertEqual(finite_vector((1, 2, 3)), (1.0, 2.0, 3.0))
        self.assertIsNone(finite_vector((1, math.nan, 3)))
        self.assertIsNone(finite_vector((1, math.inf, 3)))
        self.assertIsNone(safe_marker_vector((1, 2)))

    def test_marker_identity_is_stable(self) -> None:
        self.assertEqual(stable_marker_identity("ground_truth", 20), ("ground_truth", 20))
        with self.assertRaises(ValueError):
            stable_marker_identity("", 20)
        with self.assertRaises(ValueError):
            stable_marker_identity("ground_truth", -1)

    def test_contract_topics_and_frame_are_explicit(self) -> None:
        self.assertEqual(BASELINE_FRAME, "baseline_center_frd")
        self.assertEqual(MARKER_TOPIC, "/simulation/visualization/markers")
        self.assertEqual(GROUND_TRUTH_PATH_TOPIC, "/simulation/visualization/ground_truth_path")
        self.assertEqual(RAW_PATH_TOPIC, "/simulation/visualization/raw_path")
        self.assertEqual(CORRECTED_PATH_TOPIC, "/simulation/visualization/corrected_path")

    def test_source_contract_preserves_operator_toggles_and_launch_arguments(self) -> None:
        script = Path(__file__).resolve().parents[1] / "run_operator_view.sh"
        text = script.read_text(encoding="utf-8")
        for option in ("--gazebo-gui", "--no-gazebo-gui", "--rviz", "--no-rviz", "--headless"):
            self.assertIn(option, text)
        self.assertIn('gz_gui:=$gazebo_gui', text)
        self.assertIn('launch_rviz:=$launch_rviz', text)
        self.assertIn('launch_visualization_markers:=true', text)

    def test_source_contract_does_not_reimplement_production_math(self) -> None:
        source = (Path(__file__).resolve().parents[1] / "stereo_ground_sim" / "stereo_ground_sim" / "simulation_visualization_node.py").read_text(encoding="utf-8")
        self.assertIn("nominal_stereo_rays_from_centers", source)
        self.assertNotIn("triangulate_nominal_stereo_centers", source)
        self.assertNotIn("project_nominal_stereo_point", source)

    def test_analytic_raw_and_stale_path_contract_are_explicit(self) -> None:
        source = (Path(__file__).resolve().parents[1] / "stereo_ground_sim" / "stereo_ground_sim" / "simulation_visualization_node.py").read_text(encoding="utf-8")
        self.assertIn('"analytic_attitude_raw"', source)
        self.assertIn("self.receive_times", source)
        self.assertIn("zero stamp", source)
        self.assertIn('"rays/analytic/left"', source)
        self.assertNotIn('"rays/observed/left"', source)

    def test_legacy_distance_marker_uses_the_rviz_fixed_frame(self) -> None:
        source = (Path(__file__).resolve().parents[1] / "stereo_ground_sim" / "stereo_ground_sim" / "stereo_distance_node.py").read_text(encoding="utf-8")
        self.assertIn('marker.header.frame_id = "baseline_center_frd"', source)
        self.assertNotIn('marker.header.frame_id = "map"', source)


if __name__ == "__main__":
    unittest.main()
