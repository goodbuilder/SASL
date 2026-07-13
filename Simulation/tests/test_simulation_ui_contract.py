import math
import sys
from pathlib import Path
import unittest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "stereo_ground_sim"))

from stereo_ground_sim.simulation_ui_contract import (  # noqa: E402
    UNSYNCHRONIZED,
    plot_series_for_source,
    should_append_plot,
    source_state,
    stale_marker_action,
    synchronized_comparison_errors,
    transition_events,
)


class SimulationUiContractTests(unittest.TestCase):
    def test_sources_are_distinct_and_plot_append_is_source_local(self):
        self.assertEqual(plot_series_for_source("observed_raw"), "observed_raw")
        self.assertEqual(plot_series_for_source("analytic_raw"), "analytic_raw")
        self.assertTrue(should_append_plot("observed_raw", "observed_raw"))
        self.assertFalse(should_append_plot("analytic_raw", "observed_raw"))
        self.assertFalse(should_append_plot("corrected", "ground_truth"))

    def test_unsynchronized_comparison_is_not_calculated(self):
        result = synchronized_comparison_errors(10, 11, 12, 10.5)
        self.assertEqual(result["observed_error"], UNSYNCHRONIZED)
        self.assertEqual(result["corrected_error"], UNSYNCHRONIZED)
        self.assertEqual(result["improvement"], UNSYNCHRONIZED)

    def test_source_staleness_and_nonfinite_values(self):
        self.assertEqual(source_state(8.0, [1, 2, 3], 9.0, threshold=2.0), "valid")
        self.assertEqual(source_state(6.0, [1, 2, 3], 9.0, threshold=2.0), "stale")
        self.assertEqual(source_state(8.0, [1, math.nan, 3], 9.0, threshold=2.0), "invalid")
        self.assertEqual(stale_marker_action("stale"), "DELETE")

    def test_transitions_are_emitted_once_per_state_change(self):
        previous = {"ground_truth": "waiting", "left_image": "valid"}
        current = {"ground_truth": "valid", "left_image": "stale"}
        self.assertEqual(transition_events(previous, current), ["ground_truth: waiting → valid", "left_image: valid → stale"])
        self.assertEqual(transition_events(current, current), [])


if __name__ == "__main__":
    unittest.main()
