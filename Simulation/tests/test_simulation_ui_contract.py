import math
import sys
from pathlib import Path
import unittest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "stereo_ground_sim"))

from stereo_ground_sim.simulation_ui_contract import (  # noqa: E402
    PROFILE_ACTIVE,
    PROFILE_NOT_CONFIGURED,
    PROFILE_UNKNOWN_INVALID,
    RecordingReadinessModel,
    UNSYNCHRONIZED,
    corrected_measurement_state,
    detection_display_state,
    disparity_difference_px,
    fallback_transition,
    intersection_angle_deg,
    plot_series_for_source,
    profile_contract_state,
    should_append_plot,
    source_state,
    source_state_from_monotonic_ns,
    stale_display,
    stale_marker_action,
    synchronized_comparison_errors,
    transition_events,
)


class SimulationUiContractTests(unittest.TestCase):
    def test_disparity_difference_accepts_only_finite_numeric_values(self):
        self.assertEqual(disparity_difference_px(12.5, 10.0), 2.5)
        for invalid in (None, math.nan, math.inf, True, "12"):
            with self.subTest(invalid=invalid):
                self.assertIsNone(disparity_difference_px(invalid, 10.0))
                self.assertIsNone(disparity_difference_px(10.0, invalid))

    def test_intersection_angle_conversion_stays_at_display_boundary(self):
        self.assertAlmostEqual(intersection_angle_deg(math.pi), 180.0)
        self.assertIsNone(intersection_angle_deg("rad"))

    def test_corrected_state_distinguishes_missing_profile_from_invalid_sample(self):
        self.assertEqual(corrected_measurement_state("waiting", {}), "UNKNOWN_INVALID")
        self.assertEqual(
            corrected_measurement_state("invalid", {"profile_id": "yaw_005"}),
            "INVALID",
        )
        self.assertEqual(
            corrected_measurement_state("stale", {"profile_mode": "global_affine"}),
            "STALE",
        )
        self.assertEqual(
            corrected_measurement_state("valid", {"fallback_level": 0}),
            "UNKNOWN_INVALID",
        )
        self.assertEqual(
            corrected_measurement_state("invalid", {"profile_mode": "no_profile"}),
            "NOT CONFIGURED",
        )
        for profile_mode in ("", "none", "disabled", "unavailable"):
            with self.subTest(profile_mode=profile_mode):
                self.assertEqual(
                    corrected_measurement_state("invalid", {"profile_mode": profile_mode}),
                    "NOT CONFIGURED",
                )

    def test_profile_contract_requires_explicit_profile_identity(self):
        self.assertEqual(profile_contract_state({"profile_mode": "no_profile"}), PROFILE_NOT_CONFIGURED)
        self.assertEqual(profile_contract_state({"status": "no_profile"}), PROFILE_NOT_CONFIGURED)
        self.assertEqual(profile_contract_state({"profile_id": "unit"}), PROFILE_ACTIVE)
        self.assertEqual(profile_contract_state({"mode": "sim3"}), PROFILE_ACTIVE)
        for payload in (
            {"fallback_level": 0},
            {"fallback_reason": "nearest_cell"},
            {"profile_id": 42},
            "not-json",
            [],
            {},
        ):
            with self.subTest(payload=payload):
                self.assertEqual(profile_contract_state(payload), PROFILE_UNKNOWN_INVALID)

    def test_monotonic_freshness_ignores_sim_time_pause_and_reset(self):
        received_monotonic_ns = 10_000_000_000
        value = [1.0, 2.0, 3.0]
        self.assertEqual(
            source_state_from_monotonic_ns(received_monotonic_ns, value, received_monotonic_ns + 1_000_000_000),
            "valid",
        )
        # ROS/simulation time may pause while monotonic time advances.
        self.assertEqual(
            source_state_from_monotonic_ns(received_monotonic_ns, value, received_monotonic_ns + 3_000_000_000),
            "stale",
        )
        # A reset/rewind must not turn the old receipt into a fresh sample.
        self.assertEqual(
            source_state_from_monotonic_ns(received_monotonic_ns, value, received_monotonic_ns - 1),
            "stale",
        )

    def test_readiness_model_keeps_ros_provenance_separate_from_monotonic_age(self):
        model = RecordingReadinessModel(mode="debug")
        model.update(
            "ground_truth",
            [1.0, 2.0, 3.0, 4.0],
            receive_ros_time_ns=0,
            receive_monotonic_ns=123,
        )
        self.assertEqual(model.receive_ros_time_snapshot()["ground_truth"], 0)
        self.assertEqual(model.receive_monotonic_snapshot()["ground_truth"], 123)

    def test_detection_state_uses_bbox_validity_before_center_fallback(self):
        self.assertEqual(
            detection_display_state(
                {"bbox_left_valid": True, "bbox_left_u_px": None, "bbox_left_v_px": None},
                "left",
            ),
            "VALID",
        )
        self.assertEqual(
            detection_display_state(
                {"bbox_right_valid": False, "bbox_right_u_px": 1.0, "bbox_right_v_px": 2.0},
                "right",
            ),
            "INVALID",
        )
        self.assertEqual(
            detection_display_state(
                {"bbox_left_u_px": 1.0, "bbox_left_v_px": 2.0},
                "left",
            ),
            "VALID",
        )
        self.assertEqual(
            detection_display_state(
                {"bbox_left_u_px": math.nan, "bbox_left_v_px": 2.0},
                "left",
            ),
            "INVALID",
        )
        self.assertEqual(detection_display_state({}, "left"), "WAITING")
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
        self.assertEqual(source_state(None, None, 10.0), "waiting")
        self.assertEqual(source_state(9.0, None, 10.0), "invalid")
        self.assertEqual(source_state(9.0, [1, 2, 3], 10.0), "valid")
        self.assertEqual(source_state(7.0, [1, 2, 3], 10.0, threshold=2.0), "stale")
        self.assertEqual(source_state(8.0, [1, 2, 3], 9.0, threshold=2.0), "valid")
        self.assertEqual(source_state(6.0, [1, 2, 3], 9.0, threshold=2.0), "stale")
        self.assertEqual(source_state(8.0, [1, math.nan, 3], 9.0, threshold=2.0), "invalid")
        self.assertEqual(stale_marker_action("stale"), "DELETE")
        self.assertIn("STALE", stale_display(1000.23, "stale", unit="m", age=3.2))

    def test_image_heartbeat_transitions(self):
        self.assertEqual(source_state(None, None, 10.0), "waiting")
        self.assertEqual(source_state(9.0, {"received": True}, 10.0), "valid")
        self.assertEqual(source_state(7.0, {"received": True}, 10.0, threshold=2.0), "stale")
        self.assertEqual(transition_events({"left_image": "stale"}, {"left_image": "valid"}), ["left_image: stale → recovered"])

    def test_transitions_are_emitted_once_per_state_change(self):
        previous = {"ground_truth": "waiting", "left_image": "valid"}
        current = {"ground_truth": "valid", "left_image": "stale"}
        self.assertEqual(transition_events(previous, current), ["ground_truth: waiting → valid", "left_image: valid → stale"])
        self.assertEqual(transition_events(current, current), [])

    def test_fallback_transitions_are_normalized_and_single(self):
        self.assertEqual(fallback_transition(False, True), ["Ground Truth fallback started"])
        self.assertEqual(fallback_transition(True, False), ["Ground Truth fallback ended"])
        self.assertEqual(fallback_transition({"fallback": True, "source": "A"}, {"fallback": True, "source": "B"}), ["Ground Truth fallback source changed: A → B"])
        self.assertEqual(fallback_transition(True, True), [])


if __name__ == "__main__":
    unittest.main()
