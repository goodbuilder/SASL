from __future__ import annotations

import sys
import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "stereo_ground_sim"))

from stereo_ground_sim.simulation_ui_contract import (  # noqa: E402
    evaluate_recording_readiness,
)


def safe_px4():
    return {
        "px4_connected": True,
        "flight_contract": {
            "verified": True,
            "source": "stereo_distance_node_parameters",
            "flight_authority": "px4_sitl",
            "fly_target_in_gazebo": False,
            "mission_profile": "manual",
            "actual_distance_source": "gazebo",
        },
    }


def safe_statuses():
    return {
        "oracle_source_role": "target_reference_link",
        "oracle_valid": True,
        "oracle_fallback_used": False,
        "observation_mode": "both",
        "bbox_left_valid": True,
        "bbox_right_valid": True,
    }


def all_valid_states():
    return {
        key: "valid"
        for key in (
            "ground_truth",
            "observed_raw",
            "analytic_raw",
            "corrected",
            "attitude_status",
            "observation_status",
            "profile_status",
            "left_image",
            "right_image",
            "px4_status",
        )
    }


class RecordingReadinessTests(unittest.TestCase):
    def test_all_waiting_research_is_blocked(self):
        result = evaluate_recording_readiness(
            mode="full",
            observation_mode="both",
            source_states={},
            px4_status={},
            observation_status={},
            attitude_status={},
            profile_status={"profile_mode": "no_profile"},
        )
        self.assertFalse(result.start_allowed)
        self.assertFalse(result.formal_eligible)
        self.assertTrue(result.reasons)

    def test_safe_fresh_required_sources_enable_research(self):
        states = all_valid_states()
        result = evaluate_recording_readiness(
            mode="full",
            observation_mode="both",
            source_states=states,
            px4_status=safe_px4(),
            observation_status=safe_statuses(),
            attitude_status=safe_statuses(),
            profile_status={"profile_mode": "no_profile"},
        )
        self.assertTrue(result.start_allowed)
        self.assertTrue(result.formal_eligible)
        self.assertEqual(result.reasons, ())

    def test_observation_mode_gates_are_source_specific(self):
        states = all_valid_states()
        states["analytic_raw"] = "waiting"
        result = evaluate_recording_readiness(
            mode="full",
            observation_mode="both",
            source_states=states,
            px4_status=safe_px4(),
            observation_status=safe_statuses(),
            attitude_status=safe_statuses(),
            profile_status={"profile_mode": "no_profile"},
        )
        self.assertFalse(result.start_allowed)
        self.assertTrue(any("analytic raw" in reason for reason in result.reasons))

        states["analytic_raw"] = "valid"
        states["observed_raw"] = "invalid"
        result = evaluate_recording_readiness(
            mode="full",
            observation_mode="bbox",
            source_states=states,
            px4_status=safe_px4(),
            observation_status=safe_statuses(),
            attitude_status=safe_statuses(),
            profile_status={"profile_mode": "no_profile"},
        )
        self.assertFalse(result.start_allowed)

    def test_profile_rules_and_debug_override(self):
        states = all_valid_states()
        states["corrected"] = "waiting"
        common = dict(
            source_states=states,
            px4_status=safe_px4(),
            observation_status=safe_statuses(),
            attitude_status=safe_statuses(),
        )
        no_profile = evaluate_recording_readiness(
            mode="lite", observation_mode="both", profile_status={"profile_mode": "no_profile"}, **common
        )
        self.assertTrue(no_profile.start_allowed)
        active_profile = evaluate_recording_readiness(
            mode="lite", observation_mode="both", profile_status={"profile_mode": "sim3"}, **common
        )
        self.assertFalse(active_profile.start_allowed)

        debug = evaluate_recording_readiness(
            mode="debug",
            observation_mode="both",
            source_states={},
            px4_status={},
            observation_status={},
            attitude_status={},
            profile_status={},
        )
        self.assertTrue(debug.start_allowed)
        self.assertFalse(debug.formal_eligible)
        self.assertIn("DEBUG — NON-FORMAL", debug.reasons)

    def test_missing_profile_status_cannot_be_treated_as_no_profile(self):
        states = all_valid_states()
        states["profile_status"] = "stale"
        result = evaluate_recording_readiness(
            mode="full",
            observation_mode="both",
            source_states=states,
            px4_status=safe_px4(),
            observation_status=safe_statuses(),
            attitude_status=safe_statuses(),
            profile_status={"profile_mode": "no_profile"},
        )
        self.assertFalse(result.start_allowed)
        self.assertTrue(any("profile status STALE" in reason for reason in result.reasons))

    def test_fallback_level_only_profile_is_unknown_and_blocks_formal_readiness(self):
        result = evaluate_recording_readiness(
            mode="full",
            observation_mode="both",
            source_states=all_valid_states(),
            px4_status=safe_px4(),
            observation_status=safe_statuses(),
            attitude_status=safe_statuses(),
            profile_status={"fallback_level": 0},
        )
        self.assertFalse(result.start_allowed)
        self.assertFalse(result.formal_eligible)
        self.assertIn("profile contract UNKNOWN_INVALID", result.reasons)

    def test_malformed_profile_payload_is_unknown_and_no_profile_is_configured(self):
        malformed = evaluate_recording_readiness(
            mode="full",
            observation_mode="both",
            source_states=all_valid_states(),
            px4_status=safe_px4(),
            observation_status=safe_statuses(),
            attitude_status=safe_statuses(),
            profile_status={"profile_id": 7},
        )
        self.assertFalse(malformed.start_allowed)
        self.assertFalse(malformed.formal_eligible)
        self.assertIn("profile contract UNKNOWN_INVALID", malformed.reasons)

        no_profile = evaluate_recording_readiness(
            mode="full",
            observation_mode="both",
            source_states=all_valid_states(),
            px4_status=safe_px4(),
            observation_status=safe_statuses(),
            attitude_status=safe_statuses(),
            profile_status={"profile_mode": "no_profile"},
        )
        self.assertTrue(no_profile.start_allowed)
        self.assertTrue(no_profile.formal_eligible)


if __name__ == "__main__":
    unittest.main()
