import json
import math
import sys
import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "stereo_ground_sim"))

from stereo_ground_sim.px4_status_adapter import (  # noqa: E402
    MISSION_PHASES,
    PX4_STATUS_TOPIC,
    build_status_payload,
    decode_px4_custom_mode,
    infer_mission_phase,
    navigation_mode_from_heartbeat,
)
from stereo_ground_sim.simulation_ui_contract import (  # noqa: E402
    TOP_BAR_FIELDS,
    parse_px4_status,
    px4_contract_state,
    px4_overall_status,
    source_state,
)


class Px4OperatorUiContractTests(unittest.TestCase):
    def test_status_payload_is_authoritative_and_finite(self):
        payload = build_status_payload(
            connected=True,
            armed=False,
            navigation_mode="OFFBOARD",
            failsafe=False,
            mission_phase="CONNECTED",
            local_ned_m={"north": 1.0, "east": 2.0, "down": -3.0},
            velocity_ned_mps={"north": 0.0, "east": 0.0, "down": 0.0},
            target_distance_m=450.0,
            actual_distance_m=500.0,
            sample_timestamp_us=120000,
            receive_monotonic_s=10.0,
            status="ok",
        )
        self.assertEqual(payload["flight_authority"], "px4_sitl")
        self.assertFalse(payload["gazebo_motion_enabled"])
        self.assertEqual(parse_px4_status(json.dumps(payload))["schema_version"], 1)
        self.assertEqual(px4_contract_state(payload), "safe")
        self.assertEqual(PX4_STATUS_TOPIC, "/simulation/px4/status")
        self.assertTrue(math.isfinite(payload["receive_monotonic_s"]))

    def test_invalid_contract_is_visible_as_error(self):
        payload = {"schema_version": 1, "flight_authority": "gazebo", "gazebo_motion_enabled": True}
        self.assertEqual(px4_contract_state(payload), "violation")
        self.assertIn("CONTRACT VIOLATION", px4_overall_status(payload))

    def test_missing_status_is_not_fabricated(self):
        self.assertEqual(parse_px4_status("not-json"), {})
        self.assertEqual(px4_contract_state({}), "unavailable")
        self.assertIn("unavailable", px4_overall_status({}).lower())

    def test_phase_is_derived_from_telemetry(self):
        common = {
            "local_ned_m": {"north": 0.0, "east": 0.0, "down": -10.0},
            "velocity_ned_mps": {"north": -10.0, "east": 0.0, "down": 0.0},
        }
        self.assertEqual(infer_mission_phase(connected=False, armed=False, navigation_mode=None, **common), "WAITING")
        self.assertEqual(infer_mission_phase(connected=True, armed=False, navigation_mode="STABILIZED", **common), "CONNECTED")
        self.assertEqual(infer_mission_phase(connected=True, armed=True, navigation_mode="OFFBOARD", **common), "APPROACH")
        self.assertEqual(infer_mission_phase(connected=True, armed=True, navigation_mode="LAND", **common), "LANDING")
        self.assertTrue(set(("WAITING", "CONNECTED", "APPROACH", "LANDING")).issubset(MISSION_PHASES))

    def test_takeoff_vertical_motion_is_not_labeled_approach(self):
        self.assertEqual(
            infer_mission_phase(
                connected=True,
                armed=True,
                navigation_mode="OFFBOARD",
                local_ned_m={"north": 0.0, "east": 0.0, "down": -1.4},
                velocity_ned_mps={"north": 0.0, "east": 0.0, "down": -2.4},
            ),
            "TAKEOFF",
        )
        self.assertEqual(
            infer_mission_phase(
                connected=True,
                armed=True,
                navigation_mode="AUTO_LAND",
                local_ned_m={"north": -50.0, "east": 0.0, "down": -10.0},
                velocity_ned_mps={"north": 0.0, "east": 0.0, "down": 0.0},
            ),
            "LANDING",
        )

    def test_px4_custom_mode_decodes_offboard_and_auto_land(self):
        self.assertEqual(decode_px4_custom_mode(6 << 16), "OFFBOARD")
        self.assertEqual(decode_px4_custom_mode((4 << 16) | (6 << 24)), "AUTO_LAND")

    def test_unknown_generic_mode_falls_back_to_px4_custom_mode(self):
        class Heartbeat:
            custom_mode = 6 << 16

        class Mavlink:
            @staticmethod
            def mode_string_v10(_message):
                return "UNKNOWN"

        self.assertEqual(navigation_mode_from_heartbeat(Heartbeat(), Mavlink), "OFFBOARD")

    def test_top_bar_contract_is_exactly_ordered(self):
        self.assertEqual(
            TOP_BAR_FIELDS,
            ("AUTHORITY", "GAZEBO MOTION", "PX4 CONNECTION", "ARM", "MODE", "MISSION PHASE", "GT SOURCE", "OVERALL STATUS"),
        )

    def test_source_state_still_distinguishes_waiting_from_invalid(self):
        self.assertEqual(source_state(None, None, 10.0), "waiting")
        self.assertEqual(source_state(9.0, None, 10.0), "invalid")
        self.assertEqual(source_state(7.0, {"received": True}, 10.0, threshold=2.0), "stale")


if __name__ == "__main__":
    unittest.main()
