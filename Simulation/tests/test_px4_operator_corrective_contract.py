from __future__ import annotations

import importlib.util
import json
import os
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path


os.environ.setdefault("ROS_LOG_DIR", "/tmp/sasl-px4-ui-corrective-ros-log")
if not os.environ.get("QT_QPA_PLATFORM"):
    os.environ["QT_QPA_PLATFORM"] = "offscreen"

import yaml  # noqa: E402
from launch import LaunchContext  # noqa: E402
from launch.actions import DeclareLaunchArgument  # noqa: E402


SIM_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(SIM_ROOT / "stereo_ground_sim"))

from stereo_ground_sim.px4_status_adapter import (  # noqa: E402
    DEFAULT_CONNECT,
    FLIGHT_CONTRACT_PARAMETER_NAMES,
    build_status_payload,
    flight_contract_from_parameters,
    local_position_snapshot,
    mission_phase_provenance,
    px4_connection_snapshot,
)
from stereo_ground_sim.simulation_ui_contract import (  # noqa: E402
    FLIGHT_CONTRACT_SOURCE,
    mission_phase_display,
    px4_contract_state,
    px4_overall_status,
)


LAUNCH_FILE = SIM_ROOT / "stereo_ground_sim" / "launch" / "stereo_ground_sim.launch.py"
RUN_OPERATOR_VIEW = SIM_ROOT / "run_operator_view.sh"


def load_launch_module():
    spec = importlib.util.spec_from_file_location("stereo_ground_sim_corrective_launch", LAUNCH_FILE)
    if spec is None or spec.loader is None:
        raise RuntimeError(f"could not load {LAUNCH_FILE}")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def safe_launch_context(module):
    context = LaunchContext()
    for action in module.generate_launch_description().entities:
        if isinstance(action, DeclareLaunchArgument):
            action.execute(context)
    return context


def status_payload(**overrides):
    values = {
        "connected": True,
        "armed": False,
        "navigation_mode": "STABILIZED",
        "failsafe": False,
        "mission_phase": "CONNECTED",
        "local_ned_m": {"north": 1.0, "east": 2.0, "down": -3.0},
        "velocity_ned_mps": {"north": 0.0, "east": 0.0, "down": 0.0},
        "target_distance_m": 450.0,
        "actual_distance_m": 500.0,
        "sample_timestamp_us": 120000,
        "receive_monotonic_s": 10.0,
        "status": "OK",
    }
    values.update(overrides)
    return build_status_payload(**values)


class Px4OperatorCorrectiveContractTests(unittest.TestCase):
    def test_generated_yaml_contains_live_contract_with_yaml_types(self):
        module = load_launch_module()
        context = safe_launch_context(module)
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "stereo_distance_node_params.yaml"
            module._validate_px4_only_contract(context)
            module._write_distance_node_params(context, str(path))
            document = yaml.safe_load(path.read_text(encoding="utf-8"))

        parameters = document["/stereo_distance_node"]["ros__parameters"]
        self.assertEqual(parameters["flight_authority"], "px4_sitl")
        self.assertIs(type(parameters["flight_authority"]), str)
        self.assertIs(parameters["fly_target_in_gazebo"], False)
        self.assertEqual(parameters["mission_profile"], "manual")
        self.assertEqual(parameters["actual_distance_source"], "gazebo")

    def test_direct_launch_guard_rejects_unsafe_values(self):
        module = load_launch_module()
        context = safe_launch_context(module)
        context.launch_configurations["flight_authority"] = "gazebo"
        with self.assertRaisesRegex(RuntimeError, "flight_authority=.*gazebo"):
            module._validate_px4_only_contract(context)

    def test_adapter_constants_alone_never_verify_contract(self):
        payload = status_payload()
        self.assertIsNone(payload["flight_authority"])
        self.assertFalse(payload["flight_contract"]["verified"])
        self.assertEqual(px4_contract_state(payload), "unavailable")
        self.assertIn("UNVERIFIED", px4_overall_status(payload))

    def test_parameter_service_contract_requires_exact_values_and_freshness(self):
        valid = {
            "flight_authority": "px4_sitl",
            "fly_target_in_gazebo": False,
            "mission_profile": "manual",
            "actual_distance_source": "gazebo",
        }
        result = flight_contract_from_parameters(valid, checked_monotonic_s=10.0, now_monotonic_s=10.5)
        self.assertTrue(result["verified"])
        self.assertEqual(result["source"], FLIGHT_CONTRACT_SOURCE)
        self.assertEqual(set(FLIGHT_CONTRACT_PARAMETER_NAMES), set(valid))

        wrong = dict(valid, fly_target_in_gazebo=True)
        self.assertFalse(
            flight_contract_from_parameters(wrong, checked_monotonic_s=10.0, now_monotonic_s=10.5)["verified"]
        )
        self.assertFalse(
            flight_contract_from_parameters(valid, checked_monotonic_s=1.0, now_monotonic_s=10.5)["verified"]
        )
        self.assertFalse(
            flight_contract_from_parameters(None, checked_monotonic_s=None, now_monotonic_s=10.5)["verified"]
        )

    def test_ui_requires_parameter_source_and_live_verified_contract(self):
        contract = {
            "verified": True,
            "source": FLIGHT_CONTRACT_SOURCE,
            "flight_authority": "px4_sitl",
            "fly_target_in_gazebo": False,
            "mission_profile": "manual",
            "actual_distance_source": "gazebo",
            "checked_monotonic_s": 10.0,
            "error": None,
        }
        payload = status_payload(flight_contract=contract)
        self.assertEqual(px4_contract_state(payload), "safe")
        self.assertEqual(px4_contract_state(dict(payload, flight_contract=dict(contract, source="adapter_constant"))), "violation")
        self.assertEqual(px4_contract_state(dict(payload, flight_contract=dict(contract, fly_target_in_gazebo=0))), "violation")

    def test_mission_phase_provenance_never_promotes_offboard_to_authority(self):
        inferred = mission_phase_provenance(
            "APPROACH",
            connected=True,
            local_ned_m={"north": 10.0, "east": 0.0, "down": -10.0},
            velocity_ned_mps={"north": -2.0, "east": 0.0, "down": 0.0},
        )
        self.assertEqual(inferred, ("APPROACH", "inferred_mavlink_state", "medium"))
        self.assertEqual(mission_phase_display({"mission_phase": "APPROACH", "mission_phase_source": inferred[1]}), "APPROACH (INFERRED)")
        authoritative = mission_phase_provenance(
            "APPROACH",
            connected=True,
            local_ned_m=None,
            velocity_ned_mps=None,
            authoritative_phase="APPROACH",
        )
        self.assertEqual(authoritative, ("APPROACH", "authoritative_mission_event", "high"))
        self.assertEqual(mission_phase_display({"mission_phase": "APPROACH", "mission_phase_source": authoritative[1]}), "APPROACH")
        self.assertEqual(mission_phase_display({"mission_phase": "APPROACH", "mission_phase_source": "unknown"}), "UNAVAILABLE")

    def test_heartbeat_and_local_position_sources_expire_independently(self):
        self.assertEqual(
            px4_connection_snapshot(
                master_available=True,
                last_heartbeat_monotonic_s=10.0,
                now_monotonic_s=14.0,
            ),
            (False, "DISCONNECTED", "PX4 heartbeat stale"),
        )
        self.assertEqual(
            px4_connection_snapshot(
                master_available=True,
                last_heartbeat_monotonic_s=10.0,
                now_monotonic_s=11.0,
                connection_error="bind failed",
            ),
            (False, "ERROR", "bind failed"),
        )
        self.assertEqual(local_position_snapshot(last_receive_monotonic_s=None, now_monotonic_s=10.0), ("unavailable", False))
        self.assertEqual(local_position_snapshot(last_receive_monotonic_s=10.0, now_monotonic_s=14.0), ("stale", False))
        self.assertEqual(local_position_snapshot(last_receive_monotonic_s=10.0, now_monotonic_s=11.0), ("valid", True))

        payload = status_payload(
            connected=False,
            armed=False,
            navigation_mode="OFFBOARD",
            mission_phase="UNAVAILABLE",
            local_ned_m=None,
            velocity_ned_mps=None,
            local_position_status="stale",
            flight_contract=flight_contract_from_parameters(
                {"flight_authority": "px4_sitl", "fly_target_in_gazebo": False, "mission_profile": "manual", "actual_distance_source": "gazebo"},
                checked_monotonic_s=10.0,
                now_monotonic_s=10.1,
            ),
        )
        self.assertEqual(payload["navigation_mode"], "unavailable")
        self.assertEqual(payload["local_position_status"], "stale")
        self.assertFalse(payload["armed"])

    def test_status_schema_exposes_provenance_and_receive_sources(self):
        contract = flight_contract_from_parameters(
            {"flight_authority": "px4_sitl", "fly_target_in_gazebo": False, "mission_profile": "manual", "actual_distance_source": "gazebo"},
            checked_monotonic_s=10.0,
            now_monotonic_s=10.1,
        )
        payload = status_payload(
            flight_contract=contract,
            mission_phase="APPROACH",
            mission_phase_source="inferred_mavlink_state",
            mission_phase_confidence="medium",
            local_position_receive_monotonic_s=10.0,
            local_position_status="valid",
            connection_source="MAVLink udpout:127.0.0.1:18570",
            endpoint_coexistence="UNVERIFIED",
        )
        parsed = json.loads(json.dumps(payload, allow_nan=False))
        self.assertTrue(parsed["flight_contract"]["verified"])
        self.assertEqual(parsed["mission_phase_source"], "inferred_mavlink_state")
        self.assertEqual(parsed["mission_phase_confidence"], "medium")
        self.assertEqual(parsed["status_connection_source"], "MAVLink udpout:127.0.0.1:18570")
        self.assertEqual(parsed["endpoint_coexistence"], "UNVERIFIED")

    def test_auto_mission_gate_is_after_panel_and_recorder_start(self):
        text = RUN_OPERATOR_VIEW.read_text(encoding="utf-8")
        panel = text.index("ros2 run rqt_gui rqt_gui")
        recorder = text.index("-m stereo_ground_sim.operator_recorder")
        delay = text.index('sleep "$mission_delay_s"')
        gate = text.index('run_auto_mission.sh" --check-connection')
        mission = text.index('setsid "$ROOT_DIR/run_auto_mission.sh"')
        self.assertLess(panel, delay)
        self.assertLess(recorder, delay)
        self.assertLess(delay, gate)
        self.assertLess(gate, mission)
        self.assertIn("no arm or mission command will be sent", text)
        self.assertNotIn("PX4 connection gate: checking heartbeat", text[:panel])

    def test_default_mavlink_endpoint_does_not_bind_qgc_port(self):
        self.assertEqual(DEFAULT_CONNECT, "udpout:127.0.0.1:18570")
        self.assertNotIn("PX4_STATUS_CONNECT:-udpin:0.0.0.0:14550", RUN_OPERATOR_VIEW.read_text(encoding="utf-8"))

    def test_qt_panel_keeps_core_rows_visible_at_requested_resolutions(self):
        try:
            from python_qt_binding.QtCore import QObject
            from python_qt_binding.QtWidgets import QApplication, QGroupBox
            from stereo_ground_sim.operator_panel import OperatorPanel
        except Exception as exc:  # Qt/rqt is optional on static test hosts.
            self.skipTest(f"operator panel dependencies unavailable: {exc}")

        app = QApplication.instance() or QApplication([])

        class Context(QObject):
            def add_widget(self, widget):
                self.widget = widget

        panel = OperatorPanel(Context())
        try:
            position_box = next(
                box
                for box in panel._primary.findChildren(QGroupBox)
                if box.title().startswith("POSITION — baseline_center_frd")
            )
            for width, height in ((1700, 1017), (1920, 1080)):
                panel._widget.resize(width, height)
                panel._widget.show()
                app.processEvents()
                viewport = panel._primary_scroll.viewport()
                self.assertGreaterEqual(position_box.width(), position_box.sizeHint().width())
                self.assertGreaterEqual(position_box.geometry().top(), 0)
                self.assertLessEqual(position_box.geometry().bottom(), viewport.height())
                self.assertTrue(panel._primary_scroll.horizontalScrollBar().maximum() == 0)
                self.assertTrue(all(label.text() for label in panel._top_status_labels.values()))

            panel._widget.resize(1700, 1017)
            app.processEvents()
            self.assertTrue(panel._primary_scroll.verticalScrollBar().isVisible())

            panel._widget.resize(1920, 1080)
            app.processEvents()
            self.assertFalse(panel._primary_scroll.horizontalScrollBar().isVisible())
        finally:
            panel.shutdown_plugin()
            try:
                import rclpy

                if rclpy.ok():
                    rclpy.shutdown()
            except Exception:
                pass


if __name__ == "__main__":
    unittest.main()
