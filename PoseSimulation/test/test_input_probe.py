from __future__ import annotations

import unittest
from pathlib import Path
from types import SimpleNamespace

from pose_simulation.input_adapters import adapt_camera_pose_diagnostic, adapt_ground_truth_pose
from pose_simulation.input_probe_node import (
    ProbeSnapshot,
    format_probe_report,
    publisher_qos_compatible,
)
from pose_simulation.input_types import SOURCE_UNAVAILABLE
from pose_simulation.time_alignment import align_static_snapshot

from test_input_adapters import _camera_payload, _gt_message


def _gt(stamp_ns: int):
    return adapt_ground_truth_pose(
        _gt_message(stamp_ns=stamp_ns),
        source_topic="/simulation/ground_truth/vehicle_reference_link",
        clock_domain="gazebo_sim_time",
    )


def _camera(side: str, stamp_ns: int, *, x: float | None = None):
    if x is None:
        x = -12.5 if side == "left" else 12.5
    payload = _camera_payload(side=side, stamp_ns=stamp_ns)
    payload["observed_optical_position_world_enu_m"] = [x, 0.0, 10.0]
    return adapt_camera_pose_diagnostic(
        payload,
        expected_side=side,
        source_topic=f"/simulation/camera_pose_diagnostics/{side}",
        expected_entity_name=f"{side}_stereo_camera::optical_link",
        clock_domain="gazebo_sim_time",
    )


class InputProbeReportTests(unittest.TestCase):
    def test_known_qos_mismatch_is_detected_without_requiring_ros(self):
        incompatible = SimpleNamespace(
            qos_profile=SimpleNamespace(reliability="BEST_EFFORT", durability="VOLATILE")
        )
        self.assertFalse(
            publisher_qos_compatible(
                incompatible,
                expected_reliability="RELIABLE",
                expected_durability="TRANSIENT_LOCAL",
            )
        )
        compatible = SimpleNamespace(
            qos_profile=SimpleNamespace(
                reliability="RELIABLE", durability="TRANSIENT_LOCAL"
            )
        )
        self.assertTrue(
            publisher_qos_compatible(
                compatible,
                expected_reliability="RELIABLE",
                expected_durability="TRANSIENT_LOCAL",
            )
        )

    def test_probe_source_has_no_publisher_or_pose_command_path(self):
        source = Path(__file__).resolve().parents[1] / "pose_simulation" / "input_probe_node.py"
        text = source.read_text(encoding="utf-8")
        self.assertNotIn("create_publisher", text)
        self.assertNotIn("/set_pose", text)

    def test_report_contains_source_contract_and_alignment_identity(self):
        alignment = align_static_snapshot(_gt(10), _camera("left", 1), _camera("right", 1, x=12.5))
        report = format_probe_report(
            ProbeSnapshot(
                gt_topic="/simulation/ground_truth/vehicle_reference_link",
                left_camera_topic="/simulation/camera_pose_diagnostics/left",
                right_camera_topic="/simulation/camera_pose_diagnostics/right",
                camera_source_mode="static_snapshot",
                expected_clock_domain="gazebo_sim_time",
                gt_topic_discovered=True,
                left_topic_discovered=True,
                right_topic_discovered=True,
                ground_truth=_gt(10),
                left_camera=_camera("left", 1),
                right_camera=_camera("right", 1, x=12.5),
                alignment=alignment,
            )
        )
        self.assertIn("GT topic discovered: YES", report)
        self.assertIn("Camera source mode: static_snapshot", report)
        self.assertIn("Clock domain: gazebo_sim_time", report)
        self.assertIn("Evaluation ID: gt:10", report)
        self.assertIn("Overall input status: VALID", report)
        self.assertIn("POSE COMMAND: DISABLED", report)

    def test_report_marks_unavailable_sources_and_qos_issue(self):
        alignment = align_static_snapshot(None, None, None)
        report = format_probe_report(
            ProbeSnapshot(
                gt_topic="/simulation/ground_truth/vehicle_reference_link",
                left_camera_topic="/simulation/camera_pose_diagnostics/left",
                right_camera_topic="/simulation/camera_pose_diagnostics/right",
                camera_source_mode="static_snapshot",
                expected_clock_domain=None,
                gt_topic_discovered=False,
                left_topic_discovered=False,
                right_topic_discovered=False,
                ground_truth=None,
                left_camera=None,
                right_camera=None,
                alignment=alignment,
                qos_issues=("left topic publishers do not match RELIABLE/TRANSIENT_LOCAL QoS",),
            )
        )
        self.assertIn("Clock domain: UNRESOLVED", report)
        self.assertIn("Overall input status: QOS_INCOMPATIBLE", report)
        self.assertIn("QoS issues:", report)
        self.assertNotIn(SOURCE_UNAVAILABLE, report.split("Overall input status:", 1)[-1])


if __name__ == "__main__":
    unittest.main()
