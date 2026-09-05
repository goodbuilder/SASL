from __future__ import annotations

from types import SimpleNamespace

from Workstation.workstation_core.mag_selection import evaluate_mag_selection_verification


def test_mag_selection_requires_post_apply_active_sample_and_attitude() -> None:
    pending = evaluate_mag_selection_verification(
        expected_device_id=10,
        actual_device_id=10,
        active_sample_timestamp_s=99.0,
        attitude_timestamp_s=99.9,
        started_s=100.0,
        now_s=100.2,
    )
    assert pending.state == "VERIFYING"
    assert not pending.verified

    verified = evaluate_mag_selection_verification(
        expected_device_id=10,
        actual_device_id=10,
        active_sample_timestamp_s=100.1,
        attitude_timestamp_s=100.15,
        started_s=100.0,
        now_s=100.2,
    )
    assert verified.state == "ACTIVE VERIFIED"
    assert verified.verified


def test_mag_selection_keeps_side_mismatch_unverified_then_times_out() -> None:
    pending = evaluate_mag_selection_verification(
        expected_device_id=10,
        actual_device_id=20,
        active_sample_timestamp_s=100.1,
        attitude_timestamp_s=100.15,
        started_s=100.0,
        now_s=100.2,
    )
    assert pending.state == "VERIFYING"
    assert "expected device 10" in pending.reason

    timeout = evaluate_mag_selection_verification(
        expected_device_id=10,
        actual_device_id=20,
        active_sample_timestamp_s=100.1,
        attitude_timestamp_s=100.15,
        started_s=100.0,
        now_s=108.1,
    )
    assert timeout.state == "TIMEOUT"
    assert not timeout.verified


def test_mag_selection_rejects_stale_attitude_even_when_active_id_matches() -> None:
    result = evaluate_mag_selection_verification(
        expected_device_id=10,
        actual_device_id=10,
        active_sample_timestamp_s=100.1,
        attitude_timestamp_s=97.0,
        started_s=100.0,
        now_s=100.2,
    )
    assert result.state == "VERIFYING"

    stale = evaluate_mag_selection_verification(
        expected_device_id=10,
        actual_device_id=10,
        active_sample_timestamp_s=100.1,
        attitude_timestamp_s=100.0,
        started_s=100.0,
        now_s=102.2,
    )
    assert stale.state == "STALE"


def test_panel_measurement_gate_requires_both_live_pixhawk_sides() -> None:
    from Workstation.standalone_gui import WirisStereoDistancePanel

    panel = SimpleNamespace()
    panel.latest_vehicle_mag_device_ids = {"left": 10, "right": 20}
    panel.latest_vehicle_mag_device_id_timestamps = {"left": 100.1, "right": 100.1}
    panel.latest_extrinsics = {
        "left": SimpleNamespace(timestamp_s=100.15),
        "right": SimpleNamespace(timestamp_s=100.15),
    }
    panel._px4_mag_status_cache = {}
    panel._px4_mag_verification_targets = {
        "left": {"expected_device_id": 10, "started_s": 100.0},
        "right": {"expected_device_id": 20, "started_s": 100.0},
    }
    panel._measurement_view_controller = lambda: SimpleNamespace(
        workstation_measurement_methods_snapshot=lambda: {
            "selected_method_id": "pixhawk_rtk_live",
            "method_catalog": [
                {"method_id": "pixhawk_rtk_live", "requires_live_pixhawk": True}
            ],
        }
    )
    panel._mavlink_open_sides = lambda: ("left", "right")
    panel._log = lambda _message: None
    panel.workstation_px4_mag_measurement_gate = lambda required_sides: {
        "ready": all(
            panel.latest_vehicle_mag_device_ids[side]
            == panel._px4_mag_verification_targets[side]["expected_device_id"]
            for side in required_sides
        ),
        "required": True,
        "required_sides": list(required_sides),
        "blockers": [],
        "sides": {},
    }

    assert WirisStereoDistancePanel.measurement_mag_start_gate(panel) is True

    panel.latest_vehicle_mag_device_ids["right"] = 99
    assert WirisStereoDistancePanel.measurement_mag_start_gate(panel) is False
