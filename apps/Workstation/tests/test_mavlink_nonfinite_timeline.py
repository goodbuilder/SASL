from __future__ import annotations

import json
from collections import deque
from pathlib import Path
from types import SimpleNamespace

import pytest

from Workstation.recording_controller import RecordingController
from Workstation.workstation_core.measurement_data_store import (
    LiveDataRows,
    MeasurementDataStore,
)
from Workstation.workstation_core.pixhawk_attitude import (
    _coerce_numeric,
    _mavlink_hdg_acc_deg_e5_to_rad,
    _mavlink_heading_cdeg,
    _mavlink_optional_timeline_row,
    _mavlink_scaled_accuracy_mm,
    _mavlink_scaled_centimetres,
    _mavlink_scaled_degrees,
    _mavlink_scaled_dop,
    _mavlink_scaled_millimetres,
    _mavlink_yaw_cdeg_to_rad,
    _optional_float,
    _optional_int,
    _scaled_mavlink_array_value,
    _scaled_mavlink_value,
)


def _reject_json_constant(value: str) -> None:
    raise AssertionError(f"non-finite JSON constant persisted: {value}")


def test_vfr_hud_and_local_position_nonfinite_fields_are_unavailable_with_evidence() -> None:
    vfr_row = _mavlink_optional_timeline_row(
        "VFR_HUD",
        SimpleNamespace(
            airspeed=float("nan"),
            groundspeed=float("inf"),
            heading=90.0,
            throttle=55.0,
            alt=float("-inf"),
            climb=1.25,
        ),
        10.0,
    )

    assert vfr_row is not None
    assert vfr_row["mavlink_vfr_airspeed_m_s"] is None
    assert vfr_row["mavlink_vfr_groundspeed_m_s"] is None
    assert vfr_row["mavlink_vfr_altitude_m"] is None
    assert vfr_row["mavlink_vfr_climb_m_s"] == 1.25
    assert vfr_row["nonfinite_source_fields"] == [
        "VFR_HUD.airspeed",
        "VFR_HUD.alt",
        "VFR_HUD.groundspeed",
    ]
    assert vfr_row["nonfinite_source_reasons"] == {
        "VFR_HUD.airspeed": "nan",
        "VFR_HUD.alt": "negative_infinity",
        "VFR_HUD.groundspeed": "positive_infinity",
    }

    local_row = _mavlink_optional_timeline_row(
        "LOCAL_POSITION_NED",
        SimpleNamespace(
            x=float("nan"),
            y=2.0,
            z=3.0,
            vx=4.0,
            vy=float("inf"),
            vz=6.0,
            time_boot_ms=123,
        ),
        11.0,
    )

    assert local_row is not None
    assert local_row["mavlink_local_x_m"] is None
    assert local_row["mavlink_local_velocity_y_m_s"] is None
    assert local_row["nonfinite_source_reasons"] == {
        "LOCAL_POSITION_NED.x": "nan",
        "LOCAL_POSITION_NED.vy": "positive_infinity",
    }


@pytest.mark.parametrize("value", [float("nan"), float("inf"), float("-inf")])
def test_all_optional_scaled_and_custom_numeric_boundaries_reject_nonfinite_values(
    value: float,
) -> None:
    message = SimpleNamespace(value=value, values=[value])
    assert _optional_float(message, "value") is None
    assert _optional_int(message, "value") is None
    assert _scaled_mavlink_value(message, "value", 0.01) is None
    assert _scaled_mavlink_array_value(message, "values", 0, 0.001) is None
    for converter in (
        _mavlink_scaled_degrees,
        _mavlink_scaled_millimetres,
        _mavlink_scaled_centimetres,
        _mavlink_scaled_dop,
        _mavlink_scaled_accuracy_mm,
        _mavlink_heading_cdeg,
        _mavlink_yaw_cdeg_to_rad,
        _mavlink_hdg_acc_deg_e5_to_rad,
    ):
        assert converter(value) is None
    assert _coerce_numeric(value) is None


def test_custom_numeric_boundary_preserves_finite_values() -> None:
    assert _coerce_numeric(4.5) == 4.5


def test_external_timeline_normalizes_nested_nonfinite_values_and_persists_strict_json(
    monkeypatch,
    tmp_path: Path,
) -> None:
    store = MeasurementDataStore(tmp_path, fsync_interval_s=0.05)
    rows = LiveDataRows(store, "px4.left")
    panel = SimpleNamespace(
        distance_recording_enabled=True,
        px4_recording_rows={"left": rows},
        external_parameter_recording_rows=deque(maxlen=16),
        recording_session_started_monotonic_ns=1_000_000_000,
        recording_session_started_unix_ns=1_800_000_000_000_000_000,
        recording_session_anchor_uncertainty_ns=10,
        recording_session_id="nonfinite-session",
        _measurement_id="m000001",
        mavlink_pixhawk=None,
        _recording_side=lambda camera: camera if camera in {"left", "right"} else None,
        _session_utc_ns_for_monotonic_ns=lambda monotonic_ns: (
            1_800_000_000_000_000_000 + monotonic_ns - 1_000_000_000
        ),
        _fmt_record_float=lambda value, precision: (
            "" if value is None else f"{float(value):.{precision}f}"
        ),
        _fmt_record_bool=lambda value: "true" if value else "false",
        _triangulation_extrinsics_source=lambda: "live_px4",
        _distance_mode_label=lambda: "test",
        _refresh_recording_status_label=lambda: None,
    )
    controller = RecordingController(panel)
    panel._raw_timeline_row = controller._raw_timeline_row
    monkeypatch.setattr(
        "Workstation.recording_controller.time.monotonic_ns",
        lambda: 5_000_000_000,
    )
    monkeypatch.setattr(
        "Workstation.recording_controller.time.time_ns",
        lambda: 1_800_000_004_000_000_000,
    )

    controller._record_external_timeline_row(
        "left",
        "mavlink_vfr_hud",
        {
            "sample_app_timestamp_s": float("nan"),
            "mavlink_vfr_airspeed_m_s": float("nan"),
            "nested": {
                "groundspeed": float("inf"),
                "vector": [1.0, float("-inf")],
            },
            "nonfinite_source_fields": ["VFR_HUD.airspeed"],
            "nonfinite_source_reasons": {"VFR_HUD.airspeed": "nan"},
        },
    )

    row = rows[0]
    assert row["sample_monotonic_ns"] == 5_000_000_000
    assert row["sample_timestamp_source"] == "host_receive_monotonic_ns"
    assert row["sample_timestamp_fallback_reason"] == (
        "sample_app_timestamp_s is non-finite: nan"
    )
    assert row["sample_app_timestamp_s"] is None
    assert row["mavlink_vfr_airspeed_m_s"] is None
    assert row["nested"] == {"groundspeed": None, "vector": [1.0, None]}
    assert row["nonfinite_source_fields"] == [
        "VFR_HUD.airspeed",
        "mavlink_vfr_airspeed_m_s",
        "nested.groundspeed",
        "nested.vector[1]",
        "sample_app_timestamp_s",
    ]
    assert row["nonfinite_source_reasons"] == {
        "VFR_HUD.airspeed": "nan",
        "mavlink_vfr_airspeed_m_s": "nan",
        "nested.groundspeed": "positive_infinity",
        "nested.vector[1]": "negative_infinity",
        "sample_app_timestamp_s": "nan",
    }
    [external_row] = list(panel.external_parameter_recording_rows)
    assert external_row["mavlink_vfr_airspeed_m_s"] is None
    assert external_row["nested"] == {"groundspeed": None, "vector": [1.0, None]}
    json.dumps(external_row, allow_nan=False)

    assert store.close(timeout_s=2.0)
    persisted = json.loads(
        (tmp_path / "px4_left.jsonl").read_text(),
        parse_constant=_reject_json_constant,
    )
    assert persisted["sample_app_timestamp_s"] is None
    assert persisted["nested"] == {"groundspeed": None, "vector": [1.0, None]}
