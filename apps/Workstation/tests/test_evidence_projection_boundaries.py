"""Evidence projection inputs stay detached from later source updates."""
from __future__ import annotations

from pathlib import Path
import subprocess
import sys
import textwrap
from types import SimpleNamespace

import pytest

from Workstation.evidence_views.snapshot import snapshot_attributes, snapshot_mapping
from Workstation.evidence_views.telemetry import (
    ATTITUDE_FIELDS, DETECTION_FIELDS, GPS_FIELDS,
    project_attitude, project_detection, project_gps,
)


def test_attitude_projection_uses_captured_wxyz_and_receive_clock():
    source = SimpleNamespace(q=[1.0, 0.0, 0.0, 0.0], roll_deg=11.0, pitch_deg=-23.0,
                             yaw_deg=37.0, timestamp_s=10.0, px4_timestamp_us=1234)
    captured = snapshot_attributes(source, ATTITUDE_FIELDS)
    source.q[0] = 0.0
    source.yaw_deg = 90.0
    source.timestamp_s = 99.0
    with pytest.raises(TypeError):
        captured["yaw_deg"] = 45.0
    result = project_attitude("left", state=captured, now=12.5, provenance={})
    assert result["q_wxyz"] == (1.0, 0.0, 0.0, 0.0)
    assert result["yaw_deg"] == 37.0
    assert result["age_s"] == 2.5
    assert result["px4_timestamp_us"] == 1234
    assert result["orientation_contract"] == "orientation: px4_frd → ned; roll/pitch/yaw are degrees"


def test_gps_projection_preserves_height_datums_and_frozen_source_evidence():
    source = SimpleNamespace(latitude_deg=35.1234, longitude_deg=129.5678,
                             altitude_m=11.0, altitude_ellipsoid_m=42.0,
                             timestamp_s=10.0, fix_type=6, rtk_label="RTK_FIXED",
                             gps_utc_us=1_700_000_000_123_456)
    frozen = {"packets": 7, "utc_frozen": True}
    captured = snapshot_attributes(source, GPS_FIELDS)
    frozen_input = snapshot_mapping(frozen)
    frozen["packets"] = 0
    source.altitude_m = 999.0
    result = project_gps("right", state=captured, now=12.5, source="GPS2", provenance={}, frozen=frozen_input)
    assert result["receiver_source"] == "GPS2"
    assert result["altitude_m"] == 11.0
    assert result["altitude_ellipsoid_m"] == 42.0
    assert result["position_frozen_packets"] == 7
    assert result["gps_utc_us"] == 1_700_000_000_123_456
    assert result["age_s"] == 2.5
    assert project_gps("right", state=None, now=12.5, source="GPS2", provenance={}, frozen=None)["state"] == "NO DATA"


def test_detection_projection_keeps_pixel_and_source_clock_contracts():
    source = SimpleNamespace(timestamp_s=10.0, width=640, height=480,
                             x1=1.0, y1=3.0, x2=21.0, y2=43.0, center=[11.0, 23.0],
                             candidate_class_ids=[0, 1], class_id=0, class_name="fixture", confidence=0.8)
    captured = snapshot_attributes(source, DETECTION_FIELDS)
    source.center[0] = 999.0
    source.candidate_class_ids.append(2)
    result = project_detection(
        "left", observation=captured, stats={"decoded_hz": 30.0}, enabled=True, now=12.5,
        metadata={"source_timestamp_s": 1000.25, "timestamp_source": "camera_ntp"},
    )
    assert (result["center_u_px"], result["center_v_px"]) == (11.0, 23.0)
    assert result["candidate_count"] == 2
    assert result["source_timestamp_s"] == 1000.25
    assert result["age_s"] == 2.5
    assert result["width_px"] == 640
    assert project_detection("left", observation=None, stats={}, metadata={}, enabled=False, now=12.5)["runtime_state"] == "NO TARGET"


def test_projectors_import_and_execute_without_panel_gui_or_runtime_clock():
    apps = Path(__file__).resolve().parents[2]
    script = textwrap.dedent("""
        import importlib.abc, sys
        from pathlib import Path
        apps = Path(sys.argv[1])
        sys.path[:0] = [str(apps), str(apps.parent / 'common/python')]
        class Guard(importlib.abc.MetaPathFinder):
            def find_spec(self, fullname, path=None, target=None):
                if fullname == 'PyQt5' or fullname == 'Workstation.evidence' or fullname == 'Workstation.standalone_gui':
                    raise AssertionError('projector imported live UI: ' + fullname)
                return None
        sys.meta_path.insert(0, Guard())
        from Workstation.evidence_views.telemetry import project_attitude
        value = project_attitude('left', state={'timestamp_s': 10.0}, now=12.5, provenance={})
        assert value['age_s'] == 2.5
        print('pure evidence projection')
    """)
    result = subprocess.run(
        [sys.executable, "-I", "-B", "-c", script, str(apps)],
        text=True, capture_output=True, check=True, timeout=30,
    )
    assert result.stdout.strip() == "pure evidence projection"
