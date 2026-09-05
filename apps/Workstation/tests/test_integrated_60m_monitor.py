from __future__ import annotations

from Workstation.tools.qualify_integrated_60m import (
    MIN_FREE_BYTES,
    qualification_failures,
)
from Workstation.workstation_core.mavlink_liveness import MAVLINK_QUALIFICATION_S


def _rows(*, pixhawk_state: str = "QUALIFYING") -> dict[str, dict[str, str]]:
    return {
        "camera.left": {"state": "LIVE", "detail": "EXPERIMENTAL NON-IDR INTRA"},
        "camera.right": {"state": "LIVE", "detail": "EXPERIMENTAL NON-IDR INTRA"},
        "pixhawk.left": {"state": pixhawk_state, "detail": "HB=0.1 ATT=0.1"},
        "pixhawk.right": {"state": pixhawk_state, "detail": "HB=0.1 ATT=0.1"},
        "gnss.ntrip": {
            "state": "STREAMING",
            "detail": "connected=yes; RTCM age=0.2 s",
        },
        "recording": {"state": "ACTIVE", "detail": "storage=OK; dropped=0"},
    }


def test_preflight_accepts_fresh_qualifying_pixhawks_before_one_minute() -> None:
    assert qualification_failures(
        _rows(),
        elapsed_s=MAVLINK_QUALIFICATION_S - 0.001,
        phase="preflight",
        free_bytes=MIN_FREE_BYTES,
    ) == []


def test_soak_requires_live_pixhawks_at_ten_seconds() -> None:
    failures = qualification_failures(
        _rows(),
        elapsed_s=MAVLINK_QUALIFICATION_S,
        phase="soak",
        free_bytes=MIN_FREE_BYTES,
    )

    assert failures == [
        "pixhawk not LIVE after 10s states=['QUALIFYING', 'QUALIFYING']"
    ]


def test_camera_pixhawk_baseline_requires_ntrip_disconnected() -> None:
    rows = _rows(pixhawk_state="LIVE")
    rows["gnss.ntrip"] = {
        "state": "DISCONNECTED",
        "detail": "connected=no; RTCM age=--",
    }

    assert qualification_failures(
        rows,
        elapsed_s=MAVLINK_QUALIFICATION_S,
        phase="camera_pixhawk_baseline",
        free_bytes=MIN_FREE_BYTES,
    ) == []

    qualifying = _rows(pixhawk_state="QUALIFYING")
    qualifying["gnss.ntrip"] = rows["gnss.ntrip"]
    assert qualification_failures(
        qualifying,
        elapsed_s=MAVLINK_QUALIFICATION_S - 0.001,
        phase="camera_pixhawk_baseline",
        free_bytes=MIN_FREE_BYTES,
    ) == []
    assert qualification_failures(
        qualifying,
        elapsed_s=MAVLINK_QUALIFICATION_S,
        phase="camera_pixhawk_baseline",
        free_bytes=MIN_FREE_BYTES,
    ) == [
        "pixhawk not LIVE after 10s states=['QUALIFYING', 'QUALIFYING']"
    ]

    rows["gnss.ntrip"] = {
        "state": "STREAMING",
        "detail": "connected=yes; RTCM age=0.2 s",
    }
    assert qualification_failures(
        rows,
        elapsed_s=MAVLINK_QUALIFICATION_S,
        phase="camera_pixhawk_baseline",
        free_bytes=MIN_FREE_BYTES,
    ) == ["baseline NTRIP must remain disconnected: STREAMING"]


def test_ntrip_comparison_requires_live_eth_mav_after_one_minute() -> None:
    assert qualification_failures(
        _rows(pixhawk_state="LIVE"),
        elapsed_s=MAVLINK_QUALIFICATION_S,
        phase="camera_pixhawk_ntrip",
        free_bytes=MIN_FREE_BYTES,
    ) == []

    failures = qualification_failures(
        _rows(pixhawk_state="QUALIFYING"),
        elapsed_s=MAVLINK_QUALIFICATION_S - 0.001,
        phase="camera_pixhawk_ntrip",
        free_bytes=MIN_FREE_BYTES,
    )
    assert failures == []

    failures = qualification_failures(
        _rows(pixhawk_state="QUALIFYING"),
        elapsed_s=MAVLINK_QUALIFICATION_S,
        phase="camera_pixhawk_ntrip",
        free_bytes=MIN_FREE_BYTES,
    )
    assert failures == [
        "pixhawk not LIVE after 10s states=['QUALIFYING', 'QUALIFYING']"
    ]


def test_oracle_fails_closed_on_camera_ntrip_recording_and_disk() -> None:
    rows = _rows(pixhawk_state="LIVE")
    rows["camera.right"]["state"] = "STALE"
    rows["gnss.ntrip"] = {
        "state": "STALE",
        "detail": "connected=no; RTCM age=--",
    }
    rows["recording"] = {"state": "ERROR", "detail": "storage=FAILED"}

    failures = qualification_failures(
        rows,
        elapsed_s=900.0,
        phase="preflight",
        free_bytes=MIN_FREE_BYTES - 1,
    )

    assert "camera preflight states=['LIVE', 'STALE']" in failures
    assert "ntrip state=STALE" in failures
    assert "ntrip lacks current RTCM/connection evidence" in failures
    assert "recording state=ERROR" in failures
    assert "recording storage is not OK" in failures
    assert any(reason.startswith("disk free below 100GiB") for reason in failures)
