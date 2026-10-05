from __future__ import annotations

from types import SimpleNamespace

import pytest

from Workstation.workstation_core.yolo_stream import (
    GstRtpTimingTracker,
    RtcpSenderReportTiming,
)
from Workstation.workstation_core.rtcp_clock_observer import RtcpClockObserverWorker


def _timing(
    index: int,
    *,
    ssrc: int = 101,
    offset_s: float = -0.2,
) -> RtcpSenderReportTiming:
    received_mono_s = 1000.0 + index * 5.0
    received_wall_s = 2000.0 + index * 5.0
    return RtcpSenderReportTiming(
        ssrc=ssrc,
        ntp_raw=10_000 + index,
        ntp_unix_s=received_wall_s + offset_s,
        rtp_timestamp=90_000 * index * 5,
        clock_rate_hz=90_000,
        received_monotonic_s=received_mono_s,
        received_wall_s=received_wall_s,
        arrival_basis="rtcp-pad-probe",
    )


def test_sender_report_observation_is_available_without_a_decoded_frame() -> None:
    tracker = GstRtpTimingTracker(SimpleNamespace(), SimpleNamespace())
    for index in range(1, 4):
        tracker._register_sender_report(_timing(index))

    observations = tracker.sender_report_observations(now_s=1015.25)

    assert len(observations) == 3
    assert [row["ntp_raw"] for row in observations] == [10_001, 10_002, 10_003]
    assert observations[-1]["received_count"] == 3
    assert observations[-1]["age_s"] == pytest.approx(0.25)
    assert observations[-1]["sr_pc_offset_s"] == pytest.approx(-0.2)
    assert observations[-1]["sr_pc_offset_raw_s"] == pytest.approx(-0.2)
    assert observations[-1]["arrival_basis"] == "rtcp-pad-probe"
    assert observations[-1]["ntp_rate"] == pytest.approx(1.0)
    assert observations[-1]["media_rate"] == pytest.approx(1.0)
    assert observations[-1]["host_clock_step_s"] == pytest.approx(0.0)


def test_duplicate_sender_report_does_not_create_a_new_observation() -> None:
    tracker = GstRtpTimingTracker(SimpleNamespace(), SimpleNamespace())
    sample = _timing(1)

    tracker._register_sender_report(sample)
    tracker._register_sender_report(sample)

    observations = tracker.sender_report_observations(now_s=1005.1)
    assert len(observations) == 1
    assert observations[0]["received_count"] == 1


def test_observation_history_is_bounded_per_ssrc() -> None:
    tracker = GstRtpTimingTracker(SimpleNamespace(), SimpleNamespace())
    for index in range(1, 20):
        tracker._register_sender_report(_timing(index))

    observations = tracker.sender_report_observations(now_s=1100.0)
    assert len(observations) == 8
    assert observations[0]["ntp_raw"] == 10_012
    assert observations[-1]["ntp_raw"] == 10_019


class _FakeProbe:
    def __init__(self, side: str, *, fail_start: bool = False) -> None:
        self.side = side
        self.fail_start = fail_start
        self.started = False
        self.stopped = False

    def start(self) -> None:
        if self.fail_start:
            raise RuntimeError(f"{self.side} unavailable")
        self.started = True

    def poll(self) -> list[dict[str, object]]:
        return [
            {
                "ssrc": 101 if self.side == "left" else 202,
                "ntp_raw": 1001,
                "ntp_unix_s": 2000.0,
                "rtp_timestamp": 90_000,
                "clock_rate_hz": 90_000,
                "received_monotonic_s": 1000.0,
                "received_wall_s": 2000.2,
                "received_count": 1,
                "age_s": 0.2,
                "arrival_basis": "rtcp-pad-probe",
                "sr_pc_offset_s": -0.2,
                "ntp_rate": 1.0,
                "media_rate": 1.0,
            }
        ]

    def stop(self) -> None:
        self.stopped = True


def test_worker_emits_bounded_side_observations_without_video_decode() -> None:
    probes: dict[str, _FakeProbe] = {}

    def factory(side: str, **_kwargs: object) -> _FakeProbe:
        probe = _FakeProbe(side)
        probes[side] = probe
        return probe

    worker = RtcpClockObserverWorker(
        {
            "left": {"url": "rtsp://left/visible"},
            "right": {"url": "rtsp://right/visible"},
        },
        observer_generation=9,
        probe_factory=factory,
        max_poll_cycles=1,
        poll_interval_s=0.001,
    )
    emitted: list[dict[str, object]] = []
    worker.observation_ready.connect(emitted.append)

    worker.run()

    assert {row["side"] for row in emitted} == {"left", "right"}
    assert all(row["observer_generation"] == 9 for row in emitted)
    assert all(row["schema"] == "workstation-rtcp-clock-observation-v1" for row in emitted)
    assert all(probe.started and probe.stopped for probe in probes.values())


def test_worker_preflight_failure_stops_every_started_probe() -> None:
    probes: dict[str, _FakeProbe] = {}

    def factory(side: str, **_kwargs: object) -> _FakeProbe:
        probe = _FakeProbe(side, fail_start=side == "right")
        probes[side] = probe
        return probe

    worker = RtcpClockObserverWorker(
        {
            "left": {"url": "rtsp://left/visible"},
            "right": {"url": "rtsp://right/visible"},
        },
        observer_generation=1,
        probe_factory=factory,
        max_poll_cycles=1,
    )
    failures: list[str] = []
    worker.failed.connect(failures.append)

    worker.run()

    assert failures and "right unavailable" in failures[0]
    assert probes["left"].stopped is True
