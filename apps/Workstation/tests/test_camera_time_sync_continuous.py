"""Focused STUS ordering and sustained RTCP/NTP health regressions."""

from __future__ import annotations

from types import SimpleNamespace
from collections import deque
import threading
import time

import pytest

from PyQt5 import QtWidgets


def test_current_timing_ages_advance_when_both_camera_metadata_freezes() -> None:
    from Workstation.camera_time_sync_controller import (
        current_frame_observation_age_s,
        current_sender_report_age_s,
    )

    metadata = {
        "timestamp_s": 100.0,
        "rtcp_sr_age_s": 0.25,
        "rtcp_sr_received_wall_s": 999.75,
    }

    assert current_frame_observation_age_s(metadata, now_monotonic_s=108.0) == pytest.approx(8.0)
    assert current_sender_report_age_s(
        metadata,
        now_monotonic_s=108.0,
        now_wall_s=1008.0,
    ) == pytest.approx(8.25)


def test_stus_field_latency_hiws_probe_remains_usable_and_bounded(monkeypatch) -> None:
    """The captured 0.5--0.9 s HIWS path must not silently become 0 ms."""

    from Workstation import wiris_runtime_workers

    perf_counter_values = iter((0.0, 0.934, 1.0, 1.510, 2.0, 2.515))
    monkeypatch.setattr(wiris_runtime_workers.time, "perf_counter", lambda: next(perf_counter_values))
    probe_timeouts: list[float] = []
    monkeypatch.setattr(
        wiris_runtime_workers,
        "send_tcp_line",
        lambda command: probe_timeouts.append(float(command.timeout_s)) or "OK",
    )
    worker = wiris_runtime_workers.WirisClockCommandWorker(
        (("left", "10.0.0.231", 2240, 5.0),),
        command_text="STUS",
    )

    probe = worker._measure_control_rtt(host="10.0.0.231", port=2240, timeout_s=5.0)

    assert probe["samples_s"] == pytest.approx([0.934, 0.510])
    assert probe["median_s"] == pytest.approx(0.722)
    assert probe["warnings"]
    assert probe_timeouts == pytest.approx([5.0, 5.0])
    assert worker._stus_compensation_s(probe["median_s"]) == pytest.approx(0.363)


def test_stus_hiws_probe_retries_empty_responses_and_accepts_measured_three_second_rtt(
    monkeypatch,
) -> None:
    from Workstation import wiris_runtime_workers

    perf_counter_values = iter((0.0, 5.0, 10.0, 13.18, 20.0, 25.0, 30.0, 31.8))
    responses = iter(("", "OK", "", "OK"))
    timeouts: list[float] = []
    monkeypatch.setattr(
        wiris_runtime_workers.time,
        "perf_counter",
        lambda: next(perf_counter_values),
    )
    monkeypatch.setattr(
        wiris_runtime_workers,
        "send_tcp_line",
        lambda command: timeouts.append(float(command.timeout_s)) or next(responses),
    )
    worker = wiris_runtime_workers.WirisClockCommandWorker(
        (("left", "10.0.0.231", 2240, 5.0),),
        command_text="STUS",
    )

    probe = worker._measure_control_rtt(
        host="10.0.0.231",
        port=2240,
        timeout_s=5.0,
    )

    assert probe["samples_s"] == pytest.approx([3.18, 1.8])
    assert probe["median_s"] == pytest.approx(2.49)
    assert len(timeouts) == 4
    assert worker._stus_compensation_s(probe["median_s"]) == pytest.approx(1.247)
    assert worker._stus_compensation_s(3.9) < wiris_runtime_workers.WIRIS_STUS_MAX_ABSOLUTE_COMPENSATION_S


def test_stus_hiws_preflight_observes_left_and_right_in_parallel(monkeypatch) -> None:
    from Workstation import wiris_runtime_workers

    barrier = threading.Barrier(2)
    observed: list[str] = []

    def fake_probe(self, *, host: str, **_kwargs):
        observed.append(host)
        barrier.wait(timeout=1.0)
        return {
            "command": "HIWS",
            "samples_s": [0.8, 0.9],
            "median_s": 0.85,
            "errors": [],
            "warnings": ["high latency"],
        }

    monkeypatch.setattr(
        wiris_runtime_workers.WirisClockCommandWorker,
        "_measure_control_rtt",
        fake_probe,
    )
    monkeypatch.setattr(wiris_runtime_workers, "send_tcp_line", lambda _command: "OK")
    completed: list[object] = []
    worker = wiris_runtime_workers.WirisClockCommandWorker(
        (
            ("left", "10.0.0.231", 2240, 5.0),
            ("right", "10.0.0.232", 2240, 5.0),
        ),
        command_text="STUS",
    )
    worker.completed.connect(completed.append)

    worker.run()

    assert set(observed) == {"10.0.0.231", "10.0.0.232"}
    assert completed[0]["results"]["left"]["ack_ok"] is True
    assert completed[0]["results"]["right"]["ack_ok"] is True


def test_stus_field_apply_latency_uses_dedicated_bounded_response_timeout(monkeypatch) -> None:
    """An 8.9 s camera apply ACK must fit without changing generic 5 s control I/O."""

    from Workstation import wiris_runtime_workers

    command_timeouts: list[float] = []
    monkeypatch.setattr(
        wiris_runtime_workers.time,
        "monotonic",
        lambda: 0.0,
    )
    monkeypatch.setattr(
        wiris_runtime_workers.WirisClockCommandWorker,
        "_measure_control_rtt",
        lambda self, **_kwargs: {
            "command": "HIWS",
            "samples_s": [0.78, 0.79, 0.80],
            "median_s": 0.79,
            "errors": [],
            "warnings": ["high latency"],
        },
    )
    monkeypatch.setattr(
        wiris_runtime_workers,
        "send_tcp_line",
        lambda command: command_timeouts.append(float(command.timeout_s)) or "OK",
    )
    completed: list[object] = []
    worker = wiris_runtime_workers.WirisClockCommandWorker(
        (
            ("left", "10.0.0.231", 2240, 5.0),
            ("right", "10.0.0.232", 2240, 5.0),
        ),
        command_text="STUS",
        operation_deadline_monotonic_s=100.0,
    )
    worker.completed.connect(completed.append)

    worker.run()

    assert command_timeouts == pytest.approx([15.0, 15.0])
    assert completed[0]["results"]["left"]["ack_ok"] is True
    assert completed[0]["results"]["right"]["ack_ok"] is True
    assert completed[0]["results"]["left"]["terminal_after_response"] == ""


def test_stus_response_timeout_is_terminal_and_right_remains_untouched(monkeypatch) -> None:
    from Workstation import wiris_runtime_workers

    command_timeouts: list[float] = []
    monkeypatch.setattr(
        wiris_runtime_workers.WirisClockCommandWorker,
        "_measure_control_rtt",
        lambda self, **_kwargs: {
            "command": "HIWS",
            "samples_s": [0.78, 0.79, 0.80],
            "median_s": 0.79,
            "errors": [],
            "warnings": ["high latency"],
        },
    )

    def timeout_send(command) -> str:
        command_timeouts.append(float(command.timeout_s))
        raise TimeoutError("camera STUS response timeout")

    monkeypatch.setattr(wiris_runtime_workers, "send_tcp_line", timeout_send)
    completed: list[object] = []
    worker = wiris_runtime_workers.WirisClockCommandWorker(
        (
            ("left", "10.0.0.231", 2240, 5.0),
            ("right", "10.0.0.232", 2240, 5.0),
        ),
        command_text="STUS",
    )
    worker.completed.connect(completed.append)

    worker.run()

    assert command_timeouts == pytest.approx([15.0])
    assert completed[0]["results"]["left"]["ack_ok"] is False
    assert completed[0]["results"]["right"]["skipped_due_to"] == "left-stus-ack-not-ok"


def test_stus_closed_loop_formula_removes_only_left_right_differential() -> None:
    from Workstation.camera_time_sync_controller import (
        stus_relative_pair_compensation_by_camera,
    )

    # Absolute SR-PC common mode is diagnostic only.  The target change is
    # symmetric around that common mode and removes only the L/R differential.
    next_by_camera, common_mode_s, relative_error_by_camera = (
        stus_relative_pair_compensation_by_camera(
            previous_compensation_by_camera={"left": 0.0, "right": 0.0},
            sr_pc_offset_by_camera={"left": -0.736135482, "right": -1.307044029},
        )
    )
    assert common_mode_s == pytest.approx((-0.736135482 - 1.307044029) / 2.0)
    assert relative_error_by_camera["left"] == pytest.approx(+0.2854542735)
    assert relative_error_by_camera["right"] == pytest.approx(-0.2854542735)
    assert next_by_camera == pytest.approx(
        {"left": -0.2854542735, "right": +0.2854542735}
    )

    # A huge shared absolute offset must not change either target.
    next_by_camera, common_mode_s, relative_error_by_camera = (
        stus_relative_pair_compensation_by_camera(
            previous_compensation_by_camera={"left": 0.4, "right": 0.6},
            sr_pc_offset_by_camera={"left": 32_400.0, "right": 32_400.0},
        )
    )
    assert common_mode_s == pytest.approx(32_400.0)
    assert relative_error_by_camera == pytest.approx({"left": 0.0, "right": 0.0})
    assert next_by_camera == pytest.approx({"left": 0.4, "right": 0.6})

    with pytest.raises(ValueError, match="limit"):
        stus_relative_pair_compensation_by_camera(
            previous_compensation_by_camera={"left": 1.9, "right": -1.9},
            sr_pc_offset_by_camera={"left": -1.0, "right": 1.0},
        )


def test_stus_correction_requires_three_unique_stable_pad_probe_sender_reports(
    monkeypatch,
) -> None:
    from Workstation import camera_time_sync_controller
    from Workstation.camera_time_sync_controller import CameraTimeSyncController

    histories: dict[str, list[dict[str, object]]] = {"left": [], "right": []}
    operation = {
        "active": True,
        "phase": "observing-attempt-1",
        "attempts_started": 1,
        "max_attempts": 2,
        "last_compensation_by_camera": {"left": 0.0, "right": 0.0},
        "feedback_attempt_index": 1,
        "sr_feedback_samples_by_camera": {"left": [], "right": []},
        "observer_generation": 2,
        "last_stus_ack_finished_utc_by_camera": {"left": 999.0, "right": 999.0},
    }
    panel = SimpleNamespace(
        _camera_sync_stus_operation=operation,
        _rtcp_clock_observation_history=histories,
        _camera_sync_max_sr_age_s=lambda: 7.0,
        _camera_sync_max_clock_rate_error=lambda: 0.005,
        _camera_sync_max_pc_offset_s=lambda: 0.1,
        _camera_sync_rtcp_pair_max_delta_s=lambda: 0.05,
        _camera_name_ko=lambda camera: camera,
    )
    monkeypatch.setattr(camera_time_sync_controller.time, "monotonic", lambda: 100.1)
    monkeypatch.setattr(camera_time_sync_controller.time, "time", lambda: 1000.1)
    controller = CameraTimeSyncController(panel)
    states: list[str] = []
    # The first two SRs are old observer-generation. The fourth repeats the
    # first valid identity. Only the final three distinct current-generation
    # post-ACK identities drive mutation.
    left_offsets = (-0.9, -0.8, -0.75, -0.75, -0.7362, -0.7361)
    right_offsets = (-1.5, -1.4, -1.32, -1.32, -1.3071, -1.3070)

    for index, (left_offset, right_offset) in enumerate(
        zip(left_offsets, right_offsets, strict=True),
        start=1,
    ):
        left = {
            "observer_generation": 1 if index <= 2 else 2,
            "ssrc": 101,
            "ntp_raw": 103 if index in {3, 4} else 100 + index,
            "arrival_basis": "rtcp-pad-probe",
            "sr_pc_offset_s": left_offset,
            "ntp_rate": 1.0,
            "media_rate": 1.0,
            "received_monotonic_s": 100.0 + index * 0.001,
            "received_wall_s": 1000.0 + index * 0.001,
        }
        right = {
            **left,
            "ssrc": 202,
            "ntp_raw": 203 if index in {3, 4} else 200 + index,
            "sr_pc_offset_s": right_offset,
        }
        histories["left"].append(left)
        histories["right"].append(right)
        state, evidence = controller._camera_sync_correction_observation()
        states.append(state)

    assert states == ["waiting", "waiting", "waiting", "waiting", "waiting", "candidate"]
    assert evidence["sr_pc_offset_by_camera"]["left"] == pytest.approx(-0.7362)
    assert evidence["sr_pc_offset_by_camera"]["right"] == pytest.approx(-1.3071)
    assert evidence["next_compensation_by_camera"] == pytest.approx(
        {"left": -0.28545, "right": 0.28545}
    )
    assert evidence["relative_error_by_camera"] == pytest.approx(
        {"left": 0.28545, "right": -0.28545}
    )
    assert evidence["offset_statistics_by_camera"]["right"]["unique_samples"] == 3
    assert len(histories["left"]) == 6


def test_stus_correction_never_acts_on_poll_detected_sender_report(monkeypatch) -> None:
    from Workstation import camera_time_sync_controller
    from Workstation.camera_time_sync_controller import CameraTimeSyncController

    metadata = [
        {
            "observer_generation": 2,
            "ssrc": 101,
            "ntp_raw": 120 + index,
            "arrival_basis": "poll-detected",
            "sr_pc_offset_s": -0.7,
            "ntp_rate": 1.0,
            "media_rate": 1.0,
            "received_monotonic_s": 100.0,
            "received_wall_s": 1000.0,
        }
        for index in range(3)
    ]
    operation = {
        "active": True,
        "phase": "observing-attempt-1",
        "attempts_started": 1,
        "last_compensation_by_camera": {"left": 0.0, "right": 0.0},
        "observer_generation": 2,
        "last_stus_ack_finished_utc_by_camera": {"left": 999.0, "right": 999.0},
    }
    panel = SimpleNamespace(
        _camera_sync_stus_operation=operation,
        _rtcp_clock_observation_history={
            "left": metadata,
            "right": [{**row, "ssrc": 202} for row in metadata],
        },
        _camera_sync_max_sr_age_s=lambda: 7.0,
        _camera_sync_max_clock_rate_error=lambda: 0.005,
        _camera_sync_max_pc_offset_s=lambda: 0.1,
        _camera_sync_rtcp_pair_max_delta_s=lambda: 0.05,
        _camera_name_ko=lambda camera: camera,
    )
    monkeypatch.setattr(camera_time_sync_controller.time, "monotonic", lambda: 100.1)
    monkeypatch.setattr(camera_time_sync_controller.time, "time", lambda: 1000.1)

    state, evidence = CameraTimeSyncController(panel)._camera_sync_correction_observation()

    assert state == "waiting"
    assert "observer SR" in evidence["reason"]
    assert "sr_feedback_samples_by_camera" not in operation


def test_post_ack_shared_gross_utc_offset_is_observed_but_does_not_gate(
    monkeypatch,
) -> None:
    from Workstation import camera_time_sync_controller
    from Workstation.camera_time_sync_controller import CameraTimeSyncController

    operation = {
        "active": True,
        "phase": "observing-attempt-1",
        "attempts_started": 1,
        "observer_generation": 2,
        "last_stus_ack_finished_utc_by_camera": {"left": 999.0, "right": 999.0},
    }
    def gross_rows(side: str, offset_s: float) -> list[dict[str, object]]:
        return [
            {
                "observer_generation": 2,
                "ssrc": 101 if side == "left" else 202,
                "ntp_raw": 120 + index,
                "arrival_basis": "rtcp-pad-probe",
                "sr_pc_offset_s": None,
                "sr_pc_offset_raw_s": offset_s,
                "ntp_rate": 1.0,
                "media_rate": 1.0,
                "received_monotonic_s": 100.0 + index * 0.001,
                "received_wall_s": 1000.0 + index * 0.001,
            }
            for index in range(3)
        ]
    panel = SimpleNamespace(
        _camera_sync_stus_operation=operation,
        _rtcp_clock_observation_history={
            "left": gross_rows("left", 32_400.0),
            "right": gross_rows("right", 32_400.05),
        },
        _camera_sync_max_sr_age_s=lambda: 7.0,
        _camera_sync_max_clock_rate_error=lambda: 0.005,
        _camera_sync_max_pc_offset_s=lambda: 0.1,
        _camera_sync_rtcp_pair_max_delta_s=lambda: 0.1,
        _camera_name_ko=lambda camera: camera,
    )
    monkeypatch.setattr(camera_time_sync_controller.time, "monotonic", lambda: 100.1)
    monkeypatch.setattr(camera_time_sync_controller.time, "time", lambda: 1000.1)

    state, evidence = CameraTimeSyncController(panel)._camera_sync_correction_observation()

    assert state == "converged"
    assert evidence["pair_delta_s"] == pytest.approx(0.05)
    assert evidence["sr_pc_offset_by_camera"] == pytest.approx(
        {"left": 32_400.0, "right": 32_400.05}
    )
    assert evidence["sr_pc_role"] == "diagnostic-only"


def test_stus_both_is_left_ack_then_right_even_if_targets_arrive_reversed(monkeypatch) -> None:
    from Workstation import wiris_runtime_workers

    calls: list[tuple[str, str]] = []
    probe_hosts: list[str] = []

    def fake_send(command) -> str:
        calls.append((str(command.host), str(command.command)))
        return "OK"

    monkeypatch.setattr(wiris_runtime_workers, "send_tcp_line", fake_send)
    def fake_probe(self, *, host: str, **_kwargs):
        probe_hosts.append(host)
        return {
            "command": "HIWS",
            "samples_s": [0.010],
            "median_s": 0.010,
            "errors": [],
        }

    monkeypatch.setattr(wiris_runtime_workers.WirisClockCommandWorker, "_measure_control_rtt", fake_probe)
    results: list[object] = []
    worker = wiris_runtime_workers.WirisClockCommandWorker(
        (
            ("right", "10.0.0.232", 2240, 1.0),
            ("left", "10.0.0.231", 2240, 1.0),
        ),
        command_text="STUS",
        shared_utc=True,
    )
    worker.completed.connect(results.append)

    worker.run()

    assert [host for host, _command in calls] == ["10.0.0.231", "10.0.0.232"]
    assert set(probe_hosts) == {"10.0.0.231", "10.0.0.232"}
    assert all(command.startswith("STUS ") for _host, command in calls)
    assert results[0]["results"]["left"]["response"] == "OK"
    assert results[0]["results"]["right"]["response"] == "OK"
    assert results[0]["dispatch_policy"] == "left-ok-ack-then-right"


def test_stus_both_does_not_send_right_when_left_ack_is_not_ok(monkeypatch) -> None:
    from Workstation import wiris_runtime_workers

    calls: list[str] = []
    probe_hosts: list[str] = []

    def fake_send(command) -> str:
        calls.append(str(command.host))
        return "ERR rejected"

    monkeypatch.setattr(wiris_runtime_workers, "send_tcp_line", fake_send)
    def fake_probe(self, *, host: str, **_kwargs):
        probe_hosts.append(host)
        return {
            "command": "HIWS",
            "samples_s": [0.010],
            "median_s": 0.010,
            "errors": [],
        }

    monkeypatch.setattr(wiris_runtime_workers.WirisClockCommandWorker, "_measure_control_rtt", fake_probe)
    results: list[object] = []
    worker = wiris_runtime_workers.WirisClockCommandWorker(
        (
            ("left", "10.0.0.231", 2240, 1.0),
            ("right", "10.0.0.232", 2240, 1.0),
        ),
        command_text="STUS",
        shared_utc=True,
    )
    worker.completed.connect(results.append)

    worker.run()

    assert calls == ["10.0.0.231"]
    assert set(probe_hosts) == {"10.0.0.231", "10.0.0.232"}
    assert results[0]["results"]["left"]["response"] == "ERR rejected"
    assert "SKIPPED" in results[0]["results"]["right"]["response"]
    assert "right" not in results[0]["command_times"]
    assert results[0]["dispatch_policy"] == "left-ok-ack-then-right"


def test_stus_closed_loop_attempt_still_skips_right_after_left_failure(monkeypatch) -> None:
    from Workstation import wiris_runtime_workers

    calls: list[tuple[str, str]] = []

    def fake_send(command) -> str:
        calls.append((str(command.host), str(command.command)))
        return "OK" if str(command.command) == "HIWS" else "ERR rejected"

    monkeypatch.setattr(wiris_runtime_workers, "send_tcp_line", fake_send)
    monkeypatch.setattr(
        wiris_runtime_workers.WirisClockCommandWorker,
        "_measure_control_rtt",
        lambda self, **_kwargs: {
            "command": "HIWS",
            "samples_s": [0.51, 0.52, 0.53],
            "median_s": 0.52,
            "errors": [],
            "warnings": ["high latency"],
        },
    )
    results: list[object] = []
    worker = wiris_runtime_workers.WirisClockCommandWorker(
        (
            ("right", "10.0.0.232", 2240, 5.0),
            ("left", "10.0.0.231", 2240, 5.0),
        ),
        command_text="STUS",
        shared_utc=True,
        stus_compensation_by_camera={"left": 0.736135482, "right": 1.307044029},
        stus_feedback_offset_by_camera={"left": -0.736135482, "right": -1.307044029},
        stus_attempt_index=2,
    )
    worker.completed.connect(results.append)

    worker.run()

    assert [host for host, command in calls if command.startswith("STUS ")] == ["10.0.0.231"]
    assert results[0]["results"]["left"]["ack_ok"] is False
    assert results[0]["results"]["right"]["skipped_due_to"] == "left-stus-ack-not-ok"
    assert results[0]["stus_attempt_index"] == 2
    assert results[0]["compensation_method"] == "rtcp-relative-closed-loop"


def test_stus_preflight_side_failure_causes_zero_clock_mutations(monkeypatch) -> None:
    from Workstation import wiris_runtime_workers

    mutations: list[str] = []

    def fake_probe(self, *, host: str, **_kwargs):
        if host.endswith("232"):
            return {
                "command": "HIWS",
                "samples_s": [],
                "median_s": None,
                "errors": ["timeout"],
                "warnings": [],
            }
        return {
            "command": "HIWS",
            "samples_s": [0.51, 0.52, 0.53],
            "median_s": 0.52,
            "errors": [],
            "warnings": ["high latency"],
        }

    monkeypatch.setattr(
        wiris_runtime_workers.WirisClockCommandWorker,
        "_measure_control_rtt",
        fake_probe,
    )
    monkeypatch.setattr(
        wiris_runtime_workers,
        "send_tcp_line",
        lambda command: mutations.append(str(command.command)) or "OK",
    )
    results: list[object] = []
    worker = wiris_runtime_workers.WirisClockCommandWorker(
        (
            ("left", "10.0.0.231", 2240, 5.0),
            ("right", "10.0.0.232", 2240, 5.0),
        ),
        command_text="STUS",
    )
    worker.completed.connect(results.append)

    worker.run()

    assert mutations == []
    assert results[0]["results"]["left"]["skipped_due_to"] == "stus-hiws-preflight-failed"
    assert results[0]["results"]["right"]["skipped_due_to"] == "stus-hiws-preflight-failed"


def test_stus_direct_cancellation_after_left_io_prevents_right_mutation(monkeypatch) -> None:
    from Workstation import wiris_runtime_workers

    mutations: list[str] = []
    worker: object

    monkeypatch.setattr(
        wiris_runtime_workers.WirisClockCommandWorker,
        "_measure_control_rtt",
        lambda self, **_kwargs: {
            "command": "HIWS",
            "samples_s": [0.51, 0.52, 0.53],
            "median_s": 0.52,
            "errors": [],
            "warnings": ["high latency"],
        },
    )

    def fake_send(command) -> str:
        mutations.append(str(command.host))
        worker.cancel()
        return "OK"

    monkeypatch.setattr(wiris_runtime_workers, "send_tcp_line", fake_send)
    failures: list[str] = []
    completed: list[object] = []
    worker = wiris_runtime_workers.WirisClockCommandWorker(
        (
            ("left", "10.0.0.231", 2240, 5.0),
            ("right", "10.0.0.232", 2240, 5.0),
        ),
        command_text="STUS",
    )
    worker.failed.connect(failures.append)
    worker.completed.connect(completed.append)

    worker.run()

    assert mutations == ["10.0.0.231"]
    assert failures == []
    assert completed[0]["results"]["left"]["ack_ok"] is True
    assert (
        completed[0]["results"]["left"]["terminal_after_response"]
        == "operation-cancelled-after-response"
    )
    assert (
        completed[0]["results"]["right"]["skipped_due_to"]
        == "operation-cancelled-after-response"
    )


def test_stus_post_io_deadline_preserves_left_ack_and_skips_right(monkeypatch) -> None:
    from Workstation import wiris_runtime_workers

    monotonic_values = iter((99.9, 100.1))
    mutations: list[str] = []
    monkeypatch.setattr(
        wiris_runtime_workers.time,
        "monotonic",
        lambda: next(monotonic_values),
    )
    monkeypatch.setattr(
        wiris_runtime_workers.WirisClockCommandWorker,
        "_measure_control_rtt",
        lambda self, **_kwargs: {
            "command": "HIWS",
            "samples_s": [0.51, 0.52, 0.53],
            "median_s": 0.52,
            "errors": [],
            "warnings": ["high latency"],
        },
    )
    monkeypatch.setattr(
        wiris_runtime_workers,
        "send_tcp_line",
        lambda command: mutations.append(str(command.host)) or "OK",
    )
    completed: list[object] = []
    worker = wiris_runtime_workers.WirisClockCommandWorker(
        (
            ("left", "10.0.0.231", 2240, 5.0),
            ("right", "10.0.0.232", 2240, 5.0),
        ),
        command_text="STUS",
        operation_deadline_monotonic_s=100.0,
    )
    worker.completed.connect(completed.append)

    worker.run()

    assert mutations == ["10.0.0.231"]
    assert completed[0]["results"]["left"]["ack_ok"] is True
    assert (
        completed[0]["results"]["left"]["terminal_after_response"]
        == "operation-deadline-exceeded-after-response"
    )
    assert (
        completed[0]["results"]["right"]["skipped_due_to"]
        == "operation-deadline-exceeded-after-response"
    )


def test_stus_ack_evidence_stays_pending_until_fresh_sr_quality_verifies() -> None:
    from Workstation.wiris_control_controller import WirisControlController
    from Workstation.workstation_core.time_sync_epoch import _stus_reference

    class TextSink:
        def __init__(self) -> None:
            self.text = ""

        def setText(self, text: str) -> None:
            self.text = str(text)

        def setPlainText(self, text: str) -> None:
            self.text = str(text)

    class Button:
        def __init__(self) -> None:
            self.enabled = False

        def setEnabled(self, enabled: bool) -> None:
            self.enabled = bool(enabled)

    operation = {
        "operation_id": "stus-test",
        "active": True,
        "phase": "correction-command",
        "max_attempts": 2,
        "attempts_started": 2,
        "attempts_completed": 1,
        "attempts": [],
        "deadline_monotonic_s": time.monotonic() + 100.0,
    }
    begin_calls: list[tuple[str, str]] = []
    panel = SimpleNamespace(
        _camera_sync_stus_operation=operation,
        _last_wiris_pc_utc_sync=None,
        wiris_time_sync_thread=object(),
        wiris_set_pc_time_button=Button(),
        wiris_command_status_label=TextSink(),
        clock_sync_status_label=TextSink(),
        clock_sync_summary_text=TextSink(),
        _camera_name_ko=lambda camera: camera,
        _append_wiris_command_log=lambda _text: None,
        _begin_camera_sync_after_stus=lambda *, target, result: begin_calls.append((target, result)),
        _log=lambda _text: None,
    )
    result = {
        "command": "STUS",
        "operation_id": "stus-test",
        "stus_attempt_index": 2,
        "compensation_method": "rtcp-relative-closed-loop",
        "dispatch_policy": "left-ok-ack-then-right",
        "requested_utc_s": 1000.0,
        "requested_utc_by_camera": {"left": 1000.0, "right": 1001.0},
        "command_times": {"left": 1000.0, "right": 1001.0},
        "tcp_start_times": {"left": 10.0, "right": 11.0},
        "results": {
            "left": {
                "host": "10.0.0.231",
                "response": "OK",
                "command": "STUS 1000 0",
                "query_finished_utc_s": 1002.0,
                "stus_compensation_s": -0.200,
                "stus_compensation_source": "rtcp-relative-closed-loop",
                "stus_feedback_offset_s": 0.300,
            },
            "right": {
                "host": "10.0.0.232",
                "response": "OK",
                "command": "STUS 1001 0",
                "query_finished_utc_s": 1003.0,
                "stus_compensation_s": 0.500,
                "stus_compensation_source": "rtcp-relative-closed-loop",
                "stus_feedback_offset_s": -0.400,
            },
        },
    }
    controller = WirisControlController(panel)

    controller._apply_wiris_time_sync_result(
        "both",
        (
            ("left", "10.0.0.231", 2240, 5.0),
            ("right", "10.0.0.232", 2240, 5.0),
        ),
        result,
    )

    assert begin_calls == [("both", "success")]
    assert operation["last_compensation_by_camera"] == {"left": -0.2, "right": 0.5}
    assert operation["attempts_completed"] == 2
    assert operation["attempts"][0]["rtt_probe_errors_by_camera"] == {
        "left": [],
        "right": [],
    }
    assert panel._last_wiris_pc_utc_sync["result"] == "pending-verification"
    assert panel._last_wiris_pc_utc_sync["stus_rtt_probe_errors_by_camera"] == {
        "left": [],
        "right": [],
    }
    assert _stus_reference({"pc_utc_sync": panel._last_wiris_pc_utc_sync}) is None
    assert "L/R relative feedback=+300.00ms" in panel.clock_sync_summary_text.text

    panel.wiris_time_sync_thread = None
    controller._finish_wiris_time_sync_operation("verified", "3 stable SRs converged")

    assert panel._last_wiris_pc_utc_sync["result"] == "success"
    assert _stus_reference({"pc_utc_sync": panel._last_wiris_pc_utc_sync}) is not None
    assert panel.wiris_set_pc_time_button.enabled is True

    controller._apply_wiris_time_sync_result(
        "both",
        (
            ("left", "10.0.0.231", 2240, 5.0),
            ("right", "10.0.0.232", 2240, 5.0),
        ),
        result,
    )

    assert begin_calls == [("both", "success")]
    assert operation["late_results"][0]["attempt_index"] == 2


def test_new_stus_operation_immediately_supersedes_prior_verified_truth() -> None:
    from Workstation.wiris_control_controller import WirisControlController
    from Workstation.workstation_core.time_sync_epoch import _stus_reference

    class TextSink:
        def __init__(self) -> None:
            self.text = ""

        def setText(self, text: str) -> None:
            self.text = str(text)

    panel = SimpleNamespace(
        _shutdown_in_progress=False,
        _camera_sync_stus_operation=None,
        _camera_sync_deadline_monotonic_s=1.0,
        _camera_sync_stus_sr_baseline={"left": 3, "right": 3},
        _camera_sync_last_sr_counts={"left": 3, "right": 3},
        _last_wiris_pc_utc_sync={
            "command": "STUS",
            "operation_id": "old",
            "result": "success",
            "responses": {"left": "OK", "right": "OK"},
        },
        wiris_time_sync_thread=None,
        wiris_command_status_label=TextSink(),
        clock_sync_status_label=TextSink(),
        wiris_set_pc_time_button=SimpleNamespace(setEnabled=lambda _enabled: None),
        zoom_timeout_spin=SimpleNamespace(value=lambda: 5.0),
        _session_recording_active=lambda: False,
        zoom_hosts=lambda _target: [
            ("left", "10.0.0.231"),
            ("right", "10.0.0.232"),
        ],
        _control_port_for_camera=lambda _camera: 2240,
        _set_camera_sync_state=lambda _status, _reason: None,
        _camera_sync_timer=SimpleNamespace(start=lambda: None),
        _restart_rtcp_clock_observer=lambda **_kwargs: True,
        _log=lambda _text: None,
    )
    controller = WirisControlController(panel)
    controller._start_wiris_time_sync_worker = lambda **_kwargs: True

    assert controller.set_selected_wiris_time_from_pc("both") is None

    current = panel._last_wiris_pc_utc_sync
    assert current["operation_id"] != "old"
    assert current["result"] == "pending-command"
    assert "responses" not in current
    assert _stus_reference({"pc_utc_sync": current}) is None
    assert panel._camera_sync_deadline_monotonic_s is None
    assert panel._camera_sync_stus_sr_baseline == {}
    assert panel._camera_sync_last_sr_counts == {}

    controller._finish_wiris_time_sync_operation("failed", "preflight timeout")

    assert panel._last_wiris_pc_utc_sync["operation_id"] == current["operation_id"]
    assert panel._last_wiris_pc_utc_sync["result"] == "failed"
    assert "responses" not in panel._last_wiris_pc_utc_sync


class _Timer:
    def __init__(self) -> None:
        self.starts = 0
        self.stops = 0
        self.interval_ms: int | None = None

    def start(self) -> None:
        self.starts += 1

    def stop(self) -> None:
        self.stops += 1

    def setInterval(self, interval_ms: int) -> None:
        self.interval_ms = int(interval_ms)


def test_stus_single_action_requests_one_correction_then_converges(monkeypatch) -> None:
    from Workstation import camera_time_sync_controller
    from Workstation.camera_time_sync_controller import (
        CAMERA_SYNC_STATUS_VERIFIED,
        CAMERA_SYNC_STATUS_WARMING,
        CameraTimeSyncController,
    )

    timer = _Timer()
    operation = {
        "active": True,
        "phase": "observing-attempt-1",
        "attempts_started": 1,
        "max_attempts": 2,
        "deadline_monotonic_s": 190.0,
    }
    correction_requests: list[dict[str, object]] = []
    finishes: list[tuple[str, str]] = []
    readiness = iter(((False, "timing outside limits"), (True, "converged")))
    panel = SimpleNamespace(
        camera_sync_status=CAMERA_SYNC_STATUS_WARMING,
        camera_sync_reason="warming",
        _camera_sync_timer=timer,
        _camera_sync_deadline_monotonic_s=130.0,
        _camera_sync_stus_sr_baseline={"left": 0, "right": 0},
        _camera_sync_stus_operation=operation,
        _session_recording_active=lambda: False,
        _camera_sync_readiness=lambda: next(readiness),
        _camera_sync_correction_observation=lambda: (
            (
                "converged",
                {"reason": "3 unique stable pad-probe SRs converged"},
            )
            if operation["attempts_started"] == 2
            else (
                "candidate",
                {
                    "sr_pc_offset_by_camera": {"left": -0.736135482, "right": -1.307044029},
                    "next_compensation_by_camera": {"left": -0.2854542735, "right": 0.2854542735},
                    "relative_error_by_camera": {"left": 0.2854542735, "right": -0.2854542735},
                },
            )
        ),
    )

    def set_state(status: str, reason: str) -> None:
        panel.camera_sync_status = status
        panel.camera_sync_reason = reason

    def request_correction(**payload: object) -> bool:
        correction_requests.append(dict(payload))
        operation["attempts_started"] = 2
        operation["phase"] = "correction-command"
        return True

    panel._set_camera_sync_state = set_state
    panel._request_wiris_time_sync_correction = request_correction
    panel._finish_wiris_time_sync_operation = lambda state, reason: finishes.append((state, reason))
    monkeypatch.setattr(camera_time_sync_controller.time, "monotonic", lambda: 100.0)
    controller = CameraTimeSyncController(panel)

    controller._poll_camera_sync_state()

    assert len(correction_requests) == 1
    assert correction_requests[0]["compensation_by_camera"]["right"] == pytest.approx(0.2854542735)
    assert panel.camera_sync_status == CAMERA_SYNC_STATUS_WARMING

    operation["phase"] = "observing-attempt-2"
    controller._poll_camera_sync_state()

    assert panel.camera_sync_status == CAMERA_SYNC_STATUS_VERIFIED
    assert len(correction_requests) == 1
    assert finishes == [("verified", "3 unique stable pad-probe SRs converged")]


def test_stus_single_action_timeout_fails_without_another_mutation(monkeypatch) -> None:
    from Workstation import camera_time_sync_controller
    from Workstation.camera_time_sync_controller import (
        CAMERA_SYNC_STATUS_FAILED,
        CAMERA_SYNC_STATUS_WARMING,
        CameraTimeSyncController,
    )

    timer = _Timer()
    correction_requests: list[object] = []
    finishes: list[tuple[str, str]] = []
    panel = SimpleNamespace(
        camera_sync_status=CAMERA_SYNC_STATUS_WARMING,
        camera_sync_reason="fresh SR pending",
        _camera_sync_timer=timer,
        _camera_sync_deadline_monotonic_s=200.0,
        _camera_sync_stus_sr_baseline={"left": 0, "right": 0},
        _camera_sync_stus_operation={
            "active": True,
            "phase": "observing-attempt-1",
            "attempts_started": 1,
            "max_attempts": 2,
            "deadline_monotonic_s": 99.0,
        },
        _session_recording_active=lambda: False,
        _camera_sync_readiness=lambda: (False, "fresh SR pending"),
        _request_wiris_time_sync_correction=lambda **payload: correction_requests.append(payload),
        _finish_wiris_time_sync_operation=lambda state, reason: finishes.append((state, reason)),
    )
    panel._set_camera_sync_state = lambda status, reason: (
        setattr(panel, "camera_sync_status", status),
        setattr(panel, "camera_sync_reason", reason),
    )
    monkeypatch.setattr(camera_time_sync_controller.time, "monotonic", lambda: 100.0)

    CameraTimeSyncController(panel)._poll_camera_sync_state()

    assert panel.camera_sync_status == CAMERA_SYNC_STATUS_FAILED
    assert "90" in panel.camera_sync_reason or "timeout" in panel.camera_sync_reason
    assert correction_requests == []
    assert finishes and finishes[0][0] == "failed"


def test_stus_second_attempt_nonconvergence_fails_without_third_mutation(monkeypatch) -> None:
    from Workstation import camera_time_sync_controller
    from Workstation.camera_time_sync_controller import (
        CAMERA_SYNC_STATUS_FAILED,
        CAMERA_SYNC_STATUS_WARMING,
        CameraTimeSyncController,
    )

    timer = _Timer()
    mutations: list[object] = []
    finishes: list[tuple[str, str]] = []
    operation = {
        "active": True,
        "phase": "observing-attempt-2",
        "attempts_started": 2,
        "max_attempts": 2,
        "deadline_monotonic_s": 190.0,
    }
    panel = SimpleNamespace(
        camera_sync_status=CAMERA_SYNC_STATUS_WARMING,
        camera_sync_reason="attempt 2 outside limits",
        _camera_sync_timer=timer,
        _camera_sync_deadline_monotonic_s=130.0,
        _camera_sync_stus_sr_baseline={"left": 0, "right": 0},
        _camera_sync_stus_operation=operation,
        _session_recording_active=lambda: False,
        _camera_sync_readiness=lambda: (False, "SR-PC still 180ms"),
        _camera_sync_correction_observation=lambda: (
            "candidate",
            {
                "sr_pc_offset_by_camera": {"left": -0.18, "right": -0.17},
                "next_compensation_by_camera": {"left": 0.91, "right": 1.48},
            },
        ),
        _request_wiris_time_sync_correction=lambda **payload: mutations.append(payload),
        _finish_wiris_time_sync_operation=lambda state, reason: finishes.append((state, reason)),
    )
    panel._set_camera_sync_state = lambda status, reason: (
        setattr(panel, "camera_sync_status", status),
        setattr(panel, "camera_sync_reason", reason),
    )
    monkeypatch.setattr(camera_time_sync_controller.time, "monotonic", lambda: 100.0)

    CameraTimeSyncController(panel)._poll_camera_sync_state()

    assert panel.camera_sync_status == CAMERA_SYNC_STATUS_FAILED
    assert "2/2" in panel.camera_sync_reason
    assert mutations == []
    assert finishes and finishes[0][0] == "failed"


def test_stus_correction_continues_while_recording_is_active(monkeypatch) -> None:
    from Workstation import camera_time_sync_controller
    from Workstation.camera_time_sync_controller import (
        CAMERA_SYNC_STATUS_WARMING,
        CameraTimeSyncController,
    )

    timer = _Timer()
    mutations: list[object] = []
    panel = SimpleNamespace(
        camera_sync_status=CAMERA_SYNC_STATUS_WARMING,
        camera_sync_reason="timing outside limits",
        _camera_sync_timer=timer,
        _camera_sync_deadline_monotonic_s=130.0,
        _camera_sync_stus_sr_baseline={"left": 0, "right": 0},
        _camera_sync_stus_operation={
            "active": True,
            "phase": "observing-attempt-1",
            "attempts_started": 1,
            "max_attempts": 2,
            "deadline_monotonic_s": 190.0,
        },
        _session_recording_active=lambda: True,
        _camera_sync_readiness=lambda: (False, "timing outside limits"),
        _camera_sync_correction_observation=lambda: (
            "candidate",
            {
                "sr_pc_offset_by_camera": {"left": -0.2, "right": -0.3},
                "next_compensation_by_camera": {"left": 0.2, "right": 0.3},
            },
        ),
        _request_wiris_time_sync_correction=lambda **payload: mutations.append(payload) or True,
        _finish_wiris_time_sync_operation=lambda *_args: None,
    )
    panel._set_camera_sync_state = lambda status, reason: (
        setattr(panel, "camera_sync_status", status),
        setattr(panel, "camera_sync_reason", reason),
    )
    monkeypatch.setattr(camera_time_sync_controller.time, "monotonic", lambda: 100.0)

    CameraTimeSyncController(panel)._poll_camera_sync_state()

    assert panel.camera_sync_status == CAMERA_SYNC_STATUS_WARMING
    assert len(mutations) == 1


def test_verified_sync_keeps_monitoring_then_downgrades_on_long_run_failure() -> None:
    from Workstation.camera_time_sync_controller import (
        CAMERA_SYNC_STATUS_FAILED,
        CAMERA_SYNC_STATUS_VERIFIED,
        CAMERA_SYNC_STATUS_WARMING,
        CameraTimeSyncController,
    )

    timer = _Timer()
    observations = iter(
        (
            (True, "initial current pair verified"),
            (False, "left SR stale 7.01s"),
        )
    )
    panel = SimpleNamespace(
        camera_sync_status=CAMERA_SYNC_STATUS_WARMING,
        camera_sync_reason="warming",
        _camera_sync_timer=timer,
        _camera_sync_deadline_monotonic_s=None,
        _camera_sync_stus_sr_baseline={},
        _camera_sync_readiness=lambda: next(observations),
    )

    def set_state(status: str, reason: str) -> None:
        panel.camera_sync_status = status
        panel.camera_sync_reason = reason

    panel._set_camera_sync_state = set_state
    controller = CameraTimeSyncController(panel)

    controller._poll_camera_sync_state()

    assert panel.camera_sync_status == CAMERA_SYNC_STATUS_VERIFIED
    assert timer.stops == 0
    assert timer.interval_ms == 1000

    controller._poll_camera_sync_state()

    assert panel.camera_sync_status == CAMERA_SYNC_STATUS_FAILED
    assert "SR stale" in panel.camera_sync_reason
    assert timer.stops == 1


def test_new_stus_initial_command_does_not_revoke_verified_sync(monkeypatch) -> None:
    from Workstation import camera_time_sync_controller
    from Workstation.camera_time_sync_controller import (
        CAMERA_SYNC_STATUS_VERIFIED,
        CAMERA_SYNC_STATUS_WARMING,
        CameraTimeSyncController,
    )

    timer = _Timer()
    operation = {
        "active": True,
        "phase": "initial-command",
        "attempts_started": 1,
        "max_attempts": 2,
        "deadline_monotonic_s": 190.0,
    }
    last_sync = {
        "command": "STUS",
        "result": "pending-command",
    }
    panel = SimpleNamespace(
        camera_sync_status=CAMERA_SYNC_STATUS_VERIFIED,
        camera_sync_reason="previous operation continuously verified",
        _camera_sync_timer=timer,
        _camera_sync_deadline_monotonic_s=None,
        _camera_sync_stus_sr_baseline={},
        _camera_sync_last_sr_counts={},
        _camera_sync_stus_operation=operation,
        _last_wiris_pc_utc_sync=last_sync,
        _session_recording_active=lambda: False,
        _camera_sync_readiness=lambda: (False, "left/right readiness dipped during STUS"),
        _camera_sync_correction_observation=lambda: (
            "waiting",
            {"reason": "phase=initial-command"},
        ),
    )

    def set_state(status: str, reason: str) -> None:
        panel.camera_sync_status = status
        panel.camera_sync_reason = reason

    panel._set_camera_sync_state = set_state
    monkeypatch.setattr(camera_time_sync_controller.time, "monotonic", lambda: 100.0)

    CameraTimeSyncController(panel)._poll_camera_sync_state()

    assert panel.camera_sync_status == CAMERA_SYNC_STATUS_WARMING
    assert "1/2" in panel.camera_sync_reason
    assert "ACK" in panel.camera_sync_reason
    assert operation["active"] is True
    assert last_sync["result"] == "pending-command"
    assert "revoked_at_unix_s" not in last_sync
    assert timer.stops == 0


def _observer_rows(
    side: str,
    *,
    generation: int = 2,
    offset_s: float = 0.010,
    received_monotonic_s: float = 100.0,
    ntp_rate: float = 1.0,
    media_rate: float = 1.0,
    count: int = 3,
) -> list[dict[str, object]]:
    ssrc = 101 if side == "left" else 202
    return [
        {
            "observer_generation": generation,
            "ssrc": ssrc,
            "ntp_raw": index,
            "received_monotonic_s": received_monotonic_s - (count - index) * 0.001,
            "arrival_basis": "rtcp-pad-probe",
            "sr_pc_offset_s": offset_s,
            "sr_pc_offset_raw_s": offset_s,
            "ntp_rate": ntp_rate,
            "media_rate": media_rate,
        }
        for index in range(1, count + 1)
    ]


def test_verified_sync_rejects_stale_observer_and_regressed_sr(monkeypatch) -> None:
    from Workstation import camera_time_sync_controller
    from Workstation.camera_time_sync_controller import CameraTimeSyncController

    panel = SimpleNamespace(
        camera_sync_status="verified",
        _rtcp_clock_observer_generation=2,
        _rtcp_clock_observation_history={
            "left": _observer_rows("left", received_monotonic_s=100.0),
            "right": _observer_rows("right", received_monotonic_s=100.0),
        },
        _camera_sync_last_sr_counts={"left": 5, "right": 5},
        _camera_name_ko=lambda camera: camera,
        _camera_sync_required_sr_count=lambda: 2,
        _camera_sync_max_sr_age_s=lambda: 7.0,
        _camera_sync_max_clock_rate_error=lambda: 0.005,
        _camera_sync_rtcp_pair_max_delta_s=lambda: 0.05,
        _camera_sync_max_pc_offset_s=lambda: 0.1,
    )
    monkeypatch.setattr(camera_time_sync_controller.time, "monotonic", lambda: 108.0)
    monkeypatch.setattr(camera_time_sync_controller.time, "time", lambda: 1008.0)

    ready, reason = CameraTimeSyncController(panel)._camera_sync_readiness()

    assert ready is False
    assert "SR stale 8.00s" in reason
    assert "SR counter regressed 5→3" in reason


def test_sync_pair_selection_uses_current_pair_not_healthiest_old_pair() -> None:
    from Workstation.camera_time_sync_controller import CameraTimeSyncController

    left_old = {"id": "left-old", "timestamp_s": 9.20, "rtcp_frame_ntp_unix_s": 100.00}
    right_old = {"id": "right-old", "timestamp_s": 9.21, "rtcp_frame_ntp_unix_s": 100.01}
    left_current = {"id": "left-current", "timestamp_s": 9.90, "rtcp_frame_ntp_unix_s": 101.00}
    right_current = {"id": "right-current", "timestamp_s": 9.91, "rtcp_frame_ntp_unix_s": 101.20}
    panel = SimpleNamespace(
        camera_frame_metadata={"left": left_current, "right": right_current},
        camera_frame_metadata_history={
            "left": deque((left_old, left_current)),
            "right": deque((right_old, right_current)),
        },
        _camera_rtc_timestamp_s_from_metadata=lambda row: row.get("rtcp_frame_ntp_unix_s"),
        _camera_sync_history_window_s=lambda: 1.0,
        _camera_sync_receive_pair_max_delta_s=lambda: 0.35,
    )

    left, right = CameraTimeSyncController(panel)._clock_sync_pair_metadata(now_mono_s=10.0)

    assert left["id"] == "left-current"
    assert right["id"] == "right-current"


@pytest.mark.parametrize(
    ("mutation", "expected_reason"),
    (
        ("pair-delta", "RTCP clock Δt=100.1ms"),
        ("ntp-rate", "NTP clock=1.005100"),
        ("media-rate", "RTP clock=0.994900"),
        ("missing-sr", "left observer SR 0/3"),
        ("right-offset-missing", "right SR-PC observation 없음"),
        ("host-step", "PC clock step=+60.0ms"),
    ),
)
def test_sync_health_hard_limits_fail_closed(
    monkeypatch,
    mutation: str,
    expected_reason: str,
) -> None:
    from Workstation import camera_time_sync_controller
    from Workstation.camera_time_sync_controller import CameraTimeSyncController

    left = _observer_rows("left")
    right = _observer_rows("right")
    if mutation == "pair-delta":
        right = _observer_rows("right", offset_s=0.1101)
    elif mutation == "ntp-rate":
        left = _observer_rows("left", ntp_rate=1.0051)
    elif mutation == "media-rate":
        left = _observer_rows("left", media_rate=0.9949)
    elif mutation == "missing-sr":
        left = []
    elif mutation == "right-offset-missing":
        for row in right:
            row.pop("sr_pc_offset_s")
            row.pop("sr_pc_offset_raw_s")
    elif mutation == "host-step":
        left[-1]["host_clock_step_s"] = 0.060

    panel = SimpleNamespace(
        camera_sync_status="verified",
        _rtcp_clock_observer_generation=2,
        _rtcp_clock_observation_history={"left": left, "right": right},
        _camera_sync_last_sr_counts={},
        _camera_name_ko=lambda camera: camera,
        _camera_sync_required_sr_count=lambda: 2,
        _camera_sync_max_sr_age_s=lambda: 7.0,
        _camera_sync_max_clock_rate_error=lambda: 0.005,
        _camera_sync_rtcp_pair_max_delta_s=lambda: 0.1,
        _camera_sync_max_pc_offset_s=lambda: 0.1,
    )
    monkeypatch.setattr(camera_time_sync_controller.time, "monotonic", lambda: 100.1)
    monkeypatch.setattr(camera_time_sync_controller.time, "time", lambda: 1000.1)

    ready, reason = CameraTimeSyncController(panel)._camera_sync_readiness()

    assert ready is False
    assert expected_reason in reason


def test_sync_health_keeps_large_shared_sr_pc_as_diagnostic_only(monkeypatch) -> None:
    from Workstation import camera_time_sync_controller
    from Workstation.camera_time_sync_controller import CameraTimeSyncController

    left = _observer_rows("left", offset_s=32_400.0)
    right = _observer_rows("right", offset_s=32_400.08)
    for row in left:
        row["sr_pc_offset_s"] = None
        row["sr_pc_offset_raw_s"] = 32_400.0
    for row in right:
        row["sr_pc_offset_s"] = None
        row["sr_pc_offset_raw_s"] = 32_400.08
    panel = SimpleNamespace(
        camera_sync_status="verified",
        _rtcp_clock_observer_generation=2,
        _rtcp_clock_observation_history={"left": left, "right": right},
        _camera_sync_last_sr_counts={},
        _camera_name_ko=lambda camera: camera,
        _camera_sync_required_sr_count=lambda: 2,
        _camera_sync_max_sr_age_s=lambda: 7.0,
        _camera_sync_max_clock_rate_error=lambda: 0.005,
        _camera_sync_rtcp_pair_max_delta_s=lambda: 0.1,
        _camera_sync_max_pc_offset_s=lambda: 0.1,
    )
    monkeypatch.setattr(camera_time_sync_controller.time, "monotonic", lambda: 100.1)
    monkeypatch.setattr(camera_time_sync_controller.time, "time", lambda: 1000.1)

    ready, reason = CameraTimeSyncController(panel)._camera_sync_readiness()

    assert ready is True
    assert "diagnostic-only" in reason


def test_sync_health_robust_window_tolerates_one_sr_outlier(monkeypatch) -> None:
    from Workstation import camera_time_sync_controller
    from Workstation.camera_time_sync_controller import CameraTimeSyncController

    left = _observer_rows("left", offset_s=0.0)
    right = _observer_rows("right", offset_s=0.08)
    left[1]["sr_pc_offset_s"] = 5.0
    left[1]["sr_pc_offset_raw_s"] = 5.0
    panel = SimpleNamespace(
        camera_sync_status="verified",
        _rtcp_clock_observer_generation=2,
        _rtcp_clock_observation_history={"left": left, "right": right},
        _camera_sync_last_sr_counts={},
        _camera_name_ko=lambda camera: camera,
        _camera_sync_required_sr_count=lambda: 2,
        _camera_sync_max_sr_age_s=lambda: 7.0,
        _camera_sync_max_clock_rate_error=lambda: 0.005,
        _camera_sync_rtcp_pair_max_delta_s=lambda: 0.1,
        _camera_sync_max_pc_offset_s=lambda: 0.1,
    )
    monkeypatch.setattr(camera_time_sync_controller.time, "monotonic", lambda: 100.1)
    monkeypatch.setattr(camera_time_sync_controller.time, "time", lambda: 1000.1)

    ready, reason = CameraTimeSyncController(panel)._camera_sync_readiness()

    assert ready is True
    assert "relative clock Δt" in reason


def test_frame_metrics_keep_sr_pc_absolute_value_diagnostic_only() -> None:
    from Workstation.camera_time_sync_controller import CameraTimeSyncController

    panel = SimpleNamespace(
        _camera_rtc_timestamp_s_from_metadata=lambda row: row.get("rtcp_frame_ntp_unix_s"),
        _camera_name_ko=lambda camera: camera,
        _camera_sync_rtcp_pair_max_delta_s=lambda: 0.1,
        _camera_sync_max_pc_offset_s=lambda: 0.1,
    )
    left = {
        "timestamp_s": 10.0,
        "rtcp_frame_ntp_unix_s": 32_500.0,
        "rtcp_sr_pc_offset_s": 32_400.0,
    }
    right = {
        "timestamp_s": 10.01,
        "rtcp_frame_ntp_unix_s": 32_500.08,
        "rtcp_sr_pc_offset_s": 32_400.08,
    }

    metrics = CameraTimeSyncController(panel)._camera_time_sync_metrics(
        left,
        right,
        now_wall_s=100.0,
        now_mono_s=10.1,
    )

    assert metrics["status"] == "within-relative-threshold"
    assert metrics["pair_delta_s"] == pytest.approx(0.08)
    assert metrics["max_sr_pc_offset_ms"] == pytest.approx(32_400_080.0)
    assert "gate가 아닙니다" in metrics["warning"]


def test_configured_sr_age_target_governs_without_silent_clamp(monkeypatch) -> None:
    from Workstation import camera_time_sync_controller
    from Workstation.camera_time_sync_controller import CameraTimeSyncController

    metadata = {
        "left": _observer_rows("left", received_monotonic_s=92.0),
        "right": _observer_rows("right", received_monotonic_s=92.0),
    }
    configured_target = {"seconds": 10.0}
    panel = SimpleNamespace(
        camera_sync_status="verified",
        _rtcp_clock_observer_generation=2,
        _rtcp_clock_observation_history=metadata,
        _camera_sync_last_sr_counts={},
        _camera_name_ko=lambda camera: camera,
        _camera_sync_required_sr_count=lambda: 2,
        _camera_sync_max_sr_age_s=lambda: configured_target["seconds"],
        _camera_sync_max_clock_rate_error=lambda: 0.005,
        _camera_sync_rtcp_pair_max_delta_s=lambda: 0.05,
        _camera_sync_max_pc_offset_s=lambda: 0.1,
    )
    monkeypatch.setattr(camera_time_sync_controller.time, "monotonic", lambda: 100.1)
    monkeypatch.setattr(camera_time_sync_controller.time, "time", lambda: 1000.1)
    controller = CameraTimeSyncController(panel)

    ready_at_ten, _reason = controller._camera_sync_readiness()
    configured_target["seconds"] = 7.0
    ready_at_seven, reason_at_seven = controller._camera_sync_readiness()

    assert ready_at_ten is True
    assert ready_at_seven is False
    assert "SR stale 8.10s" in reason_at_seven


def test_time_workspace_surfaces_continuous_verified_to_failed_transition(
    workstation_qapplication,
) -> None:
    from Workstation.tests.workstation_test_support import _FakeBackend
    from Workstation.workspaces import TimeWorkspace

    backend = _FakeBackend()
    backend.panel.clock_sync_status_label = QtWidgets.QLabel("sync verified")
    backend.panel.clock_sync_summary_text = QtWidgets.QPlainTextEdit("healthy")
    backend.panel.camera_sync_status = "verified"
    backend.panel.camera_sync_reason = "current RTCP observations verified"
    backend.panel.frame_timing_status = "not-verified"
    backend.panel._camera_integrity_unavailable_sides = {"left", "right"}
    backend.panel._stus_schedule_snapshot = lambda: {
        "mode": "auto",
        "state": "ARMED",
        "remaining_s": 125.0,
    }
    workspace = TimeWorkspace(backend)
    workspace.show()
    workstation_qapplication.processEvents()

    sr_target = workspace.settings_widgets["camera_sync_max_sr_age_s"]
    sr_target.setValue(10.0)
    assert workspace.settings_payload()["camera_sync_max_sr_age_s"] == 10.0
    sr_target.setValue(7.0)
    assert workspace.settings_payload()["camera_sync_max_sr_age_s"] == 7.0

    workspace.refresh({})
    assert workspace.sync_reason.isHidden()
    assert "AUTO · ARMED · next 02:05" in workspace.schedule_status.text()
    assert "Clock VERIFIED" in workspace.schedule_status.text()
    assert "Frame timing NOT-VERIFIED" in workspace.schedule_status.text()
    assert "Media UNAVAILABLE" in workspace.schedule_status.text()

    backend.panel.clock_sync_status_label.setText("sync failed")
    backend.panel.clock_sync_summary_text.setPlainText("left/right frames frozen")
    backend.panel.camera_sync_status = "failed"
    backend.panel.camera_sync_reason = "left/right frame stale 8.00s"
    workspace.refresh({})

    assert not workspace.sync_reason.isHidden()
    assert "frame stale 8.00s" in workspace.sync_reason.text()
    assert "left/right frames frozen" in workspace.sync_summary.toPlainText()
    workspace.close()
    workspace.deleteLater()
    workstation_qapplication.processEvents()


def test_post_ack_stus_verification_keeps_video_and_record_controls_available(
    workstation_qapplication,
) -> None:
    from Workstation.main_window import WorkstationMainWindow
    from Workstation.tests.workstation_test_support import _FakeBackend

    backend = _FakeBackend()
    backend.panel.wiris_time_sync_thread = None
    backend.panel._camera_sync_stus_operation = {
        "active": True,
        "phase": "observing-attempt-1",
    }
    window = WorkstationMainWindow(backend=backend)
    window.show()

    snapshot = window.status_adapter.model.snapshot()
    window._refresh_from_status(snapshot)
    window.advanced.refresh(snapshot)
    workstation_qapplication.processEvents()

    assert not window.commands.action("time.set_pc").isEnabled()
    assert not window.time_workspace.operation_buttons["time.set_pc"].isEnabled()
    assert window.commands.action("recording.toggle").isEnabled()
    assert window.advanced.operation_buttons["recording.toggle"].isEnabled()

    backend.panel._camera_sync_stus_operation["active"] = False
    snapshot = window.status_adapter.model.snapshot()
    window._refresh_from_status(snapshot)
    window.advanced.refresh(snapshot)
    workstation_qapplication.processEvents()

    assert window.commands.action("time.set_pc").isEnabled()
    assert window.time_workspace.operation_buttons["time.set_pc"].isEnabled()
    assert window.commands.action("recording.toggle").isEnabled()
    assert window.advanced.operation_buttons["recording.toggle"].isEnabled()

    window.close()
    window.deleteLater()
    workstation_qapplication.processEvents()


def test_observer_preflight_dispatches_stus_only_after_both_sides_have_sr() -> None:
    from Workstation.camera_time_sync_controller import CameraTimeSyncController

    histories = {"left": deque(maxlen=16), "right": deque(maxlen=16)}
    dispatched: list[bool] = []
    operation = {
        "active": True,
        "phase": "observer-preflight",
    }

    def dispatch() -> None:
        dispatched.append(True)
        operation["phase"] = "initial-command"

    panel = SimpleNamespace(
        _rtcp_clock_observer_generation=3,
        _rtcp_clock_observation_history=histories,
        _camera_sync_stus_operation=operation,
        _camera_sync_required_sr_count=lambda: 2,
        _dispatch_wiris_time_sync_initial=dispatch,
    )
    controller = CameraTimeSyncController(panel)

    def emit(side: str, identity: int) -> None:
        controller._handle_rtcp_clock_observation(
            {
                "schema": "workstation-rtcp-clock-observation-v1",
                "side": side,
                "observer_generation": 3,
                "ssrc": 101 if side == "left" else 202,
                "ntp_raw": identity,
            }
        )

    emit("left", 1)
    assert dispatched == []
    emit("right", 1)
    assert dispatched == [True]


def test_frame_clock_epoch_decoration_marks_transition_then_post_ack_current() -> None:
    from Workstation.camera_time_sync_controller import CameraTimeSyncController

    panel = SimpleNamespace(
        _camera_sync_stus_operation={
            "active": True,
            "clock_epoch_id": "clock-2",
        },
        _last_wiris_pc_utc_sync=None,
        frame_timing_status_by_side={"left": "not-verified", "right": "not-verified"},
        frame_timing_status="not-verified",
        frame_timing_reason="",
    )
    controller = CameraTimeSyncController(panel)

    transition = controller._decorate_clock_epoch_metadata(
        "left",
        {"rtcp_sr_received_wall_s": 100.0},
    )
    assert transition["clock_epoch_state"] == "transition"

    panel._camera_sync_stus_operation = None
    panel._last_wiris_pc_utc_sync = {
        "result": "success",
        "clock_epoch_id": "clock-2",
        "closed_loop_operation": {
            "last_stus_ack_finished_utc_by_camera": {
                "left": 101.0,
                "right": 102.0,
            }
        },
    }
    left = controller._decorate_clock_epoch_metadata(
        "left",
        {"rtcp_sr_received_wall_s": 101.1},
    )
    right = controller._decorate_clock_epoch_metadata(
        "right",
        {"rtcp_sr_received_wall_s": 102.1},
    )
    assert left["clock_epoch_state"] == "current"
    assert right["clock_epoch_state"] == "current"
    assert panel.frame_timing_status == "current"


def test_stus_events_are_hash_bound_and_linked_to_active_run() -> None:
    import hashlib
    import json

    from Workstation.wiris_control_controller import WirisControlController

    global_events: list[dict[str, object]] = []
    run_events: list[tuple[str, dict[str, object]]] = []
    panel = SimpleNamespace(
        _camera_sync_stus_operation={
            "operation_id": "stus-1",
            "clock_epoch_id": "clock-1",
            "trigger": "auto",
            "phase": "observer-preflight",
            "attempts_started": 1,
        },
        _measurement_id="measurement-1",
        append_workstation_data=lambda stream, payload, **_kwargs: global_events.append(
            {"stream": stream, **dict(payload)}
        ),
        append_workstation_run_event=lambda kind, payload, **_kwargs: run_events.append(
            (kind, dict(payload))
        ),
        _log=lambda _text: None,
    )

    WirisControlController(panel)._append_stus_event(
        "camera_clock_sync_requested",
        {"target": "both"},
    )

    [event] = global_events
    digest = str(event.pop("event_sha256"))
    event.pop("stream")
    canonical = json.dumps(
        event,
        ensure_ascii=False,
        allow_nan=False,
        sort_keys=True,
        separators=(",", ":"),
    ).encode("utf-8")
    assert digest == hashlib.sha256(canonical).hexdigest()
    assert event["operation_id"] == "stus-1"
    assert event["clock_epoch_id"] == "clock-1"
    assert run_events[0][0] == "camera_clock_sync_requested"


def test_panel_rtcp_observer_qthread_starts_and_stops_without_leaking(
    monkeypatch,
    tmp_path,
    workstation_qapplication,
) -> None:
    from Workstation import standalone_gui
    from Workstation.workstation_core.rtcp_clock_observer import RtcpClockObserverWorker

    class Probe:
        def __init__(self, side: str) -> None:
            self.side = side

        def start(self) -> None:
            return None

        def poll(self) -> list[dict[str, object]]:
            return [
                {
                    "ssrc": 101 if self.side == "left" else 202,
                    "ntp_raw": 1,
                    "received_monotonic_s": time.monotonic(),
                }
            ]

        def stop(self) -> None:
            return None

    monkeypatch.setattr(
        standalone_gui.WirisStereoDistancePanel,
        "_initial_status",
        lambda self: None,
    )
    monkeypatch.setattr(
        standalone_gui.WirisStereoDistancePanel,
        "_auto_load_intrinsics",
        lambda self: None,
    )
    panel = standalone_gui.WirisStereoDistancePanel(app_root=tmp_path)
    panel._rtcp_clock_observer_worker_factory = (
        lambda targets, observer_generation: RtcpClockObserverWorker(
            targets,
            observer_generation=observer_generation,
            probe_factory=lambda side, **_kwargs: Probe(side),
            poll_interval_s=0.01,
        )
    )

    assert panel._start_rtcp_clock_observer(operation_id="") is True
    deadline = time.monotonic() + 2.0
    while time.monotonic() < deadline and not all(
        panel._rtcp_clock_observation_history[side] for side in ("left", "right")
    ):
        workstation_qapplication.processEvents()
        time.sleep(0.005)
    assert all(panel._rtcp_clock_observation_history[side] for side in ("left", "right"))

    panel._stop_rtcp_clock_observer()
    deadline = time.monotonic() + 2.0
    while time.monotonic() < deadline and panel._rtcp_clock_observer_thread is not None:
        workstation_qapplication.processEvents()
        time.sleep(0.005)
    assert panel._rtcp_clock_observer_thread is None
    assert panel._rtcp_clock_observer_worker is None

    panel.close()
    panel.deleteLater()
    workstation_qapplication.processEvents()
