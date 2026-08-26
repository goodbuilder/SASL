"""Focused STUS ordering and sustained RTCP/NTP health regressions."""

from __future__ import annotations

from types import SimpleNamespace
from collections import deque
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

    assert probe["samples_s"] == pytest.approx([0.934, 0.510, 0.515])
    assert probe["median_s"] == pytest.approx(0.515)
    assert probe["warnings"]
    assert probe_timeouts == pytest.approx([5.0, 5.0, 5.0])
    assert worker._stus_compensation_s(probe["median_s"]) == pytest.approx(0.2595)


def test_stus_field_apply_latency_uses_dedicated_bounded_response_timeout(monkeypatch) -> None:
    """An 8.9 s camera apply ACK must fit without changing generic 5 s control I/O."""

    from Workstation import wiris_runtime_workers

    monotonic_values = iter((0.0, 8.9, 9.0, 9.0, 17.9))
    command_timeouts: list[float] = []
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


def test_stus_closed_loop_formula_uses_signed_sr_pc_offset() -> None:
    from Workstation.camera_time_sync_controller import (
        stus_closed_loop_compensation_s,
    )

    # SR-PC is camera NTP minus PC receive UTC.  A negative observation means
    # the camera is behind, so the next absolute STUS target must move forward.
    # The field capture used the old 250 ms rejection policy, so its previous
    # lead was 0 s.  Independent pad-probe SR medians were -736.135/-1307.044 ms.
    assert stus_closed_loop_compensation_s(
        previous_compensation_s=0.0,
        sr_pc_offset_s=-0.736135482,
    ) == pytest.approx(0.736135482)
    assert stus_closed_loop_compensation_s(
        previous_compensation_s=0.0,
        sr_pc_offset_s=-1.307044029,
    ) == pytest.approx(1.307044029)
    assert stus_closed_loop_compensation_s(
        previous_compensation_s=0.5,
        sr_pc_offset_s=0.2,
    ) == pytest.approx(0.3)
    with pytest.raises(ValueError, match="limit"):
        stus_closed_loop_compensation_s(
            previous_compensation_s=0.1,
            sr_pc_offset_s=-2.1,
        )


def test_stus_correction_requires_three_unique_stable_pad_probe_sender_reports(
    monkeypatch,
) -> None:
    from Workstation import camera_time_sync_controller
    from Workstation.camera_time_sync_controller import CameraTimeSyncController

    current: dict[str, dict[str, object]] = {}
    operation = {
        "active": True,
        "phase": "observing-attempt-1",
        "attempts_started": 1,
        "max_attempts": 2,
        "last_compensation_by_camera": {"left": 0.0, "right": 0.0},
        "feedback_attempt_index": 1,
        "sr_feedback_samples_by_camera": {"left": [], "right": []},
        "pre_reconnect_stream_generation_by_camera": {"left": 1, "right": 1},
        "last_stus_ack_finished_utc_by_camera": {"left": 999.0, "right": 999.0},
    }
    panel = SimpleNamespace(
        camera_connected=True,
        _camera_sync_stus_operation=operation,
        _camera_sync_stus_sr_baseline={"left": 0, "right": 0},
        _camera_sync_history_window_s=lambda: 1.0,
        _camera_sync_required_sr_count=lambda: 2,
        _camera_sync_max_sr_age_s=lambda: 7.0,
        _camera_sync_max_clock_rate_error=lambda: 0.005,
        _camera_sync_receive_pair_max_delta_s=lambda: 0.35,
        _camera_sync_max_pc_offset_s=lambda: 0.1,
        _camera_sync_rtcp_pair_max_delta_s=lambda: 0.05,
        _camera_name_ko=lambda camera: camera,
        _clock_sync_pair_metadata=lambda **_kwargs: (
            dict(current["left"]),
            dict(current["right"]),
        ),
        _camera_time_sync_metrics=lambda *_args, **_kwargs: {
            "pair_delta_s": -0.5709,
            "receive_pair_delta_s": receive_delta["value"],
            "max_sr_pc_offset_ms": 1307.0,
        },
    )
    monkeypatch.setattr(camera_time_sync_controller.time, "monotonic", lambda: 100.1)
    monkeypatch.setattr(camera_time_sync_controller.time, "time", lambda: 1000.1)
    controller = CameraTimeSyncController(panel)
    states: list[str] = []
    # The first two SRs are old-generation. The third has a bad receive pair;
    # the fourth reuses that exact SR under a valid pair and must count only
    # then. Only the final three distinct admissible identities drive mutation.
    receive_delta = {"value": 0.010}
    left_offsets = (-0.9, -0.8, -0.75, -0.75, -0.7362, -0.7361)
    right_offsets = (-1.5, -1.4, -1.32, -1.32, -1.3071, -1.3070)

    for index, (left_offset, right_offset) in enumerate(
        zip(left_offsets, right_offsets, strict=True),
        start=1,
    ):
        current["left"] = {
            "timestamp_s": 100.0,
            "rtcp_sr_received_count": index + 1,
            "rtcp_sr_age_s": 0.1,
            "rtcp_sr_received_wall_s": 1000.0,
            "rtcp_sr_ntp_raw": 103 if index in {3, 4} else 100 + index,
            "rtcp_sr_arrival_basis": "rtcp-pad-probe",
            "rtcp_sr_pc_offset_s": left_offset,
            "rtcp_camera_ntp_rate": 1.0,
            "rtcp_camera_media_rate": 1.0,
            "stream_generation": 1 if index <= 2 else 2,
        }
        current["right"] = {
            **current["left"],
            "rtcp_sr_ntp_raw": 203 if index in {3, 4} else 200 + index,
            "rtcp_sr_pc_offset_s": right_offset,
        }
        receive_delta["value"] = 0.500 if index == 3 else 0.010
        state, evidence = controller._camera_sync_correction_observation()
        states.append(state)

    assert states == ["waiting", "waiting", "waiting", "waiting", "waiting", "candidate"]
    assert evidence["sr_pc_offset_by_camera"]["left"] == pytest.approx(-0.7362)
    assert evidence["sr_pc_offset_by_camera"]["right"] == pytest.approx(-1.3071)
    assert evidence["next_compensation_by_camera"]["right"] == pytest.approx(1.3071)
    assert evidence["offset_statistics_by_camera"]["right"]["unique_samples"] == 3
    assert len(operation["sr_feedback_samples_by_camera"]["left"]) == 3
    assert len(operation["sr_feedback_samples_by_camera"]["right"]) == 3


def test_stus_correction_never_acts_on_poll_detected_sender_report(monkeypatch) -> None:
    from Workstation import camera_time_sync_controller
    from Workstation.camera_time_sync_controller import CameraTimeSyncController

    metadata = {
        "timestamp_s": 100.0,
        "rtcp_sr_received_count": 3,
        "rtcp_sr_age_s": 0.1,
        "rtcp_sr_received_wall_s": 1000.0,
        "rtcp_sr_ntp_raw": 123,
        "rtcp_sr_arrival_basis": "poll-detected",
        "rtcp_sr_pc_offset_s": -0.7,
        "rtcp_camera_ntp_rate": 1.0,
        "rtcp_camera_media_rate": 1.0,
        "stream_generation": 2,
    }
    operation = {
        "active": True,
        "phase": "observing-attempt-1",
        "attempts_started": 1,
        "last_compensation_by_camera": {"left": 0.0, "right": 0.0},
        "pre_reconnect_stream_generation_by_camera": {"left": 1, "right": 1},
        "last_stus_ack_finished_utc_by_camera": {"left": 999.0, "right": 999.0},
    }
    panel = SimpleNamespace(
        camera_connected=True,
        _camera_sync_stus_operation=operation,
        _camera_sync_stus_sr_baseline={"left": 0, "right": 0},
        _camera_sync_history_window_s=lambda: 1.0,
        _camera_sync_required_sr_count=lambda: 2,
        _camera_sync_max_sr_age_s=lambda: 7.0,
        _camera_sync_max_clock_rate_error=lambda: 0.005,
        _camera_sync_receive_pair_max_delta_s=lambda: 0.35,
        _camera_sync_max_pc_offset_s=lambda: 0.1,
        _camera_sync_rtcp_pair_max_delta_s=lambda: 0.05,
        _camera_name_ko=lambda camera: camera,
        _clock_sync_pair_metadata=lambda **_kwargs: (dict(metadata), dict(metadata)),
        _camera_time_sync_metrics=lambda *_args, **_kwargs: {
            "pair_delta_s": 0.0,
            "receive_pair_delta_s": 0.0,
            "max_sr_pc_offset_ms": 700.0,
        },
    )
    monkeypatch.setattr(camera_time_sync_controller.time, "monotonic", lambda: 100.1)
    monkeypatch.setattr(camera_time_sync_controller.time, "time", lambda: 1000.1)

    state, evidence = CameraTimeSyncController(panel)._camera_sync_correction_observation()

    assert state == "waiting"
    assert "pad-probe" in evidence["reason"]
    assert operation["sr_feedback_samples_by_camera"] == {"left": [], "right": []}


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
    assert probe_hosts == ["10.0.0.231", "10.0.0.232"]
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
    assert probe_hosts == ["10.0.0.231", "10.0.0.232"]
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
    assert results[0]["compensation_method"] == "rtcp-sr-closed-loop"


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
        "compensation_method": "rtcp-sr-closed-loop",
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
                "stus_compensation_source": "rtcp-sr-closed-loop",
                "stus_feedback_offset_s": 0.300,
            },
            "right": {
                "host": "10.0.0.232",
                "response": "OK",
                "command": "STUS 1001 0",
                "query_finished_utc_s": 1003.0,
                "stus_compensation_s": 0.500,
                "stus_compensation_source": "rtcp-sr-closed-loop",
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
    assert panel._last_wiris_pc_utc_sync["result"] == "pending-verification"
    assert _stus_reference({"pc_utc_sync": panel._last_wiris_pc_utc_sync}) is None
    assert "signed SR-PC=+300.00ms" in panel.clock_sync_summary_text.text

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
                    "next_compensation_by_camera": {"left": 0.736135482, "right": 1.307044029},
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
    assert correction_requests[0]["compensation_by_camera"]["right"] == pytest.approx(1.307044029)
    assert panel.camera_sync_status == CAMERA_SYNC_STATUS_WARMING

    operation["phase"] = "observing-attempt-2"
    controller._poll_camera_sync_state()

    assert panel.camera_sync_status == CAMERA_SYNC_STATUS_VERIFIED
    assert len(correction_requests) == 1
    assert finishes == [("verified", "converged")]


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


def test_stus_correction_is_aborted_if_recording_starts(monkeypatch) -> None:
    from Workstation import camera_time_sync_controller
    from Workstation.camera_time_sync_controller import (
        CAMERA_SYNC_STATUS_FAILED,
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
        _request_wiris_time_sync_correction=lambda **payload: mutations.append(payload),
        _finish_wiris_time_sync_operation=lambda *_args: None,
    )
    panel._set_camera_sync_state = lambda status, reason: (
        setattr(panel, "camera_sync_status", status),
        setattr(panel, "camera_sync_reason", reason),
    )
    monkeypatch.setattr(camera_time_sync_controller.time, "monotonic", lambda: 100.0)

    CameraTimeSyncController(panel)._poll_camera_sync_state()

    assert panel.camera_sync_status == CAMERA_SYNC_STATUS_FAILED
    assert "recording" in panel.camera_sync_reason.lower() or "기록" in panel.camera_sync_reason
    assert mutations == []


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


def test_verified_sync_rejects_dual_frozen_frames_and_regressed_sr(monkeypatch) -> None:
    from Workstation import camera_time_sync_controller
    from Workstation.camera_time_sync_controller import CameraTimeSyncController

    frozen = {
        "timestamp_s": 100.0,
        "rtcp_frame_ntp_unix_s": 1000.0,
        "rtcp_sr_received_count": 4,
        "rtcp_sr_age_s": 0.2,
        "rtcp_sr_received_wall_s": 999.8,
        "rtcp_sr_pc_offset_s": 0.010,
        "rtcp_sr_arrival_basis": "rtcp-pad-probe",
        "rtcp_camera_ntp_rate": 1.0,
        "rtcp_camera_media_rate": 1.0,
        "stream_generation": 2,
    }
    panel = SimpleNamespace(
        camera_connected=True,
        camera_sync_status="verified",
        camera_frame_metadata_history={},
        _camera_sync_stus_sr_baseline={},
        _camera_sync_last_sr_counts={"left": 5, "right": 5},
        _camera_name_ko=lambda camera: camera,
        _camera_sync_history_window_s=lambda: 1.0,
        _camera_sync_required_sr_count=lambda: 2,
        _camera_sync_max_sr_age_s=lambda: 7.0,
        _camera_sync_max_clock_rate_error=lambda: 0.005,
        _camera_sync_rtcp_pair_max_delta_s=lambda: 0.05,
        _camera_sync_receive_pair_max_delta_s=lambda: 0.35,
        _camera_sync_max_pc_offset_s=lambda: 0.1,
        _clock_sync_pair_metadata=lambda **_kwargs: (dict(frozen), dict(frozen)),
        _camera_time_sync_metrics=lambda *_args, **_kwargs: {
            "pair_delta_s": 0.0,
            "receive_pair_delta_s": 0.0,
            "max_sr_pc_offset_ms": 10.0,
        },
    )
    monkeypatch.setattr(camera_time_sync_controller.time, "monotonic", lambda: 108.0)
    monkeypatch.setattr(camera_time_sync_controller.time, "time", lambda: 1008.0)

    ready, reason = CameraTimeSyncController(panel)._camera_sync_readiness()

    assert ready is False
    assert "frame stale 8.00s" in reason
    assert "SR stale 8.20s" in reason
    assert "SR counter regressed 5→4" in reason


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
        ("pair-delta", "RTCP/NTP Δt=50.1ms"),
        ("sr-pc-offset", "SR/PC UTC offset=100.1ms"),
        ("ntp-rate", "NTP clock=1.005100"),
        ("media-rate", "RTP clock=0.994900"),
        ("missing-sr", "SR 0/2"),
        ("right-offset-missing", "right signed SR-PC offset 없음"),
    ),
)
def test_sync_health_hard_limits_fail_closed(
    monkeypatch,
    mutation: str,
    expected_reason: str,
) -> None:
    from Workstation import camera_time_sync_controller
    from Workstation.camera_time_sync_controller import CameraTimeSyncController

    left = {
        "timestamp_s": 100.0,
        "rtcp_frame_ntp_unix_s": 1000.0,
        "rtcp_sr_received_count": 2,
        "rtcp_sr_age_s": 0.1,
        "rtcp_sr_received_wall_s": 999.9,
        "rtcp_sr_pc_offset_s": 0.010,
        "rtcp_sr_arrival_basis": "rtcp-pad-probe",
        "rtcp_camera_ntp_rate": 1.0,
        "rtcp_camera_media_rate": 1.0,
    }
    right = dict(left)
    metrics = {
        "pair_delta_s": 0.0,
        "receive_pair_delta_s": 0.0,
        "max_sr_pc_offset_ms": 10.0,
    }
    if mutation == "pair-delta":
        metrics["pair_delta_s"] = 0.0501
    elif mutation == "sr-pc-offset":
        metrics["max_sr_pc_offset_ms"] = 100.1
    elif mutation == "ntp-rate":
        left["rtcp_camera_ntp_rate"] = 1.0051
    elif mutation == "media-rate":
        left["rtcp_camera_media_rate"] = 0.9949
    elif mutation == "missing-sr":
        left["rtcp_sr_received_count"] = 0
    elif mutation == "right-offset-missing":
        right.pop("rtcp_sr_pc_offset_s")

    panel = SimpleNamespace(
        camera_connected=True,
        camera_sync_status="verified",
        camera_frame_metadata_history={},
        _camera_sync_stus_sr_baseline={},
        _camera_sync_last_sr_counts={},
        _camera_name_ko=lambda camera: camera,
        _camera_sync_history_window_s=lambda: 1.0,
        _camera_sync_required_sr_count=lambda: 2,
        _camera_sync_max_sr_age_s=lambda: 7.0,
        _camera_sync_max_clock_rate_error=lambda: 0.005,
        _camera_sync_rtcp_pair_max_delta_s=lambda: 0.05,
        _camera_sync_receive_pair_max_delta_s=lambda: 0.35,
        _camera_sync_max_pc_offset_s=lambda: 0.1,
        _clock_sync_pair_metadata=lambda **_kwargs: (dict(left), dict(right)),
        _camera_time_sync_metrics=lambda *_args, **_kwargs: dict(metrics),
    )
    monkeypatch.setattr(camera_time_sync_controller.time, "monotonic", lambda: 100.1)
    monkeypatch.setattr(camera_time_sync_controller.time, "time", lambda: 1000.1)

    ready, reason = CameraTimeSyncController(panel)._camera_sync_readiness()

    assert ready is False
    assert expected_reason in reason


def test_configured_sr_age_target_governs_without_silent_clamp(monkeypatch) -> None:
    from Workstation import camera_time_sync_controller
    from Workstation.camera_time_sync_controller import CameraTimeSyncController

    metadata = {
        "timestamp_s": 100.0,
        "rtcp_frame_ntp_unix_s": 1000.0,
        "rtcp_sr_received_count": 2,
        "rtcp_sr_age_s": 8.0,
        "rtcp_sr_received_wall_s": 992.0,
        "rtcp_sr_pc_offset_s": 0.010,
        "rtcp_sr_arrival_basis": "rtcp-pad-probe",
        "rtcp_camera_ntp_rate": 1.0,
        "rtcp_camera_media_rate": 1.0,
    }
    configured_target = {"seconds": 10.0}
    panel = SimpleNamespace(
        camera_connected=True,
        camera_sync_status="verified",
        camera_frame_metadata_history={},
        _camera_sync_stus_sr_baseline={},
        _camera_sync_last_sr_counts={},
        _camera_name_ko=lambda camera: camera,
        _camera_sync_history_window_s=lambda: 1.0,
        _camera_sync_required_sr_count=lambda: 2,
        _camera_sync_max_sr_age_s=lambda: configured_target["seconds"],
        _camera_sync_max_clock_rate_error=lambda: 0.005,
        _camera_sync_rtcp_pair_max_delta_s=lambda: 0.05,
        _camera_sync_receive_pair_max_delta_s=lambda: 0.35,
        _camera_sync_max_pc_offset_s=lambda: 0.1,
        _clock_sync_pair_metadata=lambda **_kwargs: (dict(metadata), dict(metadata)),
        _camera_time_sync_metrics=lambda *_args, **_kwargs: {
            "pair_delta_s": 0.0,
            "receive_pair_delta_s": 0.0,
            "max_sr_pc_offset_ms": 10.0,
        },
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


def test_post_ack_stus_verification_keeps_time_set_and_manual_record_disabled(
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
    assert not window.commands.action("recording.toggle").isEnabled()
    assert not window.advanced.operation_buttons["recording.toggle"].isEnabled()

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
