from __future__ import annotations

import copy

import pytest

from Workstation.qualify_camera_backpressure import (
    _minimum_saved_pairs,
    _qualification_checks,
    _rss_sample_summary,
    run_qualification,
)


def _passing_gate_inputs() -> dict[str, object]:
    camera = {
        "exitcode": 0,
        "stop_latency_s": 0.01,
        "forced_termination": False,
        "queue_depth_supported": True,
        "queues": {"preview": 1, "metadata": 4, "control": 8},
        "ring": {"dropped": 0, "active_leases": 0},
    }
    return {
        "camera_results": {
            "left": copy.deepcopy(camera),
            "right": copy.deepcopy(camera),
        },
        "writer_metrics": {
            "saved": 10,
            "failed": 0,
            "pending": 0,
            "frame_unavailable": 0,
            "queue_full": 0,
            "unacknowledged_results": 0,
        },
        "writer_result_summary": {"invalid": 0, "outstanding": 0},
        "minimum_saved_pairs": 10,
        "writer_close_ok": True,
        "writer_thread_stopped": True,
        "pairer_close_ok": True,
        "leaked_child_pids": [],
        "leaked_shared_memory_names": [],
        "cleanup_errors": [],
        "rss_summary": {
            "growth_bytes": 10,
            "high_water_bytes": 100,
            "slope_bytes_per_hour": 20,
        },
        "max_rss_growth_bytes": 10,
        "max_rss_high_water_bytes": 100,
        "max_rss_slope_bytes_per_hour": 20,
        "max_stop_latency_s": 0.5,
    }


def test_qualification_gate_accepts_only_a_complete_clean_result() -> None:
    passed, checks, failures = _qualification_checks(**_passing_gate_inputs())

    assert passed is True
    assert all(checks.values())
    assert failures == []


@pytest.mark.parametrize(
    ("mutation", "failed_check"),
    (
        (lambda data: data["camera_results"].pop("right"), "camera_sides_present"),
        (
            lambda data: data["camera_results"]["left"].update(exitcode=1),
            "camera_processes_exited_cleanly",
        ),
        (
            lambda data: data["camera_results"]["left"].update(
                forced_termination=True
            ),
            "camera_processes_exited_cleanly",
        ),
        (
            lambda data: data["camera_results"]["left"].update(
                stop_latency_s=0.51
            ),
            "stop_latency_within_limit",
        ),
        (
            lambda data: data["camera_results"]["left"].update(
                queue_depth_supported=False
            ),
            "queues_within_capacity",
        ),
        (
            lambda data: data["camera_results"]["left"]["queues"].update(
                preview=2
            ),
            "queues_within_capacity",
        ),
        (
            lambda data: data["camera_results"]["left"]["ring"].update(
                dropped=1
            ),
            "native_rings_lossless",
        ),
        (
            lambda data: data["camera_results"]["left"]["ring"].update(
                active_leases=1
            ),
            "native_rings_lossless",
        ),
        (lambda data: data["writer_metrics"].update(failed=1), "evidence_no_failures"),
        (
            lambda data: data["writer_metrics"].update(pending=1),
            "evidence_pending_drained",
        ),
        (lambda data: data["writer_metrics"].update(frame_unavailable=1), "evidence_no_frame_unavailable"),
        (lambda data: data["writer_metrics"].update(queue_full=1), "evidence_queue_not_full"),
        (
            lambda data: data["writer_metrics"].update(unacknowledged_results=1),
            "evidence_results_acknowledged",
        ),
        (lambda data: data["writer_metrics"].update(saved=9), "evidence_saved_minimum"),
        (
            lambda data: data["writer_result_summary"].update(invalid=1),
            "evidence_results_valid",
        ),
        (
            lambda data: data["writer_result_summary"].update(outstanding=1),
            "evidence_results_collected",
        ),
        (lambda data: data.update(writer_close_ok=False), "writer_closed"),
        (lambda data: data.update(writer_thread_stopped=False), "writer_thread_stopped"),
        (lambda data: data.update(pairer_close_ok=False), "pairer_thread_stopped"),
        (lambda data: data.update(leaked_child_pids=[1234]), "no_active_child_leaks"),
        (
            lambda data: data.update(leaked_shared_memory_names=["psm-leaked"]),
            "no_shared_memory_leaks",
        ),
        (
            lambda data: data.update(cleanup_errors=["queue close failed"]),
            "cleanup_error_free",
        ),
        (
            lambda data: data["rss_summary"].update(growth_bytes=11),
            "rss_growth_within_limit",
        ),
        (
            lambda data: data["rss_summary"].update(high_water_bytes=101),
            "rss_high_water_within_limit",
        ),
        (
            lambda data: data["rss_summary"].update(slope_bytes_per_hour=21),
            "rss_slope_within_limit",
        ),
    ),
)
def test_each_required_false_pass_condition_fails_closed(mutation, failed_check: str) -> None:
    inputs = _passing_gate_inputs()
    mutation(inputs)

    passed, checks, failures = _qualification_checks(**inputs)

    assert passed is False
    assert checks[failed_check] is False
    assert failed_check in failures


def test_default_600_second_saved_pair_expectation_keeps_headroom() -> None:
    required, theoretical = _minimum_saved_pairs(
        duration_s=600.0,
        evidence_period_s=2.0,
        configured=None,
        minimum_ratio=0.90,
    )

    assert theoretical == 299
    assert required == 269
    assert _minimum_saved_pairs(
        duration_s=600.0,
        evidence_period_s=2.0,
        configured=296,
        minimum_ratio=0.90,
    ) == (296, 299)


def test_rss_summary_reports_threshold_inputs_and_bounded_raw_edges() -> None:
    summary = _rss_sample_summary(
        [(10.0, 100), (11.0, 130), (12.0, 120), (13.0, 125)]
    )

    assert summary["sample_count"] == 4
    assert summary["growth_bytes"] == 25
    assert summary["high_water_bytes"] == 130
    assert summary["high_water_growth_bytes"] == 30
    assert len(summary["samples"]) == 4
    assert summary["first_samples"][0] == {"elapsed_s": 0.0, "rss_bytes": 100}
    assert summary["last_samples"][-1] == {"elapsed_s": 3.0, "rss_bytes": 125}


def test_short_real_qualification_drains_results_and_releases_resources() -> None:
    result = run_qualification(
        duration_s=2.0,
        gui_stall_s=0.0,
        evidence_period_s=0.4,
        minimum_saved_pairs=1,
        max_rss_growth_bytes=2**62,
        max_rss_high_water_bytes=2**62,
        max_rss_slope_bytes_per_hour=2**62,
        rss_sample_interval_s=0.1,
    )

    assert result["passed"] is True, result
    assert result["evidence"]["frame_unavailable"] == 0
    assert result["evidence"]["queue_full"] == 0
    assert result["evidence"]["unacknowledged_results"] == 0
    assert result["evidence"]["saved"] >= 1
    assert result["cleanup"] == {
        "writer_close_ok": True,
        "writer_thread_stopped": True,
        "pairer_close_ok": True,
        "active_child_pids": [],
        "leaked_shared_memory_names": [],
        "errors": [],
    }
