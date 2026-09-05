from __future__ import annotations

import pytest

from Workstation.workstation_core.stus_scheduler import (
    STUS_MODE_AUTO,
    STUS_MODE_MANUAL,
    StusScheduler,
    normalize_stus_interval_s,
    normalize_stus_mode,
)


def test_scheduler_defaults_and_bounds_are_backward_compatible() -> None:
    assert normalize_stus_mode(None) == STUS_MODE_MANUAL
    assert normalize_stus_mode("AUTO") == STUS_MODE_AUTO
    assert normalize_stus_mode("unexpected") == STUS_MODE_MANUAL
    assert normalize_stus_interval_s(None) == pytest.approx(1800.0)
    assert normalize_stus_interval_s(1.0) == pytest.approx(300.0)
    assert normalize_stus_interval_s(100_000.0) == pytest.approx(86_400.0)


def test_auto_requires_manual_verified_seed_for_each_connection_generation() -> None:
    scheduler = StusScheduler(mode="auto", interval_s=600.0)
    scheduler.reset_for_connection(7)

    assert scheduler.poll(now_monotonic_s=1000.0, operation_active=False) is None
    scheduler.mark_verified(
        now_monotonic_s=1000.0,
        connection_generation=7,
        trigger="manual",
    )
    assert scheduler.poll(now_monotonic_s=1599.9, operation_active=False) is None
    assert scheduler.poll(now_monotonic_s=1600.0, operation_active=False) == "auto"

    scheduler.reset_for_connection(8)
    assert scheduler.poll(now_monotonic_s=5000.0, operation_active=False) is None
    assert scheduler.snapshot(now_monotonic_s=5000.0)["state"] == "WAITING_MANUAL"


def test_manual_mode_detects_drift_without_scheduling_a_command() -> None:
    scheduler = StusScheduler(mode="manual", interval_s=600.0)
    scheduler.reset_for_connection(3)
    scheduler.mark_verified(
        now_monotonic_s=10.0,
        connection_generation=3,
        trigger="manual",
    )
    scheduler.mark_clock_revoked("offset exceeded")

    assert scheduler.poll(now_monotonic_s=10_000.0, operation_active=False) is None
    snapshot = scheduler.snapshot(now_monotonic_s=10_000.0)
    assert snapshot["state"] == "REVOKED"
    assert snapshot["reason"] == "offset exceeded"


def test_active_operation_coalesces_one_due_action_and_failure_waits_full_interval() -> None:
    scheduler = StusScheduler(mode="auto", interval_s=300.0)
    scheduler.reset_for_connection(1)
    scheduler.mark_verified(
        now_monotonic_s=100.0,
        connection_generation=1,
        trigger="manual",
    )

    assert scheduler.poll(now_monotonic_s=400.0, operation_active=True) is None
    assert scheduler.snapshot(now_monotonic_s=400.0)["state"] == "OVERDUE"
    assert scheduler.poll(now_monotonic_s=401.0, operation_active=False) == "auto"
    assert scheduler.poll(now_monotonic_s=402.0, operation_active=False) is None

    scheduler.mark_finished(
        now_monotonic_s=450.0,
        trigger="auto",
        verified=False,
        reason="observer timeout",
    )
    assert scheduler.poll(now_monotonic_s=749.9, operation_active=False) is None
    assert scheduler.poll(now_monotonic_s=750.0, operation_active=False) == "auto"


def test_policy_change_preserves_current_run_seed_but_cancels_future_manual_schedule() -> None:
    scheduler = StusScheduler(mode="manual", interval_s=1800.0)
    scheduler.reset_for_connection(4)
    scheduler.mark_verified(
        now_monotonic_s=100.0,
        connection_generation=4,
        trigger="manual",
    )
    scheduler.set_policy(mode="auto", interval_s=600.0)
    assert scheduler.snapshot(now_monotonic_s=200.0)["next_due_monotonic_s"] == pytest.approx(700.0)

    scheduler.set_policy(mode="manual", interval_s=600.0)
    assert scheduler.poll(now_monotonic_s=10_000.0, operation_active=False) is None
    assert scheduler.snapshot(now_monotonic_s=10_000.0)["next_due_monotonic_s"] is None
