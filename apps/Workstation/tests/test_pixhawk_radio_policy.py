from __future__ import annotations

import pytest

from Workstation.workstation_core.pixhawk_radio_policy import (
    RADIO_CAP_LIFT_S,
    RADIO_CRITICAL_STREAM_REQUESTS,
    RadioStreamPolicy,
    RadioStreamTier,
    radio_stream_requests,
)


def test_radio_profile_keeps_critical_rates_and_scales_only_optional() -> None:
    assert radio_stream_requests(RadioStreamTier.CRITICAL) == RADIO_CRITICAL_STREAM_REQUESTS
    tier_25 = dict(radio_stream_requests(RadioStreamTier.OPTIONAL_25))
    tier_50 = dict(radio_stream_requests(RadioStreamTier.OPTIONAL_50))
    stock = dict(radio_stream_requests(RadioStreamTier.STOCK))

    assert tier_25["ATTITUDE"] == pytest.approx(5.0)
    assert tier_25["GPS_RTK"] == pytest.approx(1.0)
    assert tier_25["SCALED_IMU"] == pytest.approx(5.0)
    assert tier_50["SCALED_IMU"] == pytest.approx(10.0)
    assert stock["SCALED_IMU"] == pytest.approx(20.0)
    assert stock["VFR_HUD"] == pytest.approx(5.0)


def test_radio_recovery_uses_exact_30_and_15_second_boundaries() -> None:
    policy = RadioStreamPolicy()
    assert not policy.observe(qualified=True, healthy=True, now_s=100.0)
    assert not policy.observe(qualified=True, healthy=True, now_s=129.999)
    assert policy.observe(qualified=True, healthy=True, now_s=130.0)
    assert policy.tier is RadioStreamTier.OPTIONAL_25
    assert not policy.observe(qualified=True, healthy=True, now_s=144.999)
    assert policy.observe(qualified=True, healthy=True, now_s=145.0)
    assert policy.tier is RadioStreamTier.OPTIONAL_50
    assert policy.observe(qualified=True, healthy=True, now_s=160.0)
    assert policy.tier is RadioStreamTier.STOCK


def test_radio_regression_immediately_returns_to_critical_and_restarts_timer() -> None:
    policy = RadioStreamPolicy()
    policy.observe(qualified=True, healthy=True, now_s=0.0)
    assert policy.observe(qualified=True, healthy=True, now_s=30.0)
    assert policy.observe(qualified=True, healthy=True, now_s=45.0)
    assert policy.observe(qualified=True, healthy=False, now_s=45.1)
    assert policy.tier is RadioStreamTier.CRITICAL
    assert policy.snapshot(now_s=45.1).healthy_since_s is None
    assert not policy.observe(qualified=True, healthy=True, now_s=50.0)
    assert not policy.observe(qualified=True, healthy=True, now_s=79.999)
    assert policy.observe(qualified=True, healthy=True, now_s=80.0)


def test_radio_regression_caps_escalation_below_the_failed_tier() -> None:
    policy = RadioStreamPolicy()
    policy.observe(qualified=True, healthy=True, now_s=0.0)
    assert policy.observe(qualified=True, healthy=True, now_s=30.0)
    assert policy.observe(qualified=True, healthy=True, now_s=45.0)
    assert policy.tier is RadioStreamTier.OPTIONAL_50
    # ATTITUDE went stale while OPTIONAL_50 was active.
    assert policy.observe(qualified=False, healthy=True, now_s=50.0)
    assert policy.tier is RadioStreamTier.CRITICAL
    assert policy.tier_cap is RadioStreamTier.OPTIONAL_25

    policy.observe(qualified=True, healthy=True, now_s=60.0)
    assert policy.observe(qualified=True, healthy=True, now_s=90.0)
    assert policy.tier is RadioStreamTier.OPTIONAL_25
    # The failed tier is not re-probed after the ordinary 15 s dwell...
    assert not policy.observe(qualified=True, healthy=True, now_s=105.0)
    assert not policy.observe(qualified=True, healthy=True, now_s=90.0 + RADIO_CAP_LIFT_S - 0.001)
    snapshot = policy.snapshot(now_s=105.0)
    assert snapshot.tier_cap is RadioStreamTier.OPTIONAL_25
    assert snapshot.next_transition_in_s == pytest.approx(RADIO_CAP_LIFT_S - 15.0)
    # ...but only after the long healthy dwell, which also lifts the cap.
    assert policy.observe(qualified=True, healthy=True, now_s=90.0 + RADIO_CAP_LIFT_S)
    assert policy.tier is RadioStreamTier.OPTIONAL_50
    assert policy.tier_cap is RadioStreamTier.OPTIONAL_50


def test_radio_regression_at_first_optional_tier_caps_at_critical() -> None:
    policy = RadioStreamPolicy()
    policy.observe(qualified=True, healthy=True, now_s=0.0)
    assert policy.observe(qualified=True, healthy=True, now_s=30.0)
    assert policy.observe(qualified=True, healthy=False, now_s=31.0)
    assert policy.tier_cap is RadioStreamTier.CRITICAL
    policy.observe(qualified=True, healthy=True, now_s=40.0)
    assert not policy.observe(qualified=True, healthy=True, now_s=70.0)
    assert policy.observe(qualified=True, healthy=True, now_s=40.0 + RADIO_CAP_LIFT_S)
    assert policy.tier is RadioStreamTier.OPTIONAL_25


def test_radio_transport_reset_forgets_the_cap() -> None:
    policy = RadioStreamPolicy()
    policy.observe(qualified=True, healthy=True, now_s=0.0)
    policy.observe(qualified=True, healthy=True, now_s=30.0)
    policy.observe(qualified=False, healthy=True, now_s=31.0)
    assert policy.tier_cap is RadioStreamTier.CRITICAL
    policy.reset()
    assert policy.tier_cap is RadioStreamTier.STOCK
    assert policy.snapshot(now_s=31.0).tier_cap is RadioStreamTier.STOCK
