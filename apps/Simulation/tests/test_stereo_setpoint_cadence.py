"""Reject wall-rate claims that conceal simulation-time undersampling."""
import sys
from pathlib import Path
import pytest
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from campaigns.stereo_v1_1.setpoint_cadence import summarize_setpoint_cadence


def rows(count=500, host_step=20_000_000, sim_step=20_000_000):
    return [dict(host_monotonic_ns=i*host_step, simulation_timestamp_ns=i*sim_step,
                 simulation_epoch=0) for i in range(count)]


def test_50hz_simulation_evidence_and_window():
    result = summarize_setpoint_cadence(rows(), 50, 0, 10_000_000_000)
    assert result['sufficient_unique_simulation_cadence']
    assert result['host_hz'] == pytest.approx(50)
    assert result['unique_simulation_hz'] == pytest.approx(50)
    assert result['real_time_factor'] == pytest.approx(1)


def test_wall_50hz_at_rtf_two_is_not_simulation_50hz():
    result = summarize_setpoint_cadence(rows(250, sim_step=40_000_000), 50, 0, 10_000_000_000)
    assert result['host_hz'] == pytest.approx(50)
    assert result['unique_simulation_hz'] == pytest.approx(25)
    assert result['real_time_factor'] == pytest.approx(2)
    assert not result['sufficient_unique_simulation_cadence']


def test_paused_clock_duplicates_are_not_new_simulation_samples():
    data = rows()
    data[100:200] = [dict(row, simulation_timestamp_ns=data[99]['simulation_timestamp_ns']) for row in data[100:200]]
    result = summarize_setpoint_cadence(data, 50, 0, 10_000_000_000)
    assert result['duplicate_simulation_time_count'] == 100
    assert not result['sufficient_unique_simulation_cadence']


def test_unknown_clock_and_reset_cannot_pass():
    data = rows()
    data[-1]['simulation_epoch'] = 1
    data[50]['simulation_timestamp_ns'] = None
    result = summarize_setpoint_cadence(data, 50, 0, 10_000_000_000)
    assert result['unknown_simulation_time_count'] == 1
    assert result['simulation_epoch_count'] == 2
    assert not result['sufficient_unique_simulation_cadence']


def test_fast_short_fragment_does_not_cover_science_window():
    assert not summarize_setpoint_cadence(rows(50), 50, 0, 10_000_000_000)['sufficient_unique_simulation_cadence']


def test_dense_bursts_cannot_hide_internal_outage():
    data = rows()
    # Preserve count, endpoints and mean rate, but move one sample into the
    # next reference slot, leaving its original complete slot empty.
    data[250]["simulation_timestamp_ns"] += 22_000_000
    data[251]["simulation_timestamp_ns"] += 4_000_000
    result = summarize_setpoint_cadence(data, 50, 0, 10_000_000_000)
    assert result["science_window_covered"]
    assert result["unique_simulation_hz"] == pytest.approx(50)
    assert result["max_simulation_gap_s"] == pytest.approx(0.042)
    assert not result["sufficient_unique_simulation_cadence"]


@pytest.mark.parametrize("hz", [0, -1, float("nan"), float("inf")])
def test_invalid_requested_rate_is_rejected(hz):
    with pytest.raises(ValueError, match="positive and finite"):
        summarize_setpoint_cadence(rows(), hz)


def test_keepalive_does_not_inflate_reference_cadence_or_unknown_clock_count():
    data = rows()
    for row in data:
        row['reference_update'] = True
    for i in range(1, len(data), 2):
        data[i]['reference_update'] = False
        data[i]['simulation_timestamp_ns'] = None
    result = summarize_setpoint_cadence(data, 50, 0, 10_000_000_000)
    assert result['host_hz'] == pytest.approx(50)
    assert result['unique_simulation_hz'] == pytest.approx(25)
    assert result['keepalive_count'] == 250
    assert result['unknown_simulation_time_count'] == 0
    assert not result['sufficient_unique_simulation_cadence']


def test_32ms_boundary_jitter_with_complete_slot_coverage_passes():
    data = rows()
    data[250]['simulation_timestamp_ns'] += 12_000_000
    result = summarize_setpoint_cadence(data, 50, 0, 10_000_000_000)
    assert result['max_simulation_gap_s'] == pytest.approx(.032)
    assert result['diagnostic_simulation_gap_exceeded']
    assert result['missing_full_reference_slot_count'] == 0
    assert result['sufficient_unique_simulation_cadence']
    assert result['criterion'] == 'reference_slot_coverage_v2'


def test_last_full_interval_missing_fails_despite_early_extra_sample():
    data = rows()
    data[-1]['simulation_timestamp_ns'] = 9_979_000_000
    result = summarize_setpoint_cadence(data, 50, 0, 10_000_000_000)
    assert result['missing_full_reference_slots'] == [499]
    assert not result['sufficient_unique_simulation_cadence']


def test_incomplete_final_interval_is_optional_but_full_interval_is_not():
    data = rows()
    result = summarize_setpoint_cadence(data, 50, 0, 10_010_000_000)
    assert result['expected_full_reference_slots'] == 500
    assert result['partial_final_reference_interval_ns'] == 10_000_000
    assert result['sufficient_unique_simulation_cadence']
    result = summarize_setpoint_cadence(data, 50, 0, 10_020_000_000)
    assert result['missing_full_reference_slots'] == [500]
    assert not result['sufficient_unique_simulation_cadence']


@pytest.mark.parametrize('offset,missing', [(-1, 1), (0, 0), (1, 0)])
def test_exact_slot_boundary_and_one_nanosecond_neighbors(offset, missing):
    data = rows()
    data[250]['simulation_timestamp_ns'] += offset
    result = summarize_setpoint_cadence(data, 50, 0, 10_000_000_000)
    assert result['missing_full_slot_count'] == missing
    assert result['sufficient_unique_simulation_cadence'] == (missing == 0)


@pytest.mark.parametrize('stamp', [None, -1, 10_000_000_000])
def test_invalid_extra_reference_cannot_pass_even_with_complete_slots(stamp):
    data = rows()
    data.append(dict(host_monotonic_ns=10_000_000_000,
                     simulation_timestamp_ns=stamp, simulation_epoch=0))
    result = summarize_setpoint_cadence(data, 50, 0, 10_000_000_000)
    assert result['missing_full_slot_count'] == 0
    assert not result['sufficient_unique_simulation_cadence']
