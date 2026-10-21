"""Science-send clock evidence, independent of trajectory coordinates and control.

Host time is monotonic ns; simulation timestamps are /clock ns in an epoch.
Repeated simulation timestamps indicate paused/unadvanced observations, not new
simulation-time samples. No rate conversion or scheduling policy is applied.
"""
from __future__ import annotations

import math


def summarize_setpoint_cadence(rows: list[dict], required_hz: float,
                               science_start_ns: int | None = None,
                               science_end_ns: int | None = None) -> dict:
    if not math.isfinite(required_hz) or required_hz <= 0:
        raise ValueError('required_hz must be positive and finite')
    hosts = [r['host_monotonic_ns'] for r in rows]
    updates = [r for r in rows if r.get('reference_update', True)]
    known = [r for r in updates if r['simulation_timestamp_ns'] is not None]
    epochs = {r['simulation_epoch'] for r in known}
    unadvanced_host_s = sum(
        (b['host_monotonic_ns'] - a['host_monotonic_ns']) / 1e9
        for a, b in zip(known, known[1:])
        if a['simulation_epoch'] == b['simulation_epoch']
        and a['simulation_timestamp_ns'] == b['simulation_timestamp_ns'])
    unique = []
    duplicates = backwards = 0
    for row in known:
        if unique and row['simulation_epoch'] == unique[-1]['simulation_epoch']:
            delta = row['simulation_timestamp_ns'] - unique[-1]['simulation_timestamp_ns']
            if delta == 0:
                duplicates += 1
                continue
            if delta < 0:
                backwards += 1
        unique.append(row)
    gaps = [(b['simulation_timestamp_ns'] - a['simulation_timestamp_ns']) / 1e9
            for a, b in zip(unique, unique[1:])
            if a['simulation_epoch'] == b['simulation_epoch']]
    host_s = (hosts[-1] - hosts[0]) / 1e9 if len(hosts) > 1 else 0
    sim_s = ((unique[-1]['simulation_timestamp_ns'] - unique[0]['simulation_timestamp_ns']) / 1e9
             if len(unique) > 1 and len(epochs) == 1 else 0)
    known_host_s = ((unique[-1]['host_monotonic_ns'] - unique[0]['host_monotonic_ns']) / 1e9
                    if len(unique) > 1 else 0)
    window_s = ((science_end_ns - science_start_ns) / 1e9
                if science_start_ns is not None and science_end_ns is not None else None)
    # v2 follows the 50 Hz reference grid: every COMPLETE anchored interval
    # must contain an observed reference update. Boundary jitter may make an
    # adjacent pair >1.5 periods apart without missing either reference slot.
    # Only an incomplete final interval is optional; interior holes never are.
    period_ns = round(1e9 / required_hz)
    if period_ns < 1:
        raise ValueError('required_hz exceeds nanosecond clock resolution')
    window_ns = (science_end_ns - science_start_ns
                 if science_start_ns is not None and science_end_ns is not None else 0)
    full_slot_count = max(0, window_ns // period_ns)
    partial_final_ns = max(0, window_ns % period_ns) if window_ns > 0 else 0
    occupied = {
        (r['simulation_timestamp_ns'] - science_start_ns) // period_ns
        for r in unique
        if science_start_ns is not None and science_end_ns is not None
        and science_start_ns <= r['simulation_timestamp_ns'] < science_end_ns
    }
    out_of_range_count = sum(
        not (science_start_ns <= r['simulation_timestamp_ns'] < science_end_ns)
        for r in unique
    ) if science_start_ns is not None and science_end_ns is not None else len(unique)
    unknown_count = len(updates) - len(known)
    covered_full = {slot for slot in occupied if 0 <= slot < full_slot_count}
    missing_slots = sorted(set(range(full_slot_count)) - covered_full)
    coverage = bool(unique and window_s is not None and window_s > 0
                    and 0 <= unique[0]['simulation_timestamp_ns'] - science_start_ns <= period_ns
                    and 0 <= science_end_ns - unique[-1]['simulation_timestamp_ns'] <= period_ns + partial_final_ns)
    diagnostic_gap_s = 1.5 / required_hz
    sufficient = bool(gaps and full_slot_count > 0 and not missing_slots
                      and unknown_count == 0 and out_of_range_count == 0
                      and len(epochs) == 1 and backwards == 0 and sim_s > 0 and coverage
                      and len(unique) >= required_hz * window_s - 1 - 1e-8)
    return {
        'schema_version': 'science_setpoint_cadence_v2', 'required_hz': required_hz,
        'criterion': 'reference_slot_coverage_v2',
        'supersedes_criterion': 'science_setpoint_cadence_v1_max_gap_1.5_periods',
        'reference_slot_period_ns': period_ns,
        'expected_full_reference_slots': full_slot_count,
        'covered_full_reference_slots': len(covered_full),
        'missing_full_slot_count': len(missing_slots),
        'missing_full_reference_slot_count': len(missing_slots),
        'out_of_range_reference_update_count': out_of_range_count,
        'missing_full_reference_slots': missing_slots,
        'partial_final_reference_interval_ns': partial_final_ns,
        'send_count': len(rows), 'unique_simulation_sample_count': len(unique),
        'unknown_simulation_time_count': len(updates) - len(known),
        'reference_update_count': len(updates), 'keepalive_count': len(rows) - len(updates),
        'clock_receive_to_send_max_age_s': max(
            (r.get('clock_receive_to_send_age_ns') or 0 for r in updates), default=0) / 1e9,
        'duplicate_simulation_time_count': duplicates, 'backwards_simulation_time_count': backwards,
        'simulation_epoch_count': len(epochs),
        'unadvanced_simulation_observed_host_duration_s': unadvanced_host_s,
        'host_hz': (len(rows) - 1) / host_s if host_s > 0 else None,
        'unique_simulation_hz': (len(unique) - 1) / sim_s if sim_s > 0 else None,
        'real_time_factor': sim_s / known_host_s if known_host_s > 0 and sim_s > 0 else None,
        'max_simulation_gap_s': max(gaps) if gaps else None,
        'diagnostic_simulation_gap_threshold_s': diagnostic_gap_s,
        'diagnostic_simulation_gap_exceeded': bool(gaps and max(gaps) > diagnostic_gap_s + 1e-9),
        'jitter_warning': bool(gaps and max(gaps) > diagnostic_gap_s + 1e-9),
        'science_window_duration_s': window_s, 'science_window_covered': coverage,
        'sufficient_unique_simulation_cadence': sufficient,
    }
