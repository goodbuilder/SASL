"""Observed-clock scheduling without ROS, MAVLink or historical catch-up."""
from pathlib import Path
from types import SimpleNamespace
import sys
import threading
import time
import pytest
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from campaigns.stereo_v1_1.science_dispatch import ScienceDispatcher
from campaigns.stereo_v1_1.ros_observation_bridge import RosObservationBridge


def snap(ms=0, epoch=0, reset=0):
    return SimpleNamespace(simulation_time_ns=round(ms*1e6), simulation_epoch=epoch,
                           xy_reset_counter=reset, z_reset_counter=0)


def dispatcher():
    sent, keepalive = [], []
    d = ScienceDispatcher(snap(), 1, 50,
                          lambda s, elapsed, host: sent.append((s.simulation_time_ns, elapsed, host)),
                          lambda: keepalive.append(True), 0)
    return d, sent, keepalive


def test_bursty_clock_callbacks_dispatch_exact_observed_20ms_steps():
    d, sent, keep = dispatcher()
    for ms in range(0, 105, 4):
        # Bursty arrival with gaps; simulation progression itself is unchanged.
        d.on_clock(snap(ms), (ms//24)*24_000_000 + (ms%24)*100_000)
    assert [s[0] for s in sent] == [i*20_000_000 for i in range(6)]
    assert not keep


def test_jump_is_one_actual_update_not_historical_catchup():
    d, sent, _ = dispatcher()
    d.on_clock(snap(), 0)
    d.on_clock(snap(80), 80_000_000)
    assert [s[0] for s in sent] == [0, 80_000_000]


def test_paused_clock_keepalive_is_separate_and_stale_times_out():
    d, sent, keep = dispatcher()
    d.on_clock(snap(), 0)
    d.on_clock(snap(), 20_000_000)
    d.watchdog(20_000_000)
    assert len(sent) == 1 and len(keep) == 1
    d.on_clock(None, 500_000_000)
    with pytest.raises(RuntimeError, match='stale'):
        d.watchdog(1_020_000_001)


@pytest.mark.parametrize('bad', [snap(20, epoch=1), snap(20, reset=1), snap(-4)])
def test_clock_and_local_resets_fail(bad):
    d, _, _ = dispatcher()
    d.on_clock(snap(), 0)
    with pytest.raises(RuntimeError):
        d.on_clock(bad, 20_000_000)


def test_finish_stops_callbacks_and_keepalives():
    d, sent, keep = dispatcher()
    d.on_clock(snap(), 0)
    d.on_clock(snap(1000), 1_000_000_000)
    d.watchdog(3_000_000_000)
    d.on_clock(snap(1020), 3_000_000_000)
    assert d.finished and len(sent) == 1 and not keep


def bridge_without_ros():
    b = RosObservationBridge.__new__(RosObservationBridge)
    b.diagnostic_sink = None
    b.diagnostic_error_count = 0
    b._clock_handler_lock = threading.RLock()
    b._clock_handler = None
    b.last_error = None
    b.cache = SimpleNamespace(observe_clock=lambda stamp, *args: None, snapshot=lambda host: snap())
    return b


def test_removal_waits_for_active_callback_and_prevents_future_calls():
    b = bridge_without_ros()
    entered, release, removed = threading.Event(), threading.Event(), threading.Event()
    def callback(snapshot, host):
        entered.set()
        assert release.wait(2)
    b.set_clock_handler(callback)
    msg = SimpleNamespace(clock=SimpleNamespace(sec=0, nanosec=0))
    worker = threading.Thread(target=b._clock, args=(msg,))
    worker.start()
    assert entered.wait(2)
    remover = threading.Thread(target=lambda: (b.set_clock_handler(None), removed.set()))
    remover.start()
    assert not removed.wait(.02)
    release.set()
    worker.join(2)
    remover.join(2)
    assert removed.is_set()
    entered.clear()
    b._clock(msg)
    assert not entered.is_set()


def test_callback_exception_is_propagated_and_handler_can_be_removed():
    b = bridge_without_ros()
    def fail(snapshot, host):
        raise RuntimeError('clock callback failed')
    b.set_clock_handler(fail)
    b._clock(SimpleNamespace(clock=SimpleNamespace(sec=0, nanosec=0)))
    assert b.last_error == 'RuntimeError: clock callback failed'
    b.set_clock_handler(None)
    assert b._clock_handler is None


def test_observed_jitter_does_not_shift_all_future_reference_deadlines():
    d, sent, _ = dispatcher()
    for ms in [0, 24, 40, 60]:
        d.on_clock(snap(ms), ms*1_000_000)
    assert [s[0] for s in sent] == [0, 24_000_000, 40_000_000, 60_000_000]


def test_dispatch_diagnostics_preserve_updates_and_explain_rejections():
    d, sent, keep = dispatcher()
    records = []
    d.diagnostic_sink = records.append
    d.on_clock(snap(), 0)
    d.on_clock(snap(4), 4_000_000)
    d.on_clock(None, 8_000_000)
    d.watchdog(20_000_000)
    assert [r['action'] for r in records] == ['reference_update', 'within_slot', 'snapshot_rejected', 'keepalive']
    assert len(sent) == 1 and len(keep) == 1
    assert records[1]['slot'] == 0
    assert all(r['dispatch_lock_wait_ns'] >= 0 for r in records)
    with pytest.raises(RuntimeError, match='reset'):
        d.on_clock(snap(24, epoch=1), 24_000_000)
    assert records[-1]['action'] == 'error'
    assert 'reset' in records[-1]['error']


def test_clock_diagnostic_contains_source_receipt_and_authoritative_rejection():
    from campaigns.stereo_v1_1.runtime_observation import ObservationCache
    b = bridge_without_ros()
    b.cache = ObservationCache()
    records = []
    b.diagnostic_sink = records.append
    seen = []
    b.set_clock_handler(lambda snapshot, received: seen.append(snapshot))
    b._clock(SimpleNamespace(clock=SimpleNamespace(sec=90, nanosec=304_000_000)))
    assert seen == [None]
    assert records[0]['source_timestamp_ns'] == 90_304_000_000
    assert records[0]['snapshot_rejection_reasons'] == ['missing_gt', 'missing_local']
    assert records[0]['handler_lock_wait_ns'] >= 0
    assert records[0]['callback_end_monotonic_ns'] >= records[0]['clock_receive_monotonic_ns']


def test_diagnostic_sink_failure_is_counted_without_changing_dispatch():
    def fail(record):
        raise RuntimeError('diagnostics unavailable')
    d, sent, _ = dispatcher()
    d.diagnostic_sink = fail
    d.on_clock(snap(), 0)
    assert len(sent) == 1 and d.diagnostic_error_count == 1
    b = bridge_without_ros()
    b.diagnostic_sink = fail
    b._clock(SimpleNamespace(clock=SimpleNamespace(sec=0, nanosec=0)))
    assert b.last_error is None and b.diagnostic_error_count == 1
