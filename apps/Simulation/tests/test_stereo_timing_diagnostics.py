"""Bounded evidence is explicit about loss and cannot overwrite prior records."""
import json
import sys
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from campaigns.stereo_v1_1.timing_diagnostics import BufferedTimingDiagnostics


def test_diagnostic_callbacks_do_not_write_until_persist_and_loss_is_explicit(tmp_path):
    sink = BufferedTimingDiagnostics(capacity=2)
    sink({'component': 'observer', 'source_timestamp_ns': 90304000000})
    sink({'component': 'dispatcher', 'action': 'snapshot_rejected'})
    sink({'component': 'watchdog'})
    assert list(tmp_path.iterdir()) == []
    summary = sink.persist(tmp_path)
    assert not summary['complete']
    assert summary['dropped_record_count'] == 1
    rows = [json.loads(s) for s in (tmp_path / summary['file']).read_text().splitlines()]
    assert len(rows) == 2
    assert rows[0]['source_timestamp_ns'] == 90304000000
    with pytest.raises(FileExistsError):
        sink.persist(tmp_path)


def test_diagnostic_concurrent_producers_preserve_counts(tmp_path):
    sink = BufferedTimingDiagnostics(capacity=150)
    with ThreadPoolExecutor(max_workers=2) as pool:
        list(pool.map(lambda n: sink({'sequence': n}), range(200)))
    summary = sink.persist(tmp_path)
    assert summary['record_count'] == 150
    assert summary['dropped_record_count'] == 50
    assert len({row['sequence'] for row in sink.rows}) == 150


def test_development_postprocess_cannot_collide_with_main_attempt(tmp_path, monkeypatch):
    from campaigns.stereo_v1_1 import campaign_run as runner
    main = tmp_path / "파생_재처리/G0-P3-R01/a01"
    main.mkdir(parents=True)
    marker = main / "existing.txt"
    marker.write_text("original")
    attempt = tmp_path / "개발시험/runner_G0-P3-R01_a01"
    attempt.mkdir(parents=True)
    outputs = []
    def stop_before_yolo(command, *args, **kwargs):
        if "--output-dir" in command:
            outputs.append(Path(command[command.index("--output-dir") + 1]))
            raise RuntimeError("stop before YOLO")
    monkeypatch.setattr(runner, "_run_checked", stop_before_yolo)
    with pytest.raises(RuntimeError, match="stop before YOLO"):
        runner._postprocess(tmp_path, attempt, "G0-P3-R01", "a01", "P3", True)
    assert marker.read_text() == "original"
    assert outputs[0].is_relative_to(tmp_path / "개발시험")
    assert not outputs[0].is_relative_to(main)


@pytest.mark.parametrize("verified", [True, False])
def test_completeness_requires_producer_shutdown_readback(tmp_path, verified):
    sink = BufferedTimingDiagnostics()
    sink({"component": "diagnostic_producers", "observer_shutdown_verified": verified})
    summary = sink.persist(tmp_path)
    assert summary["record_accounting_complete"]
    assert summary["complete"] == verified


@pytest.mark.parametrize("executor_stopped,thread_alive", [(True, False), (False, False), (True, True)])
def test_shutdown_timeout_cannot_be_reported_as_verified(executor_stopped, thread_alive):
    import threading
    from types import SimpleNamespace as Obj
    from campaigns.stereo_v1_1.ros_observation_bridge import RosObservationBridge
    bridge = RosObservationBridge.__new__(RosObservationBridge)
    bridge._clock_handler_lock = threading.RLock()
    bridge._clock_handler = None
    bridge.executor = Obj(shutdown=lambda **kw: executor_stopped)
    bridge.thread = Obj(join=lambda **kw: None, is_alive=lambda: thread_alive)
    bridge.node = Obj(destroy_node=lambda: None)
    bridge._rclpy = Obj(ok=lambda: False)
    bridge.close()
    assert bridge.diagnostic_shutdown_complete == (executor_stopped and not thread_alive)


def test_stage_wall_and_cpu_evidence_preserves_errors():
    from campaigns.stereo_v1_1.timing_diagnostics import timed_stage
    sink = BufferedTimingDiagnostics()
    with timed_stage(sink, 'mavlink_send', 123_132_000_000):
        sum(range(100))
    row = sink.rows[-1]
    assert row['action'] == 'mavlink_send'
    assert row['source_timestamp_ns'] == 123_132_000_000
    assert row['wall_duration_ns'] >= row['thread_cpu_duration_ns'] >= 0
    with pytest.raises(ValueError, match='original'):
        with timed_stage(sink, 'prepare_reference_and_ned', 0):
            raise ValueError('original')
    assert sink.rows[-1]['error_type'] == 'ValueError'
    with timed_stage(None, 'disabled', 0):
        pass


def test_gc_observer_lifecycle_keeps_policy_and_handles_reentrant_collector_lock(tmp_path):
    import gc
    sink = BufferedTimingDiagnostics()
    policy = (gc.isenabled(), gc.get_threshold())
    before_callbacks = list(gc.callbacks)
    sink.start_gc_tracking()
    hook = sink._gc_hook
    sink.start_gc_tracking()
    assert gc.callbacks.count(hook) == 1
    # A real collection while this thread holds the collector lock must not deadlock.
    with sink.lock:
        gc.collect(0)
    assert {'start', 'stop'} <= {r['action'] for r in sink.rows if r['component'] == 'python_gc'}
    sink.persist(tmp_path)
    assert gc.callbacks == before_callbacks
    assert (gc.isenabled(), gc.get_threshold()) == policy
    assert sink._gc_hook is None
