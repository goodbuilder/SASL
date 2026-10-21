from contextlib import closing
import multiprocessing
import os
from pathlib import Path
import sys
import time

import pytest

# This source-only package is not installed into the test interpreter. Resolve
# it from the checkout, independently of pytest's launch directory and spawn.
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from campaigns.stereo_v1_1.analysis_execution import ordered_task_results, analysis_output_lock

# Captured at module import in spawned workers, before an initializer could fix it.
IMPORT_THREADS = {key: os.environ.get(key) for key in
                  ('OPENBLAS_NUM_THREADS', 'OMP_NUM_THREADS', 'MKL_NUM_THREADS')}


def _pure_worker(payload):
    value, delay = payload
    time.sleep(delay)
    return value * value


def _environment_worker(payload):
    return IMPORT_THREADS


def _artifact_worker(payload):
    directory, name, fail, delay = payload
    path = Path(directory) / name
    path.write_text('started')
    time.sleep(delay)
    if fail:
        raise RuntimeError('fixture worker failure')
    path.write_text('finished')
    return name


def _lock_worker(root):
    try:
        with analysis_output_lock(root):
            return 'acquired'
    except RuntimeError:
        return 'contended'


def _children():
    return {child.pid for child in multiprocessing.active_children()}


def test_spawn_ordered_parity_and_preimport_thread_environment(monkeypatch):
    baseline = _children()
    monkeypatch.setenv('OPENBLAS_NUM_THREADS', '7')
    monkeypatch.delenv('MKL_NUM_THREADS', raising=False)
    before = {key: os.environ.get(key) for key in IMPORT_THREADS}
    tasks = [(i, (i, (4-i)*0.03)) for i in range(5)]
    serial = list(ordered_task_results(_pure_worker, tasks))
    assert list(ordered_task_results(_pure_worker, tasks, workers=2)) == serial == [0,1,4,9,16]
    assert list(ordered_task_results(_environment_worker, [('env',None)],workers=2)) == [dict.fromkeys(IMPORT_THREADS,'1')]
    assert {key: os.environ.get(key) for key in IMPORT_THREADS} == before
    assert _children() == baseline


def test_pending_is_bounded_and_explicit_close_joins():
    submitted=[]
    def tasks():
        for i in range(8):
            submitted.append(i)
            yield i,(i,0.01)
    baseline=_children()
    with closing(ordered_task_results(_pure_worker,tasks(),workers=2,max_in_flight=3)) as results:
        assert next(results)==0
        assert submitted==[0,1,2]
    assert _children()==baseline


@pytest.mark.parametrize('workers',[1,2])
def test_duplicate_task_key_is_error(workers):
    baseline=_children()
    with pytest.raises(ValueError,match='duplicate'):
        list(ordered_task_results(_pure_worker,[('same',(1,0)),('same',(2,0))],workers=workers))
    assert _children()==baseline


def test_worker_exception_preserves_partial_artifact_and_joins(tmp_path):
    baseline=_children()
    before={key:os.environ.get(key) for key in IMPORT_THREADS}
    jobs=[('fail',(str(tmp_path),'partial',True,0.05)),
          ('other',(str(tmp_path),'other',False,0.2))]
    with pytest.raises(RuntimeError,match='fixture worker failure'):
        list(ordered_task_results(_artifact_worker,jobs,workers=2,max_in_flight=2))
    assert (tmp_path/'partial').read_text()=='started'
    if (tmp_path/'other').exists():
        assert (tmp_path/'other').read_text()=='finished'
    assert _children()==baseline
    assert {key:os.environ.get(key) for key in IMPORT_THREADS}==before


def test_task_generator_error_stops_and_joins():
    baseline=_children()
    def broken():
        yield 'first',(1,0.05)
        raise ValueError('fixture generator failure')
    with pytest.raises(ValueError,match='generator failure'):
        list(ordered_task_results(_pure_worker,broken(),workers=2))
    assert _children()==baseline


def test_output_lock_contends_across_process_and_releases_on_error(tmp_path):
    (tmp_path/'keep').write_text('untouched')
    with pytest.raises(ValueError,match='body'):
        with analysis_output_lock(tmp_path):
            assert list(ordered_task_results(_lock_worker,[('lock',str(tmp_path))],workers=2))==['contended']
            with pytest.raises(RuntimeError,match='active coordinator'):
                with analysis_output_lock(tmp_path):
                    pass
            raise ValueError('body')
    assert list(ordered_task_results(_lock_worker,[('lock',str(tmp_path))],workers=2))==['acquired']
    assert (tmp_path/'keep').read_text()=='untouched'


@pytest.mark.parametrize('kwargs',[{'workers':0},{'workers':True},{'workers':1.5},{'max_in_flight':0}])
def test_invalid_bounds(kwargs):
    with pytest.raises(ValueError):
        list(ordered_task_results(_pure_worker,[],**kwargs))


def _reverse_completion_worker(payload):
    directory, index = payload
    signal = Path(directory) / 'second_completed'
    if index == 0:
        deadline = time.monotonic() + 10
        while not signal.exists():
            if time.monotonic() > deadline:
                raise RuntimeError('second spawned worker did not run')
            time.sleep(0.005)
        finished = time.monotonic_ns()
    else:
        finished = time.monotonic_ns()
        signal.write_text(str(finished))
    return index, finished


def test_completion_is_reversed_but_yields_submission_order(tmp_path):
    rows = list(ordered_task_results(_reverse_completion_worker,
                [(i,(str(tmp_path),i)) for i in (0,1)],workers=2,max_in_flight=2))
    assert [row[0] for row in rows] == [0,1]
    assert rows[0][1] > rows[1][1]


def test_second_parent_coordinator_rejected_until_first_closes():
    before = {key: os.environ.get(key) for key in IMPORT_THREADS}
    with closing(ordered_task_results(_pure_worker,[(0,(2,0))],workers=2,max_in_flight=1)) as first:
        assert next(first) == 4
        with pytest.raises(RuntimeError,match='coordinator owns the environment'):
            list(ordered_task_results(_pure_worker,[(1,(3,0))],workers=2))
    assert {key: os.environ.get(key) for key in IMPORT_THREADS} == before
