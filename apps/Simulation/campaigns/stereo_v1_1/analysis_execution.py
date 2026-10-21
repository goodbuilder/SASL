"""Bounded process execution; coordinator alone owns ordering and output lock.

Workers receive independent immutable task payloads. This helper never changes
scientific outcomes, retries failed tasks, deletes artifacts, or freezes results.
"""
from __future__ import annotations

from collections import deque
from concurrent.futures import ProcessPoolExecutor
from contextlib import contextmanager
import fcntl
import multiprocessing
import os
from pathlib import Path
import threading


_ENV_LOCK = threading.Lock()
_THREAD_VARIABLES = ('OPENBLAS_NUM_THREADS', 'OMP_NUM_THREADS', 'MKL_NUM_THREADS')


def ordered_task_results(worker, tasks, *, workers=1, max_in_flight=None):
    """Yield worker(payload) in submission order with bounded outstanding tasks.

    Task keys must be hashable and unique for this invocation. Closing the
    iterator, or a task/generator error, cancels unstarted work and joins running
    children before returning/raising. Consumers that stop early must close the
    iterator (e.g. contextlib.closing); do not leave a live iterator abandoned.
    """
    if isinstance(workers, bool) or not isinstance(workers, int) or workers < 1:
        raise ValueError('workers must be a positive integer')
    limit = 2 * workers if max_in_flight is None else max_in_flight
    if isinstance(limit, bool) or not isinstance(limit, int) or limit < 1:
        raise ValueError('max_in_flight must be a positive integer')
    seen = set()
    source = iter(tasks)

    def next_task():
        key, payload = next(source)
        if key in seen:
            raise ValueError(f'duplicate analysis task key: {key!r}')
        seen.add(key)
        return payload

    if workers == 1:
        while True:
            try:
                payload = next_task()
            except StopIteration:
                return
            yield worker(payload)
        return

    # spawn imports child modules before executor initializers run. Keep these
    # settings in the parent throughout the executor lifetime, including lazy
    # child creation; never modify environment concurrently with another pool.
    if not _ENV_LOCK.acquire(blocking=False):
        raise RuntimeError('another analysis process coordinator owns the environment')
    old_environment = {name: os.environ.get(name) for name in _THREAD_VARIABLES}
    executor = None
    pending = deque()
    try:
        os.environ.update({name: '1' for name in _THREAD_VARIABLES})
        executor = ProcessPoolExecutor(max_workers=workers,
                                       mp_context=multiprocessing.get_context('spawn'))
        exhausted = False
        while pending or not exhausted:
            # Check completed failures before accepting any further work, even
            # if that future follows a slower task in the output order.
            for future in pending:
                if future.done():
                    future.result()
            while not exhausted and len(pending) < limit:
                for future in pending:
                    if future.done():
                        future.result()
                try:
                    payload = next_task()
                except StopIteration:
                    exhausted = True
                    break
                for future in pending:
                    if future.done():
                        future.result()
                pending.append(executor.submit(worker, payload))
            if pending:
                # Keep the awaited future registered until result() succeeds,
                # so exceptional shutdown accounts for every submitted task.
                result = pending[0].result()
                pending.popleft()
                yield result
    finally:
        try:
            for future in pending:
                future.cancel()
            if executor is not None:
                executor.shutdown(wait=True, cancel_futures=True)
        finally:
            for name, previous in old_environment.items():
                if previous is None:
                    os.environ.pop(name, None)
                else:
                    os.environ[name] = previous
            _ENV_LOCK.release()


@contextmanager
def analysis_output_lock(output_root):
    """One nonblocking coordinator lock per output root, including pointer writes."""
    root = Path(output_root).resolve()
    root.mkdir(parents=True, exist_ok=True)
    with (root / '.analysis.lock').open('a', encoding='utf-8') as handle:
        try:
            fcntl.flock(handle.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError as exc:
            raise RuntimeError(f'analysis output already has an active coordinator: {root}') from exc
        try:
            yield root
        finally:
            fcntl.flock(handle.fileno(), fcntl.LOCK_UN)
