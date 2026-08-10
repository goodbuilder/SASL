from pathlib import Path
import sys

PACKAGE_ROOT = Path(__file__).resolve().parents[1] / "stereo_ground_sim"
sys.path.insert(0, str(PACKAGE_ROOT))

from stereo_ground_sim.runtime_worker_lifecycle import RuntimeWorkerLifecycle


class Thread:
    def __init__(self, *, target, daemon):
        self.target = target
        self.daemon = daemon
        self.started = False
        self.joined = []

    def start(self):
        self.started = True

    def is_alive(self):
        return self.started

    def join(self, timeout):
        self.joined.append(timeout)


def test_start_and_join_owns_thread_lifecycle_by_worker_role():
    lifecycle = RuntimeWorkerLifecycle(Thread)
    target = lambda: None
    assert lifecycle.start("measurement", target) is True
    thread = lifecycle.threads["measurement"]
    assert thread.target is target
    assert thread.daemon is True
    assert lifecycle.join("measurement", 1.5) is True
    assert thread.joined == [1.5]


def test_disabled_worker_has_explicit_none_state():
    lifecycle = RuntimeWorkerLifecycle(Thread)
    assert lifecycle.start("rviz", lambda: None, enabled=False) is False
    assert lifecycle.threads["rviz"] is None
    assert lifecycle.join("rviz", 2.0) is False
