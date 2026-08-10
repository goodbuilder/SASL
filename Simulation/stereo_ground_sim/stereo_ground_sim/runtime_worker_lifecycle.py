"""Thread creation and bounded shutdown ownership for node workers."""


class RuntimeWorkerLifecycle:
    def __init__(self, thread_factory):
        self.thread_factory = thread_factory
        self.threads = {"measurement": None, "rviz": None, "plot": None}

    def start(self, name, target, *, enabled=True):
        if not enabled:
            self.threads[name] = None
            return False
        thread = self.thread_factory(target=target, daemon=True)
        self.threads[name] = thread
        thread.start()
        return True

    def join(self, name, timeout_s):
        thread = self.threads.get(name)
        if thread is None or not thread.is_alive():
            return False
        thread.join(timeout=float(timeout_s))
        return True

    def join_all(self, timeout_s=2.0):
        return {
            name: self.join(name, timeout_s)
            for name in tuple(self.threads)
        }
