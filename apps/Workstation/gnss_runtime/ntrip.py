"""Own a continuous NTRIP connection and its GUI control-event lifetime.

Reconnect is part of the same session. Raw RTCM envelopes and coalesced stats
remain wired directly to the separately owned data plane; no per-chunk GUI
path is introduced. GGA remains a synchronous current-position input port.
"""

from dataclasses import dataclass, field
from PyQt5 import QtCore
from Workstation.workstation_core.rtcm_pipeline import NtripSessionStats


NTRIP_FIELDS = ("gnss_network_thread", "gnss_network_worker", "_gnss_ntrip_pending_credential")


class NtripStatsForwarder(QtCore.QObject):
    """Tag coalesced status on the producer thread, with Qt lifetime cleanup."""

    forwarded = QtCore.pyqtSignal(object)

    def __init__(self, source, pipeline, session_token):
        super().__init__(source)
        self.session_token = session_token
        source.stats_snapshot_changed.connect(self.forward, QtCore.Qt.DirectConnection)
        self.forwarded.connect(pipeline.ntrip_stats_received)

    @QtCore.pyqtSlot(object)
    def forward(self, stats):
        self.forwarded.emit(NtripSessionStats(self.session_token, stats))


@dataclass
class NtripSession:
    token: int
    thread: object
    worker: object
    pending_credential: object = field(default=None, repr=False)
    relay: object = None
    stop_requested: bool = False
    terminal: bool = False


class NtripSessionState:
    """One authoritative binding, with finite aliases for legacy callers."""

    def __init__(self, publish_legacy=None):
        self.current = None
        self._legacy_thread = None
        self._legacy_worker = None
        self._pending = None
        self._publish_legacy = publish_legacy

    def publish(self):
        if self._publish_legacy is not None:
            self._publish_legacy(self)

    @property
    def gnss_network_thread(self):
        return self.current.thread if self.current is not None else self._legacy_thread

    @gnss_network_thread.setter
    def gnss_network_thread(self, value):
        if self.current is not None:
            if value is not self.current.thread:
                raise RuntimeError("Active NTRIP thread must be retired by its owner")
            return
        self._legacy_thread = value

    @property
    def gnss_network_worker(self):
        return self.current.worker if self.current is not None else self._legacy_worker

    @gnss_network_worker.setter
    def gnss_network_worker(self, value):
        if self.current is not None:
            if value is not self.current.worker:
                raise RuntimeError("Active NTRIP worker must be retired by its owner")
            return
        self._legacy_worker = value

    @property
    def _gnss_ntrip_pending_credential(self):
        return self.current.pending_credential if self.current is not None else self._pending

    @_gnss_ntrip_pending_credential.setter
    def _gnss_ntrip_pending_credential(self, value):
        if self.current is not None:
            self.current.pending_credential = value
        else:
            self._pending = value

    def resource_inventory(self):
        return {name: getattr(self, name) for name in NTRIP_FIELDS[:2]}


class _NtripRelay(QtCore.QObject):
    def __init__(self, owner, session):
        super().__init__(owner)
        self.owner, self.session = owner, session

    @QtCore.pyqtSlot(str, object)
    def connected(self, status, stats):
        self.owner.deliver(self.session, "connected", status, stats)

    @QtCore.pyqtSlot(str)
    def reconnecting(self, message):
        self.owner.deliver(self.session, "reconnecting", message)

    @QtCore.pyqtSlot(str)
    def failed(self, message):
        self.owner.deliver(self.session, "failed", message)

    @QtCore.pyqtSlot(str)
    def diagnostic(self, message):
        self.owner.deliver(self.session, "diagnostic", message)

    @QtCore.pyqtSlot()
    def finished(self):
        self.owner.retire(self.session)


class NtripSessionRuntime(QtCore.QObject):
    def __init__(self, parent, state, *, worker_factory, connected, reconnecting, failed, diagnostic, retired):
        super().__init__(parent)
        self.state = state
        self._worker_factory = worker_factory
        self._callbacks = dict(
            connected=connected, reconnecting=reconnecting, failed=failed, diagnostic=diagnostic, retired=retired
        )
        for name, callback in {"worker_factory": worker_factory, **self._callbacks}.items():
            if not callable(callback):
                raise TypeError(f"Missing NTRIP operation: {name}")
        self._next_token = 0
        self.callback_error = ""

    @property
    def busy(self):
        return self.state.current is not None or self.state.gnss_network_thread is not None

    def start(self, network, credential, gga_provider, *, wire_data_plane):
        if QtCore.QThread.currentThread() is not self.thread():
            raise RuntimeError("NTRIP sessions must start on the Qt owner thread")
        if self.busy:
            raise RuntimeError("NTRIP session is already running")
        worker = self._worker_factory(
            config=network, username=credential.username, password=credential.password, gga_provider=gga_provider
        )
        if not isinstance(worker, QtCore.QObject):
            raise TypeError("NTRIP worker factory must return a QObject")
        for name in ("run", "request_stop"):
            if not callable(getattr(worker, name, None)):
                worker.deleteLater()
                raise TypeError(f"Missing NTRIP worker operation: {name}")
        thread = QtCore.QThread(self)
        self._next_token += 1
        session = NtripSession(
            self._next_token, thread, worker, (network, credential) if credential.source == "session" else None
        )
        relay = _NtripRelay(self, session)
        session.relay = relay
        try:
            thread.started.connect(worker.run)
            for event in ("connected", "reconnecting", "failed", "diagnostic"):
                getattr(worker, event).connect(getattr(relay, event), QtCore.Qt.QueuedConnection)
            # This port wires only the existing locked-queue/data-plane path.
            wire_data_plane(worker, session.token)
            worker.finished.connect(thread.quit)
            worker.finished.connect(worker.deleteLater)
            thread.finished.connect(relay.finished, QtCore.Qt.QueuedConnection)
            worker.moveToThread(thread)
        except Exception:
            worker.deleteLater()
            thread.deleteLater()
            relay.deleteLater()
            raise
        self.state.current = session
        self.state._pending = None
        self.state.publish()
        try:
            thread.start()
            if not thread.isRunning():
                raise RuntimeError("NTRIP thread did not start")
        except Exception:
            self.request_stop()
            thread.quit()
            if not thread.isRunning():
                self.retire(session)
                worker.deleteLater()
            raise
        return session

    def deliver(self, session, event, *args):
        if self.state.current is not session or session.stop_requested or session.terminal:
            return
        if event == "failed":
            session.terminal = True
            session.pending_credential = None
            self.state.publish()
        try:
            self._callbacks[event](*args)
        except Exception as exc:
            # Presentation/secure-store failures cannot abandon the Qt handles.
            self.callback_error = f"{event}: {exc}"

    def take_accepted_credential(self, status_line):
        session = self.state.current
        if session is None or session.stop_requested or session.terminal:
            return None
        upper = str(status_line).upper()
        if " 200 " not in f" {upper} " and not upper.endswith(" 200"):
            return None
        pending, session.pending_credential = session.pending_credential, None
        self.state.publish()
        return pending

    def accepts_stats(self, session_token):
        session = self.state.current
        return bool(
            session is not None
            and session.token == session_token
            and not session.stop_requested
            and not session.terminal
        )

    def request_stop(self):
        session = self.state.current
        if session is None:
            worker = self.state.gnss_network_worker
        else:
            session.stop_requested = True
            session.pending_credential = None
            worker = session.worker
        self.state.publish()
        if worker is not None:
            try:
                # run() blocks its thread's event loop: this thread-safe stop
                # port must be direct, not a queued invocation.
                worker.request_stop()
            except Exception as exc:
                self.callback_error = f"stop: {exc}"
        return not self.busy

    def retire(self, session):
        if self.state.current is not session:
            return
        try:
            if session.thread.isRunning():
                return
        except RuntimeError:
            pass
        session.pending_credential = None
        session.terminal = True
        self.state.current = None
        self.state.publish()
        try:
            self._callbacks["retired"]()
        except Exception as exc:
            self.callback_error = f"retired: {exc}"
        finally:
            session.relay.deleteLater()
            session.thread.deleteLater()

    def clear_finished_legacy(self, thread):
        session = self.state.current
        if session is not None and session.thread is thread:
            self.retire(session)

    def resource_inventory(self):
        return self.state.resource_inventory()
